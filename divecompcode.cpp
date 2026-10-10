/* ============================================================================
 * ESP32 Dive Computer -- main.cpp
 *
 * Fresh C++ port of the JS firmware block in divesim.html (the simulator this
 * project has been developed in), written once the simulator's feature set
 * stabilized, per porting-guide.md's "port last" convention. This is a
 * function-for-function translation of the FIRMWARE block -- same variable
 * names (shortened: S.depth stays S.depth, not reinterpreted), same control
 * flow, same comments where they explain *why* something is done. It is NOT
 * a redesign. Where the JS and the real hardware genuinely differ (fonts,
 * screen resolution, storage backend), that's called out explicitly below
 * rather than silently "fixed" -- see "KNOWN GAPS" at the bottom of this
 * header.
 *
 * Hardware (per project notes):
 *  - ESP32 dev board
 *  - 1.3" ST7735 128x128 color SPI LCD (8-pin: GND/VCC/SCL/SDA/RES/DC/CS/BLK)
 *  - LSM303 (accel+mag) for heading, plus its onboard (coarse) temp sensor
 *  - Analog pressure sensor on one ADC-capable pin
 *  - 4 momentary buttons: UP / SELECT / MODE / DOWN
 *  - Piezo buzzer for the alarm/feedback tones
 *
 * Libraries this file assumes (install via Arduino Library Manager / PlatformIO):
 *  - Adafruit GFX Library, Adafruit ST7735 and ST7789 Library
 *  - Adafruit LSM303DLHC (or Adafruit Unified Sensor + Adafruit LSM303_U)
 *  - Preferences (bundled with the ESP32 Arduino core)
 *  - LittleFS (bundled with the ESP32 Arduino core, via "LittleFS.h")
 *  - ArduinoJson (used only for the PC-sync Serial dump of the dive log)
 *
 * ============================================================================
 * KNOWN GAPS -- real things this port does not pretend to have solved:
 *
 *  1. SCREEN RESOLUTION MISMATCH. The firmware's render() logic is written
 *     for a 128x64 logical screen (inherited from the project's original
 *     monochrome-OLED-shaped layout) but the real panel is 128x128 color.
 *     This port draws into a 128x64 back-buffer (GFXcanvas16) and blits it
 *     centered onto the 128x128 panel, leaving ~32px black bars top and
 *     bottom. Reworking the UI layout to use the full square panel is a
 *     separate follow-up, deliberately NOT done here (porting a change and
 *     redesigning the layout in the same pass is exactly what
 *     porting-guide.md says not to do).
 *  2. FONT METRICS. The JS firmware's layout math hardcodes per-character
 *     pixel advances for each named font (5x7=5px, 6x10/6x12=6px, 7x14=7px,
 *     L22=13px) and uses those numbers directly in places like
 *     `gh.length*6` to position the next string. Adafruit_GFX's built-in
 *     font at text size 1 has a 6px advance, which matches '6x10'/'6x12'
 *     (the most commonly used font) exactly. It does NOT exactly match
 *     '5x7' (5px) or 'L22' (13px, vs. 12px at size 2) -- those will be off
 *     by a pixel or two until a custom bitmap font is sourced. Flagging
 *     this plainly rather than hand-waving it as solved.
 *  3. LSM303 TEMPERATURE. Wired up as a direct register read of the
 *     onboard temp output; per the datasheet this is a coarse, relative
 *     reading, not a calibrated absolute temperature. Matches what was
 *     already noted during planning -- still needs real calibration.
 *  4. BATTERY. No real voltage-divider ADC reading is wired up yet --
 *     battery.cpp's `S.batt` still just decays over time as a placeholder,
 *     same as the simulator. Flagged previously, not solved by this port.
 *  5. DIVE LOG STORAGE SIZE. A full dive log entry (profile samples +
 *     before/after tissue snapshots for 16 compartments) is a few KB.
 *     Twenty of them do not comfortably fit in NVS (Preferences), so the
 *     log is stored as a single LittleFS file instead (read-modify-write
 *     the whole array, same pattern as the JS `prefs.get/put('logs',...)`).
 *     Settings and the resumable dive state ('set' / 'st') stay in NVS via
 *     Preferences since those are small and change less often mid-dive.
 *  6. RTC. `date` in each dive log entry uses millis()-since-boot, not a
 *     real wall-clock timestamp -- an RTC chip was planned but isn't wired
 *     up in this port. Set it properly once the RTC is on the board.
 *  7. NOT COMPILED AGAINST REAL HARDWARE LIBRARIES. This sandbox has no
 *     Arduino/ESP32 toolchain, so this file has been checked for brace
 *     balance and structural completeness against the JS source, but not
 *     actually compiled. Build it in the Arduino IDE / PlatformIO before
 *     flashing, and expect to fix a few library-call signatures (Adafruit
 *     library APIs drift between versions).
 * ========================================================================= */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Pin assignments -- placeholders matching the documented wiring; adjust to
// match the actual board.
// ---------------------------------------------------------------------------
#define PIN_BTN_UP     32
#define PIN_BTN_SELECT 33
#define PIN_BTN_MODE   25
#define PIN_BTN_DOWN   26
#define PIN_PRESSURE   34   // analog, ADC1-capable
#define PIN_BUZZER     27
#define PIN_TFT_CS     5
#define PIN_TFT_DC     17
#define PIN_TFT_RST    16
#define PIN_TFT_BLK    4    // backlight, optional PWM dimming
// SPI MOSI/SCK use the ESP32's default VSPI pins; I2C (LSM303) uses the
// default Wire pins (SDA 21 / SCL 22 on most ESP32 dev boards).

// Button index convention, matches the JS HAL.edge(buttonIndex) contract:
// BTN.UP=0, BTN.SELECT=1, BTN.MODE=2, BTN.DOWN=3
enum { BTN_UP = 0, BTN_SELECT = 1, BTN_MODE = 2, BTN_DOWN = 3 };
static const uint8_t BTN_PINS[4] = { PIN_BTN_UP, PIN_BTN_SELECT, PIN_BTN_MODE, PIN_BTN_DOWN };

Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
GFXcanvas16 canvas(128, 64); // logical back-buffer; see KNOWN GAPS #1

Preferences prefsNVS; // backs the 'set' and 'st' keys (small, frequent writes)

// ---------------------------------------------------------------------------
// Color palette -- values copied verbatim from divesim.html's COLORS object
// (hex -> RGB565). Keep these in sync if the palette changes on the JS side.
// ---------------------------------------------------------------------------
static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
enum ColorKey {
  COL_NONE = -1, // sentinel: "no color given" -> drawStr etc. default to primary, matching COLORS[col]||COLORS.primary
  COL_BG, COL_BGDIVE, COL_BGMENU,
  COL_PRIMARY, COL_DIM, COL_OK, COL_MINOR, COL_CRIT, COL_DECO, COL_WHITE,
  COL_GAS1, COL_GAS2, COL_GAS3,
  COL_COUNT
};
static const uint16_t COLORS[COL_COUNT] = {
  /* BG     */ rgb565(0x04,0x08,0x0b),
  /* BGDIVE */ rgb565(0x02,0x0a,0x14),
  /* BGMENU */ rgb565(0x07,0x0a,0x12),
  /* PRIMARY*/ rgb565(0x4b,0xe3,0xff),
  /* DIM    */ rgb565(0x33,0x60,0x6e),
  /* OK     */ rgb565(0x2f,0xe6,0xa7),
  /* MINOR  */ rgb565(0xff,0xb0,0x20),
  /* CRIT   */ rgb565(0xff,0x3b,0x30),
  /* DECO   */ rgb565(0x5a,0xa9,0xff),
  /* WHITE  */ rgb565(0xea,0xf7,0xfb),
  /* GAS1   */ rgb565(0x4b,0xe3,0xff),
  /* GAS2   */ rgb565(0x9b,0xe5,0x64),
  /* GAS3   */ rgb565(0xff,0xb3,0xde),
};
static inline uint16_t colOf(int key) { return COLORS[key < 0 ? COL_PRIMARY : key]; }

// Font keys -> per-character pixel advance, copied from the JS `FS` table.
// The firmware core's own layout math (e.g. `gh.length()*6`) depends on
// these exact numbers -- see KNOWN GAPS #2 for how well each is actually
// matched by the real font in use.
enum FontKey { FONT_5x7, FONT_6x10, FONT_6x12, FONT_7x14, FONT_L22 };
static const uint8_t FONT_ADV[5] = { 5, 6, 6, 7, 13 };
static int curFont = FONT_6x10;

// ---------------------------------------------------------------------------
// u8g2-shaped drawing wrapper (see porting-guide.md section 2 -- the JS side
// takes a HAL object with this exact shape; here it's the same calls wired
// directly to Adafruit_GFX on the back-buffer canvas).
// ---------------------------------------------------------------------------
namespace u8g2 {
  void clearBuffer(int theme) { canvas.fillScreen(colOf(theme)); }
  void setFont(int f) { curFont = f; }
  void drawStr(int x, int y, const String &s, int col = COL_NONE) {
    canvas.setTextColor(colOf(col));
    // Adafruit_GFX's baseline is the glyph *bottom*; the JS canvas2d drawStr
    // baseline is also the text bottom, so y lines up directly.
    int size = (curFont == FONT_L22) ? 2 : 1;
    canvas.setTextSize(size);
    canvas.setCursor(x, y - 7 * size); // GFX cursor is top-left; shift up by one glyph height
    canvas.print(s);
  }
  void drawHLine(int x, int y, int w, int col = COL_NONE) { canvas.drawFastHLine(x, y, w, colOf(col)); }
  void drawBox(int x, int y, int w, int h, int col = COL_NONE) { canvas.fillRect(x, y, w, h, colOf(col)); }
  void drawLine(int x0, int y0, int x1, int y1, int col = COL_NONE) { canvas.drawLine(x0, y0, x1, y1, colOf(col)); }
  void drawCircle(int cx, int cy, int r, int col = COL_NONE) { canvas.drawCircle(cx, cy, r, colOf(col)); }
  void sendBuffer() {
    // Blit the 128x64 logical buffer centered onto the real 128x128 panel.
    // See KNOWN GAPS #1 -- this is the "don't redesign during a port" choice.
    tft.drawRGBBitmap(0, 32, canvas.getBuffer(), 128, 64);
  }
}

// ---------------------------------------------------------------------------
// HAL: millis/analogMv/edge/heading/temp/beep/chime -- the hardware-facing
// functions the JS side gets from its `HAL` object parameter.
// ---------------------------------------------------------------------------
unsigned long halMillis() { return millis(); }

int analogMv(int /*channel*/) {
  // Real ESP32 Arduino core function, calibrated millivolt ADC read --
  // matches the JS HAL.analogMv(pin) contract directly.
  return analogReadMilliVolts(PIN_PRESSURE);
}

bool btnPrevState[4] = { false, false, false, false };
unsigned long btnLastChange[4] = { 0, 0, 0, 0 };
static const unsigned long DEBOUNCE_MS = 25;
// Returns true exactly once per press (rising edge, debounced) -- matches
// HAL.edge(buttonIndex) in the JS HAL.
bool halEdge(int idx) {
  bool raw = digitalRead(BTN_PINS[idx]) == LOW; // active-low with INPUT_PULLUP
  unsigned long now = millis();
  if (raw != btnPrevState[idx] && now - btnLastChange[idx] > DEBOUNCE_MS) {
    btnLastChange[idx] = now;
    bool wasLow = btnPrevState[idx];
    btnPrevState[idx] = raw;
    if (raw && !wasLow) return true; // rising edge only
  }
  return false;
}

// LSM303 heading: tilt-uncompensated atan2 of the magnetometer X/Y, which is
// what the JS sim models (a plain compass card, no tilt compensation).
// Replace with a proper Adafruit_LSM303DLH_Mag read once the library is wired in.
float halHeading();
float halTemp();

void beep(const char * /*name*/, int freq, int ms) {
  // tone() on the ESP32 Arduino core is non-blocking and auto-stops after
  // `ms` -- the alarm engine below (driveAlarmSound) already does its own
  // non-blocking timing, so this can be a direct, simple call.
  if (freq > 0) tone(PIN_BUZZER, freq, ms);
}
void chime(int freq, int ms) { beep("", freq, ms); }

// ---------------------------------------------------------------------------
// Preferences ('set' / 'st') -- small, simple structs, stored as NVS blobs.
// ---------------------------------------------------------------------------
struct GasCfg { int o2; int he; bool enabled; bool labelE; };

struct SettingsBlob {
  int fo2, ppo2, algo;
  bool deep;
  char units[4]; // "met" or "imp"
  GasCfg gasList[3];
  int activeGas;
  double heFrac;
  char diveMode[4]; // "OC","G","F"
};
struct StateBlob {
  double P[16], PHE[16];
  long nofly, lock;
  int dcount, viol;
  double zero;
  bool hasLastDiveEnd;
  unsigned long lastDiveEndMs;
};

void prefsPutSet(const SettingsBlob &b) { prefsNVS.putBytes("set", &b, sizeof(b)); }
bool prefsGetSet(SettingsBlob &b) { return prefsNVS.getBytes("set", &b, sizeof(b)) == sizeof(b); }
void prefsPutSt(const StateBlob &b) { prefsNVS.putBytes("st", &b, sizeof(b)); }
bool prefsGetSt(StateBlob &b) { return prefsNVS.getBytes("st", &b, sizeof(b)) == sizeof(b); }

// ---------------------------------------------------------------------------
// Dive log ('logs') -- bigger, variable-shaped records; stored as a single
// LittleFS file, read/modified/rewritten wholesale, same pattern as the JS
// prefs.get('logs',[]) / prefs.put('logs', l.slice(-20)). See KNOWN GAPS #5.
// ---------------------------------------------------------------------------
#define LOG_MAX_ENTRIES   20
#define LOG_MAX_GASES     8    // generous cap on gas switches recorded per dive
#define LOG_MAX_PROFILE   120  // matches the JS profile-halving cap exactly

struct GasUsed { int fo2; int he; char label; }; // label: 'A' or 'E', see gasLabel()
struct ProfilePt { int t; double d; };

struct DiveLogEntry {
  unsigned long date;      // millis()-since-boot for now; see KNOWN GAPS #6
  int durSec;
  double maxD, avgD, avgTemp;
  GasUsed gases[LOG_MAX_GASES]; int gasesCount;
  int algo;
  bool deco;
  double gfUsed[2];
  bool hasSurfInterval; long surfIntervalSec;
  double surfPressure;
  double tissueStartN2[16], tissueEndN2[16];
  ProfilePt profile[LOG_MAX_PROFILE]; int profileCount;
};

// In-RAM mirror of the on-disk log, loaded at boot and rewritten wholesale
// on every endDive() -- mirrors the JS array-of-objects exactly.
DiveLogEntry logs[LOG_MAX_ENTRIES];
int logsCount = 0;

void logsLoad() {
  logsCount = 0;
  File f = LittleFS.open("/logs.bin", "r");
  if (!f) return;
  while (f.available() >= (int)sizeof(DiveLogEntry) && logsCount < LOG_MAX_ENTRIES) {
    f.read((uint8_t *)&logs[logsCount], sizeof(DiveLogEntry));
    logsCount++;
  }
  f.close();
}
void logsSave() {
  File f = LittleFS.open("/logs.bin", "w");
  if (!f) return;
  for (int i = 0; i < logsCount; i++) f.write((uint8_t *)&logs[i], sizeof(DiveLogEntry));
  f.close();
}
// Ring-buffer push matching JS `l.push(entry); prefs.put('logs', l.slice(-20))`.
void logsPush(const DiveLogEntry &e) {
  if (logsCount < LOG_MAX_ENTRIES) {
    logs[logsCount++] = e;
  } else {
    for (int i = 1; i < LOG_MAX_ENTRIES; i++) logs[i - 1] = logs[i];
    logs[LOG_MAX_ENTRIES - 1] = e;
  }
  logsSave();
}

/* ================= FIRMWARE (C++ port of divesim.html's JS firmware) =================
 * Everything below this line is a direct, function-for-function port of the
 * FIRMWARE block between the matching markers in divesim.html. Same names,
 * same structure, same order. Comments that explain *why* (not what) are
 * kept verbatim per porting-guide.md section 3.
 * ======================================================================== */

static const double P_SURF = 1.013, PH2O = 0.0627, FN2_AIR = 0.7902, MPB = 10, ASC = 9;
static const double HT[16] = {5,8,12.5,18.5,27,38.3,54.3,77,109,146,187,239,305,390,498,635};
static const double A[16] = {1.1696,1,0.8618,0.7562,0.62,0.5043,0.441,0.4,0.375,0.35,0.3295,0.3065,0.2835,0.261,0.248,0.2327};
static const double B[16] = {0.5578,0.6514,0.7222,0.7825,0.8126,0.8434,0.8693,0.891,0.9092,0.9222,0.9319,0.9403,0.9477,0.9544,0.9602,0.9653};
double K[16]; // = ln(2)/(HT*60), computed once in initConstants()
double PAIR;  // = (P_SURF-PH2O)*FN2_AIR
double P[16]; // N2 tissue loading -- "static" persistent state (JS `let P`)

// Helium: same 16-compartment structure, standard ZHL-16C He half-times/coefficients.
static const double HT_HE[16] = {1.88,3.02,4.72,6.99,10.21,14.48,20.53,29.11,41.20,55.19,70.69,90.34,115.29,147.42,188.24,240.03};
static const double A_HE[16] = {1.6189,1.3830,1.1919,1.0458,0.9220,0.8205,0.7305,0.6502,0.5950,0.5545,0.5333,0.5189,0.5181,0.5176,0.5172,0.5119};
static const double B_HE[16] = {0.4770,0.5747,0.6527,0.7223,0.7582,0.7957,0.8279,0.8553,0.8757,0.8903,0.8997,0.9073,0.9122,0.9171,0.9217,0.9267};
double K_HE[16];
double PHE[16]; // helium loading; starts at 0 (no He in air)

void initConstants() {
  for (int i = 0; i < 16; i++) K[i] = log(2.0) / (HT[i] * 60.0);
  for (int i = 0; i < 16; i++) K_HE[i] = log(2.0) / (HT_HE[i] * 60.0);
  PAIR = (P_SURF - PH2O) * FN2_AIR;
  for (int i = 0; i < 16; i++) { P[i] = PAIR; PHE[i] = 0; }
}

// Combined-gas M-value: per compartment, weight A/B by how much of that
// compartment's loading is N2 vs He, then apply the same M-value formula to
// the combined total. With no helium ever present (PHE all zero) this
// reduces exactly to the original N2-only math.
double maxTolC(const double Qn[16], const double Qh[16], double g) {
  double m = 0;
  for (int i = 0; i < 16; i++) {
    double pn = Qn[i], ph = Qh[i], tot = pn + ph;
    double a = tot > 1e-9 ? (A[i]*pn + A_HE[i]*ph) / tot : A[i];
    double b = tot > 1e-9 ? (B[i]*pn + B_HE[i]*ph) / tot : B[i];
    double t = (tot - a*g) / (g/b - g + 1);
    if (t > m) m = t;
  }
  return m > 0 ? m : 0;
}

// Surface GF%: the inverse of maxTolC -- instead of solving for the ceiling
// depth at a given gradient factor, solve for the gradient factor whose
// ceiling would put the diver exactly at the surface right now. A high value
// right after surfacing (whether or not it technically broke the ceiling) is
// the signature of a rapid/rescue-type ascent -- the computer never had time
// to off-gas before the diver was already up, unlike a normal, paced ascent
// that lands well under 100%.
double surfaceGF() {
  double m = 0;
  for (int i = 0; i < 16; i++) {
    double pn = P[i], ph = PHE[i], tot = pn + ph;
    double a = tot > 1e-9 ? (A[i]*pn + A_HE[i]*ph) / tot : A[i];
    double b = tot > 1e-9 ? (B[i]*pn + B_HE[i]*ph) / tot : B[i];
    double m0 = a + P_SURF / b; // M-value at the surface (gf=100%)
    double g = (tot - P_SURF) / (m0 - P_SURF) * 100;
    if (g > m) m = g;
  }
  return m > 0 ? m : 0;
}

static const double TD[12] = {10,12,14,16,18,20,22,25,30,35,40,42};
static const int TN[12] = {219,147,98,72,56,45,37,29,20,14,9,8};
int tadj[12];
static const char *ALG[3] = {"Buhlmann", "RGBM-sim", "PADI-tbl"};

// Dive modes, selectable only on the surface (menu isn't reachable mid-dive
// at all): OC=open circuit -- NOT itself A/NX/OCM; the gas setup you already
// configured decides which of those it actually is (see ocLabel()), so
// there's nothing to pick separately and nothing to get out of sync.
// F=freedive, G=gauge (bottom timer, diver plans their own deco).
// CC=closed circuit -- deliberately left OUT of DIVE_MODES for now (not
// selectable at all), until real CCR support (setpoint control, diluent,
// bailout) is built; add it back as a 4th entry then. The active mode is
// shown on every dive screen (see diveBody header).
static const char *DIVE_MODES[3] = {"OC", "G", "F"};
static const int DIVE_MODES_COUNT = 3;

// ---------------------------------------------------------------------------
// State -- one struct, not scattered globals, matching the JS `S = {...}`
// object exactly: makes a debug dump trivial and keeps "did I reset this on
// dive start" auditable in one place (see porting-guide.md section 3).
// ---------------------------------------------------------------------------
struct Stop { double d; int m; };
struct Bookmark { double d; int t; int h; };

struct State {
  String mode = "SURFACE"; // SURFACE/DIVE/MENU/SYNC
  int view = 0;
  int algo = 0;
  int fo2 = 21;
  int ppo2 = 140;
  bool deep = false;
  bool dirty = false;
  double zero = 250;
  bool zeroValid = false;
  double gauge = 0, depth = 0, maxD = 0, rate = 0;
  double pabs = 0 /*set from P_SURF in initConstants-adjacent setup*/, prev = 0;
  double ppo2n = .21;
  double mod = 0;
  bool fault = false;
  int bad = 0;
  unsigned long dstart = 0;
  int dur = 0;
  int dcount = 0, viol = 0;
  long nofly = 0, lock = 0;
  int ndl = 99;
  bool deco = false;
  double ceil = 0;
  int tts = 0;
  double nstop = 0;
  int nmin = 0;
  bool texc = false, dDeco = false, dViol = false;
  double breach = 0;
  double breachTot = 0;
  bool severe = false;
  double gf0 = .3, gf1 = .7;
  bool ss = false, ssDone = false; double ssEl = 0;
  bool ds = false, dsDone = false; double dsEl = 0; double dsD = 0;
  int cur = 0;
  unsigned long desc = 0, surf = 0, sync_ = 0; // `sync` is reserved-ish; trailing underscore avoids any clash
  bool wc = false, wa = false, wp = false;
  bool wpLow = false, wcns = false, wbat = false, wndl2 = false;
  int hdg = 0;
  String card = "N";
  String showLog = ""; // "" = false, else "list" / "detail"
  int logCur = 0, logPage = 0;
  double ugf0 = .3, ugf1 = .7;
  double batt = 100;
  double cns = 0;
  double atDepth = 0, atAnchor = 0;
  Stop stops[4]; int stopsCount = 0;
  long lockout = 0;
  bool compLocked = false;
  int refHdg = -1; // -1 = null
  Bookmark bookmarks[6]; int bookmarksCount = 0;
  int surfPage = 0;
  String toast = "";
  unsigned long toastUntil = 0;
  String alarmActive = ""; // "" = null
  int alarmStep = 0;
  unsigned long alarmNextAt = 0;
  int alarmCycle = 0;
  String popupMsg = "";
  unsigned long popupUntil = 0;
  String units = "met";
  double avgAccD = 0, avgAccT = 0, avgAccTemp = 0;
  ProfilePt profile[LOG_MAX_PROFILE]; int profileCount = 0;
  int profileIv = 10;
  bool hasLastDiveEnd = false; unsigned long lastDiveEndMs = 0;
  GasCfg gasList[3] = {{21,0,true,false},{21,0,false,false},{21,0,false,false}};
  int activeGas = 0;
  double heFrac = 0;
  GasUsed gasesUsedThisDive[LOG_MAX_GASES]; int gasesUsedThisDiveCount = 0;
  bool gasSetup = false;
  int gasCur = 0;
  bool gasSwitchUI = false;
  int gasSwitchSel = 0;
  String diveMode = "OC";
  // extra fields used only transiently between startDive()/endDive(), matching
  // the JS object's dynamically-added-later properties (gfUsed, tissueStart, surfIntervalSec)
  double gfUsed[2] = {.3, .7};
  double tissueStart[16]; bool tissueStartValid = false;
  bool hasSurfInterval = false; long surfIntervalSec = 0;
};
State S;

unsigned long lastSim = 0, lastS = 0, saveAcc = 0, lastR = 0;
double hist[6]; int histCount = 0;
struct PrevWarn { bool ss=false, ds=false, wbat=false, wndl2=false; } pw;

bool blinkOn(unsigned long now, unsigned long periodMs) { return ((now / periodMs) % 2) == 0; }

// Unit helpers. All physics/state stay metric; these only format what is drawn.
static const double M2FT = 3.28084;
bool isImp() { return S.units == "imp"; }
String dU() { return isImp() ? "ft" : "m"; } // depth unit label
String dS(double m) { // depth string (imperial = whole feet)
  if (isImp()) return String((long)round(m * M2FT));
  return String(m, 1);
}
String dI(double m) { // whole-number depth string
  if (isImp()) return String((long)round(m * M2FT));
  return String(m, 0);
}
String cS(double m) { // ceiling/stop depth: 3m steps shown as 10ft steps (always slightly deeper than the true value)
  if (isImp()) return String((long)round(m / 3.0) * 10);
  return String(m, 0);
}
String rS(double r) { // vertical rate string
  if (isImp()) return String(r * M2FT, 0);
  return String(r, 1);
}
String tS(double c) { // temperature string
  if (isImp()) return String(c * 9.0 / 5.0 + 32.0, 1) + "F";
  return String(c, 1) + "C";
}
double pAt(double d) { return P_SURF + d / MPB; }

void leg(double Qn[16], double Qh[16], double p0, double p1, double s, double fN2, double fHe) {
  if (s <= 0) return;
  double a = (p0 - PH2O) * fN2, b = (p1 - PH2O) * fN2, R = (b - a) / s;
  for (int i = 0; i < 16; i++) {
    double k = K[i], e = exp(-k * s);
    Qn[i] = a + R * (s - 1/k) - (a - Qn[i] - R/k) * e;
  }
  double a2 = (p0 - PH2O) * fHe, b2 = (p1 - PH2O) * fHe, R2 = (b2 - a2) / s;
  for (int i = 0; i < 16; i++) {
    double k = K_HE[i], e = exp(-k * s);
    Qh[i] = a2 + R2 * (s - 1/k) - (a2 - Qh[i] - R2/k) * e;
  }
}

double gfNow(double out[2]) {
  if (S.algo == 1) {
    double hi = min(S.ugf1, max(.45, S.ugf1 + .05 - .05*min(S.dcount,3) - .05*min(S.viol,3)));
    out[0] = S.ugf0; out[1] = hi;
  } else {
    out[0] = S.ugf0; out[1] = S.ugf1;
  }
  return out[1];
}

static const double CNS_TBL[12][2] = {
  {.5,1e9},{.6,720},{.7,570},{.8,450},{.9,360},{1.0,300},
  {1.1,240},{1.2,210},{1.3,180},{1.4,150},{1.5,120},{1.6,45}
};
double cnsRatePerMin(double ppo2) {
  if (ppo2 <= .5) return 0;
  double lim = 45;
  for (int i = 1; i < 12; i++) {
    if (ppo2 <= CNS_TBL[i][0]) {
      double a = CNS_TBL[i-1][0], la = CNS_TBL[i-1][1];
      double b = CNS_TBL[i][0], lb = CNS_TBL[i][1];
      double t = (ppo2 - a) / (b - a);
      lim = la + (lb - la) * t;
      break;
    }
    if (i == 11) lim = 45;
  }
  return 100.0 / lim;
}

double ndlCalc(double Qn[16], double Qh[16], double pa, double fN2, double fHe, double gh) {
  bool anyHe = fHe > 0;
  if (!anyHe) { for (int i = 0; i < 16; i++) if (Qh[i] > 1e-6) { anyHe = true; break; } }
  if (!anyHe) { // no helium involved: identical to the original analytic formula
    double pi = (pa - PH2O) * fN2;
    double best = 9999.0 * 60;
    for (int i = 0; i < 16; i++) {
      double M = P_SURF + gh * (A[i] + P_SURF/B[i] - P_SURF);
      if (Qn[i] >= M) return 0;
      if (pi <= M) continue;
      double v = log((pi - Qn[i]) / (pi - M)) / K[i];
      if (v < best) best = v;
    }
    return best / 60.0;
  }
  // combined N2+He loads at different rates, so there's no closed form: step
  // forward a minute at a time (at the current, constant depth) until a
  // ceiling would appear.
  double cn[16], ch[16];
  memcpy(cn, Qn, sizeof(cn)); memcpy(ch, Qh, sizeof(ch));
  for (int m = 0; m < 99; m++) {
    if (maxTolC(cn, ch, gh) > P_SURF + 1e-4) return m;
    leg(cn, ch, pa, pa, 60, fN2, fHe);
  }
  return 99;
}

double ceilCalc(double Qn[16], double Qh[16], const double g[2]) {
  if (maxTolC(Qn, Qh, g[1]) <= P_SURF + 1e-4) return 0;
  double pl = maxTolC(Qn, Qh, g[0]);
  double fm = max(3.0, ceil((pl - P_SURF) * MPB / 3 - 1e-9) * 3);
  double pf = pAt(fm);
  for (double d = 3; d <= fm + .01; d += 3) {
    double pd = pAt(d);
    double gg = g[1] + (g[0] - g[1]) * (pd - P_SURF) / (pf - P_SURF);
    if (maxTolC(Qn, Qh, gg) <= pd + 1e-4) return d;
  }
  return fm;
}

void simDeco(const double g[2], double fN2, double fHe) {
  double Qn[16], Qh[16];
  memcpy(Qn, P, sizeof(Qn)); memcpy(Qh, PHE, sizeof(Qh));
  double d = S.depth, t = 0; double fc = -1; int fmn = 0; bool fin = false;
  Stop sched[64]; int schedCount = 0; // generous scratch space; only the first 4 are kept, matching JS `sched.slice(0,4)`
  double curD = -1; int curMin = 0;
  for (int it = 0; it < 600; it++) {
    double c = ceilCalc(Qn, Qh, g);
    if (fc < 0) fc = c;
    if (c <= 0) {
      t += d / ASC * 60;
      fin = true;
      if (curD > 0 && schedCount < 64) sched[schedCount++] = {curD, curMin};
      break;
    }
    if (d > c + .01) {
      double s = (d - c) / ASC * 60;
      leg(Qn, Qh, pAt(d), pAt(c), s, fN2, fHe);
      t += s; d = c;
      if (curD > 0 && schedCount < 64) sched[schedCount++] = {curD, curMin};
      curD = c; curMin = 0;
    } else {
      d = c;
      leg(Qn, Qh, pAt(c), pAt(c), 60, fN2, fHe);
      t += 60; curMin++;
      if (fabs(c - fc) < .01) fmn++;
    }
  }
  S.tts = fin ? (int)ceil(t / 60) : 999;
  S.nstop = fc > 0 ? fc : 0;
  S.nmin = fmn > 1 ? fmn : 1;
  S.stopsCount = min(schedCount, 4);
  for (int i = 0; i < S.stopsCount; i++) S.stops[i] = sched[i];
}

int tIdx(double d) { for (int i = 0; i < 12; i++) if (d <= TD[i]) return i; return -1; }

void buildTbl() {
  double fN2 = 1 - S.fo2 / 100.0;
  double PA[16], PAHe[16];
  for (int i = 0; i < 16; i++) { PA[i] = PAIR; PAHe[i] = 0; }
  for (int j = 0; j < 12; j++) {
    double pa = pAt(TD[j]);
    double PAc[16], PAHec[16]; memcpy(PAc, PA, sizeof(PA)); memcpy(PAHec, PAHe, sizeof(PAHe));
    double fr = ndlCalc(PAc, PAHec, pa, fN2, 0, .7);
    double Pc[16], PHEc[16]; memcpy(Pc, P, sizeof(P)); memcpy(PHEc, PHE, sizeof(PHE));
    double nw = ndlCalc(Pc, PHEc, pa, fN2, 0, .7);
    double r = fr < .5 ? 1 : min(1.0, nw / fr);
    tadj[j] = (int)floor(TN[j] * max(0.0, r));
  }
}

void saveState() {
  StateBlob b;
  memcpy(b.P, P, sizeof(P));
  memcpy(b.PHE, PHE, sizeof(PHE));
  b.nofly = S.nofly; b.lock = S.lock; b.dcount = S.dcount; b.viol = S.viol; b.zero = S.zero;
  b.hasLastDiveEnd = S.hasLastDiveEnd; b.lastDiveEndMs = S.lastDiveEndMs;
  prefsPutSt(b);
}
bool loadState() {
  StateBlob b;
  if (!prefsGetSt(b)) return false;
  bool sane = true;
  for (int i = 0; i < 16; i++) if (!(b.P[i] > 0.3 && b.P[i] < 8)) { sane = false; break; }
  if (!sane) return false;
  memcpy(P, b.P, sizeof(P));
  memcpy(PHE, b.PHE, sizeof(PHE));
  S.nofly = b.nofly; S.lock = b.lock; S.dcount = b.dcount; S.viol = b.viol;
  if (b.zero > 50 && b.zero < 2450) { S.zero = b.zero; S.zeroValid = true; }
  S.hasLastDiveEnd = b.hasLastDiveEnd; S.lastDiveEndMs = b.lastDiveEndMs;
  return true;
}
void saveSet() {
  SettingsBlob b{};
  b.fo2 = S.fo2; b.ppo2 = S.ppo2; b.algo = S.algo; b.deep = S.deep;
  strncpy(b.units, S.units.c_str(), 3); b.units[3] = 0;
  for (int i = 0; i < 3; i++) b.gasList[i] = S.gasList[i];
  b.activeGas = S.activeGas; b.heFrac = S.heFrac;
  strncpy(b.diveMode, S.diveMode.c_str(), 3); b.diveMode[3] = 0;
  prefsPutSet(b);
  S.dirty = false;
}

// A single gas's A/E label: deco math is identical for air and EANx21 (same
// 21%O2/79%N2 either way -- compressor fill vs nitrox blend doesn't change
// tissue loading), so this is a *record* distinction only. At exactly 21%
// O2 / 0% He the diver's own labelE flag (set in gas setup) decides A vs E,
// so a deliberately-blended EANx21 fill can be logged as E, not lumped in
// as A. Any other composition always shows E regardless of the flag --
// there's no ambiguity to tag there.
char gasLabel(const GasCfg &g) {
  if (g.o2 == 21 && g.he == 0) return g.labelE ? 'E' : 'A';
  return 'E';
}

int enabledGasIndices(int out[3]) {
  int n = 0;
  for (int i = 0; i < 3; i++) if (S.gasList[i].enabled) out[n++] = i;
  return n;
}

// OC's displayed sub-label is derived, not stored: >1 enabled gas = OCM
// (open circuit multigas), otherwise it's just that one gas's label (see
// gasLabel above). gasList[0] can never be disabled (see gasSetupAdjust),
// so enabledGasIndices() always returns at least 1.
String ocLabel() {
  int en[3]; int n = enabledGasIndices(en);
  if (n > 1) return "OCM";
  char c = gasLabel(S.gasList[n > 0 ? en[0] : 0]);
  return String(c);
}

// Best-mix recommendation: among the enabled gases, pick the richest O2
// whose PPO2 stays within the diver's set limit at the given depth (richer
// O2 = shorter deco/less N2 loading, so "richest that's still safe" is the
// right pick). If every enabled gas would exceed the PPO2 limit at that
// depth (too deep for all of them), fall back to the leanest O2 (deepest
// MOD) instead, since that's the safest of the available options.
int bestGasIdx(double depth) {
  int en[3]; int n = enabledGasIndices(en);
  if (n == 0) return -1;
  double lim = S.ppo2 / 100.0, pa = pAt(depth);
  int best = -1, bestO2 = -1;
  for (int k = 0; k < n; k++) {
    int i = en[k]; const GasCfg &g = S.gasList[i];
    if (g.o2/100.0*pa <= lim + 1e-6 && g.o2 > bestO2) { bestO2 = g.o2; best = i; }
  }
  if (best < 0) {
    int leanO2 = 101;
    for (int k = 0; k < n; k++) { int i = en[k]; if (S.gasList[i].o2 < leanO2) { leanO2 = S.gasList[i].o2; best = i; } }
  }
  return best;
}

// Gas switching is a deliberate two-step action (open the picker, then
// confirm) rather than an instant switch on a single SELECT press,
// specifically so an accidental press mid-dive can't change gas. Only
// offered at all when 2+ gases are actually enabled.
void openGasSwitch() {
  int en[3]; int n = enabledGasIndices(en);
  if (n < 2) { S.toast = "ONLY 1 GAS SET"; S.toastUntil = halMillis() + 1500; return; }
  S.gasSwitchUI = true;
  int idx = 0; for (int k = 0; k < n; k++) if (en[k] == S.activeGas) idx = k;
  S.gasSwitchSel = en[(idx + 1) % n]; // default highlight: the "next" gas, matching the old one-press behavior
}
void cycleGasSwitchSel(int dir) {
  int en[3]; int n = enabledGasIndices(en);
  int idx = 0; for (int k = 0; k < n; k++) if (en[k] == S.gasSwitchSel) idx = k;
  idx = ((idx + dir) % n + n) % n;
  S.gasSwitchSel = en[idx];
}
void confirmGasSwitch() {
  const GasCfg &g = S.gasList[S.gasSwitchSel];
  S.activeGas = S.gasSwitchSel; S.fo2 = g.o2; S.heFrac = g.he / 100.0; S.dirty = true;
  if (S.mode == "DIVE" && S.gasesUsedThisDiveCount < LOG_MAX_GASES) {
    S.gasesUsedThisDive[S.gasesUsedThisDiveCount++] = { g.o2, g.he, gasLabel(g) };
  }
  S.toast = "GAS " + String(g.o2) + (g.he ? "/" + String(g.he) : "") + "%";
  S.toastUntil = halMillis() + 1500;
  beep("GAS SWITCH", 1100, 120);
  S.gasSwitchUI = false;
}

void sensors() {
  double mv = 0;
  for (int i = 0; i < 10; i++) mv += analogMv(0);
  mv /= 10;
  if (mv < 50 || mv > 2450) {
    if (S.bad < 255) S.bad++;
    if (S.bad >= 10) S.fault = true;
  } else {
    S.bad = 0; S.fault = false;
    double g = (mv - S.zero) / 1000.0 / 2 * 12;
    if (S.mode != "DIVE" && fabs(g) < .03) S.zero += .002 * (mv - S.zero);
    if (g < 0) g = 0;
    S.gauge += .3 * (g - S.gauge);
    S.depth = S.gauge * MPB;
    S.pabs = P_SURF + S.gauge;
  }
  double h = halHeading();
  S.hdg = ((int)round(h)) % 360; if (S.hdg < 0) S.hdg += 360;
  static const char *CARDS[8] = {"N","NE","E","SE","S","SW","W","NW"};
  int ci = ((int)floor((h + 22.5) / 45)) % 8; if (ci < 0) ci += 8;
  S.card = CARDS[ci];
}

void rateUpd(unsigned long now) {
  if (now - lastR >= 1000) {
    lastR = now;
    if (histCount < 6) { hist[histCount++] = S.depth; }
    else { for (int i = 0; i < 5; i++) hist[i] = hist[i+1]; hist[5] = S.depth; }
    S.rate = histCount == 6 ? (hist[0] - S.depth) * 12 : 0;
  }
}

void startDive(unsigned long t) {
  if (S.dirty) saveSet();
  S.mode = "DIVE"; S.view = 0; S.dstart = t; S.dur = 0; S.maxD = S.depth; S.surf = 0;
  S.ss = S.ssDone = S.ds = S.dsDone = false; S.ssEl = S.dsEl = 0;
  S.dDeco = S.dViol = false; S.breach = 0; S.breachTot = 0; S.severe = false;
  S.deco = false; S.ceil = 0; S.texc = false; S.lock = 86400;
  S.gfUsed[0] = S.ugf0; S.gfUsed[1] = S.ugf1;
  S.avgAccD = 0; S.avgAccT = 0; S.avgAccTemp = 0;
  S.profileCount = 0; S.profileIv = 10;
  memcpy(S.tissueStart, P, sizeof(P)); S.tissueStartValid = true;
  if (S.hasLastDiveEnd) { S.hasSurfInterval = true; S.surfIntervalSec = (long)round((t - S.lastDiveEndMs) / 1000.0); }
  else S.hasSurfInterval = false;
  S.gasesUsedThisDiveCount = 0;
  S.gasesUsedThisDive[S.gasesUsedThisDiveCount++] = { S.fo2, (int)round(S.heFrac * 100), gasLabel(S.gasList[S.activeGas]) };
  if (S.algo == 2) buildTbl();
  saveState();
  beep("DIVE START", 880, 150);
}

void endDive() {
  S.dur = (int)floor((S.surf - S.dstart) / 1000.0);
  S.dcount++;
  if (S.dViol) S.viol = min(255, S.viol + 1);
  bool missedSS = S.maxD >= 10 && !S.ssDone;
  long tg = (S.dDeco || S.dViol || missedSS) ? 86400 : (S.dcount > 1 ? 64800 : 43200);
  S.nofly = max(S.nofly, tg);
  S.lock = 86400;
  if (S.severe) { S.lockout = 172800; S.compLocked = true; beep("COMPUTER LOCKED", 300, 700); }

  DiveLogEntry e{};
  e.date = millis(); // see KNOWN GAPS #6
  e.durSec = S.dur;
  e.maxD = round(S.maxD * 10) / 10.0;
  e.avgD = round((S.avgAccT > 0 ? S.avgAccD / S.avgAccT : S.maxD) * 10) / 10.0;
  e.avgTemp = round((S.avgAccT > 0 ? S.avgAccTemp / S.avgAccT : halTemp()) * 10) / 10.0;
  if (S.gasesUsedThisDiveCount > 0) {
    e.gasesCount = min(S.gasesUsedThisDiveCount, LOG_MAX_GASES);
    for (int i = 0; i < e.gasesCount; i++) e.gases[i] = S.gasesUsedThisDive[i];
  } else {
    e.gasesCount = 1;
    e.gases[0] = { S.fo2, (int)round(S.heFrac * 100), gasLabel(S.gasList[S.activeGas]) };
  }
  e.algo = S.algo; e.deco = S.dDeco;
  e.gfUsed[0] = S.gfUsed[0]; e.gfUsed[1] = S.gfUsed[1];
  e.hasSurfInterval = S.hasSurfInterval; e.surfIntervalSec = S.surfIntervalSec;
  e.surfPressure = round(P_SURF * 1000) / 1000.0;
  memcpy(e.tissueStartN2, S.tissueStartValid ? S.tissueStart : P, sizeof(P));
  memcpy(e.tissueEndN2, P, sizeof(P));
  // trim the profile to durSec, matching JS `S.profile.filter(p=>p.t<=S.dur)`
  e.profileCount = 0;
  for (int i = 0; i < S.profileCount && e.profileCount < LOG_MAX_PROFILE; i++) {
    if (S.profile[i].t <= S.dur) e.profile[e.profileCount++] = S.profile[i];
  }
  logsPush(e); // richer records now (profile+tissue snapshots) -> smaller cap than before (matches JS slice(-20))

  S.hasLastDiveEnd = true; S.lastDiveEndMs = S.surf;
  saveState();
  S.mode = "SURFACE"; S.surf = 0;
  beep("DIVE END", 660, 300);
}

void upd(double dt, unsigned long now) {
  if (S.mode != "DIVE") { S.ndl = 99; S.deco = false; S.ceil = 0; S.tts = 0; S.texc = false; S.wc = S.wa = S.wp = false; return; }
  double g[2]; gfNow(g); S.gf0 = g[0]; S.gf1 = g[1];
  double f = 1 - S.fo2 / 100.0; S.texc = false;
  // F (freedive) and G (gauge) are bottom-timer modes: no NDL/deco/table
  // guidance is computed or shown at all -- the diver manages their own
  // decompression, same convention as a real gauge-mode dive computer.
  // Tissue loading (leg(), called elsewhere in tick()) still runs
  // underneath regardless of mode, so later dives the same day stay accurate.
  if (S.diveMode == "F" || S.diveMode == "G") {
    S.ndl = 99; S.deco = false; S.ceil = 0; S.tts = 0;
  } else {
    if (S.algo == 2) {
      int ix = tIdx(S.maxD); int el = (int)ceil(S.dur / 60.0);
      if (ix < 0 || el >= tadj[ix]) { S.texc = true; S.dViol = true; }
      else { S.ndl = min(99, tadj[ix] - el); S.deco = false; S.ceil = 0; S.tts = (int)ceil(S.depth / ASC); }
    }
    if (S.algo != 2 || S.texc) {
      S.ceil = ceilCalc(P, PHE, g);
      S.deco = S.ceil > 0;
      if (S.deco) {
        S.ndl = 0; S.dDeco = true;
        if (!lastSim || now - lastSim >= 2000) { simDeco(g, f, S.heFrac); lastSim = now ? now : 1; }
      } else {
        S.ndl = min(99, (int)floor(ndlCalc(P, PHE, S.pabs, f, S.heFrac, g[1])));
        S.tts = (int)ceil(S.depth / ASC);
      }
    }
  }
  S.wc = S.deco && S.depth < S.ceil - .5;
  if (S.wc) { S.breach += dt; if (S.breach > 10) S.dViol = true; } else S.breach = 0;
  S.wa = S.rate > 9; if (S.rate > 12) S.dViol = true;
  S.wp = S.ppo2n >= S.ppo2 / 100.0;
  S.wpLow = S.mode == "DIVE" && S.ppo2n < 0.18;
  S.wcns = S.cns >= 80;
  S.wbat = S.batt < 20;
  S.wndl2 = !S.deco && S.ndl <= 2;
  if (S.wc) { S.breachTot += dt; if (S.breachTot > 60) S.severe = true; }
}

void stops(double dt) {
  if (S.deco) { S.ss = S.ds = false; return; }
  if (S.deep && S.maxD >= 18 && !S.dsDone) {
    S.dsD = S.maxD / 2;
    S.ds = S.depth < S.maxD - 3 && fabs(S.depth - S.dsD) <= 1.5;
    if (S.ds) { S.dsEl += dt; if (S.dsEl >= 120) { S.dsDone = true; S.ds = false; beep("DEEP STOP DONE", 1000, 200); } }
  }
  if (S.maxD >= 10 && !S.ssDone) {
    if (S.depth >= 3 && S.depth <= 6) S.ss = true;
    else if (S.depth < 2.5 || S.depth > 7) { S.ss = false; S.ssEl = 0; }
    if (S.ss) { S.ssEl += dt; if (S.ssEl >= 180) { S.ssDone = true; S.ss = false; beep("SAFETY STOP DONE", 1000, 200); } }
  }
}

void gasSetupAdjust(int dir); // fwd decl, referenced by buttons() before its definition below

// dir=+1 (DOWN/increase) or -1 (UP/decrease); fires the item at S.cur
void menuAdjust(int dir) {
  switch (S.cur) {
    case 0: S.ppo2 = max(100, min(160, S.ppo2 + 5*dir)); S.dirty = true; break;
    case 1: S.deep = !S.deep; S.dirty = true; break; // toggle: either direction
    case 2: if (!S.lock) { S.algo = ((S.algo + dir) % 3 + 3) % 3; S.dirty = true; } break; // cycle: UP=prev, DOWN=next
    case 3: { // cycle: UP=prev, DOWN=next
      int idx = 0; for (int i = 0; i < DIVE_MODES_COUNT; i++) if (S.diveMode == DIVE_MODES[i]) idx = i;
      idx = ((idx + dir) % DIVE_MODES_COUNT + DIVE_MODES_COUNT) % DIVE_MODES_COUNT;
      S.diveMode = DIVE_MODES[idx]; S.dirty = true;
      break;
    }
    case 4: { // action: either direction fires it
      S.mode = "SYNC"; S.sync_ = halMillis() + 3000;
      // PC sync: dump the dive log as JSON over Serial for a companion app to read.
      DynamicJsonDocument doc(16384);
      JsonArray arr = doc.to<JsonArray>();
      for (int i = 0; i < logsCount; i++) {
        JsonObject o = arr.createNestedObject();
        o["durSec"] = logs[i].durSec; o["maxD"] = logs[i].maxD; o["avgD"] = logs[i].avgD;
        o["avgTemp"] = logs[i].avgTemp; o["algo"] = logs[i].algo; o["deco"] = logs[i].deco ? 1 : 0;
        JsonArray gs = o.createNestedArray("gases");
        for (int j = 0; j < logs[i].gasesCount; j++) {
          JsonObject go = gs.createNestedObject();
          go["fo2"] = logs[i].gases[j].fo2; go["he"] = logs[i].gases[j].he;
          go["label"] = String(logs[i].gases[j].label);
        }
      }
      Serial.print("LOGS ");
      serializeJson(doc, Serial);
      Serial.println();
      break;
    }
    case 5: if (S.dirty) saveSet(); S.mode = "SURFACE"; S.cur = 0; S.showLog = "list"; S.logCur = 0; break; // action
    case 6: S.ugf0 = max(.1, min(S.ugf1 - .05, round((S.ugf0 + .05*dir) * 100) / 100.0)); S.dirty = true; break;
    case 7: S.ugf1 = max(S.ugf0 + .05, min(1.0, round((S.ugf1 + .05*dir) * 100) / 100.0)); S.dirty = true; break;
    case 8: S.units = S.units == "imp" ? "met" : "imp"; S.dirty = true; break; // toggle: either direction
    case 9: if (S.dirty) saveSet(); S.gasSetup = true; S.gasCur = 0; break; // action: opens the gas-setup sub-screen
  }
}

// same dir convention, operating on the gas-setup sub-screen's 11 items
void gasSetupAdjust(int dir) {
  switch (S.gasCur) {
    case 0: { int o2 = max(1, min(100 - S.gasList[0].he, S.fo2 + dir)); S.fo2 = o2; S.gasList[0].o2 = o2; S.dirty = true; break; }
    case 1: { int he = max(0, min(100 - S.fo2, S.gasList[0].he + dir)); S.gasList[0].he = he; S.heFrac = he / 100.0; S.dirty = true; break; }
    case 2: { int o2 = max(1, min(100 - S.gasList[1].he, S.gasList[1].o2 + dir)); S.gasList[1].o2 = o2; S.dirty = true; break; }
    case 3: { int he = max(0, min(100 - S.gasList[1].o2, S.gasList[1].he + dir)); S.gasList[1].he = he; S.dirty = true; break; }
    case 4: S.gasList[1].enabled = !S.gasList[1].enabled; S.dirty = true; break; // toggle: either direction
    case 5: { int o2 = max(1, min(100 - S.gasList[2].he, S.gasList[2].o2 + dir)); S.gasList[2].o2 = o2; S.dirty = true; break; }
    case 6: { int he = max(0, min(100 - S.gasList[2].o2, S.gasList[2].he + dir)); S.gasList[2].he = he; S.dirty = true; break; }
    case 7: S.gasList[2].enabled = !S.gasList[2].enabled; S.dirty = true; break;
    // 8-10: per-gas A/E record label (see gasLabel()) -- appended after the
    // original 8 items rather than interleaved, so existing gasCur positions
    // above are untouched. Toggle: either direction.
    case 8: S.gasList[0].labelE = !S.gasList[0].labelE; S.dirty = true; break;
    case 9: S.gasList[1].labelE = !S.gasList[1].labelE; S.dirty = true; break;
    case 10: S.gasList[2].labelE = !S.gasList[2].labelE; S.dirty = true; break;
  }
}

void handleDown(); // fwd decl

void buttons() {
  bool d3 = halEdge(BTN_DOWN), d2 = halEdge(BTN_MODE), d0 = halEdge(BTN_UP), d1 = halEdge(BTN_SELECT);
  // any press dismisses the forced popup early; ongoing alarm sound/border/icon are unaffected
  if ((d3 || d2 || d0 || d1) && halMillis() < S.popupUntil) S.popupUntil = 0;
  if (S.gasSwitchUI) { // top priority: the gas-switch picker owns all 4 buttons while open
    if (d3) cycleGasSwitchSel(1);
    if (d0) cycleGasSwitchSel(-1);
    if (d1) confirmGasSwitch();
    if (d2) S.gasSwitchUI = false; // cancel, no switch made
    return;
  }
  if (d3) { // DOWN: menu=increase highlighted item; surface=next preview page; dive=bookmark/set-bearing (unchanged)
    if (S.mode == "MENU" && S.gasSetup) gasSetupAdjust(1);
    else if (S.mode == "MENU") menuAdjust(1);
    else handleDown();
  }
  if (d2) {
    if (S.showLog != "") S.showLog = "";
    else if (S.mode == "MENU" && S.gasSetup) S.gasSetup = false;
    else if (S.mode == "SURFACE" && !S.compLocked) S.mode = "MENU";
    else if (S.mode == "MENU") { if (S.dirty) saveSet(); S.mode = "SURFACE"; S.cur = 0; }
    else if (S.mode == "DIVE") S.view = (S.view + 1) % 5;
  }
  if (d0) { // UP: menu=decrease highlighted item; surface=prev preview page; log browser=prev entry/page
    if (S.mode == "MENU" && S.gasSetup) gasSetupAdjust(-1);
    else if (S.mode == "MENU") menuAdjust(-1);
    else if (S.showLog == "list") S.logCur = max(0, S.logCur - 1);
    else if (S.showLog == "detail") S.logPage = (S.logPage + 2) % 3;
    else if (S.mode == "SURFACE" && !S.compLocked) S.surfPage = (S.surfPage + 7) % 8;
  }
  if (d1) {
    if (S.mode == "MENU" && S.gasSetup) S.gasCur = (S.gasCur + 1) % 11; // SELECT: move cursor within the gas-setup sub-screen
    else if (S.mode == "MENU") S.cur = (S.cur + 1) % 10; // SELECT: move cursor to next main-menu item
    else if (S.showLog == "list") { if (logsCount > 0) { S.showLog = "detail"; S.logPage = 0; } }
    else if (S.showLog == "detail") S.showLog = "list";
    else if (S.showLog == "" && (S.mode == "DIVE" || S.mode == "SURFACE")) openGasSwitch();
  }
}

void edge(bool &prevState, bool v, const char *n, int f, int ms) { if (v && !prevState) beep(n, f, ms); prevState = v; }

struct AlarmPattern { int freq; int ms; };
// index matches ALARM key strings below: lock,ceil,ppo2,ppo2lo,ascent,cns,table,fault
static const AlarmPattern ALARM_PATTERNS[8][6] = {
  /*lock  */ {{400,300},{0,150},{400,300},{0,600},{0,0},{0,0}},
  /*ceil  */ {{1800,100},{0,60},{1800,100},{0,60},{1800,100},{0,500}},
  /*ppo2  */ {{2000,150},{0,80},{1200,150},{0,600},{0,0},{0,0}},
  /*ppo2lo*/ {{700,200},{0,120},{700,200},{0,120},{700,200},{0,600}}, // slower/lower-pitched than high-PPO2, distinguishable by ear
  /*ascent*/ {{1000,90},{0,50},{1300,90},{0,50},{1600,90},{0,500}},
  /*cns   */ {{900,180},{0,140},{1400,180},{0,700},{0,0},{0,0}},
  /*table */ {{1400,150},{0,100},{1400,150},{0,600},{0,0},{0,0}},
  /*fault */ {{500,80},{0,80},{500,80},{0,80},{500,80},{0,600}},
};
static const int ALARM_PATTERN_LEN[8] = {4,6,4,6,6,4,4,6};
static const char *ALARM_KEYS[8] = {"lock","ceil","ppo2","ppo2lo","ascent","cns","table","fault"};
static const char *CRIT_LABELS[8] = {"COMPUTER LOCKED","CEILING BREACH!","HIGH PPO2!","LOW PPO2!","ASCENT TOO FAST!","CNS HIGH!","TABLE LIMIT EXCEEDED","SENSOR FAULT"};
int alarmIdx(const String &key) { for (int i = 0; i < 8; i++) if (key == ALARM_KEYS[i]) return i; return -1; }

// All mandatory/critical alarms share one color (red) and all advisory
// warnings share another (amber) -- two consistent tiers rather than a
// different hue per alarm type, so the diver reads "critical" vs "advisory"
// at a glance underwater without having to learn/recall a color-per-alarm
// mapping. (Every entry here is 'crit' -- see divesim.html's ALARM_COL.)
int alarmCol(const String &key) { (void)key; return COL_CRIT; }
static const int GAS_COL[3] = {COL_GAS1, COL_GAS2, COL_GAS3}; // per-gas-slot accent color, keyed by gas index
int themeFor() { // subtle background shift by context (surface/dive/menu)
  if (S.mode == "DIVE") return COL_BGDIVE;
  if (S.mode == "MENU" || S.mode == "SYNC") return COL_BGMENU;
  return COL_BG;
}

// priority order: most safety-critical first
String activeAlarmKey() {
  if (S.compLocked) return "lock";
  if (S.wpLow) return "ppo2lo"; // hypoxia risk ranks above high-PPO2/ceiling: unconsciousness can happen with no other warning signs
  if (S.wc) return "ceil";
  if (S.wp) return "ppo2";
  if (S.wa) return "ascent";
  if (S.wcns) return "cns";
  if (S.texc) return "table";
  if (S.fault) return "fault";
  return "";
}

void driveAlarmSound(unsigned long now) {
  String key = activeAlarmKey();
  if (key != S.alarmActive) { // (re)start the pattern from step 0 whenever the active alarm changes
    S.alarmActive = key; S.alarmStep = 0; S.alarmNextAt = 0; S.alarmCycle = 0;
    if (key != "") {
      int ai = alarmIdx(key);
      beep(key.c_str(), ALARM_PATTERNS[ai][0].freq, ALARM_PATTERNS[ai][0].ms); // log the first note as the alert-log entry
      S.popupMsg = CRIT_LABELS[ai]; S.popupUntil = now + 4000; // one-time forced popup; ongoing sound/border/icon are separate and unaffected by this
    }
  }
  if (key == "" || S.mode != "DIVE") return; // repeating chimes only while diving; on the surface (e.g. a 48h lockout) only the one-time beep + popup fire
  if (now >= S.alarmNextAt) {
    int ai = alarmIdx(key);
    const AlarmPattern &p = ALARM_PATTERNS[ai][S.alarmStep];
    if (p.freq > 0 && !(S.alarmStep == 0 && S.alarmCycle == 0)) chime(p.freq, p.ms); // only the very first note was already played by the transition beep
    S.alarmNextAt = now + p.ms;
    S.alarmStep = (S.alarmStep + 1) % ALARM_PATTERN_LEN[ai];
    if (S.alarmStep == 0) S.alarmCycle++;
  }
}

void bookmark() {
  if (S.bookmarksCount < 6) { S.bookmarks[S.bookmarksCount++] = { round(S.depth*10)/10.0, S.dur, S.hdg }; }
  else { for (int i = 1; i < 6; i++) S.bookmarks[i-1] = S.bookmarks[i]; S.bookmarks[5] = { round(S.depth*10)/10.0, S.dur, S.hdg }; }
  S.toast = "BOOKMARK SAVED"; S.toastUntil = halMillis() + 1500;
  beep("BOOKMARK", 1600, 120);
}

void handleDown() {
  if (S.showLog == "list") { int n = logsCount; S.logCur = min(max(0, n - 1), S.logCur + 1); return; }
  if (S.showLog == "detail") { S.logPage = (S.logPage + 1) % 3; return; }
  // (S.mode=='MENU' is unreachable here -- buttons() handles DOWN in MENU via menuAdjust/gasSetupAdjust before ever calling handleDown)
  if (S.mode == "SURFACE" && !S.compLocked) S.surfPage = (S.surfPage + 1) % 8;
  else if (S.mode == "DIVE") {
    if (S.view == 4) {
      if (S.refHdg < 0) { S.refHdg = S.hdg; beep("BEARING SET", 900, 150); }
      else { S.refHdg = -1; beep("BEARING CLEAR", 500, 150); }
    } else bookmark();
  }
}

void alerts() {
  edge(pw.ss, S.ss, "SAFETY STOP", 700, 150);
  edge(pw.ds, S.ds, "DEEP STOP", 700, 150);
  edge(pw.wbat, S.wbat, "BATTERY LOW", 600, 350);
  edge(pw.wndl2, S.wndl2, "NDL 2 MIN", 1000, 250);
}

void T(int f, int x, int y, const String &s, int col = COL_NONE) { u8g2::setFont(f); u8g2::drawStr(x, y, s, col); }
String mmss(long s) {
  if (s < 0) s = 0;
  char buf[8]; snprintf(buf, sizeof(buf), "%02ld:%02ld", s/60, s%60);
  return String(buf);
}

void dots(int view) { for (int i = 0; i < 5; i++) { int y = 44 + i*4; if (view == i) u8g2::drawBox(125, y, 3, 3); else u8g2::drawBox(126, y+1, 1, 1); } }

/* Tissue chart: bar = loading vs surface M-value (0% = surface pressure,
 * 100% = raw M-value); dotted line = GF hi (NDL limit) */
void vsi(unsigned long now) {
  int cx0 = 1, yc = 31, half = 11;
  int col = S.wa ? (blinkOn(now, 180) ? COL_CRIT : COL_BG) : COL_PRIMARY;
  u8g2::drawBox(cx0, yc, 4, 1, col);
  double v = max(-1.0, min(1.0, S.rate / 18.0));
  int h = (int)round(fabs(v) * half);
  if (h > 0) { if (v > 0) u8g2::drawBox(cx0, yc - h, 4, h, col); else u8g2::drawBox(cx0, yc + 1, 4, h, col); }
}

String gasHdr() { return S.heFrac > 0 ? (String(S.fo2) + "/" + String((int)round(S.heFrac*100)) + "%") : ("O2:" + String(S.fo2) + "%"); }

String menuLabel(int i, int fo, const String &pp, int lk) {
  (void)fo;
  switch (i) {
    case 0: return "PPO2:" + pp;
    case 1: return String("DeepStop:") + (S.deep ? "ON" : "OFF");
    case 2: return S.lock ? (String(ALG[S.algo]) + " LK" + String(lk) + "h") : (String("Algo:") + ALG[S.algo]);
    case 3: return "Mode:" + (S.diveMode == "OC" ? ocLabel() : S.diveMode);
    case 4: return "PC Sync";
    case 5: return "Dive Log";
    case 6: return "GFlo:" + String((int)round(S.ugf0*100));
    case 7: return "GFhi:" + String((int)round(S.ugf1*100));
    case 8: return String("Units:") + (isImp() ? "Imperial" : "Metric");
    case 9: return "Gas Setup >";
  }
  return "";
}

// Returns the count of active critical labels into `out` (capacity 8) and
// returns how many were written -- matches JS critList()'s array.
int critList(String out[8]) {
  int n = 0;
  if (S.compLocked) out[n++] = "COMPUTER LOCKED";
  if (S.fault) out[n++] = "SENSOR FAULT";
  if (S.wpLow) out[n++] = "LOW PPO2!";
  if (S.wc) out[n++] = "CEILING BREACH!";
  if (S.wp) out[n++] = "HIGH PPO2!";
  if (S.wa) out[n++] = "ASCENT TOO FAST!";
  if (S.texc) out[n++] = "TABLE LIMIT EXCEEDED";
  if (S.wcns) out[n++] = "CNS HIGH!";
  return n;
}

void drawCritBorder(unsigned long now, int wcCount) {
  if (wcCount > 0 && blinkOn(now, 180)) {
    int col = alarmCol(activeAlarmKey());
    u8g2::drawHLine(0, 0, 128, col); u8g2::drawHLine(0, 63, 128, col);
    u8g2::drawBox(0, 0, 2, 64, col); u8g2::drawBox(126, 0, 2, 64, col);
  }
}

// One-time forced popup: appears in front of whatever page is showing when a
// NEW critical alarm starts, auto-clears after 4s or on any button press.
// Does not recur while the same alarm stays active, and never blocks paging.
void drawAlertPopup(unsigned long now) {
  if (now >= S.popupUntil) return;
  int col = alarmCol(S.alarmActive);
  u8g2::drawBox(2, 18, 124, 28, col);
  T(FONT_6x12, 6, 32, S.popupMsg, COL_BG);
  T(FONT_5x7, 6, 44, "press any button", COL_BG);
}

void chart() {
  int y0 = 57, H = 11;
  for (int x = 0; x < 120; x += 3) {
    u8g2::drawBox(x, y0, 1, 1, COL_DIM);
    u8g2::drawBox(x, y0 - (int)round(S.gf1 * H), 1, 1, COL_DIM);
  }
  for (int i = 0; i < 16; i++) {
    double m = A[i] + P_SURF/B[i] - P_SURF;
    double g = max(-.4, min(1.1, (P[i] - P_SURF) / m));
    int x = i*7 + 4;
    int col = g >= 1.0 ? COL_CRIT : g >= 0.7 ? COL_MINOR : COL_OK;
    if (g >= 0) { int h = max(1, (int)round(g*H)); u8g2::drawBox(x, y0 - h, 5, h, col); }
    else { int h = max(1, (int)round(-g*H)); u8g2::drawBox(x, y0 + 1, 5, h, COL_DIM); }
  }
}

// Rotating-card compass: N/E/S/W and the reference bearing move around the
// ring as you turn; a single fixed pointer at the top ("lubber line") always
// means "straight ahead" (the direction you're currently facing). This is
// the standard handheld-compass convention and avoids reading a moving
// needle against fixed labels.
void compassScreen(unsigned long now, bool surfacePreview) {
  (void)now;
  int F = FONT_6x10;
  if (surfacePreview) { T(F, 0, 9, "COMPASS PREVIEW", COL_DIM); }
  else {
    T(F, 0, 9, "DEPTH " + dS(S.depth) + dU(), COL_DIM);
    String cs = "CEIL " + cS(S.ceil) + dU();
    String ndlStr = "NDL " + String(S.ndl);
    T(F, 70, 9, S.deco ? (cs.length() > 9 ? ("CEIL " + cS(S.ceil)) : cs) : ndlStr, (S.ndl <= 2 && !S.deco) ? COL_MINOR : COL_DIM);
  }
  u8g2::drawHLine(0, 11, 120, COL_DIM);
  int cx0 = 55, cy0 = 39, r = 18;
  u8g2::drawCircle(cx0, cy0, r, COL_DIM);
  auto rot = [&](double trueBearing) { return (trueBearing - S.hdg - 90) * PI / 180.0; }; // screen angle for a fixed compass bearing, given current heading
  struct { int ang; const char *lbl; } cards[4] = {{0,"N"},{90,"E"},{180,"S"},{270,"W"}};
  for (auto &c : cards) {
    double a = rot(c.ang);
    T(FONT_5x7, (int)(cx0 + cos(a)*(r+3) - 3), (int)(cy0 + sin(a)*(r+3) + 3), c.lbl, COL_DIM);
  }
  int diag[4] = {45,135,225,315};
  for (int ang : diag) {
    double a = rot(ang);
    double x0 = cx0 + cos(a)*(r-4), y0 = cy0 + sin(a)*(r-4), x1 = cx0 + cos(a)*r, y1 = cy0 + sin(a)*r;
    u8g2::drawLine((int)x0, (int)y0, (int)x1, (int)y1, COL_DIM);
  }
  u8g2::drawLine(cx0, cy0, cx0, cy0 - r, COL_PRIMARY); // fixed lubber line: always "ahead"
  if (S.refHdg >= 0) {
    double ar = rot(S.refHdg);
    u8g2::drawLine((int)(cx0+cos(ar)*(r-6)), (int)(cy0+sin(ar)*(r-6)), (int)(cx0+cos(ar)*r), (int)(cy0+sin(ar)*r), COL_MINOR); // bearing marker on the ring
    int delta = ((S.hdg - S.refHdg + 540) % 360) - 180;
    char hdgBuf[4]; snprintf(hdgBuf, sizeof(hdgBuf), "%03d", S.hdg);
    T(FONT_5x7, 0, 60, "REF " + String(S.refHdg) + "* " + (delta >= 0 ? "+" : "") + String(delta) + "* HDG " + String(hdgBuf) + "*", COL_MINOR);
  } else {
    char hdgBuf[4]; snprintf(hdgBuf, sizeof(hdgBuf), "%03d", S.hdg);
    T(FONT_5x7, 0, 60, "HDG " + String(hdgBuf) + "*", COL_DIM);
  }
}

void diveBody(unsigned long now, int view); // fwd decl

void render(unsigned long now) {
  u8g2::clearBuffer(themeFor());
  int F = FONT_6x10, fo = S.fo2;
  char ppBuf[8]; snprintf(ppBuf, sizeof(ppBuf), "%.2f", S.ppo2/100.0); String pp = ppBuf;
  int lk = (int)ceil(S.lock / 3600.0);

  if (S.gasSwitchUI) {
    String wc[8]; int wn = critList(wc);
    drawCritBorder(now, wn);
    T(F, 0, 9, "SWITCH GAS"); u8g2::drawHLine(0, 11, 128);
    int best = bestGasIdx(S.depth);
    int en[3]; int n = enabledGasIndices(en);
    for (int row = 0; row < n; row++) {
      int gi = en[row]; const GasCfg &g = S.gasList[gi];
      String line = String(S.gasSwitchSel == gi ? ">" : " ") + "Gas" + String(gi+1) + ": " + String(g.o2) + (g.he ? "/" + String(g.he) : "") + "%" + (gi == S.activeGas ? " (current)" : "");
      T(FONT_5x7, 0, 22 + row*9, line, GAS_COL[gi]);
      if (gi == best) T(FONT_5x7, 118, 22 + row*9, "B", COL_OK); // "B" = best mix for the current depth
    }
    T(FONT_5x7, 0, 63, "SEL:confirm MODE:cancel");
  }
  else if (S.mode == "SURFACE" && S.showLog == "list") {
    T(F, 0, 9, "DIVE LOG (" + String(logsCount) + ")"); u8g2::drawHLine(0, 11, 128);
    if (logsCount == 0) T(FONT_5x7, 0, 22, "No dives logged yet");
    // JS reverses the array (most recent first); logs[] here is oldest-first, so index from the end.
    int shown = min(logsCount, 6);
    for (int i = 0; i < shown; i++) {
      const DiveLogEntry &e = logs[logsCount - 1 - i];
      String line = String(S.logCur == i ? ">" : " ") + mmss(e.durSec) + " " + dS(e.maxD) + dU() + " O2" + String(e.gases[0].fo2) + "% " + String(ALG[e.algo]).substring(0,4) + (e.deco ? " DECO" : "");
      T(FONT_5x7, 0, 20 + i*8, line);
    }
    T(FONT_5x7, 0, 62, "SEL: open  MODE: back");
  }
  else if (S.mode == "SURFACE" && S.showLog == "detail") {
    int idx = min(S.logCur, logsCount - 1);
    if (logsCount == 0 || idx < 0) { S.showLog = "list"; }
    else {
      const DiveLogEntry &e = logs[logsCount - 1 - idx];
      T(F, 0, 9, "DIVE #" + String(logsCount - idx) + " (" + String(S.logPage+1) + "/3)"); u8g2::drawHLine(0, 11, 128);
      if (S.logPage == 0) {
        T(F, 0, 23, "Time: " + mmss(e.durSec));
        T(FONT_5x7, 0, 33, "Max: " + dS(e.maxD) + dU() + "  Avg: " + dS(e.avgD) + dU());
        int shownG = min(e.gasesCount, 2);
        String parts = "";
        for (int i = 0; i < shownG; i++) { if (i) parts += ","; parts += String(e.gases[i].fo2) + (e.gases[i].he ? "/" + String(e.gases[i].he) : "") + "%"; }
        String extra = e.gasesCount > 2 ? ("+" + String(e.gasesCount - 2)) : "";
        T(FONT_5x7, 0, 43, "Gas: " + parts + extra);
        T(FONT_5x7, 0, 53, "Algo: " + String(ALG[e.algo]) + " T:" + tS(e.avgTemp));
      } else if (S.logPage == 1) {
        T(F, 0, 23, "GF used: " + String((int)round(e.gfUsed[0]*100)) + "/" + String((int)round(e.gfUsed[1]*100)));
        T(FONT_5x7, 0, 33, !e.hasSurfInterval ? "Surface int: first dive" : ("Surface int: " + mmss(e.surfIntervalSec)));
        char spBuf[16]; snprintf(spBuf, sizeof(spBuf), "%.3f", e.surfPressure);
        T(FONT_5x7, 0, 43, "Surface pres: " + String(spBuf) + " bar");
        T(FONT_5x7, 0, 53, "Tissue N2 (start->end):");
        for (int i = 0; i < 16; i++) {
          int x = i*7 + 4;
          int sH = max(1, (int)round(min(1.0, (e.tissueStartN2[i]-P_SURF)/1.5) * 8));
          int eH = max(1, (int)round(min(1.0, (e.tissueEndN2[i]-P_SURF)/1.5) * 8));
          u8g2::drawBox(x, 63-sH, 2, sH, COL_DIM);
          u8g2::drawBox(x+3, 63-eH, 2, eH, COL_PRIMARY);
        }
      } else {
        if (e.profileCount < 2) T(F, 0, 35, "No profile recorded");
        else {
          double maxD = max(0.1, e.maxD);
          T(FONT_5x7, 0, 17, "0", COL_DIM);
          T(FONT_5x7, 0, 60, dS(maxD) + dU(), COL_DIM);
          for (int i = 1; i < e.profileCount; i++) {
            const ProfilePt &p0 = e.profile[i-1], &p1 = e.profile[i];
            int x0 = 4 + (int)round((double)p0.t / e.durSec * 118), y0 = 16 + (int)round(p0.d / maxD * 44);
            int x1 = 4 + (int)round((double)p1.t / e.durSec * 118), y1 = 16 + (int)round(p1.d / maxD * 44);
            u8g2::drawLine(x0, y0, x1, y1, COL_PRIMARY);
          }
        }
      }
      T(FONT_5x7, 0, 62, "UP/DN: page  SEL: back");
    }
  }
  else if (S.mode == "SURFACE" && S.surfPage == 1) {
    T(F, 0, 10, "SURFACE - STATUS"); u8g2::drawHLine(0, 12, 128);
    T(F, 0, 24, "Battery: " + String(S.batt, 1) + "%");
    T(F, 0, 34, "CNS O2: " + String(S.cns, 1) + "%");
    T(F, 0, 44, "Comp lock: " + (S.compLocked ? (String((int)ceil(S.lockout/3600.0)) + "h") : "none"));
    if (logsCount > 0) { const DiveLogEntry &last = logs[logsCount-1]; T(F, 0, 54, "Last dive: " + mmss(last.durSec) + " " + dS(last.maxD) + dU()); }
    else T(F, 0, 54, "Last dive: none");
    T(FONT_5x7, 0, 63, "UP/DOWN: page  MODE: menu");
  }
  else if (S.mode == "SURFACE" && S.surfPage >= 2 && S.surfPage <= 6) {
    int pv = S.surfPage - 2;
    if (pv == 4) compassScreen(now, true); else diveBody(now, pv);
  }
  else if (S.mode == "SURFACE" && S.surfPage == 7) {
    T(F, 0, 10, "SURFACE GF"); u8g2::drawHLine(0, 12, 128);
    double sgf = surfaceGF();
    int col = sgf >= 100 ? COL_CRIT : sgf >= 80 ? COL_MINOR : COL_PRIMARY;
    T(FONT_L22, 30, 40, String((int)round(sgf)) + "%", col);
    T(FONT_5x7, 0, 52, sgf >= 100 ? "CEILING VIOLATION" : (sgf >= 80 ? "HIGH - check ascent rate" : "Normal"), col);
    T(FONT_5x7, 0, 63, "UP/DOWN: page  MODE: menu");
  }
  else if (S.mode == "SURFACE") {
    T(F, 0, 10, "SURFACE / PLANNER"); u8g2::drawHLine(0, 12, 128);
    T(F, 0, 24, "NO FLY: " + String(S.nofly/3600) + "h" + String((S.nofly%3600)/60) + "m");
    {
      String gh = gasHdr(), lbl = " PPO2:";
      T(F, 0, 34, gh, GAS_COL[S.activeGas]);
      T(F, gh.length()*6, 34, lbl);
      int ppCol = (S.wpLow || S.wp) && blinkOn(now, 180) ? COL_CRIT : COL_PRIMARY;
      T(F, (gh.length()+lbl.length())*6, 34, pp, ppCol);
      T(F, (gh.length()+lbl.length())*6 + pp.length()*6, 34, " G" + String(S.activeGas+1));
    }
    T(F, 0, 44, "MOD:" + dS(S.mod) + dU() + " Dives:" + String(S.dcount));
    T(F, 0, 54, "ALG:" + String(ALG[S.algo]) + (S.lock ? (" LK" + String(lk) + "h") : ""));
    if (S.compLocked) T(FONT_6x12, 0, 63, "LOCKED " + String((int)ceil(S.lockout/3600.0)) + "h", COL_CRIT);
    else if (S.fault) T(F, 0, 63, "SENSOR FAULT", COL_CRIT);
    else {
      String bt = "BAT:" + String((int)round(S.batt)) + "% ";
      T(F, 0, 63, bt);
      T(F, bt.length()*6, 63, "CNS:" + String((int)round(S.cns)) + "%", (S.wcns && blinkOn(now,180)) ? COL_CRIT : COL_PRIMARY);
    }
  }
  else if (S.mode == "MENU" && S.gasSetup) {
    T(F, 0, 9, "GAS SETUP"); u8g2::drawHLine(0, 11, 128);
    String labels[11] = {
      "Gas1 O2:" + String(S.gasList[0].o2) + "%", "Gas1 He:" + String(S.gasList[0].he) + "%",
      "Gas2 O2:" + String(S.gasList[1].o2) + "%", "Gas2 He:" + String(S.gasList[1].he) + "%", String("Gas2:") + (S.gasList[1].enabled ? "ON" : "OFF"),
      "Gas3 O2:" + String(S.gasList[2].o2) + "%", "Gas3 He:" + String(S.gasList[2].he) + "%", String("Gas3:") + (S.gasList[2].enabled ? "ON" : "OFF"),
      String("Gas1 Label:") + (gasLabel(S.gasList[0]) == 'E' ? "EANx21" : "Air"),
      String("Gas2 Label:") + (gasLabel(S.gasList[1]) == 'E' ? "EANx21" : "Air"),
      String("Gas3 Label:") + (gasLabel(S.gasList[2]) == 'E' ? "EANx21" : "Air"),
    };
    static const int rowGas[11] = {0,0,1,1,1,2,2,2,0,1,2}; // which gas slot each row belongs to, for per-gas coloring
    int total = 11, winSize = 6, start = max(0, min(S.gasCur - 2, total - winSize));
    for (int row = 0; row < winSize; row++) {
      int i = start + row; if (i >= total) break;
      T(FONT_5x7, 0, 18 + row*7, String(S.gasCur == i ? ">" : " ") + labels[i], GAS_COL[rowGas[i]]);
    }
    T(FONT_5x7, 0, 63, "MODE: back to menu");
  }
  else if (S.mode == "MENU") {
    T(F, 0, 9, "SETTINGS MENU"); u8g2::drawHLine(0, 11, 128);
    int total = 10, winSize = 6, start = max(0, min(S.cur - 2, total - winSize));
    for (int row = 0; row < winSize; row++) {
      int i = start + row; if (i >= total) break;
      T(FONT_5x7, 0, 18 + row*7, String(S.cur == i ? ">" : " ") + String(i+1) + " " + menuLabel(i, fo, pp, lk));
    }
    if (S.compLocked) T(FONT_6x12, 0, 63, "** LOCKED **");
    else T(FONT_5x7, 96, 63, String(S.cur+1) + "/" + String(total));
  }
  else if (S.mode == "SYNC") { T(FONT_6x12, 10, 25, "PC DEBUGGER"); T(FONT_6x12, 10, 45, "SYNCING LOGS..."); }
  else {
    if (S.view == 4) {
      compassScreen(now, false); dots(4);
      String wc[8]; int wn = critList(wc);
      drawCritBorder(now, wn); drawAlertPopup(now);
      u8g2::sendBuffer(); return;
    }
    diveBody(now, S.view);
  }
  if (halMillis() < S.toastUntil) { u8g2::drawBox(20, 26, 88, 10, COL_OK); T(FONT_5x7, 24, 33, S.toast, COL_BG); }
  drawAlertPopup(now);
  u8g2::sendBuffer();
}

// Shared by live DIVE mode and the surface preview pages. `view` is passed
// explicitly (rather than read from S.view) so the surface preview can show
// any of the 4 non-compass dive screens without S actually being in DIVE mode.
void diveBody(unsigned long now, int view) {
  int F = FONT_6x10;
  char ppBuf[8]; snprintf(ppBuf, sizeof(ppBuf), "%.2f", S.ppo2/100.0); String pp = ppBuf;
  String WC[8]; int wcCount = critList(WC);
  String WM[8]; int wmCount = 0;
  if (S.wndl2 && !S.deco) WM[wmCount++] = "NDL 2 MIN";
  if (S.wbat) WM[wmCount++] = "BATTERY LOW";
  bool crit = wcCount > 0;
  drawCritBorder(now, wcCount);
  if (!crit && wmCount > 0 && blinkOn(now, 500)) { u8g2::drawHLine(0, 0, 128, COL_MINOR); u8g2::drawHLine(0, 63, 128, COL_MINOR); }

  char hdgBuf[4]; snprintf(hdgBuf, sizeof(hdgBuf), "%03d", S.hdg);
  T(F, 0, 9, String(hdgBuf) + "* " + S.card, COL_DIM);
  T(FONT_5x7, 44, 9, S.diveMode == "OC" ? ocLabel() : S.diveMode, COL_WHITE);
  T(F, 62, 9, gasHdr(), GAS_COL[S.activeGas]);
  String tm = mmss(S.dur);
  T(F, 128 - 6*tm.length(), 9, tm, COL_DIM);
  u8g2::drawHLine(0, 11, 128, COL_DIM);

  // VSI now sits left of the depth digits (not between depth and NDL); depth
  // shifts right (x=9) to clear it, on every page.
  vsi(now);

  // On the alerts/stops page (view0) the header is always the original full
  // layout, unchanged. On every other page, an active stop condition
  // compacts the NDL/CEIL slot so the per-view info below still has room:
  // DECO/deep ceiling -> ceiling shrinks to the label row, countdown grows;
  // safety/advisory stop -> NDL shrinks to the label row, small "stop mm:ss".
  bool nearCeiling = S.deco && S.depth <= S.ceil + 1.5; // ceiling only shrinks once genuinely within 1.5m of it
  bool compact = view != 0 && (nearCeiling || S.ss || S.ds);
  if (!compact) {
    T(FONT_5x7, 9, 19, "DEPTH " + dU(), COL_DIM);
    T(FONT_5x7, 80, 19, S.deco ? ("CEIL " + dU()) : "NDL min", COL_DIM);
    T(FONT_L22, 9, 41, dS(S.depth), COL_WHITE);
    if (crit && blinkOn(now, 180)) T(FONT_6x12, 68, 19, "!", COL_CRIT);
    if (S.deco) T(FONT_L22, 80, 41, cS(S.ceil), COL_DECO);
    else if (S.ndl >= 99) { T(FONT_L22, 80, 41, "99"); T(F, 108, 41, "+"); }
    else T(FONT_L22, 80, 41, String(S.ndl), S.ndl <= 2 ? COL_MINOR : COL_PRIMARY);
  } else if (nearCeiling) {
    T(FONT_5x7, 9, 19, "DEPTH " + dU(), COL_DIM);
    T(FONT_5x7, 68, 19, "CEIL", COL_DIM);
    T(FONT_7x14, 92, 24, cS(S.ceil), COL_DECO);
    T(FONT_L22, 9, 41, dS(S.depth), COL_WHITE);
    long rem = max(0L, (long)(S.nmin*60 - S.atDepth));
    T(FONT_5x7, 64, 37, "Stop", COL_MINOR);
    T(FONT_7x14, 88, 37, mmss(rem), COL_DECO);
  } else {
    T(FONT_5x7, 9, 19, "DEPTH " + dU(), COL_DIM);
    T(FONT_5x7, 68, 19, "NDL", COL_DIM);
    T(FONT_7x14, 88, 24, String(S.ndl), COL_PRIMARY);
    T(FONT_L22, 9, 41, dS(S.depth), COL_WHITE);
    if (S.ds) { long rem = max(0L, (long)(120 - S.dsEl)); T(FONT_5x7, 70, 37, "deep " + mmss(rem), COL_MINOR); } // was wrongly using the safety-stop timer even during a deep stop
    else { long rem = max(0L, (long)(180 - S.ssEl)); T(FONT_5x7, 70, 37, "stop " + mmss(rem), COL_MINOR); }
  }
  u8g2::drawHLine(0, 43, 128, COL_DIM);
  dots(view);

  bool haveBanner = crit ? wcCount > 0 : wmCount > 0;
  String banner = crit ? WC[(now/1500) % wcCount] : (wmCount ? WM[(now/1500) % wmCount] : "");
  int bannerCol = crit ? (blinkOn(now,180) ? alarmCol(activeAlarmKey()) : COL_BG) : (wmCount ? (blinkOn(now,500) ? COL_MINOR : COL_BG) : COL_NONE);
  if (view != 0) { /* nothing drawn here, matches JS no-op branch */ }
  else if (haveBanner) T(FONT_6x12, 0, 58, banner, bannerCol);
  else if (S.deco) {
    String a = "STOP " + cS(S.nstop) + dU() + " " + String(S.nmin) + "min TTS" + String(S.tts);
    T(F, 0, 58, a.length() <= 21 ? a : ("STOP " + cS(S.nstop) + dU() + " " + String(S.nmin) + "' TTS" + String(S.tts)), COL_DECO);
  }
  else if (S.ss) T(F, 0, 58, "STOP " + String(isImp() ? "15ft" : "5m") + " | " + mmss((long)max(0.0, 180-S.ssEl)), COL_MINOR);
  else if (S.ds) T(F, 0, 58, "DEEP " + dI(S.dsD) + dU() + " | " + mmss((long)max(0.0, 120-S.dsEl)), COL_MINOR);
  else chart();

  if (view == 1) {
    String gh = gasHdr(), lbl = " PPO2:";
    T(F, 0, 53, gh, GAS_COL[S.activeGas]);
    T(F, gh.length()*6, 53, lbl);
    int ppCol = (S.wpLow || S.wp) && blinkOn(now, 180) ? COL_CRIT : COL_PRIMARY;
    char ppnBuf[8]; snprintf(ppnBuf, sizeof(ppnBuf), "%.2f", S.ppo2n);
    T(F, (gh.length()+lbl.length())*6, 53, ppnBuf, ppCol);
    int best = bestGasIdx(S.depth);
    String line = "MOD:" + dS(S.mod) + dU() + " G" + String(S.activeGas+1) + "/3";
    T(F, 0, 63, line, GAS_COL[S.activeGas]);
    if (best >= 0 && best != S.activeGas) T(F, line.length()*6, 63, " B" + String(best+1), COL_OK); // a richer/leaner gas than the active one is better for this depth
  }
  else if (view == 2) {
    if (S.deco) {
      String sc = "";
      for (int i = 0; i < S.stopsCount; i++) {
        String q = cS(S.stops[i].d) + dU() + "/" + String(S.stops[i].m) + "'";
        String tryStr = sc + (sc.length() ? " " : "") + q;
        if (tryStr.length() > 25) break;
        sc = tryStr;
      }
      T(FONT_5x7, 0, 53, sc.length() ? sc : "calc...");
      String pre = "TTS:" + String(S.tts) + "m CNS:";
      T(F, 0, 63, pre);
      T(F, pre.length()*6, 63, String((int)round(S.cns)) + "%", (S.wcns && blinkOn(now,180)) ? COL_CRIT : COL_PRIMARY);
    } else {
      T(F, 0, 53, "NDL:" + String(S.ndl) + " @depth:" + mmss((long)S.atDepth));
      String pre = (S.algo == 2 && !S.texc ? String(ALG[2]) : ("GF" + String((int)round(S.gf0*100)) + "/" + String((int)round(S.gf1*100)))) + " CNS:";
      T(F, 0, 63, pre);
      T(F, pre.length()*6, 63, String((int)round(S.cns)) + "%", (S.wcns && blinkOn(now,180)) ? COL_CRIT : COL_PRIMARY);
    }
  }
  else if (view == 3) {
    String rl = S.rate > 0.05 ? "ASCN" : (S.rate < -0.05 ? "DESC" : "STAT");
    double rv = fabs(S.rate);
    T(F, 0, 53, "MAX:" + dS(S.maxD) + dU() + " " + rl + ":" + rS(rv));
    T(F, 0, 63, "TEMP:" + tS(halTemp()));
  }
}

void sampleProfile(unsigned long now) {
  long elMs = (long)(now - S.dstart);
  bool should = S.profileCount == 0 || (elMs - S.profile[S.profileCount-1].t*1000L >= S.profileIv*1000L);
  if (should && S.profileCount < LOG_MAX_PROFILE) {
    S.profile[S.profileCount++] = { (int)round(elMs/1000.0), round(S.depth*10)/10.0 };
    if (S.profileCount >= LOG_MAX_PROFILE) {
      int half = 0;
      for (int i = 0; i < S.profileCount; i += 2) S.profile[half++] = S.profile[i];
      S.profileCount = half;
      S.profileIv *= 2;
    }
  }
}

void tick(unsigned long now, double dt) {
  sensors();
  double fo = S.fo2 / 100.0;
  S.ppo2n = S.pabs * fo;
  S.mod = max(0.0, floor((S.ppo2/100.0/fo - P_SURF) * MPB * 10) / 10.0);
  S.batt = max(0.0, S.batt - dt/86400.0);
  if (S.ppo2n > 0.5) S.cns = min(150.0, S.cns + cnsRatePerMin(S.ppo2n)*dt/60.0);
  else S.cns = max(0.0, S.cns - dt*100.0/5400.0);
  if (S.mode == "DIVE") {
    if (S.atAnchor == 0 || fabs(S.depth - S.atAnchor) > 1.5) { S.atAnchor = S.depth; S.atDepth = 0; }
    else S.atDepth += dt;
  } else { S.atAnchor = 0; S.atDepth = 0; }
  if (!S.fault) {
    double heFr = S.mode == "DIVE" ? S.heFrac : 0;
    leg(P, PHE, S.prev, S.pabs, dt, S.mode == "DIVE" ? (1 - fo - heFr) : FN2_AIR, heFr);
    S.prev = S.pabs;
    rateUpd(now);
    if (S.mode == "DIVE") {
      S.maxD = max(S.maxD, S.depth);
      S.dur = (int)floor((now - S.dstart) / 1000.0);
      S.avgAccD += S.depth*dt; S.avgAccT += dt; S.avgAccTemp += halTemp()*dt;
      sampleProfile(now);
    }
    upd(dt, now);
    if (S.mode == "DIVE") stops(dt);
  }
  if (S.mode != "DIVE") {
    if (!S.fault && S.depth >= .8) { if (!S.desc) S.desc = now; if (now - S.desc >= 3000) { startDive(S.desc); S.desc = 0; } }
    else S.desc = 0;
  } else {
    if (S.depth < .3) { if (!S.surf) S.surf = now; if (now - S.surf >= 60000) endDive(); }
    else S.surf = 0;
  }
  if (S.mode == "SYNC" && now >= S.sync_) S.mode = "SURFACE";
  if (now - lastS >= 1000) {
    lastS = now; saveAcc++;
    if (S.mode != "DIVE") {
      if (S.nofly > 0) S.nofly--;
      if (S.lock > 0) S.lock--;
      if (S.lockout > 0) { S.lockout--; if (S.lockout == 0) S.compLocked = false; }
      if (!S.nofly && !S.lock && (S.dcount || S.viol)) { S.dcount = 0; S.viol = 0; saveState(); }
    }
    if (saveAcc >= (unsigned long)(S.mode == "DIVE" ? 120 : 300)) {
      saveAcc = 0;
      if (S.mode == "DIVE" || S.nofly > 0 || S.lockout > 0) saveState();
    }
  }
  alerts();
  driveAlarmSound(now);
  render(now);
}

void fwSetup() {
  SettingsBlob s{};
  bool hasSet = prefsGetSet(s);
  S.fo2 = hasSet && s.fo2 ? s.fo2 : 21;
  S.ppo2 = hasSet && s.ppo2 ? s.ppo2 : 140;
  S.deep = hasSet && s.deep;
  S.algo = hasSet ? s.algo : 0;
  S.units = hasSet && s.units[0] ? String(s.units) : "met";
  S.diveMode = "OC";
  if (hasSet) { for (int i = 0; i < DIVE_MODES_COUNT; i++) if (String(s.diveMode) == DIVE_MODES[i]) S.diveMode = s.diveMode; }
  if (hasSet) { for (int i = 0; i < 3; i++) S.gasList[i] = s.gasList[i]; }
  S.activeGas = hasSet ? s.activeGas : 0;
  S.heFrac = hasSet ? s.heFrac : 0;
  S.fo2 = S.gasList[S.activeGas].o2; S.heFrac = S.gasList[S.activeGas].he / 100.0; // keep live fo2/heFrac in sync with the stored active slot
  if (!loadState()) { for (int i = 0; i < 16; i++) P[i] = PAIR; }
  double mv = 0;
  for (int i = 0; i < 30; i++) mv += analogMv(0);
  mv /= 30;
  if (mv < 50 || mv > 2450) S.fault = true;
  else if (!(S.zeroValid && fabs(mv - S.zero) > 16.7)) { S.zero = mv; S.zeroValid = true; }
  S.prev = P_SURF; S.pabs = P_SURF;
}

void fwLoop(unsigned long now, double dt) { buttons(); tick(now, dt); }

/* ================= END FIRMWARE ================= */

// ---------------------------------------------------------------------------
// LSM303 heading/temp -- minimal direct implementation. Swap in the
// Adafruit_LSM303DLH_Mag library for a cleaner read once it's wired into the
// build; this does the same tilt-uncompensated atan2() the simulator models.
// ---------------------------------------------------------------------------
#define LSM303_MAG_ADDR 0x1E
float halHeading() {
  Wire.beginTransmission(LSM303_MAG_ADDR);
  Wire.write(0x03); // OUT_X_H_M
  Wire.endTransmission();
  Wire.requestFrom(LSM303_MAG_ADDR, 6);
  if (Wire.available() < 6) return S.hdg; // read failed -- hold last heading rather than glitch to 0
  int16_t xh = Wire.read(), xl = Wire.read();
  int16_t zh = Wire.read(), zl = Wire.read();
  int16_t yh = Wire.read(), yl = Wire.read();
  int16_t x = (xh << 8) | xl, y = (yh << 8) | yl; (void)zh; (void)zl;
  float heading = atan2((float)y, (float)x) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}
float halTemp() {
  // Coarse onboard temp output -- see KNOWN GAPS #3, needs real calibration.
  Wire.beginTransmission(LSM303_MAG_ADDR);
  Wire.write(0x31); // TEMP_OUT_H_M
  Wire.endTransmission();
  Wire.requestFrom(LSM303_MAG_ADDR, 2);
  if (Wire.available() < 2) return 20.0; // read failed -- fall back to a plausible room/water temp rather than garbage
  int8_t th = Wire.read(); uint8_t tl = Wire.read(); (void)tl;
  return 20.0 + th / 8.0; // rough datasheet-typical scaling; needs real calibration against a known reference
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
unsigned long lastLoopMs = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  Wire.begin();

  tft.initR(INITR_144GREENTAB); // adjust tab color per the actual panel
  tft.setRotation(0);
  tft.fillScreen(ST7735_BLACK);
  if (PIN_TFT_BLK >= 0) { pinMode(PIN_TFT_BLK, OUTPUT); digitalWrite(PIN_TFT_BLK, HIGH); }

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
  prefsNVS.begin("divecomp", false);

  initConstants();
  logsLoad();
  fwSetup();

  lastLoopMs = millis();
}

void loop() {
  unsigned long now = millis();
  double dt = (now - lastLoopMs) / 1000.0;
  lastLoopMs = now;
  if (dt > 0) fwLoop(now, dt);
}