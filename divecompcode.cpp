#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Preferences.h>
#include <math.h>

// =====================================================================
// BENCH / SIMULATION FIRMWARE. NOT VALIDATED FOR REAL DIVING.
// Validate against Subsurface (same profiles) before trusting any output.
// =====================================================================

// ---------------------------------------------------------------------
// PINS
// ---------------------------------------------------------------------
#define PIN_ADC_PRESSURE  0
#define PIN_BTN_UP        1   // NEXT
#define PIN_BTN_SELECT    2   // EDIT
#define PIN_BTN_MODE      3   // MENU / BACK / SCREEN
#define PIN_I2C_SDA       8
#define PIN_I2C_SCL       9

U8g2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, PIN_I2C_SCL, PIN_I2C_SDA);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Preferences prefs;

// ---------------------------------------------------------------------
// CONSTANTS
// ---------------------------------------------------------------------
const double P_SURF      = 1.013;    // assumed surface pressure (sea level), bar
const double PH2O        = 0.0627;   // alveolar water vapour, bar
const double FN2_AIR     = 0.7902;
const double M_PER_BAR   = 10.0;     // ~salt water; used in BOTH directions
const double PLAN_ASCENT_M_MIN = 9.0;

const float  ASCENT_WARN_M_MIN  = 9.0f;   // PADI limit
const float  ASCENT_VIOL_M_MIN  = 12.0f;  // counts as a violation

const uint32_t TICK_MS           = 100;
const uint32_t DIVE_START_MS     = 3000;   // >=0.8 m for 3 s
const uint32_t DIVE_END_MS       = 60000;  // <0.3 m for 60 s
const uint32_t ALGO_LOCKOUT_SEC  = 24UL * 3600UL;  // algorithm locked this long after a dive
const uint32_t NOFLY_SINGLE_SEC  = 12UL * 3600UL;
const uint32_t NOFLY_MULTI_SEC   = 18UL * 3600UL;
const uint32_t NOFLY_DECO_SEC    = 24UL * 3600UL;

// Pressure sensor scaling (kept from original: 2.0 V span over 12 bar). VERIFY.
const float SENSOR_SPAN_V     = 2.0f;
const float SENSOR_RANGE_BAR  = 12.0f;
const float SENSOR_MV_MIN     = 50.0f;
const float SENSOR_MV_MAX     = 2450.0f;

const float HEADING_OFFSET_DEG = 0.0f;   // magnetic declination / mounting offset

// ---------------------------------------------------------------------
// BUHLMANN ZHL-16C (N2), 16 compartments (1b for compartment 1)
// ---------------------------------------------------------------------
const int NUM_COMPARTMENTS = 16;
const double HALF_TIME_N2[16] = {5.0, 8.0, 12.5, 18.5, 27.0, 38.3, 54.3, 77.0, 109.0, 146.0, 187.0, 239.0, 305.0, 390.0, 498.0, 635.0};
const double A_N2[16] = {1.1696, 1.0000, 0.8618, 0.7562, 0.6200, 0.5043, 0.4410, 0.4000, 0.3750, 0.3500, 0.3295, 0.3065, 0.2835, 0.2610, 0.2480, 0.2327};
const double B_N2[16] = {0.5578, 0.6514, 0.7222, 0.7825, 0.8126, 0.8434, 0.8693, 0.8910, 0.9092, 0.9222, 0.9319, 0.9403, 0.9477, 0.9544, 0.9602, 0.9653};
static_assert(sizeof(HALF_TIME_N2) / sizeof(double) == 16, "half-time table must have 16 entries");
static_assert(sizeof(A_N2) / sizeof(double) == 16, "A table must have 16 entries");
static_assert(sizeof(B_N2) / sizeof(double) == 16, "B table must have 16 entries");

double K_N2[16];     // decay constants, 1/s
double P_N2[16];     // live tissue N2 pressures, bar
double P_AIR[16];    // air-saturated-at-surface reference

// ---------------------------------------------------------------------
// PADI-STYLE TABLE MODE DATA (Table 1 no-deco limits, metric)
// VERIFY these against your official PADI RDP before use.
// Repetitive-dive adjustment is NOT the official Table 2/3: it is scaled
// from the tissue model (see buildTableLimits()).
// ---------------------------------------------------------------------
const int TBL_N = 12;
const uint8_t  TBL_DEPTH_M[TBL_N]  = {10, 12, 14, 16, 18, 20, 22, 25, 30, 35, 40, 42};
const uint16_t TBL_NDL_MIN[TBL_N]  = {219, 147, 98, 72, 56, 45, 37, 29, 20, 14, 9, 8};
uint16_t tbl_adj_min[TBL_N];

// ---------------------------------------------------------------------
// ENUMS / STATE
// ---------------------------------------------------------------------
enum AlgorithmChoice : uint8_t { ALGO_BUHLMANN_ZHL16C = 0, ALGO_RGBM_SIM = 1, ALGO_PADI_TABLES = 2 };
enum SystemMode { MODE_SURFACE, MODE_DIVE, MODE_MENU, MODE_PC_SYNC };
enum DiveScreenView { SCREEN_MAIN_HUD, SCREEN_COMPASS_TEMP, SCREEN_GAS_DECO };

struct GF { double lo; double hi; };

SystemMode      currentMode     = MODE_SURFACE;
DiveScreenView  currentDiveView = SCREEN_MAIN_HUD;
AlgorithmChoice selectedAlgo    = ALGO_BUHLMANN_ZHL16C;
const char* ALGO_NAMES[3] = {"Buhlmann", "RGBM-sim", "PADI-tbl"};

// Settings (integers avoid float drift: 0.29*100 -> 28)
int  fo2_pct     = 21;      // 21..40
int  ppo2_cbar   = 140;     // 100..160 step 5
bool enable_deep_stop = false;
bool settings_dirty   = false;

// Measurements
float  zero_mv = 250.0f;
bool   zero_mv_valid = false;
float  gauge_f = 0.0f;
float  current_depth_m = 0.0f;
float  max_depth_m = 0.0f;
float  ascent_rate_m_min = 0.0f;
double p_abs_bar = P_SURF;
double prev_p_abs = P_SURF;
double current_ppo2 = 0.21;
double mod_m = 0.0;
bool   sensor_fault = false;
uint8_t bad_count = 0;
bool   mag_ok = false, accel_ok = false;
int    heading_deg = 0;
const char* cardinal_dir = "N";
bool   temp_valid = false;       // no temperature sensor fitted yet
float  water_temp_c = 0.0f;

// Dive timers
uint32_t dive_start_time = 0;
unsigned long dive_duration_sec = 0;
uint32_t descend_since = 0;
uint32_t surface_since = 0;

// Series / persistence
uint16_t dive_count = 0;             // completed dives in current series
uint8_t  series_violations = 0;
uint32_t no_fly_sec = 0;
uint32_t algo_lock_sec = 0;

// Deco outputs
int    real_time_ndl_min = 99;
bool   in_deco = false;
double ceiling_m = 0.0;
double first_stop_m = 0.0;
int    tts_min = 0;
double next_stop_m = 0.0;
int    next_stop_min = 0;
bool   table_exceeded = false;
bool   dive_in_deco = false;
bool   dive_violation = false;
float  breach_sec = 0.0f;
GF     active_gf = {0.30, 0.70};

// Advisory stops
bool  safety_stop_active = false, safety_stop_done = false;
float safety_stop_elapsed = 0.0f;
bool  deep_stop_active = false, deep_stop_done = false;
float deep_stop_elapsed = 0.0f;
float deep_stop_depth_m = 0.0f;
const float SAFETY_STOP_SEC = 180.0f;
const float DEEP_STOP_SEC   = 120.0f;

// Warnings
bool warn_ascent_fast = false, warn_ppo2_high = false, warn_ceiling = false;

// UI
int menu_cursor = 0;
uint32_t last_tick_ms = 0;
uint32_t pc_sync_until = 0;

struct Btn { uint8_t pin; bool state; uint32_t t; };
Btn btnUp   = {PIN_BTN_UP, false, 0};
Btn btnSel  = {PIN_BTN_SELECT, false, 0};
Btn btnMode = {PIN_BTN_MODE, false, 0};

// ---------------------------------------------------------------------
// PERSISTENCE
// ---------------------------------------------------------------------
const uint32_t STATE_MAGIC = 0xD1C0DE02;
const uint32_t MAX_LOGS = 100;

struct SavedState {
  uint32_t magic;
  double   tissue[16];
  uint32_t no_fly_sec;
  uint32_t algo_lock_sec;
  uint16_t dive_count;
  uint8_t  series_violations;
  uint8_t  pad;
  float    zero_mv;
};

void saveSettings() {
  prefs.begin("dive_comp", false);
  prefs.putInt("fo2", fo2_pct);
  prefs.putInt("ppo2_cb", ppo2_cbar);
  prefs.putBool("deep_stop", enable_deep_stop);
  prefs.putInt("algo", (int)selectedAlgo);
  prefs.end();
  settings_dirty = false;
}

void loadSettings() {
  prefs.begin("dive_comp", false);
  fo2_pct = prefs.getInt("fo2", 21);
  ppo2_cbar = prefs.getInt("ppo2_cb", 140);
  enable_deep_stop = prefs.getBool("deep_stop", false);
  int a = prefs.getInt("algo", 0);
  prefs.end();
  if (fo2_pct < 21 || fo2_pct > 40) fo2_pct = 21;
  if (ppo2_cbar < 100 || ppo2_cbar > 160) ppo2_cbar = 140;
  selectedAlgo = (a >= 0 && a <= 2) ? (AlgorithmChoice)a : ALGO_BUHLMANN_ZHL16C;
}

void saveState() {
  SavedState s;
  memset(&s, 0, sizeof(s));
  s.magic = STATE_MAGIC;
  memcpy(s.tissue, P_N2, sizeof(s.tissue));
  s.no_fly_sec = no_fly_sec;
  s.algo_lock_sec = algo_lock_sec;
  s.dive_count = dive_count;
  s.series_violations = series_violations;
  s.zero_mv = zero_mv;
  prefs.begin("dive_state", false);
  prefs.putBytes("s", &s, sizeof(s));
  prefs.end();
}

bool loadState() {
  SavedState s;
  prefs.begin("dive_state", true);
  size_t n = prefs.getBytesLength("s");
  bool ok = (n == sizeof(SavedState)) && (prefs.getBytes("s", &s, sizeof(s)) == sizeof(s));
  prefs.end();
  if (!ok || s.magic != STATE_MAGIC) return false;
  for (int i = 0; i < NUM_COMPARTMENTS; i++) {
    if (!(s.tissue[i] > 0.3 && s.tissue[i] < 8.0)) return false;   // corrupt
  }
  memcpy(P_N2, s.tissue, sizeof(P_N2));
  no_fly_sec = s.no_fly_sec;
  algo_lock_sec = s.algo_lock_sec;
  dive_count = s.dive_count;
  series_violations = s.series_violations;
  if (s.zero_mv > SENSOR_MV_MIN && s.zero_mv < SENSOR_MV_MAX) { zero_mv = s.zero_mv; zero_mv_valid = true; }
  return true;
}

// Ring buffer of the last MAX_LOGS dives. Record: duration_s,max_depth_m,fo2_pct,algo,deco
void logDiveToFlash() {
  prefs.begin("dive_logs", false);
  uint32_t total = prefs.getUInt("count", 0);
  char key[16];
  snprintf(key, sizeof(key), "log_%u", (unsigned)(total % MAX_LOGS));
  char rec[64];
  snprintf(rec, sizeof(rec), "%lu,%.1f,%d,%d,%d", dive_duration_sec, (double)max_depth_m,
           fo2_pct, (int)selectedAlgo, dive_in_deco ? 1 : 0);
  prefs.putString(key, rec);
  prefs.putUInt("count", total + 1);
  prefs.end();
}

void streamLogDataToPC() {
  prefs.begin("dive_logs", true);
  uint32_t total = prefs.getUInt("count", 0);
  uint32_t start = (total > MAX_LOGS) ? total - MAX_LOGS : 0;
  Serial.println("--- DIVE COMPUTER LOG DUMP ---");
  Serial.println("# duration_s,max_depth_m,fo2_pct,algo,deco");
  for (uint32_t i = start; i < total; i++) {
    char key[16];
    snprintf(key, sizeof(key), "log_%u", (unsigned)(i % MAX_LOGS));
    String data = prefs.getString(key, "");
    Serial.printf("Dive #%u: %s\n", (unsigned)(i + 1), data.c_str());
  }
  Serial.println("--- END OF DUMP ---");
  prefs.end();
}

// ---------------------------------------------------------------------
// TISSUE MATH (double precision: slow compartments lose accuracy in float)
// ---------------------------------------------------------------------
double pAt(double depth_m) { return P_SURF + depth_m / M_PER_BAR; }

// Schreiner equation: linear pressure change from p0 to p1 over secs.
// Reduces to Haldane when p0 == p1.
void applyLegP(double *P, double p0, double p1, double secs, double fN2) {
  if (secs <= 0.0) return;
  double pi0 = (p0 - PH2O) * fN2;
  double pi1 = (p1 - PH2O) * fN2;
  double R = (pi1 - pi0) / secs;
  for (int i = 0; i < NUM_COMPARTMENTS; i++) {
    double k = K_N2[i];
    double e = exp(-k * secs);
    P[i] = pi0 + R * (secs - 1.0 / k) - (pi0 - P[i] - R / k) * e;
  }
}

// Ambient pressure a tissue can tolerate at a given gradient factor.
double maxTolerated(const double *P, double gf) {
  double m = 0.0;
  for (int i = 0; i < NUM_COMPARTMENTS; i++) {
    double t = (P[i] - A_N2[i] * gf) / (gf / B_N2[i] - gf + 1.0);
    if (t > m) m = t;
  }
  return m;
}

GF currentGF() {
  if (selectedAlgo == ALGO_RGBM_SIM) {
    // Bubble-style conservatism: GF_hi reduced per repetitive dive and per
    // violation in the current series. Approximation, NOT real RGBM.
    int reps = dive_count > 3 ? 3 : dive_count;
    int viol = series_violations > 3 ? 3 : series_violations;
    double hi = 0.75 - 0.05 * reps - 0.05 * viol;
    if (hi < 0.45) hi = 0.45;
    return {0.30, hi};
  }
  return {0.30, 0.70};   // Buhlmann GF 30/70 (also fallback for table mode)
}

// NDL in minutes (capped 9999) using the GF_hi surface M-value. 0 = already in deco.
double calcNDL(const double *P, double p_amb, double fN2, double gf_hi) {
  double pi = (p_amb - PH2O) * fN2;
  double best = 9999.0 * 60.0;
  for (int i = 0; i < NUM_COMPARTMENTS; i++) {
    double m0 = A_N2[i] + P_SURF / B_N2[i];
    double M = P_SURF + gf_hi * (m0 - P_SURF);
    if (P[i] >= M) return 0.0;
    if (pi <= M) continue;                       // never reaches the limit
    double t = log((pi - P[i]) / (pi - M)) / K_N2[i];
    if (t < best) best = t;
  }
  return best / 60.0;
}

// Ceiling (m, multiple of 3, 0 = none). "In deco" means GF_hi would be violated at the surface.
// first_out = first stop depth from GF_lo. GF is interpolated lo->hi between first stop and surface.
double calcCeilingM(const double *P, GF gf, double &first_out) {
  first_out = 0.0;
  if (maxTolerated(P, gf.hi) <= P_SURF + 1e-4) return 0.0;
  double p_lo = maxTolerated(P, gf.lo);
  double first_m = ceil(((p_lo - P_SURF) * M_PER_BAR) / 3.0 - 1e-9) * 3.0;
  if (first_m < 3.0) first_m = 3.0;
  first_out = first_m;
  double p_first = pAt(first_m);
  for (double d = 3.0; d <= first_m + 0.01; d += 3.0) {
    double p_d = pAt(d);
    double g = gf.hi + (gf.lo - gf.hi) * (p_d - P_SURF) / (p_first - P_SURF);
    if (maxTolerated(P, g) <= p_d + 1e-4) return d;
  }
  return first_m;
}

// Simulated ascent to the surface on a copy of the tissues: 9 m/min between
// stops, 1-minute stop increments. Fills tts_min / next_stop_m / next_stop_min.
void simulateDeco(GF gf, double fN2) {
  double P[NUM_COMPARTMENTS];
  memcpy(P, P_N2, sizeof(P));
  double depth = current_depth_m;
  double t = 0.0, dummy, first_c = -1.0;
  int first_stop_min = 0;
  bool finished = false;

  for (int iter = 0; iter < 600; iter++) {
    double c = calcCeilingM(P, gf, dummy);
    if (first_c < 0.0) first_c = c;
    if (c <= 0.0) {
      t += depth / PLAN_ASCENT_M_MIN * 60.0;
      finished = true;
      break;
    }
    if (depth > c + 0.01) {
      double secs = (depth - c) / PLAN_ASCENT_M_MIN * 60.0;
      applyLegP(P, pAt(depth), pAt(c), secs, fN2);
      t += secs;
      depth = c;
    } else {
      depth = c;   // if above the ceiling, assume the diver returns to it
      applyLegP(P, pAt(c), pAt(c), 60.0, fN2);
      t += 60.0;
      if (fabs(c - first_c) < 0.01) first_stop_min++;
    }
  }
  tts_min = finished ? (int)ceil(t / 60.0) : 999;
  next_stop_m = first_c > 0.0 ? first_c : 0.0;
  next_stop_min = first_stop_min < 1 ? 1 : first_stop_min;
}

int tableIndex(float depth) {
  for (int j = 0; j < TBL_N; j++) if (depth <= (float)TBL_DEPTH_M[j]) return j;
  return -1;   // beyond table
}

// Table mode: Table 1 limits, scaled down by residual nitrogen. The scale is
// (NDL now) / (NDL from air-saturated) at each table depth, taken at dive start.
void buildTableLimits() {
  double fN2 = 1.0 - fo2_pct / 100.0;
  for (int j = 0; j < TBL_N; j++) {
    double p_amb = pAt(TBL_DEPTH_M[j]);
    double fresh = calcNDL(P_AIR, p_amb, fN2, 0.70);
    double now_ = calcNDL(P_N2, p_amb, fN2, 0.70);
    double ratio = (fresh < 0.5) ? 1.0 : now_ / fresh;
    if (ratio > 1.0) ratio = 1.0;
    if (ratio < 0.0) ratio = 0.0;
    tbl_adj_min[j] = (uint16_t)floor(TBL_NDL_MIN[j] * ratio);
  }
}

// ---------------------------------------------------------------------
// SENSORS
// ---------------------------------------------------------------------
void calibrateZeroPoint() {
  uint32_t sum = 0;
  for (int i = 0; i < 30; i++) { sum += analogReadMilliVolts(PIN_ADC_PRESSURE); delay(5); }
  float mv = sum / 30.0f;
  if (mv < SENSOR_MV_MIN || mv > SENSOR_MV_MAX) { sensor_fault = true; return; }
  // Booting under pressure must not become the new zero: reject big jumps vs stored zero.
  float tol_mv = 0.10f / SENSOR_RANGE_BAR * SENSOR_SPAN_V * 1000.0f;   // 0.1 bar
  if (zero_mv_valid && fabsf(mv - zero_mv) > tol_mv) return;
  zero_mv = mv;
  zero_mv_valid = true;
}

void updateCompass() {
  if (!mag_ok || !accel_ok) return;
  sensors_event_t me, ae;
  mag.getEvent(&me);
  accel.getEvent(&ae);

  // Tilt compensation. Axis signs depend on how the board is mounted: verify on the bench.
  float ax = ae.acceleration.x, ay = ae.acceleration.y, az = ae.acceleration.z;
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float mx = me.magnetic.x, my = me.magnetic.y, mz = me.magnetic.z;
  float xh = mx * cosf(pitch) + mz * sinf(pitch);
  float yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);
  float hr = atan2f(yh, xh);

  static float sx = 0.0f, sc = 1.0f;          // smooth on the unit circle (no wrap glitch)
  sx = 0.8f * sx + 0.2f * sinf(hr);
  sc = 0.8f * sc + 0.2f * cosf(hr);
  float h = atan2f(sx, sc) * 180.0f / (float)M_PI + HEADING_OFFSET_DEG;
  while (h < 0.0f) h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;
  heading_deg = (int)h;
  static const char* CARD[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  cardinal_dir = CARD[((int)((h + 22.5f) / 45.0f)) % 8];
}

void updateSensors() {
  uint32_t sum = 0;
  for (int i = 0; i < 10; i++) sum += analogReadMilliVolts(PIN_ADC_PRESSURE);
  float mv = sum / 10.0f;

  if (mv < SENSOR_MV_MIN || mv > SENSOR_MV_MAX) {
    if (bad_count < 255) bad_count++;
    if (bad_count >= 10) sensor_fault = true;
  } else {
    bad_count = 0;
    sensor_fault = false;

    float gauge = (mv - zero_mv) / 1000.0f / SENSOR_SPAN_V * SENSOR_RANGE_BAR;

    // Slow re-zero at the surface (weather drift). Bounded to +/-0.03 bar so it can't absorb real depth.
    if (currentMode != MODE_DIVE && fabsf(gauge) < 0.03f) zero_mv += 0.002f * (mv - zero_mv);

    if (gauge < 0.0f) gauge = 0.0f;
    gauge_f += 0.3f * (gauge - gauge_f);          // light low-pass
    current_depth_m = gauge_f * (float)M_PER_BAR;
    p_abs_bar = P_SURF + gauge_f;                  // depth and pressure now consistent
  }
  updateCompass();
}

void updateAscentRate(uint32_t now) {
  static float hist[6];
  static uint8_t idx = 0, cnt = 0;
  static uint32_t last = 0;
  if (now - last >= 1000) {
    last = now;
    idx = (idx + 1) % 6;
    hist[idx] = current_depth_m;
    if (cnt < 6) cnt++;
    if (cnt >= 6) ascent_rate_m_min = (hist[(idx + 1) % 6] - current_depth_m) * 12.0f;  // 5 s window
    else ascent_rate_m_min = 0.0f;
  }
}

// ---------------------------------------------------------------------
// DIVE START / END
// ---------------------------------------------------------------------
void startDive(uint32_t started_at) {
  if (settings_dirty) saveSettings();
  currentMode = MODE_DIVE;
  currentDiveView = SCREEN_MAIN_HUD;
  dive_start_time = started_at;
  dive_duration_sec = 0;
  max_depth_m = current_depth_m;
  surface_since = 0;

  // Per-dive state reset
  safety_stop_active = safety_stop_done = false;  safety_stop_elapsed = 0;
  deep_stop_active = deep_stop_done = false;      deep_stop_elapsed = 0;
  dive_in_deco = false; dive_violation = false; breach_sec = 0;
  in_deco = false; ceiling_m = 0; table_exceeded = false;

  algo_lock_sec = ALGO_LOCKOUT_SEC;              // locked from the moment the dive begins
  if (selectedAlgo == ALGO_PADI_TABLES) buildTableLimits();
  saveState();
}

void endDive() {
  dive_duration_sec = (surface_since - dive_start_time) / 1000;   // exclude the surface wait
  dive_count++;
  if (dive_violation) { if (series_violations < 255) series_violations++; }

  uint32_t target = (dive_in_deco || dive_violation) ? NOFLY_DECO_SEC
                   : (dive_count > 1 ? NOFLY_MULTI_SEC : NOFLY_SINGLE_SEC);
  if (target > no_fly_sec) no_fly_sec = target;
  algo_lock_sec = ALGO_LOCKOUT_SEC;

  logDiveToFlash();
  saveState();
  currentMode = MODE_SURFACE;
  surface_since = 0;
  warn_ascent_fast = warn_ppo2_high = warn_ceiling = false;
}

// ---------------------------------------------------------------------
// DECO / SAFETY ENGINE (runs each tick)
// ---------------------------------------------------------------------
void updateDecoState(double dt, uint32_t now) {
  double fo2 = fo2_pct / 100.0;

  if (currentMode != MODE_DIVE) {
    real_time_ndl_min = 99; in_deco = false; ceiling_m = 0; tts_min = 0;
    table_exceeded = false; warn_ceiling = false; warn_ascent_fast = false; warn_ppo2_high = false;
    return;
  }

  active_gf = currentGF();
  double fN2 = 1.0 - fo2;
  table_exceeded = false;

  if (selectedAlgo == ALGO_PADI_TABLES) {
    int idx = tableIndex(max_depth_m);
    int elapsed = (int)((dive_duration_sec + 59) / 60);
    if (idx < 0 || elapsed >= (int)tbl_adj_min[idx]) {
      table_exceeded = true;
      dive_violation = true;            // fall through to the Buhlmann engine for deco
    } else {
      int rem = (int)tbl_adj_min[idx] - elapsed;
      real_time_ndl_min = rem > 99 ? 99 : rem;
      in_deco = false; ceiling_m = 0; tts_min = (int)ceil(current_depth_m / PLAN_ASCENT_M_MIN);
    }
  }

  if (selectedAlgo != ALGO_PADI_TABLES || table_exceeded) {
    ceiling_m = calcCeilingM(P_N2, active_gf, first_stop_m);
    in_deco = ceiling_m > 0.0;
    if (in_deco) {
      real_time_ndl_min = 0;
      dive_in_deco = true;
      static uint32_t last_sim = 0;
      if (now - last_sim >= 2000 || last_sim == 0) { simulateDeco(active_gf, fN2); last_sim = now; }
    } else {
      double n = calcNDL(P_N2, p_abs_bar, fN2, active_gf.hi);
      real_time_ndl_min = n > 99.0 ? 99 : (int)n;
      tts_min = (int)ceil(current_depth_m / PLAN_ASCENT_M_MIN);
    }
  }

  // Ceiling breach: 10 s cumulative above the ceiling counts as a violation
  warn_ceiling = in_deco && (current_depth_m < ceiling_m - 0.5);
  if (warn_ceiling) { breach_sec += (float)dt; if (breach_sec > 10.0f) dive_violation = true; }
  else breach_sec = 0.0f;

  warn_ascent_fast = ascent_rate_m_min > ASCENT_WARN_M_MIN;
  if (ascent_rate_m_min > ASCENT_VIOL_M_MIN) dive_violation = true;
  warn_ppo2_high = current_ppo2 >= (ppo2_cbar / 100.0);
}

void processSafetyAndDeepStops(float dt) {
  // Advisory stops only apply when not in mandatory deco
  if (in_deco) { safety_stop_active = false; deep_stop_active = false; return; }

  // Deep stop: half of max depth, 2 min, only on the way up
  if (enable_deep_stop && max_depth_m >= 18.0f && !deep_stop_done) {
    deep_stop_depth_m = max_depth_m / 2.0f;
    bool ascending = current_depth_m < max_depth_m - 3.0f;
    bool in_window = fabsf(current_depth_m - deep_stop_depth_m) <= 1.5f;
    deep_stop_active = ascending && in_window;
    if (deep_stop_active) {
      deep_stop_elapsed += dt;
      if (deep_stop_elapsed >= DEEP_STOP_SEC) { deep_stop_done = true; deep_stop_active = false; }
    }
  } else if (deep_stop_done) deep_stop_active = false;

  // Safety stop: 3 min at 3-6 m after a dive deeper than 10 m
  if (max_depth_m >= 10.0f && !safety_stop_done) {
    if (current_depth_m >= 3.0f && current_depth_m <= 6.0f) safety_stop_active = true;
    else if (current_depth_m < 2.5f || current_depth_m > 7.0f) { safety_stop_active = false; safety_stop_elapsed = 0; }
    if (safety_stop_active) {
      safety_stop_elapsed += dt;
      if (safety_stop_elapsed >= SAFETY_STOP_SEC) { safety_stop_done = true; safety_stop_active = false; }
    }
  } else if (safety_stop_done) safety_stop_active = false;
}

// ---------------------------------------------------------------------
// BUTTONS (debounced edge detect; menu edits are saved when leaving the menu)
// ---------------------------------------------------------------------
bool pressed(Btn &b) {
  bool s = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();
  if (s != b.state && (now - b.t) > 25) {
    b.state = s;
    b.t = now;
    return s;
  }
  return false;
}

bool algoLocked() { return algo_lock_sec > 0; }

void handleButtons() {
  if (pressed(btnMode)) {
    if (currentMode == MODE_SURFACE) currentMode = MODE_MENU;
    else if (currentMode == MODE_MENU) {
      if (settings_dirty) saveSettings();
      currentMode = MODE_SURFACE;
    } else if (currentMode == MODE_DIVE) {
      if (currentDiveView == SCREEN_MAIN_HUD) currentDiveView = SCREEN_COMPASS_TEMP;
      else if (currentDiveView == SCREEN_COMPASS_TEMP) currentDiveView = SCREEN_GAS_DECO;
      else currentDiveView = SCREEN_MAIN_HUD;
    }
  }

  if (pressed(btnUp)) {
    if (currentMode == MODE_MENU) menu_cursor = (menu_cursor + 1) % 5;
  }

  if (pressed(btnSel)) {
    if (currentMode == MODE_MENU) {
      switch (menu_cursor) {
        case 0:
          fo2_pct++;
          if (fo2_pct > 40) fo2_pct = 21;
          settings_dirty = true;
          break;
        case 1:
          ppo2_cbar += 5;
          if (ppo2_cbar > 160) ppo2_cbar = 100;
          settings_dirty = true;
          break;
        case 2:
          enable_deep_stop = !enable_deep_stop;
          settings_dirty = true;
          break;
        case 3:
          if (!algoLocked()) {                 // surface only, and only after the lockout has elapsed
            selectedAlgo = (AlgorithmChoice)((selectedAlgo + 1) % 3);
            settings_dirty = true;
          }
          break;
        case 4:
          currentMode = MODE_PC_SYNC;
          pc_sync_until = millis() + 3000;
          streamLogDataToPC();
          break;
      }
    }
  }
}

// ---------------------------------------------------------------------
// DISPLAY
// ---------------------------------------------------------------------
const char* pickWarning(uint32_t now) {
  const char* w[6];
  int n = 0;
  if (sensor_fault)     w[n++] = "SENSOR FAULT";
  if (warn_ceiling)     w[n++] = "CEILING BREACH!";
  if (warn_ppo2_high)   w[n++] = "HIGH PPO2!";
  if (warn_ascent_fast) w[n++] = "ASCENT TOO FAST!";
  if (table_exceeded)   w[n++] = "TABLE LIMIT EXCEEDED";
  if (n == 0) return nullptr;
  return w[(now / 1500) % n];
}

void renderUI(uint32_t now) {
  u8g2.clearBuffer();
  char buf[32];

  if (currentMode == MODE_SURFACE) {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 10, "SURFACE / PLANNER");
    u8g2.drawHLine(0, 12, 128);

    snprintf(buf, sizeof(buf), "NO FLY: %luh%02lum", (unsigned long)(no_fly_sec / 3600), (unsigned long)((no_fly_sec % 3600) / 60));
    u8g2.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "O2:%d%% PPO2:%.2f", fo2_pct, ppo2_cbar / 100.0);
    u8g2.drawStr(0, 34, buf);
    snprintf(buf, sizeof(buf), "MOD:%.1fm Dives:%u", mod_m, (unsigned)dive_count);
    u8g2.drawStr(0, 44, buf);
    if (algoLocked()) snprintf(buf, sizeof(buf), "ALG:%s LK%luh", ALGO_NAMES[selectedAlgo], (unsigned long)((algo_lock_sec + 3599) / 3600));
    else snprintf(buf, sizeof(buf), "ALG:%s", ALGO_NAMES[selectedAlgo]);
    u8g2.drawStr(0, 54, buf);
    if (sensor_fault) u8g2.drawStr(0, 63, "SENSOR FAULT");
  }
  else if (currentMode == MODE_MENU) {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 10, "SETTINGS MENU");
    u8g2.drawHLine(0, 12, 128);

    snprintf(buf, sizeof(buf), "%c1 FO2 Nitrox: %d%%", menu_cursor == 0 ? '>' : ' ', fo2_pct);
    u8g2.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "%c2 PPO2 Lim: %.2f", menu_cursor == 1 ? '>' : ' ', ppo2_cbar / 100.0);
    u8g2.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "%c3 Deep Stop: %s", menu_cursor == 2 ? '>' : ' ', enable_deep_stop ? "ON" : "OFF");
    u8g2.drawStr(0, 42, buf);
    if (algoLocked()) snprintf(buf, sizeof(buf), "%c4 %s LK %luh", menu_cursor == 3 ? '>' : ' ', ALGO_NAMES[selectedAlgo], (unsigned long)((algo_lock_sec + 3599) / 3600));
    else snprintf(buf, sizeof(buf), "%c4 Algo: %s", menu_cursor == 3 ? '>' : ' ', ALGO_NAMES[selectedAlgo]);
    u8g2.drawStr(0, 52, buf);
    snprintf(buf, sizeof(buf), "%c5 PC Debugger Sync", menu_cursor == 4 ? '>' : ' ');
    u8g2.drawStr(0, 62, buf);
  }
  else if (currentMode == MODE_DIVE) {
    if (currentDiveView == SCREEN_MAIN_HUD) {
      const char* warn = pickWarning(now);
      if (warn) {
        u8g2.setFont(u8g2_font_6x12_tr);
        u8g2.drawStr(0, 11, warn);
      } else {
        u8g2.setFont(u8g2_font_6x10_tr);
        snprintf(buf, sizeof(buf), "%03d* %s | O2:%d%%", heading_deg, cardinal_dir, fo2_pct);
        u8g2.drawStr(0, 10, buf);
      }
      u8g2.drawHLine(0, 12, 128);

      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(0, 20, "DEPTH m");
      u8g2.drawStr(80, 20, in_deco ? "CEIL m" : "NDL min");

      u8g2.setFont(u8g2_font_logisoso22_tn);
      snprintf(buf, sizeof(buf), "%.1f", current_depth_m);
      u8g2.drawStr(0, 44, buf);
      if (in_deco) {
        snprintf(buf, sizeof(buf), "%.0f", ceiling_m);
        u8g2.drawStr(80, 44, buf);
      } else if (real_time_ndl_min >= 99) {
        u8g2.drawStr(80, 44, "99");
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(112, 44, "+");
      } else {
        snprintf(buf, sizeof(buf), "%d", real_time_ndl_min);
        u8g2.drawStr(80, 44, buf);
      }

      u8g2.drawHLine(0, 47, 128);
      u8g2.setFont(u8g2_font_6x10_tr);
      if (in_deco) {
        snprintf(buf, sizeof(buf), "STOP %.0fm %dmin TTS%d", next_stop_m, next_stop_min, tts_min);
        u8g2.drawStr(0, 60, buf);
      } else if (safety_stop_active) {
        int rem = (int)(SAFETY_STOP_SEC - safety_stop_elapsed); if (rem < 0) rem = 0;
        snprintf(buf, sizeof(buf), "STOP 5m | %02d:%02d", rem / 60, rem % 60);
        u8g2.drawStr(0, 60, buf);
      } else if (deep_stop_active) {
        int rem = (int)(DEEP_STOP_SEC - deep_stop_elapsed); if (rem < 0) rem = 0;
        snprintf(buf, sizeof(buf), "DEEP %.0fm | %02d:%02d", deep_stop_depth_m, rem / 60, rem % 60);
        u8g2.drawStr(0, 60, buf);
      } else {
        snprintf(buf, sizeof(buf), "M:%.1fm", max_depth_m);
        u8g2.drawStr(0, 60, buf);
        snprintf(buf, sizeof(buf), "%02d:%02d", (int)(dive_duration_sec / 60), (int)(dive_duration_sec % 60));
        u8g2.drawStr(50, 60, buf);
        snprintf(buf, sizeof(buf), "%.2f", current_ppo2);
        u8g2.drawStr(96, 60, buf);
      }
    }
    else if (currentDiveView == SCREEN_COMPASS_TEMP) {
      u8g2.setFont(u8g2_font_7x14_tr);
      u8g2.drawStr(20, 14, "- NAV & TEMP -");
      u8g2.drawHLine(0, 16, 128);
      if (!mag_ok || !accel_ok) {
        u8g2.drawStr(0, 36, "COMPASS ERROR");
      } else {
        snprintf(buf, sizeof(buf), "HEADING: %d*", heading_deg);
        u8g2.drawStr(0, 32, buf);
        snprintf(buf, sizeof(buf), "DIR: %s", cardinal_dir);
        u8g2.drawStr(0, 46, buf);
      }
      if (temp_valid) snprintf(buf, sizeof(buf), "TEMP: %.1f C", water_temp_c);
      else snprintf(buf, sizeof(buf), "TEMP: --.- C");
      u8g2.drawStr(0, 60, buf);
    }
    else {  // SCREEN_GAS_DECO
      u8g2.setFont(u8g2_font_6x10_tr);
      snprintf(buf, sizeof(buf), "GAS O2:%d%% PPO2:%.2f", fo2_pct, current_ppo2);
      u8g2.drawStr(0, 10, buf);
      snprintf(buf, sizeof(buf), "MOD:%.1fm LIM:%.2f", mod_m, ppo2_cbar / 100.0);
      u8g2.drawStr(0, 22, buf);
      if (selectedAlgo == ALGO_PADI_TABLES && !table_exceeded) snprintf(buf, sizeof(buf), "ALG:%s", ALGO_NAMES[selectedAlgo]);
      else snprintf(buf, sizeof(buf), "ALG:%s GF%d/%d", ALGO_NAMES[selectedAlgo], (int)(active_gf.lo * 100 + 0.5), (int)(active_gf.hi * 100 + 0.5));
      u8g2.drawStr(0, 34, buf);
      if (in_deco) snprintf(buf, sizeof(buf), "CEIL:%.0fm TTS:%dmin", ceiling_m, tts_min);
      else snprintf(buf, sizeof(buf), "NO DECO  NDL:%d", real_time_ndl_min);
      u8g2.drawStr(0, 46, buf);
      snprintf(buf, sizeof(buf), "ASC:%.1fm/min", ascent_rate_m_min);
      u8g2.drawStr(0, 58, buf);
    }
  }
  else if (currentMode == MODE_PC_SYNC) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(10, 25, "PC DEBUGGER");
    u8g2.drawStr(10, 45, "SYNCING LOGS...");
  }

  u8g2.sendBuffer();
}

// ---------------------------------------------------------------------
// MAIN TICK (100 ms)
// ---------------------------------------------------------------------
void tick(uint32_t now, double dt) {
  updateSensors();
  double fo2 = fo2_pct / 100.0;

  // Gas
  current_ppo2 = p_abs_bar * fo2;
  double lim = ppo2_cbar / 100.0;
  mod_m = floor(((lim / fo2 - P_SURF) * M_PER_BAR) * 10.0) / 10.0;   // floored to 0.1 m
  if (mod_m < 0.0) mod_m = 0.0;

  if (!sensor_fault) {
    // Tissues always track (all algorithms share one tissue model).
    // Nitrox while diving, air at the surface.
    double fN2 = (currentMode == MODE_DIVE) ? (1.0 - fo2) : FN2_AIR;
    applyLegP(P_N2, prev_p_abs, p_abs_bar, dt, fN2);
    prev_p_abs = p_abs_bar;

    updateAscentRate(now);
    if (currentMode == MODE_DIVE) {
      if (current_depth_m > max_depth_m) max_depth_m = current_depth_m;
      dive_duration_sec = (now - dive_start_time) / 1000;
    }
    updateDecoState(dt, now);
    if (currentMode == MODE_DIVE) processSafetyAndDeepStops((float)dt);
  }

  // Dive start/end. Start works from the menu / sync screens too, so no dive goes unlogged.
  if (currentMode != MODE_DIVE) {
    if (!sensor_fault && current_depth_m >= 0.8f) {
      if (descend_since == 0) descend_since = now;
      if (now - descend_since >= DIVE_START_MS) { startDive(descend_since); descend_since = 0; }
    } else descend_since = 0;
  } else {
    if (current_depth_m < 0.3f) {
      if (surface_since == 0) surface_since = now;
      if (now - surface_since >= DIVE_END_MS) endDive();
    } else surface_since = 0;
  }

  if (currentMode == MODE_PC_SYNC && (int32_t)(now - pc_sync_until) >= 0) currentMode = MODE_SURFACE;

  // 1 Hz housekeeping: surface countdowns + autosave
  static uint32_t last_s = 0;
  static uint32_t save_acc = 0;
  if (now - last_s >= 1000) {
    last_s = now;
    save_acc++;
    if (currentMode != MODE_DIVE) {
      if (no_fly_sec > 0) no_fly_sec--;
      if (algo_lock_sec > 0) algo_lock_sec--;
      if (no_fly_sec == 0 && algo_lock_sec == 0 && (dive_count > 0 || series_violations > 0)) {
        dive_count = 0; series_violations = 0;       // series over
        saveState();
      }
    }
    uint32_t interval = (currentMode == MODE_DIVE) ? 120 : 300;
    if (save_acc >= interval) {
      save_acc = 0;
      if (currentMode == MODE_DIVE || no_fly_sec > 0) saveState();   // survive power loss
    }
  }

  renderUI(now);
}

// ---------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_PRESSURE, ADC_11db);

  u8g2.begin();
  mag_ok = mag.begin();
  accel_ok = accel.begin();
  if (!mag_ok || !accel_ok) Serial.println("WARN: LSM303 not found, compass disabled");

  for (int i = 0; i < NUM_COMPARTMENTS; i++) {
    K_N2[i] = log(2.0) / (HALF_TIME_N2[i] * 60.0);
    P_AIR[i] = (P_SURF - PH2O) * FN2_AIR;
  }

  loadSettings();
  if (!loadState()) {
    for (int i = 0; i < NUM_COMPARTMENTS; i++) P_N2[i] = P_AIR[i];   // saturated on AIR at the surface
  }

  calibrateZeroPoint();
  prev_p_abs = P_SURF;
  last_tick_ms = millis();
}

void loop() {
  handleButtons();
  uint32_t now = millis();
  if (now - last_tick_ms >= TICK_MS) {
    double dt = (now - last_tick_ms) / 1000.0;
    if (dt > 10.0) dt = 10.0;
    last_tick_ms = now;
    tick(now, dt);
  }
  delay(2);
}
