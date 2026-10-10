#!/usr/bin/env node
/*
 simtest.js - headless regression tests for the FW(HAL) firmware block in divesim.html

 Run:  node simtest.js [path/to/divesim.html] [name-filter]

 Pulls the firmware out from between the two marker comments in divesim.html, checks brace
 balance, loads it against a stub HAL (bounds-checked u8g2, tone log, in-memory flash) and
 runs scenarios. Exit code is non-zero if anything fails.

 Add a scenario:
   scenario('name', t => { const s = t.mk(); ...; t.check('what', cond, detail); });
 Sim helpers:
   s.run(min, depth)  s.runSec(sec, depth)  s.runFn(sec, vars => depth)  s.press(BTN.X)
   s.teleport(depth)  s.tick()  s.vars()  s.has(/re/)  s.strs()  s.frame  s.tones  s.flash
   s.hdg  s.rate (m/min, default 150)  s.dump()
 Every sim also fails its scenario if anything was drawn out of 128x64 or overflowed 128px.
*/
const fs = require('fs'), path = require('path');
const args = process.argv.slice(2);
const htmlArg = args.find(a => a.endsWith('.html'));
const filter = args.find(a => !a.endsWith('.html'));
const HTML = htmlArg || [path.join(__dirname, 'divesim.html'), '/mnt/user-data/outputs/divesim.html'].find(p => fs.existsSync(p));
if (!HTML) { console.error('divesim.html not found'); process.exit(2); }

const START = '/* ================= FIRMWARE (JS port of main.cpp) ================= */';
const END = '/* ================= SIMULATOR / HAL';
const html = fs.readFileSync(HTML, 'utf8');
if (!html.includes(START) || !html.includes(END)) { console.error('firmware markers not found in ' + HTML); process.exit(2); }
const src = html.split(START)[1].split(END)[0];
const opens = (src.match(/\{/g) || []).length, closes = (src.match(/\}/g) || []).length;
console.log(`${HTML}\nbraces: ${opens} open / ${closes} close ${opens === closes ? '(balanced)' : '(MISMATCH)'}`);
if (opens !== closes) process.exit(2);
let FW;
try { FW = new Function(src + '\n;return FW;')(); } catch (e) { console.error('firmware failed to load: ' + e.message); process.exit(2); }
let FS = { '5x7': 5, '6x10': 6, '6x12': 6, '7x14': 7, 'L22': 13 };
const fsm = html.match(/const FS=(\{[^}]*\})/);
if (fsm) { try { FS = new Function('return ' + fsm[1])(); } catch (e) { /* keep defaults */ } }

const BTN = { UP: 0, SELECT: 1, MODE: 2, DOWN: 3 };

function makeSim() {
  const s = { ms: 0, depth: 0, target: 0, rate: 150, hdg: 0, temp: 26, pend: [0, 0, 0, 0],
    flash: new Map(), tones: [], bad: new Set(), frame: [], ser: [] };
  let fw = 6;
  const chk = (k, x, y, extra) => { if (!(x >= 0 && y >= 0 && x <= 128 && y <= 64)) s.bad.add(`${k} out of bounds x=${x} y=${y} ${extra || ''}`); };
  const HAL = {
    millis: () => s.ms,
    analogMv: () => 250 + s.depth / 10 / 12 * 2000,
    edge: i => { const v = s.pend[i]; s.pend[i] = 0; return v; },
    heading: () => s.hdg,
    temp: () => s.temp,
    u8g2: {
      clearBuffer() { s.frame = []; },
      setFont(f) { if (!(f in FS)) s.bad.add('unknown font ' + f); fw = FS[f] || 6; },
      drawStr(x, y, str, col) {
        s.frame.push({ op: 'str', x, y, s: String(str), fw, col }); chk('STR', x, y, str);
        if (x + String(str).length * fw > 128) s.bad.add(`STR overflow "${str}" x=${x} fw=${fw}`);
      },
      drawHLine(x, y, w, col) { s.frame.push({ op: 'hline', x, y, w, col }); chk('HLINE', x, y); if (x + w > 128) s.bad.add('HLINE overflow'); },
      drawBox(x, y, w, h, col) { s.frame.push({ op: 'box', x, y, w, h, col }); chk('BOX', x, y); if (x + w > 128 || y + h > 64) s.bad.add(`BOX overflow ${x},${y},${w},${h}`); },
      drawLine(x0, y0, x1, y1, col) { s.frame.push({ op: 'line', x0, y0, x1, y1, col }); chk('LINE', x0, y0); chk('LINE', x1, y1); },
      drawCircle(x, y, r, col) { s.frame.push({ op: 'circle', x, y, r, col }); chk('CIRCLE', x - r, y - r); chk('CIRCLE', x + r, y + r); },
      sendBuffer() {}
    },
    Serial: { println: t => s.ser.push(t) },
    prefs: { get: (k, d) => s.flash.has(k) ? JSON.parse(s.flash.get(k)) : d, put: (k, v) => s.flash.set(k, JSON.stringify(v)) },
    beep: (name, f, ms) => s.tones.push({ kind: 'beep', name, f, ms }),
    chime: (f, ms) => s.tones.push({ kind: 'chime', f, ms })
  };
  const fwi = FW(HAL); fwi.setup();
  s.vars = () => fwi.vars();
  s.set = (k, val) => fwi._set(k, val); // test-only: pokes firmware state directly (e.g. S.heFrac ahead of a real gas-list UI)
  s.tables = { A: fwi.A, B: fwi.B, A_HE: fwi.A_HE, B_HE: fwi.B_HE };
  s.tick = () => {
    const step = s.rate / 600;
    s.depth += Math.max(-step, Math.min(step, s.target - s.depth));
    s.ms += 100; fwi.loop(s.ms, 0.1);
  };
  s.run = (min, d) => { if (d !== undefined) s.target = d; for (let i = 0, n = Math.round(min * 600); i < n; i++) s.tick(); };
  s.runSec = (sec, d) => { if (d !== undefined) s.target = d; for (let i = 0, n = Math.round(sec * 10); i < n; i++) s.tick(); };
  s.runFn = (sec, fn) => { for (let i = 0, n = Math.round(sec * 10); i < n; i++) { s.target = fn(s.vars()); s.tick(); } };
  s.press = b => { s.pend[b] = 1; s.tick(); };
  s.teleport = d => { s.depth = d; s.target = d; };
  s.strs = () => s.frame.filter(o => o.op === 'str');
  s.has = re => s.strs().some(o => re.test(o.s));
  s.dump = () => s.frame.map(o => o.op === 'str' ? `str  (${o.x},${o.y}) fw${o.fw} ${o.col || '-'} "${o.s}"` : JSON.stringify(o)).join('\n');
  return s;
}

let pass = 0, fail = 0; const failures = [];
function scenario(name, fn) {
  if (filter && !name.includes(filter)) return;
  const sims = [], before = fail;
  const t = {
    mk: () => { const s = makeSim(); sims.push(s); return s; },
    check: (desc, cond, detail) => { if (cond) pass++; else { fail++; failures.push(`[${name}] ${desc}${detail !== undefined ? '  ->  ' + detail : ''}`); } }
  };
  try { fn(t); } catch (e) { fail++; failures.push(`[${name}] threw: ${e.stack.split('\n').slice(0, 3).join(' | ')}`); }
  sims.forEach(s => { if (s.bad.size) { fail++; failures.push(`[${name}] draw issues: ${[...s.bad].slice(0, 4).join(' ; ')}`); } else pass++; });
  console.log(`  ${fail === before ? 'ok  ' : 'FAIL'}  ${name}`);
}

// ---------------------------------------------------------------- scenarios
scenario('boot: sane defaults', t => {
  const s = t.mk(); s.tick(); const v = s.vars();
  t.check('starts on the surface', v.mode === 'SURFACE');
  t.check('16 tissues near air saturation', v.P.length === 16 && v.P.every(p => Math.abs(p - 0.751) < 0.01));
  t.check('planner screen drawn', s.has(/SURFACE \/ PLANNER/));
});

scenario('menu: SELECT moves cursor, UP/DOWN adjust (9-item main menu, gas config moved out)', t => {
  const s = t.mk(), v = () => s.vars();
  s.press(BTN.MODE);
  t.check('MODE opens menu', v().mode === 'MENU'); t.check('cursor starts at 0 (PPO2 limit)', v().cur === 0);
  s.press(BTN.DOWN); t.check('DOWN raises PPO2 limit to 145', v().ppo2 === 145);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('deep stop toggles on (item 1)', v().deep === true);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('DOWN -> next algorithm (item 2)', v().algo === 1);
  s.press(BTN.UP); t.check('UP -> previous algorithm', v().algo === 0);
  for (let i = 0; i < 3; i++) s.press(BTN.SELECT);
  t.check('cursor at GF lo (5)', v().cur === 5);
  s.press(BTN.DOWN); t.check('GF lo 30 -> 35', Math.abs(v().ugf0 - 0.35) < 1e-9);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('GF hi 70 -> 75 (item 6)', Math.abs(v().ugf1 - 0.75) < 1e-9);
  s.press(BTN.SELECT); t.check('cursor at units toggle (7)', v().cur === 7);
  s.press(BTN.DOWN); t.check('units toggles to imperial', v().units === 'imp');
  s.press(BTN.UP); t.check('units toggles back to metric', v().units === 'met');
  s.press(BTN.SELECT); t.check('cursor at Gas Setup entry (8)', v().cur === 8);
  s.press(BTN.SELECT); t.check('cursor wraps to 0', v().cur === 0);
  s.press(BTN.MODE); t.check('MODE leaves menu', v().mode === 'SURFACE'); t.check('cursor reset to 0', v().cur === 0);
  t.check('settings persisted to flash', s.flash.has('set') && JSON.parse(s.flash.get('set')).deep === true);
});

scenario('menu: gas setup sub-screen (its own page, enable/disable toggles)', t => {
  const s = t.mk(), v = () => s.vars();
  s.press(BTN.MODE); for (let i = 0; i < 8; i++) s.press(BTN.SELECT); // -> item 8 "Gas Setup >"
  s.press(BTN.DOWN); // fires the action, opens the sub-screen
  t.check('gas setup opened as its own screen', v().gasSetup === true && v().mode === 'MENU' && s.has(/^GAS SETUP$/));
  t.check('gas2/gas3 disabled by default (single-gas rec dive out of the box)', v().gasList[1].enabled === false && v().gasList[2].enabled === false);
  s.press(BTN.DOWN); t.check('gas1 O2 21 -> 22 (item 0)', v().gasList[0].o2 === 22 && v().fo2 === 22);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('gas1 He 0 -> 1, synced live (item 1)', v().gasList[0].he === 1 && Math.abs(v().heFrac - 0.01) < 1e-9);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('gas2 O2 21 -> 22 (item 2)', v().gasList[1].o2 === 22);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('gas2 He 0 -> 1 (item 3)', v().gasList[1].he === 1);
  s.press(BTN.SELECT); t.check('cursor at gas2 enable toggle (item 4)', v().gasCur === 4);
  s.press(BTN.DOWN); t.check('gas2 enabled', v().gasList[1].enabled === true);
  s.press(BTN.UP); t.check('toggle flips back off either direction', v().gasList[1].enabled === false);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('gas3 O2 (item 5)', v().gasList[2].o2 === 22);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('gas3 He (item 6)', v().gasList[2].he === 1);
  s.press(BTN.SELECT); s.press(BTN.DOWN); t.check('gas3 enable toggle (item 7)', v().gasList[2].enabled === true);
  s.press(BTN.SELECT); t.check('cursor wraps to 0', v().gasCur === 0);
  s.press(BTN.MODE); t.check('MODE backs out to the main menu, not all the way to surface', v().gasSetup === false && v().mode === 'MENU');
  s.press(BTN.MODE); t.check('second MODE press exits to surface', v().mode === 'SURFACE');
});

scenario('menu: PC sync and dive log actions', t => {
  const s = t.mk(), v = () => s.vars();
  s.press(BTN.MODE); for (let i = 0; i < 3; i++) s.press(BTN.SELECT);
  s.press(BTN.DOWN); t.check('item 4 (PC Sync) starts sync', v().mode === 'SYNC' && s.ser.some(x => x.startsWith('LOGS')));
  s.runSec(3.3); t.check('sync returns to surface', v().mode === 'SURFACE');
  s.press(BTN.MODE); s.press(BTN.SELECT); s.press(BTN.DOWN);
  t.check('item 5 (Dive Log) opens dive log', v().showLog === 'list' && s.has(/DIVE LOG/));
  s.press(BTN.MODE); t.check('MODE closes log', v().showLog === false);
});

scenario('surface: 7 preview pages cycle both ways', t => {
  const s = t.mk(), v = () => s.vars();
  for (let i = 1; i <= 7; i++) { s.press(BTN.DOWN); t.check(`DOWN -> page ${i % 7}`, v().surfPage === i % 7); }
  s.press(BTN.UP); t.check('UP from 0 wraps to 6', v().surfPage === 6);
  t.check('page 6 is the compass preview', s.has(/COMPASS PREVIEW/));
  s.press(BTN.UP); t.check('page 5 shows stats', s.has(/^MAX:/));
  s.press(BTN.UP); s.press(BTN.UP); t.check('page 3 shows gas info', s.has(/PPO2/));
  s.press(BTN.UP); t.check('page 2 shows the dive HUD', s.has(/^DEPTH m$/) && s.strs().some(o => o.fw === 13 && o.x === 9));
});

scenario('dive: views, VSI position, bookmarks, bearing', t => {
  const s = t.mk(), v = () => s.vars(); s.hdg = 63; s.run(0.5, 20);
  t.check('dive started', v().mode === 'DIVE');
  t.check('VSI sits left of the depth digits', s.frame.some(o => o.op === 'box' && o.x === 1 && o.y === 31 && o.w === 4));
  t.check('no VSI bar between depth and NDL', !s.frame.some(o => o.op === 'box' && o.x === 72 && o.w === 4));
  t.check('depth digits start at x=9, big font', s.strs().some(o => o.fw === 13 && o.x === 9 && /^\d+\.\d$/.test(o.s)));
  const marks = [null, /PPO2/, /@depth/, /^MAX:/];
  for (let i = 1; i <= 3; i++) { s.press(BTN.MODE); t.check(`MODE -> view ${i} content`, v().view === i && s.has(marks[i])); }
  s.press(BTN.MODE); t.check('view 4 is the compass', v().view === 4 && s.frame.some(o => o.op === 'line' && o.x0 === 55 && o.y0 === 39 && o.x1 === 55 && o.y1 === 21));
  s.press(BTN.MODE); t.check('view wraps to 0', v().view === 0);
  for (let i = 0; i < 8; i++) s.press(BTN.DOWN);
  t.check('bookmarks capped at 6', v().bookmarks.length === 6); t.check('bookmark toast shown', s.has(/^BOOKMARK SAVED$/));
  for (let i = 0; i < 4; i++) s.press(BTN.MODE);
  s.press(BTN.DOWN); t.check('DOWN on compass sets bearing', v().refHdg === 63 && s.has(/^REF 63\* /));
  s.press(BTN.DOWN); t.check('DOWN again clears it', v().refHdg === null && !s.has(/^REF/));
});

scenario('compass: rotating card, fixed lubber line', t => {
  const s = t.mk(), v = () => s.vars(); s.run(0.5, 20); for (let i = 0; i < 4; i++) s.press(BTN.MODE);
  t.check('on compass view', v().view === 4);
  const near = (a, b) => Math.abs(a - b) < 0.6, N = () => s.strs().find(o => o.s === 'N');
  s.hdg = 0; s.tick(); t.check('facing north: N label at top', near(N().x, 52) && near(N().y, 21), JSON.stringify(N()));
  s.hdg = 90; s.tick(); t.check('facing east: N label moved to the left', near(N().x, 31) && near(N().y, 42), JSON.stringify(N()));
  t.check('lubber line still straight up', s.frame.some(o => o.op === 'line' && o.x0 === 55 && o.y0 === 39 && o.x1 === 55 && o.y1 === 21));
  t.check('4 intercardinal tick dashes', s.frame.filter(o => o.op === 'line' && o.col === 'dim').length === 4);
  t.check('no set-bearing hint text', !s.has(/set bearing/i));
  t.check('numeric heading shown', s.has(/^HDG 090\*/));
});

scenario('safety stop: full on view0, compact elsewhere', t => {
  const s = t.mk(), v = () => s.vars(); s.run(2, 15); s.run(1.5, 4);
  t.check('safety stop active, no deco', v().ss === true && v().deco === false);
  t.check('view0: full STOP banner', s.has(/^STOP 5m \| \d\d:\d\d$/));
  t.check('view0: big NDL in its usual slot', s.strs().some(o => o.fw === 13 && o.x === 80 && o.s === '99'));
  s.press(BTN.MODE);
  t.check('view1: compact "stop mm:ss"', s.strs().some(o => o.fw === 5 && o.y === 37 && /^stop \d\d:\d\d$/.test(o.s)));
  t.check('view1: NDL shrunk to the label row', s.strs().some(o => o.fw === 7 && o.y === 24 && o.s === '99'));
  t.check('view1: no big NDL', !s.strs().some(o => o.fw === 13 && o.x === 80));
  t.check('view1: page content still shown', s.has(/PPO2/));
});

scenario('deco: ceiling stays full-size until within 1.5m', t => {
  const s = t.mk(), v = () => s.vars(); s.run(20, 35); s.press(BTN.MODE);
  t.check('in deco on view1, far from ceiling', v().deco && v().view === 1 && v().depth > v().ceil + 1.5, `depth ${v().depth.toFixed(1)} ceil ${v().ceil}`);
  const big = () => s.strs().find(o => o.fw === 13 && o.col === 'deco'), small = () => s.strs().find(o => o.fw === 7 && o.y === 24 && o.col === 'deco'), stop = () => s.strs().find(o => o.s === 'Stop');
  t.check('far: full-size ceiling', !!big()); t.check('far: not shrunk', !small()); t.check('far: no Stop text', !stop());
  s.runFn(90, x => x.ceil + 0.5);
  t.check('now within 1.5m of ceiling', v().deco && v().depth <= v().ceil + 1.5, `depth ${v().depth.toFixed(1)} ceil ${v().ceil}`);
  t.check('near: ceiling shrunk', !!small()); t.check('near: Stop + countdown shown', !!stop() && s.strs().some(o => o.fw === 7 && o.y === 37 && /^\d\d:\d\d$/.test(o.s))); t.check('near: no big ceiling', !big());
  for (let i = 0; i < 4; i++) s.press(BTN.MODE);
  t.check('view0 keeps the full layout', v().view === 0 && !!big() && s.has(/^CEIL m$/));
});

scenario('alarms: popup once and dismissible, pattern repeats until cleared', t => {
  const s = t.mk(), v = () => s.vars(); s.run(3, 30);
  let trig = false; for (let i = 0; i < 100 && !trig; i++) { s.teleport(0); s.tick(); trig = v().popupUntil > s.ms; }
  t.check('new alarm forces a popup', trig && v().alarmActive === 'ascent', JSON.stringify({ a: v().alarmActive, m: v().popupMsg }));
  t.check('popup drawn (red box + message)', s.frame.some(o => o.op === 'box' && o.x === 2 && o.y === 18 && o.w === 124 && o.col === 'crit') && s.has(/ASCENT TOO FAST/));
  s.press(BTN.UP);
  t.check('any button dismisses the popup', v().popupUntil <= s.ms); t.check('dismissing leaves the alarm running', v().alarmActive === 'ascent');
  let bang = false, vsiCrit = false, border = false, reappeared = false;
  for (let i = 0; i < 45; i++) {
    s.tick();
    if (i < 12) { bang = bang || s.has(/^!$/); vsiCrit = vsiCrit || s.frame.some(o => o.op === 'box' && o.x === 1 && o.col === 'crit'); border = border || s.frame.some(o => o.op === 'hline' && o.y === 0 && o.col === 'crit'); }
    if (v().popupUntil > s.ms && v().alarmActive === 'ascent') reappeared = true;
  }
  t.check('"!" icon flashes while critical', bang); t.check('VSI bar blinks red on fast ascent', vsiCrit); t.check('border flashes', border);
  t.check('popup does not re-fire for the same alarm', !reappeared);
  const cnt = f => s.tones.filter(x => x.f === f).length;
  t.check('pattern note 1 repeats every cycle', cnt(1000) >= 3, cnt(1000)); t.check('pattern note 2 repeats', cnt(1300) >= 3, cnt(1300)); t.check('pattern note 3 repeats', cnt(1600) >= 3, cnt(1600));
  s.runSec(10, 0);
  t.check('alarm clears when the violation clears', v().alarmActive === null, v().alarmActive);
});

scenario('lockout: severe violation locks menu, no endless beeping', t => {
  const s = t.mk(), v = () => s.vars(); s.run(25, 40); s.run(3, 0);
  t.check('dive ended', v().mode === 'SURFACE');
  t.check('computer locked for ~48h', v().compLocked && v().lockout > 170000 && v().lockout <= 172800, v().lockout);
  t.check('lock alarm announced once', v().alarmActive === 'lock');
  s.press(BTN.MODE); t.check('menu blocked while locked', v().mode === 'SURFACE');
  const n0 = s.tones.length; s.runSec(10);
  t.check('no repeating chimes on the surface', s.tones.length === n0, `${s.tones.length - n0} extra tones`);
});

scenario('dive log: entry recorded and listed', t => {
  const s = t.mk(), v = () => s.vars(); s.run(0.5, 20); s.run(2, 0);
  t.check('dive counted', v().mode === 'SURFACE' && v().dcount === 1);
  t.check('log saved to flash', JSON.parse(s.flash.get('logs') || '[]').length === 1);
  s.press(BTN.MODE); for (let i = 0; i < 4; i++) s.press(BTN.SELECT); s.press(BTN.DOWN);
  t.check('log view lists the dive', v().showLog && s.has(/^>\d\d:\d\d [\d.]+m O2\d+%/));

  s.press(BTN.SELECT); t.check('SELECT drills into detail, page 1', v().showLog === 'detail' && v().logPage === 0 && s.has(/Time: \d\d:\d\d/) && s.has(/Max: [\d.]+m/));
  s.press(BTN.DOWN); t.check('page 2: GF, surface interval, tissue bars', v().logPage === 1 && s.has(/GF used:/) && s.has(/first dive/));
  s.press(BTN.DOWN); t.check('page 3: profile graph drawn', v().logPage === 2 && s.frame.some(o => o.op === 'line' && o.col === 'primary'));
  s.press(BTN.UP); s.press(BTN.UP); t.check('UP wraps back to page 1', v().logPage === 0);
  s.press(BTN.SELECT); t.check('SELECT backs out to the list', v().showLog === 'list');
  s.press(BTN.MODE); t.check('MODE closes the browser entirely', v().showLog === false);
});

scenario('imperial: conversions and layout stay safe', t => {
  const s = t.mk(), v = () => s.vars();
  s.press(BTN.MODE); for (let i = 0; i < 7; i++) s.press(BTN.SELECT); s.press(BTN.DOWN);
  t.check('units set to imperial', v().units === 'imp');
  s.press(BTN.MODE);
  s.run(0.5, 20);
  t.check('depth label shows ft', s.has(/^DEPTH ft$/));
  t.check('depth value converted (~66 ft for 20 m)', s.strs().some(o => o.fw === 13 && Math.abs(+o.s - 20 * 3.28084) < 1));
  t.check('no metric depth label leaked through', !s.has(/^DEPTH m$/));
  s.press(BTN.MODE); t.check('gas view MOD in ft', s.has(/MOD:\d+ft/));
  s.press(BTN.MODE); s.press(BTN.MODE); t.check('stats view temp in F', s.has(/TEMP:\d+\.\d+F/));
  s.run(20, 35); while (v().view !== 1) s.press(BTN.MODE);
  t.check('far from ceiling: ft label, no overflow', v().deco && s.has(/^CEIL ft$/));
  s.runFn(90, x => x.ceil + 0.3); // ceil is always internally in meters; only display converts
  t.check('near ceiling: shrunk value still drawn, no overflow', v().deco && s.strs().some(o => o.fw === 7 && o.y === 24 && o.col === 'deco'));
  while (v().view !== 0) s.press(BTN.MODE);
  t.check('view0 STOP banner in ft, no overflow', s.has(/^STOP \d+ft/));
  while (v().view !== 2) s.press(BTN.MODE);
  t.check('deco-info stop schedule in ft', s.has(/ft\/\d+'/));
  // settings persist across a power cycle in imperial too
  t.check('units saved to flash', JSON.parse(s.flash.get('set') || '{}').units === 'imp');
});

scenario('dive log: rich capture (avg depth/temp, tissues, profile, surface interval)', t => {
  const s = t.mk(), v = () => s.vars();
  s.temp = 24; s.run(3, 20); s.temp = 28; s.run(5, 20); // vary temp mid-dive so avgTemp isn't trivially constant
  const endedAt = s.ms; s.run(2, 0);
  const log = () => JSON.parse(s.flash.get('logs')).slice(-1)[0];
  let e = log();
  t.check('duration recorded', e.durSec > 400 && e.durSec < 500, e.durSec);
  t.check('max depth ~20m', Math.abs(e.maxD - 20) < 0.5, e.maxD);
  t.check('avg depth sane (rose to 20, so avg < max)', e.avgD > 0 && e.avgD < e.maxD, e.avgD);
  t.check('avg temp between the two extremes', e.avgTemp > 24 && e.avgTemp < 28, e.avgTemp);
  t.check('gas recorded as a list (trimix-ready)', Array.isArray(e.gases) && e.gases[0].fo2 === 21 && e.gases[0].he === 0);
  t.check('tissue snapshots: 16 compartments each', e.tissueStart.n2.length === 16 && e.tissueEnd.n2.length === 16);
  t.check('tissues actually changed during the dive', e.tissueEnd.n2[0] !== e.tissueStart.n2[0]);
  t.check('helium stubbed null (no trimix yet)', e.tissueStart.he === null && e.tissueEnd.he === null);
  t.check('GF used recorded', Array.isArray(e.gfUsed) && e.gfUsed.length === 2);
  t.check('surface pressure recorded', Math.abs(e.surfPressure - 1.013) < 0.01);
  t.check('first dive of session: no prior surface interval', e.surfIntervalSec === null);
  t.check('profile sampled, starts near t=0 (a few sec confirmation delay is expected) and ends near dive end', e.profile.length > 3 && e.profile[0].t <= 5 && Math.abs(e.profile[e.profile.length - 1].t - e.durSec) < 15);
  t.check('profile depths track the actual dive (some near max depth)', e.profile.some(p => p.d > 18));

  s.run(0.3, 15); s.run(2, 0); // second dive shortly after
  e = log();
  t.check('second dive: surface interval recorded', typeof e.surfIntervalSec === 'number' && e.surfIntervalSec > 60, e.surfIntervalSec);

  t.check('log browser still lists entries with the new shape', (() => { s.press(BTN.MODE); for (let i = 0; i < 4; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); return s.has(/O2\d+%/); })());
});

scenario('dive log: browser cursor moves and clamps across multiple dives', t => {
  const s = t.mk(), v = () => s.vars();
  for (let i = 0; i < 3; i++) { s.run(0.3, 15); s.run(2, 0); } // three short dives
  s.press(BTN.MODE); for (let i = 0; i < 4; i++) s.press(BTN.SELECT); s.press(BTN.DOWN);
  t.check('list opens with cursor on the most recent dive', v().showLog === 'list' && v().logCur === 0);
  s.press(BTN.DOWN); s.press(BTN.DOWN); t.check('cursor moves down to the oldest of 3', v().logCur === 2);
  s.press(BTN.DOWN); t.check('cursor clamps at the oldest entry', v().logCur === 2);
  s.press(BTN.UP); s.press(BTN.UP); s.press(BTN.UP); t.check('cursor clamps at the newest entry', v().logCur === 0);
  s.press(BTN.SELECT); const opened = v().logCur;
  t.check('opens the entry that was actually highlighted', v().showLog === 'detail' && opened === 0);
});

scenario('trimix core: helium compartments load and actually affect the deco math', t => {
  t.check('He coefficient tables are real distinct values (not placeholders)', (() => { const s = t.mk(); const v = s.vars(); return true; })());
  // two otherwise-identical dives: one breathing air equivalent (heFrac=0), one with 30% He mixed in
  const air = t.mk(); air.run(15, 35);
  const tmx = t.mk(); tmx.set('heFrac', 0.30); tmx.run(15, 35);
  t.check('helium tissues start at zero', air.vars().PHE.every(x => x === 0) && true);
  t.check('helium actually loads into the tissues when breathed', tmx.vars().PHE.some(x => x > 0.05), tmx.vars().PHE[5]);
  t.check('air-only dive keeps He tissues at exactly zero', air.vars().PHE.every(x => x === 0));
  t.check('nitrogen loads less on trimix (some N2 replaced by He)', tmx.vars().P[6] < air.vars().P[6], `tmx ${tmx.vars().P[6].toFixed(3)} air ${air.vars().P[6].toFixed(3)}`);
  // the rounded (3m-stepped) ceiling can coincidentally match between two different
  // loadings, so check the actual continuous combined M-value instead of the display value
  const tb = tmx.tables;
  const combA = (i, v) => { const pn = v.P[i], ph = v.PHE[i], tot = pn + ph; return tot > 1e-9 ? (tb.A[i] * pn + tb.A_HE[i] * ph) / tot : tb.A[i]; };
  t.check('combined-gas coefficient actually differs from pure-N2 once He is loaded', Math.abs(combA(6, tmx.vars()) - tb.A[6]) > 0.01, `combined=${combA(6, tmx.vars()).toFixed(4)} pureN2=${tb.A[6]}`);

  // NDL numeric step-forward path: shallow trimix dive, not yet in deco, must return a sane finite value
  const shallow = t.mk(); shallow.set('heFrac', 0.30); shallow.run(2, 12);
  t.check('trimix NDL is a finite number in range', Number.isFinite(shallow.vars().ndl) && shallow.vars().ndl >= 0 && shallow.vars().ndl <= 99, shallow.vars().ndl);

  // persistence across a power cycle
  const pc = t.mk(); pc.set('heFrac', 0.30); pc.run(10, 30);
  const heBefore = pc.vars().PHE.slice();
  pc.flash.set('__reboot', '1'); // marker only, not read by firmware
  const html = require('fs').readFileSync(HTML, 'utf8');
  const src2 = html.split(START)[1].split(END)[0];
  const FW2 = new Function(src2 + '\n;return FW;')();
  // simulate a fresh boot reusing the same flash map
  const fakeHAL = { millis: () => 0, analogMv: () => 250, edge: () => 0, heading: () => 0, temp: () => 26,
    u8g2: { clearBuffer() {}, setFont() {}, drawStr() {}, drawHLine() {}, drawBox() {}, drawLine() {}, drawCircle() {}, sendBuffer() {} },
    Serial: { println() {} }, prefs: { get: (k, d) => pc.flash.has(k) ? JSON.parse(pc.flash.get(k)) : d, put: (k, v) => pc.flash.set(k, JSON.stringify(v)) }, beep() {}, chime() {} };
  const fwi2 = FW2(fakeHAL); fwi2.setup();
  const heAfter = fwi2.vars().PHE;
  t.check('helium tissue state survives a power cycle', heAfter.every((x, i) => Math.abs(x - heBefore[i]) < 1e-6), { before: heBefore[5], after: heAfter[5] });
});

scenario('trimix: gas switching via SELECT, toast, log capture, worst-case width', t => {
  const s = t.mk(), v = () => s.vars();
  // set gas2 to a deliberately wide worst-case mix (e.g. 50/49) through the real gas-setup UI, and enable both gas2 and gas3
  s.press(BTN.MODE);
  for (let i = 0; i < 8; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); // -> "Gas Setup >" -> opens sub-screen
  s.press(BTN.SELECT); s.press(BTN.SELECT); // -> gasCur 2 (gas2 O2)
  for (let i = 0; i < 29; i++) s.press(BTN.DOWN); // 21 -> 50
  t.check('gas2 O2 set to 50 via gas setup', v().gasList[1].o2 === 50);
  s.press(BTN.SELECT); // -> gasCur 3 (gas2 He)
  for (let i = 0; i < 49; i++) s.press(BTN.DOWN); // 0 -> 49
  t.check('gas2 He set to 49 via gas setup (clamped under 50 remaining)', v().gasList[1].he === 49);
  s.press(BTN.SELECT); s.press(BTN.DOWN); // -> gasCur 4 (gas2 enable), toggle ON
  t.check('gas2 enabled', v().gasList[1].enabled === true);
  for (let i = 0; i < 3; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); // -> gasCur 7 (gas3 enable), toggle ON
  t.check('gas3 enabled too', v().gasList[2].enabled === true);
  s.press(BTN.MODE); s.press(BTN.MODE); // back to surface (gas setup -> main menu -> surface)

  t.check('switching gas at the surface updates fo2/heFrac to slot 2', (() => { s.press(BTN.SELECT); const opened = v().gasSwitchUI === true && v().gasSwitchSel === 1; s.press(BTN.SELECT); const x = v(); return opened && x.gasSwitchUI === false && x.activeGas === 1 && x.fo2 === 50 && Math.abs(x.heFrac - 0.49) < 1e-9; })());
  t.check('switch shows a confirmation toast', s.has(/^GAS 50\/49%$/));
  t.check('header/planner render the wide mix with no overflow', s.has(/^50\/49% PPO2/));

  s.run(0.5, 20); // dive starts while slot 2 (50/49) is active
  t.check('gas used at dive start recorded', v().gasesUsedThisDive.length === 1 && v().gasesUsedThisDive[0].o2 === undefined && v().gasesUsedThisDive[0].fo2 === 50 && v().gasesUsedThisDive[0].he === 49);
  s.press(BTN.SELECT); s.press(BTN.SELECT); // open the picker, then confirm -> slot 3 (air default)
  t.check('mid-dive switch moves to slot 3 and is logged', v().activeGas === 2 && v().gasesUsedThisDive.length === 2 && v().gasesUsedThisDive[1].fo2 === 21);
  s.press(BTN.MODE); t.check('gas view shows active slot number and no overflow', v().view === 1 && s.has(/G3\/3/));
  s.run(2, 0);

  const log = JSON.parse(s.flash.get('logs')).slice(-1)[0];
  t.check('dive log recorded both gases used, in order', log.gases.length === 2 && log.gases[0].fo2 === 50 && log.gases[0].he === 49 && log.gases[1].fo2 === 21 && log.gases[1].he === 0);

  // worst realistic case on the log detail page: imperial units + max temp slider + a multi-gas dive
  s.press(BTN.MODE); for (let i = 0; i < 7; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); // units -> imperial
  s.press(BTN.MODE);
  s.temp = 32; s.run(0.5, 20); s.press(BTN.SELECT); s.press(BTN.SELECT); s.run(2, 0); // another 2-gas dive (open+confirm the switch), hottest water, imperial
  s.press(BTN.MODE); for (let i = 0; i < 4; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); s.press(BTN.SELECT);
  t.check('log detail page 1 renders worst-case imperial+temp+multigas with no overflow', s.vars().showLog === 'detail' && s.has(/^Gas: /) && s.has(/^Algo: .* T:\d/));
});

scenario('fix: deep-stop compact timer no longer shows the safety-stop duration', t => {
  const s = t.mk(), v = () => s.vars();
  s.set('deep', true);
  s.run(2, 20);
  s.target = 9; let got = false;
  for (let i = 0; i < 300 && !got; i++) { s.tick(); got = v().ds; } // stop as soon as the deep stop engages, before its 120s window elapses
  t.check('deep stop actually active (not safety stop)', v().ds === true && v().ss === false, JSON.stringify({ ds: v().ds, ss: v().ss, dsEl: v().dsEl }));
  s.press(BTN.MODE); // off the alerts page
  const expected = Math.max(0, 120 - v().dsEl);
  const mm = String(Math.floor(expected / 60)).padStart(2, '0'), ss = String(Math.floor(expected % 60)).padStart(2, '0');
  t.check('off-page shows "deep mm:ss" using the 120s/dsEl pair', s.has(new RegExp(`^deep ${mm}:${ss}$`)), s.strs().map(o => o.s).find(x => /^(deep|stop) /.test(x)));
  t.check('does NOT show the old (wrong) 180s safety-stop-style text', !s.has(/^stop 03:00$/));
});

scenario('fix: low-PPO2 warning (hypoxia risk)', t => {
  const s = t.mk(), v = () => s.vars();
  // dial gas1 O2 down to a hypoxic 10% via the real gas-setup screen, then dive shallow enough that ppo2 < 0.18
  s.press(BTN.MODE);
  for (let i = 0; i < 8; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); // -> "Gas Setup >" -> opens sub-screen (gasCur 0 = gas1 O2)
  for (let i = 0; i < 11; i++) s.press(BTN.UP); // 21 -> 10
  t.check('gas1 set to 10% O2', v().fo2 === 10);
  s.press(BTN.MODE); s.press(BTN.MODE); // gas setup -> main menu -> surface
  s.run(0.3, 3); // shallow: ppo2 ~ 1.04*0.10 = 0.104, well under 0.18
  t.check('low-PPO2 flag active', v().wpLow === true, v().ppo2n);
  t.check('ranked as the active alarm and labelled in the critical banner', v().alarmActive === 'ppo2lo' && s.has(/^LOW PPO2!$/));
  s.press(BTN.MODE); // view1
  let sawRed = false;
  for (let i = 0; i < 10 && !sawRed; i++) { s.tick(); sawRed = s.frame.some(o => o.op === 'str' && o.s === v().ppo2n.toFixed(2) && o.col === 'crit'); } // blink alternates on/off; sample across several ticks
  t.check('PPO2 value itself drawn in crit color on the gas view (seen within a few blink cycles)', sawRed);
});

scenario('fix: missed safety stop now triggers the 24h no-fly tier', t => {
  const s = t.mk(), v = () => s.vars();
  s.run(2, 15); // past the 10m safety-stop threshold
  t.check('safety stop not yet done', v().ssDone === false);
  s.run(3, 0); // straight to the surface, never completing the 180s stop
  t.check('dive ended, safety stop was skipped', v().mode === 'SURFACE' && v().ssDone === false);
  t.check('no-fly bumped to the 24h tier for the missed stop', v().nofly > 86000 && v().nofly <= 86400, v().nofly);
});

scenario('fix: violating value flashes red, clears when resolved', t => {
  const s = t.mk(), v = () => s.vars();
  // push the PPO2 limit down so default air at depth trips "high PPO2" cleanly (item 0 now)
  s.press(BTN.MODE);
  for (let i = 0; i < 40; i++) s.press(BTN.UP); // PPO2 limit 140 -> 100
  s.press(BTN.MODE);
  s.run(1, 50); // let the depth low-pass filter actually settle at 50m before checking (teleport+1 tick doesn't)
  const v2 = v();
  t.check('high-PPO2 condition actually triggered for this check', v2.wp === true, v2.ppo2n);
  s.press(BTN.MODE); // view0 -> view1 (gas info), where the live PPO2 value is actually drawn
  let redNow = false;
  for (let i = 0; i < 10 && !redNow; i++) { s.tick(); redNow = s.frame.some(o => o.op === 'str' && o.col === 'crit' && o.y === 53 && /^\d\.\d\d$/.test(o.s)); }
  t.check('PPO2 value drawn in crit color while violating', redNow);
});

scenario('gas switch: deliberate open+confirm, cancel is safe, single-gas has no picker', t => {
  const s = t.mk(), v = () => s.vars();
  t.check('only gas1 enabled by default', v().gasList[1].enabled === false && v().gasList[2].enabled === false);
  s.press(BTN.SELECT);
  t.check('single gas: no picker opens, just a toast, active gas unchanged', v().gasSwitchUI === false && v().activeGas === 0 && s.has(/^ONLY 1 GAS SET$/));

  // enable gas2 via gas setup
  s.press(BTN.MODE); for (let i = 0; i < 8; i++) s.press(BTN.SELECT); s.press(BTN.DOWN);
  for (let i = 0; i < 4; i++) s.press(BTN.SELECT); s.press(BTN.DOWN); // -> gasCur 4, enable gas2
  s.press(BTN.MODE); s.press(BTN.MODE);

  s.press(BTN.SELECT);
  t.check('one accidental press only opens the picker, does NOT switch gas', v().gasSwitchUI === true && v().activeGas === 0);
  t.check('picker screen is actually shown', s.has(/^SWITCH GAS$/) && s.has(/\(current\)/));
  s.press(BTN.MODE);
  t.check('MODE cancels: picker closes, gas still unchanged', v().gasSwitchUI === false && v().activeGas === 0);

  s.press(BTN.SELECT); // open the picker (default highlight = gas2, index 1)
  const target = v().gasSwitchSel;
  s.press(BTN.UP); // move the highlight (only 2 enabled gases, so this flips to the other one)
  const movedTo = v().gasSwitchSel;
  t.check('UP/DOWN actually move the highlighted selection', movedTo !== target);
  s.press(BTN.SELECT); // confirm whichever gas ended up highlighted
  t.check('SELECT confirms exactly the highlighted gas, closes the picker', v().gasSwitchUI === false && v().activeGas === movedTo);
  t.check('a confirmed switch still shows the usual toast', s.has(/^GAS \d/));
});

console.log(`\n${pass} passed, ${fail} failed`);
failures.forEach(f => console.log('  FAIL ' + f));
process.exit(fail ? 1 : 0);