// DCT3 navigation / repro harness — drive the phone through menus headlessly,
// render the screen at each step, and detect spins.
//
// Boots the web build (cycle-paced, like the browser), plays a key script over
// EMULATED time (dct3_web_key enqueues taps; the run loop plays each atomically),
// renders the framebuffer to a PNG before the first key and after every key, and
// tracks the PC during each step so a spin (the firmware stuck looping on one PC,
// e.g. the 0x2eebee stack-guard reboot) is flagged with where it landed.
//
// Usage:
//   node tools/nav.mjs <fw.fls> [--eeprom <file>] [--out <dir>] [--watch 0xPC[,0xPC]]
//                       [--settle <frames>] [--per <frames>] -- <key> <key> ...
// Keys: 0-9  *  #  menu  up  down  c(=Names/Clear)  pwr(=power tap)  off(=power hold)
//       wait (just run+render, no key)
// Example (enter security code then open the menu):
//   node tools/nav.mjs "flash/My 3310 NR1 v5.79.fls" -- 6 2 6 2 8 menu
//
// Render BEFORE each key is the rule (see memory nav-harness-workflow): the PNGs
// let you verify the screen instead of pressing blind. PNGs need ImageMagick
// `convert`; without it the raw .pgm is written.

import { createRequire } from 'module';
import { readFileSync, writeFileSync, mkdirSync } from 'fs';
import { resolve } from 'path';
import { execSync } from 'child_process';

const require = createRequire(import.meta.url);
const webDir = '/home/dan/projects/nokia-dct3-emu/web';

// ---- args (resolve paths before chdir) ----
const argv = process.argv.slice(2);
let fw = null, eeprom = null, outDir = '/tmp/nav', watch = [], settle = 240, per = 45, replay = null, idle = 0;
let stubPcs = [];   // --stub: handler entry PCs to force-return (task isolation)
let wrwatchAddr = 0;   // --wrwatch: RAM addr to aggregate writer PCs for
let battAdc = null, chargerAdc = null;   // --batt / --charger: override vbatt adc[2] / charger adc[5]
// Boot config toggles (null = leave web default). --sim/--bypass/--spike/--faid 0|1.
// Defaults in main.c: sim=0 (absent), bypass=1, skip_seclock/FAID=1, spike=auto.
let cfgSim = null, cfgBypass = null, cfgSpike = null, cfgFaid = null, clockEvery = 0;
// Phase 6 Instrument 1 (alloc/free leak-tracker). LEAKTRACE=1 arms the leak-tracker
// at boot (records every outstanding heap block by allocating caller_LR); HEAPCURVE=1
// arms the [0x104844+8] used-bytes sampler and logs the curve every clockEvery frames.
const leakTrace = process.env.LEAKTRACE === '1';
const heapCurve = process.env.HEAPCURVE === '1';
// Phase 6 Instrument 2 (typed branch-trace ring). BRANCHRING=1 arms the ring (records
// every taken branch with type + call-depth + key regs); BRANCHARM=<step> arms the heavy
// per-branch decode only PAST that step (default ~200M, so the first 200M run at full
// speed and only the wedge window pays the cost). On an LCD-write-counter freeze (no UI
// redraw over W=240 frames past the onset) the ring + a one-shot wedge snapshot are dumped.
const branchRing = process.env.BRANCHRING === '1';
const branchArm  = process.env.BRANCHARM !== undefined ? +process.env.BRANCHARM : 200000000;
// Auto-recover gate: DEFAULT OFF in headless. Rationale — nav.mjs runs are for repro
// verification, so we want the bug to fire the same way the original recorded it
// (CPU spinning at the reboot fn), not be silently recovered past. Opt back in with
// --recover 1 if you specifically want to test the recovery path or run the firmware
// further past the crash for downstream behaviour.
let cfgRecover = 0;
// ─────────────────────────────────────────────────────────────────────────────
// Responsiveness probe (Phase 5) — periodic-keypress + idle-sleep A/B.
//   --check-wave0          run the Wave-0 self-checks (cadence monotonicity over
//                          ≥0x20000000 cyc + fresh-boot control-press settle
//                          calibration) and exit. No probe sweep. (Task 1)
//   --press-every N        periodic keep-awake cadence, in the cadence coordinate
//                          (C.step() — see WAVE-0 DECISIONS below) (arm B)
//   --press-key c          periodic press key (default 'c'; idempotent at home)
//   --press-from N         first periodic-press boundary (arm B)
//   --press-batch N        coarse sweep batch in CYCLES (default 4M; must be
//                          < --press-every to avoid skipping a cadence boundary)
//   --arm a|b|both         A = no-press baseline, B = press-every-N, both = run
//                          A then B as two sequential boots in one process
//   --end-probe-at T       undisturbed-idle depth (in the cadence coordinate) at
//                          which BOTH arms fire the discriminating `menu` press
//   --settle-cyc N         override the per-press settle window (cycles)
//
// ── WAVE-0 DECISIONS (Task 1; recorded here so Task 2's sweep keys off them) ──
//   CADENCE COORDINATE = C.step() (instruction count).
//     Rationale: C.cycles() exposes the emulator's raw cycle counter which is
//     documented to REBASE every 0x20000000 cyc (memory tag
//     timer-counters-rebase-monotonic-fix); only the RTC/timer path was moved to
//     rtc_mono, NOT the exported dct3_web_cycles. C.step() is a monotonic
//     instruction count. The Wave-0 --check-monotonic run (below) verifies which
//     is non-decreasing and prints the verdict. Every press is TAGGED with BOTH
//     step and cyc regardless (replay precedent), but cadence boundaries fire on
//     C.step(). End-probe depth (~5.8B) is therefore in STEPS.
//   SETTLE WINDOW default = 70_000_000 cyc (≈323 frames). Calibrated against a
//     fresh-boot `menu` control press: the wasm key sequencer plays down (hold
//     g_key_hold_insns — 200K insns on MAD2; 400K on serial-bus/5110 whose keypad debounce
//     FSM decodes only 197-293k after the down-edge) → up → settle (KEY_SETTLE_INSNS=1.2M insns) in
//     EMULATED time, and the firmware then needs tens of M cyc to dispatch
//     KEY_DOWN (0x2D84F4) and redraw the LCD. A 2M-cyc window reads a false
//     UNRESPONSIVE (KEY_DOWN+0, fbΔ=no); ~65M cyc (the proven `menu@300` window)
//     registers KEY_DOWN+1 + an LCD redraw. 70M gives margin. Widen via
//     --settle-cyc if a press at depth reads a false UNRESPONSIVE.
//   ANCHORS: 0x2D84F4 (KEY_DOWN dispatch) is NOT in the symbol store this phase —
//     used as a RAW --watch target (slot 0); Phase 6 promotes it. 0x2997B0
//     resolves to TASK_SEND_IRQ (slot 1, secondary cascade signal).
// ─────────────────────────────────────────────────────────────────────────────
let checkWave0 = false;
let pressEvery = 0, pressKey = 'c', pressFrom = 0, pressBatch = 4_000_000;
let arm = null, endProbeAt = 0, settleCyc = 70_000_000;
const keys = [];
for (let i = 0; i < argv.length; i++) {
  const a = argv[i];
  if (a === '--eeprom') eeprom = resolve(process.cwd(), argv[++i]);
  else if (a === '--out') outDir = resolve(process.cwd(), argv[++i]);
  else if (a === '--watch') watch = argv[++i].split(',').map(s => parseInt(s, 16) >>> 0);
  else if (a === '--stub') stubPcs = argv[++i].split(',').map(s => parseInt(s, 16) >>> 0);  // task-stub HLE: force-return handler entry PCs
  else if (a === '--wrwatch') wrwatchAddr = parseInt(argv[++i], 16) >>> 0;  // aggregate writer PCs of a RAM addr
  else if (a === '--batt') battAdc = parseInt(argv[++i], 16) >>> 0;         // override vbatt adc[2]
  else if (a === '--charger') chargerAdc = parseInt(argv[++i], 16) >>> 0;   // override charger adc[5]
  else if (a === '--settle') settle = +argv[++i];
  else if (a === '--per') per = +argv[++i];
  else if (a === '--sim') cfgSim = +argv[++i];        // 1=SIM inserted, 0=absent
  else if (a === '--bypass') cfgBypass = +argv[++i];  // 1=SIM-gate bypass on
  else if (a === '--spike') cfgSpike = +argv[++i];    // 1=force boot spike, 0=no spike
  else if (a === '--faid') cfgFaid = +argv[++i];      // 1=FAID/seclock bypass on
  else if (a === '--recover') cfgRecover = +argv[++i]; // 1=auto-recover firmware resets, 0=warm-reboot (headless default 0)
  else if (a === '--idle') idle = +argv[++i];   // post-replay idle-watch frames (delayed crash)
  else if (a === '--clockevery') clockEvery = +argv[++i];  // render+log a clock snapshot every N idle frames (60≈1 emulated min)
  // ── Responsiveness probe flags (Phase 5) ──
  else if (a === '--check-wave0') checkWave0 = true;       // run Wave-0 self-checks + exit (Task 1)
  else if (a === '--press-every') pressEvery = +argv[++i];
  else if (a === '--press-key') pressKey = String(argv[++i]);
  else if (a === '--press-from') pressFrom = +argv[++i];
  else if (a === '--press-batch') pressBatch = +argv[++i];
  else if (a === '--arm') arm = String(argv[++i]).toLowerCase();
  else if (a === '--end-probe-at') endProbeAt = +argv[++i];
  else if (a === '--settle-cyc') settleCyc = +argv[++i];
  // --replay: a session log from the browser. Accepts two shapes:
  //   - LEGACY flat array  [{key, step}, ...]            (dct3DumpReplay)
  //   - STRUCTURED bundle  {v,kind:"dct3-replay",fwId,fwName,config,events,context}
  //     (dct3CopyReplay)   — events have cyc + step, context has pc/step/cyc target.
  // For the structured bundle: config knobs (sim/pin/bypass/faid/spike) override CLI
  // flags, events are dispatched on cyc (deterministic), and after the last event the
  // run continues to context.cyc + auto-verdicts a PC match against context.pc.
  else if (a === '--replay') { const v = argv[++i]; try { replay = JSON.parse(v); } catch (e) { replay = JSON.parse(readFileSync(resolve(process.cwd(), v), 'utf8')); } }
  else if (a === '--') { while (++i < argv.length) keys.push(argv[i]); }
  else if (!fw) fw = resolve(process.cwd(), a);
  else keys.push(a);
}
if (!fw) { console.error('usage: nav.mjs <fw.fls> [--eeprom f] [--out d] [--watch 0xPC] -- <keys...>'); process.exit(2); }

// Probe mode is active when --arm or --check-wave0 is set. Apply the canonical
// repro-config defaults (SIM in / PIN off / bypass off / FAID pass on / spike off,
// recover off) unless the caller explicitly overrode a flag. (Phase 5)
const probeActive = checkWave0 || arm !== null;
if (probeActive) {
  if (cfgSim    === null) cfgSim    = 1;
  if (cfgBypass === null) cfgBypass = 0;
  if (cfgFaid   === null) cfgFaid   = 1;
  if (cfgSpike  === null) cfgSpike  = 0;
}
// Input validation (V5) for the probe numeric flags — positive integers, and
// --press-batch < --press-every so a coarse batch never skips a cadence boundary.
if (arm !== null) {
  if (!['a', 'b', 'both'].includes(arm)) { console.error(`bad --arm '${arm}' (want a|b|both)`); process.exit(2); }
  const posInt = (v, name) => { if (!Number.isInteger(v) || v <= 0) { console.error(`--${name} must be a positive integer (got ${v})`); process.exit(2); } };
  posInt(pressBatch, 'press-batch');
  posInt(endProbeAt, 'end-probe-at');
  posInt(settleCyc, 'settle-cyc');
  if (arm === 'b' || arm === 'both') {
    posInt(pressEvery, 'press-every');
    if (pressFrom < 0) { console.error(`--press-from must be >= 0 for arm ${arm} (got ${pressFrom})`); process.exit(2); }
    if (pressBatch >= pressEvery) { console.error(`--press-batch (${pressBatch} cyc) >= --press-every (${pressEvery} steps): note these are different units; ensure pressBatch_in_steps < pressEvery at your target IPC to avoid skipping a cadence boundary`); process.exit(2); }
  }
}
mkdirSync(outDir, { recursive: true });
process.chdir(webDir);

// Normalize --replay into { events, context, bundle }. Structured bundle shape lets
// us recover config + target cyc; legacy flat array carries only the step-keyed events.
let replayBundle = null, replayContext = null;
if (replay && !Array.isArray(replay) && typeof replay === 'object' && Array.isArray(replay.events)) {
  replayBundle = replay;
  replayContext = replay.context || null;
  // Bundle config overrides CLI flags (unless caller explicitly passed --sim etc.)
  const cfg = replay.config || {};
  if (cfgSim    === null && typeof cfg.sim    === 'boolean') cfgSim    = cfg.sim    ? 1 : 0;
  if (cfgBypass === null && typeof cfg.bypass === 'boolean') cfgBypass = cfg.bypass ? 1 : 0;
  if (cfgFaid   === null && typeof cfg.faid   === 'boolean') cfgFaid   = cfg.faid   ? 1 : 0;
  if (cfgSpike  === null && typeof cfg.spike  === 'boolean') cfgSpike  = cfg.spike  ? 1 : 0;
  replay = replay.events;
  console.log(`[replay] structured bundle (fwId=${replayBundle.fwId}, ${replay.length} events` +
              (replayContext ? `, target cyc=${(replayContext.cyc/1e6).toFixed(2)}M pc=${replayContext.pc}` : '') + ')');
}

// 3310 keypad matrix (from web/index.html data-row/data-col). [row,col].
const KEYS = {
  '0':[0,2],'1':[1,2],'2':[1,3],'3':[4,1],'4':[2,4],'5':[2,3],'6':[2,2],'7':[3,4],'8':[3,3],'9':[3,2],
  '*':[4,4],'#':[4,2],'menu':[4,3],'up':[0,1],'down':[1,1],'c':[0,4],
};

const M = await (require(webDir + '/dct3.js'))();
const C = {
  boot:   M.cwrap('dct3_web_boot', 'number', []),
  runCyc: M.cwrap('dct3_web_run_cycles', null, ['number']),
  key:    M.cwrap('dct3_web_key', null, ['number','number','number']),
  power:  M.cwrap('dct3_web_power', null, ['number']),
  off:    M.cwrap('dct3_web_powered_off', 'number', []),
  fb:     M.cwrap('dct3_web_fb', 'number', []),
  lcdw:   M.cwrap('dct3_web_lcd_writes', 'number', []),
  step:   M.cwrap('dct3_web_step', 'number', []),
  pc:     M.cwrap('dct3_web_pc', 'number', []),
  flashHi: M.cwrap('dct3_web_flash_hi', 'number', []),
  ram:    M.cwrap('dct3_web_ram', 'number', ['number']),
  ramPtr: M.cwrap('dct3_web_ram_ptr', 'number', []),
  ram:    M.cwrap('dct3_web_ram', 'number', ['number']),   // byte read (RAM is BE)
  eeOff:  M.cwrap('dct3_web_eeprom_off', 'number', []),
  eeSize: M.cwrap('dct3_web_eeprom_size', 'number', []),
  lcdW:   M.cwrap('dct3_web_lcd_w', 'number', []),
  lcdH:   M.cwrap('dct3_web_lcd_h', 'number', []),
  dbgWatch: M.cwrap('dct3_web_dbg_watch', null, ['number','number']),
  dbgCount: M.cwrap('dct3_web_dbg_count', 'number', ['number']),
  heapFailCount: M.cwrap('dct3_web_heap_fail_count', 'number', []),
  heapFailLr:    M.cwrap('dct3_web_heap_fail_lr', 'number', []),
  // Instrument 1: alloc/free leak-tracker (Phase 6). leakOn arms it (resets the
  // outstanding table); heapcurveOn arms the [0x104844+8] used-bytes sampler.
  stubAdd:       M.cwrap('dct3_web_stub_add', null, ['number']),    // task-stub HLE (--stub)
  stubHits:      M.cwrap('dct3_web_stub_hits', 'number', ['number']),
  wrwatchOn:     M.cwrap('dct3_web_wrwatch_on', null, ['number']),  // heap-counter write-watch (--wrwatch)
  wrwatchNpc:    M.cwrap('dct3_web_wrwatch_npc', 'number', []),
  wrwatchPc:     M.cwrap('dct3_web_wrwatch_pc', 'number', ['number']),
  wrwatchNet:    M.cwrap('dct3_web_wrwatch_net', 'number', ['number']),
  wrwatchCnt:    M.cwrap('dct3_web_wrwatch_cnt', 'number', ['number']),
  wrwatchLr:     M.cwrap('dct3_web_wrwatch_lr', 'number', ['number']),
  wrwatchSzn:    M.cwrap('dct3_web_wrwatch_szn', 'number', []),
  wrwatchSz:     M.cwrap('dct3_web_wrwatch_sz', 'number', ['number']),
  wrwatchSza:    M.cwrap('dct3_web_wrwatch_sza', 'number', ['number']),
  wrwatchSzf:    M.cwrap('dct3_web_wrwatch_szf', 'number', ['number']),
  acallN:        M.cwrap('dct3_web_acall_n', 'number', []),       // alloc-caller histogram (LEAKTRACE)
  acallLr:       M.cwrap('dct3_web_acall_lr', 'number', ['number']),
  acallCnt:      M.cwrap('dct3_web_acall_cnt', 'number', ['number']),
  acallEgsz:     M.cwrap('dct3_web_acall_egsz', 'number', ['number']),
  leakOn:        M.cwrap('dct3_web_leak_on', null, ['number']),
  heapcurveOn:   M.cwrap('dct3_web_heapcurve_on', null, ['number']),
  heapUsed:      M.cwrap('dct3_web_heap_used', 'number', []),
  leakCount:     M.cwrap('dct3_web_leak_count', 'number', []),
  leakDump:      M.cwrap('dct3_web_leak_dump', 'number', []),   // aggregates + returns buf ptr
  leakBuf:       M.cwrap('dct3_web_leak_buf', 'number', []),
  leakLen:       M.cwrap('dct3_web_leak_len', 'number', []),
  heapcurveCount:M.cwrap('dct3_web_heapcurve_count', 'number', []),
  heapcurveStep: M.cwrap('dct3_web_heapcurve_step', 'number', ['number']),
  heapcurveUsed: M.cwrap('dct3_web_heapcurve_used', 'number', ['number']),
  // Instrument 2: typed branch-trace ring (Phase 6). branchOn arms it (resets the
  // ring); branchArm sets the heavy-decode arm-step; per-field accessors mirror the
  // trace accessors. irq/fiq mask+pending feed the freeze-trigger wedge snapshot.
  branchOn:      M.cwrap('dct3_web_branch_on', null, ['number']),
  branchArm:     M.cwrap('dct3_web_branch_arm', null, ['number']),
  branchCount:   M.cwrap('dct3_web_branch_count', 'number', []),
  branchPc:      M.cwrap('dct3_web_branch_pc', 'number', ['number']),
  branchTarget:  M.cwrap('dct3_web_branch_target', 'number', ['number']),
  branchLr:      M.cwrap('dct3_web_branch_lr', 'number', ['number']),
  branchSp:      M.cwrap('dct3_web_branch_sp', 'number', ['number']),
  branchStep:    M.cwrap('dct3_web_branch_step', 'number', ['number']),
  branchType:    M.cwrap('dct3_web_branch_type', 'number', ['number']),
  branchDepth:   M.cwrap('dct3_web_branch_depth', 'number', ['number']),
  branchCpsrMode:M.cwrap('dct3_web_branch_cpsr_mode', 'number', ['number']),
  branchR0:      M.cwrap('dct3_web_branch_r0', 'number', ['number']),
  branchR1:      M.cwrap('dct3_web_branch_r1', 'number', ['number']),
  branchR4:      M.cwrap('dct3_web_branch_r4', 'number', ['number']),
  branchR9:      M.cwrap('dct3_web_branch_r9', 'number', ['number']),
  irqMask:       M.cwrap('dct3_web_irq_mask', 'number', []),
  fiqMask:       M.cwrap('dct3_web_fiq_mask', 'number', []),
  irqPending:    M.cwrap('dct3_web_irq_pending', 'number', []),
  fiqPending:    M.cwrap('dct3_web_fiq_pending', 'number', []),
  // send/enqueue logger ring (reused for the wedge snapshot's last-N task_send/recv).
  sendlogOn:     M.cwrap('dct3_web_sendlog_on', null, ['number']),
  sendlogW:      M.cwrap('dct3_web_sendlog_w', 'number', []),
  sendlogAt:     M.cwrap('dct3_web_sendlog_at', 'number', ['number']),
  sendlogLr:     M.cwrap('dct3_web_sendlog_lr', 'number', ['number']),
  resetReq:  M.cwrap('dct3_web_reset_request', 'number', []),
  warmReset: M.cwrap('dct3_web_warm_reset', 'number', []),
  // Reset-reason discriminator. mad2 catches every
  // [0x20001]|=4 with a per-reason counter; reset_recovered tracks how many of those
  // were exception-returned (reason 5 today; reason 12/104 once v2 LR-cap lands).
  resetTotal:     M.cwrap('dct3_web_reset_total', 'number', []),
  resetRecovered: M.cwrap('dct3_web_reset_recovered', 'number', []),
  resetLastReason:M.cwrap('dct3_web_reset_last_reason', 'number', []),
  resetLastPC:    M.cwrap('dct3_web_reset_last_pc', 'number', []),
  resetLastCpsr:  M.cwrap('dct3_web_reset_last_cpsr', 'number', []),
  resetCount:     M.cwrap('dct3_web_reset_count', 'number', ['number']),
  setRecover:     M.cwrap('dct3_web_set_recover', null, ['number']),
  getRecover:     M.cwrap('dct3_web_get_recover', 'number', []),
  eeProg:    M.cwrap('dct3_web_eeprom_writes', 'number', []),
  reg:       M.cwrap('dct3_web_reg', 'number', ['number']),
  cycles:    M.cwrap('dct3_web_cycles', 'number', []),
  rtcMin:    M.cwrap('dct3_web_rtc_min_edges', 'number', []),
  rtcRaw:    M.cwrap('dct3_web_rtc_raw', 'number', []),
  ccMask:    M.cwrap('dct3_web_ccont_mask', 'number', []),
  ccLines:   M.cwrap('dct3_web_ccont_lines', 'number', []),
  setSim:     M.cwrap('dct3_web_set_sim', null, ['number']),
  setBypass:  M.cwrap('dct3_web_set_bypass', null, ['number']),
  setBattery: M.cwrap('dct3_web_set_battery', null, ['number']),   // --batt: vbatt adc[2]
  setCharger: M.cwrap('dct3_web_set_charger', null, ['number']),   // --charger: adc[5]
  setSpike:   M.cwrap('dct3_web_set_spike', null, ['number']),
  setSeclock: M.cwrap('dct3_web_set_skip_seclock', null, ['number']),
  // frozen-on-crash PC trail.
  pcringCrashed: M.cwrap('dct3_web_pcring_crashed', 'number', []),
  pcringW:       M.cwrap('dct3_web_pcring_w', 'number', []),
  pcringN:       M.cwrap('dct3_web_pcring_n', 'number', []),
  pcringAt:      M.cwrap('dct3_web_pcring_at', 'number', ['number']),
  pcringCpsr:    M.cwrap('dct3_web_pcring_cpsr', 'number', ['number']),
};
const MODES = {0x10:'usr',0x11:'fiq',0x12:'irq',0x13:'svc',0x17:'abt',0x1b:'und',0x1f:'sys'};

// dump the last-N executed PCs (oldest→newest), each resolved to its fn.
function pcRingDump() {
  if (!C.pcringCrashed || !C.pcringCrashed()) return '';
  const N = C.pcringN(), w = C.pcringW() >>> 0;
  const lines = [];
  const count = Math.min(N, w);
  for (let k = count; k >= 1; k--) {
    const slot = ((w - k) % N + N) % N;
    const raw = C.pcringAt(slot) >>> 0;
    const cpsr = C.pcringCpsr ? (C.pcringCpsr(slot) >>> 0) : 0;
    const t = (cpsr & 0x20) ? 1 : 0;            // Thumb bit
    const md = cpsr & 0x1f;
    const ms = (MODES[md] || ('m' + md.toString(16)));
    const pc = (raw - (t ? 4 : 8)) >>> 0;       // de-pipe by the T bit
    const f = (pc >= FLASH_LO && pc < FLASH_HI) ? fnStart(pc) : 0;
    lines.push(`    [${ms}${t?'/T':'/A'}] ${hx(pc)} (raw ${hx(raw)})${f ? `  in fn ${hx(f)}` : ''}`);
  }
  return '  PC ring (oldest→newest, frozen at crash):\n' + lines.join('\n');
}

// Deep crash/spin diag (BIG-ENDIAN memory): registers + a backtrace (PC/LR + stack walk
// of plausible Thumb return addresses, each resolved to its function prologue B5xx).
// FLASH_HI is MODEL-AWARE — set from the wasm after boot (4 MB images run code up to
// 0x600000; a fixed 0x400000 false-trips upper-bank PCs as wild-PC). 0x400000 until boot.
const FLASH_LO = 0x200040;
let FLASH_HI = 0x400000;
const rd16be = (a) => ((C.ram(a) << 8) | C.ram(a + 1)) >>> 0;
const rd32be = (a) => ((C.ram(a) << 24) | (C.ram(a + 1) << 16) | (C.ram(a + 2) << 8) | C.ram(a + 3)) >>> 0;
const hx = (v) => '0x' + (v >>> 0).toString(16);
function fnStart(addr) { addr &= ~1; for (let a = addr; a >= FLASH_LO && a > addr - 0x1000; a -= 2) if ((rd16be(a) & 0xFF00) === 0xB500) return a; return 0; }

// --- Phase 6 Instrument 1: leak-attribution dump + used-bytes curve ---------
// Lines accumulated here are appended to summary.txt + printed to console. Read
// the C-side leak buffer like the postmortem buffer (HEAPU8.subarray + stringify),
// then annotate the top caller_LR with its nearest firmware symbol via fnStart.
const leakSummaryLines = [];
function dumpLeak(label) {
  if (!leakTrace || !C.leakDump) return;
  const ptr = C.leakDump() >>> 0, len = C.leakLen() >>> 0;
  if (!ptr || !len) return;
  let txt = '';
  const bytes = M.HEAPU8.subarray(ptr, ptr + len);
  for (let i = 0; i < bytes.length; i++) txt += String.fromCharCode(bytes[i]);
  // Annotate the rank-1 caller_LR with its nearest fn (the leaking site candidate).
  const m = txt.match(/^\s+1\s+\d+\s+\d+\s+0x([0-9A-Fa-f]{6})/m);
  if (m) { const lr = parseInt(m[1], 16) >>> 0, f = fnStart(lr & ~1);
    if (f) txt += `  top leaker 0x${lr.toString(16)} -> nearest fn ${hx(f)}\n`; }
  const block = `\n--- LEAK DUMP (${label}) ---\n${txt}`;
  console.log(block);
  leakSummaryLines.push(block);
}
function dumpHeapCurve(label) {
  if (!heapCurve || !C.heapcurveCount) return;
  const nC = C.heapcurveCount() >>> 0;
  if (!nC) return;
  const first = C.heapcurveUsed(0) >>> 0, firstStep = C.heapcurveStep(0) >>> 0;
  const lastI = Math.min(nC, 4096) - 1;
  const last = C.heapcurveUsed(lastI) >>> 0, lastStep = C.heapcurveStep(lastI) >>> 0;
  // Monotonicity check across the captured samples (a non-monotonic curve that
  // plateaus would argue against a steady bleed — favouring corruption, V3).
  let mono = true, prev = 0;
  for (let i = 0; i < Math.min(nC, 4096); i++) { const u = C.heapcurveUsed(i) >>> 0; if (u + 64 < prev) mono = false; prev = u; }
  const block = `\n--- HEAP-USED CURVE (${label}) ---\n` +
    `  ${first} B @ step ${firstStep} -> ${last} B @ step ${lastStep}  ` +
    `(${nC} samples, monotonic: ${mono ? 'yes' : 'NO'})\n`;
  console.log(block);
  leakSummaryLines.push(block);
}
function backtrace() {
  const r = []; for (let i = 0; i < 16; i++) r.push(C.reg(i) >>> 0);
  const cpsr = C.reg(16) >>> 0, sp = r[13], lines = [];
  const add = (w, a) => { const f = fnStart(a & ~1); lines.push(`    ${w.padEnd(9)} ${hx(a)}${f ? `  in fn ${hx(f)}` : ''}`); };
  add('PC', r[15]); add('LR', r[14]);
  let n = 0; for (let a = sp; a < sp + 0x300 && n < 14; a += 4) { const w = rd32be(a); if ((w & 1) && w >= FLASH_LO && w < FLASH_HI) { add('stk@' + hx(a).slice(2), w); n++; } }
  return `  regs: ${r.map((v, i) => `r${i}=${hx(v)}`).join(' ')}\n  cpsr=${hx(cpsr)} (${cpsr & 0x20 ? 'Thumb' : 'ARM'})\n  backtrace:\n${lines.join('\n')}`;
}

// --- Phase 6 Instrument 2: typed branch-ring dump + one-shot wedge snapshot ---
// On an LCD-stall freeze the ring (last-N taken branches, oldest->newest) + a one-shot
// snapshot (all regs+cpsr, irq/fiq mask+pending, distinct-PC histogram, last-N
// task_send/recv, final call-depth, and the 24 mailbox depths) are dumped, showing
// where/how the phone is wedged once the heap dies. Appended to summary.txt + console.
const BR_TYPE = ['B', 'BCOND', 'CALL', 'RETURN', 'BX', 'EXC'];
// Mailbox-ring table: 0x105E90, 28 bytes/task, head at +16,
// tail at +17 (A3 RESOLVED — the doc's authoritative layout, not the 0x112EA0 the
// plan/research had assumed). Depth ≈ (tail - head) modulo the ring capacity.
const MAILBOX_BASE = 0x105E90, MAILBOX_STRIDE = 28, MAILBOX_HEAD = 16, MAILBOX_TAIL = 17;
function branchRingDump(label, lastN = 256) {
  if (!branchRing || !C.branchCount) return '';
  const w = C.branchCount() >>> 0;
  if (!w) return `\n--- BRANCH RING (${label}) ---\n  (empty — ring not populated)\n`;
  const count = Math.min(w, 1024, lastN);
  const lines = [];
  for (let k = count; k >= 1; k--) {
    const i = (w - k) >>> 0;                       // global index; C masks into the ring
    const ty = C.branchType(i);
    if (ty < 0) continue;
    const pc = C.branchPc(i) >>> 0, tgt = C.branchTarget(i) >>> 0;
    const f = (pc >= FLASH_LO && pc < FLASH_HI) ? fnStart(pc) : 0;
    lines.push(
      `    [${(BR_TYPE[ty] || ty).padEnd(6)}] d${String(C.branchDepth(i)).padStart(3)}  ` +
      `${hx(pc)}${f ? `(${hx(f)})` : ''} -> ${hx(tgt)}  ` +
      `lr=${hx(C.branchLr(i) >>> 0)} sp=${hx(C.branchSp(i) >>> 0)} ` +
      `r0=${hx(C.branchR0(i) >>> 0)} r1=${hx(C.branchR1(i) >>> 0)} ` +
      `r4=${hx(C.branchR4(i) >>> 0)} r9=${hx(C.branchR9(i) >>> 0)} @step ${C.branchStep(i) >>> 0}`);
  }
  const lastIdx = (w - 1) >>> 0;
  const finalDepth = C.branchDepth(lastIdx);
  return `\n--- BRANCH RING (${label}, ${count} of ${w} taken branches, oldest->newest) ---\n` +
         `  final call-depth: ${finalDepth}\n${lines.join('\n')}\n`;
}
function wedgeSnapshot(label, hist) {
  if (!branchRing) return '';
  // All regs + cpsr.
  const r = []; for (let i = 0; i < 16; i++) r.push(C.reg(i) >>> 0);
  const cpsr = C.reg(16) >>> 0;
  // irq/fiq mask + pending (the ASIC interrupt state at the wedge).
  const im = C.irqMask ? C.irqMask() : 0, fm = C.fiqMask ? C.fiqMask() : 0;
  const ip = C.irqPending ? C.irqPending() : 0, fp = C.fiqPending ? C.fiqPending() : 0;
  // 24 mailbox depths (head/tail per slot @0x105E90).
  const mb = [];
  for (let t = 0; t < 24; t++) {
    const slot = MAILBOX_BASE + t * MAILBOX_STRIDE;
    const head = C.ram(slot + MAILBOX_HEAD) & 0xFF, tail = C.ram(slot + MAILBOX_TAIL) & 0xFF;
    const depth = (tail - head) & 0xFF;
    if (depth || head || tail) mb.push(`t${t}:${depth}(h${head}/t${tail})`);
  }
  // Last-N task_send/recv from the sendlog ring (if armed).
  let sendlog = '';
  if (C.sendlogW) {
    const sw = C.sendlogW() >>> 0, sN = Math.min(sw, 256, 24);
    const sl = [];
    for (let k = sN; k >= 1; k--) {
      const i = (sw - k) >>> 0, v = C.sendlogAt(i) >>> 0;
      sl.push(`task${(v >>> 16) & 0xFFFF}:msg${hx(v & 0xFFFF)}@${hx(C.sendlogLr(i) >>> 0)}`);
    }
    sendlog = sl.length ? `  last task_send/recv: ${sl.join(' ')}\n` : '';
  }
  // Distinct-PC histogram over the last W (reuse the idle-watch histogram).
  let histTxt = '';
  if (hist) {
    const tot = Object.values(hist).reduce((a, b) => a + b, 0) || 1;
    const top = Object.entries(hist).sort((a, b) => b[1] - a[1]).slice(0, 12);
    histTxt = '  distinct-PC histogram (top 12 over the freeze window):\n' +
      top.map(([p, cnt]) => { const a = +p, f = (a >= FLASH_LO && a < FLASH_HI) ? fnStart(a) : 0;
        return `    ${(cnt / tot * 100).toFixed(1).padStart(5)}%  ${hx(a)}${f ? ` in fn ${hx(f)}` : ''}`; }).join('\n') + '\n';
  }
  return `\n--- WEDGE SNAPSHOT (${label}) ---\n` +
    `  regs: ${r.map((v, i) => `r${i}=${hx(v)}`).join(' ')}\n` +
    `  cpsr=${hx(cpsr)} (${cpsr & 0x20 ? 'Thumb' : 'ARM'}, mode 0x${(cpsr & 0x1f).toString(16)})\n` +
    `  irq: mask=0x${(im & 0xff).toString(16)} pending=0x${(ip & 0xff).toString(16)}  ` +
    `fiq: mask=0x${(fm & 0xff).toString(16)} pending=0x${(fp & 0xff).toString(16)}\n` +
    `  mailbox depths @0x105E90 (28 B/slot, head+16/tail+17): ${mb.length ? mb.join(' ') : '(all empty)'}\n` +
    sendlog + histTxt;
}
function dumpWedge(label, hist) {
  if (!branchRing) return;
  const block = branchRingDump(label) + wedgeSnapshot(label, hist);
  console.log(block);
  leakSummaryLines.push(block);
}

// --- Reset / watchdog backtracing (firmware reboot path).
// The firmware reboots via 0x2EEBAE, storing a REASON byte at 0x11FF94 then writing
// [0x20001]|=4. mad2 catches the bit, discriminates by reason, and either RECOVERS
// (reason 5 today: exception-return from saved fault state at 0x11FF88) or sets
// reset_request so the host warm-reboots. We surface BOTH paths (so a recovered
// reset is still visible in the log) by polling resetTotal() between renders.
// Reason-byte/save-state addresses are 3310 v5.79; for other builds override via env
// RESET_REASON_ADDR / RESET_SAVE_ADDR (hex). The 0x11FF88 save block only populates
// on reason-5 path (other reasons leave it zero).
const RESET = {
  reasonAddr: process.env.RESET_REASON_ADDR ? parseInt(process.env.RESET_REASON_ADDR, 16) : 0x11FF94,
  saveAddr:   process.env.RESET_SAVE_ADDR   ? parseInt(process.env.RESET_SAVE_ADDR, 16)   : 0x11FF88,
  // reason -> name (caller of 0x2EEBAE): 5=watchdog/fatal(0x2F0BA4), 6=FIQ-canary, 4=factory reset,
  // 12=task1, 0x68=SWDSP (assertion 0x5B06 via fn 0x2D724C / 0x2EDD4A).
  names: { 2: 'reason-2', 3: 'reason-3', 4: 'factory-reset', 5: 'WATCHDOG/fatal',
           6: 'FIQ-canary', 12: 'task1-reboot', 0x68: 'SWDSP' },
};
let resetSeen = null;   // last decoded reason (for the run summary)
function reportReset(where, recovered) {
  // For recovered resets the firmware has already resumed — RAM may have moved on, so the
  // last_pc/last_cpsr the discriminator captured (above the recover_pending hook) is more
  // reliable than re-reading the save block. Fall back to RAM for non-recovered events.
  const reason = recovered ? (C.resetLastReason() >>> 0) : (C.ram(RESET.reasonAddr) >>> 0);
  const name = RESET.names[reason] || `reason-${reason}`;
  let lr, spsr, cpsr, source;
  if (recovered) {
    lr   = C.resetLastPC()   >>> 0;
    cpsr = C.resetLastCpsr() >>> 0;
    spsr = cpsr;   // same on the recovery path (we don't snapshot the handler CPSR)
    source = 'mad2 capture (recovered)';
  } else {
    lr   = rd32be(RESET.saveAddr);
    spsr = rd32be(RESET.saveAddr + 4);
    cpsr = rd32be(RESET.saveAddr + 8);
    source = `RAM @${hx(RESET.saveAddr)}`;
  }
  const f = fnStart(lr & ~1);
  const verdict = recovered ? 'RECOVERED' : 'RESET';
  console.log(`\n═══ ${verdict}: ${name} (reason ${reason}) — ${where} @ step ${(C.step() / 1e6).toFixed(1)}M ═══`);
  console.log(`  fault state (${source}): LR=${hx(lr)}${f ? `  in fn ${hx(f)}` : ''}  SPSR=${hx(spsr)}  CPSR=${hx(cpsr)}`);
  if (!recovered) console.log(backtrace());
  const ring = pcRingDump(); if (ring) console.log(ring);
  resetSeen = { reason, name, lr, recovered };
  return reason;
}
// Poll mad2's reset counters; if reset_total advanced since the last poll, that's a new
// catch. We compare delta(total) vs delta(recovered) to classify the new event(s) — a step
// where total advanced by 1 and recovered advanced by 1 was a recovery, otherwise warm-reboot.
// Returns the number of new catches seen (and prints one report).
let resetTotalSeen = 0, resetRecoveredSeen = 0;
function pollResetEvents(where) {
  const total = C.resetTotal();
  const recoveredCt = C.resetRecovered();
  if (total <= resetTotalSeen) return 0;
  const dTotal = total - resetTotalSeen;
  const dRec   = recoveredCt - resetRecoveredSeen;
  const recovered = dRec >= dTotal;   // batched-step: all-recovered if every new event was
  resetTotalSeen = total;
  resetRecoveredSeen = recoveredCt;
  reportReset(where, recovered);
  return dTotal;
}
// --watch PCs use the firmware-level hit counters (exact, every instruction) so we
// can see EXACTLY when a watched PC (e.g. the 0x2eebee spin) starts firing per step.
watch.slice(0, 6).forEach((pc, i) => C.dbgWatch(i, pc));
let watchPrev = watch.map(() => 0);

// Load fw (and optional EEPROM overlay) like the browser.
{ const b = new Uint8Array(readFileSync(fw)); try { M.FS.unlink('/fw.fls'); } catch (e) {} M.FS.writeFile('/fw.fls', b); }
C.boot();
FLASH_HI = (C.flashHi && C.flashHi() >>> 0) || 0x400000;   // model-aware code ceiling
// Apply boot-config toggles (after boot, before the settle run — they're re-applied
// each frame by the run loop, except set_sim which sets the model flag directly).
if (cfgSim    !== null) C.setSim(cfgSim);
if (cfgBypass !== null) C.setBypass(cfgBypass);
if (battAdc    !== null && C.setBattery) C.setBattery(battAdc);
if (chargerAdc !== null && C.setCharger) C.setCharger(chargerAdc);
if (cfgSpike  !== null) C.setSpike(cfgSpike);
if (cfgFaid   !== null) C.setSeclock(cfgFaid);
// Auto-recover OFF by default for headless — see the cfgRecover declaration above.
if (C.setRecover) C.setRecover(cfgRecover);
// Phase 6: arm the leak-tracker / used-bytes curve at boot if requested.
const armInstruments = () => {
  if (leakTrace && C.leakOn)      { C.leakOn(1);      console.log('LEAKTRACE armed — outstanding heap blocks attributed by caller_LR'); }
  if (heapCurve && C.heapcurveOn) { C.heapcurveOn(1); console.log('HEAPCURVE armed — sampling [0x104844+8] used-bytes'); }
  if (stubPcs.length && C.stubAdd) { stubPcs.forEach(p => C.stubAdd(p)); console.log('TASK-STUB armed — force-return at: ' + stubPcs.map(p => '0x' + p.toString(16)).join(',')); }
  if (wrwatchAddr && C.wrwatchOn) { C.wrwatchOn(wrwatchAddr); console.log('WRWATCH armed — writer-PC aggregation on 0x' + wrwatchAddr.toString(16)); }
  if (branchRing && C.branchOn)   { if (C.branchArm) C.branchArm(branchArm); C.branchOn(1);
    if (C.sendlogOn) C.sendlogOn(1);   // feed the wedge snapshot's last-N task_send/recv
    console.log(`BRANCHRING armed — typed branch ring (heavy decode from step ${branchArm})`); }
};
armInstruments();
console.log(`config: sim=${cfgSim??'def'} bypass=${cfgBypass??'def'} spike=${cfgSpike??'def'} faid=${cfgFaid??'def'}`);
// Overlay an EEPROM file at the partition base. A full-partition raw blob lands verbatim;
// a virgin partition image (native f0f0 framing, smaller than the window) is placed at its
// NAMED base — the 8-hex token in the filename that falls within [eeOff, eeOff+eeSize).
function overlayEeprom(path){
  const buf = new Uint8Array(readFileSync(path));
  const base = C.eeOff(), size = C.eeSize();
  let dst = base;
  if (buf[0] === 0xf0 && buf[1] === 0xf0 && buf.length < size) {
    for (const t of (path.split(/[\\/]/).pop().match(/[0-9a-fA-F]{8}/g) || [])) {
      const v = parseInt(t,16)>>>0; if (v>=base && v<base+size) { dst = v; break; }
    }
  }
  M.HEAPU8.set(buf, C.ramPtr() + dst);
  console.log(`eeprom overlay: ${buf.length}B @0x${dst.toString(16)} (base 0x${base.toString(16)}, win 0x${size.toString(16)})`);
}
if (eeprom) { overlayEeprom(eeprom); }

const W = C.lcdW() || 84, H = C.lcdH() || 48;
function render(name) {
  const fbp = C.fb(); const px = new Uint8Array(W * H);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const byte = M.HEAPU8[fbp + (y >> 3) * W + x]; px[y * W + x] = ((byte >> (y & 7)) & 1) ? 0 : 255;
  }
  const pgm = `${outDir}/${name}.pgm`;
  writeFileSync(pgm, Buffer.concat([Buffer.from(`P5\n${W} ${H}\n255\n`, 'ascii'), Buffer.from(px)]));
  try { execSync(`convert ${pgm} -scale 400% ${outDir}/${name}.png`, { stdio: 'ignore' }); } catch (e) {}
}

// FNV-1a 32-bit checksum over the framebuffer bytes (packed 1bpp, W*H/8 bytes).
// The redraw signal for the responsiveness probe — detects ANY pixel change
// between two presses. NON-cryptographic (collision-resistance not required;
// it only needs to flag a screen change). (Phase 5)
function fbHash() {
  const fbp = C.fb();
  let h = 0x811c9dc5 >>> 0;
  const nbytes = (W * H / 8) | 0;
  for (let i = 0; i < nbytes; i++) { h ^= M.HEAPU8[fbp + i]; h = Math.imul(h, 0x01000193) >>> 0; }
  return h >>> 0;
}

// Per-press responsiveness measurement (Phase 5). Snapshot the two probe watch
// counters (slot0=KEY_DOWN dispatch 0x2D84F4, slot1=TASK_SEND_IRQ 0x2997B0) + the
// fb hash BEFORE, enqueue ONE atomic `key` tap (down only — the wasm sequencer
// owns the up-edge; never enqueue a manual release/up-edge from JS), advance the
// settle window in cycle-paced chunks, then read deltas + fb-change AFTER.
// A press "responded" iff
// KEY_DOWN-delta>0 AND the framebuffer changed (the CONTEXT metric). Renders a PNG
// and returns a row tagged with BOTH step and cyc.
function measurePress(label, key, settleCycWin) {
  const before = [C.dbgCount(0), C.dbgCount(1)];
  const fbBefore = fbHash();
  C.key(KEYS[key][0], KEYS[key][1], 1);            // enqueue ONE atomic tap (down only)
  let adv = 0; while (adv < settleCycWin) { C.runCyc(216667); adv += 216667; }
  const dKey  = C.dbgCount(0) - before[0];
  const dSend = C.dbgCount(1) - before[1];
  const fbChanged = fbHash() !== fbBefore;
  // Two distinct signals:
  //  - dispatched: the press reached KEY_DOWN dispatch (0x2D84F4 delta > 0). The
  //    meaningful signal for the periodic keep-awake `c` press (idempotent at home
  //    → it reaches dispatch but does NOT change the screen, so fbΔ is expected no).
  //  - responded: dispatched AND the screen redrew. The CONTEXT verdict metric for
  //    the discriminating `menu` press (home → menu is a visible change).
  const dispatched = dKey > 0;
  const responded = dKey > 0 && fbChanged;
  render(label);
  return { label, key, step: C.step() >>> 0, cyc: C.cycles() >>> 0, dKey, dSend, fbChanged, dispatched, responded };
}

// Run `frames` cycle-paced frames, sampling PC each frame; returns the dominant PC +
// its share, plus per-watch hit deltas this step (exact, from the firmware counters).
function runWatched(frames) {
  const hist = {};
  for (let i = 0; i < frames; i++) {
    C.runCyc(216667);
    const p = (C.pc() >>> 0).toString(16); hist[p] = (hist[p] || 0) + 1;
  }
  const top = Object.entries(hist).sort((a, b) => b[1] - a[1])[0] || ['0', 0];
  const wdelta = watch.map((_, i) => { const c = C.dbgCount(i), d = c - watchPrev[i]; watchPrev[i] = c; return d; });
  return { pc: top[0], share: top[1] / frames, hist, wdelta };
}

const SPIN_PCS = new Set(['2eebee', '2eebec', '2ef564']);
function step(label, fn, frames) {
  if (fn) fn();
  const r = runWatched(frames);
  render(label);
  // Heuristic flag: a known spin PC, OR one PC dominating a window with ENOUGH samples to
  // mean something. The bare share test false-positives on bounded idle loops sampled over
  // just a frame or two (e.g. the find-first-set MMI loop 0x2d633c during menu nav).
  const spin = SPIN_PCS.has(r.pc) || (frames >= 8 && r.share > 0.9);
  const top3 = Object.entries(r.hist).sort((a,b)=>b[1]-a[1]).slice(0,3).map(([p,c])=>`0x${p}:${(c/frames*100|0)}%`).join(' ');
  const wstr = watch.length ? '  watch[' + watch.map((pc,i)=>`0x${pc.toString(16)}:+${r.wdelta[i]}`).join(' ') + ']' : '';
  const rr = C.resetReq();
  // Always poll for catches first — a RECOVERED reset never raises reset_request so the
  // existing rr-gated path would miss it. pollResetEvents prints one report per new catch.
  pollResetEvents(`key '${label}'`);
  console.log(`${label.padEnd(16)} lcd=${String(C.lcdw()).padStart(6)} eeProg=${String(C.eeProg()).padStart(5)} pc=0x${(C.pc()>>>0).toString(16)} off=${C.off()} ${rr?'⟳RESET':spin?'⚠SPIN':'      '} [${top3}]${wstr}`);
  if (spin) console.log(backtrace());     // deep diag on a detected spin/crash
  // Honour a firmware reset request (e.g. end of a factory reset): warm-reboot (keep
  // flash+EEPROM, clear RAM, re-enter boot) and re-render the post-reboot screen.
  if (rr) {
    C.warmReset(); runWatched(settle); render(label + '_reboot'); watchPrev = watch.map((_, i) => C.dbgCount(i));
    console.log(`${'  ↳ warm reboot'.padEnd(16)} lcd=${String(C.lcdw()).padStart(6)} pc=0x${(C.pc()>>>0).toString(16)} (post-reset screen: ${label}_reboot.png)`); }
  return spin;
}

console.log(`nav: ${fw.split('/').pop()}${eeprom ? ' + EEPROM ' + eeprom.split('/').pop() : ''}  -> ${outDir}`);

// ─────────────────────────────────────────────────────────────────────────────
// Wave-0 self-checks (Task 1): cadence-coordinate monotonicity + control-press
// settle calibration. Run with --check-wave0; prints verdicts and exits 0/non-0.
// This is the evidence that the sweep keys cadence on a proven-monotonic
// coordinate and measures with a settle window proven to register a known-good
// press, BEFORE Task 2 builds the sweep loop. (Phase 5)
// ─────────────────────────────────────────────────────────────────────────────
if (checkWave0) {
  // Arm the two probe watch slots (slot0=KEY_DOWN dispatch, slot1=TASK_SEND_IRQ).
  C.dbgWatch(0, 0x2D84F4);
  C.dbgWatch(1, 0x2997B0);
  // Boot settle so the control press lands at a steady home screen.
  for (let i = 0; i < settle; i++) C.runCyc(216667);
  render('wave0_boot');

  // (c) Settle-window calibration FIRST (while genuinely fresh-boot) — a fresh-boot
  //     control press measured over the configured settle window MUST read
  //     KEY_DOWN-delta>0 AND a framebuffer change. Calibrate against `menu` (the
  //     end-probe key) which visibly changes the screen at home (home → menu).
  const calKey = 'menu';
  const before = [C.dbgCount(0), C.dbgCount(1)];
  const fbBefore = fbHash();
  C.key(KEYS[calKey][0], KEYS[calKey][1], 1);   // enqueue ONE atomic tap (down only)
  let adv = 0; while (adv < settleCyc) { C.runCyc(216667); adv += 216667; }
  const dKey = C.dbgCount(0) - before[0];
  const dSend = C.dbgCount(1) - before[1];
  const fbChanged = fbHash() !== fbBefore;
  render('wave0_control');
  const responded = dKey > 0 && fbChanged;
  console.log(`\n=== WAVE-0 CONTROL PRESS (fresh boot, key=${calKey}, settle=${settleCyc} cyc) ===`);
  console.log(`  KEY_DOWN(0x2D84F4) +${dKey}   TASK_SEND_IRQ(0x2997B0) +${dSend}   fbΔ=${fbChanged ? 'yes' : 'no'}`);
  console.log(`  control press: ${responded ? 'RESPONSIVE' : 'UNRESPONSIVE'}  (settle ${responded ? 'sufficient' : 'TOO SHORT — widen --settle-cyc'})`);
  console.log(`  CALIBRATED SETTLE default = ${settleCyc} cyc  [Task 2 default]`);

  // (b) Monotonicity probe — advance ≥0x20000000 (537M) cyc in coarse batches and
  //     assert C.cycles() and C.step() are each non-decreasing every batch.
  const MONO_TARGET = 0x20000000;       // 537M cyc — one rebase period of the raw counter
  const MONO_BATCH  = 10_000_000;
  let prevCyc = C.cycles() >>> 0, prevStep = C.step() >>> 0;
  let cycMono = true, stepMono = true, cycStart = prevCyc, batches = 0;
  while ((C.cycles() >>> 0) - cycStart < MONO_TARGET) {
    C.runCyc(MONO_BATCH);
    const nowCyc = C.cycles() >>> 0, nowStep = C.step() >>> 0;
    if (nowCyc < prevCyc)  cycMono  = false;
    if (nowStep < prevStep) stepMono = false;
    prevCyc = nowCyc; prevStep = nowStep; batches++;
  }
  console.log(`\n=== WAVE-0 MONOTONICITY (advanced ${batches} batches, ~${((C.cycles()-cycStart)/1e6).toFixed(0)}M cyc) ===`);
  console.log(`  C.cycles(): ${cycMono ? 'MONOTONIC (non-decreasing)' : 'NON-MONOTONIC (rebases) — DO NOT use for cadence'}`);
  console.log(`  C.step()  : ${stepMono ? 'MONOTONIC (non-decreasing)' : 'NON-MONOTONIC — unexpected'}`);
  // Decision: prefer C.step() unconditionally (instruction count, structurally
  // monotonic; the raw cycle counter is documented to rebase at 0x20000000 — a
  // single-period probe cannot exclude a rebase at a later multiple).
  const coord = stepMono ? 'C.step()' : (cycMono ? 'C.cycles()' : 'NONE');
  console.log(`  CADENCE COORDINATE (chosen) = ${coord}  [Task 2 keys boundaries on this]`);

  const ok = (coord !== 'NONE') && responded;
  console.log(`\nWAVE0-VERDICT: ${ok ? 'PASS' : 'FAIL'} — cadence=${coord}, control=${responded ? 'RESPONSIVE' : 'UNRESPONSIVE'}`);
  process.exit(ok ? 0 : 4);
}

// ─────────────────────────────────────────────────────────────────────────────
// RESPONSIVENESS PROBE + IDLE-SLEEP A/B (Phase 5, Task 2).
//   Arm A (no-press baseline): idle undisturbed to --end-probe-at depth, then a
//     single `menu` end-probe → expected UNRESPONSIVE (end screen = home).
//   Arm B (press `--press-key` every --press-every steps): keep-awake presses
//     throughout, then a `menu` end-probe → does it respond (end screen = menu)?
//   Both arms keep-alive ON (no DSPNOKEEPALIVE — that's Phase 6). --arm both runs
//     A then B as two SEQUENTIAL boots in one process (shared config verbatim;
//     A and B cannot share one idle descent — Open Question 2).
//   Cadence keys on C.step() (Wave-0 decision). Each press tagged with step+cyc.
// ─────────────────────────────────────────────────────────────────────────────
if (arm !== null) {
  // (Re)boot + apply the parsed config. Called once per arm so A and B get
  // identical, fresh idle descents. Re-loads the fw image (C.boot re-inits the
  // core); arms the two probe watch slots.
  const bootArm = () => {
    try { M.FS.unlink('/fw.fls'); } catch (e) {}
    M.FS.writeFile('/fw.fls', new Uint8Array(readFileSync(fw)));
    C.boot();
    FLASH_HI = (C.flashHi && C.flashHi() >>> 0) || 0x400000;   // model-aware code ceiling
    // Re-baseline the module-level reset counters so pollResetEvents starts
    // fresh for this arm. C.boot() re-inits the firmware core and resets
    // C.resetTotal() back to 0; without this, arm B would compare against arm
    // A's accumulated count and miss early resets (WR-01).
    resetTotalSeen     = C.resetTotal();      // = 0 after C.boot()
    resetRecoveredSeen = C.resetRecovered();  // = 0
    if (cfgSim    !== null) C.setSim(cfgSim);
    if (cfgBypass !== null) C.setBypass(cfgBypass);
    if (cfgSpike  !== null) C.setSpike(cfgSpike);
    if (cfgFaid   !== null) C.setSeclock(cfgFaid);
    if (C.setRecover) C.setRecover(cfgRecover);
    if (eeprom) { overlayEeprom(eeprom); }
    C.dbgWatch(0, 0x2D84F4);   // slot 0 = KEY_DOWN dispatch (raw watch this phase)
    C.dbgWatch(1, 0x2997B0);   // slot 1 = TASK_SEND_IRQ (cascade)
    armInstruments();          // Phase 6: re-arm leak-tracker/curve after the re-boot
    for (let i = 0; i < settle; i++) C.runCyc(216667);   // boot settle to home screen
  };

  // Run one arm. armId ∈ {'A','B'}. Returns { control, rows, healthy, healthMsg, endProbe }.
  const runArm = (armId) => {
    console.log(`\n──────── ARM ${armId} ${armId === 'A' ? '(no-press baseline)' : `(press ${pressKey} every ${(pressEvery/1e6).toFixed(0)}M steps)`} ────────`);
    bootArm();
    render(`arm${armId}_00_boot`);

    // Control press (MANDATORY, SC#2): a fresh-boot `menu` press MUST register, or
    // the probe is broken (not the firmware). Abort loudly so "all unresponsive"
    // is un-fakeable.
    const control = measurePress(`arm${armId}_control`, 'menu', settleCyc);
    console.log(`  control press: KEY_DOWN+${control.dKey} TASK_SEND+${control.dSend} fbΔ=${control.fbChanged?'yes':'no'} → ${control.responded?'RESPONSIVE':'UNRESPONSIVE'}`);
    if (!control.responded) {
      console.error(`PROBE BROKEN (arm ${armId}): fresh-boot control press did not register ` +
                    `(KEY_DOWN delta=${control.dKey}, fbΔ=${control.fbChanged}). Widen --settle-cyc or check 0x2D84F4 watch arming.`);
      process.exit(3);
    }
    // After the control press the UI is on the menu; press `menu`-back/`c` is not
    // needed — re-boot semantics keep arms independent, but WITHIN an arm we want
    // the idle descent to start from the home screen. Press `c` (idempotent back)
    // to return home before the undisturbed descent.
    C.key(KEYS['c'][0], KEYS['c'][1], 1);
    { let a = 0; while (a < settleCyc) { C.runCyc(216667); a += 216667; } }
    render(`arm${armId}_01_home`);

    // Coarse-batch idle sweep (cadence on C.step()). Arm A fires NO periodic
    // presses (undisturbed idle); arm B fires a measurePress each time step crosses
    // the next --press-every boundary. Baseline health asserted via pollResetEvents
    // + fb-stability (NOT the spin heuristic).
    const rows = [];
    let healthy = true, healthMsg = 'no reset; fb periodic/stable';
    let nextPress = pressFrom;
    let lastFbHash = fbHash(), fbSamples = 0, fbDistinct = new Set([lastFbHash]);
    while ((C.step() >>> 0) < endProbeAt) {
      C.runCyc(pressBatch);
      // Baseline health: any firmware self-reset during the (arm A) idle invalidates
      // the "no response = input" attribution.
      if (pollResetEvents(`arm${armId}-sweep`) > 0) {
        healthy = false; healthMsg = `firmware reset caught during idle (${resetSeen ? resetSeen.name : '?'})`;
        console.log(`  ⚠ ARM ${armId} baseline NOT healthy: ${healthMsg}`);
        break;
      }
      // fb-stability sampling (coarse) — a healthy idle screen is stable or periodic
      // (e.g. a blinking clock), NOT churning wildly. Track distinct fb hashes.
      const h = fbHash(); if (h !== lastFbHash) { fbDistinct.add(h); lastFbHash = h; } fbSamples++;
      if (armId === 'B' && (C.step() >>> 0) >= nextPress) {
        const stepB = (C.step() / 1e9).toFixed(2);
        const row = measurePress(`armB_press_${stepB}B`, pressKey, settleCyc);
        rows.push(row);
        console.log(`  press[${pressKey}] @step=${stepB}B cyc=${(row.cyc/1e9).toFixed(2)}B  KEY_DOWN+${row.dKey} TASK_SEND+${row.dSend} fbΔ=${row.fbChanged?'yes':'no'} → ${row.dispatched?'DISPATCHED':'NOT-DISPATCHED'}`);
        nextPress += pressEvery;
      }
    }
    // Baseline-health verdict line (arm A is the one that matters; arm B reported
    // too for symmetry). A "stable/periodic" fb has a small number of distinct
    // frames relative to samples; a churning fb (>~50% distinct) is suspicious.
    if (healthy) {
      const churn = fbSamples ? fbDistinct.size / fbSamples : 0;
      healthMsg = `no reset; fb ${fbDistinct.size} distinct / ${fbSamples} samples (${(churn*100).toFixed(0)}% churn)`;
    }
    console.log(`  ARM ${armId} baseline idle: ${healthy ? 'HEALTHY' : 'UNHEALTHY'} — ${healthMsg}`);

    // END-PROBE: the discriminating `menu` press at depth (both arms).
    const endProbe = measurePress(`arm${armId}_end_menu`, 'menu', settleCyc);
    console.log(`  END-PROBE @step=${(endProbe.step/1e9).toFixed(2)}B  KEY_DOWN+${endProbe.dKey} TASK_SEND+${endProbe.dSend} fbΔ=${endProbe.fbChanged?'yes':'no'} → ${endProbe.responded?'RESPONSIVE':'UNRESPONSIVE'}`);
    return { armId, control, rows, healthy, healthMsg, endProbe };
  };

  const results = {};
  if (arm === 'a' || arm === 'both') results.A = runArm('A');
  if (arm === 'b' || arm === 'both') results.B = runArm('B');

  // ── ONSET (no-press / arm A): the undisturbed-idle depth beyond which the end
  //    press stops registering. For a single-end-probe arm A, the onset is
  //    bracketed by [0, end-probe-at] when the end press is UNRESPONSIVE, or
  //    "> end-probe-at" when it still responded. The fine pass (Task 3) narrows it.
  //    For arm B, the first UNRESPONSIVE after a run of RESPONSIVE (if any).
  const lines = [];
  const cfgLine = `config: sim=${cfgSim} bypass=${cfgBypass} faid=${cfgFaid} spike=${cfgSpike} recover=${cfgRecover}  keep-alive=ON`;
  lines.push(`=== RESPONSIVENESS PROBE — fw ${fw.split('/').pop()} (keep-alive ON) ===`);
  lines.push(cfgLine);
  lines.push(`cadence-coordinate=C.step()  press-every=${pressEvery} press-from=${pressFrom} press-batch=${pressBatch}cyc end-probe-at=${endProbeAt} settle=${settleCyc}cyc`);
  lines.push('');

  let onsetLine = 'ONSET: n/a (no no-press arm in this run)';
  if (results.A) {
    const a = results.A;
    lines.push(`ARM A (no-press baseline):`);
    lines.push(`  control press: KEY_DOWN+${a.control.dKey} TASK_SEND+${a.control.dSend} fbΔ=${a.control.fbChanged?'yes':'no'} → RESPONSIVE  (probe self-check PASS)`);
    lines.push(`  baseline idle: ${a.healthy ? 'HEALTHY' : 'UNHEALTHY'} — ${a.healthMsg}`);
    lines.push(`  END-PROBE @step=${(a.endProbe.step/1e9).toFixed(3)}B cyc=${(a.endProbe.cyc/1e9).toFixed(3)}B  KEY_DOWN+${a.endProbe.dKey} TASK_SEND+${a.endProbe.dSend} fbΔ=${a.endProbe.fbChanged?'yes':'no'} → ${a.endProbe.responded?'RESPONSIVE':'UNRESPONSIVE'}`);
    lines.push(`  end screenshot: armA_end_menu.png`);
    lines.push('');
    onsetLine = a.endProbe.responded
      ? `ONSET (no-press arm): end press at step=${(a.endProbe.step/1e9).toFixed(3)}B still RESPONSIVE → onset > ${(endProbeAt/1e9).toFixed(2)}B (not reached)`
      : `ONSET (no-press arm): end press at step=${(a.endProbe.step/1e9).toFixed(3)}B UNRESPONSIVE → onset bracketed in [0, ${(a.endProbe.step/1e9).toFixed(3)}B]; fine pass narrows`;
  }
  if (results.B) {
    const b = results.B;
    lines.push(`ARM B (press ${pressKey} every ${(pressEvery/1e6).toFixed(0)}M steps):`);
    lines.push(`  control press: KEY_DOWN+${b.control.dKey} TASK_SEND+${b.control.dSend} fbΔ=${b.control.fbChanged?'yes':'no'} → RESPONSIVE  (probe self-check PASS)`);
    lines.push(`  (periodic ${pressKey} is idempotent at home — DISPATCHED = reached KEY_DOWN; fbΔ expected no)`);
    for (const r of b.rows)
      lines.push(`  press[${pressKey}] @step=${(r.step/1e9).toFixed(3)}B cyc=${(r.cyc/1e9).toFixed(3)}B  KEY_DOWN+${r.dKey} TASK_SEND+${r.dSend} fbΔ=${r.fbChanged?'yes':'no'} → ${r.dispatched?'DISPATCHED':'NOT-DISPATCHED'}`);
    lines.push(`  baseline (B) idle: ${b.healthy ? 'HEALTHY' : 'UNHEALTHY'} — ${b.healthMsg}`);
    lines.push(`  END-PROBE @step=${(b.endProbe.step/1e9).toFixed(3)}B cyc=${(b.endProbe.cyc/1e9).toFixed(3)}B  KEY_DOWN+${b.endProbe.dKey} TASK_SEND+${b.endProbe.dSend} fbΔ=${b.endProbe.fbChanged?'yes':'no'} → ${b.endProbe.responded?'RESPONSIVE':'UNRESPONSIVE'}`);
    lines.push(`  end screenshot: armB_end_menu.png`);
    lines.push('');
    // Arm B onset (first NOT-DISPATCHED after a DISPATCHED run), if observed — the
    // depth at which the periodic keep-awake press itself stops reaching dispatch.
    let firstUnresp = -1, sawResp = false;
    for (const r of b.rows) { if (r.dispatched) sawResp = true; else if (sawResp && firstUnresp < 0) firstUnresp = r.step; }
    if (firstUnresp >= 0) onsetLine += `\nONSET (arm B periodic): first NOT-DISPATCHED after DISPATCHED at step=${(firstUnresp/1e9).toFixed(3)}B`;
  }
  lines.push(onsetLine);

  // ── PROBE-VERDICT (exactly one line). IN when B's end press responds where A's
  //    does not; OUT when B's end press fails at the same depth as A (input does
  //    not prevent the descent → input-independent). When only one arm ran, the
  //    verdict is reported as INCONCLUSIVE (needs both arms).
  let verdict;
  if (results.A && results.B) {
    if (results.B.endProbe.responded && !results.A.endProbe.responded)
      verdict = `PROBE-VERDICT: idle-sleep causation IN   (B end-probe RESPONSIVE [menu] where A end-probe UNRESPONSIVE [home])`;
    else if (!results.B.endProbe.responded && !results.A.endProbe.responded)
      verdict = `PROBE-VERDICT: idle-sleep causation OUT  (B end-probe fails at same depth as A; input-independent — Phase 6 chases a time/state descent)`;
    else if (results.A.endProbe.responded)
      verdict = `PROBE-VERDICT: inconclusive — A end-probe RESPONSIVE (idle depth ${(endProbeAt/1e9).toFixed(2)}B not deep enough to reproduce the gap; increase --end-probe-at)`;
    else
      throw new Error('PROBE-VERDICT: unreachable state — logic error (B responded && A responded should be caught by the inconclusive branch)');
  } else {
    verdict = `PROBE-VERDICT: single-arm run (--arm ${arm}) — both arms required for the A/B causation verdict; re-run with --arm both`;
  }
  lines.push(verdict);

  const summaryTxt = lines.join('\n') + '\n';
  writeFileSync(`${outDir}/summary.txt`, summaryTxt);
  console.log('\n' + summaryTxt);
  console.log(`probe done — summary + screens in ${outDir}`);
  process.exit(0);
}

step('00_boot', null, settle);                 // initial screen after boot settle
let n = 1;

if (replay) {
  // Cycle-paced replay matching the web build (REPLAY_CYC_PER_FRAME = 10M). Events prefer
  // cyc (deterministic, structured bundles) and fall back to step (legacy flat arrays).
  // Last-mile clamp lands exactly on the event's cycle target to avoid overshoot.
  const REPLAY_CYC_BATCH = 10_000_000;
  const evDue = (ev) => Number.isFinite(ev.cyc) ? C.cycles() >= ev.cyc : C.step() >= ev.step;
  let replayHaltedByReset = false;   // set when the advance loop catches an unrecovered reset
  const advanceTo = (targetFn) => {
    const hist = {}; let guard = 200000;
    while (!targetFn() && guard-- > 0) {
      // Unrecovered firmware reset surfaces as reset_request. Break here so the verdict
      // logs the bug at the cycle it actually fired, not after spinning to context.cyc.
      // (Same role as checkSpin in the web build — single canonical detection.)
      if (C.resetReq && C.resetReq()) { replayHaltedByReset = true; break; }
      let batch = REPLAY_CYC_BATCH;
      if (typeof targetFn.cycTarget === 'number') {
        const rem = Math.max(0, targetFn.cycTarget - C.cycles());
        if (rem > 0 && rem < REPLAY_CYC_BATCH) batch = rem;
      }
      if (batch <= 0) break;
      C.runCyc(batch);
      const p = (C.pc() >>> 0).toString(16); hist[p] = (hist[p] || 0) + 1;
    }
    return hist;
  };
  console.log(`replay: ${replay.length} ${replay[0] && Number.isFinite(replay[0].cyc) ? 'cyc' : 'step'}-keyed events`);
  for (const ev of replay) {
    const fn = () => evDue(ev);
    if (Number.isFinite(ev.cyc)) fn.cycTarget = ev.cyc;
    const hist = advanceTo(fn);
    if (replayHaltedByReset) break;   // unrecovered reset fired before reaching this event
    const label = String(n).padStart(2, '0') + '_' + String(ev.key).replace('*', 'star').replace('#', 'hash');
    render(label);
    const tot = Object.values(hist).reduce((a, b) => a + b, 0) || 1;
    const top = Object.entries(hist).sort((a, b) => b[1] - a[1])[0] || ['0', 0];
    const spin = SPIN_PCS.has(top[0]) || (tot >= 8 && top[1] / tot > 0.9);   // need real samples
    console.log(`${label.padEnd(16)} step=${(C.step() / 1e6).toFixed(1)}M cyc=${(C.cycles() / 1e6).toFixed(1)}M lcd=${C.lcdw()} pc=0x${(C.pc() >>> 0).toString(16)} ${spin ? '⚠SPIN' : '     '} → press ${ev.key}`);
    if (spin) console.log(backtrace());
    if (ev.key === 'pwr') { C.power(1); C.power(0); }
    else if (ev.key === 'off') C.power(1);
    else if (KEYS[ev.key]) C.key(KEYS[ev.key][0], KEYS[ev.key][1], 1);
    n++;
  }
  // After the queued events, structured bundles can carry a context target (cyc + pc).
  // Advance to it and verdict-check PC match — UNLESS an unrecovered reset already fired
  // mid-replay, in which case we stop at the reset and report the bug from there.
  if (replayHaltedByReset) {
    const curPc  = '0x' + (C.pc() >>> 0).toString(16);
    const reason = C.resetLastReason ? C.resetLastReason() : 0;
    const want   = String((replayContext && replayContext.pc) || '').toLowerCase();
    const matched = want && curPc.toLowerCase() === want;
    console.log(`RESET-VERDICT: replay halted on unrecovered firmware reset (reason ${reason}) at cyc ${(C.cycles()/1e6).toFixed(2)}M.`);
    if (want) console.log(`  ${matched ? 'PC MATCH ✓' : 'PC differs'}: replay ${curPc} vs original ${want}`);
    render(String(n++).padStart(2, '0') + '_at_reset');
  } else if (replayContext && Number.isFinite(replayContext.cyc) && replayContext.cyc > C.cycles()) {
    console.log(`replay: events done at cyc ${(C.cycles()/1e6).toFixed(2)}M — advancing to context cyc ${(replayContext.cyc/1e6).toFixed(2)}M`);
    const fn = () => C.cycles() >= replayContext.cyc;
    fn.cycTarget = replayContext.cyc;
    advanceTo(fn);
    const curPc = '0x' + (C.pc() >>> 0).toString(16);
    const want  = String(replayContext.pc || '').toLowerCase();
    const matched = want && curPc.toLowerCase() === want;
    if (replayHaltedByReset) {
      const reason = C.resetLastReason ? C.resetLastReason() : 0;
      console.log(`RESET-VERDICT: replay halted on unrecovered firmware reset (reason ${reason}) at cyc ${(C.cycles()/1e6).toFixed(2)}M.`);
      if (want) console.log(`  ${matched ? 'PC MATCH ✓' : 'PC differs'}: replay ${curPc} vs original ${want}`);
    } else {
      console.log(`RESET-VERDICT: ${matched ? 'PC MATCH ✓' : 'PC differs'}: replay ${curPc} vs original ${want || '?'}  ` +
                  `(cyc Δ=${C.cycles() - replayContext.cyc}, step ${C.step()}` +
                  (Number.isFinite(replayContext.step) ? ` Δ=${C.step() - replayContext.step}` : '') + ')');
    }
    render(String(n++).padStart(2, '0') + '_at_context');
  }
  // After the logged presses, run IDLE watching for a delayed crash/spin — the crash can
  // land billions of steps after the last key (an idle timer / periodic task). Stop when
  // the PC drops into the low exception-vector/null range (a crash) or pins (a spin).
  render(String(n).padStart(2, '0') + '_afterkeys');
  const idleMax = idle || 60000;     // frames (~13B cyc) — plenty to reach a multi-minute crash
  console.log(`replay keys done at step ${(C.step() / 1e6).toFixed(1)}M — idle-watching for a delayed crash (up to ${idleMax} frames)…`);
  let hit = false, pinPc = 0, pinN = 0, snap = 0, lowFrames = 0, wildFrames = 0;
  const heapWin = []; let heapWarned = false;   // rate-track alloc failures (heap exhaustion)
  // Phase 6 Instrument 2: LCD-write-counter freeze trigger. Track lcd_data_writes over a
  // W=240-frame window; when armed AND past the onset (step > 200M) AND the delta over the
  // window is 0 (no UI redraw -> frozen), fire ONE branch-ring + wedge-snapshot dump.
  const LCD_FREEZE_W = 240;
  const lcdWin = []; let freezeDumped = false;
  const idleHist = {};   // PC histogram across the idle-watch (finds where a freeze spins)
  const dumpIdleHist = () => {
    const tot = Object.values(idleHist).reduce((a, b) => a + b, 0) || 1;
    const top = Object.entries(idleHist).sort((a, b) => b[1] - a[1]).slice(0, 12);
    console.log(`  idle PC histogram (${tot} samples, top 12 — where the CPU spent the freeze):`);
    for (const [p, cnt] of top) { const a = +p, f = (a >= FLASH_LO && a < FLASH_HI) ? fnStart(a) : 0;
      console.log(`    ${(cnt / tot * 100).toFixed(1).padStart(5)}%  0x${a.toString(16)}${f ? `  in fn 0x${f.toString(16)}` : ''}`); }
  };
  if (clockEvery) console.log(`clock-watch: a snapshot every ${clockEvery} frames (${(clockEvery/60).toFixed(1)} emulated min); mad2 RTC-MIN edges shown — if edges climb but the on-screen clock doesn't, the tick handler isn't running.`);
  for (let c = 0; c < idleMax && !hit; c++) {
    C.runCyc(216667);
    const pc = C.pc() >>> 0;
    idleHist[pc] = (idleHist[pc] || 0) + 1;
    if (pc === pinPc) pinN++; else { pinPc = pc; pinN = 1; }
    // Heap-exhaustion rate watch (non-fatal, reported once): a sustained stream of allocation
    // FAILURES (alloc returned NULL → parked on the memory semaphore) means the heap is
    // starved — usually the precursor to a heap-smash wild PC, which the OOB checks catch
    // below. We warn but keep running so that crash is still captured. Normal operation has
    // ZERO alloc failures, so any sustained burst is abnormal.
    const heapNow = C.heapFailCount();
    heapWin.push(heapNow); if (heapWin.length > 240) heapWin.shift();
    if (!heapWarned && heapWin.length >= 240 && heapNow - heapWin[0] >= 24) {
      heapWarned = true;
      const hlr = C.heapFailLr() >>> 0, f = fnStart(hlr & ~1);
      console.log(`⚠ HEAP EXHAUSTION at step ${(C.step()/1e6).toFixed(1)}M — ${heapNow} alloc failures total ` +
                  `(${heapNow - heapWin[0]} in the last ~240 idle frames), last caller LR=${hx(hlr)}${f?`  in fn ${hx(f)}`:''}`);
      // The heap-fail watch is the FIRST signal (the victim). Pull the leak
      // ATTRIBUTION (the leaker) + the used-bytes curve at the onset.
      dumpLeak('heap-exhaustion onset');
      dumpHeapCurve('heap-exhaustion onset');
    }
    // LCD-stall freeze trigger (Instrument 2): the framebuffer-write counter stalls when
    // the UI freezes. Once armed + past the onset, no lcd-write delta over W frames =>
    // wedged. Dump the branch ring + wedge snapshot ONCE (the dump shows where/how the
    // phone is stuck — flat call-depth in a tight PC band = spin; stuck-deep = blocked recv).
    if (branchRing && !freezeDumped) {
      const lw = C.lcdw() >>> 0;
      lcdWin.push(lw); if (lcdWin.length > LCD_FREEZE_W) lcdWin.shift();
      if (lcdWin.length >= LCD_FREEZE_W && C.step() > 200e6 && (lw - lcdWin[0]) === 0) {
        freezeDumped = true;
        console.log(`⚠ LCD-STALL FREEZE at step ${(C.step()/1e6).toFixed(1)}M — no framebuffer write ` +
                    `over the last ${LCD_FREEZE_W} idle frames (lcd_writes pinned at ${lw}); dumping branch ring + wedge snapshot.`);
        dumpWedge('LCD-stall freeze', idleHist);
      }
    }
    // Periodic clock snapshot: render the home screen + log mad2's RTC-MIN edge count
    // (the timer firing) so a frozen on-screen clock vs climbing edges is unambiguous.
    if (clockEvery && c % clockEvery === clockEvery - 1) {
      const lbl = String(n).padStart(2, '0') + '_clock' + String(snap).padStart(2, '0');
      render(lbl); snap++;
      const wc = watch.length ? '  watch[' + watch.map((p,i)=>`0x${p.toString(16)}:${C.dbgCount(i)}`).join(' ') + ']' : '';
      const ccm = C.ccMask(), ccl = C.ccLines();
      const hf = C.heapFailCount();
      console.log(`  ${lbl}  emuMin≈${(C.cycles()/13e6/60).toFixed(2)}  RTC-MIN-edges=${C.rtcMin()}  CCmask=0x${(ccm&0xff).toString(16)}(INT5 ${ccm&0x20?'MASKED':'enabled'})  CClines=0x${(ccl&0xff).toString(16)}${hf?`  heapFails=${hf}`:''}  pc=0x${pc.toString(16)}${wc}`);
    }
    // Out-of-bounds PC. A SINGLE low-PC sample is NOT a crash: nav reads raw gprs[15], and a
    // normal exception entry parks it at a vector for one instruction (IRQ→0x1c, FIQ→0x20,
    // since ARMWritePC sets gprs[15]=vector+4). A real crash STAYS low (march/spin), so the
    // LOW out-of-bounds case must PERSIST a few frames. A HIGH wild PC (≥ FLASH_HI) is never a
    // legit transient — exceptions only vector LOW, no code is mapped that high — so a corrupt
    // PC there (e.g. the 0xfd…… heap-smash runaway) is fatal: catch it on sight. (The core
    // masks fetches to 16 MiB so the CPU limps through zero-fill, which is exactly why a
    // low-PC-only watch missed it.)
    lowFrames  = (pc >= 0x20 && pc < 0x100000) ? lowFrames + 1 : 0;
    wildFrames = (pc >= FLASH_HI)              ? wildFrames + 1 : 0;
    if (wildFrames >= 2 || lowFrames >= 3 || C.resetReq() || pinN > 4000) {
      hit = true;
      render(String(n).padStart(2, '0') + '_CRASH');
      const isReset = C.resetReq();
      const kind = isReset ? 'RESET' : wildFrames >= 2 ? 'WILD-PC' : lowFrames >= 3 ? 'WILD-PC(low)' : 'SPIN';
      console.log(`⚠ ${kind} at step ${(C.step() / 1e6).toFixed(1)}M  pc=0x${pc.toString(16)}  (after ${c} idle frames, wildFrames=${wildFrames} lowFrames=${lowFrames} pinN=${pinN})`);
      if (isReset) reportReset('idle-watch', /*recovered=*/false);   // decode reboot reason + backtrace
      else { pollResetEvents('idle-watch'); console.log(backtrace()); const ring = pcRingDump(); if (ring) console.log(ring); }
      dumpIdleHist();   // where the CPU spent the freeze leading up to the reset/spin/wild PC
    }
    if (!clockEvery && c % 5000 === 4999) console.log(`  …idle step ${(C.step() / 1e6).toFixed(0)}M pc=0x${pc.toString(16)}`);
  }
  if (!hit) console.log(`no crash in ${idleMax} idle frames (reached step ${(C.step() / 1e6).toFixed(1)}M)`);
  console.log(`replay done — screens in ${outDir}`);
} else
for (const tok of keys) {
  // per-key timing: "key@frames" runs <frames> after that key (else the default --per).
  // Menu shortcuts (menu→digit) need a SHORT gap (the input-timeout window, shown by the
  // three dots top-right); the keypress itself plays atomically so a short gap won't drop it.
  const at = tok.indexOf('@');
  const k = at >= 0 ? tok.slice(0, at) : tok;
  const f = at >= 0 ? (+tok.slice(at + 1) || per) : per;
  const label = String(n).padStart(2, '0') + '_' + k.replace('*','star').replace('#','hash') + (at>=0?`@${f}`:'');
  if (k === 'wait') step(label, null, f);
  else if (k === 'reboot') { C.warmReset(); step(label, null, f); }   // force a warm reboot (keep flash+EEPROM)
  else if (k === 'pwr') { step(label, () => { C.power(1); }, f); C.power(0); }
  else if (k === 'off') step(label, () => { C.power(1); }, f * 3);   // hold for shutdown
  else if (KEYS[k]) step(label, () => C.key(KEYS[k][0], KEYS[k][1], 1), f);
  else { console.log(`  (unknown key '${k}' — skipped)`); continue; }
  n++;
}
console.log(`done — ${n} screens in ${outDir} (NN_<key>.png)`);
// Greppable run summary: surface any firmware reset so it's easy to find in long output.
// Final flush in case a recovered reset landed between the last step and end-of-run.
pollResetEvents('end-of-run');
if (resetSeen) {
  const verdict = resetSeen.recovered ? 'RECOVERED' : 'RESET';
  const total = C.resetTotal(), rec = C.resetRecovered();
  console.log(`RESET-SUMMARY: ${verdict} ${resetSeen.name} (reason ${resetSeen.reason}), faulting LR=${hx(resetSeen.lr)}  [total=${total} recovered=${rec}]`);
}
// Phase 6: end-of-run leak attribution + used-bytes curve. The dump is the
// *attribution* (who leaked); the ⚠ HEAP EXHAUSTION watch above is the *first*
// signal. Both print to console AND append to summary.txt for offline RCA.
dumpLeak('end-of-run');
dumpHeapCurve('end-of-run');
if (stubPcs.length && C.stubHits) {
  const hits = stubPcs.map((p, i) => `0x${p.toString(16)}:${C.stubHits(i) >>> 0}`).join('  ');
  console.log(`TASK-STUB hits — ${hits}  (0 = stub never fired; >0 = handler was force-returned)`);
}
if (wrwatchAddr && C.wrwatchNpc) {
  const n = C.wrwatchNpc() >>> 0;
  const rows = [];
  for (let i = 0; i < n; i++) rows.push({ pc: C.wrwatchPc(i) >>> 0, net: C.wrwatchNet(i) | 0, cnt: C.wrwatchCnt(i) >>> 0, lr: C.wrwatchLr(i) >>> 0 });
  rows.sort((a, b) => b.net - a.net);
  console.log(`WRWATCH 0x${wrwatchAddr.toString(16)} — ${n} distinct writer PCs (sorted by net delta; +net grows the counter):`);
  for (const r of rows) console.log(`  pc=0x${r.pc.toString(16).padStart(6,'0')}  net=${r.net >= 0 ? '+' : ''}${r.net}  count=${r.cnt}  egLR=0x${r.lr.toString(16)}`);
  if (C.wrwatchSzn) {
    const sn = C.wrwatchSzn() >>> 0, srows = [];
    for (let i = 0; i < sn; i++) { const a = C.wrwatchSza(i) >>> 0, f = C.wrwatchSzf(i) >>> 0; srows.push({ sz: C.wrwatchSz(i) >>> 0, a, f, net: a - f }); }
    srows.sort((x, y) => (y.net * y.sz) - (x.net * x.sz));
    console.log(`  per-size net (leaking class = net>0; bytes = net*size):`);
    for (const s of srows) if (s.net !== 0 || s.a > 0) console.log(`    size=${s.sz}B  alloc=${s.a}  free=${s.f}  net=${s.net >= 0 ? '+' : ''}${s.net}  leakedBytes=${s.net * s.sz}`);
  }
}
// Phase 6 Instrument 2: end-of-run branch ring + wedge snapshot (in case the run ended
// without an LCD-stall trigger firing — still surfaces the last-N taken branches).
dumpWedge('end-of-run');
if (leakTrace && C.acallN) {
  const an = C.acallN() >>> 0, arows = [];
  for (let i = 0; i < an; i++) arows.push({ lr: C.acallLr(i) >>> 0, cnt: C.acallCnt(i) >>> 0, egsz: C.acallEgsz(i) >>> 0 });
  arows.sort((x, y) => y.cnt - x.cnt);
  console.log(`ALLOC-CALLER histogram (caller LR @ malloc entry, by count; egsz=example r0 request):`);
  for (const a of arows.slice(0, 25)) console.log(`  LR=0x${a.lr.toString(16)}  count=${a.cnt}  egReqSize=${a.egsz}B`);
}
if ((leakTrace || heapCurve || branchRing) && leakSummaryLines.length) {
  try { mkdirSync(outDir, { recursive: true }); } catch (e) {}
  writeFileSync(`${outDir}/summary.txt`, leakSummaryLines.join('') + '\n');
  console.log(`leak/curve/ring dump appended to ${outDir}/summary.txt`);
}
