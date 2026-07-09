// tools/harness_diff.mjs — the SC1 / D-04 cross-harness fault-report diff.
//
// PURPOSE (Phase 8 plan 05, requirement SC1):
//   Prove "the harness logic was NOT reimplemented per-driver" by EVIDENCE, not
//   assertion: run the SAME config in BOTH harnesses (boot_trace via the H1 step-keyed
//   replay AND nav.mjs via --replay), normalize away the input/pacing-specific noise,
//   and diff the SEMANTIC fault content. If the two normalized reports are identical,
//   both drivers demonstrably routed through the ONE shared src/harness/ core (plans
//   02–04) — the concrete D-04 acceptance ("same config → identical fault report in
//   both harnesses, modulo input/pacing").
//
// THE CANONICAL CONFIG (faithful baseline, from 08-CONTEXT §specifics):
//   no-SIM keypress repro — vbatt 0x220 + BSI 0x100; no SIM/PIN/spike/FAID/bypass/
//   charger; recovery OFF. Macro = the two C presses:
//     [{"key":"c","step":9692410261},{"key":"c","step":10321084655}]
//   which drives the firmware to the keypress-path use-after-free
//   (free(0xAAAAAAAA) via TASK_1_KEYPAD) around step ~10.93B — the HONEST termination
//   the unified harness now produces (this is MEASUREMENT, not a fix; the fix is Phase 9).
//
// ─────────────────────────────────────────────────────────────────────────────────────
// NORMALIZATION RULES (what is stripped before the diff — input/pacing noise, NOT fault
// semantics). The goal: keep the SEMANTIC fault content (reason, faulting LR/PC, the
// staged-reset cause line, the allocator-shadow / HEAPGUARD bad-free alloc-site +
// free-site, the deduped assertion tail), drop everything that legitimately differs
// because the two drivers pace/echo differently:
//
//   N1. Absolute step/cycle/time counters:
//         "@step 10930502373"  "@cyc 296.478 M"  "@123.4M"  "step=10930.5M"  "~12.345 s"
//       → replaced with a fixed token. These differ by construction (instruction-count
//         vs cycle-paced batching) and are the "pacing" the acceptance excludes.
//   N2. Driver input-echo / progress lines:
//         boot_trace:  "REPLAY: ...", "KEYPRESS: ...", "HARNESS: ...", "DCT3 boot trace",
//                      the per-address "RDw/WRw [..]" histogram lines, the "=== stopped"
//                      banner, the I/O-histogram tail.
//         nav.mjs:     "[replay] ...", "replay: ...", "NN_<key> step=... → press ...",
//                      "RESET-VERDICT: ...", "probe ...", build/asset-hash banners, the
//                      per-frame idle-watch progress lines.
//   N3. Framebuffer / PNG / filesystem paths:  "/tmp/...", "*.png", "out=...".
//   N4. Hex-address WIDTH normalization: 0x002444CD vs 0x2444CD → both → 0x2444CD
//       (drivers print the same address at different widths; the address VALUE is the
//        semantic content, the leading-zero width is cosmetic).
//   N5. Whitespace runs collapsed; blank lines dropped; lines lower-cased for the compare
//       (case differs in a few cosmetic labels but never in the semantic hex payload).
//   N6. The assertion-ring TOTAL count + the "(x15, ~973ms apart)" dedup annotations are
//       pacing-derived (how many fired before the catch depends on the exact catch cycle);
//       the ring CONTENTS (codes/LRs) are semantic and retained — only the count/Δt
//       parenthetical is normalized.
//
// After normalization, only lines that carry fault SEMANTICS survive (reason, LR/PC,
// staged-reset cause, bad-free alloc/free sites, assert codes+LRs). The diff is over
// those. A non-empty semantic diff exits non-zero (SC1 FAIL); identical exits 0.
//
// ─────────────────────────────────────────────────────────────────────────────────────
// USAGE:
//   node tools/harness_diff.mjs                 # canonical no-SIM macro, full ~11.1B run
//   STEP_SCALE=0.02 node tools/harness_diff.mjs # scaled-down macro for a fast smoke diff
//   MACRO='[{"key":"c","step":...}]' node tools/harness_diff.mjs   # custom macro
//   BOOTTRACE_ONLY=1 node tools/harness_diff.mjs # boot_trace-vs-boot_trace determinism
//                                                #   (use when the wasm can't be rebuilt)
//
// ENV:
//   FW           firmware image (default firmware/Factory Reset 3310 NR1 v5.79.fls)
//   MACRO        the step-keyed replay JSON (default = the canonical two-C-press repro)
//   STEP_SCALE   multiply every macro step + the budget by this factor (fast smoke runs)
//   BUDGET       boot_trace instruction budget (default = last macro step + 5% headroom)
//   BOOTTRACE_ONLY  if set, skip the nav side and instead run boot_trace TWICE and diff
//                   the two runs (proves determinism + harness reproducibility when the
//                   wasm could not be rebuilt against the unified harness this session)
//
// EXIT: 0 = normalized fault reports identical (SC1 PASS); 1 = semantic diff (FAIL);
//       2 = a harness failed to run / produced no fault report.

import { spawnSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';

const ROOT = '/home/dan/projects/nokia-dct3-emu';
const FW = process.env.FW || `${ROOT}/firmware/Factory Reset 3310 NR1 v5.79.fls`;
const STEP_SCALE = process.env.STEP_SCALE ? parseFloat(process.env.STEP_SCALE) : 1;
const BOOTTRACE_ONLY = !!process.env.BOOTTRACE_ONLY;

// The canonical no-SIM keypress macro (the two C presses → the keypress-path UAF).
const CANONICAL = [
  { key: 'c', step: 9692410261 },
  { key: 'c', step: 10321084655 },
];
let macro = process.env.MACRO ? JSON.parse(process.env.MACRO) : CANONICAL;
if (STEP_SCALE !== 1) macro = macro.map((e) => ({ ...e, step: Math.round(e.step * STEP_SCALE) }));

const lastStep = macro.reduce((m, e) => Math.max(m, e.step), 0);
// Run a little past the last key so the delayed UAF (it lands ~600M steps after the
// second press in the canonical repro) has room to fire honestly.
const budget = process.env.BUDGET
  ? parseInt(process.env.BUDGET, 10)
  : Math.round(lastStep * 1.08 + 700_000_000 * STEP_SCALE);

const macroJson = JSON.stringify(macro);
const macroPath = '/tmp/harness_diff_macro.json';
writeFileSync(macroPath, macroJson);

console.log('=== harness_diff (SC1 / D-04 cross-harness fault-report diff) ===');
console.log(`  fw         : ${FW}`);
console.log(`  macro      : ${macroJson}`);
console.log(`  step_scale : ${STEP_SCALE}`);
console.log(`  budget     : ${budget} insns`);
console.log(`  mode       : ${BOOTTRACE_ONLY ? 'boot_trace × boot_trace (nav deferred)' : 'boot_trace × nav'}`);
console.log('');

// ── Runners ───────────────────────────────────────────────────────────────────────────

// Capture BOTH stdout and stderr — the post-mortem + the instrument appends are written
// to stderr (mad2.c / heap_shadow.c / runloop.c fputs/fprintf(stderr, …)); the driver
// progress is stdout. The semantic fault report lives on stderr, so we MUST merge them.
function capture(cmd, args, env) {
  const r = spawnSync(cmd, args, {
    env: { ...process.env, ...env },
    maxBuffer: 1 << 30,
    encoding: 'utf8',
  });
  return (r.stdout || '') + '\n' + (r.stderr || '');
}

function runBootTrace(label) {
  console.log(`[run] boot_trace (${label}) … budget=${budget}`);
  // HEAPGUARD arms the shared allocator-shadow bad-free trap so the UAF halts honestly
  // with the ASan-style report (the semantic content we diff). No SIM / spike / FAID /
  // bypass / charger; recovery OFF is the boot_trace D-10 default.
  return capture(`${ROOT}/build/dct3_boot_trace`, [FW, String(budget)],
                 { REPLAY: macroPath, HEAPGUARD: '1', BRANCHRING: '1', HEAPSHADOW: '1' });
}

function runNav(label) {
  console.log(`[run] nav.mjs (${label}) … --sim 0 --recover 0 (no-SIM, honest halt)`);
  return capture('node',
    [`${ROOT}/tools/nav.mjs`, FW, '--sim', '0', '--recover', '0',
     '--out', '/tmp/harness_diff_nav', '--replay', macroPath], {});
}

// ── Normalization ───────────────────────────────────────────────────────────────────────

// Pull only the SEMANTIC fault region out of a full driver log, then apply N1–N6.
// We anchor on the shared report markers both drivers emit through the ONE harness:
//   - "=== POST-MORTEM"            (mad2_render_postmortem, the shared model piece)
//   - "=== HEAPGUARD: BAD FREE"    (heap_shadow.c bad-free report, shared)
//   - "=== DEEP-ANALYSIS INSTRUMENTS"  (harness_fault_report append, shared)
//   - "RESET-SUMMARY:" / "reason " lines
// Everything outside those blocks is driver echo/pacing → dropped.
function extractSemantic(log) {
  const lines = log.split(/\r?\n/);
  const keep = [];
  // A line is semantic if it sits inside a report block OR carries a fault keyword.
  const blockStart = /^(={3,}\s*(post-mortem|heapguard|deep-analysis|reset)|--- allocator shadow|--- bad free|=== heapguard)/i;
  const faultLine = /(reason\s+\d|reason=0x|faulting lr|caller-site|resume-pc|staged\s|free\(0x|originally allocated|previously freed|\[double free\]|assert|code=0x|bad-free|allocs=|live-set|backtrace|reset-summary)/i;
  // Driver-echo / progress lines to DROP even if they brush a fault keyword (N2). The
  // boot_trace "HARNESS: recovery ... wild-PC armed @step" banner is config echo, not a
  // fault — it has no nav analogue and must not leak into the semantic set.
  const driverEcho = /^(harness:|replay:|keypress:|keyrelease:|dct3 boot trace|reset-verdict:|\[replay\]|probe |rd[wbh]\s|wr[wbh]\s|=== stopped)/i;
  let inBlock = false;
  for (const raw of lines) {
    const l = raw.trimEnd();
    if (driverEcho.test(l.trim())) { inBlock = false; continue; }
    if (blockStart.test(l.trim())) { inBlock = true; keep.push(l); continue; }
    // A blank line ends a report block.
    if (inBlock && l.trim() === '') { inBlock = false; continue; }
    if (inBlock) { keep.push(l); continue; }
    if (faultLine.test(l)) keep.push(l);
  }
  return keep.map(normalizeLine).filter((l) => l.trim() !== '');
}

function normalizeLine(line) {
  let s = line;
  // N4: normalize hex-address width — 0x0024xxxx / 002444CD → 0x2444cd (drop the
  // 0x00 leading-byte padding the drivers print at different widths; keep the value).
  s = s.replace(/0x0*([0-9a-fA-F]+)/g, (_, h) => '0x' + h.toLowerCase().replace(/^0+(?=.)/, ''));
  // Also catch bare 8-hex-digit addresses (no 0x prefix) the post-mortem prints.
  s = s.replace(/\b00([0-9a-fA-F]{6})\b/g, (_, h) => h.toLowerCase());
  // N1: absolute step/cycle/time counters → fixed token <N>.
  s = s.replace(/@step\s+\d+/gi, '@step <N>');
  s = s.replace(/@cyc\s+[\d.]+\s*M/gi, '@cyc <N>');
  s = s.replace(/@\s*[\d.]+\s*M\b/gi, '@<N>M');
  s = s.replace(/\bstep[=\s]+[\d.]+\s*M?\b/gi, 'step=<N>');
  s = s.replace(/\bcyc[=\s]+[\d.]+\s*M?\b/gi, 'cyc=<N>');
  s = s.replace(/~?\s*[\d.]+\s*s\b/gi, '<T>s');
  s = s.replace(/\(step\s+\d+,\s*size\s+(\d+)/gi, '(step <N>, size $1');
  s = s.replace(/step\s+\d+\)/gi, 'step <N>)');
  // N6: assertion-ring count/Δt parentheticals (pacing-derived) → drop.
  s = s.replace(/\(x\d+,\s*~?[\d.]+\s*ms\s*apart\)/gi, '(xN, ~Δt apart)');
  s = s.replace(/tail of \d+ in ring;\s*\d+ total/gi, 'tail of <N> in ring; <N> total');
  s = s.replace(/\[total=\d+\s+recovered=\d+\]/gi, '[total=<N> recovered=<N>]');
  // N3: paths.
  s = s.replace(/\/tmp\/\S+/g, '<path>');
  // N5: collapse whitespace + lower-case.
  s = s.replace(/\s+/g, ' ').trim().toLowerCase();
  return s;
}

// ── Run + diff ──────────────────────────────────────────────────────────────────────────

const aRaw = runBootTrace('A');
const aSem = extractSemantic(aRaw);
writeFileSync('/tmp/harness_diff_A.semantic.txt', aSem.join('\n') + '\n');

let bRaw, bSem, bLabel;
if (BOOTTRACE_ONLY) {
  // No wasm rebuild available → prove determinism + reproducibility of the harness by
  // running boot_trace a SECOND time and diffing. Identical-to-itself across runs is the
  // honest fallback acceptance; the nav-side parity is explicitly flagged DEFERRED.
  bRaw = runBootTrace('A2');
  bLabel = 'boot_trace (run 2)';
} else {
  bRaw = runNav('B');
  bLabel = 'nav.mjs';
}
bSem = extractSemantic(bRaw);
writeFileSync('/tmp/harness_diff_B.semantic.txt', bSem.join('\n') + '\n');

console.log('');
console.log(`  boot_trace semantic fault lines : ${aSem.length}`);
console.log(`  ${bLabel.padEnd(31)} semantic lines : ${bSem.length}`);

if (aSem.length === 0) {
  console.error('\nSC1: INCONCLUSIVE — boot_trace produced NO semantic fault report.');
  console.error('  (Did the run reach the UAF? Check budget / macro steps. Raw log not faulting.)');
  process.exit(2);
}
if (bSem.length === 0 && !BOOTTRACE_ONLY) {
  console.error('\nSC1: nav.mjs produced NO semantic fault report.');
  console.error('  The on-disk wasm may predate the unified harness, or the macro did not');
  console.error('  reach the fault in the cycle-paced run. Re-run with BOOTTRACE_ONLY=1 to');
  console.error('  prove the boot_trace side deterministically, and defer the nav-side parity.');
  process.exit(2);
}

// Diff the two normalized semantic line-sets (order-independent on the report BODY: the
// drivers may interleave the instrument appends slightly differently, so compare as a
// multiset of normalized lines — the SEMANTIC content, not the print order).
function multiset(lines) {
  const m = new Map();
  for (const l of lines) m.set(l, (m.get(l) || 0) + 1);
  return m;
}
const ma = multiset(aSem), mb = multiset(bSem);
const onlyA = [], onlyB = [];
for (const [l, c] of ma) { const d = c - (mb.get(l) || 0); for (let i = 0; i < d; i++) onlyA.push(l); }
for (const [l, c] of mb) { const d = c - (ma.get(l) || 0); for (let i = 0; i < d; i++) onlyB.push(l); }

if (onlyA.length === 0 && onlyB.length === 0) {
  console.log('');
  if (BOOTTRACE_ONLY) {
    console.log('SC1: boot_trace fault report is IDENTICAL across runs (deterministic).');
    console.log('     nav-side parity DEFERRED — see header (BOOTTRACE_ONLY). The C side');
    console.log('     reproduces the canonical UAF honestly; once the wasm is confirmed');
    console.log('     rebuilt against the unified harness, re-run without BOOTTRACE_ONLY.');
  } else {
    console.log('SC1: IDENTICAL — boot_trace and nav produced the SAME normalized fault report.');
    console.log('     The no-reimplementation rule (D-04) is PROVEN: both drivers route');
    console.log('     through the ONE shared src/harness/ core (modulo input/pacing).');
  }
  process.exit(0);
}

console.error('');
console.error('SC1: SEMANTIC DIFFERENCE — the two harnesses produced DIFFERENT fault reports.');
console.error(`  ${onlyA.length} line(s) only in boot_trace, ${onlyB.length} only in ${bLabel}:`);
for (const l of onlyA.slice(0, 30)) console.error(`  - [boot_trace] ${l}`);
for (const l of onlyB.slice(0, 30)) console.error(`  + [${bLabel}] ${l}`);
console.error('\n  Full normalized reports: /tmp/harness_diff_A.semantic.txt /tmp/harness_diff_B.semantic.txt');
process.exit(1);
