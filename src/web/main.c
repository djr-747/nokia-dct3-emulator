// DCT3 Emulator — web shell entry point.
//
// Phase 2/3: drive the full firmware boot in-browser behind the src/mad2 device
// model, render the PCD8544 framebuffer to a canvas, and feed the keypad matrix
// from the page. This mirrors the native boot_trace harness's setup + inner loop
// (mad2 I/O hooks, per-step timer/FIQ/IRQ delivery) so the firmware reaches the
// live OS, plus the two RAM HLE pokes that pass the self-test verdict (route C)
// and bypass the SIM gate (anonymous-access equivalent) — see docs/boot-trace.
//
// Flash is preloaded into MEMFS at /fw.fls (Makefile --preload-file).

#include <emscripten.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/dct3_core.h"
#include "mad2/mad2.h"
#include "mad2/emu_host.h"   // EmuHost — the shared HAL observation/input contract
#include "models/model.h"
#include "harness/harness.h"                  // shared fault-detect + recover + D-06 instruments (plan 04)
#include "harness/seccode.h"                   // seccode_call_sync — in-context firmware cipher call
#include "mgba/internal/arm/isa-inlines.h"   // _ARMSetMode / ThumbWritePC for the CALL primitive

// Vendored ARM core interrupt entry points (declared in arm.h via dct3_core.h;
// re-declared here for clarity — same as the native harness uses).
void ARMRaiseIRQ(struct ARMCore* cpu);
void ARMRaiseFIQ(struct ARMCore* cpu);

#define DCT3_VERSION "0.17.0-fiq4-timer0-tick"

// Memory map (flash/boot-entry/I/O window) + the flash range now come from the
// selected ModelProfile (g_model->mem), so the web build boots any DCT3 image the
// firmware picker loads, not just the 3310. See src/models/.

// --- Boot spike (version-independent RAM HLE) ------------------------------
//   POKE_VERDICT (0x11FF15) = 0xC8 : self-test verdict passes -> firmware resumes the
//                      GSM group -> organic POWER_ON_NORMAL (route C).
//   SIM-gate byte           = 0x06 : disp49 (= sim_func, the SIM/PIN gate) always takes
//                      its proceed/quit branch -> SIM bypassed (anonymous access).
// These used to be version-specific: held from a magic step count (1.5M for v6.39,
// 3.4M for v5.79) with the SIM-gate address hardcoded. Now both are DERIVED so the
// spike adapts to any 3310 firmware (resolved in dct3_web_boot / applied per step):
//   - TIMING: arm the instant the firmware first writes the 0xC8 verdict (value-trigger
//     -> no step count). After arming, pin 0xC8 so it can't degrade to 0xCC/0x8C.
//   - SIM ADDRESS: a NokiX-style masked signature locates sim_func; the SIM-gate byte is
//     the RAM base literal it loads (0x11FCFE on 3310) + 29. Falls back to the known addr.
// The verdict + SIM-gate addresses now live in the model profile (g_mad2.fw.verdict /
// .sim_gate), resolved per-image by model_resolve() (signature + 3310 fallback).

// "Skip security code" (FuBu v6.39 disp77 anti-theft lock) — checksum-completeness.
// disp77 arms because the FuBu FAID runtime integrity check (0x2E8C66) finds a
// mismatch: it sums a 16-byte FAID block and compares to the stored u16 at
// [0x111B8A]; on the "security block reset" image the computed value (0x03B7) !=
// the stored value (0x0310), so it reports LOCKED and the MMI dispatches disp77
// (held, then re-armed each boot-sweep). The faithful fix is to make the check
// CONSISTENT — store the computed sum so the check passes legitimately. Poking
// [0x111B8A] = 0x03B7 does that: the check returns not-locked and the FAID state
// is internally consistent, so the phone boots past the lock with no reboot-spin.
// (Merely skipping/forcing the screen leaves FAID inconsistent and spins at
// 0x2EEBEC — verified.) Gated by g_skip_seclock,
// default OFF. The FAID sum address + expected value are per-image, carried in the
// model profile (g_mad2.fw.faid_cksum / .faid_cksum_val).

// Power key: held at boot so the power-on-reason gate (~2M insns) reads it and
// derives "powered on by key" -> boot. A real press is momentary, so once the
// reason is latched and the power-up has settled, auto-release it (raise the
// keypad IRQ so the release is seen), modelling a brief press-and-let-go.
#define PWR_RELEASE_STEP 12000000L

// The core periodically rebases cpu->cycles down by this amount (see
// DCT3_EVENT_SLICE in src/core/dct3_core.c irq_processEvents) to keep the signed
// int32 counter from overflowing. We undo that here when summing true elapsed
// cycles so the real-time (cycle-paced) clock isn't disturbed by the rebase.
#define DCT3_CYCLE_REBASE 0x20000000

static DCT3Core* g_core = NULL;
static Mad2      g_mad2;

// --- Unified PCM audio channel (HAL pcm_sink -> page) ------------------------
// The buzzer (via emu_audio_render) and the 5110 DSP codec (DXR tap) both deliver
// ch1 earpiece samples here, at DCT3_CODEC_HZ. The page pulls them each frame with
// dct3_web_pcm_read and plays them through one Web Audio node — the SAME PCM stream
// the SDL GUI plays, so buzzer onset is sample-accurate instead of frame-polled.
#define WEB_PCM_SZ   16384u          // power of two; ~0.9 s at ~18.6 kHz
#define WEB_PCM_MASK (WEB_PCM_SZ - 1u)
static int16_t  g_web_pcm[WEB_PCM_SZ];
static unsigned g_web_pcm_head, g_web_pcm_tail;   // head=producer(core), tail=consumer(page)
static void web_pcm_sink(struct Mad2* m, int ch, int16_t s) {
    (void)m;
    if (ch != 1) return;                                            // earpiece (ch1) only
    if ((unsigned)(g_web_pcm_head - g_web_pcm_tail) >= WEB_PCM_SZ - 1u) return;  // full -> drop
    g_web_pcm[g_web_pcm_head & WEB_PCM_MASK] = s;
    g_web_pcm_head++;
}
// Shared harness state (plan 04). web_step_once calls harness_observe() before the
// step + harness_post_step() after — the SAME entrypoints boot_trace uses (SC1). The
// web driver no longer owns its own recover hooks / leak-tracker / branch ring; those
// live in src/harness/ once. Recovery DEFAULT ON in the web (the device keeps running
// past a recoverable reset in-browser) — the CLI flips it OFF (D-10 honest halt); the
// web is the live-device presentation, so it warm-recovers as before.
static HarnessConfig g_hc;
static int       g_harness_init = 0;     // one-time harness_init per boot
static int       g_web_recover  = 1;     // web default: recovery ON (live device)
static int       g_fault_reported = 0;   // one-shot: deep report rendered once per fault episode
// WR-03: episode-scoped re-arm. The one-shot above latches once a fault episode is
// reported. The web does NOT warm-reset wild-PC / spin / HEAPGUARD / invariant (the page
// keeps the canvas alive), so a continuing fault returns FAULT_HALT every step and a 2nd
// DISTINCT episode later in the same boot would never be re-reported. Debounce: only
// re-arm g_fault_reported after a healthy stretch of N consecutive HARNESS_CONTINUE steps
// (Pitfall 3 — clearing on EVERY continue step re-fires the same continuing fault). A
// genuinely distinct 2nd episode (after the phone recovers + runs clean) re-reports.
static int64_t   g_fault_continue_run = 0;  // consecutive CONTINUE steps since last report
#define FAULT_REARM_CONTINUE 4096            // healthy steps before a 2nd episode may re-report
static int64_t   g_step = 0;          // 64-bit: `long` is 32-bit in wasm32 and
                                      // would overflow ~2.15B insns (~4 min),
                                      // go negative, and silently disable the
                                      // POKE_AFTER-gated route-C/SIM pokes ->
                                      // timers/OS state break. int64_t avoids it.
static uint64_t  g_cycles64 = 0;     // true elapsed CPU cycles (rebase-corrected),
                                      // for the real-time cycle-paced run loop.
static uint32_t  g_rtc_wr_lr = 0;    // external caller of the CCONT RTC-reg write (diag)
// capture (LR, logical-reg, value) at the CCONT write-reg helper entry
// 0x2E98D4 (r0=packed reg<<8|mask, r1=value). Finds who writes the RTC regs. Strip before commit.
#define CCWR_N 24
static uint32_t g_ccwr_lr[CCWR_N], g_ccwr_reg[CCWR_N], g_ccwr_val[CCWR_N];
static uint32_t g_ccwr_w = 0;

// PPM string tracer: hook get_string (0x2BBFAC) / w_get_string (0x2BBCB8); on each call log
// the caller, the decoded nokstr (PPM string ID or ascii), and the resolved text (read at the
// return). Lets the page map any on-screen text -> the firmware code that shows it. Off by
// default; enable via dct3_web_getstr_on(1), reproduce a screen, then dct3_web_getstr_dump().
// get_string / w_get_string addresses are per-image (g_mad2.fw.get_string / .w_get_string).
static int       g_gs_on = 0;
static char      g_gs_log[16384];
static int       g_gs_len = 0;
static uint32_t  g_gs_ret = 0, g_gs_lr = 0, g_gs_in = 0; static int g_gs_wide = 0;
static uint32_t  g_gs_stk[8];        // real BL-return-addrs from the stack (call chain)
static uint32_t  g_gs_last = 0;      // dedup: last (id<<8 ^ caller)
static int       g_bypass_sim = 1;   // toggle via dct3_web_set_bypass()
static int       g_skip_seclock = 1;  // "FAID Pass": recompute FAID so disp77 never arms; default ON
static const ModelProfile* g_model = NULL;   // selected phone profile (autodetected at boot)
static uint32_t  g_sim_addr = 0;             // SIM-gate byte; resolved at boot (g_mad2.fw.sim_gate)
static uint32_t  g_simfunc  = 0;             // located sim_func addr (0 = signature miss)
static int       g_spike_armed = 0;          // boot spike active (firmware wrote 0xC8 verdict)
static int       g_spike_force = -1;         // boot-spike (verdict pin) override: -1 auto
                                             // (per-firmware), 0 = force off, 1 = force on
static int       g_pin_verdict = 0;          // pin the self-test verdict at 0xC8 (only for
                                             // firmware whose self-test can't pass organically
                                             // in our model — see resolve_spike)
static int       g_pwr_released = 0;  // one-shot auto-release guard
static int       g_pwr_auto = 1;      // auto-release power key after boot

// Keypad auto-release — decouple the EMULATED key-hold from the wall-clock hold.
//
// The page paces the core at ~real-time, so a physical tap (~100 ms) spans only
// ~900k emulated instructions. On key-down we hold the matrix bit for a FIXED
// window then release it ourselves (clear the bit + pulse the keypad IRQ -> the
// firmware runs a no-key scan -> its key-up handler emits MSG_KEYRELEASE). The
// physical key-up is IGNORED (see dct3_web_key): this window is authoritative.
//
// WHY THE WINDOW MUST BE ~5M (the menu-roll fix): the menu dispatcher (disp2)
// auto-repeats the scroll by re-emitting MSG_KEY_DOWN (0x366 = the down-arrow
// KEY) once per MSG_ANIMATION_POINT, and it only HONORS the key release after its
// repeat grace (~3.6M insns). A release before that (the old 150k window — and a
// real ~900k physical tap) is missed, so the menu scrolls forever. PROVEN with
// the in-page "Send KEYRELEASE" button (invokes the firmware key-up handler): a
// release fired after the grace stops the scroll; before it, no effect. Measured
// total scroll per single tap vs this window: 1.5M/3M = 9 steps (rolls); 4M..8M =
// exactly 1 step; 12M = 2. So ~5M sits in the one-step band: the press scrolls
// once, the grace suppresses repeats, and the auto-release lands the KEYRELEASE
// in the effective window -> the menu stops. (0x366/0x367 are the down/up ARROW
// keys, not press/release; the stop signal is 0xc9 / the up handler.) See memory
// project-state SESSION 12.
#define KEY_HOLD_INSNS 200000L   // single-press hold (snappy taps, just past the scan/deliver floor)
static int64_t  g_key_hold_insns = KEY_HOLD_INSNS;
static int      g_key_row = -1, g_key_col = -1;  // the key awaiting auto-release
static int      g_key_armed = 0;

// --- Key-press queue (atomic, self-paced taps) ------------------------------
// A tap's down/hold/up must play out over EMULATED time (the firmware needs the
// down held past its repeat grace, then a clean up). But JS delivers presses at
// wall-clock pace decoupled from emulated time: tap too fast and the next key
// stomps the previous one mid-hold (keys lost -> stuck on the home screen); tap
// too slow and the long fixed hold reads as a long-press that exits the menu
// (reverts to home). Fix: dct3_web_key enqueues presses here and a small state
// machine in the run loop plays each one atomically — release the previous key
// and let the firmware settle, deliver the new down, hold past the grace,
// release cleanly, settle — so the tap result is independent of how fast/slow
// the JS calls arrive. The queue absorbs bursts; gaps don't matter.
#define KEY_Q_N 16
#define KEY_SETTLE_INSNS 1200000L          // run after a release before the next down
static struct { int row, col; } g_keyq[KEY_Q_N];
static int      g_keyq_head = 0, g_keyq_tail = 0;   // ring indices (count = tail-head)
static int      g_key_seq_state = 0;        // 0 idle, 1 holding (down), 2 settling (after up)
static int64_t  g_key_seq_at = -1;          // g_step of the next state transition
// Experimental opt-in "one-step keys" (dct3_web_set_oneshot, default off): the
// firmware re-issues MSG_KEY_DOWN (0x366) every keypad scan while a key is held,
// so on animated menus it scrolls once per frame. When this PC is reached the
// menu has just QUEUED a KEY_DOWN (sent via 0x2D84F0); release the key shortly
// after, SILENTLY (no IRQ) so the queued event still reaches the handler but no
// further scan re-issues it -> one step per tap. Off => the fixed window above.
#define FW_KEYDOWN_SENT 0x002D84F4u      // just after the menu's KEY_DOWN send
#define KEY_RELEASE_GRACE 1500L
static int      g_key_oneshot  = 0;              // experimental mode on/off
static int      g_key_detected = 0;              // KEY_DOWN queued this press

// --- Debug instrumentation (headless keypad-timing measurement) -------------
// A handful of PC-hit counters so a Node harness can measure firmware behaviour
// (keypad scan period, new-press vs repeat posts) without a debugger. Zero cost
// when no watch is armed (single branch in the hot loop).
#define DBG_NWATCH 6
static uint32_t g_dbg_pc[DBG_NWATCH]  = {0};
static int64_t  g_dbg_cnt[DBG_NWATCH] = {0};
static int      g_dbg_any = 0;

// last-N executed PCs, frozen the instant control jumps into the low
// (exception-vector / null) range. The clock-tick null-callback crash lands at PC=0x20
// via a bx/blx to a 0-handler; a heuristic stack-walk mislabels the frames, so this
// records the EXACT instruction trail into the bad branch. Freeze-on-crash keeps the
// lead-up from being overwritten by post-crash garbage execution. Strip before commit.
#define PCRING_N 96
static uint32_t g_pcring[PCRING_N];        // RAW gprs[15] (= PC + pipeline) at each step
static uint32_t g_pcring_cpsr[PCRING_N];   // full cpsr.packed at each step (mode + T bit)
static uint32_t g_pcring_w = 0;
static int      g_pcring_frozen = 0;
static uint64_t g_pcring_step     = 0;   // local monotonic step counter
static uint64_t g_pcring_last_low = 0;   // step of the previous low PC
static int      g_pcring_lowcnt   = 0;   // clustered low-PC entries

// Divergence-finder trace ring. Sample (cyc, step, pc, cpsr, fiq_pending, irq_pending)
// at fixed cycle intervals to compare two runs of the same bundle and pinpoint the
// FIRST instant they diverge. The trace is captured per-instruction (in web_step_once)
// only when the cycle accumulator crosses the next sample boundary, so the cost is
// one cmp + branch per step plus occasional writes. Turned off by default; the replay
// path turns it on automatically so paste-replay traces are immediately diffable.
#define TRACE_CYC_INTERVAL 100000ULL    // one sample per 100k cycles (~1490 entries / 149M cyc)
#define TRACE_N 8192                     // 8K entries -> covers ~800M cycles before ring wrap
typedef struct {
    uint64_t cyc;
    uint64_t step;
    uint32_t pc;
    uint32_t cpsr;
    uint8_t  fiq_pending;
    uint8_t  irq_pending;
    uint16_t _pad;
} TraceEntry;
static TraceEntry g_trace[TRACE_N];
static uint32_t   g_trace_w = 0;
static uint64_t   g_trace_next_cyc = 0;
static int        g_trace_on = 0;

// UI-message log: capture r0 (the message id) each time the firmware's send/route
// core is hit, into a ring. Lets the page show a live histogram of messages sent
// to handlers — e.g. to spot a draw/key message re-posted every animation frame.
#define MSGLOG_N 512
static uint32_t g_msglog[MSGLOG_N]    = {0};   // r0 (message id) at the capture PC
static uint32_t g_msglog_lr[MSGLOG_N] = {0};   // LR (caller) at the capture PC
static uint32_t g_msglog_w  = 0;       // total captures (ring index = w % N)
static uint32_t g_msglog_pc = 0;       // PC to capture r0 at (0 = disabled)

// --- Firmware-function CALL primitive (debug) ------------------------------
// Invoke a Thumb firmware function from JS, in the live boot context. Mirrors the
// native boot_trace CALL: when a call is pending and the CPU is in System mode,
// hijack PC to the fn (args in r0/r1, LR set to return to the hijack point), let
// it run, and restore the interrupted context when it returns. Used to send a
// key event on demand (e.g. the key-up handler) to test cause/effect live.
static uint32_t g_call_fn = 0, g_call_r0 = 0, g_call_r1 = 0;
static int      g_call_pending = 0, g_call_active = 0;
static uint32_t g_call_ret_pc = 0;
static int32_t  g_call_save_gpr[15];
static uint32_t g_call_save_cpsr = 0;
static uint32_t g_call_count = 0;     // completed calls (so JS can confirm it ran)
static uint32_t g_call_result = 0;    // r0 captured at call return
static uint32_t g_cap_pc = 0, g_cap_val = 0, g_cap_hits = 0;   // capture r0 at a PC
// send-logger — ring of (task<<16|msg) at send 0x29921C / enqueue 0x2997B0.
// Lets a node probe trace the post-ready message cascade. Strip before commit.
#define SENDLOG_N 256
static uint32_t g_sendlog[SENDLOG_N]; static uint32_t g_sendlog_lr[SENDLOG_N]; static uint32_t g_sendlog_w = 0; static int g_sendlog_on = 0;

// --- Instrument 1+2: allocator shadow + typed branch ring (now in src/harness/) -
// The Phase-6 alloc/free leak-tracker (entry->return latch + outstanding table) and
// the typed branch-trace ring USED to live here, duplicated against the native CLI.
// In plan 04 they moved into the shared core: src/harness/heap_shadow.c (allocator
// shadow live-set + HEAPGUARD) and src/harness/telemetry.c (shadow call stack). The
// web driver now ARMS them through the cwrap toggles below (re-pointed to the harness
// instrument knobs) and reads their data through the shared accessors — no web-side
// reimplementation (memory: feedback-shared-core-no-reimplementation).

// --- used-bytes curve sampler ([0x10484C] over the run, driver-side diagnostic) --
// The curve is still rendered driver-side (the page plots it), reading the heap-acct
// used-bytes through the shared allocator-shadow accessor (heap_shadow_used_d).
#define HEAPCURVE_N        4096u                 // ring of used-bytes samples
#define HEAPCURVE_EVERY    65536u                // sample every N steps
static int      g_heapcurve_on = 0;              // default OFF
static uint32_t g_heapcurve_step[HEAPCURVE_N];   // low 32 of g_step at the sample
static uint32_t g_heapcurve_used[HEAPCURVE_N];   // [0x10484C] used-bytes (BE u32)
static uint32_t g_heapcurve_w = 0;               // total samples (ring idx = w % N)

EMSCRIPTEN_KEEPALIVE
void dct3_web_call(uint32_t fn, uint32_t r0, uint32_t r1) {
    g_call_fn = fn; g_call_r0 = r0; g_call_r1 = r1; g_call_pending = 1;
}

// One-shot register spike: when PC next reaches g_ts_pc, overwrite gprs[reg] with
// val and disarm. Used by the web "Type text" box to ride a REAL keypress and replace
// the char the firmware posts (spike r1 at the char-post call site) — so the char goes
// out through the firmware's own balanced down/up/commit flow. latch→key→watch→spike→release.
static uint32_t g_ts_pc = 0, g_ts_val = 0;
static int      g_ts_reg = 0, g_ts_armed = 0, g_ts_count = 0;
// Security-code unlock: while armed, repair verify's input string (r0 -> "12345") at the
// verify call site (see dct3_web_seccode_reset). Persists so both gates pass with one arm.
static int      g_seccode_armed = 0, g_seccode_count = 0;
EMSCRIPTEN_KEEPALIVE
void dct3_web_regspike(uint32_t pc, int reg, uint32_t val) {
    g_ts_pc = pc & ~1u; g_ts_reg = reg & 15; g_ts_val = val; g_ts_armed = 1;
}
EMSCRIPTEN_KEEPALIVE
int dct3_web_regspike_count(void) { return g_ts_count; }   // total spikes fired
EMSCRIPTEN_KEEPALIVE
double dct3_web_call_count(void) { return (double)g_call_count; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_call_result(void) { return g_call_result; }

// Unlock the security-code screen by setting the code to the factory default 12345. ARMs a
// hook at the verify call site (g_mad2.fw.seccode_verify) that, on every verify, (1) re-provisions
// the stored code to encrypt("12345") via the firmware's OWN cipher called in-context (matching
// the live digest verify uses) and (2) repairs verify's input string to "12345". The firmware's
// own verify then genuinely accepts and runs its real unlock path — see the hook in web_step_once
// for the full rationale. Needed because this 2100 image's grafted cross-version EEPROM stored a
// ciphertext no typed code could satisfy under the v5.84 cipher. The arm persists so one click
// covers the boot lock AND the Restore-Factory-Settings re-prompt. Profile-driven — no-ops on
// models without seccode addresses. Returns 1 if armed, 0 if the model has no verify address.
EMSCRIPTEN_KEEPALIVE
int dct3_web_seccode_reset(void) {
    if (!g_core || !g_mad2.fw.seccode_verify) return 0;
    g_seccode_armed = 1;
    return 1;
}
EMSCRIPTEN_KEEPALIVE
void dct3_web_cap_set(uint32_t pc) { g_cap_pc = pc; g_cap_hits = 0; g_cap_val = 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_cap_val(void) { return g_cap_val; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_cap_hits(void) { return g_cap_hits; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_ccw(int reg) { return g_core ? ((g_mad2.dbg_ccw_count[reg&15] << 8) | g_mad2.dbg_ccw_last[reg&15]): 0; }   // per-CCONT-reg count<<8|last
EMSCRIPTEN_KEEPALIVE
void dct3_web_difftrace(int on, double every, double lo, double hi) { difftrace_arm(on, every, lo, hi); }   // Phase9 09-02: native-vs-wasm lockstep hash (gated; fprintf->console)
EMSCRIPTEN_KEEPALIVE
void dct3_web_sendlog_on(int en) { g_sendlog_on = en ? 1: 0; if (en) g_sendlog_w = 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_sendlog_w(void) { return g_sendlog_w; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_sendlog_at(int i) { return g_sendlog[i % SENDLOG_N]; }   // (task<<16|msg)
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_sendlog_lr(int i) { return g_sendlog_lr[i % SENDLOG_N]; }   // sender LR

EMSCRIPTEN_KEEPALIVE
const char* dct3_version(void) { return DCT3_VERSION; }

// Masked byte-signature search over the loaded flash (NokiX-style smart-match). Kept
// here only for the SIM-gate diagnostic below; the authoritative firmware-address
// resolution is model_resolve() in src/models/model.c. Halfword-aligned; returns the
// device address of the first match in [lo,hi), or 0 if none.
static uint32_t sig_find(const uint8_t* pat, const uint8_t* msk, int n, uint32_t lo, uint32_t hi) {
    const uint8_t* r = g_core->ram;
    for (uint32_t a = lo & ~1u; a + (uint32_t)n <= hi; a += 2) {
        int ok = 1;
        for (int i = 0; i < n; i++)
            if ((r[(a + i) & DCT3_RAM_MASK] & msk[i]) != (pat[i] & msk[i])) { ok = 0; break; }
        if (ok) return a;
    }
    return 0;
}

// Resolve the per-image firmware addresses for the selected model (signatures over the
// loaded flash, profile fallbacks otherwise) into g_mad2.fw, then mirror the boot-spike
// fields the run loop reads. Adapts to whatever DCT3 image the firmware picker loaded.
static void resolve_spike(void) {
    model_resolve(g_model, g_core->ram, DCT3_RAM_MASK, &g_mad2.fw);
    g_sim_addr = g_mad2.fw.sim_gate;
    g_spike_armed = 0;

    // Diagnostic only (spike_info exposes it to the page): did the SIM signature locate
    // sim_func, or did sim_gate fall back? Re-run the profile's "sim_gate" signature.
    g_simfunc = 0;
    for (int i = 0; i < g_model->n_sigs; i++) {
        if (strcmp(g_model->sigs[i].name, "sim_gate") != 0) continue;
        const Sig* s = &g_model->sigs[i].sig;
        uint32_t lo = s->lo ? s->lo : g_model->mem.flash_base;
        uint32_t hi = s->hi ? s->hi : g_model->mem.flash_base + g_model->mem.flash_size;
        g_simfunc = sig_find(s->pat, s->mask, (int)s->len, lo, hi);
        break;
    }

    // Decide whether to PIN the self-test verdict at 0xC8 (vs let the firmware drive it).
    //
    // The self-test verdict byte [0x11FF15] evolves the same way on every 3310 image:
    // 00->08->48->C8->CC->8C->88->08 (it momentarily passes 0xC8, then settles to 0x08).
    // Pinning it at 0xC8 forever is an HLE compensation: on the FuBu v6.39 image our
    // incomplete DSP/GSM model fails the self-test downstream, so without the pin it never
    // brings up the lock screen (it draws a blank/charge screen). The pin keeps the test
    // "passing" so v6.39 reaches its UI.
    //
    // But on genuine retail firmware (v5.79) the self-test passes ORGANICALLY: pinning the
    // verdict freezes its state machine mid-sequence, which slowly corrupts the OS state
    // and — with the (faithful) Timer1/FIQ5 soft-timer walk also running — eventually
    // overflows the firmware's tight interrupt-dispatcher stack (canary 0x1164D0), tripping
    // its stack-overflow guard -> reboot reason 6 (0x2EEBAE) -> spin at 0x2EEBEC (~320M
    // instructions). Native boot_trace reproduces this EXACTLY when given the same pin
    // (LED still times out, then 0x2EEBEC); with the verdict left alone it runs clean to
    // budget. So the pin must NOT be applied to firmware whose self-test we model well.
    //
    // UPDATE: the faithful DSP self-test responder (mad2)
    // now passes the verdict ORGANICALLY for v6.39 (group-0x74 reply -> command-13 -> 0xC8),
    // and retail v5.79 always self-tested organically. So NO firmware needs the HLE verdict
    // pin any more -> default OFF. (Pinning v5.79 froze its state machine into the 0x2EEBEC
    // spin per the note above; the responder avoids that and makes the pin redundant for
    // v6.39 too.) The user can still force the pin on via the Boot-spike toggle.
    g_pin_verdict = g_model->boot.pin_verdict_default;
    if (g_spike_force >= 0) g_pin_verdict = g_spike_force;   // UI/CLI override (Boot-spike toggle)
}

// Boot the firmware: load flash, wire mad2, HLE the boot ROM. Returns 0 on ok.
EMSCRIPTEN_KEEPALIVE
int dct3_web_boot(void) {
    if (!g_core) g_core = dct3_core_create();
    else dct3_core_reset(g_core);
    if (!g_core) return -4;

    // Cold power-on: wipe working RAM so a reboot starts from the same zero state as
    // the very first boot (dct3_core_create callocs to zero). dct3_core_reset only
    // resets the CPU; without this, the firmware re-runs against the previous run's
    // dirty RAM — boot-gating flags that should read 0 are stale, so the phone never
    // comes up. Flash (+ embedded EEPROM) is reloaded just below and the JS layer
    // overlays any saved NVRAM after boot(), so a reboot == a fresh power-on.
    memset(g_core->ram, 0, DCT3_RAM_SIZE);

    FILE* f = fopen("/fw.fls", "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    if (n <= 0) { fclose(f); return -2; }
    uint8_t* b = (uint8_t*)malloc((size_t)n);
    if (!b || fread(b, 1, (size_t)n, f) != (size_t)n) { free(b); fclose(f); return -3; }
    fclose(f);

    // Select the phone profile from the loaded image (autodetect by product code),
    // falling back to the 3310. Determines the memory map + per-model behaviour.
    g_model = model_detect(b, (uint32_t)n);
    if (!g_model) g_model = model_default();

    // Model-aware key-hold floor (keypad.hold_insns; 0 -> the default). The serial keypad
    // (5110) is IRQ-driven but runs a multi-tick debounce FSM (jump-table 0x25B5F4, each
    // state re-posts a self-message via 0x25422A): a press is not decoded into the rawkey
    // store [0x10B6C8] until 3-4 RTOS scheduler ticks (~70k insns each) after the down-edge
    // — measured 197k-293k. The default 200k sits *under* that floor so taps are released
    // before decode and dropped; the profile holds them 400k. Snappy 200k elsewhere.
    //
    g_key_hold_insns = (g_model && g_model->keypad.hold_insns) ? g_model->keypad.hold_insns
                                                               : KEY_HOLD_INSNS;

    dct3_write_bytes(g_core, g_model->mem.flash_base, b, (size_t)n);
    free(b);

    // Repair the MCU flash-region checksum so a file-patched image (e.g. NokiX
    // kill_faid_check) passes the boot integrity check instead of warm-rebooting
    // (reset reason 4). No-op on an unpatched image (byte-identical).
    dct3_fix_mcu_checksum(g_core);

    memset(&g_mad2, 0, sizeof g_mad2);
    mad2_init(&g_mad2, g_model);
    g_mad2.sim_present = 0;          // web default: SIM not inserted (UI toggle reflects this)
    g_mad2.verbose = 0;
    g_mad2.mem = g_core->ram;
    g_mad2.mem_mask = DCT3_RAM_MASK;
    g_web_pcm_head = g_web_pcm_tail = 0;
    g_mad2.pcm_sink = web_pcm_sink;   // unified audio: buzzer + DSP codec -> one PCM stream

    resolve_spike();   // resolve per-image firmware addresses (signatures + fallbacks) into g_mad2.fw

    // mad2_read/mad2_write match the dct3 io hook signatures (ctx = Mad2*).
    dct3_set_io_hooks(g_core, &g_mad2, g_model->mem.io_lo, g_model->mem.io_hi, mad2_read, mad2_write);
    dct3_set_io_range2(g_core, g_model->mem.flash_base,
                       g_model->mem.flash_base + g_model->mem.flash_size);

    dct3_boot_at(g_core, g_model->mem.boot_entry);
    g_step = 0;
    g_cycles64 = 0;
    // Re-arm the power-key auto-hold/release for this boot. A prior manual power action
    // (dct3_web_power, e.g. the power-off hold) sets g_pwr_auto=0; without re-arming, the
    // power key mad2_init re-holds for the power-on reason is never released on a reboot,
    // so the firmware sees the power key stuck down -> power menu -> spin (no key response).
    g_pwr_auto = 1;
    g_pwr_released = 0;
    g_key_armed = 0; g_key_row = g_key_col = -1; g_key_detected = 0;
    g_keyq_head = g_keyq_tail = 0; g_key_seq_state = 0; g_key_seq_at = -1;
    g_pcring_w = 0; g_pcring_frozen = 0; g_pcring_step = 0; g_pcring_last_low = 0; g_pcring_lowcnt = 0;   // re-arm the crash trail
    g_trace_w = 0; g_trace_next_cyc = 0;   // divergence-finder ring resets per (re)boot
    // NOTE: the old web-only EEPROM-write gate was REMOVED. It blocked the
    // firmware's boot-time FFS init writes (CFI reported success, array unchanged), leaving the
    // FFS half-initialised — NVRAM saves (Nokia Club ID) wrote garbage and in-flash app launch
    // (Snake 2) hung in the FFS readiness check. Native never gated flash writes; the web now
    // matches it. Any cold-boot persistence crash must be fixed in the NVRAM snapshot path, not
    // by dropping live flash writes. (git history has the gate if ever needed.)
    // Init the shared harness (plan 04). The web is the live-device presentation, so it
    // keeps recovery ON by default (warm-recovers a recoverable reset in-browser) — the
    // CLI flips it OFF (D-10 honest halt). harness_init mirrors the recovery flag onto
    // m->recover_enabled and arms the wild-PC / spin detectors with the 16-step boot
    // warmup (the boot-ROM HLE de-pipe artifact). SPINLIMIT/wild-PC use the defaults.
    memset(&g_hc, 0, sizeof g_hc);
    // wild_pc_after = 16: the boot-ROM HLE de-pipe leaves a transient PC=0xFFFFFFFC at
    // step 0 (>= 0x400000) that would false-fire wild-PC on sight — the SAME 16-step
    // warmup boot_trace uses. spin_limit 0 => the harness default (2,000,000).
    harness_init(&g_hc, &g_mad2, g_web_recover, 0 /*spin default*/, 16 /*wild warmup*/);
    g_harness_init = 1;
    g_fault_reported = 0;
    return 0;
}

// One emulated instruction plus the harness housekeeping (time-gated HLE pokes,
// per-step timer tick, FIQ (priority) / IRQ delivery, and true-cycle accounting).
// Factored out so the instruction-paced run (dct3_web_run) and the real-time
// cycle-paced run (dct3_web_run_cycles) share identical per-step behaviour.
// True once the firmware has cleanly powered off (CCONT regulators cut) AND we are
// past the boot-settle window — so a transient early-boot watchdog write can never
// self-halt the boot. The web run loop and step both honour this (CPU parks until a
// reboot zeroes g_mad2.power_off).
static int web_parked(void) { return g_mad2.power_off && g_step > PWR_RELEASE_STEP; }

// Task-stub HLE (diagnostic, default OFF): force a handler to "return 1" by catching
// its entry PC and redirecting to LR — for bisecting which task owns the heap leak.
// Set via dct3_web_stub_add (nav --stub). Observationally identical to injecting
// `movs r0,#1; bx lr` at the entry, but without faking the fetch stream.
#define STUB_N 32
static uint32_t g_stub_pc[STUB_N];
static uint32_t g_stub_hits[STUB_N];
static int      g_n_stub = 0;

// Heap-used counter write-watch (diagnostic, default OFF): poll a RAM addr (32-bit BE)
// each step and aggregate, per writer-PC, the net delta + count + an example caller LR.
// Answers "what is growing [0x104844+8]?" — allocator (0x299Axx) vs a stray/corrupting write.
#define WRW_N 64
static uint32_t g_wrw_addr = 0;          // watched addr (0 = off)
static uint32_t g_wrw_last = 0;
static int      g_wrw_init = 0;
static uint32_t g_wrw_pc[WRW_N];
static int32_t  g_wrw_net[WRW_N];
static uint32_t g_wrw_cnt[WRW_N];
static uint32_t g_wrw_lr[WRW_N];
static int      g_wrw_npc = 0;
// Per-size-class net (alloc-count vs free-count keyed by |delta|): names the leaking
// object size directly, tail-call-immune (no ptr pairing needed).
#define WRW_SZ_N 48
static uint32_t g_wrw_sz[WRW_SZ_N];      // size = |delta| (alloc size incl header+4)
static uint32_t g_wrw_sz_a[WRW_SZ_N];    // alloc count (delta > 0)
static uint32_t g_wrw_sz_f[WRW_SZ_N];    // free count  (delta < 0)
static int      g_wrw_sz_n = 0;

static void web_step_once(struct ARMCore* cpu) {
    if (web_parked()) return;            // powered off: stop executing until reboot
    if (g_n_stub) {                      // task-stub: force-return at handler entry
        uint32_t spc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
        for (int k = 0; k < g_n_stub; ++k) if (g_stub_pc[k] == spc) {
            uint32_t lr = (uint32_t)cpu->gprs[ARM_LR];
            cpu->gprs[0] = 1;                                       // "return 1"
            cpu->gprs[ARM_PC] = lr & ~1u;
            if (lr & 1u) { _ARMSetMode(cpu, MODE_THUMB); cpu->cpsr.t = 1; cpu->cycles += ThumbWritePC(cpu); }
            else         { _ARMSetMode(cpu, MODE_ARM);   cpu->cpsr.t = 0; cpu->cycles += ARMWritePC(cpu); }
            g_stub_hits[k]++;
            break;
        }
    }
    if (g_wrw_addr) {                    // heap-counter write-watch (poll-on-change)
        uint32_t a = g_wrw_addr & g_mad2.mem_mask;
        uint32_t v = ((uint32_t)g_mad2.mem[a] << 24) | ((uint32_t)g_mad2.mem[a + 1] << 16)
                   | ((uint32_t)g_mad2.mem[a + 2] << 8) | (uint32_t)g_mad2.mem[a + 3];
        if (!g_wrw_init) { g_wrw_last = v; g_wrw_init = 1; }
        else if (v != g_wrw_last) {
            uint32_t wpc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
            int slot = -1;
            for (int k = 0; k < g_wrw_npc; ++k) if (g_wrw_pc[k] == wpc) { slot = k; break; }
            if (slot < 0 && g_wrw_npc < WRW_N) {
                slot = g_wrw_npc++; g_wrw_pc[slot] = wpc; g_wrw_net[slot] = 0;
                g_wrw_cnt[slot] = 0; g_wrw_lr[slot] = (uint32_t)cpu->gprs[14];
            }
            if (slot >= 0) { g_wrw_net[slot] += (int32_t)(v - g_wrw_last); g_wrw_cnt[slot]++; }
            { int32_t d = (int32_t)(v - g_wrw_last); uint32_t sz = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
              int ss = -1;
              for (int k = 0; k < g_wrw_sz_n; ++k) if (g_wrw_sz[k] == sz) { ss = k; break; }
              if (ss < 0 && g_wrw_sz_n < WRW_SZ_N) { ss = g_wrw_sz_n++; g_wrw_sz[ss] = sz; g_wrw_sz_a[ss] = 0; g_wrw_sz_f[ss] = 0; }
              if (ss >= 0) { if (d > 0) g_wrw_sz_a[ss]++; else g_wrw_sz_f[ss]++; } }
            g_wrw_last = v;
        }
    }
    {
        // Pending firmware CALL: fire in System mode, restore on return.
        if (g_call_pending || g_call_active) {
            uint32_t cpc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
            if (g_call_active && cpc == g_call_ret_pc) {
                g_call_result = (uint32_t)cpu->gprs[0];   // capture return value before restore
                for (int k = 0; k < 15; ++k) cpu->gprs[k] = g_call_save_gpr[k];
                cpu->cpsr.packed = g_call_save_cpsr;
                g_call_active = 0; g_call_count++;
            }
            if (g_call_pending && !g_call_active && cpu->privilegeMode == MODE_SYSTEM) {
                for (int k = 0; k < 15; ++k) g_call_save_gpr[k] = (int32_t)cpu->gprs[k];
                g_call_save_cpsr = cpu->cpsr.packed; g_call_ret_pc = cpc;
                cpu->gprs[0] = g_call_r0; cpu->gprs[1] = g_call_r1;
                cpu->gprs[ARM_LR] = cpc | (cpu->cpsr.t ? 1u : 0u);
                cpu->gprs[ARM_PC] = g_call_fn & ~1u;
                _ARMSetMode(cpu, MODE_THUMB); cpu->cpsr.t = 1;
                cpu->cycles += ThumbWritePC(cpu);
                g_call_pending = 0; g_call_active = 1;
            }
        }
        if (g_dbg_any || g_msglog_pc || g_cap_pc || g_sendlog_on || g_ts_armed || g_seccode_armed || (g_key_oneshot && g_key_armed)) {
            uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
            if (g_ts_armed && pc == g_ts_pc) {        // one-shot reg spike (Type text)
                cpu->gprs[g_ts_reg] = g_ts_val; g_ts_armed = 0; g_ts_count++;
            }
            // Security-code unlock (at the verify call site). Two faithful repairs so the
            // firmware's OWN verify passes 12345 and takes its real accept path:
            //  (1) Re-provision the stored code to encrypt("12345") via the firmware's own
            //      cipher (seccode_encrypt), called IN-CONTEXT here — so it uses the exact
            //      same live digest (FFS 0x6405) that verify is about to use. This is the only
            //      sound way to set a DCT3 code (an out-of-band encrypt uses a stale digest).
            //      Needed because this image's grafted cross-version EEPROM stored a ciphertext
            //      incompatible with the build's digest, so no typed code could ever match.
            //  (2) Repair verify's input string (r0 -> "12345"), defeating the emulation keypad
            //      first-keypress char-null bug that would otherwise null byte0 (strlen 0).
            // verify then computes encrypt("12345")==stored GENUINELY (no forced compare) and
            // returns 1 -> real success path (counter reset, return 1) -> unlock. Persisting so
            // one arm covers the boot lock AND the Restore-Factory-Settings re-prompt.
            if (g_seccode_armed && g_mad2.fw.seccode_verify &&
                pc == (g_mad2.fw.seccode_verify & ~1u)) {
                if (g_mad2.fw.seccode_encrypt && g_mad2.fw.seccode_store) {
                    const uint32_t scratch = 0x00800000u;          // unused-RAM input buffer
                    dct3_write_bytes(g_core, scratch, "12345", 6);
                    seccode_call_sync(g_core, g_mad2.fw.seccode_encrypt, 0, scratch,
                                      g_mad2.fw.seccode_store, 2000000);  // encrypt -> stored
                }
                dct3_write_bytes(g_core, (uint32_t)cpu->gprs[0], "12345", 6);   // verify input
                g_seccode_count++;
            }
            if (g_dbg_any)
                for (int k = 0; k < DBG_NWATCH; ++k)
                    if (g_dbg_pc[k] == pc) g_dbg_cnt[k]++;
            if (g_cap_pc && pc == g_cap_pc) { g_cap_val = (uint32_t)cpu->gprs[0]; g_cap_hits++; }   // capture r0 at PC
            if (g_sendlog_on && (pc == 0x0029921Cu || pc == 0x002997B0u)) {   // send/enqueue(task=r0,msg=r1)
                uint32_t i = g_sendlog_w % SENDLOG_N;
                g_sendlog[i]    = (((uint32_t)cpu->gprs[0] & 0xFFFFu) << 16) | ((uint32_t)cpu->gprs[1] & 0xFFFFu);
                g_sendlog_lr[i] = (uint32_t)cpu->gprs[14] & 0xFFFFFFu;   // caller (sender)
                g_sendlog_w++;
            }
            if (g_msglog_pc && pc == g_msglog_pc) {
                uint32_t idx = g_msglog_w % MSGLOG_N;
                g_msglog[idx]    = (uint32_t)cpu->gprs[0];
                g_msglog_lr[idx] = (uint32_t)cpu->gprs[14];
                g_msglog_w++;
            }
            // Experimental: KEY_DOWN just queued -> release silently very soon.
            if (g_key_oneshot && g_key_seq_state == 1 && !g_key_detected && pc == FW_KEYDOWN_SENT) {
                g_key_detected = 1;
                int64_t want = g_step + KEY_RELEASE_GRACE;
                if (want < g_key_seq_at) g_key_seq_at = want;
            }
        }
        // === Shared harness, pre-step (plan 04 / SC1) ===========================
        // The reset-recovery entry snapshot, the four model-record hooks (reboot /
        // fatal / assert / stage / heap-fail), the wild-PC / spin / CCONT-reset /
        // power-off classify, AND the D-06 instruments (allocator shadow + HEAPGUARD,
        // shadow call stack, last-writer ring, IRQ/CCONT log, invariant checker) all
        // live in src/harness/ now — the SAME entrypoint boot_trace calls. The web
        // driver no longer mirrors its own (drifting) copy. One call, returning a per-
        // step verdict the driver acts on. The de-pipe + every PC compare happen inside.
        HarnessStatus hstatus = harness_observe(&g_hc, &g_mad2, g_core, g_step);
        // WR-03: maintain the consecutive-CONTINUE debounce counter. A FAULT_HALT (whether
        // the new report or a continuing fault still halting every step) restarts the
        // healthy stretch from 0; a CONTINUE step extends it and, once the run is long
        // enough AFTER a prior report, re-arms the one-shot so a DISTINCT later episode is
        // reported. Never cleared on a single CONTINUE step (Pitfall 3).
        if (hstatus == HARNESS_FAULT_HALT) {
            g_fault_continue_run = 0;
        } else if (g_fault_reported) {
            if (++g_fault_continue_run >= FAULT_REARM_CONTINUE) {
                g_fault_reported     = 0;   // episode boundary: a 2nd distinct fault re-reports
                g_fault_continue_run = 0;
            }
        }
        if (hstatus == HARNESS_FAULT_HALT && !g_fault_reported) {
            g_fault_reported = 1;   // one-shot: render the deep report once per fault episode
            g_fault_continue_run = 0;
            // Honest fault halt (D-10/D-11). The web is the live device, so by default
            // it warm-recovers a recoverable reset (g_web_recover=1 -> recover_pending
            // set by the model, applied in harness_post_step) and keeps running — the
            // FAULT_HALT here is only the un-recovered classes (wild-PC / spin / HEAPGUARD
            // / invariant / a reset the model chose not to recover). Surface the SAME
            // unified deep report boot_trace produces (post-mortem + the five instruments)
            // so the in-browser crash panel shows the identical analysis. The model has
            // already rendered m->postmortem at its catch site; the page polls
            // dct3_web_postmortem_buf / dct3_web_reset_request to react.
            harness_fault_report(&g_hc, &g_mad2, g_core, g_hc.fault_reason);
            // Do NOT break the web run loop here: the page drives reboot/park via the
            // existing reset_request / powered_off polls (a headless break would freeze
            // the canvas). The deep report is now attached for the page to read.
        } else if (hstatus == HARNESS_POWERED_OFF) {
            // Clean shutdown — web_parked() already gates execution; no fault dump.
        }
        // Used-bytes curve sampler (web diagnostic, kept driver-side): read the heap-acct
        // used-bytes via the shared allocator-shadow accessor every N steps. Timing uses
        // g_step (NEVER cpu->cycles — it rebases every 0x20000000, Landmine #4).
        if (g_heapcurve_on && (g_step % HEAPCURVE_EVERY) == 0) {
            uint32_t i = g_heapcurve_w % HEAPCURVE_N;
            g_heapcurve_step[i] = (uint32_t)g_step;
            g_heapcurve_used[i] = (uint32_t)heap_shadow_used_d(&g_mad2);
            g_heapcurve_w++;
        }
        // PPM string tracer (get_string / w_get_string): entry captures caller+nokstr, the
        // return reads the resolved text -> appended to g_gs_log for the page to dump.
        if (g_gs_on) {
            uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
            // send_ui_message(0x2E9684): log MSG_DISPLAY_MESSAGE (0x226) sends with their args
            // + caller -> the code that decided to show e.g. "Not charging" (text arg = a 0x562
            // nokstr) is the caller, NOT the generic renderer. (r0=msg, r1/r2=args, lr=caller.)
            if (pc == 0x002E9684u && ((uint32_t)cpu->gprs[0] & 0x0FFFu) == 0x226u
                && g_gs_len < (int)sizeof(g_gs_log) - 96) {
                g_gs_len += snprintf(g_gs_log + g_gs_len, 96, "MSG %04X a0=%X a1=%06X caller=%06X\n",
                                     (uint32_t)cpu->gprs[0] & 0xFFFFu, (uint32_t)cpu->gprs[1],
                                     (uint32_t)cpu->gprs[2], (uint32_t)cpu->gprs[14] & ~1u);
            }
            if (g_mad2.fw.get_string && (pc == g_mad2.fw.get_string || pc == g_mad2.fw.w_get_string)
                && g_gs_ret == 0) {
                g_gs_in = (uint32_t)cpu->gprs[0];
                g_gs_lr = (uint32_t)cpu->gprs[14];
                g_gs_ret = g_gs_lr & ~1u;
                g_gs_wide = (pc == g_mad2.fw.w_get_string);
                // Walk the stack for the call chain so we can see PAST the generic text
                // renderer to the code that decided to show the string. Only keep REAL return
                // addresses (the halfwords at R-4/R-2 must be a 32-bit Thumb BL), which filters
                // out spilled data pointers / blank-flash words that pollute a naive walk.
                uint32_t sp = (uint32_t)cpu->gprs[13]; int sc = 0;
                for (int k = 0; k < 200 && sc < 8; ++k) {
                    uint32_t a = sp + (uint32_t)k * 4;
                    uint32_t w = ((uint32_t)g_core->ram[a & DCT3_RAM_MASK] << 24)
                               | ((uint32_t)g_core->ram[(a + 1) & DCT3_RAM_MASK] << 16)
                               | ((uint32_t)g_core->ram[(a + 2) & DCT3_RAM_MASK] << 8)
                               |  (uint32_t)g_core->ram[(a + 3) & DCT3_RAM_MASK];
                    uint32_t r = w & ~1u;
                    if (r < 0x00200000u || r >= 0x00340000u) continue;
                    uint32_t p1 = (r - 4) & DCT3_RAM_MASK, p2 = (r - 2) & DCT3_RAM_MASK;
                    uint32_t h1 = ((uint32_t)g_core->ram[p1] << 8) | g_core->ram[p1 + 1];
                    uint32_t h2 = ((uint32_t)g_core->ram[p2] << 8) | g_core->ram[p2 + 1];
                    if ((h1 & 0xF800u) == 0xF000u && (h2 & 0xF800u) == 0xF800u)  // preceded by BL
                        g_gs_stk[sc++] = r;
                }
                for (; sc < 8; ++sc) g_gs_stk[sc] = 0;
            } else if (g_gs_ret && pc == g_gs_ret) {
                uint32_t res = (uint32_t)cpu->gprs[0];
                uint8_t b0 = g_core->ram[g_gs_in & DCT3_RAM_MASK];
                uint8_t b1 = g_core->ram[(g_gs_in + 1) & DCT3_RAM_MASK];
                uint8_t b2 = g_core->ram[(g_gs_in + 2) & DCT3_RAM_MASK];
                char txt[48]; int n = 0, zeros = 0;
                for (int i = 0; i < 96 && n < 44; ++i) {
                    uint8_t ch = g_core->ram[(res + (uint32_t)i) & DCT3_RAM_MASK];
                    if (ch == 0) { if (++zeros >= 2) break; continue; }
                    zeros = 0; txt[n++] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.';
                }
                txt[n] = 0;
                uint32_t key = ((uint32_t)((b0 == 0x04 && b1 != 0xFF) ? (b1 << 8) | b2 : 0) << 8) ^ g_gs_lr;
                if (key != g_gs_last && g_gs_len < (int)sizeof(g_gs_log) - 220) {  // dedup consecutive repeats
                    g_gs_last = key;
                    char chain[80]; int cl = 0;
                    for (int s = 0; s < 8 && g_gs_stk[s]; ++s)
                        cl += snprintf(chain + cl, sizeof(chain) - cl, " %06X", g_gs_stk[s]);
                    chain[cl] = 0;
                    if (b0 == 0x04 && b1 != 0xFF)
                        g_gs_len += snprintf(g_gs_log + g_gs_len, 220, "%sid=%04X caller=%06X via%s \"%s\"\n",
                                             g_gs_wide ? "w " : "  ", (b1 << 8) | b2, g_gs_lr, chain, txt);
                    else
                        g_gs_len += snprintf(g_gs_log + g_gs_len, 220, "%sascii caller=%06X via%s \"%s\"\n",
                                             g_gs_wide ? "w " : "  ", g_gs_lr, chain, txt);
                }
                g_gs_ret = 0;
            }
        }
        // RTC-writer attribution: at the CCONT masked-write entry (0x2EA750: r0=(reg<<8)
        // |mask, r1=value), capture the EXTERNAL caller (LR) for writes to the RTC time
        // regs (Nokia indices 0x08..0x0C = sec..day) -> finds the clock task driving them.
        {
            uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
            if (pc == 0x002EA750u) {
                uint32_t nreg = ((uint32_t)cpu->gprs[0] >> 8) & 0xFFu;
                if (nreg >= 0x08u && nreg <= 0x0Cu) g_rtc_wr_lr = (uint32_t)cpu->gprs[14];
            }
            // every CCONT write-reg helper call -> ring (find the RTC writer).
            if (pc == 0x002E98D4u) {
                uint32_t i = g_ccwr_w % CCWR_N;
                g_ccwr_lr[i]  = (uint32_t)cpu->gprs[14];
                g_ccwr_reg[i] = ((uint32_t)cpu->gprs[0] >> 8) & 0x7Fu;   // logical reg
                g_ccwr_val[i] = (uint32_t)cpu->gprs[1] & 0xFFu;
                g_ccwr_w++;
            }
        }
        // Boot spike: arm the instant the firmware first writes the 0xC8 self-test
        // verdict (RAM was zeroed at boot, so the first 0xC8 IS the verdict). Once armed:
        //   - hold the SIM-gate bypass at the signature-derived address (all firmware);
        //   - pin the verdict at 0xC8 ONLY for firmware that needs the HLE compensation
        //     (g_pin_verdict, set per-firmware in resolve_spike). Genuine retail firmware
        //     (v5.79) runs its self-test organically and must NOT be pinned, or the frozen
        //     verdict + the faithful FIQ5 soft-timer walk overflow the dispatcher stack and
        //     reboot/spin at 0x2EEBEC. The FuBu v6.39 image needs the pin to reach its UI.
        uint32_t verdict_addr = g_mad2.fw.verdict & DCT3_RAM_MASK;
        if (!g_spike_armed && g_core->ram[verdict_addr] == 0xC8) g_spike_armed = 1;
        if (g_spike_armed) {
            if (g_pin_verdict) g_core->ram[verdict_addr] = 0xC8;
            if (g_bypass_sim) g_core->ram[g_sim_addr & DCT3_RAM_MASK] = 0x06;
        }
        // "Skip security code" (default OFF): make the FuBu v6.39 FAID integrity
        // check pass legitimately so the disp77 anti-theft lock never arms (see the
        // FAID note above). Hold the stored sum at the computed value while on.
        // (No-op on images without a known FAID address — faid_cksum == 0.)
        if (g_skip_seclock && g_mad2.fw.faid_cksum) {
            uint32_t fa = g_mad2.fw.faid_cksum & DCT3_RAM_MASK;
            g_core->ram[fa]                       = (g_mad2.fw.faid_cksum_val >> 8) & 0xFF;
            g_core->ram[(fa + 1) & DCT3_RAM_MASK] = g_mad2.fw.faid_cksum_val & 0xFF;
        }
        if (g_pwr_auto && !g_pwr_released && g_step >= PWR_RELEASE_STEP) {
            g_mad2.kbd_special_cols = 0;       // release the held power key
            g_mad2.irq_pending |= 0x01;        // keypad IRQ so the up-edge is seen
            g_pwr_released = 1;
        }
        // Key-press sequencer: play queued taps atomically over emulated time.
        //   state 0 (idle):     if a tap is queued, deliver its key-down + IRQ,
        //                       schedule the up at +g_key_hold_insns (past grace).
        //   state 1 (holding):  at the scheduled time, release (matrix clear + IRQ
        //                       so the firmware runs a no-key scan and processes
        //                       the RELEASE — 0x2D84D0 re-issues MSG_KEY_DOWN from
        //                       stored state, so a silent clear alone won't stop a
        //                       repeat), then settle.
        //   state 2 (settling): wait KEY_SETTLE_INSNS so the firmware fully
        //                       digests the release before the next down -> no
        //                       collision, then go idle and pick up the next tap.
        if (g_key_seq_state == 0) {
            if (g_keyq_head != g_keyq_tail) {           // a tap is waiting
                g_key_row = g_keyq[g_keyq_head].row;
                g_key_col = g_keyq[g_keyq_head].col;
                g_keyq_head = (g_keyq_head + 1) % KEY_Q_N;
                g_mad2.kbd_norm_cols[g_key_row & 7] |= (uint8_t)(1u << (g_key_col & 7));
                mad2_keypad_irq(&g_mad2);   // matrix edge (8850-class source bit too)
                g_key_armed = 1; g_key_detected = 0;
                g_key_seq_state = 1;
                g_key_seq_at = g_step + g_key_hold_insns;
            }
        } else if (g_key_seq_state == 1) {
            if (g_step >= g_key_seq_at) {
                g_mad2.kbd_norm_cols[g_key_row & 7] &= (uint8_t)~(1u << (g_key_col & 7));
                mad2_keypad_irq(&g_mad2);
                g_key_armed = 0; g_key_detected = 0;
                g_key_seq_state = 2;
                g_key_seq_at = g_step + KEY_SETTLE_INSNS;
            }
        } else { /* state 2 */
            if (g_step >= g_key_seq_at) { g_key_seq_state = 0; g_key_seq_at = -1; }
        }
        // record this step's PC; freeze the ring once we branch into the
        // low/null range so the trail into the bad branch survives. Strip before commit.
        if (!g_pcring_frozen) {
            uint32_t raw  = (uint32_t)cpu->gprs[15];
            uint32_t cpsr = cpu->cpsr.packed;
            uint32_t pc   = raw - ((cpsr & 0x20u) ? 4u : 8u);   // de-pipe by the T bit
            int mode = (int)(cpsr & 0x1Fu);
            uint32_t idx = g_pcring_w % PCRING_N;
            g_pcring[idx] = raw;
            g_pcring_cpsr[idx] = cpsr;
            g_pcring_w++;
            g_pcring_step++;
            if (pc < 0x00100000u) {
                // The exact anomaly: a low PC (a vector) reached while in FIQ mode but NOT
                // at the FIQ vector 0x1c. Legit FIQ enters 0x1c (FIQ mode) and branches
                // high; legit IRQ enters 0x18 in IRQ mode. The crash is at the IRQ vector
                // 0x18 (or marching) while still in FIQ mode → a vector/mode mismatch.
                if (mode == MODE_FIQ && pc != 0x1cu) g_pcring_frozen = 1;
                // Fallback: a low-PC cluster (rapid vector re-entry) for any non-FIQ stuck state.
                g_pcring_lowcnt = (g_pcring_step - g_pcring_last_low < 150) ? g_pcring_lowcnt + 1 : 1;
                g_pcring_last_low = g_pcring_step;
                if (g_pcring_lowcnt >= 6) g_pcring_frozen = 1;
            }
        }
        int32_t cyc_before = cpu->cycles;
        dct3_step(g_core);
        int32_t cyc_delta = cpu->cycles - cyc_before;
        if (cyc_delta < 0) cyc_delta += DCT3_CYCLE_REBASE;   // undo the periodic rebase
        g_cycles64 += (uint32_t)cyc_delta;
        mad2_timers_tick(&g_mad2, (uint32_t)cpu->cycles);
        // === Shared harness, post-step (plan 04 / SC1) ==========================
        // Applies a pending recover (dct3_core_force_pc_cpsr) AFTER the step + tick and
        // BEFORE the IRQ/FIQ raise — the CANONICAL web ordering this codebase converged
        // on (so the spinning `b .` runs once and the resume mode sticks). It also drives
        // the shadow call stack push/pop (telemetry_post_branch) for the exact backtrace.
        // The web's own recover-apply + typed branch ring used to live inline here; both
        // moved into src/harness/ (no reimplementation). Gated by recovery policy — a
        // no-op when recover_pending is clear.
        harness_post_step(&g_hc, &g_mad2, g_core);
        int raised = 0;
        if (!cpu->cpsr.f) {
            int fiq = mad2_fiq_poll(&g_mad2);
            if (fiq >= 0) {
                ARMRaiseFIQ(cpu); g_mad2.fiqs_raised++; raised = 1;
                telemetry_event_irq(&g_mad2, fiq, 1, g_step);   // shared IRQ/CCONT log (D-06.5)
            }
        }
        if (!raised && !cpu->cpsr.i) {
            int irq = mad2_irq_poll(&g_mad2);
            if (irq >= 0) {
                ARMRaiseIRQ(cpu); g_mad2.irqs_raised++;
                telemetry_event_irq(&g_mad2, irq, 0, g_step);   // shared IRQ/CCONT log (D-06.5)
            }
        }
        g_step++;
        // (The typed branch-trace ring moved into src/harness/telemetry.c as the shadow
        // call stack — driven by harness_post_step above. No web-side reimplementation.)
        // Divergence-finder sample: take a snapshot every TRACE_CYC_INTERVAL cycles.
        // Captures the state JUST AFTER an instruction's IRQ/FIQ delivery, so two runs
        // that diverged in interrupt timing will show different (pc, cpsr) at the same
        // cyc threshold. Sample-after-step matches what the halt button captures.
        if (g_trace_on && g_cycles64 >= g_trace_next_cyc) {
            TraceEntry* e = &g_trace[g_trace_w & (TRACE_N - 1)];
            e->cyc  = g_cycles64;
            e->step = (uint64_t)g_step;
            e->pc   = (uint32_t)cpu->gprs[ARM_PC];
            e->cpsr = (uint32_t)cpu->cpsr.packed;
            e->fiq_pending = g_mad2.fiq_pending;
            e->irq_pending = g_mad2.irq_pending;
            g_trace_w++;
            // Schedule the NEXT sample at the next interval boundary (not cur + interval,
            // so two runs hitting slightly different cycle counts still land on the same grid).
            g_trace_next_cyc = ((g_cycles64 / TRACE_CYC_INTERVAL) + 1) * TRACE_CYC_INTERVAL;
        }
    }
}

// Run n instructions (instruction-paced). Used for the flat-out boot fast-forward
// and headless node harnesses; emulated-clock speed then tracks cpu cpi.
EMSCRIPTEN_KEEPALIVE
void dct3_web_run(int n) {
    if (!g_core) return;
    struct ARMCore* cpu = &g_core->cpu;
    for (int i = 0; i < n && !web_parked(); ++i) web_step_once(cpu);
    emu_audio_render(&g_mad2);   // flush the buzzer voice up to now (onset already stamped at writes)
}

// Run until ~target_cycles CPU cycles have elapsed (real-time pacing). This pins
// the emulated clock to a fixed cycle rate per wall-frame regardless of the
// instruction mix's cycles-per-instruction, so every cycle-derived timer (FIQ4
// scheduler tick, Timer0, the sleep counter) — and thus the firmware's tick-based
// delays (e.g. v6.39 Snake's move cadence) — runs at the correct, steady speed.
// (Instruction-paced runs make the effective clock = 13MHz only when cpi matches
// the assumed value; cpi is ~3 here, so 150k insns/frame ran the phone ~2x fast.)
EMSCRIPTEN_KEEPALIVE
void dct3_web_run_cycles(int target_cycles) {
    if (!g_core || target_cycles <= 0) return;
    struct ARMCore* cpu = &g_core->cpu;
    uint64_t until = g_cycles64 + (uint64_t)target_cycles;
    // Safety cap: a pathological low-cpi tight loop shouldn't spin unbounded.
    int guard = 20000000;
    while (g_cycles64 < until && guard-- > 0 && !web_parked()) web_step_once(cpu);
    emu_audio_render(&g_mad2);   // flush the buzzer voice up to now (onset already stamped at writes)
}

// True elapsed CPU cycles since boot (rebase-corrected). Lets the page measure the
// real emulated-clock rate and lets harnesses compute cycles-per-instruction.
EMSCRIPTEN_KEEPALIVE
double dct3_web_cycles(void) { return (double)g_cycles64; }

// Pointer to the 504-byte PCD8544 framebuffer (6 banks x 84 cols). Pixel (x,y):
// bank = y>>3, on = (fb[bank*84 + x] >> (y&7)) & 1  (on = dark).
EMSCRIPTEN_KEEPALIVE
uint8_t* dct3_web_fb(void) { return g_mad2.fb; }

// Detected model name + active LCD geometry (so the page can label the device and
// size the canvas for non-3310 profiles). Valid after dct3_web_boot().
EMSCRIPTEN_KEEPALIVE
const char* dct3_web_model(void) { return g_model ? g_model->name : ""; }
// Top of the mapped flash code window (flash_base + flash_size). The JS-side wild-PC /
// backtrace logic must use THIS, not a hardcoded 0x400000 — 4 MB models (5210/7110/6210)
// legitimately execute in the upper bank up to 0x600000, so a fixed 2 MB ceiling false-trips
// every call above 0x400000 as a wild-PC. Mirrors the model-aware C detector (fault.c).
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_flash_hi(void) { return g_model ? g_model->mem.flash_base + g_model->mem.flash_size : 0x00400000u; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_lcd_w(void) { return g_model ? g_model->lcd.width : 84; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_lcd_h(void) { return g_model ? g_model->lcd.height : 48; }
// LCD bank count (ceil(height/8)); the fb is flat, stride = lcd.width, `banks` rows
// (mad2.c). The page uses w/banks to size the canvas + unpack the framebuffer for
// non-3310 geometries (e.g. 3410 = 96x65, 9 banks).
EMSCRIPTEN_KEEPALIVE
int dct3_web_lcd_banks(void) { return g_model ? g_model->lcd.banks : 6; }
// Keypad visual-layout family (KeypadFamily: 0=3310/B, 1=8210/A, 2=3410/C). The page
// renders the matching on-screen layout (soft keys / send-end / volume) per family.
EMSCRIPTEN_KEEPALIVE
int dct3_web_kp_family(void) { return g_model ? (int)g_model->keypad.family : 0; }

EMSCRIPTEN_KEEPALIVE
double dct3_web_step(void) { return (double)g_step; }

// Clean power-off latch. mad2 sets power_off=1 when the firmware runs its shutdown
// sequence (ccont_poweroff writes the CCONT watchdog / reg05 = 0x00 to cut the
// regulators) — the faithful end of a power-button hold AND the real NVRAM-flush
// point. The web run loop parks the CPU once this is set (see web_step_once); the
// page polls this to show "Powered off" and to snapshot the flushed NVRAM. Cleared
// by dct3_web_boot (g_mad2 is zeroed) so a reboot powers back on.
EMSCRIPTEN_KEEPALIVE
int dct3_web_powered_off(void) { return g_core ? (int)g_mad2.power_off : 0; }

// Firmware software-reset/reboot request: mad2 sets reset_request=1 when the firmware
// writes [0x20001] bit2 (the ASIC SW-reset line) and spins at `b .` waiting for the CPU
// to reset — e.g. the end of a factory reset. The host polls this and warm-reboots.
EMSCRIPTEN_KEEPALIVE
int dct3_web_reset_request(void) { return g_core ? (int)g_mad2.reset_request : 0; }

// Reset-reason counters / last-event accessors. nav.mjs uses these to label every
// catch (RECOVERED vs WARM-REBOOT vs LOG-ONLY) instead of pinning a PC/LR. The reason
// byte travels through m->fw.reboot_reason so this works across firmware builds.
EMSCRIPTEN_KEEPALIVE
int dct3_web_reset_last_reason(void) { return g_core ? (int)g_mad2.reset_last_reason : 0; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_reset_total(void) { return g_core ? (double)g_mad2.reset_total : 0; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_reset_recovered(void) { return g_core ? (double)g_mad2.reset_recovered : 0; }
// reason 0..15 → per-reason count (slot 15 holds the clamped count for any reason ≥15).
EMSCRIPTEN_KEEPALIVE
double dct3_web_reset_count(int reason) {
    if (!g_core || reason < 0 || reason > 15) return 0;
    return (double)g_mad2.reset_counts[reason];
}
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_reset_last_pc(void) { return g_core ? g_mad2.reset_last_pc : 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_reset_last_cpsr(void) { return g_core ? g_mad2.reset_last_cpsr : 0; }

// CPU register access for crash/spin deep-diag: i = 0..15 → gprs (13=SP,14=LR,15=PC),
// 16 → CPSR. Lets the page build a backtrace (PC/LR + a stack walk) on a detected spin.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_reg(int i) {
    if (!g_core) return 0;
    if (i >= 0 && i < 16) return (uint32_t)g_core->cpu.gprs[i];
    if (i == 16) return g_core->cpu.cpsr.packed;
    return 0;
}

// Warm reboot — honour a firmware reset request faithfully: KEEP flash (code) and the
// EEPROM/NVRAM partition (both non-volatile; the EEPROM holds whatever the firmware just
// wrote, e.g. a factory reset), CLEAR working RAM, reset the CPU/stack, and re-enter boot.
// Unlike dct3_web_boot (a cold power-on that reloads /fw.fls and so would discard the
// just-written NVRAM), this preserves the live flash so the reset result survives.
EMSCRIPTEN_KEEPALIVE
int dct3_web_warm_reset(void) {
    if (!g_core || !g_model) return -1;
    dct3_core_reset(g_core);                          // reset CPU registers + stack
    memset(g_core->ram, 0, g_model->mem.flash_base);  // clear RAM below flash; keep flash+EEPROM
    memset(&g_mad2, 0, sizeof g_mad2);                 // clears reset_request / power_off / etc.
    mad2_init(&g_mad2, g_model);
    g_mad2.sim_present = 0;
    g_mad2.verbose = 0;
    g_mad2.mem = g_core->ram;
    g_mad2.mem_mask = DCT3_RAM_MASK;
    resolve_spike();                                  // flash code is intact, so sigs still resolve
    dct3_set_io_hooks(g_core, &g_mad2, g_model->mem.io_lo, g_model->mem.io_hi, mad2_read, mad2_write);
    dct3_set_io_range2(g_core, g_model->mem.flash_base,
                       g_model->mem.flash_base + g_model->mem.flash_size);
    dct3_boot_at(g_core, g_model->mem.boot_entry);
    g_step = 0; g_cycles64 = 0;
    g_pwr_released = 0; g_pwr_auto = 1;
    g_key_armed = 0; g_key_row = g_key_col = -1; g_key_detected = 0;
    g_keyq_head = g_keyq_tail = 0; g_key_seq_state = 0; g_key_seq_at = -1;
    g_pcring_w = 0; g_pcring_frozen = 0; g_pcring_step = 0; g_pcring_last_low = 0; g_pcring_lowcnt = 0;   // re-arm the crash trail
    g_trace_w = 0; g_trace_next_cyc = 0;   // divergence-finder ring resets per (re)boot
    // NOTE: the old web-only EEPROM-write gate was REMOVED. It blocked the
    // firmware's boot-time FFS init writes (CFI reported success, array unchanged), leaving the
    // FFS half-initialised — NVRAM saves (Nokia Club ID) wrote garbage and in-flash app launch
    // (Snake 2) hung in the FFS readiness check. Native never gated flash writes; the web now
    // matches it. Any cold-boot persistence crash must be fixed in the NVRAM snapshot path, not
    // by dropping live flash writes. (git history has the gate if ever needed.)
    // Init the shared harness (plan 04). The web is the live-device presentation, so it
    // keeps recovery ON by default (warm-recovers a recoverable reset in-browser) — the
    // CLI flips it OFF (D-10 honest halt). harness_init mirrors the recovery flag onto
    // m->recover_enabled and arms the wild-PC / spin detectors with the 16-step boot
    // warmup (the boot-ROM HLE de-pipe artifact). SPINLIMIT/wild-PC use the defaults.
    memset(&g_hc, 0, sizeof g_hc);
    // wild_pc_after = 16: the boot-ROM HLE de-pipe leaves a transient PC=0xFFFFFFFC at
    // step 0 (>= 0x400000) that would false-fire wild-PC on sight — the SAME 16-step
    // warmup boot_trace uses. spin_limit 0 => the harness default (2,000,000).
    harness_init(&g_hc, &g_mad2, g_web_recover, 0 /*spin default*/, 16 /*wild warmup*/);
    g_harness_init = 1;
    g_fault_reported = 0;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
double dct3_web_lcd_writes(void) { return (double)g_mad2.lcd_data_writes; }

// Backlight LED state: bit0 = LCD LEDs (McuGenIO 0x20 bit3), bit1 = keypad LEDs
// (CTRL-I/O-3 0x33 bit1). The page renders the LCD glow + keypad button backlight.
EMSCRIPTEN_KEEPALIVE
int dct3_web_leds(void) { return emu_led_bits(&g_mad2); }

// Backlight LED colour, 0xRRGGBB from the model profile (period-correct: 5210
// orange, 8250 blue, ...). which: 0 = LCD glow, 1 = keypad buttons (falls back
// to the LCD colour). 0 = no profile colour — page keeps its classic
// yellow-green palette. (host-interface outputs).
EMSCRIPTEN_KEEPALIVE
int dct3_web_led_rgb(int which) {
    return (int)emu_led_rgb(&g_mad2, which);
}

// PCD8544 display-control mode (0=blank, 1=all-on, 2=normal, 3=inverse). The page
// applies this whole-screen transform when presenting the framebuffer, so blank/
// inverse during screen transitions render faithfully instead of holding stale DDRAM.
EMSCRIPTEN_KEEPALIVE
int dct3_web_lcd_mode(void) { return g_mad2.lcd_mode; }

// Buzzer / vibra (PUP/PWM registers, observed in mad2). The page drives a Web Audio
// square oscillator: freq = 13 MHz / divider (confirmed on hardware — the divider is
// 16-bit, e.g. div=4844 -> 2684 Hz), gated by the enable bit.
EMSCRIPTEN_KEEPALIVE
int dct3_web_buzzer_on(void)  { return g_mad2.buzz_on; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_buzzer_div(void) { return g_mad2.buzz_div; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_buzzer_vol(void) { return g_mad2.buzz_vol; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_vibra_on(void)   { return g_mad2.vibra_on; }

// Buzzer chirp latch: rising-edge count (low byte) packed with the divider snapshot
// at the last edge (high 16 bits). Read-and-clear. (Legacy level-poll fallback; the
// unified PCM path below is now the primary buzzer voice.)
EMSCRIPTEN_KEEPALIVE
int dct3_web_buzzer_chirp(void) { return emu_buzzer_chirp_drain(&g_mad2); }

// Unified PCM audio: each frame the page calls dct3_web_pcm_read(max) to drain up to
// `max` int16 earpiece samples (buzzer + DSP codec, at dct3_web_pcm_rate) into a static
// staging buffer, then reads them from dct3_web_pcm_ptr() via HEAP16 — the same PCM the
// SDL GUI plays. Buzzer onset is sample-accurate (stamped at the register write) instead
// of frame-polled. (Pointer/count model mirrors dct3_web_fb — no _malloc needed.)
#define WEB_PCM_OUT 4096
static int16_t g_web_pcm_out[WEB_PCM_OUT];
EMSCRIPTEN_KEEPALIVE
int dct3_web_pcm_read(int max) {
    if (max > WEB_PCM_OUT) max = WEB_PCM_OUT;
    int n = 0;
    while (n < max && g_web_pcm_tail != g_web_pcm_head) {
        g_web_pcm_out[n++] = g_web_pcm[g_web_pcm_tail & WEB_PCM_MASK];
        g_web_pcm_tail++;
    }
    return n;
}
EMSCRIPTEN_KEEPALIVE
int16_t* dct3_web_pcm_ptr(void) { return g_web_pcm_out; }
// Producer sample rate for the PCM stream (Hz) — the page resamples this to the
// AudioContext rate. Codec runs set it from the DSP; the buzzer defaults to DCT3_CODEC_HZ.
EMSCRIPTEN_KEEPALIVE
int dct3_web_pcm_rate(void) { return (int)emu_pcm_rate(&g_mad2); }

// DSP tone generator (the keypad/UI beep + DTMF — earpiece path, not the buzzer). The
// MCU programs the DSP via the HPI mailbox; the tone player writes oscillator frequencies
// and amplitude (reg 0x25) as big-endian 16-bit, then commits + signals the DSP.
// The 4 oscillators map linearly: reg 0x21+n -> [0x100AE + 2n] (confirmed: reg 0x21 =
// [0x100AE], amp reg 0x25 = [0x100B6], 4 regs/8 bytes apart). DTMF lights TWO:
// osc1 = column/high freq, osc2 = row/low freq; a plain UI beep uses osc1 only.
// ★ The freq register is in QUARTER-Hz units: freq_Hz = reg / 4 (verified in-browser
// against standard DTMF — col regs/4 = 1209/1336/1477, row regs/4 = 697/770/852/941;
// the keypad beep reg 0x0E10 -> 900 Hz). Silent when amplitude == 0.
#define DCT3_TONE_FREQ_ADDR  0x100AEu   // osc1 freq reg 0x21 (DTMF high / column)
#define DCT3_TONE_FREQ2_ADDR 0x100B0u   // osc2 freq reg 0x22 (DTMF low / row)
#define DCT3_TONE_AMP_ADDR   0x100B6u   // amplitude reg 0x25
static int dct3_tone_reg16(uint32_t a) {
    if (!g_core) return 0;
    return ((int)g_core->ram[a & DCT3_RAM_MASK] << 8)
         |  (int)g_core->ram[(a + 1u) & DCT3_RAM_MASK];   // big-endian core
}
static int dct3_tone_hz_at(uint32_t a) {
    if (dct3_tone_reg16(DCT3_TONE_AMP_ADDR) == 0) return 0;   // gated by amplitude
    int reg = dct3_tone_reg16(a);
    if (reg == 0) return 0;
    return reg >> 2;                                          // reg is in 1/4-Hz units
}
EMSCRIPTEN_KEEPALIVE
int dct3_web_tone_amp(void) { return dct3_tone_reg16(DCT3_TONE_AMP_ADDR); }
EMSCRIPTEN_KEEPALIVE
int dct3_web_tone_hz(void)  { return dct3_tone_hz_at(DCT3_TONE_FREQ_ADDR); }   // osc1
EMSCRIPTEN_KEEPALIVE
int dct3_web_tone_hz2(void) { return dct3_tone_hz_at(DCT3_TONE_FREQ2_ADDR); }  // osc2 (DTMF pair)

// Battery (CCONT A/D ch2 Vbatt) and charger (ch5) — let the page set the raw 10-bit
// ADC reading so a UI slider/toggle drives the on-screen battery bars / charge state.
// Full battery ~0x2C0; charger absent = 0. Charger-detect lives at firmware 0x2E8CFC.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_battery(int adc) { g_mad2.adc[2] = (uint16_t)(adc & 0x3FF); }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_battery(void)    { return g_mad2.adc[2]; }
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_charger(int adc) { g_mad2.adc[5] = (uint16_t)(adc & 0x3FF); }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_charger(void)    { return g_mad2.adc[5]; }

// SIM card present/absent toggle (modeled GSM test SIM). Setting present=1 makes the
// SIMI UART deliver an ATR on the firmware's next card activate (and FIQ7 fires on
// the insert/remove edge); the firmware's SIM driver then runs its ATR + EF reads.
// dct3_web_sim_apdus() exposes the count of APDUs the SIM model has answered, so the
// page can confirm the SIM session is live. (default present; see mad2_init.)
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_sim(int present) { g_mad2.sim_present = present ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_sim(void)        { return g_mad2.sim_present; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_sim_apdus(void)     { return (double)g_mad2.sim_apdus; }

// SIM CHV1 (PIN) configuration + state, mirroring set_sim. The PIN/PUK are passed as
// integers (e.g. 1234) and stored ASCII, 0xFF-padded to 8 (GSM 11.11). With the PIN
// enabled and unverified the SIM advertises CHV1-required (FCP) and refuses protected
// EF reads, so the firmware shows "Enter PIN"; the firmware's VERIFY CHV against the
// stored PIN clears the gate. dct3_web_sim_pin_state() packs the runtime state for the
// page: bit0 enabled, bit1 verified, bits8-11 PIN tries left, bits12-15 PUK tries.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_sim_pin_enabled(int en) { g_mad2.sim_pin_enabled = en ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_sim_pin(int pin) {        // set the stored CHV1 from an integer
    char b[16]; int n = snprintf(b, sizeof b, "%d", pin < 0 ? 0 : pin);
    for (int i = 0; i < 8; ++i) g_mad2.sim_pin[i] = (i < n) ? (uint8_t)b[i] : 0xFF;
    g_mad2.sim_pin_verified = 0; g_mad2.sim_pin_tries = 3;
}
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_sim_puk(int puk) {        // set the stored unblock code
    char b[16]; int n = snprintf(b, sizeof b, "%d", puk < 0 ? 0 : puk);
    for (int i = 0; i < 8; ++i) g_mad2.sim_puk[i] = (i < n) ? (uint8_t)b[i] : 0xFF;
    g_mad2.sim_puk_tries = 10;
}
EMSCRIPTEN_KEEPALIVE
int dct3_web_sim_pin_state(void) {
    return (g_mad2.sim_pin_enabled ? 1 : 0)
         | (g_mad2.sim_pin_verified ? 2 : 0)
         | ((g_mad2.sim_pin_tries & 0xF) << 8)
         | ((g_mad2.sim_puk_tries & 0xF) << 12);
}

// CCONT interrupt cascade registers (reg 0x0F mask / reg 0x0E pending lines), for
// observing whether the firmware unmasks the RTC interrupts (INT4-7, bits 4-7) once
// the time is set. At boot the firmware writes mask 0xF0 (only CHARGER/INT3 passed);
// if setting the time clears bit4, the clock IS interrupt-driven and we should raise it.
EMSCRIPTEN_KEEPALIVE
int  dct3_web_ccont_mask(void)     { return g_mad2.cc_int_mask; }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_ccont_lines(void)    { return g_mad2.cc_int_lines; }

// Our model's current RTC time in total seconds (base + elapsed), and the count of
// RTC-MIN (INT5) interrupts raised. Diagnostics: confirms our clock advances and the
// minute interrupt actually fires, vs. whether the firmware reacts to it.
EMSCRIPTEN_KEEPALIVE
int    dct3_web_rtc_now(void) {
    return (int)(g_mad2.rtc_base_sec + (uint32_t)((g_mad2.rtc_mono - g_mad2.rtc_base_cyc) / 13000000u));
}
EMSCRIPTEN_KEEPALIVE
double dct3_web_rtc_min_edges(void) { return (double)g_mad2.rtc_min_edges; }
// Count of firmware writes to the RTC regs (climbing = the firmware re-writes the clock
// continuously) and the raw CCONT RTC bytes it wrote (sec | min<<8 | hour<<16 | day<<24).
EMSCRIPTEN_KEEPALIVE
double dct3_web_rtc_writes(void) { return (double)g_mad2.rtc_writes; }
// read ring entry k (0..23, newest-first via w). Returns packed
// lr (low24) | reg<<24; val via the _val accessor. Strip before commit.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_ccwr_w(void) { return g_ccwr_w; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_ccwr_lr(int i)  { return g_ccwr_lr[i % CCWR_N]; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_ccwr_reg(int i) { return g_ccwr_reg[i % CCWR_N]; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_ccwr_val(int i) { return g_ccwr_val[i % CCWR_N]; }
EMSCRIPTEN_KEEPALIVE
int    dct3_web_rtc_raw(void) {
    return g_mad2.ccont[0x07] | (g_mad2.ccont[0x08] << 8) | (g_mad2.ccont[0x09] << 16) | (g_mad2.ccont[0x0A] << 24);
}
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_rtc_wr_pc(void) { return g_mad2.rtc_wr_pc; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_rtc_wr_lr(void) { return g_rtc_wr_lr; }

// PPM string tracer controls. dct3_web_getstr_on(1) clears the log and starts tracing every
// get_string/w_get_string call; reproduce a screen, then dct3_web_getstr_dump() returns the
// accumulated "id/ascii caller text" lines (UTF8ToString in JS). on(0) stops.
EMSCRIPTEN_KEEPALIVE
void dct3_web_getstr_on(int en) {
    g_gs_on = en ? 1 : 0; g_gs_ret = 0; g_gs_last = 0;
    if (en) { g_gs_len = 0; g_gs_log[0] = 0; }
}
EMSCRIPTEN_KEEPALIVE
const char* dct3_web_getstr_dump(void) { return g_gs_log; }

// Keypad: ENQUEUE a tap (down=1) for the run-loop sequencer to play atomically.
// The sequencer drives the matrix + keypad IRQ (bit0, the wake-on-keypress edge,
// ISR 0x2E8C10 -> scan) so each tap is: clean release of the previous key + a
// settle, key-down, hold past the firmware's repeat grace (g_key_hold_insns,
// ~5M), clean key-up, settle. Because it plays out in EMULATED time, the tap is
// independent of how fast/slow the JS calls arrive: a burst is queued and played
// one-at-a-time (no stomping -> no "stuck on home"); a slow gap doesn't turn the
// hold into an exit-the-menu long-press (-> no "revert to home"). The physical
// key-UP (down=0) is ignored: the sequencer owns the up-edge timing. (Headless
// raw control with manual up edges: dct3_web_key_raw.) See SESSION 12 / the
// queue note above.
EMSCRIPTEN_KEEPALIVE
void dct3_web_key(int row, int col, int down) {
    if (!g_core || !down) return;
    int next = (g_keyq_tail + 1) % KEY_Q_N;
    if (next == g_keyq_head) return;            // queue full: drop (don't overrun)
    g_keyq[g_keyq_tail].row = row; g_keyq[g_keyq_tail].col = col;
    g_keyq_tail = next;
}

// Send a LOGICAL key (KeyId from model.h) — resolves to THIS model's (row,col) via
// the profile keypad matrix (HW truth in src/models/<m>/profile.c), then enqueues a
// tap exactly like dct3_web_key. The web UI stays model-agnostic: it sends KK_1 /
// KK_SOFT1 / KK_SEND / … and the active profile decides the matrix lines + which keys
// exist on the device. Special-scan keys (PWR) assert kbd_special_cols directly (the
// caller drives the up edge). Returns 1 if the key exists on this model, else 0.
EMSCRIPTEN_KEEPALIVE
int dct3_web_key_logical(int key_id, int down) {
    if (!g_core || !g_model) return 0;
    const KeyLine* k = emu_keyline(g_model, key_id);   // shared KK_*->matrix lookup (EmuHost)
    if (!k) return 0;                                  // key not present on this model
    if (k->special) {                                  // PWR-class special-scan (no auto-release)
        emu_key_special(&g_mad2, k->special, down);
        return 1;
    }
    // Normal keys: web INPUT POLICY = enqueue for the auto-release sequencer (timing is
    // harness-specific; the façade shares only the lookup + special-scan primitive).
    if (!down) return 1;
    int next = (g_keyq_tail + 1) % KEY_Q_N;
    if (next == g_keyq_head) return 1;      // queue full: drop
    g_keyq[g_keyq_tail].row = k->row; g_keyq[g_keyq_tail].col = k->col;
    g_keyq_tail = next;
    return 1;
}

// Raw keypad control (no auto-release) — for headless timing experiments. Sets
// or clears a matrix position and pulses the keypad IRQ; the caller drives the
// up edge explicitly. Disarms any pending auto-release.
EMSCRIPTEN_KEEPALIVE
void dct3_web_key_raw(int row, int col, int down) {
    if (!g_core) return;
    if (down) g_mad2.kbd_norm_cols[row & 7] |= (uint8_t)(1u << (col & 7));
    else      g_mad2.kbd_norm_cols[row & 7] &= (uint8_t)~(1u << (col & 7));
    mad2_keypad_irq(&g_mad2);   // matrix edge (8850-class source bit too)
    g_key_armed = 0;
    g_keyq_head = g_keyq_tail = 0; g_key_seq_state = 0; g_key_seq_at = -1;  // drop any queued taps
}

// Faithful interactive key: resolve THIS model's matrix line (like
// dct3_web_key_logical) but drive a REAL matrix edge — set the bit on down, clear
// it on up — with NO host auto-release, exactly mirroring the native GUI's
// key_apply (tools/gui_sdl.c). The firmware's own scan period + repeat grace +
// long-press thresholds then decide tap-vs-hold, so press-and-HOLD works as on
// hardware (menu auto-repeat, hold-1 voicemail, long-Menu voice dial, hold-#
// silent profile, …). The CALLER owns the up edge and MUST release on
// pointer-leave / blur so a lost up can't stick a key. Returns 1 if the key
// exists on this model, else 0. PWR-class keys go through the special scan.
EMSCRIPTEN_KEEPALIVE
int dct3_web_key_logical_raw(int key_id, int down) {
    if (!g_core || !g_model) return 0;
    const KeyLine* k = emu_keyline(g_model, key_id);   // shared KK_*->matrix lookup (EmuHost)
    if (!k) return 0;                                  // key not present on this model
    if (k->special) {                                  // PWR-class special-scan
        emu_key_special(&g_mad2, k->special, down);
        g_mad2.irq_pending |= 0x01;
        return 1;
    }
    int row = k->row & 7, col = k->col & 7;
    if (down) g_mad2.kbd_norm_cols[row] |=  (uint8_t)(1u << col);
    else      g_mad2.kbd_norm_cols[row] &= (uint8_t)~(1u << col);
    mad2_keypad_irq(&g_mad2);   // matrix edge (8850-class source bit too)
    // Drop any queued auto-release tap so the sequencer can't also drive this key.
    g_key_armed = 0;
    g_keyq_head = g_keyq_tail = 0; g_key_seq_state = 0; g_key_seq_at = -1;
    return 1;
}

// Keypad auto-release window (instructions) — live-tunable from the UI slider.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_key_hold(int insns) {
    if (insns < 2000) insns = 2000;
    g_key_hold_insns = insns;
}
EMSCRIPTEN_KEEPALIVE
int dct3_web_get_key_hold(void) { return (int)g_key_hold_insns; }

// Experimental one-step-keys mode (silent release after KEY_DOWN is queued).
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_oneshot(int on) { g_key_oneshot = on ? 1 : 0; }

// Task-stub HLE (diagnostic): register/clear handler entry PCs to force-return, and
// read per-stub hit counts (confirms the stub actually fired).
void dct3_web_stub_add(uint32_t pc) { if (g_n_stub < STUB_N) { g_stub_pc[g_n_stub] = pc & ~1u; g_stub_hits[g_n_stub] = 0; g_n_stub++; } }
void dct3_web_stub_clear(void) { g_n_stub = 0; }
uint32_t dct3_web_stub_hits(int slot) { return (slot >= 0 && slot < g_n_stub) ? g_stub_hits[slot] : 0u; }

// Heap-counter write-watch accessors (diagnostic).
void     dct3_web_wrwatch_on(uint32_t addr) { g_wrw_addr = addr; g_wrw_init = 0; g_wrw_npc = 0; g_wrw_sz_n = 0; }
uint32_t dct3_web_wrwatch_npc(void)         { return (uint32_t)g_wrw_npc; }
uint32_t dct3_web_wrwatch_pc(int i)         { return (i >= 0 && i < g_wrw_npc) ? g_wrw_pc[i]  : 0u; }
int32_t  dct3_web_wrwatch_net(int i)        { return (i >= 0 && i < g_wrw_npc) ? g_wrw_net[i] : 0;  }
uint32_t dct3_web_wrwatch_cnt(int i)        { return (i >= 0 && i < g_wrw_npc) ? g_wrw_cnt[i] : 0u; }
uint32_t dct3_web_wrwatch_lr(int i)         { return (i >= 0 && i < g_wrw_npc) ? g_wrw_lr[i]  : 0u; }
uint32_t dct3_web_wrwatch_szn(void)         { return (uint32_t)g_wrw_sz_n; }
uint32_t dct3_web_wrwatch_sz(int i)         { return (i >= 0 && i < g_wrw_sz_n) ? g_wrw_sz[i]   : 0u; }
uint32_t dct3_web_wrwatch_sza(int i)        { return (i >= 0 && i < g_wrw_sz_n) ? g_wrw_sz_a[i] : 0u; }
uint32_t dct3_web_wrwatch_szf(int i)        { return (i >= 0 && i < g_wrw_sz_n) ? g_wrw_sz_f[i] : 0u; }
// Alloc-caller histogram accessors + size-window setter (plan 04: the alloc-caller
// attribution moved into the shared allocator shadow — heap_shadow.c aggregates the
// live-set by alloc-site, surfaced via dct3_web_leak_dump / the unified deep report).
// These export symbols are retained as stubs so the JS dev panel + nav still load.
void     dct3_web_acall_window(uint32_t lo, uint32_t hi) { (void)lo; (void)hi; }
uint32_t dct3_web_acall_n(void)             { return 0; }
uint32_t dct3_web_acall_lr(int i)           { (void)i; return 0; }
uint32_t dct3_web_acall_cnt(int i)          { (void)i; return 0; }
uint32_t dct3_web_acall_egsz(int i)         { (void)i; return 0; }

// UI-message log. Enable by setting the capture PC (default send/route 0x2E84B6,
// where r0 = message id); 0 disables. The page reads the ring + write counter.
EMSCRIPTEN_KEEPALIVE
void dct3_web_msglog_pc(uint32_t pc) { g_msglog_pc = pc; g_msglog_w = 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t* dct3_web_msglog_buf(void) { return g_msglog; }
EMSCRIPTEN_KEEPALIVE
uint32_t* dct3_web_msglog_lrbuf(void) { return g_msglog_lr; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_msglog_size(void) { return MSGLOG_N; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_msglog_w(void) { return (double)g_msglog_w; }

EMSCRIPTEN_KEEPALIVE
void dct3_web_dbg_watch(int slot, uint32_t pc) {
    if (slot < 0 || slot >= DBG_NWATCH) return;
    g_dbg_pc[slot] = pc; g_dbg_cnt[slot] = 0;
    g_dbg_any = 0;
    for (int k = 0; k < DBG_NWATCH; ++k) if (g_dbg_pc[k]) g_dbg_any = 1;
}

EMSCRIPTEN_KEEPALIVE
double dct3_web_dbg_count(int slot) {
    return (slot >= 0 && slot < DBG_NWATCH) ? (double)g_dbg_cnt[slot] : 0;
}

// Release / press the power key (special-scan col1). Held by default so the
// power-on-reason gate sees it; release to model a momentary press.
EMSCRIPTEN_KEEPALIVE
void dct3_web_power(int held) {
    if (!g_core) return;
    g_pwr_auto = 0;                       // manual control overrides auto-release
    g_pwr_released = 1;
    g_mad2.kbd_special_cols = held ? 0x02 : 0x00;
    g_mad2.irq_pending |= 0x01;
    // 8850-class (uif_irq) models route IRQ0 by interrupt-SOURCE bit: the dispatcher
    // (0x301714) only reaches the keypad handler/special-scan when the matrix source bit
    // (bit4) is asserted. Normal keys get it via mad2_keypad_irq; the power button skipped
    // it, so PWR (a special-scan key) raised IRQ0 but was never decoded -> dead power key on
    // the 2100/5210/8850. Assert it here too. (3310-class uif_irq=0 goes straight to the ISR,
    // so the raw IRQ0 above already suffices — matches its working power-key construct.)
    if (g_model && g_model->keypad.uif_irq) g_mad2.kpd_im_status |= 0x10;
}

EMSCRIPTEN_KEEPALIVE
void dct3_web_set_bypass(int on) { g_bypass_sim = on ? 1 : 0; }

// Master gate for the reset-recovery framework. Off = every firmware self-reset
// falls through to warm-reboot regardless of reason (the pre-recovery behaviour).
// On (default) = recover what we can in place (see mad2 case 0x01). The page
// surfaces this as the "Auto-recover crash" checkbox.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_recover(int on) { g_mad2.recover_enabled = on ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_recover(void)   { return g_mad2.recover_enabled; }

// Service-mode watchdog inhibit. Faithful to Nokia's diagnostic IO that
// disabled the CCONT WDT so service engineers could fault-find a live device
// without a clean-shutdown write (WDT=0) ending the session. ON: WDT=0 writes
// are counted (g_mad2.wdt_inhibited_count) but power_off is not latched.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_wdt_service(int on) { g_mad2.wdt_service_mode = on ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_wdt_service(void)   { return g_mad2.wdt_service_mode; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_wdt_inhibited(void) { return (uint32_t)g_mad2.wdt_inhibited_count; }

// Eager panic-chain intercept (off by default).
// When on, the host step loop intercepts at the reboot fn / ARM fatal handler entry,
// arming recover_pending before any of the noreturn panic chain runs. Behaviour with
// the flag off is byte-identical to the existing late `[0x20001]|=4` catch.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_reboot_early(int on) { g_mad2.reboot_early = on ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_reboot_early(void)   { return g_mad2.reboot_early; }

// Post-mortem text — populated by mad2_render_postmortem() at every reset catch.
// JS reads it as `mod.HEAPU8.subarray(ptr, ptr+len)` and stringifies. Empty/len=0
// until the first reset catch. Surfaced in the crash panel + console for RCA.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_postmortem_buf(void) { return (uint32_t)(uintptr_t)g_mad2.postmortem; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_postmortem_len(void) { return (uint32_t)g_mad2.postmortem_len; }
// Total assertion-log count (since cold boot) — diagnostic, lets the page tell whether
// the firmware has emitted any internal asserts even without a reset.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_assert_count(void)   { return g_mad2.assert_w; }

// Heap allocation-failure accounting (from the m->fw.malloc_fail hook). count = total
// allocs that returned NULL and had to wait for memory; lr = the latest such caller. A
// sustained climb in count is heap exhaustion — the harness watches the delta to flag it.
EMSCRIPTEN_KEEPALIVE
double   dct3_web_heap_fail_count(void) { return (double)g_mad2.heap_fail_count; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_heap_fail_lr(void)    { return g_mad2.heap_fail_lr; }

// --- Instrument 1: allocator-shadow exports (plan 04 — re-pointed to the harness) -
// The leak-tracker moved into src/harness/heap_shadow.c (the allocator shadow live-set
// + HEAPGUARD). These cwrap toggles now ARM the SHARED instrument (gated OFF by default
// — the dev panel opts in), and the data accessors DELEGATE to the harness. No web-side
// reimplementation: both drivers read the same live-set (SC1).
EMSCRIPTEN_KEEPALIVE
void dct3_web_leak_on(int on) { heap_shadow_arm(on, on); }   // arm allocator shadow + HEAPGUARD
EMSCRIPTEN_KEEPALIVE
void dct3_web_heapcurve_on(int on) { g_heapcurve_on = on ? 1 : 0; if (on) g_heapcurve_w = 0; }
// Live [0x10484C] used-bytes (the curve point), via the shared allocator-shadow accessor.
EMSCRIPTEN_KEEPALIVE
double dct3_web_heap_used(void) { return heap_shadow_used_d(&g_mad2); }
// Live outstanding block count, from the shared live-set.
EMSCRIPTEN_KEEPALIVE
double dct3_web_leak_count(void) { return heap_shadow_live_d(); }

// Allocator-shadow live-set summary into a static buffer (JS reads it as
// HEAPU8.subarray(ptr, ptr+len), like dct3_web_postmortem_buf). The text is produced
// by the SHARED heap_shadow_text() so it matches the CLI's --- allocator shadow ---
// fault-report append. Replaces the old web-private leak attribution (no reimplementation).
static char     g_leak_buf[8192];
static uint32_t g_leak_len = 0;

EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_leak_dump(void) {
    unsigned len = 0;
    const char* src = heap_shadow_text(&g_mad2, &len);
    if (len > sizeof(g_leak_buf) - 1) len = sizeof(g_leak_buf) - 1;
    memcpy(g_leak_buf, src, len);
    g_leak_buf[len] = 0;
    g_leak_len = len;
    return (uint32_t)(uintptr_t)g_leak_buf;
}
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_leak_buf(void) { return (uint32_t)(uintptr_t)g_leak_buf; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_leak_len(void) { return g_leak_len; }

// used-bytes curve accessors (step_lo + used pairs), mirroring the trace accessors.
EMSCRIPTEN_KEEPALIVE
double   dct3_web_heapcurve_count(void) { return (double)g_heapcurve_w; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_heapcurve_step(int i) {
    return (i < 0 || (uint32_t)i >= g_heapcurve_w) ? 0 : g_heapcurve_step[i & (HEAPCURVE_N - 1)];
}
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_heapcurve_used(int i) {
    return (i < 0 || (uint32_t)i >= g_heapcurve_w) ? 0 : g_heapcurve_used[i & (HEAPCURVE_N - 1)];
}

// --- Instrument 2: typed branch ring exports (plan 04 — re-pointed to the harness) -
// The typed branch ring moved into src/harness/telemetry.c as the shadow call stack.
// The toggle now ARMs that shared instrument (gated OFF by default); the exact backtrace
// + branch-ring tail surface through the unified deep report (harness_fault_report ->
// telemetry_dump, the SAME stderr block boot_trace emits). The per-field cwrap accessors
// are retained as stubs (the symbols stay in the export list + the dev panel keeps
// loading) but return 0 — the canonical control-flow trace is now the unified report,
// not a web-private ring. dct3_web_branch_arm is a no-op (the harness arms on sight).
EMSCRIPTEN_KEEPALIVE
void dct3_web_branch_on(int on) { telemetry_arm_branchring(on); }
EMSCRIPTEN_KEEPALIVE
void dct3_web_branch_arm(double step) { (void)step; }   // harness arms the shadow call stack on sight
EMSCRIPTEN_KEEPALIVE
double dct3_web_branch_count(void) { return telemetry_branch_count(); }
// Per-field accessors: the ring data now lives in the harness (telemetry.c) and is
// rendered into the unified deep report. These return 0 so the export symbols + the JS
// dev panel still load; read the control-flow trace from the deep report on a fault.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_pc(int i)        { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_target(int i)    { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_lr(int i)        { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_sp(int i)        { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_step(int i)      { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
int      dct3_web_branch_type(int i)      { (void)i; return -1; }
EMSCRIPTEN_KEEPALIVE
int      dct3_web_branch_depth(int i)     { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
int      dct3_web_branch_cpsr_mode(int i) { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_r0(int i)        { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_r1(int i)        { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_r4(int i)        { (void)i; return 0; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_branch_r9(int i)        { (void)i; return 0; }

// --- Snapshot exports: ASIC IRQ/FIQ mask + pending (Phase 6 wedge snapshot) --
// The branch-ring freeze dump reads these alongside the regs (dct3_web_reg) +
// mailbox depths (via dct3_web_ram @0x105E90) + sendlog to characterise the wedge.
EMSCRIPTEN_KEEPALIVE
int dct3_web_irq_mask(void)    { return g_core ? (int)g_mad2.irq_mask : 0; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_fiq_mask(void)    { return g_core ? (int)g_mad2.fiq_mask : 0; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_irq_pending(void) { return g_core ? (int)g_mad2.irq_pending : 0; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_fiq_pending(void) { return g_core ? (int)g_mad2.fiq_pending : 0; }

// Divergence-finder trace accessors. Run twice with tracing on, dump both traces,
// diff to find the first (cyc, pc) where two replays of the same bundle disagree.
EMSCRIPTEN_KEEPALIVE
void dct3_web_trace_on(int on) {
    g_trace_on = on ? 1 : 0;
    if (on) { g_trace_w = 0; g_trace_next_cyc = 0; }
}
EMSCRIPTEN_KEEPALIVE
double dct3_web_trace_count(void)         { return (double)g_trace_w; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_trace_cyc(int i)          { return (i < 0 || (uint32_t)i >= g_trace_w) ? 0 : (double)g_trace[i & (TRACE_N - 1)].cyc; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_trace_step(int i)         { return (i < 0 || (uint32_t)i >= g_trace_w) ? 0 : (double)g_trace[i & (TRACE_N - 1)].step; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_trace_pc(int i)         { return (i < 0 || (uint32_t)i >= g_trace_w) ? 0 : g_trace[i & (TRACE_N - 1)].pc; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_trace_cpsr(int i)       { return (i < 0 || (uint32_t)i >= g_trace_w) ? 0 : g_trace[i & (TRACE_N - 1)].cpsr; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_trace_fiq(int i)             { return (i < 0 || (uint32_t)i >= g_trace_w) ? 0 : g_trace[i & (TRACE_N - 1)].fiq_pending; }
EMSCRIPTEN_KEEPALIVE
int dct3_web_trace_irq(int i)             { return (i < 0 || (uint32_t)i >= g_trace_w) ? 0 : g_trace[i & (TRACE_N - 1)].irq_pending; }

// "Skip security code": when on, suppress the FuBu v6.39 disp77 "Enter security
// code" lock by redirecting its dispatch to the idle screen (see web_step_once).
// Default OFF -> the lock shows exactly as it would on real hardware.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_skip_seclock(int on) { g_skip_seclock = on ? 1 : 0; }

// Boot spike (HLE self-test verdict pin). set_spike overrides the per-firmware
// auto-decision (takes effect on the next boot); get_spike returns the EFFECTIVE
// pin state after a boot (so the UI checkbox can sync to reality). Turning the
// spike OFF lets v6.39 attempt an organic boot — the DSP-bring-up milestone.
EMSCRIPTEN_KEEPALIVE
void dct3_web_set_spike(int on) { g_spike_force = on ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE
int  dct3_web_get_spike(void)   { return g_pin_verdict; }

// Current ARM PC (gprs[15], = instruction+pipeline offset). For localizing a spin/hang
// by sampling it repeatedly between run chunks.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_pc(void) { return g_core ? (uint32_t)g_core->cpu.gprs[15] : 0; }

// frozen-on-crash PC ring. crashed = 1 once control jumped into the
// low range; w() = total recorded (ring index = w%N); at(i) = entry i (0..N-1, raw
// slot). Read the last N in order: for k in 0..N-1, slot=(w-N+k)%N. Strip before commit.
EMSCRIPTEN_KEEPALIVE int      dct3_web_pcring_crashed(void) { return g_pcring_frozen; }
EMSCRIPTEN_KEEPALIVE uint32_t dct3_web_pcring_w(void)       { return g_pcring_w; }
EMSCRIPTEN_KEEPALIVE int      dct3_web_pcring_n(void)       { return PCRING_N; }
EMSCRIPTEN_KEEPALIVE uint32_t dct3_web_pcring_at(int i)     { return g_pcring[((i % PCRING_N) + PCRING_N) % PCRING_N]; }
EMSCRIPTEN_KEEPALIVE uint32_t dct3_web_pcring_cpsr(int i)   { return g_pcring_cpsr[((i % PCRING_N) + PCRING_N) % PCRING_N]; }

// Timer ticks + edges (for tracing the soft-timer / handler-reset machinery).
// t0_ticks = Timer0/FIQ4 scheduler-tick index (the heartbeat); t1_edges = Timer1
// overflow count = the FIQ5 soft-timer-walk driver (the inactivity/LED/menu-close
// timers are serviced off these). t0/t1_counter = the live 16-bit timer values.
EMSCRIPTEN_KEEPALIVE
double dct3_web_t0_ticks(void)  { return (double)g_mad2.fiq4_lasttick; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_t1_edges(void)  { return (double)g_mad2.t1_overflows; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_fiq8_ticks(void){ return (double)g_mad2.fiq8_ticks; }   // FIQ8 ct_timer (centisecond) ticks
EMSCRIPTEN_KEEPALIVE
int    dct3_web_t0_counter(void){ return g_mad2.t0_counter; }
EMSCRIPTEN_KEEPALIVE
int    dct3_web_t1_counter(void){ return g_mad2.t1_counter; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_fiqs(void)      { return (double)g_mad2.fiqs_raised; }
EMSCRIPTEN_KEEPALIVE
double dct3_web_irqs(void)      { return (double)g_mad2.irqs_raised; }

// Boot-spike diagnostics (packed): bit0 = armed (verdict pinned), bit1 = signature found,
// bits8-31 = the resolved SIM-gate RAM address. Lets the page confirm the smart-match.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_spike_info(void) {
    return (uint32_t)(g_spike_armed ? 1 : 0) | (g_simfunc ? 2u : 0u) | ((g_sim_addr & 0xFFFFFFu) << 8);
}

// --- diagnostics -----------------------------------------------------------
// Read a RAM byte (any address; masked). For live memory inspection from the page.
EMSCRIPTEN_KEEPALIVE
int dct3_web_ram(uint32_t addr) { return g_core ? g_core->ram[addr & DCT3_RAM_MASK] : 0; }

// Base of the flat memory buffer (for fast RAM diffing from a Node probe).
EMSCRIPTEN_KEEPALIVE
uint8_t* dct3_web_ram_ptr(void) { return g_core ? g_core->ram : 0; }

// EEPROM/NVRAM region for browser persistence: the page snapshots
// ram_ptr()+EE_OFF .. +EE_SIZE to localStorage and overlays it on next boot.
// dct3_web_eeprom_writes() = count of flash program ops (firmware NVRAM writes);
// the page saves only when it changes, so it isn't re-encoding the region every
// frame. The window comes from the ACTIVE profile (g_model->mem) — it is NOT a
// constant: 2 MB images keep NVRAM at 0x3D0000, but the 4 MB images push it late
// (3330/5510 0x550000, 3410 0x570000, 5210/8855 0x580000, 7110 0x5D0000). Hardcoding
// the 3310 window here persisted firmware *code* (not NVRAM) on every 4 MB model and
// never captured their real EEPROM. The pre-boot fallback matches the 3310 default.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_eeprom_off(void)   { return g_model ? g_model->mem.eeprom_base : 0x003D0000u; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_eeprom_size(void)  { return g_model ? g_model->mem.eeprom_size : 0x00030000u; }
// EEPROM "writes" the persistence layer watches = programs that actually landed
// in the NVRAM partition (>=0x3D0000). This is the faithful auto-save trigger:
// the page snapshots the region only once the firmware has really committed a
// record to it (a settings change reaches flash). Programs below eeprom_base
// (non-EEPROM region) are excluded here, so a write outside the NVRAM partition
// never spuriously persists.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_eeprom_writes(void){ return (uint32_t)g_mad2.flash_eeprom_programs; }

// External I2C-EEPROM (24Cxx) persistence for early-MAD2 serial-bus models (5110/…). Unlike the
// in-flash NVRAM partition above (a region of the flat RAM), this EEPROM is a separate buffer
// (g_mad2.i2c_eeprom), so the page persists it via its OWN pointer/size/writes trio — same scheme
// (snapshot on write-count change, overlay on boot). size==0 => the model has no external EEPROM
// (the page then only persists the flash region). The buffer is eager-loaded with the baked blob
// in mad2_init, so an overlay applied right after boot() lands on a populated default.
EMSCRIPTEN_KEEPALIVE
uint8_t* dct3_web_i2c_eeprom_ptr(void)   { return g_mad2.i2c_eeprom; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_i2c_eeprom_size(void)  { return (g_model && g_model->i2c_eeprom_default)
                                                  ? g_model->i2c_eeprom_size : 0u; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_i2c_eeprom_writes(void){ return g_mad2.i2c_eeprom_writes; }

// Total flash program ops + last command address — for diagnosing the commit
// path (deferred-commit vs a dropped write) from a probe/console.
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_flash_cmds(void){ return (uint32_t)g_mad2.flash_cmds; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_flash_programs(void){ return (uint32_t)g_mad2.flash_programs; }
EMSCRIPTEN_KEEPALIVE
uint32_t dct3_web_flash_last_addr(void){ return g_mad2.flash_last_cmd_addr; }

// Current keypad matrix column byte for a row (mad2 state, not RAM) — bit set =
// that column is pressed. Plus the special-scan cols via row 8.
EMSCRIPTEN_KEEPALIVE
int dct3_web_kbd(int row) {
    if (!g_core) return 0;
    if (row == 8) return g_mad2.kbd_special_cols;
    return (row >= 0 && row < 8) ? g_mad2.kbd_norm_cols[row] : 0;
}
