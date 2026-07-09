// Shared harness layer (Phase 8, H0). The ONE place fault-detection + reset-
// recovery orchestration lives — consumed by BOTH the native CLI driver
// (tools/boot_trace.c) and the web driver (src/web/main.c, converged in plan 04).
//
// The no-reimplementation rule (memory: feedback-shared-core-no-reimplementation):
// the per-step reboot/fatal/assert/stage/heap-fail snapshots, the recover-apply,
// and the wild-PC / spin / CCONT-reset / power-off classification used to be
// DUPLICATED + drifted across the two drivers. They now live here once. A driver
// becomes a thin input(keypad/replay) + output(PNG/stdout/framebuffer) shell that
// calls harness_observe() once per step BEFORE dct3_step, harness_post_step()
// after, and harness_fault_report() on a FAULT_HALT.
//
// The harness reads cpu->gprs[N] READ-ONLY and mutates the core ONLY through the
// sanctioned recover primitive dct3_core_force_pc_cpsr (src/core/dct3_core.c). It
// never pokes guest RAM or firmware flags — faithful modelling only.

#ifndef DCT3_HARNESS_H
#define DCT3_HARNESS_H

#include <stdint.h>
#include "core/dct3_core.h"
#include "mad2/mad2.h"

// Per-step verdict returned by harness_observe(). The driver acts on it:
//   HARNESS_CONTINUE   — healthy step, keep running.
//   HARNESS_FAULT_HALT — a fault was detected (wild-PC / spin / reboot caught with
//                        recovery OFF / CCONT-reset = WDT starvation lapse). The
//                        driver should call harness_fault_report() then stop.
//   HARNESS_POWERED_OFF — a deliberate clean power-off (WDT=0 write) was latched.
//                        Graceful run-end — NO fault dump.
typedef enum {
    HARNESS_CONTINUE = 0,
    HARNESS_FAULT_HALT,
    HARNESS_POWERED_OFF
} HarnessStatus;

// Harness config + per-step detector state. One instance lives in the driver
// (zero-initialised, then harness_init). Kept minimal — the Phase-6 instruments
// (leak-tracker, branch-ring, allocator shadow) migrate here in plan 03.
typedef struct {
    // --- policy (set by harness_init / driver) ---
    int      recovery_enabled;  // mirror of mad2 recover_enabled gate at init time
                                // (informational; the model owns the live gate)
    // --- wild-PC detector (D-11) ---
    int      wild_pc_armed;     // fire on PC >= 0x400000 (the web severity-split:
                                // a high PC fires on sight, never sample-gated)
    int64_t  wild_pc_after;     // only arm at/after this step (0 = always)
    // --- spin detector (D-11, sample-gated) ---
    uint32_t spin_anchor;       // 64-byte block anchor of the current basic block
    long     spin_count;        // consecutive steps inside spin_anchor
    long     spin_limit;        // trip threshold (SPINLIMIT env, default 2,000,000)
    // --- fault reporting ---
    const char* fault_reason;   // set by fault.c when a FAULT_HALT is returned
} HarnessConfig;

// Initialise the harness config from the live model + driver policy. `recovery`
// is the driver's recovery default (boot_trace flips it OFF — D-10); the master
// gate lives in m->recover_enabled which the driver has already set. `spin_limit`
// and `wild_pc_after` come from the driver's env knobs (0 => defaults).
void harness_init(HarnessConfig* cfg, Mad2* m, int recovery,
                  long spin_limit, int64_t wild_pc_after);

// Called ONCE per step, BEFORE dct3_step. Runs the pre-step de-pipe, the reboot-
// entry snapshot + the four model-record hooks (gated on m->fw.<addr> != 0), and
// the fault classify. Returns the per-step HarnessStatus. Reads core->cpu read-
// only + records into the mad2 rings; never mutates the core here.
HarnessStatus harness_observe(HarnessConfig* cfg, Mad2* m, DCT3Core* core,
                              int64_t step);

// Called ONCE per step, AFTER dct3_step + mad2_timers_tick. Applies a pending
// recover (dct3_core_force_pc_cpsr) when recovery is enabled, converging on the
// canonical web ordering (recover BEFORE the FIQ/IRQ raise). Resets the spin
// window after a recover so the resume lands in a fresh basic-block.
void harness_post_step(HarnessConfig* cfg, Mad2* m, DCT3Core* core);

// Deep fault dump on a FAULT_HALT. In this plan it WRAPS mad2_render_postmortem
// (the byte-stable oracle — Pitfall 2); the five instrument appends arrive in
// plan 03. `reason` is the short tag the driver carries (cfg->fault_reason).
void harness_fault_report(HarnessConfig* cfg, Mad2* m, DCT3Core* core,
                          const char* reason);

// --- fault.c internal classify (exposed for runloop.c) -----------------------
// Classify the current pre-step CPU state into a HarnessStatus. `pc` is the de-
// piped PC. Sets cfg->fault_reason on a non-CONTINUE verdict.
HarnessStatus harness_classify(HarnessConfig* cfg, Mad2* m, DCT3Core* core,
                               uint32_t pc, int64_t step);

// --- D-06 instruments (heap_shadow.c + telemetry.c) --------------------------
// All five instruments live in the shared core and are gated OFF by default (env
// var = CLI, cwrap toggle = web in plan 04). harness_observe dispatches the per-
// step hooks (gated); harness_fault_report appends their dumps AFTER the byte-
// stable post-mortem. Forward-declare struct ARMCore so harness.h need not pull in
// the full mGBA header (the .c files include it).
struct ARMCore;

// heap_shadow.c (D-06.1 allocator shadow live-set + D-07 HEAPGUARD poison-free).
int  heap_shadow_armed(void);                                   // any of HEAPSHADOW/HEAPGUARD/HEAPSAMPLE set
int  heap_shadow_step(Mad2* m, struct ARMCore* cpu, uint32_t pc, int64_t step);  // ret 1 => FAULT_HALT
void heap_shadow_dump(Mad2* m);                                 // append the live-set summary
// web cwrap arming + accessors (plan 04 — the web dev panel arms the same gates).
void heap_shadow_arm(int shadow, int guard);                    // flip HEAPSHADOW/HEAPGUARD gates
void heap_shadow_set_sample(long every);                        // flip HEAPSAMPLE cadence
double heap_shadow_used_d(Mad2* m);                             // [0x10484C] used-bytes (curve point)
double heap_shadow_live_d(void);                                // live outstanding block count
const char* heap_shadow_text(Mad2* m, unsigned* len);           // live-set summary into a static buffer

// telemetry.c (D-06.2 shadow call stack, .4 last-writer ring, .5 IRQ/CCONT log,
// .3 periodic invariant checker).
int  telemetry_armed(void);                                     // any telemetry instrument set
int  telemetry_step(Mad2* m, struct ARMCore* cpu, uint32_t pc, int64_t step);    // ret 1 => FAULT_HALT (invariant tripped)
void telemetry_post_branch(Mad2* m, struct ARMCore* cpu, int64_t step);          // shadow call stack push/pop (post-step)
void telemetry_event_irq(Mad2* m, int irq_n, int is_fiq, int64_t step);          // IRQ/CCONT event log stamp
int  telemetry_backtrace(uint32_t* out, int max);               // exact backtrace (shadow call stack)
void telemetry_dump(Mad2* m);                                   // append the four instrument dumps
// web cwrap arming + accessors (plan 04 — the web dev panel arms the same gates).
void telemetry_arm_branchring(int on);                          // flip BRANCHRING gate (resets the ring)
void telemetry_arm_invariant(int on);                           // flip INVARIANT gate
void telemetry_arm_eventlog(int on);                            // flip EVENTLOG gate
void telemetry_set_lastwriter(uint32_t addr, uint32_t len);     // set LASTWRITER watch range
double telemetry_branch_count(void);                            // taken-branch count

// difftrace.c (Phase 9 09-02 — native-vs-wasm lockstep guest-state differential).
// Gated OFF by default (DIFFTRACE env = CLI; difftrace_arm = web). Folds the per-step
// {pc, r0..r15, cpsr} into a rolling FNV-1a-64 hash; prints checkpoint hashes at a
// cadence + optional full per-step rows in a window. Observation-only (reads cpu
// read-only). The first checkpoint mismatch between the builds localises the divergence.
int  difftrace_armed(void);                                     // DIFFTRACE set / armed
void difftrace_step(struct ARMCore* cpu, uint32_t pc, int64_t step);  // per-step fold + emit
void difftrace_arm(int on, double every, double lo, double hi); // web arm (cadence + row window)

#endif // DCT3_HARNESS_H
