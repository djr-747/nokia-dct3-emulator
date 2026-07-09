// Per-step orchestrator (Phase 8, H0). The shared run-loop controller: the pre-
// step reboot-entry snapshot + the four model-record hooks (lifted VERBATIM from
// tools/boot_trace.c's "mirror of src/web/main.c" block) and the post-step recover
// apply (converged on the canonical web ordering). One place, both drivers.
//
// Cadence rule (anti-pattern guard): all timing here uses m->rtc_mono indirectly
// (the model owns it); this file never reads raw cpu->cycles for a period check.

#include "harness/harness.h"
#include <mgba/internal/arm/arm.h>   // ARM_LR / ARM_SP / ARM_PC

void harness_init(HarnessConfig* cfg, Mad2* m, int recovery,
                  long spin_limit, int64_t wild_pc_after) {
    if (!cfg) return;
    cfg->recovery_enabled = recovery;
    // The driver owns the master gate (m->recover_enabled). Mirror the driver's
    // recovery default onto it so the model's late-catch makes the same decision.
    if (m) m->recover_enabled = recovery ? 1 : 0;
    cfg->wild_pc_armed = 1;                 // wild-PC always armed (fires on sight)
    cfg->wild_pc_after = wild_pc_after;     // 0 = from step 0
    cfg->spin_anchor   = 0xFFFFFFFFu;
    cfg->spin_count    = 0;
    // Default 16M steps (~3.3 s emulated). The old 2M (~0.4 s) was too eager — a legit tight
    // compute loop (a checksum/scan over a large region) can run >2M same-block iterations, and
    // such a loop makes NO peripheral progress, so duration is the only cheap signal that
    // separates it from an infinite spin. 16M lets bounded loops finish while a true infinite
    // spin (self-branch / poll-forever) still trips; the WDT-starvation (49 s) + wild-PC traps
    // are the backstops for slower/other hangs. Tunable via SPINLIMIT.
    cfg->spin_limit    = spin_limit > 0 ? spin_limit : 16000000;
    cfg->fault_reason  = 0;
}

HarnessStatus harness_observe(HarnessConfig* cfg, Mad2* m, DCT3Core* core,
                              int64_t step) {
    struct ARMCore* cpu = &core->cpu;
    // De-pipe by the T bit — every hook starts here (== boot_trace.c:783 == main.c:694).
    uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);

    // Reset-recovery entry hook: snapshot the panic-site state at the EARLIEST point —
    // before the reboot fn (m->fw.reboot_fn) destroys LR with its first internal bl.
    // The post-step recover apply picks these up. (Lifted from boot_trace.c:1027-1040.)
    if (m->fw.reboot_fn && pc == (m->fw.reboot_fn & ~1u)) {
        uint32_t lr       = (uint32_t)cpu->gprs[ARM_LR];
        uint32_t reason_r = (uint32_t)cpu->gprs[0];
        uint32_t cpsr     = cpu->cpsr.packed;
        uint32_t sp       = (uint32_t)cpu->gprs[ARM_SP];
        m->reboot_entry_lr     = lr;
        m->reboot_entry_reason = reason_r;
        m->reboot_entry_cpsr   = cpsr;
        m->reboot_entry_sp     = sp;
        m->reboot_entry_seen   = 1;
        if (m->reboot_early)
            mad2_intercept_panic_early(m, reason_r, lr, cpsr, lr, sp, "reboot-entry");
    }

    // ARM fatal-handler entry hook (reason 5 = CPU exception). Always-on capture so the
    // post-mortem can label undef vs SWI vs abort; eager intercept gated on reboot_early.
    if (m->fw.fatal_handler && pc == (m->fw.fatal_handler & ~1u) && !cpu->cpsr.t) {
        uint32_t lr_banked   = (uint32_t)cpu->gprs[ARM_LR];
        uint32_t spsr_banked = cpu->spsr.packed;
        mad2_record_fatal(m, (uint8_t)cpu->cpsr.priv, lr_banked, spsr_banked);
        if (m->reboot_early)
            mad2_intercept_panic_early(m, 5u, lr_banked, spsr_banked,
                                       lr_banked, 0u, "fatal-entry");
    }

    // Assertion-log ring (m->fw.assert_log). r0 = code, r1 = data, LR = the caller.
    if (m->fw.assert_log && pc == (m->fw.assert_log & ~1u))
        mad2_record_assertion(m,
                              (uint16_t)(cpu->gprs[0] & 0xFFFFu),
                              (uint16_t)(cpu->gprs[1] & 0xFFFFu),
                              (uint32_t)cpu->gprs[ARM_LR]);

    // Staged-resets ring (m->fw.reason_setter). r0 = reason staged; LR = the caller.
    if (m->fw.reason_setter && pc == (m->fw.reason_setter & ~1u))
        mad2_record_stage(m,
                          (uint8_t)(cpu->gprs[0] & 0xFFu),
                          (uint32_t)cpu->gprs[ARM_LR]);

    // Heap allocation-failure ring (m->fw.malloc_fail). LR = the waiting caller.
    if (m->fw.malloc_fail && pc == (m->fw.malloc_fail & ~1u))
        mad2_record_heap_fail(m, (uint32_t)cpu->gprs[ARM_LR]);

    // D-06 instruments (heap_shadow + telemetry + difftrace). Each is env-armed and
    // OFF by default. PERF: probe the arming set ONCE; when nothing is armed, skip all
    // three cross-TU calls. They are no-ops when disarmed, so this is byte-identical —
    // but the trace build has no LTO, so the calls + their internal early-returns could
    // not be inlined away and dominated the per-step cost (~140 ns/step, the single
    // biggest hot-loop item across every model). A HEAPGUARD bad-free or a tripped
    // INVARIANT still raises a FAULT_HALT at corruption ONSET when armed.
    static int instr_armed = -1;
    if (instr_armed < 0)
        instr_armed = (getenv("HEAPSHADOW") || getenv("HEAPGUARD") || getenv("HEAPSAMPLE") ||
                       getenv("BRANCHRING") || getenv("INVARIANT")  || getenv("EVENTLOG")  ||
                       getenv("LASTWRITER")  || getenv("DIFFTRACE") ||
                       difftrace_armed()) ? 1 : 0;   // web: armed via dct3_web_difftrace cwrap (no env)
    if (instr_armed) {
        if (heap_shadow_step(m, cpu, pc, step)) {
            cfg->fault_reason = "HEAPGUARD: bad free";
            return HARNESS_FAULT_HALT;
        }
        if (telemetry_step(m, cpu, pc, step)) {
            cfg->fault_reason = "INVARIANT tripped (corruption onset)";
            return HARNESS_FAULT_HALT;
        }
        difftrace_step(cpu, pc, step);   // native-vs-wasm lockstep guest-state hash
    }

    // Classify the current state into a per-step verdict.
    return harness_classify(cfg, m, core, pc, step);
}

void harness_post_step(HarnessConfig* cfg, Mad2* m, DCT3Core* core) {
    // Canonical web ordering (main.c:1022-1043): the driver already ran dct3_step +
    // mad2_timers_tick BEFORE calling us; we apply a pending recover here, AFTER the
    // step (so the spinning `b .` runs once) and BEFORE the FIQ/IRQ raise (so the
    // resume mode sticks). Gated by recovery policy — the model only sets
    // recover_pending when recover_enabled is on, so this is a no-op when OFF.
    if (m->recover_pending) {
        dct3_core_force_pc_cpsr(core, m->recover_pc, m->recover_cpsr, m->recover_sp);
        m->recover_pending = 0;
        m->recover_sp      = 0;
        // Resume in a fresh basic-block window so the spin detector doesn't trip on
        // the just-redirected PC (the driver-local spin state moved here with fault.c).
        cfg->spin_anchor = 0xFFFFFFFFu;
        cfg->spin_count  = 0;
    }
    // Shadow call stack (D-06.2): update the typed branch ring + return-address
    // stack AFTER the step (the de-piped PC is final for the instruction just run).
    // Gated OFF by default internally — byte-identical when BRANCHRING is unset.
    // step is unavailable here (post_step takes no step arg); the ring stamps with
    // the model's rtc_mono-derived ordering instead — pass 0, the ring is order-only.
    telemetry_post_branch(m, &core->cpu, 0);
}

void harness_fault_report(HarnessConfig* cfg, Mad2* m, DCT3Core* core,
                          const char* reason) {
    (void)cfg;
    (void)core;
    (void)reason;
    // Thin wrapper over the byte-stable post-mortem oracle (Pitfall 2 — do NOT touch
    // the postmortem body). The model already renders into m->postmortem at the catch
    // site (the late-catch / early-intercept paths in mad2.c); ensure it is rendered
    // here too for fault classes the model didn't already cover (wild-PC / spin), then
    // the driver prints m->postmortem. The five instrument appends arrive in plan 03.
    if (m && m->postmortem[0] == '\0') {
        // No model-rendered post-mortem yet (e.g. a wild-PC / spin halt the model
        // didn't catch). Render the current reason snapshot so the driver has a dump.
        struct ARMCore* cpu = &core->cpu;
        uint32_t pc   = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
        uint32_t cpsr = cpu->cpsr.packed;
        uint32_t lr   = (uint32_t)cpu->gprs[ARM_LR];
        mad2_render_postmortem(m, m->reset_last_reason, pc, cpsr, lr,
                               reason ? reason : "harness-fault");
        // The model's own catch sites fputs(m->postmortem) to stderr; for the classes
        // it didn't render (wild-PC / spin / HEAPGUARD / invariant) we emit it here so
        // the instrument appends below stay attached to the post-mortem block.
        fputs(m->postmortem, stderr);
    }

    // D-06 instrument appends: APPEND the five instrument dumps AFTER the byte-stable
    // post-mortem (Pitfall 2 — the postmortem body is never edited). Each dump is a
    // no-op when its instrument was never armed, so a default (unarmed) fault report
    // is byte-identical. Append order: allocator shadow / HEAPGUARD live-set first
    // (the heap is the Phase-9 RE target), then the four telemetry instruments
    // (shadow call stack, IRQ/CCONT log, last-writer ring, invariant verdict).
    if (m && (heap_shadow_armed() || telemetry_armed())) {
        fprintf(stderr, "\n=== DEEP-ANALYSIS INSTRUMENTS (D-06) ===\n");
        heap_shadow_dump(m);
        telemetry_dump(m);
    }
}
