// Fault detect + classify (Phase 8, H0). Lifted out of tools/boot_trace.c's
// inline detector soup so BOTH drivers classify identically (no reimplementation,
// no CLI-vs-web drift). Maps the live CPU + model state to a HarnessStatus:
//
//   wild-PC (PC left flash, >= 0x400000)            -> FAULT_HALT  (severity-split:
//                                                      a high PC fires ON SIGHT, never
//                                                      sample-gated — the web version,
//                                                      memory spin-detector-false-
//                                                      positive-bounded-loops; the bare
//                                                      CLI spin gate false-positives on
//                                                      bounded loops, so wild-PC and spin
//                                                      are SEPARATE here)
//   spin (64-byte block held > spin_limit steps)    -> FAULT_HALT  (sample-gated)
//   reset caught with recovery OFF (reset_request)  -> FAULT_HALT  (D-10 honest halt:
//                                                      the firmware self-reset, the model
//                                                      rendered the post-mortem, we stop
//                                                      instead of silently continuing)
//   CCONT-reset = WDT starvation lapse (wdt_tripped)-> FAULT_HALT  (D-08/D-09: the 49 s
//                                                      window lapsed with no kick)
//   deliberate clean power-off (power_off latch)    -> POWERED_OFF (graceful, NO dump)
//
// Power-off is checked FIRST and wins over everything (a clean shutdown is not a
// fault — Pitfall 4). The starvation window itself lives in the model
// (mad2_timers_tick, retargeted to the real-HW 49 s in plan 01); this file only
// READS the latch the model raises.

#include "harness/harness.h"

HarnessStatus harness_classify(HarnessConfig* cfg, Mad2* m, DCT3Core* core,
                               uint32_t pc, int64_t step) {
    (void)core;

    // (1) Deliberate clean power-off — graceful run-end, NOT a fault. Wins over all
    // fault checks (Pitfall 4: never render a fault dump for a deliberate shutdown).
    if (m->power_off) {
        cfg->fault_reason = "POWER-OFF (clean shutdown, WDT=0)";
        return HARNESS_POWERED_OFF;
    }

    // (2) Wild-PC: the PC left the flash code window [flash_base, flash_base+flash_size).
    // A high PC fires ON SIGHT — this is a hard memory-corruption / wild-branch signal,
    // never a bounded loop. (Web severity-split; the bare CLI version conflated this with
    // spin.) The upper bound is MODEL-AWARE: 2 MB images (3310/3410/5110) cap at 0x400000
    // as before, but 4 MB images (6210/7110/5210) legitimately execute code in the upper
    // bank (e.g. 6210 v5.56 reboot_fn/fatal/assert all live at 0x4Fxxxx-0x50xxxx) — a fixed
    // 0x400000 ceiling false-tripped every call into that bank.
    uint32_t flash_hi = m->model->mem.flash_base + m->model->mem.flash_size;
    if (cfg->wild_pc_armed && step >= cfg->wild_pc_after && pc >= flash_hi) {
        cfg->fault_reason = "WILD-PC (PC left flash)";
        return HARNESS_FAULT_HALT;
    }

    // (3) Spin: the PC has been stuck in one 64-byte basic-block for > spin_limit
    // steps. Sample-gated (the counter only trips after a long dwell) so legitimate
    // bounded loops don't false-fire. The window resets on a recover / IRQ raise in
    // harness_post_step / the driver's interrupt raise.
    uint32_t block = pc & ~0x3Fu;
    if (block == cfg->spin_anchor) {
        if (++cfg->spin_count > cfg->spin_limit) {
            cfg->fault_reason = "SPIN (tight loop)";
            return HARNESS_FAULT_HALT;
        }
    } else {
        cfg->spin_anchor = block;
        cfg->spin_count  = 0;
    }

    // (4) CCONT-reset: the CCONT watchdog starvation window (49 s on rtc_mono, RE-
    // confirmed in plan 01) lapsed with no kick AND the trip is armed (WDTRESET). The
    // model raises wdt_tripped; we classify it as a deep-analysis FAULT_HALT (D-08).
    if (m->wdt_tripped) {
        cfg->fault_reason = "CCONT-RESET (WDT starvation)";
        return HARNESS_FAULT_HALT;
    }

    // (5) A firmware self-reset was caught and NOT recovered (recovery OFF by default
    // in the CLI — D-10). The model already rendered the post-mortem to m->postmortem
    // on the catch; we halt honestly instead of running silently to budget.
    if (m->reset_request) {
        cfg->fault_reason = "RESET (caught, recovery off)";
        return HARNESS_FAULT_HALT;
    }

    return HARNESS_CONTINUE;
}
