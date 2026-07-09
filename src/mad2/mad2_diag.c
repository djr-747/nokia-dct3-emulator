// MAD2 — root-cause attribution (assertion ring + post-mortem) and the eager
// panic-chain intercept. Extracted from mad2.c; see mad2_internal.h.
//
// All exported functions (mad2_record_*, mad2_render_postmortem,
// mad2_intercept_panic_early) are declared in mad2.h. The static helpers
// caller_label_3310_v579 / fatal_mode_label are private to this file.

#include "mad2/mad2_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Auto-generated 3310 v5.79 symbol table for the post-mortem caller-LR fallback
// (nearest-symbol "near SYMBOL + 0xN" hints; binary search, ~185 entries).
// Regenerate via tools/symbols/gen_c_table.py 3310-v579.
#include "models/3310/symbols_3310_v579.h"

// --- Root-cause attribution (assertion ring + post-mortem) -------------------

void mad2_record_assertion(Mad2* m, uint16_t code, uint16_t data, uint32_t lr) {
    if (!m) return;
    uint32_t i = m->assert_w & (MAD2_ASSERT_RING_N - 1u);
    m->assert_ring[i].cyc  = m->rtc_mono;
    m->assert_ring[i].code = code;
    m->assert_ring[i].data = data;
    m->assert_ring[i].lr   = lr;
    m->assert_w++;
}

void mad2_record_stage(Mad2* m, uint8_t reason, uint32_t lr) {
    if (!m) return;
    uint32_t i = m->stage_w & (MAD2_STAGE_RING_N - 1u);
    m->stage_ring[i].cyc    = m->rtc_mono;
    m->stage_ring[i].reason = reason;
    m->stage_ring[i].lr     = lr;
    m->stage_w++;
}

void mad2_record_fatal(Mad2* m, uint8_t cpsr_mode, uint32_t lr_banked, uint32_t spsr) {
    if (!m) return;
    m->fatal_mode = cpsr_mode & 0x1Fu;
    m->fatal_lr   = lr_banked;
    m->fatal_spsr = spsr;
}

void mad2_record_heap_fail(Mad2* m, uint32_t lr) {
    if (!m) return;
    if (m->heap_fail_count == 0) m->heap_fail_first_cyc = m->rtc_mono;
    m->heap_fail_count++;
    m->heap_fail_lr  = lr;
    m->heap_fail_cyc = m->rtc_mono;
}

// One-line semantic label for the reboot-fn / fatal-handler caller. v5.79
// addresses (Factory Reset / My 3310 NR1) are recognised by exact match for
// the 7 known reboot-fn callers; otherwise the symbol-table fallback (the
// auto-generated dct3_symbols_3310_v579 array) is consulted for a
// "near SYMBOL + 0xN" hint. `buf` is a caller-provided scratch for the
// nearest-symbol formatter (the hand-coded labels are static strings + don't
// need it). Returns NULL only if no label found AND no nearby symbol within
// 0x1000 bytes — rare.
static const char* caller_label_3310_v579(uint32_t lr, char* buf, size_t buflen) {
    uint32_t a = lr & ~1u;
    // Callers of the universal reboot fn 0x2EEBAE (return-addr ≈ call+4 in Thumb).
    if (a == 0x002F0BC6u) return "fatal handler 0x2F0BA4 (reason 5 — CPU exception)";
    if (a == 0x002E4F7Eu) return "FIQ handler 0x2E4F38 (reason 6 — FIQ canary)";
    if (a == 0x002E50E2u) return "FIQ handler 0x2E50A0 (reason 6 — FIQ canary)";
    if (a == 0x002E524Eu) return "FIQ handler 0x2E51F0 (reason 6 — FIQ canary)";
    if (a == 0x002EEC6Eu) return "factory-reset path 0x2EEC50 (reason 4)";
    if (a == 0x00291B42u) return "task1 / clock region 0x291xxx (reason 12)";
    if (a == 0x002444CCu) return "task-2 broker msg-0xDC re-issue 0x2444C8";
    // Fall back to nearest-symbol from the generated table. Only useful if
    // the offset is small (< 0x1000) — anything further is "not near anything".
    if (buf && buflen) {
        int idx = dct3_symbols_3310_v579_nearest(a);
        if (idx >= 0) {
            uint32_t base = dct3_symbols_3310_v579[idx].addr;
            uint32_t delta = a - base;
            if (delta < 0x1000) {
                snprintf(buf, buflen, "near %s + 0x%X",
                         dct3_symbols_3310_v579[idx].name, (unsigned)delta);
                return buf;
            }
        }
    }
    return NULL;
}

// Exception-mode → exception-type label for reason 5. cpu->cpsr.mode at PC=fatal_handler
// is the mode the CPU switched into when the exception fired (HW-defined per the ARM ARM).
static const char* fatal_mode_label(uint8_t mode) {
    switch (mode & 0x1Fu) {
        case 0x11: return "FIQ (unexpected — FIQ handler shouldn't B to fatal)";
        case 0x12: return "IRQ (unexpected — IRQ handler shouldn't B to fatal)";
        case 0x13: return "SVC (software interrupt — SWI instruction executed)";
        case 0x17: return "Abort (data abort / prefetch abort — bad memory access)";
        case 0x1B: return "Undef (undefined instruction — bogus opcode reached)";
        default:   return "user/system (unexpected mode at fatal entry)";
    }
}

void mad2_render_postmortem(Mad2* m, uint32_t reason, uint32_t resume_pc,
                            uint32_t resume_cpsr, uint32_t lr_diag,
                            const char* source) {
    if (!m) return;
    char* buf = m->postmortem;
    size_t cap = sizeof(m->postmortem);
    int n = 0;
    uint32_t r8 = reason & 0xFFu;
    char lbl_scratch[80];   // scratch for the nearest-symbol fallback formatter
    const char* lbl = caller_label_3310_v579(lr_diag, lbl_scratch, sizeof lbl_scratch);
    n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
        "=== POST-MORTEM — reason %u (0x%02X)  source=%s  @cyc %.3f M ===\n",
        (unsigned)r8, (unsigned)r8,
        source ? source : "?",
        (double)m->rtc_mono / 1e6);
    if (lr_diag) {
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  caller-site:  LR=0x%06X%s%s\n",
            lr_diag & 0xFFFFFFu,
            lbl ? "  — " : "", lbl ? lbl : "");
    }
    if (resume_pc) {
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  resume-PC:    0x%06X  cpsr=0x%08X\n",
            resume_pc & 0xFFFFFFu, resume_cpsr);
    }
    if (r8 == 5 && m->fatal_mode) {
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  fatal-mode:   0x%02X  — %s\n",
            (unsigned)m->fatal_mode, fatal_mode_label(m->fatal_mode));
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  fatal-LR:     0x%06X  (faulting PC + ARM offset — see ARM ARM)\n",
            m->fatal_lr & 0xFFFFFFu);
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  fatal-SPSR:   0x%08X  (interrupted CPSR)\n",
            m->fatal_spsr);
    }
    // Staged-resets ring — every reason staged via the reason_setter fn (v5.79 = 0x2E0ED8),
    // *before* the visible reset edge. The cycle gap between a stage and the fire IS the
    // graceful-shutdown duration; when the gap is big (e.g. 22 s for the SWDSP/DSP-reset chain),
    // the assertion-log tail will be dominated by unrelated post-stage activity, so the
    // stages are the real causal signal. See docs/dsp-reset-chain-3310-v579.md.
    uint32_t stage_total = m->stage_w;
    if (stage_total) {
        uint32_t stage_show = stage_total > MAD2_STAGE_RING_N ? MAD2_STAGE_RING_N : stage_total;
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  staged resets (last %u of %u total):\n", stage_show, stage_total);
        for (uint32_t k = 0; k < stage_show; k++) {
            uint32_t idx = (m->stage_w - 1u - k) & (MAD2_STAGE_RING_N - 1u);
            double gap = ((double)(int64_t)(m->rtc_mono - m->stage_ring[idx].cyc)) / 13.0e6;
            uint32_t lr = m->stage_ring[idx].lr;
            char lr_lbl[80] = "";
            int idx2 = dct3_symbols_3310_v579_nearest(lr & ~1u);
            if (idx2 >= 0) {
                uint32_t base = dct3_symbols_3310_v579[idx2].addr;
                uint32_t delta = (lr & ~1u) - base;
                if (delta < 0x1000)
                    snprintf(lr_lbl, sizeof lr_lbl, "  [%s+0x%X]",
                             dct3_symbols_3310_v579[idx2].name, (unsigned)delta);
            }
            n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
                "    @%.3fM  reason=0x%02X  from LR=0x%06X%s  (%.2f s before fire)\n",
                (double)m->stage_ring[idx].cyc / 1e6,
                (unsigned)m->stage_ring[idx].reason,
                lr & 0xFFFFFFu, lr_lbl, gap);
        }
    }

    // Heap-exhaustion — alloc-failure count from the malloc_fail hook (the blocking
    // allocator returned NULL and parked on the memory semaphore). A sustained climb means
    // the heap is exhausted; it's a common precursor to a heap-smash wild-PC. LR = a caller
    // that was left waiting for memory.
    if (m->heap_fail_count) {
        uint32_t lr = m->heap_fail_lr;
        char lr_lbl[80] = "";
        int hidx = dct3_symbols_3310_v579_nearest(lr & ~1u);
        if (hidx >= 0) {
            uint32_t base  = dct3_symbols_3310_v579[hidx].addr;
            uint32_t delta = (lr & ~1u) - base;
            if (delta < 0x1000)
                snprintf(lr_lbl, sizeof lr_lbl, "  [%s+0x%X]",
                         dct3_symbols_3310_v579[hidx].name, (unsigned)delta);
        }
        double span = ((double)(int64_t)(m->heap_fail_cyc - m->heap_fail_first_cyc)) / 13.0e6;
        n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
            "  heap: %llu alloc-failure(s) over %.2f s — last @%.3fM from LR=0x%06X%s%s\n",
            (unsigned long long)m->heap_fail_count, span,
            (double)m->heap_fail_cyc / 1e6, lr & 0xFFFFFFu, lr_lbl,
            m->heap_fail_count >= 8 ? "  <- HEAP EXHAUSTION" : "");
    }

    // Assertion-log tail — newest-first, with consecutive identical entries collapsed
    // (same code AND same LR — a repeating heartbeat shows up as one line with ×N + the
    // cycle gap between fires). Walks the ring backward up to 8 raw slots to surface up
    // to 4 distinct events. The dedup is what surfaces signal in a sea of noise — the
    // charge-task heartbeat 0x5B0F repeats every ~12.6 M cyc; without dedup it would fill
    // the tail and hide the actual fault code 0x5B06. See docs/dsp-reset-chain-3310-v579.md §"Two takeaways".
    uint32_t total = m->assert_w;
    uint32_t ring_have = total > MAD2_ASSERT_RING_N ? MAD2_ASSERT_RING_N : total;
    n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
        "  assertion log (deduped tail of %u in ring; %u total):\n",
        ring_have, total);
    int shown = 0;
    uint32_t k = 0;
    while (k < ring_have && shown < 4) {
        uint32_t idx = (m->assert_w - 1u - k) & (MAD2_ASSERT_RING_N - 1u);
        uint16_t code = m->assert_ring[idx].code;
        uint32_t lr   = m->assert_ring[idx].lr;
        uint64_t cyc  = m->assert_ring[idx].cyc;
        // Collapse consecutive identical (code, lr) into one entry; advance k past the run.
        uint32_t run = 1;
        uint64_t prev_cyc = cyc;
        while (k + run < ring_have) {
            uint32_t jdx = (m->assert_w - 1u - k - run) & (MAD2_ASSERT_RING_N - 1u);
            if (m->assert_ring[jdx].code != code) break;
            if (m->assert_ring[jdx].lr   != lr)   break;
            prev_cyc = m->assert_ring[jdx].cyc;
            run++;
        }
        // Name the asserting LR via the symbol table — "[SYMBOL+0xN]" or empty
        // if no symbol within 0x1000 bytes. Surfaces who emitted the assertion
        // directly inline (BROKER_FAULT_NOTIFIER+0x13, CHARGE_HEARTBEAT_HELPER+0x21, …).
        char lr_lbl[80] = "";
        {
            int idx2 = dct3_symbols_3310_v579_nearest(lr & ~1u);
            if (idx2 >= 0) {
                uint32_t base = dct3_symbols_3310_v579[idx2].addr;
                uint32_t delta = (lr & ~1u) - base;
                if (delta < 0x1000)
                    snprintf(lr_lbl, sizeof lr_lbl, "  [%s+0x%X]",
                             dct3_symbols_3310_v579[idx2].name, (unsigned)delta);
            }
        }
        if (run > 1) {
            // newest -> oldest cyc gap, in ms (avoid pulling float ops dep on libm if not linked)
            double gap_ms = ((double)(int64_t)(cyc - prev_cyc)) / (13.0e3 * (double)(run - 1));
            n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
                "    @%.3fM  code=0x%04X  data=0x%04X  from LR=0x%06X%s  (x%u, ~%.1f ms apart)\n",
                (double)cyc / 1e6, code, m->assert_ring[idx].data,
                lr & 0xFFFFFFu, lr_lbl, run, gap_ms);
        } else {
            n += snprintf(buf + n, (n < (int)cap) ? cap - (size_t)n : 0,
                "    @%.3fM  code=0x%04X  data=0x%04X  from LR=0x%06X%s\n",
                (double)cyc / 1e6, code, m->assert_ring[idx].data,
                lr & 0xFFFFFFu, lr_lbl);
        }
        k += run;
        shown++;
    }
    m->postmortem_len    = (uint16_t)((n < 0) ? 0 : (n >= (int)cap ? cap - 1 : n));
    m->postmortem[m->postmortem_len] = 0;
    m->postmortem_at_cyc = m->rtc_mono;
}

// --- Eager panic-chain intercept ---------------------------------------------
// Called from the host step loop when PC enters the universal reboot fn (m->fw.reboot_fn)
// or the ARM fatal handler (m->fw.fatal_handler) AND m->reboot_early is set. Sets up the
// same recover_pending state the late `[0x20001]|=4` catch would, just with the values
// captured BEFORE the noreturn panic chain mutated runtime state (task notify, CTSI int-
// control write, busy delay, persisted reason byte). See docs/watchdog-deep-re-findings.md.
//
// The caller still must advance the CPU past the panic site — either by force-applying
// recover_pending immediately (web_step_once already has a post-step apply) or by leaving
// the CPU at the entry instruction and trusting the post-step apply on this same iteration.
//
// Returns 1 if recover_pending was armed (the caller can skip the panic execution), 0 if
// the reason is on the exclusion list (recover_reasons[reason] == 0) — in that case the
// caller MUST let the normal panic chain run so the existing late catch can warm-reboot.
int mad2_intercept_panic_early(Mad2* m, uint32_t reason, uint32_t resume_pc,
                               uint32_t resume_cpsr, uint32_t lr_diag,
                               uint32_t resume_sp, const char* source) {
    if (!m) return 0;
    uint32_t r8 = reason & 0xFFu;
    // Honour the master gate and per-reason exclusion list — the eager path must respect
    // the same policy the late catch does. Reason 4 (factory reset, intentional) and any
    // user-disabled reason stay on the warm-reboot path.
    if (!m->recover_enabled || !m->recover_reasons[r8]) return 0;
    // Bump the same counters the late catch would.
    m->reset_last_reason = (uint8_t)r8;
    m->reset_total++;
    m->reset_counts[r8 < 16 ? r8 : 15]++;
    m->reset_last_lr   = lr_diag;
    m->reset_last_pc   = resume_pc;
    m->reset_last_cpsr = resume_cpsr;
    m->recover_pending = 1;
    m->recover_pc      = resume_pc;
    m->recover_cpsr    = resume_cpsr;
    // Multi-frame pop: keep parity with the late lr-return path. resume_sp is non-zero
    // only when the caller could supply the entry SP (reboot_fn entry); the fatal-handler
    // entry passes 0 (mode-banked SP isn't the right base — reason 5 always uses
    // exc-return semantics directly).
    m->recover_sp      = 0;
    if (resume_sp && m->recover_pop[r8]) {
        uint32_t mask = m->mem_mask ? m->mem_mask : 0x00FFFFFFu;
        uint32_t sp = resume_sp;
        int need = m->recover_pop[r8];
        const uint32_t flash_lo = m->model ? m->model->mem.flash_base : 0x00200000u;
        const uint32_t flash_hi = m->model ? (flash_lo + m->model->mem.flash_size)
                                           : 0x00400000u;
        for (int k = 0; k < 256 && need > 0; k++) {
            uint32_t a = (sp + (uint32_t)k * 4u) & mask;
            if (!m->mem) break;
            const uint8_t* p = m->mem + a;
            uint32_t w = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                       | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
            uint32_t r = w & ~1u;
            if (!(w & 1u))               continue;
            if (r < flash_lo || r >= flash_hi) continue;
            uint32_t pp1 = (r - 4u) & mask, pp2 = (r - 2u) & mask;
            uint16_t h1 = ((uint16_t)m->mem[pp1] << 8) | m->mem[pp1 + 1u];
            uint16_t h2 = ((uint16_t)m->mem[pp2] << 8) | m->mem[pp2 + 1u];
            if ((h1 & 0xF800u) != 0xF000u || (h2 & 0xF800u) != 0xF800u) continue;
            if (--need == 0) {
                m->recover_pc = w;
                m->recover_sp = (uint32_t)(sp + (uint32_t)k * 4u + 4u);
            }
        }
    }
    m->reset_recovered++;
    // Consume the entry snapshot now in case the late catch fires after us (defence
    // against double-counting if recover_pending isn't applied in time).
    m->reboot_entry_seen = 0;
    fprintf(stderr,
        "[reset] CATCH reason=%u (#%llu) EARLY-INTERCEPT (%s) pc=0x%08X cpsr=0x%08X%s\n",
        (unsigned)r8,
        (unsigned long long)m->reset_counts[r8 < 16 ? r8 : 15],
        source ? source : "?",
        m->recover_pc, m->recover_cpsr,
        m->recover_sp ? " +pop" : "");
    // Render the post-mortem block — surfaces the firmware's own narrative (assertion
    // ring, caller-site label, exception type for reason 5) for the recover path.
    mad2_render_postmortem(m, reason, m->recover_pc, m->recover_cpsr, lr_diag, source);
    fputs(m->postmortem, stderr);
    return 1;
}
