// Phase 9 Wave-0 instrument-regression checks (WR-01 / WR-02 / WR-04).
//
// These pin the three Phase-8 instrument-quality defects the Phase-9 plan repairs,
// so a future edit cannot silently re-introduce them. They are pure-C unit checks
// against the shared-core instruments (src/harness/heap_shadow.c + telemetry.c),
// driven through the public arming APIs + the per-step hooks, with a minimal Mad2 +
// ARMCore the test owns (no firmware, no mGBA execution).
//
// The two instrument TUs are #include'd directly so the checks can read the file-
// static instrument state (cadence re-arm value, bad-free cap, last-writer ring)
// without widening the shared-core public API surface. heap_shadow.c carries a weak
// telemetry_backtrace(); telemetry.c carries the strong one — the strong definition
// wins in this single combined TU.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "harness/harness.h"
#include <mgba/internal/arm/arm.h>   // struct ARMCore (gprs / cpsr.t)

// Bring the instrument file-statics into this TU (see header note). telemetry.c
// carries the STRONG telemetry_backtrace(); heap_shadow.c carries a weak fallback of
// the same name for standalone linking. In one combined TU that is a redefinition, so
// include telemetry.c first (strong def), then rename heap_shadow.c's weak fallback
// out of the way for this TU only — we never call it here.
#include "../src/harness/telemetry.c"
#define telemetry_backtrace hs_weak_telemetry_backtrace_unused_in_test
#include "../src/harness/heap_shadow.c"
#undef telemetry_backtrace

static int g_pass, g_fail;
#define CHECK(desc, cond) do { if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL %s\n", desc); } } while (0)
#define CHECK_EQ(desc, got, want) do { \
    uint64_t G=(uint64_t)(got), W=(uint64_t)(want); \
    if (G==W) { g_pass++; } \
    else { g_fail++; printf("  FAIL %-40s got=0x%llX want=0x%llX\n", desc, \
            (unsigned long long)G, (unsigned long long)W); } } while (0)

// --- minimal host state the instruments read --------------------------------
#define TEST_RAM_BITS 22u                 // 4 MiB window (covers 0x10484C + 0x11xxxx)
#define TEST_RAM_SIZE (1u << TEST_RAM_BITS)

static Mad2          g_m;
static struct ARMCore g_cpu;
static uint8_t*      g_mem;

static void host_reset(void) {
    memset(&g_m, 0, sizeof g_m);
    memset(&g_cpu, 0, sizeof g_cpu);
    if (!g_mem) g_mem = calloc(1, TEST_RAM_SIZE);
    memset(g_mem, 0, TEST_RAM_SIZE);
    g_m.mem      = g_mem;
    g_m.mem_mask = TEST_RAM_SIZE - 1u;
    g_m.rtc_mono = 0;
    g_cpu.cpsr.t = 1;                      // Thumb (irrelevant — we drive pc directly)
}

// --- WR-01: HEAPSAMPLE cadence does not wrap past 2.1B (int64_t) --------------
// On wasm32 a `long` cadence wraps at ~2.147e9; the comparison `step >= next` and the
// re-arm `next = step + period` go negative. With int64_t the cadence keeps firing and
// the next-fire point is the exact (step + period) far past 2^31.
static void test_wr01_heapsample_past_2g(void) {
    printf("[WR-01] HEAPSAMPLE cadence past 2.1B (no wrap)\n");
    host_reset();
    const int64_t period = 1000000000LL;          // 1e9
    heap_shadow_arm(0, 0);                          // shadow+guard off
    heap_shadow_set_sample(period);                 // arm the curve sampler only
    CHECK("sample armed", hs_sample_every == period);

    // First fire at a step already past 2^31 — the wrap regime.
    int64_t step = 3000000000LL;                    // 3e9 > 2^31 (2.147e9)
    (void)heap_shadow_step(&g_m, &g_cpu, 0x00200000u, step);
    // The re-arm must be the exact step+period (no negative wrap), proving the
    // comparison + the stored next-fire point are 64-bit.
    CHECK_EQ("next-fire = step+period (no wrap)", hs_sample_next, step + period);
    CHECK("next-fire stays > 2^31", hs_sample_next > (int64_t)0x7FFFFFFF);

    // A step between fires must NOT re-fire (cadence is monotonic past 2^31).
    int64_t before = hs_sample_next;
    (void)heap_shadow_step(&g_m, &g_cpu, 0x00200000u, step + period / 2);
    CHECK_EQ("no premature re-arm mid-period", hs_sample_next, before);

    // The next scheduled step fires + re-arms again, still far past 2^31.
    (void)heap_shadow_step(&g_m, &g_cpu, 0x00200000u, hs_sample_next);
    CHECK_EQ("second fire re-arms +period", hs_sample_next, before + period);

    heap_shadow_set_sample(0);                      // disarm for the next test
}

// --- WR-02: last-writer ring records the STORING instruction's PC -------------
// A change detected at loop-top in telemetry_step is observed AFTER the storing
// instruction ran, so the recorded writer PC must be the PREVIOUS step's PC
// (tlm_lw_prev_pc), not the current de-piped PC.
static void test_wr02_lastwriter_prev_pc(void) {
    printf("[WR-02] last-writer ring records the storing PC (prev_pc)\n");
    host_reset();
    const uint32_t watch = 0x00110000u;             // a RAM byte inside the window
    const uint32_t PC_STORE = 0x002D11F4u;          // the instruction that does the store
    const uint32_t PC_NEXT  = 0x002D11F6u;          // the next-to-execute insn (the bug value)

    // Force-arm the last-writer ring on `watch` (env-free path).
    telemetry_set_lastwriter(watch, 1);
    tlm_lw_on   = 1;                                // override the env gate for the test
    tlm_lw_init = 0;                                // fresh seed
    tlm_lw_w    = 0;

    // Step 1: at PC_STORE the watched byte is UNCHANGED — this is the seeding poll.
    // (Models: the store has not landed yet when the loop-top poll runs.)
    g_mem[watch & g_m.mem_mask] = 0x00;
    (void)telemetry_step(&g_m, &g_cpu, PC_STORE, 1);
    CHECK("lw seeded prev_pc = store PC", tlm_lw_prev_pc == PC_STORE);
    CHECK("no record yet (unchanged)", tlm_lw_w == 0);

    // The store lands (PC_STORE wrote it); the NEXT step's loop-top poll detects it.
    g_mem[watch & g_m.mem_mask] = 0xAB;
    (void)telemetry_step(&g_m, &g_cpu, PC_NEXT, 2);
    CHECK_EQ("one change recorded", tlm_lw_w, 1u);
    // The recorded writer PC must be the STORING instruction (PC_STORE), NOT PC_NEXT.
    CHECK_EQ("writer PC = storing insn (prev_pc)", tlm_lw_ring[0].pc, PC_STORE);
    CHECK("writer PC != next-to-execute (the bug)", tlm_lw_ring[0].pc != PC_NEXT);
    CHECK_EQ("recorded new byte", tlm_lw_ring[0].new_b, 0xAB);

    tlm_lw_on = 0;                                  // disarm
}

// --- WR-04: a guard-only re-arm resets the bad-free cap -----------------------
// heap_shadow_arm(0,1) (guard only, no shadow) must clear hs_badfree_n so a stale
// prior run's saturated report cap does not silently drop the next run's poison-free.
static void test_wr04_guard_only_cap_reset(void) {
    printf("[WR-04] heap_shadow_arm(0,1) resets the bad-free cap\n");
    host_reset();
    heap_shadow_arm(0, 1);                           // guard only
    CHECK("guard armed, shadow off", hs_guard_on == 1 && hs_shadow_on == 0);
    CHECK_EQ("cap starts at 0", hs_badfree_n, 0u);

    // Drive poison-frees through the guard path to saturate the report cap. r0 = a
    // 0xAA-poisoned pointer at the HEAP_FREE entry PC. HEAPGUARD_NOSTOP avoids the
    // FAULT_HALT return so we can keep driving past the first hit.
    setenv("HEAPGUARD_NOSTOP", "1", 1);
    g_cpu.gprs[0] = (int32_t)0xAABBCCDDu;            // poison top byte = 0xAA
    for (int i = 0; i < HS_BADFREE_CAP + 4; ++i)
        (void)heap_shadow_step(&g_m, &g_cpu, HS_HEAP_FREE, (int64_t)i + 1);
    CHECK_EQ("cap saturated by poison frees", hs_badfree_n, (unsigned)HS_BADFREE_CAP);

    // The WR-04 fix: a guard-only re-arm must zero the cap (pre-fix it was inside the
    // hs_shadow_on-only block and stayed saturated).
    heap_shadow_arm(0, 1);
    CHECK_EQ("guard-only re-arm cleared the cap (WR-04)", hs_badfree_n, 0u);

    // And the next poison-free is reported again (cap budget restored).
    g_cpu.gprs[0] = (int32_t)0xAA112233u;
    (void)heap_shadow_step(&g_m, &g_cpu, HS_HEAP_FREE, 1);
    CHECK_EQ("post-rearm poison-free is counted again", hs_badfree_n, 1u);

    unsetenv("HEAPGUARD_NOSTOP");
    heap_shadow_arm(0, 0);                           // disarm
}

int main(void) {
    printf("DCT3 harness — Phase 9 Wave-0 instrument-regression checks\n\n");
    test_wr01_heapsample_past_2g();
    test_wr02_lastwriter_prev_pc();
    test_wr04_guard_only_cap_reset();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
