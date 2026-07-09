// Allocator shadow live-set + HEAPGUARD poison-free trap (Phase 8, H0 part 2 —
// D-06.1, D-07). Built ONCE in the shared harness so BOTH drivers (the native CLI
// tools/boot_trace.c and, in plan 04, the web src/web/main.c) inherit it — the
// no-reimplementation rule (memory: feedback-shared-core-no-reimplementation).
//
// Two instruments live here, both gated OFF by default (the canonical SWDSP
// post-mortem stays byte-identical and the determinism guarantee holds):
//
//   (1) Allocator shadow (HEAPSHADOW=1) — a fixed-capacity open-addressed live-set
//       keyed by ptr. Tracks alloc(ptr, size, alloc_site, step) at the two alloc
//       wrappers (HEAP_ALLOC 0x299A84 / BLOCKING_MALLOC 0x299AF8) via the depth-N
//       LIFO entry->return latch lifted VERBATIM from the Phase-6 leak-tracker
//       (src/web/main.c:803-836); and free(ptr, free_site, step) at HEAP_FREE
//       0x299A36. On a bad free (free-of-unallocated or double-free) it prints the
//       ASan-style "freed ptr X: originally allocated at <alloc_site>, previously
//       freed at <prev_free_site>" (D-06.1).
//
//   (2) HEAPGUARD (HEAPGUARD=1) — the 0xAA poison-free trap moved out of
//       tools/boot_trace.c into the core (D-07). At HEAP_FREE entry r0 = the ptr
//       being freed; a top-byte == 0xAA is the freed-block fill pattern = a stale
//       pointer read out of an already-freed (0xAA-filled) block = use-after-free /
//       double-free. A valid ptr never matches. On a hit it prints the bad-free
//       report (using the live-set provenance + the shadow call stack from
//       telemetry.c for an EXACT backtrace) and signals FAULT_HALT (heap
//       corruption, D-11) — unless HEAPGUARD_NOSTOP keeps it running.
//
// HEAPSAMPLE=<N> keeps the [0x10484C] used-bytes curve read available too.
//
// Observation-only invariant: every hook reads ONLY r0/r13/r14 + guest RAM through
// m->mem. It NEVER writes guest RAM, cpu->gprs, or cpu->cycles. NEVER allocates
// host memory inside a hook — all rings/tables are pre-sized and never grow
// mid-run (allocation in a hook perturbs the FIQ-reentrancy window, Pitfall 5).

#include "harness/harness.h"
#include <mgba/internal/arm/arm.h>   // ARM_LR / ARM_SP
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- verified hook PCs (symbol store; do not re-derive) ----------------------
// 3310 defaults; per-model override via env (heap fns are NOT sig-supplied yet).
// 3410 v5.46: HEAPSHADOW_ALLOC=0x2F91DC HEAPSHADOW_FREE=0x2F9194 (BMALLOC=ALLOC, one wrapper).
static uint32_t hs_pc_alloc   = 0x00299A84u;   // non-blocking alloc wrapper entry (r0 = size)
static uint32_t hs_pc_bmalloc = 0x00299AF8u;   // blocking alloc wrapper entry (r0 = size)
static uint32_t hs_pc_free    = 0x00299A36u;   // heap free entry (r0 = ptr)
#define HS_HEAP_ALLOC       hs_pc_alloc
#define HS_BLOCKING_MALLOC  hs_pc_bmalloc
#define HS_HEAP_FREE        hs_pc_free
#define HS_BLOCK_WALK       0x002E5492u   // coalesce / free-block walk (HEAP_BLOCK_WALK)
#define HS_ACCT_STRUCT      0x00104844u   // heap accounting struct; +8 = used-bytes (BE u32)

// --- live-set sizing (pre-sized, never grows) --------------------------------
#define HS_N        16384u                // live-set slots (power-of-two, linear-probe)
#define HS_MASK     (HS_N - 1u)
#define HS_MAXLOAD  ((HS_N * 85u) / 100u) // 0.85 load factor — stop inserting past this
#define HS_LATCH_N  4                     // depth of the alloc entry->return latch stack
#define HS_BADFREE_CAP 12                 // cap bad-free reports (avoid flood)

// One live block. state: 0 = empty, 1 = live, 2 = freed (retained for provenance).
typedef struct {
    uint32_t ptr;          // 0 = empty slot
    uint32_t alloc_site;   // outermost firmware caller LR at alloc (the leaker / owner)
    uint32_t prev_free_site; // LR of the LAST free of this ptr (0 = never freed)
    uint32_t size;         // requested size (r0 at entry)
    uint32_t step_lo;      // low 32 of the step counter when allocated
    uint8_t  state;        // HS_EMPTY / HS_LIVE / HS_FREED
} HsSlot;
#define HS_EMPTY 0u
#define HS_LIVE  1u
#define HS_FREED 2u

// Alloc entry->return latch (depth-N LIFO, keyed by (ret_pc, sp)) — lifted from the
// Phase-6 leak-tracker (main.c:803-836). A FIQ can preempt between an alloc entry and
// its return (Landmine #3 / Pitfall 5), so a single slot could lose the outer alloc.
typedef struct {
    uint32_t ret_pc;       // LR & ~1 at alloc entry (the return PC to match)
    uint32_t sp;           // r13 at alloc entry (disambiguates FIQ-nested allocs)
    uint32_t lr;           // r14 at alloc entry (the caller / alloc-site)
    uint32_t size;         // r0 at alloc entry
} HsLatch;

// --- instrument state (file-scope; pre-sized; gated by the env probes) -------
static int      hs_shadow_on = -2;        // -2 = unprobed, 0/1 after first probe
static int      hs_guard_on  = -2;
static int64_t  hs_sample_every = -2;     // -2 = unprobed; 0 = off; >0 = period (WR-01: int64_t — wasm32 long wraps at ~2.1B, exactly where the >2.1B cadence + the ~10.9B UAF live)

static HsSlot   hs_set[HS_N];             // the live-set (≈ 360 KB)
static HsLatch  hs_latch[HS_LATCH_N];
static int      hs_latch_n = 0;
static uint32_t hs_alloc_n = 0;           // total allocs recorded
static uint32_t hs_free_n  = 0;           // total frees that hit a live slot
static uint32_t hs_live_n  = 0;           // current live count
static uint32_t hs_badfree_n = 0;         // bad-free events seen
static int      hs_overflow = 0;          // live-set saturated (>0.85)
static int64_t  hs_sample_next = 0;       // WR-01: int64_t (the re-arm step + period must not wrap on wasm32)

// Provide the exact backtrace via telemetry.c's shadow call stack (D-06.2 /
// Don't-Hand-Roll — replaces the old flash-word-scan heuristic). The strong
// definition lives in telemetry.c; a weak fallback here keeps heap_shadow.c
// linkable on its own (returns 0 = "call stack not armed").
__attribute__((weak)) int telemetry_backtrace(uint32_t* out, int max) {
    (void)out; (void)max; return 0;
}

// Lazily probe the env gates ONCE (the gated-OFF idiom, boot_trace.c:914-915).
static void hs_probe(void) {
    if (hs_shadow_on == -2) hs_shadow_on = getenv("HEAPSHADOW") ? 1 : 0;
    if (hs_guard_on  == -2) hs_guard_on  = getenv("HEAPGUARD")  ? 1 : 0;
    if (hs_sample_every == -2) {
        hs_sample_every = getenv("HEAPSAMPLE") ? atol(getenv("HEAPSAMPLE")) : 0;
        if (getenv("HEAPSAMPLE") && hs_sample_every <= 0) hs_sample_every = 10000000;
        hs_sample_next = 0;
    }
    // Per-model hook-PC overrides (heap fns aren't sig-supplied). If only ALLOC is
    // given, point BMALLOC at it too (single-wrapper models like 3410 v5.46).
    if (getenv("HEAPSHADOW_ALLOC")) hs_pc_alloc   = (uint32_t)strtoul(getenv("HEAPSHADOW_ALLOC"), NULL, 0);
    if (getenv("HEAPSHADOW_FREE"))  hs_pc_free    = (uint32_t)strtoul(getenv("HEAPSHADOW_FREE"),  NULL, 0);
    if (getenv("HEAPSHADOW_BMALLOC")) hs_pc_bmalloc = (uint32_t)strtoul(getenv("HEAPSHADOW_BMALLOC"), NULL, 0);
    else if (getenv("HEAPSHADOW_ALLOC")) hs_pc_bmalloc = hs_pc_alloc;
}

int heap_shadow_armed(void) {
    hs_probe();
    return (hs_shadow_on || hs_guard_on || hs_sample_every > 0) ? 1 : 0;
}

// --- web cwrap arming (plan 04) ----------------------------------------------
// The native CLI arms each instrument from a getenv probe (above). The web driver
// has no env, so the dev panel arms them through these setters instead — they
// flip the SAME static gates the env probe sets, so the instrument is still gated
// OFF by default (the panel must opt in) and the no-reimplementation rule holds.
// Calling with on=1 resets the live-set so a run starts from a clean slate.
void heap_shadow_arm(int shadow, int guard) {
    hs_probe();                       // resolve any env first, then override
    hs_shadow_on = shadow ? 1 : 0;
    hs_guard_on  = guard  ? 1 : 0;
    // WR-04: the bad-free cap (hs_badfree_n vs HS_BADFREE_CAP) is the report-budget
    // gate the GUARD path also reads (heap_shadow_step :302). Reset it whenever EITHER
    // gate is (re)armed — not only on shadow — so a stale prior run's saturated cap does
    // not silently drop the next run's poison-free. A guard-only re-arm — heap_shadow_arm(0,1)
    // — must clear the cap; before WR-04 this reset was inside the hs_shadow_on-only block.
    if (hs_shadow_on || hs_guard_on) {
        hs_badfree_n = 0;
        hs_overflow  = 0;
    }
    if (hs_shadow_on) {
        // The full live-set wipe stays shadow-gated (the guard path only READS the
        // live-set for provenance, :308-314; it never depends on a cleared table).
        memset(hs_set, 0, sizeof(hs_set));
        hs_latch_n = 0;
        hs_alloc_n = hs_free_n = hs_live_n = 0;
    }
}
void heap_shadow_set_sample(long every) {
    hs_probe();
    hs_sample_every = every > 0 ? every : 0;
    hs_sample_next  = 0;
}
// Current live (outstanding) block count — for the web leak_count export.
double heap_shadow_live_d(void)    { return (double)hs_live_n; }
// (heap_shadow_used_d is defined just after hs_read_used below.)

// Read [0x10484C] used-bytes as a big-endian u32 through the model's RAM backing.
static uint32_t hs_read_used(Mad2* m) {
    if (!m || !m->mem) return 0;
    uint32_t a = (HS_ACCT_STRUCT + 8u);
    uint32_t k = m->mem_mask;
    return ((uint32_t)m->mem[a & k] << 24)
         | ((uint32_t)m->mem[(a + 1) & k] << 16)
         | ((uint32_t)m->mem[(a + 2) & k] << 8)
         |  (uint32_t)m->mem[(a + 3) & k];
}

// Live [0x10484C] used-bytes (the heap curve point) — for the web heap_used export.
double heap_shadow_used_d(Mad2* m) { return (double)hs_read_used(m); }

// Insert / update a live block (open-addressing, linear-probe). A re-alloc of a ptr
// whose slot is FREED reuses the slot (the allocator handed the address back out).
static void hs_record(uint32_t ptr, uint32_t lr, uint32_t size, uint32_t step_lo) {
    if (ptr == 0) return;                          // NULL alloc (failed) — nothing to track
    uint32_t h = (ptr >> 2) & HS_MASK;             // heap ptrs are well-distributed
    for (uint32_t i = 0; i < HS_N; ++i) {
        uint32_t s = (h + i) & HS_MASK;
        if (hs_set[s].state == HS_EMPTY || hs_set[s].ptr == ptr) {
            if (hs_set[s].state != HS_LIVE) {       // empty or freed -> a new live block
                if (hs_set[s].state == HS_EMPTY && hs_live_n >= HS_MAXLOAD) {
                    hs_overflow = 1; return;        // never grow mid-run
                }
                hs_live_n++;
            }
            hs_set[s].ptr = ptr;
            hs_set[s].alloc_site = lr;
            hs_set[s].size = size;
            hs_set[s].step_lo = step_lo;
            hs_set[s].state = HS_LIVE;
            // prev_free_site retained from the prior free of this slot (provenance).
            hs_alloc_n++;
            return;
        }
    }
    hs_overflow = 1;
}

// Free a ptr. Returns the slot index (or -1 if the ptr is not tracked / already
// freed = a BAD FREE). MARKs the block freed (RETAIN alloc_site for the report)
// rather than clearing it — coalesce (HEAP_BLOCK_WALK) merges adjacent free blocks,
// so a 1:1 future-alloc mapping cannot be assumed (Pitfall 5).
static int hs_free(uint32_t ptr, uint32_t free_site, int* was_double) {
    *was_double = 0;
    if (ptr == 0) return -1;
    uint32_t h = (ptr >> 2) & HS_MASK;
    for (uint32_t i = 0; i < HS_N; ++i) {
        uint32_t s = (h + i) & HS_MASK;
        if (hs_set[s].state == HS_EMPTY) return -1;  // not present (linear-probe stop)
        if (hs_set[s].ptr == ptr) {
            if (hs_set[s].state == HS_FREED) {       // double free
                *was_double = 1;
                hs_set[s].prev_free_site = free_site;
                return (int)s;
            }
            hs_set[s].state = HS_FREED;              // MARK freed, retain alloc_site
            hs_set[s].prev_free_site = free_site;
            hs_free_n++;
            if (hs_live_n) hs_live_n--;
            return (int)s;
        }
    }
    return -1;
}

// Print the ASan-style bad-free report (D-06.1) to stderr, with the EXACT backtrace
// from telemetry.c's shadow call stack (falls back to "(call stack not armed)").
static void hs_report_badfree(Mad2* m, struct ARMCore* cpu, uint32_t ptr,
                              int slot, int is_double, const char* trap, int64_t step) {
    uint32_t free_lr = (uint32_t)cpu->gprs[ARM_LR] & ~1u;
    uint32_t sp      = (uint32_t)cpu->gprs[ARM_SP];
    fprintf(stderr, "\n=== %s @step %lld ===\n", trap, (long long)step);
    fprintf(stderr, "  free(0x%08X)  via HEAP_FREE  caller LR=0x%08X\n", ptr, free_lr);
    if (slot >= 0) {
        HsSlot* b = &hs_set[slot];
        fprintf(stderr, "  freed ptr 0x%08X: originally allocated at 0x%06X (size %u, step %u)",
                ptr, b->alloc_site & ~1u, b->size, b->step_lo);
        if (b->prev_free_site && is_double)
            fprintf(stderr, ", previously freed at 0x%06X", b->prev_free_site & ~1u);
        fprintf(stderr, "%s\n", is_double ? "  [DOUBLE FREE]" : "");
    } else {
        fprintf(stderr, "  freed ptr 0x%08X: NOT in the live-set (free of unallocated / pre-arm)\n", ptr);
    }
    fprintf(stderr, "  regs: r0=%08X r1=%08X r2=%08X r3=%08X r4=%08X r5=%08X r6=%08X sp=%08X\n",
            (uint32_t)cpu->gprs[0], (uint32_t)cpu->gprs[1], (uint32_t)cpu->gprs[2],
            (uint32_t)cpu->gprs[3], (uint32_t)cpu->gprs[4], (uint32_t)cpu->gprs[5],
            (uint32_t)cpu->gprs[6], sp);
    // Exact backtrace from the shadow call stack (telemetry.c). Replaces the old
    // flash-word-scan heuristic (Don't-Hand-Roll — the scan misfires under tail-calls).
    uint32_t frames[32];
    int nf = telemetry_backtrace(frames, 32);
    if (nf > 0) {
        fprintf(stderr, "  backtrace (shadow call stack, innermost first):\n    %06X(LR)", free_lr);
        for (int k = nf - 1; k >= 0; --k) fprintf(stderr, " %06X", frames[k] & ~1u);
        fprintf(stderr, "\n");
    } else {
        fprintf(stderr, "  backtrace: (shadow call stack not armed — set BRANCHRING=1)\n");
    }
    (void)m;
}

// Per-step alloc/free hook dispatch. Called from harness_observe (gated). `pc` is the
// de-piped PC; `step` is the monotonic instruction counter. Returns 1 if a FAULT_HALT
// should be raised (HEAPGUARD bad-free with stop enabled), 0 otherwise.
int heap_shadow_step(Mad2* m, struct ARMCore* cpu, uint32_t pc, int64_t step) {
    hs_probe();
    int halt = 0;

    // (1) Allocator shadow — alloc entry/return latch + free.
    if (hs_shadow_on) {
        if (pc == HS_HEAP_ALLOC || pc == HS_BLOCKING_MALLOC) {
            // Alloc ENTRY: latch {ret_pc, sp, lr, size}. Depth-N LIFO; if full (>N
            // nested allocs) we simply don't track the innermost (never grow mid-run).
            if (hs_latch_n < HS_LATCH_N) {
                HsLatch* L = &hs_latch[hs_latch_n++];
                L->lr     = (uint32_t)cpu->gprs[ARM_LR];
                L->size   = (uint32_t)cpu->gprs[0];
                L->ret_pc = (uint32_t)cpu->gprs[ARM_LR] & ~1u;
                L->sp     = (uint32_t)cpu->gprs[ARM_SP];
            }
        } else if (hs_latch_n > 0) {
            // Alloc RETURN: the wrapper returned to ret_pc with r0 = ptr and the
            // stack popped (sp >= the latched entry sp). Match the NEWEST latch first
            // (LIFO) to handle nesting. r0 = the freshly-returned ptr.
            uint32_t sp = (uint32_t)cpu->gprs[ARM_SP];
            for (int k = hs_latch_n - 1; k >= 0; --k) {
                HsLatch* L = &hs_latch[k];
                if (pc == L->ret_pc && sp >= L->sp) {
                    hs_record((uint32_t)cpu->gprs[0], L->lr, L->size, (uint32_t)step);
                    for (int j = k; j < hs_latch_n - 1; ++j) hs_latch[j] = hs_latch[j + 1];
                    hs_latch_n--;
                    break;
                }
            }
        }
        if (pc == HS_HEAP_FREE) {                    // free ENTRY: r0 = ptr to free
            uint32_t ptr = (uint32_t)cpu->gprs[0];
            uint32_t free_site = (uint32_t)cpu->gprs[ARM_LR];
            // HEAP_FREE legitimately receives non-heap pointers (flash/ROM
            // 0x2xxxxx-0x3xxxxx) which it safely no-ops; only classify a bad free for
            // a ptr that LOOKS like a heap address (top byte != 0x00/0x2x/0x3x) OR is
            // a poison ptr. A free of a flash/ROM ptr is a no-op, not a bug.
            int is_double = 0;
            int slot = hs_free(ptr, free_site, &is_double);
            int poison = (((ptr >> 24) & 0xFFu) == 0xAAu);
            if ((is_double || poison) && hs_badfree_n < HS_BADFREE_CAP) {
                hs_badfree_n++;
                hs_report_badfree(m, cpu, ptr, slot,
                                  is_double, is_double ? "HEAPSHADOW: DOUBLE FREE"
                                                       : "HEAPSHADOW: POISON FREE", step);
            }
        }
    }

    // (2) HEAPGUARD poison-free trap (D-07; moved from boot_trace.c). Trap ONLY the
    // 0xAA top byte — a valid ptr never matches. Independent of the shadow so it works
    // even when HEAPSHADOW is off.
    if (hs_guard_on && pc == HS_HEAP_FREE && hs_badfree_n < HS_BADFREE_CAP) {
        uint32_t ptr = (uint32_t)cpu->gprs[0];
        if (((ptr >> 24) & 0xFFu) == 0xAAu) {        // poison/wild pointer = UAF / double-free
            if (!hs_shadow_on) {                     // shadow already reported it if both on
                int dummy = 0;
                int slot = -1;
                // Look the ptr up for provenance without mutating the shadow state.
                uint32_t h = (ptr >> 2) & HS_MASK;
                for (uint32_t i = 0; i < HS_N; ++i) {
                    uint32_t s = (h + i) & HS_MASK;
                    if (hs_set[s].state == HS_EMPTY) break;
                    if (hs_set[s].ptr == ptr) { slot = (int)s; break; }
                }
                hs_badfree_n++;
                hs_report_badfree(m, cpu, ptr, slot, 0,
                                  "HEAPGUARD: BAD FREE (poison/wild pointer)", step);
                (void)dummy;
            }
            if (!getenv("HEAPGUARD_NOSTOP")) halt = 1;   // FAULT_HALT (heap corruption, D-11)
        }
    }

    // (3) HEAPSAMPLE: the [0x10484C] used-bytes curve. Cadence on the step counter,
    // NEVER raw cpu->cycles (it rebases every 0x20000000 — Landmine #4).
    if (hs_sample_every > 0 && step >= hs_sample_next) {
        fprintf(stderr, "[heapsample] step=%lld used=%u (0x%X)\n",
                (long long)step, hs_read_used(m), hs_read_used(m));
        hs_sample_next = step + hs_sample_every;
    }

    return halt;
}

// Dump the live-set summary into the fault report (D-06.1). Called by
// harness_fault_report AFTER mad2_render_postmortem (append-only — never edits the
// byte-stable post-mortem body). No-op when the shadow was never armed.
void heap_shadow_dump(Mad2* m) {
    hs_probe();
    if (!hs_shadow_on) return;
    fprintf(stderr, "\n--- allocator shadow (live-set) ---\n");
    fprintf(stderr, "  allocs=%u  frees=%u  live=%u  bad-frees=%u  used=%u (0x%X)%s\n",
            hs_alloc_n, hs_free_n, hs_live_n, hs_badfree_n,
            hs_read_used(m), hs_read_used(m),
            hs_overflow ? "  [LIVE-SET SATURATED]" : "");
    // The top live blocks by alloc-site (the leak candidates) — a small aggregation.
    // Keep it bounded: aggregate by alloc_site over a single pass into a tiny table.
    #define HS_AGG 16
    uint32_t agg_site[HS_AGG]; uint32_t agg_cnt[HS_AGG]; uint32_t agg_sz[HS_AGG];
    int agg_n = 0;
    for (uint32_t s = 0; s < HS_N; ++s) {
        if (hs_set[s].state != HS_LIVE) continue;
        int found = 0;
        for (int k = 0; k < agg_n; ++k)
            if (agg_site[k] == hs_set[s].alloc_site) { agg_cnt[k]++; found = 1; break; }
        if (!found && agg_n < HS_AGG) {
            agg_site[agg_n] = hs_set[s].alloc_site;
            agg_cnt[agg_n] = 1; agg_sz[agg_n] = hs_set[s].size; agg_n++;
        }
    }
    for (int k = 0; k < agg_n; ++k)
        fprintf(stderr, "    live x%-6u  alloc-site=0x%06X  (eg size %u)\n",
                agg_cnt[k], agg_site[k] & ~1u, agg_sz[k]);
}

// Web text dump (plan 04): the same allocator-shadow summary formatted into a static
// buffer the web cwrap can hand to JS as HEAPU8.subarray(ptr, ptr+len). Mirrors
// heap_shadow_dump's content; returns the buffer pointer + writes *len. The live-set
// is the shared owner of the leak data (no web-side reimplementation).
const char* heap_shadow_text(Mad2* m, unsigned* len) {
    static char buf[8192];
    unsigned n = 0;
    n += (unsigned)snprintf(buf + n, sizeof(buf) - n,
        "=== ALLOCATOR SHADOW (live-set) ===\n"
        "  allocs=%u  frees=%u  live=%u  bad-frees=%u  used=%u (0x%X)%s\n",
        hs_alloc_n, hs_free_n, hs_live_n, hs_badfree_n,
        hs_read_used(m), hs_read_used(m),
        hs_overflow ? "  [LIVE-SET SATURATED]" : "");
    // Aggregate live blocks by alloc-site (the leak candidates).
    #define HSW_AGG 16
    uint32_t agg_site[HSW_AGG]; uint32_t agg_cnt[HSW_AGG]; uint32_t agg_sz[HSW_AGG]; uint32_t agg_bytes[HSW_AGG];
    int agg_n = 0;
    for (uint32_t s = 0; s < HS_N; ++s) {
        if (hs_set[s].state != HS_LIVE) continue;
        int found = 0;
        for (int k = 0; k < agg_n; ++k)
            if (agg_site[k] == hs_set[s].alloc_site) { agg_cnt[k]++; agg_bytes[k] += hs_set[s].size; found = 1; break; }
        if (!found && agg_n < HSW_AGG) {
            agg_site[agg_n] = hs_set[s].alloc_site;
            agg_cnt[agg_n] = 1; agg_sz[agg_n] = hs_set[s].size;
            agg_bytes[agg_n] = hs_set[s].size; agg_n++;
        }
    }
    for (int k = 0; k < agg_n && n < sizeof(buf) - 96; ++k)
        n += (unsigned)snprintf(buf + n, sizeof(buf) - n,
            "  live x%-6u  alloc-site=0x%06X  bytes=%u (eg size %u)\n",
            agg_cnt[k], agg_site[k] & ~1u, agg_bytes[k], agg_sz[k]);
    if (len) *len = n;
    return buf;
}
