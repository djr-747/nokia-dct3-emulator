// Telemetry instruments (Phase 8, H0 part 2 — D-06.2/.3/.4/.5). Four instruments,
// built ONCE in the shared harness, gated OFF by default (the canonical SWDSP post-
// mortem stays byte-identical; both drivers inherit them — no reimplementation):
//
//   (1) Shadow call stack (D-06.2, BRANCHRING=1) — a parallel fixed-capacity return-
//       address stack maintained from the typed branch ring (BR_CALL push site PC,
//       BR_RETURN/BR_BX pop). Gives an EXACT backtrace at any FAULT_HALT, replacing
//       the flash-word-scan heuristic (Don't-Hand-Roll). heap_shadow.c's HEAPGUARD
//       backtrace calls telemetry_backtrace().
//
//   (2) Last-writer provenance ring (D-06.4, LASTWRITER=0xADDR[,LEN]) — polls a
//       watched RAM range each step and records (pc, lr, value, step) on a change.
//       Answers "who poisoned this word?" — base on the RAMWATCH poll + the per-PC
//       aggregation. Pre-sized ring.
//
//   (3) Interrupt + CCONT event log (D-06.5, EVENTLOG=1) — stamps each IRQ/FIQ raise
//       and the CCONT line/mask/power state with rtc_mono into a ring. Feeds the
//       FIQ-reentrancy correlation. Driver calls telemetry_event_irq() at each raise.
//
//   (4) Periodic invariant checker (D-06.3, INVARIANT=1) — COMPOSED (no single
//       analog): heap-accounting [0x10484C] vs the freelist/soft-timer head
//       [0x11182C], SP-in-bounds, PC-in-flash, soft-timer list integrity. Trips at
//       corruption ONSET (not detonation) and signals FAULT_HALT on a tripped
//       invariant. Cadence on rtc_mono.
//
// Observation-only invariant: reads ONLY regs + guest RAM (m->mem); NEVER writes
// guest RAM, cpu->gprs, or cpu->cycles. No host allocation inside a hook — all
// rings are pre-sized and never grow mid-run.

#include "harness/harness.h"
#include <mgba/internal/arm/arm.h>   // ARM_LR / ARM_SP
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- composed-invariant targets (symbol store) -------------------------------
#define TLM_ACCT_USED   0x0010484Cu   // heap accounting used-bytes (BE u32) = 0x104844 + 8
#define TLM_FREELIST     0x0011182Cu   // freelist / soft-timer list head (invariant target)
#define TLM_FLASH_LO     0x00200000u   // flash code window
#define TLM_FLASH_HI     0x00400000u   // wild-PC bound (PC must stay below this)
#define TLM_RAM_LO       0x00100000u   // approximate stack/RAM band (SP-in-bounds heuristic)
#define TLM_RAM_HI       0x00200000u

// ============================================================================
// (1) Shadow call stack — parallel return-address stack from the typed branch ring.
// ============================================================================
#define TLM_BR_N      1024u            // typed branch ring depth (power-of-two)
#define TLM_BR_MASK   (TLM_BR_N - 1u)
#define TLM_CS_N      256              // shadow call stack depth (return addresses)

// Branch type codes (mirrors src/web/main.c branch ring).
#define TLM_BR_B      0u
#define TLM_BR_BCOND  1u
#define TLM_BR_CALL   2u
#define TLM_BR_RETURN 3u
#define TLM_BR_BX     4u
#define TLM_BR_EXC    5u

typedef struct {
    uint32_t step_lo;
    uint32_t pc;        // branch-site PC
    uint32_t target;    // post-branch PC
    uint32_t lr;
    uint32_t sp;
    uint16_t depth;
    uint8_t  type;
    uint8_t  cpsr_mode;
} TlmBranch;

static int       tlm_branch_on = -2;        // -2 unprobed
static TlmBranch tlm_bring[TLM_BR_N];
static uint32_t  tlm_bring_w = 0;
static uint32_t  tlm_expected_next = 0;     // sequential next-PC after the previous insn
static uint16_t  tlm_depth = 0;             // running call depth
static uint32_t  tlm_cstack[TLM_CS_N];      // shadow return-address stack (the exact backtrace)
static int       tlm_cs_n = 0;

// ============================================================================
// (2) Last-writer provenance ring.
// ============================================================================
#define TLM_LW_LEN_MAX  256u
#define TLM_LW_RING_N   256u            // (pc,lr,value,step) records (power-of-two)
#define TLM_LW_MASK     (TLM_LW_RING_N - 1u)

typedef struct {
    uint32_t pc; uint32_t lr; uint32_t value; uint32_t step_lo; uint32_t addr;
    uint8_t  old_b; uint8_t new_b;
} TlmLastWriter;

static int           tlm_lw_on = -2;
static uint32_t      tlm_lw_addr = 0;
static uint32_t      tlm_lw_len  = 1;
static uint8_t       tlm_lw_prev[TLM_LW_LEN_MAX];
static int           tlm_lw_init = 0;
static TlmLastWriter tlm_lw_ring[TLM_LW_RING_N];
static uint32_t      tlm_lw_w = 0;
// WR-02: the PC of the instruction that performed the store. A change detected at
// loop-top in telemetry_step is observed AFTER the storing instruction's dct3_step,
// so the current de-piped `pc` is the NEXT-to-execute instruction, not the writer.
// Track the previous step's PC (the storing instruction) and attribute the change to
// it — mirrors boot_trace.c's [ramw] prev_pc idiom. Seeded on the lw-init path so the
// first detected change attributes a real prior instruction, not PC=0.
static uint32_t      tlm_lw_prev_pc = 0;

// ============================================================================
// (3) Interrupt + CCONT event log.
// ============================================================================
#define TLM_EV_N    512u
#define TLM_EV_MASK (TLM_EV_N - 1u)

typedef struct {
    uint64_t cyc;       // rtc_mono at the raise
    uint32_t step_lo;
    uint8_t  irq_n;     // IRQ/FIQ line number
    uint8_t  is_fiq;    // 1 = FIQ, 0 = IRQ
    uint8_t  cc_lines;  // CCONT reg 0x0E pending at the raise
    uint8_t  cc_mask;   // CCONT reg 0x0F mask
    uint8_t  power_off; // CCONT power state latch
} TlmEvent;

static int      tlm_ev_on = -2;
static TlmEvent tlm_ev_ring[TLM_EV_N];
static uint32_t tlm_ev_w = 0;

// ============================================================================
// (4) Periodic invariant checker.
// ============================================================================
#define TLM_INV_EVERY_CYC  (13000000ull)   // ~1 s on rtc_mono (13 MHz)
static int      tlm_inv_on = -2;
static uint64_t tlm_inv_next_cyc = 0;
static int      tlm_inv_tripped = 0;
static char     tlm_inv_msg[160];

// --- env probe (gated-OFF idiom, once) ---------------------------------------
static void tlm_probe(void) {
    if (tlm_branch_on == -2) tlm_branch_on = getenv("BRANCHRING") ? 1 : 0;
    if (tlm_inv_on == -2)    tlm_inv_on    = getenv("INVARIANT")  ? 1 : 0;
    if (tlm_ev_on == -2)     tlm_ev_on     = getenv("EVENTLOG")   ? 1 : 0;
    if (tlm_lw_on == -2) {
        const char* e = getenv("LASTWRITER");
        if (e) {
            tlm_lw_on = 1;
            tlm_lw_addr = (uint32_t)strtoul(e, NULL, 0);
            const char* comma = strchr(e, ',');
            tlm_lw_len = comma ? (uint32_t)strtoul(comma + 1, NULL, 0) : 1u;
            if (tlm_lw_len == 0 || tlm_lw_len > TLM_LW_LEN_MAX) tlm_lw_len = 1;
        } else {
            tlm_lw_on = 0;
        }
    }
}

int telemetry_armed(void) {
    tlm_probe();
    return (tlm_branch_on || tlm_inv_on || tlm_ev_on || tlm_lw_on) ? 1 : 0;
}

// --- web cwrap arming (plan 04) ----------------------------------------------
// The CLI arms these from getenv (above); the web dev panel arms them through these
// setters, flipping the SAME static gates. Still gated OFF by default (the panel
// opts in). on=1 resets the relevant ring so a run starts clean.
void telemetry_arm_branchring(int on) {
    tlm_probe();
    tlm_branch_on = on ? 1 : 0;
    if (tlm_branch_on) { tlm_bring_w = 0; tlm_depth = 0; tlm_expected_next = 0; tlm_cs_n = 0; }
}
void telemetry_arm_invariant(int on) { tlm_probe(); tlm_inv_on = on ? 1 : 0; if (on) { tlm_inv_tripped = 0; tlm_inv_next_cyc = 0; } }
void telemetry_arm_eventlog(int on)  { tlm_probe(); tlm_ev_on  = on ? 1 : 0; if (on) tlm_ev_w = 0; }
void telemetry_set_lastwriter(uint32_t addr, uint32_t len) {
    tlm_probe();
    if (addr == 0) { tlm_lw_on = 0; return; }
    tlm_lw_on = 1; tlm_lw_addr = addr;
    tlm_lw_len = (len == 0 || len > TLM_LW_LEN_MAX) ? 1u : len;
    tlm_lw_init = 0; tlm_lw_w = 0;
}
// Taken-branch count (for the web branch_count export).
double   telemetry_branch_count(void) { return (double)tlm_bring_w; }

// BE u32 read through the model's RAM backing.
static uint32_t tlm_rd32(Mad2* m, uint32_t a) {
    if (!m || !m->mem) return 0;
    uint32_t k = m->mem_mask;
    return ((uint32_t)m->mem[a & k] << 24) | ((uint32_t)m->mem[(a + 1) & k] << 16)
         | ((uint32_t)m->mem[(a + 2) & k] << 8) | (uint32_t)m->mem[(a + 3) & k];
}

// --- (1) shadow call stack: backtrace export (called by heap_shadow.c) -------
// Fills out[] with the live return-address stack (oldest..newest). Returns the
// number of frames written (0 = the ring was never armed).
int telemetry_backtrace(uint32_t* out, int max) {
    tlm_probe();
    if (!tlm_branch_on || tlm_cs_n == 0) return 0;
    int n = tlm_cs_n < max ? tlm_cs_n : max;
    for (int i = 0; i < n; ++i) out[i] = tlm_cstack[i];
    return n;
}

// --- (1) shadow call stack: post-step branch decode + push/pop ---------------
// Mirrors the web typed-branch decode (main.c:1052-1135). Called AFTER dct3_step so
// the de-piped PC is final for the instruction we just ran. On a CALL we push the
// call-site PC onto the shadow stack; on RETURN/BX we pop. Observation-only.
void telemetry_post_branch(Mad2* m, struct ARMCore* cpu, int64_t step) {
    tlm_probe();
    if (!tlm_branch_on) return;
    if (!m || !m->mem) return;
    uint32_t k = m->mem_mask;
    uint32_t t  = cpu->cpsr.t ? 1u : 0u;
    uint32_t pc = (uint32_t)cpu->gprs[15] - (t ? 4u : 8u);

    if (tlm_expected_next != 0 && pc != tlm_expected_next) {
        uint32_t exc = (pc < 0x40u) ? 1u : 0u;
        uint32_t type = TLM_BR_B;
        uint32_t site = 0;
        if (exc) {
            type = TLM_BR_EXC;
            site = tlm_expected_next - 2u;
        } else {
            uint32_t s2  = tlm_expected_next - 2u;
            uint32_t h   = ((uint32_t)m->mem[s2 & k] << 8) | (uint32_t)m->mem[(s2 + 1) & k];
            uint32_t s2p = tlm_expected_next - 4u;
            uint32_t hp  = ((uint32_t)m->mem[s2p & k] << 8) | (uint32_t)m->mem[(s2p + 1) & k];
            if ((hp & 0xF800u) == 0xF000u && (h & 0xF800u) == 0xF800u) {
                type = TLM_BR_CALL;   site = s2p;            // 32-bit Thumb BL pair
            } else if ((h & 0xFF87u) == 0x4700u) {
                type = TLM_BR_BX;     site = s2;             // BX
            } else if ((h & 0xFF00u) == 0xBD00u) {
                type = TLM_BR_RETURN; site = s2;             // POP {..,pc}
            } else if ((h & 0xF800u) == 0xE000u) {
                type = TLM_BR_B;      site = s2;             // unconditional B
            } else if ((h & 0xF000u) == 0xD000u && ((h >> 8) & 0xFu) < 0xEu) {
                type = TLM_BR_BCOND;  site = s2;             // B<cond> taken
            } else {
                uint32_t s4 = tlm_expected_next - 4u;
                uint32_t w  = ((uint32_t)m->mem[s4 & k] << 24) | ((uint32_t)m->mem[(s4 + 1) & k] << 16)
                            | ((uint32_t)m->mem[(s4 + 2) & k] << 8) | (uint32_t)m->mem[(s4 + 3) & k];
                if ((w & 0x0FFFFFF0u) == 0x012FFF10u) {
                    type = TLM_BR_BX;   site = s4;           // ARM BX
                } else if ((w & 0x0E000000u) == 0x0A000000u) {
                    site = s4;
                    type = (w & 0x01000000u) ? TLM_BR_CALL
                         : (((w >> 28) & 0xFu) != 0xEu) ? TLM_BR_BCOND : TLM_BR_B;
                } else {
                    type = TLM_BR_BX;   site = s2;
                }
            }
        }
        // Maintain the shadow call stack: push the call-site on CALL, pop on RETURN/BX.
        if (type == TLM_BR_CALL) {
            tlm_depth++;
            if (tlm_cs_n < TLM_CS_N) tlm_cstack[tlm_cs_n++] = site;
        } else if (type == TLM_BR_RETURN || type == TLM_BR_BX) {
            if (tlm_depth) tlm_depth--;
            if (tlm_cs_n) tlm_cs_n--;
        }
        TlmBranch* be = &tlm_bring[tlm_bring_w & TLM_BR_MASK];
        be->step_lo = (uint32_t)step; be->pc = site; be->target = pc;
        be->lr = (uint32_t)cpu->gprs[ARM_LR]; be->sp = (uint32_t)cpu->gprs[ARM_SP];
        be->depth = tlm_depth; be->type = (uint8_t)type;
        be->cpsr_mode = (uint8_t)(cpu->cpsr.packed & 0x1Fu);
        tlm_bring_w++;
    }
    tlm_expected_next = pc + (t ? 2u : 4u);
}

// --- (3) IRQ/CCONT event log: stamp a raise (called by the driver) -----------
void telemetry_event_irq(Mad2* m, int irq_n, int is_fiq, int64_t step) {
    tlm_probe();
    if (!tlm_ev_on || !m) return;
    TlmEvent* e = &tlm_ev_ring[tlm_ev_w & TLM_EV_MASK];
    e->cyc = m->rtc_mono; e->step_lo = (uint32_t)step;
    e->irq_n = (uint8_t)irq_n; e->is_fiq = (uint8_t)(is_fiq ? 1 : 0);
    e->cc_lines = m->cc_int_lines; e->cc_mask = m->cc_int_mask; e->power_off = m->power_off;
    tlm_ev_w++;
}

// --- per-step pre-dispatch: last-writer poll + periodic invariant check -------
// Called from harness_observe (gated). Returns 1 if an invariant tripped (FAULT_HALT
// at corruption ONSET). The shadow call stack is updated POST-step via
// telemetry_post_branch (the driver calls it after dct3_step).
int telemetry_step(Mad2* m, struct ARMCore* cpu, uint32_t pc, int64_t step) {
    tlm_probe();
    int halt = 0;

    // (2) Last-writer provenance ring — poll the watched range on change.
    if (tlm_lw_on && m && m->mem) {
        uint32_t kk = m->mem_mask;
        if (!tlm_lw_init) {
            for (uint32_t i = 0; i < tlm_lw_len; ++i)
                tlm_lw_prev[i] = m->mem[(tlm_lw_addr + i) & kk];
            tlm_lw_init = 1;
            tlm_lw_prev_pc = pc;   // WR-02: seed so the first change has a real prior PC
        } else {
            for (uint32_t i = 0; i < tlm_lw_len; ++i) {
                uint8_t v = m->mem[(tlm_lw_addr + i) & kk];
                if (v != tlm_lw_prev[i]) {
                    TlmLastWriter* r = &tlm_lw_ring[tlm_lw_w & TLM_LW_MASK];
                    r->addr = tlm_lw_addr + i; r->old_b = tlm_lw_prev[i]; r->new_b = v;
                    r->value = v; r->pc = tlm_lw_prev_pc; r->lr = (uint32_t)cpu->gprs[ARM_LR];
                    r->step_lo = (uint32_t)step;
                    tlm_lw_w++;
                    tlm_lw_prev[i] = v;
                }
            }
        }
    }

    // (4) Periodic invariant checker — cadence on rtc_mono, trip at ONSET. Skip the
    // boot-ROM warmup window (the reset-vector branch at step 0 leaves a transient
    // de-piped PC = 0xFFFFFFFC before the HLE pipeline is primed — same artifact the
    // wild-PC detector guards with a 16-step warmup; not a firmware fault).
    if (tlm_inv_on && m && !tlm_inv_tripped && step >= 16) {
        if (m->rtc_mono >= tlm_inv_next_cyc) {
            tlm_inv_next_cyc = m->rtc_mono + TLM_INV_EVERY_CYC;
            uint32_t sp = (uint32_t)cpu->gprs[ARM_SP];
            // (a) PC-in-flash (the wild-PC bound) — corruption that left the code window.
            // MODEL-AWARE upper bound (see harness/fault.c): 4 MB images run code past the
            // 2 MB TLM_FLASH_HI default, so derive the ceiling from the model's flash span.
            uint32_t flash_hi = m->model->mem.flash_base + m->model->mem.flash_size;
            if (pc >= flash_hi) {
                snprintf(tlm_inv_msg, sizeof(tlm_inv_msg),
                         "PC out of flash: 0x%08X (>= 0x%08X)", pc, flash_hi);
                tlm_inv_tripped = 1;
            }
            // (b) SP-in-bounds — a wild SP precedes most heap/stack smashes.
            else if (sp != 0 && (sp < TLM_RAM_LO || sp >= TLM_RAM_HI)) {
                snprintf(tlm_inv_msg, sizeof(tlm_inv_msg),
                         "SP out of bounds: 0x%08X (expect [0x%08X,0x%08X))",
                         sp, TLM_RAM_LO, TLM_RAM_HI);
                tlm_inv_tripped = 1;
            }
            // (c) Soft-timer / freelist head sanity: the head pointer, when set, must
            // point into the RAM band — a head poisoned to flash/garbage is corruption
            // onset (heap-acct [0x10484C] is read for context).
            else {
                uint32_t head = tlm_rd32(m, TLM_FREELIST);
                if (head != 0 && (head < TLM_RAM_LO || head >= TLM_FLASH_HI)) {
                    snprintf(tlm_inv_msg, sizeof(tlm_inv_msg),
                             "freelist/soft-timer head [0x11182C]=0x%08X out of RAM "
                             "(heap used=0x%X)", head, tlm_rd32(m, TLM_ACCT_USED));
                    tlm_inv_tripped = 1;
                }
            }
            if (tlm_inv_tripped) {
                fprintf(stderr, "\n=== INVARIANT TRIPPED @step %lld (corruption onset) ===\n  %s\n",
                        (long long)step, tlm_inv_msg);
                halt = 1;
            }
        }
    }

    // WR-02: maintain prev_pc at the END of the step — this PC is the instruction
    // whose dct3_step the driver is about to run, i.e. next iteration's storing PC.
    tlm_lw_prev_pc = pc;
    return halt;
}

// --- fault-report appends (called by harness_fault_report, AFTER post-mortem) -
void telemetry_dump(Mad2* m) {
    tlm_probe();

    // (1) shadow call stack — the exact backtrace.
    if (tlm_branch_on) {
        fprintf(stderr, "\n--- shadow call stack (depth %d, innermost last) ---\n", tlm_cs_n);
        int from = tlm_cs_n > 32 ? tlm_cs_n - 32 : 0;
        for (int i = from; i < tlm_cs_n; ++i)
            fprintf(stderr, "    #%-3d  0x%06X\n", i, tlm_cstack[i] & ~1u);
        // typed branch ring tail (last 16 taken branches) for control-flow context.
        uint32_t total = tlm_bring_w < TLM_BR_N ? tlm_bring_w : TLM_BR_N;
        uint32_t show = total < 16u ? total : 16u;
        uint32_t start = tlm_bring_w - show;
        fprintf(stderr, "  branch ring (last %u of %u):\n", show, tlm_bring_w);
        for (uint32_t i = 0; i < show; ++i) {
            TlmBranch* be = &tlm_bring[(start + i) & TLM_BR_MASK];
            static const char* tn[] = {"B   ","BCND","CALL","RET ","BX  ","EXC "};
            fprintf(stderr, "    %s %06X -> %06X  d%-3u lr=%06X (step %u)\n",
                    tn[be->type & 7], be->pc & ~1u, be->target & ~1u, be->depth,
                    be->lr & ~1u, be->step_lo);
        }
    }

    // (3) IRQ/CCONT event log — last events.
    if (tlm_ev_on) {
        uint32_t total = tlm_ev_w < TLM_EV_N ? tlm_ev_w : TLM_EV_N;
        uint32_t show = total < 24u ? total : 24u;
        uint32_t start = tlm_ev_w - show;
        fprintf(stderr, "\n--- IRQ/FIQ + CCONT event log (last %u of %u) ---\n", show, tlm_ev_w);
        for (uint32_t i = 0; i < show; ++i) {
            TlmEvent* e = &tlm_ev_ring[(start + i) & TLM_EV_MASK];
            fprintf(stderr, "    @%.3fM  %s%d  cc_lines=%02X cc_mask=%02X power_off=%u (step %u)\n",
                    (double)e->cyc / 1.0e6, e->is_fiq ? "FIQ" : "IRQ", e->irq_n,
                    e->cc_lines, e->cc_mask, e->power_off, e->step_lo);
        }
    }

    // (2) last-writer provenance ring.
    if (tlm_lw_on) {
        uint32_t total = tlm_lw_w < TLM_LW_RING_N ? tlm_lw_w : TLM_LW_RING_N;
        uint32_t show = total < 24u ? total : 24u;
        uint32_t start = tlm_lw_w - show;
        fprintf(stderr, "\n--- last-writer ring for [%08X..%08X) (last %u of %u) ---\n",
                tlm_lw_addr, tlm_lw_addr + tlm_lw_len, show, tlm_lw_w);
        for (uint32_t i = 0; i < show; ++i) {
            TlmLastWriter* r = &tlm_lw_ring[(start + i) & TLM_LW_MASK];
            fprintf(stderr, "    [%08X] %02X -> %02X  at PC=%06X LR=%06X (step %u)\n",
                    r->addr, r->old_b, r->new_b, r->pc & ~1u, r->lr & ~1u, r->step_lo);
        }
    }

    // (4) invariant checker verdict.
    if (tlm_inv_on) {
        fprintf(stderr, "\n--- periodic invariant checker ---\n  %s\n",
                tlm_inv_tripped ? tlm_inv_msg : "(all invariants held)");
    }
    (void)m;
}
