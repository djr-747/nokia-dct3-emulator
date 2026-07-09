/*
 * calypso_c54x.c — TMS320C54x DSP emulator for Calypso
 *
 * Minimal C54x core: enough to run the Calypso DSP ROM for GSM
 * signal processing (Viterbi, deinterleaving, burst decode).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "calypso_c54x.h"
#include "hw/arm/calypso/calypso_full_pcb.h"  /* daram_lock, api_ram_lock */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_boot_trace = 0;

#include "hw/arm/calypso/calypso_debug.h"

/* Nokia DCT3 HPI/mailbox bridge hooks. Small port addresses (PA < 0x100) are the
 * Nokia MAD host-mailbox ports (port 1/2/3 = HPI mailbox, 0x27 = codec, etc.),
 * NOT Calypso peripherals. When set, PORTR/PORTW route to the host (MCU) side. */
uint16_t (*nokia_port_read_hook)(uint16_t pa, uint16_t pc) = 0;
void     (*nokia_port_write_hook)(uint16_t pa, uint16_t val, uint16_t pc) = 0;

/* DSP->host HINT doorbell edge counter. In this Nokia DSP wiring, toggling BSCR
 * (data MMR 0x0029) bit3 is the "service me" doorbell to the MCU (-> IRQ4, handler
 * 0x2BCEF0 = the DSP mailbox). The store path increments this on each rising edge
 * so the host side can latch IRQ4. (The pulse is one instruction wide — set at
 * loader2 0xF30 / cleared at 0xF33 — so it must be caught at the write, not polled.) */
unsigned long long g_c54x_bscr_hint_edges = 0;

/* MFI host hook — MAD "Interface to COBBA AD/DA". The DSP writes the COBBA
 * parallel-bus CONTROL frames via mmr(22h)/mmr(32h); every observed value is
 * 0xC0xx = 4-bit COBBA register address (high nibble) + 12-bit data (the
 * service manual's "12 data bits, 4 address bits" parallel bus). The host
 * (mad2_dsp_c54x.c COBBA model) registers a callback to decode/latch them;
 * NULL = unmodelled (plain RAM-backed store, the historical behaviour). */
void (*g_c54x_mfi_write_cb)(uint16_t mmr, uint16_t val) = NULL;

/* ================================================================
 * DSP54_PERIPHMAP — peripheral-I/O surface map (default OFF, read-only)
 * ================================================================
 * Two surfaces, instrumented at the choke points:
 *   PORT — I/O port space (PORTR/PORTW, PA < 0x100): COBBA serial 0x2C/0x2D,
 *          COBBA parallel 0x30-0x3F, host mailbox 0x01-0x03, codec 0x21/0x27.
 *   MMR  — memory-mapped band data[0x00-0x5F]: CPU MMRs 0x00-0x1F + on-chip
 *          peripherals 0x20-0x5F (timer 0x24-0x26, BSCR 0x29, MFI 0x22/0x32,
 *          McBSP 0x38/0x39, DMA 0x54/0x55/0x57, CLKMD 0x58...). Recorded in
 *          the data_read/data_write wrappers; anything that falls through to
 *          the backing store is UNMODELED.
 * "modeled?" is STRUCTURAL, not value-based: an explicit handler case sets
 * g_c54x_pmap_handled / g_pmap_mmr_handled = 1; the default/backing-store
 * fall-through leaves it 0 (a real reg can legitimately read 0).
 * Env:
 *   DSP54_PERIPHMAP=1                  aggregate map, dumped at exit
 *   DSP54_PERIPHMAP_LO/_HI=<insn>      raw per-access log inside the window
 *   DSP54_PERIPHMAP_PA=0xNN            raw log filtered to one address
 * Cycle stamps are DSP insn_count (≈ dsp_steps at RATIO-paced cosim). */
#define PMAP_NPC  6
#define PMAP_NVAL 5
typedef struct {
    uint32_t hits, handled_hits;
    uint32_t first_insn;
    uint16_t first_pc;
    uint16_t pc[PMAP_NPC];   uint32_t pc_n[PMAP_NPC]; uint32_t pc_other;
    uint16_t val[PMAP_NVAL]; uint8_t  nval;           uint16_t last_val;
    uint8_t  npc, val_more;
} PmapCell;
static PmapCell g_pmap_port[2][0x100];   /* [0]=read [1]=write */
static PmapCell g_pmap_mmr[2][0x60];
static int      g_pmap_on = -1;          /* lazy: -1 unread, 0 off, 1 on */
static uint32_t g_pmap_lo, g_pmap_hi;    /* raw-mode insn window */
static int      g_pmap_raw;              /* raw mode armed (LO/HI/PA given) */
static long     g_pmap_pa = -1;          /* raw-mode PA filter (-1 = all) */
static uint32_t g_pmap_rawn;             /* raw lines emitted (capped) */
static uint32_t g_pmap_hi_pa_n;          /* PORT accesses with PA >= 0x100 */
static uint16_t g_pmap_hi_pa_first;
int g_c54x_pmap_handled;                 /* set by the port glue (mad2_dsp_c54x.c) */
static int g_pmap_mmr_handled;           /* set by data_read/write_locked handler cases */

static void c54x_pmap_dump(void)
{
    static const char *sname[2] = { "PORT", "MMR " };
    static const char *dname[2] = { "R", "W" };
    fprintf(stderr, "[pmap] ================ DSP peripheral-I/O surface map ================\n");
    fprintf(stderr, "[pmap] addr      R/W hits       first@insn(pc)      modeled  values(first..,last) pcs(top)\n");
    for (int sf = 0; sf < 2; sf++)
        for (int w = 0; w < 2; w++) {
            int n = sf ? 0x60 : 0x100;
            PmapCell *tab = sf ? g_pmap_mmr[w] : g_pmap_port[w];
            for (int a = 0; a < n; a++) {
                PmapCell *c = &tab[a];
                if (!c->hits) continue;
                const char *mod = (c->handled_hits == c->hits) ? "yes  "
                                : (c->handled_hits == 0)       ? "NO   " : "part ";
                fprintf(stderr, "[pmap] %s 0x%02X  %s  %-9u  %-10u(0x%04X)  %s  ",
                        sname[sf], a, dname[w], c->hits, c->first_insn, c->first_pc, mod);
                for (int i = 0; i < c->nval; i++) fprintf(stderr, "%04x ", c->val[i]);
                if (c->val_more) fprintf(stderr, "... ");
                fprintf(stderr, "last=%04x  |", c->last_val);
                for (int i = 0; i < c->npc; i++)
                    fprintf(stderr, " %04x:%u", c->pc[i], c->pc_n[i]);
                if (c->pc_other) fprintf(stderr, " other:%u", c->pc_other);
                fprintf(stderr, "\n");
            }
        }
    if (g_pmap_hi_pa_n)
        fprintf(stderr, "[pmap] PORT PA>=0x100: %u accesses (first PA=0x%04X) — outside the Nokia port map\n",
                g_pmap_hi_pa_n, g_pmap_hi_pa_first);
    fprintf(stderr, "[pmap] ================================================================\n");
}

static void c54x_pmap_record(int surface, int is_write, uint16_t addr, uint16_t val,
                             uint16_t pc, uint32_t insn, int handled)
{
    if (g_pmap_on < 0) {
        g_pmap_on = getenv("DSP54_PERIPHMAP") ? 1 : 0;
        const char *lo = getenv("DSP54_PERIPHMAP_LO"), *hi = getenv("DSP54_PERIPHMAP_HI");
        const char *pa = getenv("DSP54_PERIPHMAP_PA");
        g_pmap_lo = lo ? (uint32_t)strtoul(lo, 0, 0) : 0;
        g_pmap_hi = hi ? (uint32_t)strtoul(hi, 0, 0) : 0xFFFFFFFFu;
        if (pa && *pa) g_pmap_pa = strtol(pa, 0, 0);
        g_pmap_raw = (lo || hi || pa) ? 1 : 0;
        if (g_pmap_raw) g_pmap_on = 1;            /* raw mode implies the map */
        if (g_pmap_on) atexit(c54x_pmap_dump);
    }
    if (!g_pmap_on) return;
    if (surface == 0 && addr >= 0x100) {          /* off-map port space */
        if (!g_pmap_hi_pa_n) g_pmap_hi_pa_first = addr;
        g_pmap_hi_pa_n++;
        return;
    }
    if (surface && addr >= 0x60) return;          /* defensive (callers pre-filter) */
    PmapCell *c = surface ? &g_pmap_mmr[is_write][addr]
                          : &g_pmap_port[is_write][addr];
    if (!c->hits) { c->first_insn = insn; c->first_pc = pc; }
    c->hits++;
    if (handled) c->handled_hits++;
    c->last_val = val;
    int i;
    for (i = 0; i < c->nval && c->val[i] != val; i++) ;
    if (i == c->nval) {
        if (c->nval < PMAP_NVAL) c->val[c->nval++] = val;
        else c->val_more = 1;
    }
    for (i = 0; i < c->npc && c->pc[i] != pc; i++) ;
    if (i < c->npc) c->pc_n[i]++;
    else if (c->npc < PMAP_NPC) { c->pc[c->npc] = pc; c->pc_n[c->npc] = 1; c->npc++; }
    else c->pc_other++;
    if (g_pmap_raw && insn >= g_pmap_lo && insn <= g_pmap_hi
        && (g_pmap_pa < 0 || (long)addr == g_pmap_pa)
        && g_pmap_rawn < 6000) {
        fprintf(stderr, "[pmap] %s %s 0x%02X %s 0x%04X  pc=0x%04X insn=%u handled=%d\n",
                surface ? "MMR " : "PORT", is_write ? "WR" : "RD", addr,
                is_write ? "<-" : "->", val, pc, insn, handled);
        if (++g_pmap_rawn == 6000)
            fprintf(stderr, "[pmap] raw log capped at 6000 lines\n");
    }
}

/* Legacy C54_LOG : gated par CALYPSO_DEBUG containing "C54X" or "ALL".
 * Pour gating fin par probe, utiliser C54_DBG("PROBE_NAME", fmt, ...). */
#define C54_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("C54X")) \
        fprintf(stderr, "[c54x] " fmt "\n", ##__VA_ARGS__); } while (0)

/* ================================================================
 * Helpers
 * ================================================================ */

/* Sign-extend 40-bit accumulator */
static inline int64_t sext40(int64_t v)
{
    if (v & ((int64_t)1 << 39))
        v |= ~(((int64_t)1 << 40) - 1);
    else
        v &= ((int64_t)1 << 40) - 1;
    return v;
}

/* Saturate 40-bit to 32-bit (OVM mode) */
static inline int64_t sat32(int64_t v)
{
    if (v > 0x7FFFFFFF) return 0x7FFFFFFF;
    if (v < (int64_t)(int32_t)0x80000000) return (int64_t)(int32_t)0x80000000;
    return v;
}

/* Get ARP from ST0 */
static inline int arp(C54xState *s)
{
    return (s->st0 >> ST0_ARP_SHIFT) & 7;
}

/* Get DP from ST0 */
static inline uint16_t dp(C54xState *s)
{
    return s->st0 & ST0_DP_MASK;
}

/* Get ASM from ST1 (5-bit signed) */
static inline int asm_shift(C54xState *s)
{
    int v = s->st1 & ST1_ASM_MASK;
    if (v & 0x10) v |= ~0x1F;  /* sign extend */
    return v;
}

/* ================================================================
 * Memory access
 * ================================================================ */

/* Forward decl: used by data_write() VECDUMP at MMR_PMST. */
static uint16_t prog_read(C54xState *s, uint32_t addr);
static uint16_t prog_fetch(C54xState *s, uint16_t pc);

/* Propagated by D_BURST_D probe, consumed by A_CD-BY-BURST correlation. */
static uint16_t g_last_d_burst_d;

/* PROBE 2026-05-31 (review c web) : dernier PC/op ayant écrit chaque slot de
 * pile (zone 0x1000-0x1FFF). Nomme le PSHM qui a posé l'orphelin 0x80fd lu par
 * POPM ST0 @0x94f3 → matching-frame vs frame étranger. À RETIRER. */

/* === Generic watch-write zone helper (2026-05-15 matin) ===
 *
 * Factorisation du pattern COEFFS-WR / A_CD-WR / ... : pour chaque zone
 * mémoire surveillée, maintient per-PC counter + log throttled + summary
 * périodique. La 4e+ instrumentation devient triviale au call site.
 *
 * Cost : 512 KB de statique par zone (per_pc[0x10000] × uint64_t). Acceptable
 * pour debug. */
typedef struct {
    uint64_t per_pc[0x10000];
    uint64_t total;
    uint64_t last_log_insn;
    uint64_t last_summary_insn;
    uint16_t last_exec_pc;
} WatchWriteState;

static bool watch_write_zone_check(C54xState *s, uint16_t addr, uint16_t val,
                                   const char *name,
                                   uint16_t lo, uint16_t hi,
                                   WatchWriteState *st)
{
    if (addr < lo || addr > hi) return false;
    uint16_t exec_pc = s->last_exec_pc;
    st->per_pc[exec_pc]++;
    st->total++;
    bool should_log =
        st->total <= 500
        || exec_pc != st->last_exec_pc
        || (s->insn_count - st->last_log_insn) > 100000;
    if (should_log) {
        const char *wk_name[] = {
            "UNK", "F3", "8x", "77", "76", "PSHM",
            "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
        };
        uint8_t wk = s->writer_kind;
        const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                          ? wk_name[wk] : "??";
        fprintf(stderr,
                "[c54x] %s-WR #%llu addr=0x%04x val=0x%04x "
                "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x wk=%s "
                "AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                name, (unsigned long long)st->total, addr, val,
                exec_pc, s->pc, s->prog[s->pc], wkn,
                s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        st->last_exec_pc = exec_pc;
        st->last_log_insn = s->insn_count;
    }
    if (s->insn_count - st->last_summary_insn >= 5000000) {
        st->last_summary_insn = s->insn_count;
        /* Format `<NAME>-WR-SUMMARY` (avec -WR- infix) pour cohérence avec
         * les per-hit lines `<NAME>-WR #N` et backward-compat regex tests. */
        fprintf(stderr, "[c54x] %s-WR-SUMMARY insn=%u total=%llu",
                name, s->insn_count, (unsigned long long)st->total);
        for (int p = 0; p < 0x10000; p++) {
            if (st->per_pc[p]) {
                fprintf(stderr, " pc[0x%04x]=%llu",
                        p, (unsigned long long)st->per_pc[p]);
            }
        }
        fprintf(stderr, "\n");
    }
    return true;
}

/* === FB-det timing/content stats (2026-05-14 night) ===
 *
 * Captures sur les ~928 fires de 0x8f51 (au lieu du cap 50 actuel) :
 *   - AR4 dans/hors zone [0x2bc0..0x2bff] → addressing vs timing
 *   - delta insn depuis dernier write par cluster (compute/clear/pattern)
 *   - histogramme val[AR4] : zero / 0xfffe sentinel / other
 *
 * Verdict :
 *   ar4_in_zone < 100% → bug d'addressing (AR4 pointe hors zone)
 *   delta_clear < delta_compute systématique → timing race (clear gagne)
 *   delta_compute >> 1M → compute jamais dans la fenêtre du fire
 *   other > 0 → certains sweeps voient des données — creuser ces fires
 */
static struct {
    /* Timing trackers (mis à jour dans data_write côté 0x2bc0..0x2bff) */
    uint64_t last_compute_insn;
    uint16_t last_compute_addr;
    uint64_t last_clear_insn;
    uint16_t last_clear_addr;
    uint64_t last_pattern_insn;
    uint16_t last_pattern_addr;
    /* Stats au moment du fire 0x8f51 */
    uint64_t fb_det_total;
    uint64_t fb_det_ar4_in_zone;
    uint64_t fb_det_ar4_outside;
    uint64_t fb_det_dar4_zero;
    uint64_t fb_det_dar4_sentinel;
    uint64_t fb_det_dar4_other;
    /* Sweep tracking : un sweep = 50 fires 0x8f51 consécutifs avec AR3
     * progressant 0..0x3A3 stride+19. Wrap (ar3 < last_ar3) = nouveau sweep. */
    uint16_t last_ar3_at_fire;
    uint64_t sweep_id;
    uint64_t sweep_nonzero_count;
} g_fb_det_timing;

/* === Generic ARn write tracer with provenance (2026-05-25 v3 unified) ===
 * Tracer paramétré pour AR0..AR7. Remplace les ad-hoc ar2/ar4. Env mask
 * `CALYPSO_AR_TRACE` (hex, default 0) :
 *   =0xFF  → trace tous les AR0..AR7
 *   =0x14  → trace AR2 + AR4 seulement (bits 2 et 4 set)
 *   =0x04  → AR2 only
 *   =0     → désactivé (zéro coût)
 *
 * Hook : `case MMR_AR0..AR7` dans data_write_locked. Skip auto-modify
 * noise (Δ ∈ [-3, 3]). Classification opcode via decode + flag ZERO
 * automatique (= suspect clobber MMR via STL A,*AR-).
 *
 * Question résolue par cette sonde : qui pose ARn = mauvaise valeur ?
 *   STM-#lk    → immediate hardcoded ROM (silicon-intentional, le fix
 *                est ailleurs : étape ultérieure qui ré-set manque)
 *   MVDM-mem   → load depuis mem (slot uninit divergence QEMU vs silicon)
 *   MVMM       → copie d'un autre AR (remonter le tracer sur source)
 *   STM Smem   → load mem indirect (idem MVDM)
 *   STLM-A     → from accumulator A (vérifier d'où A vient) */
#define AR_HIST_MAX 64
typedef struct {
    uint16_t pc;
    uint16_t op_last;
    uint16_t val_last;
    uint32_t count;
} ArEntry;
static ArEntry  g_ar_hist[8][AR_HIST_MAX];
static unsigned g_ar_used[8]    = {0};
static unsigned g_ar_total[8]   = {0};
static unsigned g_ar_mask       = 0;
static int      g_ar_enabled    = -1;
static unsigned g_ar_log_cap    = 50;

static void ar_write_track(C54xState *s, unsigned idx, uint16_t new_val)
{
    if (g_ar_enabled < 0) {
        const char *e = getenv("CALYPSO_AR_TRACE");
        g_ar_mask = (e && *e) ? (unsigned)strtoul(e, NULL, 0) : 0xFFu;
        g_ar_enabled = calypso_debug_enabled("AR-TRACE") ? 1 : 0;
        if (g_ar_enabled) {
            fprintf(stderr,
                "[c54x] AR-TRACE enabled, mask=0x%02x (AR0..AR7), "
                "log_cap=%u hist_max=%u\n",
                g_ar_mask, g_ar_log_cap, AR_HIST_MAX);
        }
    }
    if (g_ar_enabled <= 0) return;
    if (idx >= 8) return;
    if (!(g_ar_mask & (1u << idx))) return;

    uint16_t old_val = s->ar[idx];
    int32_t delta = (int32_t)new_val - (int32_t)old_val;
    if (delta >= -3 && delta <= 3) return;  /* skip auto-modify noise */
    g_ar_total[idx]++;

    uint16_t op = prog_fetch(s, s->pc);
    const char *kind = "MISC";
    if ((op & 0xFF80) == 0x7700) kind = "STM-#lk";
    else if ((op & 0xFF00) == 0x8400) kind = "STLM-A";
    else if ((op & 0xFF00) == 0x8600) kind = "MVDM-mem";
    else if ((op & 0xFF00) == 0x8800) kind = "MVMM";
    else if ((op & 0xF800) == 0x8000) kind = "STL-A";

    unsigned i;
    for (i = 0; i < g_ar_used[idx]; i++) {
        if (g_ar_hist[idx][i].pc == s->pc) {
            g_ar_hist[idx][i].count++;
            g_ar_hist[idx][i].val_last = new_val;
            g_ar_hist[idx][i].op_last = op;
            break;
        }
    }
    if (i == g_ar_used[idx] && g_ar_used[idx] < AR_HIST_MAX) {
        g_ar_hist[idx][i].pc = s->pc;
        g_ar_hist[idx][i].op_last = op;
        g_ar_hist[idx][i].val_last = new_val;
        g_ar_hist[idx][i].count = 1;
        g_ar_used[idx]++;
    }
    if (g_ar_total[idx] <= g_ar_log_cap) {
        fprintf(stderr,
            "[c54x] AR%u-W #%u %s @insn=%u PC=0x%04x op=0x%04x  "
            "AR%u %04x → %04x (Δ=%+d)  A=%010llx SP=%04x\n",
            idx, g_ar_total[idx], kind, s->insn_count, s->pc, op,
            idx, old_val, new_val, delta,
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->sp);
    }
    /* Distinction sémantique critique (cf review Claude web 2026-05-25 v2) :
     * - STM-#lk    = deliberate AR update via immediate hardcoded ROM
     *                (silicon-intentional, l'AR change par design firmware)
     * - LD-#k      = idem (small immediate)
     * - STL-A / autres = side-effect d'un MMR write where AR happens to
     *                    self-alias (= AR pointing at its own MMR slot).
     *                    NOT an explicit AR update — coincidence pointer.
     * Label clairement pour ne pas confondre les 2 dans le hunt. */
    if (new_val == 0) {
        int deliberate = ((op & 0xFF80) == 0x7700);  /* STM-#lk only */
        fprintf(stderr,
            "[c54x] AR%u-W ZERO %s @insn=%u PC=0x%04x op=0x%04x AR%u←0 "
            "(kind=%s)\n",
            idx,
            deliberate ? "DELIBERATE" : "SIDE-EFFECT",
            s->insn_count, s->pc, op, idx, kind);
    }
}

/* === A accumulator provenance tracer (2026-05-25 v3) ===
 * Capture le LAST WRITER de A via chokepoint top-of-loop : compare A
 * iter-à-iter, si change → mémorise PC + op du writer. Quand un trigger
 * PC fire (default = 0x9ac0 = STL A,*AR2- clobber IMR), dump A + last
 * writer. Réponse à la question Claude web : A=0 délibéré (= mask-all
 * design firmware) ou A=0 divergence (= A devait porter mask valide) ?
 *
 * Env : CALYPSO_A_TRACE_PC=0x9ac0 (hex PC trigger, default 0xFFFF=off)
 * Zéro coût si env non set. */
static int64_t  g_a_last_value      = 0;
static uint16_t g_a_last_writer_pc  = 0;
static uint16_t g_a_last_writer_op  = 0;
static unsigned g_a_last_writer_insn = 0;
static int      g_a_trace_enabled   = -1;
static uint16_t g_a_trace_pc        = 0xFFFF;
static unsigned g_a_trace_hits      = 0;
static unsigned g_a_trace_log_cap   = 50;

static void a_track_init_lazy(void)
{
    if (g_a_trace_enabled >= 0) return;
    const char *e = getenv("CALYPSO_A_TRACE_PC");
    if (calypso_debug_enabled("A-TRACE")) {
        g_a_trace_pc = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0;
        g_a_trace_enabled = 1;
        fprintf(stderr,
            "[c54x] A-TRACE enabled, trigger PC=0x%04x log_cap=%u\n",
            g_a_trace_pc, g_a_trace_log_cap);
    } else {
        g_a_trace_enabled = 0;
    }
}

static void a_track_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op)
{
    if (g_a_trace_enabled <= 0) return;
    /* Detect A change → mémorise dernier writer */
    if (s->a != g_a_last_value) {
        g_a_last_writer_pc   = prev_pc;
        g_a_last_writer_op   = prev_op;
        g_a_last_writer_insn = s->insn_count;
        g_a_last_value       = s->a;
    }
    /* Trigger : PC about to execute matches target */
    if (s->pc == g_a_trace_pc) {
        g_a_trace_hits++;
        int a_zero = ((s->a & 0xFFFF) == 0);
        /* Log if (a) parmi les N premiers (contexte) OR (b) A_low=0 (= cas
         * suspect STL clobber zone). Évite cap log silencieux qui masque
         * les events critiques tardifs (cf cas insn=253328 IMR clobber). */
        if (g_a_trace_hits <= g_a_trace_log_cap || a_zero) {
            fprintf(stderr,
                "[c54x] A-AT-PC #%u @insn=%u PC=0x%04x  A=%010llx (low=0x%04x, %s) "
                "last_writer: PC=0x%04x op=0x%04x @insn=%u\n",
                g_a_trace_hits, s->insn_count, s->pc,
                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned)(s->a & 0xFFFF),
                a_zero ? "A_low=0 → STL clobber zone" : "A_low≠0",
                g_a_last_writer_pc, g_a_last_writer_op,
                g_a_last_writer_insn);
        }
    }
}

/* === AR6 windowed snapshot at trigger PC (2026-05-25 v4) ===
 * Capture AR6 + B + source provenance à chaque fire d'un PC trigger,
 * fenêtré sur [insn_lo, insn_hi] pour éviter explosion log (le PC 0x821a
 * fire 10M+ fois). Réponse à la question Claude web : aux fires qui
 * clobber IMR à PC=0x821a, AR6 vaut 0 (= base divergence) ou 0x16
 * (= self-alias feedback) ?
 *
 * Tracking AR6's last writer (= what set AR6 to its current value)
 * via top-of-loop comparison (même pattern que A tracer).
 *
 * Env :
 *   CALYPSO_AR6_AT_PC=0x821a    PC trigger
 *   CALYPSO_AR6_WIN_LO=3619500  insn window start
 *   CALYPSO_AR6_WIN_HI=3619810  insn window end (one outer-loop iter)
 *   CALYPSO_AR6_AT_LOG_CAP=200  max log lines (default 200)
 */
static uint16_t g_ar6_last_value     = 0;
static uint16_t g_ar6_last_writer_pc = 0;
static uint16_t g_ar6_last_writer_op = 0;
static unsigned g_ar6_last_writer_insn = 0;
static int      g_ar6_at_enabled     = -1;
static uint16_t g_ar6_at_pc          = 0xFFFF;
static unsigned g_ar6_at_win_lo      = 0;
static unsigned g_ar6_at_win_hi      = 0;
static unsigned g_ar6_at_hits        = 0;
static unsigned g_ar6_at_log_cap     = 200;

static void ar6_at_init_lazy(void)
{
    if (g_ar6_at_enabled >= 0) return;
    const char *e = getenv("CALYPSO_AR6_AT_PC");
    if (calypso_debug_enabled("AR6-AT")) {
        g_ar6_at_pc = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0;
        g_ar6_at_enabled = 1;
        const char *lo = getenv("CALYPSO_AR6_WIN_LO");
        const char *hi = getenv("CALYPSO_AR6_WIN_HI");
        const char *cap = getenv("CALYPSO_AR6_AT_LOG_CAP");
        g_ar6_at_win_lo = (lo && *lo) ? (unsigned)strtoul(lo, NULL, 0) : 0;
        g_ar6_at_win_hi = (hi && *hi) ? (unsigned)strtoul(hi, NULL, 0) : 0xFFFFFFFFu;
        g_ar6_at_log_cap = (cap && *cap) ? (unsigned)strtoul(cap, NULL, 0) : 200;
        fprintf(stderr,
            "[c54x] AR6-AT-PC enabled, trigger PC=0x%04x window=[%u..%u] cap=%u\n",
            g_ar6_at_pc, g_ar6_at_win_lo, g_ar6_at_win_hi, g_ar6_at_log_cap);
    } else {
        g_ar6_at_enabled = 0;
    }
}

static void ar6_at_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op)
{
    if (g_ar6_at_enabled <= 0) return;
    /* Track AR6 last writer */
    if (s->ar[6] != g_ar6_last_value) {
        g_ar6_last_writer_pc   = prev_pc;
        g_ar6_last_writer_op   = prev_op;
        g_ar6_last_writer_insn = s->insn_count;
        g_ar6_last_value       = s->ar[6];
    }
    /* Trigger : PC about to execute matches AND within window */
    if (s->pc == g_ar6_at_pc &&
        s->insn_count >= g_ar6_at_win_lo &&
        s->insn_count <= g_ar6_at_win_hi) {
        g_ar6_at_hits++;
        if (g_ar6_at_hits <= g_ar6_at_log_cap) {
            uint16_t ar6 = s->ar[6];
            const char *regime;
            if (ar6 == 0)              regime = "AR6=0 → addr=IMR (BUFFER BASE DIVERGENCE)";
            else if (ar6 == 0x16)      regime = "AR6=0x16 → addr=MMR_AR6 (SELF-ALIAS)";
            else if (ar6 < 0x20)       regime = "AR6 in MMR zone";
            else                        regime = "AR6 normal";
            fprintf(stderr,
                "[c54x] AR6-AT-PC #%u @insn=%u PC=0x%04x  AR6=0x%04x (%s) "
                "B=%010llx (high=0x%04x)  last_writer: PC=0x%04x op=0x%04x @insn=%u\n",
                g_ar6_at_hits, s->insn_count, s->pc,
                ar6, regime,
                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                (unsigned)((s->b >> 16) & 0xFFFF),
                g_ar6_last_writer_pc, g_ar6_last_writer_op,
                g_ar6_last_writer_insn);
        }
    }
}


/* RSBX INTM hits counter (cheap probe, candidat 1 du doc §7). */
static uint64_t g_rsbx_intm_hits = 0;
static int      g_rsbx_intm_enabled = -1;

/* DISP-ENTRY discriminateur (c web 2026-05-29) : capture du contexte de
 * préemption d'IT, pour trancher "dispatcher atteint via vecteur IT avec DP
 * foreground sale" (b) vs "DP clobbé" (a). C54x n'empile PAS ST0/DP à l'IT →
 * l'ISR hérite du DP du code interrompu. Si les entrées dispatcher KO
 * (DP≠0x124) corrèlent avec une IT récente → préemption confirmée. */
static uint64_t g_last_intr_insn  = 0;      /* insn_count de la dernière IT servie */
static int      g_last_intr_vec   = -1;     /* vecteur de la dernière IT */
static uint16_t g_last_intr_fg_pc = 0;      /* PC foreground préempté */
static uint16_t g_last_intr_fg_dp = 0;      /* DP foreground préempté */
/* dernier LDP (qui a posé DP) + PC prédécesseur (comment on arrive à 0x8341) */
static uint16_t g_last_ldp_pc  = 0;         /* PC de l'instruction qui a posé DP */
static uint16_t g_last_ldp_val = 0;         /* valeur DP posée */
static int      g_last_ldp_kind = 0;        /* 1=LDP#k(5902) 2=LDP#k9(6262) 3=LD Smem,DP(7049) */
static uint16_t g_prev_pc = 0;              /* PC de l'instruction exécutée juste avant */
static uint16_t g_last_st0w_pc  = 0;        /* PC du dernier write ST0 entier (POPM ST0/STLM) */
static uint16_t g_last_st0w_val = 0;        /* valeur ST0 restaurée */
static uint16_t g_last_st0w_op  = 0;        /* opcode de l'instruction qui écrit ST0 */
static uint16_t g_last_st0w_xpc = 0;        /* XPC au moment du write (0xf48b dépend de XPC) */
static uint16_t g_last_st0w_prev = 0;       /* PC prédécesseur du write (comment on y arrive) */

/* === ST0 push/pop ring (C-sweep 2026-05-30, gated DISP-ENTRY) ============
 * Capture PSHM ST0 (push) + POPM/STLM ST0 (write) dans un ring. Dumpé au
 * dispatcher BAD (lut != 0xff72) pour discriminer le DP périmé en 3 branches :
 *   - dernier PUSH val=0x3124 mais POP=0x3125 → CLOBBE pile entre push/pop
 *     (famille SP/circulaire)
 *   - dernier PUSH val=0x3125 → DP déjà faux au push (LDP sauté en amont)
 *   - pas de PUSH ST0 apparié au POP → désalignement SP (pop lit autre slot) */
typedef struct { uint16_t pc, op, val, sp; char kind; } St0Ev; /* kind P=push p=pop L=ldp */
#define ST0_RING_N 24
static St0Ev    g_st0_ring[ST0_RING_N];
static unsigned g_st0_ring_idx = 0;
static int      g_st0_ring_on  = -1;
static void st0_ring_rec(C54xState *s, uint16_t val, char kind)
{
    if (g_st0_ring_on < 0) g_st0_ring_on = calypso_debug_enabled("DISP-ENTRY") ? 1 : 0;
    if (!g_st0_ring_on) return;
    St0Ev *e = &g_st0_ring[g_st0_ring_idx % ST0_RING_N];
    e->pc = s->pc; e->op = prog_fetch(s, s->pc); e->val = val; e->sp = s->sp; e->kind = kind;
    g_st0_ring_idx++;
}
/* SURGICAL 2026-05-30 : slot LUT lu à l'entrée dispatcher 0x834d (capture
 * silencieuse, pour épingler le DP coupable du self-CALA 0x70c3 sans le
 * spam de DISP-TRACE qui décale le timing et masque le bug). */
static uint16_t g_disp_lut_ea  = 0;
static uint16_t g_disp_lut_val = 0;
/* SURGICAL 2026-05-30 : ring des évènements SP (push/pop) au chokepoint
 * unique de la boucle run. Sur tout changement de SP on enregistre
 * {pc, op, delta}. Dumpé par BLACKHOLE-CALA → nomme la source récurrente
 * de drain (push jamais dépoppé). Écritures array only = ~zéro coût. */
struct sp_evt { uint16_t pc; uint16_t op; int16_t delta; uint16_t sp; };
static struct sp_evt g_spring[64];
static uint32_t g_spring_idx = 0;

/* === SHADOW STACK (pairing push/pop, 2026-05-30, c-web) ===
 * Miroir logique de la pile DSP : à chaque PUSH (CALL/CALLD/PSHM/IRQ → SP-)
 * on empile {pc,op,kind} ; à chaque POP (RET/RETD/RETE/FRET/RETED/POPM → SP+)
 * on dépile et on VÉRIFIE l'appariement. Un POP sur shadow VIDE = return SANS
 * call apparié = LA source de l'over-pop (lit la pile vierge au-dessus de SP_base).
 * On nomme ce return orphelin (PC/op/SP), ce que les 15 victimes 0xc8be ne disent
 * pas. Gated CALYPSO_DEBUG=ORPHAN. kind: 'C'=call 'P'=pshm/pshd 'I'=irq 'R'=reti.
 * Array-only quand off → ~zéro coût. */
struct shadow_ent { uint16_t pc; uint16_t op; uint16_t sp; char kind; };
#define SHADOW_N 512
static struct shadow_ent g_shadow[SHADOW_N];
static int  g_shadow_depth = 0;     /* nb de mots actuellement empilés (logique) */
static int  g_shadow_on   = -1;     /* -1 = pas encore résolu le gate */
static uint64_t g_orphan_hits = 0;  /* nb de POP-orphelins détectés */
static uint64_t g_mismatch_hits = 0;/* nb de POP avec kind mismatché */

/* Tracker stores directs zone pile [0x1100..0x1140] (AU-DESSUS de SP_base) :
 * ces slots ne sont JAMAIS écrits par un push (la pile descend SOUS 0x1100),
 * donc uniquement par un ST direct. Discrimine vecteur-init LÉGIT (slot écrit
 * par le firmware) vs slot VIERGE (jamais écrit = vrai over-pop garbage). */
#define STKSLOT_LO 0x1100
#define STKSLOT_HI 0x1140
#define STKSLOT_N  (STKSLOT_HI - STKSLOT_LO + 1)
static uint16_t g_stkslot_wpc[STKSLOT_N];
static uint16_t g_stkslot_wop[STKSLOT_N];
static uint8_t  g_stkslot_written[STKSLOT_N];

static void rsbx_intm_check(C54xState *s, uint16_t op)
{
    if (g_rsbx_intm_enabled < 0) {
        const char *e = cdbg_env("RSBX-INTM");
        g_rsbx_intm_enabled = (e && *e == '1') ? 1 : 0;
        if (g_rsbx_intm_enabled) {
            fprintf(stderr, "[c54x] RSBX-INTM-TRACE enabled (op=0xF6BB)\n");
        }
    }
    if (g_rsbx_intm_enabled <= 0) return;
    if (op == 0xF6BB) {
        g_rsbx_intm_hits++;
        if (g_rsbx_intm_hits <= 20 || (g_rsbx_intm_hits % 1000) == 0) {
            fprintf(stderr,
                "[c54x] RSBX-INTM #%llu @insn=%u PC=0x%04x  ST1 INTM 0x%04x → "
                "(cleared) — IRQ enable path atteint !\n",
                (unsigned long long)g_rsbx_intm_hits, s->insn_count, s->pc,
                s->st1);
        }
    }
}

/* === AR4 write tracer with provenance (2026-05-25) ===
 * Hooke chaque write vers MMR_AR4 (= 0x14) via data_write_locked.
 * Logge (insn, PC, opcode courant, val écrite, ancien AR4) + tente une
 * classification de provenance via decode opcode :
 *   STM #lk, ARn  (0x77yx)  → immediate value depuis next prog word
 *   STLM src,ARn  (0x84yx)  → from accumulator A/B low
 *   MVDM dmad,ARn (0x86yx)  → from data memory absolute
 *   MVDD/MVMM     (autres)  → from another register
 * Flag SUSPECT si nouvelle valeur AR4 ∈ [0x2b80..0x2c00] (observed bug
 * zone) ou [0x3fb0..0x3fbf] (BSP buffer area). Env CALYPSO_AR4_TRACE=1.
 *
 * Question critique (cf Claude web) : provenance = const/mem/register ?
 * Si AR4 vient d'un mem load (LDM/MVDM), le corrupter remonte au TCB
 * en mémoire — pas l'instruction qui charge AR4, mais le TCB lui-même
 * (potentiellement uninitialized faute de re-init firmware sautée). */
/* === SP absolute-write tracer (2026-05-25 — nohack hunt) ===
 * Logge chaque write SP via STL/STM/STLM absolute, FRAME #imm, MVMM
 * register transfer — c'est-à-dire les sites où SP est *téléporté* à une
 * valeur arbitraire, par opposition aux PUSH/POP/CALL/RET qui sont des
 * inc/dec de 1. Si un site téléporte SP=0x3fbe, on tient le corrupter
 * exact du bootstub-entry observé à insn=3995013.
 *
 * Hooké aux 3 sites identifiés :
 *   L1218 : data_write_locked case MMR_SP — STL/STM/STLM to MMR_SP
 *   L3875 : F7Dx case 0xD — LD #k8u, SP
 *   L4285 : MVMM register transfer — dst==8 (SP via MMR enc 3-bit)
 *
 * Env-gated CALYPSO_SP_ABS_TRACE=1, zéro coût si OFF.
 * Limite N premiers writes verbatim + histo per-PC (cap SP_ABS_HIST_MAX). */
#define SP_ABS_HIST_MAX 128
typedef struct {
    uint16_t pc;
    uint16_t value_last;
    uint32_t count;
    uint8_t  site;   /* 0=MMR_SP, 1=LDK8, 2=MVMM */
} SpAbsEntry;
static SpAbsEntry g_sp_abs_hist[SP_ABS_HIST_MAX];
static unsigned   g_sp_abs_used     = 0;
static unsigned   g_sp_abs_total    = 0;
static int        g_sp_abs_enabled  = -1;
static unsigned   g_sp_abs_log_cap  = 50;

static void sp_abs_track(C54xState *s, uint16_t new_val, uint8_t site)
{
    if (g_sp_abs_enabled < 0) {
        const char *e = cdbg_env("SP-ABS");
        g_sp_abs_enabled = (e && *e == '1') ? 1 : 0;
        if (g_sp_abs_enabled) {
            fprintf(stderr, "[c54x] SP-ABS-TRACE enabled, log_cap=%u hist_max=%u\n",
                    g_sp_abs_log_cap, SP_ABS_HIST_MAX);
        }
    }
    if (g_sp_abs_enabled <= 0) return;
    g_sp_abs_total++;
    /* Per-PC histo */
    unsigned i;
    for (i = 0; i < g_sp_abs_used; i++) {
        if (g_sp_abs_hist[i].pc == s->pc && g_sp_abs_hist[i].site == site) {
            g_sp_abs_hist[i].count++;
            g_sp_abs_hist[i].value_last = new_val;
            break;
        }
    }
    if (i == g_sp_abs_used && g_sp_abs_used < SP_ABS_HIST_MAX) {
        g_sp_abs_hist[i].pc         = s->pc;
        g_sp_abs_hist[i].value_last = new_val;
        g_sp_abs_hist[i].count      = 1;
        g_sp_abs_hist[i].site       = site;
        g_sp_abs_used++;
    }
    /* Verbatim log first N */
    if (g_sp_abs_total <= g_sp_abs_log_cap) {
        const char *site_name = site == 0 ? "MMR_SP-W" :
                                site == 1 ? "LD-#k8-SP" : "MVMM-SP";
        int32_t delta = (int32_t)new_val - (int32_t)s->sp;
        fprintf(stderr,
            "[c54x] SP-ABS #%u %s @insn=%u PC=0x%04x SP %04x → %04x (Δ=%+d) "
            "A=%010llx AR4=%04x\n",
            g_sp_abs_total, site_name, s->insn_count, s->pc,
            s->sp, new_val, delta,
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->ar[4]);
    }
    /* Flag if SP lands in suspect zone (0x3fb0..0x3fbf = BSP read region
     * OR 0x2b80..0x2c00 = 0xfd2a A=AR4 historical) */
    if ((new_val >= 0x3fb0 && new_val <= 0x3fbf) ||
        (new_val >= 0x2b80 && new_val <= 0x2c00)) {
        fprintf(stderr,
            "[c54x] SP-ABS SUSPECT! @insn=%u PC=0x%04x SP←0x%04x (corrupter ?)\n",
            s->insn_count, s->pc, new_val);
    }
}

/* === MVPD overlay occupancy trace (2026-05-25) ===
 * Bucket writes à data[0x0080..0x27FF] en buckets de 0x80 words.
 * Dump occupancy à la fin de la boot phase (insn cap) ou périodiquement.
 * Objectif : identifier quelles sub-ranges de [0x0080..0x27FF] sont
 * chargées par MVPD au boot (= code overlay). Critique pour décider si
 * BSP buffer peut vivre dans la read-region [0x0000..0x03A3] sans
 * écraser du code en cours d'exécution.
 * Env gates :
 *   CALYPSO_MVPD_TRACE=1       active (default OFF)
 *   CALYPSO_MVPD_BOOT_LIMIT=N  cap insn pour dump (default 500000) */
#define MVPD_BUCKET_BITS 7      /* 0x80 = 128 words per bucket */
#define MVPD_BUCKET_SZ   (1u << MVPD_BUCKET_BITS)
#define MVPD_RANGE_LO    0x0080
#define MVPD_RANGE_HI    0x2800
#define MVPD_BUCKETS_N   (((MVPD_RANGE_HI - MVPD_RANGE_LO) + MVPD_BUCKET_SZ - 1) / MVPD_BUCKET_SZ)
static uint32_t g_mvpd_buckets[MVPD_BUCKETS_N];  /* 80 buckets */
static int      g_mvpd_trace_enabled = -1;
static unsigned g_mvpd_boot_limit    = 0;
static int      g_mvpd_dumped        = 0;

static void mvpd_trace_init_lazy(void)
{
    if (g_mvpd_trace_enabled >= 0) return;
    const char *e = cdbg_env("MVPD");
    g_mvpd_trace_enabled = (e && *e == '1') ? 1 : 0;
    const char *l = getenv("CALYPSO_MVPD_BOOT_LIMIT");
    g_mvpd_boot_limit = (l && *l) ? (unsigned)strtoul(l, NULL, 0) : 500000u;
    if (g_mvpd_trace_enabled) {
        fprintf(stderr,
            "[c54x] MVPD-TRACE enabled, range=[0x%04x..0x%04x] bucket_sz=%u "
            "buckets=%u boot_limit=%u\n",
            MVPD_RANGE_LO, MVPD_RANGE_HI, MVPD_BUCKET_SZ, MVPD_BUCKETS_N,
            g_mvpd_boot_limit);
    }
}

static void mvpd_trace_record(uint16_t addr)
{
    if (g_mvpd_trace_enabled <= 0) return;
    if (addr < MVPD_RANGE_LO || addr >= MVPD_RANGE_HI) return;
    unsigned b = (addr - MVPD_RANGE_LO) >> MVPD_BUCKET_BITS;
    if (b < MVPD_BUCKETS_N) g_mvpd_buckets[b]++;
}

static void mvpd_trace_dump_if_due(unsigned insn)
{
    if (g_mvpd_trace_enabled <= 0) return;
    if (g_mvpd_dumped) return;
    if (insn < g_mvpd_boot_limit) return;
    g_mvpd_dumped = 1;
    fprintf(stderr, "[c54x] MVPD-OCCUPANCY DUMP @insn=%u (boot phase end)\n", insn);
    for (unsigned b = 0; b < MVPD_BUCKETS_N; b++) {
        if (g_mvpd_buckets[b] == 0) continue;
        uint16_t lo = MVPD_RANGE_LO + b * MVPD_BUCKET_SZ;
        uint16_t hi = lo + MVPD_BUCKET_SZ - 1;
        fprintf(stderr,
            "[c54x] MVPD-BUCKET [0x%04x..0x%04x] writes=%u\n",
            lo, hi, g_mvpd_buckets[b]);
    }
    /* Verdict pour decision buffer placement : si [0x0080..0x03A3]
     * (correlator read region, bucket 0..6) a peu/zéro writes → safe
     * pour BSP DMA. Sinon il faut une autre zone. */
    unsigned bucket_0_to_6_total = 0;
    for (unsigned b = 0; b < 7 && b < MVPD_BUCKETS_N; b++)
        bucket_0_to_6_total += g_mvpd_buckets[b];
    fprintf(stderr,
        "[c54x] MVPD-VERDICT correlator_read_region [0x0080..0x03A3] "
        "writes=%u → %s\n",
        bucket_0_to_6_total,
        bucket_0_to_6_total == 0
            ? "EMPTY (safe pour BSP buffer placement ici)"
            : bucket_0_to_6_total < 100
            ? "lightly used (probably safe, audit specifics)"
            : "HEAVILY USED (code overlay, NE PAS placer BSP buffer ici)");
}

/* === Correlator trace (2026-05-25 — pour run 0x6000 dual-purpose) ===
 * Capture AR3/AR4/AR5 à l'entrée du correlator FB-det + les data reads
 * pendant son exécution. Objectif : valider empiriquement que le firmware
 * lit son input I/Q dans [0x0000..0x03A3] (assertion TODO.md:13 jamais
 * exercée avec real data — l'A/B précédent mesurait WR-SITE pré-BSP).
 * Env-gated CALYPSO_CORRELATOR_TRACE=1, zéro coût si OFF.
 *
 * Correlator range : [0x8d00..0x9000] (FB-det handler PROM0).
 *
 * 2026-05-25 night : range étendu de 0x8F80 → 0x9000. Évidence runtime
 * (d_fb_det WATCH-READ) montrait des reads à PC=0x8FAC et 0x8FB5 qui
 * étaient HORS l'ancien filtre → CORR-ENTRY=0 alors que firmware FAIT
 * des accès dans la zone FB-det. Range élargi pour capturer ces hits.
 *
 * À l'entrée from-outside : log AR0..7, SP, ST0/1.
 * Pendant exec : log les data_read addr (top N uniques, capped pour
 * éviter explosion log sur runs longs). */
#define CORR_PC_LO 0x8d00
#define CORR_PC_HI 0x9000   /* exclusif */
#define CORR_READ_HIST_MAX 128
typedef struct { uint16_t addr; uint32_t count; } CorrReadEntry;
static CorrReadEntry g_corr_read_hist[CORR_READ_HIST_MAX];
static unsigned    g_corr_read_used    = 0;
static int         g_corr_trace_enabled = -1; /* -1 uninit, 0 off, 1 on */
static unsigned    g_corr_entry_count  = 0;
static unsigned    g_corr_entry_log_cap = 100000;  /* uncap : voir le par-frame post-+3s */

/* Posés par calypso_trx.c quand l'ARM écrit d_task_md=5 (commande FB).
 * La sonde D_TASK_MD-RD timestampe les reads DSP par rapport à ce write
 * (test H1 : EA write ARM vs EA read DSP + ordre). */
uint32_t g_arm_taskmd5_insn = 0;
uint16_t g_arm_taskmd5_ea   = 0;
static uint64_t    g_corr_read_total   = 0;
static uint16_t    g_corr_last_pc      = 0xFFFF; /* track PC transitions */

static void corr_trace_init_lazy(void)
{
    if (g_corr_trace_enabled >= 0) return;
    const char *e = cdbg_env("CORRELATOR");
    g_corr_trace_enabled = (e && *e == '1') ? 1 : 0;
    if (g_corr_trace_enabled) {
        fprintf(stderr, "[c54x] CORRELATOR-TRACE enabled, range=[0x%04x..0x%04x) "
                        "hist_max=%u entry_log_cap=%u\n",
                CORR_PC_LO, CORR_PC_HI,
                CORR_READ_HIST_MAX, g_corr_entry_log_cap);
    }
}

/* CORR-ENTRY tracker : appelé au top-of-loop pour chaque insn dispatch.
 * Détecte transition PC out→in du range FB-det. Log les premières N
 * entrées avec contexte AR/SP/ST. */
static void corr_entry_track(uint16_t pc, void *s_void)
{
    if (g_corr_trace_enabled <= 0) return;
    bool was_in  = (g_corr_last_pc >= CORR_PC_LO && g_corr_last_pc < CORR_PC_HI);
    bool is_in   = (pc >= CORR_PC_LO && pc < CORR_PC_HI);
    g_corr_last_pc = pc;
    if (!was_in && is_in) {
        g_corr_entry_count++;
        if (g_corr_entry_count <= g_corr_entry_log_cap
            || (g_corr_entry_count % 100) == 0) {
            C54xState *s = (C54xState *)s_void;
            fprintf(stderr,
                "[c54x] CORR-ENTRY #%u @PC=0x%04x from=0x%04x SP=0x%04x "
                "ST0=0x%04x ST1=0x%04x AR=[%04x %04x %04x %04x %04x %04x %04x %04x] "
                "A=%010llx B=%010llx T=%04x\n",
                g_corr_entry_count, pc, g_corr_last_pc, s->sp,
                s->st0, s->st1,
                s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                s->t);
        }
    }
}

/* === FBDB/FBF3 + 0x3DC0 probes (c web reframe 2026-05-25 night2) ========
 *
 * Sondes diagnostiques POST-désassemblage fc50-fc6f, sans aucun fix.
 * Trois questions à trancher :
 *   A) B @ PC=0xfbd9 vaut-il une valeur cohérente avant `SUB #8, B, A` ?
 *      (= si B faux upstream, F2xx fix donne A faux downstream)
 *   B) A @ PC=0xfbdb juste après SUB → AR4 setup à PC=0xfbf3 → corruption ?
 *   C) Le bit 4 du flag 0x3DC0 (= testé par BITF à fc63) est-il jamais set ?
 *      Si jamais set par aucune routine DSP → BCD NTC à fc66 toujours branche →
 *      fc50-fc6f loop forever sans toucher au body.
 *
 * Env-gated : CALYPSO_FBDB_PROBE=1 active toutes les sondes.
 * Coût off : 1 compare + 1 branch par opcode (négligeable). */
static int g_fbdb_probe_enabled = -1;
static unsigned g_fbdb_probe_log_cap = 100;
static unsigned g_fbdb_probe_count_b = 0;
static unsigned g_fbdb_probe_count_a = 0;
static unsigned g_fbdb_probe_count_fbf3 = 0;
static unsigned g_addr3dc0_wr_count = 0;
static unsigned g_addr3dc0_rd_count = 0;

static void fbdb_probe_init_lazy(void)
{
    if (g_fbdb_probe_enabled >= 0) return;
    const char *e = cdbg_env("FBDB");
    g_fbdb_probe_enabled = (e && *e == '1') ? 1 : 0;
    if (g_fbdb_probe_enabled) {
        fprintf(stderr,
            "[c54x] FBDB-PROBE enabled : track B@0xfbd9 + A@0xfbdb + A@0xfbf3 "
            "+ ALL r/w to 0x3DC0 (= SARAM flag polled by fc63 BITF)\n");
    }
}

/* Hook called from c54x_run top-of-loop, before c54x_exec_one. */
static void fbdb_probe_check_pc(uint16_t pc, void *s_void)
{
    if (g_fbdb_probe_enabled < 0) fbdb_probe_init_lazy();
    if (g_fbdb_probe_enabled <= 0) return;
    C54xState *s = (C54xState *)s_void;
    if (pc == 0xfbd9 && g_fbdb_probe_count_b < g_fbdb_probe_log_cap) {
        g_fbdb_probe_count_b++;
        int64_t b = s->b & 0xFFFFFFFFFFLL;
        fprintf(stderr,
            "[c54x] FBDB-PROBE B@fbd9 #%u: B=0x%010llx (low=0x%04x sign=%d) "
            "A=0x%010llx AR2=0x%04x AR3=0x%04x AR4=0x%04x insn=%u\n",
            g_fbdb_probe_count_b,
            (unsigned long long)b, (unsigned)(b & 0xFFFF),
            (b & 0x8000000000LL) ? -1 : 1,
            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
            s->ar[2], s->ar[3], s->ar[4], s->insn_count);
    }
    else if (pc == 0xfbdb && g_fbdb_probe_count_a < g_fbdb_probe_log_cap) {
        g_fbdb_probe_count_a++;
        int64_t a = s->a & 0xFFFFFFFFFFLL;
        fprintf(stderr,
            "[c54x] FBDB-PROBE A@fbdb #%u (= after SUB #8,B,A): A=0x%010llx "
            "(low=0x%04x), TC=%d insn=%u\n",
            g_fbdb_probe_count_a, (unsigned long long)a,
            (unsigned)(a & 0xFFFF),
            !!(s->st0 & (1 << 13)), s->insn_count);  /* TC = ST0 bit 13 per SPRU131G */
    }
    else if (pc == 0xfbf3 && g_fbdb_probe_count_fbf3 < g_fbdb_probe_log_cap) {
        g_fbdb_probe_count_fbf3++;
        int64_t a = s->a & 0xFFFFFFFFFFLL;
        fprintf(stderr,
            "[c54x] FBDB-PROBE A@fbf3 #%u (= just before STLM A,AR4): "
            "A=0x%010llx (low=0x%04x) — if low=0 → AR4=0 → AR5=1=MMR_IFR corruption "
            "insn=%u\n",
            g_fbdb_probe_count_fbf3, (unsigned long long)a,
            (unsigned)(a & 0xFFFF), s->insn_count);
    }
}

/* === STUCK-STATE PC+XPC histogram (c web reframe 2026-05-25 night3) =====
 *
 * Sonde diagnostique : quand le DSP est en "stuck state" (= INTM=1 ET
 * BRINT0 pending dans IFR), on enregistre PC+XPC. Permet d'identifier
 * la VRAIE boucle de blocage XPC-qualifiée — sans présumer que c'est
 * fc50 ou autre PC particulier.
 *
 * Le PC HIST classique ne distingue pas les pages XPC (= ambiguous "fc50"
 * peut être page 0x1F mirror ou 0x28/0x38 etc.).
 *
 * Entry/exit du stuck state logué (= delimit la fenêtre).
 * Top-20 PC+XPC dump périodique (= quand stuck dure).
 *
 * Env-gated CALYPSO_STUCK_PROBE=1. Coût off : 1 bit-check + 1 branch. */
#define STUCK_HIST_SIZE 64
typedef struct {
    uint16_t pc;
    uint8_t  xpc;
    uint32_t count;
} StuckHistEntry;
static StuckHistEntry g_stuck_hist[STUCK_HIST_SIZE];
static unsigned g_stuck_hist_used = 0;
static int g_stuck_probe_enabled = -1;
static int g_stuck_active = 0;
static uint32_t g_stuck_duration = 0;
static uint64_t g_stuck_start_insn = 0;
static unsigned g_stuck_dump_count = 0;

static void stuck_probe_init_lazy(void)
{
    if (g_stuck_probe_enabled >= 0) return;
    const char *e = cdbg_env("STUCK");
    g_stuck_probe_enabled = (e && *e == '1') ? 1 : 0;
    if (g_stuck_probe_enabled) {
        fprintf(stderr,
            "[c54x] STUCK-PROBE enabled : capture PC+XPC histogramme quand "
            "INTM=1 + IFR bit5 (BRINT0) pending\n");
    }
}

static void stuck_probe_record(uint16_t pc, uint8_t xpc)
{
    /* Linear scan small hist (cap 64). Insert or increment. */
    for (unsigned i = 0; i < g_stuck_hist_used; i++) {
        if (g_stuck_hist[i].pc == pc && g_stuck_hist[i].xpc == xpc) {
            g_stuck_hist[i].count++;
            return;
        }
    }
    if (g_stuck_hist_used < STUCK_HIST_SIZE) {
        g_stuck_hist[g_stuck_hist_used].pc = pc;
        g_stuck_hist[g_stuck_hist_used].xpc = xpc;
        g_stuck_hist[g_stuck_hist_used].count = 1;
        g_stuck_hist_used++;
    }
    /* If hist full, silently drop new PCs — top hot ones already captured. */
}

static void stuck_probe_dump(uint64_t cur_insn, const char *trig)
{
    /* Bubble sort by count desc (n<=64). */
    for (unsigned k = 0; k < g_stuck_hist_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_stuck_hist_used; i++) {
            if (g_stuck_hist[i].count > g_stuck_hist[best].count) best = i;
        }
        if (best != k) {
            StuckHistEntry tmp = g_stuck_hist[k];
            g_stuck_hist[k] = g_stuck_hist[best];
            g_stuck_hist[best] = tmp;
        }
    }
    if (calypso_debug_enabled("STUCK-HIST")) fprintf(stderr,
        "[c54x] STUCK-HIST [%s] duration=%u insn since insn=%llu (now=%llu) top:\n",
        trig, g_stuck_duration,
        (unsigned long long)g_stuck_start_insn,
        (unsigned long long)cur_insn);
    unsigned n_show = g_stuck_hist_used > 20 ? 20 : g_stuck_hist_used;
    for (unsigned i = 0; i < n_show; i++) {
        fprintf(stderr,
            "[c54x]   #%2u PC=0x%04x XPC=%u  count=%u\n",
            i + 1, g_stuck_hist[i].pc, g_stuck_hist[i].xpc,
            g_stuck_hist[i].count);
    }
}

/* === FORCE-INTM-ONESHOT (c web reframe 2026-05-25 night4) ===============
 *
 * Sonde d'arbitrage : quand INTM=1 ET BRINT0 (IFR bit 5) pending,
 * forcer UNE SEULE FOIS INTM=0 pour permettre dispatch. Observer ce
 * qui se passe ensuite via les tracers existants (CORR-ENTRY, a_sync_demod
 * writes, RETE log, INTM-TRANS).
 *
 * Partitionne l'arbre :
 *   - snr/toa réels après dispatch → INTM est le SEUL blocker
 *   - garbage / rien → aval cassé aussi (≥2 bugs) OU corruption ISR state
 *
 * IMPORTANT : c'est une SONDE diagnostique, PAS un fix. Le one-shot
 * permet d'observer sans masquer un comportement régulier. Si on confirme
 * "INTM est le seul blocker", le vrai fix sera côté ISR (= pourquoi pas
 * de RETE), pas un INTM-clear systématique.
 *
 * Env-gated CALYPSO_FORCE_INTM_ONESHOT=1. */
static int g_force_intm_oneshot_enabled = -1;
static int g_force_intm_oneshot_done = 0;
static uint64_t g_force_intm_oneshot_insn = 0;
/* CALYPSO_FORCE_INTM_AT_PC=0xfc6f : restreindre le force au PC donné (=
 * point sûr = RET du compute kernel fc50 par exemple). 0xFFFF sentinel
 * = pas de restriction PC (= comportement v1 = première opportunité). */
static uint16_t g_force_intm_at_pc = 0xFFFF;

static void force_intm_oneshot_check(C54xState *s)
{
    if (g_force_intm_oneshot_enabled < 0) {
        const char *e = getenv("CALYPSO_FORCE_INTM_ONESHOT");
        g_force_intm_oneshot_enabled = (e && *e == '1') ? 1 : 0;
        /* Optional PC gate : si CALYPSO_FORCE_INTM_AT_PC=0xXXXX présent,
         * fire seulement quand PC matche. Permet de départager state-
         * corruption vs aval-cassé per c web : force à un PC sûr (= RET
         * fc6f, idle dispatcher, etc.) au lieu de mid-compute fc57. */
        const char *pc_e = getenv("CALYPSO_FORCE_INTM_AT_PC");
        if (pc_e && *pc_e) {
            unsigned long pc_val = strtoul(pc_e, NULL, 0);
            if (pc_val <= 0xFFFF) {
                g_force_intm_at_pc = (uint16_t)pc_val;
            }
        }
        if (g_force_intm_oneshot_enabled) {
            if (g_force_intm_at_pc != 0xFFFF) {
                fprintf(stderr,
                    "[c54x] FORCE-INTM-ONESHOT enabled : will clear INTM ONCE "
                    "when INTM=1 + BRINT0 pending + PC=0x%04x (= safe-PC gate, "
                    "départage state-corruption vs aval-cassé)\n",
                    g_force_intm_at_pc);
            } else {
                fprintf(stderr,
                    "[c54x] FORCE-INTM-ONESHOT enabled : will clear INTM ONCE "
                    "when INTM=1 + BRINT0 pending (= sonde aval-sain, no PC gate)\n");
            }
        }
    }
    if (g_force_intm_oneshot_enabled <= 0) return;
    if (g_force_intm_oneshot_done) return;
    int intm_set = !!(s->st1 & ST1_INTM);
    int brint0_pending = !!(s->ifr & (1 << 5));
    if (!(intm_set && brint0_pending)) return;
    /* Skip very early boot — wait for stable stuck state. */
    if (s->insn_count < 1000000) return;
    /* Optional PC gate : if set, only fire at the specified PC (= safe point). */
    if (g_force_intm_at_pc != 0xFFFF && s->pc != g_force_intm_at_pc) return;
    /* FIRE one-shot : clear INTM, log context. */
    g_force_intm_oneshot_done = 1;
    g_force_intm_oneshot_insn = s->insn_count;
    fprintf(stderr,
        "[c54x] FORCE-INTM-ONESHOT FIRED @insn=%llu PC=0x%04x XPC=%u SP=0x%04x "
        "ST1=0x%04x IMR=0x%04x IFR=0x%04x%s — clearing INTM to allow dispatch\n",
        (unsigned long long)s->insn_count, s->pc, s->xpc & 0xFF, s->sp,
        s->st1, s->imr, s->ifr,
        (g_force_intm_at_pc != 0xFFFF) ? " (safe-PC gate)" : "");
    s->st1 &= ~ST1_INTM;  /* clear INTM bit 11 */
    fprintf(stderr,
        "[c54x] FORCE-INTM-ONESHOT post-clear : ST1=0x%04x — watch next IRQ "
        "dispatch + CORR-ENTRY + a_sync_demod writes\n", s->st1);
}

/* === INT3 cycle tracer + control-flow signature (c web reframe 2026-05-25 night5)
 *
 * Sonde décisive pour départager F1 (= pourquoi ISR INT3 ne RETE pas).
 *
 * Per cycle INT3 :
 *   - START : INT3 dispatched (vec=19) → reset trace, log cycle_id + entry PC
 *   - DURING : chaque branch conditionnelle exécutée → (PC, op, target, taken)
 *   - END (good) : RETE fire → dump trace tagged GOOD + insn count
 *   - END (orphan) : nouveau INT3 dispatch avant RETE → dump previous tagged
 *                   ORPHAN-NEXT-INT3 + reason
 *
 * Diff offline good_cycle vs orphan_cycle → 1ère branche qui diverge
 * = trigger du bug. À cette branche, lire l'état testé = la vraie cause.
 *
 * Cappé 256 branches/cycle (= overflow tagué pour borne).
 * Env-gated CALYPSO_INT3_CYCLE_TRACE=1. */
#define INT3_BRANCH_TRACE_MAX 1024
typedef struct {
    uint16_t pc;        /* PC of branch insn */
    uint16_t op;        /* opcode word 0 */
    uint16_t next_pc;   /* PC after exec (= branch taken target OR fall-through) */
    uint32_t insn_offset; /* delta from cycle start (first occurrence) */
    uint32_t repeat;    /* consecutive identical (pc,op,next_pc) collapsed count */
} Int3BranchEvent;
static Int3BranchEvent g_int3_trace[INT3_BRANCH_TRACE_MAX];
static unsigned g_int3_trace_count = 0;
static int g_int3_trace_overflow = 0;
static int g_int3_cycle_active = 0;
static uint64_t g_int3_cycle_id = 0;
static uint16_t g_int3_cycle_entry_pc = 0;
static uint64_t g_int3_cycle_entry_insn = 0;
static int g_int3_trace_enabled = -1;

static void int3_trace_init_lazy(void)
{
    if (g_int3_trace_enabled >= 0) return;
    const char *e = cdbg_env("INT3-CYCLE");
    g_int3_trace_enabled = (e && *e == '1') ? 1 : 0;
    if (g_int3_trace_enabled) {
        fprintf(stderr,
            "[c54x] INT3-CYCLE-TRACE enabled : par cycle vec=19, log toutes "
            "branches conditionnelles + RETE/orphan tag. Cap=%u branches/cycle.\n",
            INT3_BRANCH_TRACE_MAX);
    }
}

/* Detect conditional branch / call / return family. Returns 1 if op
 * is in a tracked branch family, else 0. */
static int is_int3_traced_branch(uint16_t op)
{
    uint16_t hi = op & 0xFF00;
    if (hi == 0x6C00) return 1;  /* BANZ pmad,Sind */
    if (hi == 0x6E00) return 1;  /* BANZD pmad,Sind */
    if (hi == 0xF800) return 1;  /* BC pmad,cond */
    if (hi == 0xF900) return 1;  /* CC pmad,cond */
    if (hi == 0xFA00) return 1;  /* BCD pmad,cond */
    if (hi == 0xFB00) return 1;  /* CCD pmad,cond */
    if (hi == 0xFC00) {
        /* FC00 unconditional = RET ; FCxx where xx is cond = RC */
        if (op != 0xFC00) return 1;
        return 0;
    }
    if (hi == 0xFE00) {
        if (op != 0xFE00) return 1; /* RCD cond */
        return 0;
    }
    return 0;
}

/* Called from c54x_interrupt_ex when vec=19 (INT3 FRAME) dispatched. */
static void int3_cycle_start(C54xState *s, uint16_t target_pc)
{
    if (g_int3_trace_enabled < 0) int3_trace_init_lazy();
    if (g_int3_trace_enabled <= 0) return;
    /* If previous cycle still active = orphan (= didn't RETE before re-entry) */
    if (g_int3_cycle_active) {
        fprintf(stderr,
            "[c54x] INT3-CYCLE #%llu ORPHAN-NEXT-INT3 — previous cycle didn't "
            "RETE, new entry @insn=%llu PC=0x%04x. Trace below (%u branches%s) :\n",
            (unsigned long long)g_int3_cycle_id,
            (unsigned long long)s->insn_count, target_pc,
            g_int3_trace_count,
            g_int3_trace_overflow ? "+ OVERFLOW" : "");
        for (unsigned i = 0; i < g_int3_trace_count; i++) {
            Int3BranchEvent *e = &g_int3_trace[i];
            /* 2-word branches (no lk_used here for simplicity) : BC/CC/BCD/CCD
             * (0xF8-0xFB) and BANZ/BANZD (0x6C/0x6E). 1-word : RET/RC/RCD. */
            uint16_t hi = e->op & 0xFF00;
            bool two_word = (hi >= 0xF800 && hi <= 0xFB00)
                            || hi == 0x6C00 || hi == 0x6E00;
            uint16_t fallthrough = e->pc + (two_word ? 2 : 1);
            fprintf(stderr,
                "[c54x]   #%3u Δ%u PC=0x%04x op=0x%04x → next=0x%04x %s ×%u\n",
                i + 1, e->insn_offset, e->pc, e->op, e->next_pc,
                (e->next_pc == fallthrough) ? "(NOT_TAKEN)" : "(TAKEN)",
                e->repeat);
        }
    }
    g_int3_cycle_id++;
    g_int3_cycle_active = 1;
    g_int3_cycle_entry_pc = target_pc;
    g_int3_cycle_entry_insn = s->insn_count;
    g_int3_trace_count = 0;
    g_int3_trace_overflow = 0;
    if (calypso_debug_enabled("INT3-CYCLE")) fprintf(stderr,
        "[c54x] INT3-CYCLE #%llu START @insn=%llu PC→0x%04x SP=0x%04x "
        "PMST=0x%04x IFR=0x%04x\n",
        (unsigned long long)g_int3_cycle_id,
        (unsigned long long)s->insn_count, target_pc, s->sp,
        s->pmst, s->ifr);
}

/* Called from c54x_run after c54x_exec_one. exec_pc/exec_op are the
 * instruction that just executed; s->pc is the resulting PC. */
static void int3_cycle_track_branch(C54xState *s, uint16_t exec_pc,
                                    uint16_t exec_op, int consumed)
{
    if (g_int3_trace_enabled <= 0) return;
    if (!g_int3_cycle_active) return;
    if (!is_int3_traced_branch(exec_op)) return;
    /* Compute next_pc correctly across all branch-handler patterns :
     * 1. Non-delayed branch TAKEN  → handler set s->pc=target, returned consumed=0
     *    → s->pc already = target.
     * 2. Delayed branch TAKEN      → handler armed delay_slots=2 + delayed_pc,
     *    returned consumed>0; main loop hasn't run +=consumed yet
     *    → eventual target = s->delayed_pc.
     * 3. Branch FALL-THROUGH (any) → handler returned consumed>0, s->pc unchanged,
     *    delay_slots not set; main loop will += consumed → next insn
     *    → next = exec_pc + consumed. */
    uint16_t actual_next;
    if (consumed == 0) {
        actual_next = s->pc;
    } else if (s->delay_slots == 2) {
        actual_next = s->delayed_pc;
    } else {
        actual_next = (uint16_t)(exec_pc + consumed);
    }

    /* Dedup-pattern : look up to 4 slots back. Catches consecutive
     * identical (distance 1, AAA), strict alternation (distance 2,
     * ABAB), and short cycles up to length 4 (ABCDABCD). Each iteration
     * of the repeating pattern bumps the matched slot's repeat — total
     * iterations = max(repeat) across slots forming the cycle. */
    for (unsigned back = 1; back <= 4 && back <= g_int3_trace_count; back++) {
        Int3BranchEvent *cand = &g_int3_trace[g_int3_trace_count - back];
        if (cand->pc == exec_pc && cand->op == exec_op && cand->next_pc == actual_next) {
            cand->repeat++;
            return;
        }
    }
    if (g_int3_trace_count >= INT3_BRANCH_TRACE_MAX) {
        g_int3_trace_overflow = 1;
        return;
    }
    Int3BranchEvent *e = &g_int3_trace[g_int3_trace_count++];
    e->pc = exec_pc;
    e->op = exec_op;
    e->next_pc = actual_next;
    e->insn_offset = (uint32_t)(s->insn_count - g_int3_cycle_entry_insn);
    e->repeat = 1;
}

/* Called from RETE handler (L3300 area) BEFORE INTM is cleared. */
static void int3_cycle_end_good(C54xState *s, uint16_t return_addr)
{
    if (g_int3_trace_enabled <= 0) return;
    if (!g_int3_cycle_active) return;
    uint64_t duration = s->insn_count - g_int3_cycle_entry_insn;
    fprintf(stderr,
        "[c54x] INT3-CYCLE #%llu RETE-GOOD @insn=%llu duration=%llu PC→0x%04x "
        "branches=%u%s\n",
        (unsigned long long)g_int3_cycle_id,
        (unsigned long long)s->insn_count, (unsigned long long)duration,
        return_addr, g_int3_trace_count,
        g_int3_trace_overflow ? "+ OVERFLOW" : "");
    for (unsigned i = 0; i < g_int3_trace_count; i++) {
        Int3BranchEvent *e = &g_int3_trace[i];
        uint16_t hi = e->op & 0xFF00;
        bool two_word = (hi >= 0xF800 && hi <= 0xFB00)
                        || hi == 0x6C00 || hi == 0x6E00;
        uint16_t fallthrough = e->pc + (two_word ? 2 : 1);
        fprintf(stderr,
            "[c54x]   #%3u Δ%u PC=0x%04x op=0x%04x → next=0x%04x %s ×%u\n",
            i + 1, e->insn_offset, e->pc, e->op, e->next_pc,
            (e->next_pc == fallthrough) ? "(NOT_TAKEN)" : "(TAKEN)",
            e->repeat);
    }
    g_int3_cycle_active = 0;
}

/* Called from c54x_run top-of-loop. */
static void stuck_probe_check(C54xState *s)
{
    if (g_stuck_probe_enabled < 0) stuck_probe_init_lazy();
    if (g_stuck_probe_enabled <= 0) return;
    int intm_set = !!(s->st1 & ST1_INTM);
    int brint0_pending = !!(s->ifr & (1 << 5));
    int now_stuck = (intm_set && brint0_pending);
    if (now_stuck && !g_stuck_active) {
        g_stuck_active = 1;
        g_stuck_start_insn = s->insn_count;
        g_stuck_duration = 0;
        g_stuck_hist_used = 0;  /* fresh hist per stuck window */
        fprintf(stderr,
            "[c54x] STUCK-ENTER insn=%llu PC=0x%04x XPC=%u IFR=0x%04x IMR=0x%04x\n",
            (unsigned long long)s->insn_count, s->pc, s->xpc & 0xFF,
            s->ifr, s->imr);
    }
    if (now_stuck) {
        g_stuck_duration++;
        /* Sample every 100 insns to bound hist diversity */
        if ((g_stuck_duration % 100) == 0) {
            stuck_probe_record(s->pc, s->xpc & 0xFF);
        }
        /* Dump periodically while stuck */
        if ((g_stuck_duration % 5000000) == 0 && g_stuck_dump_count < 5) {
            g_stuck_dump_count++;
            stuck_probe_dump(s->insn_count, "periodic-5M");
        }
    } else if (g_stuck_active) {
        g_stuck_active = 0;
        fprintf(stderr,
            "[c54x] STUCK-EXIT insn=%llu duration=%u PC=0x%04x XPC=%u IFR=0x%04x\n",
            (unsigned long long)s->insn_count, g_stuck_duration,
            s->pc, s->xpc & 0xFF, s->ifr);
        if (g_stuck_duration >= 10000) {  /* only dump if long-ish stuck */
            stuck_probe_dump(s->insn_count, "on-exit");
        }
    }
}

/* Hook called from data_write_locked when an absolute write hits 0x3DC0. */
static void fbdb_probe_write_3dc0(uint16_t addr, uint16_t old_val,
                                  uint16_t new_val, uint16_t pc, unsigned insn)
{
    if (g_fbdb_probe_enabled <= 0) return;
    g_addr3dc0_wr_count++;
    if (g_addr3dc0_wr_count <= 50) {
        uint16_t set_mask = new_val & ~old_val;  /* bits set by this write */
        fprintf(stderr,
            "[c54x] FBDB-PROBE WR 0x%04x : 0x%04x → 0x%04x (set=0x%04x) "
            "PC=0x%04x insn=%u %s\n",
            addr, old_val, new_val, set_mask, pc, insn,
            (set_mask & 0x0010) ? "*** BIT 4 SET ***" : "");
    }
}

static void fbdb_probe_read_3dc0(uint16_t addr, uint16_t val,
                                 uint16_t pc, unsigned insn)
{
    if (g_fbdb_probe_enabled <= 0) return;
    g_addr3dc0_rd_count++;
    if (g_addr3dc0_rd_count <= 30
        || (g_addr3dc0_rd_count % 10000) == 0) {
        fprintf(stderr,
            "[c54x] FBDB-PROBE RD 0x%04x = 0x%04x (bit4=%d) PC=0x%04x insn=%u\n",
            addr, val, !!(val & 0x0010), pc, insn);
    }
}

static void corr_read_record(uint16_t addr)
{
    if (g_corr_trace_enabled <= 0) return;
    g_corr_read_total++;
    unsigned i;
    for (i = 0; i < g_corr_read_used; i++) {
        if (g_corr_read_hist[i].addr == addr) {
            g_corr_read_hist[i].count++;
            return;
        }
    }
    if (g_corr_read_used < CORR_READ_HIST_MAX) {
        g_corr_read_hist[g_corr_read_used].addr  = addr;
        g_corr_read_hist[g_corr_read_used].count = 1;
        g_corr_read_used++;
    }
}

static void corr_read_dump(const char *trig)
{
    if (g_corr_trace_enabled <= 0) return;
    fprintf(stderr, "[c54x] CORR-READ DUMP[%s] total=%llu uniq=%u\n",
            trig, (unsigned long long)g_corr_read_total, g_corr_read_used);
    /* Sort by count descending (simple selection sort, n<=128). */
    for (unsigned k = 0; k < g_corr_read_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_corr_read_used; i++) {
            if (g_corr_read_hist[i].count > g_corr_read_hist[best].count)
                best = i;
        }
        if (best != k) {
            CorrReadEntry tmp = g_corr_read_hist[k];
            g_corr_read_hist[k] = g_corr_read_hist[best];
            g_corr_read_hist[best] = tmp;
        }
        fprintf(stderr, "[c54x] CORR-READ #%u addr=0x%04x count=%u\n",
                k + 1, g_corr_read_hist[k].addr, g_corr_read_hist[k].count);
    }
}

/* === DSP throughput emission (2026-05-14 evening) ===
 *
 * Émet `[c54x] INSN-COUNT-STATS total=N delta=N elapsed_ms=N rate=N/s` toutes
 * les 1M insn. Lu en stéréo par :
 *   - test_dsp_throughput_5x (milestones, static)
 *   - test_dsp_throughput_above_threshold (observability, runtime)
 * Seuil pytest : 50M/s (marge ×2 sous les 100M/s historiques). */
#include <time.h>
static struct {
    uint64_t last_logged_insn;
    struct timespec last_logged_ts;
} g_throughput;

static inline void throughput_tick(uint64_t insn_count)
{
    if (insn_count - g_throughput.last_logged_insn < 1000000) return;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (g_throughput.last_logged_ts.tv_sec == 0 &&
        g_throughput.last_logged_ts.tv_nsec == 0) {
        g_throughput.last_logged_ts = now;
        g_throughput.last_logged_insn = insn_count;
        return;
    }
    int64_t delta_ns =
        (int64_t)(now.tv_sec - g_throughput.last_logged_ts.tv_sec) * 1000000000LL +
        (int64_t)(now.tv_nsec - g_throughput.last_logged_ts.tv_nsec);
    uint64_t delta_insn = insn_count - g_throughput.last_logged_insn;
    uint64_t rate = (delta_ns > 0)
        ? (delta_insn * 1000000000ULL / (uint64_t)delta_ns) : 0;
    if (calypso_debug_enabled("INSN-COUNT-STATS")) fprintf(stderr,
            "[c54x] INSN-COUNT-STATS total=%llu delta=%llu elapsed_ms=%lld rate=%llu/s\n",
            (unsigned long long)insn_count,
            (unsigned long long)delta_insn,
            (long long)(delta_ns / 1000000),
            (unsigned long long)rate);
    g_throughput.last_logged_ts = now;
    g_throughput.last_logged_insn = insn_count;
}

/* === Read-by-range tracking for FB-det path analysis (2026-05-14 evening) ===
 *
 * Cible : identifier la zone DARAM lue par la routine FB-det sans préjuger.
 * Compteurs cumulatifs par plage + snapshot/delta à chaque "trigger PC"
 * (sites qui écrivent d_fb_det, identifiés par grep ZERO-WR + WR-SITE).
 *
 * Plages mutuellement exclusives :
 *   RR_MMRS   [0x0000..0x005F]  registres MMR C54x
 *   RR_LOW    [0x0060..0x03A3]  zone correlator linéaire (hypothèse 05-14)
 *   RR_APIRAM [0x0800..0x27FF]  API RAM partagée ARM/DSP (hypothèse β)
 *   RR_TARGET [0x3FB0..0x3FFF]  où BSP DMA écrit par défaut
 *   RR_WRAP   [0xFC5D..0xFFED]  zone correlator wrap BK=176 (AR2/AR7)
 *   RR_OTHER  tout le reste (incluant overlay 0x80..7FF, debord 0x4000+, etc.)
 *
 * Trigger PCs : 5 sites observés écrivant d_fb_det (4 ZERO-WR rares + 0x8f51
 * en boucle 50 fois). Le delta entre 2 triggers consécutifs = reads
 * cumulés dans la fenêtre amont. Cap à 200 triggers loggés pour ne pas
 * flooder. */
enum { RR_MMRS, RR_LOW, RR_APIRAM, RR_TARGET, RR_WRAP, RR_OTHER, RR_NUM };

static struct {
    uint64_t cumulative[RR_NUM];
    uint64_t snapshot[RR_NUM];
    uint64_t trigger_count;
} g_read_stats;

static inline void read_stats_record(uint16_t addr)
{
    int r;
    if      (addr <= 0x005F)                    r = RR_MMRS;
    else if (addr <= 0x03A3)                    r = RR_LOW;
    else if (addr >= 0x0800 && addr <= 0x27FF)  r = RR_APIRAM;
    else if (addr >= 0x3FB0 && addr <= 0x3FFF)  r = RR_TARGET;
    else if (addr >= 0xFC5D && addr <= 0xFFED)  r = RR_WRAP;
    else                                        r = RR_OTHER;
    g_read_stats.cumulative[r]++;
}

static void read_stats_trigger_check(C54xState *s)
{
    /* Trigger PC réduit à 0x8f51 uniquement (FB-det compute loop, 50 hits/sweep).
     * 2026-05-14 — Run précédent : trigger list large {0x8f51, 0x778a, 0x9ac0,
     * 0x9ad0, 0x9b00, 0x821a} → 0x821a en boot mailbox poll loop (14 insns
     * entre hits) a dévoré les 200 lignes de cap avant que 0x8f51 ne fire.
     * Les autres PCs étaient init/reset (1-3 hits chacun sur tout le run).
     * Cap remonté à 5000 pour couvrir plusieurs sweeps FB-det. */
    if (s->pc != 0x8f51) return;
    g_read_stats.trigger_count++;
    if (g_read_stats.trigger_count > 5000) return;
    uint64_t delta[RR_NUM];
    for (int r = 0; r < RR_NUM; r++) {
        delta[r] = g_read_stats.cumulative[r] - g_read_stats.snapshot[r];
        g_read_stats.snapshot[r] = g_read_stats.cumulative[r];
    }
    if (calypso_debug_enabled("READ-AMONT")) fprintf(stderr,
            "[c54x] READ-AMONT #%llu PC=0x%04x insn=%u "
            "mmrs=%llu low=%llu apiram=%llu target=%llu wrap=%llu other=%llu\n",
            (unsigned long long)g_read_stats.trigger_count, s->pc, s->insn_count,
            (unsigned long long)delta[RR_MMRS],
            (unsigned long long)delta[RR_LOW],
            (unsigned long long)delta[RR_APIRAM],
            (unsigned long long)delta[RR_TARGET],
            (unsigned long long)delta[RR_WRAP],
            (unsigned long long)delta[RR_OTHER]);
}

/* === NOP-region guard + transfer ring + A-write ring (2026-05-27 Plan B) ===
 * Trip ONCE on first entry into the unmapped prog zone (= PC < 0x7000 in
 * bank 0, outside OVLY DARAM 0x80-0x27FF). At trip, dump :
 *   (a) trigger transfer (the call/branch that landed in NOP zone)
 *   (b) N last control-flow transfers (most recent → oldest)
 *   (c) N last A-writes
 * Together they name the racine without spéculation. */
#define NOP_RING_N 32

typedef struct {
    uint16_t src_pc;
    uint8_t  src_xpc;
    uint16_t op;
    uint16_t tgt_pc;
    uint8_t  tgt_xpc;
    int64_t  a_val;
    uint64_t insn;
    char     type[8];   /* "B", "BACC", "CALA", "FB", "FCALL", "FBACC", "FCALA", "RET", "FRET", "OTHER" */
} XferLog;

typedef struct {
    uint16_t pc;
    uint8_t  xpc;
    uint16_t op;
    int64_t  old_a;
    int64_t  new_a;
    uint64_t insn;
} AWriteLog;

static XferLog   g_xfer_ring[NOP_RING_N];
static unsigned  g_xfer_idx;
static AWriteLog g_awrite_ring[NOP_RING_N];
static unsigned  g_awrite_idx;
static int       g_nop_tripped;

static const char *classify_xfer_op(uint16_t op)
{
    if ((op & 0xFF80) == 0xF880) return "FB";
    if ((op & 0xFF80) == 0xF980) return "FCALL";
    if ((op & 0xFF80) == 0xFA80) return "FBD";
    if ((op & 0xFF80) == 0xFB80) return "FCALLD";
    if (op == 0xF4E2 || op == 0xF5E2) return "BACC";
    if (op == 0xF4E3 || op == 0xF5E3) return "CALA";
    if (op == 0xF4E6 || op == 0xF5E6) return "FBACC";
    if (op == 0xF4E7 || op == 0xF5E7) return "FCALA";
    if (op == 0xF6E6) return "FBACCD";
    if (op == 0xF6E7) return "FCALAD";
    if (op == 0xF4E4) return "FRET";
    if (op == 0xF4EB) return "RETE";
    if (op == 0xF6E4 || op == 0xF6E5) return "FRETD";
    if (op == 0xF073) return "B";
    if (op == 0xF273) return "BD";
    if (op == 0xF074) return "CALL";
    if (op == 0xF274) return "CALLD";
    return "OTHER";
}

static void xfer_log_push(uint16_t src_pc, uint8_t src_xpc, uint16_t op,
                          uint16_t tgt_pc, uint8_t tgt_xpc, int64_t a_val,
                          uint64_t insn)
{
    XferLog *e = &g_xfer_ring[g_xfer_idx % NOP_RING_N];
    e->src_pc = src_pc;
    e->src_xpc = src_xpc;
    e->op = op;
    e->tgt_pc = tgt_pc;
    e->tgt_xpc = tgt_xpc;
    e->a_val = a_val;
    e->insn = insn;
    const char *t = classify_xfer_op(op);
    /* strncpy without padding */
    int k = 0;
    while (k < 7 && t[k]) { e->type[k] = t[k]; k++; }
    e->type[k] = '\0';
    g_xfer_idx++;
}

static void awrite_log_push(uint16_t pc, uint8_t xpc, uint16_t op,
                            int64_t old_a, int64_t new_a, uint64_t insn)
{
    AWriteLog *e = &g_awrite_ring[g_awrite_idx % NOP_RING_N];
    e->pc = pc;
    e->xpc = xpc;
    e->op = op;
    e->old_a = old_a;
    e->new_a = new_a;
    e->insn = insn;
    g_awrite_idx++;
}

/* NOP-region predicate :
 *   xpc == 0 && pc < 0x7000 && !(OVLY && pc in [0x80, 0x2800])
 * Anything that lands here is in the unmapped prog area = NOP slide. */
static inline int pc_in_nop_region(const C54xState *s, uint16_t pc, uint8_t xpc)
{
    if (xpc != 0) return 0;                 /* banque sup : géré ailleurs */
    if (pc >= 0x7000) return 0;             /* PROM0 + PROM1 mirror = valid */
    if ((s->pmst & PMST_OVLY) && pc >= 0x80 && pc < 0x2800)
        return 0;                           /* OVLY DARAM mapping = valid */
    return 1;
}

static void nop_guard_dump(C54xState *s, uint16_t pc, uint8_t xpc)
{
    if (g_nop_tripped) return;
    g_nop_tripped = 1;
    C54_LOG("================================================");
    C54_LOG("NOP-REGION GUARD TRIPPED");
    C54_LOG("  trigger PC=0x%04x XPC=%u  prog[lin]=0x%04x  insn=%u",
            pc, xpc, s->prog[((uint32_t)xpc << 16) | pc], s->insn_count);
    C54_LOG("  state : A=%010llx B=%010llx SP=0x%04x ST1=0x%04x INTM=%d "
            "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x",
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
            s->sp, s->st1, !!(s->st1 & ST1_INTM),
            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
            s->ar[4], s->ar[5], s->ar[6], s->ar[7]);

    C54_LOG("--- last %d control-flow transfers (oldest → newest) ---", NOP_RING_N);
    unsigned start = g_xfer_idx > NOP_RING_N ? (g_xfer_idx - NOP_RING_N) : 0;
    for (unsigned i = start; i < g_xfer_idx; i++) {
        const XferLog *t = &g_xfer_ring[i % NOP_RING_N];
        C54_LOG("  [%u] %-7s src=(xpc=%u,pc=0x%04x) op=0x%04x → tgt=(xpc=%u,pc=0x%04x) "
                "A=%010llx insn=%llu",
                i, t->type, t->src_xpc, t->src_pc, t->op,
                t->tgt_xpc, t->tgt_pc,
                (unsigned long long)(t->a_val & 0xFFFFFFFFFFULL),
                (unsigned long long)t->insn);
    }

    C54_LOG("--- last %d A-writes (oldest → newest) ---", NOP_RING_N);
    unsigned astart = g_awrite_idx > NOP_RING_N ? (g_awrite_idx - NOP_RING_N) : 0;
    for (unsigned i = astart; i < g_awrite_idx; i++) {
        const AWriteLog *a = &g_awrite_ring[i % NOP_RING_N];
        int64_t do_old = a->old_a & 0xFFFFFFFFFFLL;
        int64_t do_new = a->new_a & 0xFFFFFFFFFFLL;
        C54_LOG("  [%u] PC=0x%04x xpc=%u op=0x%04x  A: %010llx → %010llx "
                "(Δ=%+lld) insn=%llu",
                i, a->pc, a->xpc, a->op,
                (unsigned long long)do_old, (unsigned long long)do_new,
                (long long)(do_new - do_old),
                (unsigned long long)a->insn);
    }
    C54_LOG("================================================");
}

static uint16_t data_read_locked(C54xState *s, uint16_t addr);

/* FBWATCH (2026-05-30 soir) : sonde FB-dispatch gatée par une env DÉDIÉE
 * (CALYPSO_FBWATCH=1), résolue UNE fois en static int → check int cheap, PAS
 * via calypso_debug_enabled (master-gate reste 0, 127 gates court-circuités →
 * QEMU temps-réel → mobile vivant). Déclaré ici (avant data_read_locked qui
 * l'utilise). Filme : page-read / dispatch 0x833b / 0x9ac0 / d_fb_det / canary. */
static int      g_fbwatch_on = -1;

static uint16_t data_read(C54xState *s, uint16_t addr)
{
    /* DSP54_MBOXLOG=1: log loader1's reads of the UPLOADHEADER/mailbox cells (0x87B-0x881)
     * so we can see what loader1 actually GETS vs what the MCU wrote. Shows both stores
     * (data[] and api_ram) to expose any coherence split. Cap 120. */
    {
        static int mbr = -1; static unsigned mrn = 0;
        if (mbr < 0) mbr = getenv("DSP54_MBOXLOG") ? 1 : 0;
        /* CRC length/pointer probe: at 0x802b (per-word CRC entry, `ld *ar3+`) dump ar2 (loop
         * count = words this CRC pass covers) + ar3 (src ptr). Tells whether loader1 CRCs the
         * 512-word buffer or over-runs. Cap 12. */
        if (mbr && s->pc == 0x802b && s->ar[3] >= 0x8F0 && s->ar[3] < 0xD00) {  /* only the buffer-region (block) CRC */
            static unsigned crn = 0;
            if (crn < 16) { fprintf(stderr, "[c54x] CRCPROBE #%u ar2(len)=0x%04x ar3(src)=0x%04x insn=%u\n",
                                    crn, s->ar[2], s->ar[3], s->insn_count); crn++; }
        }
        if (mbr && ((addr >= 0x87B && addr <= 0x87E) || addr == 0x881) && mrn < 4000) {  /* skip mbox spin (7F/80) */
            uint16_t ar = (s->api_ram && (addr - 0x800u) < C54X_API_SIZE) ? s->api_ram[addr - 0x800u] : 0xFFFF;
            fprintf(stderr, "[c54x] MBOXRD #%u data[0x%04x] -> data=%04x api_ram=%04x  PC=0x%04x insn=%u\n",
                    mrn, addr, s->data[addr], ar, s->pc, s->insn_count);
            mrn++;
        }
    }
    /* Correlator read tracer (env-gated CALYPSO_CORRELATOR_TRACE=1).
     * Record addr seulement quand PC ∈ [CORR_PC_LO..CORR_PC_HI) (FB-det range).
     * Range étendu 2026-05-25 night à 0x8d00..0x9000 (cf comment block au L639).
     * Lazy-init ici plutôt qu'au top-of-loop pour rester centralisé.
     * Coût quand OFF : 1 compare + 1 branch (g_corr_trace_enabled). */
    if (g_corr_trace_enabled > 0 && s->pc >= CORR_PC_LO && s->pc < CORR_PC_HI) {
        corr_read_record(addr);
    }
    /* IQ-READ tracer (2026-05-30) : qui lit le buffer DMA BSP [0x2a00..0x2b27] ?
     * Confirme que le corrélateur FB consomme bien la vraie I/Q écrite par le
     * BSP, et à quel PC (= le vrai site corrélateur). Cap 60, ~zéro coût hors zone. */
    if (addr >= 0x2a00 && addr < 0x2b28 && s->data[addr] != 0) {
        static unsigned iqr = 0, iqseen = 0;
        iqseen++;
        /* boot (first 60) + DÉTECTION : tire aussi 1/8000 après insn>50M pour
         * voir ce que le corrélateur lit VRAIMENT à l'instant FB-det (insn~71M),
         * pas seulement le buffer stale du boot (2026-06-02). */
        if (iqr < 60 || (s->insn_count > 50000000u && (iqseen % 8000) == 0)) {
            uint16_t val = s->data[addr];
            /* A/B (accumulateurs corr complexe, sign-ext 40b) + valeurs aux
             * AUTRES pointeurs (candidats réf cos/sin) PENDANT la lecture I/Q. */
            int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
            fprintf(stderr, "[c54x] IQ-READ #%u addr=0x%04x val=0x%04x PC=0x%04x A=%lld B=%lld "
                    "T=%04x | AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                    iqr, addr, val, s->pc, (long long)a, (long long)b, s->t,
                    s->ar[3], s->data[s->ar[3]], s->ar[4], s->data[s->ar[4]],
                    s->ar[5], s->data[s->ar[5]], s->insn_count);
            iqr++;
        }
        /* SPAN write-vs-read (2026-06-02, spec CC-web) : à l'instant détection,
         * dumpe UNE fois le span contigu data[0x2a00..0x2a1f] (ce que le
         * corrélateur PEUT lire) ET bsp_buf[0..31] (ce que le BSP a écrit).
         *   data span = waveform & bsp_buf = waveform  -> buffer OK -> bug = AR3/corrélateur (lit que [0]).
         *   data span = [0] puis DC, bsp_buf = waveform -> livraison PORTR ne copie que [0] (stride/len).
         *   bsp_buf = DC                                -> conversion cs16 BSP tronque. */
        static int span_done = 0;
        if (!span_done && s->insn_count > 60000000u) {
            span_done = 1;
            fprintf(stderr, "[c54x] SPAN-READ  data[0x2a00..0x2a1f] insn=%u:", s->insn_count);
            for (int _i = 0; _i < 32; _i++) fprintf(stderr, " %04x", s->data[0x2a00 + _i]);
            fprintf(stderr, "\n[c54x] SPAN-WRITE bsp_buf[0..31] (bsp_len=%d):", s->bsp_len);
            for (int _i = 0; _i < 32 && _i < s->bsp_len; _i++) fprintf(stderr, " %04x", s->bsp_buf[_i]);
            fprintf(stderr, "\n");
        }
    }
    /* MTTCG : protège DARAM access (DSP + ARM-OVLY peuvent racer).
     * Sans MTTCG : mutex non contesté (overhead minimal). */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    uint16_t v = data_read_locked(s, addr);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    if (addr < 0x60)
        c54x_pmap_record(1, 0, addr, v, s->pc, s->insn_count, g_pmap_mmr_handled);
    return v;
}

static uint16_t data_read_locked(C54xState *s, uint16_t addr)
{
    g_pmap_mmr_handled = 0;   /* PERIPHMAP structural flag: handler cases set 1 */
    read_stats_record(addr);
    /* PROBE 2026-05-31 frame-IT : valeur FIGÉE des flags polled + qui polle.
     * La valeur lue (jamais changée) = ce que le BSP doit produire/toggler. */
    if (addr == 0x006e || addr == 0x585f || addr == 0x8a44) {
        static uint32_t fr_n = 0;
        if (fr_n < 24) {
            fprintf(stderr, "[c54x] FLAGRD data[0x%04x]=0x%04x PC=0x%04x A=0x%04x "
                    "TC=%d insn=%u\n", addr, s->data[addr], s->pc,
                    (uint16_t)(s->a & 0xFFFF), !!(s->st0 & ST0_TC), s->insn_count);
            fr_n++;
        }
        /* one-shot : dump overlay de la loop 0x010b (hors dump ROM) + état IT
         * (IFR/IMR/INTM) pendant le spin → tranche source-dead vs IMR-masqué
         * vs INTM-collé (review c web). */
        static int dumped_010b = 0;
        if (addr == 0x006e && s->pc == 0x010b && !dumped_010b) {
            dumped_010b = 1;
            fprintf(stderr, "[c54x] SPIN-IT @0x010b IFR=0x%04x IMR=0x%04x INTM=%d "
                    "(INT3 bit3 IFR=%d IMR=%d ; BRINT0 bit5 IFR=%d IMR=%d) insn=%u\n",
                    s->ifr, s->imr, !!(s->st1 & ST1_INTM),
                    !!(s->ifr&(1<<3)), !!(s->imr&(1<<3)),
                    !!(s->ifr&(1<<5)), !!(s->imr&(1<<5)), s->insn_count);
            fprintf(stderr, "[c54x] OVERLAY-DUMP prog[0x0100..0x0118] (loop poll 0x006e):\n");
            for (uint16_t a = 0x0100; a <= 0x0118; a++)
                fprintf(stderr, "[c54x]   prog[0x%04x]=0x%04x\n", a, prog_fetch(s, a));
        }
    }
    /* D_TASK_MD-RD probe : trace DSP reads of d_task_md (write page 0
     * @ data[0x0804], write page 1 @ data[0x0818]). The DSP dispatcher
     * reads task_md then branches to FB / SB / ALLC / etc. routines.
     * If only one PC reads it, that's the single dispatcher. Capped 30. */
    if ((addr == 0x0804 || addr == 0x0818) && calypso_debug_enabled("D_TASK_MD-RD")) {
        /* UNCAP (gated par token) + EA/page/val + Δ depuis le write ARM=5.
         * Si après le write (Δ>0) la valeur lue ≠ 5 → le 5 n'atteint pas cette
         * EA : compare ARM5_EA (EA écrite par l'ARM) vs EA (EA lue par le DSP).
         *  - mêmes EA, val≠5 → timing/ordre (read avant write dans la frame)
         *  - EA ≠ ARM5_EA (stride page) → parité de flip w_page/r_page. */
        fprintf(stderr,
                "[c54x] D_TASK_MD-RD EA=0x%04x page=%d val=0x%04x "
                "ARM5_EA=0x%04x dArm5=%lld PC=0x%04x insn=%u\n",
                addr, (addr == 0x0804) ? 0 : 1, s->data[addr],
                g_arm_taskmd5_ea,
                (long long)((int64_t)s->insn_count - (int64_t)g_arm_taskmd5_insn),
                s->pc, s->insn_count);
    }
    /* WATCH-RD-ADDR (générique, gated CALYPSO_DEBUG=WATCH-RD + env
     * CALYPSO_WATCH_RD_ADDR=0xNNNN) : log toute LECTURE d'une adresse data
     * arbitraire (PC + valeur lue + insn) — voir QUI lit une cellule
     * pointeur-dispatcher (ex. 0x3af7) et AVEC QUELLE valeur (0 avant
     * peuplement vs valeur valide après). Symétrique du WATCH-WR. */
    {
        static int watch_rd_addr = -1;
        if (watch_rd_addr < 0) {
            const char *e = getenv("CALYPSO_WATCH_RD_ADDR");
            watch_rd_addr = (e && *e) ? (int)strtol(e, NULL, 0) : 0;
        }
        if (watch_rd_addr && addr == (uint16_t)watch_rd_addr) {
            C54_DBG("WATCH-RD",
                "WATCH-RD data[0x%04x] = 0x%04x PC=0x%04x DP=0x%03x insn=%u",
                addr, s->data[addr], s->pc, (s->st0 & 0x1FF),
                (unsigned)s->insn_count);
        }
    }
    /* DISP-POLL (CALYPSO_DEBUG=DISP-POLL) : le busy-loop dispatcher (d1xx↔da0d)
     * polle la zone flag DARAM[0x60-0x70]. On veut voir EN STEADY-STATE (post
     * +1.9s) CE qu'il lit (quel slot) et si ce flag devient jamais non-zéro
     * (= qqn le pose). Throttlé : 1ère/40000 par slot pour éviter l'explosion
     * du busy-loop, + TOUTE valeur non-zéro (l'évènement qui compte). */
    if (addr >= 0x0060 && addr <= 0x0070 && calypso_debug_enabled("DISP-POLL")) {
        static uint64_t poll_n = 0;
        uint16_t v = s->data[addr];
        if (v != 0 || (poll_n++ % 40000) == 0)
            fprintf(stderr,
                "[c54x] DISP-POLL-RD data[0x%04x]=0x%04x PC=0x%04x INTM=%d insn=%u%s\n",
                addr, v, s->pc, !!(s->st1 & ST1_INTM), s->insn_count,
                v ? "  <-- NON-ZERO (flag posé !)" : "");
    }
    /* FBDB-PROBE read 0x3DC0 (= SARAM flag polled by fc63 BITF).
     * Env CALYPSO_FBDB_PROBE=1. Logs first 30 reads + each 10000th. */
    if (addr == 0x3DC0 && g_fbdb_probe_enabled > 0) {
        fbdb_probe_read_3dc0(addr, s->data[addr], s->pc, s->insn_count);
    }
    /* D_BURST_D_W probe : DSP lit db_w->d_burst_d ?
     * 0x0801 (W_PAGE_0 + offset 1), 0x0815 (W_PAGE_1 + offset 1).
     * Si DSP read voit 0,1,2,3 séquentiel → ARM écrit correctement db_w.
     * Si DSP read voit 0 toujours → ARM ne configure pas burst_id.
     * Si DSP ne lit jamais → DSP ne consulte pas db_w pour le burst sequence. */
    if (addr == 0x0801 || addr == 0x0815) {
        static uint64_t dbw_total[2];
        static uint64_t dbw_per_val[2][16];
        static uint64_t dbw_last_log[2];
        static uint16_t dbw_last_val[2];
        int page = (addr == 0x0815) ? 1 : 0;
        uint16_t cur_val = s->data[addr] & 0xF;
        dbw_total[page]++;
        if (cur_val < 16) dbw_per_val[page][cur_val]++;
        bool changed = (cur_val != dbw_last_val[page]);
        dbw_last_val[page] = cur_val;
        if (dbw_total[page] <= 100 || changed
            || (s->insn_count - dbw_last_log[page]) > 1000000) {
            fprintf(stderr,
                    "[c54x] D_BURST_D_W-RD page=%d #%llu addr=0x%04x "
                    "val=0x%04x exec_pc=0x%04x insn=%u\n",
                    page, (unsigned long long)dbw_total[page], addr,
                    s->data[addr], s->last_exec_pc, s->insn_count);
            dbw_last_log[page] = s->insn_count;
        }
        /* Summary toutes les 50000 reads : histogramme valeurs lues */
        if ((dbw_total[page] % 50000) == 0) {
            if (calypso_debug_enabled("D_BURST_D_W-SUMMARY")) fprintf(stderr,
                    "[c54x] D_BURST_D_W-SUMMARY page=%d total=%llu "
                    "val[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                    page, (unsigned long long)dbw_total[page],
                    (unsigned long long)dbw_per_val[page][0],
                    (unsigned long long)dbw_per_val[page][1],
                    (unsigned long long)dbw_per_val[page][2],
                    (unsigned long long)dbw_per_val[page][3],
                    (unsigned long long)(dbw_total[page]
                        - dbw_per_val[page][0] - dbw_per_val[page][1]
                        - dbw_per_val[page][2] - dbw_per_val[page][3]));
        }
    }
    /* PC-histogram pour identifier la routine PM. Deux ranges :
     *   [0x3fb0..0x3fbf] = buffer BSP (samples I/Q)
     *   [0x3dcf..0x3dd5] = buffer scratch dominant (78k+52k reads observés)
     * Compte par PC, dump top-10 toutes les 50k reads dans chaque range.
     * Plus compteur d'entrée par PC dominant pour distinguer
     * "PM cassée" vs "PM jamais appelée" (vu IRQ rate 1.5 Hz). */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static uint32_t pc_hist_3fb[65536];
        static uint32_t total_3fb;
        pc_hist_3fb[s->pc]++;
        total_3fb++;
        if ((total_3fb % 50000) == 0 && calypso_debug_enabled("PC-HIST-3FB")) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3fb[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            fprintf(stderr, "[c54x] PC-HIST-3FB total=%u :", total_3fb);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    if (addr >= 0x3dcf && addr <= 0x3dd5) {
        static uint32_t pc_hist_3dd[65536];
        static uint32_t total_3dd;
        pc_hist_3dd[s->pc]++;
        total_3dd++;
        if ((total_3dd % 50000) == 0 && calypso_debug_enabled("PC-HIST-3DD")) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3dd[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            fprintf(stderr, "[c54x] PC-HIST-3DD total=%u :", total_3dd);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    /* === CANARY-READ probe (2026-05-28 method 3) ===
     * Quand CALYPSO_BSP_INJECT_CANARY=1, le BSP overwrite tous les samples
     * avec 0xCAFE. Si le DSP lit 0xCAFE depuis une addr, c'est que cette
     * addr est dans son chemin de lecture des samples = la vraie addr cible
     * pour CALYPSO_BSP_DARAM_ADDR. Trace cap 100 pour eviter le flood. */
    if (addr < 0x4000) {
        uint16_t v = s->data[addr];
        if (v == 0xCAFE) {
            static unsigned canary_log;
            const unsigned LIMIT = 100;
            if (canary_log < LIMIT) {
                fprintf(stderr,
                        "[c54x] CANARY-READ #%u addr=0x%04x PC=0x%04x "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        canary_log, addr, s->pc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                        s->insn_count);
                canary_log++;
                if (canary_log == LIMIT)
                    fprintf(stderr, "[c54x] CANARY-READ capped at %u\n", LIMIT);
            }
        }
    }

    /* Watch the mailbox slots that the firmware polls at PROM0 0xb41a
     * (LDU *(0x0ffe), A then BACC A) and 0xb41c (CMPM *(0x0fff), 4).
     * If these stay zero / 0x10 forever, ARM never wrote them. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x0ffc || addr == 0x0ffd) {
        static unsigned watch_count;
        watch_count++;
        if (watch_count <= 60 || (watch_count % 10000) == 0) {
            uint16_t vd = s->data[addr];
            uint16_t va = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0xDEAD;
            fprintf(stderr,
                    "[c54x] WATCH-READ #%u data[0x%04x] data=0x%04x api_ram=0x%04x api_set=%d PC=0x%04x insn=%u\n",
                    watch_count, addr, vd, va, s->api_ram ? 1 : 0, s->pc, s->insn_count);
        }
    }
    /* Wait-loop diagnostic: 0x3dd0 was found to absorb ~99.5 % of DARAM
     * reads after the first ~500k reads — the DSP is stuck polling it.
     * Log the first PCs and then sample once per million reads so we can
     * trace the loop without flooding the log. */
    if (addr == 0x3dd0) {
        static unsigned wait_log;
        static unsigned wait_seen;
        wait_seen++;
        if (wait_log < 20 || (wait_seen % 1000000) == 0) {
            wait_log++;
            fprintf(stderr,
                    "[c54x] WAIT-3DD0 #%u data[0x3dd0]=0x%04x PC=0x%04x AR2=%04x AR3=%04x insn=%u\n",
                    wait_seen, s->data[0x3dd0], s->pc,
                    s->ar[2], s->ar[3], s->insn_count);
        }
    }
    /* d_fb_det watch — REAL DSP word address is 0x08F8.
     * Mapping: ARM 0xFFD001F0 (BASE_API_NDB 0xFFD001A8 + 36 words × 2)
     *        = DSP word 0x0800 + 0x1F0/2 = 0x08F8.
     * Earlier 0x01F0 was the ARM byte-offset, NOT a DSP word address —
     * watching it logged unrelated DARAM 0x01F0 (junk). Now we trace
     * the real slot the firmware polls. */
    if (addr == 0x08F8) {
        static unsigned fb_read;
        if (fb_read++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-READ d_fb_det[0x08F8]=0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x08F8], s->pc, s->insn_count);
        }
    }
    /* === DIAG-FORCE-DARAM62 ===
     * Pinned diag (env-gated, default OFF) : when set, override the read
     * of daram[0x62] inside the dispatcher loop (PC ∈ 0xCC62..0xCC6F) to
     * return 1 instead of the actual stored value. Goal: force the
     * dispatcher's "branch if flag != 0" to fire and observe whether the
     * DSP escapes the loop and jumps to api[0x1f0c]=0x770c (the dispatch
     * target). Three outcomes (binary diagnostic) :
     *   - PC leaves cc62..cc6f → 0x770c and new code paths run :
     *     loop hypothesis correct, flag is the gate, INT3 ISR is the
     *     missing writer (next step: trace writes to confirm).
     *   - PC reaches 0x770c then returns to cc62 immediately :
     *     flag is set but handler bails because something else missing
     *     (a_cd[] init, NDB cell, ...).
     *   - No change : branch / compare is more subtle than read.
     * This is a force-test, not a fix — remove or env-leave-off after. */
    /* === DSP idle dispatcher trace (PC ∈ 0xCC62..0xCC6F) ===
     * The DSP gets stuck in this PROM0 loop polling task slots. Dump the
     * exact (PC, addr, value, AR2..AR5) for the first N reads so we can
     * see WHICH memory location the dispatcher inspects to decide whether
     * to branch out (task_md ? db_r ? api_ram ? other ?). Capped to keep
     * log size manageable.
     *
     * Captures all reads (DARAM + API RAM + MMR) so we don't miss the
     * critical poll address. */
    /* === BOOT-POLL-RD probe (2026-05-28) ===
     * Trace data reads in the boot polling loop (DARAM 0x00ed..0x00ff =
     * OVLY mirror of PROM0[0x70ed..0x70ff]). Per CLAUDE.md DSP_ROM_MAP :
     * "0x7026-0x71FF = Boot polling loop (writes API RAM tables)".
     * Goal : identifier QUELLE adresse le firmware polle pour décider
     * de sortir → quel signal devrait être émis par notre émulateur
     * peripherals pour débloquer légitimement (au lieu du drift AR1 bug).
     * Cap 300. */
    if (s->pc >= 0x00ed && s->pc <= 0x010f) {
        static unsigned bpr_log;
        const unsigned LIMIT = 300;
        if (bpr_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/oth";
            }
            fprintf(stderr,
                    "[c54x] BOOT-POLL-RD #%u PC=0x%04x op=0x%04x [%s 0x%04x]=0x%04x "
                    "AR1=%04x AR2=%04x AR3=%04x SP=%04x insn=%u\n",
                    bpr_log, s->pc, s->prog[s->pc], region, addr, v,
                    s->ar[1], s->ar[2], s->ar[3], s->sp, s->insn_count);
            bpr_log++;
            if (bpr_log == LIMIT) {
                fprintf(stderr, "[c54x] BOOT-POLL-RD log capped at %u\n", LIMIT);
            }
        }
    }

    /* === CORR-RD probe (2026-05-28) ===
     * Trace data reads in the FB-det correlator inner body
     * (PROM0[0x9aba..0x9abf] = RPTBD body of publish routine at 0x9aaf+).
     * Goal : identifier QUELLE zone DARAM le correlateur lit. Si addr ∈
     * [0x3fb0..0x3fbf] → bonne zone (BSP RX), valeur = sample. Si addr
     * ailleurs (low DARAM, MMR, ...) → bug d'addressing : correlateur
     * regarde un buffer vide → A reste à 0 → publish 0 → no lock.
     * Cap 200. */
    if (s->pc >= 0x9aba && s->pc <= 0x9abf) {
        static unsigned cr_log;
        const unsigned LIMIT = 200;
        if (cr_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/oth";
            }
            fprintf(stderr,
                    "[c54x] CORR-RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x A_lo=%04x B_lo=%04x insn=%u\n",
                    cr_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    (uint16_t)(s->a & 0xFFFF),
                    (uint16_t)(s->b & 0xFFFF),
                    s->insn_count);
            cr_log++;
            if (cr_log == LIMIT) {
                fprintf(stderr, "[c54x] CORR-RD log capped at %u\n", LIMIT);
            }
        }
    }
    if (s->pc >= 0xCC62 && s->pc <= 0xCC6F) {
        static unsigned idle_rd_log;
        const unsigned LIMIT = 200;
        if (idle_rd_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/other";
            }
            fprintf(stderr,
                    "[c54x] IDLE-DISP RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    idle_rd_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            idle_rd_log++;
            if (idle_rd_log == LIMIT) {
                fprintf(stderr,
                        "[c54x] IDLE-DISP RD log capped at %u — pattern should be visible above\n",
                        LIMIT);
            }
        }
    }

    /* === UPPER-DARAM RD HIST (2026-05-28) ===
     * Histogramme des reads addr ∈ [0x4000..0xFFFF] (zone haute DARAM).
     * Complementaire au DARAM RD HIST existant (low DARAM). Goal :
     * identifier sur quelles addresses upper le DSP polle ses samples.
     * Permet de trouver la vraie zone DARAM cible pour
     * CALYPSO_BSP_DARAM_ADDR sans brute-force. Dump top-16 tous les
     * 100k reads. Skip les 1M premieres insn pour eviter le bruit boot. */
    if (addr >= 0x4000 && s->insn_count > 1000000) {
        static unsigned uhist[0xC000]; /* 0x4000..0xFFFF = 48K words */
        static unsigned ureads;
        uhist[addr - 0x4000]++;
        ureads++;
        if ((ureads % 100000) == 0 && calypso_debug_enabled("UPPER-DARAM")) {
            unsigned best[16] = {0}; uint16_t baddr[16] = {0};
            for (unsigned a = 0; a < 0xC000; a++) {
                unsigned c = uhist[a];
                if (c <= best[15]) continue;
                int p = 15;
                while (p > 0 && best[p-1] < c) {
                    best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                }
                best[p] = c; baddr[p] = (uint16_t)(0x4000 + a);
            }
            fprintf(stderr, "[c54x] UPPER-DARAM RD HIST (reads=%u): ", ureads);
            for (int i = 0; i < 16 && best[i]; i++)
                fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
            fprintf(stderr, "\n");
        }
    }
    /* === DARAM discovery histogram ===
     * Track ALL data reads from DARAM (addr < 0x4000) regardless of PC.
     * The FB handler runs from both PROM0 (0xBD47) and DARAM overlay,
     * so filtering by PC misses critical reads. */
    if (addr < 0x4000 && addr >= 0x20) {  /* skip MMRs 0x00-0x1F */
        static unsigned hist[0x4000]; /* 16 KW DARAM */
        static unsigned reads;
        if (addr < 0x4000) {
            hist[addr]++;
            reads++;
            if ((reads % 50000) == 0 && calypso_debug_enabled("DARAM")) {
                /* find top-16 */
                unsigned best[16] = {0}; uint16_t baddr[16] = {0};
                for (uint16_t a = 0; a < 0x4000; a++) {
                    unsigned c = hist[a];
                    if (c <= best[15]) continue;
                    int p = 15;
                    while (p > 0 && best[p-1] < c) {
                        best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                    }
                    best[p] = c; baddr[p] = a;
                }
                fprintf(stderr, "[c54x] DARAM RD HIST (FB-det, reads=%u): ", reads);
                for (int i = 0; i < 16 && best[i]; i++)
                    fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
                fprintf(stderr, "\n");
            }
        }
    }
    /* === BSP discovery: trace data reads in FB-det handler ===
     * Wide range over the PROM0 user-code area: handler PCs observed in
     * timeout traces cluster around 0x7e92..0x7eb8 (the FB-det inner
     * loop), so we widen the catch zone to 0x7000..0x7fff. */
    /* FB-det / dispatcher subroutine trace.
     * The 0x7e80..0x7eb8 wrapper CALLS into 0x81a5/0x81c8 with AR5=0x0e4c
     * (the FB sample buffer). Cover both ranges to catch both wrapper
     * polls and inner correlator reads. Skip the boot init phase. */
    if ((s->pc >= 0x7e80 && s->pc <= 0x7ec0) ||
        (s->pc >= 0x81a0 && s->pc <= 0x82ff)) {
        static int fbdet_rd_log = 0;
        if (s->insn_count > 50000000 && fbdet_rd_log < 2000) {
            uint16_t v;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
            else
                v = s->data[addr];
            C54_LOG("FBDET RD [0x%04x]=0x%04x PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                    addr, v, s->pc, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            fbdet_rd_log++;
        }
    }
    /* Log AR0..AR7 when entering FB-det subroutines to understand
     * what each AR points at (sample buffer? coeffs? status?). */
    if ((s->pc == 0x81a5 || s->pc == 0x81c8) && s->insn_count > 50000000) {
        static int ar_log = 0;
        if (ar_log < 10) {
            C54_LOG("FB-CALL PC=0x%04x AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x BK=%04x",
                    s->pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp, s->bk);
            ar_log++;
        }
    }
    /* d_spcx_rif (NDB word 2 = api 0xD6 = DSP data 0x08D6) */
    if (addr == 0x08D6) {
        static int spcx_rd = 0;
        if (spcx_rd < 32) {
            C54_LOG("d_spcx_rif RD = 0x%04x PC=0x%04x insn=%u",
                    s->api_ram ? s->api_ram[0xD6] : s->data[addr],
                    s->pc, s->insn_count);
            spcx_rd++;
        }
    }
    /* Log reads from API RAM at 0x08D4 (d_dsp_page) */
    if (addr == 0x08D4) {
        static int dsp_page_log = 0;
        if (dsp_page_log < 50) {
            C54_LOG("d_dsp_page RD = 0x%04x PC=0x%04x insn=%u SP=0x%04x",
                    s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                    s->pc, s->insn_count, s->sp);
            dsp_page_log++;
        }
        /* FBWATCH PRODUCTEUR : le DSP re-lit-il d_dsp_page PAR-FRAME ? (env one-shot) */
        if (g_fbwatch_on < 0) g_fbwatch_on = getenv("CALYPSO_FBWATCH") ? 1 : 0;
        if (g_fbwatch_on) {
            static unsigned wpg = 0;
            if (wpg++ < 60)
                fprintf(stderr, "[c54x] FBWATCH-PAGE-RD #%u val=0x%04x PC=0x%04x insn=%u\n",
                        wpg, s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                        s->pc, s->insn_count);
        }
    }
    /* Timer registers (0x0024-0x0026) — read returns current value */
    if (addr == TIM_ADDR) { g_pmap_mmr_handled = 1; return s->data[TIM_ADDR]; }
    if (addr == PRD_ADDR) { g_pmap_mmr_handled = 1; return s->data[PRD_ADDR]; }
    if (addr == TCR_ADDR) {
        /* TCR: PSC is read from bits 9:6, rest from stored value */
        uint16_t tcr = s->data[TCR_ADDR] & ~TCR_PSC_MASK;
        tcr |= (s->timer_psc & 0xF) << TCR_PSC_SHIFT;
        g_pmap_mmr_handled = 1;
        return tcr;
    }

    /* CLKMD (0x58) — C54x clock-mode register (SPRU131G): bit1 = PLLNDIV
     * (requested mode, R/W), bit0 = PLLSTATUS (read-only: 0 = divider mode,
     * 1 = PLL mode, set once the switch/lock completes). The 5110 PROM
     * deep-sleep wrapper (0x7EB7) writes 0xFFFD (PLLNDIV=0) and polls
     * bit0==0, idle(3)s, then writes 0x3146 (PLLNDIV=1) and polls bit0==1.
     * Echo the requested mode as locked-instantly so both polls fall
     * through (real lock = PLLCOUNT cycles; firmware only ever polls). */
    if (addr == 0x58) {
        g_pmap_mmr_handled = 1;
        return (uint16_t)((s->data[0x58] & ~1u) | ((s->data[0x58] >> 1) & 1u));
    }

    /* MMR region */
    if (addr < 0x20) {
        g_pmap_mmr_handled = 1;   /* CPU MMRs — modeled (switch default reverts) */
        switch (addr) {
        case MMR_IMR:  return s->imr;
        case MMR_IFR:
        {
            static int ifr_log = 0;
            if ((s->ifr & 0x0020) && ifr_log < 10) {
                /* bit 5 = BRINT0 per C54X header (vec 21). */
                C54_LOG("IFR READ=0x%04x (BRINT0 pending) PC=0x%04x", s->ifr, s->pc);
                ifr_log++;
            }
            return s->ifr;
        }
        case MMR_ST0:  return s->st0;
        case MMR_ST1:  return s->st1;
        case MMR_AL:   return (uint16_t)(s->a & 0xFFFF);
        case MMR_AH:   return (uint16_t)((s->a >> 16) & 0xFFFF);
        case MMR_AG:   return (uint16_t)((s->a >> 32) & 0xFF);
        case MMR_BL:   return (uint16_t)(s->b & 0xFFFF);
        case MMR_BH:   return (uint16_t)((s->b >> 16) & 0xFFFF);
        case MMR_BG:   return (uint16_t)((s->b >> 32) & 0xFF);
        case MMR_T:    return s->t;
        case MMR_TRN:  return s->trn;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            return s->ar[addr - MMR_AR0];
        case MMR_SP:   return s->sp;
        case MMR_BK:   return s->bk;
        case MMR_BRC:  return s->brc;
        case MMR_RSA:  return s->rsa;
        case MMR_REA:  return s->rea;
        case MMR_PMST: return s->pmst;
        case MMR_XPC:  return s->xpc;
        default: g_pmap_mmr_handled = 0; return 0;   /* undefined CPU MMR */
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram) {
            uint16_t val = s->api_ram[addr - C54X_API_BASE];
            /* Log ALL API reads during interrupt handler (first 100) */
            static int api_rd_log = 0;
            if (api_rd_log < 100 && s->insn_count > 66000) {
                C54_LOG("API RD [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                        addr, val, s->pc, s->insn_count);
                api_rd_log++;
            }
            return val;
        }
    }

    /* Log data reads during SINT17 handler (PC in 0xFFC0-0xFFFF) */
    if (s->pc >= 0xFFC0 && s->insn_count > 66090) {
        static int handler_rd_log = 0;
        if (handler_rd_log < 30) {
            C54_LOG("H_RD [0x%04x]=0x%04x PC=0x%04x", addr, s->data[addr], s->pc);
            handler_rd_log++;
        }
    }

    return s->data[addr];
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val);

/* === Stack-write ring (ORPHAN trace 2026-05-30) : capture les data_write vers
 * la zone pile, pour nommer AU POPM ST0 @0xf48b qui a écrit le slot lu (clobber
 * sous SP, famille circulaire/dual-operand). No-fire au dump = personne n'a
 * écrit le slot dans la frame → slot STALE → SP désaligné (POP sans PUSH). */
typedef struct { uint16_t addr, val, pc, op; } StkwEv;
#define STKW_RING_N 48
static StkwEv   g_stkw_ring[STKW_RING_N];
static unsigned g_stkw_idx  = 0;
static int      g_orphan_on = -1;
static void stkw_rec(C54xState *s, uint16_t addr, uint16_t val)
{
    if (g_orphan_on < 0) g_orphan_on = getenv("CALYPSO_ORPHAN") ? 1 : 0;  /* env dédiée (hors CALYPSO_DEBUG → master reste 0, anti-Heisenbug) */
    if (!g_orphan_on) return;
    if (addr < 0x1000 || addr > 0x6000) return;   /* zone pile (SP dérive 0x1100→0x56xx) */
    StkwEv *e = &g_stkw_ring[g_stkw_idx % STKW_RING_N];
    e->addr = addr; e->val = val; e->pc = s->pc; e->op = prog_fetch(s, s->pc);
    g_stkw_idx++;
    /* Track-value : nomme le CALL/push qui pose la valeur orpheline (défaut
     * 0x3125, override CALYPSO_TRACK_STKVAL=0xNNNN) → corréler avec RCD@0x765c
     * et POPM@0xf48b en ordre insn. */
    {
        static int tval = -2;
        if (tval == -2) {
            const char *te = getenv("CALYPSO_TRACK_STKVAL");
            tval = (te && *te) ? (int)strtol(te, NULL, 0) : 0x3125;
        }
        if (tval >= 0 && val == (uint16_t)tval)
            fprintf(stderr, "[c54x] STK-PUSH val=0x%04x addr=0x%04x PC=0x%04x op=0x%04x SP=0x%04x insn=%u\n",
                    val, addr, s->pc, prog_fetch(s, s->pc), s->sp, s->insn_count);
    }
}

static void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    /* DSP54_CELLWATCH=0xLO:0xHI — log every DSP-side store into a DARAM range (who
     * clobbers live code). MCU window writes are logged separately in mad2_dsp_c54x. */
    {
        static int cw = -1; static unsigned cwlo, cwhi, cwn = 0;
        if (cw < 0) { const char *e = getenv("DSP54_CELLWATCH");
            if (e && *e && sscanf(e, "%x:%x", &cwlo, &cwhi) == 2) cw = 1; else cw = 0; }
        if (cw && addr >= cwlo && addr <= cwhi && cwn < 200000) { cwn++;
            fprintf(stderr, "[c54x] CELLWATCH data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count); }
    }
    /* === SENTINELLE cohérence ARM<->DSP (CALYPSO_FBDET_SENTINEL=1) ===
     * Force toute écriture DSP de d_fb_det (0x08f8) à 0xDEAD. L'ARM lit ce mot
     * via arm=0x01f0 -> s->dsp->data[0x08f8] (calypso_trx.c). La sonde "ARM RD
     * d_fb_det" (token TRX) montre alors :
     *   ARM lit 0xDEAD  -> mémoire COHÉRENTE (même cellule) -> H1 mort, c'est H3 (PM/seuil).
     *   ARM lit 0x0000  -> DÉSYNC ARM<->DSP -> H1 confirmé.
     * NE PAS combiner avec CALYPSO_FORCE_TOA (qui override la lecture 0x01f0). */
    {
        static int sent = -1;
        if (sent < 0) { const char *e = getenv("CALYPSO_FBDET_SENTINEL"); sent = e ? atoi(e) : 0;
            if (sent==1) fprintf(stderr, "[c54x] FBDET-SENTINEL=1 FORCE : data[0x08f8] forcé à 0xDEAD\n");
            else if (sent==2) fprintf(stderr, "[c54x] FBDET-SENTINEL=2 MONITOR : logge la vraie valeur écrite à 0x08f8 (pas de force)\n"); }
        if (sent && addr == 0x08f8) {
            uint16_t orig = val;
            if (sent == 1) val = 0xDEAD;   /* mode FORCE (test cohérence) */
            static unsigned sn = 0;
            if (sn++ < 40 || (sn % 2000) == 0)
                fprintf(stderr, "[c54x] FBDET-SENTINEL #%u DSP write d_fb_det[0x08f8] orig=0x%04x%s PC=0x%04x insn=%u\n",
                        sn, orig, (sent==1) ? " ->0xDEAD" : " (monitor)", s->pc, s->insn_count);
        }
    }
    /* DSP54_MBOXLOG=1 (loader1<->MCU upload ping-pong): log DSP-side writes to the
     * upload mailbox/header/remaining cells (0x87B-0x881) and any write that clobbers
     * loader1's own code (0xF00-0xFB9). Pairs with MDILOG (MCU side) to pin where the
     * double-buffered block handshake desyncs. PC names the loader1 site: f24=mbox0 ack,
     * f40=mbox1 ack, f1f=dest-ptr advance. Cap 300. */
    {
        static int mbl = -1; static unsigned mbn = 0;
        if (mbl < 0) mbl = getenv("DSP54_MBOXLOG") ? 1 : 0;
        if (mbl && mbn < 4000) {
            const char *tag = 0;
            if (addr == 0x87B) tag = "hdr.dest_even[7B]";
            else if (addr == 0x87C) tag = "hdr.dest_odd[7C]";
            else if (addr == 0x87D) tag = "remaining_hi[7D]";
            else if (addr == 0x87E) tag = "remaining_lo[7E]";
            else if (addr == 0x87F) tag = "MBOX0[7F]";
            else if (addr == 0x880) tag = "MBOX1[80]";
            else if (addr == 0x881) tag = "hdr.len[81]";
            else if (addr >= 0x0F00 && addr <= 0x0FB9) tag = "**LOADER1-CODE-CLOBBER**";
            if (tag) {
                fprintf(stderr, "[c54x] MBOXLOG #%u data[0x%04x] <- 0x%04x  %-22s PC=0x%04x insn=%u\n",
                        mbn, addr, val, tag, s->pc, s->insn_count);
                mbn++;
            }
        }
    }
    stkw_rec(s, addr, val);   /* ORPHAN : ring écritures pile */
    /* ORPHAN : tracker stores directs zone [0x1100..0x1140] (vecteur légit vs
     * vierge). Slots au-dessus de SP_base → jamais touchés par un push. */
    if (addr >= STKSLOT_LO && addr <= STKSLOT_HI) {
        int _si = addr - STKSLOT_LO;
        g_stkslot_wpc[_si] = s->pc;
        g_stkslot_wop[_si] = prog_fetch(s, s->pc);
        g_stkslot_written[_si] = 1;
    }
    /* MVPD overlay occupancy : count writes to [0x0080..0x27FF] during
     * boot phase. Env-gated CALYPSO_MVPD_TRACE=1. */
    if (g_mvpd_trace_enabled > 0) {
        mvpd_trace_record(addr);
    }
    /* ANGLE-WR tracer (2026-05-30) : qui écrit a_sync_demod[ANGLE]=0x08fc
     * (et TOA=0x08fa, SNR=0x08fd) = la SORTIE du vrai détecteur de fréquence
     * FCCH. Capture A/B (corr complexe) + réf (AR3/4/5) au store → le vrai
     * site de corrélation fréquentielle. Cap 40. */
    /* FBMODE-WR : qui écrit d_fb_mode (0x08f9) et avec quelle valeur (large
     * vs étroit) ? Si bascule étroit après le rejet boot → plus de cold-acq. */
    if (addr == 0x08f9) {
        static unsigned fm = 0;
        if (fm < 40) {
            fprintf(stderr, "[c54x] FBMODE-WR #%u d_fb_mode <- 0x%04x PC=0x%04x insn=%u\n",
                    fm, val, s->pc, s->insn_count);
            fm++;
        }
    }
    /* === FBWATCH (env CALYPSO_FBWATCH, one-shot, hors master-gate) === */
    if (g_fbwatch_on < 0) g_fbwatch_on = getenv("CALYPSO_FBWATCH") ? 1 : 0;
    if (g_fbwatch_on) {
        /* (1) qui ÉCRIT le flag dispatch FB (slot data[0x60-0x70] / 0x3dc0-2) ? */
        if ((addr >= 0x0060 && addr <= 0x0070) || (addr >= 0x3dc0 && addr <= 0x3dc2)) {
            static unsigned wf = 0;
            if (wf++ < 80)
                fprintf(stderr, "[c54x] FBWATCH-FLAG data[0x%04x] <- 0x%04x PC=0x%04x insn=%u%s\n",
                        addr, val, s->pc, s->insn_count, val ? "  *** NON-ZERO ***" : "");
        }
        /* (3) le DSP écrit-il une détection FB (d_fb_det 0x08f8 non-zéro) ? */
        if (addr == 0x08f8 && val) {
            static unsigned wd = 0;
            if (wd++ < 40)
                fprintf(stderr, "[c54x] FBWATCH-DET d_fb_det <- 0x%04x PC=0x%04x insn=%u\n",
                        val, s->pc, s->insn_count);
        }
        /* (5) LE FLAG PRODUCTEUR : qui écrit data[0x585f] (mot d'état foreground/ISR) ?
         * bit7 (0x0080) pollé par le foreground @0xf7af, bit8 (0x0100) par les ISR.
         * Si jamais écrit / bit7 jamais posé = le producteur du flag manque. */
        if (addr == 0x585f) {
            static unsigned w5 = 0;
            if (w5++ < 80)
                fprintf(stderr, "[c54x] FBWATCH-585F data[0x585f] <- 0x%04x PC=0x%04x insn=%u%s\n",
                        val, s->pc, s->insn_count, (val & 0x0080) ? "  *** BIT7 SET ***" : "");
        }
        /* la table de dispatch est-elle peuplée ? data[0x4c5b] = cible BACC A @0x7127 */
        if (addr == 0x4c5b) {
            static unsigned wt = 0;
            if (wt++ < 20)
                fprintf(stderr, "[c54x] FBWATCH-INITTAB-WR data[0x4c5b] <- 0x%04x PC=0x%04x insn=%u\n",
                        val, s->pc, s->insn_count);
        }
    }
    if (addr >= 0x08fa && addr <= 0x08fd) {
        static unsigned aw = 0;
        if (aw < 40) {
            int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
            const char *nm = addr==0x08fa?"TOA":addr==0x08fb?"PM":addr==0x08fc?"ANGLE":"SNR";
            fprintf(stderr, "[c54x] ANGLE-WR #%u %s[0x%04x]<-0x%04x PC=0x%04x A=%lld B=%lld T=%04x | "
                    "AR2=%04x[%04x] AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                    aw, nm, addr, val, s->pc, (long long)a, (long long)b, s->t,
                    s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                    s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]], s->insn_count);
            aw++;
        }
    }
    /* MTTCG lock : voir data_read ci-dessus. */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    data_write_locked(s, addr, val);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    if (addr < 0x60)
        c54x_pmap_record(1, 1, addr, val, s->pc, s->insn_count, g_pmap_mmr_handled);
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val)
{
    g_pmap_mmr_handled = 0;   /* PERIPHMAP structural flag: handler cases set 1 */
    /* === NDB-CTL-WR : trace ARM-side writes to NDB control flags in
     * [data[0x08F8]..data[0x0900]] = d_fb_det, d_fb_mode, a_sync_demod[],
     * d_sb_ext, etc. The firmware writes mode flags before scheduling
     * SB task — finding which flag toggles SB vs FB tells the DSP
     * dispatcher selector. Capped 50. */
    {
        bool ndb_ctl = (addr >= 0x08F8 && addr <= 0x0900);
        /* Filter on s->pc to identify ARM-side writes : ARM has no PC
         * concept here (it writes via MMIO callback), so s->pc reflects
         * the DSP PC at the moment. ARM-side calls land via calypso_dsp_write
         * which does direct s->data[] write, NOT data_write_locked → so
         * this probe sees only DSP-side writes to NDB. Both paths are
         * useful to discriminate. */
        if (ndb_ctl) {
            static unsigned ndb_log = 0;
            if (ndb_log++ < 50) {
                if (calypso_debug_enabled("NDB-CTL-WR")) fprintf(stderr,
                        "[c54x] NDB-CTL-WR data[0x%04x] <- 0x%04x "
                        "(was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, s->insn_count);
            }
        }
    }

    /* === D13D-WR probe (selftest off20/off21 cipher-input cells 0x13d1..0x13d5) ===
     * Enable with CALYPSO_DEBUG=D13D-WR. Traces DSP-side writes to the cipher's
     * true input cells so we can find their real (analog/COBBA) source. Capped 400. */
    if (addr >= 0x13d1 && addr <= 0x13d5) {
        if (calypso_debug_enabled("D13DWR")) {
            static unsigned d13d_log;
            if (d13d_log++ < 400)
                fprintf(stderr,
                        "[c54x] D13D-WR #%u data[0x%04x] <- 0x%04x (was 0x%04x) "
                        "PC=0x%04x op=0x%04x A=0x%010llx B=0x%010llx "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        d13d_log, addr, val, s->data[addr], s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        }
    }

    /* === SYNC-DEMOD-WR probe (2026-05-28) ===
     * Trace writes to a_sync_demod cells [0x08FA..0x08FD] (= TOA/PM/ANGLE/SNR
     * per NDB layout, indices D_TOA=0 D_PM=1 D_ANGLE=2 D_SNR=3).
     * Filter OUT les PCs stale-AR connus (0x821a 0x8213 0x8217) qui sont
     * juste du AR4-walk garbage. Le but est de capturer le VRAI publisher
     * (= la routine FB-det code DSP qui calcule les vraies valeurs). Si
     * apres tout le run on ne voit que les 3 PCs garbage → le vrai code
     * n'est jamais atteint. Cap 200 entries. */
    if (addr >= 0x08FA && addr <= 0x08FD) {
        if (s->pc != 0x821a && s->pc != 0x8213 && s->pc != 0x8217) {
            static unsigned sd_log;
            const unsigned LIMIT = 200;
            if (sd_log < LIMIT) {
                const char *name = (addr == 0x08FA) ? "TOA"
                                 : (addr == 0x08FB) ? "PM"
                                 : (addr == 0x08FC) ? "ANGLE"
                                 : "SNR";
                fprintf(stderr,
                        "[c54x] SYNC-DEMOD-WR #%u %s[0x%04x] <- 0x%04x "
                        "(was 0x%04x) PC=0x%04x op=0x%04x "
                        "A=0x%010llx B=0x%010llx "
                        "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                        sd_log, name, addr, val, s->data[addr],
                        s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                        s->insn_count);
                sd_log++;
                if (sd_log == LIMIT)
                    fprintf(stderr,
                            "[c54x] SYNC-DEMOD-WR log capped at %u\n", LIMIT);
            }
        }
    }

    /* === FB-DET-WR probe (2026-05-28) ===
     * Trace specifically writes to d_fb_det (DSP word 0x08F8) by PC.
     * Run post-option-A shows d_fb_det stuck at 0x1255 (96×), no varied
     * lock signature. Question : ONE site écrit toujours 0x1255, OU plusieurs
     * sites écrivent (mais ARM ne consume que celui qui produit 0x1255) ?
     * Snapshot par-événement : PC + B accumulator (= source du STH/STL),
     * prev_op (= instruction précédente, contexte d'addressing), trail des
     * 4 PCs précédents pour identifier la routine. Cap 300. */
    if (addr == 0x08F8) {
        static unsigned fbdet_log;
        const unsigned LIMIT = 300;
        if (fbdet_log < LIMIT) {
            fprintf(stderr,
                    "[c54x] FB-DET-WR #%u data[0x08F8] <- 0x%04x "
                    "PC=0x%04x op=0x%04x prev=0x%04x "
                    "B=0x%010llx A=0x%010llx SP=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    fbdet_log, val,
                    s->pc, s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc - 1)],
                    (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->sp,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                    s->insn_count);
            fbdet_log++;
            if (fbdet_log == LIMIT)
                fprintf(stderr, "[c54x] FB-DET-WR log capped at %u\n", LIMIT);
        }
    }

    /* === A_SCH-WR probe : trace DSP writes to a_sch[0..4] in both db_r
     * pages. If DSP never writes these cells, firmware reads stale RAM
     * → random BSIC in FBSB_CONF. If DSP writes garbage, the SCH demod
     * path is broken upstream. Helps discriminate the SB sync root cause.
     * Capped at 50 logged hits to avoid spam. */
    {
        bool a_sch_p0 = (addr >= 0x0837 && addr <= 0x083B);  /* page 0 a_sch[0..4] */
        bool a_sch_p1 = (addr >= 0x084B && addr <= 0x084F);  /* page 1 a_sch[0..4] */
        if (a_sch_p0 || a_sch_p1) {
            static unsigned a_sch_log = 0;
            if (a_sch_log++ < 50) {
                if (calypso_debug_enabled("A_SCH-WR")) fprintf(stderr,
                        "[c54x] A_SCH-WR data[0x%04x] <- 0x%04x page=%d "
                        "idx=%d PC=0x%04x insn=%u\n",
                        addr, val,
                        a_sch_p0 ? 0 : 1,
                        (int)(addr - (a_sch_p0 ? 0x0837 : 0x084B)),
                        s->pc, s->insn_count);
            }
        }
    }

    /* === BLOB-WR diagnostic for dsp_blobs/ test harness ===
     * Logs writes either targeting scratch [0x2000..0x200F] (dsp-deadbeef
     * etc.) or carrying a known blob signature value. Self-throttled to
     * 5000 hits per process. Zero impact when no blob test is running. */
    {
        static unsigned blob_wr_count = 0;
        bool is_scratch = (addr >= 0x2000 && addr <= 0x200F);
        bool is_magic = (val == 0xCAFE || val == 0xBEEF || val == 0xDEAD ||
                         val == 0x2B2B || val == 0x4906 || val == 0x1B00 ||
                         val == 0x7080 || val == 0x4000);
        if ((is_scratch || is_magic) && blob_wr_count < 5000) {
            blob_wr_count++;
            fprintf(stderr,
                    "[c54x] BLOB-WR data[0x%04x] <- 0x%04x PC=0x%04x insn=%u\n",
                    addr, val, s->pc, s->insn_count);
        }
    }

    /* === SP-CATASTROPHE fix : DROM[0x9187] silicon-correct read-only ===
     * Per SPRU172C : when PMST.DROM=1, the DSP ROM in data space is
     * read-only. Our emulator previously allowed writes which corrupted
     * data[0x9187] (0xFF86 → 0xF6B7) via a walking `STH B,*AR2+` inside
     * the RPTB body [0x815E..0x8176]. Dispatcher at 0x8341..0x8353 then
     * read the corrupted value and computed CALAD-A target = 0x70C3
     * (= the CALA-A opcode itself in PROM0) instead of the legit MAC
     * routine 0x8261, falling into a self-call loop. Each iteration
     * pushed one word, SP eventually reached MMR_SP (0x0018), the push
     * aliased SP itself → SP-CATASTROPHE Δ=+28843.
     *
     * Surgical : only data[0x9187] needs protection (the dispatcher LUT
     * slot). Wider ranges break other firmware paths (calibrated against
     * historical SARAM-overlay writability). Drop silently — matches
     * silicon. Probe first 20 attempts for diag. */
    /* 2026-05-29 v2 : protège la COLONNE LUT du dispatcher dans la DROM, PAS
     * toute la DROM (le full-DROM v1 bloquait le scratch firmware 0x9380/
     * 0x93c2/0xd4xx → readback stale → DSP coincé en 0xebf0). Le dispatcher
     * lit une LUT par-tâche à data[(DP<<7)|0x07] = 0x9187, 0x9207, 0x9287…
     * (toujours offset 0x07 ; DP = numéro de tâche). Le `STH B,*AR2+` walking
     * (RPTB 0x815E-0x8176) corrompait 0x9207 (natif 0xff72 → 0xf6b7) →
     * CALAD-A=0x70c3 au lieu de 0x8239 → self-CALA → SP drain → snr=0.
     * On bloque uniquement les writes DROM à offset 0x07 (la colonne LUT) :
     * le scratch firmware est à d'autres offsets (0x00/0x42/0x60…), préservé. */
    /* DROM-LUT column protection — garde PMST.DROM RETIRÉE 2026-05-29.
     * Preuve (run 459M, /root/qemu.log) : PMST=0x70c4 vu 149× = PMST(MMR 0x1D)
     * lui-même clobbé par le self-CALA 0x70c3 (SP drain) → bit DROM(0x08)=0 →
     * la garde se désactivait elle-même → LUT 0x9207 re-corrompue → 0x70c3 →
     * boucle auto-entretenue (148838× le self-CALA). DROM-W-DROP n'avait fiché
     * que 2× (DROM encore =1) puis silence. Le firmware ne tourne JAMAIS
     * légitimement en DROM=0 (PMST légit = 0xffa8/0xffb8, DROM=1 dans les deux)
     * → offset 0x07 read-only SANS garde DROM. */
    if (addr >= 0x9000 && addr <= 0xDFFF && (addr & 0x7F) == 0x07) {
        static unsigned drom_w_attempts = 0;
        if (drom_w_attempts++ < 40) {
            if (calypso_debug_enabled("DROM-W-DROP")) fprintf(stderr,
                    "[c54x] DROM-W-DROP data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u pmst=0x%04x (LUT col, read-only)\n",
                    addr, val, s->data[addr], s->pc, s->insn_count, s->pmst);
        }
        return;
    }

    /* FBDB-PROBE write to 0x3DC0 (= SARAM flag polled by fc63 BITF).
     * Env CALYPSO_FBDB_PROBE=1. Logs old→new + which bits set, with focus
     * on bit 4 (= 0x0010) since that's the bit fc63 tests via BITF. */
    if (addr == 0x3DC0 && g_fbdb_probe_enabled > 0) {
        fbdb_probe_write_3dc0(addr, s->data[addr], val, s->pc, s->insn_count);
    }
    /* COEFFS-WR probe : watch-write sur la zone [0x2bc0..0x2bff] (64 mots).
     * 2026-05-14 evening — COEFFS-DUMP a montré une séquence init→clear→use :
     *   insn=2M  PC=0x9b05  vraies coeffs (f320, a660, ...)
     *   insn=3M  PC=0x9abc  pattern uniforme 0x0001 (suspect)
     *   insn=4M  PC=0x9abd  ALL ZERO (clear)
     *   insn=9M+ PC=0x8f51  ALL ZERO toujours (read par correlator)
     *
     * v2 (run précédent cap 200) : 3 clusters identifiés —
     *   0x8216 (wk=OTHER, 23 hits) = vraies coeffs
     *   0x9ace (wk=8, 64 hits)    = clear partiel
     *   0x9abf (wk=8x, 113 hits)  = pattern 0x0001
     * Cap atteint à insn=1.65M. On veut savoir si 0x8216 refire après.
     *
     * v3 (ce patch) :
     *   - Per-PC counter global sur tout le run (jamais capé)
     *   - Throttled log : 500 premiers + transitions PC + 1/100k insns
     *   - Summary périodique tous les 5M insns dump tous les PCs avec count>0
     * Tranche (1) re-fire 0x8216 manqué vs (2) one-shot définitif. */
    /* Timing trackers par cluster (utilisés par la sonde 0x8f51) — gardés
     * en dehors du helper car cluster-specific. Le watch_write_zone_check
     * factorise le compteur per-PC + log + summary. */
    if (addr >= 0x2bc0 && addr <= 0x2bff) {
        uint16_t exec_pc = s->last_exec_pc;
        if (exec_pc == 0x8216 || exec_pc == 0x8217 || exec_pc == 0x8218) {
            g_fb_det_timing.last_compute_insn = s->insn_count;
            g_fb_det_timing.last_compute_addr = addr;
        } else if (exec_pc == 0x9ace) {
            g_fb_det_timing.last_clear_insn = s->insn_count;
            g_fb_det_timing.last_clear_addr = addr;
        } else if (exec_pc == 0x9abf) {
            g_fb_det_timing.last_pattern_insn = s->insn_count;
            g_fb_det_timing.last_pattern_addr = addr;
        }
    }
    {
        static WatchWriteState wws_coeffs;
        watch_write_zone_check(s, addr, val, "COEFFS", 0x2bc0, 0x2bff, &wws_coeffs);
    }
    /* A_CD-WR : a_cd[15] in NDB starts at DSP word 0x09D0 (= API byte 0x03A0,
     * = NDB byte offset 0x1F8). 15 words = [0x09D0..0x09DE].
     * Cible : tracker si le DSP CCCH demod (DSP_TASK_ALLC) écrit ses résultats. */
    {
        static WatchWriteState wws_a_cd;
        watch_write_zone_check(s, addr, val, "A_CD", 0x09d0, 0x09de, &wws_a_cd);
        /* A_CD-BY-BURST : corrélation a_cd[] writes avec d_burst_d courant.
         * Si DSP fait burst 0→1→2→3 → ~25% des writes par burst_id.
         * Si on voit 0 writes avec burst=3 → DSP n'écrit jamais la fin de
         * séquence, d'où firmware ARM nb_resp bail (sous-cause #3). */
        if (addr >= 0x09d0 && addr <= 0x09de) {
            static uint64_t a_cd_by_burst[16];
            static uint64_t a_cd_corr_total;
            static uint64_t a_cd_corr_last_log;
            uint16_t b = g_last_d_burst_d & 0xF;
            a_cd_by_burst[b]++;
            a_cd_corr_total++;
            if (a_cd_corr_total - a_cd_corr_last_log >= 1000) {
                a_cd_corr_last_log = a_cd_corr_total;
                if (calypso_debug_enabled("A_CD-BY-BURST")) fprintf(stderr,
                        "[c54x] A_CD-BY-BURST total=%llu "
                        "burst[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                        (unsigned long long)a_cd_corr_total,
                        (unsigned long long)a_cd_by_burst[0],
                        (unsigned long long)a_cd_by_burst[1],
                        (unsigned long long)a_cd_by_burst[2],
                        (unsigned long long)a_cd_by_burst[3],
                        (unsigned long long)(a_cd_corr_total -
                                             a_cd_by_burst[0] - a_cd_by_burst[1] -
                                             a_cd_by_burst[2] - a_cd_by_burst[3]));
            }
        }
    }
    /* D_BURST_D probe (2026-05-15 midi) — watch d_burst_d à 0x0829 (page 0)
     * et 0x083D (page 1). Mesures : per-PC counter, transition matrix,
     * histogramme. Tranche la sous-cause :
     *   0,1,2,3 séquentiel → DSP signale correct, bug ARM-side
     *   0,1,2 jamais 3     → DSP cale au 4e burst (sous-cause #3)
     *   pas de write       → DSP n'écrit pas cette cellule du tout
     */
    if (addr == 0x0829 || addr == 0x083D) {
        static uint64_t db_total[2];
        static uint64_t db_per_pc[2][0x10000];
        static uint16_t db_prev[2];
        static uint64_t db_trans[2][16][16];
        static uint64_t db_last_log[2];
        static uint64_t db_last_summary[2];
        int page = (addr == 0x083D) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = db_prev[page];
        uint16_t curr_val = val & 0xF;
        db_total[page]++;
        db_per_pc[page][exec_pc]++;
        if (prev_val < 16 && curr_val < 16) {
            db_trans[page][prev_val][curr_val]++;
        }
        db_prev[page] = curr_val;
        g_last_d_burst_d = curr_val;  /* propage pour A_CD-BY-BURST */
        bool should_log = db_total[page] <= 200
            || (s->insn_count - db_last_log[page]) > 100000;
        if (should_log) {
            if (calypso_debug_enabled("D_BURST_D-WR")) fprintf(stderr,
                    "[c54x] D_BURST_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=%u curr=%u insn=%u\n",
                    page, (unsigned long long)db_total[page], addr, val,
                    exec_pc, prev_val, curr_val, s->insn_count);
            db_last_log[page] = s->insn_count;
        }
        if (s->insn_count - db_last_summary[page] >= 5000000) {
            db_last_summary[page] = s->insn_count;
            if (calypso_debug_enabled("D_BURST_D-SUMMARY")) fprintf(stderr,
                    "[c54x] D_BURST_D-SUMMARY page=%d total=%llu trans:",
                    page, (unsigned long long)db_total[page]);
            for (int p = 0; p < 8; p++) {
                for (int c = 0; c < 8; c++) {
                    if (db_trans[page][p][c]) {
                        fprintf(stderr, " %u->%u=%llu",
                                p, c, (unsigned long long)db_trans[page][p][c]);
                    }
                }
            }
            fprintf(stderr, "\n");
        }
    }
    /* D_TASK_D probe (2026-05-15 fin journée) — watch d_task_d à 0x0828
     * (page 0) et 0x083C (page 1), READ side de la struct db_buf_r.
     * ARM L1 prim_rx_nb lit ce champ via dsp_api.db_r->d_task_d et bail
     * avec puts("EMPTY") si == 0. 60 EMPTY printf observés sous synth=1
     * banc d'essai déterministe : DSP n'écrit jamais cette cellule (ou
     * écrit 0). Cette probe trace : qui écrit ? quand ? avec quelle valeur ?
     * Distinguer entre :
     *   - DSP ne touche jamais 0x0828/0x083C
     *   - DSP écrit val=0 (clear/init seulement)
     *   - DSP écrit val=24 (DSP_TASK_ALLC) mais ARM lit avant le write
     */
    if (addr == 0x0828 || addr == 0x083C) {
        static uint64_t dt_total[2];
        static uint16_t dt_prev[2];
        static uint64_t dt_last_log[2];
        int page = (addr == 0x083C) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = dt_prev[page];
        uint16_t curr_val = val;
        dt_total[page]++;
        dt_prev[page] = curr_val;
        bool should_log = dt_total[page] <= 200
            || (s->insn_count - dt_last_log[page]) > 100000;
        if (should_log) {
            if (calypso_debug_enabled("D_TASK_D-WR")) fprintf(stderr,
                    "[c54x] D_TASK_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=0x%04x insn=%u\n",
                    page, (unsigned long long)dt_total[page], addr, val,
                    exec_pc, prev_val, s->insn_count);
            dt_last_log[page] = s->insn_count;
        }
    }
    /* DATA-W-MMR : log every write into the low MMR window (addr <= 0x1F)
     * with full attribution context. Goal : disambiguate the IMR-W trace
     * cascade observed at PC=0x8eb9 (op=0xf3e1) and PC=0x9ad0 (op=0x8192).
     * The writer_kind field tells us *which path* triggered the write
     * (opcode family / IRQ ack / ARM MMIO / resolve_smem side effect).
     * Cap at 200 distinct events to avoid log flood. */
    if (addr <= 0x1F) {
        static unsigned mmrw_log;
        if (mmrw_log++ < 200) {
            const char *wk_name[] = {
                "UNK", "F3", "8x", "77", "76", "PSHM",
                "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
            };
            uint8_t wk = s->writer_kind;
            const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                              ? wk_name[wk] : "??";
            if (calypso_debug_enabled("DATA-W-MMR")) fprintf(stderr,
                    "[c54x] DATA-W-MMR addr=0x%02x val=0x%04x "
                    "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x "
                    "xpc=%d wk=%s "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "SP=%04x DP=%d INTM=%d insn=%u\n",
                    addr, val,
                    s->last_exec_pc, s->pc, s->prog[s->pc],
                    s->xpc, wkn,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    s->sp, dp(s),
                    !!(s->st1 & ST1_INTM),
                    s->insn_count);
        }
    }

    /* WATCH-WR-ADDR (générique, gated CALYPSO_DEBUG=WATCH-WR + env
     * CALYPSO_WATCH_WR_ADDR=0xNNNN) : log tout write vers une adresse data
     * arbitraire — pour tracer qui écrit (ou jamais) une cellule pointeur-
     * dispatcher SARAM (ex. 0x3af7) qui tombe à 0 → CALA→0 → bootstub. */
    {
        static int watch_wr_addr = -1;
        if (watch_wr_addr < 0) {
            const char *e = getenv("CALYPSO_WATCH_WR_ADDR");
            watch_wr_addr = (e && *e) ? (int)strtol(e, NULL, 0) : 0;
        }
        if (watch_wr_addr && addr == (uint16_t)watch_wr_addr) {
            C54_DBG("WATCH-WR",
                "WATCH-WR data[0x%04x] <- 0x%04x (was 0x%04x) PC=0x%04x "
                "DP=0x%03x insn=%u",
                addr, val, s->data[addr], s->pc, (s->st0 & 0x1FF),
                (unsigned)s->insn_count);
        }
        /* CALYPSO_WR_LO/HI=<addr> + CALYPSO_WR_WIN=<insn>: log EVERY DSP data-write
         * into [LO,HI] up to insn WIN (find the early host-mailbox / boot-status
         * writer the firmware reads via the HPI window). Stderr direct (not C54_DBG). */
        {
            static int wr_init = 0, wr_lo = -1, wr_hi = 0; static unsigned wr_win = 0; static unsigned wr_n = 0;
            if (!wr_init) { wr_init = 1; const char *l = getenv("CALYPSO_WR_LO");
                if (l && *l) { wr_lo = (int)strtol(l,0,0);
                    const char *h = getenv("CALYPSO_WR_HI"); wr_hi = h&&*h ? (int)strtol(h,0,0) : wr_lo;
                    const char *w = getenv("CALYPSO_WR_WIN"); wr_win = w&&*w ? (unsigned)strtoul(w,0,0) : 1000000u; } }
            if (wr_lo >= 0 && addr >= (uint16_t)wr_lo && addr <= (uint16_t)wr_hi
                && s->insn_count <= wr_win && wr_n < 200) {
                wr_n++;
                fprintf(stderr, "[c54x] WRWIN data[0x%04x] <- 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                        addr, val, s->data[addr], s->pc, (unsigned)s->insn_count);
            }
        }
    }

    /* WATCH-WRITE on the same mailbox slots tracked in data_read.
     * Whoever writes them — DSP or ARM via api_ram alias — gets logged
     * so we can attribute the source of the value the firmware polls. */
    /* WATCH-WRITE 0x3dd2 — la cellule sur laquelle 0x75db poll en boucle
     * (37M reads/15s). Identifier qui écrit (et qui ne le fait pas).
     * Cas 1 : zéro write → un bloc compute ne fire jamais.
     * Cas 2 : write boot only → init OK mais set steady-state manquant.
     * Cas 3 : writes périodiques avec valeur jamais matchée par le test
     *         à 0x75db → bug dans le compute en amont. */
    /* WATCH-3FBE — observ. writes sur la zone [0x3fb0..0x3fbf] qui inclut
     * 0x3fbe (le slot pop=0 du bootstub-entry insn=3995013). Le BSP DMA
     * bypasse data_write_locked (direct s->data[]) donc ce hook ne voit
     * QUE les writes via instructions DSP (STL/STM/STLM/STH). Si zéro
     * write vu avant insn=3995013 → le firmware ne pousse jamais à cet
     * addr → trajectoire SP divergente (RETD à 0x8ed1 sans CALL apparié).
     * Env-gated CALYPSO_WATCH_3FBE=1, zéro coût sinon. */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static int      w3fbe_enabled = -1;
        static unsigned w3fbe_total = 0;
        if (w3fbe_enabled < 0) {
            const char *e = cdbg_env("WATCH-3FBE");
            w3fbe_enabled = (e && *e == '1') ? 1 : 0;
            if (w3fbe_enabled) {
                fprintf(stderr,
                    "[c54x] WATCH-3FBE enabled — range [0x3fb0..0x3fbf] "
                    "(DSP-side writes only, BSP DMA bypassed)\n");
            }
        }
        if (w3fbe_enabled > 0) {
            w3fbe_total++;
            if (w3fbe_total <= 100 || (w3fbe_total % 5000) == 0) {
                fprintf(stderr,
                    "[c54x] WATCH-3FBE #%u addr=0x%04x val=0x%04x "
                    "(was 0x%04x) PC=0x%04x insn=%u\n",
                    w3fbe_total, addr, val, s->data[addr],
                    s->pc, s->insn_count);
            }
        }
    }

    if (addr == 0x3dd2) {
        static unsigned w3dd2;
        w3dd2++;
        if (w3dd2 <= 100 || (w3dd2 % 1000) == 0) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE 0x3dd2 #%u <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u INTM=%d\n",
                    w3dd2, val, s->data[addr], s->pc, s->insn_count,
                    !!(s->st1 & ST1_INTM));
        }
    }
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x01F0) {
        static unsigned wcount;
        if (wcount++ < 30) {
            if (calypso_debug_enabled("WATCH-WRITE")) fprintf(stderr,
                    "[c54x] WATCH-WRITE data[0x%04x] <- 0x%04x  (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher pointer at data[0x3f65] — `LD *(0x3f65),A; CALA A` at
     * DARAM 0x008a-0x008c. When this slot holds 0xfff8/0x0000/garbage the
     * CALA jumps into PROM1 vec or boot stub NOPs and the SP runs away.
     * Trace every write so we can identify who populates / corrupts it. */
    if (addr == 0x3f65) {
        static unsigned dpw;
        if (dpw++ < 100) {
            if (calypso_debug_enabled("DISP-PTR")) fprintf(stderr,
                    "[c54x] DISP-PTR data[0x3f65] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher poll addresses — log ANY write so we identify the
     * code path that should populate them. Currently 0 PORTR PA=0xF430
     * fires because dispatcher reads 0 here forever. */
    if (addr == 0x4359 || addr == 0x3fab) {
        static unsigned dispw;
        if (dispw++ < 50) {
            if (calypso_debug_enabled("DISP-WRITE")) fprintf(stderr,
                    "[c54x] DISP-WRITE data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* CALAD source zone 0x4180-0x41FF — LD-A-TRACE shows the firmware
     * reads 0x4189 (DP=0x83) but our emulation has it as 0. Log every
     * write to this range so we can tell whether (a) anyone is meant to
     * populate it and we missed the path, or (b) DP=0x83 is itself a
     * symptom upstream of an unrelated bug. */
    if (addr >= 0x4180 && addr <= 0x41FF) {
        static unsigned cwz;
        if (cwz++ < 5000) {
            if (calypso_debug_enabled("CALAD-ZONE-W")) fprintf(stderr,
                    "[c54x] CALAD-ZONE-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dedicated watch on 0x4189 — never capped. The LD-A loop reads this
     * slot in the CALAD trap; we want to know if/when *anyone* finally
     * writes a non-zero value, and from which PC. */
    if (addr == 0x4189) {
        fprintf(stderr,
                "[c54x] *** WR-0x4189 *** data[0x4189] <- 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                val, s->data[addr], s->pc, s->insn_count);
    }
    /* === DARAM[0x40..0x90] watch — dispatcher flag area ===
     * The PROM0 idle dispatcher (0xCC62..0xCC6F) polls data[0x62] and
     * other slots in [0x60..0x70]. FORCE-DARAM62=1 (env) proves that
     * setting data[0x62]=1 makes the DSP escape and reach 0x770c, so
     * this range gates the runtime task pipeline. ARM-side writes to
     * the API page mirror at +0x0800 (calypso_trx.c calypso_dsp_write)
     * but never to DARAM 0x40..0x90 — so any value here must come from
     * DSP-self stores (ST/STH/STM/...) or stay zero forever. Capture
     * EVERY write with PC+INTM+insn so we can attribute the source.
     * INTM annotation lets us tell ISR-context writes from main code. */
    if (addr >= 0x0040 && addr <= 0x0090) {
        static unsigned daram_disp_w;
        if (daram_disp_w++ < 1000) {
            if (calypso_debug_enabled("DISP-FLAG-W")) fprintf(stderr,
                    "[c54x] DISP-FLAG-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x INTM=%d IFR=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc,
                    !!(s->st1 & ST1_INTM), s->ifr, s->insn_count);
            if (daram_disp_w == 1000) {
                if (calypso_debug_enabled("DISP-FLAG-W")) fprintf(stderr,
                        "[c54x] DISP-FLAG-W log capped at 1000 — pattern visible above\n");
            }
        }
    }
    /* Timer registers (0x0024-0x0026) — before MMR check */
    if (addr == TCR_ADDR) {
        if (s->timer_enabled) { static int tw = 0; if (tw < 16) { tw++;
            C54_LOG("TCR <= 0x%04x PC=0x%04x insn=%u (TSS=%d TRB=%d TDDR=%d) TIM=0x%04x PRD=0x%04x",
                    val, s->pc, s->insn_count, !!(val & TCR_TSS), !!(val & TCR_TRB),
                    val & TCR_TDDR_MASK, s->data[TIM_ADDR], s->data[PRD_ADDR]); } }
        /* TRB: write 1 → reload TIM from PRD, PSC from TDDR */
        if (val & TCR_TRB) {
            s->data[TIM_ADDR] = s->data[PRD_ADDR];
            s->timer_psc = val & TCR_TDDR_MASK;
        }
        /* Store TCR without TRB (TRB is write-only, always reads 0) */
        s->data[TCR_ADDR] = val & ~TCR_TRB;
        g_pmap_mmr_handled = 1;
        return;
    }
    if (addr == TIM_ADDR) { g_pmap_mmr_handled = 1; s->data[TIM_ADDR] = val; return; }
    if (addr == PRD_ADDR) {
        if (s->timer_enabled) { static int pw = 0; if (pw < 16) { pw++;
            C54_LOG("PRD <= 0x%04x PC=0x%04x insn=%u", val, s->pc, s->insn_count); } }
        g_pmap_mmr_handled = 1;
        s->data[PRD_ADDR] = val; return;
    }

    /* MMR region */
    if (addr < 0x20) {
        g_pmap_mmr_handled = 1;   /* CPU MMRs — modeled */
        /* === BOOT-MMR-WR probe : reach+effect test for boot init STMs ===
         * The DSP boot is supposed to STM #imm into SP, IMR, AR0..7, BK,
         * BRC, PMST shortly after the jump from 0xb418→0x76f8. Observed
         * runtime says SP/IMR/AR4/AR5 never receive their init values, so
         * either (a) PC never reaches the STM, or (b) the STM handler writes
         * to the wrong target. This probe answers BOTH : every write to
         * MMR 0..0x1E during boot phase is logged with PC + opcode + delta,
         * so we can see what got written, when, by what instruction. */
        if (s->insn_count <= 300000) {
            static unsigned bmw_log;
            const unsigned LIMIT = 800;
            if (bmw_log < LIMIT) {
                static const char *names[0x20] = {
                    "IMR","IFR","??02","??03","??04","??05","ST0","ST1",
                    "AL","AH","AG","BL","BH","BG","T","TRN",
                    "AR0","AR1","AR2","AR3","AR4","AR5","AR6","AR7",
                    "SP","BK","BRC","RSA","REA","PMST","XPC","??1F",
                };
                uint16_t old_val = (addr == MMR_IMR) ? s->imr
                                 : (addr == MMR_IFR) ? s->ifr
                                 : (addr == MMR_SP)  ? s->sp
                                 : (addr >= MMR_AR0 && addr <= MMR_AR7) ? s->ar[addr - MMR_AR0]
                                 : s->data[addr];
                fprintf(stderr,
                        "[c54x] BOOT-MMR-WR #%u insn=%u PC=%04x op=%04x "
                        "MMR[%02x %s] %04x → %04x\n",
                        bmw_log, s->insn_count, s->pc, s->prog[s->pc],
                        (unsigned)addr, names[addr],
                        old_val, val);
                bmw_log++;
                if (bmw_log == LIMIT) {
                    fprintf(stderr,
                            "[c54x] BOOT-MMR-WR log capped at %u\n", LIMIT);
                }
            }
        }
        switch (addr) {
        case MMR_IMR:
            if (val != s->imr) {
                static unsigned imr_log = 0;
                /* Always log transitions TO zero (mask-everything) — that
                 * is the cascade root suspected in 2026-05-08 v2 diag :
                 * IMR=0 → INT3 IFR pending forever → RPTB at 0xe9ac never
                 * exits. We need the PC + opcode of every IMR=0 write,
                 * uncapped, so we can identify the buggy code path. */
                bool to_zero = (val == 0);
                if (imr_log++ < 50 || to_zero) {
                    if (calypso_debug_enabled("IMR-W")) fprintf(stderr,
                            "[c54x] IMR-W %s 0x%04x → 0x%04x PC=0x%04x "
                            "op=0x%04x prev_op=0x%04x SP=0x%04x INTM=%d "
                            "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                            "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x "
                            "B=0x%010llx insn=%u\n",
                            to_zero ? "*ZERO*" : "      ",
                            s->imr, val, s->pc,
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc - 1)],
                            s->sp,
                            !!(s->st1 & ST1_INTM),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->insn_count);
                }
            }
            { static int il = -1; if (il < 0) il = getenv("DSP54_IMRLOG") ? 1 : 0;
              if (il) fprintf(stderr, "[c54x] IMR <- 0x%04x (was 0x%04x) PC=0x%04x xpc=%u insn=%u\n",
                              val, s->imr, s->pc, s->xpc & 3, s->insn_count); }
            s->imr = val; return;
        case MMR_IFR:  s->ifr &= ~val; return;  /* write 1 to clear */
        case MMR_ST0:  s->st0 = val;
            /* DISP-ENTRY : trace restauration ST0 entière (POPM ST0/STLM) =
             * chemin NON-LDP qui change DP. C'est ICI que DP devient 0x087. */
            g_last_st0w_pc = s->pc; g_last_st0w_val = val;
            g_last_st0w_op = prog_fetch(s, s->pc); g_last_st0w_xpc = s->xpc;
            g_last_st0w_prev = g_prev_pc;
            st0_ring_rec(s, val, 'p'); /* pop/write ST0 (C-sweep) */
            if (g_orphan_on > 0 && s->pc == 0xf48b) {
                /* POPM ST0 @0xf48b : le slot juste poppé = data[sp-1]. Cherche
                 * son dernier écrivain dans le ring pile. NO-WRITER = slot stale
                 * → SP désaligné (POP sans PUSH apparié, cas b de CC). */
                uint16_t slot = (uint16_t)(s->sp - 1);
                fprintf(stderr, "[c54x] ORPHAN@f48b SP=0x%04x slot=0x%04x val=0x%04x(DP=%03x)",
                        s->sp, slot, val, (unsigned)(val & 0x1FF));
                int found = 0;
                unsigned rn = g_stkw_idx < STKW_RING_N ? g_stkw_idx : STKW_RING_N;
                for (unsigned i = 0; i < rn; i++) {
                    StkwEv *e = &g_stkw_ring[(g_stkw_idx - 1 - i) % STKW_RING_N];
                    if (e->addr == slot) {
                        fprintf(stderr, "  WRITER@%04x op=%04x val=%04x", e->pc, e->op, e->val);
                        found = 1; break;
                    }
                }
                if (!found)
                    fprintf(stderr, "  NO-WRITER → slot STALE → SP désaligné (POP sans PUSH)");
                fprintf(stderr, " insn=%u\n", s->insn_count);
                /* SP-EVENTS ring : les push/pop récents → le return RET-family
                 * déséquilibré (pop delta>0 sans push apparié) qui décale SP. */
                fprintf(stderr, "[c54x]   ORPHAN-SP-RING (anciens→récents, pc:op±delta) :");
                for (int k = 28; k >= 1; k--) {
                    struct sp_evt *e = &g_spring[(g_spring_idx - k) & 63];
                    fprintf(stderr, " %04x:%04x%+d", e->pc, e->op, e->delta);
                }
                fprintf(stderr, "\n");
            }
            return;
        case MMR_ST1:  s->st1 = val; return;
        case MMR_AL:   s->a = (s->a & ~0xFFFF) | val; return;
        case MMR_AH:   s->a = (s->a & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_AG:   s->a = (s->a & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_BL:   s->b = (s->b & ~0xFFFF) | val; return;
        case MMR_BH:   s->b = (s->b & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_BG:   s->b = (s->b & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_T:    s->t = val; return;
        case MMR_TRN:  s->trn = val; return;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            ar_write_track(s, addr - MMR_AR0, val);  /* unified probe AR0..AR7 */
            s->ar[addr - MMR_AR0] = val; return;
        case MMR_SP:
            if (val >= 0x0800 && val < 0x0900) {
                if (calypso_debug_enabled("SP-GUARD")) fprintf(stderr,
                        "[c54x] SP-GUARD: refused MMR_SP write 0x%04x "
                        "(API mailbox); keeping 0x%04x PC=0x%04x\n",
                        val, s->sp, s->pc);
                return;
            }
            sp_abs_track(s, val, 0);  /* site=0 : MMR_SP via STL/STM/STLM */
            s->sp = val;
            return;
        case MMR_BK:
            /* PROBE 2026-06-01 : qui écrit BK ? (BK=0 casse l'adressage circulaire
             * → runaway AR2 0xfa98/0xf17c). Nomme le writer + valeur. À RETIRER. */
            {
                static uint32_t bkw_n = 0;
                if (bkw_n < 40) {
                    fprintf(stderr, "[c54x] BK-WR (MMR) 0x%04x→0x%04x PC=0x%04x op=0x%04x "
                            "%s insn=%u\n", s->bk, val, s->pc, prog_fetch(s, s->pc),
                            (val == 0) ? "<<< BK=0 (casse circular!)" : "", s->insn_count);
                    bkw_n++;
                }
            }
            s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            {
                static unsigned pmst_wr_attempts = 0;
                if (pmst_wr_attempts++ < 100)
                    C54_LOG("PMST WR attempt #%u: val=0x%04x cur=0x%04x PC=0x%04x insn=%u",
                            pmst_wr_attempts, val, s->pmst, s->pc, s->insn_count);
            }
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                {
                    static unsigned pmst_log = 0;
                    if (pmst_log++ < 100)
                        C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u #%u/100",
                                s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count, pmst_log);
                }

                static uint16_t last_dumped_iptr = 0xFFFF;
                static unsigned vecdump_count = 0;
                /* Cap at 8 dumps total — firmware may oscillate between 2-3
                 * IPTR values thousands of times during a session, and each
                 * dump emits 32 fprintf lines. Without cap : 250k+ log lines
                 * = saturates host I/O = bridge stops emitting CLK INDs =
                 * BTS shutdown "No more clock from transceiver". */
                if (new_iptr != last_dumped_iptr && vecdump_count < 8) {
                    vecdump_count++;
                    last_dumped_iptr = new_iptr;
                    uint32_t base = (uint32_t)new_iptr << 7;
                    uint16_t saved_pmst = s->pmst;
                    s->pmst = val;
                    C54_LOG("VECDUMP IPTR=0x%03x base=0x%04x (32 vectors) #%u/8:",
                            new_iptr, (uint16_t)base, vecdump_count);
                    for (int vec = 0; vec < 32; vec++) {
                        uint32_t a = base + vec * 4;
                        uint16_t w0 = prog_read(s, a + 0);
                        uint16_t w1 = prog_read(s, a + 1);
                        uint16_t w2 = prog_read(s, a + 2);
                        uint16_t w3 = prog_read(s, a + 3);
                        fprintf(stderr,
                                "[c54x] vec %2d @ 0x%04x : %04x %04x %04x %04x\n",
                                vec, (uint16_t)a, w0, w1, w2, w3);
                    }
                    s->pmst = saved_pmst;
                }
            }
            s->pmst = val; return;
        case MMR_XPC:
            {
                static int xpc_log = 0;
                if (xpc_log++ < 50)
                    C54_LOG("MMR_XPC WR val=0x%04x (was %d) PC=0x%04x SP=0x%04x insn=%u",
                            val, s->xpc, s->pc, s->sp, s->insn_count);
            }
            s->xpc = val & 3;
            return;
        default: g_pmap_mmr_handled = 0; return;   /* undefined CPU MMR — swallowed */
        }
    }

    /* BSCR (0x0029) — DSP->host HINT doorbell. loader2 pulses bit3 (0xF30 set /
     * 0xF33 clear) to tell the MCU "run-mode loaded, ready"; the MCU's IRQ4 mailbox
     * handler (0x2BCEF0) then delivers the run-mode go-command [0x872]. Count the
     * rising edge of bit3 at the write (the pulse is too narrow to poll at RATIO>1)
     * so the host side can latch IRQ4. Non-bit3 BSCR uses fall through to the store. */
    if (addr == 0x0029) {
        if ((val & 0x0008u) && !(s->data[0x0029] & 0x0008u)) g_c54x_bscr_hint_edges++;
        s->data[0x0029] = val;
        g_pmap_mmr_handled = 1;
        return;
    }

    /* MFI (COBBA parallel control) — mmr(22h) primary / mmr(32h) INT-masked
     * variant; both carry the 0xC0xx 4-bit-addr + 12-bit-data COBBA control
     * frame (docs/research/cobba-parallel-interface.md §2). Notify the host
     * model and keep the RAM-backed store so a readback still works. */
    if (addr == 0x0022 || addr == 0x0032) {
        if (g_c54x_mfi_write_cb) g_c54x_mfi_write_cb(addr, val);
        s->data[addr] = val;
        g_pmap_mmr_handled = 1;
        return;
    }

    /* DMA sub-register bank (C54x DMA controller).
     * DMSA (0x0054): sets the sub-register address.
     * DMSDI (0x0055): writes sub-register data, auto-increments DMSA.
     * DMSDN (0x0057): writes sub-register data, no auto-increment.
     * DMA channel 0 sub-registers (BSP receive DMA):
     *   sub 0x00=DMSRC0, 0x01=DMDST0, 0x02=DMCTR0, 0x03=DMMCR0 */
    if (addr == 0x0054) {
        s->dma_subaddr = val;
        s->data[0x0054] = val;
        g_pmap_mmr_handled = 1;
        return;
    }
    if (addr == 0x0055 || addr == 0x0057) {
        uint16_t sa = s->dma_subaddr;
        if (sa < 24) {  /* 6 channels × 4 regs */
            s->dma_subregs[sa] = val;
            int ch = sa / 4;
            int reg = sa % 4;
            static const char *rnames[] = {"SRC","DST","CTR","MCR"};
            C54_LOG("DMA ch%d %s = 0x%04x (sub 0x%02x) PC=0x%04x",
                    ch, rnames[reg], val, sa, s->pc);
        }
        s->data[addr] = val;
        if (addr == 0x0055) s->dma_subaddr++;  /* auto-increment */
        g_pmap_mmr_handled = 1;
        return;
    }

    /* McBSP sub-register bank (serial port extended config).
     * SPSA (0x0038): sub-address. SPSD (0x0039): sub-data. */
    if (addr == 0x0038 || addr == 0x0039) {
        if (addr == 0x0038) s->spsa = val;
        else {
            C54_LOG("McBSP sub[0x%02x] = 0x%04x PC=0x%04x", s->spsa, val, s->pc);
        }
        s->data[addr] = val;
        g_pmap_mmr_handled = 1;
        return;
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        uint16_t woff = addr - C54X_API_BASE;
        if (s->api_ram) {
            s->api_ram[woff] = val;
            if (woff < 0x800u) s->win_dirty = 1;   /* PERF: DSP wrote a signal-window cell (cosim monitor gate) */
        }
        /* CALYPSO_RINGWR=1: trace writes into the MDIRCV ring window (DSP 0x880-0x8E5
         * = woff 0x80-0xE5: payload 0x80-0xE3, tail ptr 0xE4, head ptr 0xE5). Logs PC +
         * the address registers + the instruction word, so we can see WHICH AR (and which
         * routine) drives each write to the ring/pointer cells — settling whether PC 0xe348
         * (an FIR task whose AR7 statically points at 0x1214) genuinely targets the ring
         * pointer 0x8E4, vs a co-sim aliasing/scheduling artifact. Read-only diagnostic. */
        /* RINGWR=1: pointer cells only (0xE4 tail / 0xE5 head) — catches ring INIT +
         * real posts across a full boot without the BIST payload sweep eating the budget.
         * RINGWR=2: the whole window (payload too). */
        const char *ringwr_env = getenv("CALYPSO_RINGWR");
        if (ringwr_env && (woff == 0xE4 || woff == 0xE5
                           || (woff <= 0x00E3 && strcmp(ringwr_env, "2") == 0))) {
            static unsigned rw_log; const unsigned RW_LIMIT = 120;
            if (rw_log < RW_LIMIT) {
                const char *tag = (woff == 0xE4) ? "TAIL" : (woff == 0xE5) ? "HEAD"
                                : (woff >= 0x80) ? "PAYLOAD" : "below";
                fprintf(stderr,
                        "[c54x] RINGWR #%u %s woff=0x%04x(dsp 0x%04x) val=0x%04x PC=0x%04x "
                        "AR0=%04x AR2=%04x AR3=%04x AR4=%04x AR6=%04x AR7=%04x op=%04x insn=%u\n",
                        rw_log, tag, woff, addr, val, s->pc, s->ar[0], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[6], s->ar[7], s->prog[s->pc], s->insn_count);
                if (++rw_log == RW_LIMIT) fprintf(stderr, "[c54x] RINGWR log capped at %u\n", RW_LIMIT);
            }
        }
        /* === DSP→ARM STATUS / DEMOD probe (CCCH chain tracing) ===
         * Track DSP writes to the four critical mailbox regions :
         *   (1) a_pm[3]  + a_serv_demod[4]  on read page 0  : woff 0x30..0x36
         *   (2) a_pm[3]  + a_serv_demod[4]  on read page 1  : woff 0x44..0x4A
         *   (3) a_cd[15] CCCH demod result (CLAUDE.md DWARF) : woff 0x1C2..0x1D0
         *   (4) a_cd[15] CCCH demod result (shunt DWARF v2)  : woff 0x1D2..0x1E0
         * If a_serv_demod_WP* never appears → DSP never advances past
         * cell-search. If a_cd ranges stay silent → CCCH demod never
         * publishes (= bridge GMSK not converging, or task-md chain
         * never reaches CCCH). */
        {
            bool in_serv = (woff >= 0x0030 && woff <= 0x0036)
                        || (woff >= 0x0044 && woff <= 0x004A);
            bool in_acd  = (woff >= 0x01C2 && woff <= 0x01E0);
            if (in_serv || in_acd) {
                static unsigned cd_log;
                const unsigned LIMIT = 400;
                if (cd_log < LIMIT) {
                    const char *tag;
                    if (woff >= 0x0030 && woff <= 0x0032) tag = "A_PM_WP0";
                    else if (woff >= 0x0033 && woff <= 0x0036) tag = "A_SERV_DEMOD_WP0";
                    else if (woff >= 0x0044 && woff <= 0x0046) tag = "A_PM_WP1";
                    else if (woff >= 0x0047 && woff <= 0x004A) tag = "A_SERV_DEMOD_WP1";
                    else tag = "A_CD";
                    fprintf(stderr,
                            "[c54x] DSP-API-WR #%u %s woff=0x%04x val=0x%04x "
                            "PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                            cd_log, tag, woff, val, s->pc,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5],
                            s->insn_count);
                    cd_log++;
                    if (cd_log == LIMIT) {
                        fprintf(stderr,
                                "[c54x] DSP-API-WR log capped at %u\n", LIMIT);
                    }
                }
            }
        }
        /* Notify the ARM-side mailbox watcher (calypso_trx) so it can
         * pulse IRQ_API, mirror to dsp_ram, and run the d_fb_det hook.
         * Without this, DSP writes to NDB cells are invisible to ARM. */
        if (s->api_write_cb)
            s->api_write_cb(s->api_write_cb_opaque, woff, val);
        /* Stack-corruption watch: stack push landing in the NDB
         * mailbox region [0x0800..0x08FF]. Only fires when SP has
         * already been corrupted into that range. */
        if (addr == s->sp && addr >= 0x0800 && addr < 0x0900) {
            if (calypso_debug_enabled("STACK-IN-NDB")) fprintf(stderr,
                    "[c54x] STACK-IN-NDB addr=0x%04x val=0x%04x SP=0x%04x "
                    "PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x\n",
                    addr, val, s->sp, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* Always log writes to d_dsp_page (0x08D4) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_spcx_rif (NDB word 2 = DSP data 0x08D6) — BSP serial port config */
        if (addr == 0x08D6) {
            C54_LOG("DSP WR d_spcx_rif = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_fb_det (NDB word 36 = DSP data 0x08F8). The DSP correlator
         * output here is treated as Q15-signed by the firmware FB-det
         * path — small unsigned BSIC was a wrong assumption. Log every
         * write unconditionally (thinned past 200) and dump the
         * adjacent NDB cells [0x08F0..0x0900] so we can see correlator
         * + flag + a_sync_demod fields together. */
        /* Silent NDB cells watch — d_fb_mode (binary "FB matched" flag,
         * THE actual trigger ARM tests), a_sync_PM (power), a_sync_SNR
         * (SNR). All read as 0 by ARM during 200M run despite d_fb_det
         * varying. Confirms: DSP never declares valid detection.
         * Three discriminating outcomes:
         *   (α) never written → "FB confirmed" code path unreached
         *   (β) written =0 explicitly → DSP scans, never matches threshold
         *   (γ) written !=0 but ARM reads 0 → coherence bug */
        /* W1C latch system removed 2026-05-28 (FBSB host-side synth purge).
         * The only env-gated override on the a_sync_demod read path is now
         * CALYPSO_FORCE_ANGLE_ZERO (calypso_trx.c). DSP writes pass
         * straight through to s->data[] and ARM reads them direct. */
        /* Full a_sync_demod + d_fb_mode WR watch — every cell, no PC
         * filter (so we catch real-fb-det writes AND stomp candidates).
         * Stomp zone PC=0x06xx tagged for easy grep. */
        if (addr == 0x08F9 || addr == 0x08FA ||
            addr == 0x08FB || addr == 0x08FC || addr == 0x08FD) {
            static unsigned ts_log[5] = {0};
            static uint16_t prev_d_fb_mode = 0xFFFF;
            int idx = (addr == 0x08F9) ? 0 :
                      (addr == 0x08FA) ? 1 :
                      (addr == 0x08FB) ? 2 :
                      (addr == 0x08FC) ? 3 : 4;
            const char *name = (idx == 0) ? "d_fb_mode"  :
                               (idx == 1) ? "a_sync_TOA" :
                               (idx == 2) ? "a_sync_PM"  :
                               (idx == 3) ? "a_sync_ANG" : "a_sync_SNR";
            ts_log[idx]++;
            bool transition = (idx == 0) &&
                              (prev_d_fb_mode != 0xFFFF) &&
                              (prev_d_fb_mode != val) &&
                              (val != 0 || prev_d_fb_mode != 0);
            bool stomp_zone = (s->pc >= 0x0600 && s->pc < 0x0700);
            bool log_it = transition ||
                          (idx == 0 && val != 0) ||
                          (val != 0 && ts_log[idx] <= 50) ||
                          (ts_log[idx] % 1000) == 0;
            if (log_it) {
                C54_LOG("DSP WR %s = 0x%04x (s=%d) PC=0x%04x%s insn=%u #%u%s",
                        name, val, (int)(int16_t)val, s->pc,
                        stomp_zone ? " [STOMP?]" : "",
                        s->insn_count, ts_log[idx],
                        transition ? " *TRANSITION*" : "");
            }
            if (idx == 0) prev_d_fb_mode = val;
        }
        if (addr == 0x08F8) {
            static unsigned fbd_log = 0;
            /* Filter out stack-stomp at d_fb_det: only PCs known to be
             * actual fb-det correlator stores (0x8d33, 0x8eb9, 0x8f51) get
             * the full per-write log + NDB+DARAM dumps. Other PCs (e.g.
             * 0xb906 push site, 0x7763/0x7764 SP-overflow) get a counted
             * one-line tag so we don't lose visibility on them, but they
             * stop polluting the watch stream. */
            bool real_fbdet = (s->pc == 0x8d33 || s->pc == 0x8eb9 ||
                               s->pc == 0x8f51);
            /* FBDET-DIVERSITY: count distinct values per 1M-insn window.
             * 1 = DSP pegged on stale data. >5 = real scan. Discriminates
             * "BSP delivers fresh I/Q" from "DSP recorrelates same window". */
            if (real_fbdet) {
                static uint16_t recent_vals[8] = {0};
                static unsigned next_window = 1000000;
                static int n_distinct = 0;
                int seen = 0;
                for (int i = 0; i < 8; i++) {
                    if (recent_vals[i] == val) { seen = 1; break; }
                }
                if (!seen) {
                    recent_vals[n_distinct & 7] = val;
                    n_distinct++;
                }
                if (s->insn_count >= next_window) {
                    C54_LOG("FBDET-DIVERSITY window=%uM distinct=%d",
                            next_window / 1000000, n_distinct);
                    n_distinct = 0;
                    for (int i = 0; i < 8; i++) recent_vals[i] = 0;
                    next_window = (s->insn_count / 1000000 + 1) * 1000000;
                }
            }
            if (real_fbdet && (fbd_log < 200 || (fbd_log % 1000) == 0)) {
                C54_LOG("DSP WR d_fb_det = 0x%04x (s=%d) PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                        val, (int)(int16_t)val, s->pc, s->insn_count,
                        s->prog[(uint16_t)(s->pc - 2)],
                        s->prog[(uint16_t)(s->pc - 1)],
                        s->prog[s->pc],
                        s->prog[(uint16_t)(s->pc + 1)]);
                C54_LOG("  NDB[0x08F0..0x0900]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        s->data[0x08F0], s->data[0x08F1], s->data[0x08F2], s->data[0x08F3],
                        s->data[0x08F4], s->data[0x08F5], s->data[0x08F6], s->data[0x08F7],
                        val,             s->data[0x08F9], s->data[0x08FA], s->data[0x08FB],
                        s->data[0x08FC], s->data[0x08FD], s->data[0x08FE], s->data[0x08FF],
                        s->data[0x0900]);
                if (fbd_log < 5) {
                    C54_LOG("  DARAM[0x3FB0..0x3FBF]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->data[0x3FB0], s->data[0x3FB1], s->data[0x3FB2], s->data[0x3FB3],
                            s->data[0x3FB4], s->data[0x3FB5], s->data[0x3FB6], s->data[0x3FB7],
                            s->data[0x3FB8], s->data[0x3FB9], s->data[0x3FBA], s->data[0x3FBB],
                            s->data[0x3FBC], s->data[0x3FBD], s->data[0x3FBE], s->data[0x3FBF]);
                }
            } else if (!real_fbdet) {
                static unsigned other_pc_count = 0;
                other_pc_count++;
                if (other_pc_count == 1 || other_pc_count == 100 ||
                    other_pc_count == 10000 || other_pc_count == 1000000) {
                    C54_LOG("d_fb_det NON-FBDET-PC write #%u val=0x%04x PC=0x%04x SP=0x%04x",
                            other_pc_count, val, s->pc, s->sp);
                }
            }
            /* === D_FB_DET ZERO-OVERRIDE TRACE ===
             * Race-window observed (memory `project_fbdet_threshold_blocker`):
             * DSP writes high SNR (e.g. 0x7902, 0x7766) at fb-det PCs, then
             * SOMETHING zeroes d_fb_det before ARM reads. ARM sees 200×
             * 0x0000 → no FB found → endless L1CTL_FBSB_REQ retries.
             *
             * Capture EVERY write of val=0 to 0x08F8 with full context so
             * we identify the zero-ifying PCs and reconstruct the condition
             * (threshold check, post-correlation reset, error path, etc.).
             * Cap 200 events. */
            if (val == 0) {
                static unsigned zero_log = 0;
                if (zero_log < 200) {
                    C54_DBG("FBDET", "D_FB_DET ZERO-WR #%u PC=0x%04x op=0x%04x prev=0x%04x "
                            "A=%010llx B=%010llx T=0x%04x ST0=0x%04x ST1=0x%04x insn=%u",
                            zero_log + 1,
                            s->pc, s->prog[s->pc],
                            s->data[0x08F8],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                            s->t, s->st0, s->st1, s->insn_count);
                    zero_log++;
                }
            }
            /* Transition trace : non-zero → zero (the override moment).
             * Logs whenever d_fb_det was non-zero just before this write
             * but the new write makes it zero. Cap 100. */
            if (val == 0 && s->data[0x08F8] != 0) {
                static unsigned override_log = 0;
                if (override_log < 100) {
                    C54_DBG("FBDET", "D_FB_DET OVERRIDE #%u prev=0x%04x → 0 PC=0x%04x op=0x%04x "
                            "A=%010llx ST0=0x%04x insn=%u",
                            override_log + 1,
                            s->data[0x08F8], s->pc, s->prog[s->pc],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            s->st0, s->insn_count);
                    override_log++;
                }
            }
            /* === NEW 2026-05-15 : SET non-zero trace + SET→CLEAR delta ===
             *
             * Symétrique au ZERO-WR. Capture chaque write de val != 0 à 0x08F8
             * pour identifier QUI set le FB found, et combien de cycles ça
             * tient avant qu'un PC clear ne l'écrase.
             *
             * Si delta SET→CLEAR < 100 insn → bug opcode tape immédiatement
             * (style POPM fix). Si delta = milliers d'insn → race timing
             * légitime entre DSP set et ARM read.
             */
            {
                static uint64_t last_set_insn;
                static uint16_t last_set_val;
                static uint16_t last_set_pc;
                static unsigned set_log_n = 0;
                static unsigned delta_log_n = 0;
                if (val != 0) {
                    /* SET event */
                    if (set_log_n < 500) {
                        C54_DBG("FBDET", "D_FB_DET SET #%u val=0x%04x PC=0x%04x op=0x%04x "
                                "prev=0x%04x A=%010llx insn=%u",
                                set_log_n + 1,
                                val, s->pc, s->prog[s->pc],
                                s->data[0x08F8],
                                (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                                s->insn_count);
                        set_log_n++;
                    }
                    last_set_insn = s->insn_count;
                    last_set_val = val;
                    last_set_pc = s->pc;
                } else if (s->data[0x08F8] != 0 && last_set_insn != 0) {
                    /* CLEAR after non-zero — log delta */
                    uint64_t delta = (uint64_t)s->insn_count - last_set_insn;
                    if (delta_log_n < 100) {
                        C54_LOG("D_FB_DET SET-TO-CLEAR-DELTA #%u "
                                "set_PC=0x%04x set_insn=%llu set_val=0x%04x "
                                "clear_PC=0x%04x clear_insn=%u delta=%llu cycles",
                                delta_log_n + 1,
                                last_set_pc, (unsigned long long)last_set_insn,
                                last_set_val, s->pc, s->insn_count,
                                (unsigned long long)delta);
                        delta_log_n++;
                    }
                }
            }
            fbd_log++;
        }
    }

    /* Log DARAM writes to code target area and count total */
    if (addr >= 0x0020 && addr < 0x0800) {
        static int dw_total = 0;
        dw_total++;
        if (addr >= 0x1200 && addr <= 0x1240) {
            C54_LOG("DARAM WR [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                    addr, val, s->pc, s->insn_count);
        }
        if (dw_total == 1 || dw_total == 100 || dw_total == 1000 || dw_total == 10000)
            C54_LOG("DARAM write count: %d (last: [0x%04x]=0x%04x)", dw_total, addr, val);
    }

    /* PROBE 2026-05-31 frame-IT : qui écrit les flags polled par le DSP wedgé
     * (data[0x006e], data[0x585f]) ? Tranche (a) ISR-relocate vs (b) HW-write.
     * Si AUCUN write ou valeur jamais "attendue" → flag jamais posé = deadlock.
     * À RETIRER après diag. */
    if (addr == 0x006e || addr == 0x585f || addr == 0x8a44) {
        static uint32_t fw_n = 0;
        if (fw_n < 80) {
            fprintf(stderr, "[c54x] FLAGWR data[0x%04x] 0x%04x→0x%04x PC=0x%04x "
                    "INTM=%d insn=%u\n", addr, s->data[addr], val, s->pc,
                    !!(s->st1 & ST1_INTM), s->insn_count);
            fw_n++;
        }
    }

    /* PROBE 2026-05-31 : qui écrit d_fb_mode (0x08f9) avec du GARBAGE (>1) ?
     * Le détecteur tourne mais d_fb_mode=0x435b (≠0/1) → fenêtre/scaling faux.
     * Hypothèse : runaway AR2/BK=0 le corrompt. Nomme le corrupteur. À RETIRER. */
    if (addr == 0x08f9 && val > 1) {   /* GARBAGE uniquement (val ≠ 0/1) */
        static uint32_t fm_n = 0;
        if (fm_n < 40) {
            fprintf(stderr, "[c54x] FBMODE-GARBAGE data[0x08f9] 0x%04x→0x%04x PC=0x%04x "
                    "op=0x%04x SP=0x%04x AR2=%04x AR3=%04x BK=%04x %s insn=%u\n",
                    s->data[0x08f9], val, s->pc, prog_fetch(s, s->pc),
                    s->sp, s->ar[2], s->ar[3], s->bk,
                    (s->sp == 0x08f9) ? "<<< SP-PUSH" :
                    (s->ar[2] == 0x08f9 || s->ar[2] == 0x08f8) ? "<<< AR2-STORE" : "?",
                    s->insn_count);
            fm_n++;
        }
    }

    s->data[addr] = val;
}

/* 23-bit program address translation : honors XPC for ≥0x8000 (extended
 * program memory / banked area), passes through for <0x8000 (common bank 0).
 * Shared by prog_fetch and prog_read so they cannot diverge again.
 * OVLY (DARAM mirror) is handled at call site because it routes to s->data[]. */
static inline uint32_t c54x_prog_xlate(const C54xState *s, uint16_t addr16)
{
    /* FIX 2026-06-02 (ROOT CAUSE #2 — runaway 0xee00) : seule la fenêtre
     * 0x8000-0xDFFF est bankée par XPC (overlay PROM0/2/3). 0xE000-0xFFFF est
     * de la ROM FIXE (PROM1, mirror à prog[0xE000+]) — NON bankée sur le
     * silicium. L'ancien `>= 0x8000` appliquait XPC à 0xE000+ aussi → quand
     * XPC=3, PC=0xee00 fetchait prog[0x3ee00] (au-delà des 8K de PROM3) = vide
     * → exécution de PROM nulle (op=0x0000) → runaway de l'étage FB
     * post-corrélateur. La ROM haute doit ignorer XPC. */
    if (addr16 >= 0x8000 && addr16 < 0xE000) {
        return (((uint32_t)s->xpc << 16) | addr16) & (C54X_PROG_SIZE - 1);
    }
    return addr16;   /* 0x0000-0x7FFF on-chip + 0xE000-0xFFFF PROM1 ROM (XPC-indépendant) */
}

/* Single-store DARAM. On the C54x, on-chip DARAM is ONE physical dual-access memory; with
 * PMST.OVLY=1 it is mapped into BOTH program and data space at the same address, and the HPI
 * RAM is a slice of it the host shares (SPRU131G §3/§8). So api_ram[] IS that DARAM over
 * [C54X_API_BASE,+SIZE): the DSP's *data* access, an OVLY *program* fetch (callers gate on the
 * LIVE PMST.OVLY bit), and the MCU HPI window all resolve to the SAME cell. Unconditional — no
 * ovly_unified knob; the old data[]/api_ram[] split (program→data[], data→api_ram[]) was the
 * "uploaded code never lands" bug. Returns the cell, or NULL (no api_ram, e.g. a decode-only
 * core) to fall back to data[]. */
static inline uint16_t *ovly_cell(C54xState *s, uint16_t a)
{
    if (s->api_ram &&
        a >= C54X_API_BASE && a < C54X_API_BASE + C54X_API_SIZE)
        return &s->api_ram[a - C54X_API_BASE];
    return NULL;
}

static uint16_t prog_fetch(C54xState *s, uint16_t pc)
{
    if ((s->pmst & PMST_OVLY) && pc >= 0x80 && pc < 0x2800) {
        uint16_t *c = ovly_cell(s, pc);
        return c ? *c : s->data[pc];
    }
    return s->prog[c54x_prog_xlate(s, pc)];
}

static uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800) {
        uint16_t *c = ovly_cell(s, addr16);
        return c ? *c : s->data[addr16];
    }
    return s->prog[c54x_prog_xlate(s, addr16)];
}

static void __attribute__((unused)) prog_write(C54xState *s, uint32_t addr, uint16_t val)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* PROM1 (0xE000-0xFFFF) is ROM — reject writes */
    if (addr16 >= 0xE000) return;
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800) {
        uint16_t *c = ovly_cell(s, addr16);
        if (c) *c = val; else s->data[addr16] = val;
    }
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        s->prog[ext] = val;
    }
    s->prog[addr16] = val;
}

/* ================================================================
 * Addressing mode helpers
 * ================================================================ */

/* MOD-MISMATCH probe helper (2026-06-01) : adressage circulaire canonique
 * C54x (SPRU172/tic54x-dis.c). step ±1 ou ±AR0, |step| <= BK attendu.
 * BK=0 → linéaire (règle #6396 : STM #0,BK délibéré, on ne wrappe pas).
 * NB : pur calcul de référence pour la sonde — N'ALTÈRE PAS l'exécution. */
static uint16_t c54x_circ_ref(uint16_t ar, int step, uint16_t bk)
{
    if (bk == 0) return (uint16_t)(ar + step);
    /* SPRU131G circular addressing: the buffer base is ARx with its low K bits
     * CLEARED, K = bits needed to hold BK (BK=82 -> K=7, align 128). The old
     * `ar - (ar % bk)` modulo base wrapped a base-aligned buffer to the WRONG
     * region on the first step: MDISND ring ar=0x801 BK=82 gave base 0x7B0/
     * idx 81 -> ar walked to 0x7B0-0x7B2 OUTSIDE the ring -> read zeros ->
     * msg len 0 -> BRC=(0+1)/2-1 wraps to 0xFFFF -> a 64K rptb smears DARAM
     * 0x1200..0x25xx (the CONTACT SERVICE dispatch crash, 2026-06-10).
     * Power-of-2 base = the silicon behavior. */
    int k = 0; while ((1u << k) <= bk) k++;
    uint16_t mask = (uint16_t)((1u << k) - 1);
    uint16_t base = (uint16_t)(ar & ~mask);
    int idx = (int)(ar & mask) + step;
    if (idx >= (int)bk) idx -= bk;
    else if (idx < 0)   idx += bk;
    return (uint16_t)(base + idx);
}

/* Resolve Smem operand: direct or indirect addressing.
 * Returns the data memory address. */
static uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)
{
    if (opcode & 0x80) {
        /* Indirect addressing.
         * Per SPRU131G §5.4.1 Table 5-5: bits 2:0 = ARF select the AR for
         * THIS instruction. ARP (in ST0) is then updated to ARF for the
         * NEXT direct-Smem reference. Earlier this code used arp(s) for
         * cur_arp, which made every indirect insn operate on the
         * PREVIOUS insn's ARF — off-by-one. Symptoms: BANZD *AR1- after
         * STL *AR2+ would decrement AR2 instead of AR1 (BANZD test
         * against AR2 stayed non-zero forever, AR1 frozen). Diagnosed
         * via 5×500M-insn STATE-DUMP showing AR1=0x1c / AR2=0x2b0c
         * frozen across 2B insns at PC=0xa2c2..0xa2ca. */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = nar;
        uint16_t addr = s->ar[cur_arp];
        uint16_t ar_before = s->ar[cur_arp];  /* MOD-MISMATCH probe : base avant post-modify */

        /* PROBE 2026-05-31 convergence modes : 1er usage de chaque AR comme base
         * d'adresse. Si la valeur == reset (AR0=0xff75/0x5aad, AR6=0/0xbae6,
         * AR7=0/0x1e44) → read-before-write → reset load-bearing = driver de la
         * divergence bin/c54x. À RETIRER. */
        {
            static uint8_t ar_used = 0;
            if (!(ar_used & (1 << cur_arp))) {
                ar_used |= (1 << cur_arp);
                fprintf(stderr, "[c54x] AR-FIRSTUSE AR%d=0x%04x PC=0x%04x insn=%u\n",
                        cur_arp, addr, s->pc, s->insn_count);
            }
        }

        /* AR2-FLOOR guard : le pointeur d'écriture corrélateur (AR2) peut
         * sous-déborder le buffer DARAM (0x0800) jusqu'à l'espace MMR
         * (0x1E=XPC, 0x00=IMR) → clobber. WARN-log diag (token AR2-FLOOR) ;
         * DROP expérimental (env CALYPSO_AR2_FLOOR_DROP=1) redirige l'accès
         * vers un scratch pour voir si le corrélateur converge sans le crash. */
        if (cur_arp == 2 && addr < 0x0820) {
            static int ar2_drop = -1;
            if (ar2_drop < 0) {
                const char *e = getenv("CALYPSO_AR2_FLOOR_DROP");
                ar2_drop = (e && *e == '1') ? 1 : 0;  /* env-gated, OFF par défaut */
            }
            if (calypso_debug_enabled("AR2-FLOOR"))
                C54_DBG("AR2-FLOOR",
                    "AR2=0x%04x < floor PC=0x%04x op=0x%04x BK=0x%04x A=%010llx insn=%u",
                    addr, s->pc, prog_fetch(s, s->pc), s->bk,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->insn_count);
            if (ar2_drop && addr < 0x0800)
                addr = 0xFFFF;  /* scratch : empêche le clobber MMR (expérience) */
        }

        /* Post-modify */
        switch (mod) {
        case 0x0: /* *ARn */
            break;
        case 0x1: /* *ARn- */
            s->ar[cur_arp]--;
            break;
        case 0x2: /* *ARn+ */
            s->ar[cur_arp]++;
            break;
        case 0x3: /* *+ARn */
            addr = ++s->ar[cur_arp];
            break;
        /* MOD 4-11 : encodage canonique C54x (tic54x-dis.c:506-518, vérifié
         * cross-run via sonde MOD-MISMATCH 2026-06-01 : QEMU divergeait sur
         * 5/6/9/10/11 — signe inversé 5/6, mauvais op 9/10, wrap absent 11).
         * Ordre réel : 4=-0B 5=-0 6=+0 7=+0B 8=-% 9=-0% 10=+% 11=+0%.
         * Circulaire (8-11) via c54x_circ_ref → BK=0 reste LINÉAIRE (règle
         * #6396 : STM #0,BK délibéré, confirmé par sonde BK-WR). */
        case 0x4: /* *ARn-0B (bit-reversed) — reverse-carry différé, voir GAP */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x5: /* *ARn-0 */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x6: /* *ARn+0 */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x7: /* *ARn+0B (bit-reversed) — reverse-carry différé, voir GAP */
            s->ar[cur_arp] += s->ar[0];
            break;
        /* GAP bitrev (4/7) : ±AR0 plat, signe correct, reverse-carry ignoré.
         * OK hors-FFT ; tout chemin FCCH/SCH bit-reverse mal-adresserait en
         * silence. À implémenter si une sonde le montre exercé en bitrev. */
        case 0x8: /* *ARn-% (circular -1) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], -1, s->bk);
            break;
        case 0x9: /* *ARn-0% (circular -AR0) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], -(int16_t)s->ar[0], s->bk);
            break;
        case 0xA: /* *ARn+% (circular +1) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], +1, s->bk);
            break;
        case 0xB: /* *ARn+0% (circular +AR0) */
            s->ar[cur_arp] = c54x_circ_ref(s->ar[cur_arp], +(int16_t)s->ar[0], s->bk);
            break;
        /* Indirect modes 12..15 use a long-immediate operand from the next
         * program word. Encoding per tic54x-dis.c (MOD field = bits 6:3 of
         * the smem byte) and SPRU131G Table 5-9:
         *   12 : *AR(x)(lk)        — addr = AR(x) + lk, NO modify
         *   13 : *+AR(x)(lk)       — premod: AR(x) += lk; addr = AR(x)
         *   14 : *+AR(x)(lk)%      — premod circular: AR(x) = circ(AR(x)+lk)
         *   15 : *(lk)             — ABSOLUTE long address (lk itself)
         *
         * The bootloader at PROM0 0xb429 uses MOD=15 (`LDU *(0x0ffe), A`)
         * to read BL_ADDR_LO. Misdecoding 15 as "AR + lk circular"
         * produced AR0+0x0ffe instead of 0x0ffe — one of the multiple
         * subtle off-by-AR bugs that left A=0 after the load. */
        case 0xC: /* *AR(x)(lk) */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xD: /* *+AR(x)(lk) */
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            addr = s->ar[cur_arp];
            s->lk_used = true;
            break;
        case 0xE: { /* *+AR(x)(lk)% — circular */
            uint16_t lk = prog_fetch(s, s->pc + 1);
            uint16_t v  = s->ar[cur_arp] + lk;
            if (s->bk) {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                if (v >= base + s->bk) v -= s->bk;
            }
            s->ar[cur_arp] = v;
            addr = v;
            s->lk_used = true;
            break;
        }
        case 0xF: /* *(lk) — absolute address */
            addr = prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        }

        /* PROBE 2026-06-01 MOD-MISMATCH : delta silicium-correct EN PARALLÈLE
         * (n'altère PAS l'exécution — pur compare). Confirme sur le flux réel
         * que seuls mods 5/6/9/10/11 divergent ET que la firmware les touche.
         * Réf : tic54x-dis.c:506-518 (MOD canonique), macros tic54x.h:97-98
         * identiques à l'extraction QEMU. À RETIRER après validation du patch. */
        {
            int16_t  a0  = (int16_t)s->ar[0];
            uint16_t bk  = s->bk;
            uint16_t sil;               /* AR attendu côté silicium */
            switch (mod) {
            case 0x0: sil = ar_before;                       break; /* *ar      */
            case 0x1: sil = (uint16_t)(ar_before - 1);       break; /* *ar-     */
            case 0x2: sil = (uint16_t)(ar_before + 1);       break; /* *ar+     */
            case 0x3: sil = (uint16_t)(ar_before + 1);       break; /* *+ar     */
            case 0x4: sil = (uint16_t)(ar_before - a0);      break; /* *ar-0B   */
            case 0x5: sil = (uint16_t)(ar_before - a0);      break; /* *ar-0    */
            case 0x6: sil = (uint16_t)(ar_before + a0);      break; /* *ar+0    */
            case 0x7: sil = (uint16_t)(ar_before + a0);      break; /* *ar+0B   */
            case 0x8: sil = c54x_circ_ref(ar_before, -1,  bk); break; /* *ar-%  */
            case 0x9: sil = c54x_circ_ref(ar_before, -a0, bk); break; /* *ar-0% */
            case 0xA: sil = c54x_circ_ref(ar_before, +1,  bk); break; /* *ar+%  */
            case 0xB: sil = c54x_circ_ref(ar_before, +a0, bk); break; /* *ar+0% */
            default:  sil = s->ar[cur_arp];                  break; /* 12-15 lk : skip */
            }
            /* quels mods la firmware touche (1er hit chacun) */
            static uint16_t mod_seen = 0;
            if (!(mod_seen & (1u << mod))) {
                mod_seen |= (1u << mod);
                fprintf(stderr, "[c54x] MOD-FIRSTHIT mod=%2d AR%d PC=0x%04x op=0x%04x insn=%u\n",
                        mod, cur_arp, s->pc, opcode, s->insn_count);
            }
            /* divergence silicium vs QEMU (modes 0..11 seulement) */
            if (mod <= 0xB && sil != s->ar[cur_arp]) {
                static uint32_t mm_n[16] = {0};
                if (mm_n[mod] < 8)
                    fprintf(stderr, "[c54x] MOD-MISMATCH mod=%2d AR%d ar0=0x%04x bk=0x%04x "
                            "base=0x%04x qemu=0x%04x silicon=0x%04x PC=0x%04x op=0x%04x insn=%u\n",
                            mod, cur_arp, (uint16_t)a0, bk, ar_before,
                            s->ar[cur_arp], sil, s->pc, opcode, s->insn_count);
                mm_n[mod]++;
            }
        }

        /* Update ARP */
        s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);

        return addr;
    } else {
        /* Direct addressing: DP:offset */
        *indirect = false;
        uint16_t offset = opcode & 0x7F;
        return (dp(s) << 7) | offset;
    }
}

/* SP ledger for IRQ-asymmetry diag (web 2026-05-23).
 * Pushes/pops counted by SP delta sign in dispatch loop (c54x_run).
 * IRQ entries counted explicitly in c54x_interrupt_ex with word count.
 * Periodic dump in dispatch loop shows whether net_words ≈ 0 (balanced)
 * or drifts (indicates push/pop word-count asymmetry, e.g. IRQ entry
 * pushes 1 word but FRET pops 2 → drift -1/IRQ-cycle → SP wraps). */
static struct {
    uint64_t sp_pushes;        /* SP delta < 0 events */
    uint64_t sp_pops;          /* SP delta > 0 events */
    int64_t  net_words;        /* total words pushed - total words popped */
    uint64_t irq_entries;      /* count of c54x_interrupt_ex actual dispatches */
    uint64_t irq_words_pushed; /* words written by IRQ entry path (1 or 2 per APTS) */
    uint64_t last_dump_insn;
} g_sp_ledger;

/* Xmem operand decode per binutils tic54x.h (XMEM/XMOD/XARX macros) :
 *   XMEM(OP) = bits [7:4] of opcode (the Xmem 4-bit nibble)
 *   XMOD    = nibble bits [3:2] : 0=*AR, 1=*AR-, 2=*AR+, 3=*AR+0%
 *   XARX    = nibble bits [1:0] + 2 (= AR2..AR5 only, no AR0/AR1/AR6/AR7)
 *
 * Xmem is INDIRECT-ONLY (no DP-relative direct mode, unlike Smem). Using
 * resolve_smem on an Xmem operand mis-decodes the low byte as Smem direct
 * addressing whenever bit 7 is clear, which lands writes in MMR space
 * (0x00-0x1F) — empirically observed at PC=0x8a46 op=0x9918 (STL B,*AR2)
 * 2026-05-23, stomp SP=0x4800→0x0000 cascading to IMR=0 → DSP idle forever.
 *
 * Fix 2026-06-01 : xmod=3 (*AR+0%) désormais CIRCULAIRE modulo BK via
 * c54x_circ_ref (BK=0→linéaire, règle #6396). Appliqué à tous les handlers
 * duaux (resolve_xmem, MVDD, MAC D0-D9, MASA DB, SQDST DC) — était linéaire
 * `addr + AR0` → drift 16-bit (runaway AR2 @0xfa98, op 0xd3dc Ymem *AR2+0%).
 * Cohérent avec le handler ST||LD C8-CB qui wrappait déjà correctement.
 * NB : la convention 1/2 (±) diffère entre handlers (MVDD 1=- 2=+ vs MAC
 * 1=+ 2=-) — incohérence séparée NON traitée ici, à mesurer (sonde). */
static uint16_t resolve_xmem(C54xState *s, uint16_t op)
{
    uint8_t xmem  = (op >> 4) & 0xF;
    int     xar   = (xmem & 0x3) + 2;
    int     xmod  = (xmem & 0xC) >> 2;
    uint16_t addr = s->ar[xar];
    switch (xmod) {
    case 0: break;
    case 1: s->ar[xar] = addr - 1; break;
    case 2: s->ar[xar] = addr + 1; break;
    case 3: s->ar[xar] = c54x_circ_ref(addr, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circulaire modulo BK (BK=0→linéaire) — fix 2026-06-01 */
    }
    return addr;
}

/* ================================================================
 * Instruction execution
 * ================================================================ */

/* Execute one instruction. Returns number of words consumed (1 or 2). */
/* PC ring buffer for pre-IDLE trace */
static uint16_t pc_ring[256];
static int pc_ring_idx = 0;

/* Warm-boot gate for the SPUNDER stack-underflow probe (2026-06-09). Set by
 * dsp54_warm_reset via c54x_mark_warm() at the post-upload release #2 so the
 * probe ignores phase-1 (loader1 CRC gate) and only watches run-mode. */
static int g_c54x_warm_active = 0;
void c54x_mark_warm(int on) {
    g_c54x_warm_active = on;
    if (calypso_debug_enabled("SPUNDER"))
        fprintf(stderr, "[c54x] MARK-WARM(%d) — SPUNDER probe armed\n", on);
}

/* Évalue une condition C54x depuis l'octet bas de l'opcode, per binutils
 * condition_codes[] (opcodes/tic54x-opc.c) : CC1=0x40 (test accu), CCB=0x08
 * (accu B sinon A), test bits[2:0] = EQ=5 NEQ=4 LT=3 LEQ=7 GT=6 GEQ=2 ;
 * AOV=0x70 ANOV=0x60 ; TC=0x30 NTC=0x20 ; C=0x0C NC=0x08 ; UNC=0x00.
 * Identique à l'évaluation du handler RC/RCD (correcte). Remplace l'ancien
 * décode (op>>4)&0xF des handlers CC/CCD qui lisait le MAUVAIS champ (seuls
 * UNC/AEQ justes par coïncidence ; NEQ/LT/LEQ/GT/GEQ/TC/C faux) → mauvais
 * call/no-call dans la power-scan 0xb1xx (CC[TC] f930) → push manquants →
 * over-pop SP → orphelin 0x80fd @0x94f3 → self-CALA 0x70c3 (=28868).
 * cf doc/SP_CATASTROPHE_70c4_SEQUENCE.md, vérifié sonde CC-MISMATCH. */
/* CALYPSO_CONDLOG=1 (+ CALYPSO_CONDLOG_AT=<insn>, CALYPSO_CONDLOG_N=<count>,
 * CALYPSO_CONDLOG_LO/HI=<pc>): windowed trace of every conditional-branch decision
 * — PC, condition code/name, taken?, and the deciding accumulator/flag state. Pins
 * which conditional in a stuck loop never flips (and what state it tests). */
static bool c54x_cond_true_impl(C54xState *s, uint8_t cc);
static const char *c54x_cond_name(uint8_t cc)
{
    if (cc == 0x00) return "UNC";
    if (cc & 0x40) {
        if ((cc & 0x70) == 0x70) return (cc&0x08)?"BOV":"AOV";
        if ((cc & 0x70) == 0x60) return (cc&0x08)?"BNOV":"ANOV";
        const char *b = (cc&0x08)?"B":"A";
        switch (cc & 0x07) { case 5:return b[0]=='B'?"BEQ":"AEQ"; case 4:return b[0]=='B'?"BNEQ":"ANEQ";
            case 3:return b[0]=='B'?"BLT":"ALT"; case 7:return b[0]=='B'?"BLEQ":"ALEQ";
            case 6:return b[0]=='B'?"BGT":"AGT"; case 2:return b[0]=='B'?"BGEQ":"AGEQ"; default:return "A?"; }
    }
    if ((cc & 0x30) == 0x30) return "TC";
    if ((cc & 0x30) == 0x20) return "NTC";
    if ((cc & 0x0C) == 0x0C) return "C";
    if ((cc & 0x0C) == 0x08) return "NC";
    return "?";
}
static bool c54x_cond_true(C54xState *s, uint8_t cc)
{
    {
        static int cl_init=0, cl_on=0, cl_n=300; static unsigned long cl_at=28000000u;
        static int cl_lo=0, cl_hi=0xFFFF; static int cl_left=-1;
        if (!cl_init){ cl_init=1; cl_on=getenv("CALYPSO_CONDLOG")?1:0;
            const char*a=getenv("CALYPSO_CONDLOG_AT"); if(a&&*a)cl_at=strtoul(a,0,0);
            const char*n=getenv("CALYPSO_CONDLOG_N"); if(n&&*n)cl_n=atoi(n);
            const char*lo=getenv("CALYPSO_CONDLOG_LO"); if(lo&&*lo)cl_lo=(int)strtol(lo,0,0);
            const char*hi=getenv("CALYPSO_CONDLOG_HI"); if(hi&&*hi)cl_hi=(int)strtol(hi,0,0); }
        if (cl_on && s->insn_count>=cl_at && cl_left!=0 && s->pc>=cl_lo && s->pc<=cl_hi){
            if (cl_left<0) cl_left=cl_n;
            bool r = (cc==0x00)?true:c54x_cond_true_impl(s,cc);
            int64_t aa=sext40(s->a), bb=sext40(s->b);
            fprintf(stderr,"[c54x] COND PC=0x%04x %s(cc=0x%02x)=%d A=%lld B=%lld TC=%d C=%d OVA=%d insn=%u\n",
                s->pc, c54x_cond_name(cc), cc, r, (long long)aa,(long long)bb,
                !!(s->st0&ST0_TC),!!(s->st0&ST0_C),!!(s->st0&(1<<8)), s->insn_count);
            if(--cl_left==0) fprintf(stderr,"[c54x] CONDLOG done\n");
            return r;
        }
    }
    return c54x_cond_true_impl(s, cc);
}
static bool c54x_cond_true_impl(C54xState *s, uint8_t cc)
{
    if (cc == 0x00) return true;                       /* UNC */
    if (cc & 0x40) {                                   /* CC1 : test accu */
        int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
        bool ov = (cc & 0x08) ? !!(s->st0 & (1 << 9))  /* OVB */
                              : !!(s->st0 & (1 << 8));  /* OVA */
        if ((cc & 0x70) == 0x70) return ov;            /* AOV/BOV  */
        if ((cc & 0x70) == 0x60) return !ov;           /* ANOV/BNOV */
        switch (cc & 0x07) {
        case 0x05: return acc == 0;                    /* EQ  */
        case 0x04: return acc != 0;                    /* NEQ */
        case 0x03: return acc <  0;                    /* LT  */
        case 0x07: return acc <= 0;                    /* LEQ */
        case 0x06: return acc >  0;                    /* GT  */
        case 0x02: return acc >= 0;                    /* GEQ */
        default:   return true;
        }
    }
    if ((cc & 0x30) == 0x30) return !!(s->st0 & ST0_TC);
    if ((cc & 0x30) == 0x20) return  !(s->st0 & ST0_TC);
    if ((cc & 0x0C) == 0x0C) return !!(s->st0 & ST0_C);
    if ((cc & 0x0C) == 0x08) return  !(s->st0 & ST0_C);
    return true;
}

static int c54x_exec_one(C54xState *s)
{
    uint16_t op = prog_fetch(s, s->pc);
    /* F06B=1: trace the 0xF05E table-init routine (IMR-clobber site) — AR5 (dest ptr that
     * drifts to 0), AR2/AR4 (source ptrs), B (the add-b,a increment), at entry + the loop +
     * the clobber store. Pins WHERE AR5 first goes wrong post-warm-boot. */
    {
        static int f6b = -1; static unsigned fn = 0; static uint16_t prev = 0;
        if (f6b < 0) f6b = getenv("F06B") ? 1 : 0;
        /* Catch the branch-IN: PC now in [0xf05e,0xf088] but the previous PC was OUTSIDE that
         * range = the wrong branch target that skips the routine's init (B/AR7 setup). */
        if (f6b && fn < 60 && s->pc >= 0xF05E && s->pc <= 0xF088 &&
            !(prev >= 0xF05E && prev <= 0xF088)) {
            fprintf(stderr, "[c54x] F06B-ENTER from PC=0x%04x -> 0x%04x  AR5=0x%04x B=0x%010llx "
                    "AR7=0x%04x SP=0x%04x ret[SP]=0x%04x insn=%u\n", prev, s->pc, s->ar[5],
                    (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->ar[7], s->sp,
                    s->data[s->sp], s->insn_count);
            fn++;
        }
        prev = s->pc;
    }
    uint16_t op2;
    bool ind;
    uint16_t addr;
    int consumed = 1;
    s->lk_used = false;  /* reset before each instruction */
    s->writer_kind = WK_UNKNOWN;  /* attribution tag for DATA-W-MMR */

    /* === CORR-TRACE (2026-06-02) : trace instruction-par-instruction la boucle
     * MAC du corrélateur FB autour de 0x8576 à l'instant détection. Montre
     * PC/opcode/AR3/A AVANT chaque instr : si AR3 ne bouge pas d'une ligne à
     * l'autre, ou si A n'accumule pas, on a le coupable (post-incr *AR3+ / RPT
     * / MAC). One-shot ~60 instr. CALYPSO_DEBUG=CORR-TRACE. */
    /* DERAIL-EE00 (2026-06-02) : attrape le saut DANS la zone PROM vide 0xee00
     * (op=0x0000) post-fix SACCD. Logge le PC source + opcode + XPC pour
     * trancher runaway firmware (branche fausse) vs bug paging XPC (adresse
     * légitime bankée fetchée page 0). One-shot ~12. */
    if (s->pc >= 0xee00 && s->pc < 0xef00 &&
        !(s->last_exec_pc >= 0xee00 && s->last_exec_pc < 0xef00)) {
        static unsigned dr = 0;
        if (dr < 12) {
            fprintf(stderr, "[c54x] DERAIL-EE00 #%u entré PC=0x%04x DEPUIS last_pc=0x%04x op_src=0x%04x XPC=%u op@pc=0x%04x SP=0x%04x insn=%u\n",
                    dr, s->pc, s->last_exec_pc, prog_fetch(s, s->last_exec_pc),
                    s->xpc & 0xFF, op, s->sp, s->insn_count);
            dr++;
        }
    }

    static int ct_lo = -1, ct_hi = -1;
    if (ct_lo < 0) {
        const char *l = getenv("CALYPSO_CORR_LO"); const char *h = getenv("CALYPSO_CORR_HI");
        ct_lo = l ? (int)strtol(l, NULL, 0) : 0x8560;
        ct_hi = h ? (int)strtol(h, NULL, 0) : 0x8590;
    }
    if (s->insn_count > 60000000u && s->pc >= (uint16_t)ct_lo && s->pc <= (uint16_t)ct_hi
        && calypso_debug_enabled("CORR-TRACE")) {
        static unsigned ct = 0;
        if (ct < 60) {
            int64_t aa = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
            fprintf(stderr, "[c54x] CORR-TRACE #%u PC=0x%04x op=%04x op2=%04x AR3=%04x data[AR3]=%04x A=%lld T=%04x BRC=%u insn=%u\n",
                    ct, s->pc, op, prog_fetch(s, s->pc + 1), s->ar[3], s->data[s->ar[3]],
                    (long long)aa, s->t, s->brc, s->insn_count);
            ct++;
        }
    }

    /* === AR-CLOBBER probe (2026-05-29) ===
     * Track AR1/AR2/AR6/AR7 transitions to 0 — when an AR pointer
     * becomes 0, any subsequent indirect store *ARx will write to
     * data[0x00] = IMR MMR (= clobber). Documented as the 2026-05-25
     * fix reason (cf c54x_reset comment). Capture l'instruction qui
     * a fait la transition (= last_exec_pc + s->prog[last_exec_pc])
     * pour identifier le coupable. Gated CALYPSO_DEBUG=AR_CLOBBER. */
    {
        static uint16_t prev_ar1, prev_ar2, prev_ar6, prev_ar7;
        static bool init_done = false;
        static unsigned clob_log = 0;
        if (!init_done) {
            prev_ar1 = s->ar[1]; prev_ar2 = s->ar[2];
            prev_ar6 = s->ar[6]; prev_ar7 = s->ar[7];
            init_done = true;
        }
        for (int i = 0; i < 4; i++) {
            int idx = (int[]){1, 2, 6, 7}[i];
            uint16_t *prev = (uint16_t*[]){&prev_ar1, &prev_ar2,
                                            &prev_ar6, &prev_ar7}[i];
            if (*prev != 0 && s->ar[idx] == 0) {
                if (calypso_debug_enabled("AR_CLOBBER") && clob_log < 30) {
                    uint16_t culprit_op = prog_fetch(s, s->last_exec_pc);
                    fprintf(stderr,
                            "[c54x] AR-CLOBBER #%u AR%d %04x->0 by "
                            "PC=0x%04x op=0x%04x cur_PC=0x%04x cur_op=0x%04x "
                            "SP=0x%04x insn=%u\n",
                            clob_log, idx, *prev,
                            s->last_exec_pc, culprit_op,
                            s->pc, op, s->sp, s->insn_count);
                    fflush(stderr);
                    clob_log++;
                }
            }
            *prev = s->ar[idx];
        }
    }

    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* DISP-ENTRY (CALYPSO_DEBUG=DISP-ENTRY, c web 2026-05-29) : discriminateur
     * préemption-IT vs clobber. Logge UNIQUEMENT l'entrée dispatcher 0x8341,
     * avec DP/ST0/SP/AR2 + état IT (INTM/IFR/INT3-pending) + contexte de la
     * DERNIÈRE IT servie (vec, Δinsn, PC+DP foreground préemptés) + prédiction
     * du slot LUT qui sera lu à 0x834d = data[(DP<<7)|0x07] → handler vs garbage.
     * DIFF entrées OK (DP=0x124) vs KO (DP≠0x124) : si KO ⟺ IT récente (Δinsn
     * petit, fg_dp=DP-KO) → (b) préemption confirmée, root = INTM/IT. */
    /* ORACLE (border, debug pas fix) : CALYPSO_FORCE_DP=0x124 force le champ DP
     * de ST0 à l'entrée dispatcher 0x8341. Si FB lock + AFC converge → le bit
     * est load-bearing, la chasse au DP périmé est justifiée. Sinon → faute DSP
     * plus profonde DERRIÈRE le dispatcher, et chasser 0x3125 est prématuré. */
    if (s->pc == 0x8341) {
        static int inited = 0, force_dp = -1, force_from = -1;
        if (!inited) {
            inited = 1;
            const char *e = getenv("CALYPSO_FORCE_DP");
            force_dp = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
            const char *ef = getenv("CALYPSO_FORCE_DP_FROM"); /* SCOPÉ : ne force que si DP==FROM */
            force_from = (ef && *ef) ? (int)strtol(ef, NULL, 0) : -1; /* -1 = global (ancien) */
        }
        if (force_dp >= 0) {
            int cur = s->st0 & 0x1FF;
            if (force_from < 0 || cur == force_from)
                s->st0 = (uint16_t)((s->st0 & ~0x1FF) | (force_dp & 0x1FF));
        }
    }
    if (s->pc == 0x8341 && calypso_debug_enabled("DISP-ENTRY")) {
        static unsigned de_n = 0;
        if (de_n++ < 20000) {
            uint16_t lut_ea = (uint16_t)(((s->st0 & 0x1FF) << 7) | 0x07);
            uint16_t lut    = s->data[lut_ea];
            uint64_t d_intr = s->insn_count - g_last_intr_insn;
            fprintf(stderr,
                "[c54x] DISP-ENTRY DP=0x%03x ST0=0x%04x SP=0x%04x AR2=0x%04x "
                "INTM=%d IFR=0x%04x INT3pend=%d  lut[0x%04x]=0x%04x %s  "
                "prevPC=0x%04x  lastLDP{pc=0x%04x val=0x%03x kind=%d}  "
                "lastST0w{pc=0x%04x op=0x%04x xpc=%u val=0x%04x prev=0x%04x}  "
                "lastIT{vec=%d dInsn=%llu fgPC=0x%04x fgDP=0x%03x} insn=%u\n",
                (unsigned)(s->st0 & 0x1FF), s->st0, s->sp, s->ar[2],
                !!(s->st1 & ST1_INTM), s->ifr, !!(s->ifr & (1 << 3)),
                lut_ea, lut, (lut == 0xff72 ? "OK" : "BAD"),
                g_prev_pc, g_last_ldp_pc, g_last_ldp_val, g_last_ldp_kind,
                g_last_st0w_pc, g_last_st0w_op, g_last_st0w_xpc,
                g_last_st0w_val, g_last_st0w_prev,
                g_last_intr_vec, (unsigned long long)d_intr,
                g_last_intr_fg_pc, g_last_intr_fg_dp, s->insn_count);
            if (lut != 0xff72) {   /* dispatcher BAD → dump ring ST0 push/pop (C-sweep) */
                fprintf(stderr, "[c54x] ST0-RING@dispBAD DP=0x%03x SP=0x%04x (anciens→récents) :",
                        (unsigned)(s->st0 & 0x1FF), s->sp);
                unsigned rn = g_st0_ring_idx < ST0_RING_N ? g_st0_ring_idx : ST0_RING_N;
                for (unsigned i = 0; i < rn; i++) {
                    St0Ev *e = &g_st0_ring[(g_st0_ring_idx - rn + i) % ST0_RING_N];
                    fprintf(stderr, " %c@%04x:%04x v=%04x(DP=%03x)SP=%04x",
                            e->kind, e->pc, e->op, e->val,
                            (unsigned)(e->val & 0x1FF), e->sp);
                }
                fprintf(stderr, "\n");
            }
        }
    }

    /* DISP-TRACE (CALYPSO_DEBUG=DISP-TRACE) : trace le dispatcher de tâches
     * 0x8341-0x8353 qui calcule la cible CALAD (0x8353 = CALAD A). Le bug :
     * A_L finit = 0x70c3 (garbage) au lieu d'une entrée de la branch-table
     * 0x8359 (B 0x8365/0x8394/...). On veut A à l'ENTRÉE (0x8341) = l'index
     * pré-chargé par l'appelant (sélecteur de tâche / d_task_md). Si A est
     * déjà garbage à 0x8341 → bug upstream confirmé (dispatcher innocent). */
    if (s->pc >= 0x8341 && s->pc <= 0x8354 && calypso_debug_enabled("DISP-TRACE")) {
        static unsigned disp_n = 0;
        if (disp_n++ < 300) {
            /* À 0x834d (op 0x6f07 = LD Smem<<1,A) : calcule l'EA direct exact
             * (DP<<7)|dma et logge la valeur lue — c'est elle qui devient A.
             * Légit = 0xff86 (→ A_L=0x8261) ; corrompu = 0xf6b7 (→ 0x70c3). */
            uint16_t ea = (uint16_t)(((s->st0 & 0x1FF) << 7) | (op & 0x7F));
            fprintf(stderr,
                "[c54x] DISP-TRACE PC=0x%04x op=0x%04x A=0x%010llx DP=0x%03x EA=0x%04x "
                "data[EA]=0x%04x d[9187]=0x%04x d[9207]=0x%04x AR1=%04x AR5=%04x insn=%u\n",
                s->pc, op, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned)(s->st0 & 0x1FF), ea, s->data[ea],
                s->data[0x9187], s->data[0x9207],
                s->ar[1], s->ar[5], s->insn_count);
        }
    }

    /* Coarse default: any MMR write happening inside this opcode handler
     * gets attributed to the opcode family so we can read the trace. */
    if (hi8 == 0xF3)                    s->writer_kind = WK_OPCODE_F3;
    else if (hi8 >= 0x80 && hi8 <= 0x8F) s->writer_kind = WK_OPCODE_8x;
    else if (hi8 == 0x77)                s->writer_kind = WK_OPCODE_77;
    else if (hi8 == 0x76)                s->writer_kind = WK_OPCODE_76;
    else                                 s->writer_kind = WK_OPCODE_OTHER;

    /* INTM-TRANS probe : log toute transition INTM 0→1.
     * Le SSBX INTM orphelin se cache entre insn=89.83M (last write 0x3dd2)
     * et insn=98.38M (entrée wait permanente). Cap à 200 transitions pour
     * éviter le flood au boot ; capture le PC qui a fait passer INTM à 1
     * et l'adresse de retour stack pour identifier le caller. */
    {
        static int prev_intm = -1;
        static unsigned itrans_total;
        int cur_intm = !!(s->st1 & ST1_INTM);
        if (prev_intm == 0 && cur_intm == 1) {
            itrans_total++;
            if (itrans_total <= 200) {
                uint16_t ret = s->data[s->sp];
                uint16_t ret_p1 = s->data[(uint16_t)(s->sp + 1)];
                if (calypso_debug_enabled("INTM-TRANS")) fprintf(stderr,
                        "[c54x] INTM-TRANS #%u 0->1 PC=0x%04x insn=%u SP=0x%04x "
                        "RET=%04x RET+1=%04x op=0x%04x IMR=0x%04x IFR=0x%04x\n",
                        itrans_total, s->pc, s->insn_count, s->sp,
                        ret, ret_p1, op, s->imr, s->ifr);
            }
        }
        prev_intm = cur_intm;
    }

    /* Detect when DSP enters DARAM code zone (0x0080-0x27FF) from ROM */
    {
        static uint16_t prev_pc = 0;
        static int daram_log = 0;
        if (s->pc >= 0x0080 && s->pc < 0x2800 && prev_pc >= 0x7000 && daram_log < 3) {
            C54_LOG("ROM->DARAM jump: 0x%04x->0x%04x op=0x%04x insn=%u SP=0x%04x XPC=%d",
                    prev_pc, s->pc, op, s->insn_count, s->sp, s->xpc);
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
            daram_log++;
        }
        /* 0x7700 entry tracer: log when PC enters 0x7700 from elsewhere
         * (i.e. prev_pc != 0x76FF, the natural sequential predecessor).
         * Reveals which CALL/B/RET sources land here. PC HIST shows
         * 7700/7701 as the hottest non-loop addresses — find the callers. */
        if (s->pc == 0x7700 && prev_pc != 0x76FF) {
            static uint64_t e7700;
            e7700++;
            if (e7700 <= 30 || (e7700 % 5000) == 0) {
                C54_LOG("ENTER-7700 #%llu from PC=0x%04x A=%010llx B=%010llx SP=0x%04x trail: %04x %04x %04x %04x %04x",
                        (unsigned long long)e7700, prev_pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        pc_ring[(pc_ring_idx-5)&255], pc_ring[(pc_ring_idx-4)&255],
                        pc_ring[(pc_ring_idx-3)&255], pc_ring[(pc_ring_idx-2)&255],
                        pc_ring[(pc_ring_idx-1)&255]);
            }
        }
        /* === ENTER-770c — dispatcher target, post-flag entry ===
         * The PROM0 idle dispatcher at 0xCC62..0xCC6F polls data[0x62];
         * when set, it CALAs to api[0x1f0c]=0x770c. So 0x770c is the
         * runtime task handler entry. If DARAM[0x60..0x70] never gets
         * set, this PC is never reached. Its appearance in the log is
         * therefore the binary signal that the dispatcher gate has
         * unlocked. Log every entry with full AR/SP/INTM context.
         * Cap to avoid log explosion if it ever runs hot. */
        if (s->pc == 0x770c) {
            static uint64_t e770c;
            e770c++;
            if (e770c <= 30 || (e770c % 1000) == 0) {
                C54_LOG("ENTER-770c #%llu from PC=0x%04x SP=0x%04x INTM=%d "
                        "ARs: %04x %04x %04x %04x %04x %04x %04x %04x insn=%u",
                        (unsigned long long)e770c, prev_pc, s->sp,
                        !!(s->st1 & ST1_INTM),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->insn_count);
            }
        }
        /* === MVDD-CASCADE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * PC=0x8e8c op=0xe5ba = MVDD-family — documented cascade writer
         * (`project_dtaskd_corruption_8e8x`) that writes garbage values
         * into NDB cells (random vals at d_fb_det vs legitimate 0x001e).
         * Track AR fields + B accumulator + source address read to find
         * if it's true firmware compute or corrupted indirect addressing. */
        if (s->pc == 0x8e8c) {
            static int probe_mvdd = -1;
            if (probe_mvdd < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_mvdd = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_mvdd) {
                static uint32_t e_mvdd;
                e_mvdd++;
                if (e_mvdd <= 50 || (e_mvdd % 1000) == 0) {
                    fprintf(stderr,
                            "[c54x] MVDD-CASCADE #%u PC=0x8e8c op=0x%04x SP=0x%04x "
                            "A=0x%010llx B=0x%010llx T=0x%04x "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x "
                            "data[AR4]=0x%04x data[AR5]=0x%04x "
                            "trail: %04x %04x %04x %04x %04x %04x\n",
                            e_mvdd, op, s->sp,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->t,
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->data[s->ar[4]], s->data[s->ar[5]],
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === DF92-LOOP probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Compute loop at PC=0xdf92-0xdfa3 = correlator accumulator with
         * 15× unrolled ADD *AR7+. Called via CALL 0xdfb1 from 0xdf90.
         * Probe at first PC=0xdf92 (loop entry) — log AR7, BRC, accumulator,
         * caller (from stack[SP]). If AR7 is corrupted or BRC mis-set, the
         * loop runs forever and blocks task=24 scheduling downstream.
         * Fire only on entries from non-loop-internal predecessors. */
        if (s->pc == 0xdf92 && (prev_pc < 0xdf90 || prev_pc > 0xdfa3)) {
            static int probe_df = -1;
            if (probe_df < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_df = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_df) {
                static uint32_t e_df;
                e_df++;
                if (e_df <= 30) {
                    fprintf(stderr,
                            "[c54x] DF92-LOOP #%u entry from PC=0x%04x prev_op=0x%04x "
                            "SP=0x%04x ret_addr=stk[SP]=0x%04x "
                            "A=0x%010llx B=0x%010llx "
                            "AR7=0x%04x BK=0x%04x BRC=0x%04x  "
                            "trail: %04x %04x %04x %04x %04x %04x\n",
                            e_df, prev_pc, s->prog[prev_pc],
                            s->sp, s->data[s->sp],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->ar[7], s->bk, s->brc,
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === BL-REENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * The DSP bootloader at PC=0xb41c polls data[0x0fff] for cmd code
         * 4 or 2. Legitimate loop entry comes from 0xb427 (BC NTC 0xb41c)
         * or 0xb433 (CALL 0xb41c). Any OTHER entry path indicates the DSP
         * has been routed back into the bootloader by mistake after boot
         * has completed — that's the post-cascade blocker. Logs prev_pc,
         * SP, stack contents, ARs, and a trail to identify the bad caller.
         * Caps: 50 events to avoid log flood. */
        if (s->pc == 0xb41c && prev_pc != 0xb427 && prev_pc != 0xb433) {
            static int probe_bl = -1;
            if (probe_bl < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_bl = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_bl) {
                static uint32_t e_bl;
                e_bl++;
                if (e_bl <= 50) {
                    fprintf(stderr,
                            "[c54x] BL-REENTRY #%u from PC=0x%04x prev_op=0x%04x "
                            "SP=0x%04x stk[SP..+3]= %04x %04x %04x %04x "
                            "data[0x0fff]=0x%04x data[0x0ffe]=0x%04x "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x "
                            "trail: %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            e_bl, prev_pc, s->prog[prev_pc],
                            s->sp,
                            s->data[(uint16_t)(s->sp+0)], s->data[(uint16_t)(s->sp+1)],
                            s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                            s->data[0x0fff], s->data[0x0ffe],
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === SEED-SOURCE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Probe at PC=0xf8de (CALA B → 0x7700) — the SINGLE source event
         * that spawns the entire boot-stub RET-loop cascade (per session
         * 2026-05-24 BOOTSTUB-ENTRY analysis : 1 ENTER-7700 → 435 entries
         * to PC=0x0000). Captures full state BEFORE the CALA fires :
         *   - SP + stack contents (what subsequent POPs will pull)
         *   - A, B (B = jump target)
         *   - AR0..AR7, ST0, ST1
         *   - 10-PC trail (extends visibility upstream of 0xf8de).
         * Goal: identify whether the function containing 0xf8de was itself
         * called with proper push, and what was supposed to be on stack
         * when POPM ST0 + RCD UNC fire at dispatcher 0x7706/0x7707. */
        if (s->pc == 0xf8de) {
            static int probe_seed = -1;
            if (probe_seed < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_seed = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_seed) {
                static uint32_t e_seed;
                e_seed++;
                if (e_seed <= 50) {
                    fprintf(stderr,
                            "[c54x] SEED-SOURCE #%u PC=0xf8de op=0x%04x "
                            "SP=0x%04x  stk[SP..+7]= %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "A=0x%010llx B=0x%010llx "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "ST0=0x%04x ST1=0x%04x  "
                            "trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            e_seed, op, s->sp,
                            s->data[(uint16_t)(s->sp+0)], s->data[(uint16_t)(s->sp+1)],
                            s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                            s->data[(uint16_t)(s->sp+4)], s->data[(uint16_t)(s->sp+5)],
                            s->data[(uint16_t)(s->sp+6)], s->data[(uint16_t)(s->sp+7)],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->st0, s->st1,
                            pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                            pc_ring[(pc_ring_idx-8)&255],  pc_ring[(pc_ring_idx-7)&255],
                            pc_ring[(pc_ring_idx-6)&255],  pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255],  pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255],  pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === BOOTSTUB-ENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Traces every entry to PC=0x0000 (boot stub LDMM SP,B + RET).
         * Boot stub re-entered at runtime is the documented-never-nailed
         * seed of the SP-wrap → AR6=0 → IMR=0 cascade. Captures :
         *   - prev_pc + op@prev_pc  → who jumped to 0x0000
         *   - entry mechanism (RET-family / branch / other)
         *   - B accumulator (becomes SP via LDMM SP,B at 0x0000)
         *   - SP + stk[SP-1] (just-popped value if RET)
         *   - 6-entry PC trail (caller context). */
        if (s->pc == 0x0000) {
            static int probe_bootstub = -1;
            if (probe_bootstub < 0) {
                const char *e = cdbg_env("BOOTSTUB");
                probe_bootstub = (e && e[0] == '1') ? 1 : 0;
                if (probe_bootstub)
                    fprintf(stderr, "[c54x] PROBE-BOOTSTUB enabled\n");
            }
            if (probe_bootstub) {
                static uint32_t e0;
                e0++;
                if (e0 <= 200 || (e0 % 500) == 0) {
                    uint16_t prev_op = s->prog[prev_pc];
                    const char *mech;
                    if (prev_op == 0xFC00)                          mech = "RET";
                    else if (prev_op == 0xF273)                     mech = "RETD";
                    else if (prev_op == 0xF4EB || prev_op == 0xF4E3) mech = "RETE";
                    else if (prev_op == 0xF4E4 || prev_op == 0xF4E5) mech = "FRET";
                    else if ((prev_op & 0xFF00) == 0xF800)          mech = "B/CC";
                    else if ((prev_op & 0xFF00) == 0xF000)          mech = "F0xx";
                    else if (prev_op == 0xF074)                     mech = "CALL";
                    else                                            mech = "OTHER";
                    /* Just-popped slot is at SP-1 after RET (SP was incremented). */
                    uint16_t stk_just_popped = s->data[(uint16_t)(s->sp - 1)];
                    fprintf(stderr,
                            "[c54x] BOOTSTUB-ENTRY #%u prev_PC=0x%04x prev_op=0x%04x "
                            "mech=%s B=0x%010llx B[31:16]=0x%04x SP=0x%04x "
                            "stk[SP-1]=0x%04x trail: %04x %04x %04x %04x %04x %04x\n",
                            e0, prev_pc, prev_op, mech,
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            (unsigned)((s->b >> 16) & 0xFFFF),
                            s->sp, stk_just_popped,
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }
        /* === INT3-VEC-TRACE probe (2026-05-29) ===
         * Trigger à PC=0xFFCC (= INT3 vector entry, IPTR=0x1FF + vec 19*4).
         * Capture les ~32 PCs suivants pour voir le chemin ISR.
         * Objectif : identifier où DSP saute hors path attendu (= soit RSBX
         * INTM dans zone 0xA4D0+, soit retour normal via RETE). Si DSP finit
         * à 0x0000 boot stub → identifier l'opcode/PC qui dérive le saut.
         * Gated par CALYPSO_DEBUG=INT3_VEC ou ALL. */
        {
            static int trace_n = -1;        /* -1 = not active, ≥0 = countdown */
            static uint16_t trace_pcs[64];
            static uint16_t trace_ops[64];
            static int trace_idx = 0;
            static unsigned trace_dumps = 0;
            const unsigned DUMP_LIMIT = 8;   /* max 8 full traces logged */

            if (s->pc == 0xFFCC && trace_n < 0 && trace_dumps < DUMP_LIMIT) {
                trace_n = 32;                /* capture next 32 insns */
                trace_idx = 0;
                if (calypso_debug_enabled("INT3_VEC")) {
                    fprintf(stderr,
                            "[c54x] INT3-VEC-TRACE BEGIN #%u pc=0xFFCC "
                            "ST1=0x%04x INTM=%d IFR=0x%04x IMR=0x%04x SP=0x%04x\n",
                            trace_dumps + 1, s->st1, !!(s->st1 & ST1_INTM),
                            s->ifr, s->imr, s->sp);
                }
            }
            if (trace_n >= 0 && trace_idx < 64) {
                trace_pcs[trace_idx] = s->pc;
                trace_ops[trace_idx] = prog_fetch(s, s->pc);
                trace_idx++;
                trace_n--;
                if (trace_n <= 0) {
                    if (calypso_debug_enabled("INT3_VEC")) {
                        fprintf(stderr,
                                "[c54x] INT3-VEC-TRACE END #%u captured=%d "
                                "final_ST1=0x%04x INTM=%d\n",
                                trace_dumps + 1, trace_idx,
                                s->st1, !!(s->st1 & ST1_INTM));
                        for (int i = 0; i < trace_idx; i++) {
                            fprintf(stderr,
                                    "[c54x] INT3-VEC-TRACE #%u step %02d: "
                                    "PC=0x%04x op=0x%04x\n",
                                    trace_dumps + 1, i,
                                    trace_pcs[i], trace_ops[i]);
                        }
                        fflush(stderr);
                    }
                    trace_dumps++;
                    trace_n = -1;            /* re-arm */
                }
            }
        }

        /* D_FB_DET-WR-SITE probe : à PC=0x8f51 (le PC qui écrit d_fb_det).
         * Snapshot AR0..AR7 + data[AR0/1/2] + BK + A pour identifier la
         * zone DARAM lue par le correlator FB-det au moment de produire
         * sa valeur d'output. Comparer la zone source avec le BSP DMA
         * target (default 0x3fb0..0x3fbf) :
         *   - zone source = BSP target → correlator lit bien les samples
         *   - zone source ≠ BSP target → mismatch source/sink, blocker
         *     structurel : DSP attend les samples ailleurs que là où le
         *     BSP les écrit. Suite : tracer init AR, table coeffs, ou
         *     MAC sur autre buffer. */
        /* COEFFS-TABLE-DUMP : 1× au tout début + à chaque sweep FB-det.
         * Dump data[0x2bc0..0x2bcF] (zone censée contenir les coefficients
         * du correlator selon AR4 observé). 2026-05-14 : capture étendue
         * D_FB_DET-WR-SITE a révélé data[AR4]=0x0000 sur 50 hits → la table
         * de coeffs est VIDE en mémoire. Vérifier ici si elle l'est aussi
         * en boot et si quelqu'un l'écrit jamais. */
        {
            static int coeffs_log_n;
            static uint64_t coeffs_last_insn;
            bool first_call = (coeffs_log_n == 0);
            bool periodic = (s->insn_count - coeffs_last_insn > 1000000);
            bool at_8f51 = (s->pc == 0x8f51);
            if ((first_call || periodic || at_8f51) && coeffs_log_n < 30) {
                coeffs_log_n++;
                coeffs_last_insn = s->insn_count;
                C54_LOG("COEFFS-DUMP #%d insn=%u PC=0x%04x "
                        "data[0x2bc0..0x2bcF]= %04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x",
                        coeffs_log_n, s->insn_count, s->pc,
                        s->data[0x2bc0], s->data[0x2bc1], s->data[0x2bc2], s->data[0x2bc3],
                        s->data[0x2bc4], s->data[0x2bc5], s->data[0x2bc6], s->data[0x2bc7],
                        s->data[0x2bc8], s->data[0x2bc9], s->data[0x2bca], s->data[0x2bcb],
                        s->data[0x2bcc], s->data[0x2bcd], s->data[0x2bce], s->data[0x2bcf]);
            }
        }
        if (s->pc == 0x8f51) {
            /* Cap bumpé 50 → 500 (2026-05-14 night) pour couvrir plusieurs
             * sweeps FB-det au lieu du seul premier. + stats agrégées sur
             * tous les fires (cap n'est que pour le log per-fire). */
            static int dfbwr_n;
            g_fb_det_timing.fb_det_total++;
            uint16_t ar4 = s->ar[4];
            uint16_t dAR4 = s->data[ar4];
            uint16_t ar3 = s->ar[3];
            bool ar4_in_zone = (ar4 >= 0x2bc0 && ar4 <= 0x2bff);
            if (ar4_in_zone) g_fb_det_timing.fb_det_ar4_in_zone++;
            else             g_fb_det_timing.fb_det_ar4_outside++;
            if (dAR4 == 0x0000)      g_fb_det_timing.fb_det_dar4_zero++;
            else if (dAR4 == 0xfffe) g_fb_det_timing.fb_det_dar4_sentinel++;
            else                     g_fb_det_timing.fb_det_dar4_other++;
            /* Sweep boundary detection : AR3 retombe en dessous de la
             * dernière valeur observée → nouveau sweep commence.
             * Log le sweep précédent (count non-zero + A final + insn). */
            uint64_t A_lo = (uint64_t)(s->a & 0xFFFFFFFFFFULL);
            if (ar3 < g_fb_det_timing.last_ar3_at_fire
                && g_fb_det_timing.last_ar3_at_fire > 0) {
                C54_LOG("D_FB_DET-SWEEP id=%llu nonzero=%llu/50 "
                        "A_final=0x%010llx insn=%u",
                        (unsigned long long)g_fb_det_timing.sweep_id,
                        (unsigned long long)g_fb_det_timing.sweep_nonzero_count,
                        (unsigned long long)A_lo, s->insn_count);
                g_fb_det_timing.sweep_id++;
                g_fb_det_timing.sweep_nonzero_count = 0;
            }
            g_fb_det_timing.last_ar3_at_fire = ar3;
            if (dAR4 != 0) g_fb_det_timing.sweep_nonzero_count++;
            int64_t delta_compute = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_compute_insn;
            int64_t delta_clear   = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_clear_insn;
            int64_t delta_pattern = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_pattern_insn;
            if (dfbwr_n++ < 500) {
                C54_LOG("D_FB_DET-WR-SITE #%d AR0..AR7=%04x %04x %04x %04x %04x %04x %04x %04x "
                        "data[AR0]=%04x data[AR1]=%04x data[AR2]=%04x "
                        "data[AR3]=%04x data[AR4]=%04x data[AR5]=%04x "
                        "data[AR6]=%04x data[AR7]=%04x "
                        "BK=%04x A=0x%010llx "
                        "ar4_in_zone=%d dcompute=%lld dclear=%lld dpattern=%lld "
                        "last_compute_addr=0x%04x last_clear_addr=0x%04x "
                        "last_pattern_addr=0x%04x "
                        "insn=%u",
                        dfbwr_n, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->data[s->ar[0]], s->data[s->ar[1]], s->data[s->ar[2]],
                        s->data[s->ar[3]], s->data[s->ar[4]], s->data[s->ar[5]],
                        s->data[s->ar[6]], s->data[s->ar[7]],
                        s->bk, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        ar4_in_zone ? 1 : 0,
                        (long long)delta_compute, (long long)delta_clear,
                        (long long)delta_pattern,
                        g_fb_det_timing.last_compute_addr,
                        g_fb_det_timing.last_clear_addr,
                        g_fb_det_timing.last_pattern_addr,
                        s->insn_count);
            }
            /* Stats summary toutes les 100 fires de 0x8f51 — distribution
             * AR4-in-zone + histogramme val[AR4] sur tout l'historique. */
            if ((g_fb_det_timing.fb_det_total % 100) == 0) {
                C54_LOG("D_FB_DET-STATS total=%llu "
                        "ar4_in_zone=%llu outside=%llu "
                        "dar4_zero=%llu sentinel=%llu other=%llu",
                        (unsigned long long)g_fb_det_timing.fb_det_total,
                        (unsigned long long)g_fb_det_timing.fb_det_ar4_in_zone,
                        (unsigned long long)g_fb_det_timing.fb_det_ar4_outside,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_zero,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_sentinel,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_other);
            }
        }
        /* READ-AMONT probe : à chaque trigger PC (sites d_fb_det), émet delta
         * des reads par plage depuis le trigger précédent. Tranche entre :
         *   - dominant LOW    → correlator lit la zone [0..0x3A3]
         *   - dominant APIRAM → samples viennent via API RAM (ARM-driven)
         *   - dominant WRAP   → correlator tourne sur le wrap PROM1 mirror
         *   - dominant OTHER  → zone non cataloguée à identifier */
        read_stats_trigger_check(s);
        throughput_tick(s->insn_count);
        /* WAIT-A21A probe : à PC=0xa21a, snapshot INTM + IMR + IFR.
         * Tranche H1/H2/H3 :
         *   INTM=1 + IFR=0  + IMR plein → H3 strict, hardware silencieux
         *   INTM=1 + IFR≠0  + IMR plein → H3 + IRQ pending bloquée (BUG)
         *   INTM=0                       → H1/H2 (IRQ servable mais path
         *                                  vers 0x7740 cassé en amont) */
        /* === CORR-PUBLISH-A probe (2026-05-28) ===
         * À PC=0x9ac0 (juste avant STL A → *AR2-), snapshot A complet +
         * AR2 (= adresse de publication). Cap 200. */
        if (s->pc == 0x9ac0) {
            static unsigned cpa_log;
            const unsigned LIMIT = 200;
            if (cpa_log < LIMIT) {
                fprintf(stderr,
                        "[c54x] CORR-PUBLISH-A #%u PC=0x9ac0 "
                        "A=0x%010llx (A_lo=0x%04x A_hi=0x%04x) "
                        "AR2=0x%04x (= *AR2- target) insn=%u\n",
                        cpa_log,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (uint16_t)(s->a & 0xFFFF),
                        (uint16_t)((s->a >> 16) & 0xFFFF),
                        s->ar[2], s->insn_count);
                cpa_log++;
                if (cpa_log == LIMIT) {
                    fprintf(stderr,
                            "[c54x] CORR-PUBLISH-A log capped at %u\n",
                            LIMIT);
                }
            }
        }
        if (s->pc == 0xa21a) {
            static uint64_t a21a_total;
            a21a_total++;
            if (a21a_total <= 5 || (a21a_total % 100000) == 0) {
                C54_LOG("WAIT-A21A #%llu insn=%u INTM=%d IMR=0x%04x IFR=0x%04x "
                        "ST0=0x%04x ST1=0x%04x SP=0x%04x",
                        (unsigned long long)a21a_total, s->insn_count,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->st0, s->st1, s->sp);
            }
        }
        /* CALLER-7740 tracer : à l'entrée 0x7740, log le contexte caller.
         * data[sp] = adresse de retour pushée par le CALL/CALLD précédent.
         * INTM=1 → on est dans un IRQ context. Permet de distinguer
         * "appelé via IRQ ISR" vs "appelé via flow régulier", et de
         * remonter la chaîne caller→callee jusqu'à l'IRQ vector. */
        if (s->pc == 0x7740) {
            static uint64_t enter7740;
            enter7740++;
            uint16_t ret_addr = s->data[s->sp];
            uint16_t ret_addr_p1 = s->data[(uint16_t)(s->sp + 1)];
            C54_LOG("ENTER-7740 #%llu insn=%u SP=%04x RET=%04x RET+1=%04x "
                    "INTM=%d XPC=%02x AR2=%04x AR3=%04x BK=%04x",
                    (unsigned long long)enter7740, s->insn_count,
                    s->sp, ret_addr, ret_addr_p1,
                    !!(s->st1 & ST1_INTM), s->xpc,
                    s->ar[2], s->ar[3], s->bk);
        }
        /* MAC-7700 tracer: at PC=0x7700 (MAC *AR2-, A) we want to know
         * what AR2 points at, what data[AR2] holds, T, and A before/after.
         * Helps determine if AR2 references the BSP RX zone (correlator
         * FB-det) or somewhere else. Also dumps full AR0-AR7 + ST0/ST1. */
        if (s->pc == 0x7700) {
            static uint64_t mac7700_total;
            mac7700_total++;
            if (mac7700_total <= 50 || (mac7700_total % 5000) == 0) {
                uint16_t ar2 = s->ar[2];
                uint16_t v_at_ar2 = s->data[ar2];
                C54_LOG("MAC-7700 #%llu AR2=0x%04x data[AR2]=0x%04x T=0x%04x "
                        "A_pre=%010llx ST0=0x%04x ST1=0x%04x",
                        (unsigned long long)mac7700_total, ar2, v_at_ar2,
                        s->t,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->st0, s->st1);
                C54_LOG("MAC-7700 #%llu ARs: AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x",
                        (unsigned long long)mac7700_total,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp);
            }
        }
        /* RCD-75e8 tracer: when DSP arrives at PC=0x75e8 (cond=0x47 = LEQ),
         * log A. The RCD takes if A <= 0; report whether the loop will
         * exit this iteration. */
        if (s->pc == 0x75e8) {
            static uint64_t rcd75e8_total;
            rcd75e8_total++;
            if (rcd75e8_total <= 50 || (rcd75e8_total % 5000) == 0) {
                int64_t acc = sext40(s->a);
                C54_LOG("RCD-75e8 #%llu A=%010llx (signed=%lld) RCD-taken=%d AR2=%04x",
                        (unsigned long long)rcd75e8_total,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (long long)acc, (acc <= 0), s->ar[2]);
            }
        }
        prev_pc = s->pc;
        /* DARAM 0x1100-0x1130 tracer: dump first 64 visits */
        static int daram1110_log = 0;
        if (s->pc >= 0x1100 && s->pc <= 0x1130 && daram1110_log < 64) {
            C54_LOG("DARAM110x PC=0x%04x op=0x%04x A=%08x B=%08x AR2=%04x AR3=%04x AR4=%04x AR5=%04x BRC=%d",
                    s->pc, op, (uint32_t)s->a, (uint32_t)s->b,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->brc);
            daram1110_log++;
        }
    }
    if (s->pc >= 0xFE00 && s->pc <= 0xFFFF && op == 0x0000) {
        static int nop_slide = 0;
        if (nop_slide == 0) {
            C54_LOG("NOP-SLIDE PC=0x%04x insn=%u SP=0x%04x PMST=0x%04x XPC=%d OVLY=%d",
                    s->pc, s->insn_count, s->sp, s->pmst, s->xpc, !!(s->pmst & PMST_OVLY));
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
        }
        nop_slide++;
    }

    switch (hi4) {
    case 0xF:
        /* 0xF --- large group: branches, misc, short immediates */
        if (op == 0xF495) return consumed + s->lk_used;  /* NOP */

        /* XC n, cond — Execute Conditionally (SPRU172C p.4-198)
         * Opcode: 1111 11N1 CCCCCCCC
         * 0xFDxx = XC 1, cond (N=0, execute next 1 instruction)
         * 0xFFxx = XC 2, cond (N=1, execute next 2 instructions)
         * If condition true: execute normally. If false: skip n instructions. */
        if (hi8 == 0xFD || hi8 == 0xFF) {
            int n_insns = (hi8 == 0xFF) ? 2 : 1;
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition code per SPRU172C condition table */
            /* Conditions can be combined (OR'd bits), but common single conditions: */
            if (cc == 0x00)      cond = true;                          /* UNC */
            else if (cc == 0x0C) cond = (s->st0 & ST0_C) != 0;       /* C */
            else if (cc == 0x08) cond = !(s->st0 & ST0_C);            /* NC */
            else if (cc == 0x30) cond = (s->st0 & ST0_TC) != 0;       /* TC */
            else if (cc == 0x20) cond = !(s->st0 & ST0_TC);           /* NTC */
            else if (cc == 0x45) cond = (sext40(s->a) == 0);          /* AEQ */
            else if (cc == 0x44) cond = (sext40(s->a) != 0);          /* ANEQ */
            else if (cc == 0x46) cond = (sext40(s->a) > 0);           /* AGT */
            else if (cc == 0x42) cond = (sext40(s->a) >= 0);          /* AGEQ */
            else if (cc == 0x43) cond = (sext40(s->a) < 0);           /* ALT */
            else if (cc == 0x47) cond = (sext40(s->a) <= 0);          /* ALEQ */
            else if (cc == 0x4D) cond = (sext40(s->b) == 0);          /* BEQ */
            else if (cc == 0x4C) cond = (sext40(s->b) != 0);          /* BNEQ */
            else if (cc == 0x4E) cond = (sext40(s->b) > 0);           /* BGT */
            else if (cc == 0x4A) cond = (sext40(s->b) >= 0);          /* BGEQ */
            else if (cc == 0x4B) cond = (sext40(s->b) < 0);           /* BLT */
            else if (cc == 0x4F) cond = (sext40(s->b) <= 0);          /* BLEQ */
            else if (cc == 0x70) cond = (s->st0 & ST0_OVA) != 0;     /* AOV */
            else if (cc == 0x60) cond = !(s->st0 & ST0_OVA);          /* ANOV */
            else if (cc == 0x78) cond = (s->st0 & ST0_OVB) != 0;     /* BOV */
            else if (cc == 0x68) cond = !(s->st0 & ST0_OVB);          /* BNOV */
            else {
                /* Combined conditions: OR the individual condition bits */
                cond = false;
                if (cc & 0x0C) cond |= ((cc & 0x04) ? (s->st0 & ST0_C) != 0 : !(s->st0 & ST0_C));
                if (cc & 0x30) cond |= ((cc & 0x10) ? (s->st0 & ST0_TC) != 0 : !(s->st0 & ST0_TC));
                if (cc & 0x40) {
                    int64_t acc = (cc & 0x08) ? s->b : s->a;
                    int c3 = cc & 0x07;
                    switch (c3) {
                    case 0x5: cond |= (sext40(acc) == 0); break;
                    case 0x4: cond |= (sext40(acc) != 0); break;
                    case 0x6: cond |= (sext40(acc) > 0); break;
                    case 0x2: cond |= (sext40(acc) >= 0); break;
                    case 0x3: cond |= (sext40(acc) < 0); break;
                    case 0x7: cond |= (sext40(acc) <= 0); break;
                    default: cond = true; break;
                    }
                }
                if (cc & 0x70 && !(cc & 0x40)) {
                    if (cc & 0x08) cond |= (s->st0 & ST0_OVB) != 0;
                    else           cond |= (s->st0 & ST0_OVA) != 0;
                }
            }
            if (!cond) {
                /* Skip n instructions — count consumed words for skipped insns */
                /* Each skipped insn is 1 word (simplified — multi-word insns rare after XC) */
                return 1 + n_insns;
            }
            return consumed + s->lk_used;  /* condition true: just advance past XC, execute next normally */
        }

        /* F4E2 = RSBX INTM (enable interrupts), F4E3 = SSBX INTM (disable interrupts) */
        /* F4E2 = BACC A, F5E2 = BACC B (per tic54x-opc.c, mask 0xFEFF) */
        /* F4E3 = CALA A, F5E3 = CALA B — push next-PC, jump to acc low 16 bits */
        /* DYN-CALL tracer: targets are computed at runtime, invisible to static
         * disasm. Log every BACC/CALA, plus an extra hot tag when the target
         * lands in any FB-det zone (PROM0 0x77xx-0x79xx, 0x88xx, 0xa0xx-0xa1xx). */
        if (op == 0xF4E2 || op == 0xF5E2 || op == 0xF4E3 || op == 0xF5E3) {
            int is_b = (op & 0x0100) != 0;
            int is_call = (op & 1) != 0;
            uint16_t tgt = (uint16_t)((is_b ? s->b : s->a) & 0xFFFF);
            uint16_t src_pc = s->pc;
            /* SURGICAL 2026-05-30 : self-CALA black-hole capture. Fire UNE
             * fois quand un CALA cible lui-même dans la zone 0x7000-0x70FF
             * (= le trou noir 0x70c3). Donne le DP hérité + le slot LUT lu
             * au dispatcher 0x834d (ea/val) + le POPM ST0 et le LDP qui ont
             * posé ce DP — le coupable complet, sans spam (≠ DISP-TRACE). */
            if (is_call && tgt == src_pc && tgt >= 0x7000 && tgt <= 0x70FF) {
                static int bh_logged = 0;
                if (!bh_logged++) {
                    fprintf(stderr,
                        "[c54x] *** BLACKHOLE-CALA *** tgt=0x%04x DP=0x%03x "
                        "SP=0x%04x prevPC=0x%04x dispLUT{ea=0x%04x val=0x%04x} "
                        "lastST0w{pc=0x%04x val=0x%04x prev=0x%04x} "
                        "lastLDP{pc=0x%04x val=0x%03x} insn=%u\n",
                        tgt, (unsigned)(s->st0 & 0x1FF), s->sp, g_prev_pc,
                        g_disp_lut_ea, g_disp_lut_val,
                        g_last_st0w_pc, g_last_st0w_val, g_last_st0w_prev,
                        g_last_ldp_pc, g_last_ldp_val, s->insn_count);
                    /* Dump pile autour de SP : montre l'orphelin (0xf487) et
                     * ses voisins. Si le vrai ST0 (genre 0x0xxx/0x4xxx, DP
                     * plausible) est à SP±1, c'est un imbalance d'1 mot. Les
                     * mots PC-shaped (0xf4xx/0x7xxx) empilés = drain cumulatif. */
                    fprintf(stderr, "[c54x]     STACK around SP=0x%04x :", s->sp);
                    for (int k = -2; k <= 9; k++) {
                        uint16_t a = (uint16_t)(s->sp + k);
                        fprintf(stderr, " %s[%04x]=%04x",
                                k == 0 ? ">" : "", a, s->data[a]);
                    }
                    fprintf(stderr, "\n");
                    /* SP-event ring : les 28 derniers push/pop (pc:op delta).
                     * Cherche un push (delta<0) sans pop apparié = la fuite. */
                    fprintf(stderr, "[c54x]     SP-EVENTS net_words=%lld pushes=%llu pops=%llu (récents, anciens→récents):\n[c54x]    ",
                            (long long)g_sp_ledger.net_words,
                            (unsigned long long)g_sp_ledger.sp_pushes,
                            (unsigned long long)g_sp_ledger.sp_pops);
                    for (int k = 28; k >= 1; k--) {
                        struct sp_evt *e = &g_spring[(g_spring_idx - k) & 63];
                        fprintf(stderr, " %04x:%04x%+d", e->pc, e->op, e->delta);
                    }
                    fprintf(stderr, "\n");
                    fflush(stderr);
                }
            }
            int fb_zone = (tgt >= 0x7730 && tgt <= 0x7990) ||
                          (tgt >= 0x8800 && tgt <= 0x88FF) ||
                          (tgt >= 0xA000 && tgt <= 0xA1FF);
            static uint64_t dyn_total = 0;
            static uint64_t dyn_fb = 0;
            dyn_total++;
            if (fb_zone) dyn_fb++;
            /* When OVLY=1 and src_pc in [0x80, 0x2800], the executed opcode
             * comes from data[] (DARAM), not prog[]. Reflect this in the
             * dump so we see the *actual* bytes that drove the CALA. */
            int ovly_active = (s->pmst & PMST_OVLY) && src_pc >= 0x80 && src_pc < 0x2800;
            uint16_t m0 = ovly_active ? s->data[(uint16_t)(src_pc - 2)] : s->prog[(uint16_t)(src_pc - 2)];
            uint16_t m1 = ovly_active ? s->data[(uint16_t)(src_pc - 1)] : s->prog[(uint16_t)(src_pc - 1)];
            uint16_t m2 = ovly_active ? s->data[src_pc] : s->prog[src_pc];
            uint16_t m3 = ovly_active ? s->data[(uint16_t)(src_pc + 1)] : s->prog[(uint16_t)(src_pc + 1)];
            if (dyn_total <= 200 || fb_zone || (dyn_total % 5000) == 0) {
                C54_LOG("DYN-CALL #%llu %s%c src=0x%04x tgt=0x%04x A=%010llx B=%010llx SP=0x%04x mem[%c]=%04x %04x %04x %04x%s",
                        (unsigned long long)dyn_total,
                        is_call ? "CALA" : "BACC",
                        is_b ? 'B' : 'A',
                        src_pc, tgt,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        ovly_active ? 'D' : 'P',
                        m0, m1, m2, m3,
                        fb_zone ? " *FB-ZONE*" : "");
            }
            if (is_call) {
                uint16_t ret_pc = src_pc + 1;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, ret_pc);
            }
            s->pc = tgt;
            return 0;
        }
        /* F4E6 = FBACC A   FL_FAR  (far branch acc, no push, no delay)
         * F4E7 = FCALA A   FL_FAR  (far call  acc, push 2 mots, no delay)
         * F5E6 = FBACC B / F5E7 = FCALA B  (acc B variants)
         *
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C :
         *   XPC = A(22:16), PC = A(15:0). FCALA push XPC puis ret_pc (PC+1),
         *   ordre compatible avec FRET (F4E4 — pop PC d'abord puis XPC).
         *
         * 2026-05-27 (c web review): non-delayed variants WERE NOP-fallthrough
         * via the F4E0-F4FF block below. Pre-XPC-fix le code far-acc n'était
         * jamais atteint, post-fix il l'est → silent control-flow derailment.
         * Sémantique identique au FCALAD/FBACCD (F6E6/F6E7) existant, sans
         * delay slots. */
        if (op == 0xF4E6 || op == 0xF4E7 || op == 0xF5E6 || op == 0xF5E7) {
            int is_b    = (op & 0x0100) != 0;
            int is_call = (op & 1) != 0;
            int64_t acc = is_b ? s->b : s->a;
            uint16_t tgt     = (uint16_t)(acc & 0xFFFF);
            uint8_t  new_xpc = (uint8_t)((acc >> 16) & 0xFF);
            if (new_xpc > 3) new_xpc &= 3;
            static uint64_t facc_total;
            facc_total++;
            if (facc_total <= 30 || (facc_total % 5000) == 0) {
                C54_LOG("%s%c FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x "
                        "(A=%010llx SP=0x%04x was XPC=%u)",
                        is_call ? "FCALA" : "FBACC",
                        is_b ? 'B' : 'A',
                        (unsigned long long)facc_total,
                        s->pc, new_xpc, tgt,
                        (unsigned long long)(acc & 0xFFFFFFFFFFULL),
                        s->sp, s->xpc & 0x3);
            }
            if (is_call) {
                /* FCALA : push XPC first (deeper in stack), then ret_pc (top).
                 * FRET (F4E4) pops PC d'abord puis XPC — ordre compatible. */
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                uint16_t ret_pc = (uint16_t)(s->pc + 1);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, ret_pc);
            }
            s->xpc = new_xpc;
            s->pc  = tgt;
            return 0;
        }
        /* F4E0-F4FF catch-all : NOP par défaut, sauf opcodes connus qui ont
         * leur handler dédié. Comment historique "RSBX/SSBX" était faux
         * (RSBX=F4B0, SSBX=F5B0 hors range). En fait ce range contient :
         *   F4E1: IDLE 1            ← exception (handler ligne ~4058)
         *   F4E2: BACC A            (handler ligne ~3920)
         *   F4E3: CALA A            (handler ligne ~3920)
         *   F4E4: FRET              ← exception (handler ligne ~4041)
         *   F4E6: FBACC             (handler ligne ~3974)
         *   F4E7: FCALA             (handler ligne ~3974)
         *   F4EB: RETE              ← exception (handler ligne ~4012)
         * Sans l'exception F4E1, IDLE 1 était silencieusement avalé en NOP,
         * empêchant DSP de signaler s->idle=true → IRQ handler ne dispatchait
         * jamais → INTM stuck à 1 (2026-05-29 fix). */
        if (op >= 0xF4E0 && op <= 0xF4FF &&
            op != 0xF4E1 && op != 0xF4E4 && op != 0xF4EB) {
            return consumed + s->lk_used;
        }
        /* F4EB = RETE (return from interrupt). Pop PC, pop XPC iff APTS=1.
         * Symmetric with c54x_interrupt_ex push order. */
        if (op == 0xF4EB) {
            uint16_t prev_xpc = s->xpc;
            /* IT return : pop XPC (top, poussé en 2e par l'entrée IT) PUIS PC.
             * Inconditionnel (APTS == AVIS). Miroir entrée-IT / FRETED.
             * C542-class (no_xpc): there IS no XPC — the frame is the 16-bit
             * PC alone, and `call ...; rete` is a legal task-level idiom. */
            if (!s->no_xpc) {
                uint16_t nx = data_read(s, s->sp); s->sp++;   /* pop XPC */
                if (nx > 2)
                    C54_DBG("XPC-OOR", "RETE xpc=0x%04x PC=0x%04x SP=0x%04x insn=%u",
                            nx, s->pc, s->sp, s->insn_count);
                s->xpc = nx & 3;
            }
            uint16_t ra = data_read(s, s->sp); s->sp++;   /* pop PC */
            s->st1 &= ~ST1_INTM;
            { static int rl = -1; static int rn = 0;
              if (rl < 0) rl = cdbg_env("IRQPAIR") ? 1 : 0;
              if (rl && rn < 24) { rn++;
                  fprintf(stderr, "[c54x] IRQPAIR RETE@0x%04x pops ra=0x%04x sp 0x%04x->0x%04x insn=%u\n",
                          s->pc, ra, (uint16_t)(s->sp-1), s->sp, s->insn_count); } }
            /* INT3-CYCLE-TRACE end-good hook NOT here : firmware exits ISR via
             * POPM ST1 + RCD (not RETE 0xF4EB), so this path is dead. Hook
             * moved to generic INTM 1→0 detector below — catches all idioms. */
            {
                static uint64_t rete_count;
                rete_count++;
                if (rete_count <= 20 || (rete_count % 100) == 0)
                    C54_LOG("RETE #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)rete_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra; return 0;
        }
        /* 0xF4E4 = FRET (far return). Pop PC + XPC unconditionally.
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C Table 2-15:
         *   FRET[D]: XPC = TOS, ++SP, PC = TOS, ++SP
         * Symmetric with FCALL/FCALLD push (also unconditional, see below).
         * 2026-04-28 — fixed: was conditional on PMST_APTS (bit 4) which is
         * actually AVIS (Address Visibility) per SPRU131G — has no stack
         * semantics. The misnomer caused FRET to skip XPC pop when AVIS=0,
         * leading to stack imbalance against FCALL FAR which always pushes 2. */
        if (op == 0xF4E4) {
            uint16_t ra = data_read(s, s->sp); s->sp++;
            uint16_t prev_xpc = s->xpc;
            uint16_t nx = data_read(s, s->sp); s->sp++;
            if (nx > 2)
                C54_DBG("XPC-OOR", "FRET xpc=0x%04x PC=0x%04x SP=0x%04x insn=%u",
                        nx, s->pc, s->sp, s->insn_count);
            s->xpc = nx & 3;
            {
                static uint64_t fret_count;
                fret_count++;
                if (fret_count <= 30 || (fret_count % 1000) == 0)
                    C54_LOG("FRET #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)fret_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra;
            return 0;
        }
        /* IDLE 1/2/3: 0xF4E1, 0xF5E1, 0xF6E1, 0xF7E1 (mask 0xFCFF) */
        if ((op & 0xFCFF) == 0xF4E1) {
            int level = ((op >> 8) & 0x3) + 1;
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE%d @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u XPC=%d",
                        level, s->pc, !!(s->st1 & ST1_INTM),
                        s->imr, s->sp, s->insn_count, s->xpc);
            idle_log++;
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed + s->lk_used;
            }
            s->idle = true;
            return 0;
        }
        /* ================================================================
         * F[4-7]xx generic accumulator family — promoted from F4 block
         * to handle F5/F6/F7 variants. Handlers use bits 8/9 for src/dst,
         * with masks FCE0/FCFF/FEFF naturally covering all 4 combinations
         * (A->A, B->A, A->B, B->B). The matching handler bodies remain
         * inside the F4 block as dead code (never reached for arith ops
         * because of the early return here). 2026-04-28.
         * ================================================================ */
            /* F483/F583: SAT src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF483) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                return consumed + s->lk_used;
            }

            /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF484) {
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int64_t val = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                return consumed + s->lk_used;
            }

            /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF485) {
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int64_t val = sext40(src ? s->b : s->a);
                if (val < 0) val = -val;
                if (dst) s->b = sext40(val); else s->a = sext40(val);
                return consumed + s->lk_used;
            }

            /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
             * Multiply T * A(high), accumulate into dst */
            if ((op & 0xFEFF) == 0xF48C) {
                int dst = (op >> 8) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                return consumed + s->lk_used;
            }

            /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF48D) {
                int dst = (op >> 8) & 1;
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t prod = (int64_t)ah * (int64_t)ah;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                return consumed + s->lk_used;
            }

            /* F48E/F58E: EXP src (mask FEFF, 1 word)
             * Count leading sign bits of accumulator, store in T */
            if ((op & 0xFEFF) == 0xF48E) {
                int src = (op >> 8) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                int exp = 0;
                if (val == 0 || val == -1) { exp = 31; }
                else {
                    uint64_t uv = (val < 0) ? ~val : val;
                    uv &= 0xFFFFFFFFFFULL;
                    /* Count leading zeros from bit 38 down */
                    for (int i = 38; i >= 0; i--) {
                        if (uv & (1ULL << i)) break;
                        exp++;
                    }
                    exp -= 8; /* EXP = leading sign bits - 8 */
                }
                s->t = (uint16_t)(int16_t)exp;
                return consumed + s->lk_used;
            }

            /* F486/F586: MAX src (mask FEFF, 1 word) — keep max of A,B
             * F-AUDIT 2026-05-25 v5 : était à 0xF492 (= roltc per binutils).
             * binutils tic54x-opc.c : "max" 1,1,1, 0xF486, 0xFEFF
             * → constant moved from 0xF492 to 0xF486 (impl est correct). */
            if ((op & 0xFEFF) == 0xF486) {
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                else { s->st0 &= ~ST0_C; }
                return consumed + s->lk_used;
            }

            /* F487/F587: MIN src (mask FEFF, 1 word) — keep min of A,B
             * F-AUDIT 2026-05-25 v5 : était à 0xF493 (= cmpl per binutils).
             * binutils : "min" 1,1,1, 0xF487, 0xFEFF
             * → constant moved from 0xF493 to 0xF487. */
            if ((op & 0xFEFF) == 0xF487) {
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                else { s->st0 &= ~ST0_C; }
                return consumed + s->lk_used;
            }

            /* === MAC/MAS family Smem,SRC (0x28xx..0x2Fxx, mask FE00, 1 word).
             * Per tic54x-opc.c + tic54x_hi8_map.md :
             *   0x2800 mac Smem,SRC      SRC = SRC + T * data[Smem]
             *   0x2A00 macr Smem,SRC     SRC = SRC + T * data[Smem] + 0x8000
             *   0x2C00 mas Smem,SRC      SRC = SRC - T * data[Smem]
             *   0x2E00 masr Smem,SRC     SRC = SRC - T * data[Smem] + 0x8000
             * bit 8 = SRC selector (0=A, 1=B).
             * FRCT (ST1 bit) : si set, produit shift << 1 (Q15*Q15 = Q31).
             *
             * BUG observé : MAC family non-implémentée → DSP correlator
             * ne fait jamais d'accumulation, A reste stale → a_sync_ANG
             * écrit 0x498D constant (garbage acc state).
             * Implémentation Smem-only ici (variantes Xmem/Ymem dual-MAC
             * 0xA000..0xBFFF non couvertes). */
            if ((op & 0xFC00) == 0x2800) {
                int mac_sub = (op >> 9) & 1;       /* 0=add, 1=subtract */
                int mac_rnd = (op >> 8) & 0; /* not used here, separate below */
                (void)mac_rnd;
                bool mac_ind;
                uint16_t mac_addr = resolve_smem(s, op, &mac_ind);
                int16_t mac_mem = (int16_t)data_read(s, mac_addr);
                int32_t mac_prod = (int32_t)(int16_t)s->t * (int32_t)mac_mem;
                if (s->st1 & ST1_FRCT) mac_prod <<= 1;
                /* bit 8 selects SRC accumulator (A=0/B=1).
                 * Actually per binutils encoding bit 9 is op variant (mac/mas)
                 * and bit 8 is round (R). The SRC selector is bit 0 of Smem?
                 * No — looking at tic54x table: opcode 0x2800/FE00 encodes :
                 *   bits 9..15 = op family (mac/mas/macr/masr)
                 *   bit 8      = SRC (A=0, B=1)
                 *   bits 0..7  = Smem
                 * Mais l'encoding pose mac=0x28xx (bit 8=0=A), 0x29xx (bit 8=1=B). */
                int mac_dst = (op >> 8) & 1;
                int64_t *mac_acc = mac_dst ? &s->b : &s->a;
                int64_t mac_term = (int64_t)(int32_t)mac_prod;
                if (mac_sub) mac_term = -mac_term;
                int64_t mac_new = sext40((*mac_acc + mac_term) & 0xFFFFFFFFFFULL);
                *mac_acc = mac_new;
                return consumed + s->lk_used;
            }
            /* MACR/MASR (mask FE00, base 0x2A00/0x2E00) : same + round +0x8000.
             * bit 9 distingue add/sub : déjà géré ci-dessus via mac_sub. mais le
             * round est sur les opcodes 0x2A.../0x2E... → bit 10 ? Re-check */
            if ((op & 0xFE00) == 0x2A00 || (op & 0xFE00) == 0x2E00) {
                int macr_sub = ((op & 0xFE00) == 0x2E00) ? 1 : 0;
                bool macr_ind;
                uint16_t macr_addr = resolve_smem(s, op, &macr_ind);
                int16_t macr_mem = (int16_t)data_read(s, macr_addr);
                int32_t macr_prod = (int32_t)(int16_t)s->t * (int32_t)macr_mem;
                if (s->st1 & ST1_FRCT) macr_prod <<= 1;
                macr_prod += 0x8000; /* round */
                macr_prod &= ~0xFFFF; /* zero low half after round */
                int macr_dst = (op >> 8) & 1;
                int64_t *macr_acc = macr_dst ? &s->b : &s->a;
                int64_t macr_term = (int64_t)(int32_t)macr_prod;
                if (macr_sub) macr_term = -macr_term;
                *macr_acc = sext40((*macr_acc + macr_term) & 0xFFFFFFFFFFULL);
                return consumed + s->lk_used;
            }

            /* 0x3500 MACA Smem [, B] (mask FF00, 1 word) — B = B + A.hi * data[Smem].
             * Spécial : utilise A.hi (= A[31:16]) comme multiplicateur. */
            /* NOTE 2026-06-12: seven 0x3xxx handlers (LD,T / MPYA / LD,ASM / MASA /
             * BITT / MACA / MACAR) used to sit HERE — dead code, since hi4==0x3
             * never reaches case 0xF. They live (corrected per SPRU172C: T-load +
             * result-rounding) in case 0x3 now, with the rest of the 3xxx family.
             * (The old 0x3400 BITT comment's PC=0x9ABF correlator-angle fix was
             * therefore never in effect — re-validate the correlator angle path.) */

            /* F492/F592: ROLTC src (rotate left through TC, mask FEFF, 1 word)
             * F-AUDIT 2026-05-25 v5 : NOUVEAU handler. binutils :
             * "roltc" 1,1,1, 0xF492, 0xFEFF — était mis-décodé en MAX.
             * Semantic SPRU172C : src bit 31 → TC, src << 1, src bit 0 ← TC_old.
             * Bug observé pré-fix : A_low devenait 0 systématiquement via le
             * faux MAX (A=B if A<B) à PC=0x9abf, causant cascade STL→IMR=0. */
            if ((op & 0xFEFF) == 0xF492) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t v = *acc & 0xFFFFFFFFFFLL;
                int new_tc = (int)((v >> 31) & 1);
                int old_tc = (s->st0 & ST0_TC) ? 1 : 0;
                *acc = sext40(((v << 1) | (int64_t)old_tc) & 0xFFFFFFFFFFULL);
                if (new_tc) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

            /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
            if ((op & 0xFEFF) == 0xF49E) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val >= 0) { *acc = sext40((val << 1) + 1); }
                else { *acc = sext40(val << 1); }
                return consumed + s->lk_used;
            }

            /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
             * Per SPRU172C p.4-118: if the two MSBs of src accumulator
             * are different (not sign-extended), shift src left by 1
             * and decrement T. Otherwise do nothing. Used by the FB-det
             * correlator to normalize results; the loop exits when
             * NORM stops shifting (MSBs match = value is normalized). */
            if ((op & 0xFEFF) == 0xF48F) {
                /* mask FEFF: f48f=norm a, f58f=norm a,b — src is ALWAYS A, bit8=DST
                 * (objdump GT 2026-06-10; the old bit8-as-src read B for `norm a,b`) */
                int dst = (op >> 8) & 1;
                int64_t val = sext40(s->a);
                /* Check bits 39 and 38 — if they differ, shift left */
                int bit39 = (val >> 39) & 1;
                int bit38 = (val >> 38) & 1;
                if (bit39 != bit38) {
                    val = sext40(val << 1);
                    if (dst) s->b = val; else s->a = val;
                    s->t = (uint16_t)(s->t - 1);
                }
                /* TC flag: set if shift occurred */
                if (bit39 != bit38)
                    s->st0 |= ST0_TC;
                else
                    s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

            /* F490/F590: ROR src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF490) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint16_t c = (s->st0 >> 8) & 1; /* carry */
                uint16_t lsb = *acc & 1;
                *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F491/F591: ROL src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF491) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint16_t c = (s->st0 >> 8) & 1;
                uint16_t msb = (*acc >> 39) & 1;
                *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF488) {
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                return consumed + s->lk_used;
            }

            /* F493/F593: CMPL src (complement, mask FCFF, 1 word)
             * F-AUDIT 2026-05-25 v5 : était à 0xF486. binutils :
             * "cmpl" 1,1,2, 0xF493, 0xFCFF, {OP_SRC,OPT|OP_DST}
             * → constant moved from 0xF486 to 0xF493. Mask FEFF→FCFF
             * (= permet variant SRC=B via bit 9). */
            if ((op & 0xFCFF) == 0xF493) {
                /* bit9=SRC, bit8=DST (objdump GT: f593=cmpl a,b / f693=cmpl b,a) */
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;
                int64_t v = sext40(~(src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                if (dst) s->b = v; else s->a = v;
                return consumed + s->lk_used;
            }

            /* F49F/F59F: RND src (round, mask FCFF, 1 word)
             * F-AUDIT 2026-05-25 v5 : était à 0xF487. binutils :
             * "rnd" 1,1,2, 0xF49F, 0xFCFF, {OP_SRC,OPT|OP_DST} */
            if ((op & 0xFCFF) == 0xF49F) {
                /* bit9=SRC, bit8=DST (objdump GT: f59f=rnd a,b / f69f=rnd b,a) */
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;
                int64_t v = sext40((src ? s->b : s->a) + 0x8000);
                if (dst) s->b = v; else s->a = v;
                return consumed + s->lk_used;
            }

            /* F480/F481/F482 (mask FCFF): ADD/SUB/LD src, ASM, dst — shift the
             * source accumulator by ASM (ST1[4:0], a SIGNED 5-bit count: + left,
             * - right arithmetic) then add/sub/load into dst. AUDIT FIX 2026-06-12:
             * these IGNORED ASM (shifted by 0) — the partner bug to the dead
             * 0x3200 `LD Smem,ASM` (now fixed). The pair drives the GO handler's
             * task-bit post: 0x31E9 `LD *AR2+,ASM` loads the shift, 0x31EB `LD A,ASM,A`
             * does `A = 1 << ASM`; with ASM=0 the post landed in [0x6D1] bit0 instead
             * of bit [0x6D0] -> the overlay page chain stalled (docs/research/
             * dsp-demand-page-chain.md). ASM=0 is the common case so most prior runs
             * were unaffected, which masked it. */
            if ((op & 0xFCFC) == 0xF480 && (op & 0x3) != 0x3) {  /* exclude F483 SAT */
                int sub = op & 0x3;                            /* 0=ADD 1=SUB 2=LD */
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;
                int asm5 = s->st1 & ST1_ASM_MASK;
                int shift = (asm5 & 0x10) ? (asm5 - 32) : asm5; /* signed 5-bit */
                int64_t sv = sext40(src ? s->b : s->a);
                sv = (shift >= 0) ? sext40(sv << shift) : sext40(sv >> (-shift));
                int64_t dv = sext40(dst ? s->b : s->a);
                int64_t r = (sub == 0) ? (dv + sv) : (sub == 1) ? (dv - sv) : sv;
                if (dst) s->b = sext40(r); else s->a = sext40(r);
                return consumed + s->lk_used;
            }

            /* F4xx accumulator shift/load (1-word, mask FCE0):
             * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
            if ((op & 0xFCE0) == 0xF400) {
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF420) {
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF440) {
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF460) {
                /* SFTA src,shift,dst — arithmetic shift accumulator */
                int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            /* F[4-7]A0-BF: NOT SFTL — bogus catch-all removed (2026-06-10).
             * Real 1-word SFTL is 0xF0E0 mask 0xFCE0 (see the F3xx block).
             * Per binutils tic54x-opc.c this window actually holds:
             *   F4A0-F4A7      ld #k,arp
             *   F[4-7]A8-AF    cmpr cond,ARx   (cond bits 9-8: eq/lt/gt/neq)
             *   F[4-7]B0-BF    rsbx/ssbx ST0/ST1 (dedicated handlers below)
             * The old `(op & 0xFCE0) == 0xF4A0` "SFTL" executed ALL of these
             * as an accumulator shift — in particular the 5110 run-mode idle
             * loop's `rsbx st1,intm` @0x31A3 silently NO-OPed, so INTM stayed
             * 1 and `idle 1` was woken-but-never-serviced: the DSP idled deaf
             * forever (producer dormant, MDIRCV ring at rest). RSBX/SSBX fall
             * through to their dedicated handlers (F4B0/F5B0/F6Bx/F7B0). */
            if ((op & 0xFCF8) == 0xF4A8) {
                /* CMPR cond,ARx — compare ARx with AR0 (unsigned), set TC */
                int cond = (op >> 8) & 3;
                uint16_t arx = s->ar[op & 7], ar0 = s->ar[0];
                int tc = (cond == 0) ? (arx == ar0)
                       : (cond == 1) ? (arx <  ar0)
                       : (cond == 2) ? (arx >  ar0)
                       :               (arx != ar0);
                if (tc) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }
            if ((op & 0xFFF8) == 0xF4A0) {
                /* LD #k,ARP */
                s->st0 = (uint16_t)((s->st0 & ~ST0_ARP_MASK)
                                    | ((op & 7) << ST0_ARP_SHIFT));
                return consumed + s->lk_used;
            }

        /* F494/F594: SFTC src (mask FEFF, 1 word).
         * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
         * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
         * — without it the correlator sums never normalise. */
        if ((op & 0xFEFF) == 0xF494) {
            int src = (op >> 8) & 1;
            int64_t *acc = src ? &s->b : &s->a;
            int64_t val = sext40(*acc);
            if (val != 0) {
                int b31 = (val >> 31) & 1;
                int b30 = (val >> 30) & 1;
                if (b31 == b30) *acc = sext40(val << 1);
            }
            return consumed + s->lk_used;
        }

        if (hi8 == 0xF4) {
            /* F4xx: unconditional branch/call and special instructions.
             * Some F4xx instructions are 1-word (FRET, FRETE, RETE, TRAP, NOP, etc.)
             * Must check specific opcodes BEFORE the 2-word switch. */

            /* Note: 0xF4E4 = IDLE (handled above, not FRET).
             * Real FRET = 0xF072 (algebraic), handled in F0xx section. */
            /* NOP — F495 per SPRU172C p.4-121 */
            if (op == 0xF495) {
                return 1; /* 1-word NOP */
            }
            /* TRAP K — F4C0-F4DF per SPRU172C p.4-195:
             * SP-1, PC+1 → TOS, vector(IPTR*128 + K*4) → PC */
            if ((op & 0xFFE0) == 0xF4C0) {
                int k = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + k * 4;
                C54_LOG("TRAP-FIRED #%d → PC=0x%04x (from PC=0x%04x) [XPC non sauve : corruption vs RETE pop-2 si fire]", k, s->pc,
                        (uint16_t)(s->pc - (iptr * 0x80 + k * 4) + 1 - 1));
                return 0;
            }

            /* F4xx arithmetic instructions (1-word, per tic54x-opc.c).
             * These MUST be checked before the 2-word branch/call switch. */
            {
                /* F483/F583: SAT src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF483) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                    else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                    return consumed + s->lk_used;
                }
                /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF484) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int64_t val = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                    return consumed + s->lk_used;
                }
                /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF485) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int64_t val = sext40(src ? s->b : s->a);
                    if (val < 0) val = -val;
                    if (dst) s->b = sext40(val); else s->a = sext40(val);
                    return consumed + s->lk_used;
                }
                /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
                 * Multiply T * A(high), accumulate into dst */
                if ((op & 0xFEFF) == 0xF48C) {
                    int dst = (op >> 8) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF48D) {
                    int dst = (op >> 8) & 1;
                    int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                    int64_t prod = (int64_t)ah * (int64_t)ah;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                    return consumed + s->lk_used;
                }
                /* F48E/F58E: EXP src (mask FEFF, 1 word)
                 * Count leading sign bits of accumulator, store in T */
                if ((op & 0xFEFF) == 0xF48E) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    int exp = 0;
                    if (val == 0 || val == -1) { exp = 31; }
                    else {
                        uint64_t uv = (val < 0) ? ~val : val;
                        uv &= 0xFFFFFFFFFFULL;
                        /* Count leading zeros from bit 38 down */
                        for (int i = 38; i >= 0; i--) {
                            if (uv & (1ULL << i)) break;
                            exp++;
                        }
                        exp -= 8; /* EXP = leading sign bits - 8 */
                    }
                    s->t = (uint16_t)(int16_t)exp;
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM — handled below (real implementation, not NOP) */
                /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
                if ((op & 0xFEFF) == 0xF492) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
                if ((op & 0xFEFF) == 0xF493) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
                if ((op & 0xFEFF) == 0xF49E) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val >= 0) { *acc = sext40((val << 1) + 1); }
                    else { *acc = sext40(val << 1); }
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
                 * Per SPRU172C p.4-118: if the two MSBs of src accumulator
                 * are different (not sign-extended), shift src left by 1
                 * and decrement T. Otherwise do nothing. Used by the FB-det
                 * correlator to normalize results; the loop exits when
                 * NORM stops shifting (MSBs match = value is normalized). */
                if ((op & 0xFEFF) == 0xF48F) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    /* Check bits 39 and 38 — if they differ, shift left */
                    int bit39 = (val >> 39) & 1;
                    int bit38 = (val >> 38) & 1;
                    if (bit39 != bit38) {
                        val = sext40(val << 1);
                        if (src) s->b = val; else s->a = val;
                        s->t = (uint16_t)(s->t - 1);
                    }
                    /* TC flag: set if shift occurred */
                    if (bit39 != bit38)
                        s->st0 |= ST0_TC;
                    else
                        s->st0 &= ~ST0_TC;
                    return consumed + s->lk_used;
                }
                /* F49F: DELAY (pipeline flush, NOP) */
                if (op == 0xF49F) { return consumed + s->lk_used; }
                /* F490/F590: ROR src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF490) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1; /* carry */
                    uint16_t lsb = *acc & 1;
                    *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                    if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F491/F591: ROL src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF491) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1;
                    uint16_t msb = (*acc >> 39) & 1;
                    *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                    if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF488) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF486) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                    return consumed + s->lk_used;
                }
                /* F487/F587: RND src (round, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF487) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(*acc + 0x8000);
                    return consumed + s->lk_used;
                }
                /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF480) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF481) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF482) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                /* F4xx accumulator shift/load (1-word, mask FCE0):
                 * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
                if ((op & 0xFCE0) == 0xF400) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF420) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF440) {
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF460) {
                    /* SFTA src,shift,dst — arithmetic shift accumulator */
                    int src = (op >> 9) & 1, dst = (op >> 8) & 1;   /* bit9=SRC, bit8=DST (objdump GT 2026-06-10: f517=add a,-9,b) */
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                /* (the old F4A0 "SFTL" dead-code copy removed 2026-06-10 —
                 * F4A0-F4BF is ld#k,arp / cmpr / rsbx-ssbx ST0, decoded by the
                 * promoted family above + the F4B0 handler below. With the
                 * promoted copy fixed, this one would have gone LIVE and
                 * swallowed F4Bx RSBX before its dedicated handler.) */
            }
                        /* F4Bx: RSBX -- reset bit in ST0 (bit 9=0, bit 8=0).
             * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF4B0) {
                int bit = op & 0x0F;
                s->st0 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* F494/F594: SFTC src (mask FEFF, 1 word).
             * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
             * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
             * — without it the correlator sums never normalise. */
            if ((op & 0xFEFF) == 0xF494) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val != 0) {
                    int b31 = (val >> 31) & 1;
                    int b30 = (val >> 30) & 1;
                    if (b31 == b30) *acc = sext40(val << 1);
                }
                return consumed + s->lk_used;
            }
            /* Remaining F4xx: unhandled — treat as 1-word NOP */
            C54_LOG("F4xx unhandled: 0x%04x PC=0x%04x", op, s->pc);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xF0 || hi8 == 0xF1) {
            /* FIRS catch RETIRÉ (2026-05-25 v3, Claude web review).
             *
             * Le bloc `if (hi8 == 0xF1) { FIRS treatment }` qui était ici
             * était FAUX : per binutils tic54x-opc.c, vrai FIRS = 0xE000
             * mask 0xFF00 (handled separately at the 0xE0 case), JAMAIS
             * 0xF1xx. Le catch-all capture-tout 0xF1xx faisait :
             *   s->a = sext40((int64_t)sum << 16);
             * → A_low = 0 inconditionnellement → STL A,*AR2- à PC=0x9ac0
             * écrivait 0 à mem[AR2] qui se trouvait être MMR_IMR (0x12 via
             * self-aliasing) → IMR cleared → DSP bloqué (bloqueur #2).
             *
             * Diagnostic via A provenance tracer (CALYPSO_A_TRACE_PC=0x9ac0)
             * a montré last_writer = PC=0x9abd op=0xf1fe = `SFTL A,-2,B`
             * (binutils mask 0xFCE0 base 0xF0E0). SFTL handler EXISTE à
             * L3915, capture correctement 0xF1FE & 0xFCE0 = 0xF0E0.
             *
             * Après retrait du catch, les 0xF1xx tombent dans :
             *   - SFTL/AND/OR/XOR 1-word (mask FCE0)  : L3915
             *   - AND/OR/XOR/MAC #lk<<16 (mask FCFF) : L3852
             *   - AND/OR/XOR #lk+shift  (mask FCF0)  : L3886
             * Si une opcode 0xF1xx n'a pas de handler (par ex. add/sub lk
             * variants avec DST=B), tombe à F4xx unhandled NOP log à la fin
             * du bloc → diagnostic visible. */
            /* F073: B pmad — unconditional branch (2-word).
             * Per tic54x-opc.c: 0xF073 mask 0xFFFF. */
            if (op == 0xF073) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F074: CALL pmad — unconditional call (2-word).
             * Per tic54x-opc.c: call 0xF074 mask 0xFFFF.
             * Push PC+2 (return address), branch to pmad.
             * NOTE: RETE is 0xF4EB (already handled above), NOT F074. */
            if (op == 0xF074) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }







            /* F072: RPTB pmad — block repeat (2-word, non-delayed).
             * Per tic54x-opc.c: 0xF072 mask 0xFFFF.
             * RSA = PC+2, REA = pmad. */
            if (op == 0xF072) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            /* F07x: RPT/RPTZ/misc (F072-F074 handled above) */
            if (op == 0xF070) {
                /* F070: RPT #lku — repeat next instruction lku+1 times (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rpt_count = op2;
                s->rpt_active = true; s->rpt_arming = true;
                s->pc += 2;
                return 0;
            }
            if (op == 0xF071) {
                /* F071: RPTZ dst, #lku — zero accumulator and repeat (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int dst = (op >> 8) & 1; /* bit 8 via FEFF mask */
                if (dst) s->b = 0; else s->a = 0;
                s->rpt_count = op2;
                s->rpt_active = true; s->rpt_arming = true;
                s->pc += 2;
                return 0;
            }
            if ((op & 0xFFF0) == 0xF070) {
                /* F075-F07F: undefined, treat as 1-word NOP */
                return consumed + s->lk_used;
            }
            /* F0Bx/F1Bx: RSBX/SSBX */
            if ((op & 0x00F0) == 0x00B0) {
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;
                if (st == 0) { if (set) s->st0 |= (1<<bit); else s->st0 &= ~(1<<bit); }
                else         { if (set) s->st1 |= (1<<bit); else s->st1 &= ~(1<<bit); }
                return consumed + s->lk_used;
            }
            /* F0xx/F1xx ALU with #lk immediate (2-word).
             * Per tic54x-opc.c: bits 7:4 = op (0=ADD,1=SUB,2=LD,3=AND,4=OR,5=XOR),
             * bit 8 = SRC (ADD/SUB/AND/OR/XOR) or DST (LD), bit 9 = DST,
             * bits 3:0 = shift. Second word = lk. */
            {
                uint8_t alu_op = (op >> 4) & 0xF;
                if (alu_op <= 5) {
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int shift = op & 0xF;
                    /* SRC/DST convention: TI/binutils strict is bit9=SRC, bit8=DST
                     * — identical to the F2xx handler (L5977) and the acc-to-acc fix
                     * (2026-05-25 v4, L5840). The legacy map here was REVERSED
                     * (bit8=src, bit9=dst), which silently SWAPPED operands for
                     * CROSS-accumulator ALU #lk ops (e.g. F130 `B = A & #lk` in the
                     * host-cmd ISR dispatch @0x35C0 → A→0, B stale → cmd misrouted to
                     * the 0x1000 handler → the port channel never worked, §8v).
                     * Same-accumulator ops (A=A+lk) are unaffected. For LD (alu_op==2)
                     * the dst sits at bit8 in BOTH maps, so LD is unchanged.
                     * CALYPSO_FLK_LEGACY=1 restores the old reversed map for A/B. */
                    static int flk_leg = -1;
                    if (flk_leg < 0) flk_leg = getenv("CALYPSO_FLK_LEGACY") ? 1 : 0;
                    int src_sel = flk_leg ? ((op >> 8) & 1) : ((op >> 9) & 1);
                    int dst_sel = flk_leg ? ((op >> 9) & 1) : ((op >> 8) & 1);
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = (alu_op == 2 && flk_leg)
                        ? (src_sel ? &s->b : &s->a)
                        : (dst_sel ? &s->b : &s->a);
                    int64_t lk_val;
                    if (alu_op <= 2)
                        lk_val = (int64_t)(int16_t)op2 << shift;
                    else
                        lk_val = (int64_t)(uint16_t)op2 << shift;
                    switch (alu_op) {
                    case 0: *dst = sext40(src_val + lk_val); break; /* ADD */
                    case 1: *dst = sext40(src_val - lk_val); break; /* SUB */
                    case 2: *dst = sext40(lk_val); break;           /* LD  */
                    case 3: *dst = src_val & lk_val; break;         /* AND */
                    case 4: *dst = src_val | lk_val; break;         /* OR  */
                    case 5: *dst = src_val ^ lk_val; break;         /* XOR */
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op == 6) {
                    /* F06x: ADD/SUB/LD/AND/OR/XOR #lk,16 + MPY/MAC #lk */
                    uint8_t sub6 = op & 0xF;
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    switch (sub6) {
                    case 0: *dst = sext40(src_val + ((int64_t)(int16_t)op2 << 16)); break;
                    case 1: *dst = sext40(src_val - ((int64_t)(int16_t)op2 << 16)); break;
                    case 2: dst = src_sel ? &s->b : &s->a;
                            *dst = sext40((int64_t)(int16_t)op2 << 16); break;
                    case 3: *dst = src_val & ((int64_t)(uint16_t)op2 << 16); break;
                    case 4: *dst = src_val | ((int64_t)(uint16_t)op2 << 16); break;
                    case 5: *dst = src_val ^ ((int64_t)(uint16_t)op2 << 16); break;
                    case 6: /* MPY #lk, dst */
                            dst = src_sel ? &s->b : &s->a;
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(p); } break;
                    case 7: /* MAC #lk, src[,dst] */
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(src_val + p); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op >= 8) {
                    /* F08x-F0Fx: accumulator-to-accumulator ops (1-word).
                     * bits 7:5 = op (100=AND,101=OR,110=XOR,111=SFTL)
                     * bits 4:0 = shift (signed 5-bit), bits 9:8 = src,dst
                     *
                     * Fix 2026-05-25 v4 : src/dst inversés. Per binutils
                     * tic54x convention (et confirmé par L3915 même opcode
                     * famille mais handler shadowed) : bit 9 = SRC,
                     * bit 8 = DST. L'inversion mettait dst=A pour 0xf1fe
                     * (= SFTL A,-2,B), qui calculait A = B>>2 au lieu de
                     * B = A>>2. Si B=0 → A=0 → STL A,*AR2- pose 0 à IMR
                     * (bloqueur #2 racine, cf [[blocker-2-dsp-dispatcher]]).
                     * Diagnostic via A-AT-PC tracer : 8454 fires A_low=0
                     * last_writer=0xf1fe @PC=0x9abd. */
                    int src_sel = (op >> 9) & 1;   /* bit 9 = SRC (was bit 8) */
                    int dst_sel = (op >> 8) & 1;   /* bit 8 = DST (was bit 9) */
                    int64_t sv = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    int shift = op & 0x1F;
                    if (shift > 15) shift -= 32;
                    uint8_t aop = (op >> 5) & 0x7;
                    int64_t shifted;
                    if (shift >= 0) shifted = sv << shift;
                    else            shifted = sv >> (-shift);
                    switch (aop) {
                    case 4: *dst = sext40(sv) & sext40(shifted); break;
                    case 5: *dst = sext40(sv) | sext40(shifted); break;
                    case 6: *dst = sext40(sv) ^ sext40(shifted); break;
                    case 7: { uint64_t uv = (uint64_t)(sv & 0xFFFFFFFFFFULL);
                              if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                              *dst = sext40(uv & 0xFFFFFFFFFFULL); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
            }
            goto unimpl;
        }
        /* F272/F274/F273: RPTBD/CALLD/RETD — must check BEFORE LMS */
        if (op == 0xF272) {
            /* RPTBD pmad — delayed block repeat (2 words).
             * Delayed: 2 delay slots after the 2-word instruction.
             * RSA = PC + 4 (skip RPTBD + 2 delay slot words). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            { static int _rb=0; if (_rb<20) { C54_LOG("RPTBD PC=0x%04x REA=0x%04x RSA=0x%04x BRC=%d", s->pc, s->rea, s->rsa, s->brc); _rb++; } }
            return consumed + s->lk_used;
        }
        if (op == 0xF274) {
            /* CALLD pmad — delayed call (2 words, 2 delay slots).
             * Push PC+4 (return = past CALLD + 2 delay slots), PUIS exécute les
             * 2 delay-slots via delayed_pc/delay_slots AVANT de brancher.
             * Fix 2026-05-30 : était saut immédiat (s->pc=op2; return 0) → les
             * 2 delay-slots étaient SKIPPÉS ; si un slot contient un push/pop,
             * la pile se désaligne d'1 mot → POPM ST0 ramasse un PC orphelin
             * (ex. 0x80fd) → DP garbage → CALA 0x70c3 = trou noir → TOA figé →
             * AFC bloqué. Arme la machinerie delay_slots (comme RCD/RETED). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 4));
            s->delayed_pc  = op2;
            s->delay_slots = 2;
            return consumed + s->lk_used;
        }
        if (op == 0xF273) {
            /* BD pmad — delayed branch (2 words, 2 delay slots). AUCUNE pile.
             * Per tic54x-opc.c: bd 0xF273 mask 0xFFFF. Le vrai RETD = 0xFE00
             * (hi8==0xFE, géré plus bas avec pop + delay_slots).
             * Fix 2026-05-30 : était traité comme RETD (pop parasite) → SP
             * désaligné d'1 mot par BD → POPM ST0 0xf48b pop un PC orphelin
             * → DP=0x087 → dispatcher 0x8341 → CALAD 0x70c3 = trou noir.
             * Identique au B (F073) ci-dessus : saut, pas de pile.
             * Fix 2026-05-30 v2 : était saut IMMÉDIAT (skip des 2 delay-slots) →
             * si un slot a un push/pop, pile désalignée → POPM ST0 orphelin →
             * 0x70c3. Arme delay_slots=2 pour exécuter les slots avant le saut. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->delayed_pc  = op2;
            s->delay_slots = 2;
            return consumed + s->lk_used;
        }
        /* === F2xx dispatch (audit F-class 2026-05-25) =====================
         *
         * Per binutils tic54x-opc.c, les ALU masks FCF0/FCFF/FCE0 couvrent
         * F0xx/F1xx/F2xx/F3xx avec bit 9=SRC bit 8=DST (= convention
         * binutils stricte). F2xx était le seul gap :
         *   - F0/F1 → handler legacy L3565 (convention REVERSED bit 8=src,
         *     gardée pour back-compat ; firmware s'y est calé)
         *   - F3 → handler dispatch L3966 (binutils convention OK)
         *   - F2 → fallthrough vers unimpl → 0xf210 tight loop at PC=0xfbd9
         *
         * Bug runtime résolu : op=0xf210 op2=0x0008 → `SUB #8,B,A` (next BC
         * fbe2, ALEQ → wait loop pre-correlator). Confirmed across 3 silicon
         * ROM dumps (3416, 3606, our local) — cf doc/datasheets/.
         *
         * Coverage (binutils strict) :
         *   - F260-F267 mask FCFF : ALU #lk,16 + MAC
         *   - F200/F210/F220/F230/F240/F250 mask FCF0 : ADD/SUB/LD/AND/OR/XOR #lk,shift
         *   - F280-F2FF mask FCE0 : 1-word AND/OR/XOR/SFTL src,shift,dst
         *
         * F272/F273/F274 (exact-match RPTBD/BD/CALLD) restent gérés AVANT
         * (handlers ci-dessus), inchangés. */
        if (hi8 == 0xF2) {
            /* F260-F267 : 2-word ALU #lk,16 + MAC #lk (mask FCFF) */
            if ((op & 0xFCFF) == 0xF060 ||  /* ADD */
                (op & 0xFCFF) == 0xF061 ||  /* SUB */
                (op & 0xFCFF) == 0xF062 ||  /* LD  */
                (op & 0xFCFF) == 0xF063 ||  /* AND */
                (op & 0xFCFF) == 0xF064 ||  /* OR  */
                (op & 0xFCFF) == 0xF065 ||  /* XOR */
                (op & 0xFCFF) == 0xF067) {  /* MAC */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int sub = op & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x0: result = src + ((int64_t)(int16_t)op2 << 16); break;
                case 0x1: result = src - ((int64_t)(int16_t)op2 << 16); break;
                case 0x2: result = ((int64_t)(int16_t)op2 << 16); break;
                case 0x3: result = src & (((int64_t)op2) << 16); break;
                case 0x4: result = src | (((int64_t)op2) << 16); break;
                case 0x5: result = src ^ (((int64_t)op2) << 16); break;
                case 0x7: {
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    result = src + prod; break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F200/F210/F220/F230/F240/F250 : 2-word ALU #lk,shift (mask FCF0) */
            if ((op & 0xFCF0) == 0xF000 ||  /* ADD */
                (op & 0xFCF0) == 0xF010 ||  /* SUB  ← 0xF210 hit ici */
                (op & 0xFCF0) == 0xF020 ||  /* LD   */
                (op & 0xFCF0) == 0xF030 ||  /* AND  */
                (op & 0xFCF0) == 0xF040 ||  /* OR   */
                (op & 0xFCF0) == 0xF050) {  /* XOR  */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t lk_signed = (subop <= 2) ? (int64_t)(int16_t)op2
                                                 : (int64_t)(uint16_t)op2;
                int64_t lk_val = (shift >= 0) ? (lk_signed << shift)
                                              : (lk_signed >> (-shift));
                int64_t result = src;
                switch (subop) {
                case 0x0: result = src + lk_val; break;
                case 0x1: result = src - lk_val; break;
                case 0x2: result = lk_val; break;
                case 0x3: result = src & lk_val; break;
                case 0x4: result = src | lk_val; break;
                case 0x5: result = src ^ lk_val; break;
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F280-F2FF : 1-word shift class (AND/OR/XOR/SFTL src,shift,dst) FCE0 */
            if ((op & 0xFCE0) == 0xF080 ||  /* AND */
                (op & 0xFCE0) == 0xF0A0 ||  /* OR  */
                (op & 0xFCE0) == 0xF0C0 ||  /* XOR */
                (op & 0xFCE0) == 0xF0E0) {  /* SFTL */
                int sub = (op >> 5) & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int shift_raw = op & 0x1F;
                int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x4: { int64_t dst_in = dst_b ? s->b : s->a;
                            int64_t sh = (shift >= 0) ? (dst_in << shift)
                                                      : (dst_in >> (-shift));
                            result = src & sh; break; }
                case 0x5: { int64_t dst_in = dst_b ? s->b : s->a;
                            int64_t sh = (shift >= 0) ? (dst_in << shift)
                                                      : (dst_in >> (-shift));
                            result = src | sh; break; }
                case 0x6: { int64_t dst_in = dst_b ? s->b : s->a;
                            int64_t sh = (shift >= 0) ? (dst_in << shift)
                                                      : (dst_in >> (-shift));
                            result = src ^ sh; break; }
                case 0x7: { uint64_t usrc = (uint64_t)src & 0xFFFFFFFFFFULL;
                            result = (int64_t)((shift >= 0) ? (usrc << shift)
                                                            : (usrc >> (-shift)));
                            break; }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }
            /* F2xx unmapped — log-once + NOP fallback. Si tu vois ce log,
             * c'est qu'un bit pattern F2xx n'est pas couvert (audit incomplete). */
            { static int f2_unm = 0;
              if (f2_unm++ < 20)
                  C54_LOG("F2xx unmapped op=0x%04x PC=0x%04x (NOP)", op, s->pc); }
            return consumed + s->lk_used;
        }
        /* LMS Xmem, Ymem — Least Mean Square step (1-word dual-operand)
         * Encoding: 1111 001D XXXX YYYY
         * Per SPRU172C: dst += T * Xmem; Ymem += rnd(AH * T); T = Xmem
         * Exclude F272 (RPTBD), F273 (RETD), F274 (CALLD) — exact-match
         * opcodes that share the F2xx range but are handled below. */
        /* REMOVED 2026-05-08 night : the previous "LMS Xmem,Ymem" handler
         * for hi8 ∈ {0xF2, 0xF3} (excluding F272/F273/F274) was mis-decoded
         * — it claimed encoding `1111 001D XXXX YYYY` but per binutils
         * tic54x-opc.c LMS is actually :
         *
         *   { "lms", 1,2,2, 0xE100, 0xFF00, {OP_Xmem,OP_Ymem}, ... }
         *
         * i.e. hi8 == 0xE1, NOT 0xF2/F3. The 0xE1 handler already exists
         * (line ~3247) and is correct.
         *
         * The F2xx/F3xx range per binutils contains only :
         *   F272 RPTBD, F273 RETD, F274 CALLD                (3 special-cases)
         *   F300-F31F INTR k                                 (handled below)
         *   F330-F35F AND/OR/XOR with shift  (mask FCF0)     (handled below)
         *   F360-F367 ADD/SUB/AND/OR/XOR/MAC #lk (mask FCFF) (handled below)
         *   F380-F3FF AND/OR/XOR/SFTL src,SHIFT,DST (FCE0)   (handled below)
         *   F320-F32F + F368-F37F unmapped (NOP fallback)
         *
         * The bogus LMS catch-all stole every F3xx instruction before the
         * proper F3 dispatch could see it. For 0xF3E1 (= SFTL B,1,B,
         * 4872 sites in firmware) it computed `new_ym = AH*T-derived junk`
         * and called data_write(s, AR1, new_ym). When AR1=0, that wrote
         * the junk to MMR_IMR. This is the IMR-thrash cascade observed
         * post-0x76-fix at PC=0x8eb9.
         *
         * Discovered after the 0x76 fix exposed the second-level cascade.
         * Trace evidence : IMR-W 0x0000→{0x0540, 0x0525, 0x082b, 0xfd57,
         * 0xfacf, ...} all PC=0x8eb9 op=0xf3e1, INTM-TRANS XPC=0
         * (confirms genuine PROM0 execution, not XPC artifact).
         *
         * Fix : let the existing F3 dispatch (line 2468+) handle F3xx
         * properly. F2xx (other than F272/3/4) falls through to F-class
         * NOP fallback — firmware does not appear to use it. */
        /* F8xx: branches, RPT, BANZ, CALL, RET variants */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            /* F820 (624 sites) and F830 (543 sites) are BC pmad,cond per
             * tic54x-opc.c (bc = 0xF800 mask 0xFF00). The dispatcher at
             * PROM0 0xb968-0xb9a4 relies on these branching when the ACC
             * comparison succeeds. Cond 0x20 = C set, cond 0x30 = ?
             * (we treat both via ACC compare for now since dispatcher uses
             * cmp-style behaviour). The full F8xx range is BC per binutils
             * but historically the firmware tolerates the legacy decode
             * for the other sub-codes — surgical override here only.
             *
             * REVERTED 2026-05-15 nuit : tentative de fix vers SPRU172C-strict
             * cond eval (cond=0x20=NTC, cond=0x30=TC) a cassé le firmware DSP
             * Calypso (DSP stuck à PC=0xcc51 / 0xfa95 selon régime, task=24
             * tombait à 0). Le binaire DSP semble utiliser une convention
             * dialectale où F82x/F83x s'attend au comportement ACC-based.
             * Hypothèse alternative : BITF (0x61) émulé incorrectement, TC
             * jamais set correctement → cond NTC/TC ne donne pas le bon
             * résultat. Investiguer BITF avant de retenter le fix BC strict. */
            /* F800-F87F = BC pmad,cond (NON-delayed conditional branch). The cond
             * is the low 7 bits, decoded per binutils condition_codes[] via
             * c54x_cond_true() — the SAME table CC/CCD/BCD use. The legacy lift
             * used an accumulator heuristic for F82x/F83x (treating BC NTC/TC as
             * A!=0/A==0), which corrupts all TC-gated control flow in the Nokia
             * DSP (the audio/BIST loop's `bitf`→`if(TC) goto` exits) → it never
             * drains to idle (docs §8k). Honour the real condition. BITF (0x61)
             * verified to set TC correctly. CALYPSO_F8_ACC=1 restores old heuristic.
             * (F880-F8FF handled below = FAR unconditional branch, op&0x80 set.) */
            if ((op & 0x80) == 0) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                static int f8_acc = -1;
                if (f8_acc < 0) f8_acc = getenv("CALYPSO_F8_ACC") ? 1 : 0;
                bool take;
                if (f8_acc && (sub == 0x2 || sub == 0x3)) {
                    int64_t acc_signed = (s->a & 0x8000000000LL)
                                         ? (s->a | ~0xFFFFFFFFFFLL) : s->a;
                    take = (sub == 0x2) ? (acc_signed != 0) : (acc_signed == 0);
                } else {
                    take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
                }
                if (take) { s->pc = op2; return 0; }
                return consumed + s->lk_used;
            }
            /* Per tic54x-opc.c:
             *   F880-F8FF mask FF80 = FB pmad (FAR branch unconditional)
             * The low 7 bits of the opcode word encode the target XPC bits.
             * Calypso uses 2-bit XPC, so & 0x3 is sufficient.
             *
             * Earlier this range was treated as plain B pmad — a bug that
             * kept XPC=0 forever (DSP never reached PROM1 user code). */
            if ((op & 0xFF80) == 0xF880) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fb_total;
                fb_total++;
                if (fb_total <= 30 || (fb_total % 5000) == 0) {
                    C54_LOG("FB FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u)",
                            (unsigned long long)fb_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            /* F88x..F8Bx (mask FF80=0): historic plain B pmad (NEAR), kept
             * for sub-codes that fall outside the FAR mask above. */
            if (sub >= 0x8 && sub <= 0xB) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F86x/F87x: BANZ *ARn, pmad — branch if ARn != 0 (2 words) */
            if (sub == 0x6 || sub == 0x7) {
                op2 = prog_fetch(s, s->pc + 1);
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;  /* skip 2 words, fall through */
            }
            /* F84x/F85x: BANZ with condition / CALL variants */
            if (sub == 0x4 || sub == 0x5) {
                op2 = prog_fetch(s, s->pc + 1);
                /* BANZ ARn, pmad */
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;
            }
            /* F8Cx-F8Fx: CALL/CALLD pmad (2 words) */
            if (sub >= 0xC) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            /* F80x-F81x: BANZ pmad, Smem (2 words)
             * Per SPRU172C + tic54x-opc.c: entire F8xx range is BANZ.
             * Sind operand selects AR via op[2:0] (nar). Test pre-mod
             * value; resolve_smem applies Sind post-mod. Same off-by-ARP
             * fix as 0x6C00 / 0x6E00 BANZ/BANZD. */
            if (sub <= 0x1) {
                int nar = op & 0x07;
                uint16_t old_ar = s->ar[nar];
                addr = resolve_smem(s, op, &ind);
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                if (old_ar != 0) {
                    s->pc = op2;
                    return 0;
                }
                return consumed + s->lk_used;
            }
            /* Fallback: RPT Smem (F8xx sub not handled above) */
            addr = resolve_smem(s, op, &ind);
            s->rpt_count = data_read(s, addr);
            s->rpt_active = true; s->rpt_arming = true;
            s->pc += consumed;
            return 0;
        }
        /* F3xx: dispatch per binutils tic54x-opc.c (verified against
         * insn_template struct include/opcode/tic54x.h:85-150).
         *
         * 8 sub-families:
         *   F300-F31F  INTR k                                 1-word
         *   F320-F32F  unmapped                               (NOP fallback)
         *   F330-F35F  AND/OR/XOR #lk,SHIFT,SRC,DST  mask FCF0 2-word
         *   F360-F367  ADD/SUB/AND/OR/XOR/MAC #lk var. FCFF   2-word
         *   F368-F37F  unmapped                               (NOP fallback)
         *   F380-F39F  AND  src,SHIFT,DST            mask FCE0 1-word
         *   F3A0-F3BF  OR   src,SHIFT,DST            mask FCE0 1-word
         *   F3C0-F3DF  XOR  src,SHIFT,DST            mask FCE0 1-word
         *   F3E0-F3FF  SFTL src,SHIFT,DST            mask FCE0 1-word
         *
         * Dispatch order: most-specific masks first (FCFF → FCF0 → FCE0).
         *
         * 2026-04-29 — replaces previous "F320+ → LD #k9, DP" fallback
         * which mass-mis-decoded 364 firmware sites. Wedge at PC=0x8eb9
         * (0xF3E1 SFTL B,1,B) was directly tied to this bug.
         * See doc/opcodes/0xF3.md for full spec. */
        if (hi8 == 0xF3) {
            /* === F300-F31F INTR k REMOVED (2026-05-25 night, audit F-class)
             *
             * AUDIT-FINDING : the "INTR k" handler placed at 0xF300-0xF31F
             * was WRONG. Per binutils tic54x-opc.c L311 :
             *   { "intr", 1,1,1, 0xF7C0, 0xFFE0, ... }  ← REAL INTR k
             * INTR k base is 0xF7C0, NOT 0xF300. The F3xx range belongs to
             * ALU #lk class (per mask 0xFCF0).
             *
             * Symptom captured runtime (CALYPSO_AR_TRACE=0x08) :
             *   PC=0xe9a2 op=0x8913 STLM B,AR3 fires 10243× with B=0 → AR3=0
             *   The preceding PC=0xe9a0 op=0xf310 was MEANT to be `SUB #5,B,B`
             *   (= B -= 5) but our wrong INTR handler pushed PC+1 and jumped
             *   to vec table → SUB never executed → B stayed 0 → AR3=0 →
             *   BANZ fc54,*AR3- loop infinite at fc50-fc6d → INT3 ISR never
             *   RETE → INTM=1 forever → IRQ subséquentes pending only.
             *
             * Fix : retirer ce handler ; F310 etc. tombent dans la FCF0
             * dispatch ci-dessous (ADD/SUB/LD/AND/OR/XOR pour F3xx).
             *
             * A real INTR k handler should be added at F7Cx if firmware
             * uses it — TODO. Pas urgent (zero F7Cx hits observed in run). */

            /* F360-F367: 2-word with mask FCFF (#lk<<16 variants).
             * Most-specific mask, check first. */
            if ((op & 0xFCFF) == 0xF060 ||  /* ADD #lk<<16, src, [dst] */
                (op & 0xFCFF) == 0xF061 ||  /* SUB */
                (op & 0xFCFF) == 0xF063 ||  /* AND */
                (op & 0xFCFF) == 0xF064 ||  /* OR  */
                (op & 0xFCFF) == 0xF065 ||  /* XOR */
                (op & 0xFCFF) == 0xF067) {  /* MAC #lk, src, [dst] */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int sub = op & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x0: result = src + ((int64_t)(int16_t)op2 << 16); break;
                case 0x1: result = src - ((int64_t)(int16_t)op2 << 16); break;
                case 0x3: result = src & (((int64_t)op2) << 16); break;
                case 0x4: result = src | (((int64_t)op2) << 16); break;
                case 0x5: result = src ^ (((int64_t)op2) << 16); break;
                case 0x7: { /* MAC: dst = src + T * lk */
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    result = src + prod;
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F300-F35F: 2-word with mask FCF0 (ALU #lk + 4-bit shift).
             * ADD (sub=0), SUB (sub=1), LD (sub=2), AND (sub=3), OR (sub=4),
             * XOR (sub=5).
             *
             * 2026-05-25 night : ADD/SUB/LD ADDED here (étaient mis-décodés
             * par le faux INTR k F300 retiré ci-dessus). Fix smoking-gun
             * 0xf310 = SUB #lk,B,B au PC=0xe9a0 → B=0 → AR3=0 → loop fc50. */
            if ((op & 0xFCF0) == 0xF000 ||  /* ADD #lk, SHIFT, src, [dst] */
                (op & 0xFCF0) == 0xF010 ||  /* SUB ← FIX 0xf310 */
                (op & 0xFCF0) == 0xF020 ||  /* LD  (binutils mask FEF0, no src) */
                (op & 0xFCF0) == 0xF030 ||  /* AND */
                (op & 0xFCF0) == 0xF040 ||  /* OR */
                (op & 0xFCF0) == 0xF050) {  /* XOR */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                /* ADD/SUB/LD : lk signed-extended ; AND/OR/XOR : lk unsigned. */
                int64_t lk_val;
                if (subop <= 2) {
                    int64_t lk_signed = (int64_t)(int16_t)op2;
                    lk_val = (shift >= 0) ? (lk_signed << shift)
                                          : (lk_signed >> (-shift));
                } else {
                    int64_t lk_unsigned = (int64_t)(uint16_t)op2;
                    lk_val = (shift >= 0) ? (lk_unsigned << shift)
                                          : (lk_unsigned >> (-shift));
                }
                int64_t result = src;
                switch (subop) {
                case 0x0: result = src + lk_val; break;   /* ADD */
                case 0x1: result = src - lk_val; break;   /* SUB */
                case 0x2: result = lk_val; break;         /* LD (src ignored) */
                case 0x3: result = src & lk_val; break;   /* AND */
                case 0x4: result = src | lk_val; break;   /* OR  */
                case 0x5: result = src ^ lk_val; break;   /* XOR */
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F380-F3FF: 1-word AND/OR/XOR/SFTL src,SHIFT,DST (mask FCE0).
             * Sub-opcode in bits 7-5: 100=AND, 101=OR, 110=XOR, 111=SFTL. */
            if ((op & 0xFCE0) == 0xF080 ||  /* AND */
                (op & 0xFCE0) == 0xF0A0 ||  /* OR  */
                (op & 0xFCE0) == 0xF0C0 ||  /* XOR */
                (op & 0xFCE0) == 0xF0E0) {  /* SFTL */
                int sub = (op >> 5) & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int shift_raw = op & 0x1F;
                int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x4: { /* AND src,SHIFT,DST: DST = SRC & (DST_in << shift) */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src & sh;
                    break;
                }
                case 0x5: { /* OR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src | sh;
                    break;
                }
                case 0x6: { /* XOR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src ^ sh;
                    break;
                }
                case 0x7: { /* SFTL src,SHIFT,DST: DST = SRC << shift (logical) */
                    uint64_t usrc = (uint64_t)src & 0xFFFFFFFFFFULL;
                    result = (int64_t)((shift >= 0) ? (usrc << shift) : (usrc >> (-shift)));
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F320-F32F + F368-F37F: unmapped per binutils. NOP fallback +
             * log-once for diagnostic. 9 firmware sites total. */
            {
                static int unmapped_log = 0;
                if (unmapped_log++ < 20)
                    C54_LOG("F3xx unmapped op=0x%04x PC=0x%04x (NOP)",
                            op, s->pc);
            }
            return consumed + s->lk_used;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x2) {
                /* F62x: LD A, dst_shift, B or LD B, dst_shift, A */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0x6) {
                /* F66x: LD A/B with shift to other acc */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0xB) {
                /* F6Bx: RSBX -- reset bit in ST1 (bit 9=1, bit 8=0).
                 * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0 covers F6Bx. */
                int bit = op & 0x0F;
                rsbx_intm_check(s, op);  /* probe candidat 1 doc §7 */
                s->st1 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* Delayed branches/calls/returns from PROM (per tic54x-opc.c).
             * MUST be checked BEFORE the MVDD catch-all because they share
             * the high nibbles 0xE/0x9. Without these the DSP cannot return
             * from interrupt service routines — RETED in particular leaves
             * INTM=1 forever, blocking every subsequent INT3 and stalling
             * the firmware↔DSP frame loop (the original CLAUDE.md root bug).
             *
             * All delayed forms execute 2 delay-slot words before the jump
             * commits; we arm the existing delayed_pc/delay_slots machinery
             * (the same one RCD uses) so the slots run with the right PC. */
            if (op == 0xF6EB) {
                /* RETED — return from interrupt, enable interrupts, delayed.
                 * Pop PC, clear INTM, then run 2 delay slots before jumping. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static uint64_t reted_count;
                    reted_count++;
                    if (reted_count <= 20 || (reted_count % 100) == 0)
                        C54_LOG("RETED-FIRED #%llu PC=0x%04x -> ra=0x%04x SP=0x%04x INTM=0 [XPC non poppe : si fire post-XPC-fix = reopener drain, fix en pending-XPC]",
                                (unsigned long long)reted_count,
                                s->pc, ra, s->sp);
                }
                return consumed + s->lk_used;
            }
            if (op == 0xF69B) {
                /* RETFD — fast return, delayed (no INTM change). */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E2 || op == 0xF6E3) {
                /* CALAD-AT-8353-PROBE (c web review 2026-05-27) : at the
                 * exact site we know self-loops, dump XPC + full A + delay
                 * slot state. CALAD per SPRU172C preserves XPC ; the probe
                 * confirms XPC value at entry (1 → firmware was on far page,
                 * 0 → firmware threw far-pointer at near call). First hit only. */
                if (s->pc == 0x8353) {
                    static int p8353_first = 0;
                    if (!p8353_first) {
                        p8353_first = 1;
                        C54_LOG("PROBE-CALAD-8353-FIRST insn=%u XPC=%u "
                                "A=%010llx (A_G=0x%02x A_H=0x%04x A_L=0x%04x) "
                                "B=%010llx SP=0x%04x PMST=0x%04x",
                                s->insn_count, s->xpc & 0x3,
                                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                                (uint8_t)((s->a >> 32) & 0xFF),
                                (uint16_t)((s->a >> 16) & 0xFFFF),
                                (uint16_t)(s->a & 0xFFFF),
                                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                s->sp, s->pmst);
                    }
                }
                /* BACCD A / CALAD A — delayed branch/call to acc(low).
                 * 1-word op + 2 delay slots. CALAD pushes PC+3 (skip op +
                 * 2 delay slots) per TI convention (cf. CALLD which pushes
                 * PC+4 for its 2-word form). Branch is armed via the
                 * delayed_pc/delay_slots mechanism so the 2 slots run
                 * before PC commits to tgt. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                bool is_call = (op == 0xF6E3);
                static uint64_t bcd_total;
                bcd_total++;
                /* Pre-load context: dump the 8 words preceding PC (in OVLY
                 * the executor reads from DARAM, mirror that). Lets us see
                 * which LD/MAR sequence was supposed to put a valid target
                 * in A before the CALAD/BACCD. */
                int pre_ovly = (s->pmst & PMST_OVLY) && s->pc >= 0x80 && s->pc < 0x2800;
                uint16_t pre[8];
                for (int i = 0; i < 8; i++) {
                    uint16_t a = (uint16_t)(s->pc - 8 + i);
                    pre[i] = pre_ovly ? s->data[a] : s->prog[a];
                }
                if (bcd_total <= 60 || (bcd_total % 5000) == 0) {
                    C54_LOG("BCD/CAD F6E%c #%llu PC=0x%04x tgt=0x%04x A=%010llx SP=0x%04x DP=0x%03x mem[%c PC-8..-1]=%04x %04x %04x %04x %04x %04x %04x %04x%s",
                            is_call ? '3' : '2',
                            (unsigned long long)bcd_total,
                            s->pc, tgt,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            (s->st0 & 0x1FF),
                            pre_ovly ? 'D' : 'P',
                            pre[0], pre[1], pre[2], pre[3],
                            pre[4], pre[5], pre[6], pre[7],
                            is_call ? " CALAD" : " BACCD");
                }
                if (is_call) {
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E4 || op == 0xF6E5) {
                /* FRETD / FRETED — far return, delayed.
                 * Pop XPC + PC unconditionally (FL_FAR). FRETED also clears INTM.
                 * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
                uint16_t ra = data_read(s, s->sp); s->sp++;
                if (op == 0xF6E5) s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E6 || op == 0xF6E7) {
                /* FBACCD A / FCALAD A — far delayed branch/call to A.
                 * A(22:16) → XPC, A(15:0) → tgt. XPC update is immediate
                 * (mirrors FRETED at line ~1639). FCALAD pushes ret PC+3,
                 * and (when APTS) pushes XPC first (so RETF/FRETD pops in
                 * order). 2 delay slots. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                uint8_t  new_xpc = (uint8_t)((s->a >> 16) & 0xFF);
                if (new_xpc > 3) new_xpc &= 3;
                bool is_call = (op == 0xF6E7);
                static uint64_t fbcd_total;
                fbcd_total++;
                if (fbcd_total <= 10 || (fbcd_total % 5000) == 0) {
                    C54_LOG("FBCD/FCAD F6E%c #%llu PC=0x%04x tgt=0x%04x newXPC=%u A=%010llx SP=0x%04x%s",
                            is_call ? '7' : '6',
                            (unsigned long long)fbcd_total,
                            s->pc, tgt, new_xpc,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            is_call ? " FCALAD" : " FBACCD");
                }
                if (is_call) {
                    /* FCALAD (F6E7): push XPC + return PC unconditionally (FL_FAR).
                     * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, s->xpc);
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->xpc         = new_xpc;
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (sub >= 0x8) {
                /* F68x-F6Fx: MVDD Xmem, Ymem — dual data-memory operand move
                 * Encoding: 1111 0110 XXXX YYYY
                 *   bit 7   = Xmod (0=inc, 1=dec)
                 *   bits 6:4 = Xar  (source AR register)
                 *   bit 3   = Ymod (0=inc, 1=dec)
                 *   bits 2:0 = Yar  (dest AR register) */
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t val = data_read(s, s->ar[xar]);
                data_write(s, s->ar[yar], val);
                if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
                if ((op >> 3) & 1) s->ar[yar]--; else s->ar[yar]++;
                return consumed + s->lk_used;
            }
            /* Other F6xx: treat as NOP for now */
            return consumed + s->lk_used;
        }
        /* F5xx: SSBX or RPT #k */
        if (hi8 == 0xF5) {
            /* F5Bx: SSBX -- set bit in ST0 (bit 9=0, bit 8=1).
             * Per tic54x-opc.c: SSBX 0xF5B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF5B0) {
                int bit = op & 0x0F;
                s->st0 |= (1 << bit);
                return consumed + s->lk_used;
            }
            /* Note: 0xF5E2/F5E3 (BACC B / CALA B) are handled earlier alongside
             * their F4 counterparts, so they never reach this F5xx block. */
            /* RPT #k (short immediate) — kept as fallback, must advance PC. */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true; s->rpt_arming = true;
            s->pc += 1;
            return 0;
        }
        /* DIAG: log F7xx executions before the (buggy) LD #k8 dispatch.
         * Per tic54x-opc.c the F7xx range contains SSBX ST1 (0xF7Bx) and
         * other instructions, NOT LD #k8 (which is at E800-E9FF).
         * Caps at 5 per distinct sub-opcode to avoid spam. */
        if (hi8 == 0xF7) {
            static int f7xx_seen[256] = {0};
            int sub_idx = op & 0xFF;
            if (++f7xx_seen[sub_idx] <= 100 || (f7xx_seen[sub_idx] % 1000) == 0) {
                C54_LOG("F7xx EXEC op=0x%04x PC=0x%04x XPC=%d insn=%u",
                        op, s->pc, s->xpc, s->insn_count);
            }
        }
        /* F7Bx: SSBX bit, ST1 (incl. SSBX INTM at F7BB).
         * Per binutils tic54x-opc.c: opcode "ssbx" 0xF5B0 mask 0xFDF0,
         * where bit 9 selects ST0 (0xF5Bx) vs ST1 (0xF7Bx).
         * Symmetric counterpart of RSBX ST1 (F6Bx) handler above.
         * MUST be tested before the F7xx LD #k8 dispatch (which is
         * itself incorrect — per SPRU172C, LD #k8 lives at E800-E9FF). */
        if ((op & 0xFFF0) == 0xF7B0) {
            int bit = op & 0x0F;
            bool is_intm = (bit == 11);
            s->st1 |= (1 << bit);
            if (is_intm)
                C54_LOG("*** SSBX INTM (F7BB) *** PC=0x%04x ST1=0x%04x insn=%u",
                        s->pc, s->st1, s->insn_count);
            return consumed + s->lk_used;
        }
        /* F7xx: LD/ST #k to various registers */
        if (hi8 == 0xF7) {
            uint8_t sub = (op >> 4) & 0xF;
            uint16_t k = op & 0xFF;
            /* F7C0..F7DF = INTR k (handled elsewhere if implemented),
             * F7E0       = RESET (exact opcode, 0xFFFF mask per tic54x-opc.c)
             * F7E1..F7FF = reserved/undefined per SPRU172C.
             *   The old LD #k8 dispatch here corrupted BRC at op=0xF7E3
             *   (= sub 0xE, k=0xE3) inside the DSP idle loop @ PC=0x9b1d,
             *   making RPTB count wrong → DSP stuck in 0x9aXX..0x9bXX
             *   block. Silicon treats reserved opcodes as NOP, not LD. */
            if (sub == 0xE || sub == 0xF) {
                /* F7E0..F7FF : RESET (0xF7E0 exact) + reserved.
                 * Treat as NOP — don't touch BRC. RESET (0xF7E0) would
                 * soft-reset the DSP; if firmware ever issues it we'd
                 * jump to vec 0 = 0xFF80. For now leave as NOP — has
                 * not been observed as a legitimate firmware path. */
                return consumed + s->lk_used;
            }
            if (sub == 0xC || sub == 0xD) {
                /* F7C0-F7DF = INTR k (binutils tic54x-opc.c: intr 0xF7C0 mask
                 * 0xFFE0; SPRU172C INTR). The old qemu-inherited "LD #k8,BK"
                 * (F7Cx) and "LD #k8u,SP" (F7Dx) here were FICTION — the 5110
                 * sleep overlay's wake epilogue `intr 25` (0xF7D9 @0x1D8E-area)
                 * executed as SP=0x00D9, collapsing the stack from ~0x1EC4 and
                 * making the very next `ret` pop 0 -> PC=0 storm (2026-06-12).
                 * Real INTR: software interrupt, taken regardless of INTM/IMR;
                 * pushes the return address, clears the matching IFR bit, sets
                 * INTM, vectors via IPTR. Frame mirrors c54x_interrupt_ex
                 * (no_xpc class pushes PC only). */
                int k5 = op & 0x1F;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                if (!s->no_xpc) {
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, s->xpc);
                    s->xpc = 0;
                }
                if (k5 >= 16) s->ifr &= (uint16_t)~(1u << (k5 - 16));
                s->st1 |= ST1_INTM;
                {
                    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                    s->pc = (uint16_t)((iptr * 0x80) + k5 * 4);
                }
                { static int il = -1; static unsigned in2 = 0;
                  if (il < 0) il = cdbg_env("IRQPAIR") ? 1 : 0;
                  if (il && in2 < 24) { in2++;
                      fprintf(stderr, "[c54x] IRQPAIR entry(INTR) vec=%d pushed ra=0x%04x sp=0x%04x insn=%u\n",
                              k5, (uint16_t)0, s->sp, s->insn_count); } }
                return 0;   /* PC set by vector */
            }
            switch (sub) {
            case 0x0: /* F70x: LD #k8, ASM */
                s->st1 = (s->st1 & ~ST1_ASM_MASK) | (k & ST1_ASM_MASK);
                break;
            case 0x1: /* F71x: LD #k8, AR0 */
                s->ar[0] = k; break;
            case 0x2: /* F72x: LD #k8, AR1 */
                s->ar[1] = k; break;
            case 0x3: s->ar[2] = k; break;
            case 0x4: s->ar[3] = k; break;
            case 0x5: s->ar[4] = k; break;
            case 0x6: s->ar[5] = k; break;
            case 0x7: s->ar[6] = k; break;
            case 0x8: /* F78x: LD #k8, T */
                s->t = (s->st1 & ST1_SXM) ? (uint16_t)(int8_t)k : k; break;
            case 0x9: /* F79x: LD #k8, DP */
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (k & ST0_DP_MASK);
                g_last_ldp_pc = s->pc; g_last_ldp_val = (k & ST0_DP_MASK); g_last_ldp_kind = 1;
                break;
            case 0xA: /* F7Ax: LD #k8, ARP */
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((k & 7) << ST0_ARP_SHIFT); break;
            case 0xB: s->ar[7] = k; break; /* F7Bx: LD #k8, AR7 */
            /* cases 0xC/0xD removed 2026-06-12: F7C0-F7DF is INTR k (handled
             * above). The old "LD #k8,BK" / "LD #k8u,SP" entries were
             * misdecodes; the BK-WR probe hits they logged were INTR opcodes
             * executing as register loads. */
#if 0
            case 0xC: /* F7Cx: LD #k8u, BK */
                {
                    static uint32_t bkw2_n = 0;
                    if (bkw2_n < 40) {
                        fprintf(stderr, "[c54x] BK-WR (F7Cx LD#k) 0x%04x→0x%04x PC=0x%04x "
                                "%s insn=%u\n", s->bk, k, s->pc,
                                (k == 0) ? "<<< BK=0 (casse circular!)" : "", s->insn_count);
                        bkw2_n++;
                    }
                }
                s->bk = k; break;
            case 0xD: sp_abs_track(s, k, 1); s->sp = k; break;  /* LD #k8u, SP */
#endif
            }
            return consumed + s->lk_used;
        }
        /* F9xx encoding split per tic54x-opc.c:
         *   F900-F97F mask FF00 = CC pmad cond (NEAR conditional call)
         *   F980-F9FF mask FF80 = FCALL pmad   (FAR call unconditional)
         * The bit 7 of the opcode low byte distinguishes them. */
        if (hi8 == 0xF9) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALL FAR : push XPC + return PC unconditionally (FL_FAR).
             * Per binutils tic54x-opc.c (fcall 0xF980 mask 0xFF80, FL_FAR)
             * and SPRU172C: FAR call always saves XPC for FRET to restore.
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics).
             * Old behavior caused 281 firmware FCALL FAR sites to push only PC,
             * imbalanced with 142 FRET pop expecting both PC + XPC. */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcall_total;
                fcall_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                if (fcall_total <= 30 || (fcall_total % 5000) == 0) {
                    C54_LOG("FCALL FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x)",
                            (unsigned long long)fcall_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            /* FIX 2026-05-31 : cond décodée depuis l'octet bas (binutils
             * condition_codes[]) via c54x_cond_true(). L'ancien (op>>4)&0xF
             * lisait le mauvais champ → CC[TC/NEQ/LT/...] faux → push manquants
             * power-scan 0xb1xx → over-pop → 0x80fd → self-CALA 0x70c3. */
            bool take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                /* CC leak tracer */
                {
                    static uint32_t cc_targets[64];
                    static uint32_t cc_counts[64];
                    static int cc_n = 0;
                    static uint32_t total_cc = 0;
                    bool found = false;
                    for (int i = 0; i < cc_n; i++) {
                        if (cc_targets[i] == op2) { cc_counts[i]++; found = true; break; }
                    }
                    if (!found && cc_n < 64) { cc_targets[cc_n] = op2; cc_counts[cc_n++] = 1; }
                    if ((++total_cc % 100) == 0) {
                        C54_LOG("F9xx CC TOP TARGETS (SP=0x%04x total=%u):", s->sp, total_cc);
                        for (int i = 0; i < cc_n && i < 10; i++)
                            C54_LOG("  CC→0x%04x count=%u", cc_targets[i], cc_counts[i]);
                    }
                }
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FAxx encoding split per tic54x-opc.c:
         *   FA80-FAFF mask FF80 = FBD pmad (FAR branch delayed)
         *   FA00-FA7F = various NEAR delayed ops (treated as branch). */
        if (hi8 == 0xFA) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            if ((op & 0x80) != 0) {
                /* FBD FAR delayed branch — XPC change, no push */
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fbd_total;
                fbd_total++;
                if (fbd_total <= 30 || (fbd_total % 5000) == 0) {
                    C54_LOG("FBD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u, delayed 2 slots)",
                            (unsigned long long)fbd_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            /* FA00-FA7F = BCD: NEAR conditional delayed branch ("if(cond) dgoto").
             * The legacy lift treated this as "branch ALWAYS" (ignore cond) — a
             * Calypso-GSM-DSP workaround. For the Nokia DCT3 DSP that is WRONG: the
             * resident BIST/audio loops exit via exactly such an `if(cond) dgoto`
             * (e.g. the BIST FIR 0xf170 `bcd 0xf133,alt` and 0xE01D `FA0C if(C)
             * dgoto`), so branch-always makes them loop FOREVER (the DSP spun the
             * whole boot in the 0xf15f FIR — that was the "6-min boot").
             * 2026-06-09: made CONDITIONAL the DEFAULT. The old reason for keeping
             * branch-always was that conditional "derailed into a dead self-spin at
             * DARAM 0x2493" — but that derail was itself a downstream symptom of the
             * dual-operand / MV* / MVPD decode bugs since fixed; with those correct,
             * snapshot-replay from the BIST loop now evaluates the condition and the
             * FIR EXITS cleanly (advances 0xf15f→0xDE9x, no 0x2493 derail). Opt back
             * into the legacy branch-always with CALYPSO_FA_ALWAYS=1 for A/B. */
            static int fa_cond = -1;
            if (fa_cond < 0) fa_cond = getenv("CALYPSO_FA_ALWAYS") ? 0 : 1;
            if (!fa_cond) { s->pc = op2; return 0; }
            bool fa_take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
            if (fa_take) { s->delayed_pc = op2; s->delay_slots = 2; }
            return consumed + s->lk_used;
        }
        /* FBxx encoding split per tic54x-opc.c:
         *   FB80-FBFF mask FF80 = FCALLD pmad (FAR call delayed)
         *   FB00-FB7F mask FF00 = CCD pmad cond (NEAR conditional call delayed) */
        if (hi8 == 0xFB) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALLD FAR : push XPC + return PC+4 unconditionally (FL_FAR delayed).
             * Per binutils (fcalld 0xFB80 mask 0xFF80, FL_FAR|FL_DELAY).
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcalld_total;
                fcalld_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                if (fcalld_total <= 30 || (fcalld_total % 5000) == 0) {
                    C54_LOG("FCALLD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x, delayed)",
                            (unsigned long long)fcalld_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            /* FIX 2026-05-31 : cond décodée depuis l'octet bas via
             * c54x_cond_true() (cf CC ci-dessus). */
            bool take = c54x_cond_true(s, (uint8_t)(op & 0x7F));
            if (take) {
                /* FIX 2026-05-31 : CCD est DIFFÉRÉ — arme delay_slots=2 +
                 * delayed_pc (comme CALLD f274, fixé 2026-05-30), au lieu de
                 * sauter immédiatement (s->pc=op2; return 0) qui SKIPPAIT les
                 * 2 delay-slots → push perdu si un slot pousse → over-pop →
                 * 0x80fd → self-CALA 0x70c3. Retour poussé = pc+4 (past CCD +
                 * 2 slots). Not-taken : PC avance de consumed (2) = past CCD. */
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            return consumed + s->lk_used;
        }
        /* FCxx: LD #k, 16, B */
        /* FCxx: RC cond / RET -- return conditional (1-word).
         * Per tic54x-opc.c: RET=0xFC00, RC=0xFC00 mask 0xFF00. */
        if (hi8 == 0xFC) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                {
                    static int rc_log = 0;
                    if (rc_log < 50)
                        C54_LOG("RC/RET PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x",
                                s->pc, cc, ra, s->sp);
                    rc_log++;
                }
                /* POST-BOOTSTUB-RET : si on est en train de RET depuis le
                 * boot stub (PC ∈ 0x0000..0x0008), c'est la sortie du
                 * task-switch trampoline 0x701b/0x701d → 0x0000. Le ra
                 * poppé est le PC du task qui prend le contrôle. À insn≈90.2M
                 * (dernière transition INTM), ce PC = le task qui ne clear
                 * jamais INTM ensuite. */
                if (s->pc <= 0x0008) {
                    static unsigned bsr;
                    bsr++;
                    if (bsr <= 200 || (bsr % 50) == 0) {
                        fprintf(stderr,
                                "[c54x] POST-BOOTSTUB-RET #%u PC=0x%04x -> task=0x%04x "
                                "SP_new=0x%04x B=0x%010llx INTM=%d insn=%u\n",
                                bsr, s->pc, ra, s->sp,
                                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                !!(s->st1 & ST1_INTM), s->insn_count);
                    }
                    /* DEEP-TRAIL : pour les 5 premiers POST-BOOTSTUB-RET,
                     * dump pc_ring[-64..-1] pour révéler le caller chain
                     * qui mène au stack-underflow loop. Gated par
                     * CALYPSO_DEBUG=BOOTSTUB_TRAIL. */
                    if (bsr <= 5 && calypso_debug_enabled("BOOTSTUB_TRAIL")) {
                        fprintf(stderr,
                            "[c54x] BOOTSTUB DEEP-TRAIL #%u (last 64 PCs):\n",
                            bsr);
                        for (int row = 0; row < 8; row++) {
                            fprintf(stderr, "[c54x] BS-DEEP[%3d..%3d] :",
                                    -64 + row*8, -57 + row*8);
                            for (int col = 0; col < 8; col++) {
                                int idx = -64 + row*8 + col;
                                fprintf(stderr, " %04x",
                                    pc_ring[(pc_ring_idx + idx) & 255]);
                            }
                            fprintf(stderr, "\n");
                        }
                        /* Dump aussi 16 valeurs sur la pile à partir de SP. */
                        fprintf(stderr, "[c54x] BS-DEEP stack[SP..SP+15] :");
                        for (int i = 0; i < 16; i++) {
                            fprintf(stderr, " %04x",
                                s->data[(s->sp + i) & 0xFFFF]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
                s->pc = ra;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FExx: RCD cond / RETD -- return conditional delayed (1-word).
         * Per tic54x-opc.c: RETD=0xFE00, RCD=0xFE00 mask 0xFF00.
         * Simplified: immediate return (delay slots skipped). */
        if (hi8 == 0xFE) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                /* RCD is *delayed*: per SPRU172C the next 2 instructions
                 * after RCD execute before the return takes effect. The
                 * old "skip delay slots" implementation broke FB-detection
                 * because slots like `LD #0, B` at PROM0 0x75ea were never
                 * run, leaving accumulator state stale and the dispatcher
                 * at 0x7700 looping forever.
                 *
                 * Fix: arm the existing delayed_pc/delay_slots machinery —
                 * pop the return address now, advance PC normally so the
                 * next 2 instructions execute as delay slots, then the
                 * main loop forces PC = delayed_pc. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static int rcd_log = 0;
                    if (rcd_log < 50)
                        C54_LOG("RCD/RETD PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x (delayed)",
                                s->pc, cc, ra, s->sp);
                    rcd_log++;
                }
                return consumed + s->lk_used;
            }
            return consumed + s->lk_used;
        }
        /* FFxx is XC 2,cond — handled above with FDxx. No ADD here. */
        goto unimpl;

    case 0xE:
        /* Exxxx: single-word ALU, status, misc */
        /* CMPS src, Smem — Compare, Select, and Store (Viterbi)
         * Encoding: 1110 00SD IAAAAAAA (1 word)
         * Per SPRU172C p.4-35: if |A(32-16)| >= |Smem| then TC=1,
         * TRN = (TRN<<1)|1, dst=A; else TC=0, TRN=(TRN<<1), dst=Smem<<16 */
        if ((op & 0xFC00) == 0xE000) {
            int src_s = (op >> 9) & 1;
            int dst_d = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t acc = src_s ? s->b : s->a;
            int32_t ah = (int32_t)((acc >> 16) & 0xFFFF);
            if (ah < 0) ah = -ah;
            int32_t sv = (int16_t)val;
            if (sv < 0) sv = -sv;
            s->trn <<= 1;
            if (ah >= sv) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                int64_t nv = (int64_t)(int16_t)val << 16;
                if (dst_d) s->b = sext40(nv); else s->a = sext40(nv);
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFE00) == 0xEA00) {
            /* EAxx: LD #k9, DP — Load Data Page pointer (1-word).
             * Per tic54x-opc.c: ld 0xEA00 mask 0xFE00, 1 word. */
            uint16_t k9 = op & 0x01FF;
            uint16_t old_dp = s->st0 & ST0_DP_MASK;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            g_last_ldp_pc = s->pc; g_last_ldp_val = k9; g_last_ldp_kind = 2;
            {
                static uint64_t dpc;
                dpc++;
                if (dpc <= 80 || (dpc % 5000) == 0 || k9 == 0x83) {
                    C54_LOG("DP-SET EAxx #%llu PC=0x%04x DP 0x%03x → 0x%03x %s",
                            (unsigned long long)dpc, s->pc,
                            old_dp, k9,
                            k9 == 0x83 ? "*** 0x83 (CALAD-zone base 0x4180) ***" : "");
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEC) {
            /* ECxx: RPT #k8u — repeat next instruction k8u+1 times.
             * Per tic54x-opc.c: rpt 0xEC00 mask 0xFF00, single word.
             * Must advance PC past RPT now and return 0 so the dispatcher
             * re-executes the NEXT instruction (not RPT itself). */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true; s->rpt_arming = true;
            s->pc += 1;
            return 0;
        }
        if (hi8 == 0xE5) {
            /* E5xx: MVDD Xmem, Ymem  (per tic54x-opc.c, NOT MVMM)
             * 1-word, 2-cycle dual-operand data-to-data move:
             *   *Ymem = *Xmem
             * Per tic54x.h:
             *   XMEM = (op & 0xF0) >> 4
             *   YMEM = op & 0x0F
             *   XMOD/YMOD = (nibble & 0xC) >> 2  (0=*AR,1=*AR-,2=*AR+,3=*AR+0%)
             *   XARX/YARX = (nibble & 0x3) + 2   (AR2..AR5 only) */
            uint8_t xnib = (op >> 4) & 0xF;
            uint8_t ynib = op & 0xF;
            int xar = (xnib & 0x3) + 2;
            int yar = (ynib & 0x3) + 2;
            int xmod = (xnib & 0xC) >> 2;
            int ymod = (ynib & 0xC) >> 2;
            uint16_t xa = s->ar[xar];
            uint16_t ya = s->ar[yar];
            uint16_t v = data_read(s, xa);
            data_write(s, ya, v);
            /* Post-modify both ARs per their mod field */
            switch (xmod) {
                case 0: break;                        /* *AR     */
                case 1: s->ar[xar] = xa - 1; break;   /* *AR-    */
                case 2: s->ar[xar] = xa + 1; break;   /* *AR+    */
                case 3: s->ar[xar] = c54x_circ_ref(xa, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ modulo BK — fix 2026-06-01 */
            }
            switch (ymod) {
                case 0: break;
                case 1: s->ar[yar] = ya - 1; break;
                case 2: s->ar[yar] = ya + 1; break;
                case 3: s->ar[yar] = c54x_circ_ref(ya, +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ modulo BK — fix 2026-06-01 */
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE4) {
            /* E4xx: BITF Smem, #lk (2-word) or BIT Smem, bit */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t val = data_read(s, addr);
            s->st0 = (val & op2) ? (s->st0 | ST0_TC) : (s->st0 & ~ST0_TC);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE7) {
            /* E7xx: MVMM mmrx, mmry  (per tic54x-opc.c)
             * 1-word, 2-cycle, MMR-to-MMR move using a constrained set
             * (MMRX/MMRY operand types). */
            int src = (op >> 4) & 0xF;
            int dst = op & 0xF;
            uint16_t val;
            if (src <= 7) val = s->ar[src];
            else if (src == 8) val = s->sp;
            else val = data_read(s, src + 0x10);
            if (dst <= 7) s->ar[dst] = val;
            else if (dst == 8) { sp_abs_track(s, val, 2); s->sp = val; }  /* MVMM SP-dest */
            else data_write(s, dst + 0x10, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE8 || hi8 == 0xE9) {
            /* E8xx/E9xx: LD #k8u, dst — Load 8-bit unsigned immediate (1-word).
             * Per tic54x-opc.c: ld 0xE800 mask 0xFE00.
             * bit 8 = dst (0=A, 1=B), bits 7:0 = k8u.
             * NOTE: This was previously decoded as CC (conditional call, 2-word)
             * which caused stack overflow by pushing return addresses in a loop. */
            int dst = (op >> 8) & 1;
            uint8_t k = op & 0xFF;
            /* LD #K, dst loads the 8-bit immediate RIGHT-JUSTIFIED into the accumulator
             * low word (bits 0-15), sign-extended through the guard under SXM — NOT shifted
             * into the high word. The old `v << 16` placed K at bit16, so e.g. `LD #5,B`
             * (the COBBA register select at 0x4665) left BL=0 -> port 0x2C select wrote reg0
             * instead of reg5/reg6, breaking the cmd-0x70 self-test ADC reads. Disasm renders
             * E905 as `B = #5h` (no shift), confirming the right-justified load. */
            int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int8_t)k : (int64_t)k;
            if (dst) s->b = sext40(v);
            else     s->a = sext40(v);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE1) {
            /* E1xx: single-word acc ops — NEG, ABS, CMPL, SAT, EXP, etc. */
            uint8_t sub = op & 0xFF;
            switch (sub) {
            case 0xE0: s->a = ~s->a; s->a = sext40(s->a); break;  /* CMPL A */
            case 0xE1: s->b = ~s->b; s->b = sext40(s->b); break;  /* CMPL B */
            case 0xE2: s->a = -s->a; s->a = sext40(s->a); break;  /* NEG A */
            case 0xE3: s->b = -s->b; s->b = sext40(s->b); break;  /* NEG B */
            case 0xE4: /* SAT A */ if (s->st0 & ST0_OVA) s->a = (s->a < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE5: /* SAT B */ if (s->st0 & ST0_OVB) s->b = (s->b < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE8: /* ABS A */ s->a = (s->a < 0) ? -s->a : s->a; s->a = sext40(s->a); break;
            case 0xE9: /* ABS B */ s->b = (s->b < 0) ? -s->b : s->b; s->b = sext40(s->b); break;
            case 0xEA: /* ROR A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & 1) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a >> 1) | ((int64_t)c << 39); s->a = sext40(s->a); } break;
            case 0xEB: /* ROL A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & ((int64_t)1<<39)) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a << 1) | c; s->a = sext40(s->a); } break;
            default:
                /* EXP A/B etc — return 0 for now */
                break;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEF) {
            /* EFxx: RPTZ dst, #lk — Zero accumulator and repeat (2 words)
             * Per SPRU172C: dst = 0; RPT #lk
             * Encoding: 1110 1111 xxxx xxxx + lk_word */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int rptz_dst = (op >> 0) & 1;
            if (rptz_dst) s->b = 0; else s->a = 0;
            s->rpt_count = op2;
            s->rpt_active = true; s->rpt_arming = true;
            s->pc += 2;
            return 0;
        }
        if (hi8 == 0xEB) {
            /* EBxx: RPTB[D] pmad — Block repeat (2 words)
             * Per SPRU172C: REA = pmad, RSA = PC+2, BRAF=1 */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE6) {
            /* E6xx: SFTA/SFTL acc, #shift (single-word immediate shift) */
            int shift = op & 0x1F;
            if (shift & 0x10) shift |= ~0x1F;  /* sign extend 5-bit */
            int dst = (op >> 5) & 1;
            int logical = (op >> 6) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            if (logical) {
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else {
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEE) {
            /* FRAME #k8 — stack-frame pointer adjust : SP = SP + sign_ext(k8).
             * Per tic54x-opc.c { "frame", 1,1,1, 0xEE00, 0xFF00, {OP_k8} }
             * = 1 MOT, et SPRU172C §4 (FRAME adjusts SP by a signed 8-bit imm).
             * FRAME -N alloue le cadre (SP descend) ; FRAME +N le libère.
             *
             * BUG FIX 2026-05-31 : ce handler décodait 0xEExx en "BCD pmad,cond"
             * (branche conditionnelle différée 2-MOTS) — FAUX sur les 2 plans :
             *   (1) longueur : 1 mot, pas 2 → désync de tout l'aval
             *   (2) sémantique : SP+=k8, pas une branche
             * BCD n'existe même pas en 0xEE (le vrai bc=0xF8, cc=0xF9, bcd=0xFA).
             * 124 sites 0xEExx en PROM0, dont le chemin de boot (paires
             * FRAME #-1/#+1 = prologue/épilogue). Le SP jamais ajusté → over-pop
             * (SP-EVENTS pops>pushes) → POPM ST0 @0x94f3 ramasse l'orphelin
             * 0x80fd → DP=0x0fd → dispatcher LUT garbage → self-CALA 0x70c3 →
             * écrit 0x70c4 (=28868) dans d_fb_det/a_pm → rxlev/TOA poison.
             * cf doc/SP_CATASTROPHE_70c4_SEQUENCE.md. */
            int8_t k = (int8_t)(op & 0xFF);
            s->sp = (uint16_t)(s->sp + k);
            return consumed + s->lk_used;
        }
        if ((op & 0xFFE0) == 0xED00) {
            /* ED00-ED1F: LD #k5, ASM — load 5-bit immediate into ASM field of ST1.
             * Per tic54x-opc.c: ld 0xED00 mask 0xFFE0, 1 word.
             * NOT BCD (which is 0xFA00 mask 0xFF00). */
            uint8_t k5 = op & 0x1F;
            s->st1 = (s->st1 & ~ST1_ASM_MASK) | k5;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xED) {
            /* EDxx (not ED00-ED1F): BCD pmad, cond (conditional branch delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x00) take = true;            /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x6: case 0x7:
        /* 7Exx: READA Smem — read prog[A_low] → data[Smem]
         * Per tic54x-opc.c: reada 0x7E00 mask 0xFF00 (1 word).
         * Per SPRU131G : program address = (XPC[6:0] | A[15:0]). A.high is
         * NOT used as XPC source — XPC reg is. prog_read already implements
         * this via c54x_prog_xlate for addr ≥ 0x8000.
         * Under RPT, the prog address auto-increments each iteration;
         * accumulator A is preserved (we mirror via mvpd_src state).
         *
         * 2026-05-27 c web review revert : a speculative 23-bit fix
         * (A.high → XPC override) was tried but contradicts SPRU131G and
         * did not move the symptom — reverted to canonical semantics. */
        if (hi8 == 0x7E) {
            addr = resolve_smem(s, op, &ind);
            /* SPRU172C: A -> PAR on the FIRST execution, repeat HW increments
             * PAR (mvpd_src). par_set marks "PAR already seeded this repeat";
             * it is cleared at rpt arming + rpt end. Without the seed the
             * first iteration used the STALE PAR of the previous READA/WRITA
             * (demand-page id-8 overlay landed at the prior copy's end). */
            uint16_t psrc = (s->rpt_active && s->par_set)
                            ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            if (s->rpt_active) s->par_set = true;
            uint16_t v = prog_read(s, psrc);
            data_write(s, addr, v);
            s->mvpd_src = psrc + 1;
            { static int reada_log = 0; if (reada_log++ < 20)
                C54_LOG("READA: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x rpt=%d insn=%u",
                        psrc, v, addr, s->pc, s->rpt_count, s->insn_count); }
            return consumed + s->lk_used;
        }
        /* 7Fxx: WRITA Smem — write data[Smem] → prog[A_low] (mirror of READA) */
        if (hi8 == 0x7F) {
            addr = resolve_smem(s, op, &ind);
            /* Same PAR-seeding protocol as READA above (A -> PAR first pass). */
            uint16_t pdst = (s->rpt_active && s->par_set)
                            ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            if (s->rpt_active) s->par_set = true;
            prog_write(s, pdst, data_read(s, addr));
            s->mvpd_src = pdst + 1;
            return consumed + s->lk_used;
        }
        /* 6Dxx: MAR Smem — modify address register (side effects only) */
        if (hi8 == 0x6D) {
            addr = resolve_smem(s, op, &ind);
            /* MAR only modifies AR via addressing mode, no data access */
            return consumed + s->lk_used;
        }
        /* 76xx: ST #lk, Smem  (2 or 3 words) — store 16-bit literal to data
         * memory. Per binutils tic54x-opc.c {st, 2,2,2, 0x7600, 0xFF00,
         * {OP_lk, OP_Smem}} and tic54x-dis.c get_insn_size = words +
         * has_lkaddr (extra word when Smem mode in 0xC..0xF).
         *
         * Encoding (verified via tic54x-dis.c:192-204):
         *   word 0 = opcode (0x76xx)
         *   word 1 = lkaddr  (Smem extension, only if mode in 0xC..0xF)
         *   word N = opcode2 (the #lk value being stored, last extension)
         *
         * Was previously misdecoded as LDM MMR,dst (1 word) — copy/paste
         * of the wrong mnemonic. The real LDM is 0x48xx mask 0xFE00,
         * already correctly handled in the 0x4 group. Misdecoding caused
         * PC to advance by 1 instead of 2-3 ; the literal then executed
         * as a stray opcode. In particular the 0x4F00 (DST B,Lmem with
         * DP=0 → MMR_IMR) stray write zeroed IMR forever, masking
         * INT3+BRINT0 → DSP parked in RPTB at e9ab..e9b6 awaiting a
         * frame interrupt that was never serviced. Fix 2026-05-08. */
        /* 74xx: PORTR PA, Smem — read I/O port PA into Smem.
         * Nokia DCT3 host mailbox (PA = port 1/2/3 etc). The bbaranoff core never
         * implemented real PORTR (Calypso used DMA/BSP); our DSP needs it. */
        if (hi8 == 0x74) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));   /* PA */
            consumed = 2;
            g_c54x_pmap_handled = 0;
            uint16_t v = (op2 < 0x0100 && nokia_port_read_hook)
                         ? nokia_port_read_hook(op2, (uint16_t)s->pc) : 0;
            c54x_pmap_record(0, 0, op2, v, s->pc, s->insn_count, g_c54x_pmap_handled);
            data_write(s, addr, v);
            return consumed + s->lk_used;
        }
        /* 75xx: PORTW Smem, PA — write Smem to I/O port PA. */
        if (hi8 == 0x75) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));   /* PA */
            consumed = 2;
            uint16_t v = data_read(s, addr);
            g_c54x_pmap_handled = 0;
            if (op2 < 0x0100 && nokia_port_write_hook)
                nokia_port_write_hook(op2, v, (uint16_t)s->pc);
            c54x_pmap_record(0, 1, op2, v, s->pc, s->insn_count, g_c54x_pmap_handled);
            return consumed + s->lk_used;
        }
        if (hi8 == 0x76) {
            static unsigned hit76_log;
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (hit76_log++ < 30) {
                if (calypso_debug_enabled("HIT-76")) fprintf(stderr,
                        "[c54x] HIT-76 PC=0x%04x op=0x%04x addr=0x%04x "
                        "lk=0x%04x lk_used=%d insn=%u\n",
                        s->pc, op, addr, op2, s->lk_used, s->insn_count);
            }
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        /* 77xx: STM #lk, MMR (2 words) */
        if (hi8 == 0x77) {
            uint8_t mmr = op & 0x7F;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* WATCH-ST1-WRITE : MMR 0x07 = ST1. Capture toutes les
             * écritures de ST1 (STM #lk, ST1) — incluant celles qui
             * ne changent pas la valeur d'INTM mais redéfinissent
             * tout le mot ST1. Sortie : valeur écrite, bit 11 (INTM),
             * delta vs current ST1. Cap 200 entries pour boot, puis
             * sample 1/100. */
            if (mmr == 0x07) {
                static unsigned st1w;
                st1w++;
                if (st1w <= 200 || (st1w % 100) == 0) {
                    int new_intm = !!(op2 & (1 << 11));
                    int cur_intm = !!(s->st1 & ST1_INTM);
                    if (calypso_debug_enabled("ST1-WR")) fprintf(stderr,
                            "[c54x] ST1-WR #%u STM #0x%04x,ST1 PC=0x%04x "
                            "cur=0x%04x->0x%04x INTM:%d->%d insn=%u XPC=%d\n",
                            st1w, op2, s->pc, s->st1, op2,
                            cur_intm, new_intm, s->insn_count, s->xpc);
                }
            }
            data_write(s, mmr, op2);
            return consumed + s->lk_used;
        }
        /* 0x70-0x73 MV* family — data↔data / data↔MMR moves, ALL 2-word
         * (opcode + dmad, + Smem-long lk for the Smem variants). Per tic54x-opc.c:
         *   0x70 MVKD dmad,Smem   Smem = (dmad)            [prog→? no: data→data]
         *   0x71 MVDK Smem,dmad   (dmad) = Smem
         *   0x72 MVDM dmad,MMR    MMR  = (dmad)
         *   0x73 MVMD MMR,dmad    (dmad) = MMR
         * 2026-06-09 FIX (whole family at once): these had NO handler and fell into
         * the 0x7000 STL catch-all below = 1-word → the dmad operand ran as a stray
         * instruction → desync. The 2026-06-02 note (kept in git history) found
         * MVDM@0xf564 mis-decoded and that fixing 0x72 ALONE regressed (AR3
         * out-of-buffer @0xee38) because the AR3 setup is ALSO an MV* op that stayed
         * mis-decoded — i.e. a COMPENSATING pair. Hypothesis: fixing the whole
         * 0x70-0x73 family together corrects both sides. Verified non-desyncing via
         * tools/dsp/decode_audit_diff.py. Same root as the 0x7C/0x7D MVPD/MVDP fix. */
        if (hi8 == 0x72 || hi8 == 0x73) {       /* MMR variants: opcode + dmad */
            uint16_t mmr  = op & 0x7F;           /* MMRs alias data addr 0x00..0x1F */
            uint16_t dmad = prog_fetch(s, (uint16_t)(s->pc + 1));
            if (hi8 == 0x72) data_write(s, mmr,  data_read(s, dmad));  /* MVDM (dmad)->MMR */
            else             data_write(s, dmad, data_read(s, mmr));   /* MVMD MMR->(dmad) */
            consumed = 2;
            return consumed + s->lk_used;        /* lk_used==0 here (no Smem) */
        }
        if (hi8 == 0x70 || hi8 == 0x71) {       /* Smem variants: opcode + dmad (+Smem-long) */
            addr = resolve_smem(s, op, &ind);    /* Smem (sets s->lk_used for long form) */
            uint16_t dmad = prog_fetch(s, (uint16_t)(s->pc + 1 + (s->lk_used ? 1 : 0)));
            if (hi8 == 0x70) data_write(s, addr, data_read(s, dmad));  /* MVKD (dmad)->Smem */
            else             data_write(s, dmad, data_read(s, addr));  /* MVDK Smem->(dmad) */
            consumed = 2;
            return consumed + s->lk_used;
        }
        /* LD / ST operations */
        if ((op & 0xF800) == 0x7000) {
            /* 70xx: STL src, Smem */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x7C00 MVPD pmad,Smem  (Smem = Pmem[pmad], program→data)
         * 0x7D00 MVDP Smem,pmad  (Pmem[pmad] = Smem, data→program)
         * Per tic54x-opc.c both are { ..,2,2,2, 0x7C00/0x7D00, 0xFF00,
         * {OP_pmad,OP_Smem} } = opcode + pmad word, + 1 more for the long-Smem
         * (absolute *(0xNNNN)) form. 2026-06-09 FIX: these had NO handler and
         * fell into the 0x7800 "STH" catch-all below, which decoded them as
         * 2-word STH. The long-Smem form is 3 words (opcode + Smem addr + pmad);
         * dropping the pmad word left it to run as a STRAY instruction. At
         * loader1's finalize (0xf67 `mvpd 0xff87,*(0x801)`, only reached at
         * insn~482M when the cold upload completes) that desynced the stream
         * into the CRC-seed routine → ret@0xf73 over-pop → cascade into run-mode
         * → 0x940d retd→0xf074 → AR5=0 → IMR=0. MUST precede the 0x7800 group. */
        if (hi8 == 0x7C || hi8 == 0x7D) {
            addr = resolve_smem(s, op, &ind);   /* sets s->lk_used for long Smem */
            uint16_t pmad = prog_fetch(s, (uint16_t)(s->pc + 1 + (s->lk_used ? 1 : 0)));
            /* SPRU172C: under RPT the program address auto-increments each iteration
             * (pmad seeds PAR on the first pass; HW then increments PAR). Without this,
             * `RPT #n; MVPD pmad,*AR+` re-reads the SAME immediate every pass and copies
             * one source word n times instead of a contiguous block. Mirror READA/WRITA's
             * PAR seeding via s->mvpd_src (par_set is cleared at rpt arm + rpt end). A
             * non-repeated MVPD is unaffected (rpt_active==0 → psrc = pmad). Fix 2026-07-07. */
            uint16_t psrc = (s->rpt_active && s->par_set) ? s->mvpd_src : pmad;
            if (s->rpt_active) s->par_set = true;
            if (hi8 == 0x7C) data_write(s, addr, prog_fetch(s, psrc)); /* MVPD P→D */
            else             s->prog[psrc] = data_read(s, addr);       /* MVDP D→P */
            s->mvpd_src = psrc + 1;
            consumed = 2;  /* opcode + pmad ; long-Smem extra added via lk_used */
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x7800) {
            /* 78xx-7Bxx (+ legacy 7C-7F before the MVPD/MVDP/READA/WRITA
             * handlers above): STH src, Smem.
             * Note: BANZ (0x78xx per doc) shares this range but is handled
             * via F84x (BANZ with condition) in the F8xx group. */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x6000-0x60FF: CMPM Smem, lk  (compare memory with long immediate)
         * Per tic54x-opc.c: { "cmpm", 2,2,2, 0x6000, 0xFF00 }
         * Sets TC = (data[Smem] == lk).
         *
         * The DSP bootloader at PROM0 0xb41c / 0xb424 polls
         *   CMPM *(0x0fff), 4   →  CMPM *(0x0fff), 2
         * to wait for ARM-side BL_CMD_STATUS write. Without TC being set
         * the subsequent BC NTC always branches back, looping forever.
         * Was previously folded into the generic 0x6000-0x67FF "LD" path
         * which set the accumulator instead and never updated TC. */
        if ((op & 0xFF00) == 0x6000) {
            addr = resolve_smem(s, op, &ind);
            uint16_t cmp_val = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            if (mem_val == cmp_val) s->st0 |= ST0_TC;
            else                    s->st0 &= ~ST0_TC;
            consumed = 2;  /* opcode + cmp_val (smem extra lk added via lk_used) */
            return consumed + s->lk_used;
        }
        /* 0x6100-0x61FF: BITF Smem, lk — bit-field test, TC = (Smem & lk)!=0 */
        if ((op & 0xFF00) == 0x6100) {
            addr = resolve_smem(s, op, &ind);
            uint16_t mask = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            bool tc_before = (s->st0 & ST0_TC) != 0;
            if (mem_val & mask) s->st0 |= ST0_TC;
            else                s->st0 &= ~ST0_TC;
            bool tc_after = (s->st0 & ST0_TC) != 0;
            consumed = 2;
            /* FBWATCH : capture EXACTE au site de poll foreground (0xf7af/0xf7b7)
             * — addr résolue + valeur data_read + TC. ID le flag jamais vrai. */
            if (g_fbwatch_on > 0 && (s->pc == 0xf7af || s->pc == 0xf7b7)
                && s->insn_count > 100000) {   /* post-wire (1er wire @insn 32768) */
                static unsigned wbf = 0;
                if (wbf++ < 30)
                    fprintf(stderr, "[c54x] FBWATCH-BITF pc=0x%04x addr=0x%04x "
                            "mem=0x%04x mask=0x%04x -> TC=%d insn=%u\n",
                            s->pc, addr, mem_val, mask, tc_after, s->insn_count);
            }
            /* BITF instrumentation (2026-05-15 nuit) — pour confirmer si TC
             * est set correctement. Hypothèse : si BITF appelle souvent mais
             * tc_after=1 rarement → masque/mem_val pattern empêche TC=1,
             * ce qui fait que BC NTC branche toujours et `ST #1, d_task_d`
             * à PROM 0x9ab1 n'est jamais atteint. Format :
             *   BITF-PROBE #N PC=0xXXXX addr=0xXXXX mem=0xXXXX mask=0xXXXX
             *               tc_before=N tc_after=N
             * Cap 200 + 1/1000 ensuite. */
            {
                static uint64_t bitf_total;
                static uint64_t bitf_tc_set;
                static uint64_t bitf_tc_clear;
                bitf_total++;
                if (tc_after) bitf_tc_set++;
                else          bitf_tc_clear++;
                if (bitf_total <= 200 || (bitf_total % 1000) == 0) {
                    if (calypso_debug_enabled("BITF-PROBE")) fprintf(stderr,
                            "[c54x] BITF-PROBE #%llu PC=0x%04x addr=0x%04x "
                            "mem=0x%04x mask=0x%04x tc_before=%d tc_after=%d "
                            "(total=%llu set=%llu clear=%llu)\n",
                            (unsigned long long)bitf_total, s->last_exec_pc,
                            addr, mem_val, mask, tc_before, tc_after,
                            (unsigned long long)bitf_total,
                            (unsigned long long)bitf_tc_set,
                            (unsigned long long)bitf_tc_clear);
                }
            }
            return consumed + s->lk_used;
        }
        /* 0x6200-0x67FF: MPY / MAC Smem, #lk, dst (2-word: opcode + 16-bit lk).
         * AUDIT FIX 2026-06-12 (decode_audit_diff vs objdump 2.42): these fell into
         * the 1-word LD fallback below -> the #lk constant ran as a stray instruction
         * -> stream desync. Surfaced once the corrected page chain loaded the task
         * overlays: the FIR/filter code at 0xF100-0xF800 (e.g. 0x6421 @0xF157) desynced
         * into a spin at 0xF975, so the busy superloop never re-reached the MDISND drain
         * (docs/research/dsp-demand-page-chain.md). bits[10:8]: 2 MPY->A, 3 MPY->B,
         * 4 MAC A->A, 5 MAC A->B, 6 MAC B->A, 7 MAC B->B (src=bit9, dst=bit8). */
        if ((op & 0xF800) == 0x6000 && ((op >> 8) & 0x7) >= 2) {
            int sub = (op >> 8) & 0x7;
            addr = resolve_smem(s, op, &ind);
            uint16_t mem = data_read(s, addr);
            uint16_t lk  = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            int64_t prod = (int64_t)(int16_t)mem * (int64_t)(int16_t)lk;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (op >> 8) & 1;                          /* bit8 */
            int64_t r;
            if (sub < 4) r = prod;                            /* MPY: dst = Smem*lk    */
            else {                                            /* MAC: dst = src + ...  */
                int src = (op >> 9) & 1;                      /* bit9                  */
                r = sext40(src ? s->b : s->a) + prod;
            }
            if (dst) s->b = sext40(r); else s->a = sext40(r);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6000) {
            /* 60xx/61xx residual: LD Smem, dst (sub 0/1 after CMPM/BITF handled above) */
            int dst_acc = (op >> 9) & 1;
            int shift = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            if (shift) v <<= 16;  /* LD Smem, 16, dst */
            if (dst_acc) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* 0x6800-0x6BFF + 0x6Cxx + 0x6Exx: companion to the 0x6F00 fix below.
         * Per binutils tic54x-opc.c (verified against insn_template struct):
         *   0x6800 ANDM  #lk, Smem      data[Smem] = data[Smem] & lk     (2-word)
         *   0x6900 ORM   #lk, Smem      data[Smem] = data[Smem] | lk     (2-word)
         *   0x6A00 XORM  #lku, Smem     data[Smem] = data[Smem] ^ lku    (2-word)
         *   0x6B00 ADDM  #lk, Smem      data[Smem] = data[Smem] + lk     (2-word)
         *   0x6C00 BANZ  pmad, Sind     if (ARx != 0) PC = pmad          (2-word)
         *   0x6E00 BANZD pmad, Sind     same as BANZ but with 2 delay slots
         *
         * Without these, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes them all as LD Smem,T (1-word), causing PC drift +1
         * word and the lk/pmad operand executing as parasitic instruction.
         * 1259 (ANDM/ORM/XORM/ADDM) + 304 (BANZ/BANZD) = 1563 sites in ROM.
         *
         * 2026-04-28 — companion fix to 0x6F00 already inserted below.
         * See doc/opcodes/0x68_0x6F.md for spec. */
        if ((op & 0xFF00) == 0x6800) {
            /* ANDM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v & lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6900) {
            /* ORM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v | lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6A00) {
            /* XORM #lku, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lku = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v ^ lku);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6B00) {
            /* ADDM #lk, Smem — add signed lk to memory (wrap mod 2^16) */
            addr = resolve_smem(s, op, &ind);
            int16_t lk = (int16_t)prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, (uint16_t)((int16_t)v + lk));
            consumed = 2;
            /* TODO: TC/OVM/SXM flag effects per SPRU172C (verify) */
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6C00) {
            /* BANZ pmad, Sind — branch if ARx (selected by ARF in op[2:0])
             * is non-zero. Test on PRE-modify value; resolve_smem applies
             * post-mod regardless of branch outcome. Previously read ARP
             * from ST0 (the PREVIOUS instruction's nar) — wrong AR was
             * tested. Cf resolve_smem comment for the off-by-ARP bug. */
            int nar = op & 0x07;
            uint16_t pre = s->ar[nar];
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (pre != 0) {
                s->pc = pmad;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6E00) {
            /* BANZD pmad, Sind — delayed BANZ (2 slots after the 2-word op).
             * Same off-by-ARP fix as BANZ above. */
            int nar = op & 0x07;
            uint16_t pre = s->ar[nar];
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (pre != 0) {
                s->delayed_pc  = pmad;
                s->delay_slots = 2;
            }
            return consumed + s->lk_used;
        }
        /* 0x6F00-0x6FFF: Extended ADD/SUB/LD/STH/STL Smem, SHIFT, DST/SRC (2-word).
         * Per binutils tic54x-opc.c (verified against insn_template struct
         * include/opcode/tic54x.h:85-150):
         *   word0 = 0x6F00 mask 0xFF00 (Smem in low 7 bits)
         *   word1 = sub-opcode in bits 7:5, SRC=bit 9, DST/SRC1=bit 8,
         *           SHIFT=signed 5-bit in bits 4:0
         *     bits 7:5 = 000 → ADD Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 001 → SUB Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 010 → LD  Smem,SHIFT,DST
         *     bits 7:5 = 011 → STH SRC1,SHIFT,Smem
         *     bits 7:5 = 100 → STL SRC1,SHIFT,Smem
         *
         * Without this handler, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes 0x6Fxx as LD Smem,T (1-word), causing PC drift +1 word
         * and the lk-side operand to be executed as parasitic instruction.
         * 544 sites in firmware ROM. See doc/opcodes/0x68_0x6F.md for spec.
         *
         * 2026-04-28 — fix introduced for wedge at PC=0x8353 (CALAD A self-loop)
         * caused by 0x6F07 0x0C41 mis-decoded → 0x0C41 executed as parasitic
         * SUB Smem,TS,A → A_low=0xFFFA → A_low=0x8353 after subsequent ADD. */
        if ((op & 0xFF00) == 0x6F00) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            int sub = (op2 >> 5) & 0x7;
            int shift_raw = op2 & 0x1F;
            int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
            int dst_b = (op2 >> 8) & 1;   /* bit 8 = DST/SRC1 */
            int src_b = (op2 >> 9) & 1;   /* bit 9 = SRC (ADD/SUB only) */
            consumed = 2;

            switch (sub) {
            case 0: { /* ADD Smem,SHIFT,SRC,[DST]: DST = SRC + (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src + v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 1: { /* SUB Smem,SHIFT,SRC,[DST]: DST = SRC - (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src - v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 2: { /* LD Smem,SHIFT,DST: DST = data[Smem] << shift (SXM-aware) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                if (dst_b) s->b = sext40(v); else s->a = sext40(v);
                break;
            }
            case 3: { /* STH SRC1,SHIFT,Smem: data[Smem] = (SRC1 high 16) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int16_t high = (int16_t)((src >> 16) & 0xFFFF);
                int64_t shifted = (shift >= 0) ? ((int64_t)high << shift)
                                               : ((int64_t)high >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            case 4: { /* STL SRC1,SHIFT,Smem: data[Smem] = (SRC1 low) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int64_t shifted = (shift >= 0) ? (src << shift) : (src >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            default:
                { static int unk6f = 0; if (unk6f++ < 10)
                    C54_LOG("0x6F unknown sub=%d op=0x%04x op2=0x%04x PC=0x%04x",
                            sub, op, op2, s->pc); }
                break;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6800) {
            /* DEAD CODE since 2026-04-28: all 0x68xx-0x6Fxx now intercepted
             * by specific handlers above (ANDM/ORM/XORM/ADDM/BANZ/BANZD/
             * extended-0x6F00) plus the existing 0x6Dxx MAR. This generic
             * "LD Smem, T" fallback was the source of the 2107-site mass
             * mis-dispatch that caused PC drift on every 0x68xx-0x6Fxx
             * encounter. Kept here for safety in case a previously unseen
             * sub-encoding slips through; if you ever see this trigger,
             * the new handler above for the matching 0xNN00 prefix is
             * incomplete. See doc/opcodes/0x68_0x6F.md. */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x1: {
        /* 1xxx: LD / LDU / LDR Smem, DST  (per tic54x-opc.c, all mask FE00):
         *   0x1000  LD  Smem, DST          — signed load (SXM-aware)
         *   0x1200  LDU Smem, DST          — unsigned load (zero-extend)
         *   0x1400  LD  Smem, TS, DST      — load shifted by T low bits
         *   0x1600  LDR Smem, DST          — load with rounding
         *
         * Critical: bootloader at PROM0 0xb429 does `LDU *(0x0ffe), A`
         * (op=0x12f8 + lk=0x0ffe) to read BL_ADDR_LO, then BACC A to that
         * target. The previous "case 0x1: SUB" decoded this as a subtract,
         * leaving A=0 and the BACC dropping into boot-stub NOPs. */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* selects LD/LDU/LD,TS/LDR within case 1 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        switch (sub) {
        case 0x0:  /* 0x1000: LD Smem, DST — signed (SXM honoured) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        case 0x1: { /* 0x1200: LDU Smem, DST — always zero-extended */
            v = (uint16_t)val;
            break;
        }
        case 0x2: { /* 0x1400: LD Smem, TS, DST — shift by T[5:0] (signed) */
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            int64_t base = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (ts >= 0) ? (base << ts) : (base >> -ts);
            break;
        }
        case 0x3: { /* 0x1600: LDR Smem, DST — load with rounding (+0x8000) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (v << 16) + 0x8000;
            v &= 0xFFFFFFFF0000LL;  /* clear low 16 after rounding */
            if (dst) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* 0x18/0x1A/0x1C: AND/OR/XOR Smem, DST — 16-bit Smem ZERO-extended, combined
         * with the existing accumulator (NOT a load). These were missing: sub 4/5/6
         * fell through to `default` and ran as a sign-extended LD, which broke e.g.
         * loader1's 32-bit `remaining` math (`or *(0x87e),a` with 0x8000 got
         * sign-extended to 0xFFFF8000 -> remaining went negative -> premature finalize). */
        case 0x4:  /* 0x1800: AND Smem, DST — zero-ext clears the upper guard bits */
            if (dst) s->b = s->b & (int64_t)(uint16_t)val;
            else     s->a = s->a & (int64_t)(uint16_t)val;
            return consumed + s->lk_used;
        case 0x5:  /* 0x1A00: OR Smem, DST — upper bits preserved */
            if (dst) s->b = sext40(s->b | (int64_t)(uint16_t)val);
            else     s->a = sext40(s->a | (int64_t)(uint16_t)val);
            return consumed + s->lk_used;
        case 0x6:  /* 0x1C00: XOR Smem, DST — upper bits preserved */
            if (dst) s->b = sext40(s->b ^ (int64_t)(uint16_t)val);
            else     s->a = sext40(s->a ^ (int64_t)(uint16_t)val);
            return consumed + s->lk_used;
        default:
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        }
        if (dst) s->b = sext40(v); else s->a = sext40(v);
        /* LDU-PTR (patch #2 diag, gated CALYPSO_DEBUG=LDU-PTR) : au site qui
         * charge A pour le CALA->0 (defaut PC=0xfa7e, override
         * CALYPSO_TRACE_LDU_PC=0xNNNN). Dump l'EA lue + valeur + indirect +
         * AR/DP pour nommer la case = 0 (pointeur table non init / EA fausse). */
        {
            static int ldu_trace_pc = -1;
            if (ldu_trace_pc < 0) {
                const char *e = getenv("CALYPSO_TRACE_LDU_PC");
                ldu_trace_pc = (e && *e) ? (int)strtol(e, NULL, 0) : 0xfa7e;
            }
            if (s->pc == (uint16_t)ldu_trace_pc) {
                C54_DBG("LDU-PTR",
                    "LDU-PTR PC=0x%04x op=0x%04x sub=%d EA=0x%04x val=0x%04x ind=%d "
                    "DP=0x%03x AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x "
                    "AR5=%04x AR6=%04x AR7=%04x insn=%u",
                    s->pc, op, sub, addr, val, ind, (s->st0 & 0x1FF),
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    (unsigned)s->insn_count);
            }
        }
        /* CALAD-zone LD trace: every LD/LDU/LDR that targets A while
         * executing in DARAM near the CALAD cluster. Reveals what
         * address/value is feeding A right before each CALAD A. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t ldA_total;
            ldA_total++;
            if (ldA_total <= 60 || (ldA_total % 5000) == 0) {
                C54_LOG("LD-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=0x%04x DP=0x%03x",
                        (unsigned long long)ldA_total,
                        s->pc, op, sub, addr, val,
                        (uint16_t)(s->a & 0xFFFF),
                        (s->st0 & 0x1FF));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x0: {
        /* 0xxx: ADD / ADDS / ADD,TS / SUB / SUBS / SUB,TS  (mask FE00):
         *   0x0000 ADD  Smem, SRC1 (no shift, SXM honoured)
         *   0x0200 ADDS Smem, SRC1 (no shift, zero-extended)
         *   0x0400 ADD  Smem, TS, SRC1
         *   0x0800 SUB  Smem, SRC1
         *   0x0A00 SUBS Smem, SRC1
         *   0x0C00 SUB  Smem, TS, SRC1
         * Previous handler always shifted by 16 — wrong for plain ADD/SUB.
         */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* 0..7 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        bool is_sub = (sub & 0x4) != 0;
        bool is_unsigned = (sub == 1 || sub == 5);  /* ADDS / SUBS */
        bool ts_shift = (sub == 2 || sub == 6);     /* ,TS variants */
        v = is_unsigned ? (uint16_t)val
                        : ((s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val);
        if (ts_shift) {
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            v = (ts >= 0) ? (v << ts) : (v >> -ts);
        }
        if (is_sub) {
            if (dst) s->b = sext40(s->b - v);
            else     s->a = sext40(s->a - v);
        } else {
            if (dst) s->b = sext40(s->b + v);
            else     s->a = sext40(s->a + v);
        }
        /* CALAD-zone ADD/SUB trace: same scope as LD-A-TRACE. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t addA_total;
            addA_total++;
            if (addA_total <= 30 || (addA_total % 5000) == 0) {
                C54_LOG("ADDSUB-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=%010llx",
                        (unsigned long long)addA_total,
                        s->pc, op, sub, addr, val,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x3: {
        /* 3xxx — 16 DISTINCT sub-opcodes (binutils tic54x-opc.c + SPRU172C), NOT a
         * MAC catch-all. AUDIT FIX 2026-06-12 (the demand-page-chain stall root,
         * docs/research/dsp-demand-page-chain.md §3): the old flat `acc += T*Smem`
         * swallowed every 0x3xxx op — in particular 0x3292 `LD *AR2+,ASM` @0x31E9
         * (ASM stayed 0 -> the GO handler posted the task-ready bit to [0x6D1] bit0
         * instead of bit [0x6D0] -> block-18's wait for task 1 never passed -> the
         * overlay page chain died after 2 of its 5+ boot pages) and 0x34F8 `BITT`
         * @0x31B3 (the idle-exit task test ran as a MAC -> TC stale garbage).
         * Exact handlers for 30/31/32/33/34/35/37 previously existed as DEAD CODE
         * inside case 0xF (unreachable; removed there). Pseudocode per SPRU172C:
         * the AH-multiply family (31 MPYA / 33 MASA / 35 MACA / 37 MACAR) and the
         * square family (38-3B SQURA/SQURS) ALL load T = Smem; rounding ([R])
         * applies to the ACCUMULATED result (add 2^15, clear bits 15-0). */
        int sub = (op >> 8) & 0xF;
        addr = resolve_smem(s, op, &ind);
        uint16_t val = data_read(s, addr);
        switch (sub) {
        case 0x0:                                        /* LD Smem, T */
            s->t = val;
            return consumed + s->lk_used;
        case 0x2:                                        /* LD Smem, ASM (5 bits) */
            s->st1 = (uint16_t)((s->st1 & ~ST1_ASM_MASK) | (val & ST1_ASM_MASK));
            return consumed + s->lk_used;
        case 0x4: {                                      /* BITT Smem: TC = Smem(15-T(3:0)) */
            int b = (val >> (15 - (s->t & 0xF))) & 1;
            if (b) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
            return consumed + s->lk_used;
        }
        case 0x1: case 0x3: case 0x5: case 0x7: {        /* MPYA / MASA / MACA / MACAR Smem */
            int16_t ahi = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t prod = (int64_t)ahi * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int64_t r;
            if      (sub == 0x1) r = prod;               /* MPYA: B = Smem*AH       */
            else if (sub == 0x3) r = sext40(s->b) - prod;/* MASA: B -= Smem*AH      */
            else                 r = sext40(s->b) + prod;/* MACA[R]: B += Smem*AH   */
            if (sub == 0x7) r = (r + 0x8000) & ~0xFFFFLL;/* [R]: round the result   */
            s->b = sext40(r);
            s->t = val;                                  /* all load T = Smem       */
            return consumed + s->lk_used;
        }
        case 0x6: {                                      /* POLY Smem: A = rnd(AH*T + B); */
            int16_t ahi = (int16_t)((s->a >> 16) & 0xFFFF); /*           B = Smem << 16   */
            int64_t prod = (int64_t)ahi * (int64_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            s->a = sext40(((prod + sext40(s->b)) + 0x8000) & ~0xFFFFLL);
            int64_t bv = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)val : (int64_t)val;
            s->b = sext40(bv << 16);
            return consumed + s->lk_used;
        }
        case 0x8: case 0x9: case 0xA: case 0xB: {        /* SQURA/SQURS Smem, A/B */
            int64_t prod = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            s->t = val;                                  /* T = Smem first          */
            int64_t *acc = (sub & 1) ? &s->b : &s->a;
            *acc = sext40((sub & 2) ? (sext40(*acc) - prod)   /* 3A/3B SQURS */
                                    : (sext40(*acc) + prod)); /* 38/39 SQURA */
            return consumed + s->lk_used;
        }
        default: {                                       /* 3C-3F ADD Smem, 16, src [, dst] */
            int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)val : (int64_t)val;
            v <<= 16;
            int64_t src = (sub & 2) ? sext40(s->b) : sext40(s->a);
            int64_t r = src + v;
            if (sub & 1) s->b = sext40(r); else s->a = sext40(r);
            return consumed + s->lk_used;
        }
        }
    }

    case 0x2:
        /* 2xxx: MPY, SQUR, MAS, MAC variants */
        {
            int sub = (op >> 8) & 0xF;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t product;
            int dst;
            switch (sub) {
            case 0x0: case 0x1: /* MPY Smem, A/B */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x4: case 0x5: /* SQUR Smem, A/B */
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x8: case 0x9: /* MPYA Smem (A = T * Smem, B += A) or variants */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) { s->a += s->b; s->b = sext40(product); }
                else         { s->b += s->a; s->a = sext40(product); }
                return consumed + s->lk_used;
            case 0xA: case 0xB: /* MACA[R] Smem, A/B (A += B * Smem then B = T * Smem) */
                dst = sub & 1;
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (dst) { s->a = sext40(s->a + s->b); s->b = sext40(product); }
                else     { s->b = sext40(s->b + s->a); s->a = sext40(product); }
                s->t = val;
                return consumed + s->lk_used;
            default:
                /* MAS variants and others */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                dst = sub & 1;
                if (dst) s->b = sext40(s->b - product);
                else     s->a = sext40(s->a - product);
                return consumed + s->lk_used;
            }
        }

    case 0x4:
        /* 0x4xxx group — per binutils tic54x-opc.c:
         *   0x40-0x43  SUB Smem,16,src[,dst]    (mask 0xFC00)
         *   0x44-0x45  LD  Smem,16,dst          (mask 0xFE00)
         *   0x4600     LD  Smem,DP              (mask 0xFF00)
         *   0x4700     RPT Smem                 (mask 0xFF00)
         *   0x48-0x49  LDM MMR,dst              (mask 0xFE00)
         *   0x4A00     PSHM MMR                 (mask 0xFF00)
         *   0x4B00     PSHD Smem                (mask 0xFF00)
         *   0x4C00     LTD Smem                 (mask 0xFF00)
         *   0x4D00     DELAY Smem               (mask 0xFF00)
         *   0x4E-0x4F  DST src,Lmem             (mask 0xFE00) */
        {
            uint8_t op8 = hi8;            /* (op >> 8) & 0xFF */
            int dst_b = op8 & 0x01;        /* bit8 = src/dst select (A=0, B=1) */
            int64_t *acc_dst = dst_b ? &s->b : &s->a;

            if (op8 >= 0x40 && op8 <= 0x43) {
                /* SUB Smem << 16, src, dst — sub of shifted Smem from acc */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(*acc_dst - val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x44 || op8 == 0x45) {
                /* LD Smem << 16, dst */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x46) {
                /* LD Smem, DP — load DP from low 9 bits of Smem */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (val & ST0_DP_MASK);
                g_last_ldp_pc = s->pc; g_last_ldp_val = (val & ST0_DP_MASK); g_last_ldp_kind = 3;
                return consumed + s->lk_used;
            }
            if (op8 == 0x47) {
                /* RPT Smem — repeat the NEXT insn data[Smem]+1 times. Old version
                 * loaded BRC (the BLOCK-repeat counter) + a boolean rpt_active and
                 * skipped the repeat protocol -> the repeated insn ran with a STALE
                 * rpt_count (typically 1) regardless of the real count. First
                 * casualty: the MDIRCV enqueue's `rpt *(0x8)` (= MMR AL — the
                 * computed (lenBytes+1)/2-1 payload word count!) copied 2 words for
                 * EVERY d2m message, truncating multi-word replies (header said
                 * len 0x10, ring got 1 payload word -> MCU drain desync, 2026-06-10). */
                addr = resolve_smem(s, op, &ind);
                s->rpt_count = data_read(s, addr);
                s->rpt_active = true; s->rpt_arming = true;
                s->pc += consumed + s->lk_used;
                return 0;
            }
            if (op8 == 0x48 || op8 == 0x49) {
                /* LDM MMR, dst — load accumulator from a memory-mapped reg */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                *acc_dst = sext40((int16_t)val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4A) {
                /* PSHM MMR — push memory-mapped reg onto stack */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                if (mmr == MMR_ST0) st0_ring_rec(s, val, 'P'); /* push ST0 (C-sweep) */
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4B) {
                /* PSHD Smem — push data memory onto stack */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4C) {
                /* LTD Smem — T = mem[Smem]; mem[Smem+1] = mem[Smem] */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->t = val;
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4D) {
                /* DELAY Smem — mem[Smem+1] = mem[Smem] (delay-line shift) */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4E || op8 == 0x4F) {
                /* DST src, Lmem — store accumulator to long memory.
                 * Lmem = even-aligned 32-bit pair: mem[L]=high, mem[L+1]=low */
                addr = resolve_smem(s, op, &ind) & 0xFFFE;
                int64_t v = *acc_dst;
                data_write(s, addr,         (uint16_t)((v >> 16) & 0xFFFF));
                data_write(s, (addr+1)&0xFFFF, (uint16_t)(v & 0xFFFF));
                return consumed + s->lk_used;
            }
        }
        return consumed + s->lk_used;

    case 0x5:
        /* 5xxx: shifts — SFTA, SFTL, various forms.
         * NOTE: 0x56xx/0x57xx are SFTL/SFTA with Smem (1-word), NOT MVPD.
         * MVPD is at 0x8Cxx (hi8=0x8C). The old 0x56 MVPD decode was wrong
         * and caused writes to MMR_SP via resolve_smem, corrupting the stack. */
        {
            int dst = (op >> 8) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            int sub = (op >> 9) & 0x7;
            if (sub <= 1) {
                /* 50xx/51xx: SFTA src, ASM shift */
                int shift = asm_shift(s);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 2 || sub == 3) {
                /* 54xx/55xx: SFTA src, #shift (immediate in Smem) */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 4 || sub == 5) {
                /* 58xx/59xx: SFTL src, ASM shift (logical) */
                int shift = asm_shift(s);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else if (sub == 6 || sub == 7) {
                /* 5Cxx/5Dxx/5Exx/5Fxx: SFTL with Smem or other */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
        }
        return consumed + s->lk_used;

    case 0x8: case 0x9:
        /* 8xxx/9xxx: Memory moves, PORTR/PORTW */

        /* ---- Dual-operand MAC Xmem, Ymem, dst (1-word) ----
         * 0x90: MAC Xmem,Ymem,A   0x92: MAC Xmem,Ymem,B
         * 0x91: MACR Xmem,Ymem,A  0x93: MACR Xmem,Ymem,B
         * Same encoding as 0xA4 family: OOOO OOOD XXXX YYYY */
        if (hi8 == 0x90 || hi8 == 0x91 || hi8 == 0x92 || hi8 == 0x93) {
            int xar_m = (op >> 4) & 0x07;
            int yar_m = op & 0x07;
            uint16_t xval_m = data_read(s, s->ar[xar_m]);
            uint16_t yval_m = data_read(s, s->ar[yar_m]);
            if ((op >> 7) & 1) s->ar[xar_m]--; else s->ar[xar_m]++;
            if ((op & 0x08) == 0) s->ar[yar_m]++; else s->ar[yar_m]--;
            int64_t prod_m = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_m;
            if (s->st1 & ST1_FRCT) prod_m <<= 1;
            if (hi8 & 0x01) prod_m += 0x8000; /* round */
            int dst_m = (hi8 & 0x02) ? 1 : 0;
            if (dst_m) s->b = sext40(s->b + prod_m);
            else       s->a = sext40(s->a + prod_m);
            s->t = yval_m;
            return consumed + s->lk_used;
        }

        /* AUDIT FIX 2026-06-12 (decode_audit_diff vs tic54x-objdump 2.42, the
         * recurring decode-length bug class): 0x94/0x95/0x96 are NOT the MV*
         * moves (those live at 0x70-0x73, fixed 2026-06-09) — per binutils:
         *   9400 mask FE00 : ld  Xmem, SHFT, dst   (1 word; bit8 = dst A/B)
         *   9600 mask FF00 : bit Xmem, BITC        (1 word; TC = Xmem(15-BITC))
         * The old 2-word MVDK/MVKD/MVDP aliases both DESYNCED the stream
         * (swallowed the next instruction as an operand) and CORRUPTED memory
         * (data_write to the next opcode's value as an address). Live hit:
         * 0x94AF @0xB50A `ld *ar4+,15,a` inside the tone-FIR dblockrepeat
         * (0xB505-0xB54E) — skipped the first MAC and wrote garbage to
         * data[0xB009] every pass. */
        if (hi8 == 0x94 || hi8 == 0x95) {        /* LD Xmem, SHFT, dst (1 word) */
            uint16_t xaddr = resolve_xmem(s, op);
            int shft = op & 0xF;
            uint16_t v = data_read(s, xaddr);
            int64_t val = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)v : (int64_t)v;
            val = sext40(val << shft);
            if (hi8 & 0x01) s->b = val; else s->a = val;
            return consumed + s->lk_used;        /* consumed == 1 */
        }
        if (hi8 == 0x96) {                       /* BIT Xmem, BITC (1 word) */
            uint16_t xaddr = resolve_xmem(s, op);
            int bitc = op & 0xF;
            uint16_t v = data_read(s, xaddr);
            if ((v >> (15 - bitc)) & 1) s->st0 |= ST0_TC;
            else                        s->st0 &= ~ST0_TC;
            return consumed + s->lk_used;        /* consumed == 1 */
        }

        /* AUDIT FIX 2026-05-08 night : STL ↔ STH swap.
         * Per binutils tic54x-opc.c :
         *   { "stl", 1,3,3, 0x9800, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         *   { "sth", 1,3,3, 0x9A00, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         * Old decoder claimed 0x98/99=STH and 0x9A/9B=STL — exactly inverted.
         * Effect: every STL/STH-with-shift in firmware wrote the WRONG half
         * of the accumulator. Hot pattern in DSP code (post-MAC scaling),
         * so this corrupted ~half of all data writes from compute paths.
         * Shift application is intentionally simplified (no SHFT decode)
         * matching prior-art handlers — Tier B will add proper 4-bit shift
         * decode from low nibble. Mirror swap : write low for 0x98/99,
         * write high for 0x9A/9B, src bit 8 selects A/B. */
        if (hi8 == 0x98 || hi8 == 0x99) {
            /* STL src, SHFT, Xmem — store LOW (acc&0xFFFF).
             * FIX 2026-05-23 : Xmem operand decoded via resolve_xmem (per
             * binutils OP_Xmem), not resolve_smem. The latter mis-mapped
             * low byte 0x00-0x1F with bit 7=0 to MMR space, clobbering SP/
             * IMR/IFR. Empirical proof : PC=0x8a46 op=0x9918 stomp SP→0
             * captured by existing SP-CATASTROPHE probe. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A || hi8 == 0x9B) {
            /* STH src, SHFT, Xmem — store HIGH (acc>>16).
             * FIX 2026-05-23 : same as STL above — Xmem decoded via
             * resolve_xmem (per binutils), not resolve_smem. See STL block. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }

        /* 0x9C-0x9F range: SACCD/SRCCD/STRCD — conditional stores */

        /* SACCD src, Xmem, cond — Conditional accumulator store
         * Encoding: 1001 11SD XXXX COND per SPRU172C p.4-152 */
        if ((op & 0xFC00) == 0x9C00) {
            int src_s = (op >> 9) & 1;
            int64_t acc = src_s ? s->b : s->a;
            /* FIX 2026-06-02 (ROOT CAUSE FB-det) : opérande Xmem décodé via
             * resolve_xmem (xar=((op>>4)&3)+2 = AR2-5, + xmod post-modify),
             * exactement comme STL/STH 0x98-0x9B. L'ancien `(op>>4)&0x07` lisait
             * le MAUVAIS AR (AR1 au lieu de AR3 pour op=0x9e9b) et `(op>>7)&1` un
             * faux sens → AR3 jamais incrémenté → la boucle de recherche de pic
             * FCCH (@0x8576 RPTB) relisait sample[0] 15× → corrélation figée,
             * d_fb_det garbage, rxlev plancher, FBSB jamais fermé. resolve_xmem
             * applique le post-incrément ; ne PAS re-modifier en fin de handler. */
            uint16_t xaddr = resolve_xmem(s, op);
            int cond = op & 0x0F;
            /* Evaluate condition */
            int take = 0;
            switch (cond) {
            case 0x0: take = (acc == 0); break;    /* EQ */
            case 0x1: take = (acc != 0); break;    /* NEQ */
            case 0x2: take = (acc > 0); break;     /* GT */
            case 0x3: take = (acc < 0); break;     /* LT */
            case 0x4: take = (acc >= 0); break;    /* GEQ */
            case 0x5: take = (acc == 0); break;    /* AEQ */
            case 0x6: take = (acc > 0); break;     /* AGT */
            case 0x7: take = (acc <= 0); break;    /* LEQ/ALEQ */
            default: take = 0; break;
            }
            int asm_val = asm_shift(s);
            if (take) {
                /* Store shifted accumulator high part */
                int64_t shifted = acc << (asm_val > 0 ? asm_val : 0);
                if (asm_val < 0) shifted = acc >> (-asm_val);
                uint16_t val = (uint16_t)((shifted >> 16) & 0xFFFF);
                data_write(s, xaddr, val);
            } else {
                /* Read and write back (no change) */
                uint16_t val = data_read(s, xaddr);
                data_write(s, xaddr, val);
            }
            /* post-modify Xmem déjà appliqué par resolve_xmem (cf FIX ci-dessus) */
            return consumed + s->lk_used;
        }
        /* POPM MMR — pop top-of-stack into MMR (1-word).
         * Per tic54x-opc.c: { "popm", 0x8A00, 0xFF00, {OP_MMR} }.
         * Per SPRU172C section 4 : value at SP popped to MMR, SP++.
         *
         * Bug fix 2026-05-08 : 0x8Axx était précédemment mal décodé en
         * MVDK Smem,dmad (qui est en réalité 0x7100 mask 0xFF00). Le
         * pattern PSHM/POPM symétrique du firmware (e.g. PROM0 0x7013-0x7023
         * sauve/restaure 6 MMRs autour d'un CALA) ne fonctionnait jamais
         * post-CALA → ST1 jamais restauré → INTM=1 dwell perpétuel
         * → IRQ vectoring bloqué → DSP wait stuck → L1 mort.
         * Le case MVDK ci-dessous devient dead code mais est laissé pour
         * référence historique. */
        if ((op & 0xFF00) == 0x8A00) {
            uint16_t mmr = op & 0x7F;
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            /* POPM-ST1 probe (CALYPSO_DEBUG=POPM-ST1) : ST1 == MMR 0x07.
             * Discrimine (a) POPM ST1 jamais exécuté vs (b) exécuté mais
             * la valeur poppée a déjà INTM=1 → restaure 1, ne clear jamais.
             * Silent par défaut. */
            if (mmr == 0x07) {
                C54_DBG("POPM-ST1",
                        "POPM ST1 val=0x%04x INTM_bit=%u PC=0x%04x SP=0x%04x insn=%u",
                        val, !!(val & ST1_INTM), s->pc, s->sp, s->insn_count);
            }
            data_write(s, mmr, val);
            return consumed + s->lk_used;
        }
        /* OBSOLETE — superseded by POPM above. The 0x8Axx range belongs to
         * POPM per tic54x-opc.c, not MVDK (which is 0x7100 mask 0xFF00).
         * Kept commented for one revision so any caller depending on the
         * old (incorrect) behaviour is forced to be re-examined. */
        if (0 && hi8 == 0x8A) {
            /* MVDK Smem, dmad — INCORRECT for 0x8Axx, see POPM above */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 0x88xx-0x89xx: STLM src, MMR  (1-word!)
         * Per tic54x-opc.c: { "stlm", 1,2,2, 0x8800, 0xFE00, ... }
         *   bits 9-15 = fixed (0x44)
         *   bit 8     = src (0 = A, 1 = B)
         *   bits 0-6  = MMR address (0x00..0x7F)
         *
         * Critical for the DSP bootloader at PROM0 0xb42d (`STLM B, AR1`):
         * if decoded as 2-word MVDM the emulator eats the next opcode
         * (0xb42e = 0xf84c, a BC), then jumps into 0xb431 (MACR family)
         * with an uninitialised T register, producing A=0x10 — which
         * the immediately-following BACC A at 0xb430 then uses as the
         * jump target, dropping the DSP into the boot-stub NOPs at
         * PC=0x0010 instead of continuing the bootloader handshake. */
        if (hi8 == 0x88 || hi8 == 0x89) {
            int src = (op >> 8) & 1;  /* 0 = A, 1 = B */
            int mmr = op & 0x7F;
            uint16_t val = src ? (uint16_t)(s->b & 0xFFFF)
                               : (uint16_t)(s->a & 0xFFFF);
            data_write(s, (uint16_t)mmr, val);  /* MMRs alias addr 0x00..0x1F */
            return consumed + s->lk_used;
        }
        if (hi8 == 0x80) {
            /* AUDIT FIX 2026-05-08 night : was stubbed NOP because old
             * decoder claimed MVDD (2-word, wrong). Per binutils tic54x-opc.c :
             *   { "stl", 1,2,2, 0x8000, 0xFE00, {OP_SRC1,OP_Smem}, 0, REST }
             * 0x80xx/0x81xx = STL src, Smem (1-word, no shift). bit 8 = src.
             * Range 0x8000-0x80FF = STL A, Smem (since bit 8 = 0 here).
             * Stubbing this silently dropped every STL A in the firmware ;
             * variables that should have been written to DARAM kept stale
             * values (junk-state cascade). Mirror of the existing 0x82
             * STH-with-shift handler but no shift here. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->a & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8C) {
            /* AUDIT FIX 2026-05-08 night : was MVPD pmad,Smem (2 mots,
             * prog→data move). Per binutils tic54x-opc.c :
             *   { "mvpd", 2,2,2, 0x7C00, 0xFF00, {OP_pmad,OP_Smem}, 0, REST }
             *   { "st",   1,2,2, 0x8C00, 0xFF00, {OP_T,OP_Smem},    0, REST }
             * Real MVPD is at 0x7C — the 0x8C handler should be ST T, Smem
             * (1 mot, store T register to data memory). Run-trace confirms
             * 0 MVPD hits with the old handler, meaning firmware did not
             * issue any 0x7Cxx → our wrong 0x8C MVPD was never triggered
             * for legitimate MVPD anyway (PROM0 OVLY happens via DSP
             * bootloader, not via 0x7C MVPD instruction). Switching to
             * ST T,Smem is safe and unblocks the legitimate ST T pattern
             * used after MAC for T persistence. Old MVPD-LOG instrumentation
             * removed — was dead-code in current run. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, s->t);
            return consumed + s->lk_used;
        }
        /* 0x8E/0x8F : CMPS src, Smem — Compare, Select & Store Maximum
         * (SPRU172C p.4-35). Opcode 1000 111 S I AAAAAAA = 0x8E00/0xFE00,
         * bit8 = src (0=A, 1=B). 1 MOT (+1 si long-offset/absolu → lk_used).
         *   if src(31–16) > src(15–0):  src(31–16)→Smem ; TRN<<=1,TRN(0)=0 ; TC=0
         *   else:                       src(15–0)→Smem  ; TRN<<=1,TRN(0)=1 ; TC=1
         * = compare les 2 moitiés 16-bit 2s-comp de l'accu, stocke la MAX, TRN/TC
         * tracent le gagnant. Cœur de la recherche de pic FCCH (Viterbi).
         *
         * FIX 2026-06-02 (audit décodeur DECODE-AUDIT) : 0x8E était décodé MVDP
         * 2-mots et 0x8F PORTR 2-mots → chaque paire CMPS A/CMPS B consécutive
         * (op=8e94 op2=8f93 dans la zone FB-det 0xa0xx) voyait le 2e CMPS BOUFFÉ
         * comme phantom-pmad → désync corrélateur, d_fb_det jamais armé. SÛR :
         * l'assembleur TI encode MVDP=0x7D, PORTR=0x74 — jamais 0x8E/0x8F ; et
         * l'I/Q arrive par DMA DARAM (data[0x2a00]), pas par opcode PORTR (audit
         * 0x8F=0 exécution). Le vieux handler 0x8F=PORTR est neutralisé plus bas. */
        if (hi8 == 0x8E || hi8 == 0x8F) {
            addr = resolve_smem(s, op, &ind);
            int src = (op >> 8) & 1;
            int64_t acc = src ? s->b : s->a;
            int16_t hi = (int16_t)((acc >> 16) & 0xFFFF);
            int16_t lo = (int16_t)(acc & 0xFFFF);
            s->trn = (uint16_t)(s->trn << 1);
            if (hi > lo) {
                data_write(s, addr, (uint16_t)hi);
                s->trn &= ~0x0001u;
                s->st0 &= ~ST0_TC;
            } else {
                data_write(s, addr, (uint16_t)lo);
                s->trn |= 0x0001u;
                s->st0 |= ST0_TC;
            }
            return consumed + s->lk_used;
        }
        /* SUPERSEDED 2026-06-02 : 0x8F = CMPS B (traité par le handler CMPS
         * 0x8E/0x8F ci-dessus, qui return avant d'arriver ici). Ce bloc PORTR
         * est ISA-faux (vrai PORTR=0x74) et jamais atteint (audit 0x8F=0 exec ;
         * I/Q via DMA DARAM). Gardé en dead-code (if(0)) pour réf si on relocalise
         * PORTR vers 0x74 un jour. */
        if (0 && hi8 == 0x8F) {
            /* PORTR PA, Smem — read I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* BSP RX data register — return next burst sample.
             * The DSP firmware uses PORTR PA=0xF430 (64 sites in PROM0,
             * verified from ROM dump). We also accept 0x0034 for legacy
             * compatibility with earlier QEMU experiments. */
            uint16_t portr_val;
            bool is_bsp_pa = (op2 == 0xF430 || op2 == 0x0034);
            g_c54x_pmap_handled = 0;
            if (op2 < 0x0100 && nokia_port_read_hook) {        /* Nokia HPI/mailbox port */
                portr_val = nokia_port_read_hook(op2, (uint16_t)s->pc);
                c54x_pmap_record(0, 0, op2, portr_val, s->pc, s->insn_count, g_c54x_pmap_handled);
                data_write(s, addr, portr_val);
            } else if (is_bsp_pa && s->bsp_pos < s->bsp_len) {
                portr_val = s->bsp_buf[s->bsp_pos++];
                c54x_pmap_record(0, 0, op2, portr_val, s->pc, s->insn_count, 1);
                data_write(s, addr, portr_val);
            } else {
                portr_val = 0;
                c54x_pmap_record(0, 0, op2, 0, s->pc, s->insn_count, 0);
                data_write(s, addr, 0);
            }
            /* === PORTR-DEST-HIST (2026-05-28) ===
             * Histogramme des Smem destinations quand PA=BSP. Repond directement
             * a "ou le firmware stocke les samples BSP". Top addresses = la
             * vraie zone DARAM cible pour CALYPSO_BSP_DARAM_ADDR. */
            if (is_bsp_pa) {
                static unsigned phist[0x10000];
                static unsigned ptotal;
                phist[addr]++;
                ptotal++;
                if ((ptotal % 5000) == 0 && calypso_debug_enabled("PORTR-DEST-HIST")) {
                    unsigned best[10] = {0};
                    uint16_t baddr[10] = {0};
                    for (unsigned a = 0; a < 0x10000; a++) {
                        unsigned c = phist[a];
                        if (c <= best[9]) continue;
                        int p = 9;
                        while (p > 0 && best[p-1] < c) {
                            best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                        }
                        best[p] = c; baddr[p] = (uint16_t)a;
                    }
                    fprintf(stderr, "[c54x] PORTR-DEST-HIST (ptotal=%u): ", ptotal);
                    for (int i = 0; i < 10 && best[i]; i++)
                        fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
                    fprintf(stderr, "\n");
                }
            }
            /* Per-PA counters so we can see which I/O ports the DSP polls
             * and how often. */
            {
                static uint64_t portr_total[16];
                static uint64_t portr_since_summary;
                int pa_bucket = (op2 >> 4) & 0xF;
                portr_total[pa_bucket]++;
                portr_since_summary++;

                static int portr_log = 0;
                if (portr_log < 50) {
                    C54_LOG("PORTR PA=0x%04x → [0x%04x] val=0x%04x "
                            "bsp_pos=%u/%u PC=0x%04x",
                            op2, addr, portr_val,
                            (unsigned)s->bsp_pos, (unsigned)s->bsp_len,
                            s->pc);
                    portr_log++;
                }
                if ((portr_since_summary % 10000) == 0) {
                    C54_LOG("PORTR summary (last 10000): "
                            "PA0x=%llu 1x=%llu 2x=%llu 3x=%llu 4x=%llu "
                            "5x=%llu 6x=%llu 7x=%llu",
                            (unsigned long long)portr_total[0],
                            (unsigned long long)portr_total[1],
                            (unsigned long long)portr_total[2],
                            (unsigned long long)portr_total[3],
                            (unsigned long long)portr_total[4],
                            (unsigned long long)portr_total[5],
                            (unsigned long long)portr_total[6],
                            (unsigned long long)portr_total[7]);
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9F) {
            /* PORTW Smem, PA — write I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Log I/O port writes + route Nokia ports to the host (MCU) side */
            {
                uint16_t wval = data_read(s, addr);
                g_c54x_pmap_handled = 0;
                if (op2 < 0x0100 && nokia_port_write_hook)
                    nokia_port_write_hook(op2, wval, (uint16_t)s->pc);
                c54x_pmap_record(0, 1, op2, wval, s->pc, s->insn_count, g_c54x_pmap_handled);
                static int portw_log = 0;
                if (portw_log < 30) {
                    C54_LOG("PORTW PA=0x%04x val=0x%04x PC=0x%04x", op2, wval, s->pc);
                    portw_log++;
                }
            }
            return consumed + s->lk_used;
        }
        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
        if (hi8 == 0x85) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed + s->lk_used;
        }
        /* REVERTED 2026-05-15 nuit : handlers 0x72/0x73 RETIRÉS.
         * Voir doc/REVERT_MVMD_KNOWLEDGE.md et premier emplacement revert
         * ci-dessus (avant le bloc `(op & 0xF800) == 0x7000`). 0x86/0x87
         * restent comme avant (DUPLICATE MVDM/MVMD au lieu de STH A/B ASM
         * vrai — non swappés). */

        /* 0x86/0x87 : STH src, ASM, Smem — store HIGH (acc>>16) shifted by ASM,
         * to Smem (1-WORD). bit8 = src (0=A, 1=B). Per tic54x_hi8_map.md L95 :
         *   { "sth", 0x8600, 0xFE00, ASM variant }  ← 1 mot, mask 0xFE00.
         *
         * FIX 2026-06-02 (bug #3, ROOT CAUSE AR3-zero) : l'ancien décode était
         * MVDM dmad,MMR (0x86) / MVMD MMR,dmad (0x87) en 2 MOTS — faux sur deux
         * axes : (1) longueur (2 au lieu de 1) → consommait l'opcode suivant
         * → désync du flux de décode en cascade (= SP→0xcade observé) ; (2) ne
         * touchait jamais l'AR du Smem → AR3 figé/0 dans la boucle corrélateur
         * FB (IQ-READ @0x7e6f montrait AR3=0000 sur op=0x8693 @0x7e71 = STH A,
         * ASM, *AR3+ : AR3 doit post-incrémenter pour balayer le buffer I/Q
         * 0x2a00+). Mirror EXACT du handler 0x84 (STL A,ASM,Smem) déjà validé,
         * mais store HIGH word au lieu de LOW. resolve_smem applique le
         * post-incrément du Smem indirect → ne PAS re-modifier l'AR ici.
         *
         * SÛR vs le revert 0x72/0x73 (REVERT_MVMD_KNOWLEDGE.md) : ORTHOGONAL.
         * L'assembleur TI encode MVDM=0x72, MVMD=0x73 — JAMAIS à 0x86/0x87.
         * Donc aucun 0x86xx/0x87xx de la ROM n'est une vraie MVDM/MVMD : c'est
         * toujours un STH. Le side-effect dont dépend le firmware est sur 0x73
         * (site 0x8208 op=0x7317), inchangé par ce fix. */
        if (hi8 == 0x86 || hi8 == 0x87) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int src = hi8 & 1;            /* 0x86→A, 0x87→B */
            int64_t v = src ? s->b : s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)((v >> 16) & 0xFFFF));  /* STH = high word */
            return consumed + s->lk_used;
        }
        /* AUDIT FIX 2026-05-15 fin journée : 0x81/0x82/0x83 mal décodés.
         * Per tic54x-opc.c + SPRU172C :
         *   stl 0x8000 / 0xFE00 → 0x80..0x81 STL src,Smem (no shift)
         *   sth 0x8200 / 0xFE00 → 0x82..0x83 STH src,Smem (no shift)
         *   stl 0x8400 / 0xFE00 → 0x84..0x85 STL src,ASM,Smem (with shift) [TODO]
         *   sth 0x8600 / 0xFE00 → 0x86..0x87 STH src,ASM,Smem (with shift) [TODO]
         * bit 8 = src (0=A, 1=B). Old code applied asm_shift incorrectly
         * to 0x81/0x82 (basic variants — no shift) AND used s->a for 0x81
         * (should be s->b). Le bug causait toutes les STL B / STH * vers
         * adressing indirect *ARn à écrire la mauvaise valeur ; en particulier
         * d_burst_d (DSP word 0x0829/0x083D) et d_task_d (0x0828/0x083C) du
         * NDB CCCH demod ARM bail dans prim_rx_nb.c::l1s_nb_resp avec
         * "EMPTY" et "BURST ID 33414!=N" sous synth=1 banc d'essai. */

        /* 0x81xx: STL B, Smem  (src=B, no shift) */
        if (hi8 == 0x81) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->b & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x82xx: STH A, Smem  (src=A, no shift) */
        if (hi8 == 0x82) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 89xx: ST src, Smem with shift or MVDK variants */
        if (hi8 == 0x89) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 8Bxx: POPD Smem (per tic54x-opc.c; the old qemu MVDK-long-addr
         * classification was wrong — see doc/opcodes/tic54x_hi8_map.md).
         * Pops TOS into Smem, SP++ (SPRU172). Was a stub-NOP (2026-06-09 note:
         * "fix separately") — that left every firmware `pshd ... popd` pair
         * with a net +2 SP leak: the 5110 vec-25 host-doorbell ISR long path
         * (pshd *(0x60) / *(0x61) @0x3785/87, popd @0x37AE/B0) desynced the
         * frame so its RETE popped a data word and "returned" into DARAM
         * zeros (the post-doorbell ret-storm). Implemented 2026-06-10. */
        if (hi8 == 0x8B) {
            addr = resolve_smem(s, op, &ind);
            uint16_t v = data_read(s, s->sp); s->sp++;
            data_write(s, addr, v);
            return consumed + s->lk_used;
        }
        /* 8Dxx: MVDD Smem, Smem */
        if (hi8 == 0x8D) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* AUDIT FIX 2026-05-15 fin journée : 0x83 misclassifié comme WRITA
         * (qui est en réalité 0x7F per tic54x-opc.c). Vrai 0x83 = STH B, Smem.
         * Et 0x84 misclassifié comme READA (vrai = 0x7E). Vrai 0x84 = STL A,
         * ASM, Smem (with shift). 0x85..0x87 idem TODO. */
        /* 0x83xx: STH B, Smem  (src=B, no shift) */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->b >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x84xx: STL A, ASM, Smem (src=A, with ASM shift) — TODO compléter
         * variantes 0x85 (STL B), 0x86 (STH A), 0x87 (STH B) with ASM shift.
         * Pour l'instant fix uniquement 0x84 vers la sémantique tic54x correcte. */
        if (hi8 == 0x84) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)(v & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 91xx: MVKD dmad, Smem (another encoding) */
        if (hi8 == 0x91) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 97xx: ST #lk, Smem (2-word). 0x96xx is caught above as MVDP. */
        if (hi8 == 0x97) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xA: case 0xB:
        /* Axx/Bxx: STLM, LDMM, misc accumulator ops */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * MAC:  dst += T * Xmem; T = Ymem
         * MACR: dst += rnd(T * Xmem); T = Ymem
         * MAS:  dst -= T * Xmem; T = Ymem
         * MASR: dst -= rnd(T * Xmem); T = Ymem
         * Encoding: OOOO OOOD XXXX YYYY (1 word)
         *   Xmem: AR[ARP], post-mod by bit4 (0=inc,1=dec)
         *   Ymem: AR[bits2:0], post-mod by bit3 (0=inc,1=dec)
         *   D: 0=A, 1=B
         * hi8 mapping per SPRU172C:
         *   0xA4/0xA5: MAC[R] Xmem,Ymem,A   0xA6/0xA7: MAC[R] Xmem,Ymem,B
         *   0xB4/0xB5: MAS[R] Xmem,Ymem,A   0xB6/0xB7: MAS[R] Xmem,Ymem,B
         *   0xB0/0xB1: MAC[R] Xmem,Ymem,A (alt)  0xB2/0xB3 already handled
         */
        /* ============================================================
         * 0xA0-0xBF : C54x dual-operand (Xmem,Ymem) class — UNIFIED handler
         * (rewrite 2026-06-09). The ENTIRE case 0xA/0xB was previously a pile of
         * mis-identified handlers (0xA0=accumulator-ops, 0xA1=sqdst, 0xA2/A3=
         * ADD/SUB #lk, 0xA5=CMPS, 0xA8/A9=AND #lk, 0xAA/AB=nop-stub, 0xAC-AF=
         * MACP/MACD, 0xBA=LDMM, 0xBC-BF=poly) — ALL wrong per tic54x objdump,
         * and the dual-MAC sub-handler mis-extracted the Xmem/Ymem AR index with
         * raw (op>>4)&7 / op&7 (the 0xb399 IMR-clobber root: 0xa413 mpy *ar3,*ar5
         * landed on AR1 and force-incremented it). Authoritative map (objdump
         * -m tic54x 0xA0-0xBF sweep):
         *   0xA0/A1 add   Xmem,Ymem,dst     dst = Xmem + Ymem
         *   0xA2/A3 sub   Xmem,Ymem,dst     dst = Xmem - Ymem
         *   0xA4/A5 mpy   Xmem,Ymem,dst     dst = Xmem * Ymem
         *   0xA6/A7 macsu Xmem,Ymem,src     src += uns(Xmem)*signed(Ymem)
         *   0xA8-AF ld Xmem,d || mac/macr/mas/masr Ymem  (parallel, T-based mac)
         *   0xB0-B3 mac   Xmem,Ymem,src[,dst]  dst = src + Xmem*Ymem
         *   0xB4-B7 macr  Xmem,Ymem,src[,dst]  dst = src + Xmem*Ymem + rnd
         *   0xB8-BB mas   Xmem,Ymem,src[,dst]  dst = src - Xmem*Ymem
         *   0xBC-BF masr  Xmem,Ymem,src[,dst]  dst = src - Xmem*Ymem + rnd
         * Encoding: Xmem=bits[7:4] AR=(nib&3)+2 mod=(nib>>2)&3;
         *           Ymem=bits[3:0] AR=(nib&3)+2 mod=(nib>>2)&3;
         *           mod 0=none 1=*AR- 2=*AR+ 3=*AR+0% (circular, +AR0 mod BK).
         * For 0xB0-BF: bit9=src(0=A,1=B), bit8=dst(0=A,1=B).
         * The MAC/MAS product is Xmem*Ymem DIRECT (dual C/D bus) and T<-Xmem;
         * the old "T*Xmem,T=Ymem" was a CONFOUND of the broken addressing (with
         * wrong ARs, X*Y read zeros so T*X looked better). DSP54_MAC_TX=1 forces
         * the legacy T*Xmem+T=Ymem accumulate (A/B test the FCCH correlator). */
        if (hi8 >= 0xA0 && hi8 <= 0xBF) {
            static int mac_tx = -1;
            if (mac_tx < 0) mac_tx = getenv("DSP54_MAC_TX") ? 1 : 0;
            int xar  = ((op >> 4) & 0x03) + 2;
            int yar  = (op & 0x03) + 2;
            int xmod = (op >> 6) & 0x03;
            int ymod = (op >> 2) & 0x03;
            uint16_t xval = data_read(s, s->ar[xar]);
            uint16_t yval = data_read(s, s->ar[yar]);
            switch (xmod) { case 1: s->ar[xar]--; break; case 2: s->ar[xar]++; break;
                            case 3: s->ar[xar] = c54x_circ_ref(s->ar[xar], +(int16_t)s->ar[0], s->bk); break; }
            switch (ymod) { case 1: s->ar[yar]--; break; case 2: s->ar[yar]++; break;
                            case 3: s->ar[yar] = c54x_circ_ref(s->ar[yar], +(int16_t)s->ar[0], s->bk); break; }
            int sx = (int16_t)xval, sy = (int16_t)yval;

            /* add / sub Xmem,Ymem,dst — dst = sext(Xmem) +/- sext(Ymem). dst=bit0. */
            if (hi8 <= 0xA3) {
                int64_t r = (hi8 <= 0xA1) ? ((int64_t)sx + sy) : ((int64_t)sx - sy);
                if (hi8 & 0x01) s->b = sext40(r); else s->a = sext40(r);
                return consumed + s->lk_used;
            }
            /* mpy Xmem,Ymem,dst — dst = Xmem*Ymem (SET). dst=bit0. T<-Xmem. */
            if (hi8 == 0xA4 || hi8 == 0xA5) {
                int64_t prod = (int64_t)sx * (int64_t)sy;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (hi8 & 0x01) s->b = sext40(prod); else s->a = sext40(prod);
                s->t = xval;
                return consumed + s->lk_used;
            }
            /* macsu Xmem,Ymem,src — src += unsigned(Xmem)*signed(Ymem). src=bit0. */
            if (hi8 == 0xA6 || hi8 == 0xA7) {
                int64_t pu = (int64_t)(uint16_t)xval * (int64_t)sy;
                if (s->st1 & ST1_FRCT) pu <<= 1;
                if (hi8 & 0x01) s->b = sext40(s->b + pu); else s->a = sext40(s->a + pu);
                s->t = xval;
                return consumed + s->lk_used;
            }
            /* parallel : ld Xmem,dst_ld || {mac|macr|mas|masr} Ymem,dst_mac.
             * The MAC half is single-operand (dst_mac +/-= T*Ymem). bit0 selects
             * which acc is loaded vs accumulated (0: ld->A,mac->B ; 1: ld->B,mac->A).
             * sub-op: A8/9 mac, AA/B macr, AC/D mas, AE/F masr. T<-Ymem. */
            if (hi8 >= 0xA8 && hi8 <= 0xAF) {
                int64_t mprod = (int64_t)(int16_t)s->t * (int64_t)sy;
                if (s->st1 & ST1_FRCT) mprod <<= 1;
                if (hi8 == 0xAA || hi8 == 0xAB || hi8 == 0xAE || hi8 == 0xAF) mprod += 0x8000;
                int is_sub = (hi8 >= 0xAC);
                int64_t xld = sext40((int64_t)sx);
                if (hi8 & 0x01) {            /* ld->B, mac->A */
                    s->b = xld;
                    s->a = sext40(is_sub ? s->a - mprod : s->a + mprod);
                } else {                     /* ld->A, mac->B */
                    s->a = xld;
                    s->b = sext40(is_sub ? s->b - mprod : s->b + mprod);
                }
                s->t = yval;
                return consumed + s->lk_used;
            }
            /* 0xB0-0xBF : mac / macr / mas / masr Xmem,Ymem,src[,dst]. */
            {
                int is_sub = (hi8 >= 0xB8);
                int is_r   = (hi8 & 0x04) ? 1 : 0;       /* B4-B7, BC-BF = round */
                int src_b  = (hi8 >> 1) & 1;             /* op bit9 */
                int dst_b  = hi8 & 1;                    /* op bit8 */
                int64_t prod, src, r;
                if (mac_tx) {
                    /* legacy confound: T*Xmem accumulate, T<-Ymem */
                    prod = (int64_t)(int16_t)s->t * (int64_t)sx;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (is_r) prod += 0x8000;
                    src = src_b ? s->b : s->a;
                    r = is_sub ? (src - prod) : (src + prod);
                    if (dst_b) s->b = sext40(r); else s->a = sext40(r);
                    s->t = yval;
                } else {
                    prod = (int64_t)sx * (int64_t)sy;    /* true: Xmem*Ymem */
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (is_r) prod += 0x8000;
                    src = src_b ? s->b : s->a;
                    r = is_sub ? (src - prod) : (src + prod);
                    if (dst_b) s->b = sext40(r); else s->a = sext40(r);
                    s->t = xval;
                }
                return consumed + s->lk_used;
            }
        }
        goto unimpl;

    case 0xC: case 0xD:
        /* C/Dxxx: PSHM, POPM, PSHD, POPD, RPT, FRAME, etc. */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * 0xD0: MAC Xmem,Ymem,A   0xD2: MAC Xmem,Ymem,B
         * 0xD1: MACR Xmem,Ymem,A  0xD3: MACR Xmem,Ymem,B
         * 0xD4-0xD7: MAS variants (subtract)
         *
         * Encoding per binutils tic54x.h (XARX/YARX = ((C&0x3)+2)) :
         *   bits 7:6 Xmod  | 5:4 Xar (AR2..AR5) | 3:2 Ymod | 1:0 Yar (AR2..AR5)
         * Was 3-bit AR raw — same bug as C8/CB had (fixed 2026-05-08). Now
         * aligned with binutils. Expected aftermath : new SP-CATASTROPHE on
         * D-class opcodes when firmware ARs land at MMR — same root pattern
         * as 0xc8be at PC=0xa0e7. That's correct exposure, not regression. */
        if (hi8 >= 0xD0 && hi8 <= 0xD9 && hi8 != 0xDA) {
            int xmod_c = (op >> 6) & 0x03;
            int xar_c  = ((op >> 4) & 0x03) + 2;
            int ymod_c = (op >> 2) & 0x03;
            int yar_c  = (op & 0x03) + 2;
            uint16_t xval_c = data_read(s, s->ar[xar_c]);
            uint16_t yval_c = data_read(s, s->ar[yar_c]);
            switch (xmod_c) {
            case 0: break;
            case 1: s->ar[xar_c]++; break;
            case 2: s->ar[xar_c]--; break;
            case 3: s->ar[xar_c] = c54x_circ_ref(s->ar[xar_c], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            switch (ymod_c) {
            case 0: break;
            case 1: s->ar[yar_c]++; break;
            case 2: s->ar[yar_c]--; break;
            case 3: s->ar[yar_c] = c54x_circ_ref(s->ar[yar_c], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ (Ymem 0xd3dc @0xfa98) — fix 2026-06-01 */
            }
            /* MAC dual-mem formula : T × Xmem (pas X × Y per SPRU pure).
             *
             * 2026-05-08 retest empirique avec pipeline stable :
             *   T×X  : BRC variable, A/B accumulator drift, d_fb_det reaches
             *          high SNR values (0x7902 / 0x7766) at moments
             *   X×Y  : BRC=0 uniforme (201/201), A=B=0 forever, d_fb_det
             *          mostly 0 — correlation produces only zeros
             *
             * Le firmware Calypso s'appuie sur le pipeline c54x : T est
             * latched depuis Ymem du MAC précédent (T = Y(post)). Ainsi
             * MAC dual-mem effectivement calcule `T_old × X_current` =
             * `Y[n-1] × X[n]`. Notre `prod = T × X` reproduit fidèlement
             * cet effet pipelined. `X × Y` (les 2 du buffer courant) ne
             * matche pas la sémantique attendue par le firmware. */
            int64_t prod_c = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_c;
            if (s->st1 & ST1_FRCT) prod_c <<= 1;
            if (hi8 & 0x01) prod_c += 0x8000; /* round */
            int is_sub_c = (hi8 >= 0xD4);
            int dst_c = (hi8 & 0x02) ? 1 : 0;
            if (dst_c) {
                if (is_sub_c) s->b = sext40(s->b - prod_c);
                else          s->b = sext40(s->b + prod_c);
            } else {
                if (is_sub_c) s->a = sext40(s->a - prod_c);
                else          s->a = sext40(s->a + prod_c);
            }
            s->t = yval_c;
            return consumed + s->lk_used;
        }

        /* DBxx: MASA Xmem, Ymem, dst — MAC with accumulator sign extension
         * Per SPRU172C: same as MAC but T loaded from Xmem instead of Ymem.
         * dst += T * Xmem, T = Xmem
         * Encoding fixed 2026-05-08 : same 2-bit AR + offset 2 + 2-bit mod
         * format as the rest of the dual-operand class. */
        if (hi8 == 0xDB) {
            int xmod_db = (op >> 6) & 0x03;
            int xar_db  = ((op >> 4) & 0x03) + 2;
            int ymod_db = (op >> 2) & 0x03;
            int yar_db  = (op & 0x03) + 2;
            uint16_t xval_db = data_read(s, s->ar[xar_db]);
            (void)data_read(s, s->ar[yar_db]); /* Ymem read (unused) */
            switch (xmod_db) {
            case 0: break;
            case 1: s->ar[xar_db]++; break;
            case 2: s->ar[xar_db]--; break;
            case 3: s->ar[xar_db] = c54x_circ_ref(s->ar[xar_db], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            switch (ymod_db) {
            case 0: break;
            case 1: s->ar[yar_db]++; break;
            case 2: s->ar[yar_db]--; break;
            case 3: s->ar[yar_db] = c54x_circ_ref(s->ar[yar_db], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            int64_t prod_db = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_db;
            if (s->st1 & ST1_FRCT) prod_db <<= 1;
            s->a = sext40(s->a + prod_db);
            s->t = xval_db;
            return consumed + s->lk_used;
        }

        /* DCxx: SQUR Xmem, dst — Square and accumulate (1-word dual-operand)
         * Per SPRU172C p.4-165: T = Xmem, dst = dst + T * T
         * Encoding fixed 2026-05-08 : same dual-operand format as D0-D9. */
        if (hi8 == 0xDC) {
            int xmod_dc = (op >> 6) & 0x03;
            int xar_dc  = ((op >> 4) & 0x03) + 2;
            int ymod_dc = (op >> 2) & 0x03;
            int yar_dc  = (op & 0x03) + 2;
            uint16_t xval_dc = data_read(s, s->ar[xar_dc]);
            (void)data_read(s, s->ar[yar_dc]); /* Ymem pipeline read */
            switch (xmod_dc) {
            case 0: break;
            case 1: s->ar[xar_dc]++; break;
            case 2: s->ar[xar_dc]--; break;
            case 3: s->ar[xar_dc] = c54x_circ_ref(s->ar[xar_dc], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            switch (ymod_dc) {
            case 0: break;
            case 1: s->ar[yar_dc]++; break;
            case 2: s->ar[yar_dc]--; break;
            case 3: s->ar[yar_dc] = c54x_circ_ref(s->ar[yar_dc], +(int16_t)s->ar[0], s->bk); break; /* *AR+0% circ — fix 2026-06-01 */
            }
            s->t = xval_dc;
            int64_t prod_dc = (int64_t)(int16_t)xval_dc * (int64_t)(int16_t)xval_dc;
            if (s->st1 & ST1_FRCT) prod_dc <<= 1;
            s->a = sext40(s->a + prod_dc);
            return consumed + s->lk_used;
        }

        /* CA/CB handled by the unified C8/C9/CA/CB block below. */
        /* CF: variant parallel or DELAY */
        if (hi8 == 0xCF) {
            /* Treat as NOP for now — rare instruction */
            return consumed + s->lk_used;
        }
        /* RPTB[D] pmad — Block repeat (2 words)
         * C2xx: RPTB pmad, C3xx: RPTBD pmad (delayed)
         * Per SPRU172C: RSA = PC+2, REA = pmad, BRAF = 1 */
        if (hi8 == 0xC2 || hi8 == 0xC3 || hi8 == 0xC6 || hi8 == 0xC7) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC5) {
            /* STUB-NOP : tic54x dit 0xC5 = ST||family (parallel).
             * Ancienne classification qemu = PSHM MMR (incorrect — vrai
             * PSHM est en 0x4A, correctement décodé ligne 3816).
             * Le sp-- ici causait des pushes fantômes. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xCD) {
            /* STUB-NOP : tic54x dit 0xCD = ST||family (parallel).
             * Ancienne classification qemu = POPM MMR (incorrect — vrai
             * POPM est en 0x8A, fixé 2026-05-08).
             * Le sp++ ici causait des pops fantômes. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xCE) {
            /* STUB-NOP : tic54x dit 0xCE = ST||family (parallel).
             * Ancienne classification qemu = FRAME #k (incorrect — FRAME
             * n'a pas de hi8 fixe, encodage différent).
             * Le sp+=k ici causait des sauts SP arbitraires. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xC4) {
            /* C4xx: PSHD dmad (push data from absolute addr) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC0 || hi8 == 0xC1) {
            /* PSHD Smem / RPT Smem variants */
            addr = resolve_smem(s, op, &ind);
            if (hi8 == 0xC0) {
                /* PSHD Smem */
                s->sp--;
                data_write(s, s->sp, data_read(s, addr));
            } else {
                /* RPT Smem */
                s->rpt_count = data_read(s, addr);
                s->rpt_active = true; s->rpt_arming = true;
                s->pc += consumed;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCC) {
            /* CCxx: SACCD Smem, ARmem — Store Acc Conditionally (1-word)
             * Per SPRU172C: conditionally store AH or BH to Smem.
             * Simplified: always store (condition always true). */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDA) {
            /* DAxx: RPTBD pmad (block repeat delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4); /* delayed: skip 2 delay slots */
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDD) {
            /* STUB-NOP : tic54x dit 0xDD = ST||family (parallel) — base
             * 0xDC00 mask 0xFC00. Ancienne classification qemu = POPD Smem
             * (incorrect — vrai POPD en 0x8B, neutralisé en stub).
             * Le sp++ ici causait le SP runaway post-POPM-fix observé
             * 2026-05-08 (~13k faux pops en 64k insn). Neutralisé. */
            return 1;
        }
        if (hi8 == 0xDE) {
            /* STUB-NOP : tic54x dit 0xDE = ST||family (parallel).
             * Ancienne classification qemu = POPD dmad 2-word (incorrect).
             * Le sp++ ici causait le SP runaway. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xDF) {
            /* DELAY Smem — shift delay line: data(Smem) → data(Smem+1)
             * Per SPRU172C: used with RPT for FIR filter delay lines */
            addr = resolve_smem(s, op, &ind);
            uint16_t dval = data_read(s, addr);
            data_write(s, addr + 1, dval);
            return consumed + s->lk_used;
        }
        /* 0xC8/C9/CA/CB: ST SRC, Ymem || LD Xmem, DST  (1-word parallel)
         *
         * Encoding per SPRU172C §5.5 (Parallel store + arithmetic format,
         * cross-checked against tic54x-opc.c entry "0xC800/0xFC00 st||ld") :
         *
         *   bit 15..10 : opcode (110010)
         *   bit  9     : reserved (used to disambiguate; here: 0 for C8/CA,
         *                bit 9 of 0xC9/CB still in opcode space — but the
         *                effective operand bits for parallel are 7:0)
         *   bit  8     : SRC accumulator select (0 = A, 1 = B)
         *   bits 7:6   : Xmod  (0=*ARi  1=*ARi+  2=*ARi-  3=*ARi+0%)
         *   bits 5:4   : Xar   (00=AR2, 01=AR3, 10=AR4, 11=AR5) — only AR2..AR5
         *   bits 3:2   : Ymod  (same encoding as Xmod)
         *   bits 1:0   : Yar   (same encoding as Xar)
         *
         * Bug fix 2026-05-08 v2 evidence (DUAL-OP-INTERPRET log) :
         *   Previously decoded as `xar=(op>>4)&7`, `yar=op&7` (3-bit AR
         *   field) with bit 7 = Xmod ±, bit 3 = Ymod ±. That picked
         *   AR0/AR1 instead of AR2/AR3 and made post-mod always ± with
         *   no support for "no mod" or `*ARi+0%`. When firmware loaded
         *   AR1=0x0018 (= MMR_SP) for an unrelated reason, the *AR1
         *   write landed on the SP MMR slot — observed catastrophes
         *   Δ=+16601 / -16640 at PC=0x7818 / 0x786b are the consequence.
         *
         * Note on 0xCA/CB : per tic54x-opc.c, 0xC800 mask 0xFC00 covers
         * 0xC800..0xCBFF for ST||LD (single instruction class). The
         * earlier emulator split CA/CB into a separate block — that
         * block is now removed, the C8..CB handler is unified here. */
        if (hi8 >= 0xC8 && hi8 <= 0xCB) {
            int s_acc = (hi8 & 0x01) ? 1 : 0;          /* C9/CB store from B */
            int xmod  = (op >> 6) & 0x03;
            int xar   = ((op >> 4) & 0x03) + 2;        /* AR2..AR5 */
            int ymod  = (op >> 2) & 0x03;
            int yar   = (op & 0x03) + 2;               /* AR2..AR5 */
            int d_acc = s_acc ? 0 : 1;                 /* LD into the OTHER acc */
            int64_t st_val = s_acc ? s->b : s->a;
            /* STLD-SP (patch #2 diag, gated CALYPSO_DEBUG=STLD-SP) : au site
             * SP-CATASTROPHE (défaut PC=0xa0e7, env CALYPSO_TRACE_STLD_PC),
             * dump la cible RÉELLE = AR[yar] PRÉ-modify + flag MMR_SP. Tranche
             * le fork CC : si AR[yar]==MMR_SP(0x18) → le ST écrit SP (AR stale
             * via STM skippé) ; sinon → write DARAM légal. */
            {
                static int stld_pc = -1;
                if (stld_pc < 0) {
                    const char *e = getenv("CALYPSO_TRACE_STLD_PC");
                    stld_pc = (e && *e) ? (int)strtol(e, NULL, 0) : 0xa0e7;
                }
                if (s->pc == (uint16_t)stld_pc) {
                    C54_DBG("STLD-SP",
                        "STLD-SP op=0x%04x PC=0x%04x s_acc=%d yar=AR%d "
                        "tgt(AR%d_pre)=0x%04x is_MMR_SP=%d xar=AR%d AR%d=0x%04x "
                        "st_val=0x%010llx",
                        op, s->pc, s_acc, yar, yar, s->ar[yar],
                        (s->ar[yar] == MMR_SP), xar, xar, s->ar[xar],
                        (unsigned long long)(st_val & 0xFFFFFFFFFFULL));
                }
            }
            data_write(s, s->ar[yar], (uint16_t)(st_val & 0xFFFF));
            uint16_t ld_val = data_read(s, s->ar[xar]);
            int64_t loaded = (int64_t)(int16_t)ld_val << 16;
            if (d_acc) s->b = sext40(loaded); else s->a = sext40(loaded);
            switch (xmod) {
            case 0: break;                             /* *ARi (no mod) */
            case 1: s->ar[xar]++; break;               /* *ARi+ */
            case 2: s->ar[xar]--; break;               /* *ARi- */
            case 3:                                    /* *ARi+0% — CIRCULAIRE modulo BK
                                                        * (était linear += AR0 → AR drift
                                                        * 16-bit vers 0x18=MMR_SP → SP-CATAS).
                                                        * Miroir du single-operand case 0xE. */
                if (s->bk) {
                    uint16_t base = s->ar[xar] - (s->ar[xar] % s->bk);
                    uint16_t v = s->ar[xar] + s->ar[0];
                    if (v >= (uint16_t)(base + s->bk)) v -= s->bk;
                    s->ar[xar] = v;
                } else {
                    s->ar[xar] += s->ar[0];            /* BK=0 → linéaire (pas de circ) */
                }
                break;
            }
            switch (ymod) {
            case 0: break;
            case 1: s->ar[yar]++; break;
            case 2: s->ar[yar]--; break;
            case 3:                                    /* *ARi+0% — circulaire modulo BK */
                if (s->bk) {
                    uint16_t base = s->ar[yar] - (s->ar[yar] % s->bk);
                    uint16_t v = s->ar[yar] + s->ar[0];
                    if (v >= (uint16_t)(base + s->bk)) v -= s->bk;
                    s->ar[yar] = v;
                } else {
                    s->ar[yar] += s->ar[0];
                }
                break;
            }
            return consumed + s->lk_used;
        }
        goto unimpl;

    default:
        break;
    }

unimpl:
    s->unimpl_count++;
    if (s->unimpl_count <= 200 || op != s->last_unimpl) {
        C54_LOG("UNIMPL @0x%04x: 0x%04x (hi8=0x%02x) [#%u]",
                s->pc, op, hi8, s->unimpl_count);
        s->last_unimpl = op;
    }
    return consumed + s->lk_used;
}

/* ================================================================
 * Main execution loop
 * ================================================================ */

/* DSP idle fast-forward — simulator optimisation, NOT a hack.
 *
 * The Calypso DSP polls its task slots in NDB and write pages while
 * waiting for ARM/TPU to post work. Empirically this dispatcher loop
 * lives in PROM1 mirror at PC 0xe9ac..0xe9b7 (8-instruction body cycled
 * ~285k times per 1.4G insn window when nothing pending). Each iteration
 * costs C-level MAC/branch emulation that ends up consuming 80%+ of host
 * CPU for zero useful work, making QEMU run ~3x slower than wall-clock
 * GSM and starving the BTS scheduler of CLK INDs.
 *
 * Detection: PC inside the polling range AND all four task fields in
 * both write pages are zero AND no interrupt pending. When confirmed,
 * advance cycles/insn_count without invoking c54x_exec_one. The DSP
 * exits idle naturally next iteration if either:
 *   - ARM writes a task field (mirrored via calypso_dsp_write to
 *     s->data[0x0800+offset])
 *   - An IRQ fires (calypso_c54x_interrupt_ex sets s->ifr)
 *   - PC moves outside the range (shouldn't happen while polling)
 *
 * Env vars (default ON) :
 *   CALYPSO_DSP_IDLE_FF=0          disable
 *   CALYPSO_DSP_IDLE_RANGE=lo:hi   override hex PC range
 */
#define DSP_IDLE_FF_MAX_RANGES 4
static bool dsp_idle_fast_forward(C54xState *s, int *consumed_out)
{
    static int     ff_enabled = -1;
    static int     ff_n_ranges = 0;
    static uint16_t ff_lo[DSP_IDLE_FF_MAX_RANGES];
    static uint16_t ff_hi[DSP_IDLE_FF_MAX_RANGES];
    static uint64_t ff_hits = 0;

    if (ff_enabled < 0) {
        const char *e = getenv("CALYPSO_DSP_IDLE_FF");
        ff_enabled = (!e || *e != '0') ? 1 : 0;
        /* Defaults: two empirically observed dispatcher loops in the
         * stock layer1.highram.elf firmware:
         *   1) 0xe9ac..0xe9b7 — PROM1 mirror, init/SP-aware path
         *   2) 0xcc62..0xcc6f — PROM0 page 0, runtime mailbox poll loop
         * Override via CALYPSO_DSP_IDLE_RANGE="lo1:hi1,lo2:hi2,..."
         * (max 4 ranges). Each range is hex. Empty = use defaults. */
        const char *r = getenv("CALYPSO_DSP_IDLE_RANGE");
        if (r && *r) {
            const char *p = r;
            while (*p && ff_n_ranges < DSP_IDLE_FF_MAX_RANGES) {
                unsigned lo, hi;
                if (sscanf(p, "%x:%x", &lo, &hi) == 2 && lo <= hi &&
                    lo <= 0xFFFF && hi <= 0xFFFF) {
                    ff_lo[ff_n_ranges] = (uint16_t)lo;
                    ff_hi[ff_n_ranges] = (uint16_t)hi;
                    ff_n_ranges++;
                }
                while (*p && *p != ',') p++;
                if (*p == ',') p++;
            }
        }
        if (ff_n_ranges == 0) {
            ff_lo[0] = 0xe9ac; ff_hi[0] = 0xe9b7;
            ff_lo[1] = 0xcc62; ff_hi[1] = 0xcc6f;
            ff_n_ranges = 2;
        }
        char buf[160] = ""; int blen = 0;
        for (int i = 0; i < ff_n_ranges; i++) {
            blen += snprintf(buf + blen, sizeof(buf) - blen,
                             "%s0x%04x..0x%04x",
                             i ? "," : "", ff_lo[i], ff_hi[i]);
        }
        C54_LOG("DSP IDLE FF: %s, ranges=[%s]",
                ff_enabled ? "enabled" : "disabled", buf);
    }
    if (!ff_enabled) return false;
    bool in_range = false;
    for (int i = 0; i < ff_n_ranges; i++) {
        if (s->pc >= ff_lo[i] && s->pc <= ff_hi[i]) {
            in_range = true;
            break;
        }
    }
    if (!in_range) return false;

    /* Task slots in both write pages — DSP word addresses :
     *   page 0 : 0x0800 (d_task_d), 0x0802 (d_task_u),
     *            0x0804 (d_task_md), 0x0807 (d_task_ra)
     *   page 1 : 0x0814, 0x0816, 0x0818, 0x081B (offsets +0x14)
     */
    if (s->data[0x0800] | s->data[0x0802] | s->data[0x0804] | s->data[0x0807] |
        s->data[0x0814] | s->data[0x0816] | s->data[0x0818] | s->data[0x081B]) {
        return false;  /* something pending → exec normally */
    }
    /* Pending IRQ would also break us out of the dispatcher next iter. */
    if (!(s->st1 & ST1_INTM) && (s->ifr & s->imr)) {
        return false;
    }

    /* Fast-forward this dispatcher iteration.
     *
     * Cycle-budget calibration: real C54x at 65 MHz means 1 cycle ≈ 15 ns.
     * The dispatcher body is ~8 instructions per pass (matches the 8 hot
     * PCs observed). One pass ≈ 8 cycles ≈ 120 ns of *DSP* time.
     *
     * Per Claude web 2026-05-07 review: previously this returned a
     * fixed 8-cycle skip per call regardless of host wall time. Combined
     * with c54x_run(256000) that meant a single tick callback could
     * burn through 32k FF iterations in microseconds host time but
     * accumulate the full 256k cycles credit on the DSP — the net
     * effect on QEMU virtual time was minimal (DSP cycles aren't a
     * QEMU clock anyway), so this isn't itself the cause of the BTS
     * timing skew. But to match wall-clock more honestly we now cap
     * the FF run length per c54x_run invocation: at most enough skips
     * to consume the budget (n_insns) without overshooting.
     *
     * The actual wall-clock alignment (CLK IND cadence) is owned by
     * the TDMA timer in calypso_trx.c, not by this function. */
    *consumed_out = 8;
    ff_hits++;
    if ((ff_hits & 0xFFFFFFu) == 0) {
        C54_LOG("DSP IDLE FF: %llu skips so far (PC=0x%04x SP=0x%04x)",
                (unsigned long long)ff_hits, s->pc, s->sp);
    }
    return true;
}

/* === CALYPSO_TRAP_OOR hook v2 (root-cause probe SP descent 0..checkpoint) ===
 * v2 redesign: T1/T2 dropped (scheduler exonerated → SP clobber lives in
 * legit code, whitelist can't see it). Pure observability:
 *   - g_sp_trail[256] : SP changes with |Δ|>32 (scheduler reloads, large
 *     allocations) — skip push/pop ±1 noise.
 *   - sp_low watermark : every new low logged (PC-coalesced power-of-10)
 *     — catches BOTH absolute reloads AND push-drain runaway.
 *   - Per-event A_low captured (= candidate STL A,Smem source).
 *   - Halt at fixed checkpoint (env CALYPSO_TRAP_CHECKPOINT, default 4.2M
 *     = just after the insn=4.09M SP recovery 0x0008→0x2900). */

static struct {
    unsigned insn;
    uint16_t old_sp, new_sp, exec_pc, exec_op, a_low;
} g_sp_trail[256];
static unsigned g_sp_trail_idx = 0;

/* sp_low watermark — coalesced by PC */
static uint16_t g_sp_low = 0xFFFF;
static uint16_t g_sp_low_pc = 0xFFFF;
static unsigned g_sp_low_hits_at_pc = 0;
static unsigned g_sp_low_distinct_pcs = 0;

/* SP-decrement histogram per-PC (fix 2026-05-24 v3 — Claude web correction).
 *
 * Gating par VALEUR SP (pas insn_count) — robuste à :
 *   - DSP idle fast-forward (dsp_idle_fast_forward L5937 inflate insn_count
 *     sans exécuter d'opcodes)
 *   - jitter externe wall-clock (bridge/osmocon/BTS sur UDP+PTY → instant
 *     guest où arrive un burst varie run-à-run, insn_count des events
 *     déclenchés par bursts pas stable)
 *
 * Logique : armé quand SP descend SOUS le plateau (default < 0x2000, sous
 * 0x3fb0 où SP stationne 632k→3.5M insns du run jackpot). Reste armé
 * jusqu'au dump quand SP < 0x0100 (proche underflow). Pendant cette fenêtre,
 * compte chaque SP-décrement par PC.
 *
 * 3 bénéfices :
 *   1. Auto-aligne sur la descente quels que soient FF et jitter externe
 *   2. Rend la question FF caduque (descente = real insns, pas idle-poll)
 *   3. Exclut churn équilibré du plateau (PSHM/POP matched à PCs distincts
 *      pollue insn-window mais pas SP-window — leaker domine mécaniquement)
 *
 * Override env :
 *   CALYPSO_SP_HIST_ARM   (default 0x2000) — threshold pour armer
 *   CALYPSO_SP_HIST_DUMP  (default 0x0100) — threshold pour dumper
 *
 * Capture toutes voies SP-write : direct s->sp--, MMR_SP via data_write
 * callback, IRQ push (audit couverture 2026-05-24 : tous paths passent
 * par s->sp variable). */
typedef struct {
    uint16_t pc;
    uint16_t op_last;
    uint32_t dec_count;
    int32_t  delta_sum;   /* négatif = drain net */
} SpDecEntry;
#define SP_HIST_MAX 512
static SpDecEntry g_sp_dec_hist[SP_HIST_MAX];
static unsigned   g_sp_dec_used = 0;
static unsigned   g_sp_dec_total_events = 0;
static uint16_t   g_sp_dec_arm_threshold = 0;
static uint16_t   g_sp_dec_dump_threshold = 0;
static int        g_sp_dec_enabled = -1;
static int        g_sp_dec_armed = 0;
static int        g_sp_dec_dumped = 0;
static unsigned   g_sp_dec_arm_insn = 0;   /* insn at which we armed */
static uint16_t   g_sp_dec_arm_sp = 0;     /* SP value at arm */

/* === Raw SP ring buffer (Patch 3 — 2026-05-25, rev 2) ===
 * Per-iteration record of (insn, PC, SP, op) at top-of-loop, no filter,
 * no sign classification. Plusieurs triggers configurables.
 *
 * Rev 1 (floor-cross) : a montré que le « plongeon SP→0 » est en réalité
 * un wrap-forward par pops dans un boot-stub spiral (PC=0x0000/0x0001
 * en boucle, SP++ par RET). Le kill réel = un RET corrompu qui saute
 * à 0x0000 BIEN AVANT le wrap, dans [3.5M, 4.09M] insns. Floor-cross
 * arrive 600k insns trop tard, déjà dans le spiral.
 *
 * Rev 2 (bootstub-entry) : trigger sur l'EDGE prev_pc ∉ [0x00,0x7F] →
 * s->pc ∈ [0x00,0x7F]. Capture la transition exacte = le RET fauteur
 * + son SP + le mot poppé (mem[topgate_last_sp]). Discrimine 2 bugs
 * radicalement différents :
 *   - SP valide (~0x3fbb) + mem[SP]=0 → return slot écrasé par un
 *     write sauvage. fd28-fd2a n'y change rien. À chasser autrement.
 *   - SP en non-stack (~0x2bc0) → famille 0xfd2a A=AR4. fd28-fd2a
 *     devient le fix.
 *
 * Env gates :
 *   CALYPSO_SP_RING=1          active (default OFF, zéro coût sinon)
 *   CALYPSO_SP_RING_MAX=N      cap dumps par run (default 4)
 *   CALYPSO_SP_RING_TRIG=mode  floor|bootstub|both (default bootstub)
 *   CALYPSO_SP_RING_INSN_MIN=N skip first N insns (default 1000000 — le
 *                              firmware Calypso fait des CALL légitimes
 *                              au boot stub 0x0000/0x0001 en phase init,
 *                              le 1er CALL captérait un faux positif et
 *                              consommerait le one-shot. Le vrai bug
 *                              observé est dans [3.5M, 4.09M] insns) */
#define SP_RING_SZ 4096  /* must be power of 2 — bumped 512→4096 for
                          * coverage des 1000s d'insns avant le spiral */
typedef struct {
    unsigned insn;
    uint16_t pc;
    uint16_t sp;
    uint16_t op;
    uint16_t _pad;
} SpRingEntry;
static SpRingEntry g_sp_ring[SP_RING_SZ];
static unsigned    g_sp_ring_head = 0;
static uint64_t    g_sp_ring_total = 0;
static int         g_sp_ring_enabled = -1;
static unsigned    g_sp_ring_dump_count = 0;
static unsigned    g_sp_ring_dump_max = 0;
/* Trigger mode (rev 2) : 1 = floor-cross, 2 = bootstub-entry, 3 = both */
static int         g_sp_ring_trig_mode = 0;
static unsigned    g_sp_ring_insn_min  = 0;  /* skip first N insns (boot phase) */

static void sp_ring_record(unsigned insn, uint16_t pc, uint16_t sp, uint16_t op)
{
    if (g_sp_ring_enabled <= 0) return;
    SpRingEntry *e = &g_sp_ring[g_sp_ring_head];
    e->insn = insn; e->pc = pc; e->sp = sp; e->op = op;
    g_sp_ring_head = (g_sp_ring_head + 1) & (SP_RING_SZ - 1);
    g_sp_ring_total++;
}

static void sp_ring_dump(const char *trig, unsigned insn_now, uint16_t sp_now)
{
    if (g_sp_ring_enabled <= 0) return;
    if (g_sp_ring_dump_max && g_sp_ring_dump_count >= g_sp_ring_dump_max) return;
    g_sp_ring_dump_count++;
    fprintf(stderr,
        "[c54x] SP-RING DUMP[%s] @insn=%u sp_now=0x%04x total_recorded=%llu "
        "dump#%u\n",
        trig, insn_now, sp_now,
        (unsigned long long)g_sp_ring_total, g_sp_ring_dump_count);
    unsigned n = (g_sp_ring_total < SP_RING_SZ)
                 ? (unsigned)g_sp_ring_total : SP_RING_SZ;
    unsigned start = (g_sp_ring_total < SP_RING_SZ) ? 0 : g_sp_ring_head;
    for (unsigned k = 0; k < n; k++) {
        unsigned idx = (start + k) & (SP_RING_SZ - 1);
        SpRingEntry *e = &g_sp_ring[idx];
        fprintf(stderr,
            "[c54x] SP-RING[%u] insn=%u PC=0x%04x SP=0x%04x op=0x%04x\n",
            k, e->insn, e->pc, e->sp, e->op);
    }
}

static void sp_ring_init_lazy(void)
{
    if (g_sp_ring_enabled >= 0) return;
    const char *e = cdbg_env("SP-RING");
    g_sp_ring_enabled = (e && *e == '1') ? 1 : 0;
    const char *m = getenv("CALYPSO_SP_RING_MAX");
    g_sp_ring_dump_max = (m && *m) ? (unsigned)strtoul(m, NULL, 0) : 4u;
    /* Trigger mode parse (rev 2). Default = bootstub (le seul utile post
     * rev-1 — floor-cross firait dans le spiral, trop tard). */
    const char *t = getenv("CALYPSO_SP_RING_TRIG");
    if (!t || !*t || !strcmp(t, "bootstub")) g_sp_ring_trig_mode = 2;
    else if (!strcmp(t, "floor"))            g_sp_ring_trig_mode = 1;
    else if (!strcmp(t, "both"))             g_sp_ring_trig_mode = 3;
    else                                     g_sp_ring_trig_mode = 2;
    const char *im = getenv("CALYPSO_SP_RING_INSN_MIN");
    g_sp_ring_insn_min = (im && *im) ? (unsigned)strtoul(im, NULL, 0) : 1000000u;
    if (g_sp_ring_enabled) {
        fprintf(stderr,
            "[c54x] SP-RING enabled, sz=%u, max_dumps=%u, trig=%s, "
            "insn_min=%u\n",
            SP_RING_SZ, g_sp_ring_dump_max,
            g_sp_ring_trig_mode == 1 ? "floor" :
            g_sp_ring_trig_mode == 2 ? "bootstub" :
            g_sp_ring_trig_mode == 3 ? "both" : "?",
            g_sp_ring_insn_min);
    }
}

/* Rev 2 : detect edge PC entry into boot stub area [0x0000, 0x007F].
 * topgate_last_pc = PC of insn just executed (the RET that branched).
 * cur_pc          = destination = popped return address.
 * topgate_last_sp = SP before the RET pop.
 * cur_sp          = SP after the RET pop (= topgate_last_sp + 1 if 1-word).
 * Capture verbose state + dump ring to identify the corrupting RET.
 * Static cap : one detailed dump per run (le 1er, qui contient le
 * caller; subsequent fires sont des re-entries du même spiral). */
static int g_bootstub_dumped = 0;
static void sp_ring_check_bootstub_entry(C54xState *s,
                                         uint16_t prev_pc, uint16_t prev_op,
                                         uint16_t prev_sp, uint16_t cur_pc,
                                         uint16_t cur_sp, unsigned insn)
{
    if (g_sp_ring_enabled <= 0) return;
    if (!(g_sp_ring_trig_mode & 2)) return;
    if (g_bootstub_dumped) return;
    /* Skip boot phase : firmware fait des CALL légitimes au boot stub
     * 0x0000-0x0001 pendant l'init (LDMM SP,B est documenté). Le 1er
     * trigger sans gate fire à insn=145 et consomme le one-shot. */
    if (insn < g_sp_ring_insn_min) return;
    int was_inside = (prev_pc <= 0x007F);
    int now_inside = (cur_pc  <= 0x007F);
    if (was_inside || !now_inside) return;

    g_bootstub_dumped = 1;
    uint16_t popped = s->data[prev_sp & 0xFFFF];
    uint16_t neighborhood[8];
    for (int k = 0; k < 8; k++) {
        neighborhood[k] = s->data[(uint16_t)(prev_sp + k) & 0xFFFF];
    }
    fprintf(stderr,
        "[c54x] BOOTSTUB-ENTRY caught @insn=%u\n"
        "[c54x]   prev_pc=0x%04x prev_op=0x%04x  (the RET site = corrupter)\n"
        "[c54x]   cur_pc=0x%04x  (destination, in bootstub)\n"
        "[c54x]   prev_sp=0x%04x  cur_sp=0x%04x  (delta=%+d)\n"
        "[c54x]   mem[prev_sp]=0x%04x  (= popped return addr, must equal cur_pc)\n"
        "[c54x]   AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  ARP=%d DP=%d\n"
        "[c54x]   ST0=0x%04x ST1=0x%04x INTM=%d XPC=%d\n"
        "[c54x]   stack neighborhood mem[prev_sp..+7]: %04x %04x %04x %04x %04x %04x %04x %04x\n",
        insn, prev_pc, prev_op, cur_pc,
        prev_sp, cur_sp, (int)cur_sp - (int)prev_sp,
        popped,
        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
        arp(s), dp(s),
        s->st0, s->st1, !!(s->st1 & ST1_INTM), s->xpc,
        neighborhood[0], neighborhood[1], neighborhood[2], neighborhood[3],
        neighborhood[4], neighborhood[5], neighborhood[6], neighborhood[7]);

    /* Diagnostic discriminator — match user's discrimination criteria :
     *   SP valide (~0x3fbb plage observée) + popped==0 → return slot
     *     écrasé par write sauvage. 0xfd2a est innocent.
     *   SP en zone non-stack (~0x2bc0 ou similar buffer) → famille
     *     0xfd2a A=AR4. fd28-fd2a est le fix. */
    int sp_in_valid_stack = (prev_sp >= 0x3000 && prev_sp <= 0x5FFF);
    int sp_in_buffer_area = (prev_sp >= 0x2000 && prev_sp <= 0x2FFF);
    if (calypso_debug_enabled("BOOTSTUB-ENTRY")) fprintf(stderr,
        "[c54x] BOOTSTUB-ENTRY VERDICT: sp_in_valid_stack=%d "
        "sp_in_buffer_area=%d popped_is_zero=%d\n"
        "[c54x]   → %s\n",
        sp_in_valid_stack, sp_in_buffer_area, (popped == 0),
        sp_in_valid_stack && popped == 0
            ? "RETURN SLOT OVERWRITTEN (sauvage write near SP), audit ailleurs"
            : sp_in_buffer_area
            ? "SP IN NON-STACK BUFFER (likely 0xfd2a family), audit fd28-fd2a"
            : "INCONCLUSIVE — inspect ring + state above");

    sp_ring_dump("bootstub-entry", insn, cur_sp);
}

static void sp_hist_dump(const char *trig, unsigned insn_now, uint16_t sp_now)
{
    if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
        "[c54x] SP-HIST DUMP[%s] arm@(insn=%u,sp=0x%04x) now@(insn=%u,sp=0x%04x) "
        "events=%u distinct_pcs=%u\n",
        trig, g_sp_dec_arm_insn, g_sp_dec_arm_sp, insn_now, sp_now,
        g_sp_dec_total_events, g_sp_dec_used);

    /* Top-K par dec_count (trickle leak). */
    if (calypso_debug_enabled("SP-HIST")) fprintf(stderr, "[c54x] SP-HIST TOP BY COUNT (corrupteur trickle):\n");
    for (unsigned k = 0; k < 20 && k < g_sp_dec_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_sp_dec_used; i++) {
            if (g_sp_dec_hist[i].dec_count > g_sp_dec_hist[best].dec_count)
                best = i;
        }
        if (best != k) {
            SpDecEntry tmp = g_sp_dec_hist[k];
            g_sp_dec_hist[k] = g_sp_dec_hist[best];
            g_sp_dec_hist[best] = tmp;
        }
        if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
            "[c54x] SP-HIST #%u pc=0x%04x op_last=0x%04x dec_count=%u "
            "delta_sum=%d\n",
            k + 1, g_sp_dec_hist[k].pc, g_sp_dec_hist[k].op_last,
            g_sp_dec_hist[k].dec_count, g_sp_dec_hist[k].delta_sum);
    }

    /* Top-K par |delta_sum| (single-event jump corrupteur — 1 event huge). */
    if (calypso_debug_enabled("SP-HIST")) fprintf(stderr, "[c54x] SP-HIST TOP BY |delta_sum| (corrupteur single-jump):\n");
    for (unsigned k = 0; k < 10 && k < g_sp_dec_used; k++) {
        unsigned best = k;
        int32_t best_abs = g_sp_dec_hist[k].delta_sum < 0
                           ? -g_sp_dec_hist[k].delta_sum
                           :  g_sp_dec_hist[k].delta_sum;
        for (unsigned i = k + 1; i < g_sp_dec_used; i++) {
            int32_t a = g_sp_dec_hist[i].delta_sum < 0
                        ? -g_sp_dec_hist[i].delta_sum
                        :  g_sp_dec_hist[i].delta_sum;
            if (a > best_abs) { best = i; best_abs = a; }
        }
        if (best != k) {
            SpDecEntry tmp = g_sp_dec_hist[k];
            g_sp_dec_hist[k] = g_sp_dec_hist[best];
            g_sp_dec_hist[best] = tmp;
        }
        if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
            "[c54x] SP-HIST D#%u pc=0x%04x op_last=0x%04x dec_count=%u "
            "delta_sum=%d\n",
            k + 1, g_sp_dec_hist[k].pc, g_sp_dec_hist[k].op_last,
            g_sp_dec_hist[k].dec_count, g_sp_dec_hist[k].delta_sum);
    }
    g_sp_dec_dumped = 1;
}

static void sp_hist_account(uint16_t exec_pc, uint16_t exec_op,
                            uint16_t sp_before, uint16_t sp_now,
                            unsigned insn)
{
    if (g_sp_dec_enabled < 0) {
        const char *e_arm = getenv("CALYPSO_SP_HIST_ARM");
        const char *e_dump = getenv("CALYPSO_SP_HIST_DUMP");
        unsigned arm  = (e_arm  && *e_arm)  ? (unsigned)strtoul(e_arm,  NULL, 0) : 0x2000u;
        unsigned dump = (e_dump && *e_dump) ? (unsigned)strtoul(e_dump, NULL, 0) : 0x0100u;
        if (arm > 0xFFFF) arm = 0xFFFF;
        if (dump > 0xFFFF) dump = 0xFFFF;
        if (dump >= arm) dump = (arm > 0x100) ? (arm - 0x100) : 0;
        g_sp_dec_arm_threshold  = (uint16_t)arm;
        g_sp_dec_dump_threshold = (uint16_t)dump;
        g_sp_dec_enabled = calypso_debug_enabled("SP-HIST") ? 1 : 0;
        fprintf(stderr,
            "[c54x] SP-HIST gating SP-value : ARM<0x%04x DUMP<0x%04x\n",
            g_sp_dec_arm_threshold, g_sp_dec_dump_threshold);
    }
    if (!g_sp_dec_enabled) return;
    /* Patch 1 (2026-05-25) : freeze RETIRÉ. L'ancien `if (g_sp_dec_dumped)
     * return;` faisait du one-shot, donc tous les events post-1er-dump
     * étaient perdus. Sans freeze, plusieurs dumps consécutifs si SP
     * reste sous threshold — c'est borné en pratique par le rate-limit
     * du sp_ring_dump_max et par le edge-detect dans le top-of-loop. */

    /* Patch 2 (2026-05-25) : drop le cast (int16_t). Le wrap signé
     * mis-classifiait les chutes high→low en pop. Pure int32 sub :
     *   0x9006→0x0000 : delta = -36870 (correct, descent capturé)
     *   0xC000→0x0000 : delta = -49152 (correct, descent capturé)
     * Note : casse l'underflow wrap (0x2bc0→0xfff8 = +52280, vu comme
     * pop), mais l'histo n'est plus la source de vérité pour le kill
     * — c'est le ring buffer qui tranche. Histo = drift trickle uniquement. */
    if (!g_sp_dec_armed) {
        int32_t first_check = (int32_t)sp_now - (int32_t)sp_before;
        if (first_check < 0) {
            g_sp_dec_armed = 1;
            g_sp_dec_arm_insn = insn;
            g_sp_dec_arm_sp = sp_now;
            fprintf(stderr,
                "[c54x] SP-HIST ARMED @insn=%u SP=0x%04x (sp_before=0x%04x delta=%d) "
                "pc=0x%04x op=0x%04x\n",
                insn, sp_now, sp_before, first_check, exec_pc, exec_op);
        } else {
            return;  /* no negative delta yet — wait */
        }
    }

    /* Record event AVANT le dump check (fix 2026-05-24 v4 — sinon un
     * single-event jump qui franchit DUMP en une instruction est perdu). */
    int32_t delta = (int32_t)sp_now - (int32_t)sp_before;
    if (delta < 0) {
        g_sp_dec_total_events++;
        unsigned i;
        for (i = 0; i < g_sp_dec_used; i++) {
            if (g_sp_dec_hist[i].pc == exec_pc) break;
        }
        if (i == g_sp_dec_used) {
            if (g_sp_dec_used >= SP_HIST_MAX) {
                static int sat_log = 0;
                if (!sat_log) {
                    fprintf(stderr,
                        "[c54x] SP-HIST saturated (>%u distinct PCs) — "
                        "broaden if needed\n", SP_HIST_MAX);
                    sat_log = 1;
                }
            } else {
                g_sp_dec_hist[i].pc = exec_pc;
                g_sp_dec_hist[i].op_last = exec_op;
                g_sp_dec_hist[i].dec_count = 0;
                g_sp_dec_hist[i].delta_sum = 0;
                g_sp_dec_used++;
            }
        }
        if (i < g_sp_dec_used) {
            g_sp_dec_hist[i].op_last = exec_op;
            g_sp_dec_hist[i].dec_count++;
            g_sp_dec_hist[i].delta_sum += delta;
        }
        /* Log first 10 events verbatim — for single-event jumps the corrupteur
         * est dans les premiers events (souvent un seul mot dans le histo). */
        if (g_sp_dec_total_events <= 10) {
            if (calypso_debug_enabled("SP-HIST")) fprintf(stderr,
                "[c54x] SP-HIST EVENT #%u pc=0x%04x op=0x%04x "
                "sp_before=0x%04x sp_now=0x%04x delta=%d insn=%u\n",
                g_sp_dec_total_events, exec_pc, exec_op,
                sp_before, sp_now, delta, insn);
        }
    }

    /* DUMP : APRÈS l'accounting, vérifier seuil dump.
     * Patch 1 (2026-05-25) : edge-trigger only — dump quand SP croise
     * sous le floor (sp_before >= threshold && sp_now < threshold).
     * Rev 2 : gaté par g_sp_ring_trig_mode (bit 0 = floor). Par défaut
     * bootstub seulement, parce que floor-cross fire dans le spiral
     * (trop tard) — cf rev 1 finding. */
    if ((g_sp_ring_trig_mode & 1) &&
        sp_before >= g_sp_dec_dump_threshold &&
        sp_now    <  g_sp_dec_dump_threshold) {
        sp_ring_dump("sp-floor-cross", insn, sp_now);
        sp_hist_dump("sp-floor-cross", insn, sp_now);
    }
}

static void dsp_trap_dump(C54xState *s, uint16_t exec_pc, uint16_t exec_op,
                          uint16_t sp_before, const char *trig)
{
    if (calypso_debug_enabled("TRAP")) fprintf(stderr,
        "[c54x] TRAP[%s] insn=%u exec_pc=0x%04x exec_op=0x%04x "
        "next_pc=0x%04x sp_before=0x%04x sp_now=0x%04x INTM=%d\n",
        trig, s->insn_count, exec_pc, exec_op, s->pc,
        sp_before, s->sp, !!(s->st1 & ST1_INTM));
    if (calypso_debug_enabled("TRAP")) fprintf(stderr, "[c54x] TRAP pc_ring[-16..-1]:");
    for (int i = 16; i >= 1; i--)
        fprintf(stderr, " %04x", pc_ring[(pc_ring_idx - i) & 255]);
    fprintf(stderr, "\n[c54x] TRAP sp_low=0x%04x at last_pc=0x%04x hits_at_pc=%u distinct_pcs=%u\n",
            g_sp_low, g_sp_low_pc, g_sp_low_hits_at_pc, g_sp_low_distinct_pcs);
    /* SP-HIST dump (fix v3 2026-05-24 — SP-windowed, no-insn-dep). */
    if (g_sp_dec_used > 0 && !g_sp_dec_dumped)
        sp_hist_dump("trap", s->insn_count, s->sp);
    fprintf(stderr, "[c54x] TRAP sp_trail[-256..-1] (|Δ|>32 only; insn old->new @pc op A_low):\n");
    for (int i = 256; i >= 1; i--) {
        unsigned k = (g_sp_trail_idx - i) & 255;
        if (g_sp_trail[k].insn == 0 && g_sp_trail[k].exec_pc == 0) continue;
        fprintf(stderr, "  %u  %04x->%04x  pc=%04x op=%04x A_low=%04x\n",
                g_sp_trail[k].insn, g_sp_trail[k].old_sp,
                g_sp_trail[k].new_sp, g_sp_trail[k].exec_pc,
                g_sp_trail[k].exec_op, g_sp_trail[k].a_low);
    }
    fprintf(stderr,
        "[c54x] TRAP regs A=%010llx B=%010llx T=%04x  "
        "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  "
        "BK=%04x ARP=%d DP=%d  ST0=%04x ST1=%04x PMST=%04x  "
        "RSA=%04x REA=%04x BRC=%d  IFR=%04x IMR=%04x XPC=%d\n",
        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
        s->t,
        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
        s->bk, (s->st0 >> 13) & 7, s->st0 & 0x1FF,
        s->st0, s->st1, s->pmst,
        s->rsa, s->rea, s->brc, s->ifr, s->imr, s->xpc);
    fprintf(stderr,
        "[c54x] TRAP prog[exec_pc..+3]=%04x %04x %04x %04x  "
        "prog[next_pc..+3]=%04x %04x %04x %04x\n",
        s->prog[exec_pc], s->prog[(uint16_t)(exec_pc+1)],
        s->prog[(uint16_t)(exec_pc+2)], s->prog[(uint16_t)(exec_pc+3)],
        s->prog[s->pc], s->prog[(uint16_t)(s->pc+1)],
        s->prog[(uint16_t)(s->pc+2)], s->prog[(uint16_t)(s->pc+3)]);
}

/* Optional per-instruction PC histogram (16-bit PC index). When non-NULL, every executed DSP
 * instruction increments g_pc_histogram[pc] — RELIABLE at any DSP:MCU step ratio (samples inside
 * the run loop, not once per MCU tick). NULL by default => one null-check per insn, no behaviour
 * change. Set via c54x_set_pc_histogram(). Idle fast-forward credits its skipped cycles to the
 * idle PC so "time spent" stays faithful. */
uint32_t *g_pc_histogram = 0;
void c54x_set_pc_histogram(uint32_t *h) { g_pc_histogram = h; }
/* Zero the histogram in place (keeps the same buffer registered). Called at the
 * loader1->run-mode warm reset so a PCHIST dump reflects RUN-MODE only — the
 * one-time cold-upload loader1 CRC verification (~524k hits at 0x8031, dsp_steps
 * 37k-865k) otherwise dominates the cumulative top-30 and masks the real run-mode
 * hot loops. See dsp54_warm_reset (gated, default-on). */
void c54x_reset_pc_histogram(void) {
    if (g_pc_histogram) memset(g_pc_histogram, 0, 0x10000u * sizeof(uint32_t));
}

/* Conditional halt: when the DSP reaches (PC & 0xFFFF) == g_halt_pc for the g_halt_after-th time,
 * the run loop sets g_halted and breaks BEFORE executing that instruction. The host (mad2) detects
 * g_halted, dumps the full DSP memory map, and stops. Lets us freeze the live machine at an exact
 * PC/hit and disassemble the REAL running memory instead of inferring. 0xFFFFFFFF = disabled. */
uint32_t g_halt_pc    = 0xFFFFFFFFu;
uint32_t g_halt_after = 1;
uint32_t g_halt_hits  = 0;
int      g_halted     = 0;

/* Rolling ring of the last PC_RING_N executed PCs (with xpc) — recorded per instruction in the run
 * loop. Dumped on halt to reveal the EXACT control flow into the halt PC (e.g. the branch/CALA that
 * jumped into a data region). Enabled once a halt is armed (no cost otherwise). */
#define PC_RING_N 1024
uint16_t g_pcring_pc[PC_RING_N];
uint8_t  g_pcring_xpc[PC_RING_N];
uint32_t g_pcring_idx = 0;      /* total recorded; (idx-1)&mask = newest */
int      g_pcring_on  = 0;
int c54x_pcring_size(void) { return PC_RING_N; }

void c54x_set_halt(uint32_t pc, uint32_t after) {
    g_halt_pc = pc; g_halt_after = (after ? after : 1); g_halt_hits = 0; g_halted = 0;
    g_pcring_on = 1;   /* record the PC ring so the halt dump shows the path into the halt PC */
}
int c54x_halted(void) { return g_halted; }

/* SP-floor halt: freeze right AFTER the instruction that drops SP below g_halt_sp — catches the
 * over-pop that drives the stack into the MMR region. 0 = disabled. */
uint16_t g_halt_sp = 0;
void c54x_set_halt_sp(uint16_t floor) { g_halt_sp = floor; if (floor) g_pcring_on = 1; }

/* Dump the SP-event ring (g_spring): the last 64 stack changes with the PC/op/delta responsible —
 * names the exact push/pop instructions leading to a stack imbalance. (f is a FILE*.) */
void c54x_dump_sp_ring(void *fv) {
    FILE *f = (FILE *)fv;
    int n = g_spring_idx < 64 ? (int)g_spring_idx : 64;
    fprintf(f, "# last %d SP events (oldest->newest): pc op delta sp\n", n);
    for (int k = n; k >= 1; k--) {
        struct sp_evt *e = &g_spring[(g_spring_idx - (uint32_t)k) & 63];
        fprintf(f, "  pc=0x%04X op=0x%04X delta=%+d -> sp=0x%04X\n", e->pc, e->op, e->delta, e->sp);
    }
}

void c54x_set_no_xpc(C54xState *s, int on)
{
    s->no_xpc = (on != 0);
}

void c54x_set_timer_enabled(C54xState *s, int on)
{
    s->timer_enabled = (on != 0);
    if (on) {
        /* Arm per SPRU131G §8.8 reset state: TIM=PRD=FFFFh, TSS cleared —
         * "At reset, TSS is cleared and the timer immediately starts timing."
         * (The inert legacy model parked TSS=1.) */
        s->data[TCR_ADDR] &= ~TCR_TSS;
    }
}

/* On-chip timer (SPRU131G §8.8): PSC decrements once per CPU clock; when it
 * decrements past 0 it reloads from TDDR and TIM decrements; when TIM
 * decrements past 0 it reloads from PRD and TINT fires (vec 19, IMR bit 3).
 * Called with the per-c54x_run insn budget — including while IDLE, since the
 * firmware's idle loop uses `idle 1` (CPU halted, peripheral clocks running);
 * the TINT wake is exactly how the RTOS tick resumes it. */
static inline void c54x_timer_tick(C54xState *s, int cycles)
{
    if (!s->timer_enabled || (s->data[TCR_ADDR] & TCR_TSS)) return;
    while (cycles-- > 0) {
        if (s->timer_psc == 0) {
            s->timer_psc = s->data[TCR_ADDR] & TCR_TDDR_MASK;
            if (s->data[TIM_ADDR] == 0) {
                s->data[TIM_ADDR] = s->data[PRD_ADDR];
                c54x_interrupt_ex(s, 19, 3);    /* TINT */
            } else {
                s->data[TIM_ADDR]--;
            }
        } else {
            s->timer_psc--;
        }
    }
}

int c54x_run(C54xState *s, int n_insns)
{
    int executed = 0;

    c54x_timer_tick(s, n_insns);

    /* Log first 10 instructions of each run (for 2nd cycle debug) */
    static int run_num = 0;
    run_num++;

    /* SP history ring buffer (64 entries × insn/PC/SP). Sampled every
     * 1M insns at top of run-loop. Dumped on STATE-DUMP. Reveals whether
     * SP descends monotonically (cumulative leak — each ISR entry leaks
     * one stack frame) or oscillates around a value (one big initial
     * drop then steady-state). Different fixes. */
    static struct { unsigned insn; uint16_t pc; uint16_t sp; } sp_ring[64];
    static unsigned sp_ring_idx = 0;
    static unsigned next_sp_sample = 1000000u;
    if (s->insn_count >= next_sp_sample) {
        next_sp_sample += 1000000u;
        sp_ring[sp_ring_idx & 63].insn = s->insn_count;
        sp_ring[sp_ring_idx & 63].pc   = s->pc;
        sp_ring[sp_ring_idx & 63].sp   = s->sp;
        sp_ring_idx++;
    }

    /* XPC tracking probe (2026-05-15 nuit, per Claude web Q1).
     * Hypothèse à valider : le path completion CCCH demod passe par PROM1
     * (XPC=1) via le B 0x9ab1 à 0x19aac. Si XPC=1 jamais atteint → bug
     * dans le route initial. Si atteint mais PC pas dans 0x9aac+ → entrée
     * OK mais pas cette zone. Tracking :
     *   - insn count par XPC (0..3)
     *   - dernier PC visité par XPC
     *   - first_visit_insn par XPC (= quand on entre en XPC=N pour la 1ère fois)
     *   - ring buffer 16 derniers PCs visités sous XPC=1 (zone d'intérêt)
     */
    {
        static uint64_t xpc_insn_count[4] = {0};
        static uint16_t xpc_last_pc[4]    = {0};
        static uint64_t xpc_first_insn[4] = {0,0,0,0};
        static uint16_t xpc1_pc_ring[16];
        static unsigned xpc1_pc_ring_idx = 0;
        static unsigned xpc1_pc_ring_count = 0;
        static unsigned next_xpc_dump = 100000000u;  /* 100M */
        uint8_t cur_xpc = s->xpc & 0x3;
        xpc_insn_count[cur_xpc]++;
        xpc_last_pc[cur_xpc] = s->pc;
        if (xpc_first_insn[cur_xpc] == 0)
            xpc_first_insn[cur_xpc] = s->insn_count;
        if (cur_xpc == 1) {
            xpc1_pc_ring[xpc1_pc_ring_idx & 15] = s->pc;
            xpc1_pc_ring_idx++;
            xpc1_pc_ring_count++;
        }
        if (s->insn_count >= next_xpc_dump) {
            next_xpc_dump += 100000000u;
            if (calypso_debug_enabled("XPC-STATS")) fprintf(stderr,
                    "[c54x] XPC-STATS insn=%u counts: 0=%llu 1=%llu 2=%llu 3=%llu | "
                    "first_insn: 0=%llu 1=%llu 2=%llu 3=%llu | last_pc: 0=0x%04x 1=0x%04x 2=0x%04x 3=0x%04x\n",
                    s->insn_count,
                    (unsigned long long)xpc_insn_count[0],
                    (unsigned long long)xpc_insn_count[1],
                    (unsigned long long)xpc_insn_count[2],
                    (unsigned long long)xpc_insn_count[3],
                    (unsigned long long)xpc_first_insn[0],
                    (unsigned long long)xpc_first_insn[1],
                    (unsigned long long)xpc_first_insn[2],
                    (unsigned long long)xpc_first_insn[3],
                    xpc_last_pc[0], xpc_last_pc[1], xpc_last_pc[2], xpc_last_pc[3]);
            if (xpc1_pc_ring_count > 0) {
                /* Dernier 16 PCs visités sous XPC=1 (ring buffer) */
                if (calypso_debug_enabled("XPC1-PC-RING")) fprintf(stderr,
                        "[c54x] XPC1-PC-RING count=%u last16: "
                        "%04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x\n",
                        xpc1_pc_ring_count,
                        xpc1_pc_ring[(xpc1_pc_ring_idx-16)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-15)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-14)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-13)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-12)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-11)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-10)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-9)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-8)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-7)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-6)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-5)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-4)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-3)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-2)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-1)&15]);
            }
        }
    }

    /* DISPATCH-CALLER probe (2026-05-15 nuit, per Claude web).
     * Les 3 callers de 0x9aaf identifiés par scan PROM :
     *   PC=0x8815 : f074 9aaf  (B 0x9aaf depuis table @0x8810)
     *   PC=0x9296 : f274 9aaf  (BD 0x9aaf depuis routine spécifique)
     *   PC=0x9418 : f274 9aaf  (BD 0x9aaf depuis autre routine)
     * Log A, AR0..2, data[0x0828/9] à chaque hit. */
    if (s->pc == 0x8815 || s->pc == 0x9296 || s->pc == 0x9418) {
        static unsigned hit_counts[3] = {0, 0, 0};
        int idx = (s->pc == 0x8815) ? 0 : (s->pc == 0x9296) ? 1 : 2;
        hit_counts[idx]++;
        if (hit_counts[idx] <= 20 || hit_counts[idx] % 100 == 0) {
            fprintf(stderr,
                    "[c54x] DISPATCH-CALLER hit=%u pc=0x%04x "
                    "A=0x%010llx AR0=0x%04x AR1=0x%04x AR2=0x%04x "
                    "data[0x0828]=0x%04x data[0x0829]=0x%04x "
                    "data[0x083c]=0x%04x data[0x083d]=0x%04x insn=%u\n",
                    hit_counts[idx], s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->ar[0], s->ar[1], s->ar[2],
                    s->data[0x0828], s->data[0x0829],
                    s->data[0x083c], s->data[0x083d],
                    s->insn_count);
        }
    }

    /* AR7-INIT-CHAIN + MVMD-AR7-BRC + RPTB-ARMED probe (Claude web 2026-05-15
     * nuit étape 3). Diagnostic : valeur AR7 au moment du MVMD AR7,BRC à
     * PC=0x8208, sa chaîne causale (16 derniers writes AR7), et l'état BRC
     * post-RPTBD setup. */
    {
        static uint16_t prev_ar7 = 0xFFFF;
        static struct {
            uint16_t pc;
            uint16_t old_val;
            uint16_t new_val;
            uint64_t insn;
            uint8_t  xpc;
        } ar7_history[16] = {{0}};
        static unsigned ar7_hist_idx = 0;

        if (s->ar[7] != prev_ar7) {
            ar7_history[ar7_hist_idx & 15].pc = s->pc;
            ar7_history[ar7_hist_idx & 15].old_val = prev_ar7;
            ar7_history[ar7_hist_idx & 15].new_val = s->ar[7];
            ar7_history[ar7_hist_idx & 15].insn = s->insn_count;
            ar7_history[ar7_hist_idx & 15].xpc = s->xpc & 0x3;
            ar7_hist_idx++;
            prev_ar7 = s->ar[7];
        }

        /* (b) Snapshot complet à chaque hit de PC=0x8208 (MVMD AR7, BRC) */
        if (s->pc == 0x8208) {
            static unsigned mvmd_hits = 0;
            mvmd_hits++;
            if (mvmd_hits <= 20 || (mvmd_hits % 100) == 0) {
                if (calypso_debug_enabled("MVMD-AR7-BRC")) fprintf(stderr,
                        "[c54x] MVMD-AR7-BRC #%u AR7=0x%04x BRC_before=0x%04x "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR6=0x%04x DP=%d insn=%u\n",
                        mvmd_hits, s->ar[7], s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[6], dp(s),
                        s->insn_count);
                int n_hist = ar7_hist_idx < 16 ? ar7_hist_idx : 16;
                for (int i = 0; i < n_hist; i++) {
                    int slot = (ar7_hist_idx - n_hist + i) & 15;
                    if (calypso_debug_enabled("AR7-HIST")) fprintf(stderr,
                            "[c54x]   AR7-HIST[%d] pc=XPC%u:0x%04x 0x%04x->0x%04x insn=%llu\n",
                            i, ar7_history[slot].xpc, ar7_history[slot].pc,
                            ar7_history[slot].old_val, ar7_history[slot].new_val,
                            (unsigned long long)ar7_history[slot].insn);
                }
            }
        }

        /* (c) État RPTB après setup (PC=0x820c = delay slot post-RPTBD) */
        if (s->pc == 0x820c) {
            static unsigned rptb_hits = 0;
            rptb_hits++;
            if (rptb_hits <= 20 || (rptb_hits % 100) == 0) {
                if (calypso_debug_enabled("RPTB-ARMED")) fprintf(stderr,
                        "[c54x] RPTB-ARMED #%u BRC=0x%04x RSA=0x%04x REA=0x%04x "
                        "ST1=0x%04x (INTM=%d) insn=%u\n",
                        rptb_hits, s->brc, s->rsa, s->rea, s->st1,
                        (s->st1 >> 11) & 1, s->insn_count);
            }
        }
    }

    /* INT3-BLOCKED probe (Claude web 2026-05-15 nuit, étape 2).
     * Sample 1/1000 du context (PC/ST1/BRC/XPC) quand INT3 pending + INTM=1.
     * Discrimine : (a) opcode set INTM=1 sans clear (variante POPM),
     * (b) RPTB long non-interruptible (BRC > 0 partout),
     * (c) STM ST1 / MVDM ST1 brut. Cf matrice Claude web. */
    {
        static uint64_t blocked_count = 0;
        static uint16_t sample_pcs[32] = {0};
        static uint16_t sample_st1s[32] = {0};
        static uint16_t sample_brcs[32] = {0};
        static uint8_t  sample_xpcs[32] = {0};
        static unsigned sample_idx = 0;
        static unsigned next_blocked_dump = 100000000u;

        bool int3_pending_now = (s->ifr & 0x08) != 0;
        bool intm_set_now = ((s->st1 >> 11) & 1) != 0;

        if (int3_pending_now && intm_set_now) {
            blocked_count++;
            if ((blocked_count % 1000) == 0) {
                sample_pcs[sample_idx & 31] = s->pc;
                sample_st1s[sample_idx & 31] = s->st1;
                sample_brcs[sample_idx & 31] = s->brc;
                sample_xpcs[sample_idx & 31] = s->xpc & 0x3;
                sample_idx++;
            }
        }

        if (s->insn_count >= next_blocked_dump) {
            next_blocked_dump += 100000000u;
            if (calypso_debug_enabled("INT3-BLOCKED")) fprintf(stderr,
                    "[c54x] INT3-BLOCKED insn=%u blocked_total=%llu blocked_samples=%u\n",
                    s->insn_count,
                    (unsigned long long)blocked_count,
                    sample_idx);
            int n = sample_idx < 32 ? sample_idx : 32;
            for (int i = 0; i < n; i++) {
                int slot = (sample_idx - n + i) & 31;
                if (calypso_debug_enabled("INT3-BLOCKED-SAMPLE")) fprintf(stderr,
                        "[c54x] INT3-BLOCKED-SAMPLE pc=XPC%u:0x%04x st1=0x%04x brc=0x%04x\n",
                        sample_xpcs[slot], sample_pcs[slot],
                        sample_st1s[slot], sample_brcs[slot]);
            }
        }
    }

    /* IRQ-FRAME-HEALTH probe (Claude web 2026-05-15 nuit, étape 1).
     * Diagnostic timing TDMA vs wall-clock : INT3 = frame interrupt
     * (IMR bit 3, vec 19, addr 0xFFCC). Mesure fire/serviced/missed/latency.
     * Discrimine : ISR mal vectorisée (service<fire), TPU/TSP fail (fire=0),
     * compute trop lent (missed>0). Cause root LOST 3468 + variance XPC. */
    {
        static uint64_t int3_fire_count = 0;
        static uint64_t int3_serviced_count = 0;
        static uint64_t int3_missed_count = 0;
        static uint64_t last_int3_fire_insn = 0;
        static uint64_t last_int3_service_insn = 0;
        static uint64_t total_service_latency_insn = 0;
        static bool int3_pending_prev = false;
        static unsigned next_irq_dump = 200000000u;

        bool int3_now_pending = (s->ifr & 0x08) != 0;
        bool int3_just_fired = int3_now_pending && !int3_pending_prev;
        /* ISR enter approximation : INT3 cleared from IFR while INTM=0 */
        bool int3_just_serviced = !int3_now_pending && int3_pending_prev &&
                                  ((s->st1 >> 11) & 1) == 0;

        if (int3_just_fired) {
            int3_fire_count++;
            if (int3_pending_prev) {
                int3_missed_count++;
            }
            last_int3_fire_insn = s->insn_count;
        }
        if (int3_just_serviced) {
            int3_serviced_count++;
            if (last_int3_fire_insn > last_int3_service_insn) {
                total_service_latency_insn += (s->insn_count - last_int3_fire_insn);
            }
            last_int3_service_insn = s->insn_count;
        }
        int3_pending_prev = int3_now_pending;

        if (s->insn_count >= next_irq_dump) {
            next_irq_dump += 200000000u;
            uint64_t avg_latency = int3_serviced_count > 0
                ? total_service_latency_insn / int3_serviced_count : 0;
            double service_ratio = int3_fire_count > 0
                ? (double)int3_serviced_count / int3_fire_count : 0.0;
            if (calypso_debug_enabled("IRQ-FRAME-HEALTH")) fprintf(stderr,
                    "[c54x] IRQ-FRAME-HEALTH insn=%u int3_fire=%llu int3_serviced=%llu "
                    "int3_missed=%llu avg_latency_insn=%llu service_ratio=%.2f\n",
                    s->insn_count,
                    (unsigned long long)int3_fire_count,
                    (unsigned long long)int3_serviced_count,
                    (unsigned long long)int3_missed_count,
                    (unsigned long long)avg_latency,
                    service_ratio);
        }
    }

    /* EXIT-COMPUTE + IRQ-DURING-COMPUTE probe (Claude web 2026-05-15 nuit).
     * Le DSP tourne en XPC=2 dans zone hot 0xdf80..0xdfc0 (CCCH demod MAC loop).
     * Discrimine entre 3 hypothèses :
     *   (1) compute jamais exit (threshold non franchi)
     *   (2) IRQ jamais fire (TPU/TSP source manquante)
     *   (3) IRQ fire mais pas serviced (INTM stuck ou ISR mal vectorisée)
     * Matrice de décision basée sur exits_count + irq_pending_in_compute. */
    {
        static uint16_t last_pc_sample = 0;
        static uint8_t  last_xpc_sample = 0;
        static unsigned exits_count = 0;
        static unsigned irqs_pending_during_compute = 0;
        static unsigned int3_pending_during_compute = 0;
        static uint64_t insns_in_compute = 0;
        static unsigned next_compute_dump = 200000000u;

        bool in_compute_now = ((s->xpc & 0x3) == 2 &&
                               s->pc >= 0xdf80 && s->pc <= 0xdfc0);
        bool was_in_compute = (last_xpc_sample == 2 &&
                               last_pc_sample >= 0xdf80 && last_pc_sample <= 0xdfc0);

        if (was_in_compute && !in_compute_now) {
            exits_count++;
            if (exits_count <= 30 || exits_count % 200 == 0) {
                fprintf(stderr,
                        "[c54x] EXIT-COMPUTE #%u from=XPC%u:0x%04x to=XPC%u:0x%04x "
                        "A=0x%010llx IFR=0x%04x INTM=%d insn=%u\n",
                        exits_count,
                        last_xpc_sample, last_pc_sample,
                        s->xpc & 0x3, s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->ifr, (s->st1 >> 11) & 1, s->insn_count);
            }
        }

        if (in_compute_now) {
            insns_in_compute++;
            if (s->ifr != 0) {
                irqs_pending_during_compute++;
                if (s->ifr & 0x08) int3_pending_during_compute++;
            }
        }

        if (s->insn_count >= next_compute_dump) {
            next_compute_dump += 200000000u;
            fprintf(stderr,
                    "[c54x] COMPUTE-STATS insn=%u in_compute=%llu exits=%u "
                    "irq_pending_in_compute=%u int3_pending_in_compute=%u\n",
                    s->insn_count,
                    (unsigned long long)insns_in_compute,
                    exits_count,
                    irqs_pending_during_compute,
                    int3_pending_during_compute);
        }

        last_pc_sample = s->pc;
        last_xpc_sample = s->xpc & 0x3;
    }

    /* DISPATCH-ENTRY probe (per Claude web option 3 hybride).
     * Le dispatcher caller saute vers 0x8810 + task_id*3, où chaque entry =
     * { 0xf4e4 (FRET ou padding), 0xf074 (B opcode), <target> }.
     * On probe le PC qui correspond au début d'un entry (PC = 0x8810 + N*3).
     * task_id estimé = (PC - 0x8810) / 3.
     * Si entry exec OK → on lit data[PC+2] qui est le target. */
    if (s->pc >= 0x8810 && s->pc < 0x8900 && ((s->pc - 0x8810) % 3) == 0) {
        static unsigned entry_hits = 0;
        entry_hits++;
        if (entry_hits <= 50 || entry_hits % 200 == 0) {
            uint16_t entry_idx = (s->pc - 0x8810) / 3;
            uint16_t header = s->prog[s->pc];     /* normally 0xf4e4 */
            uint16_t branch = s->prog[s->pc + 1]; /* normally 0xf074 */
            uint16_t target = s->prog[s->pc + 2];
            if (calypso_debug_enabled("DISPATCH-ENTRY")) fprintf(stderr,
                    "[c54x] DISPATCH-ENTRY #%u pc=0x%04x entry_idx=%u "
                    "header=0x%04x branch=0x%04x target=0x%04x "
                    "A=0x%010llx insn=%u\n",
                    entry_hits, s->pc, entry_idx,
                    header, branch, target,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->insn_count);
        }
    }

    /* Periodic DSP state dump (every 500M insns, starting at 500M).
     * Captures: state regs, hot-zone disasm (0xa2c0..0xa2d0 + 0xb8e0..0xb910),
     * vector table at current PMST IPTR base, hot-PC opcodes, SP history. */
    {
        static unsigned next_dump = 500000000u;
        if (s->insn_count >= next_dump) {
            next_dump += 500000000u;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            uint16_t vbase = iptr * 0x80;
            C54_LOG("STATE-DUMP insn=%u PC=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d IMR=0x%04x IFR=0x%04x XPC=%d PMST=0x%04x SP=0x%04x AR1=0x%04x AR2=0x%04x BRC=%d",
                    s->insn_count, s->pc, s->st0, s->st1,
                    !!(s->st1 & ST1_INTM),
                    s->imr, s->ifr, s->xpc, s->pmst, s->sp,
                    s->ar[1], s->ar[2], s->brc);
            C54_LOG("STATE-DUMP prog[0xa2c0..0xa2d0]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xa2c0], s->prog[0xa2c1], s->prog[0xa2c2], s->prog[0xa2c3],
                    s->prog[0xa2c4], s->prog[0xa2c5], s->prog[0xa2c6], s->prog[0xa2c7],
                    s->prog[0xa2c8], s->prog[0xa2c9], s->prog[0xa2ca], s->prog[0xa2cb],
                    s->prog[0xa2cc], s->prog[0xa2cd], s->prog[0xa2ce], s->prog[0xa2cf],
                    s->prog[0xa2d0]);
            /* Hot zone after ARP fix: b8e9..b906 (run 2, vec1 handler). */
            C54_LOG("STATE-DUMP prog[0xb8e0..0xb910]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xb8e0], s->prog[0xb8e1], s->prog[0xb8e2], s->prog[0xb8e3],
                    s->prog[0xb8e4], s->prog[0xb8e5], s->prog[0xb8e6], s->prog[0xb8e7],
                    s->prog[0xb8e8], s->prog[0xb8e9], s->prog[0xb8ea], s->prog[0xb8eb],
                    s->prog[0xb8ec], s->prog[0xb8ed], s->prog[0xb8ee], s->prog[0xb8ef],
                    s->prog[0xb8f0], s->prog[0xb8f1], s->prog[0xb8f2], s->prog[0xb8f3],
                    s->prog[0xb8f4], s->prog[0xb8f5], s->prog[0xb8f6], s->prog[0xb8f7],
                    s->prog[0xb8f8], s->prog[0xb8f9], s->prog[0xb8fa], s->prog[0xb8fb],
                    s->prog[0xb8fc], s->prog[0xb8fd], s->prog[0xb8fe], s->prog[0xb8ff],
                    s->prog[0xb900]);
            C54_LOG("STATE-DUMP prog[0xb900..0xb920]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xb900], s->prog[0xb901], s->prog[0xb902], s->prog[0xb903],
                    s->prog[0xb904], s->prog[0xb905], s->prog[0xb906], s->prog[0xb907],
                    s->prog[0xb908], s->prog[0xb909], s->prog[0xb90a], s->prog[0xb90b],
                    s->prog[0xb90c], s->prog[0xb90d], s->prog[0xb90e], s->prog[0xb90f],
                    s->prog[0xb910], s->prog[0xb911], s->prog[0xb912], s->prog[0xb913],
                    s->prog[0xb914], s->prog[0xb915], s->prog[0xb916], s->prog[0xb917],
                    s->prog[0xb918], s->prog[0xb919], s->prog[0xb91a], s->prog[0xb91b],
                    s->prog[0xb91c], s->prog[0xb91d], s->prog[0xb91e], s->prog[0xb91f],
                    s->prog[0xb920]);
            C54_LOG("STATE-DUMP vbase=0x%04x prog[vbase..vbase+0x18]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    vbase,
                    s->prog[vbase+0x00], s->prog[vbase+0x01], s->prog[vbase+0x02], s->prog[vbase+0x03],
                    s->prog[vbase+0x04], s->prog[vbase+0x05], s->prog[vbase+0x06], s->prog[vbase+0x07],
                    s->prog[vbase+0x08], s->prog[vbase+0x09], s->prog[vbase+0x0a], s->prog[vbase+0x0b],
                    s->prog[vbase+0x0c], s->prog[vbase+0x0d], s->prog[vbase+0x0e], s->prog[vbase+0x0f],
                    s->prog[vbase+0x10], s->prog[vbase+0x11], s->prog[vbase+0x12], s->prog[vbase+0x13],
                    s->prog[vbase+0x14], s->prog[vbase+0x15], s->prog[vbase+0x16], s->prog[vbase+0x17],
                    s->prog[vbase+0x18]);
            /* Hot-PC opcode dump for known correlator/handler sites */
            C54_LOG("STATE-DUMP HOT-OPS: 0x8d33=%04x 0x8eb9=%04x 0x8f51=%04x 0xa2c7=%04x 0xa2c8=%04x 0xb8e9=%04x 0xb8eb=%04x 0xb8f4=%04x 0xb8f5=%04x 0xb906=%04x",
                    s->prog[0x8d33], s->prog[0x8eb9], s->prog[0x8f51],
                    s->prog[0xa2c7], s->prog[0xa2c8],
                    s->prog[0xb8e9], s->prog[0xb8eb], s->prog[0xb8f4],
                    s->prog[0xb8f5], s->prog[0xb906]);
            /* DARAM 0x066F..0x0682 wait-loop disasm (run 3 stuck zone).
             * Looking for B-self (f073 066f) vs IDLE n (f7e1/f7e2/f7e3)
             * vs poll-and-branch. If IDLE found → emulator IDLE handler
             * is the real bug (3 runs all hit the same opcode, terminate
             * in different bassins because PMST/IPTR varies). */
            C54_LOG("STATE-DUMP prog[0x0660..0x0690]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0x0660], s->prog[0x0661], s->prog[0x0662], s->prog[0x0663],
                    s->prog[0x0664], s->prog[0x0665], s->prog[0x0666], s->prog[0x0667],
                    s->prog[0x0668], s->prog[0x0669], s->prog[0x066a], s->prog[0x066b],
                    s->prog[0x066c], s->prog[0x066d], s->prog[0x066e], s->prog[0x066f],
                    s->prog[0x0670], s->prog[0x0671], s->prog[0x0672], s->prog[0x0673],
                    s->prog[0x0674], s->prog[0x0675], s->prog[0x0676], s->prog[0x0677],
                    s->prog[0x0678], s->prog[0x0679], s->prog[0x067a], s->prog[0x067b],
                    s->prog[0x067c], s->prog[0x067d], s->prog[0x067e], s->prog[0x067f],
                    s->prog[0x0680]);
            /* Same range but data[] view in case OVLY=1 routes fetches
             * to data array (different memory than prog). */
            C54_LOG("STATE-DUMP data[0x0660..0x0680]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->data[0x0660], s->data[0x0661], s->data[0x0662], s->data[0x0663],
                    s->data[0x0664], s->data[0x0665], s->data[0x0666], s->data[0x0667],
                    s->data[0x0668], s->data[0x0669], s->data[0x066a], s->data[0x066b],
                    s->data[0x066c], s->data[0x066d], s->data[0x066e], s->data[0x066f],
                    s->data[0x0670], s->data[0x0671], s->data[0x0672], s->data[0x0673],
                    s->data[0x0674], s->data[0x0675], s->data[0x0676], s->data[0x0677],
                    s->data[0x0678], s->data[0x0679], s->data[0x067a], s->data[0x067b],
                    s->data[0x067c], s->data[0x067d], s->data[0x067e], s->data[0x067f],
                    s->data[0x0680]);
            /* IRQ entry handler at PC=0x1854 (last 0→1 transition) */
            C54_LOG("STATE-DUMP prog[0x1850..0x1860]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0x1850], s->prog[0x1851], s->prog[0x1852], s->prog[0x1853],
                    s->prog[0x1854], s->prog[0x1855], s->prog[0x1856], s->prog[0x1857],
                    s->prog[0x1858], s->prog[0x1859], s->prog[0x185a], s->prog[0x185b],
                    s->prog[0x185c], s->prog[0x185d], s->prog[0x185e], s->prog[0x185f],
                    s->prog[0x1860]);
            /* SP history ring (last 32 sampled at 1M-insn intervals) */
            {
                char buf[2048]; int o = 0;
                int start = (sp_ring_idx >= 32) ? (sp_ring_idx - 32) : 0;
                for (unsigned i = start; i < sp_ring_idx; i++) {
                    int idx = i & 63;
                    o += snprintf(buf+o, sizeof(buf)-o,
                                  "[%u:PC=%04x SP=%04x] ",
                                  sp_ring[idx].insn,
                                  sp_ring[idx].pc,
                                  sp_ring[idx].sp);
                    if (o > (int)sizeof(buf) - 64) break;
                }
                C54_LOG("STATE-DUMP SP-RING (last %d): %s",
                        (int)(sp_ring_idx >= 32 ? 32 : sp_ring_idx), buf);
            }
        }
    }

    while (executed < n_insns && s->running && !s->idle) {
        if (g_pc_histogram) g_pc_histogram[s->pc & 0xFFFF]++;   /* reliable per-insn PC sample */
        if (g_pcring_on) {                                      /* path-into-halt PC ring */
            uint32_t i = g_pcring_idx & (PC_RING_N - 1);
            g_pcring_pc[i] = (uint16_t)s->pc; g_pcring_xpc[i] = (uint8_t)(s->xpc & 3); g_pcring_idx++;
        }
        /* XPC-CHG=1: log every XPC change with the PC/op that caused it (capped). On a C542-class
         * chip (5110 MAD1 LEAD: single 64K prog page, pages 1-3 EMPTY) XPC must never leave 0 —
         * any transition is a core artifact (far-op or IRQ push/pop asymmetry) sending the DSP
         * into a zero-filled page. */
        { static int xchg = -1;
          if (xchg < 0) xchg = cdbg_env("XPC-CHG") ? 1 : 0;
          if (xchg) {                       /* probe off => zero per-insn work */
              static uint16_t xprev = 0; static int xn = 0;
              if ((uint16_t)(s->xpc & 3) != xprev && xn < 16) { xn++;
                  fprintf(stderr, "[c54x] XPC %u -> %u at PC=0x%04x prevPC=0x%04x op@prev=0x%04x SP=0x%04x insn=%u\n",
                          xprev, s->xpc & 3, s->pc, s->last_exec_pc, prog_fetch(s, s->last_exec_pc), s->sp, s->insn_count); }
              xprev = (uint16_t)(s->xpc & 3); } }
        if (g_halt_pc != 0xFFFFFFFFu && (uint32_t)(s->pc & 0xFFFF) == g_halt_pc) {
            if (++g_halt_hits >= g_halt_after) { g_halted = 1; break; }  /* freeze BEFORE this insn */
        }
        /* === SILICON-BOOT-ROM REDIRECT (réactivé 2026-05-30) ===========
         * Le dump PROM ne contient PAS le mask-ROM silicon du Calypso. Sur
         * vrai HW, ce ROM masqué tourne au reset, pose SP=0x5AC8 + MMR, puis
         * saute à l'entrée firmware PROM0[0x7120] (= STM #0x5AC8,SP vérifié :
         * prog[0x7120]=0x7718 STM #lk,SP, prog[0x7121]=0x5ac8). On MODÉLISE
         * ce hardware manquant — ce n'est PAS un override d'instruction
         * firmware : on route vers l'entrée firmware propre, qui fait elle-
         * même son init SP.
         *
         * RÉGRESSION corrigée : retiré le 29/05 (c3ec660 « relancer via
         * 0xFF80 réel »). Sans lui, reset → 0xff80(FB) → 0xb410 → CC → 0x76f8
         * SANS jamais exécuter STM #0x5AC8,SP → SP coincé à 0x1100 (invalide,
         * = aire MMR/AR0) → over-pop boot (net→-57) → return corrompu →
         * self-CALA 0x70c3 → spirale (16M pushes) → PMST 0x70C4 fuit en
         * TOA=28868 côté osmocon → FB jamais locké.
         *
         * Gate SP==0x1100 = cold-reset uniquement (valeur silicon-reset). Une
         * fois SP=0x5AC8 posé par 0x7120, la condition retombe → les walks
         * séquentiels firmware passant par 0xff80 plus tard NE sont PAS
         * hijackés (cf SOFT-RESET-TRIG ci-dessous, insn>100k). */
        /* EXPÉRIENCE 2026-05-30 (CC-web) testée et CONCLUE : poser SP=0x5AC8 sans
         * rediriger le PC (laisser 0xff80→0xb410 tourner) → le reset-handler
         * 0xb410 s'exécute MAIS n'appelle PAS le boot-init 0x7000-0x7025 (reste
         * 1 hit incident) ; FB-dispatch échoue identiquement. Donc FB-dispatch
         * n'est PAS la queue du boot-init = issue steady-state SÉPARÉE. Acquis :
         * l'over-pop était 100% un artefact de SP=0x1100 (F@0x76f8 tourne propre
         * depuis 0x5AC8, no wedge). On revient au redirect 0x7120 committé. */
        /* === REDIRECT NEUTRALISÉ PAR DÉFAUT (2026-05-31) ===================
         * Ce bloc MODÉLISE un mask-ROM TI absent du dump = un HACK (simulation).
         * Méthode user : NE RIEN simuler. On laisse le vrai reset vector jouer
         * (0xff80 = FB 0xb410 = vrai reset handler firmware) et on débugge
         * CHAQUE bug réel de la chaîne de boot avec les valeurs qu'elle produit.
         * Le redirect est conservé derrière CALYPSO_REDIR_LEGACY=1 UNIQUEMENT
         * pour comparaison A/B ; OFF par défaut. Premier bug réel attendu sans
         * lui (cf ancien commentaire) : à 0xb410 le CC saute le STM #0x5AC8,SP
         * → SP reste 0x1100 → over-pop. C'est CE bug qu'on trace, pas qu'on
         * contourne. */
        static int redir_legacy = -1;
        if (redir_legacy < 0) redir_legacy = getenv("CALYPSO_REDIR_LEGACY") ? 1 : 0;
        if (redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) {
            static int redirect_log;
            /* EXPÉRIENCE CALYPSO_REDIR7000 (2026-05-30) : le redirect→0x7120 saute
             * l'init qui peuple les tables BACC-A (data[0x4c5b]/0x3fe1) → A=0 →
             * boot stub → dispatch dormant. Test : poser SP=0x5AC8 (mask-ROM) +
             * rediriger vers 0x7000 (init COMPLÈTE : tables + A) pour que BACC A
             * atteigne la vraie entrée firmware. cf SESSION_2026-05-29 fix#2. */
            static int redir7000 = -1;
            if (redir7000 < 0) redir7000 = getenv("CALYPSO_REDIR7000") ? 1 : 0;
            if (redirect_log < 3) {
                C54_LOG("SILICON-BOOT-REDIRECT PC=0xFF80 SP=0x1100 → 0x%04x%s",
                        redir7000 ? 0x7000 : 0x7120,
                        redir7000 ? " (REDIR7000: SP=0x5AC8 + init complète A-tables)" : "");
                redirect_log++;
            }
            /* VALIDATION CALYPSO_INITTAB (env, réversible) : prouve que peupler la
             * table de dispatch débloque FB. Pose SP, PUSH retour=0x7120, saute à
             * 0xc704 (table-init) → peuple data[0x4c24-0x4c5d] → RET vers 0x7120 →
             * boot normal continue AVEC table peuplée → BACC A atteint les vrais
             * handlers. Débloque FB → root+fix prouvés ; sinon → table pas le seul. */
            static int inittab = -1;
            if (inittab < 0) inittab = getenv("CALYPSO_INITTAB") ? 1 : 0;
            if (inittab) {
                s->sp = 0x5AC8;
                s->sp--; s->data[s->sp] = 0x7120;   /* retour = boot normal */
                s->pc = 0xc704;                       /* run table-init → RET 0x7120 */
            } else if (redir7000) { s->sp = 0x5AC8; s->pc = 0x7000; }
            else s->pc = 0x7120;
        }
        /* === SOFT-RESET-TRIGGER probe (2026-05-28) ===
         * SP-CATASTROPHE trace montre PC=0x7120 (boot init via notre override
         * 0xFF80) re-firing à insn=190M. C'est un soft-reset interne firmware.
         * Pour pinpointer le déclencheur : log toute arrivée à PC=0xFF80 ou
         * PC=0x7120 APRÈS insn > 100k (= silicon reset initial déjà passé).
         * Trail pc_ring[-16..-1] + SP/AR/IMR/IFR/INTM → on voit l'instr qui
         * a sauté ici. */
        if ((s->pc == 0xFF80 || s->pc == 0x7120) && s->insn_count > 100000) {
            /* Deeper trail probe — gated par CALYPSO_DEBUG=SOFT_RESET_TRAIL.
             * pc[-64..-1] permet de remonter ~64 instructions avant la
             * réception du soft-reset pour identifier le caller chain. */
            if (calypso_debug_enabled("SOFT_RESET_TRAIL")) {
                static unsigned deep_log;
                if (deep_log < 5) {
                    fprintf(stderr,
                        "[c54x] SOFT-RESET DEEP-TRAIL #%u (last 64 PCs):\n",
                        deep_log);
                    for (int row = 0; row < 8; row++) {
                        fprintf(stderr, "[c54x] SR-DEEP[%2d-%2d] :",
                                -64 + row*8, -57 + row*8);
                        for (int col = 0; col < 8; col++) {
                            int idx = -64 + row*8 + col;
                            fprintf(stderr, " %04x",
                                pc_ring[(pc_ring_idx + idx) & 255]);
                        }
                        fprintf(stderr, "\n");
                    }
                    deep_log++;
                }
            }
            static unsigned srt_log;
            if (srt_log < 30) {
                C54_LOG("SOFT-RESET-TRIG #%u PC=0x%04x insn=%u SP=0x%04x "
                        "IMR=0x%04x IFR=0x%04x INTM=%d B=0x%010llx "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        srt_log, s->pc, s->insn_count, s->sp,
                        s->imr, s->ifr, !!(s->st1 & ST1_INTM),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                C54_LOG("SOFT-RESET-TRIG #%u trail pc[-16..-1] = "
                        "%04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x",
                        srt_log,
                        pc_ring[(pc_ring_idx-17)&255], pc_ring[(pc_ring_idx-16)&255],
                        pc_ring[(pc_ring_idx-15)&255], pc_ring[(pc_ring_idx-14)&255],
                        pc_ring[(pc_ring_idx-13)&255], pc_ring[(pc_ring_idx-12)&255],
                        pc_ring[(pc_ring_idx-11)&255], pc_ring[(pc_ring_idx-10)&255],
                        pc_ring[(pc_ring_idx-9)&255],  pc_ring[(pc_ring_idx-8)&255],
                        pc_ring[(pc_ring_idx-7)&255],  pc_ring[(pc_ring_idx-6)&255],
                        pc_ring[(pc_ring_idx-5)&255],  pc_ring[(pc_ring_idx-4)&255],
                        pc_ring[(pc_ring_idx-3)&255],  pc_ring[(pc_ring_idx-2)&255]);
                srt_log++;
            }
        }
        /* === PROM3-VISIT probe (2026-05-28) ===
         * Compte les visites du DSP aux entries SB-decode candidats :
         *   0x8167, 0x81ff, 0x82b8 (PROM3 dispatch SB candidates per session).
         * Log à la première visite uniquement (insn_count + caller via ring),
         * puis compteur silencieux. Si à la fin du run task=6 a fire 30×
         * mais aucune visite → bug dispatch (item 5). Si visites OK mais
         * sb_att=0 → bug demod plus profond. */
        {
            static uint64_t v8167, v81ff, v82b8;
            static uint32_t v8167_first_insn, v81ff_first_insn, v82b8_first_insn;
            uint16_t pc = s->pc;
            if (pc == 0x8167) {
                if (v8167 == 0) {
                    v8167_first_insn = s->insn_count;
                    C54_LOG("PROM3-VISIT 0x8167 FIRST-HIT insn=%u SP=%04x "
                            "AR2=%04x AR3=%04x AR4=%04x AR5=%04x",
                            v8167_first_insn, s->sp,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5]);
                }
                v8167++;
                if ((v8167 % 100000) == 0)
                    C54_LOG("PROM3-VISIT 0x8167 count=%llu insn=%u",
                            (unsigned long long)v8167, s->insn_count);
            }
            if (pc == 0x81ff) {
                if (v81ff == 0) {
                    v81ff_first_insn = s->insn_count;
                    C54_LOG("PROM3-VISIT 0x81ff FIRST-HIT insn=%u SP=%04x "
                            "AR2=%04x AR3=%04x AR4=%04x AR5=%04x",
                            v81ff_first_insn, s->sp,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5]);
                }
                v81ff++;
                if ((v81ff % 100000) == 0)
                    C54_LOG("PROM3-VISIT 0x81ff count=%llu insn=%u",
                            (unsigned long long)v81ff, s->insn_count);
            }
            if (pc == 0x82b8) {
                if (v82b8 == 0) {
                    v82b8_first_insn = s->insn_count;
                    C54_LOG("PROM3-VISIT 0x82b8 FIRST-HIT insn=%u SP=%04x "
                            "AR2=%04x AR3=%04x AR4=%04x AR5=%04x",
                            v82b8_first_insn, s->sp,
                            s->ar[2], s->ar[3], s->ar[4], s->ar[5]);
                }
                v82b8++;
                if ((v82b8 % 100000) == 0)
                    C54_LOG("PROM3-VISIT 0x82b8 count=%llu insn=%u",
                            (unsigned long long)v82b8, s->insn_count);
            }
        }
        /* === TOP-OF-LOOP SP CHOKEPOINT (fix 2026-05-24 v6 Claude web) ===
         * Le hook SP existant est en BAS de boucle (L7471). Toute
         * instruction qui sort tôt (goto unimpl, return, continue, handler
         * qui sort de la dispatch chain) bypasse le hook → l'écriture SP
         * a lieu mais n'est pas comptabilisée. Hier l'audit "tout passe
         * par s->sp" était correct sur les SITES d'écriture mais ne
         * vérifiait pas si le hook tourne pour ces instructions.
         *
         * Symptôme : 61 events captés vs descente attendue de 11k+ mots
         * → la descente passe par bypass(es). Fix : observer s->sp à un
         * CHOKEPOINT obligé (top de boucle), comparer avec la valeur de
         * l'itération précédente. Bypass-proof par construction : on
         * regarde la VALEUR à un point de passage, pas le SITE.
         *
         * Implementation : statics (persistent inter-c54x_run-calls). */
        {
            static uint16_t topgate_last_sp = 0;
            static uint16_t topgate_last_pc = 0;
            static uint16_t topgate_last_op = 0;
            static int      topgate_valid   = 0;

            if (topgate_valid && s->sp != topgate_last_sp) {
                /* Compte l'instruction PRÉCÉDENTE qui a changé SP, quelle
                 * que soit sa voie de sortie (early-exit, return, etc.) */
                sp_hist_account(topgate_last_pc, topgate_last_op,
                                topgate_last_sp, s->sp, s->insn_count);
            }

            /* Patch 3 rev 2 : bootstub-entry trigger (le bon signal post
             * rev 1). Détecte l'edge prev_pc ∉ bootstub → cur_pc ∈ bootstub
             * = le RET corrompu qui a sauté à 0x00XX. Capture verbose +
             * dump ring contenant ~4096 iters d'approche. */
            if (topgate_valid) {
                sp_ring_check_bootstub_entry(s,
                    topgate_last_pc, topgate_last_op, topgate_last_sp,
                    s->pc, s->sp, s->insn_count);
            }

            /* A provenance tracer (2026-05-25 v3, Claude web review).
             * Track A's last writer + dump at trigger PC. Resout fork
             * NMI-vs-A-divergence avant impl invasive. */
            a_track_init_lazy();
            if (topgate_valid) {
                a_track_iter(s, topgate_last_pc, topgate_last_op);
            }

            /* AR6 windowed snapshot (2026-05-25 v4) — disambigue AR6=0
             * (base divergence) vs AR6=0x16 (self-alias feedback) au PC
             * trigger. Env CALYPSO_AR6_AT_PC=0x821a + window. */
            ar6_at_init_lazy();
            if (topgate_valid) {
                ar6_at_iter(s, topgate_last_pc, topgate_last_op);
            }

            topgate_last_sp = s->sp;
            topgate_last_pc = s->pc;
            topgate_last_op = prog_fetch(s, s->pc);
            topgate_valid   = 1;

            sp_ring_init_lazy();
            sp_ring_record(s->insn_count, s->pc, s->sp, topgate_last_op);

            /* MVPD overlay occupancy : lazy-init + dump-if-boot-phase-ended. */
            mvpd_trace_init_lazy();
            mvpd_trace_dump_if_due(s->insn_count);

            /* Correlator entry trace : detect edge prev_pc ∉ [0x8d00..0x8f80]
             * → cur_pc ∈ same range. Log full state (AR3/4/5 = buffer pointers
             * probables) au moment de l'entrée. Dump des reads accumulés
             * périodiquement (toutes 20 entrées) pour observer si pattern
             * se stabilise vs varie entre runs. */
            corr_trace_init_lazy();
            if (g_corr_trace_enabled > 0 && topgate_valid) {
                int prev_in = (topgate_last_pc >= 0x8d00 && topgate_last_pc < 0x8f80);
                int cur_in  = (s->pc >= 0x8d00 && s->pc < 0x8f80);
                if (!prev_in && cur_in) {
                    g_corr_entry_count++;
                    if (g_corr_entry_count <= g_corr_entry_log_cap) {
                        fprintf(stderr,
                            "[c54x] CORR-ENTRY #%u @insn=%u prev_pc=0x%04x → cur_pc=0x%04x\n"
                            "[c54x]   AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "ARP=%d DP=%d BK=0x%04x\n"
                            "[c54x]   SP=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d XPC=%d\n",
                            g_corr_entry_count, s->insn_count,
                            topgate_last_pc, s->pc,
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            arp(s), dp(s), s->bk,
                            s->sp, s->st0, s->st1, !!(s->st1 & ST1_INTM), s->xpc);
                    }
                    /* Dump tous les 20 entrées pour observer si addr lues
                     * stabilisent (correlator répète) ou varient. */
                    if ((g_corr_entry_count % 20) == 0) {
                        char tag[32];
                        snprintf(tag, sizeof(tag), "every20-entry%u", g_corr_entry_count);
                        corr_read_dump(tag);
                    }
                }
            }
        }

        /* DSP idle fast-forward — see dsp_idle_fast_forward() comment.
         * Skips MAC simulation when DSP is in its empty-task-slot
         * polling loop, returning host CPU to the rest of QEMU. */
        {
            int ff_cyc;
            if (dsp_idle_fast_forward(s, &ff_cyc)) {
                s->cycles    += ff_cyc;
                s->insn_count += ff_cyc;
                executed     += ff_cyc;
                if (g_pc_histogram) g_pc_histogram[s->pc & 0xFFFF] += (uint32_t)ff_cyc; /* credit idle time */
                continue;
            }
        }

        /* Replay any interrupt that fired while INTM=1.
         * c54x_interrupt_ex sets IFR but does nothing else when INTM=1;
         * the real C54x re-evaluates pending interrupts every cycle, so
         * as soon as INTM clears (via RETE or RSBX INTM) a pending
         * BRINT0/TINT0/... must dispatch. Without this, a BRINT0 that
         * arrived inside another ISR is lost and the FB correlator never
         * receives its I/Q samples (d_fb_det stays 0). */
        if (!(s->st1 & ST1_INTM)) {
            uint16_t pending = s->ifr & s->imr;
            if (pending) {
                int imr_bit = __builtin_ctz(pending);
                int vec = imr_bit + 16;
                s->ifr &= ~(1 << imr_bit);
                s->sp--;
                data_write(s, s->sp, s->pc);
                /* IT C54x = transition far : save XPC inconditionnel (APTS
                 * == AVIS, zéro sémantique pile) + force page 0 pour le fetch
                 * du vecteur (sinon vecteur lu via XPC vivant = bug racine).
                 * C542-class (no_xpc): 1-word frame, PC only. */
                if (!s->no_xpc) {
                    s->sp--;
                    data_write(s, s->sp, s->xpc);
                }
                s->st1 |= ST1_INTM;
                s->xpc = 0;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                static int pending_log = 0;
                if (pending_log < 20) {
                    C54_LOG("PENDING IRQ replay vec=%d bit=%d PC->0x%04x SP=0x%04x insn=%u",
                            vec, imr_bit, s->pc, s->sp, s->insn_count);
                    pending_log++;
                }
            }
        }

        /* Record PC in ring buffer */
        pc_ring[pc_ring_idx & 255] = s->pc;
        pc_ring_idx++;

        /* Push counter at PC=0xb906 (and other suspected push sites).
         * Logs at powers of 10 to track cadence. SP captured at hit. */
        {
            static unsigned hit_b906 = 0;
            if (s->pc == 0xb906) {
                hit_b906++;
                if (hit_b906 == 1 || hit_b906 == 10 || hit_b906 == 100 ||
                    hit_b906 == 1000 || hit_b906 == 10000 ||
                    hit_b906 == 100000 || hit_b906 == 1000000) {
                    C54_LOG("HIT-b906 #%u op=0x%04x SP=0x%04x XPC=%d insn=%u",
                            hit_b906, s->prog[0xb906], s->sp, s->xpc,
                            s->insn_count);
                }
            }
        }

        /* INTM transition tracer: every change of ST1 bit 11 with
         * surrounding state. Identifies which IRQ entered the trap and
         * whether RETE / RSBX paths ever execute again. On each 0->1
         * (IRQ entry), also dump prog[PC..PC+8] and the 4 most-recently
         * pushed stack words (data[SP..SP+3]) so we can see what handler
         * we're entering and why it never RETEs.
         *
         * NOTE: this block runs BEFORE c54x_exec_one of the current
         * iteration. So when a transition is observed, the cause was
         * either (a) the previous iteration's exec_one (RETE, RSBX INTM
         * etc. — INTM 1→0), or (b) the pending-IRQ replay block above
         * (INTM 0→1, PC moved to vector). For (a), s->pc has already
         * advanced past the cause — log the previous iteration's
         * exec_pc/exec_op (captured at end of loop into last_exec_*) so
         * the cause is unambiguous. For (b), s->pc IS the vector entry
         * and is informative as-is. */
        {
            static int intm_log = 0;
            static uint16_t prev_intm = 0xFFFF;
            uint16_t cur_intm = !!(s->st1 & ST1_INTM);
            if (prev_intm != 0xFFFF && cur_intm != prev_intm && intm_log < 200) {
                C54_LOG("INTM-TRANS %u->%u current PC=0x%04x op=0x%04x | "
                        "cause prev_exec PC=0x%04x op=0x%04x | "
                        "XPC=%d IFR=0x%04x SP=0x%04x insn=%u",
                        (unsigned)prev_intm, (unsigned)cur_intm,
                        s->pc, s->prog[s->pc],
                        s->last_exec_pc, s->last_exec_op,
                        s->xpc, s->ifr, s->sp,
                        s->insn_count);
                if (cur_intm == 1) {
                    C54_LOG("  HANDLER prog[PC..PC+8]: %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc + 1)],
                            s->prog[(uint16_t)(s->pc + 2)],
                            s->prog[(uint16_t)(s->pc + 3)],
                            s->prog[(uint16_t)(s->pc + 4)],
                            s->prog[(uint16_t)(s->pc + 5)],
                            s->prog[(uint16_t)(s->pc + 6)],
                            s->prog[(uint16_t)(s->pc + 7)],
                            s->prog[(uint16_t)(s->pc + 8)]);
                    C54_LOG("  STACK data[SP..SP+3]: %04x %04x %04x %04x",
                            s->data[s->sp],
                            s->data[(uint16_t)(s->sp + 1)],
                            s->data[(uint16_t)(s->sp + 2)],
                            s->data[(uint16_t)(s->sp + 3)]);
                }
                intm_log++;
            }
            /* INT3-CYCLE-TRACE : fire end-good on ANY INTM 1→0 transition,
             * not just RETE — firmware uses POPM ST1 + RCD pattern. The
             * function itself is a no-op when probe disabled or no cycle
             * active, so unconditional call is safe. */
            if (prev_intm == 1 && cur_intm == 0) {
                int3_cycle_end_good(s, s->pc);
            }
            prev_intm = cur_intm;
        }

        /* SP-WATCH: log every transition where SP enters / leaves the
         * API mailbox region [0x0800..0x08FF]. This pinpoints the exact
         * instruction that corrupts the stack pointer so we don't have
         * to keep recoding to investigate. */
        {
            static uint16_t prev_sp = 0xFFFF;
            bool was_in = (prev_sp >= 0x0800 && prev_sp < 0x0900);
            bool is_in  = (s->sp  >= 0x0800 && s->sp  < 0x0900);
            if (was_in != is_in) {
                if (calypso_debug_enabled("SP-WATCH")) fprintf(stderr,
                        "[c54x] SP-WATCH %s SP=0x%04x (prev=0x%04x) "
                        "PC=0x%04x op=0x%04x insn=%u\n",
                        is_in ? "ENTER api" : "LEAVE api",
                        s->sp, prev_sp, s->pc, s->prog[s->pc], s->insn_count);
            }
            prev_sp = s->sp;
        }

        /* SP-DRAIN probe (CALYPSO_DEBUG=SP-DRAIN) : attribue chaque
         * décrément net de SP à l'instruction qui vient de s'exécuter
         * (last_exec_pc/op — capturés en fin de boucle précédente).
         * Ces blocs tournent AVANT exec_one de l'itération courante, donc
         * s->sp reflète le résultat de l'insn précédente = last_exec_pc.
         * Isole l'instruction non-appariée qui draine SP dans le trampoline
         * boot 0x0000↔0xffcd. Histogramme 8-slots + log des 120 premiers
         * events. Silent par défaut. */
        if (calypso_debug_enabled("SP-DRAIN")) {
            static uint16_t sd_prev_sp = 0xFFFF;
            static unsigned  sd_log = 0;
            static uint32_t  sd_cnt[8];
            static uint16_t  sd_pc[8];
            static uint32_t  sd_events;
            if (sd_prev_sp != 0xFFFF) {
                int delta = (int)(uint16_t)(sd_prev_sp - s->sp); /* >0 = push */
                if (delta > 0 && delta < 0x100) {
                    uint16_t cpc = s->last_exec_pc;
                    int slot = -1, freeslot = -1;
                    for (int i = 0; i < 8; i++) {
                        if (sd_cnt[i] && sd_pc[i] == cpc) { slot = i; break; }
                        if (!sd_cnt[i] && freeslot < 0) freeslot = i;
                    }
                    if (slot < 0 && freeslot >= 0) { slot = freeslot; sd_pc[slot] = cpc; }
                    if (slot >= 0) sd_cnt[slot] += (uint32_t)delta;
                    if (sd_log < 120) {
                        sd_log++;
                        C54_DBG("SP-DRAIN",
                                "push -%d SP=0x%04x<-0x%04x by PC=0x%04x op=0x%04x insn=%u",
                                delta, s->sp, sd_prev_sp, cpc,
                                s->last_exec_op, s->insn_count);
                    }
                    if ((++sd_events % 1000) == 0) {
                        C54_DBG("SP-DRAIN",
                                "TOP pushers: %04x:%u %04x:%u %04x:%u %04x:%u "
                                "%04x:%u %04x:%u %04x:%u %04x:%u (events=%u SP=0x%04x)",
                                sd_pc[0], sd_cnt[0], sd_pc[1], sd_cnt[1],
                                sd_pc[2], sd_cnt[2], sd_pc[3], sd_cnt[3],
                                sd_pc[4], sd_cnt[4], sd_pc[5], sd_cnt[5],
                                sd_pc[6], sd_cnt[6], sd_pc[7], sd_cnt[7],
                                sd_events, s->sp);
                    }
                }
            }
            sd_prev_sp = s->sp;
        }

        /* CALLSITE probe (CALYPSO_DEBUG=CALLSITE) : à l'épilogue RCD 0x7707,
         * dump l'adresse de retour que RCD va popper + l'opcode du call-site
         * (FCALL F9xx vs CALL F074) + pc-ring pré-RETD = park-vs-crash. */
        if (s->pc == 0x7707 && calypso_debug_enabled("CALLSITE")) {
            static int n7707 = 0;
            if (n7707 < 8) {
                n7707++;
                uint16_t ret = data_read(s, s->sp); (void)ret;   /* used only by C54_DBG (no-op in this build) */
                C54_DBG("CALLSITE",
                    "RCD@7707 #%d SP=0x%04x ret=0x%04x caller[ret-2..ret-1]=0x%04x 0x%04x XPC=%d insn=%u",
                    n7707, s->sp, ret, prog_read(s, (uint16_t)(ret-2)),
                    prog_read(s, (uint16_t)(ret-1)), s->xpc, s->insn_count);
                char buf[300]; int o=0;
                for (int i=20;i>=1;i--)
                    o+=snprintf(buf+o,sizeof(buf)-o,"%04x ", pc_ring[(pc_ring_idx-i)&255]);
                C54_DBG("CALLSITE", "  pre-RETD pcring(20): %s", buf);
            }
        }

        /* XPC-WR tracer (CALYPSO_DEBUG=XPC-WR) : toute transition de XPC avec
         * l'instruction qui l'a causée (= origine du XPC=3 garbage). */
        if (calypso_debug_enabled("XPC-WR")) {
            static uint8_t xprev = 0xFF;
            if (xprev != 0xFF && (uint8_t)s->xpc != xprev) {
                C54_DBG("XPC-WR",
                    "XPC %u->%u cause prev_exec PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                    xprev, (unsigned)(s->xpc & 0xFF), s->last_exec_pc,
                    s->last_exec_op, s->sp, s->insn_count);
            }
            xprev = (uint8_t)s->xpc;
        }

        /* AR2-WR tracer (CALYPSO_DEBUG=AR2-WR) : discrimine reset vs runaway.
         * delta==-1 = post-décrément normal (progression, log tous les 200).
         * delta!=-1 = reset/jump/load = LE discriminateur (#1 reset existe
         * vs #2 jamais de reset). Reporte BK + la cible du reset. */
        if (calypso_debug_enabled("AR2-WR")) {
            static int      ar2_first = 1;
            static uint16_t ar2_prev = 0;
            static uint32_t ar2_dec  = 0;
            uint16_t cur = s->ar[2];
            if (!ar2_first && cur != ar2_prev) {
                int delta = (int)(int16_t)(cur - ar2_prev);
                if (delta == -1) {
                    if ((++ar2_dec % 200) == 0)
                        C54_DBG("AR2-WR", "AR2 dec #%u ->0x%04x (linear -1) PC=0x%04x insn=%u",
                                ar2_dec, cur, s->last_exec_pc, s->insn_count);
                } else {
                    C54_DBG("AR2-WR",
                        "AR2 %s 0x%04x->0x%04x (delta=%+d) cause PC=0x%04x op=0x%04x BK=0x%04x insn=%u",
                        delta > 0 ? "RESET/UP" : "JUMP-DN", ar2_prev, cur, delta,
                        s->last_exec_pc, s->last_exec_op, s->bk, s->insn_count);
                }
            }
            ar2_first = 0; ar2_prev = cur;
        }

        /* TRACE: dump entry into 0xe260 loop (first 5 hits) */
        if (s->pc == 0xe260 || s->pc == 0xe261) {
            static int e260_log = 0;
            if (e260_log < 5) {
                e260_log++;
                C54_LOG("E260-ENTRY #%d PC=0x%04x AR2=%04x AR5=%04x BRC=%d RSA=%04x REA=%04x rptb=%d IMR=%04x SP=%04x insn=%u",
                        e260_log, s->pc, s->ar[2], s->ar[5], s->brc, s->rsa, s->rea, s->rptb_active, s->imr, s->sp, s->insn_count);
                int idx = pc_ring_idx;
                char buf[1024]; int o = 0;
                for (int i = 50; i >= 1; i--) {
                    o += snprintf(buf+o, sizeof(buf)-o, "%04x ", pc_ring[(idx-i)&255]);
                }
                C54_LOG("E260-PCRING (last 50): %s", buf);
                /* Dump runtime opcodes 0xe255..0xe28f */
                char ob[1024]; int oo = 0;
                for (uint16_t a = 0xe255; a <= 0xe28f; a++) {
                    oo += snprintf(ob+oo, sizeof(ob)-oo, "%04x ", s->prog[a]);
                }
                C54_LOG("E260-PROG[e255..e28f]: %s", ob);
            }
        }

        /* CALA loop tracer: dump A and SP at PC=0xd24e and 0xd250 (first 40) */
        if (s->pc == 0xd24e || s->pc == 0xd250) {
            static int cala_log = 0;
            if (cala_log++ < 40) {
                C54_LOG("CALA-TRACE PC=0x%04x A=%08x SP=0x%04x BRC=%d AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                        s->pc, (uint32_t)(s->a & 0xFFFFFFFF), s->sp, s->brc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            }
        }

        /* PC histogram: count visits per PC, dump top 20 every 2M insns */
        {
            static uint32_t pc_hist[0x10000];
            static uint64_t hist_last_dump = 0;
            pc_hist[s->pc]++;
            if (s->insn_count - hist_last_dump >= 2000000) {
                hist_last_dump = s->insn_count;
                /* find top 20 */
                uint32_t top_cnt[20] = {0};
                uint16_t top_pc[20] = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_hist[i];
                    if (c == 0) continue;
                    for (int j = 0; j < 20; j++) {
                        if (c > top_cnt[j]) {
                            for (int k = 19; k > j; k--) {
                                top_cnt[k] = top_cnt[k-1];
                                top_pc[k] = top_pc[k-1];
                            }
                            top_cnt[j] = c;
                            top_pc[j] = (uint16_t)i;
                            break;
                        }
                    }
                }
                C54_LOG("PC HIST insn=%u top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        s->insn_count,
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1], top_pc[2], top_cnt[2],
                        top_pc[3], top_cnt[3], top_pc[4], top_cnt[4], top_pc[5], top_cnt[5],
                        top_pc[6], top_cnt[6], top_pc[7], top_cnt[7], top_pc[8], top_cnt[8],
                        top_pc[9], top_cnt[9]);
                C54_LOG("PC HIST cont:        %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[10], top_cnt[10], top_pc[11], top_cnt[11], top_pc[12], top_cnt[12],
                        top_pc[13], top_cnt[13], top_pc[14], top_cnt[14], top_pc[15], top_cnt[15],
                        top_pc[16], top_cnt[16], top_pc[17], top_cnt[17], top_pc[18], top_cnt[18],
                        top_pc[19], top_cnt[19]);
                memset(pc_hist, 0, sizeof(pc_hist));
            }
        }

        /* === Rolling PC sampler (v6 — find the REAL stuck zone) ===
         * The cumulative-since-boot PC HIST shows 0xa218..0xa222 dominant
         * because the init loop at 0xa222 (BANZD AR5, 60k iters) ran once
         * early. After that, the DSP moved on but the cumulative histogram
         * still shows those PCs at the top.
         *
         * BANZD-A222 traces (2026-05-08) confirmed AR5 was the actual loop
         * counter (61523→61499 in 25 iter), not AR1. Loop finishes in
         * ~984k insns (= 0.06% of a 1.7B run). Whatever IS currently
         * burning DSP cycles is in a different zone, invisible to the
         * cumulative top-N.
         *
         * Solution : rolling histogram per 100k-insn window. Resets each
         * window so we always see "what is the DSP doing RIGHT NOW".
         * Logs top-5 PCs of the most recent window. */
        {
            static uint32_t pc_recent[0x10000];
            static uint32_t recent_last_dump = 0;
            pc_recent[s->pc]++;
            if (s->insn_count - recent_last_dump >= 100000) {
                recent_last_dump = s->insn_count;
                uint32_t top_cnt[5] = {0};
                uint16_t top_pc[5]  = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_recent[i];
                    if (c <= top_cnt[4]) continue;
                    top_cnt[4] = c; top_pc[4] = (uint16_t)i;
                    for (int j = 4; j > 0 && top_cnt[j] > top_cnt[j-1]; j--) {
                        uint32_t tc = top_cnt[j]; top_cnt[j] = top_cnt[j-1]; top_cnt[j-1] = tc;
                        uint16_t tp = top_pc[j]; top_pc[j] = top_pc[j-1]; top_pc[j-1] = tp;
                    }
                }
                C54_LOG("PC RECENT (last 100k) top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1],
                        top_pc[2], top_cnt[2], top_pc[3], top_cnt[3],
                        top_pc[4], top_cnt[4]);
                memset(pc_recent, 0, sizeof(pc_recent));
            }
        }

        /* === ENTER-RPTB-A218 probe (Q-BRC investigation 2026-05-08 v5+v6) ===
         * v5 hypothesis (BRC≈30770) was REFUTED by first 20 events :
         *   BRC=0 systematic, AR1=0 systematic, AR2 increments by 2,
         *   16 insns between visits.
         * v6 expands to capture the late-run behaviour : the cap=20 saturated
         * at insn=48M while the run reached 2.4B. We now have :
         *   (a) cap=200 for early events
         *   (b) periodic sampler at 100k-visits intervals (late-run)
         *   (c) BANZD-A222 probe to capture the actual AR used by the
         *       branch-back instruction at 0xa222 op=0x6e81.
         * The !s->rpt_active guard avoids spurious mid-RPTB hits. */
        if (s->pc == 0xa218 && !s->rpt_active) {
            static unsigned a218_total = 0;
            static int a218_log = 0;
            a218_total++;
            bool log_now = (a218_log < 200) ||
                           (a218_total % 100000 == 0);
            if (log_now) {
                C54_LOG("ENTER-RPTB-A218 #%d total=%u BRC=%u (0x%04x) "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                        "AR4=0x%04x AR5=0x%04x A=%010llx T=0x%04x "
                        "ST0=0x%04x insn=%u",
                        a218_log + 1, a218_total, s->brc, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->t, s->st0, s->insn_count);
                a218_log++;
            }
        }
        /* === BANZD-A222 probe (v6) ===
         * 0xa222 op=0x6e81 + opnd 0x8208 = `BANZD pmad, *Sind`.
         * The *Sind operand decodes some AR but my v5 guess (AR1) was
         * unverified — capture all ARs so we see which one is non-zero
         * and how it evolves. If AR1=0 systematically, the branch test
         * uses a different AR. Cap=200, plus periodic 100k. */
        if (s->pc == 0xa222 && !s->rpt_active) {
            static unsigned a222_total = 0;
            static int a222_log = 0;
            a222_total++;
            bool log_now = (a222_log < 200) ||
                           (a222_total % 100000 == 0);
            if (log_now) {
                C54_LOG("BANZD-A222 #%d total=%u op=0x%04x op2=0x%04x "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                        "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x "
                        "BRC=%u insn=%u",
                        a222_log + 1, a222_total,
                        s->prog[s->pc], s->prog[(uint16_t)(s->pc + 1)],
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->brc, s->insn_count);
                a222_log++;
            }
        }
        /* Companion probe at 0xa215 (BRC setup) and 0xa217 (outer entry).
         * 0xa215 op=0x4492 + 0xa216 opnd 0x0092 = `ADD/SUB Smem,16,dst` per
         * tic54x (2-word, mask FE00 base 0x4400). Logs A_pre / A_post and
         * the Smem read so we can trace what value lands in dst (may feed
         * BRC eventually). 30-event cap. */
        if (s->pc == 0xa215 || s->pc == 0xa217) {
            static int brc_setup_215 = 0;
            static int brc_setup_217 = 0;
            int *cnt = (s->pc == 0xa215) ? &brc_setup_215 : &brc_setup_217;
            if (*cnt < 30) {
                C54_LOG("ENTER-A%04x #%d AR0=%04x AR1=%04x AR2=%04x "
                        "A=%010llx B=%010llx T=%04x BRC=%u DP=0x%03x insn=%u",
                        s->pc, *cnt + 1,
                        s->ar[0], s->ar[1], s->ar[2],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t, s->brc, (s->st0 & 0x1FF), s->insn_count);
                (*cnt)++;
            }
        }

        /* === XC-COND probe at PC=0xa0e0 / 0xa0e4 (Q1 hypothesis test) ===
         * Per Claude web v3 diag (2026-05-08) : routine 0xa0e0..0xa0e9 ends
         * at PC=0xa0e7 op=0xc8be where AR4 is consistently 0x18 (=MMR_SP)
         * pre-instruction → ST||LD writes to SP, catastrophe.
         *
         * Static dump shows two `XC 1, cond` instructions before 0xc8be :
         *   0xa0e0 = 0xfd30  ; XC 1, cond=0x30 (TC)
         *   0xa0e4 = 0xfd43  ; XC 1, cond=0x43 (ALT, A<0)
         *
         * Hypothesis : if XC condition evaluates to FALSE (TC bit not set, or
         * A not negative), the conditional STM #lk, AR4 (likely at 0xa0e5) is
         * SKIPPED → AR4 keeps stale value of 0x18 from earlier code path.
         *
         * Log every visit with : cond byte, TC/A/B flag values, AR4 value,
         * and the next opcode (which would be skipped or executed). If the
         * "taken" decision is consistently false at one of these XCs, that's
         * the bug. Cap to 100 events per PC. */
        if (s->pc == 0xa0e0 || s->pc == 0xa0e4) {
            static unsigned xc_log_e0;
            static unsigned xc_log_e4;
            unsigned *cnt = (s->pc == 0xa0e0) ? &xc_log_e0 : &xc_log_e4;
            if (*cnt < 100) {
                uint16_t op_xc = s->prog[s->pc];
                uint8_t  cond_byte = op_xc & 0xFF;
                uint16_t next_op   = s->prog[(uint16_t)(s->pc + 1)];
                /* Mirror the condition decode from c54x_exec_one (case 0xF
                 * XC handler around line 1108+) — only the common subset. */
                bool cond = false;
                if      (cond_byte == 0x00) cond = true;
                else if (cond_byte == 0x0C) cond = (s->st0 & ST0_C) != 0;
                else if (cond_byte == 0x08) cond = !(s->st0 & ST0_C);
                else if (cond_byte == 0x30) cond = (s->st0 & ST0_TC) != 0;
                else if (cond_byte == 0x20) cond = !(s->st0 & ST0_TC);
                else if (cond_byte == 0x45) cond = (sext40(s->a) == 0);
                else if (cond_byte == 0x44) cond = (sext40(s->a) != 0);
                else if (cond_byte == 0x46) cond = (sext40(s->a) > 0);
                else if (cond_byte == 0x42) cond = (sext40(s->a) >= 0);
                else if (cond_byte == 0x43) cond = (sext40(s->a) < 0);
                else if (cond_byte == 0x47) cond = (sext40(s->a) <= 0);
                else if (cond_byte == 0x4D) cond = (sext40(s->b) == 0);
                else if (cond_byte == 0x4C) cond = (sext40(s->b) != 0);
                else if (cond_byte == 0x4E) cond = (sext40(s->b) > 0);
                else if (cond_byte == 0x4A) cond = (sext40(s->b) >= 0);
                else if (cond_byte == 0x4B) cond = (sext40(s->b) < 0);
                else if (cond_byte == 0x4F) cond = (sext40(s->b) <= 0);
                if (calypso_debug_enabled("XC-COND")) fprintf(stderr,
                        "[c54x] XC-COND #%u PC=0x%04x op=0x%04x cond=0x%02x "
                        "→ %s | TC=%d C=%d A=%010llx (sgn:%c) "
                        "B=%010llx (sgn:%c) AR4=0x%04x next_op=0x%04x insn=%u\n",
                        *cnt + 1, s->pc, op_xc, cond_byte,
                        cond ? "TAKEN " : "SKIPPED",
                        !!(s->st0 & ST0_TC),
                        !!(s->st0 & ST0_C),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        sext40(s->a) < 0 ? '-' : (sext40(s->a) == 0 ? '0' : '+'),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        sext40(s->b) < 0 ? '-' : (sext40(s->b) == 0 ? '0' : '+'),
                        s->ar[4], next_op, s->insn_count);
                (*cnt)++;
            }
        }

        /* === MAC-8d33 trace — FB-det inner correlator ===
         * The DSP loops indefinitely in 0x8d2d..0x8d36. Static dump shows :
         *   8d2d 0x771a 0x0004      ; (2-word) — likely setup
         *   8d2f 0xf072 0x8d33      ; RPTB pmad, end=0x8d33 (per tic54x)
         *   8d31 0xf461             ; F46x = SFTA src,shift,dst (1-word)
         *   8d32 0xf591             ; F591 = ROL B (per our decoder)
         *   8d33 0xf3e2             ; F3E0-F3FF = SFTL src,SHIFT,DST  ← writes a_sync_SNR
         *   8d34 0x6e89 0x8d2d      ; BANZD pmad=0x8d2d, *AR — outer back-branch
         *   8d36 0xf3e1             ; SFTL B,1,B (exit path)
         * PC HIST counts (105k outer / 526k inner = 5×) confirm the 5-iter
         * RPTB body is (0x8d32, 0x8d33, 0x8d34) repeated 5 times.
         *
         * Capture A_pre, T, AR2..AR5 at each PC inside this zone. Rate-limit :
         *   first 50 always (init + early convergence)
         *   every 5000th (steady-state cadence)
         *   when |A_after - last_logged_A| > 0x100000 (significant accumulator
         *   shift = convergence event worth dumping)
         * Plus a dedicated "ENTER 0x8d2d" outer-iter counter that always logs
         * A_pre at the OUTER entry, so we can tell whether the accumulator
         * is reset between FB-det attempts (Observation 1 from session diag). */
        if (s->pc >= 0x8d2c && s->pc <= 0x8d3a) {
            static uint64_t mac8d_count;
            static int64_t  last_logged_a;
            int64_t a_now = sext40(s->a);
            int64_t da = a_now - last_logged_a;
            if (da < 0) da = -da;
            mac8d_count++;
            bool log_now = (mac8d_count <= 50) ||
                           (mac8d_count % 5000) == 0 ||
                           da > 0x100000LL;
            if (log_now) {
                C54_LOG("MAC-8d33 #%llu PC=0x%04x op=0x%04x A_pre=%010llx B=%010llx "
                        "T=0x%04x ARs: %04x %04x %04x %04x %04x %04x BRC=%d insn=%u",
                        (unsigned long long)mac8d_count,
                        s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->t,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->brc, s->insn_count);
                last_logged_a = a_now;
            }
        }
        /* Dedicated outer-entry tracer at PC=0x8d2d : ALWAYS log A_pre on
         * entry (cap to 200 events). If A is non-zero on outer entry,
         * the accumulator wasn't reset between attempts — observation 1
         * from 2026-05-08 session : 21× 0x2fb0 SNR could mean stuck
         * accumulator across attempts. */
        if (s->pc == 0x8d2d) {
            static uint64_t enter_8d2d;
            enter_8d2d++;
            if (enter_8d2d <= 200) {
                C54_LOG("ENTER-8d2d #%llu A_pre=%010llx B_pre=%010llx T=0x%04x "
                        "ARs: %04x %04x %04x %04x %04x %04x %04x %04x SP=0x%04x BRC=%d insn=%u",
                        (unsigned long long)enter_8d2d,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->t,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->sp, s->brc, s->insn_count);
            }
        }

        /* === HOT-OPS PROBE for 0xe9ac..0xe9b7 + 0xe981..0xe983 ===
         * Diag v2 2026-05-08 : DSP locked in deterministic 7-instruction
         * loop at 0xe9ac..0xe9b7 (PROM1 mirror), with outer 3-PC loop
         * 0xe981..0xe983 reloading a BRC counter — pattern consistent
         * with `RPTB end_addr` + outer reset. We need the actual opcodes
         * to confirm/refute the RPTB hypothesis. One-shot dump on first
         * entry into the body range, with surrounding context (a few
         * words before for the RPTB instruction itself, and the outer). */
        {
            static bool e9ac_dumped = false;
            if (!e9ac_dumped && s->pc >= 0xe9ac && s->pc <= 0xe9b7) {
                e9ac_dumped = true;
                fprintf(stderr,
                        "[c54x] HOT-OPS-DUMP triggered at PC=0x%04x insn=%u\n",
                        s->pc, s->insn_count);
                fprintf(stderr,
                        "[c54x] HOT-OPS prog[0xe9a0..0xe9bf]:");
                for (uint16_t a = 0xe9a0; a <= 0xe9bf; a++)
                    fprintf(stderr, " %04x", s->prog[a]);
                fprintf(stderr, "\n");
                fprintf(stderr,
                        "[c54x] HOT-OPS prog[0xe97c..0xe98f] (outer):");
                for (uint16_t a = 0xe97c; a <= 0xe98f; a++)
                    fprintf(stderr, " %04x", s->prog[a]);
                fprintf(stderr, "\n");
                fprintf(stderr,
                        "[c54x] HOT-OPS state: BRC=%d RSA=0x%04x REA=0x%04x "
                        "rptb_active=%d ST1=0x%04x AR0..7: %04x %04x %04x %04x "
                        "%04x %04x %04x %04x\n",
                        s->brc, s->rsa, s->rea, s->rptb_active, s->st1,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* Track SP changes inside RPTB loops */
        uint16_t sp_before = s->sp;
        /* === Plan B captures (c web review) : snapshot for transfer ring,
         * A-write ring, NOP-region guard. */
        uint16_t pre_pc  = s->pc;
        uint8_t  pre_xpc = s->xpc & 0x3;
        uint16_t pre_op  = prog_fetch(s, s->pc);
        int64_t  pre_a   = s->a;

        /* Trace EB04 loop — dump first 20 iterations */
        if (s->pc == 0xEB04) {
            static int eb04_log = 0;
            if (eb04_log < 20) {
                C54_LOG("EB04 op=%04x A=0x%010llx B=0x%010llx T=%04x "
                        "INTM=%d IMR=%04x IFR=%04x rptb=%d RSA=%04x REA=%04x BRC=%d "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->rptb_active, s->rsa, s->rea, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                eb04_log++;
            }
        }

        /* Dump DSP state when stuck — triggers once after 500M instructions
         * if DSP hasn't reached IDLE yet */
        {
            static int dumped = 0;
            if (s->insn_count > 500000000 && !dumped && !s->idle) {
                dumped = 1;
                C54_LOG("DSP NO-IDLE dump at insn=%u PC=0x%04x:", s->insn_count, s->pc);
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  IMR=0x%04x IFR=0x%04x rptb=%d RSA=0x%04x REA=0x%04x BRC=%d",
                        s->imr, s->ifr, s->rptb_active, s->rsa, s->rea, s->brc);
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x XPC=%d",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t, s->xpc);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                /* Dump code around current PC (using prog_fetch for correct OVLY) */
                C54_LOG("  Code around PC:");
                for (int i = -4; i < 16; i++) {
                    uint16_t a = s->pc + i;
                    C54_LOG("  %c [0x%04x] = 0x%04x",
                            i == 0 ? '>' : ' ', a, prog_fetch(s, a));
                }
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* BSP read entry points — these functions contain PORTR PA=0xF430
         * (read BSP sample). If DSP never visits them, the FB-det chain is
         * dead. Targets identified by static analysis of PROM0 callers of
         * the 64 PORTR PA=0xF430 sites at 0x9b80+. */
        if (!s->rpt_active &&
            (s->pc == 0x9a78 || s->pc == 0x9aaf || s->pc == 0x9ad3 ||
             s->pc == 0x9b4c || s->pc == 0x8811)) {
            static unsigned bsp_visits[5];
            int idx = (s->pc == 0x9a78) ? 0 :
                      (s->pc == 0x9aaf) ? 1 :
                      (s->pc == 0x9ad3) ? 2 :
                      (s->pc == 0x9b4c) ? 3 : 4;
            if (bsp_visits[idx] < 5) {
                bsp_visits[idx]++;
                C54_LOG("BSP-ENTRY PC=0x%04x  A=0x%010llx ar0=%04x ar1=%04x "
                        "ar2=%04x ar3=%04x ar4=%04x SP=0x%04x insn=%u",
                        s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4],
                        s->sp, s->insn_count);
            }
        }

        /* Trace any write touching the dispatcher poll addresses
         * data[0x4359] / data[0x3fab]. We never see them go non-zero;
         * confirm whether ANY code path writes them. */
        /* (handled in data_write — see below) */

        /* Dispatcher hot loop trace at PROM0 0xb968-0xb9a4 — the state
         * machine the DSP spins in when waiting for ARM tasks. Logs the
         * first 8 visits per PC so we see the full conditional structure
         * (which addresses it polls, which constants it compares to). */
        if (s->pc >= 0xb968 && s->pc <= 0xb9a4 && !s->rpt_active) {
            static uint8_t disp_visits[64];
            int idx = s->pc - 0xb968;
            if (idx >= 0 && idx < 64 && disp_visits[idx] < 8) {
                disp_visits[idx]++;
                C54_LOG("DISP-TRACE PC=0x%04x op=0x%04x A=0x%010llx "
                        "B=0x%010llx ar0=%04x ar1=%04x ar2=%04x ar3=%04x "
                        "ar4=%04x ar5=%04x TC=%d",
                        s->pc, prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        !!(s->st0 & ST0_TC));
            }
        }

        /* IRQ vec area trace: log every PC visit in 0xFFCC-0xFFE0
         * (INT3 + TINT0 + BRINT0 vec slots). Captures the 3 actual
         * 4-word handlers our IRQ INT3 dispatch lands on at IPTR=0x1ff.
         * 80 unique PCs max, log first 4 visits each. */
        if (s->pc >= 0xFFCC && s->pc < 0xFFE0 && !s->rpt_active) {
            static uint8_t vec_visits[20];   /* index 0 = 0xffcc */
            int idx = s->pc - 0xFFCC;
            if (vec_visits[idx] < 4) {
                vec_visits[idx]++;
                C54_LOG("VEC-TRACE PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx "
                        "B=0x%010llx TC=%d INTM=%d ar7=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        !!(s->st0 & ST0_TC),
                        !!(s->st1 & ST1_INTM),
                        s->ar[7]);
            }
        }

        /* Trace DSP init - log once per unique PC in E900-E960 */
        if (s->pc >= 0xE900 && s->pc < 0xE960 && !s->rpt_active) {
            static uint16_t seen_pcs[96];
            int idx = s->pc - 0xE900;
            if (!seen_pcs[idx]) {
                seen_pcs[idx] = 1;
                C54_LOG("INIT PC=0x%04x op=0x%04x SP=0x%04x BRC=%d rptb=%d RSA=0x%04x REA=0x%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, s->brc,
                        s->rptb_active, s->rsa, s->rea);
            }
        }

        /* Trace SINT17 handler (0x8a00-0x8a5f) */
        if (s->pc >= 0x8a00 && s->pc < 0x8a60) {
            static int sint17_log = 0;
            if (sint17_log < 500) {
                C54_LOG("SINT17 PC=0x%04x op=0x%04x SP=0x%04x DP=0x%03x A=0x%010llx B=0x%010llx AR0=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, dp(s),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->ar[0]);
                sint17_log++;
            }
        }

        /* Sample PC every 1M instructions to find stuck loops */
        if (executed > 0 && (executed % 1000000) == 0) {
            static int sample_log = 0;
            if (sample_log < 20)
                C54_LOG("@%dM: PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                        executed/1000000, s->pc, prog_read(s, s->pc), s->sp, s->insn_count);
            sample_log++;
        }
        if (run_num <= 2 && executed < 2000) {
            C54_LOG("BOOT[%d.%d] PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx B=0x%010llx",
                    run_num, executed, s->pc, prog_fetch(s, s->pc), s->sp,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFLL));
        }
        /* RPTB check moved below — must run AFTER `s->pc += consumed` so
         * that when the body's last instruction has executed and PC has
         * advanced to REA+1, the redirect to RSA is the FINAL operation
         * on PC for this iteration. The previous placement (before PC
         * advance) caused a 1-instruction off-by-one : redirect set
         * pc=RSA, then `s->pc += consumed` bumped it to RSA+1, so the
         * first body instruction was never re-executed across iterations
         * (PC HIST showed body=[RSA+1..REA+1] instead of [RSA..REA]). */

        /* Trace the IMR loop: how does the DSP reach 0x03F0? */
        /* Trace RPTB entry at 0x76FD: dump all AR values */
        if (s->pc == 0x76FD) {
            static int rptb_entry_log = 0;
            if (rptb_entry_log < 30)
                C54_LOG("RPTB-ENTRY PC=0x76FD AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x ARP=%d DP=%d BRC=%d SP=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        arp(s), dp(s), s->brc, s->sp);
            rptb_entry_log++;
        }
        if (s->pc == 0x03F0) {
            static int f3_log = 0;
            if (f3_log < 2) {
                C54_LOG("PC=0x03F0 op=0x%04x insn=%u SP=0x%04x IMR=0x%04x XPC=%d PMST=0x%04x",
                        prog_fetch(s, s->pc), s->insn_count, s->sp, s->imr, s->xpc, s->pmst);
                C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        pc_ring[(pc_ring_idx-20)&255], pc_ring[(pc_ring_idx-19)&255],
                        pc_ring[(pc_ring_idx-18)&255], pc_ring[(pc_ring_idx-17)&255],
                        pc_ring[(pc_ring_idx-16)&255], pc_ring[(pc_ring_idx-15)&255],
                        pc_ring[(pc_ring_idx-14)&255], pc_ring[(pc_ring_idx-13)&255],
                        pc_ring[(pc_ring_idx-12)&255], pc_ring[(pc_ring_idx-11)&255],
                        pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                        pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                        pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                        pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                        pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                f3_log++;
            }
        }

        /* Boot trace */
        if (g_boot_trace > 0) {
            C54_LOG("BOOT[%d] PC=0x%04x op=0x%04x SP=0x%04x PMST=0x%04x",
                    51 - g_boot_trace, s->pc, prog_fetch(s, s->pc), s->sp, s->pmst);
            g_boot_trace--;
        }

        /* Execute instruction */
        int consumed;
        uint16_t exec_pc = s->pc;
        uint16_t exec_op = prog_fetch(s, s->pc);
        /* === FBWATCH-ALIVE (canary) : PROUVE que la sonde est armée + sample PC
         * foreground. Si CE log sort, g_fbwatch_on=1 et le silence des autres
         * FBWATCH est RÉEL. S'il ne sort PAS, les probes étaient mortes. Fire
         * garanti tous les ~20M insns (≈10 lignes sur le run). === */
        if (g_fbwatch_on > 0 && (s->insn_count % 20000000u) == 0) {
            fprintf(stderr, "[c54x] FBWATCH-ALIVE insn=%u PC=0x%04x INTM=%d SP=0x%04x\n",
                    s->insn_count, exec_pc, !!(s->st1 & ST1_INTM), s->sp);
        }
        /* === FBWATCH-POLL : le foreground polle un flag via BITF @0xf7af/0xf7b7
         * (RC NTC = boucle tant que TC=0). Capture l'adresse du flag (AR0..AR2 +
         * data) + TC pour ID le bit jamais posé = ce qu'il faut câbler (modèle HW). */
        if (g_fbwatch_on > 0 && (exec_pc == 0xf7af || exec_pc == 0xf7b7)) {
            static unsigned wpoll = 0;
            if (wpoll++ < 30) {
                uint16_t mask = prog_fetch(s, exec_pc + 1);
                fprintf(stderr, "[c54x] FBWATCH-POLL pc=0x%04x mask=0x%04x | "
                        "AR0=0x%04x d=0x%04x | AR1=0x%04x d=0x%04x | AR2=0x%04x d=0x%04x | TC=%d insn=%u\n",
                        exec_pc, mask,
                        s->ar[0], s->data[s->ar[0]], s->ar[1], s->data[s->ar[1]],
                        s->ar[2], s->data[s->ar[2]], !!(s->st0 & ST0_TC), s->insn_count);
            }
        }
        /* === FBWATCH (2) : le handler FB 0x9ac0 tourne-t-il ? (env one-shot) === */
        if (g_fbwatch_on > 0 && exec_pc == 0x9ac0) {
            static unsigned w9 = 0;
            if (w9++ < 40)
                fprintf(stderr, "[c54x] FBWATCH-9AC0 #%u insn=%u SP=0x%04x DP=0x%03x\n",
                        w9, s->insn_count, s->sp, s->st0 & 0x1FF);
        }
        /* === FBWATCH-INITTAB : la routine d'init de la table de dispatch
         * (0xc704, peuple data[0x4c24-0x4c5d] = cibles BACC-A/CALA) tourne-t-elle ?
         * 0 hit = jamais atteinte = root confirmé (boot saute le setup-pass). */
        if (g_fbwatch_on > 0 && (exec_pc == 0xc704 || exec_pc == 0xc472)) {
            static unsigned wit = 0;
            if (wit++ < 10)
                fprintf(stderr, "[c54x] FBWATCH-INITTAB pc=0x%04x insn=%u SP=0x%04x\n",
                        exec_pc, s->insn_count, s->sp);
        }
        /* === FBWATCH (4) PRODUCTEUR/CONSOMMATEUR : le dispatch CALAD @0x833b
         * tourne-t-il par-frame, et quelle adresse handler calcule-t-il dans A ?
         * 0 ligne = dispatcher mort (producteur). A jamais 0x9ac0 = la jump-table/
         * formule ne produit jamais le handler FB. A=0x9ac0 = FB dispatché mais
         * ne détecte pas (bug handler). Cap haut pour voir la distribution. */
        if (g_fbwatch_on > 0 && exec_pc == 0x833b) {
            static unsigned wdp = 0;
            if (wdp++ < 120)
                fprintf(stderr, "[c54x] FBWATCH-DISP #%u insn=%u A_handler=0x%04x DP=0x%03x SP=0x%04x\n",
                        wdp, s->insn_count, (uint16_t)(s->a & 0xffff), s->st0 & 0x1FF, s->sp);
        }
        /* CORR-ENTRY tracker (env CALYPSO_CORRELATOR_TRACE=1) : capture
         * transition out→in du range FB-det [0x8d00..0x9000). Cf top of
         * file pour la lazy-init + l'évidence runtime 2026-05-25 night. */
        corr_entry_track(s->pc, s);
        /* FBDB-PROBE (env CALYPSO_FBDB_PROBE=1, c web reframe 2026-05-25 night2) :
         * trace B@fbd9, A@fbdb (= post F2xx SUB), A@fbf3 (= before STLM A,AR4). */
        fbdb_probe_check_pc(s->pc, s);
        /* FORCE-INTM-ONESHOT (env CALYPSO_FORCE_INTM_ONESHOT=1, c web reframe
         * 2026-05-25 night4) : sonde arbitrage — clear INTM UNE FOIS quand
         * INTM=1 + BRINT0 pending. Observe via tracers existants si aval sain. */
        force_intm_oneshot_check(s);
        /* STUCK-PROBE (env CALYPSO_STUCK_PROBE=1, c web reframe 2026-05-25 night3) :
         * capture PC+XPC histogramme quand INTM=1 + BRINT0 pending. */
        stuck_probe_check(s);

        /* === CALA-70C3 FORENSIC PROBES (2026-05-27, c web review) ===
         * Pourquoi : DSP boucle infiniment sur CALA A à PROM0[0x70c3] avec
         * A=0x0001_70c3 (auto-référence). A_H=0x0001 ne peut PAS venir d'un
         * `LD Smem,A` sext40-é (qui donne A_H ∈ {0x0000, 0xFFFF}), donc
         * writer = DLD upstream ou compose H+L. Probes pour identifier :
         *   1. Source du jump vers 0x70c3 (XPC:PC + opcode@prev_pc), gated
         *      FIRST-HIT pour échapper à la pollution post-runaway (MMR XPC
         *      écrasé quand SP rampage à travers data[0x18..0x1F]).
         *   2. Compteur LD@0x70c1 — si 0, confirme le jump direct (skip LD).
         *   3. Dernier writer de A (PC qui a posé 0x0001_70c3 dans A).
         * Active par défaut, coût ~3 branches/insn. */
        static int      p70c3_first    = 0;
        static uint64_t p70c1_counter  = 0;
        static uint16_t p_last_a_pc    = 0xFFFF;
        static int64_t  p_last_a_val   = 0;
        int64_t a_before_exec = s->a;

        if (s->pc == 0x70c1) p70c1_counter++;

        if (s->pc == 0x70c3 && !p70c3_first) {
            p70c3_first = 1;
            uint16_t prev_pc = pc_ring[(pc_ring_idx - 2) & 255];
            uint16_t prev_op = prog_fetch(s, prev_pc);
            C54_LOG("PROBE-CALA70C3-FIRST insn=%u XPC=%u PC=0x%04x op=0x%04x "
                    "prev_pc=0x%04x prev_op=0x%04x "
                    "A=%010llx (A_G=0x%02x A_H=0x%04x A_L=0x%04x) "
                    "LD@70C1_count=%llu last_A_writer_pc=0x%04x last_A_val=%010llx "
                    "SP=0x%04x BK=0x%04x",
                    s->insn_count, s->xpc & 0x3, s->pc, prog_fetch(s, s->pc),
                    prev_pc, prev_op,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    (uint8_t)((s->a >> 32) & 0xFF),
                    (uint16_t)((s->a >> 16) & 0xFFFF),
                    (uint16_t)(s->a & 0xFFFF),
                    (unsigned long long)p70c1_counter,
                    p_last_a_pc,
                    (unsigned long long)(p_last_a_val & 0xFFFFFFFFFFULL),
                    s->sp, s->bk);
            C54_LOG("PROBE-CALA70C3-TRAIL pc[-12..-1] = "
                    "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-13)&255], pc_ring[(pc_ring_idx-12)&255],
                    pc_ring[(pc_ring_idx-11)&255], pc_ring[(pc_ring_idx-10)&255],
                    pc_ring[(pc_ring_idx-9)&255],  pc_ring[(pc_ring_idx-8)&255],
                    pc_ring[(pc_ring_idx-7)&255],  pc_ring[(pc_ring_idx-6)&255],
                    pc_ring[(pc_ring_idx-5)&255],  pc_ring[(pc_ring_idx-4)&255],
                    pc_ring[(pc_ring_idx-3)&255],  pc_ring[(pc_ring_idx-2)&255]);
        }

        {
            /* DISP-ENTRY : prédécesseur = PC exécuté à l'itération précédente */
            static uint16_t s_last_run_pc = 0;
            g_prev_pc = s_last_run_pc;
            s_last_run_pc = s->pc;
        }
        /* SURGICAL : capture silencieuse du slot LUT lu au 0x834d (LD
         * (DP<<7|0x07)<<1,A). 1 compare/insn, pas de log → ~zéro impact
         * timing. Sert le probe BLACKHOLE-CALA (self-CALA 0x70c3). */
        if (s->pc == 0x834d) {
            g_disp_lut_ea  = (uint16_t)(((s->st0 & 0x1FF) << 7) | 0x07);
            g_disp_lut_val = s->data[g_disp_lut_ea];
        }
        uint16_t sp_before_exec = s->sp;
        uint16_t ds_before = s->delay_slots;  /* delay-slot word-count fix 2026-05-31 */
        consumed = c54x_exec_one(s);
        if (g_halt_sp && s->sp < g_halt_sp && s->sp != sp_before_exec) { g_halted = 1; break; }  /* over-pop catch */

        /* === STACK-UNDERFLOW probe (SPUNDER=1, 2026-06-09) ===
         * ⚠ THEORY DISPROVEN: assumed SP=0x1EC6 was a fixed run-mode base and any
         * pop above it was an underflow. The SP-HIWM probe below showed SP ranges
         * widely (0x1100→0x4ab7+) in normal nested execution, so this 0x1EC6
         * threshold never fires usefully. The real warm-boot clobber is a SINGLE
         * deterministic orphan `retd @0x940d` (pops stale 0xf074) at insn≈482M —
         * a stack MISALIGNMENT, not a drift. Use CALYPSO_DEBUG=ORPHAN (shadow
         * stack, line ~499) instead. Kept gated/off; see docs/5110-dsp-boot-sequence.md §5. */
        if (g_c54x_warm_active && s->sp > sp_before_exec && s->sp > 0x1EC6) {
            static int spu = 0;
            if (spu < 30 && calypso_debug_enabled("SPUNDER")) {
                fprintf(stderr, "[c54x] SP-UNDERFLOW #%d sp 0x%04x->0x%04x "
                        "popped=0x%04x exec_pc=0x%04x op=0x%04x insn=%u  trail:",
                        spu, sp_before_exec, s->sp,
                        s->data[(uint16_t)(s->sp - 1)],
                        exec_pc, prog_fetch(s, exec_pc), s->insn_count);
                for (int i = 12; i >= 1; i--)
                    fprintf(stderr, " %04x", pc_ring[(pc_ring_idx - i) & 255]);
                fprintf(stderr, "\n");
                spu++;
            }
        }
        /* SPHIWM diag: after warm boot, log the running MAX sp (empty-stack
         * level) and the sp trajectory through the clobber routines, so we can
         * see the actual base + whether the pop crosses it. Gated SPUNDER. */
        if (g_c54x_warm_active && calypso_debug_enabled("SPUNDER")) {
            static uint16_t hi = 0; static int hilog = 0;
            if (s->sp > hi && hilog < 400) {
                fprintf(stderr, "[c54x] SP-HIWM new max sp=0x%04x exec_pc=0x%04x "
                        "op=0x%04x insn=%u\n", s->sp, exec_pc,
                        prog_fetch(s, exec_pc), s->insn_count);
                hi = s->sp; hilog++;
            }
        }
        /* === PC-WINDOW trace (PCWIN=1) ===
         * Dump every executed PC + op + SP + key regs inside a tight insn window,
         * to see EXACTLY how a deterministic control-flow lands somewhere it
         * shouldn't. Default window brackets the cold-loader1 finalize over-pop
         * (ret@0xf73, insn~482274537) and its cascade to the 0xf08d IMR clobber.
         * Override bounds with PCWINLO / PCWINHI (decimal insn counts). */
        if (calypso_debug_enabled("PCWIN")) {
            static long long pw_lo = -1, pw_hi = -1;
            if (pw_lo < 0) {
                const char *l = getenv("PCWINLO"), *h = getenv("PCWINHI");
                pw_lo = l ? strtoll(l, 0, 0) : 482274480LL;
                pw_hi = h ? strtoll(h, 0, 0) : 482274585LL;
            }
            if (s->insn_count >= (uint64_t)pw_lo && s->insn_count <= (uint64_t)pw_hi)
                fprintf(stderr, "[c54x] PCWIN insn=%u pc=0x%04x op=0x%04x sp=0x%04x "
                        "A=%010llx ar0=0x%04x ar1=0x%04x ar3=0x%04x ar4=0x%04x ar5=0x%04x "
                        "ar7=0x%04x brc=%u intm=%d\n",
                        s->insn_count, exec_pc, prog_fetch(s, exec_pc), s->sp,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->ar[0], s->ar[1], s->ar[3], s->ar[4], s->ar[5],
                        s->ar[7], s->brc, !!(s->st1 & ST1_INTM));
        }

        /* === PCEQ probe (PCEQ=0xADDR) ===
         * Log ONLY when exec_pc == PCEQ — dumps AR0/AR1/AR7/BRC, the AR0-base
         * pointer data[0x1448], and IMR (mem[0]). Built 2026-06-09 to trace the
         * 0xb399 `sth a,*ar0-` AR0 underflow that writes mem[0]=IMR=0 (the legacy
         * vocoder-spin / reason-0x68 class). It pinned the root: the A/B
         * dual-operand handler (case 0xA/0xB, ~L8617) mis-extracts the Xmem/Ymem
         * AR index (raw (op>>4)&7 / op&7 instead of ((nib)&3)+2) so 0xa413
         * `mpy *ar3,*ar5` lands on AR1 and force-increments it → AR1 (BRC table
         * ptr) drifts → stray BRC=0xCF04 → 52996-iter runaway → AR0→0 → IMR=0.
         * See memory c54x-decode-length-bug-class. Rate-cap via PCEQMAX (def 80).
         * PCEQDATA=0xADDR : also dump 8 data[] words from ADDR (e.g. the BIST
         * loop params 0x1220..) + accumulator A — to trace the 0xf133 BIST
         * outer-loop tap count data[0x1220] (loops forever if <40). */
        if (calypso_debug_enabled("PCEQ")) {
            static long pq_pc = -1, pq_max = 80, pq_n = 0, pq_data = -2;
            if (pq_pc < 0) { const char *p = getenv("PCEQ"); pq_pc = p ? strtol(p,0,0) : 0xb399;
                             const char *m = getenv("PCEQMAX"); if (m) pq_max = strtol(m,0,0); }
            if (pq_data == -2) { const char *d = getenv("PCEQDATA"); pq_data = d ? strtol(d,0,0) : -1; }
            if (exec_pc == (uint16_t)pq_pc && pq_n < pq_max) {
                pq_n++;
                fprintf(stderr, "[c54x] PCEQ insn=%u pc=0x%04x ar0=0x%04x ar1=0x%04x "
                        "ar2=0x%04x ar3=0x%04x ar7=0x%04x brc=%u A=%010llx IMR=0x%04x intm=%d",
                        s->insn_count, exec_pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[7], s->brc, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->data[0], !!(s->st1 & ST1_INTM));
                if (pq_data >= 0) {
                    fprintf(stderr, " data[0x%04lx]:", pq_data);
                    for (int _i = 0; _i < 8; _i++)
                        fprintf(stderr, " %04x", s->data[(uint16_t)(pq_data + _i)]);
                }
                fprintf(stderr, "\n");
            }
        }

        /* === In-core snapshot trigger (per-instruction granularity) ===
         * C54X_SNAPSHOT=<path> + C54X_SNAPPC=<pc> : when the DSP is about to
         * execute <pc> (s->pc == pc, after this instruction advanced), freeze the
         * whole core (regs+prog+data+api_ram) for standalone replay. Unlike the
         * MCU-side DSP54_SNAPSHOT this catches transient one-time PCs that fall
         * between the RATIO-step boundaries. C54X_SNAPAFTER=<insn> guards the cold
         * boot; C54X_SNAPSTOP=1 exits the process right after saving. */
        if (calypso_debug_enabled("C54X_SNAPSHOT")) {
            static int   sn_init = 0, sn_stop = 0, sn_done = 0;
            static long  sn_pc = -1; static uint64_t sn_after = 0;
            static const char *sn_path = NULL;
            if (!sn_init) { sn_init = 1;
                sn_path = getenv("C54X_SNAPSHOT");
                const char *p = getenv("C54X_SNAPPC"); if (p && *p) sn_pc = strtol(p,0,0);
                const char *a = getenv("C54X_SNAPAFTER"); if (a && *a) sn_after = strtoull(a,0,0);
                sn_stop = getenv("C54X_SNAPSTOP") ? 1 : 0;
            }
            if (sn_path && !sn_done && sn_pc >= 0
                && s->pc == (uint16_t)sn_pc && s->insn_count >= sn_after) {
                sn_done = 1;
                int rc = c54x_snapshot_save(s, s->api_ram, sn_path);
                fprintf(stderr, "[c54x] SNAPSHOT %s @insn=%u pc=0x%04x sp=0x%04x (rc=%d)\n",
                        sn_path, s->insn_count, (unsigned)s->pc, s->sp, rc);
                if (sn_stop) { fflush(stderr); exit(rc ? 1 : 0); }
            }
        }

        /* === DECODE-AUDIT (2026-06-02, brief CC-web : audit différentiel) ===
         * Inventaire du décodeur sur l'overlay corrélateur 0x8000-0x9FFF :
         * logge UNE fois par (PC,op) distinct l'opcode brut, le mot suivant, et
         * la LONGUEUR consommée (= consumed, inclut lk_used). À diff contre
         * doc/opcodes/tic54x_hi8_map.md — colonne longueur d'abord (un mauvais
         * len désync tout le flux suivant, cf bug 0x86/0x87→SP runaway). Vise à
         * vider la classe « décode/longueur/mode » d'un coup au lieu de peler
         * bug par bug. dedup par (PC,op) → capture aussi les variantes XPC d'un
         * même Pag. Gate CALYPSO_DEBUG=DECODE-AUDIT, coût ~nul après couverture. */
        static int da_lo = -1, da_hi = -1;
        static long long da_insn = -1;
        if (da_lo < 0) {
            const char *l = getenv("CALYPSO_DA_LO"); const char *h = getenv("CALYPSO_DA_HI");
            const char *n = getenv("CALYPSO_DA_INSN");
            da_lo = l ? (int)strtol(l, NULL, 0) : 0x8000;   /* overlay corrélateur par défaut */
            da_hi = h ? (int)strtol(h, NULL, 0) : 0x9FFF;   /* CALYPSO_DA_LO/HI pour élargir (ex. 0x7000..0xFFFF) */
            da_insn = n ? strtoll(n, NULL, 0) : 0;          /* CALYPSO_DA_INSN : skip le boot, viser la fenêtre détection (ex. 250000000) */
        }
        if (exec_pc >= (uint16_t)da_lo && exec_pc <= (uint16_t)da_hi
            && s->insn_count >= (uint64_t)da_insn
            && calypso_debug_enabled("DECODE-AUDIT")) {
            static uint16_t da_op_seen[0x10000];
            static uint8_t  da_has[0x10000];
            static unsigned da_n = 0;
            uint16_t da_op = prog_fetch(s, exec_pc);
            /* skip op=0x0000 : trous PROM vide (runaway control-flow, pas un bug
             * de DÉCODE) — ils empoisonnaient le cap au boot (sweep 0xCB00-0xD3FF). */
            if (da_op != 0x0000 && da_n < 4000
                && (!da_has[exec_pc] || da_op_seen[exec_pc] != da_op)) {
                da_has[exec_pc] = 1;
                da_op_seen[exec_pc] = da_op;
                da_n++;
                fprintf(stderr, "[c54x] DECODE-AUDIT PC=0x%04x op=%04x op2=%04x "
                        "hi8=%02x len=%d XPC=%u insn=%u\n",
                        exec_pc, da_op, prog_fetch(s, exec_pc + 1),
                        (da_op >> 8) & 0xFF, consumed, s->xpc & 0xFF,
                        s->insn_count);
            }
        }
        /* SP-event ring : enregistre tout changement de SP (push/pop) avec
         * le PC/op responsable. Sert le dump BLACKHOLE-CALA. */
        if (s->sp != sp_before_exec) {
            struct sp_evt *e = &g_spring[g_spring_idx++ & 63];
            e->pc = exec_pc;
            e->op = prog_fetch(s, exec_pc);
            e->delta = (int16_t)(s->sp - sp_before_exec);
            e->sp = s->sp;
            g_sp_ledger.net_words += (int16_t)(sp_before_exec - s->sp);
            if ((int16_t)(s->sp - sp_before_exec) < 0) g_sp_ledger.sp_pushes++;
            else g_sp_ledger.sp_pops++;

            /* PROBE 2026-05-31 : SP qui PLONGE dans la zone API RAM (0x0700-0x0a00)
             * = corruption → CALLD push clobber d_fb_det/d_fb_mode (0x08f8/9).
             * Loggue l'instruction qui y fait entrer SP (transition depuis hors
             * zone) + delta + voisinage pile. Nomme le setter de SP fautif. */
            {
                static uint32_t spdz_n = 0;
                int in_zone   = (s->sp >= 0x0700 && s->sp <= 0x0a00);
                int was_out   = (sp_before_exec < 0x0700 || sp_before_exec > 0x0a00);
                if (in_zone && was_out && spdz_n < 30) {
                    fprintf(stderr, "[c54x] SP-DANGER SP 0x%04x→0x%04x (delta=%+d) "
                            "PC=0x%04x op=0x%04x XPC=%u insn=%u\n",
                            sp_before_exec, s->sp,
                            (int)(int16_t)(s->sp - sp_before_exec),
                            exec_pc, prog_fetch(s, exec_pc), s->xpc, s->insn_count);
                    spdz_n++;
                }
            }

            /* === SHADOW STACK : appariement push/pop (gate ORPHAN) ===
             * Nomme LE return orphelin (over-pop), pas les 15 victimes 0xc8be. */
            if (g_shadow_on < 0) {
                const char *eo = getenv("CALYPSO_ORPHAN");  /* env dédiée (hors CALYPSO_DEBUG) */
                g_shadow_on = (eo && *eo) ? 1 : 0;
            }
            if (g_shadow_on) {
                uint16_t op = e->op;
                int16_t  d  = e->delta;
                int is_call = (op==0xF074||op==0xF274||op==0xF4E3||op==0xF4E7||op==0xF6E3);
                int is_ret  = (op==0xFC00||op==0xFE00||op==0xF4EB||op==0xF4E4||op==0xF6EB
                               ||(op&0xFF00)==0xFC00);   /* RET/RETD/RETE/FRET/RETED + RC cond */
                int is_pshm = ((op&0xFF00)==0x4A00||(op&0xFF00)==0x4B00);
                (void)is_call;
                if (d < 0) {                 /* PUSH : SP a baissé */
                    int words = -d, w;
                    char kind = is_pshm ? 'P' : 'C';   /* PSHM=data ; reste=adresse retour */
                    for (w = 0; w < words; w++) {
                        if (g_shadow_depth >= 0 && g_shadow_depth < SHADOW_N) {
                            g_shadow[g_shadow_depth].pc   = exec_pc;
                            g_shadow[g_shadow_depth].op   = op;
                            g_shadow[g_shadow_depth].sp   = s->sp;
                            g_shadow[g_shadow_depth].kind = kind;
                        }
                        g_shadow_depth++;
                    }
                } else if (d > 0) {          /* POP : SP a monté */
                    int words = d, w;
                    for (w = 0; w < words; w++) {
                        g_shadow_depth--;
                        if (is_ret) {
                            if (g_shadow_depth < 0) {
                                g_orphan_hits++;
                                if (g_orphan_hits <= 40) {
                                    /* cible du return : RETD/RETED arment delayed_pc
                                     * (commit différé), RET/FRET immédiat = s->pc. */
                                    uint16_t ret_tgt = (s->delay_slots ? s->delayed_pc : s->pc);
                                    /* dernier PUSH réel de g_spring (le CALL apparié
                                     * manquant) : scan arrière sur delta<0. */
                                    uint16_t lp_pc = 0, lp_op = 0; int lp_found = 0, scan;
                                    for (scan = 1; scan <= 64; scan++) {
                                        struct sp_evt *pe = &g_spring[(g_spring_idx - scan) & 63];
                                        if (pe->delta < 0) { lp_pc = pe->pc; lp_op = pe->op;
                                                             lp_found = 1; break; }
                                    }
                                    fprintf(stderr,
                                        "[c54x] ORPHAN-RETURN #%llu insn=%u pc=0x%04x op=0x%04x "
                                        "SP=0x%04x → ret_tgt=0x%04x  lastPUSH=%s(pc=0x%04x op=0x%04x) "
                                        "net_words=%lld — over-pop (pile vierge au-dessus de SP_base)\n",
                                        (unsigned long long)g_orphan_hits, s->insn_count,
                                        exec_pc, op, s->sp, ret_tgt,
                                        lp_found ? "" : "AUCUN", lp_pc, lp_op,
                                        (long long)g_sp_ledger.net_words);
                                    /* slot lu par ce return : écrit (vecteur
                                     * légit) ou VIERGE (vrai garbage) ? */
                                    {
                                        uint16_t rs = (uint16_t)(s->sp - 1);
                                        if (rs >= STKSLOT_LO && rs <= STKSLOT_HI) {
                                            int si = rs - STKSLOT_LO;
                                            if (g_stkslot_written[si])
                                                fprintf(stderr, "[c54x]     slot 0x%04x ÉCRIT par "
                                                    "ST@pc=0x%04x op=0x%04x → VECTEUR LÉGIT (pas un bug)\n",
                                                    rs, g_stkslot_wpc[si], g_stkslot_wop[si]);
                                            else
                                                fprintf(stderr, "[c54x]     slot 0x%04x JAMAIS écrit "
                                                    "→ VIERGE = vrai over-pop garbage\n", rs);
                                        }
                                    }
                                    /* Au TOUT premier orphan : dump complet du ring
                                     * g_spring (reset→over-pop) pour compter push vs
                                     * pop directement = racine structurelle vs bug. */
                                    if (g_orphan_hits == 1) {
                                        int k;
                                        fprintf(stderr, "[c54x]   g_spring (anciens→récents, reset→#1):\n");
                                        for (k = 64; k >= 1; k--) {
                                            struct sp_evt *pe = &g_spring[(g_spring_idx - k) & 63];
                                            if (pe->pc == 0 && pe->op == 0 && pe->delta == 0) continue;
                                            fprintf(stderr, "[c54x]     pc=0x%04x op=0x%04x %s%d SP→0x%04x\n",
                                                    pe->pc, pe->op, pe->delta < 0 ? "PUSH" : "POP ",
                                                    pe->delta < 0 ? -pe->delta : pe->delta, pe->sp);
                                        }
                                    }
                                }
                            } else if (g_shadow[g_shadow_depth].kind != 'C') {
                                g_mismatch_hits++;
                                if (g_mismatch_hits <= 40)
                                    fprintf(stderr,
                                        "[c54x] MISMATCH-RETURN #%llu insn=%u pc=0x%04x op=0x%04x "
                                        "SP=0x%04x dépile kind='%c' poussé par pc=0x%04x op=0x%04x — "
                                        "return lit une valeur non-retour (PSHM)\n",
                                        (unsigned long long)g_mismatch_hits, s->insn_count,
                                        exec_pc, op, s->sp, g_shadow[g_shadow_depth].kind,
                                        g_shadow[g_shadow_depth].pc, g_shadow[g_shadow_depth].op);
                            }
                        }
                    }
                }
                if (g_shadow_depth < 0) g_shadow_depth = 0;  /* re-ancre après orphan */
            }
        }

        /* === CORR-ABG probe (2026-05-30, c-web) : la FB-det est FRÉQUENTIELLE
         * (FCCH = ton pur), pas un pic d'amplitude. Au site corrélateur 0xec07,
         * capture A & B SÉPARÉS (= I/Q de la corr complexe), l'angle atan2(B,A)
         * (= la fréquence vue par le détecteur), et les valeurs aux 4 pointeurs
         * AR (data-I/Q vs table de réf cos/sin — vérifie que la réf est un vrai
         * sinus, pas du garbage/zéro). Cap 30. */
        /* DETECTOR-RUN (2026-05-30) : compteur d'exécutions du VRAI détecteur
         * freq FCCH (0x9ac0). Pourquoi ne tourne-t-il qu'1× au boot ? Loggue
         * insn + d_fb_mode (0x08f9, large vs étroit) + d_task_md (0x0804/0x0818)
         * à chaque passage. */
        if (exec_pc == 0x9ac0) {
            static unsigned dr = 0;
            if (dr < 30 || (dr % 200) == 0)
                fprintf(stderr, "[c54x] DETECTOR-RUN #%u @0x9ac0 d_fb_mode[08f9]=0x%04x "
                        "d_fb_det[08f8]=0x%04x insn=%u\n",
                        dr, s->data[0x08f9], s->data[0x08f8], s->insn_count);
            dr++;
        }
        if (exec_pc == 0xec07) {
            static unsigned cr = 0;
            if (cr < 30) {
                int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
                int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
                /* angle=atan2(B,A) calculé à l'analyse (pas de math.h ici). */
                fprintf(stderr, "[c54x] CORR-ABG #%u A=%lld B=%lld | "
                        "AR2=%04x[%04x] AR3=%04x[%04x] AR4=%04x[%04x] AR5=%04x[%04x] insn=%u\n",
                        cr, (long long)a, (long long)b,
                        s->ar[2], s->data[s->ar[2]], s->ar[3], s->data[s->ar[3]],
                        s->ar[4], s->data[s->ar[4]], s->ar[5], s->data[s->ar[5]],
                        s->insn_count);
                cr++;
            }
        }

        /* === CORR-PEAK probe (2026-05-30) : au store du TOA (PC=0x9ac0, STL A
         * dans a_sync_demod) dumper A/B complets + AR + T + la fenêtre d'entrée
         * lue (buffer BSP 0x2a00) → voir comment le corrélateur dérive le TOA
         * (offset peak, wrap, référence) à partir d'une FCCH pourtant correcte.
         * Cap 40, ~zéro coût hors site. */
        if (exec_pc == 0xa0e7 || exec_pc == 0x9ac0) {
            static unsigned cp_log = 0;
            /* Ne fire QUE quand une vraie I/Q est présente (input non-nul),
             * sinon le cap est gaspillé sur le boot (buffer vide). On teste
             * 0x2a00 (BSP write) ET 0x2c00 (où AR3 pointe = lecture corr). */
            /* Gate sur RX buffer 0x2a00 NON-NUL : ne fire que quand une vraie
             * I/Q est présente (sinon gaspillé au boot/vide). Capture la vraie
             * corrélation FCCH dès que le BSP livre. */
            if (cp_log < 40 && (s->data[0x2a00] || s->data[0x2a02] ||
                                s->data[0x2a04] || s->data[0x2a08] ||
                                s->data[0x2a10] || s->data[0x2a20])) {
                uint16_t a3 = s->ar[3];
                fprintf(stderr,
                    "[c54x] CORR-PEAK #%u PC=0x%04x A=%010llx B=%010llx T=%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x\n"
                    "[c54x]   in@0x2a00: %04x %04x %04x %04x %04x %04x %04x %04x\n"
                    "[c54x]   in@0x2c00: %04x %04x %04x %04x %04x %04x %04x %04x\n"
                    "[c54x]   *AR3@0x%04x: %04x %04x %04x %04x %04x %04x %04x %04x insn=%u\n",
                    cp_log, exec_pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                    s->t, s->ar[2], a3, s->ar[4], s->ar[5],
                    s->data[0x2a00], s->data[0x2a01], s->data[0x2a02], s->data[0x2a03],
                    s->data[0x2a04], s->data[0x2a05], s->data[0x2a06], s->data[0x2a07],
                    s->data[0x2c00], s->data[0x2c01], s->data[0x2c02], s->data[0x2c03],
                    s->data[0x2c04], s->data[0x2c05], s->data[0x2c06], s->data[0x2c07],
                    a3,
                    s->data[a3], s->data[(uint16_t)(a3+1)], s->data[(uint16_t)(a3+2)], s->data[(uint16_t)(a3+3)],
                    s->data[(uint16_t)(a3+4)], s->data[(uint16_t)(a3+5)], s->data[(uint16_t)(a3+6)], s->data[(uint16_t)(a3+7)],
                    s->insn_count);
                fflush(stderr);
                cp_log++;
            }
        }

        /* Track A writes (probe 3, post-exec). exec_pc = PC qui vient
         * d'exécuter. Si A a changé, c'est cet opcode qui a écrit A. */
        if (s->a != a_before_exec) {
            p_last_a_pc  = exec_pc;
            p_last_a_val = s->a;
        }
        /* === END CALA-70C3 FORENSIC PROBES === */

        /* === BOOT-BRANCH probe : control-flow skeleton for boot phase ===
         * Hunt the opcode that branches OVER the DSP init code (which should
         * set SP=0x5AC8 + AR4/AR5 + IMR=0xFFFF). IMR-W trace localised the
         * collateral damage to PC=0xf03a (insn=262501) and PC=0x8ebc
         * (insn=5247868) via stale ARx=0 → STH B,*ARx+ → MMR_IMR=0. The bad
         * branch happens earlier. Log every PC discontinuity (branch, call,
         * return, IRQ entry) while insn_count <= 300000 + snapshot SP/IMR/
         * AR4/AR5 — visualizes when each register got armed (or never did). */
        if (s->insn_count <= 300000) {
            /* c54x_exec_one does NOT advance s->pc for sequential insn — that
             * happens in `s->pc += consumed` further down. So a real branch
             * (or CALL/RET/IRQ entry) is the only thing that modifies s->pc
             * during the exec itself. */
            if (s->pc != exec_pc) {
                static unsigned boot_br_log;
                const unsigned LIMIT = 8000;
                if (boot_br_log < LIMIT) {
                    if (calypso_debug_enabled("BOOT-BRANCH")) fprintf(stderr,
                            "[c54x] BOOT-BRANCH #%u insn=%u %04x(op=%04x,c=%d) → %04x "
                            "SP=%04x IMR=%04x AR4=%04x AR5=%04x INTM=%d\n",
                            boot_br_log, s->insn_count,
                            exec_pc, exec_op, consumed, s->pc,
                            s->sp, s->imr, s->ar[4], s->ar[5],
                            !!(s->st1 & ST1_INTM));
                    boot_br_log++;
                    if (boot_br_log == LIMIT) {
                        if (calypso_debug_enabled("BOOT-BRANCH")) fprintf(stderr,
                                "[c54x] BOOT-BRANCH log capped at %u\n", LIMIT);
                    }
                }
            }
        }

        /* INT3-CYCLE-TRACE (env CALYPSO_INT3_CYCLE_TRACE=1, c web reframe night5) :
         * record branch decisions during INT3 ISR cycle. */
        int3_cycle_track_branch(s, exec_pc, exec_op, consumed);

        /* Detect SP changes — only log after init (insn > 490M) */
        if (s->sp != sp_before && s->insn_count > 490000000) {
            static int sp_leak_log = 0;
            if (sp_leak_log < 100) {
                C54_LOG("SP %+d PC=0x%04x op=0x%04x SP 0x%04x→0x%04x insn=%u",
                        (int16_t)(s->sp - sp_before), exec_pc, exec_op, sp_before, s->sp, s->insn_count);
                sp_leak_log++;
            }
        }

        /* === SP-FLOOR guard + delta histogram (2026-05-27, c web review) ===
         * Trip on FIRST SP descent below SP_FLOOR — that snapshot is BEFORE
         * the MMR auto-corruption (SP at MMR data[0x18]), so it captures the
         * cause not the crash. Plus running delta histogram to identify
         * leaking call/ret pairs (FAR push 2 vs near pop 1 = -1 per pair).
         * Active by default — minimal cost (a few branches per insn). */
        #define SP_FLOOR 0x0080
        {
            static int sp_floor_tripped = 0;
            static uint64_t sp_delta_pushf = 0;  /* delta == -2 */
            static uint64_t sp_delta_pushn = 0;  /* delta == -1 */
            static uint64_t sp_delta_popn  = 0;  /* delta == +1 */
            static uint64_t sp_delta_popf  = 0;  /* delta == +2 */
            static uint64_t sp_delta_other = 0;  /* anything else (jumps, LD#k,SP) */
            static uint64_t sp_delta_log_n = 0;

            int delta = (int)(int16_t)(s->sp - sp_before);
            if (delta != 0) {
                switch (delta) {
                    case -2: sp_delta_pushf++; break;
                    case -1: sp_delta_pushn++; break;
                    case  1: sp_delta_popn++;  break;
                    case  2: sp_delta_popf++;  break;
                    default: sp_delta_other++; break;
                }
                /* Log first 80 SP changes + every 5000 — enough to characterize
                 * the leaking call/ret pair WITHOUT drowning the 1.3 GB log. */
                sp_delta_log_n++;
                if (sp_delta_log_n <= 80 || (sp_delta_log_n % 5000) == 0) {
                    C54_LOG("SP-Δ #%llu PC=0x%04x op=0x%04x XPC=%u Δ%+d  SP 0x%04x→0x%04x insn=%u",
                            (unsigned long long)sp_delta_log_n,
                            exec_pc, exec_op, s->xpc & 0x3,
                            delta, sp_before, s->sp, s->insn_count);
                }
            }

            /* === Plan B detectors (run once per insn AFTER exec_one) === */
            /* (1) A-write ring : track each modification of s->a */
            if (s->a != pre_a) {
                awrite_log_push(pre_pc, pre_xpc, pre_op, pre_a, s->a, s->insn_count);
            }
            /* (2) Transfer ring : detect non-sequential PC change.
             *   exec_one returns `consumed` = instruction size in words. If
             *   new PC != pre_pc + consumed (and no delay-slot pending), it's
             *   a transfer. We also catch XPC changes (FAR transfers). */
            uint16_t expected_pc = (uint16_t)(pre_pc + consumed);
            if ((s->pc != expected_pc || (s->xpc & 0x3) != pre_xpc)
                && s->delay_slots == 0) {
                xfer_log_push(pre_pc, pre_xpc, pre_op,
                              s->pc, s->xpc & 0x3, pre_a, s->insn_count);
            }
            /* (3) NOP-region guard : trip ONCE at first entry into the
             * unmapped prog zone (PC <0x7000 in bank 0, outside OVLY DARAM
             * 0x80-0x27FF). Dumps trigger + transfer ring + A-write ring. */
            if (!g_nop_tripped && pc_in_nop_region(s, s->pc, s->xpc & 0x3)) {
                nop_guard_dump(s, s->pc, s->xpc & 0x3);
            }

            if (!sp_floor_tripped && s->sp < SP_FLOOR) {
                sp_floor_tripped = 1;
                long long net_push = (long long)(sp_delta_pushf*2 + sp_delta_pushn);
                long long net_pop  = (long long)(sp_delta_popf *2 + sp_delta_popn);
                C54_LOG("================================================");
                C54_LOG("SP-FLOOR TRIPPED  SP=0x%04x < 0x%04x  insn=%u",
                        s->sp, (unsigned)SP_FLOOR, s->insn_count);
                C54_LOG("  trigger PC=0x%04x op=0x%04x XPC=%u Δ%+d (SP 0x%04x→0x%04x)",
                        exec_pc, exec_op, s->xpc & 0x3, delta, sp_before, s->sp);
                C54_LOG("  ST1 INTM=%d  IFR=0x%04x  IMR=0x%04x",
                        !!(s->st1 & ST1_INTM), s->ifr, s->imr);
                C54_LOG("SP delta histogram :");
                C54_LOG("  push far  (Δ=-2) : %llu", (unsigned long long)sp_delta_pushf);
                C54_LOG("  push near (Δ=-1) : %llu", (unsigned long long)sp_delta_pushn);
                C54_LOG("  pop  near (Δ=+1) : %llu", (unsigned long long)sp_delta_popn);
                C54_LOG("  pop  far  (Δ=+2) : %llu", (unsigned long long)sp_delta_popf);
                C54_LOG("  other            : %llu", (unsigned long long)sp_delta_other);
                C54_LOG("  net push - pop   : %lld words (positive = SP leaked downward)",
                        net_push - net_pop);
                C54_LOG("================================================");
            }
        }
        #undef SP_FLOOR

        /* v2 SP observability — only when CALYPSO_TRAP_OOR=1.
         * (a) sp_trail[256] : |Δ|>32 events (scheduler reloads + big allocs)
         * (b) sp_low watermark : every new low, PC-coalesced power-of-10
         * Gated to insn>33754 (after init stack 0x9022→0x5ac8 normal). */
        {
            static int trap_armed = -1;
            if (trap_armed < 0) {
                const char *e = cdbg_env("TRAP-OOR"); (void)e;
                /* TRAP-OOR RETIRED 2026-05-29 : c'était l'analyse de descente
                 * SP / SP-CATASTROPHE, résolue (0x70c3 self-CALA / DROM-LUT /
                 * stub). Le site 2 faisait s->running=0 au checkpoint 4.2M =
                 * LE bottleneck (CALYPSO_DEBUG=ALL haltait le DSP). Forcé OFF. */
                trap_armed = 0;
            }
            if (trap_armed && s->sp != sp_before && s->insn_count > 33754) {
                int16_t delta = (int16_t)(s->sp - sp_before);
                uint16_t a_low = (uint16_t)(s->a & 0xFFFF);

                /* SP-HIST per-PC accounting déplacé en TOP-of-loop chokepoint
                 * (fix v6 2026-05-24) — bypass-proof. Voir L6773.
                 * Pas d'appel ici sinon double-count. */

                /* (a) trail — only big jumps (skip push/pop ±1..32 noise) */
                if (delta > 32 || delta < -32) {
                    unsigned k = g_sp_trail_idx & 255;
                    g_sp_trail[k].insn    = s->insn_count;
                    g_sp_trail[k].old_sp  = sp_before;
                    g_sp_trail[k].new_sp  = s->sp;
                    g_sp_trail[k].exec_pc = exec_pc;
                    g_sp_trail[k].exec_op = exec_op;
                    g_sp_trail[k].a_low   = a_low;
                    g_sp_trail_idx++;
                }

                /* (b) sp_low watermark — fires on any new low (incl Δ=-1). */
                if (s->sp < g_sp_low) {
                    g_sp_low = s->sp;
                    if (exec_pc == g_sp_low_pc) {
                        g_sp_low_hits_at_pc++;
                        unsigned n = g_sp_low_hits_at_pc;
                        bool milestone = (n == 1 || n == 10 || n == 100 ||
                                          n == 1000 || n == 10000 || n == 100000);
                        if (milestone) {
                            if (calypso_debug_enabled("SP-LOW")) fprintf(stderr,
                                "[c54x] SP-LOW #%u @pc=0x%04x op=0x%04x "
                                "sp 0x%04x->0x%04x A_low=0x%04x insn=%u\n",
                                n, exec_pc, exec_op,
                                sp_before, s->sp, a_low, s->insn_count);
                        }
                    } else {
                        g_sp_low_pc = exec_pc;
                        g_sp_low_hits_at_pc = 1;
                        g_sp_low_distinct_pcs++;
                        if (calypso_debug_enabled("SP-LOW")) fprintf(stderr,
                            "[c54x] SP-LOW NEW (#%u distinct) @pc=0x%04x op=0x%04x "
                            "sp 0x%04x->0x%04x A_low=0x%04x insn=%u\n",
                            g_sp_low_distinct_pcs, exec_pc, exec_op,
                            sp_before, s->sp, a_low, s->insn_count);
                    }
                }
            }
        }

        /* SP-LEDGER + SP-INTO-MMR probes RETIRÉS 2026-05-23 :
         * info diagnostic déjà extraite (irq_entries=1 sur 144s, SP wrap
         * via stack-relative writes en MMR). Ces probes fire à CHAQUE
         * instruction → overhead non-négligeable sur DSP throughput
         * (mesuré 9.1M insn/s vs 10M required = 9% slow). Reviendront en
         * cas de régression. SP-CATASTROPHE garde la haut |Δ|>256. */

        /* === SP catastrophic delta tracer ===
         * Diag v2 2026-05-08 : SP went from 0x9c1e → 0x0001 in one window
         * (lost ~40k stack words). The progressive-leak log above caps at
         * 100 small deltas and misses the single catastrophic event.
         * This block flags any |Δ| > 100 in one instruction — never
         * capped — so the buggy STM/PSHM/POPM/RETE-corrupted-stack /
         * FRAME-with-huge-offset is unambiguously identified the FIRST
         * time it happens. ARs included so we can see if the ST/LD
         * destination resolved to an MMR slot (e.g. *AR=0x18 → MMR_SP).
         *
         * Threshold raised from 100→256 on 2026-05-08 to filter legitimate
         * FRAME #imm8s (signed 8-bit can be ±127). Real catastrophes from
         * dual-op writing to MMR_SP are always thousands of words. */
        {
            int32_t dsp = (int32_t)(int16_t)(s->sp - sp_before);
            if (dsp > 256 || dsp < -256) {
                if (calypso_debug_enabled("SP-CATASTROPHE")) fprintf(stderr,
                        "[c54x] SP-CATASTROPHE Δ=%+d PC=0x%04x op=0x%04x "
                        "SP 0x%04x → 0x%04x INTM=%d "
                        "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x "
                        "BK=%04x A=%010llx insn=%u\n",
                        (int)dsp, exec_pc, exec_op, sp_before, s->sp,
                        !!(s->st1 & ST1_INTM),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->bk,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->insn_count);
            }
        }
        /* === v2 TRAP-OOR firing point — fixed checkpoint halt ===
         * T1/T2 dropped (v1 v2 redesign: scheduler at 0xfd2a exonerated,
         * SP clobber lives in legit code → no PC whitelist nor SP edge
         * can catch it). Halt at fixed insn checkpoint, dump trail+sp_low
         * for offline analysis of full descent.
         * Checkpoint configurable via CALYPSO_TRAP_CHECKPOINT (default
         * 4200000 = just after the insn=4.09M SP recovery 0x0008→0x2900). */
        {
            static int trap_armed = -1;
            static int tripped = 0;
            static unsigned checkpoint = 0;
            if (trap_armed < 0) {
                const char *e = cdbg_env("TRAP-OOR"); (void)e;
                /* TRAP-OOR RETIRED 2026-05-29 : c'était l'analyse de descente
                 * SP / SP-CATASTROPHE, résolue (0x70c3 self-CALA / DROM-LUT /
                 * stub). Le site 2 faisait s->running=0 au checkpoint 4.2M =
                 * LE bottleneck (CALYPSO_DEBUG=ALL haltait le DSP). Forcé OFF. */
                trap_armed = 0;
                const char *c = getenv("CALYPSO_TRAP_CHECKPOINT");
                checkpoint = (c && *c) ? (unsigned)strtoul(c, NULL, 0) : 4200000u;
            }
            if (trap_armed && !tripped && s->insn_count >= checkpoint) {
                tripped = 1;
                dsp_trap_dump(s, exec_pc, exec_op, sp_before, "CHECKPOINT");
                s->running = 0;
            }
        }

        /* === VARIATEUR DE VITESSE osmocon : le break est MAINTENANT en
         * FIN d'itération (après s->pc += consumed + commit delay-slots),
         * cf bloc plus bas. Casser ici (avant l'avance du pc) ré-exécutait
         * l'instruction courante à la ré-entrée → double-pop des RET/RETD/RCD
         * → over-pop SP → garbage DP → self-CALA 0x70c3. Fix 2026-05-30. */

        /* === DUAL-OP-INTERPRET diagnostic ===
         * Compare current decoder's AR field interpretation (3-bit fields)
         * with SPRU172C's dual-operand encoding (2-bit AR fields + offset 2,
         * AR2..AR5 only). If the two disagree on which AR is used and the
         * SP-CATASTROPHE just fired, we have evidence the encoding is
         * wrong. Cap to 100 entries to avoid log explosion. */
        if ((exec_op & 0xFC00) == 0xC800 && (
             (int32_t)(int16_t)(s->sp - sp_before) > 100 ||
             (int32_t)(int16_t)(s->sp - sp_before) < -100)) {
            static unsigned dop_log;
            if (dop_log++ < 100) {
                int xar_cur = (exec_op >> 4) & 0x07;
                int yar_cur = exec_op & 0x07;
                int xar_spru = ((exec_op >> 4) & 0x03) + 2;
                int yar_spru = (exec_op & 0x03) + 2;
                int xmod_spru = (exec_op >> 6) & 0x03;
                int ymod_spru = (exec_op >> 2) & 0x03;
                fprintf(stderr,
                        "[c54x] DUAL-OP-INTERPRET op=0x%04x PC=0x%04x : "
                        "current_dec X=AR%d Y=AR%d (3bit) | "
                        "SPRU172C    X=AR%d Y=AR%d xmod=%d ymod=%d (2bit+2) | "
                        "AR%d_cur=%04x AR%d_spru=%04x | "
                        "AR%d_cur=%04x AR%d_spru=%04x\n",
                        exec_op, exec_pc,
                        xar_cur, yar_cur,
                        xar_spru, yar_spru, xmod_spru, ymod_spru,
                        xar_cur, s->ar[xar_cur],
                        xar_spru, s->ar[xar_spru],
                        yar_cur, s->ar[yar_cur],
                        yar_spru, s->ar[yar_spru]);
            }
        }

        /* Snapshot the just-executed PC/op into C54xState so other
         * tracers (in particular INTM-TRANS at top of next iteration)
         * can attribute post-instruction state changes to the cause. */
        s->last_exec_pc = exec_pc;
        s->last_exec_op = exec_op;

        /* CALYPSO_CELLWATCH=0xA[,0xB,...] : store-path-INDEPENDENT transition watcher.
         * Polled here, after every executed instruction, reading the backing store
         * (api_ram for the 0x800-0x2800 window, else s->data[]) DIRECTLY — so it catches
         * writes that bypass data_write_locked (block loads, warm-reset DARAM re-init, any
         * direct s->data[] store) which CALYPSO_WATCH_WR cannot see. Reports old->new + the
         * PC that just executed (= the writer) + insn. Default OFF; up to 8 cells; ~1 branch
         * when off, <=8 compares when armed; read-only (no execution effect). */
        {
            static int cw_init = 0, cw_n = 0, cw_armed = 0;
            static uint16_t cw_addr[8], cw_shadow[8];
            if (!cw_init) {
                cw_init = 1;
                const char *e = getenv("CALYPSO_CELLWATCH");
                if (e && *e) {
                    cw_armed = 1;
                    char buf[256]; strncpy(buf, e, sizeof buf - 1); buf[sizeof buf - 1] = 0;
                    for (char *tok = strtok(buf, ","); tok && cw_n < 8; tok = strtok(NULL, ",")) {
                        uint16_t a = (uint16_t)strtol(tok, 0, 0);
                        cw_addr[cw_n] = a;
                        cw_shadow[cw_n] = (a >= 0x800 && a < 0x2800 && s->api_ram)
                                          ? s->api_ram[a - 0x800] : s->data[a];
                        cw_n++;
                    }
                }
            }
            if (cw_armed) {
                for (int i = 0; i < cw_n; i++) {
                    uint16_t a = cw_addr[i];
                    uint16_t v = (a >= 0x800 && a < 0x2800 && s->api_ram)
                                 ? s->api_ram[a - 0x800] : s->data[a];
                    if (v != cw_shadow[i]) {
                        fprintf(stderr, "[c54x] CELLWATCH data[0x%04x] 0x%04x -> 0x%04x  "
                                "writer_pc=0x%04x op=0x%04x insn=%u\n",
                                a, cw_shadow[i], v, s->last_exec_pc, s->last_exec_op,
                                (unsigned)s->insn_count);
                        cw_shadow[i] = v;
                    }
                }
            }
        }


        /* RPT: after executing an instruction while repeat is active,
         * re-execute the SAME instruction (don't advance PC) until count=0. */
        if (s->rpt_active && !s->idle) {
            if (s->rpt_arming) {
                /* Arming pass (the RPT instruction itself): real silicon executes the
                 * TARGET count+1 times; decrementing here made every RPT form run the
                 * target only N times (caught 2026-06-10: the MDIRCV enqueue's
                 * `rpt *(0x8)` copied 7 payload words where the MCU consumer reads
                 * (len+1)/2 = 8 -> ring desync). Fall through to the normal return-0
                 * handling (incl. the delay-slot countdown below).  */
                s->rpt_arming = false;
                /* Fresh repeat: READA/WRITA must re-seed PAR from A on their
                 * first iteration (SPRU172C "A -> PAR"). */
                s->par_set = false;
            } else if (s->rpt_count > 0) {
                s->rpt_count--;
                /* Delayed-branch window accounting for `retd/bd; rpt #k; insn`:
                 * the RPT instruction itself (arming pass — consumed==0, its
                 * handler self-advanced PC) occupies ONE WORD of the 2-word
                 * delay window. Decrement it here (this path `continue`s past
                 * the countdown below); the repeated insn's word is counted
                 * once, on its final fall-through. Clamp at 1 so the branch
                 * never commits mid-repeat — without this the window slipped
                 * one insn past the repeat and executed a bogus 3rd slot. */
                if (consumed == 0 && ds_before > 0 && s->delay_slots > 1)
                    s->delay_slots--;
                /* Don't advance PC — re-execute same instruction next cycle */
                s->cycles++;
                executed++;
                if (s->rpt_count == 0) {
                    static int rpt_done_log = 0;
                    if (rpt_done_log < 10)
                        C54_LOG("RPT DONE PC=0x%04x op=0x%04x count_was=%d", s->pc, prog_fetch(s, s->pc), 0);
                    rpt_done_log++;
                }
                continue;
            } else {
                s->rpt_active = false;
                s->par_set = false;
            }
        }

        if (consumed > 0)
            s->pc += consumed;
        s->pc &= 0xFFFF;  /* C54x has 16-bit PC (23-bit with XPC, but wrap at 16-bit) */
        /* consumed == 0 means PC was set by branch */

        /* Delayed-branch slot countdown.
         * RCD (and later CALLD/RETD/BD/CCD if extended) sets delayed_pc and
         * delay_slots = 2. The two instructions following the RCD execute
         * as normal pipeline slots; once both have completed the branch
         * commits by forcing PC to delayed_pc. */
        /* Delay-slot countdown — compté en MOTS, pas en instructions.
         * BUG FIX 2026-05-31 : l'ancien code décrémentait dès l'itération qui
         * arme delay_slots=2, PUIS d'1 par instruction → une SEULE instruction
         * de delay-slot exécutée. OK pour un slot = 1 insn 2-mots (STM #k), mais
         * pour un slot = DEUX insns 1-mot, la 2e était SAUTÉE. Quand cette 2e
         * insn est un PSHM/PSHD (sites PROM0 0xb53a/0xc9a2/0xcaab = code
         * power-scan que le mobile exécute en cell-search), le push est perdu →
         * over-pop cumulé (~58 mots) → POPM ST0 @0x94f3 ramasse l'orphelin
         * 0x80fd → DP=0x0fd → dispatcher 0x8341 lit la LUT garbage → CALAD
         * 0x70c3 = self-CALA → écrit 0x70c4 (=28868) dans d_fb_det/a_pm →
         * rxlev/TOA poison → NO_CELL_FOUND. cf doc/SP_CATASTROPHE_70c4_SEQUENCE.
         * Le C54x a TOUJOURS 2 mots de delay (SPRU172C) : 1 insn 2-mots OU 2
         * insns 1-mot. On décrémente du nombre de MOTS exécutés (= consumed), et
         * on NE compte PAS l'itération qui arme (ds_before==0 = la branche
         * elle-même ; le delay commence à l'instruction suivante). */
        if (s->delay_slots > 0) {
            if (ds_before == 0) {
                /* itération de la branche différée elle-même : ne rien
                 * décrémenter ; delay_slots (=2) est un compteur de MOTS. */
            } else {
                int wexec = (consumed > 0) ? consumed : 1;
                if (s->delay_slots > wexec) s->delay_slots -= wexec;
                else                        s->delay_slots = 0;
                if (s->delay_slots == 0)
                    s->pc = s->delayed_pc;
            }
        }


        /* === RPTB (block repeat) end-of-body check ===
         * Must run AFTER PC advance and delayed-branch settle so the
         * redirect to RSA is the final word on s->pc for this iteration.
         * Triggers when PC has overshot REA (= reached REA+1 or beyond,
         * accounting for 2-word instructions at the body's tail). Skip
         * during RPT (single-instruction repeat has priority). */
        if (s->rptb_active && !s->rpt_active && s->pc >= s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 20) {
                C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d",
                        s->pc, s->rsa, s->rea, s->brc);
                rptb_log++;
            }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false;
                { static int _re=0;
                  if (_re<50) {
                    C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x",
                            s->pc, s->rsa, s->rea, s->insn_count, s->sp);
                    _re++;
                  }
                }
                s->st1 &= ~ST1_BRAF;
            }
        }

        s->cycles++;
        s->insn_count++;

        executed++;

        /* SP-LEDGER : dump périodique pour valider net_words→0 sur run long
         * (métrique de balance push/pop post-yield-fix). ~1 compare/insn.
         * Gated (CALYPSO_DEBUG=SP-LEDGER) — was unconditional console spam. */
        if (s->insn_count - g_sp_ledger.last_dump_insn >= 20000000u) {
            g_sp_ledger.last_dump_insn = s->insn_count;
            if (calypso_debug_enabled("SP-LEDGER")) {
                fprintf(stderr,
                    "[c54x] SP-LEDGER insn=%u SP=0x%04x net_words=%lld pushes=%llu pops=%llu irq=%llu\n",
                    s->insn_count, s->sp, (long long)g_sp_ledger.net_words,
                    (unsigned long long)g_sp_ledger.sp_pushes,
                    (unsigned long long)g_sp_ledger.sp_pops,
                    (unsigned long long)g_sp_ledger.irq_entries);
                fflush(stderr);
            }
        }

        /* === VARIATEUR DE VITESSE osmocon (gated, CALYPSO_DSP_YIELD=N) ===
         * Le DSP c54x tourne SYNCHRONE dans tdma_tick sur le thread principal.
         * Tous les N insns on sort de c54x_run → la mainloop pompe l'I/O
         * (osmocon) puis délivre les IT au DSP.
         *   N PETIT = yield fréquent = osmocon rapide / DSP ralenti
         *   N GRAND = yield rare / 0 = OFF (legacy, DSP garde tout le budget)
         * IMPÉRATIF : ne casser qu'à un BOUNDARY PROPRE — ici, après
         * `s->pc += consumed` ET le commit des delay-slots (delay_slots==0).
         * Sinon (a) l'instruction courante est ré-exécutée à la ré-entrée
         * (double-pop) et (b) un IT délivré par la mainloop tomberait au
         * milieu des delay-slots d'un RETD/RCD → retour différé corrompu.
         * Les deux mènent à l'over-pop SP → DP garbage → self-CALA 0x70c3.
         * (Valeur idéale = statique à déterminer ; gardée en env pour l'instant.) */
        {
            static int dsp_yield = -1;
            if (dsp_yield < 0) {
                const char *e = getenv("CALYPSO_DSP_YIELD");
                /* Défaut statique 32768 (2^15) : cadence DSP↔osmocon/IT calée
                 * (valeur trouvée empiriquement, ON par défaut). 0 = OFF legacy
                 * seulement si CALYPSO_DSP_YIELD=0 explicite. */
                dsp_yield = (e && *e) ? atoi(e) : 32768;
                if (dsp_yield < 0) dsp_yield = 0;
                fprintf(stderr, "[c54x] CALYPSO_DSP_YIELD = %d insn/yield %s\n",
                        dsp_yield, dsp_yield ? "(variateur ON)" : "(OFF, legacy)");
            }
            /* Bien implémenté (fix 2026-05-30) : ne yielder qu'à un point
             * INTERRUPTIBLE. Le yield rend la main à la mainloop qui délivre
             * l'IT (INT3) au DSP ; sur vrai C54x une IT n'est prise qu'à INTM=0
             * (hors section critique). Couper sur un simple compteur d'insns
             * tombait en pleine séquence de dispatch (INTM=1, DP hérité avant
             * LDP) → l'IT au resume corrompait DP/ST0 → CALAD vers la LUT
             * (wedge 0x9207) ou self-CALA. On exige donc :
             *   - executed >= dsp_yield ET INTM=0 (point sûr), OU
             *   - executed >= 4×dsp_yield (cap dur : évite la famine mainloop
             *     si le firmware reste en INTM=1 anormalement longtemps).
             * delay_slots==0 garde inchangée (jamais mid-branche-différée). */
            /* 4 gardes = les 4 états non-interruptibles du C54x :
             *   delay_slots==0  : pas mid-branche-différée (RETD/RCD/CALLD/BD)
             *   !rpt_active     : pas mid-RPT (single-repeat = NON interruptible
             *                     sur HW jusqu'à RC épuisé ; RPTB l'est, lui)
             *   INTM==0         : interruptible (hors section critique/dispatch)
             *   (+ break après commit pc/delay = pas mid-instruction)
             * Cap dur 4× : si INTM reste 1 anormalement, force le yield pour
             * éviter la famine mainloop (cas "illégal" tracé ci-dessous). */
            if (dsp_yield > 0 && s->delay_slots == 0 && !s->rpt_active &&
                ((executed >= (unsigned)dsp_yield && !(s->st1 & ST1_INTM)) ||
                 executed >= (unsigned)dsp_yield * 4u)) {
                /* Preuve du gate : log les premiers breaks + tout break "cap-forcé"
                 * (INTM=1 = illégal toléré). Si on ne voit JAMAIS de cap-forcé sur
                 * N runs ET 0x9207 disparaît → le gate est prouvé, pas juste constaté. */
                if (calypso_debug_enabled("YIELD-BREAK")) {
                    static unsigned yb = 0;
                    int forced = (s->st1 & ST1_INTM) ? 1 : 0;
                    if (yb < 40 || forced)
                        fprintf(stderr, "[c54x] YIELD-BREAK #%u INTM=%d delay=%d rpt=%d pc=0x%04x exec=%u %s\n",
                                yb, forced, s->delay_slots, s->rpt_active, s->pc, executed,
                                forced ? "*** CAP-FORCED (INTM=1 illegal) ***" : "(safe)");
                    yb++;
                }
                break;   /* boundary propre + interruptible → mainloop sert I/O + IT */
            }
        }
    }
    return executed;
}

/* ================================================================
 * ROM loader
 * ================================================================ */



/* ================================================================
 * Init / Reset / Interrupts
 * ================================================================ */

C54xState *c54x_init(void)
{
    C54xState *s = calloc(1, sizeof(C54xState));
    if (!s) return NULL;
    return s;
}

void c54x_set_api_ram(C54xState *s, uint16_t *api_ram)
{
    s->api_ram = api_ram;
}

void c54x_set_ovly_unified(C54xState *s, int on)
{
    s->ovly_unified = on ? 1 : 0;
}

/* === Core snapshot save / load (2026-06-09) ===
 * Freeze the ENTIRE C54x core (regs + prog[] + data[] + bsp + the api_ram OVLY
 * window) to a flat file, so the post-upload / pre-BIST state can be replayed
 * standalone in seconds instead of re-running the ~6-min MCU co-sim each time.
 * Host-endian, single-machine use only (sizeof check guards struct drift).
 * api_ram (the [0x800,0x2800) OVLY window backing store) is a separate buffer
 * the caller owns; we serialize C54X_API_SIZE words of it and on load re-point
 * s->api_ram at the caller's buffer (the in-struct pointer is stale). */
#define C54X_SNAP_MAGIC 0x35345053u  /* "SP45" */
struct c54x_snap_hdr { uint32_t magic, version, state_size, api_words; };

int c54x_snapshot_save(const C54xState *s, const uint16_t *api_ram, const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    struct c54x_snap_hdr h = { C54X_SNAP_MAGIC, 1, (uint32_t)sizeof(C54xState),
                               api_ram ? (uint32_t)C54X_API_SIZE : 0u };
    int ok = (fwrite(&h, sizeof h, 1, f) == 1) && (fwrite(s, sizeof *s, 1, f) == 1);
    if (ok && api_ram) ok = (fwrite(api_ram, sizeof(uint16_t), C54X_API_SIZE, f) == C54X_API_SIZE);
    fclose(f);
    return ok ? 0 : -1;
}

/* Load a snapshot into *s and (if present) the caller's api_ram buffer; re-points
 * s->api_ram at it and clears the stale ARM callbacks. Returns 0 on success. */
int c54x_snapshot_load(C54xState *s, uint16_t *api_ram_buf, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    struct c54x_snap_hdr h;
    int ok = (fread(&h, sizeof h, 1, f) == 1) && h.magic == C54X_SNAP_MAGIC
             && h.version == 1 && h.state_size == sizeof(C54xState)
             && (fread(s, sizeof *s, 1, f) == 1);
    if (ok && h.api_words) {
        if (api_ram_buf && h.api_words == C54X_API_SIZE)
            ok = (fread(api_ram_buf, sizeof(uint16_t), C54X_API_SIZE, f) == C54X_API_SIZE);
        else ok = 0;
    }
    fclose(f);
    if (!ok) return -1;
    /* fix up stale pointers from the saved struct image */
    s->api_ram = (h.api_words && api_ram_buf) ? api_ram_buf : NULL;
    s->api_write_cb = NULL;
    s->api_write_cb_opaque = NULL;
    return 0;
}

void c54x_set_initial_pc(C54xState *s, uint32_t pc)
{
    s->pc = pc;
    s->blob_loaded = true;
    C54_LOG("set_initial_pc: PC=0x%05x (blob_loaded=1)", pc);
}

int c54x_load_blob_daram(C54xState *s, const char *path, uint16_t daram_addr)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        C54_LOG("load_blob_daram: cannot open '%s'", path);
        return -1;
    }
    int words = 0;
    uint32_t addr = daram_addr;
    uint8_t buf[2];
    while (addr < C54X_DATA_SIZE && fread(buf, 1, 2, f) == 2) {
        uint16_t w = buf[0] | ((uint16_t)buf[1] << 8);
        s->data[addr] = w;
        if (s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = w;
        addr++;
        words++;
    }
    fclose(f);
    C54_LOG("load_blob_daram: %d words at DARAM[0x%04x..0x%04x] from %s",
            words, daram_addr, addr - 1, path);
    return words;
}

int c54x_load_section(C54xState *s, const char *path,
                      uint32_t start_addr, bool is_program)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        C54_LOG("load_section: cannot open '%s'", path);
        return -1;
    }
    uint16_t *mem = is_program ? s->prog : s->data;
    uint32_t limit = is_program ? C54X_PROG_SIZE : C54X_DATA_SIZE;
    int words = 0;
    uint32_t addr = start_addr;
    uint8_t buf[2];
    while (addr < limit && fread(buf, 1, 2, f) == 2) {
        uint16_t w = buf[0] | ((uint16_t)buf[1] << 8);
        mem[addr] = w;
        if (!is_program && s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = w;
        addr++;
        words++;
    }
    fclose(f);
    C54_LOG("load_section: %d words at %s[0x%05x..0x%05x] from %s",
            words, is_program ? "prog" : "data",
            start_addr, addr - 1, path);
    return words;
}

int c54x_load_registers(C54xState *s, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        C54_LOG("load_registers: cannot open '%s'", path);
        return -1;
    }
    int words = 0;
    uint8_t buf[2];
    /* First 0x20 words = MMR page → reset-override buffer (applied in
     * c54x_reset). Remaining words (0x20..0x5F = low scratch DARAM) →
     * data[] directly, like a section load. */
    while (words < C54X_DATA_SIZE && fread(buf, 1, 2, f) == 2) {
        uint16_t w = buf[0] | ((uint16_t)buf[1] << 8);
        if (words < 0x20)
            s->reg_init[words] = w;
        else
            s->data[words] = w;
        words++;
    }
    fclose(f);
    if (words >= 0x20)
        s->reg_init_valid = true;
    C54_LOG("load_registers: %d words from %s (MMR reset override %s)",
            words, path, s->reg_init_valid ? "ON" : "OFF (file too short)");
    return words;
}

void c54x_reset(C54xState *s)
{
    g_boot_trace = 50;
    s->blob_loaded = false;  /* explicit reset exits dsp-blob fixture mode */
    s->a = 0; s->b = 0;   /* mode c54x = reset datasheet propre : A=B=0
                           * (le snapshot a BL=0x60, appliqué seulement en mode bin) */
    /* ── REVIEW registres : hardcode C ↔ calypso_dsp.Registers.bin (2026-05-31) ──
     * Le hardcode ci-dessous = mode "c54x" = RESET DATASHEET PROPRE (champs
     * critiques alignés au snapshot, champs bénins/garbage à 0). Les 3 modes
     * sont ainsi orthogonaux (sélecteur plus bas) :
     *   c54x   = ce hardcode propre, indépendant du fichier
     *   bin    = override depuis .Registers.bin VERBATIM (snapshot exact, défaut)
     *   hybrid = bin pour l'opérationnel + champs critiques forcés propres
     *            (IFR=0, AR0=0xFF75, BRC/RSA/REA=0) → ≈ c54x sur ces champs
     *
     * Le .bin est un SNAPSHOT mi-exécution (post-handshake bootloader). Review
     * champ par champ (verdict = pertinence de la valeur AU RESET) :
     *
     *   MMR        valeur(bin=hardcode)  classe        commentaire
     *   IMR  0x00  0x52FD                CRITIQUE-OK   masque IRQ, identique 3 dumps
     *   IFR  0x01  0x0008                BÉNIN         bit3 INT3 pending ; INTM=1 masque
     *                                                  → hybrid le met à 0 (datasheet pur)
     *   ST0  0x06  0x181F                CRITIQUE-OK   DP=0x1F
     *   ST1  0x07  0x2900                CRITIQUE-OK   INTM/SXM/XF
     *   A    08-0a  0x000000             OK            accumulateur A = 0
     *   B    0b-0d  0x000060             BÉNIN         BL=0x60 (rechargé avant usage)
     *   T    0x0E  0x0000                OK
     *   TRN  0x0F  0xFF75                BÉNIN         Viterbi, neutre au reset
     *   AR0  0x10  0x5AAD                BÉNIN         rechargé (LD @0x7120) ; hybrid=0xFF75
     *   AR1-5 11-15 invariants           CRITIQUE-OK   identiques 3 dumps (API_RAM, etc.)
     *   AR6  0x16  0xBAE6                BÉNIN         rechargé avant usage
     *   AR7  0x17  0x1E44                BÉNIN         idem
     *   SP   0x18  0x1100                CRITIQUE-OK   pile post-handshake
     *   BK   0x19  0xFFF6                CRITIQUE-OK   circular buffer
     *   BRC  0x1A  0x8FD7                GARBAGE       reste RPTB mi-vol ; neutre
     *   RSA  0x1B  0xD9EC                GARBAGE       (rptb_active=false au reset)
     *   REA  0x1C  0xBBEF                GARBAGE       → hybrid les met à 0
     *   PMST 0x1D  0xFFA8                CRITIQUE-OK   IPTR=0x1FF, MP_MC, OVLY, DROM
     *   XPC  0x1E  0x0000                — jamais overridé (page runtime, fetch vec)
     *
     * CONCLUSION review : les champs CRITIQUE-OK (SP/ST0/ST1/PMST/IMR/AR1-5/BK)
     * sont identiques bin↔hardcode et pilotent le reset. Les divergences (IFR,
     * AR0/6/7, TRN, B, BRC/RSA/REA) sont toutes BÉNIGNES ou GARBAGE et neutres
     * au reset (IT masquée par INTM=1 ; AR rechargés ; rptb_active=false). Donc
     * aucune n'explique le stuck FB (boucle BITF @0xf2cd sur data[0x585f]).
     */
    /* AR registers aligned with silicon spec (doc/datasheets/README.md §3,
     * 2026-05-25). Cross-checked 3 ROM dumps (3311/3416/3606) + local osmocom :
     *   AR1=0x005F, AR2=0x0813, AR3=0x0014, AR4=0x0003, AR5=0x0014  (invariant)
     *   AR0=0xFF75, BK=0xFFF6  (local osmocom dump values)
     *   AR6, AR7 : non documenté invariant, garde 0
     *
     * Précédent : memset 0 = init shortcut, même problème que SP/IMR.
     * Symptôme : STL A,*AR2 à PC=0x9ac0 avec AR2=0 écrivait à mem[0x00]=IMR
     * → IMR cleared → toutes IRQ FRAME/BRINT0 masquées → DSP bloqué en df9x. */
    /* AR registers — mode c54x = reset datasheet propre.
     * AR1-5 = invariants cross-dump (3311/3416/3606) = identiques au snapshot,
     * gardés ici car CRITIQUES. AR0/AR6/AR7 = 0 (neutres : le firmware les
     * recharge avant usage, ex. LD @0x7120). Le snapshot a AR0=0x5aad,
     * AR6=0xbae6, AR7=0x1e44 → appliqués seulement en mode bin. */
    memset(s->ar, 0, sizeof(s->ar));
    s->ar[0] = 0x5AAD;  /* FIX 2026-05-31 : AR0 = valeur silicium (snapshot bin).
                         * PROUVÉ read-before-write à insn=1 (PC=0xb410 ORM
                         * data[*AR0]) via sonde AR-FIRSTUSE → AR0 reset est
                         * load-bearing, l'ancien 0xFF75 (dump local, "neutre")
                         * faisait diverger c54x vs bin dès la 1ʳᵉ instruction du
                         * boot. Aligné sur le silicium → convergence des modes. */
    s->ar[1] = 0x005F;
    s->ar[2] = 0x0813;  /* API_RAM-related — clobber IMR si =0 (cf 2026-05-25) */
    s->ar[3] = 0x0014;
    s->ar[4] = 0x0003;
    s->ar[5] = 0x0014;
    s->t = 0; s->trn = 0;   /* TRN=0 (snapshot 0xff75, neutre au reset) */
    s->sp = 0x1100; s->bk = 0xFFF6;  /* SP+BK init aligned with silicon (2026-05-25).
                                 * 3 ROM dumps (3311/3416/3606) + local : SP=0x1100
                                 * post-bootloader-handshake. Let firmware repoint
                                 * to its own stack (0x5AC8 historically observed)
                                 * via init sequence, comme sur silicon réel.
                                 * Précédent : SP=0x5AC8 = shortcut anticipant
                                 * la re-init firmware. Suspect d'être la racine
                                 * du clobber AR5↔SP overlap à mem[0x3fbe].
                                 * Voir doc/datasheets/README.md §3-4. */
    /* BRC/RSA/REA = 0 (reset datasheet propre). Le snapshot capture des restes
     * de RPTB mi-vol (BRC=0x8fd7 RSA=0xd9ec REA=0xbbef) = GARBAGE sans sens au
     * reset ; appliqués seulement en mode bin. rptb_active=false (posé plus bas)
     * → ces registres ne sont consultés qu'après qu'un RPTB les recharge. */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    /* MMR reset values aligned with Calypso silicon (3 FreeCalypso ROM dumps + local).
     * Empirically validated 2026-04-28. See doc/datasheets/README.md §3.
     * Previous QEMU values (st0=0, st1=ST1_INTM, pmst=0xFFE0) were partial. */
    s->st0  = 0x181F;                              /* DP=0x01F per silicon */
    s->st1  = ST1_INTM | ST1_SXM | ST1_XF;         /* 0x2900: INTM=1, SXM=1, XF=1 */
    s->pmst = 0xFFA8;                              /* IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1 */
    s->imr = 0x52FD;                               /* IMR aligned avec local osmocom dump
                                                    * (doc/datasheets/README.md §3, post-
                                                    * bootloader-handshake). 0 était un autre
                                                    * shortcut comme SP. IRQ #2..#10 vus
                                                    * INTM=1 IMR=0x0000 IFR=0x28 → IRQs
                                                    * masquées toutes → handlers jamais run
                                                    * → flags dispatcher pas écrits → DSP
                                                    * boucle indéfiniment en df9x (= bloqueur
                                                    * #2 chain FBSB). Fix 2026-05-25. */
    s->ifr = 0;        /* IFR=0 (reset datasheet propre). Le snapshot a 0x0008
                        * (bit3 INT3 pending) ; appliqué seulement en mode bin.
                        * Neutre de toute façon : INTM=1 (ST1=0x2900) masque l'IT. */
    s->xpc = 0;
    /* ===================== Sélecteur d'état reset registres =====================
     * Trois modes, choisis par env CALYPSO_DSP_REG_MODE :
     *   "c54x"   → hardcode C ci-dessus UNIQUEMENT (le .bin chargé est ignoré).
     *   "bin"    → snapshot calypso_dsp.Registers.bin override TOUT (verbatim).
     *   "hybrid" → snapshot bin POUR les registres opérationnels validés, MAIS
     *              garde le hardcode pour les champs où le .bin est jugé faux
     *              par l'audit anti-drift (cf table plus haut) :
     *                IFR  : bin=0x0008 (IRQ pending résiduel) → hardcode 0
     *                AR0  : bin=0x5aad (non validé)           → hardcode 0xFF75
     *                BRC/RSA/REA : bin=garbage RPTB mi-vol     → hardcode 0
     * Défaut : "bin" si un .bin est chargé (continuité avec le comportement
     * câblé par run.sh), sinon forcément le hardcode (rien à overrider).
     * Tous lus une fois (reset appelé 2× : boot + DSP_DL_STATUS_READY). */
    {
        static int reg_mode = -1;  /* 0=c54x 1=bin 2=hybrid */
        if (reg_mode < 0) {
            const char *e = getenv("CALYPSO_DSP_REG_MODE");
            if      (e && !strcasecmp(e, "c54x"))   reg_mode = 0;
            else if (e && !strcasecmp(e, "hybrid")) reg_mode = 2;
            else                                    reg_mode = 1; /* "bin"/défaut */
            C54_LOG("reset: CALYPSO_DSP_REG_MODE=%s → mode=%s",
                    e ? e : "(unset)",
                    reg_mode == 0 ? "c54x(hardcode)" :
                    reg_mode == 2 ? "hybrid" : "bin");
        }
        if (s->reg_init_valid && reg_mode != 0) {
            const uint16_t *r = s->reg_init;
            /* Registres opérationnels — communs bin + hybrid */
            s->imr  = r[0x00];
            s->st0  = r[0x06];
            s->st1  = r[0x07];
            s->a    = ((int64_t)(r[0x0a] & 0xFF) << 32) |
                      ((uint32_t)r[0x09] << 16) | r[0x08];
            s->b    = ((int64_t)(r[0x0d] & 0xFF) << 32) |
                      ((uint32_t)r[0x0c] << 16) | r[0x0b];
            s->t    = r[0x0e];
            s->trn  = r[0x0f];
            for (int i = 1; i < 8; i++)   /* AR1..AR7 ; AR0 traité plus bas */
                s->ar[i] = r[0x10 + i];
            s->sp   = r[0x18];
            s->bk   = r[0x19];
            s->pmst = r[0x1d];
            if (reg_mode == 1) {
                /* FIX 2026-05-31 : bin ne doit PLUS avaler le garbage RPTB du
                 * snapshot. BRC/RSA/REA = restes d'une boucle RPTB capturée
                 * mi-vol → invalides au reset → RPTB fantôme au boot → wedge
                 * précoce 0x010b. Avec clean (=0), bin CONVERGE avec c54x
                 * (progresse à 0xf17c, INTM=0). IFR aussi forcé 0 (cohérent).
                 * Prouvé : c54x (RPTB propre) progresse, bin (garbage) wedge. */
                s->ifr   = 0x0000;
                s->ar[0] = r[0x10];   /* 0x5aad (silicium) */
                s->brc   = 0x0000;
                s->rsa   = 0x0000;
                s->rea   = 0x0000;
            } else { /* reg_mode == 2 : hybrid → registres opérationnels du bin,
                      * MAIS champs critiques forcés aux valeurs datasheet pures
                      * (cf audit anti-drift) pour un reset propre. */
                s->ifr   = 0x0000;   /* pas d'IRQ pending au reset */
                s->ar[0] = r[0x10];  /* FIX 2026-05-31 : AR0 = snapshot silicium
                                      * (0x5aad), pas le hardcode 0xFF75 : prouvé
                                      * read-before-write insn=1 → load-bearing. */
                s->brc   = 0x0000;
                s->rsa   = 0x0000;
                s->rea   = 0x0000;
            }
            /* XPC jamais overridé (registre de page runtime ; le fetch vecteur
             * reset à IPTR*0x80 doit venir de la page 0 → XPC=0 conservé). */
            C54_LOG("reset: dsp-registers %s applied (SP=0x%04x PMST=0x%04x "
                    "ST0=0x%04x ST1=0x%04x IMR=0x%04x IFR=0x%04x AR0=0x%04x "
                    "BRC=0x%04x RSA=0x%04x REA=0x%04x)",
                    reg_mode == 2 ? "HYBRID" : "BIN",
                    s->sp, s->pmst, s->st0, s->st1, s->imr, s->ifr, s->ar[0],
                    s->brc, s->rsa, s->rea);
        } else {
            C54_LOG("reset: registres = hardcode C (mode c54x ou pas de .bin) "
                    "SP=0x%04x PMST=0x%04x IMR=0x%04x IFR=0x%04x AR0=0x%04x",
                    s->sp, s->pmst, s->imr, s->ifr, s->ar[0]);
        }
    }
    s->timer_psc = 0;
    /* SPRU131G §8.8: at reset TIM=PRD=FFFFh and "TSS is cleared and the timer
     * immediately starts timing". The legacy inert model (no counting wired)
     * parked TSS=1; keep that unless the counting timer is enabled. */
    s->data[TCR_ADDR] = s->timer_enabled ? 0 : TCR_TSS;
    s->data[TIM_ADDR] = 0xFFFF;   /* TIM = max at reset */
    s->data[PRD_ADDR] = 0xFFFF;   /* PRD = max at reset */
    s->rpt_active = false;
    s->rptb_active = false; { static int _re=0; if (_re<50) { C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp); _re++; } }
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

    /* Boot ROM MVPD: copy PROM0 code to DARAM overlay.
     * On real Calypso, the internal boot ROM copies PROM0[0x7080..0x9FFF]
     * to DARAM data[0x0080..0x27FF] before jumping to user code.
     * This populates the DARAM code overlay that the DSP executes with OVLY=1.
     *
     * On real silicon, DARAM and API RAM share one physical memory in the
     * range 0x0800-0x27FF (DSP-words). Mirror the copy into api_ram so the
     * ARM-side view matches the DSP-side view from boot — without this
     * mirror, every ARM read into the overlay zone returns 0 while the
     * DSP executes the copied code, which silently splits the two views.
     *
     * NOT under no_xpc (C542-class / 5110 MAD1): this is CALYPSO mask-ROM
     * behavior — the MAD1 DARAM starts clean and is populated by the MCU's
     * real HPI upload + the DSP's own stores. Planting prog[0x7080+] here
     * left 700+ GHOST words in data[0x800-0x27FF] that the (api_ram) fetch
     * can never see — pure analysis poison. */
    if (!s->no_xpc) {
        for (int i = 0; i < 0x2780; i++) {
            uint16_t addr = 0x0080 + i;
            uint16_t val = s->prog[0x7080 + i];
            s->data[addr] = val;
            if (s->api_ram &&
                addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
                s->api_ram[addr - C54X_API_BASE] = val;
        }
    }

    /* 2026-05-28 v3 : runtime conditional override (option A).
     *
     * v1 (static override prog[0xFF80] = B 0x7120) intercepted both the
     * silicon reset AND every sequential firmware walk into address 0xff80.
     * The firmware ROM contains a normal subroutine that includes a RET at
     * 0xff79 popping 0xff7a + POPM/STM sequence walking through 0xff80
     * during routine epilogues. The static override turned each such walk
     * into a soft-reset → SP=0x5AC8 → state derailed → DSP stuck in
     * boot-reset cycle, FB never stabilising past the first 4s.
     *
     * v3 fix : leave prog[0xFF80..0xFF83] as legitimate PROM1 mirror
     * (= 0x56d0, 0x9631, 0xf820, 0xff89 from the dump). Apply the boot-init
     * redirect at runtime ONLY when SP still holds the silicon-reset value
     * 0x1100 (i.e., the very first reach of 0xff80 after silicon-reset,
     * before any STM #imm,SP has run). See c54x_run main loop for the
     * runtime check. */

    /* Boot ROM stubs at 0x0000-0x007F.
     * Discriminant test 2026-04-26 confirmed FRET stub did NOT block the
     * firmware path to 0x0810 (reverting to NOPs gave identical PC HIST
     * + same IMR change=0). FRET stub kept: prevents stack runaway when
     * CALAA targets the stub area, with no downside.
     *
     * Fallback per slot (2026-05-29 v3 — RET@0x0000 équilibre le near-CALA) :
     *   - 0x0000/0x0001: RET (0xFC00) — pop ret_pc, retour ÉQUILIBRÉ.
     *   - rest (0x02..0x7F): FRET (0xF4E4) — retour-from-far (far-call→stub).
     *
     * Pourquoi RET@0x0000 (et pas IDLE/LDMM) : le firmware fait des near-CALA
     * `CALA → 0x0000` avec A=0 (chemin handler-nul/défaut ; ex. LDU@0xfa7e lit
     * un ptr de table = 0 → CALA A=0). Un near-CALA push 1 mot (ret_pc) ; RET
     * pop 1 mot → ÉQUILIBRÉ → retour propre au caller → le boot continue.
     *   - IDLE (v2) ne retournait pas → halt+slide 0x0000→0x0002 → FRET-loop
     *     → fuite SP (0x1106→0x4d75).
     *   - LDMM SP,B+RET (old/good 2026-05-28) retournait MAIS posait SP=B :
     *     si B=0 → SP=0 → pop garbage ("wake-on-IRQ" loop de v1).
     * Le rest reste FRET : les far-calls (FCALA, push 2) qui tombent dans la
     * zone sont équilibrés par FRET (pop 2). Le firmware idle à son vrai
     * point (IDLE de la table TDMA slots, cf doc/DSP_ROM_MAP.md). */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xF4E4;  /* FRET — retour-from-far (far-call-into-stub) */
    s->prog[0x0000] = 0xFC00;  /* RET — pop ret_pc, équilibre le near-CALA→0 */
    s->prog[0x0001] = 0xFC00;  /* RET — idem */

    /* Reset vector: IPTR * 0x80 */
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = iptr * 0x80;  /* 0xFF80 for default PMST */

    C54_LOG("Reset: PC=0x%04x PMST=0x%04x SP=0x%04x prog[PC]=0x%04x",
            s->pc, s->pmst, s->sp, s->prog[s->pc]);

    /* Build identity dump (2026-05-25) — permet attribution causale dans
     * les rapports/bundles. Cf review Claude web : "Le rapport ne peut pas
     * s'attribuer à un état de code". On dump ici les valeurs reset
     * silicon-aligned utilisées par CE binaire — si elles changent, le
     * comportement firmware change. Lecture de qemu.log = identité du build. */
    C54_LOG("BUILD-IDENT silicon-reset: SP=0x%04x BK=0x%04x IMR=0x%04x "
            "ST0=0x%04x ST1=0x%04x PMST=0x%04x",
            s->sp, s->bk, s->imr, s->st0, s->st1, s->pmst);
    C54_LOG("BUILD-IDENT silicon-AR: AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
            "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x",
            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
            s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
    /* Decoder fix flags : si ces fixes sont retirés du source, ce log
     * n'apparaîtra plus ou aura un format différent — preuve immédiate
     * de quel binaire produit le run. */
    C54_LOG("BUILD-IDENT decoder-fixes: F1xx-FIRS-catch=REMOVED "
            "L3609-src-dst=FIXED F-AUDIT-v5=max-min-cmpl-rnd-roltc-fixed "
            "F2xx-ALU-block=ADDED-2026-05-25-night "
            "F3xx-INTR-mis-REMOVED-ADD-SUB-LD-ADDED "
            "2026-05-25");

}

void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit)
{
    if (vec < 0 || vec >= 32) return;
    if (imr_bit < 0 || imr_bit >= 16) return;
    s->ifr |= (1 << imr_bit);

    /* EXPÉRIENCE WIRE585F RETIRÉE 2026-05-30 : forcer data[0x585f] bit7 au frame-IRQ
     * a prouvé (mem=0x0180 TC=1) que le mécanisme est sain MAIS que 0x585f n'est
     * qu'UNE porte d'une chaîne (→ pose 0x3fd3 puis reboucle) — pas le verrou.
     * Whack-a-mole démontré, pas supposé. cf [[feedback_debug_gate_heisenbug]]. */

    bool unmasked = (s->imr & (1 << imr_bit)) != 0;

    /* Per SPRU131: an (IMR-)unmasked interrupt EXITS IDLE, but the CPU only
     * SERVICES it (vectors) when INTM=0. With INTM=1 the CPU resumes at the
     * instruction after IDLE and the IFR bit stays PENDING (it is serviced once
     * software clears INTM). 2026-06-09 FIX: the old code vectored on `unmasked`
     * alone, IGNORING INTM — so a frame/timer IRQ would vector straight INTO
     * loader1's INTM=1 idle/park (f6a), where real silicon only wakes-and-resumes.
     * That spurious servicing corrupted the stack at the finalize (the ret@0xf73
     * over-pop → 0x940d retd→0xf074 → AR5=0 → IMR=0). Now the idle-wake honours
     * INTM exactly like the non-idle path below. */
    if (s->idle) {
        s->idle = false;
        bool service = unmasked && !(s->st1 & ST1_INTM);
        if (service) {
            /* Service the interrupt: branch to vector */
            s->ifr &= ~(1 << imr_bit);
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 1));
            g_sp_ledger.irq_words_pushed++;
            if (!s->no_xpc) {                      /* C542-class: 1-word frame (PC only) */
                s->sp--;
                data_write(s, s->sp, s->xpc);      /* save XPC inconditionnel */
                g_sp_ledger.irq_words_pushed++;
                g_sp_ledger.net_words += 1;
            }
            g_sp_ledger.irq_entries++;
            g_sp_ledger.net_words += 1;
            s->st1 |= ST1_INTM;
            { static int el = -1; static int en = 0;
              if (el < 0) el = cdbg_env("IRQPAIR") ? 1 : 0;
              if (el && en < 24) { en++;
                  fprintf(stderr, "[c54x] IRQPAIR entry(idle) vec=%d pushed ra=0x%04x sp=0x%04x insn=%u\n",
                          vec, (uint16_t)(s->pc + 1), s->sp, s->insn_count); } }
            /* DISP-ENTRY : capture contexte préempté (DP foreground inchangé) */
            g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
            g_last_intr_fg_pc = (uint16_t)(s->pc + 1); g_last_intr_fg_dp = dp(s);
            s->xpc = 0;                            /* fetch vecteur sur page 0 */
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            s->pc = (iptr * 0x80) + vec * 4;
        } else {
            /* Woken but NOT serviced (INTM=1, or IMR-masked): resume at the
             * instruction after IDLE; leave the IFR bit pending. */
            s->pc++;
        }
    } else if (!(s->st1 & ST1_INTM) && unmasked && s->delay_slots == 0) {
        /* Normal (non-IDLE) interrupt servicing.
         * Garde delay_slots==0 (fix 2026-05-30) : faithful C54x — une IT
         * n'est PAS reconnue entre une branche différée (RETD/RCD/CALLD/BD)
         * et ses 2 delay-slots. Vectoriser mid-delay laisserait delay_slots
         * armé → au retour (RETE) le commit delayed_pc se ferait dans le
         * mauvais contexte → over-pop SP → DP garbage → self-CALA 0x70c3.
         * IFR reste set (non clearé) → l'IT est servie au prochain appel,
         * delay_slots étant retombé à 0 (max ~2 insns plus tard). */
        s->ifr &= ~(1 << imr_bit);
        s->sp--;
        data_write(s, s->sp, (uint16_t)s->pc);
        g_sp_ledger.irq_words_pushed++;
        if (!s->no_xpc) {                          /* C542-class: 1-word frame (PC only) */
            s->sp--;
            data_write(s, s->sp, s->xpc);          /* save XPC inconditionnel */
            g_sp_ledger.irq_words_pushed++;
            g_sp_ledger.net_words += 1;
        }
        g_sp_ledger.irq_entries++;
        g_sp_ledger.net_words += 1;
        s->st1 |= ST1_INTM;
        { static int el2 = -1; static int en2 = 0;
          if (el2 < 0) el2 = cdbg_env("IRQPAIR") ? 1 : 0;
          if (el2 && en2 < 24) { en2++;
              fprintf(stderr, "[c54x] IRQPAIR entry(run) vec=%d pushed ra=0x%04x sp=0x%04x insn=%u\n",
                      vec, (uint16_t)s->pc, s->sp, s->insn_count); } }
        /* DISP-ENTRY : capture contexte préempté (DP foreground inchangé) */
        g_last_intr_insn = s->insn_count; g_last_intr_vec = vec;
        g_last_intr_fg_pc = (uint16_t)s->pc; g_last_intr_fg_dp = dp(s);
        s->xpc = 0;                                /* fetch vecteur sur page 0 */
        uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        s->pc = (iptr * 0x80) + vec * 4;
        /* INT3-CYCLE-TRACE : hook cycle start for vec=19 (INT3 FRAME) */
        if (vec == 19) {
            int3_cycle_start(s, s->pc);
        }
    }

    /* Log interrupts: first 20 + every 100th, so we can count them.
     * PMST/IPTR included so we can correlate which vector base the IRQ
     * lands at — INT3 at IPTR=0x1ff (vec=0xffcc) hits a garbage ROM stub,
     * INT3 at IPTR=0x140 (vec=0xa04c) hits the firmware's real handler. */
    static uint64_t int_log_count;
    int_log_count++;
    if (int_log_count <= 20 || (int_log_count % 100) == 0) {
        uint16_t iptr_now = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        C54_LOG("IRQ #%llu vec=%d bit=%d: INTM=%d IMR=0x%04x IFR=0x%04x "
                "idle=%d PC=0x%04x PMST=0x%04x IPTR=0x%03x",
                (unsigned long long)int_log_count,
                vec, imr_bit, !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                s->idle, s->pc, s->pmst, iptr_now);
    }
}

void c54x_wake(C54xState *s)
{
    s->idle = false;
}

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 2048) n = 2048;
    memcpy(s->bsp_buf, samples, n * sizeof(uint16_t));
    s->bsp_len = n;
    s->bsp_pos = 0;

    /* Confirm what the PORTR PA=0x0034 serving path will hand the DSP,
     * and also flag if the DSP consumed less than half of the previous
     * batch before a new one arrived (would indicate correlator starvation
     * or DSP never reading via PORTR at all). */
    static uint64_t load_count;
    load_count++;
    if (load_count <= 10 || (load_count % 1000) == 0) {
        C54_LOG("BSP LOAD #%llu n=%d: %04x %04x %04x %04x %04x %04x %04x %04x",
                (unsigned long long)load_count, n,
                n > 0 ? samples[0] : 0, n > 1 ? samples[1] : 0,
                n > 2 ? samples[2] : 0, n > 3 ? samples[3] : 0,
                n > 4 ? samples[4] : 0, n > 5 ? samples[5] : 0,
                n > 6 ? samples[6] : 0, n > 7 ? samples[7] : 0);
    }
}
