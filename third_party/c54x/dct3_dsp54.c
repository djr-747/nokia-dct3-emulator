/* dct3_dsp54 — wraps the lifted C54x core + Nokia HPI/port bridge. See header. */
#include "dct3_dsp54.h"
#include "calypso_c54x.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>   /* DSP54_PORTPC diagnostic */

/* Bridge hooks live at file scope in calypso_c54x.c (the core is single-instance
 * here, which is exactly the DCT3 case — one DSP per phone). */
extern uint16_t (*nokia_port_read_hook)(uint16_t pa, uint16_t pc);
extern void     (*nokia_port_write_hook)(uint16_t pa, uint16_t val, uint16_t pc);

#define DSP_RESET_ENTRY 0x0F00u   /* real reset target (vector @0xFF80 -> 0x0F00) */

struct Dsp54 {
    C54xState *s;
    uint16_t   api_ram[C54X_API_SIZE];          /* Calypso 0x0800 window backing store */
    uint16_t (*port_read)(void *, uint16_t);
    void     (*port_write)(void *, uint16_t, uint16_t);
    void      *opaque;
};

static Dsp54 *g_active;   /* single active instance for the global bridge hooks */

/* === Static-upload regression guard (2026-06-08) =========================================
 * The FAITHFUL 5110/MAD2 DSP boot is the MCU firmware's REAL staged block-upload: the MCU
 * pages each DSP code block through the HPI window + a 5-word UPLOADHEADER and the DSP loader
 * scatters it to its dest (; memory 5110-dsp-staged-upload-pivot).
 * The DSP54_OVERLAY / DSP54_BLOCKS / DSP54_UPLOAD / DSP54_LOADERGO knobs are NON-FAITHFUL
 * static shortcuts: they pre-place code or poke the handshake instead of letting the MCU
 * upload. They provably TOP OUT (a one-shot static image can't model the multi-stage DARAM
 * scatter — 0xD00 is time-multiplexed) and silently corrupt measurements. They are RETAINED
 * only for deliberate A/B reference and are INERT unless DSP54_ALLOW_STATIC=1, so no future
 * session can regress onto the static path by accident. Returns 1 if the static knob may act. */
int dsp54_faithful(const char *name) {
    const char *e = name ? getenv(name) : 0;
    if (e && *e) return atoi(e) != 0;       // explicit value wins (DSP54_X=0 opts a knob out)
    static int cosim = -1;                   // else default ON under the cosim master switch
    if (cosim < 0) cosim = getenv("DSP54_COSIM") ? 1 : 0;
    return cosim;
}

int dsp54_static_allowed(const char *knob) {
    static int allow = -1;
    if (allow < 0) allow = getenv("DSP54_ALLOW_STATIC") ? 1 : 0;
    if (!allow) {
        fprintf(stderr,
            "[dsp54] *** %s IGNORED — the static-upload path is RETIRED (non-faithful). The\n"
            "[dsp54]     faithful boot is the MCU's REAL block upload. Set DSP54_ALLOW_STATIC=1\n"
            "[dsp54]     ONLY for a deliberate A/B reference run. See memory\n"
            "[dsp54] 5110-dsp-staged-upload-pivot +.\n", knob);
        return 0;
    }
    fprintf(stderr, "[dsp54] WARNING: %s active under DSP54_ALLOW_STATIC — NON-FAITHFUL A/B path\n", knob);
    return 1;
}

/* DSP54_PORTPC=1 (diagnostic): log every host-port access with the DSP PC issuing it, so
 * the co-sim can pin which PROM site polls which port (the codec/serial boot wait). */
static int g_portpc = -1;
static uint16_t bridge_port_read(uint16_t pa, uint16_t pc) {
    if (g_portpc < 0) g_portpc = getenv("DSP54_PORTPC") ? 1 : 0;
    uint16_t v = (g_active && g_active->port_read)
               ? g_active->port_read(g_active->opaque, pa) : 0;
    if (g_portpc) fprintf(stderr, "[dsp54-port] RD pa=0x%02X -> 0x%04X @dsp_pc=0x%04X\n", pa, v, pc);
    return v;
}
static void bridge_port_write(uint16_t pa, uint16_t val, uint16_t pc) {
    if (g_portpc < 0) g_portpc = getenv("DSP54_PORTPC") ? 1 : 0;
    if (g_portpc) fprintf(stderr, "[dsp54-port] WR pa=0x%02X = 0x%04X @dsp_pc=0x%04X\n", pa, val, pc);
    if (g_active && g_active->port_write)
        g_active->port_write(g_active->opaque, pa, val);
}

Dsp54 *dsp54_create(const uint8_t *image, size_t nbytes) {
    Dsp54 *d = calloc(1, sizeof *d);
    if (!d) return NULL;
    d->s = c54x_init();
    /* DSP54_NOXPC (faithful default ON under cosim): the early-MAD2 LEAD is C542-class — single 64K
     * program page, NO XPC. Interrupt frames are the 16-bit PC alone and RETE pops only PC, so
     * the firmware's task-level `call ...; rete` (return-and-enable-ints, e.g. the sleep/wake
     * sequencer tail 0x90BE) stays stack-balanced. The lift's C548/549-style 2-word {PC,XPC}
     * frames desynced it by one word -> "return" into the empty page 1 -> zero-walk -> MMR/IMR
     * spray (the post-vec25 strand). */
    if (dsp54_faithful("DSP54_NOXPC")) {
        c54x_set_no_xpc(d->s, 1);
        /* c54x_init() already ran its (Calypso-only) boot-ROM prog[0x7080+]->DARAM copy
         * before this flag could gate it — undo: MAD2 DARAM powers up clean and is
         * populated only by the MCU upload + the DSP's own stores. Kills the ghost
         * data[] words the api_ram fetch can never see. */
        for (uint32_t a = 0x80; a < 0x2800; a++) d->s->data[a] = 0;
    }
    size_t nw = nbytes / 2;
    if (nw > C54X_PROG_SIZE) nw = C54X_PROG_SIZE;
    for (size_t i = 0; i < nw; i++)
        d->s->prog[i] = (uint16_t)((image[2*i] << 8) | image[2*i + 1]);
    /* DSP54_UPLOAD=1: populate the OVLY DARAM. PMST=0xFFA8 has OVLY=1, so DSP
     * addresses [0x80,0x2800) execute/read from data[] (the on-chip DARAM), NOT prog[].
     * The uploadable overlays (block dests 0x126..0x700) live there. The flat image
     * captured them in the low (OVLY-aliased) words, so mirror prog[0x80..0x2800) into
     * data[] — i.e. model the MCU having uploaded the code blocks to their DSP mem map.
     * Without this the DSP runs with empty overlay DARAM (the FIR-grind half-boot). */
    if (getenv("DSP54_UPLOAD") && dsp54_static_allowed("DSP54_UPLOAD")) {
        for (uint32_t k = 0x80; k < 0x2800 && k < nw; k++)
            d->s->data[k] = d->s->prog[k];
    }
    /* DSP54_OVERLAY is loaded AFTER c54x_reset() — see below. (c54x_reset writes its clean
     * register/DARAM state and clobbers data[0x80,0x2800), so a pre-reset overlay load is
     * wiped. The load was moved past reset on 2026-06-08.) */
    /* Load the on-chip DATA ROM (DROM) into data space. With PMST.DROM=1 the C54x maps its
     * on-chip ROM into data space (the recovered dump is at data 0xB000-0xEFFF); the resident
     * boot copies coefficient/dispatch tables FROM the DROM INTO DARAM. The core never loaded
     * it (only prog[]), so data[0xB000+] was zero and those copies produced garbage. Format:
     * "   addr:   value" hex per line. DSP54_DROM overrides the path; DSP54_NODROM disables. */
    if (!getenv("DSP54_NODROM")) {
        const char *dp = getenv("DSP54_DROM");
        if (!dp || !*dp) dp = "re/dsp-5110/raw/dsp_drom.txt";
        FILE *df = fopen(dp, "r");
        if (df) {
            char line[128]; unsigned long loaded = 0;
            while (fgets(line, sizeof line, df)) {
                unsigned a, v;
                if (sscanf(line, " %x : %x", &a, &v) == 2 && a < C54X_DATA_SIZE) {
                    d->s->data[a] = (uint16_t)v; loaded++;
                }
            }
            fclose(df);
            if (getenv("DSP54_LOG")) fprintf(stderr, "[dsp54] loaded DROM %s (%lu words)\n", dp, loaded);
        } else if (getenv("DSP54_LOG")) {
            fprintf(stderr, "[dsp54] DROM '%s' not found\n", dp);
        }
    }
    c54x_set_api_ram(d->s, d->api_ram);
    d->s->win_dirty = 1;                       /* PERF: prime so the first tick always evaluates */
    c54x_reset(d->s);                          /* running=1, PMST=0xFFA8, sane defaults */
    /* DSP54_OVERLAY=<path> (F5 experiment, default-OFF): load a reconstructed OVLY DARAM image
     * into data[0x80,0x2800) — AFTER reset, which would otherwise wipe it. Models "the DSP
     * mask-ROM bootstrap already placed the uploaded code blocks at their run addresses"
     * (loader1@0xF00, loader2@0xD80, branch table@0x23C4, scheduler@0x926). Built by
     * tools/dsp/build_daram_overlay.py from re/dsp-5110/raw/full.s54. 'addr: value' word hex. */
    {
        const char *op = getenv("DSP54_OVERLAY");
        if (op && *op && dsp54_static_allowed("DSP54_OVERLAY")) {
            FILE *of = fopen(op, "r");
            if (of) {
                char line[128]; unsigned long loaded = 0;
                while (fgets(line, sizeof line, of)) {
                    unsigned a, v;
                    if (sscanf(line, " %x : %x", &a, &v) == 2 && a >= 0x80 && a < 0x2800) {
                        d->s->data[a] = (uint16_t)v;        // FETCH source (prog_fetch under OVLY)
                        // DSP DATA accesses to [0x800,0x2800) route to api_ram[a-0x800] (the core's
                        // shared-window mapping), NOT data[a] — so the overlay's data/tables (e.g. the
                        // branch-table source loader1 copies from 0xD00) must ALSO live there or the
                        // DSP reads empty api_ram. Overlay min addr is 0x926 (> mailbox 0x8E5) so this
                        // never clobbers the MCU<->DSP mailbox/ring region.
                        if (a >= 0x800 && (a - 0x800) < C54X_API_SIZE)
                            d->api_ram[a - 0x800] = (uint16_t)v;
                        loaded++;
                    }
                }
                fclose(of);
                if (getenv("DSP54_LOG"))
                    fprintf(stderr, "[dsp54] loaded OVERLAY %s (%lu words, post-reset)\n", op, loaded);
            } else if (getenv("DSP54_LOG")) {
                fprintf(stderr, "[dsp54] OVERLAY '%s' not found\n", op);
            }
        }
    }
    /* DSP54_RESET=<hex> overrides the reset entry (default 0x0F00 = the dump's relocated
     * reset target; 0x3000 = the resident PROM cold-init entry — used to bring up the
     * "boot to await-upload" cold model). */
    uint16_t rpc = DSP_RESET_ENTRY;
    const char *rp = getenv("DSP54_RESET");
    if (rp && *rp) rpc = (uint16_t)strtol(rp, 0, 0);
    c54x_set_initial_pc(d->s, rpc);
    g_active = d;
    nokia_port_read_hook  = bridge_port_read;
    nokia_port_write_hook = bridge_port_write;
    return d;
}

void dsp54_destroy(Dsp54 *d) {
    if (!d) return;
    if (g_active == d) { g_active = NULL; nokia_port_read_hook = NULL; nokia_port_write_hook = NULL; }
    free(d->s);
    free(d);
}

int dsp54_step(Dsp54 *d, int n_insns) { return c54x_run(d->s, n_insns); }

/* PERF: read+clear the cosim window-dirty flag (see header). Conservative — returns 1 (never
 * skip) when there is no core, so the gate degrades to "always eval". */
int dsp54_window_dirty_clear(Dsp54 *d) {
    if (!d || !d->s) return 1;
    int v = d->s->win_dirty;
    d->s->win_dirty = 0;
    return v;
}

/* Enable the C54x on-chip timer (TIM/PRD/TCR counting + TINT vec 19, SPRU131G §8.8). The 5110
 * run-mode firmware enables IMR bit3 and its IVT routes vec 19 (0xFFCC) -> the DARAM RTOS-tick
 * dispatcher 0x241A: the on-chip timer IS the steady-state heartbeat that wakes `idle 1`. */
void dsp54_set_timer(Dsp54 *d, int on) { if (d && d->s) c54x_set_timer_enabled(d->s, on); }

/* Register a per-instruction PC histogram (see c54x_set_pc_histogram) — reliable at any RATIO. */
void dsp54_set_pc_histogram(Dsp54 *d, uint32_t *h) { (void)d; c54x_set_pc_histogram(h); }

/* Seed the OVLY DARAM (api_ram) with the resident image's content for DSP addresses [lo,hi).
 * The resident dispatch/overlay code (e.g. prog[0x2000..0x2800]) lives in the image but loader2's
 * bootstrap rptz-clears DARAM 0x1000..0x2FFF and nothing copies it back — so an OVLY=1 program-fetch
 * of those addresses hits an empty cell. This models the chip's DARAM being (re)loaded from ROM.
 * Returns the number of words copied. */
uint32_t dsp54_seed_daram_from_prog(Dsp54 *d, uint16_t lo, uint16_t hi) {
    if (!d) return 0;
    uint32_t n = 0;
    for (uint32_t a = lo; a < hi; a++) {
        if (a < C54X_API_BASE || a >= C54X_API_BASE + C54X_API_SIZE) continue;
        d->api_ram[a - C54X_API_BASE] = d->s->prog[a];
        n++;
    }
    return n;
}

/* DSP->host HINT doorbell: cumulative count of BSCR(0x29) bit3 rising edges (loader2's
 * "run-mode ready" pulse). The host side latches IRQ4 when this advances. */
extern unsigned long long g_c54x_bscr_hint_edges;
unsigned long long dsp54_hint_edges(const Dsp54 *d) { (void)d; return g_c54x_bscr_hint_edges; }

/* Conditional halt on a DSP PC (freeze BEFORE the `after`-th hit) — see c54x_set_halt. */
void dsp54_set_halt(Dsp54 *d, uint16_t pc, uint32_t after) { (void)d; c54x_set_halt(pc, after); }
void dsp54_set_halt_sp(Dsp54 *d, uint16_t floor) { (void)d; c54x_set_halt_sp(floor); }
int  dsp54_halted(const Dsp54 *d) { (void)d; return c54x_halted(); }

/* Dump the full LIVE DSP memory map (BE words) + a register snapshot for offline disassembly:
 *   <prefix>.prog  256K words program memory (prog[]) — resident PROM, vectors, 0x3000 superloop
 *   <prefix>.api     8K words OVLY/HPI DARAM (api_ram, [0x800,0x2800)) — the REAL fetch source there
 *   <prefix>.data   64K words data memory (data[])
 *   <prefix>.regs   text register snapshot (pc/sp/st/pmst/ar/acc)
 * Disassemble with tools/dsp/disasm_dump.py. Returns 0 on success. */
int dsp54_dump_full(const Dsp54 *d, const char *prefix) {
    const C54xState *s = d->s;
    char p[1024]; FILE *f;
    snprintf(p, sizeof p, "%s.prog", prefix);
    if (!(f = fopen(p, "wb"))) return -1;
    for (uint32_t i = 0; i < C54X_PROG_SIZE; i++) {
        uint8_t b[2] = { (uint8_t)(s->prog[i] >> 8), (uint8_t)s->prog[i] }; fwrite(b, 1, 2, f); }
    fclose(f);
    snprintf(p, sizeof p, "%s.api", prefix);
    if ((f = fopen(p, "wb"))) {
        for (uint32_t i = 0; i < C54X_API_SIZE; i++) {
            uint16_t w = d->api_ram[i];
            uint8_t b[2] = { (uint8_t)(w >> 8), (uint8_t)w }; fwrite(b, 1, 2, f); }
        fclose(f); }
    snprintf(p, sizeof p, "%s.data", prefix);
    if ((f = fopen(p, "wb"))) {
        for (uint32_t i = 0; i < C54X_DATA_SIZE; i++) {
            uint8_t b[2] = { (uint8_t)(s->data[i] >> 8), (uint8_t)s->data[i] }; fwrite(b, 1, 2, f); }
        fclose(f); }
    snprintf(p, sizeof p, "%s.regs", prefix);
    if ((f = fopen(p, "w"))) {
        fprintf(f, "pc=0x%04X xpc=%u sp=0x%04X st0=0x%04X st1=0x%04X pmst=0x%04X imr=0x%04X ifr=0x%04X idle=%d\n",
                (uint16_t)s->pc, s->xpc & 3, s->sp, s->st0, s->st1, s->pmst, s->imr, s->ifr, s->idle ? 1 : 0);
        for (int i = 0; i < 8; i++) fprintf(f, "ar%d=0x%04X ", i, s->ar[i]);
        fprintf(f, "\na=0x%010llX b=0x%010llX\n",
                (unsigned long long)(s->a & 0xFFFFFFFFFFLL), (unsigned long long)(s->b & 0xFFFFFFFFFFLL));
        fclose(f); }
    /* PC ring (path into the halt PC) — oldest->newest, consecutive identical PCs collapsed with a
     * run count so the control-flow transitions (the jump that lands in a wrong region) stand out. */
    extern uint16_t g_pcring_pc[]; extern uint8_t g_pcring_xpc[]; extern uint32_t g_pcring_idx;
    extern int c54x_pcring_size(void);
    int N = c54x_pcring_size();
    uint32_t total = g_pcring_idx, count = total < (uint32_t)N ? total : (uint32_t)N;
    snprintf(p, sizeof p, "%s.pcring", prefix);
    if (count && (f = fopen(p, "w"))) {
        fprintf(f, "# last %u executed PCs (oldest->newest), consecutive repeats collapsed\n", count);
        uint32_t start = total - count;
        int run = 0; uint16_t lpc = 0; uint8_t lx = 0;
        for (uint32_t k = 0; k < count; k++) {
            uint32_t i = (start + k) & ((uint32_t)N - 1);
            uint16_t pc = g_pcring_pc[i]; uint8_t x = g_pcring_xpc[i];
            if (run && pc == lpc && x == lx) { run++; continue; }
            if (run) fprintf(f, "  xpc=%u pc=0x%04X  x%d\n", lx, lpc, run);
            lpc = pc; lx = x; run = 1;
        }
        if (run) fprintf(f, "  xpc=%u pc=0x%04X  x%d\n", lx, lpc, run);
        fclose(f); }
    /* SP-event ring — the exact push/pop instructions (PC/op/delta) behind a stack imbalance. */
    snprintf(p, sizeof p, "%s.spring", prefix);
    if ((f = fopen(p, "w"))) { c54x_dump_sp_ring(f); fclose(f); }
    return 0;
}

/* Load a code block (flat BE words) into DSP program memory at `dest` (word addr).
 * Models the MCU uploading a DSP code block to its mem map (block descriptor dest). */
void dsp54_load_block(Dsp54 *d, uint16_t dest, const uint8_t *bytes, size_t nbytes) {
    size_t nw = nbytes / 2;
    for (size_t i = 0; i < nw && (dest + i) < C54X_PROG_SIZE; i++)
        d->s->prog[dest + i] = (uint16_t)((bytes[2*i] << 8) | bytes[2*i + 1]);
}

/* HPI window: MCU 0x10000 + 2k <-> DSP shared word k.
 * CRITICAL: the DSP accesses the shared mailbox/ring
 * region at DSP data 0x800+k (the C54x API RAM — MDIRCV tail at DSP 0x8E4 = MCU
 * 0x101C8), and the core routes [0x800,0x2800) to api_ram[] (calypso_c54x.c:2207/3168).
 * So MCU 0x10000+2k <-> api_ram[k], NOT data[k]. The old data[k] mapping left the
 * MCU and DSP looking at DIFFERENT buffers for the mailbox/ring (the upload worked
 * only because it lands below 0x800 = data[], OVLY-shared). DSP54_HPIDATA=1 restores
 * the old data[] mapping for A/B. */
// ASYMMETRIC surgical api_ram bridge (DSP54_HPIAPI=1): the DSP accesses the shared mailbox/ring
// at DSP 0x800+k = api_ram[k]. For the shared sub-region (words
// 0x000..0x0E5 = MDISND ring + upload mailbox + MDIRCV ring + version slots) the MCU *reads* from
// api_ram[] so it sees the real DSP version/ring — but MCU *writes* stay on data[] so they DON'T
// clobber the DSP's api_ram working copy (a full read+write bridge stops the DSP idling). Outside
// the sub-region everything stays on data[] (DSP-private scratch + the code-upload region 0x100+).
// DSP54_APILO/APIHI tune the window; DSP54_HPIAPIW=1 also bridges writes (A/B, breaks idle).
static int __attribute__((unused)) hpi_api_region(uint32_t k) {  // RETIRED by the single-store window
    static int init = 0, use_api = 0; static uint32_t lo = 0x000, hi = 0x0E5;
    if (!init) { init = 1; use_api = dsp54_faithful("DSP54_HPIAPI");
        const char *l = getenv("DSP54_APILO"); if (l && *l) lo = (uint32_t)strtoul(l,0,0);
        const char *h = getenv("DSP54_APIHI"); if (h && *h) hi = (uint32_t)strtoul(h,0,0); }
    return use_api && k >= lo && k <= hi && k < C54X_API_SIZE;
}
// The HPI window IS the DSP's dual-port DARAM (SPRU131G §8): MCU window word k = DSP 0x800+k =
// api_ram[k] — the SAME single-store cell the DSP fetches/reads/writes (see ovly_cell). So the
// whole window resolves to api_ram unconditionally; the old ovly_unified/SHAREDWIN/HPIAPI gating
// is retired (it was compensating for the data[]/api_ram[] split that no longer exists).
uint16_t dsp54_hpi_read(Dsp54 *d, uint32_t mcu_addr) {
    uint32_t k = (mcu_addr - 0x10000u) >> 1;
    if (k < C54X_API_SIZE) return d->api_ram[k];   // single store (= DSP 0x800+k)
    return (k < C54X_DATA_SIZE) ? d->s->data[k] : 0;             // outside DARAM window
}
void dsp54_hpi_write(Dsp54 *d, uint32_t mcu_addr, uint16_t val) {
    uint32_t k = (mcu_addr - 0x10000u) >> 1;
    /* DSP54_CELLWATCH=0xLO:0xHI (DSP data addrs) — MCU-side window writes into the range. */
    { static int cw = -1; static unsigned lo, hi, n = 0;
      if (cw < 0) { const char *e = getenv("DSP54_CELLWATCH");
          if (e && *e && sscanf(e, "%x:%x", &lo, &hi) == 2) cw = 1; else cw = 0; }
      if (cw) { uint32_t da = 0x800u + k;
          if (da >= lo && da <= hi && n < 200000) { n++;
              fprintf(stderr, "[dsp54] CELLWATCH MCU->win [0x%04X] <- 0x%04X (mcu 0x%05X)\n",
                      da, val, mcu_addr); } } }
    if (k < C54X_API_SIZE) {
        d->api_ram[k] = val;
        if (d->s && k < 0x800u) d->s->win_dirty = 1;   // PERF: MCU wrote a signal-window cell
        return;
    }  // single store
    if (k < C54X_DATA_SIZE) d->s->data[k] = val;                 // outside DARAM window
}

void dsp54_set_pc(Dsp54 *d, uint16_t pc) { if (d && d->s) c54x_set_initial_pc(d->s, pc); }

/* Warm boot — faithful C54x reset of the CPU registers/status (PC=0xFF80, INTM=1, IFR=0, IPTR=0x1FF,
 * PMST=0xFFA8/OVLY=1, sane ST0/ST1/SP/AR) but PRESERVING DARAM [0x80,0x2800). Models the MCU's
 * post-upload reset cycle (0x20002 0->0->1, ~120M): TI C54x reset forces PC->0xFF80 and resets the
 * CPU state, but does NOT clear RAM — so the verified/uploaded code in DARAM survives and loader1's
 * 2nd pass runs on it. c54x_reset() does the register reset but ALSO copies prog[0x7080+] into DARAM
 * (the bootloader ROM->DARAM step), clobbering it — so snapshot+restore both OVLY stores (data[] and
 * api_ram) around it. Without this the warm boot only set PC and re-entered 0xFF80 carrying stale
 * mid-execution registers -> PMST/OVLY wrong -> DSP ran DROM garbage (0xDExx) / wedged at 0xFFC0. */
void dsp54_warm_reset(Dsp54 *d, uint16_t pc) {
    if (!d || !d->s) return;
    static uint16_t save_data[0x2800u - 0x80u];
    static uint16_t save_api[C54X_API_SIZE];
    for (uint32_t k = 0x80u; k < 0x2800u; k++) save_data[k - 0x80u] = d->s->data[k];
    for (uint32_t k = 0; k < C54X_API_SIZE; k++) save_api[k] = d->api_ram[k];
    c54x_reset(d->s);                                  /* clean regs: PMST=0xFFA8(OVLY=1), INTM=1, ... */
    for (uint32_t k = 0x80u; k < 0x2800u; k++) d->s->data[k] = save_data[k - 0x80u];
    for (uint32_t k = 0; k < C54X_API_SIZE; k++) d->api_ram[k] = save_api[k];
    c54x_set_api_ram(d->s, d->api_ram);                /* re-point (c54x_reset may have dropped it) */
    d->s->win_dirty = 1;                               /* PERF: re-eval the monitors on the first post-warm-reset tick */
    c54x_set_initial_pc(d->s, pc);
    c54x_mark_warm(1);                                 /* arm the SPUNDER run-mode stack-underflow probe */
    /* Clear the PC histogram at the loader1->run-mode transition so a PCHIST dump
     * reflects RUN-MODE activity only. The cold-boot loader1 CRC verification
     * (~524k hits at 0x8031, dsp_steps 37k-865k) otherwise dominates the cumulative
     * top-30 and masks the real run-mode hot loops (it misled a 2026-06-12 analysis).
     * Default ON; DSP54_PCHIST_KEEPBOOT=1 keeps the old cumulative-from-cold behaviour. */
    { static int keepboot = -1;
      if (keepboot < 0) keepboot = getenv("DSP54_PCHIST_KEEPBOOT") ? 1 : 0;
      if (!keepboot) c54x_reset_pc_histogram(); }
}

/* DSP54_UNIDARAM: collapse the OVLY window [0x800,0x2800) to a single store (api_ram) shared by
 * program-fetch, DSP data access, and the MCU HPI window — the faithful C54x OVLY + HPI dual-port
 * model. */
void dsp54_set_ovly_unified(Dsp54 *d, int on) { if (d && d->s) c54x_set_ovly_unified(d->s, on); }

void dsp54_host_interrupt(Dsp54 *d, int vec) {
    // The DSP's pmst IPTR is observed != reset 0x1FF at idle (0x0598 ⇒ vectors relocated to
    // 0x580, a region the filter processing scribbles with 0xa598 garbage). The host-command
    // ISR is the RESIDENT vector 0xFFC8→0x3598 (IPTR=0x1FF, in prog[], never scribbled), and
    // forcing IPTR=0x1FF before vectoring makes the real ISR run: it reads port1/port2,
    // dispatches the §7c command bits, and replies via port2 (verified: hostISR3598/35B9 run,
    // mailbox port RD/WR at 0x35B9/0x35BD/0x35FF). Default ON (host interrupt only — other
    // interrupts unaffected); DSP54_NORESIDENTVEC=1 disables for A/B.
    static int resvec = -1;
    if (resvec < 0) resvec = getenv("DSP54_NORESIDENTVEC") ? 0 : 1;
    if (resvec)
        d->s->pmst = (uint16_t)((d->s->pmst & 0x007F) | (0x1FF << 7));
    // The C54x core gates servicing on the IMR bit for this vector and sets the matching
    // IFR bit. For the maskable interrupts the mapping is vec = imr_bit + 16 (SPRU131;
    // INT0..INT5/TINT0/... -> vec 16..). Derive imr_bit so the right IMR mask is honoured
    // and the right IFR bit is raised (passing 0 would gate INT2 on IMR bit0 -> never taken).
    int imr_bit = (vec >= 16 && vec < 32) ? (vec - 16) : 0;
    c54x_interrupt_ex(d->s, vec, imr_bit);
}

void dsp54_set_port_io(Dsp54 *d,
                       uint16_t (*port_read)(void *, uint16_t),
                       void     (*port_write)(void *, uint16_t, uint16_t),
                       void *opaque) {
    d->port_read  = port_read;
    d->port_write = port_write;
    d->opaque     = opaque;
}

/* Dump live DSP memory (BE words) for offline disassembly. prog[]=program space,
 * data[]=data space (the OVLY DARAM [0x80,0x2800) where the scheduler/handlers run). */
void dsp54_dump_mem(const Dsp54 *d, const char *prog_path, const char *data_path,
                    uint32_t data_words) {
    if (prog_path) {
        FILE *f = fopen(prog_path, "wb");
        if (f) { for (uint32_t i = 0; i < C54X_PROG_SIZE; i++) {
                     uint8_t b[2] = { (uint8_t)(d->s->prog[i] >> 8), (uint8_t)d->s->prog[i] };
                     fwrite(b, 1, 2, f); } fclose(f); }
    }
    if (data_path) {
        if (data_words == 0 || data_words > C54X_DATA_SIZE) data_words = C54X_DATA_SIZE;
        FILE *f = fopen(data_path, "wb");
        if (f) { for (uint32_t i = 0; i < data_words; i++) {
                     uint8_t b[2] = { (uint8_t)(d->s->data[i] >> 8), (uint8_t)d->s->data[i] };
                     fwrite(b, 1, 2, f); } fclose(f); }
    }
}

/* Snapshot the whole DSP core (regs + prog + data + the api_ram OVLY window)
 * for fast standalone replay via tools/dsp/dsp_replay.c. */
int dsp54_snapshot(const Dsp54 *d, const char *path) {
    return c54x_snapshot_save(d->s, d->api_ram, path);
}

void dsp54_status(const Dsp54 *d, Dsp54Status *out) {
    if (!out) return;
    const C54xState *s = d->s;
    out->pc   = (uint16_t)s->pc;
    out->imr  = s->imr;
    out->ifr  = s->ifr;
    out->st1  = s->st1;
    out->pmst = s->pmst;
    out->sp   = s->sp;
    out->idle = s->idle ? 1 : 0;
    for (int i = 0; i < 8; i++) out->ar[i] = s->ar[i];
    out->st0   = s->st0;
    out->acc_a = (uint32_t)(s->a & 0xFFFFFFFF);
    out->xpc   = s->xpc;
}

uint16_t dsp54_data_peek(const Dsp54 *d, uint16_t addr) {
    // The DSP core executes window accesses [0x800,0x2800) against api_ram[] (OVLY/HPI), NOT
    // data[]. Reading data[] here returned a PHANTOM copy that diverges from what the DSP uses —
    // so window-cell diagnostics (ring ptrs 0x8E4/0x8E5, gates 0x866/0x195C/0x12C8) were wrong.
    // Read the SAME store the DSP does. (Diagnostic helper only — no effect on execution.)
    if (d->s->api_ram && addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
        return d->s->api_ram[addr - C54X_API_BASE];
    return d->s->data[addr];
}

/* Poke a word into the DSP's API-RAM (the shared MCU<->DSP window backing store) at
 * window offset woff (= MCU word index = DSP addr - 0x800). Lets the bridge deliver the
 * MCU's mailbox/command writes to the exact cells the DSP dequeues from (api_ram), which
 * the default data[]-only write path does not. See mad2_dsp_c54x.c DSP54_CMDBRIDGE. */
void dsp54_api_poke(Dsp54 *d, uint16_t woff, uint16_t val) {
    if (woff < C54X_API_SIZE) {
        d->api_ram[woff] = val;
        if (d->s && woff < 0x800u) d->s->win_dirty = 1;   // PERF: signal-window cell touched
    }
}

/* Poke a word into the DSP's DARAM data space (the OVLY FETCH source). For [0x800,0x2800) also
 * mirror it into api_ram, because the core routes DSP *fetches* to data[] but DSP *data-reads*
 * of that range to api_ram[] — on silicon both are the same OVLY DARAM, so keeping them coherent
 * models the single physical memory. Used to scatter the MCU's uploaded code blocks to their
 * dest (see mad2_dsp_c54x.c DSP54_REALUP base-set seed). */
void dsp54_data_poke(Dsp54 *d, uint16_t addr, uint16_t val) {
    if (!d || !d->s) return;
    // DSP54_PLACELOG=1: tag GLUE writes to the block run-homes. If our code (data_poke) ever
    // writes loader2(0xD80)/IVT(0xD00)/scheduler(0x926)/loader1(0xF00), the EMULATOR is arranging
    // blocks — NOT the real DSP. The DSP core's own stores do NOT go through here, so silence at a
    // home that nonetheless gets populated = the real loader1 scattered it. (Settles "who moved the
    // uploaded blocks?")
    { static int pl = -1; if (pl < 0) pl = getenv("DSP54_PLACELOG") ? 1 : 0;
      if (pl && (addr==0xD00u||addr==0xD80u||addr==0x926u||addr==0xF00u)) {
          static int n = 0; if (n++ < 64)
              fprintf(stderr, "[dsp54] PLACELOG: GLUE data_poke DSP[0x%04X]=0x%04X (emulator arranged this, not the DSP)\n", addr, val); } }
    d->s->data[addr] = val;
    if (addr >= 0x0800u && (uint32_t)(addr - 0x0800u) < C54X_API_SIZE) {   /* api_ram is an inline array (always present) */
        d->api_ram[addr - 0x0800u] = val;
        if ((uint32_t)(addr - 0x0800u) < 0x800u) d->s->win_dirty = 1;   // PERF: signal-window cell touched
    }
}

/* Zero a DSP data range [lo,hi] (inclusive). Models the codec delivering silence
 * into the FIR input/state buffers (DSP54_SILENCE convergence experiment). */
void dsp54_zero_range(Dsp54 *d, uint16_t lo, uint16_t hi) {
    for (uint32_t a = lo; a <= hi && a < C54X_DATA_SIZE; a++) d->s->data[a] = 0;
}
