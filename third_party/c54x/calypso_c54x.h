/*
 * calypso_c54x.h — TMS320C54x DSP emulator for Calypso
 *
 * Emulates the C54x DSP core found in the TI Calypso baseband chip.
 * Loads ROM dump, executes instructions, shares API RAM with ARM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_C54X_H
#define CALYPSO_C54X_H

#include <stdint.h>
#include <stdbool.h>

/* Memory sizes (in 16-bit words) */
#define C54X_PROG_SIZE   0x40000  /* 256K words program space */
#define C54X_DATA_SIZE   0x10000  /* 64K words data space */
#define C54X_IO_SIZE     0x10000  /* 64K words I/O space */

/* API RAM: shared between ARM and DSP */
#define C54X_API_BASE    0x0800   /* DSP data address of API RAM */
#define C54X_API_SIZE    0x2000   /* 8K words */

/* DSP start address (after boot) */
#define C54X_DSP_START   0x7000

/* MMR addresses (data memory 0x00-0x1F) */
#define MMR_IMR   0x00
#define MMR_IFR   0x01
#define MMR_ST0   0x06
#define MMR_ST1   0x07
#define MMR_AL    0x08
#define MMR_AH    0x09
#define MMR_AG    0x0A
#define MMR_BL    0x0B
#define MMR_BH    0x0C
#define MMR_BG    0x0D
#define MMR_T     0x0E
#define MMR_TRN   0x0F
#define MMR_AR0   0x10
#define MMR_AR1   0x11
#define MMR_AR2   0x12
#define MMR_AR3   0x13
#define MMR_AR4   0x14
#define MMR_AR5   0x15
#define MMR_AR6   0x16
#define MMR_AR7   0x17
#define MMR_SP    0x18
#define MMR_BK    0x19
#define MMR_BRC   0x1A
#define MMR_RSA   0x1B
#define MMR_REA   0x1C
#define MMR_PMST  0x1D
#define MMR_XPC   0x1E

/* Timer registers (memory-mapped at 0x0024-0x0026) */
#define TIM_ADDR  0x0024   /* Timer counter */
#define PRD_ADDR  0x0025   /* Timer period */
#define TCR_ADDR  0x0026   /* Timer control */

/* TCR bit positions (TMS320C54x hardware spec) */
#define TCR_TDDR_MASK  0x000F   /* bits 3:0 — prescaler reload value */
#define TCR_TSS        (1 << 4) /* bit 4 — Timer Stop Status (1=stopped) */
#define TCR_TRB        (1 << 5) /* bit 5 — Timer Reload (write 1 reloads) */
#define TCR_PSC_SHIFT  6        /* bits 9:6 — prescale counter */
#define TCR_PSC_MASK   (0xF << TCR_PSC_SHIFT)
#define TCR_SOFT       (1 << 10)
#define TCR_FREE       (1 << 11)

/* ST0 bit positions */
#define ST0_DP_MASK  0x01FF  /* bits 8-0: data page pointer */
#define ST0_OVB      (1 << 9)
#define ST0_OVA      (1 << 10)
#define ST0_C        (1 << 11)
#define ST0_TC       (1 << 12)
#define ST0_ARP_SHIFT 13
#define ST0_ARP_MASK (7 << ST0_ARP_SHIFT)

/* ST1 bit positions */
#define ST1_ASM_MASK 0x001F  /* bits 4-0: accumulator shift mode */
#define ST1_CMPT     (1 << 5)
#define ST1_FRCT     (1 << 6)
#define ST1_C16      (1 << 7)
#define ST1_SXM      (1 << 8)
#define ST1_OVM      (1 << 9)
#define ST1_INTM     (1 << 11)
#define ST1_HM       (1 << 12)
#define ST1_XF       (1 << 13)
#define ST1_BRAF     (1 << 14)

/* PMST bit positions (per SPRU131: SST=0 SMUL=1 CLKOFF=2 DROM=3 APTS=4 OVLY=5 MP/MC=6) */
#define PMST_SST     (1 << 0)
#define PMST_SMUL    (1 << 1)
#define PMST_CLKOFF  (1 << 2)
#define PMST_DROM    (1 << 3)
#define PMST_APTS    (1 << 4)
#define PMST_OVLY    (1 << 5)
#define PMST_MP_MC   (1 << 6)
#define PMST_IPTR_SHIFT 7
#define PMST_IPTR_MASK (0x1FF << PMST_IPTR_SHIFT)

/* Interrupt vectors */
#define C54X_INT_RESET   0
#define C54X_INT_NMI     1
/* TMS320C54x interrupt mapping: vector = IMR_bit + 2
 * IMR bit 0 → INT0 → vec 2    IMR bit 5 → BRINT0 → vec 7
 * IMR bit 1 → INT1 → vec 3    IMR bit 6 → BXINT0 → vec 8
 * IMR bit 2 → INT2 → vec 4    IMR bit 7 → DMAC0  → vec 9
 * IMR bit 3 → INT3 → vec 5    IMR bit 8 → DMAC1  → vec 10
 * IMR bit 4 → TINT0→ vec 6    IMR bit 9 → INT4   → vec 11
 *                              IMR bit 10→ INT5   → vec 12 */
/* TMS320C54x interrupt vector mapping (SPRU131):
 * Vec 0: RESET     Vec 16: INT0 (IMR bit 0)
 * Vec 1: NMI       Vec 17: INT1 (IMR bit 1)
 * Vec 2: SINT17    Vec 18: INT2 (IMR bit 2)
 * Vec 3: SINT18    Vec 19: INT3 (IMR bit 3)
 * Vec 4: SINT19    Vec 20: TINT0 (IMR bit 4)
 * Vec 5: SINT20    Vec 21: BRINT0 (IMR bit 5)
 * ...              Vec 22: BXINT0 (IMR bit 6)
 *                  Vec 23: DMAC0 (IMR bit 7)
 *                  Vec 24: DMAC1 (IMR bit 8)
 * Formula: vec = imr_bit + 16 */
/* Calypso DSP firmware enables IMR bits 3 + 7 + upper (observed IMR=0xFF88).
 * Bit 3 = INT3 = vec 19 — this is the external frame-sync line from the TPU,
 * the only "frame" interrupt the firmware actually unmasks. Use it. */
#define C54X_INT_FRAME_VEC   19  /* INT3 = vec (3+16) */
#define C54X_INT_FRAME_BIT   3   /* IMR bit 3 */
#define C54X_NUM_INTS        16

typedef struct C54xState {
    /* Accumulators (40-bit) stored as int64 for convenience */
    int64_t a;   /* A accumulator: bits 39-0 */
    int64_t b;   /* B accumulator: bits 39-0 */

    /* Auxiliary registers */
    uint16_t ar[8];

    /* Other registers */
    uint16_t t;      /* Temporary register */
    uint16_t trn;    /* Transition register (Viterbi) */
    uint16_t sp;
    uint16_t bk;     /* Circular buffer size */
    uint16_t brc;    /* Block repeat counter */
    uint16_t rsa;    /* Block repeat start address */
    uint16_t rea;    /* Block repeat end address */

    /* Status registers */
    uint16_t st0;
    uint16_t st1;
    uint16_t pmst;

    /* Interrupt registers */
    uint16_t imr;
    uint16_t ifr;

    /* Optional reset-state override loaded from calypso_dsp.Registers.bin via
     * `-M calypso,dsp-registers=<path>` (default-wired by run.sh, like the
     * other ROM sections). reg_init[i] = value for MMR index i (0x00..0x1F).
     * When reg_init_valid, c54x_reset() applies these AFTER its silicon
     * hardcode defaults, so the .bin snapshot is authoritative. */
    uint16_t reg_init[0x20];
    bool     reg_init_valid;

    /* Program counter */
    uint32_t pc;     /* 16-bit (or 23-bit with XPC) */
    uint16_t xpc;

    /* Timer0 prescale counter (PSC) — not memory-mapped directly */
    uint16_t timer_psc;
    /* On-chip timer COUNTING enable (SPRU131G §8.8). Off by default so legacy
     * users (Calypso spike) keep the inert register-file-only model; when on,
     * TIM/PSC count per executed insn (and per idle-1 budget — IDLE1 keeps
     * peripheral clocks running), underflow reloads from PRD/TDDR and raises
     * TINT (vec 19 / IMR bit 3), and reset starts the timer (TSS=0) with
     * TIM=PRD=0xFFFF per spec. */
    bool     timer_enabled;
    /* C542-class chip mode: NO extended program memory / XPC. Interrupt entry
     * pushes ONLY the 16-bit PC (1 word) and RETE pops only PC — unlike the
     * C548/549 2-word {PC,XPC} frames the lift was built around. Firmware for
     * these chips legitimately uses `call ...; rete` as a task-level
     * return-and-enable-interrupts idiom; 2-word frames desync the stack by
     * one word there and "return" into a zero page. Off by default (legacy
     * Calypso behavior); the 5110 MAD1 LEAD sets it. */
    bool     no_xpc;

    /* DMA sub-register bank (6 channels × 4 regs) */
    uint16_t dma_subaddr;
    uint16_t dma_subregs[24];
    /* McBSP sub-register bank */
    uint16_t spsa;

    /* RPT state */
    uint16_t rpt_count;  /* remaining RPT iterations */
    uint16_t rpt_pc;     /* PC of repeated instruction */
    bool     rpt_active;
    bool     rpt_arming;  /* set by the RPT arm handlers; the run loop skips the
                             repeat-decrement on the arming pass so RPT N executes
                             the target N+1 times (silicon semantics, SPRU131) */
    uint16_t par;        /* Program Address Register (for READA/WRITA/MACD/MACP) */
    bool     par_set;
    bool     lk_used;    /* resolve_smem consumed extra word for lk */
    uint16_t mvpd_src;   /* MVPD auto-increment source address during RPT */

    /* RPTB state */
    bool     rptb_active;

    /* Delayed-branch state (CALLD/RETD/BD/CCD/...): when set, the next
     * `delay_slots` instructions execute normally, then PC is forced to
     * `delayed_pc`. */
    uint16_t delayed_pc;
    uint8_t  delay_slots;

    /* Memory */
    uint16_t prog[C54X_PROG_SIZE];   /* Program memory */
    uint16_t data[C54X_DATA_SIZE];   /* Data memory */

    /* API RAM pointer (shared with ARM calypso_trx.c) */
    uint16_t *api_ram;  /* points into ARM's dsp_ram[] */

    /* OVLY single-store: when set, program-space accesses (fetch / MVPD / MVDP) to the
     * api_ram window [C54X_API_BASE, +SIZE) route to api_ram[] — the SAME cells the DSP's
     * data accesses use. Models the C54x OVLY bit faithfully (DARAM is ONE dual-mapped
     * memory; program addr == data addr == same cell). Without it, fetch reads data[] while
     * data reads api_ram[] = two stores for one OVLY cell (uploaded code never executes).
     * See docs/dsp-c54x-memory-model.md. Set by DSP54_UNIDARAM via c54x_set_ovly_unified(). */
    int ovly_unified;

    /* DSP → ARM notify hook: called whenever the DSP writes to api_ram. */
    void (*api_write_cb)(void *opaque, uint16_t woff, uint16_t val);
    void  *api_write_cb_opaque;

    /* PERF (cosim window-dirty gate): set whenever ANY agent writes the low DSP window
     * (woff < 0x800 = mailbox / MDISND+MDIRCV rings / pointer / cmd cells) — by the DSP
     * core's api_ram store (data_write) and by the MCU-side dct3 write helpers. Lets the
     * per-MCU-tick monitors (cmdlevel_eval + RINGFIQ in mad2_dsp_c54x.c) skip their window
     * peeks on quiescent ticks: when no signal cell changed since the last eval, the eval is
     * a no-op (level/pointers unchanged -> no edge/FIQ). Read+cleared once per tick via
     * dsp54_window_dirty_clear(). Conservative: a superset of writes sets it, so it can never
     * skip a real change (the determinism oracle is `make guard` byte-identical). */
    int win_dirty;

    /* State */
    bool     running;
    bool     idle;       /* IDLE instruction executed */
    bool     blob_loaded; /* Test fixture: set by c54x_set_initial_pc().
                           * Suppresses the secondary c54x_reset() that
                           * normally fires when ARM writes DSP_DL_STATUS_READY,
                           * which would otherwise clobber the user's blob
                           * via the reset-time PROM→DARAM auto-copy. */
    uint64_t cycles;
    uint32_t insn_count;

    /* BSP (Baseband Serial Port) — burst sample buffer */
    uint16_t bsp_buf[2048]; /* burst I/Q samples from radio */
    int      bsp_len;       /* number of samples */
    int      bsp_pos;       /* read position */

    /* Debug */
    uint32_t unimpl_count;
    uint16_t last_unimpl;
    /* Last executed instruction snapshot — captured at end of each
     * c54x_run iteration. Used by the INTM-TRANS tracer (and others)
     * to attribute post-instruction state changes to the actual cause
     * PC/opcode rather than the post-advance PC. */
    uint16_t last_exec_pc;
    uint16_t last_exec_op;

    /* writer_kind : set by each opcode handler / external writer before
     * calling data_write. Logged in DATA-W-MMR trace to disambiguate
     * which path is responsible for stray writes to MMR (addr<=0x1F).
     * Reset to WK_UNKNOWN at the top of c54x_exec_one. */
    uint8_t  writer_kind;
} C54xState;

/* writer_kind enum — keep small, extend as needed */
enum {
    WK_UNKNOWN     = 0,
    WK_OPCODE_F3   = 1,   /* 0xF3xx family (SFTL/AND/OR/XOR/INTR/etc.) */
    WK_OPCODE_8x   = 2,   /* 0x80xx-0x8Fxx (STL/STH/STLM/STM/LD-Smem) */
    WK_OPCODE_77   = 3,   /* 0x77xx STM #lk, MMR */
    WK_OPCODE_76   = 4,   /* 0x76xx ST #lk, Smem */
    WK_OPCODE_PSHM = 5,   /* PSHM/POPM stack ops */
    WK_OPCODE_RET  = 6,   /* RET/RETI/RETD frame restore */
    WK_IRQ_ACK     = 7,   /* IRQ acknowledge / vector dispatch */
    WK_ARM_MMIO    = 8,   /* ARM-side write through shared region */
    WK_RESOLVE_AR  = 9,   /* resolve_smem AR-modify side effect */
    WK_OPCODE_OTHER= 10,  /* anything else inside an opcode handler */
};

/* Feed burst samples to BSP (called by calypso_trx) */
void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n);

/* Create and initialize C54x state */
C54xState *c54x_init(void);

/* Link API RAM (shared memory with ARM) */
void c54x_set_api_ram(C54xState *s, uint16_t *api_ram);
void c54x_set_ovly_unified(C54xState *s, int on);

/* Snapshot the whole core (regs + prog[] + data[] + api_ram window) to a flat
 * file and reload it for fast standalone replay (see tools/dsp/dsp_replay.c). */
int c54x_snapshot_save(const C54xState *s, const uint16_t *api_ram, const char *path);
int c54x_snapshot_load(C54xState *s, uint16_t *api_ram_buf, const char *path);

/* Reset the DSP */
void c54x_reset(C54xState *s);

/* Execute N instructions (returns actual count executed) */
int c54x_run(C54xState *s, int n_insns);
/* Register a 0x10000-entry PC histogram filled per executed instruction (NULL=off, default).
 * Reliable at any DSP:MCU ratio (counts inside the run loop, not once per MCU tick). */
void c54x_set_pc_histogram(uint32_t *h);
/* Zero the registered histogram in place (keeps the buffer). Called at the
 * loader1->run-mode warm reset so a dump shows run-mode, not the cold-upload CRC. */
void c54x_reset_pc_histogram(void);

/* Conditional halt: freeze the run loop BEFORE executing (PC&0xFFFF)==pc for the `after`-th time.
 * c54x_halted() returns nonzero once frozen. pc=0xFFFFFFFF disables. */
void c54x_set_halt(uint32_t pc, uint32_t after);
int  c54x_halted(void);
/* SP-floor halt: freeze right after SP drops below `floor` (catches the over-pop). 0 = off. */
void c54x_set_halt_sp(uint16_t floor);
/* Dump the SP-event ring (last 64 push/pop events with PC/op/delta) to a FILE* (passed as void*). */
void c54x_dump_sp_ring(void *f);

/* Raise an interrupt */
/* Send interrupt: vec = vector number (for PC), imr_bit = bit in IMR/IFR */
void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit);
void c54x_set_timer_enabled(C54xState *s, int on);
void c54x_set_no_xpc(C54xState *s, int on);

/* Wake from IDLE */
void c54x_wake(C54xState *s);

/* Mark the core as past a warm-boot (post-upload release #2). Gates the
 * SPUNDER stack-underflow probe so it only watches run-mode, not phase-1. */
void c54x_mark_warm(int on);

/* Test fixture: override PC after reset.
 * Used by `-M calypso,dsp-blob=<path>` to start execution at a custom
 * address instead of the silicon-default reset vector (IPTR * 0x80). */
void c54x_set_initial_pc(C54xState *s, uint32_t pc);

/* Test fixture: load a raw binary blob into DARAM starting at daram_addr.
 * File bytes are read pairwise as little-endian DSP words.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_blob_daram(C54xState *s, const char *path, uint16_t daram_addr);

/* Explicit per-section ROM load: write raw LE 16-bit words from `path`
 * into either prog[] (when is_program=true) or data[] (when false),
 * starting at DSP word address `start_addr`. Used by the per-section
 * machine properties (dsp-prom0/prom1/prom2/prom3/drom/pdrom) to load
 * each ROM section at its silicon-correct DSP address.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_section(C54xState *s, const char *path,
                       uint32_t start_addr, bool is_program);

/* Load the DSP register snapshot (calypso_dsp.Registers.bin: raw LE 16-bit
 * words, MMR page 0x00..0x1F first) into reg_init[] so c54x_reset() applies
 * it as the reset state. Words >= 0x20 are written into data[] (low scratch).
 * Used by the `-M calypso,dsp-registers=<path>` machine property.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_registers(C54xState *s, const char *path);

#endif /* CALYPSO_C54X_H */
