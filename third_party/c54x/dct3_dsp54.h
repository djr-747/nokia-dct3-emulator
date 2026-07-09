/* dct3_dsp54 — integration contract for the DCT3 (MAD2/MAD2) DSP core.
 *
 * Wraps the lifted TMS320C54x interpreter + the Nokia HPI/port bridge behind a
 * small API that the platform model (mad2) drives. This is the surface the
 * co-simulation is built on:
 *
 *   mad2 per-step tick:        dsp54_step(d, n)
 *   MCU reads DSP shared RAM:  dsp54_hpi_read(d, mcu_addr)
 *   MCU writes DSP shared RAM: dsp54_hpi_write(d, mcu_addr, val)
 *   MCU pulses DSPINT:         dsp54_host_interrupt(d, vec)
 *
 * HPI window: the MCU sees DSP data words 0x000..0x7FF at MCU 0x10000..0x10FFF,
 * i.e. MCU_addr = 0x10000 + 2*DSP_word.  (Mailbox: cobba=0x070/0x100E0,
 * req=0x071/0x100E2, reply=0x072/0x100E4, mbox0=0x07F/0x100FE, mbox1=0x080/0x10100,
 * boot_status=0x002/0x10004.)
 *
 * The I/O-port mailbox (PORTR/PORTW ports 1/2/3, used by the host-command ISR
 * 0x35B9) is routed through the port callbacks — the host side decides how those
 * alias onto the MCU shared RAM (still being pinned in co-sim).
 */
#ifndef DCT3_DSP54_H
#define DCT3_DSP54_H

#include <stdint.h>
#include <stddef.h>

typedef struct Dsp54 Dsp54;

/* Create from a flat big-endian C54x program image (e.g. dsp_full.bin).
 * Performs reset (PMST=0xFFA8) and sets PC to the real reset entry 0x0F00. */
Dsp54 *dsp54_create(const uint8_t *image, size_t nbytes);
void   dsp54_destroy(Dsp54 *d);

/* Advance the DSP by n instructions (returns #executed). */
int    dsp54_step(Dsp54 *d, int n_insns);
/* Enable the on-chip timer (TIM/PRD counting + TINT vec 19) — the run-mode RTOS-tick heartbeat. */
void   dsp54_set_timer(Dsp54 *d, int on);
/* Register a 0x10000-entry per-instruction PC histogram (NULL=off). Reliable at any DSP:MCU ratio. */
void   dsp54_set_pc_histogram(Dsp54 *d, uint32_t *h);

/* Seed the OVLY DARAM (api_ram) from the resident image for DSP addrs [lo,hi). Models the chip's
 * DARAM being (re)loaded from ROM after loader2's bootstrap clears it. Returns words copied. */
uint32_t dsp54_seed_daram_from_prog(Dsp54 *d, uint16_t lo, uint16_t hi);

/* Cumulative count of DSP->host HINT doorbell pulses (BSCR 0x29 bit3 rising edge). The MCU
 * side latches IRQ4 (DSP mailbox handler 0x2BCEF0) when this advances. See mad2_dsp_c54x. */
unsigned long long dsp54_hint_edges(const Dsp54 *d);

/* PERF: read+clear the low-window dirty flag (set by any DSP/MCU write to a signal cell).
 * Returns nonzero if the window changed since the last call. The per-MCU-tick cosim monitors
 * gate their window peeks on this — a quiescent tick (return 0) skips a no-op eval. Returns 1
 * (never skip) when no DSP core is present, so non-cosim/HLE paths are unaffected. */
int dsp54_window_dirty_clear(Dsp54 *d);

/* Conditional halt on a DSP PC: freeze the run loop BEFORE executing (PC&0xFFFF)==pc for the
 * `after`-th time. dsp54_halted() returns nonzero once frozen. pc=0xFFFF + after=0 disables. */
void dsp54_set_halt(Dsp54 *d, uint16_t pc, uint32_t after);
void dsp54_set_halt_sp(Dsp54 *d, uint16_t floor);   /* freeze when SP drops below floor (over-pop) */
int  dsp54_halted(const Dsp54 *d);

/* Dump the full live DSP memory map (BE words) + regs to <prefix>.{prog,api,data,regs} for
 * offline disassembly (tools/dsp/disasm_dump.py). Returns 0 on success. */
int  dsp54_dump_full(const Dsp54 *d, const char *prefix);

/* Load a DSP code block (flat BE words) into program memory at word addr `dest`. */
void   dsp54_load_block(Dsp54 *d, uint16_t dest, const uint8_t *bytes, size_t nbytes);

/* HPI shared-RAM window access from the MCU side (mcu_addr in 0x10000..0x10FFF). */
uint16_t dsp54_hpi_read (Dsp54 *d, uint32_t mcu_addr);
void     dsp54_hpi_write(Dsp54 *d, uint32_t mcu_addr, uint16_t val);

/* Pulse a DSP interrupt (e.g. the host-command vector). vec = C54x interrupt #. */
void   dsp54_host_interrupt(Dsp54 *d, int vec);

/* DIAGNOSTIC: force the DSP program counter (e.g. jump into the superloop head 0x3000 to
 * A/B whether the frame chain runs once reached — not a faithful path; bring-up probe only). */
void   dsp54_set_pc(Dsp54 *d, uint16_t pc);
/* Warm boot: faithful C54x register/status reset (PC=0xFF80, INTM=1, OVLY=1, ...) PRESERVING DARAM
 * [0x80,0x2800). Used for the MCU's post-upload reset cycle so the verified code survives. */
void   dsp54_warm_reset(Dsp54 *d, uint16_t pc);
void   dsp54_set_ovly_unified(Dsp54 *d, int on);

/* Host-side I/O-port mailbox callbacks (PORTR/PORTW, PA < 0x100). The host
 * provides the command on read and consumes the reply on write. */
void dsp54_set_port_io(Dsp54 *d,
                       uint16_t (*port_read)(void *opaque, uint16_t pa),
                       void     (*port_write)(void *opaque, uint16_t pa, uint16_t val),
                       void *opaque);

/* Diagnostic peek at the DSP core state (for co-sim bring-up — where is the DSP parked,
 * is INT2 unmasked, is it IDLE). Read-only snapshot. */
typedef struct Dsp54Status {
    uint16_t pc;     /* current program counter */
    uint16_t imr;    /* interrupt mask register  */
    uint16_t ifr;    /* interrupt flag (pending) register */
    uint16_t st1;    /* status reg 1 (bit11 = INTM global mask) */
    uint16_t pmst;   /* PMST (bits 7-15 = IPTR interrupt-vector base / 0x80) */
    uint16_t sp;     /* stack pointer (data addr) — watch for runaway stack into DARAM */
    int      idle;   /* nonzero if the DSP is in IDLE (awaiting interrupt) */
    uint16_t ar[8];  /* AR0..AR7 (auxiliary/address registers) */
    uint16_t st0;    /* status reg 0 (bits 0-8 = DP data-page pointer) */
    uint32_t acc_a;  /* accumulator A (low 32 bits) */
    uint16_t xpc;    /* extended program counter (program-bank select; linear PC = xpc<<16 | pc) */
} Dsp54Status;
void dsp54_status(const Dsp54 *d, Dsp54Status *out);

/* Peek a live DSP data-memory word (full data space, beyond the HPI window). */
uint16_t dsp54_data_peek(const Dsp54 *d, uint16_t addr);

/* Poke a word into the shared API-RAM window at offset woff (= MCU word index). */
void dsp54_api_poke(Dsp54 *d, uint16_t woff, uint16_t val);

/* Poke a word into DSP DARAM data space (fetch store); mirrors into api_ram for [0x800,0x2800)
 * so fetch and data-read views stay coherent. Used to scatter uploaded code blocks to dest. */
void dsp54_data_poke(Dsp54 *d, uint16_t addr, uint16_t val);

/* Zero a DSP data range [lo,hi] inclusive (codec-silence convergence experiment). */
void dsp54_zero_range(Dsp54 *d, uint16_t lo, uint16_t hi);

/* Static-upload regression guard: returns 1 (and warns) only if DSP54_ALLOW_STATIC=1, else
 * prints a "retired/non-faithful" notice and returns 0. Gates the NON-FAITHFUL static
 * shortcuts (DSP54_OVERLAY/BLOCKS/UPLOAD/LOADERGO) so future sessions rely on the MCU's real
 * block upload. See memory 5110-dsp-staged-upload-pivot +. */
int dsp54_static_allowed(const char *knob);

/* Dump live DSP memory (BE words) for offline disassembly (tic54x-objdump). */
void dsp54_dump_mem(const Dsp54 *d, const char *prog_path, const char *data_path,
                    uint32_t data_words);

/* Snapshot the whole DSP core (regs + prog + data + api_ram OVLY window) to a
 * flat file for fast standalone replay (tools/dsp/dsp_replay.c). Returns 0 ok. */
int dsp54_snapshot(const Dsp54 *d, const char *path);

/* Faithful-path knob resolver. The 5110 DSP faithful load path otherwise needs ~8 knobs
 * (UNIDARAM/SEEDBASE/REALUPLOAD/SHAREDWIN/RATIO=4/REALUP/HPIAPI/NOSELFTEST) — a soup that
 * drifts across sessions. Collapse it: once DSP54_COSIM is selected, every faithful knob
 * DEFAULTS ON. An explicit env value still WINS (set DSP54_X=0 to opt a single knob out).
 * Returns the resolved boolean. (Run-mode crutches MMR58/FRAMEINT stay explicit — temporary.) */
int dsp54_faithful(const char *name);

#endif /* DCT3_DSP54_H */
