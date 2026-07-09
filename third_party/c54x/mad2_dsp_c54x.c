// mad2_dsp_c54x — per-model DSP backend that runs the REAL recovered TMS320C54x
// DSP image (re/dsp-5110/raw/dsp_full.bin) in co-simulation with the MCU, instead
// of behaviourally faking the mailbox. Native-only (the 635 KB C54x interpreter is
// kept out of the wasm build); the 5110 profile selects it under #ifndef __EMSCRIPTEN__.
//
// GOAL: make the real DSP produce the boot reply in the
// HPI shared-RAM window so the firmware's DSP-block loader builds a valid branch table
// and sets [0x10A9E4]=1 ITSELF — clearing the "DSP ROM MISMATCH" screen faithfully,
// not by poking the flag.
//
// Two modes (env-gated so bring-up is incremental and never regresses a known-good boot):
//
//   default (DSP54_COSIM unset): PASS-THROUGH. The real DSP is created (proving it
//     loads + boots in-process) but the firmware still sees the faithful legacy
//     mailbox model (dsp_default_*). 5110 behaviour is byte-identical to before — the
//     safe baseline while the co-sim is brought up.
//
//   DSP54_COSIM=1: CO-SIM. The DSP is stepped each MCU tick and the HPI window
//     (MCU 0x10000..0x10FFF <-> DSP data words) is synced both directions; MCU mailbox
//     writes that strobe DSPINT pulse the DSP's host-command interrupt. This is the
//     path under active development (the exact port<->0x100xx alias + host-cmd vector
//     are still being pinned empirically — see the README / handoff).
//
// Env knobs (all native-only, default OFF):
//   DSP54_COSIM=1        enable the real co-sim HPI bridge (else pass-through)
//   DSP54_IMAGE=<path>   override the DSP image (default re/dsp-5110/raw/dsp_full.bin)
//   DSP54_RATIO=<n>      DSP instructions stepped per MCU step in co-sim (default 1)
//   DSP54_INTVEC=<n>     C54x interrupt vector pulsed on DSPINT (default 18 = INT2 ->
//                        vector 0xFFC8 -> host-cmd ISR 0x3598/0x35B9; full.s54 vector table)
//   DSP54_LOG=1          trace lazy-init + first port/HPI activity

#include "mad2/mad2.h"
#include "dct3_dsp54.h"
#include "calypso_c54x.h"   // C54X_DATA_SIZE (image buffer bound)
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define DSP_IMAGE_DEFAULT "re/dsp-5110/raw/dsp_full.bin"
#define HPI_LO  0x00010000u
#define HPI_HI  0x00010FFFu   /* DSP data words 0x000..0x7FF aliased here (dct3_dsp54.h) */

// Single DSP per phone (matches the file-scope core in calypso_c54x.c). Created lazily
// on first DSP-region access so a build without the gitignored image still runs (tests,
// the wasm build never reaches here): on load failure we latch UNAVAILABLE and forward
// to the legacy model forever.
static Dsp54 *g_dsp;
static int    g_state;     // 0=untried, 1=active, -1=unavailable
static int    g_cosim;     // DSP54_COSIM
static int    g_realup = -1;   // DSP54_REALUP: faithful staged boot upload — loader1 staged at the
                               //   cold 0x20002 reset-release edge, then it drives the live MCU upload
static int    g_dsp_run;       // co-sim DSP gated by the MCU's DSP-reset-ctrl (0x20002 bit0):
                               //   held in reset (0) until the firmware releases it (1). REALUP only.
static int    g_ratio = 1; // DSP54_RATIO
static int    g_intvec = 18;// DSP54_INTVEC (C54x INT2 = vec 18 -> 0x3598 host-cmd ISR)
static int    g_log;
static int    g_intrepeat;  // DSP54_INTREPEAT: re-pulse host int every N MCU steps (0=off;
                            // level-trigger experiment — a single edge is lost while INTM=1)
static int    g_frameint;   // DSP54_FRAMEINT: C54x vector for the simulated codec frame int
                            // (e.g. 21=BRINT0 / 22=BXINT0, both enabled in IMR=0x52FD) (0=off)
static int    g_frameper = 50000; // DSP54_FRAMEPER: raise the frame int every N MCU steps
static uint64_t g_frameafter;     // DSP54_FRAMEAFTER: gate FRAMEINT until dsp_steps > N (so
                                  // the codec INT0 fires only AFTER the host-cmd handshake +
                                  // superloop entry settle — firing from step 0 corrupts boot)
static int    g_p27_init, g_p27_on; static uint16_t g_p27_val; // DSP54_P27: codec RX sample on
                                  // port 0x27 (INT0 0x3204 reads it into the FIR buffer); silence=0
static unsigned long long g_dspint_edges; // DSPINT doorbell strobes seen (diagnostic)

// --- host I/O-port mailbox bridge -------------------------------------------------
// The DSP PROM's host-command ISR (0x35B9) reads/writes PORTR/PORTW ports 1/2/3. In
// co-sim those alias onto the MCU 0x100xx shared RAM; the exact mapping is still being
// pinned, so for now we just observe (DSP54_LOG) and return 0. opaque = Mad2*.
//
// HLE peripheral stubs (the user's "stub gates in DSP ROM so it doesn't wait on
// unmodeled peripherals"): the resident PROM polls the COBBA codec serial port 0x2D
// at boot (0x45F8/0x465E/0x466F) for a bit12 ready/busy handshake. Without a modelled
// codec the DSP spins there forever. Env-tunable so the exact pass value is pinned
// empirically:
//   DSP54_P2D=<hex>      value returned for every port 0x2D read (default 0)
//   DSP54_P2DTOGGLE=1    toggle bit12 (0x1000) each 0x2D read (models a busy->ready
//                        handshake that flips per access — the usual serial-codec idiom)
static int      g_p2d_init;
static uint16_t g_p2d_val;       // DSP54_P2D fixed return for port 0x2D
static int      g_p2d_toggle;    // DSP54_P2DTOGGLE: flip bit12 per read
static uint16_t g_p2d_state;     // toggling accumulator
// Host-command mailbox port <-> MCU shared-RAM (0x100xx) alias. The DSP host-cmd ISR
// (0x35B9) computes cmd=(~port2)&port1 and replies via port2 (0x35FF/0x3605).
// GROUNDED 2026-06-10 (CONTACT SERVICE chain): the MCU's generic DSP-API poster
// (0x272042) RMWs the COMMAND WORD at [0x100A8] (= window k=0x54, DSP cell 0x854; e.g.
// 0x8002 @1042k = cmd bit1 -> handler 0x3621 -> [0x866]|=1 = the MDISND dequeue enable),
// then strobes [0x30000] bit2 (INT2). The ACK word is the next cell [0x100AA] (0x855):
// each handler ORs its bit into port2 and the dequeue disarm (0x3963) clears bit1 to
// RE-ARM. So port1(read)<->0x100A8, port2<->0x100AA. With these, h3621/dequeue3860 fire
// for the first time (the old 0x10102/0x10100 guess read back the DSP's own writes).
// pa1 WRITES are a SEPARATE DSP->MCU status latch (0x8DA8 boot-report port1=0x0700,
// 0x362F port1=0x001C, version 0x8EDF port1=0x0200): landing them on 0x100A8 would
// clobber the MCU's command word, so they go to g_port1_waddr if set, else a private
// latch (dropped). Addresses env-overridable for empirical pinning. 0 = return 0.
static uint32_t g_port1_addr = 0x000100A8u, g_port2_addr = 0x000100AAu, g_port3_addr = 0;
static uint32_t g_port1_waddr = 0, g_port2_waddr = 0;  // asymmetric write-latch targets (opt-in)
static int      g_port_init;
// Dispatchable cmd bits of ISR 0x35B9 (first-match table 0x35C0-0x35F6): bit12, bits2-4,
// bits8-10, bit7, bit1, bit0, bits5-6. Bits 11/13-15 are NOT dispatched (status flags) —
// excluded from the level computation or an unacked bit15 would re-interrupt forever.
#define HOSTCMD_DISPATCH_MASK 0x17FFu
static uint16_t mcu_word(Mad2 *m, uint32_t a) {
    if (!a || !m->mem) return 0;
    uint32_t i = a & m->mem_mask;
    return (uint16_t)((m->mem[i] << 8) | m->mem[i + 1]);   // BE halfword (ARM core is BE)
}
static void mcu_set_word(Mad2 *m, uint32_t a, uint16_t v) {
    if (!a || !m->mem) return;
    uint32_t i = a & m->mem_mask;
    m->mem[i] = (uint8_t)(v >> 8); m->mem[i + 1] = (uint8_t)v;
}
static void port_lazy(void) {
    if (g_port_init) return;
    g_port_init = 1;
    const char *p1 = getenv("DSP54_PORT1ADDR"); if (p1 && *p1) g_port1_addr = (uint32_t)strtoul(p1,0,0);
    const char *p2 = getenv("DSP54_PORT2ADDR"); if (p2 && *p2) g_port2_addr = (uint32_t)strtoul(p2,0,0);
    const char *p3 = getenv("DSP54_PORT3ADDR"); if (p3 && *p3) g_port3_addr = (uint32_t)strtoul(p3,0,0);
    const char *w1 = getenv("DSP54_PORT1WADDR"); if (w1 && *w1) g_port1_waddr = (uint32_t)strtoul(w1,0,0);
    const char *w2 = getenv("DSP54_PORT2WADDR"); if (w2 && *w2) g_port2_waddr = (uint32_t)strtoul(w2,0,0);
}

// === DSP54_CMDLEVEL — the MAD2 glue level comparator on the host-cmd INT2 line ===========
// RE-grounded 2026-06-11. The C54x INT0-INT3 pins are EDGE-SAMPLED (SPRS039C: two-flip-flop
// synchronizer latches a falling edge into IFR), but the firmware's shared INT1/2/3 ISR
// (0x3598) is written against a LEVEL source: its exit (0x35F7) pulses port2=0xFFFF then
// restores — masking everything so the line deasserts and RE-EDGES if requests remain. That
// idiom only makes sense if MAD2 glue asserts the pin combinationally while
//     ((~port2) & port1) & dispatchable != 0
// i.e. "any unmasked host request pending". No MCU register write reaches INT2 (the MDISND
// poster touches only its own FIQ_ACT; MADos has no second doorbell primitive) and the DSP's
// on-chip timer is provably dead (the ONLY TIM/PRD/TCR access in the whole image is loader2's
// stop at 0xD80) — the request latch is the only armer of the MDISND dequeue left standing.
//
// bit1 (the dequeue-arm request) is the HARDWARE RING-STATE: tail!=head of the MDISND ring
// (DSP 0x852/0x853 = MCU [0x100A4]/[0x100A6]) — the same DSPIF comparator that already
// faithfully raises the MCU-side FIQs on ring-pointer advance (RINGFIQ), seen from the DSP
// side. This is what kills the 2026-06-10 ping-pong (level on the LATCHED memory bit1:
// ISR->arm->empty-dequeue->disarm->re-pend forever): with bit1 = ring-non-empty, an empty
// ring drops the request, so the disarm's port2-bit1 reopen cannot re-fire; the next MCU
// ring post re-edges it. Other request bits stay memory-latched (the MCU RMWs them at
// 0x272042) and their handlers mask-ack without ever reopening — one edge each, no loop.
// (That earlier ping-pong was also measured pre-no_xpc/POPD, i.e. on the corrupted stack.)
//
// Faithful default under DSP54_COSIM (DSP54_CMDLEVEL=0 opts out for A/B). Run-mode only
// (g_dsp_run): the boot-phase handshake is already verified without it.
static int g_cmdlevel = -1;      // knob cache (-1 = unread)
static int g_cmdlevel_state;     // current line level (1 = asserted at the pin)
static unsigned long long g_cmdlevel_edges;
static int g_hpivec = -1;       // vec for the host-cmd line (INT2=18); DSP54_HPIVEC overrides

// === DSP54_COMMLOG — unified bidirectional MCU<->DSP comms trace =========================
// One decoded, timestamped, PC-tagged line per signalling event so a run shows EXACTLY what
// the two cores send each other: reset/hold/release, the DSPINT (M->D) and HINT (D->M)
// doorbells, the MDISND (M->D) and MDIRCV (D->M) rings, and the mailbox/version handshake.
//   M->D events carry the exact MCU PC (m->cur_io_pc, set by mad2_read/mad2_write).
//   D->M events carry the DSP PC sampled when the change is detected (±a few insns at RATIO>1).
// Default OFF; read-only. Decodes a window offset k (DSP 0x800+k <-> MCU 0x10000+2k).
static int g_commlog = -1;
static const char *comm_kname(uint32_t k) {
    switch (k) {
        case 0x02: return "VERSION_A";       case 0x03: return "VERSION_B";
        case 0x52: return "MDISND_TAIL";     case 0x53: return "MDISND_HEAD";
        case 0x54: return "HOSTCMD_REQ(p1)"; case 0x55: return "HOSTCMD_ACK(p2)";
        case 0x5D: return "AUDIO_CMD";
        case 0x70: return "API_MAILBOX";
        case 0x71: return "UPLOADREQ/READY"; case 0x72: return "UPLOADREPLY/GO";
        case 0x7F: return "MBOX0";           case 0x80: return "MBOX1/PORT2";
        case 0x81: return "PORT1/CMD(legacy)";
        case 0xE4: return "MDIRCV_TAIL";     case 0xE5: return "MDIRCV_HEAD";
        default: break;
    }
    if (k >= 0x7B && k <= 0x7E)   return "UPLOADHEADER";
    if (k <= 0x51)                return "MDISND_RING";
    if (k >= 0x82 && k <= 0xE3)   return "MDIRCV_RING";
    if (k >= 0x100 && k <= 0x4FF) return "UPLOAD_BUF";
    return "window";
}
static uint16_t comm_dsp_pc(void) { Dsp54Status st; dsp54_status(g_dsp, &st); return st.pc; }
// Signal cells = the handshake/ring-pointer/mailbox words (vs bulk code-upload + ring payload).
// COMMLOG=1 shows signals only; COMMLOG>=2 also shows every payload/upload word.
static int comm_is_signal(uint32_t k) {
    // MBOX0/MBOX1 (0x7F/0x80) are the per-block upload ping-pong (~4 writes/block) — level 2 only.
    // 0x54/0x55 = the host-command API channel (request word [0x100A8]=port1-read / accept mask
    // [0x100AA]=port2 — the settled mapping, what CMDLEVEL watches); 0x70 = the [0x870] API
    // mailbox (vec25 doorbell command cell); 0x5D = the [0x85D] audio/tone command.
    return k==0x02 || k==0x03 || k==0x52 || k==0x53 || k==0x54 || k==0x55 || k==0x5D ||
           k==0x70 || k==0x71 || k==0x72 ||
           (k>=0x7B && k<=0x7E) || k==0x81 || k==0xE4 || k==0xE5;
}

// Evaluate the CMDLEVEL comparator and deliver one INT2 edge per 0->1 level transition.
// Called event-driven where both line inputs can change (DSP port2 writes = the ISR's
// mask/ack + the exit pulse; MCU window writes to cmd/mask/ring-pointer cells) and sampled
// per-tick (catches the ring-empty deassert from the DSP's own head-advance store, which
// has no write hook). Deasserts only update state — the edge is the assert transition,
// exactly like the pin's falling-edge synchronizer.
static void cmdlevel_eval(Mad2 *m) {
    if (g_cmdlevel < 0) g_cmdlevel = dsp54_faithful("DSP54_CMDLEVEL");
    if (!g_cmdlevel || !g_dsp || !g_dsp_run) return;
    port_lazy();
    uint16_t p1   = dsp54_hpi_read(g_dsp, g_port1_addr);
    uint16_t p2   = dsp54_hpi_read(g_dsp, g_port2_addr);
    uint16_t tail = dsp54_hpi_read(g_dsp, 0x000100A4u);   // MDISND tail (MCU advances)
    uint16_t head = dsp54_hpi_read(g_dsp, 0x000100A6u);   // MDISND head (DSP advances)
    // bit1 is the glue's ring-state comparator, NOT the latched memory bit (see block comment).
    p1 = (uint16_t)((p1 & ~0x0002u) | ((tail != head) ? 0x0002u : 0u));
    int level = (((uint16_t)~p2 & p1) & HOSTCMD_DISPATCH_MASK) != 0;
    if (level && !g_cmdlevel_state) {
        g_cmdlevel_edges++;
        dsp54_host_interrupt(g_dsp, 18);                  // INT2 -> vec18 -> 0x3598
        if ((g_log || g_commlog > 0) && g_cmdlevel_edges <= 32)
            fprintf(stderr, "[dsp54] CMDLEVEL edge#%llu INT2 p1=0x%04X p2=0x%04X tail=0x%02X "
                    "head=0x%02X @dsp_steps=%lluk\n", g_cmdlevel_edges, p1, p2, tail, head,
                    (unsigned long long)(m->dsp_steps/1000));
    }
    g_cmdlevel_state = level;
}

// === Fake COBBA-GJ codec/RF-interface serial register file ==============================
// COBBA is the baseband↔RF + audio analog ASIC (service manual): A/D-D/A of the I/Q RX/TX
// paths, the analog TXC/AFC to RF + AGC from RF, AND the audio codec (mic/ear, keypad-tone/
// DTMF decode). MAD↔COBBA = a PARALLEL link (high-speed RF signalling) + a SERIAL link (PCM
// audio). The DSP serial link is bit-banged over I/O ports 0x2C (write = 0x10|reg select) and
// 0x2D (read = bit12 ready/busy + 12-bit data); primitive 0x465C, reader 0x4610 (sign-extends
// via 0x4650). **The cmd-0x70 self-test reads COBBA regs 5 & 6 as 12-bit ADC values**
// (0x4AEA/0x4AF5) — analog measurements (AGC/AFC/reference) — and the MCU validator range-
// checks them. With no RF those read a nominal reference; a faithful COBBA returning in-band
// values makes the self-test PASS organically (the real CONTACT SERVICE fix — SELFTEST_MEAS
// patched the OUTPUT, this models the INPUT). Register-level model: latch addr from 0x2C,
// return the reg file on 0x2D with bit12=0 (ready, so the bring-up spin at 0x45F8 exits).
// === COBBA serial protocol (RE'd 2026-06-11, dsp_prom.s54 0x465C/0x4610/0x4604) ===========
// The DSP↔COBBA serial link is a register-addressed transfer over I/O ports 0x2C (select) and
// 0x2D (data + bit12 busy/ready flag). Decoded from the silicon:
//
//   0x465C  READ primitive   : poll port 0x2D until bit12 CLEAR (ready); write port 0x2C =
//                              (reg | 0x10)  [bit4 SET = READ]; delay 14; read port 0x2D until
//                              bit12 CLEAR -> A = 12-bit data (bit12 = valid flag).
//   0x4604  WRITE primitive  : write port 0x2D = data; clear bit4 of the select word; write
//                              port 0x2C = (reg & ~0x10)  [bit4 CLEAR = WRITE].
//   0x4610  reader           : sets port0 |= 0x0C (COBBA chip-enable); [0x21A1]&2 picks SINGLE
//                              (0x462A: one 0x465C) vs MULTI (0x462E: handshake via STATUS reg
//                              0x0D — wait (reg0xD & 3)==0, send, wait (reg0xD & 0x0C)==0x0C,
//                              then read reg); 0x4650 sign-extends the 12-bit value (bit11).
//
// So: port 0x2C low nibble = register address, bit4 = R/W (1 read / 0 write). The 12-bit ADC
// value comes back on port 0x2D with bit12 always reading 0 here (instant ready -> the bring-up
// spin 0x45F8 and the 0x465C/0x466F polls all exit immediately). The self-test measurement
// (0x4AE1) reads regs 0 (enable), 5, 6 (the 12-bit AGC/AFC/reference ADC values), 0 (disable).
// The MULTI path also transfers STATUS reg 0x0D, which must read (b&3)==0 and (b&0x0C)==0x0C
// to clear the handshake -> reg[0x0D] = 0x0C (idle: ready to accept + done).
//
// ⚠ THE REAL BLOCKER WAS A C54x DECODE BUG, NOT THE PORT MODEL. `LD #k8u,B` (E9xx) loaded the
// immediate into B's HIGH word (`v<<16`) in calypso_c54x.c — so `B=#5` at 0x4AE9 left BL=0, and
// 0x465C's `port(2ch)=*(BL)` selected reg0 instead of reg5/reg6 (every self-test ADC read hit
// reg0). Fixed (right-justified load) — reg5/reg6 now read organically: reg5=0x160 / reg6=0x010
// (the nominal no-signal pass vector) flow into the subcmd-0x13 measurement report (header 0x340E,
// MCU validator 0x258968).
//
// LIMITATION (un-paged overlay): COBBA does NOT fully replace DSP54_SELFTEST_MEAS for the through-
// boot. The report that gates CONTACT SERVICE / reason-4 is the *subcmd-0x16* 52-byte report
// (header 0x3532, validator 0x258B60). Its builder 0x4B73 copies 11 words from AR2 = the MDISND
// command ring, which `call 0x250b` should repoint to a real measurement buffer — but 0x250b is a
// RESIDENT NO-OP STUB in our image (the acquisition overlay is never demand-paged in). So the 0x16
// report reads ring garbage (verified C9F4 CD44 … at 0x4B86) regardless of COBBA; COBBA's reg5/reg6
// feed only the 0x13/0x258968 report. DSP54_SELFTEST_MEAS still stages the 0x16 report's wire bytes
// directly. Closing this last gap needs the 0x250b acquisition overlay block (not on hand) — the
// same gap as "Modelling option (B)". COBBA + SELFTEST_MEAS
// coexist (PASS to "Security code"); COBBA-only (MEAS=0) -> reason-4 on the 0x16 ring report.
#define COBBA_NREG 16
static struct {
    int      init;
    uint16_t reg[COBBA_NREG];   // 12-bit COBBA register file (analog measurements / control)
    uint16_t addr;              // last addressed register (low nibble of port 0x2C)
    int      rw;                // last transfer direction (1 = read, 0 = write)
} g_cobba;
static int g_cobbalog = -1;     // DSP54_COBBALOG: per-transfer 0x2C select + 0x2D value + DSP PC
static uint16_t g_cobba_p2d_last;// last value written to port 0x2D (COBBA write data latch)

static void cobba_lazy(void) {
    if (g_cobba.init) return;
    g_cobba.init = 1;
    // Nominal no-signal self-test ADC defaults. The cmd-0x70 self-test reads reg5 then reg6 and
    // packs them (0x4AEA/0x4AF5) into the measurement report; the MCU validator (0x258B60) range-
    // checks the resulting buf[12,13]/buf[20,21] -> the low-band accept needs the *builder source*
    // word[0]=0x0010, word[4]=0x0160. Default reg
    // values + STATUS reg 0x0D = 0x0C (handshake idle). Override any reg via DSP54_COBBA_REG<n>.
    g_cobba.reg[5] = 0x0160; g_cobba.reg[6] = 0x0010;
    g_cobba.reg[0x0D] = 0x000C;
    for (int i = 0; i < COBBA_NREG; i++) {
        char k[24]; snprintf(k, sizeof k, "DSP54_COBBA_REG%d", i);
        const char *v = getenv(k); if (v && *v) g_cobba.reg[i] = (uint16_t)(strtol(v,0,0) & 0xFFFF);
    }
    if (g_cobbalog < 0) g_cobbalog = getenv("DSP54_COBBALOG") ? 1 : 0;
}

// === COBBA PARALLEL (the MAD "MFI" block — Interface to COBBA AD/DA) =======================
// RE: docs/research/cobba-. Two halves:
//   CONTROL — the DSP writes mmr(22h) (primary) / mmr(32h) (INT-masked variant); every frame is
//     0xC0xx = 4-bit COBBA register ADDRESS (high nibble) + 12-bit DATA (the service manual's
//     "parallel bus [with] 12 data bits, 4 address bits"). Delivered via the core hook
//     g_c54x_mfi_write_cb (calypso_c54x.c data_write_locked) -> decoded into g_cobba_par.ctrl[].
//     Exercised ORGANICALLY at boot: the run-mode reset-init writes mmr(22h)=0xC008 @0x301F.
//   DATAPATH — DSP I/O ports 0x30-0x3C + the 14-bit AGC ADC on port(0Dh):
//     0x38 R = bus status/handshake (bit7 = busy/data-available; the poll 0x41AD spins while
//              bit7 SET -> model returns READY, bit7 clear)
//     0x39 R = RX I/Q burst samples (no RF -> silence; DSP54_COBBA_IQ=<hex> injects a constant)
//     0x3A/0x3B R = AGC/measurement readback A/B (0x4150 bias-folds them; nominal defaults)
//     0x0D R = 14-bit AGC ADC (0x44DC double-read, mask 0x3FFF @0x44DF; read by the closed-loop
//              power control before every TXC/AFC DAC pair and by self-test sub-0x14 0x4B1F)
//     0x30-0x3C W = latched (TXC/AFC DAC pair 0x31/0x32, strobes 0x3C, mode 0x30/0x34) for
//              logging/inspection. Write-sinks at no-RF standby (canonical doc peripheral map).
// Enable: DSP54_COBBAPAR=1, or implied by DSP54_COBBA (one chip; =0 opts the parallel half out).
// Default OFF -> behaviour identical to the historical write-sink model.
#define COBBA_PAR_NCTL 16
static struct {
    int      init;
    int      on;                       // resolved enable (COBBAPAR / COBBA)
    uint16_t ctrl[COBBA_PAR_NCTL];     // COBBA parallel control register file (12-bit data)
    uint8_t  ctrl_seen[COBBA_PAR_NCTL];// which addresses the firmware actually wrote
    uint16_t latch[16];                // last write per datapath port 0x30..0x3F
    unsigned nwr_ctrl, nwr_port;       // write counters (control frames / datapath ports)
    uint16_t agc;                      // port(0Dh) 14-bit AGC ADC nominal
    uint16_t meas_a, meas_b;           // port(3Ah)/port(3Bh) measurement readback
    uint16_t iq;                       // port(39h) RX sample value (0 = silence)
} g_cobba_par;

static void cobba_par_lazy(void) {
    if (g_cobba_par.init) return;
    g_cobba_par.init = 1;
    const char *p = getenv("DSP54_COBBAPAR");
    g_cobba_par.on = (p && *p) ? atoi(p) : dsp54_faithful("DSP54_COBBA");
    // Nominal no-signal AGC: same scale family as the serial probe's reg5 nominal (0x160 —
    // the validator-accepted no-signal vector). 14-bit field, override DSP54_COBBA_AGC.
    g_cobba_par.agc = 0x0160;
    { const char *v = getenv("DSP54_COBBA_AGC");   if (v && *v) g_cobba_par.agc    = (uint16_t)(strtol(v,0,0) & 0x3FFF); }
    { const char *v = getenv("DSP54_COBBA_MEASA"); if (v && *v) g_cobba_par.meas_a = (uint16_t)(strtol(v,0,0) & 0x0FFF); }
    { const char *v = getenv("DSP54_COBBA_MEASB"); if (v && *v) g_cobba_par.meas_b = (uint16_t)(strtol(v,0,0) & 0x0FFF); }
    { const char *v = getenv("DSP54_COBBA_IQ");    if (v && *v) g_cobba_par.iq     = (uint16_t)strtol(v,0,0); }
    if (g_cobbalog < 0) g_cobbalog = getenv("DSP54_COBBALOG") ? 1 : 0;
}

// Core hook: a COBBA parallel CONTROL frame left the DSP on mmr 0x22/0x32.
static void cobba_mfi_write(uint16_t mmr, uint16_t val) {
    cobba_par_lazy();
    if (!g_cobba_par.on) return;                  // unmodelled: core keeps the RAM-backed store
    uint16_t a = (uint16_t)((val >> 12) & 0xF), d = (uint16_t)(val & 0x0FFF);
    g_cobba_par.ctrl[a] = d;
    g_cobba_par.ctrl_seen[a] = 1;
    g_cobba_par.nwr_ctrl++;
    if (g_cobbalog)
        fprintf(stderr, "[cobba] PAR ctrl mmr(%02Xh)<=0x%04X  reg[0x%X]=0x%03X  dsp_pc=0x%04X (#%u)\n",
                mmr, val, a, d, comm_dsp_pc(), g_cobba_par.nwr_ctrl);
}

// PERIPHMAP structural "modeled?" flag (calypso_c54x.c records it at the PORTR/PORTW
// sites): set 1 ONLY where a real model produced/consumed the value — bare-knob
// constants and the default rv=0 fall-through stay 0 (= "we silently fake this").
extern int g_c54x_pmap_handled;

static uint16_t c54x_port_read(void *opaque, uint16_t pa) {
    Mad2 *m = (Mad2*)opaque;
    port_lazy();
    if (!g_p2d_init) {
        g_p2d_init = 1;
        const char *v = getenv("DSP54_P2D");       if (v && *v) g_p2d_val = (uint16_t)strtol(v, 0, 0);
        g_p2d_toggle = getenv("DSP54_P2DTOGGLE") ? 1 : 0;
    }
    if (!g_p27_init) { g_p27_init = 1;
        const char *v = getenv("DSP54_P27"); if (v && *v) { g_p27_on = 1; g_p27_val = (uint16_t)strtol(v, 0, 0); } }
    uint16_t rv = 0;
    // Fake COBBA codec input: while DSP54_COBBA is on, the codec serial handshake (port 0x2D)
    // reports READY (bit12) and the ADC/sample ports (0x21 read, 0x27 FIFO) return the COBBA
    // input — silence (0) by default, DSP54_COBBA_IN=<hex> to inject a constant test sample
    // (e.g. a mic level). The DAC side (0x21 WRITE) is captured by PCMCAP in c54x_port_write.
    static int cob = -1, cob_in_init = 0; static uint16_t cob_in = 0; static uint16_t cob_2d = 0;
    if (cob < 0) cob = dsp54_faithful("DSP54_COBBA");
    if (cob && !cob_in_init) { cob_in_init = 1; const char *v = getenv("DSP54_COBBA_IN");
        if (v && *v) cob_in = (uint16_t)strtol(v, 0, 0); }
    if (pa == 0x2D) {
        if (cob) { cobba_lazy();                                // COBBA serial: bit12=0 (ready) + reg value
            // bit12 always reads 0 (instant ready) so the 0x465C/0x466F/0x45F8 busy-polls exit;
            // the low 12 bits are the addressed register's value (the 12-bit ADC reading).
            rv = (uint16_t)(g_cobba.reg[g_cobba.addr & (COBBA_NREG-1)] & 0x0FFF);
            g_c54x_pmap_handled = 1;
            if (g_cobbalog)
                fprintf(stderr, "[cobba] R reg[0x%02X] -> 0x%03X  dsp_pc=0x%04X\n",
                        g_cobba.addr & (COBBA_NREG-1), rv, comm_dsp_pc());
        }
        else { rv = g_p2d_toggle ? (g_p2d_state ^= 0x1000) : g_p2d_val;
               if (g_p2d_toggle || g_p2d_val) g_c54x_pmap_handled = 1; }   // explicit knob only
        (void)cob_2d;
    }
    else if (pa == 0x27) { rv = cob ? cob_in : (g_p27_on ? g_p27_val : 0);   // codec RX FIFO
        if (cob || g_p27_on) g_c54x_pmap_handled = 1; }
    else if (pa == 0x21 && cob) { rv = cob_in; g_c54x_pmap_handled = 1; }   // codec ADC sample (mic in)
    // COBBA parallel datapath reads (MFI; see the g_cobba_par block). All return 0 with the
    // model OFF — identical to the historical unmodelled behaviour.
    else if (pa == 0x0D) { cobba_par_lazy();                    // 14-bit AGC ADC (0x44DC reader)
        if (g_cobba_par.on) { rv = (uint16_t)(g_cobba_par.agc & 0x3FFF);
            g_c54x_pmap_handled = 1;
            if (g_cobbalog) fprintf(stderr, "[cobba] PAR AGC port(0Dh) -> 0x%04X  dsp_pc=0x%04X\n",
                                    rv, comm_dsp_pc()); } }
    else if (pa == 0x38) { cobba_par_lazy();                    // bus status: bit7 busy -> READY
        rv = 0; if (g_cobba_par.on) g_c54x_pmap_handled = 1; }
    else if (pa == 0x39) { cobba_par_lazy();                    // RX I/Q burst sample
        if (g_cobba_par.on) { rv = g_cobba_par.iq; g_c54x_pmap_handled = 1; } }
    else if (pa == 0x3A) { cobba_par_lazy();                    // AGC/meas readback A
        if (g_cobba_par.on) { rv = (uint16_t)(g_cobba_par.meas_a & 0x0FFF); g_c54x_pmap_handled = 1; } }
    else if (pa == 0x3B) { cobba_par_lazy();                    // AGC/meas readback B
        if (g_cobba_par.on) { rv = (uint16_t)(g_cobba_par.meas_b & 0x0FFF); g_c54x_pmap_handled = 1; } }
    // Host-cmd mailbox ports: on MAD2 the DSP's ports 0-3 ARE the MCU 0x100xx window mailbox
    // (docs/archive/5110-dsp/dsp-). Under the single-store model the window IS
    // api_ram — m->mem at those addresses is an invisible shadow (MCU reads route to api_ram),
    // so both port READS and port WRITES must use the api_ram cell or the conversation splits
    // (the audit class: the DSP's version/checksum reply landed where the MCU never looks ->
    // DSP ROM MISMATCH verdict -> CONTACT SERVICE).
    else if (pa == 0x01 && m) { rv = g_dsp ? dsp54_hpi_read(g_dsp, g_port1_addr) : mcu_word(m, g_port1_addr); g_c54x_pmap_handled = 1; }
    else if (pa == 0x02 && m) { rv = g_dsp ? dsp54_hpi_read(g_dsp, g_port2_addr) : mcu_word(m, g_port2_addr); g_c54x_pmap_handled = 1; }
    else if (pa == 0x03 && m) { rv = g_dsp ? dsp54_hpi_read(g_dsp, g_port3_addr) : mcu_word(m, g_port3_addr); g_c54x_pmap_handled = 1; }
    if (g_log) fprintf(stderr, "[dsp54] PORTR pa=0x%02X -> 0x%04X\n", pa, rv);
    // The cmd ISR (0x35B9) computes cmd=(~port2)&port1 from these reads — log them with the
    // DSP PC so each ISR invocation's dispatched bitmask is visible (capped: idle may poll).
    if (g_commlog < 0) g_commlog = getenv("DSP54_COMMLOG") ? atoi(getenv("DSP54_COMMLOG")) : 0;
    if (g_commlog && (pa == 0x01 || pa == 0x02 || pa == 0x03)) {
        static unsigned nrd; if (++nrd <= 256)
            fprintf(stderr, "[comm] @%-7lluk D->R PORTR pa=0x%02X -> 0x%04X (host cmd read)          dsp_pc=0x%04X\n",
                    (unsigned long long)(m->dsp_steps/1000), pa, rv, comm_dsp_pc());
    }
    return rv;
}
// === DSP54_PCMCAP — tap the DSP's COBBA codec output (port 0x21 DAC) ======================
// ARCHITECTURE (RE'd 2026-06-11 eve): MAD2 audio is NOT a C54x McBSP/serial-MMR path — the
// transmit interrupt vectors (0xFFD4/D8/DC) are STUBS (return_fast), so there is no XINT-driven
// serial transmit. Audio is the COBBA codec over I/O-PORT 0x21 (bidirectional: read = mic ADC,
// write = earpiece DAC), driven per codec frame by the vec16 ISR (0x3204): read input, process
// (FIR or the tone oscillator 0xA598/0xA5E8), write the output sample to port 0x21. (mmr 0x22 =
// COBBA CONTROL regs, 0xC0xx — not samples.) So tapping pa 0x21 writes IS the right DAC tap.
// CAVEAT: at standby the vec16 ISR takes its inactive/drain branch and writes only the frame-
// sync control word (A|0x0C00 = the constant 3072,0,... pattern) — real samples appear only
// when (a) vec16 runs at the codec frame rate (DSP54_FRAMEINT=16) AND (b) the codec/tone mode
// is active ([0xAA]/[0xAC] set + the [0x856]-enabled oscillator). DSP54_PCMCAP=<path> appends
// each pa 0x20/0x21 write as host-endian int16; summary = peak + zero-crossing freq est @8kHz.
static struct {
    int      init;
    FILE    *f1, *f0;            // 0x21 (primary), 0x20 (channel 0)
    long     n;                  // samples on ch1
    int16_t  prev;               // last ch1 sample (zero-cross detect)
    long     zc;                 // zero crossings since last summary
    long     win;                // summary window (samples)
    int      pk_lo, pk_hi;       // amplitude extremes in window
    double   rate;               // assumed codec sample rate (Hz)
} g_pcm;

static void pcm_capture(Mad2 *m, uint16_t pa, uint16_t val) {
    if (!g_pcm.init) {
        g_pcm.init = 1;
        const char *p = getenv("DSP54_PCMCAP");
        if (p && *p) {
            g_pcm.f1 = fopen(p, "wb");
            char ch0[512]; snprintf(ch0, sizeof ch0, "%s.ch0", p);
            g_pcm.f0 = fopen(ch0, "wb");
            const char *w = getenv("DSP54_PCMWIN"); g_pcm.win = (w && *w) ? strtol(w, 0, 0) : 800;
            if (g_pcm.win < 1) g_pcm.win = 800;
            const char *r = getenv("DSP54_PCMRATE"); g_pcm.rate = (r && *r) ? strtod(r, 0) : (double)DCT3_CODEC_HZ;
            g_pcm.pk_lo = 32767; g_pcm.pk_hi = -32768;
            fprintf(stderr, "[dsp54] PCMCAP on: ch1(pa0x21)->%s ch0(pa0x20)->%s.ch0  win=%ld rate=%.0fHz\n",
                    p, p, g_pcm.win, g_pcm.rate);
        }
    }
    int16_t s = (int16_t)val;
    // HAL PCM channel (mad2.h pcm_sink): deliver every sample to the host,
    // whether or not the PCMCAP file capture is armed.
    if (m->pcm_sink && (pa == 0x20 || pa == 0x21)) {
        // First codec sample = the codec TAKES the sink: set the producer rate uncondition-
        // ally (the HLE mixer may have published its own 48 kHz rate up to this point).
        if (!m->pcm_codec_seen) m->pcm_rate = g_pcm.rate ? g_pcm.rate : (double)DCT3_CODEC_HZ;
        m->pcm_codec_seen = 1;   // codec owns the sink this run -> the buzzer mixer stands down
        m->pcm_sink(m, pa == 0x21 ? 1 : 0, s);
    }
    if (!g_pcm.f1) return;
    if (pa == 0x20) { if (g_pcm.f0) fwrite(&s, 2, 1, g_pcm.f0); return; }
    if (pa != 0x21) return;
    fwrite(&s, 2, 1, g_pcm.f1);
    g_pcm.n++;
    if ((g_pcm.prev < 0) != (s < 0)) g_pcm.zc++;   // sign change = zero crossing
    g_pcm.prev = s;
    if (s < g_pcm.pk_lo) g_pcm.pk_lo = s;
    if (s > g_pcm.pk_hi) g_pcm.pk_hi = s;
    if ((g_pcm.n % g_pcm.win) == 0) {
        // freq ≈ (zero_crossings / 2) / (window / rate)
        double hz = (g_pcm.zc / 2.0) * g_pcm.rate / (double)g_pcm.win;
        fprintf(stderr, "[dsp54] PCMCAP @%-7lluk ch1 n=%ld  peak=[%d,%d]  ~%.0f Hz (zc=%ld/%ld)\n",
                (unsigned long long)(m->dsp_steps/1000), g_pcm.n, g_pcm.pk_lo, g_pcm.pk_hi, hz,
                g_pcm.zc, g_pcm.win);
        g_pcm.zc = 0; g_pcm.pk_lo = 32767; g_pcm.pk_hi = -32768;
    }
}

static void c54x_port_write(void *opaque, uint16_t pa, uint16_t val) {
    Mad2 *m = (Mad2*)opaque;
    port_lazy();
    if (g_log) fprintf(stderr, "[dsp54] PORTW pa=0x%02X val=0x%04X\n", pa, val);
    if (pa == 0x20 || pa == 0x21) { pcm_capture(m, pa, val);   // codec DXR -> PCM tap
        if (g_pcm.f1) g_c54x_pmap_handled = 1; }               // consumed only when PCMCAP is on
    // COBBA serial register-select: the select word on port 0x2C is (reg | 0x10) for a READ
    // (0x465C) or (reg & ~0x10) for a WRITE (0x4604). Low nibble = register address, bit4 = R/W.
    // Latch addr + direction; a WRITE select commits the value last written to port 0x2D into
    // the register file (faithful: 0x4604 wrote port 0x2D=data, then port 0x2C=reg&~0x10).
    { static int cob = -1; if (cob < 0) cob = dsp54_faithful("DSP54_COBBA");
      if (cob && pa == 0x2C) { cobba_lazy();
          g_cobba.addr = (uint16_t)(val & 0x0F);
          g_cobba.rw   = (val & 0x10) ? 1 : 0;
          if (!g_cobba.rw) g_cobba.reg[g_cobba.addr] = g_cobba_p2d_last;   // WRITE: data from 0x2D
          g_c54x_pmap_handled = 1;
          if (g_cobbalog)
              fprintf(stderr, "[cobba] %c sel 0x2C<=0x%02X reg=0x%02X%s  dsp_pc=0x%04X\n",
                      g_cobba.rw ? 'R' : 'W', val & 0xFF, g_cobba.addr,
                      g_cobba.rw ? "" : " (write)", comm_dsp_pc());
      }
      // capture the last value written to port 0x2D (the COBBA write data, latched before the
      // 0x2C write-select commits it).
      if (cob && pa == 0x2D) { cobba_lazy(); g_cobba_p2d_last = val; g_c54x_pmap_handled = 1; } }
    // COBBA parallel datapath write latch (ports 0x30-0x3C: TXC/AFC DAC pair 0x31/0x32, TX
    // burst 0x39, strobes 0x3C, mode/rate 0x30/0x34). Latch + capped log; the store itself
    // remains a sink (no RF model consumes it) so OFF==ON behaviourally except inspection.
    if (pa >= 0x30 && pa <= 0x3F) { cobba_par_lazy();
        if (g_cobba_par.on) {
            g_cobba_par.latch[pa - 0x30] = val;
            g_cobba_par.nwr_port++;
            g_c54x_pmap_handled = 1;
            if (g_cobbalog && g_cobba_par.nwr_port <= 64)
                fprintf(stderr, "[cobba] PAR port(%02Xh)<=0x%04X  dsp_pc=0x%04X (#%u)\n",
                        pa, val, comm_dsp_pc(), g_cobba_par.nwr_port);
        } }
    if (g_commlog < 0) g_commlog = getenv("DSP54_COMMLOG") ? atoi(getenv("DSP54_COMMLOG")) : 0;
    if (g_commlog && (pa == 0x01 || pa == 0x02 || pa == 0x03))
        fprintf(stderr, "[comm] @%-7lluk M<-D PORT%-10u pa=0x%02X <= 0x%04X (DSP->host mailbox)   dsp_pc=0x%04X\n",
                (unsigned long long)(m->dsp_steps/1000), pa, pa, val, comm_dsp_pc());
    // Accept-mask/reply path: the ISR manages port2 (the DSP's cmd ACCEPT MASK — handlers
    // close their bit, the dequeue disarm reopens bit1; the version reply value also rides
    // here). Land it in the SINGLE STORE (api_ram window cell) — that is what the next
    // cmd=(~port2)&port1 computation and the MCU read.
    if (pa == 0x02 && m) {
        g_c54x_pmap_handled = 1;
        if (g_dsp) dsp54_hpi_write(g_dsp, g_port2_addr, val);
        else mcu_set_word(m, g_port2_addr, val);
        // pa2 = the in-service/mask register: marking a request bit in-service ACCEPTS it,
        // so the glue clears it from the pa1 latch (same ack-clear contract as the pa1
        // status write below). This is what retires NO-OP requests (bit0's handler 0x362D
        // is literally `goto exit` — its only effect is riding the exit pulse port2=0xFFFF,
        // which under this contract is a global EOI of everything the DSP has seen).
        // Without it bit0 stays latched-and-unmasked after its no-op dispatch -> the exit
        // pulse re-edges INT2 forever -> ISR storm starves the main loop (~309k entries/8M,
        // MDISND never drains, self-test times out, 2026-06-12). bit1 excluded as always
        // (hardware ring-state comparator, not a latch).
        if (g_dsp) {
            if (g_cmdlevel < 0) g_cmdlevel = dsp54_faithful("DSP54_CMDLEVEL");
            if (g_cmdlevel) {
                uint16_t req = dsp54_hpi_read(g_dsp, g_port1_addr);
                uint16_t clr = (uint16_t)(val & (HOSTCMD_DISPATCH_MASK & ~0x0002u));
                if (req & clr) dsp54_hpi_write(g_dsp, g_port1_addr, (uint16_t)(req & ~clr));
            }
        }
        cmdlevel_eval(m);   // mask changed: the ISR's ack + exit pulse-restore re-edge here
    }
    // pa1 WRITES are the DSP->MCU STATUS latch, physically separate from the pa1 READ register
    // (= the MCU's command word at g_port1_addr): the boot report (0x8DA8 port1=0x0700), the
    // 0x1C-group handler (0x362F port1=0x001C) and the version handler (0x8EDF port1=0x0200)
    // would all CLOBBER the in-flight command if landed on the read cell. Route to the opt-in
    // asymmetric target (DSP54_PORT1WADDR) or drop — never to g_port1_addr.
    if (pa == 0x01 && m) g_c54x_pmap_handled = 1;   // pa1 status-latch contract (ack/doorbell below)
    if (pa == 0x01 && m && !g_dsp) mcu_set_word(m, g_port1_addr, val);   // legacy stub path only
    // Req/ack handshake completion (CMDLEVEL, 2026-06-11 eve): the DSP's pa1 STATUS write
    // is the ACK for the request bits it just served (version 0x8EDF -> 0x0200, task-group
    // 0x362F -> 0x001C, boot report -> 0x0700, ...) — the glue clears those bits in the MCU
    // command word so the INT2 level drops. Without this, any handler that (unlike the bit1
    // arm 0x3621) does not mask its own port2 bit leaves an unmasked latched request, and
    // the ISR's exit pulse (port2 0xFFFF->restore) re-edges INT2 forever: an ISR storm of
    // PORT2/PORT1 writes at dsp_pc 0x35FF/0x3605/0x8EDF (~820k log lines by 17M, observed).
    // bit1 is excluded — it is the hardware ring-state comparator, not a latched request.
    if (pa == 0x01 && m && g_dsp) {
        if (g_cmdlevel < 0) g_cmdlevel = dsp54_faithful("DSP54_CMDLEVEL");
        if (g_cmdlevel) {
            uint16_t req = dsp54_hpi_read(g_dsp, g_port1_addr);
            uint16_t clr = (uint16_t)(val & (HOSTCMD_DISPATCH_MASK & ~0x0002u));
            if (req & clr) dsp54_hpi_write(g_dsp, g_port1_addr, (uint16_t)(req & ~clr));
        }
        cmdlevel_eval(m);
    }
    // DSP54_FIQ0OUT=1: the FAITHFUL DSP->MCU ring-post interrupt. The MDIRCV enqueue
    // (PROM 0x37CE) signals the MCU with `port(1h) = 1` at 0x3807 after writing a ring
    // entry + advancing the tail; the MCU's FIQ0 handler (0x2EE9B6 -> 0x2C2824) drains
    // head->tail. Raise FIQ0 on exactly that signal — the same line dsp_default raises for
    // its MODELLED ring (dsp_default.c:226). Default OFF + env-gated. NOTE: gated downstream
    // of a real blocker — the DSP's task bitmap [0x6E4] is 0 (the enable handlers in the
    // 0x8Exx host-cmd region never fire because the MCU's task-enable commands are absorbed
    // by the HLE), so the enqueue never runs and port1 never sees val=1 yet. This lands the
    // correct wiring; the unlock is retiring the HLE so the real DSP gets those commands.
    // NOTE: the real DSP writes port(1)=0x0100 @PC 0xB7E1 under REALUP, but it is NOT
    // a validated MDIRCV doorbell — the real DSP never advances the MDIRCV tail at boot (0x101C8
    // stays 0x80 with the HLE off), so wiring FIQ0 to it raises a spurious interrupt on an empty
    // ring. Kept as the legacy port(1)=1 wiring only (DSP54_FIQ0OUT, default OFF). The real DSP->MCU
    // ring delivery is an open gap (the producer/scheduler that posts MDIRCV is dormant at idle).
    // CORRECTED 2026-06-10: the enqueue's signal `portw *(0x8),pa1` @0x3807 writes a SCRATCH
    // cell (don't-care value; the `ld #1,a` before it is the enqueue's return value, not the
    // port value — observed live value 0x0000). So the pa1 WRITE EVENT itself is the doorbell
    // line: any DSP port1 write raises the DSP->MCU interrupt and the MCU's FIQ0 handler
    // drains MDIRCV head->tail (empty ring = benign wake, e.g. the 0x9077/0x395B status
    // writes). Trigger on the event, not the value.
    if (pa == 0x01 && m) {
        static int f0_on = -1;
        if (f0_on < 0) { const char *e = getenv("DSP54_FIQ0OUT"); f0_on = (e && *e) ? atoi(e) : 0; }
        // mode 1 = every pa1 write; mode 2 = only the MDIRCV enqueue signal (PC 0x3807) —
        // A/B for whether the boot-time status writes (0x9077/0x395B) raise spurious FIQ0s.
        if (f0_on == 1 || (f0_on == 2 && comm_dsp_pc() == 0x3807)) {
            mad2_raise_fiq(m, 0);                              // FIQ0: DSP pa1 doorbell
            if (g_log) { static unsigned n; if (++n <= 16)
                fprintf(stderr, "[dsp54] FIQ0-OUT #%u: DSP port(1) write (val=0x%04X) -> FIQ0\n", n, val); }
        }
    }
    // ASYMMETRIC mailbox test (DSP54_PORT{1,2}WADDR): some MAD2 mailbox HW latches port READS
    // and port WRITES to DIFFERENT MCU addresses (cmd-in vs status-out). If set, ALSO write the
    // port value to the write-address (e.g. the firmware's version slots 0x10004/0x10006).
    if (pa == 0x02 && m && g_port2_waddr) mcu_set_word(m, g_port2_waddr, val);
    if (pa == 0x01 && m && g_port1_waddr) mcu_set_word(m, g_port1_waddr, val);
}

static void c54x_lazy_init(Mad2 *m) {
    if (g_state) return;
    static int parsed = 0;
    if (!parsed) {
        parsed  = 1;
        g_cosim = getenv("DSP54_COSIM") ? 1 : 0;
        g_log   = getenv("DSP54_LOG") ? 1 : 0;
        g_realup = dsp54_faithful("DSP54_REALUP");   // faithful staged boot upload (default ON under cosim)
        const char *r = getenv("DSP54_RATIO");
        if (r && *r) { int v = atoi(r); if (v > 0) g_ratio = v; }
        else if (g_cosim) g_ratio = 4;               // faithful default — boots loader1 to run-mode (doc Recipe C)
        const char *v = getenv("DSP54_INTVEC");
        if (v && *v) g_intvec = (int)strtol(v, 0, 0);
        else if (g_realup) g_intvec = 25;  // [0x20008] bit1 -> vec 25 (0x3772: work flag
        // [0x866]|=2 + [0x870] mailbox + HINT ack). Vector-table ground truth (0xFF80
        // dump, 2026-06-10): vec 17/18/24 -> 0x3598 host-cmd ISR (only INT2/vec18
        // unmasked in IMR=0x52FD), vec 25 -> 0x3772. OPEN QUESTION (do not re-land
        // blindly): a "unified doorbell" (0x20008 bit1 -> vec18+vec25) lets ring posts
        // re-arm the dequeue (the disarm 0x3960/0x3969 reopens port2 bit1 for exactly
        // that) BUT the extra vec-25 wake at run-mode entry drives the scheduler's
        // message-copy (0x3EEE) on an EMPTY internal queue -> len 0 -> BRC=(0+1)/2-1
        // underflows to 0xFFFF -> 64K-word rptb smears DARAM 0x1200..0x25xx (speed-
        // invariant, RATIO 2/4/8). Candidate missing piece: the second doorbell ISR
        // 0x38B5 (checks [0x872]+[0x870]) which is NOT in the 0xFF80 vector table.
        const char *ir = getenv("DSP54_INTREPEAT"); if (ir && *ir) { int x = atoi(ir); if (x > 0) g_intrepeat = x; }
        const char *fi = getenv("DSP54_FRAMEINT"); if (fi && *fi) g_frameint = (int)strtol(fi, 0, 0);
        const char *fp = getenv("DSP54_FRAMEPER"); if (fp && *fp) { int x = atoi(fp); if (x > 0) g_frameper = x; }
        const char *fa = getenv("DSP54_FRAMEAFTER"); if (fa && *fa) g_frameafter = strtoull(fa, 0, 0);
    }
    const char *path = getenv("DSP54_IMAGE");
    if (!path || !*path) path = DSP_IMAGE_DEFAULT;
    FILE *f = fopen(path, "rb");
    if (!f) {
        if (g_log) fprintf(stderr, "[dsp54] image '%s' not found -> legacy pass-through\n", path);
        g_state = -1;
        return;
    }
    static uint8_t img[2 * C54X_DATA_SIZE];
    size_t nb = fread(img, 1, sizeof img, f);
    fclose(f);
    g_dsp = dsp54_create(img, nb);
    if (!g_dsp) { g_state = -1; return; }
    dsp54_set_port_io(g_dsp, c54x_port_read, c54x_port_write, m);
    // COBBA parallel CONTROL frames (mmr 0x22/0x32) — core hook into the MFI model above.
    // Registered unconditionally; the callback itself gates on DSP54_COBBAPAR/DSP54_COBBA.
    { extern void (*g_c54x_mfi_write_cb)(uint16_t mmr, uint16_t val);
      g_c54x_mfi_write_cb = cobba_mfi_write; }
    g_state = 1;
    // DSP54_DSPHALT=0xPC [+DSP54_DSPHALTN=N, default 1]: freeze the DSP just BEFORE the N-th time it
    // reaches PC, then dump the FULL live memory map (.prog/.api/.data/.regs) for offline disasm
    // (caught in c54x_tick). Lets us snapshot what the DSP is REALLY executing instead of inferring.
    { const char *h = getenv("DSP54_DSPHALT");
      if (h && *h) { uint32_t after = 1; const char *n = getenv("DSP54_DSPHALTN");
          if (n && *n) { long vv = atoi(n); if (vv > 0) after = (uint32_t)vv; }
          long pc = strtol(h, 0, 0);
          dsp54_set_halt(g_dsp, (uint16_t)pc, after);
          if (g_log) fprintf(stderr, "[dsp54] DSPHALT armed: freeze at PC=0x%04lX hit #%u\n", pc & 0xFFFF, after); } }
    // DSP54_DSPHALTSP=0xNN: freeze + dump the moment SP drops below NN (catches the over-pop that
    // drives the stack into the MMR region). Same dump path as DSP54_DSPHALT.
    { const char *sp = getenv("DSP54_DSPHALTSP");
      if (sp && *sp) { long fl = strtol(sp, 0, 0); dsp54_set_halt_sp(g_dsp, (uint16_t)fl);
          if (g_log) fprintf(stderr, "[dsp54] DSPHALTSP armed: freeze when SP < 0x%04lX\n", fl & 0xFFFF); } }
    // DSP54_UNIDARAM: faithful C54x OVLY + HPI dual-port — collapse the window [0x800,0x2800) to a
    // SINGLE store (api_ram) shared by program-fetch, DSP data access, and the MCU HPI window, so an
    // OVLY program address and its data address are ONE cell (SPRU131G §3/§8; docs/dsp-c54x-memory-
    //). Without it, fetch reads data while loader1 scatters into api_ram → uploaded code
    // never executes. Faithful (default ON under cosim); supersedes SHAREDWIN/HPIAPIW for the window.
    if (dsp54_faithful("DSP54_UNIDARAM")) {
        dsp54_set_ovly_unified(g_dsp, 1);
        if (g_log) fprintf(stderr, "[dsp54] UNIDARAM: OVLY window [0x800,0x2800) is a single shared store\n");
    }
    // DSP54_TIMER (faithful default ON under cosim): the C54x on-chip timer counts and raises
    // TINT (vec 19) — SPRU131G §8.8: at reset TIM=PRD=0xFFFF, TSS cleared, timer free-runs, so
    // TINT fires every 65536 DSP insns until the firmware reprograms PRD/TDDR. The 5110 run-mode
    // IVT routes 0xFFCC -> the DARAM RTOS-tick dispatcher 0x241A and IMR bit3 is enabled: this
    // timer is the steady-state heartbeat that wakes the `idle 1` loop (0x31A4) so the superloop
    // can run tasks + the MDIRCV producer. Without it the DSP idles deaf forever.
    if (dsp54_faithful("DSP54_TIMER")) {
        dsp54_set_timer(g_dsp, 1);
        if (g_log) fprintf(stderr, "[dsp54] TIMER: on-chip timer armed (TINT vec 19, period (PRD+1)*(TDDR+1) insns)\n");
    }
    // DSP54_REALUP: the DSP is held in reset (g_dsp_run=0) and loader1 is staged into DARAM at the
    // COLD reset-RELEASE edge (see 0x20002 handler in c54x_write), NOT here. Seeding at t=0 was
    // unfaithful: the MCU's HPI window-clear during its own boot (28k-397k) wiped it, forcing a
    // reseed. The faithful HPI boot order is: MCU clears the window -> drops the loader -> releases
    // the DSP. So we seed loader1 exactly at the release edge (2026-06-09; t=0 seed removed).
    // (Manual block-upload knobs DSP54_BLOCKS / DSP54_BLOCKONLY REMOVED 2026-06-17: they
    // loaded extracted re/dsp-5110/blockNN.bin into DSP memory by hand — a debugging crutch
    // from before the firmware drove its own upload faithfully. The MCU now uploads every
    // block itself over the HPI window; there is nothing to pre-load. See the 0x20002
    // reset-release handler + ModelProfile.dsp_hpi_alias_base.)
    if (g_log)
        fprintf(stderr, "[dsp54] loaded %s (%zu bytes) cosim=%d ratio=%d intvec=%d\n",
                path, nb, g_cosim, g_ratio, g_intvec);
    // DSP54_WARMUP=N: step the DSP N instructions up-front so it reaches its host-ready idle
    // loop (0x408B: INTM=0 / idle / wait on data[0x195C].b1) BEFORE the MCU issues the DSP
    // command + DSPINT (~183k MCU steps). The real DSP boots in parallel at full clock and is
    // ready quickly; our per-step ratio-1 pacing only gets it there ~20-25M insns in, far too
    // late. Bootstrapping it here closes the pacing gap so the doorbell lands on an idle DSP.
    if (g_cosim) {
        const char *w = getenv("DSP54_WARMUP");
        long warm = (w && *w) ? strtol(w, 0, 0) : 0;
        if (warm > 0) {
            dsp54_step(g_dsp, (int)warm);
            if (g_log) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                fprintf(stderr, "[dsp54] warmup %ld insns -> pc=0x%04X INTM=%d idle=0x%X "
                        "INT2en=%d\n", warm, st.pc, (st.st1 >> 11) & 1, st.idle, (st.imr >> 2) & 1);
            }
        }
    }
}

// True once the firmware's (dsp_default-modelled) DSP code-block upload has completed
// (it sets dsp_uploaded itself). Until then the boot/upload handshake MUST be served by
// the legacy model so the firmware actually performs its upload (the upload writes are
// mirrored into the C54x core in c54x_write). After upload, optionally switch HPI reads to
// the live DSP (DSP54_LIVE=1) so the firmware sees the real DSP's mailbox replies.
static int upload_done(Mad2 *m) {
    return m->mem && m->fw.dsp_uploaded &&
           m->mem[m->fw.dsp_uploaded & m->mem_mask] != 0;
}

// DSP-region READ. In pass-through, delegate entirely to the legacy model. In co-sim, the
// boot/upload handshake is served by the legacy model (so the firmware uploads its blocks);
// only after upload completes — and only with DSP54_LIVE=1 — do HPI reads reflect the live
// C54x data RAM (where the real DSP posts its replies).
// trace EVERY MCU access to the DSPIF/CTSI->DSP MMIO block
// [0x30000,0x30040) — hunting the register the MCU programs to arm a periodic CTSI->DSP
// interrupt (the dispatcher-wake gap). Logs value+PC+dsp_step, capped. Default OFF.
static void dspif_audit(Mad2 *m, const char *dir, uint32_t addr, int size, uint32_t value) {
    static int en = -1; static int n = 0;
    if (en < 0) en = getenv("DSP54_DSPIFAUDIT") ? 1 : 0;
    if (!en || n >= 400) return;
    if (!((addr >= 0x00030000u && addr < 0x00030040u) ||
          (addr >= 0x00040000u && addr < 0x00040040u))) return;
    fprintf(stderr, "[dspif-audit] @%-9llu %s [%05X] %s 0x%04X sz=%d mcu_pc=0x%06X (#%d)\n",
            (unsigned long long)m->dsp_steps, dir, addr,
            dir[0]=='W' ? "<=" : "->", (uint16_t)value, size,
            m->cur_io_pc & 0xFFFFFFu, n);
    n++;
}

// Normalise the 3210's aliased bulk-upload window (MCU 0x11xxx) down to the canonical
// HPI window (0x10000) so the shared window/DSP-word mapping below sees it uniformly.
// No-op for every model with dsp_hpi_alias_base==0 (alias coincides with the control
// window). See models/model.h ModelProfile.dsp_hpi_alias_base.
static inline uint32_t c54x_hpi_norm(Mad2 *m, uint32_t addr) {
    uint32_t alias = (m && m->model) ? m->model->dsp_hpi_alias_base : 0;
    if (alias && addr >= alias && addr < alias + 0x1000u)
        addr -= (alias - HPI_LO);
    return addr;
}

static int c54x_read(Mad2 *m, uint32_t addr, int size, uint32_t ram_value, uint32_t *out) {
    c54x_lazy_init(m);
    addr = c54x_hpi_norm(m, addr);
    dspif_audit(m, "RD", addr, size, ram_value);
    // DSP54_COMMLOG: log MCU reads of the DSP comm cells (the MCU polling the DSP) with the MCU
    // PC, deduped per cell so a tight poll loop logs once until the value the MCU sees changes.
    if (g_commlog < 0) g_commlog = getenv("DSP54_COMMLOG") ? atoi(getenv("DSP54_COMMLOG")) : 0;
    if (g_commlog && g_cosim && addr >= HPI_LO && addr <= HPI_HI) {
        uint32_t k = (addr - HPI_LO) >> 1;
        int interesting = (k==0x02||k==0x03||k==0x71||k==0x72||k==0xE4||k==0xE5
                           || (g_commlog>=2 && (k==0x7F||k==0x80||k==0x81)));
        if (interesting && k < 0x100) {
            static uint16_t rd_last[0x100]; static uint8_t rd_seen[0x100];
            uint16_t cur = dsp54_data_peek(g_dsp, (uint16_t)(0x800u + k));
            if (!rd_seen[k] || rd_last[k] != cur) {
                rd_seen[k] = 1; rd_last[k] = cur;
                fprintf(stderr, "[comm] @%-7lluk M->D READ      %-14s [%05X k=%03X DSP %04X]->0x%04X  mcu_pc=0x%06X\n",
                        (unsigned long long)(m->dsp_steps/1000), comm_kname(k), addr, k, 0x800u + k, cur,
                        m->cur_io_pc & 0xFFFFFFu);
            }
        }
    }
    if (g_state == 1 && g_cosim && addr >= HPI_LO && addr <= HPI_HI) {
        // DSP54_BOOTDELAY=<dsp_step>: patience probe. For the firmware's DSP-version
        // boot-status slots, return "still parked" (0xFFFF) until dsp_steps reaches the
        // delay, THEN let the normal (ready) reply through. Tells whether the firmware
        // polls indefinitely (boots after the delay) or has a bounded timeout (MISMATCH).
        {
            static int bd_init = 0; static uint64_t bd_at = 0;
            if (!bd_init) { bd_init = 1; const char *p = getenv("DSP54_BOOTDELAY");
                if (p && *p) bd_at = strtoull(p, 0, 0); }
            if (bd_at && m->dsp_steps < bd_at &&
                (addr == m->fw.dsp_boot_status || addr == m->fw.dsp_boot_status2)) {
                *out = 0xFFFF; return 1;   // version not ready yet — keep the firmware polling
            }
        }
        static int live = -1;
        if (live < 0) live = getenv("DSP54_LIVE") ? 1 : 0;
        // DSP54_LIVEVER — *** RETIRED / DISABLED — DO NOT RE-ENABLE casually ***
        // It served the version slots (0x10004/0x10006) LIVE from the DSP's api_ram from step 0.
        // That only "works" under the wild RATIO=4 boot recipe, which DIVERGES the real DSP — it
        // runs wild (taskbmp garbage, never idles, vocoder/BIST dominant) while still booting the
        // MCU LCD, so it LOOKS fine. An entire Phase-1 investigation (the "never-reaches-idle /
        // write-coherence-breaks / vocoder-limit-cycle" findings) was measured on this diverged
        // substrate and was WRONG. LIVEVER is NOT a DSP-work substrate.
        // For ANY DSP-internal work use the HEALTHY substrate: DSP54_COSIM=1 DSP54_HPIAPI=1
        // (no LIVEVER) — there the DSP correctly parks at idle ~98.6%.
        //
        // The divergent read-redirect is commented out below. If you ever genuinely need the old
        // MCU-LCD regression recipe, uncomment it AND set DSP54_LIVEVER_UNSAFE=1 — but NEVER trust
        // any DSP-internal (reach/cascade/PCHIST) measurement taken under it.
        {
            static int lv = -1;
            if (lv < 0) {
                lv = getenv("DSP54_LIVEVER") ? 1 : 0;
                if (lv && !getenv("DSP54_LIVEVER_UNSAFE")) {
                    fprintf(stderr,
                      "\n[dsp54] **** DSP54_LIVEVER IS DISABLED — IGNORED **** it diverges the DSP;\n"
                      "[dsp54]      reach/cascade/PCHIST numbers under it are INVALID (see §8s). Use\n"
                      "[dsp54]      DSP54_COSIM=1 DSP54_HPIAPI=1 (no LIVEVER) for DSP work. The bad\n"
                      "[dsp54]      path is commented out in c54x_read(); LCD-guard-only re-enable\n"
                      "[dsp54]      requires uncommenting it AND DSP54_LIVEVER_UNSAFE=1.\n\n");
                    lv = 0;   // force OFF — never take the divergent path silently
                }
            }
            (void)lv;
            // --- RETIRED divergent path (kept for reference; see warning above) ---------------
            // if (lv && (addr == m->fw.dsp_boot_status || addr == m->fw.dsp_boot_status2)) {
            //     uint16_t v = dsp54_hpi_read(g_dsp, addr);
            //     *out = v ? v : 0xFFFF;   // 0 = not written yet -> keep polling
            //     return 1;
            // }
            // ----------------------------------------------------------------------------------
        }
        if (live && upload_done(m)) {
            *out = dsp54_hpi_read(g_dsp, addr);
            return 1;
        }
        // DSP54_SHAREDWIN: the HPI window is ONE shared store. Once the upload completes, MCU reads
        // of the ring/mailbox region come from api_ram (the real DSP's cells) — NOT the HLE — so the
        // MCU finally sees what the DSP actually wrote (ring tail/head, version). Retires the HLE for
        // window reads (the real DSP is self-sufficient at RATIO=4). dsp54_hpi_read returns api_ram
        // for hpi_api_region cells, data[] otherwise — so non-ring window cells fall through cleanly.
        { static int sw = -1; if (sw < 0) sw = dsp54_faithful("DSP54_SHAREDWIN");
          if (sw && upload_done(m)) { *out = dsp54_hpi_read(g_dsp, addr); return 1; } }
        // DSP54_REALUPLOAD: let the REAL loader1 drive the upload handshake. Under UNIDARAM the window
        // IS api_ram, so route MCU window READS to api_ram DURING boot too — so the MCU's bulk-ack poll
        // (mbox0/mbox1 = words 0x7F/0x80) reads loader1's acks, not the HLE's instant dsp_ack echo. This
        // makes the MCU pace the upload by the real loader1 (so loader1 consumes every chunk -> its count
        // exhausts -> finalize -> idle), instead of the HLE racing ahead and stranding loader1.
        // The old version/boot-status (0x10004/0x10006) HLE EXCEPTION is RETIRED:
        // COMMLOG proves the real loader1 writes the version cells itself ([0x802/0x803]=0x0004 at
        // dsp_pc=0x0F15 — its ready signal, not an ISR reply), so the HLE answer was the last faked
        // DSP->MCU signal in the read path. DSP54_REALSTATUS=0 restores the exception for A/B.
        { static int ru = -1, rs = -1;
          if (ru < 0) { ru = dsp54_faithful("DSP54_REALUPLOAD") && dsp54_faithful("DSP54_UNIDARAM");
                        rs = dsp54_faithful("DSP54_REALSTATUS"); }
          if (ru && (rs || (addr != m->fw.dsp_boot_status && addr != m->fw.dsp_boot_status2))) {
              *out = dsp54_hpi_read(g_dsp, addr); return 1; } }
        // boot/upload phase (or DSP54_LIVE/SHAREDWIN unset): let the legacy model answer so the
        // firmware's modelled upload handshake runs.
    }
    return dsp_default_read(m, addr, size, ram_value, out);
}

// DSP-region WRITE. Pass-through delegates to the legacy model. In co-sim we mirror the
// MCU's write into the DSP's data RAM so the DSP sees the command words.
static int c54x_write(Mad2 *m, uint32_t addr, int size, uint32_t value) {
    c54x_lazy_init(m);
    addr = c54x_hpi_norm(m, addr);
    dspif_audit(m, "WR", addr, size, value);
    // DSP54_COMMLOG: log this MCU->DSP signalling write with the exact MCU PC.
    if (g_commlog < 0) g_commlog = getenv("DSP54_COMMLOG") ? atoi(getenv("DSP54_COMMLOG")) : 0;
    if (g_commlog) {
        uint16_t v16 = (size >= 2) ? (uint16_t)value : (uint16_t)(value & 0xFF);
        unsigned long long st = (unsigned long long)(m->dsp_steps / 1000);
        uint32_t pc = m->cur_io_pc & 0xFFFFFFu;
        if (addr == 0x00020002u)   // IO_CTSI_DSP: the real DSP reset line (bit0 = release)
            fprintf(stderr, "[comm] @%-7lluk M->D RESET-CTRL  [%05X]=0x%02X  bit0=%d (%s)        mcu_pc=0x%06X\n",
                    st, addr, v16 & 0xFF, (int)(v16 & 1), (v16 & 1) ? "RELEASE" : "HOLD", pc);
        else if (addr == 0x00020003u)
            // IO_CTSI_WDT (MADos ioports.h) — the SYSTEM hardware-watchdog register, NOT DSP
            // comms: ccont_reset_wdt (5110: 0x292796) kicks it with 0x31 every ~18.8M steps
            // (~1.45 s); 0x00 = power-off. Was mislabeled "RESET-CTRL RELEASE" (adjacent reg).
            fprintf(stderr, "[comm] @%-7lluk M     CTSI-WDT    [%05X]=0x%02X  (%s)        mcu_pc=0x%06X\n",
                    st, addr, v16 & 0xFF, (v16 & 0xFF) ? "kick" : "POWER-OFF", pc);
        else if (addr >= 0x00020008u && addr <= 0x0002000Bu) {
            // [0x20008]-[0x2000B] = the MCU's OWN ARM FIQ act/status block (IO_FIQ_ACT) —
            // it NEVER reaches the DSP (settled 2026-06-10), so it is not MCU<->DSP comms at
            // any level. NOT logged by COMMLOG: the reboot/recovery path (0x2866xx) and the
            // timer-FIQ ack hammer it in tight loops -> storms the trace. (A dedicated ARM-FIQ
            // probe could resurrect it, but it does not belong in the DSP comms log.)
        }
        else if ((addr == 0x00030000u || addr == 0x00030001u))
            fprintf(stderr, "[comm] @%-7lluk M->D DSPINT      [%05X]=0x%04X  API=%d INT=%d (doorbell)  mcu_pc=0x%06X\n",
                    st, addr, v16, (v16 >> 1) & 1, (v16 >> 2) & 1, pc);
        else if (addr >= HPI_LO && addr <= HPI_HI) {
            uint32_t k = (addr - HPI_LO) >> 1;
            if (g_commlog >= 2 || comm_is_signal(k))
                fprintf(stderr, "[comm] @%-7lluk M->D %-14s [%05X k=%03X DSP %04X]=0x%04X     mcu_pc=0x%06X\n",
                        st, comm_kname(k), addr, k, 0x800u + k, v16, pc);
        }
    }
    // DSP54_INITLOG=1: ordered MCU->DSP signal trace for the MCU-init BASELINE (works against the
    // HLE — runs OUTSIDE the cosim gate). Logs every reset/doorbell/window-protocol write with the
    // MCU step ("instruction#"), so we can document the real init/upload protocol independent of the
    // (broken) co-sim. Default OFF.
    { static int il = -1; if (il < 0) il = getenv("DSP54_INITLOG") ? 1 : 0;
      if (il) {
        uint16_t v16 = (size >= 2) ? (uint16_t)value : (uint16_t)(value & 0xFF);
        unsigned long long st = (unsigned long long)m->dsp_steps;
        if (addr == 0x00020002u || addr == 0x00020003u)
            fprintf(stderr, "[init] @%-9llu RESET-CTRL 0x20002 <= 0x%02X (bit0/release=%d)\n", st, v16 & 0xFF, (int)(v16 & 1));
        else if ((addr == 0x00030000u || addr == 0x00030001u) && (v16 & 0x04))
            fprintf(stderr, "[init] @%-9llu DSPINT doorbell (MCU->DSP INT, 0x30000 bit2)\n", st);
        else if (addr >= 0x10000u && addr <= 0x10FFFu) {
            uint32_t k = (addr - 0x10000u) >> 1;
            const char *nm = (k==0x52)?"MDISND_TAIL":(k==0x53)?"MDISND_HEAD":(k==0x71)?"UPLOADREQUEST":
                             (k==0x72)?"UPLOADREPLY":(k>=0x7B&&k<=0x7F)?"UPLOADHEADER":(k==0x80)?"PORT2/mbox":
                             (k==0x81)?"PORT1/cmd":(k==0xE4)?"MDIRCV_TAIL":(k==0xE5)?"MDIRCV_HEAD":0;
            if (nm) fprintf(stderr, "[init] @%-9llu %-13s word 0x%02X (mcu 0x%05X) <= 0x%04X\n", st, nm, k, addr, v16);
        }
      } }
    // DSP reset-control register (MMIO 0x20002, bit0 = release): the firmware holds the DSP in
    // reset during the code upload and RELEASES it (0->1) exactly when the upload completes — the
    // real boot moment (measured: held 327k-397k, released @397k = upload FINISHED). Under
    // DSP54_REALUP we gate the co-sim DSP on this real signal: held (not stepped) while bit0=0,
    // and on the 0->1 release edge we (re-)enter loader1 @0xF00 so it boots on the now-complete
    // DARAM. (Faithful HW reset modelling; 3310 also reads 0x53 'running' back via the profile.)
    if (addr == 0x00020002u || addr == 0x00020003u) {
        if (g_log)
            fprintf(stderr, "[dsp54] DSPRESETCTRL 0x%05X <= 0x%04X (size %d) @dsp_steps=%lluk\n",
                    addr, (uint16_t)value, size, (unsigned long long)(m->dsp_steps/1000));
        if (g_realup && addr == 0x00020002u) {
            int bit0 = (int)(value & 1u);
            if (bit0 && !g_dsp_run) {          // reset RELEASE: enter at the REAL reset vector 0xFF80
                // (NOT 0xF00 directly) so the DSP runs PMST=#0xFFA8 (OVLY=1, IPTR) -> goto 0xF00 —
                // loader1 needs that machine state; jumping straight to 0xF00 leaves PMST stale.
                g_dsp_run = 1;
                // WARM BOOT: the MCU asserts+re-releases reset AFTER the upload completes (~120M:
                // 0x20002 0->0->1) = a C54x warm boot. The COLD (first) release runs the upload; the
                // WARM (subsequent) releases restart into run-mode on the verified DARAM. On a warm
                // boot the DARAM (verified code + overlays) must be PRESERVED and the CPU must be
                // RESET (TI: RS forces PC=0xFF80 + resets regs/status but does NOT clear RAM). The old
                // path did dsp54_set_pc(0xFF80) ONLY (no CPU reset) -> stale mid-CRC regs -> OVLY=0 ->
                // DSP ran DROM garbage @0xDExx / wedged @0xFFC0. Now: cold = set_pc + seed + init;
                // warm = dsp54_warm_reset (regs reset, DARAM preserved), no re-seed/re-init.
                // DSP54_RESEED_ALWAYS=1 forces the old cold-path behaviour on every release (A/B).
                static int g_release_n = 0;
                int cold = (g_release_n == 0) || getenv("DSP54_RESEED_ALWAYS");
                g_release_n++;
                if (!cold) {
                    // C54x "warm boot" SHOULD branch to ALREADY-RESIDENT code (the uploaded block),
                    // NOT re-run loader1 (per the bootloader docs). DSP54_WARMENTRY exposes the entry
                    // for A/B (e.g. 0xD80 = loader2/run-mode -> b 0x3000). DEFAULT stays 0xFF80
                    // (faithful to RS-forces-PC=0xFF80) because the warm entry is NOT the current
                    // blocker: the IMR clobber (ret@0xf73 over-pop -> 0x940d retd->0xf074->AR5=0->
                    // IMR=0) happens in the COLD (1st-pass) loader1 at insn~482M, the moment the upload
                    // completes — VERIFIED to occur BEFORE the MCU asserts/re-releases reset (warm
                    // entry=0xD80 left the clobber byte-identical). So fix the cold loader1 finalize
                    // over-pop first; THEN A/B the warm entry.
                    uint16_t warm_pc = 0xFF80;
                    const char *we = getenv("DSP54_WARMENTRY");
                    if (we && *we) warm_pc = (uint16_t)strtol(we, 0, 0);
                    // Did loader1's cold pass RELOCATE the base set to its run homes? (Expect, if
                    // relocation works: 0xD80=0x7726 loader2, 0xD00=0xFC00 IVT, 0x926=scheduler.)
                    if (g_log) fprintf(stderr, "[dsp54] WARM-BOOT block homes: 0xD80=0x%04X(loader2?7726) "
                        "0xD00=0x%04X(IVT?FC00) 0x926=0x%04X(sched) | bufA 0x900=0x%04X bufB 0xB00=0x%04X\n",
                        dsp54_data_peek(g_dsp,0xD80), dsp54_data_peek(g_dsp,0xD00), dsp54_data_peek(g_dsp,0x926),
                        dsp54_data_peek(g_dsp,0x900), dsp54_data_peek(g_dsp,0xB00));
                    // DSP54_F00DUMP=1: ground-truth dump of the rehomed reset-target region the warm
                    // RS (0xFF80 -> b 0x0f00) actually enters. Decides whether warm_pc=0xFF80 is faithful.
                    if (getenv("DSP54_F00DUMP")) {
                        fprintf(stderr, "[dsp54] F00DUMP (DSP 0xF00..0xF5F, true DARAM):");
                        for (uint16_t a = 0xF00; a <= 0xF5F; a++) {
                            if (((a - 0xF00) & 7) == 0) fprintf(stderr, "\n  0x%04X:", a);
                            fprintf(stderr, " %04X", dsp54_data_peek(g_dsp, a));
                        }
                        fprintf(stderr, "\n[dsp54] F00DUMP also 0xD00..0xD0F (rehomed IVT?):");
                        for (uint16_t a = 0xD00; a <= 0xD0F; a++) fprintf(stderr, " %04X", dsp54_data_peek(g_dsp, a));
                        fprintf(stderr, "\n");
                    }
                    dsp54_warm_reset(g_dsp, warm_pc);  // faithful C54x reset, DARAM preserved
                    if (g_log) fprintf(stderr, "[dsp54] REALUP: WARM BOOT (release #%d) -> 0x%04X, "
                                       "CPU reset + DARAM PRESERVED @dsp_steps=%lluk\n",
                                       g_release_n, warm_pc, (unsigned long long)(m->dsp_steps/1000));
                } else {
                // loader1 is uploaded by the MCU ITSELF — its boot routine STM-copies block25 into the
                // HPI bulk-upload window BEFORE this release edge, so loader1 is resident at DSP 0xF00
                // by the time we enter the PROM reset vector. The window is at MCU 0x10E00 on most DCT3
                // and aliased to 0x11E00 on the 3210 (folded back by c54x_hpi_norm via the profile's
                // dsp_hpi_alias_base). The faithful path needs NO manual seed. If loader1 ISN'T present,
                // that is a genuine regression (wrong upload-window mapping, or a missed upload) — make
                // it LOUD rather than papering over it with a hand-staged block (the retired fallback
                // seed hid exactly the 3210 window-alias bug for a long time).
                if (dsp54_data_peek(g_dsp, 0xF00) != 0xF7BBu)
                    fprintf(stderr, "[dsp54] WARNING: loader1 NOT uploaded by the MCU (DSP[0xF00]=0x%04X, "
                                    "expected 0xF7BB) — the DSP will not boot. Check the HPI bulk-upload "
                                    "window mapping (ModelProfile.dsp_hpi_alias_base) for this model.\n",
                                    dsp54_data_peek(g_dsp, 0xF00));
                else if (g_log)
                    fprintf(stderr, "[dsp54] loader1 present from MCU STM upload (DSP[0xF00]=0xF7BB) — faithful\n");
                dsp54_set_pc(g_dsp, 0xFF80);
                // FAITHFUL UPLOAD HANDSHAKE (CORRECTED BOOT MODEL): loader1 is a CRC-gate over the
                // double-buffer. The MCU writes the UPLOADHEADER + both mailboxes (words 0x7B-0x81)
                // and sets the mailboxes = 1 ("MCU owns / not ready") BEFORE this release edge, then
                // streams buffers, writing mbox=0 ("buffer ready") per transfer; loader1 waits
                // (f17/f33: bc ...,aneq while mbox != 0), CRCs the buffer, re-arms mbox=1. So the
                // mailboxes MUST be MCU-owned (=1) the instant loader1 first polls them, else loader1
                // races: it reads the calloc-0 api_ram, falls straight through f17, and CRC-walks an
                // empty buffer -> desync -> MCU spins at 0x271B2A. Force the MCU-owned state here so
                // loader1's first f17/f33 poll blocks until the MCU hands off buffer A. (mbox0=word
                // 0x7F=api_ram[0x7F]; mbox1=word 0x80=api_ram[0x80].)
                dsp54_data_poke(g_dsp, 0x87F, 1); // MBOX0 = MCU-owned (loader1 waits for hand-off)
                dsp54_data_poke(g_dsp, 0x880, 1); // MBOX1 = MCU-owned  (both stores — loader1 reads data[])
                if (g_log) fprintf(stderr, "[dsp54] RELEASE header api_ram[0x7B..0x81] = "
                    "destA=%04X destB=%04X remHi=%04X remLo=%04X mbox0=%04X mbox1=%04X len=%04X\n",
                    dsp54_data_peek(g_dsp,0x87B), dsp54_data_peek(g_dsp,0x87C),
                    dsp54_data_peek(g_dsp,0x87D), dsp54_data_peek(g_dsp,0x87E),
                    dsp54_data_peek(g_dsp,0x87F), dsp54_data_peek(g_dsp,0x880), dsp54_data_peek(g_dsp,0x881));
                if (g_log) fprintf(stderr, "[dsp54] REALUP: DSP RELEASED (0x20002 bit0=1) -> reset vec 0xFF80 "
                                   "@dsp_steps=%lluk\n", (unsigned long long)(m->dsp_steps/1000));
                }   // end cold-release seed/init
            } else if (!bit0 && g_dsp_run) {   // reset ASSERTED: hold the DSP
                g_dsp_run = 0;
                if (g_log) fprintf(stderr, "[dsp54] REALUP: DSP HELD in reset (0x20002 bit0=0) "
                                   "@dsp_steps=%lluk\n", (unsigned long long)(m->dsp_steps/1000));
            }
        }
    }
    // DSP54_HIWIN=1: count/log MCU writes to 0x11000-0x14FFF (= DSP DARAM words 0x800-0x27FF,
    // beyond the current 0x10000-0x10FFF mirror window). If the firmware writes here, the HPI
    // window is too small and the dispatch table at data[0x2470] etc. never reaches the C54x.
    if (getenv("DSP54_HIWIN") && addr >= 0x11000u && addr < 0x15000u) {
        static uint32_t hw_n = 0;
        if (hw_n < 16) fprintf(stderr, "[dsp54] HIWIN write mcu=0x%05X (dsp word 0x%04X) = 0x%04X @step %lluk\n",
                               addr, (addr-0x10000u)>>1, (uint16_t)value, (unsigned long long)(m->dsp_steps/1000));
        hw_n++;
        if (m->dsp_steps > 6000000u && hw_n && (hw_n % 100000 == 0 || hw_n == 17))
            fprintf(stderr, "[dsp54] HIWIN total writes so far: %u\n", hw_n);
    }
    // DSP54_WRSCAN=1: count every DSP-bound IO write per address (works in BOTH pass-through
    // and co-sim), dump the top targets at DSP54_WRSCAN steps. Finds the firmware's actual
    // DSP code-block upload channel (a burst to an HPIA/HPID-style register, or a different
    // DSP window) vs the small 0x100xx mailbox we already see. Answers: does the phone upload?
    {
        static int ws_init = 0, ws_on = 0, ws_dumped = 0; static uint64_t ws_at = 8000000u;
        static uint32_t *ws_a = 0; static uint32_t *ws_c = 0; static int ws_n = 0;
        if (!ws_init) { ws_init = 1; const char *p = getenv("DSP54_WRSCAN");
            if (p && *p) { ws_on = 1; long v = strtol(p,0,0); if (v>1) ws_at=(uint64_t)v;
                ws_a = calloc(4096,sizeof *ws_a); ws_c = calloc(4096,sizeof *ws_c); } }
        if (ws_on && ws_a) {
            int found = 0;
            for (int i=0;i<ws_n;i++) if (ws_a[i]==addr) { ws_c[i]++; found=1; break; }
            if (!found && ws_n<4096) { ws_a[ws_n]=addr; ws_c[ws_n]=1; ws_n++; }
            if (m->dsp_steps >= ws_at && !ws_dumped) { ws_dumped = 1;
                fprintf(stderr, "[dsp54] WRSCAN top DSP-bound write targets @%lluk (cosim=%d):\n",
                        (unsigned long long)(m->dsp_steps/1000), g_cosim);
                for (int k=0;k<25;k++){ uint32_t best=0,bi=0; for(int i=0;i<ws_n;i++) if(ws_c[i]>best){best=ws_c[i];bi=i;}
                    if(!best) break;
                    fprintf(stderr,"    0x%05X : %u writes\n", ws_a[bi], best); ws_c[bi]=0; }
            }
        }
    }
    if (g_state == 1 && g_cosim) {
        // MCU->DSP doorbell: DSPIF reg [0x30000] (16-bit BE), bit2 = DSPINT. The firmware
        // strobes it via io_set_bit(0x30000, 0x04) after staging a mailbox command. Pulse
        // the DSP's host-command interrupt (INT2 -> vector 0xFFC8 -> ISR 0x3598/0x35B9).
        // Side-effect only: return 0 so the normal MMIO write still proceeds (APIMODE etc.).
        if (addr == 0x00030000u || addr == 0x00030001u) {
            uint16_t v16 = (size >= 2) ? (uint16_t)value
                         : (addr == 0x00030001u) ? (uint16_t)(value & 0xFF)
                         : (uint16_t)((value & 0xFF) << 8);
            // 0x30000 bit2 = HPIC DSPINT -> the DSP's HPINT = **vec 25** (-> 0x3772, which sets the
            // idle work-bit [0x866]|=2 the dispatcher gates on, and services the [0x870] mailbox via
            // 0xA51B). This is the well-documented TI C54x HPI semantic (host writes HPIC.DSPINT ->
            // DSP HPINT) and is now confirmed authoritative: HPINT IS [0x30000].
            // VERIFIED: with [0x30000]->vec25 the boot still handshakes to verdict 0xC4 (the version
            // exchange runs through the [0x870] mailbox/HPINT path, NOT the port cmd-word) and the
            // dispatcher wakes faithfully. The OLD default of 18 routed the host doorbell to the
            // cmd-word ISR 0x3598/0x35B9 (INT2) instead — that decoder is vector-only (IVT vec17/18/24,
            // never called) and is a SEPARATE channel that arms the MDISND dequeue (bit1->0x3621->
            // [0x866]bit0). With [0x30000]->vec25 that arm is NOT reached, because INT2 (vec18) is NOT
            // MCU-pulsed (the poster 0x272018 writes only [0x30000]+[0x20008]) — INT2 is a separate,
            // likely periodic (timer-poll) source, still to be modelled. The prior "vec25 starves the
            // dispatcher" note was exactly that lost arm. DSP54_HPIVEC overrides.
            if (g_hpivec < 0) { const char *hv = getenv("DSP54_HPIVEC");
                g_hpivec = (hv && *hv) ? (int)strtol(hv, 0, 0) : 25; }
            if ((v16 & 0x0004u) && (!g_realup || g_dsp_run)) {   // bit2 = DSPINT (held DSP ignores)
                if (g_log && g_dspint_edges < 32) {
                    Dsp54Status st; dsp54_status(g_dsp, &st);
                    fprintf(stderr, "[dsp54] DSPINT #%llu vec=%d @dsp_steps=%lluk  "
                            "DSP{pc=0x%04X imr=0x%04X INT2en=%d INTM=%d idle=%d}\n",
                            g_dspint_edges + 1, g_hpivec, (unsigned long long)(m->dsp_steps/1000),
                            st.pc, st.imr, (st.imr >> 2) & 1, (st.st1 >> 11) & 1, st.idle);
                }
                dsp54_host_interrupt(g_dsp, g_hpivec);
                g_dspint_edges++;
            }
            return 0;
        }
        // [0x20008] is the MCU's OWN ARM FIQ pending/ack register (IO_FIQ_ACT=0x08; mad2_bus.c
        // ACKs on write), NOT a DSP doorbell. The MDISND enqueue's `[0x20008]=2` @0x271844 is the
        // ack-stale+unmask of FIQ1=FIQ_MDISND (ref/Nok-MADos-master/include/hw/int.h), the DSP->MCU
        // flow-control source we now raise faithfully on the MDISND-head advance (DSP54_RINGFIQ).
        // It does NOT reach the DSP. Firing a DSP interrupt from it was the old "unified doorbell"
        // HACK that stood in for the real host doorbell [0x30000]->vec25 (HPINT) — now that that is
        // wired correctly (g_hpivec=25), the hack is RETIRED (DSP54_RINGDOORBELL, default OFF). Kept
        // only for A/B; DSP54_UNIDOORBELL likewise requires it. (The MDISND consumer's wake is HPINT
        // ([0x866]bit1 via 0x3772); its dequeue ARM is INT2/vec18 — a separate, non-MCU-pulsed
        // source still to be modelled. Neither is [0x20008].)
        { static int rd_on = -1; if (rd_on < 0) rd_on = getenv("DSP54_RINGDOORBELL") ? 1 : 0;
          if (rd_on && addr == 0x00020008u && (!g_realup || g_dsp_run)) {
            uint8_t b8 = (size == 1) ? (uint8_t)value
                       : (size == 2) ? (uint8_t)(value >> 8) : (uint8_t)(value >> 24);
            if (b8 & 0x02) {
                if (g_log) { static unsigned n; if (++n <= 16)
                    fprintf(stderr, "[dsp54] RING-DOORBELL(legacy) [0x20008] bit1 -> vec %d @dsp_steps=%lluk\n",
                            g_intvec, (unsigned long long)(m->dsp_steps/1000)); }
                dsp54_host_interrupt(g_dsp, g_intvec);
                { static int ud = -1; if (ud < 0) ud = getenv("DSP54_UNIDOORBELL") ? 1 : 0;
                  if (ud && g_intvec != 18) dsp54_host_interrupt(g_dsp, 18); }
                g_dspint_edges++;
            }
          }
        }
        if (addr >= HPI_LO && addr <= HPI_HI) {
            // DSP54_UPLOADLOG=1: tally MCU writes per DSP data word, dump the populated
            // ranges at the end — pins WHERE the firmware uploads block payloads (the
            // "accept block upload to correct mem map" question). DSP data word k = (addr-0x10000)/2.
            static int ul_init = 0, ul_on = 0;
            static uint32_t *ul = 0;
            if (!ul_init) { ul_init = 1; ul_on = getenv("DSP54_UPLOADLOG") ? 1 : 0; if (ul_on) ul = calloc(0x800, sizeof *ul); }
            if (ul_on && ul) {
                uint32_t k = (addr - 0x10000u) >> 1;
                if (k < 0x800) ul[k]++;
                if (g_log && (k < 0x800) && ul[k] == 1)
                    fprintf(stderr, "[dsp54] HPI write FIRST-TOUCH word 0x%03X (mcu 0x%05X) = 0x%04X @dsp_steps=%lluk\n",
                            k, addr, (uint16_t)value, (unsigned long long)(m->dsp_steps/1000));
            }
            // DSP54_CMDLOG=1: log NON-ZERO MCU writes into the MDISND command channel
            // (MCU->DSP send ring words 0x00-0x51 + tail 0x52/head 0x53 = MCU 0x10000-0x100A6).
            // First-touch (UPLOADLOG) only shows the zero-init; this catches the real commands
            // + the tail advance — answering "does the MCU send task-enable MDISND traffic?".
            {
                static int cl_init = 0, cl_on = 0; static unsigned cl_n = 0;
                if (!cl_init) { cl_init = 1; cl_on = getenv("DSP54_CMDLOG") ? 1 : 0; }
                uint32_t k = (addr - 0x10000u) >> 1;
                if (cl_on && (k <= 0x53 || k == 0x80 || k == 0x81) && (uint16_t)value != 0 && cl_n < 60) {
                    const char *tag = (k == 0x52) ? "MDISND_TAIL" : (k == 0x53) ? "MDISND_HEAD"
                                    : (k == 0x81) ? "PORT1_CMD" : (k == 0x80) ? "PORT2" : "MDISND_RING";
                    fprintf(stderr, "[dsp54] CMDLOG #%u %s word 0x%02X (mcu 0x%05X) = 0x%04X @dsp_steps=%lluk\n",
                            cl_n, tag, k, addr, (uint16_t)value, (unsigned long long)(m->dsp_steps/1000));
                    cl_n++;
                }
            }
            // DSP54_HDRLOG=1 (Step A — pin the per-block upload protocol): at each MCU write to
            // UPLOADREPLY (mcu 0x100E4 = word 0x72) snapshot the 5-word UPLOADHEADER (words
            // 0x7B-0x7F) + the first 2 words at each candidate dest, so we can tell whether the
            // MCU writes each block payload to word=dest directly or to a fixed staging slot that
            // the DSP loader then scatters. Settles the faithful scatter model. (read-only)
            {
                static int hl_init = 0, hl_on = 0; static unsigned hl_n = 0;
                if (!hl_init) { hl_init = 1; hl_on = getenv("DSP54_HDRLOG") ? 1 : 0; }
                if (hl_on && (addr == 0x000100E4u || addr == 0x000100FEu) && hl_n < 80) {
                    // Peek data[] directly (where MCU HPI writes land: word k -> data[k]). Probe
                    // header (words 0x7B-0x7F) + both the low word=dest (e.g. data[0x126]) AND the
                    // run address dest+0x800 (e.g. data[0x926]) so we see WHERE the payload lands
                    // and whether a +0x800 relocate is needed. Trigger on full-descriptor (0x7F
                    // written last) or UPLOADREPLY. (read-only)
                    fprintf(stderr,
                        "[dsp54] HDRLOG #%u %s=0x%04X @%lluk hdr[7B..7F]=%04X %04X %04X %04X %04X | "
                        "low: 100=%04X 126=%04X 500=%04X 580=%04X 700=%04X | "
                        "run: 926=%04X D00=%04X D80=%04X F00=%04X\n",
                        hl_n, (addr == 0x000100E4u) ? "reply" : "hdr7F", (uint16_t)value,
                        (unsigned long long)(m->dsp_steps/1000),
                        dsp54_data_peek(g_dsp,0x7B), dsp54_data_peek(g_dsp,0x7C), dsp54_data_peek(g_dsp,0x7D),
                        dsp54_data_peek(g_dsp,0x7E), dsp54_data_peek(g_dsp,0x7F),
                        dsp54_data_peek(g_dsp,0x100), dsp54_data_peek(g_dsp,0x126),
                        dsp54_data_peek(g_dsp,0x500), dsp54_data_peek(g_dsp,0x580), dsp54_data_peek(g_dsp,0x700),
                        dsp54_data_peek(g_dsp,0x926), dsp54_data_peek(g_dsp,0xD00),
                        dsp54_data_peek(g_dsp,0xD80), dsp54_data_peek(g_dsp,0xF00));
                    hl_n++;
                }
            }
            // HOMELOG: WHEN does the MCU write the block RUN-HOMES (k>=0x500, OUTSIDE bufA/bufB
            // which are k=0x100-0x4FF)? Tests the hypothesis that loader1 is CRC-ONLY and the MCU
            // REHOMES blocks itself in the HOLD->WARM-RESET window (~865k-934k), not during upload.
            { static int hm = -1; if (hm < 0) hm = getenv("DSP54_HOMELOG") ? 1 : 0;
              if (hm) { uint32_t k0 = (addr - 0x10000u) >> 1;
                if (k0 >= 0x500u && k0 <= 0x7FFu) { static int hn = 0; if (hn++ < 120)
                  fprintf(stderr, "[dsp54] HOMELOG: MCU->home k=0x%03X (DSP 0x%04X) = 0x%04X @dsp_steps=%lluk\n",
                          k0, 0x800u+k0, (uint16_t)value, (unsigned long long)(m->dsp_steps/1000)); } } }
            // DSP54_HOLDLOG=1: log EVERY shared-window write during the HOLD->RELEASE window
            // (default dsp_steps 855k-945k; override DSP54_HOLDLO/HOLDHI). Answers "does the host
            // write anything ELSE into the window after the hold, before release?" — e.g. mailbox/
            // header re-setup that steers loader1's warm run, or the full block rehome.
            { static int hl2 = -1; static uint64_t lo = 855000, hi = 945000;
              if (hl2 < 0) { hl2 = getenv("DSP54_HOLDLOG") ? 1 : 0;
                  const char *l=getenv("DSP54_HOLDLO"); if(l&&*l) lo=strtoull(l,0,0);
                  const char *h=getenv("DSP54_HOLDHI"); if(h&&*h) hi=strtoull(h,0,0); }
              if (hl2 && m->dsp_steps>=lo && m->dsp_steps<=hi) { uint32_t k0=(addr-0x10000u)>>1;
                if (k0 <= 0xFFFu) { static int n=0; if(n++<6000)
                  fprintf(stderr, "[dsp54] HOLDWR k=0x%03X (DSP 0x%04X / MCU 0x%05X) = 0x%04X sz%d @%lluk\n",
                          k0, 0x800u+k0, addr, (uint16_t)value, size, (unsigned long long)(m->dsp_steps/1000)); } } }
            // WIDTH-AWARE window write — the fix that lets the MCU's OWN loader1 HPI-write land.
            // The MCU stages loader1 with 32-bit STM stores (PC 0x2936F4 `stmia r1!,{r3-r12}`),
            // but the header + bulk upload use 16-bit `strh`. A 32-bit write spans TWO DSP words
            // (addr, addr+2); the ARM core is BIG-ENDIAN, so the HIGH halfword (value>>16) lands at
            // the LOWER address. Truncating to one `(uint16_t)value` dropped every other word — so
            // loader1 (32-bit STM) never assembled in DARAM, which is exactly why the cold-edge seed
            // had to fake it. Splitting by width makes the firmware's real upload land where the DSP
            // fetches it. 16-bit writes keep the single-word path.
            // The window IS the DSP's dual-port DARAM (single store): dsp54_hpi_write lands each
            // word in api_ram[k] = DSP 0x800+k — the SAME cell the DSP fetches AND data-reads (the
            // data-read path resolves [0x800,0x2800) to api_ram; calypso_c54x.c). The old scoped
            // dsp54_data_poke "upload mirror" that also wrote data[] is RETIRED — data[] for that
            // range is vestigial under the single store, so it was pure duplication.
            {
                int nw = (size >= 4) ? 2 : 1;
                for (int w = 0; w < nw; w++) {
                    uint32_t a  = addr + 2u * (uint32_t)w;
                    uint16_t hv = (size >= 4) ? (uint16_t)(value >> (16 * (1 - w))) : (uint16_t)value;
                    // DSP54_CELLWATCH companion: stamp MCU window writes with the ARM PC + step.
                    { static int cw = -1; static unsigned lo, hi, n = 0;
                      if (cw < 0) { const char *e = getenv("DSP54_CELLWATCH");
                          if (e && *e && sscanf(e, "%x:%x", &lo, &hi) == 2) cw = 1; else cw = 0; }
                      if (cw) { uint32_t da = 0x800u + ((a - 0x10000u) >> 1);
                          if (da >= lo && da <= hi && n < 200000) { n++;
                              fprintf(stderr, "[dsp54] CELLWATCH-MCU [0x%04X] <- 0x%04X mcu_pc=0x%06X @%lluk\n",
                                      da, hv, m->cur_io_pc & 0xFFFFFFu,
                                      (unsigned long long)(m->dsp_steps/1000)); } } }
                    dsp54_hpi_write(g_dsp, a, hv);
                    // CMDLEVEL inputs: MDISND tail/head (k=0x52/0x53), cmd word (0x54), mask (0x55).
                    { uint32_t k = (a - 0x10000u) >> 1;
                      if (k >= 0x52u && k <= 0x55u) cmdlevel_eval(m); }
                }
            }
            // DSP54_CMDBRIDGE=1 (experiment, §8p unlock): ALSO deliver the MCU's MDISND
            // command channel (send-ring words 0x00-0x51 + tail 0x52/head 0x53 = MCU
            // 0x10000-0x100A6) into api_ram — the cells the real DSP dequeues from (DSP
            // 0x800+). The default write path lands these in data[] only, so the DSP never
            // sees the MCU's task-enable/config commands (its task bitmap [0x6E4] stays 0).
            // Scoped to the command region (NOT the whole window) to avoid the full-window
            // HLE-blast that breaks idle (§8m). Default OFF.
            {
                static int cb_on = -1; if (cb_on < 0) cb_on = getenv("DSP54_CMDBRIDGE") ? 1 : 0;
                if (cb_on) { uint32_t k = (addr - 0x10000u) >> 1; if (k <= 0x53) dsp54_api_poke(g_dsp, (uint16_t)k, (uint16_t)value); }
            }
            return dsp_default_write(m, addr, size, value);
        }
    }
    return dsp_default_write(m, addr, size, value);
}

// DSP54_REALUP — faithful staged boot upload.
// KEY: loader1 (block25) IS the upload consumer — it runs from 0xF00 and polls the UPLOADHEADER
// (0xF17: A=*(0x87F); if(cond) goto 0xF17), reads each block descriptor + payload from the HPI
// window, and SCATTERS it to dest in real DSP code. So we do NOT model the scatter and do NOT
// hold the DSP: we only seed loader1 (modelling the HPI hardware boot that places the loader
// before reset-release), let it run from t=0, and BRIDGE the MCU's upload-region writes into
/* seed_block_from_flash + realup_seed_loader1 (manual loader1/IVT/loader2 staging) REMOVED
 * 2026-06-17: the faithful path is the firmware uploading every DSP block itself over the HPI
 * window (loader1 lands at DSP 0xF00 before the reset-release edge — see the 0x20002 handler).
 * The hand-seed fallbacks (DSP54_SEEDBASE / the block25 fallback) hardcoded the 5110's flash
 * descriptor addresses and MASKED the 3210's +0x1000 upload-window misalignment by faking
 * loader1 from the wrong source — exactly the confusion they caused. The window alias is now
 * carried as data (ModelProfile.dsp_hpi_alias_base) so the firmware's own upload lands correctly
 * on every model; a missing loader1 is reported LOUDLY instead of silently seeded. */

// Per-MCU-step pump. Always run the legacy tick (harmless in co-sim — its writes target
// the same HPI RAM; superseded as the DSP takes over). In co-sim, advance the real DSP.
static void c54x_tick(Mad2 *m) {
    c54x_lazy_init(m);
    // DSP54_INITLOG (DSP->MCU side, HLE baseline): log the DSP->MCU signals the model raises —
    // FIQ/IRQ (HINT) + MDIRCV ring tail advance + the version-reply slots — on CHANGE, with the MCU
    // step. Captures the reply half of the init protocol. Outside the cosim gate (works for HLE).
    { static int il = -1; if (il < 0) il = getenv("DSP54_INITLOG") ? 1 : 0;
      if (il && m->mem) {
        static uint32_t last_fiq = 0; static uint16_t last_rcv = 0xFFFF, last_ver = 0xFFFF;
        unsigned long long st = (unsigned long long)m->dsp_steps;
        if (m->fiq_pending != last_fiq) {
            if (m->fiq_pending & ~last_fiq)
                fprintf(stderr, "[init] @%-9llu DSP->MCU FIQ raised (pending=0x%X)\n", st, m->fiq_pending);
            last_fiq = m->fiq_pending;
        }
        uint16_t rcv = (uint16_t)((m->mem[0x101C8 & m->mem_mask] << 8) | m->mem[(0x101C8 & m->mem_mask) + 1]);
        if (rcv != last_rcv) {
            if (last_rcv != 0xFFFF) fprintf(stderr, "[init] @%-9llu MDIRCV_TAIL [0x101C8] %04X -> %04X (DSP posted ring)\n", st, last_rcv, rcv);
            last_rcv = rcv;
        }
        uint16_t ver = (uint16_t)((m->mem[0x10004 & m->mem_mask] << 8) | m->mem[(0x10004 & m->mem_mask) + 1]);
        if (ver != last_ver) {
            if (last_ver != 0xFFFF) fprintf(stderr, "[init] @%-9llu boot_status [0x10004] %04X -> %04X (DSP version/ready reply)\n", st, last_ver, ver);
            last_ver = ver;
        }
      } }
    if (g_state == 1 && g_cosim) {
        // NOTE: the 5110 DSP is largely RESIDENT mask-ROM — it self-populates its overlay/
        // vector/working RAM from its own DROM at boot (data[0x0F00]/0x500/0x580 etc. get real
        // content the firmware never wrote). The firmware only streams small block descriptors
        // (13-word staging window 0x100-0x10C) + handshake. So we do NOT hold the DSP in reset:
        // free-running, it boots and drains to the 0x408B idle (~23M steps, ~43% idle). Holding
        // it until "upload complete" actually prevents it reaching idle (it boots into a
        // continuous filter loop), so hold is opt-in only (DSP54_HOLD=1, for A/B). The essential
        // fix is on the READ side (c54x_read serves the handshake from dsp_default so the
        // firmware boots + performs its upload); writes still mirror into the C54x (harmless).
        static int released = -1, hold = -1;
        if (hold < 0) hold = getenv("DSP54_HOLD") ? 1 : 0;
        if (released < 0) released = 0;
        if (hold && !released) {
            if (upload_done(m)) {
                released = 1;
                if (g_log) fprintf(stderr, "[dsp54] DSP released from reset at step %lluk "
                                   "(firmware upload complete)\n", (unsigned long long)(m->dsp_steps/1000));
            } else {
                dsp_default_tick(m);   // keep the legacy upload-handshake pump running
                return;                 // DSP held: do not step
            }
        }
        // DSP54_REQWATCH=1 (demand-page request diag): does the DSP ever ASK the MCU for an overlay
        // block? The DSP requests via UPLOADREQUEST [0x871]=word0x71 (also loader1's ready-signal
        // [0x871]=0x12) and the MCU answers via UPLOADREPLY [0x872]. The DSP→MCU "service me" doorbell
        // is the HINT: BSCR(0x29) bit3 toggle -> MCU IRQ4. Poll all three + log every CHANGE (capped),
        // so we can tell whether the producer/codec overlay is never paged in because the DSP never
        // requests it (faithful idle silence) vs. because a request is issued but unanswered (missing
        // signal). data_peek(0x871/0x872/0x87F) returns api_ram (the cells the DSP actually writes).
        {
            static int rw_init = 0, rw_on = 0; static unsigned rw_n = 0;
            static uint16_t rw_req = 0xFFFF, rw_rep = 0xFFFF, rw_hdr = 0xFFFF, rw_bscr = 0xFFFF;
            if (!rw_init) { rw_init = 1; rw_on = getenv("DSP54_REQWATCH") ? 1 : 0; }
            if (rw_on && rw_n < 80) {
                uint16_t req = dsp54_data_peek(g_dsp, 0x871);   // UPLOADREQUEST / ready
                uint16_t rep = dsp54_data_peek(g_dsp, 0x872);   // UPLOADREPLY
                uint16_t hdr = dsp54_data_peek(g_dsp, 0x87F);   // UPLOADHEADER w4 (loader1 polls this)
                uint16_t bscr= dsp54_data_peek(g_dsp, 0x29);    // BSCR (HINT doorbell bit3)
                if (req != rw_req || rep != rw_rep || hdr != rw_hdr || bscr != rw_bscr) {
                    Dsp54Status st; dsp54_status(g_dsp, &st);
                    fprintf(stderr, "[reqwatch] @%lluk pc=0x%04X REQ[0x871]=0x%04X REPLY[0x872]=0x%04X "
                            "HDR[0x87F]=0x%04X BSCR[0x29]=0x%04X(hint.b3=%u)\n",
                            (unsigned long long)(m->dsp_steps/1000), st.pc, req, rep, hdr, bscr, (bscr>>3)&1);
                    rw_req = req; rw_rep = rep; rw_hdr = hdr; rw_bscr = bscr; rw_n++;
                }
            }
        }
        // DSP54_SILENCE=<lo>:<hi> (hex DSP data words): every tick, zero data[lo..hi] to model the
        // codec delivering SILENCE into the FIR input/state buffers. The BIST is a filter that, with
        // real DROM coefficients but uninitialised input, sustains a limit-cycle oscillation forever
        // (never decays to the quiet/done state that exits to idle). Holding its buffers at zero
        // models boot silence so it can converge. Default range 0x13A4:0x1470 (the FIR data buffers
        // past the DARAM scheduler code). EXPERIMENTAL — proves the convergence hypothesis.
        {
            static int sz_init = 0, sz_on = 0; static uint16_t sz_lo = 0x13A4, sz_hi = 0x1470;
            if (!sz_init) { sz_init = 1; const char *p = getenv("DSP54_SILENCE");
                if (p && *p) { sz_on = 1; unsigned lo, hi;
                    if (sscanf(p, "%x:%x", &lo, &hi) == 2) { sz_lo=(uint16_t)lo; sz_hi=(uint16_t)hi; } } }
            if (sz_on) dsp54_zero_range(g_dsp, sz_lo, sz_hi);
        }
        // DSP54_PCTRACE=<step>: once dsp_steps >= step, dump the next N (default 400) consecutive
        // DSP PCs (ratio-1) so the exact steady-state cycle is visible — which the histogram alone
        // can't show (it loses control-flow order).
        {
            static int pt_init = 0, pt_on = 0, pt_left = 0; static uint64_t pt_at = 0;
            if (!pt_init) { pt_init = 1; const char *p = getenv("DSP54_PCTRACE");
                if (p && *p) { pt_on = 1; long v = strtol(p,0,0); if (v>0) pt_at=(uint64_t)v;
                    const char *n = getenv("DSP54_PCTRACEN"); pt_left = n&&*n ? atoi(n) : 400; } }
            if (pt_on && m->dsp_steps >= pt_at && pt_left > 0) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                fprintf(stderr, "%04X ", st.pc);
                if (--pt_left == 0) fprintf(stderr, "\n[dsp54] PCTRACE done\n");
            }
        }
        // DSP54_PCWHEN=0xADDR[,0xADDR2] (R1 timing diag): log dsp_steps the first 16 times the
        // DSP executes each given PC. Pins WHEN the scheduler 0x926 / frame-gate 0x0A0C run vs the
        // MCU's MDISND post (~577k) — i.e. did the one scheduler sweep see an empty ring? Default OFF.
        { static int pw_init = 0; static uint16_t pw_a[2]; static int pw_n = 0; static int pw_hit[2];
          if (!pw_init) { pw_init = 1; const char *p = getenv("DSP54_PCWHEN");
              if (p && *p) { char buf[64]; snprintf(buf,sizeof buf,"%s",p);
                  for (char *t = strtok(buf,","); t && pw_n < 2; t = strtok(0,",")) pw_a[pw_n++] = (uint16_t)strtoul(t,0,0); } }
          if (pw_n) { Dsp54Status st; dsp54_status(g_dsp, &st);
              for (int i = 0; i < pw_n; i++) if (st.pc == pw_a[i] && pw_hit[i] < 16) {
                  pw_hit[i]++;
                  fprintf(stderr, "[pcwhen] PC=0x%04X hit #%d @dsp_steps=%lluk\n",
                          pw_a[i], pw_hit[i], (unsigned long long)(m->dsp_steps/1000)); } } }
        // DSP54_ENQLOG=1 (R1 boot-burst diag): log every MDIRCV enqueue attempt (PROM 0x37CE),
        // the tail-update (0x3803), the doorbell (0x3807), and the secondary-buffer route (0x3812).
        // Captures WHEN the autonomous boot self-test posts + WHERE it routes ([0x6E1] bit2: set =>
        // 0x12C8 secondary buf, clear => MCU MDIRCV ring). Pins whether the boot-burst is misrouted.
        { static int eq_init = 0, eq_on = 0;
          if (!eq_init) { eq_init = 1; eq_on = getenv("DSP54_ENQLOG") ? 1 : 0; }
          if (eq_on) { Dsp54Status st; dsp54_status(g_dsp, &st);
            const char *tag = (st.pc==0x37CE)?"ENQ-ENTRY":(st.pc==0x3803)?"TAIL-UPDATE":
                              (st.pc==0x3807)?"DOORBELL-port1=1":(st.pc==0x3812)?"->SEC-BUF-0x12C8":0;
            if (tag) fprintf(stderr, "[enqlog] %-16s @%lluk [0x6E1]=0x%04X bit2=%u tail[0x8E4]=0x%04X head[0x8E5]=0x%04X\n",
                        tag, (unsigned long long)(m->dsp_steps/1000), dsp54_data_peek(g_dsp,0x6E1),
                        dsp54_data_peek(g_dsp,0x6E1)&4, dsp54_data_peek(g_dsp,0x8E4), dsp54_data_peek(g_dsp,0x8E5)); } }
        // DSP54_WINTRACE=<start>:<count> (R1 flow diag): once dsp_steps>=start, print the next
        // <count> DSP PCs, collapsing consecutive repeats (so a tight loop shows as "PC xN" and the
        // EXIT is visible). Pins what the steady-state self-test/math loop transitions to. Default OFF.
        { static int wt_init = 0; static uint64_t wt_start = 0; static long wt_left = 0;
          static uint16_t wt_last = 0xFFFF; static unsigned wt_rep = 0;
          if (!wt_init) { wt_init = 1; const char *p = getenv("DSP54_WINTRACE");
              if (p && *p) { wt_start = strtoull(p,0,0); const char *c = strchr(p,':'); wt_left = c ? atol(c+1) : 200; } }
          if (wt_left > 0 && m->dsp_steps >= wt_start) { Dsp54Status st; dsp54_status(g_dsp, &st);
              if (st.pc == wt_last) wt_rep++;
              else { if (wt_last != 0xFFFF) { if (wt_rep) fprintf(stderr, "%04Xx%u ", wt_last, wt_rep+1);
                                              else fprintf(stderr, "%04X ", wt_last); }
                     wt_last = st.pc; wt_rep = 0; }
              if (--wt_left == 0) fprintf(stderr, "\n[dsp54] WINTRACE done\n"); } }
        // DSP54_GATELOG: pin the never-draining frame loop. At ratio-1 every insn is observable;
        // sample the PC BEFORE stepping. The audio frame loop is gated at 0xDE10
        // (`if (*AR7- != 0) dgoto 0xdd83`): the loop walks AR7 down through the frame buffer and
        // re-enters the FIR math at 0xDD83 while the word it reads is nonzero. Log AR7 + data[AR7]
        // at the gate (first N hits) + per-window taken/not-taken counts + whether idle is reached.
        {
            static int gl_init = 0, gl_on = 0, gl_n = 0; static uint64_t gl_next = 0;
            static uint64_t gl_taken = 0, gl_nottaken = 0, gl_idle = 0;
            if (!gl_init) { gl_init = 1; gl_on = getenv("DSP54_GATELOG") ? 1 : 0; }
            if (gl_on) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                static int e019_seen = 0;
                if (st.pc == 0xE019 && e019_seen < 6) { e019_seen++;
                    uint16_t dp = st.st0 & 0x1FF;
                    uint16_t a35 = (uint16_t)(dp*0x80 + 0x35);
                    fprintf(stderr, "[dsp54] E019 #%d @%lluk DP=0x%03X -> @35=[0x%04X]=0x%04X accA=0x%08X (C=bit0=%d -> %s)\n",
                            e019_seen, (unsigned long long)(m->dsp_steps/1000), dp, a35,
                            dsp54_data_peek(g_dsp, a35), st.acc_a, st.acc_a&1,
                            (st.acc_a&1)?"LOOP(e02b)":"fall-through");
                }
                static int de37_seen = 0;
                if (st.pc == 0xDE37 && de37_seen < 3) { de37_seen++;
                    fprintf(stderr, "[dsp54] BIST-ENTRY #%d @%lluk pc=0xDE37 sp=0x%04X ret-stack:",
                            de37_seen, (unsigned long long)(m->dsp_steps/1000), st.sp);
                    for (int k = 0; k < 10; k++)
                        fprintf(stderr, " 0x%04X", dsp54_data_peek(g_dsp, (uint16_t)(st.sp + k)));
                    fprintf(stderr, "\n");
                }
                if (st.pc == 0xDE10) {
                    uint16_t ar7 = st.ar[7];
                    uint16_t v   = dsp54_data_peek(g_dsp, ar7);
                    if (v) gl_taken++; else gl_nottaken++;
                    if (gl_n < 40) { gl_n++;
                        fprintf(stderr, "[dsp54] GATE@DE10 #%d AR7=0x%04X *AR7=0x%04X -> %s "
                                "(AR1=0x%04X AR2=0x%04X AR3=0x%04X AR4=0x%04X AR5=0x%04X AR6=0x%04X)\n",
                                gl_n, ar7, v, v ? "loop(dd83)" : "EXIT(de12)",
                                st.ar[1], st.ar[2], st.ar[3], st.ar[4], st.ar[5], st.ar[6]);
                    }
                }
                if (st.pc == 0x407C || st.pc == 0x408C) gl_idle++;
                if (m->dsp_steps >= gl_next) {
                    gl_next = m->dsp_steps + 5000000u;
                    fprintf(stderr, "[dsp54] GATELOG @%lluM: DE10 taken=%llu nottaken=%llu idle(407C/408C)=%llu\n",
                            (unsigned long long)(m->dsp_steps/1000000u),
                            (unsigned long long)gl_taken, (unsigned long long)gl_nottaken,
                            (unsigned long long)gl_idle);
                }
            }
        }
        // DSP54_SPLEAK: per-PC net SP-delta accounting (ratio-1: one insn/tick). Each tick,
        // attribute (sp_after - sp_before) to the PC that executed. The PCs with the largest
        // NEGATIVE net delta are the instructions leaking the stack downward. Dumps at the
        // configured step. Pins the exact leaking instruction (interpreter mis-account vs code).
        {
            static int sl_init = 0, sl_on = 0; static uint64_t sl_at = 8000000u;
            static int32_t *sl = 0; static uint16_t sl_prevpc = 0; static int sl_have = 0;
            static uint16_t sl_prevsp = 0;
            if (!sl_init) { sl_init = 1; const char *p = getenv("DSP54_SPLEAK");
                if (p && *p) { sl_on = 1; long v = strtol(p,0,0); if (v>1) sl_at=(uint64_t)v; sl = calloc(0x10000,sizeof *sl); } }
            if (sl_on && sl) {
                Dsp54Status st0; dsp54_status(g_dsp, &st0);   // pc/sp BEFORE this step
                uint16_t pc_before = st0.pc, sp_before = st0.sp;
                if (!g_realup || g_dsp_run) dsp54_step(g_dsp, g_ratio);
                Dsp54Status st1; dsp54_status(g_dsp, &st1);
                int16_t d = (int16_t)(st1.sp - sp_before);
                if (d) sl[pc_before] += d;
                static long long sl_tot = 0; static uint64_t sl_m2=0,sl_m1=0,sl_p1=0,sl_p2=0,sl_other=0;
                if (d) { sl_tot += d; switch(d){case -2:sl_m2++;break;case -1:sl_m1++;break;case 1:sl_p1++;break;case 2:sl_p2++;break;default:sl_other++;} }
                (void)sl_prevpc; (void)sl_have; (void)sl_prevsp;
                if (m->dsp_steps == sl_at) {
                    fprintf(stderr, "[dsp54] SP-LEAK total net=%lld  hist: push2(-2)=%llu push1(-1)=%llu pop1(+1)=%llu pop2(+2)=%llu other=%llu\n",
                            sl_tot, (unsigned long long)sl_m2,(unsigned long long)sl_m1,(unsigned long long)sl_p1,(unsigned long long)sl_p2,(unsigned long long)sl_other);
                    fprintf(stderr, "[dsp54] SP-LEAK per-PC net delta (top 20 most-negative) @%lluk:\n",
                            (unsigned long long)(m->dsp_steps/1000));
                    long long sum_neg=0, sum_pos=0;
                    for (uint32_t i=0;i<0x10000;i++){ if(sl[i]<0) sum_neg+=sl[i]; else sum_pos+=sl[i]; }
                    fprintf(stderr,"    sum pushes(neg)=%lld sum pops(pos)=%lld\n", sum_neg, sum_pos);
                    fprintf(stderr,"  TOP PUSH PCs (net negative):\n");
                    int32_t saved[40]; uint32_t savedi[40]; int ns=0;
                    for (int k=0;k<12;k++){ int32_t worst=0; uint32_t wi=0;
                        for (uint32_t i=0;i<0x10000;i++) if (sl[i]<worst){worst=sl[i];wi=i;}
                        if (!worst) break;
                        fprintf(stderr,"    pc=0x%04X net=%d\n", wi, worst); saved[ns]=sl[wi]; savedi[ns]=wi; ns++; sl[wi]=0; }
                    fprintf(stderr,"  TOP POP PCs (net positive):\n");
                    for (int k=0;k<12;k++){ int32_t best=0; uint32_t bi=0;
                        for (uint32_t i=0;i<0x10000;i++) if (sl[i]>best){best=sl[i];bi=i;}
                        if (!best) break;
                        fprintf(stderr,"    pc=0x%04X net=+%d\n", bi, best); sl[bi]=0; }
                    for (int k=0;k<ns;k++) sl[savedi[k]]=saved[k];
                }
                goto spleak_done;
            }
        }
        { // DSP54_PCTRACE=N (temp diag): print the DSP PC before each of the first N steps
          static int pt = -1; static long ptn = 0;
          if (pt < 0) { const char *p = getenv("DSP54_PCTRACE"); pt = (p && *p) ? atoi(p) : 0; }
          if (pt && ptn < pt) { Dsp54Status st; dsp54_status(g_dsp, &st);
            uint16_t opc = (st.pc >= 0x80 && st.pc < 0x2800) ? dsp54_data_peek(g_dsp, st.pc) : 0xFFFF;
            fprintf(stderr, "[pctrace %ld] xpc=%u pc=0x%04X op=0x%04X sp=0x%04X st1=0x%04X INTM=%d\n",
                    ptn, st.xpc, st.pc, opc, st.sp, st.st1, (st.st1 >> 11) & 1); ptn++; } }
        // DSP54_LOADERGO=<cmd> (diag): once the DSP is parked in loader1's [0x872] command-wait
        // (PC 0x0F3D-0x0F41), poke api_ram[0x72] = cmd (the cb_reply the loader reads as [0x872])
        // to release it -> loader2 (0xD80) run mode. Confirms the boot->run handshake path before
        // wiring the real MCU cb_reply delivery. One-shot.
        { static int lg = -2; static int done = 0;
          if (lg == -2) { const char *p = getenv("DSP54_LOADERGO");
              lg = (p && *p && dsp54_static_allowed("DSP54_LOADERGO")) ? (int)strtol(p,0,0) : -1; }
          if (lg >= 0 && !done) { Dsp54Status st; dsp54_status(g_dsp, &st);
            if (st.pc >= 0x0F3D && st.pc <= 0x0F41) {
                dsp54_api_poke(g_dsp, 0x72, (uint16_t)lg); done = 1;
                if (g_log) fprintf(stderr, "[dsp54] LOADERGO: poked api_ram[0x72]=0x%04X (release loader1) @step %lluk\n",
                                   lg, (unsigned long long)(m->dsp_steps/1000)); } } }
        if (!g_realup || g_dsp_run)   // REALUP: step only while the MCU has released the DSP (0x20002 bit0)
            dsp54_step(g_dsp, g_ratio);
        spleak_done:;
        // DSP54_DSPHALT: the DSP froze at the armed PC — dump the full live memory map + regs ONCE,
        // then stop the process so we can disassemble what is REALLY there (DSP54_DSPHALTGO=1 to
        // dump-and-continue instead of exiting).
        { static int halt_dumped = 0;
          if (!halt_dumped && dsp54_halted(g_dsp)) {
              halt_dumped = 1;
              const char *pre = getenv("DSP54_DSPHALTDUMP"); if (!pre || !*pre) pre = "/tmp/dsphalt";
              Dsp54Status st; dsp54_status(g_dsp, &st);
              dsp54_dump_full(g_dsp, pre);
              fprintf(stderr, "\n[dsp54] *** DSP HALTED at PC=0x%04X  sp=0x%04X st1=0x%04X INTM=%d xpc=%u "
                      "pmst=0x%04X @dsp_steps=%lluk ***\n", st.pc, st.sp, st.st1, (st.st1 >> 11) & 1, st.xpc,
                      st.pmst, (unsigned long long)(m->dsp_steps / 1000));
              fprintf(stderr, "[dsp54] full live memory map -> %s.{prog,api,data,regs}\n", pre);
              fprintf(stderr, "[dsp54] disasm: tools/dsp/disasm_dump.py %s 0xADDR N   "
                      "(auto: api for [0x800,0x2800), prog elsewhere)\n\n", pre);
              if (!getenv("DSP54_DSPHALTGO")) { fflush(0); exit(0); }
          } }
        // === DSP54_HSLOG / DSP54_HSHALT — MCU<->DSP loader1 upload-handshake monitor ===
        // The block-upload handshake (MCU 0x271B00 loop) is a double-buffer mailbox protocol:
        //   MCU copies a block into the HPI window [0x10200..], then per block-parity signals a
        //   mailbox and SPIN-WAITS (unbounded beq) for loader1 to consume+clear it:
        //     MBOX0 = MCU[0x100FE] <-> DSP word 0x87F (api_ram[0x7F])   (even blocks)
        //     MBOX1 = MCU[0x10100] <-> DSP word 0x880 (api_ram[0x80])   (odd blocks)
        //   loader1 polls 0x87F (f17: ld *(0x87f); bc 0xf17,aneq), CRCs the buffer (call 0x8023),
        //   then clears the mailbox so the MCU advances. A timing race / non-coherent window =
        //   loader1 never sees the signal (mcu-view != dsp-view) OR never clears it = DEADLOCK.
        // HSLOG=1 logs every state-machine TRANSITION (mbox values BOTH views + loader1 phase),
        //   so the exact ordering "MCU set MBOX0 @T1 / loader1 read @T2 / cleared @T3-or-never" is
        //   visible. HSHALT=0xDSPPC freezes + dumps a combined state block when the DSP hits that
        //   PC (e.g. 0x8023 = first CRC, 0xF17 = first mailbox poll) and exits. Read-only.
        {
            static int hs_init = 0, hs_log = 0; static long hs_halt = -1;
            static uint64_t hs_after = 0;
            static uint16_t p_m0m=0xFFFF,p_m0d=0xFFFF,p_m1m=0xFFFF,p_m1d=0xFFFF,
                            p_cmd=0xFFFF,p_sts=0xFFFF,p_vAm=0xFFFF,p_vAd=0xFFFF;
            static int p_phase=-1; static int hs_done=0;
            if (!hs_init) { hs_init = 1;
                hs_log = getenv("DSP54_HSLOG") ? 1 : 0;
                const char *h = getenv("DSP54_HSHALT"); if (h && *h) hs_halt = strtol(h,0,0);
                const char *a = getenv("DSP54_HSAFTER"); if (a && *a) hs_after = strtoull(a,0,0);
            }
            if ((hs_log || hs_halt >= 0) && m->dsp_steps >= hs_after) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                uint16_t m0m = mcu_word(m, 0x100FEu), m0d = dsp54_data_peek(g_dsp, 0x87F);
                uint16_t m1m = mcu_word(m, 0x10100u), m1d = dsp54_data_peek(g_dsp, 0x880);
                uint16_t cmd = dsp54_data_peek(g_dsp, 0x872), sts = dsp54_data_peek(g_dsp, 0x871);
                // version slots: MCU's bounded readiness gate reads [0x10004]/[0x10006] (0x271ABE/
                // 0x271AF4); DSP-side cells are 0x802/0x803. Shows WHO writes the version (real DSP
                // loader1 vs the HLE) and whether the two views agree.
                uint16_t vAm = mcu_word(m, 0x10004u), vAd = dsp54_data_peek(g_dsp, 0x802);
                uint16_t vBm = mcu_word(m, 0x10006u), vBd = dsp54_data_peek(g_dsp, 0x803);
                // loader1 execution phase (so we don't log on every PC tick):
                //  0=loader1-init/clear (<0xF17) 1=mailbox-poll(0xF17-0xF21) 2=CRC(0x8023 region)
                //  3=cmd-wait(0xF3D-0xF41) 4=parked(0xF6A+) 5=RESIDENT/BIST(>=0x2800) 6=other-window
                int ph;
                uint16_t pc = st.pc;
                if      (pc >= 0x2800)               ph = 5;
                else if (pc >= 0x8000 && pc < 0x8060) ph = 2;            // 0x8023 CRC routine
                else if (pc >= 0x0F17 && pc <= 0x0F21) ph = 1;
                else if (pc >= 0x0F3D && pc <= 0x0F41) ph = 3;
                else if (pc >= 0x0F6A && pc <  0x0F80) ph = 4;
                else if (pc <  0x0F17)                ph = 0;
                else                                  ph = 6;
                int changed = (m0m!=p_m0m)||(m0d!=p_m0d)||(m1m!=p_m1m)||(m1d!=p_m1d)||
                              (cmd!=p_cmd)||(sts!=p_sts)||(ph!=p_phase)||(vAm!=p_vAm)||(vAd!=p_vAd);
                if (hs_log && changed) {
                    static const char *PHN[7] = {"init","POLL","CRC","CMD-WAIT","PARK","RESIDENT/BIST","win"};
                    const char *coh0 = (m0m==m0d)?"":" !M0COH"; const char *coh1 = (m1m==m1d)?"":" !M1COH";
                    const char *cohV = (vAm==vAd)?"":" !VCOH";
                    fprintf(stderr,
                      "[hs %8lluk] dpc=0x%04X %-13s | M0 mcu=%04X dsp=%04X%s | M1 mcu=%04X dsp=%04X%s | sts=%04X cmd=%04X | ver[4]m=%04X d=%04X[6]m=%04X d=%04X%s\n",
                      (unsigned long long)(m->dsp_steps/1000), st.pc, PHN[ph],
                      m0m,m0d,coh0, m1m,m1d,coh1, sts, cmd, vAm,vAd,vBm,vBd, cohV);
                }
                p_m0m=m0m;p_m0d=m0d;p_m1m=m1m;p_m1d=m1d;p_cmd=cmd;p_sts=sts;p_phase=ph;
                p_vAm=vAm;p_vAd=vAd;
                if (hs_halt >= 0 && !hs_done && st.pc == (uint16_t)hs_halt) {
                    hs_done = 1;
                    fprintf(stderr, "\n=== DSP54_HSHALT — DSP reached 0x%04X @dsp_steps=%lluk ===\n",
                            (unsigned)hs_halt, (unsigned long long)(m->dsp_steps/1000));
                    fprintf(stderr, "  DSP regs: pc=0x%04X sp=0x%04X st0=0x%04X st1=0x%04X imr=%04X ifr=%04X pmst=%04X xpc=%u INTM=%d\n",
                            st.pc, st.sp, st.st0, st.st1, st.imr, st.ifr, st.pmst, st.xpc, (st.st1>>11)&1);
                    fprintf(stderr, "  DSP AR0..7:");
                    for (int i=0;i<8;i++) fprintf(stderr," %04X", st.ar[i]);
                    fprintf(stderr, "  A=0x%08X\n", st.acc_a);
                    fprintf(stderr, "  MBOX0 0x87F: mcu[0x100FE]=%04X dsp[0x7F]=%04X %s\n",
                            m0m, m0d, (m0m==m0d)?"COHERENT":"*** INCOHERENT (MCU signal not in DSP window) ***");
                    fprintf(stderr, "  MBOX1 0x880: mcu[0x10100]=%04X dsp[0x80]=%04X %s\n",
                            m1m, m1d, (m1m==m1d)?"COHERENT":"*** INCOHERENT ***");
                    fprintf(stderr, "  status 0x871=%04X  cmd 0x872=%04X  ver[0x10004]=%04X [0x10006]=%04X\n",
                            sts, cmd, mcu_word(m,0x10004u), mcu_word(m,0x10006u));
                    fprintf(stderr, "  upload buf [0x10200..] mcu:");
                    for (int i=0;i<8;i++) fprintf(stderr," %04X", mcu_word(m, 0x10200u + 2u*i));
                    fprintf(stderr, "\n  bufA 0x900 dsp:");
                    for (int i=0;i<8;i++) fprintf(stderr," %04X", dsp54_data_peek(g_dsp, 0x900+i));
                    fprintf(stderr, "\n=== HSHALT end (DSP54_HSHALT) ===\n");
                    fflush(stderr);
                    if (getenv("DSP54_HSSTOP")) exit(0);
                }
            }
        }
        // DSP54_SNAPSHOT=<path>: freeze the whole DSP core (regs+prog+data+api_ram
        // OVLY window) to a file for fast standalone replay (tools/dsp/dsp_replay).
        // Trigger: DSP54_SNAPPC=<dsp pc> (first hit, after DSP54_SNAPAFTER steps) OR
        // DSP54_SNAPAT=<dsp_step>. DSP54_SNAPSTOP=1 ends the run right after saving.
        // Lets us snapshot the post-upload / pre-BIST state once, then iterate on the
        // BIST in seconds instead of re-running the ~6-min co-sim each time.
        {
            static int   snp_init = 0, snp_stop = 0, snp_done = 0;
            static const char *snp_path = NULL;
            static long  snp_pc = -1; static uint64_t snp_at = 0, snp_after = 0;
            if (!snp_init) { snp_init = 1;
                snp_path = getenv("DSP54_SNAPSHOT");
                const char *p = getenv("DSP54_SNAPPC"); if (p && *p) snp_pc = strtol(p,0,0);
                const char *a = getenv("DSP54_SNAPAT"); if (a && *a) snp_at = strtoull(a,0,0);
                const char *g = getenv("DSP54_SNAPAFTER"); if (g && *g) snp_after = strtoull(g,0,0);
                snp_stop = getenv("DSP54_SNAPSTOP") ? 1 : 0;
            }
            if (snp_path && !snp_done && m->dsp_steps >= snp_after) {
                int hit = 0;
                if (snp_at && m->dsp_steps >= snp_at) hit = 1;
                else if (snp_pc >= 0) { Dsp54Status st; dsp54_status(g_dsp, &st);
                                        if (st.pc == (uint16_t)snp_pc) hit = 1; }
                if (hit) {
                    snp_done = 1;
                    int rc = dsp54_snapshot(g_dsp, snp_path);
                    Dsp54Status st; dsp54_status(g_dsp, &st);
                    fprintf(stderr, "[dsp54] SNAPSHOT %s @dsp_steps=%lluk pc=0x%04X (rc=%d)\n",
                            snp_path, (unsigned long long)(m->dsp_steps/1000), st.pc, rc);
                    if (snp_stop) { fprintf(stderr, "[dsp54] SNAPSHOT done — DSP54_SNAPSTOP, exiting.\n");
                                    fflush(stderr); exit(rc ? 1 : 0); }
                }
            }
        }
        // DSP54_CMDDEFER=1 (experiment, §8q): the timing wall — the MCU streams MDISND
        // commands at ~577k steps but the real DSP needs ~19.5M insns to reach idle/ready, so
        // write-time delivery (CMDBRIDGE) lands them in the DSP's boot scratch and is lost.
        // Defer instead: once the DSP has reached its idle loop (PC 0x407C/0x408C/0x408B),
        // mirror the MCU's MDISND command region (data[0x00..0x53] = where the MCU's writes
        // landed) into api_ram every tick so the now-READY DSP can dequeue the accumulated
        // ring + advanced tail. Models a faithful ready-gated command delivery (the real MCU
        // waits for DSP-ready before sending). Default OFF. Needs HPIAPI (MCU reads back).
        {
            static int cd_init = 0, cd_on = 0, cd_ready = 0;
            if (!cd_init) { cd_init = 1; cd_on = getenv("DSP54_CMDDEFER") ? 1 : 0; }
            if (cd_on) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                if (!cd_ready && (st.pc == 0x407C || st.pc == 0x408C || st.pc == 0x408B)) {
                    cd_ready = 1;
                    if (g_log) fprintf(stderr, "[dsp54] CMDDEFER: DSP reached idle @dsp_steps=%lluk — "
                                       "begin MDISND command sync\n", (unsigned long long)(m->dsp_steps/1000));
                }
                if (cd_ready)
                    for (uint16_t k = 0; k <= 0x52; k++)  // ring + tail(0x52); NOT head(0x53) = DSP's read ptr
                        dsp54_api_poke(g_dsp, k, dsp54_data_peek(g_dsp, k));
            }
        }
        // DSP54_INJMDISND=1 (proof experiment, §8q): the firmware only ever sends cmd 0x200
        // (version, bit9) via port1; it never sends the MDISND-ENABLE command (cmd bit1 ->
        // PROM 0x3621 -> *(866h)|=1). Test whether DRIVING that gate cascades: once the DSP
        // idles, stage a bit1 host command in the port mailbox (port1=0x0002, port2=0) and
        // pulse the host interrupt so the DSP's cmd ISR (0x35B9) routes to 0x3621. If the gate
        // cascades, [0x866] bit0 sets -> MDISND dequeue runs -> [0x6E4] task bits -> producers
        // -> enqueue -> port1=1 -> FIQ0. One-shot, post-idle. Default OFF.
        {
            // The host-cmd ISR (0x35B9) dispatches FIRST-MATCH by cmd bit, so a single port1
            // value reaches only ONE handler. The cascade needs TWO commands (P3/§8s):
            //   cmd bit4 (0x10, in the 0x1C group) -> 0x362F -> 0x363C *(6e4h)|=0x40  (lifts the
            //     idle gate 0x0A0B so the frame-task chain 0x0A44/0x0A8B/0x0AF6 -> 0x37CE runs)
            //   cmd bit1 (0x02)                    -> 0x3621 *(866h)|=1               (enables the
            //     MDISND dequeue 0x394F so the streamed command ring drains)
            // So cycle the injected port1 through a sequence (default {0x10, 0x02}); each ISR run
            // latches one and OR-sets its sticky gate. DSP54_INJCMDS overrides (comma list).
            static int ij_init = 0, ij_on = 0, ij_armed = 0, ij_left = 0, ij_pulse = 0;
            static uint16_t ij_seq[8]; static int ij_nseq = 0;
            if (!ij_init) {
                ij_init = 1; ij_on = getenv("DSP54_INJMDISND") ? 1 : 0;
                const char *s = getenv("DSP54_INJCMDS");
                if (s && *s) { char buf[64]; snprintf(buf,sizeof buf,"%s",s);
                    for (char *t = strtok(buf,","); t && ij_nseq < 8; t = strtok(0,","))
                        ij_seq[ij_nseq++] = (uint16_t)strtoul(t,0,0); }
                if (!ij_nseq) { ij_seq[0] = 0x0010; ij_seq[1] = 0x0002; ij_nseq = 2; }
            }
            if (ij_on) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                if (!ij_armed && (st.pc == 0x407C || st.pc == 0x408C || st.pc == 0x408B)) {
                    ij_armed = 1; ij_left = 4000;   // re-pulse window (single edge lost under INTM=1)
                    if (g_log) fprintf(stderr, "[dsp54] INJMDISND: DSP idle @dsp_steps=%lluk — "
                                       "begin %d-cmd injection window (seq[0]=0x%04X)\n",
                                       (unsigned long long)(m->dsp_steps/1000), ij_nseq, ij_seq[0]);
                }
                if (ij_armed && ij_left > 0) {
                    ij_left--;
                    uint16_t cmd = ij_seq[(ij_pulse++) % ij_nseq];
                    mcu_set_word(m, g_port1_addr, cmd);      // cmd bits (cycled)
                    mcu_set_word(m, g_port2_addr, 0x0000);   // port2 clear so (~port2)&port1 = cmd
                    dsp54_host_interrupt(g_dsp, g_intvec);    // re-pulse host-cmd ISR (INT2 -> 0x35B9)
                }
            }
        }
        // DSP54_HINTIRQ4 (faithful default ON under cosim): wire the DSP->host HINT doorbell to
        // MCU IRQ4. The run-mode bootstrap (loader2 @0xF00) signals "ready" by pulsing BSCR(0x29)
        // bit3 (0xF30 set / 0xF33 clear) then parks at 0xF3D waiting for the MCU's run-mode go-
        // command in [0x872]. That doorbell is the HINT line -> MCU IRQ4 (handler 0x2BCEF0, the DSP
        // mailbox), which delivers the go-command AFTER the DSP cleared [0x872] in its own init
        // (0xF2A). Without this wiring the doorbell is invisible: the MCU only writes [0x872]=2 once,
        // BEFORE release (0x271D1A), the DSP clobbers it, and the DSP spins at 0xF3D forever. The
        // core counts bit3 rising edges (g_c54x_bscr_hint_edges); latch IRQ4 each time it advances.
        // DSP54_SEEDDARAM (faithful default ON under cosim): loader2's bootstrap rptz-clears DARAM
        // 0x1000..0x2FFF, wiping the resident dispatch/overlay code at 0x2000..0x2800 (present in the
        // image's prog[], absent from api_ram). Nothing copies it back, so the run-mode `call 0x2470`
        // hits an empty DARAM cell, never returns, and the stack runs away into the MMRs (IMR->0).
        // Reload the resident DARAM from the image once the DSP branches into run-mode (0xD80, after
        // the clear). Models the chip's DARAM being (re)loaded from ROM.
        { static int sd_init = 0, sd_on = 0, sd_done = 0;
          if (!sd_init) { sd_init = 1; sd_on = dsp54_faithful("DSP54_SEEDDARAM"); }
          if (sd_on && !sd_done && g_dsp_run) {
              Dsp54Status st; dsp54_status(g_dsp, &st);
              // First time the DSP executes in the run-mode PROM (>=0x2800, persistently hit so the
              // per-tick sample can't miss it at RATIO>1) — loader2's clear is done by then.
              if (st.pc >= 0x2800 && st.pc < 0x8000) {
                  uint32_t n = dsp54_seed_daram_from_prog(g_dsp, 0x2000, 0x2800);
                  sd_done = 1;
                  if (g_log) fprintf(stderr, "[dsp54] SEEDDARAM: reloaded %u resident DARAM words "
                                     "[0x2000,0x2800) at run-mode entry @dsp_steps=%lluk\n",
                                     n, (unsigned long long)(m->dsp_steps/1000)); } }
        }
        { static int hi_init = 0, hi_on = 0; static unsigned long long hi_seen = 0, hi_seen_cl = 0;
          if (!hi_init) { hi_init = 1; hi_on = dsp54_faithful("DSP54_HINTIRQ4"); }
          if (hi_on && g_dsp_run) {
              unsigned long long e = dsp54_hint_edges(g_dsp);
              if (e != hi_seen) {
                  hi_seen = e;
                  mad2_raise_irq(m, 4);   // IRQ4 = DSP mailbox doorbell (handler 0x2BCEF0)
                  if (g_log) { static unsigned n; if (++n <= 16)
                      fprintf(stderr, "[dsp54] HINT->IRQ4 #%u: DSP BSCR.b3 doorbell (edge %llu) -> "
                              "MCU IRQ4 @dsp_steps=%lluk\n", n, e,
                              (unsigned long long)(m->dsp_steps/1000)); } }
              if (g_commlog < 0) g_commlog = getenv("DSP54_COMMLOG") ? atoi(getenv("DSP54_COMMLOG")) : 0;
              if (g_commlog && e != hi_seen_cl) { hi_seen_cl = e;
                  fprintf(stderr, "[comm] @%-7lluk M<-D HINT       BSCR.b3 doorbell -> MCU IRQ4 (edge %llu)   dsp_pc=0x%04X\n",
                          (unsigned long long)(m->dsp_steps/1000), e, comm_dsp_pc()); } }
        }
        // DSP54_COMMLOG (D->M): change-detect the DSP-written comm cells — the MDIRCV producer ring
        // (tail/head), the DSP's version reply, and the loader2 ready flag. Logged with the DSP PC at
        // detection (MCU-written cells like MDISND/REPLY are logged on the M->D write side instead).
        { if (g_commlog < 0) g_commlog = getenv("DSP54_COMMLOG") ? atoi(getenv("DSP54_COMMLOG")) : 0;
          if (g_commlog && g_dsp_run) {
              static const uint16_t W[] = { 0xE4, 0xE5, 0x02, 0x03, 0x71 };
              static uint16_t last[5]; static uint8_t seen[5];
              for (int i = 0; i < 5; i++) {
                  uint16_t cur = dsp54_data_peek(g_dsp, (uint16_t)(0x800u + W[i]));
                  if (!seen[i] || last[i] != cur) {
                      if (seen[i])  // skip the initial sighting (not a transition)
                          fprintf(stderr, "[comm] @%-7lluk M<-D %-14s [k=%03X DSP %04X] 0x%04X->0x%04X  dsp_pc=0x%04X\n",
                                  (unsigned long long)(m->dsp_steps/1000), comm_kname(W[i]), W[i], 0x800u+W[i],
                                  last[i], cur, comm_dsp_pc());
                      seen[i] = 1; last[i] = cur; } } } }
        // DSP54_RINGFIQ (faithful default under cosim): the DSP->MCU MDI doorbells are RING-POINTER
        // ADVANCES, exactly as MADos models them (ref/Nok-MADos-master/include/hw/int.h: FIQ_MDIRCV=0,
        // FIQ_MDISND=1; hw/mdi.c). The MAD ASIC raises the MCU FIQ when the DSP moves a ring pointer:
        //   - DSP advances the MDIRCV tail [0x8E4] (posts a reply)  -> MCU FIQ0 (FIQ_MDIRCV, data-ready)
        //   - DSP advances the MDISND head [0x853] (consumes a cmd) -> MCU FIQ1 (FIQ_MDISND, space-freed)
        // This is the hardware-faithful replacement for the FIQ0OUT pa1-write event-hack: the DSP's pa1
        // writes carry STATUS words (0x0700/0x0100/0x0000 @0x8DA8/0x9077/0x395B), NOT FIQ bitmasks, so
        // both the "event" and "value" port models were wrong. Byte-identical at standby: the rings do
        // NOT advance (the DSP posts/consumes nothing), so no FIQ is raised. The MCU writes only the
        // MDISND TAIL [0x852] and the boot-time ring inits, never the MDIRCV tail or MDISND head post-
        // init, so every advance seen here is genuinely the DSP's.
        // PERF (cosim window-dirty gate): read+clear the dirty flag ONCE here and reuse it for
        // BOTH per-tick window monitors (RINGFIQ below + the cmdlevel_eval sample further down).
        // On a quiescent tick — no DSP/MCU write to a signal cell since the last eval — the
        // ring pointers (0x8E4/0x853) and the cmdlevel inputs (port1/port2, MDISND tail/head)
        // are unchanged, so both monitors are no-ops (no edge, no FIQ); skip their window peeks.
        // Read ONCE (not once per block) so clearing it here can't hide the change from cmdlevel.
        // Set by every DSP api_ram store (calypso data_write) + the MCU-side dct3 write helpers,
        // so it is a conservative superset — byte-identical is the oracle (make guard 5110 cosim).
        int win_dirty = dsp54_window_dirty_clear(g_dsp);
        { static int rf_init = 0, rf_on = 0; static uint16_t rf_rcv = 0xFFFF, rf_snd = 0xFFFF;
          if (!rf_init) { rf_init = 1; rf_on = dsp54_faithful("DSP54_RINGFIQ"); }
          if (rf_on && g_dsp_run && win_dirty) {
              uint16_t rcv = dsp54_data_peek(g_dsp, 0x8E4);   // MDIRCV tail: DSP posts -> advances
              uint16_t snd = dsp54_data_peek(g_dsp, 0x853);   // MDISND head: DSP consumes -> advances
              if (rf_rcv == 0xFFFF) rf_rcv = rcv;             // prime on first sight (not a transition)
              else if (rcv != rf_rcv) { mad2_raise_fiq(m, 0); rf_rcv = rcv;   // FIQ0 = MDIRCV
                  if (g_log) { static unsigned n; if (++n <= 16) fprintf(stderr,
                      "[dsp54] RINGFIQ FIQ0: MDIRCV tail -> 0x%02X (DSP posted) @dsp_steps=%lluk\n",
                      rcv & 0xFF, (unsigned long long)(m->dsp_steps/1000)); } }
              if (rf_snd == 0xFFFF) rf_snd = snd;
              else if (snd != rf_snd) { mad2_raise_fiq(m, 1); rf_snd = snd;   // FIQ1 = MDISND
                  if (g_log) { static unsigned n; if (++n <= 16) fprintf(stderr,
                      "[dsp54] RINGFIQ FIQ1: MDISND head -> 0x%02X (DSP consumed) @dsp_steps=%lluk\n",
                      snd & 0xFF, (unsigned long long)(m->dsp_steps/1000)); } }
          }
        }
        // DSP54_FRAMETICK=1 (EXPLICIT opt-in — NOT faithful-default yet, it changes the cosim boot):
        // the DSP's autonomous poll cadence is the external MAD2 frame-sync interrupt INT0 (vec16 ->
        // frame ISR 0x3204, which reads the frame-status port pa33). MADos confirms mdi_send rings NO
        // doorbell (ref/Nok-MADos-master/hw/mdi.c) — the MCU just advances the MDISND tail and the DSP
        // consumes the ring on its own frame cadence. Without a frame tick the DSP never re-polls after
        // the initial host-cmd arm (the cosim faked the wake via the [0x20008]->vec25 hack). Period =
        // DSP54_FRAMEPER dsp_steps (default 50000). Kept OFF by default so the tested config stays
        // byte-identical; promote to faithful-default only once it's proven to drive consumption.
        { static int ft_init = 0, ft_on = 0; static long ft_per = 50000; static unsigned long long ft_next = 0;
          if (!ft_init) { ft_init = 1; ft_on = getenv("DSP54_FRAMETICK") ? 1 : 0;
              const char *p = getenv("DSP54_FRAMEPER"); if (p && *p) ft_per = strtol(p, 0, 0);
              if (ft_per < 1) ft_per = 1; }
          if (ft_on && g_dsp_run) {
              if (ft_next == 0) ft_next = m->dsp_steps + (unsigned long long)ft_per;
              else if (m->dsp_steps >= ft_next) {
                  ft_next = m->dsp_steps + (unsigned long long)ft_per;
                  dsp54_host_interrupt(g_dsp, 16);   // INT0 = MAD2 frame sync (vec16 -> 0x3204)
                  if (g_log) { static unsigned n; if (++n <= 8) fprintf(stderr,
                      "[dsp54] FRAMETICK: INT0 (vec16 frame) @dsp_steps=%lluk (period=%ld)\n",
                      (unsigned long long)(m->dsp_steps/1000), ft_per); }
              }
          }
        }
        // === DSP54_COBBA — fake COBBA-GJ codec: the codec FRAME CLOCK (vec20 RINT0) =========
        // RE'd 2026-06-11: MAD2 audio = the COBBA codec over I/O-port 0x21 (read=mic ADC,
        // write=ear DAC), clocked per sample by the vec20 RINT0 serial ISR (0x3416 -> 0x3558
        // -> tone gen 0xA617 / FIR). The C54x transmit vectors are STUBS, so there is no McBSP
        // transmit — COBBA *is* the frame clock + the data exchange. Our cosim has no codec, so
        // vec20 never fires and the audio loop never runs (tone synthesizer proven via INJTONE
        // but can't sustain without a frame clock — see memory 5110-dsp-tone-synthesis-runs).
        // This models the minimum faithful COBBA: a continuous vec20 at the codec sample rate
        // while in run-mode, so a tone the MCU legitimately starts (keypress with tones-on ->
        // 0xA598 setup) is clocked frame-by-frame. ADC input + DAC sink are the port handlers
        // (c54x_port_read/write: 0x21/0x27 read = COBBA_IN silence/test, 0x21 write = captured
        // by PCMCAP). Period = DSP54_COBBA_FRAMEPER dsp_steps (default 1000 ~= 8 kHz at the
        // cosim clock; tune to taste). Default OFF; promote once proven to drive a clean tone.
        // Codec-active flag, computed AT MOST ONCE per tick and SHARED by the COBBA frame block
        // and the BSP loopback block below (each used to peek [0xFE]/[0xAA] itself — 4 peeks/tick
        // for the same 2 cells). -1 = not yet computed this tick.
        int audio_raw = -1;
        { static int cb_init = 0, cb_on = 0; static long cb_per = 0; static unsigned long long cb_next = 0;
          if (!cb_init) { cb_init = 1; cb_on = dsp54_faithful("DSP54_COBBA");
              // FRAME CLOCK = the codec sample rate in EMULATED (rtc_mono) time, NOT dsp_steps.
              // The codec runs at DCT3_CODEC_HZ; one vec20 frame = one sample, so fire every
              // DCT3_ARM_HZ/DCT3_CODEC_HZ rtc_mono cycles (~697 @ 13 MHz / 18642 Hz). The old
              // cb_per=1000 dsp_steps was an arbitrary clock that worked out to ~5 kHz in real
              // time (dsp_steps is an INSTRUCTION counter, ~2.6 cyc each) -> the DSP produced
              // samples 3.7x too slowly for an 18.6 kHz device, so the GUI ring starved between
              // frames and the tone played as fast taps. Clocking the frame off rtc_mono at the
              // true rate makes the emulated-time tone genuinely 900 Hz and feeds the device at
              // its own rate. (At sub-realtime cosim speed the ring can still under-run — that is
              // a host-speed ceiling, separate from this rate match.) DSP54_COBBA_FRAMEPER
              // overrides the period in rtc_mono cycles (A/B).
              cb_per = (long)(13000000L / (long)DCT3_CODEC_HZ);   // DCT3_ARM_HZ / codec rate
              const char *p = getenv("DSP54_COBBA_FRAMEPER"); if (p && *p) cb_per = strtol(p, 0, 0);
              if (cb_per < 1) cb_per = 1; }
          // Gate on codec-ACTIVE: the vec20 RINT0 ISR (0x3416) must NOT run during boot (it
          // corrupts the cosim boot sequence — uniform screen). On real HW the codec is set up
          // before the DSP services frames; here we fire only once audio is started, i.e. the
          // tone-active flag [0xFE] bit0 (set by 0xA598 on an enabled tone) or codec mode [0xAA]
          // is nonzero. DSP54_COBBA_FORCE=1 fires unconditionally (the old always-on, for A/B).
          if (cb_on && g_dsp_run) {
              static int cf = -1; if (cf < 0) cf = getenv("DSP54_COBBA_FORCE") ? 1 : 0;
              if (audio_raw < 0) audio_raw = (dsp54_data_peek(g_dsp, 0x00FE) & 1)
                                          || (dsp54_data_peek(g_dsp, 0x00AA) != 0);
              int audio_active = cf || audio_raw;
              if (!audio_active) { cb_next = 0; }   // idle: re-arm on next activation
              else if (cb_next == 0) cb_next = m->rtc_mono + (unsigned long long)cb_per;
              else if (m->rtc_mono >= cb_next) {
                  cb_next = m->rtc_mono + (unsigned long long)cb_per;
                  // FIXED-RATE PCM RESAMPLE (the pitch/warble fix, 2026-06-14). The codec FRAME
                  // is the sample clock: the DSP writes exactly one earpiece sample to DXR (MMR
                  // 0x21) per vec20 frame, and a frame is a fixed cb_per rtc_mono cycles — uniform
                  // in emulated time. So capture the HELD DXR once per frame, here, with NO dedup.
                  // The old per-write tap (`if (dxr != pc_last)`) DROPPED repeated samples (flat parts
                  // of the wave / low-amplitude envelope) and fired on c54x_tick's irregular
                  // instruction cadence -> non-uniform spacing -> the captured 900 Hz beep read
                  // ~980 Hz and warbled (period jittered 13-27 samples/cycle vs a flat 20.7). One
                  // sample per frame here is the firmware's true sample stream: a rock-steady
                  // 65536/0x0C5C = 20.713 samples/cycle (the keypad-beep phase increment 0x0C5C is
                  // baked in the DSP; play at DCT3_CODEC_HZ to reproduce its pitch). DXR holds the
                  // PREVIOUS frame's output (the vec20 ISR writes it a few hundred steps later), so
                  // this is a clean one-frame-delayed feed. Gated like the frame itself (COBBA on +
                  // audio_active), so standby stays silent. pcm_capture self-gates the file/sink.
                  { static int pc_on = -1; if (pc_on < 0) pc_on = getenv("DSP54_PCMCAP") ? 1 : 0;
                    if (pc_on || m->pcm_sink) pcm_capture(m, 0x21, dsp54_data_peek(g_dsp, 0x21)); }
                  dsp54_host_interrupt(g_dsp, 20);   // RINT0 = COBBA codec frame (vec20 -> 0x3416)
                  if (g_log) { static unsigned n; if (++n <= 8) fprintf(stderr,
                      "[dsp54] COBBA: codec frame RINT0 (vec20) @dsp_steps=%lluk (audio active)\n",
                      (unsigned long long)(m->dsp_steps/1000)); }
              }
          }
        }
        // === DSP54_COBBA — COBBA BSP serial DIGITAL LOOPBACK (DXR -> DRR) ===================
        // RE'd 2026-06-12 (docs/research/5110-, phase-2): the cmd-0x70
        // self-test's loader2 routine (demand-page block 18, flash 0x298934, dest 0x0D80) runs
        // a TWO-PHASE COBBA codec check that folds the verdict cell data[0x06F9] (== MCU verdict
        // byte[9] = [0x101441]; gate = byte9&3==0):
        //   phase 1 (0x0E35-0x0E47): walking-bit loopback over the bit-banged COBBA SERIAL link
        //            (I/O ports 0x2C select / 0x2D data; helpers 0x45C2 W / 0x4610 R) — ALREADY
        //            PASSES in our model (cobba serial reg file round-trips 0x555/0xAAA/0x000);
        //   phase 2 (0x0E58-0x0E66): the BSP (baseband serial port to the COBBA codec) loopback.
        //            It writes DXR (MMR 0x21) = 0x0AAA and polls DRR (MMR 0x20, `ldm drr0`) for
        //            5001 iterations expecting the SAME word to come back (re-arming DXR=0x0AAA
        //            at 0x0E62 each pass); on the first DRR==DXR it skips the fail store, leaving
        //            data[0x06F9]=0x00 (FULL PASS). Otherwise it writes 0x01 (phase-2 FAIL = bit0).
        // The BSPC0 control (MMR 0x22, 0xC008/0xC0C8 at 0x0E20/0x0E22) puts the codec serial port
        // into self-test/loopback mode. On the physical COBBA-GJ, digital loopback makes the
        // transmit register's word appear in the receive register each serial frame — so DRR
        // mirrors DXR. Our core has no McBSP/BSP model: DRR (data[0x20]) is never driven, so the
        // poll never matches and the test scores 0x01 (MEASURED: CELLWATCH 0x21<-0x0AAA @0x0E30,
        // 0x20 never changes, 0x06F9<-0x01 @0x0E63). Model the codec's faithful loopback here:
        // while COBBA is present, mirror DXR (0x21) -> DRR (0x20). This is store-path independent
        // (the BSP regs are plain MMR/DARAM cells with no write hook reachable from the host model)
        // and gated entirely by DSP54_COBBA, so the 3310/MAD2 path and base-off cosim are untouched.
        // FAITHFUL VALUE = the echoed transmit word (a loopback returns what was written), NOT an
        // arbitrary constant: DRR := DXR. Opt out for A/B with DSP54_COBBA_NOBSPLOOP=1.
        // BOOT-ONLY (perf, 2026-06-14): the loopback exists ONLY to make the cmd-0x70
        // self-test's DRR-echo poll pass — the synth never reads DRR, so once the self-test
        // is done the per-tick DXR/DRR peeks are pure overhead. The self-test is a one-shot
        // early-boot event: it drives the probe DXR=0x0AAA and reads it back. We latch the
        // loopback OFF once it has echoed that probe and the verdict has had time to validate
        // (~1.77M steps). MEASURED: only the ~939k self-test echo (dxr=0x0AAA) affects the boot
        // — disabling the loopback after it leaves the 30M LCD byte-identical (4921792d); the
        // sole sanity check (loopback fully off) regresses to CONTACT SERVICE, confirming the
        // probe echo is load-bearing. (A warm reboot re-runs the self-test, but DSP warm
        // re-staging is a separate open gap; not re-armed here — matches current behaviour.)
        { static int bl_init = 0, bl_on = 0, bl_done = 0; static unsigned long long bl_echo_step = 0;
          if (!bl_init) { bl_init = 1;
              bl_on = (dsp54_faithful("DSP54_COBBA") && !getenv("DSP54_COBBA_NOBSPLOOP")) ? 1 : 0; }
          if (bl_on && !bl_done && g_dsp && g_dsp_run) {
              // Digital loopback and live audio playback are MUTUALLY EXCLUSIVE codec modes — only
              // mirror DXR->DRR when NOT playing audio. The phase-2 self-test runs in loader2 BEFORE
              // any audio command (audio_active is false there, so the gate still closes the verdict),
              // and during a tone the loopback stays OUT of the transmit path so it cannot feed the
              // transmitted audio back into the receive register. (Faithfulness fix 2026-06-12: the
              // always-on echo was behaviourally inert for the synthesized tone — the synth never
              // reads DRR — but mirroring during playback is not what a real codec does.)
              if (audio_raw < 0) audio_raw = (dsp54_data_peek(g_dsp, 0x00FE) & 1)
                                          || (dsp54_data_peek(g_dsp, 0x00AA) != 0);
              if (!audio_raw) {
                  uint16_t dxr = dsp54_data_peek(g_dsp, 0x21);   // BSP transmit reg (BDXR0)
                  uint16_t drr = dsp54_data_peek(g_dsp, 0x20);   // BSP receive  reg (BDRR0)
                  if (drr != dxr) {
                      dsp54_data_poke(g_dsp, 0x20, dxr);          // codec digital loopback echo
                      if (dxr == 0x0AAA && !bl_echo_step)         // the self-test probe was just echoed
                          bl_echo_step = m->dsp_steps ? m->dsp_steps : 1;
                      if (g_log) { static unsigned n; if (++n <= 8) fprintf(stderr,
                          "[dsp54] COBBA: BSP loopback DRR[0x20] <- DXR[0x21]=0x%04X @dsp_steps=%lluk\n",
                          dxr, (unsigned long long)(m->dsp_steps/1000)); }
                  }
              }
              // Self-test echoed + validated -> codec out of loopback; stop the per-tick peeks.
              if (bl_echo_step && m->dsp_steps > bl_echo_step + 2000000ull) bl_done = 1;
          }
        }
        // DSP54_CMDLEVEL per-tick sample: keeps the line-level state honest between event
        // hooks (the DSP's MDISND head-advance during a drain is a plain DARAM store with no
        // write hook — sampling here observes the resulting deassert so the next MCU ring
        // post produces a fresh edge).
        // PERF: gated on the once-per-tick window-dirty read above — a quiescent tick leaves the
        // line inputs unchanged, so the eval would be a pure no-op (state already settled).
        if (win_dirty) cmdlevel_eval(m);
        // DSP54_CMDPOLL=1 (EXPLICIT opt-in, default OFF): the host-command channel INT2 (vec18 ->
        // 0x3598/0x35B9) is NOT MCU-pulsed (the command poster 0x272018 writes only [0x30000]+
        // [0x20008]; 0x3598 is IVT-only). So INT2 must be a SEPARATE periodic source — a tick that
        // makes the DSP poll the LATCHED port1 cmd word (cmd=(~[0x100AA])&[0x100A8]) and act on it:
        // bit1 -> 0x3621 arms the MDISND dequeue ([0x866]bit0). This probe fires vec18 every
        // DSP54_CMDPOLLPER steps to TEST that timer-poll model (does it reach 0x35B9 + arm bit0?).
        { static int cp_init = 0, cp_on = 0; static long cp_per = 50000; static unsigned long long cp_next = 0;
          if (!cp_init) { cp_init = 1; cp_on = getenv("DSP54_CMDPOLL") ? 1 : 0;
              const char *p = getenv("DSP54_CMDPOLLPER"); if (p && *p) cp_per = strtol(p, 0, 0);
              if (cp_per < 1) cp_per = 1; }
          if (cp_on && g_dsp_run) {
              if (cp_next == 0) cp_next = m->dsp_steps + (unsigned long long)cp_per;
              else if (m->dsp_steps >= cp_next) {
                  cp_next = m->dsp_steps + (unsigned long long)cp_per;
                  dsp54_host_interrupt(g_dsp, 18);   // INT2 = host-cmd poll (vec18 -> 0x3598/0x35B9)
                  if (g_log) { static unsigned n; if (++n <= 8) fprintf(stderr,
                      "[dsp54] CMDPOLL: INT2 (vec18 host-cmd poll) @dsp_steps=%lluk (period=%ld)\n",
                      (unsigned long long)(m->dsp_steps/1000), cp_per); }
              }
          }
        }
        // DSP54_INJTONE=1 (probe): drive a TONE command STRAIGHT into the audio mailbox + ring HPINT,
        // bypassing the MCU MMI (which only stuffs these same cells + rings the bell — no observable
        // feedback). Lets us watch the REAL DSP's tone path 0xA51B -> 0xA484 (cmd!=0). The web harness
        // reads the SAME cells to sound it: 0x857/0x858/0x85B = MCU 0x100AE/0x100B0/0x100B6 (osc1/osc2
        // freq in 1/4-Hz units, amp; Hz=reg>>2, amp-gated; src/web/main.c). One-shot at run-mode idle.
        // Params via env (1/4-Hz): defaults = DTMF '1' (osc1 1209Hz, osc2 697Hz). DSP54_INJTONE_CMD
        // A/Bs the [0x85D]&0x1F audio command until 0xA484 runs.
        // CORRECTED 2026-06-11 eve: the tone SYNTHESIZER at 0xA598 gates on [0x856] bit0 (the
        // oscillator ENABLE) — `A=*(856h); A&=1; if(AEQ) goto disabled`. The real keypad path
        // writes [0x856]=0x00E1 (bit0=1 + osc-select bits[7:2] indexing the waveform tables
        // 0xB711/0xB709) alongside [0x857] freq / [0x85B] amp. The old INJTONE set [0x85D] but
        // NOT [0x856], so 0xA598 took the disabled branch and emitted only the flush pattern
        // (3072,0,...). Now staged EXACTLY like the MCU keypress: [0x856] ctrl + [0x857] freq +
        // [0x85B] amp + [0x870] mailbox + doorbell. DSP54_INJTONE_CTL overrides [0x856].
        { static int it_init = 0, it_on = 0, it_done = 0;
          static uint16_t it_f1 = 0x0E10, it_f2 = 0, it_amp = 0x7FFF, it_cmd = 1, it_ctl = 0x00E1;
          if (!it_init) { it_init = 1; it_on = getenv("DSP54_INJTONE") ? 1 : 0; const char *e;
              if ((e = getenv("DSP54_INJTONE_F1")))  it_f1  = (uint16_t)strtol(e, 0, 0);
              if ((e = getenv("DSP54_INJTONE_F2")))  it_f2  = (uint16_t)strtol(e, 0, 0);
              if ((e = getenv("DSP54_INJTONE_AMP"))) it_amp = (uint16_t)strtol(e, 0, 0);
              if ((e = getenv("DSP54_INJTONE_CMD"))) it_cmd = (uint16_t)strtol(e, 0, 0);
              if ((e = getenv("DSP54_INJTONE_CTL"))) it_ctl = (uint16_t)strtol(e, 0, 0); }
          if (it_on && !it_done && g_dsp_run) {
              Dsp54Status st; dsp54_status(g_dsp, &st);
              // REQUIRE the run-mode idle loop PC (0x319D-0x31A5) — NOT st.idle (which is also true
              // at the loader2 wait 0x0F6A, where the injection would be wiped by run-mode init).
              if (st.pc >= 0x319D && st.pc <= 0x31A5 && m->dsp_steps > 1600000) {
                  it_done = 1;
                  dsp54_api_poke(g_dsp, 0x56, it_ctl);   // 0x856 OSC CTRL: bit0=enable (the 0xA598 gate)
                  dsp54_api_poke(g_dsp, 0x57, it_f1);    // 0x857 osc1 freq (1/4-Hz; keypad beep 0x0E10=900Hz)
                  dsp54_api_poke(g_dsp, 0x58, it_f2);    // 0x858 osc2 freq (DTMF low; 0 = single tone)
                  dsp54_api_poke(g_dsp, 0x5B, it_amp);   // 0x85B amplitude
                  dsp54_api_poke(g_dsp, 0x5D, it_cmd);   // 0x85D audio cmd (&0x1F -> 0xA51B dispatch)
                  dsp54_api_poke(g_dsp, 0x70, 1);        // 0x870 mailbox trigger (-> 0x3772 calls 0xA51B)
                  dsp54_host_interrupt(g_dsp, 25);       // HPINT wake
                  if (g_log) fprintf(stderr, "[dsp54] INJTONE: staged ctl=0x%04X f1=%u f2=%u amp=0x%04X "
                      "cmd=0x%02X + HPINT @dsp_steps=%lluk (pc=0x%04X)\n", it_ctl, it_f1, it_f2, it_amp,
                      it_cmd, (unsigned long long)(m->dsp_steps/1000), st.pc);
              }
          }
        }
        // DSP54_TICKVEC=<vec> [+DSP54_TICKPER=<steps>, default 50000] — DIAGNOSTIC PROBE (default OFF).
        //
        // ⚠ RETIRED FROM THE FAITHFUL PATH. This was a periodic-interrupt STAND-IN for
        // the DSP self-wake that drains the rings — back when the demand-page sleep overlay (id 8)
        // never landed, so the real 0x30A7 sleep-loop dispatcher crashed and something had to pump
        // the superloop. Once id 8 lands at 0x1CA4 (READA/WRITA PAR-seed + INTR k fixes, commit
        // 8ab4a07) the sleep loop self-pumps and TICKVEC is REDUNDANT — MEASURED byte-identical:
        // base (no knobs) and DSP54_TICKVEC=25 produce the SAME MDIRCV organic drain (tail
        // 0x80->0x89->0xA4->0xA6->0xA9 at the same steps), the SAME 25 CMDLEVEL edges, and the SAME
        // verdict-byte [0x10FDDD] timeline to the SAME end state — even though vec25 demonstrably
        // FIRES every 400k steps and changes nothing. The old "TICKVEC=25 -> Security code"
        // (4921792d) PASS only ever worked on MAIN, whose BUGGY decode kept the DSP out of the sleep
        // path so the spurious vec25 happened to squeak the cmd-0x70 self-test past the validator;
        // it was never a faithful wake. The faithful dispatcher-wake question is hereby settled: the
        // sleep loop is the real pump (see docs/research/dsp- and the
        // 5110-decode-bugs HANDOFF §1). KEPT as an opt-in diagnostic only (A/B against main's crutch,
        // or to probe a periodic source); DO NOT re-adopt it as a default/canonical recipe.
        //
        // Original probe note: vec19 (MAD2 system-logic tick, IMR bit3) and vec25 (HPINT, the KNOWN
        // dispatcher-wake but normally MCU-pulsed) were the periodic-source candidates.
        { static int tv_init = 0, tv_vec = -1; static long tv_per = 50000; static unsigned long long tv_next = 0;
          if (!tv_init) { tv_init = 1; const char *v = getenv("DSP54_TICKVEC");
              if (v && *v) tv_vec = (int)strtol(v, 0, 0);
              const char *p = getenv("DSP54_TICKPER"); if (p && *p) tv_per = strtol(p, 0, 0);
              if (tv_per < 1) tv_per = 1; }
          if (tv_vec >= 0 && g_dsp_run) {
              if (tv_next == 0) tv_next = m->dsp_steps + (unsigned long long)tv_per;
              else if (m->dsp_steps >= tv_next) { tv_next = m->dsp_steps + (unsigned long long)tv_per;
                  dsp54_host_interrupt(g_dsp, tv_vec);
                  if (g_log) { static unsigned n; if (++n <= 6) fprintf(stderr,
                      "[dsp54] TICKVEC: fired vec %d @dsp_steps=%lluk (per=%ld)\n",
                      tv_vec, (unsigned long long)(m->dsp_steps/1000), tv_per); } }
          }
        }
        // DSP54_SELFTEST_MEAS=1 — model the NOMINAL idle measurement the DSP self-test packages.
        // BACKGROUND (, exhaustively traced 2026-06-11): the MCU
        // legitimately requests the cmd-0x70 / subcmd-0x16 diagnostic MEASUREMENT report at boot. The
        // DSP builder 0x4B73 copies 11 words from AR2 (= 0x800+MDISND_tail, the ring) into the report
        // and ships it; the MCU validator 0x258B60 then range/BCD-checks the measurement fields. At
        // no-call standby there is NO live measurement source — proven by elimination: no DSP ISR
        // writes the source (CELLWATCH), the repoint hook 0x250b is a resident no-op, and the 0x2286
        // codec overlay is never entered by the (fully resident) self-test path. ⚠ CORRECTED by the
        // first live end-to-end run: the builder tail (call 0x3900 + 2x 0x7f2d) DOES
        // rewrite staging words 2+ in place (staged 0010/0/0/0/0160 left the DSP as BC4E/803B/E300/
        // A5F2/4C1B; word0 0x3532 and word1 survive) — the old "writes 0x13DC, not the report"
        // reading was wrong. On real HW that tail inserts/encodes the live measurement; in cosim it
        // runs on ring garbage -> the report fails validation -> reason-4 retry reboot (dequeue
        // armed) or the 8.63M timeout -> CONTACT SERVICE. This knob models what the un-stimulated
        // analog front-end would yield at idle: the nominal no-signal reading, staged as FINAL WIRE
        // BYTES after the transform. The validator's low-band accept (0x258BD8 -> 0x258BF6 ->
        // 0x258CA2, no checksum for the short form buf[9]=0x32) needs only:
        //   [0x1202]=0x0010 (buf[12]=0,buf[13]=0x10), [0x1206]=0x0160 (buf[20]&0xF=1,buf[21]=0x60),
        //   [0x1203..1205]=0 (BCD-clean tail).  The other reports/cmd-13 pass then clear the verdict
        //   (verdict C4->C0 @1.77M observed; no [0x10FDDE] 0x5A->0xA5 marker, no reason-4).
        // ⚠ The values are the validator-correct nominal STAND-IN, NOT silicon-confirmed (we have no
        // working self-test reference). Requires the self-test to actually execute (the MDISND dequeue
        // armed — organic under DSP54_CMDLEVEL).
        // Fires once per builder entry (re-arms on exit, so a self-test retry is also covered).
        // Faithful default ON under cosim (2026-06-11, end-to-end validated with CMDLEVEL): models
        // the analog front-end's NOMINAL idle reading. Without it the organically-drained report
        // carries garbage -> the validator faithfully REJECTS -> the firmware's real one-shot retry
        // reboot ([0x10FDDE] 0x5A/0xA5 marker protocol, reason-4 @0x258D2E) -> exposes the (open)
        // warm-reboot DSP re-staging gap. With it, base = CONTACT SERVICE (8.63M timeout).
        // ⚠ 2026-06-12: TICKVEC is RETIRED (see the TICKVEC block above) — it no longer changes the
        // outcome now that the sleep loop self-pumps. SELFTEST_MEAS alone is also insufficient on the
        // branch: it patches only the FIRST report (0x4BA8 in-tail enqueue ~1.5M); the organic DSP
        // posts a SECOND self-test report (~3.2M) that this hook does NOT cover, so the validator
        // clears verdict bit6 (MCU 0x22F082) -> CONTACT SERVICE. The faithful through-boot needs the
        // COBBA measurement-INPUT model (regs 5/6 @0x4AEA/0x4AF5), not an output patch. See HANDOFF
        // §4a-NEW.
        { static int sm_init = 0, sm_on = 0, sm_armed = 1;
          if (!sm_init) { sm_init = 1; sm_on = dsp54_faithful("DSP54_SELFTEST_MEAS"); }
          if (sm_on && g_dsp_run) {
              Dsp54Status st; dsp54_status(g_dsp, &st);
              // Stage POST-TRANSFORM (corrected 2026-06-11, first end-to-end run): the builder's
              // tail (call 0x3900 + 2x 0x7f2d + the [0x1207]/[0x120D] checksum-zeroing) REWRITES
              // staging words 2+ — on real HW that pipeline inserts the live measurement; in cosim
              // it runs on garbage and clobbered the old pre-copy AR2 pokes (wire bytes 12-21 came
              // out BC4E/803B/... and the validator rejected -> marker [0x10FDDE]=0x5A -> reason-4).
              // So stage the validator-correct NOMINAL wire fields into the finished report at
              // 0x1202..0x1206, in the window after both transforms and before the MDIRCV enqueue
              // (dcall 0x37CE @0x4BA8): wire buf[12,13]=00,10 / buf[14..19] BCD-clean zeros /
              // buf[20,21]=01,60 (low-band accept 0x258BD8->0x258BF6->0x258CA2).
              int in_tail = (st.pc >= 0x4B8C && st.pc <= 0x4BA7);  // post-7f2d .. pre-enqueue
              if (in_tail && sm_armed) {
                  sm_armed = 0;
                  dsp54_data_poke(g_dsp, 0x1202, 0x0010);  // buf[12,13] low-band markers
                  dsp54_data_poke(g_dsp, 0x1203, 0x0000);  // buf[14,15] BCD-clean
                  dsp54_data_poke(g_dsp, 0x1204, 0x0000);  // buf[16,17]
                  dsp54_data_poke(g_dsp, 0x1205, 0x0000);  // buf[18,19]
                  dsp54_data_poke(g_dsp, 0x1206, 0x0160);  // buf[20]&0xF=1, buf[21]=0x60 (96)
                  if (g_log) fprintf(stderr, "[dsp54] SELFTEST_MEAS: nominal idle measurement staged "
                      "post-transform at [0x1202..0x1206] @dsp_steps=%lluk (pc=0x%04X)\n",
                      (unsigned long long)(m->dsp_steps / 1000), st.pc);
              } else if (!in_tail) { sm_armed = 1; }
          }
        }
        // DSP54_GOTEST=1 (diagnostic): the run-mode bootstrap (loader2 @0xF00) clears [0x872] in its
        // own init (0xF2A) AFTER the MCU already delivered the run-mode go-command (0x0002) at warm
        // boot -> the command is clobbered and the DSP spins at 0xF3D forever. This probe re-delivers
        // [0x872]=0x0002 ONCE while the DSP is parked at 0xF3D (PC in the wait loop, [0x872]==0) to
        // PROVE the ordering/clobber is the whole gap (expect: DSP exits -> b 0x0D80 -> run-mode 0x3000).
        { static int go_init = 0, go_on = 0, go_done = 0; static uint16_t go_cmd = 0x0002;
          if (!go_init) { go_init = 1; const char *p = getenv("DSP54_GOTEST");
              if (p && *p) { go_on = 1; long v = strtol(p,0,0); if (v > 1) go_cmd = (uint16_t)v; } }
          if (go_on && !go_done && g_dsp_run) {
              Dsp54Status gs; dsp54_status(g_dsp, &gs);
              if (gs.pc >= 0x0F3D && gs.pc <= 0x0F42 && dsp54_data_peek(g_dsp, 0x872) == 0) {
                  dsp54_data_poke(g_dsp, 0x872, go_cmd); go_done = 1;
                  fprintf(stderr, "[dsp54] GOTEST: re-delivered [0x872]=0x%04X at PC=0x%04X @dsp_steps=%lluk\n",
                          go_cmd, gs.pc, (unsigned long long)(m->dsp_steps/1000)); } }
        }
        // DSP54_MMR58PER=<ticks> (run-mode frame-timer model): the run-mode wait loop busy-polls
        // MMR 0x58 bit0 in a TWO-PHASE handshake — 0x7EBE spins while bit0 SET (wait peripheral
        // done), idle(3), then 0x7ECC spins while bit0 CLEAR (wait next tick). So bit0 is a periodic
        // frame-timer flag that must TOGGLE each frame. The C54x lift never models it, so the DSP
        // spins forever (RATIO=4). Toggle bit0 every <ticks> MCU steps to model the frame timebase.
        // (DSP54_MMR58CLR=1 = legacy "always clear", only passes phase 1.) Default OFF.
        { static int m58_init = 0, m58_clr = 0; static uint64_t m58_per = 0, m58_next = 0; static uint16_t m58_bit = 0;
          if (!m58_init) { m58_init = 1; m58_clr = getenv("DSP54_MMR58CLR") ? 1 : 0;
              const char *p = getenv("DSP54_MMR58PER"); if (p && *p) { long v = strtol(p,0,0); if (v>0) m58_per=(uint64_t)v; } }
          if (m58_clr) { uint16_t v = dsp54_data_peek(g_dsp, 0x58); if (v & 1) dsp54_data_poke(g_dsp, 0x58, v & ~1u); }
          else if (m58_per && m->dsp_steps >= m58_next) { m58_next = m->dsp_steps + m58_per;
              m58_bit ^= 1; uint16_t v = dsp54_data_peek(g_dsp, 0x58);
              dsp54_data_poke(g_dsp, 0x58, (uint16_t)((v & ~1u) | m58_bit)); } }
        // DSP54_FRAMECADENCE=<steps> (R1 decisive test): model the MCU's per-frame codec/call
        // command. After the DSP first reaches idle, every <steps> MCU steps deliver ONE 0x1c-group
        // host command (default 0x10 = bit4) via the real INT1 path and let the DSP return to idle
        // between pulses (NOT a tight burst like INJMDISND). FINDING (R1, canonical doc): this
        // flips the DSP out of idle into the codec/frame regime (codec 0x45C2/0x460E/port-2D-wait
        // 0x45F8 climb, int0_3204 fires, bist collapses) but MDIRCV (tail 0x101C8) STILL does not
        // post — MDIRCV carries L2/control msgs, so the keep-alive also needs a network downlink
        // source, not just codec frames. Watch RAMWATCH=0x101C8 + DSP54_PCHIST. DSP54_FRAMECMD
        // overrides the command word. Default OFF.
        {
            static int fc_init = 0; static uint64_t fc_every = 0, fc_next = 0;
            static uint16_t fc_cmd = 0x0010; static int fc_armed = 0; static uint64_t fc_n = 0;
            if (!fc_init) { fc_init = 1; const char *p = getenv("DSP54_FRAMECADENCE");
                if (p && *p) { long v = strtol(p,0,0); if (v > 0) fc_every = (uint64_t)v; }
                const char *c = getenv("DSP54_FRAMECMD"); if (c && *c) fc_cmd = (uint16_t)strtoul(c,0,0); }
            if (fc_every) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                if (!fc_armed && (st.pc == 0x408C || st.pc == 0x408B || st.pc == 0x407C)) {
                    fc_armed = 1; fc_next = m->dsp_steps + fc_every;
                    if (g_log) fprintf(stderr, "[dsp54] FRAMECADENCE: DSP idle @%lluk — cmd 0x%04X every %llu steps\n",
                                       (unsigned long long)(m->dsp_steps/1000), fc_cmd, (unsigned long long)fc_every);
                }
                if (fc_armed && m->dsp_steps >= fc_next) {
                    fc_next = m->dsp_steps + fc_every; fc_n++;
                    mcu_set_word(m, g_port1_addr, fc_cmd);
                    mcu_set_word(m, g_port2_addr, 0x0000);
                    dsp54_host_interrupt(g_dsp, g_intvec);
                    if (g_log && fc_n <= 8) fprintf(stderr, "[dsp54] FRAMECADENCE: frame %llu cmd 0x%04X @%lluk\n",
                                       (unsigned long long)fc_n, fc_cmd, (unsigned long long)(m->dsp_steps/1000));
                }
            }
        }
        // DSP54_SPMIN: track the lowest SP seen + the PC at that moment, dump at DSP54_SPMIN
        // steps. Pins how deep the stack descends (does it reach the upload region 0x126-0x800?)
        // and which PC drives it there — the stack-runaway that clobbers the uploaded blocks.
        {
            static int sm_init = 0, sm_on = 0; static uint64_t sm_at = 30000000u;
            static uint16_t sm_min = 0xFFFF, sm_pc = 0; static uint64_t sm_step = 0;
            if (!sm_init) { sm_init = 1; const char *p = getenv("DSP54_SPMIN");
                if (p && *p) { sm_on = 1; long v = strtol(p,0,0); if (v>1) sm_at=(uint64_t)v; } }
            if (sm_on) {
                Dsp54Status st; dsp54_status(g_dsp, &st);
                if (st.sp < sm_min) { sm_min = st.sp; sm_pc = st.pc; sm_step = m->dsp_steps; }
                if (m->dsp_steps == sm_at)
                    fprintf(stderr, "[dsp54] SP-MIN watermark: sp=0x%04X at pc=0x%04X step=%lluk (upload region=0x126-0x800)\n",
                            sm_min, sm_pc, (unsigned long long)(sm_step/1000));
            }
        }
        // DSP54_PCHIST: sample the co-sim DSP PC every MCU step into a fine histogram, dump the
        // top PCs at DSP54_PCHIST steps (default 15M). Pins the actual hot loop the DSP cycles
        // (its RTOS dispatch + the data/HPI word it polls) — the missing codec/host input.
        {
            static int      ph_init = 0, ph_on = 0;
            static uint64_t ph_at = 15000000u;
            static uint32_t *ph = 0;
            if (!ph_init) {
                ph_init = 1;
                const char *p = getenv("DSP54_PCHIST");
                if (p && *p) { ph_on = 1; long v = strtol(p, 0, 0); if (v > 1) ph_at = (uint64_t)v; }
                if (ph_on) { ph = calloc(0x10000, sizeof *ph);
                    // Fill the histogram inside the core's run loop (per executed DSP instruction)
                    // so reach/PC counts are RELIABLE at any DSP:MCU RATIO (the old per-MCU-tick
                    // ph[st.pc]++ sampled 1-of-RATIO and aliased badly at RATIO>1).
                    dsp54_set_pc_histogram(g_dsp, ph); }
            }
            if (ph_on && ph) {
                if (m->dsp_steps == ph_at) {
                    // DSP54_DATADUMP=<path>: snapshot live DSP data[] (DARAM, BE words) for
                    // offline tic54x-objdump of the scheduler/handler code that runs from OVLY
                    // DARAM ([0x80,0x2800)) — not visible in the PROM disasm (0x3000+).
                    const char *dd = getenv("DSP54_DATADUMP");
                    if (dd && *dd) {
                        char pp[512]; snprintf(pp, sizeof pp, "%s.prog", dd);
                        dsp54_dump_mem(g_dsp, pp, dd, 0x2800);
                        fprintf(stderr, "[dsp54] dumped DSP data[0..0x2800) -> %s, prog[] -> %s\n", dd, pp);
                        // ALSO dump the OVLY window api_ram (the store the DSP's own data writes hit,
                        // and where loader1 SCATTERS the uploaded overlays). prog_fetch reads data[]
                        // for [0x80,0x2800) but DSP stores go to api_ram — so for [0x800,0x2800) these
                        // are TWO copies. data_peek(0x800+i) returns api_ram[i]. (diagnostic, BE words)
                        char ap[512]; snprintf(ap, sizeof ap, "%s.api", dd);
                        FILE *af = fopen(ap, "wb");
                        if (af) { for (uint32_t i = 0; i < 0x2000u; i++) {
                                uint16_t w = dsp54_data_peek(g_dsp, (uint16_t)(0x800u + i));
                                uint8_t b[2] = { (uint8_t)(w >> 8), (uint8_t)w }; fwrite(b, 1, 2, af); }
                            fclose(af);
                            fprintf(stderr, "[dsp54] dumped DSP api_ram[0x800..0x2800) -> %s\n", ap); }
                    }
                    // Idle-reachability probe: does the DSP ever reach the 0x408B idle / its
                    // scheduler idle-entry 0x407C? (never-idles vs idles-but-rewoken). Plus the
                    // host-cmd ISR 0x3598/0x35B9 — did a host command ever get serviced?
                    fprintf(stderr, "[dsp54] reach: idle408C=%u idleEntry407C=%u hostISR3598=%u hostCmd35B9=%u\n",
                            ph[0x408C], ph[0x407C], ph[0x3598], ph[0x35B9]);
                    // INTM=0 windows + frame/codec sites: 0x45C2 codec-read entry, 0x460E codec
                    // INTM=0 return, 0x3204 INT0(frame ISR), 0x3416 TINT0, 0x45F8 codec port-2D wait.
                    fprintf(stderr, "[dsp54] reach2: codec45C2=%u codecRet460E=%u int0_3204=%u tint0_3416=%u p2dwait45F8=%u\n",
                            ph[0x45C2], ph[0x460E], ph[0x3204], ph[0x3416], ph[0x45F8]);
                    // reach3: superloop dispatch (0x24D3/0x2470/0x2503) vs audio-frame entry/loop
                    // (0xE035 frameproc call, 0xDE37, 0xE08A outer codec loop, 0xDE10 gate). Tells
                    // whether the scheduler is sweeping tasks or we're stuck inside one audio task.
                    fprintf(stderr, "[dsp54] reach3: disp24D3=%u disp2470=%u disp2503=%u | E035=%u DE37=%u E08A=%u DE10=%u DD83=%u\n",
                            ph[0x24D3], ph[0x2470], ph[0x2503], ph[0xE035], ph[0xDE37], ph[0xE08A], ph[0xDE10], ph[0xDD83]);
                    // reach4: functional-region time split. superloop=PROM 0x0900-0x0CFF (the task
                    // sweep that calls idle 0x407C via the [0x6E4] bitmap), idle=0x4070-0x4097,
                    // bist=0xD800-0xF200, daram=0x0080-0x2800 (OVLY), prom_lo=0x0000-0x0080 (ret/fret pad).
                    { uint64_t sl=0,id=0,bi=0,da=0,pad=0,other=0;
                      for (uint32_t i=0;i<0x10000;i++){ uint32_t c=ph[i]; if(!c) continue;
                        if(i>=0x0900&&i<=0x0CFF) sl+=c; else if(i>=0x4070&&i<=0x4097) id+=c;
                        else if(i>=0xD800&&i<=0xF200) bi+=c; else if(i>=0x0080&&i<0x2800) da+=c;
                        else if(i<0x0080) pad+=c; else other+=c; }
                      fprintf(stderr, "[dsp54] reach4: superloop=%llu idle=%llu bist=%llu daram=%llu retpad=%llu other=%llu\n",
                            (unsigned long long)sl,(unsigned long long)id,(unsigned long long)bi,
                            (unsigned long long)da,(unsigned long long)pad,(unsigned long long)other);
                      fprintf(stderr, "[dsp54] superloop top 0x0926=%u 0x0935=%u | taskbmp[0x6E4]=0x%04X [0x195C]=0x%04X [0x195B]=0x%04X\n",
                            ph[0x0926], ph[0x0935], dsp54_data_peek(g_dsp,0x6E4),
                            dsp54_data_peek(g_dsp,0x195C), dsp54_data_peek(g_dsp,0x195B));
                      // cascade cells: [0x866] bit0 = MDISND-enable gate (set by port cmd bit1 @0x3621);
                      // [0x6E1] bit3 = MDISND dequeue enable (gate @0x387C). PC hits on the gate/dequeue.
                      fprintf(stderr, "[dsp54] cascade: [0x866]=0x%04X [0x6E1]=0x%04X | dequeueGate394F=%u dequeue3860=%u enq37CE=%u 3804=%u idleGate0A0C=%u\n",
                            dsp54_data_peek(g_dsp,0x866), dsp54_data_peek(g_dsp,0x6E1),
                            ph[0x394F], ph[0x3860], ph[0x37CE], ph[0x3804], ph[0x0A0C]);
                      // enqueue-route (R1): MDIRCV ring head/tail (DSP 0x8E5/0x8E4 == MCU 0x101CA/0x101C8)
                      // + the tail-update gate [0x6E1]bit2 (set => skip tail, route to secondary buf 0x12C8);
                      // tailUpd3803 = PC hits on the tail-write, enqRetEmpty37E1 = early "ring full" returns.
                      fprintf(stderr, "[dsp54] enqroute: MDIRCVhead[0x8E5]=0x%04X tail[0x8E4]=0x%04X [0x6E1]&4=%u sec12C8=0x%04X..%04X tailUpd3803=%u enqFull37E1=%u\n",
                            dsp54_data_peek(g_dsp,0x8E5), dsp54_data_peek(g_dsp,0x8E4),
                            dsp54_data_peek(g_dsp,0x6E1)&4, dsp54_data_peek(g_dsp,0x12C8),
                            dsp54_data_peek(g_dsp,0x12C9), ph[0x3803], ph[0x37E1]);
                      // cmd-handler reach: did the ISR route to the gate setters?
                      // 0x3621=bit1->[0x866]|=1 ; 0x362F/0x363C=0x1C-grp bit4->[0x6E4]|=0x40 ;
                      // 0x25C0=alt bit4 setter ; 0x35B9=ISR entry ; 0x0A14=int(19h) frame-launch.
                      fprintf(stderr, "[dsp54] handlers: isr35B9=%u h3621=%u h362F=%u h363C=%u h25C0=%u idleSpin0A0B=%u frameLaunch0A14=%u\n",
                            ph[0x35B9], ph[0x3621], ph[0x362F], ph[0x363C], ph[0x25C0], ph[0x0A0B], ph[0x0A14]);
                      // reach5: superloop/codec transition. 0x3000=superloop head, 0x30BF=frame
                      // block (calls 241e->926 + a15f), 0x31AE=task dispatcher, 0x9956=standby
                      // idle-loop, 0xF1A5=vocoder caller, 0xE310=vocoder wrapper, 0xA15F=ovl frame.
                      fprintf(stderr, "[dsp54] reach5: superloop3000=%u frameblk30BF=%u disp31AE=%u standby9956=%u | vocF1A5=%u vocE310=%u ovlA15F=%u\n",
                            ph[0x3000], ph[0x30BF], ph[0x31AE], ph[0x9956], ph[0xF1A5], ph[0xE310], ph[0xA15F]); }
                    // simple top-30 selection
                    fprintf(stderr, "[dsp54] PC histogram (top 30) @%lluM steps:\n",
                            (unsigned long long)(m->dsp_steps / 1000000u));
                    for (int k = 0; k < 30; k++) {
                        uint32_t best = 0, bi = 0;
                        for (uint32_t i = 0; i < 0x10000; i++) if (ph[i] > best) { best = ph[i]; bi = i; }
                        if (!best) break;
                        fprintf(stderr, "    0x%04X : %u\n", bi, best);
                        ph[bi] = 0;
                    }
                }
            }
        }
        // Level-trigger experiment: a single DSPINT edge that lands while the DSP has INTM=1
        // (mid DSP-math block-repeat) goes pending then is lost. Re-pulse periodically to test
        // whether the host-cmd ISR runs once it gets an INTM=0 window. Only after the firmware
        // has issued at least one real doorbell (g_dspint_edges>0), so we don't poke pre-boot.
        if (g_intrepeat && g_dspint_edges && (m->dsp_steps % (uint64_t)g_intrepeat) == 0) {
            if (g_log) {
                static int nlog = 0;
                if (nlog < 12) { nlog++;
                    Dsp54Status st; dsp54_status(g_dsp, &st);
                    fprintf(stderr, "[dsp54] INTREPEAT inject vec=%d @%lluk pre{pc=0x%04X idle=%d INTM=%d imr.b2=%d ifr.b2=%d pmst=0x%04X iptr=0x%X}\n",
                            g_intvec, (unsigned long long)(m->dsp_steps/1000), st.pc, st.idle,
                            (st.st1>>11)&1, (st.imr>>2)&1, (st.ifr>>2)&1, st.pmst, (st.pmst>>7)&0x1FF);
                }
            }
            dsp54_host_interrupt(g_dsp, g_intvec);
            if (g_log) {
                static int nlog2 = 0;
                if (nlog2 < 12) { nlog2++;
                    Dsp54Status st; dsp54_status(g_dsp, &st);
                    fprintf(stderr, "[dsp54]   post-inject{pc=0x%04X idle=%d INTM=%d ifr.b2=%d}\n",
                            st.pc, st.idle, (st.st1>>11)&1, (st.ifr>>2)&1);
                }
            }
        }
        // Codec frame cadence (approach A): the real DSP is paced by the McBSP serial codec
        // (BRINT0/BXINT0 per frame), which bounds its FIR processing and returns it to the
        // 0x408B idle (INTM=0). The C54x timer is dead in the lift + left stopped, so pace it
        // via the serial frame interrupt instead. Experimental — find the vector/period that
        // settles the DSP at idle so it can service the host INT2.
        if (g_frameint && m->dsp_steps > g_frameafter &&
            (m->dsp_steps % (uint64_t)g_frameper) == 0) {
            // DSP54_FRAMEDBG=1: dump DSP state around the first 8 injections (wake-vs-service
            // diagnosis — a woken-but-INTM-masked idle resumes inline and the ISR never runs).
            static int fdbg = -1; if (fdbg < 0) fdbg = getenv("DSP54_FRAMEDBG") ? 1 : 0;
            if (fdbg) { static int fdn = 0; if (fdn < 8) { fdn++;
                Dsp54Status st; dsp54_status(g_dsp, &st);
                fprintf(stderr, "[framedbg] inject vec=%d @%lluk pre{pc=0x%04X idle=%d INTM=%d imr=0x%04X ifr=0x%04X}\n",
                        g_frameint, (unsigned long long)(m->dsp_steps/1000), st.pc, st.idle,
                        (st.st1>>11)&1, st.imr, st.ifr); } }
            dsp54_host_interrupt(g_dsp, g_frameint);
            if (fdbg) { static int fdn2 = 0; if (fdn2 < 8) { fdn2++;
                Dsp54Status st; dsp54_status(g_dsp, &st);
                fprintf(stderr, "[framedbg]   post{pc=0x%04X idle=%d INTM=%d ifr=0x%04X}\n",
                        st.pc, st.idle, (st.st1>>11)&1, st.ifr); } }
        }
        // DSP54_FORCEPC=<hex> at DSP54_FORCEPCAT=<dsp_steps> (DIAGNOSTIC, default-OFF):
        // one-shot jump the DSP PC (e.g. into the superloop head 0x3000) to A/B whether the
        // frame chain runs once reached, bypassing the boot fork that diverts into the codec.
        // NOT a faithful path — bring-up probe only (§8 codec-env fork investigation).
        { static int fp_init = 0, fp_done = 0; static uint16_t fp_pc = 0; static uint64_t fp_at = 0;
          if (!fp_init) { fp_init = 1;
              const char *p = getenv("DSP54_FORCEPC"); if (p && *p) fp_pc = (uint16_t)strtol(p, 0, 0);
              const char *a = getenv("DSP54_FORCEPCAT"); if (a && *a) fp_at = strtoull(a, 0, 0); }
          if (fp_pc && !fp_done && m->dsp_steps >= fp_at) {
              dsp54_set_pc(g_dsp, fp_pc); fp_done = 1;
              if (g_log) fprintf(stderr, "[dsp54] FORCEPC -> 0x%04X @dsp_steps=%llu\n",
                                 fp_pc, (unsigned long long)m->dsp_steps); } }
        // DSP54_PCHIST=1: every ~5M MCU steps, snapshot where the DSP is parked + whether
        // INT2 is unmasked / it's IDLE. Pins the boot blocker during co-sim bring-up.
        if (g_log) {
            static uint64_t next = 5000000u;
            if (m->dsp_steps >= next) {
                next += 5000000u;
                Dsp54Status st; dsp54_status(g_dsp, &st);
                fprintf(stderr, "[dsp54] @%lluM pc=0x%04X sp=0x%04X pmst=0x%04X imr=0x%04X ifr=0x%04X "
                        "INTM=%d idle=%d INT2(imr.b2)=%d (ifr.b2)=%d\n",
                        (unsigned long long)(m->dsp_steps / 1000000u), st.pc, st.sp, st.pmst, st.imr, st.ifr,
                        (st.st1 >> 11) & 1, st.idle, (st.imr >> 2) & 1, (st.ifr >> 2) & 1);
                // HPI boot-reply surface the firmware loader reads (§7e). The DSP should write
                // these during its boot handshake; count what's actually populated.
                int nz = 0; for (int k = 0; k < 0x100; k++) if (dsp54_hpi_read(g_dsp, 0x10000 + 2*k)) nz++;
                fprintf(stderr, "[dsp54]   HPI: boot_status[0x10004]=0x%04X cobba[0x100E0]=0x%04X "
                        "req[0x100E2]=0x%04X reply[0x100E4]=0x%04X mbox0[0x100FE]=0x%04X "
                        "mbox1[0x10100]=0x%04X  (nonzero words in 0x000-0x0FF: %d)\n",
                        dsp54_hpi_read(g_dsp, 0x10004), dsp54_hpi_read(g_dsp, 0x100E0),
                        dsp54_hpi_read(g_dsp, 0x100E2), dsp54_hpi_read(g_dsp, 0x100E4),
                        dsp54_hpi_read(g_dsp, 0x100FE), dsp54_hpi_read(g_dsp, 0x10100), nz);
            }
        }
    }
    dsp_default_tick(m);
}

const DspOps mad2_dsp_c54x = {
    "c54x", c54x_read, c54x_write, c54x_tick,
};
