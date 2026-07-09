// dsp_bridge.h — transport seam for a REMOTE DSP backend.
//
// `mad2_dsp_bridge` is a fourth DspOps backend (beside mad2_dsp_default / _rom4 /
// _c54x). Instead of computing DSP behaviour locally, it forwards the entire MCU<->DSP
// coupling over a byte stream to a remote DSP, which can be:
//   (a) another emulator process running the c54x co-sim  (local validation), or
//   (b) a real phone running minimal proxy FW            (hardware-in-the-loop).
// Same wire protocol either way — selected at runtime with DSP_BRIDGE (see below), so
// it drops in "decisively instead of the HLE DSP".
//
// FOUR CHANNELS cross the wire (this is the whole MCU<->DSP boundary):
//   1. HPI window     — MCU 0x10000..0x10FFF  <-> DSP dual-port DARAM (control/mailbox)
//   2. DSP reset/run  — 0x20002 bit0 (IO_CTSI_DSP reset-release line)
//   3. Doorbells      — 0x30000 (DSPINT=INT2 / APIMODE = HPI-window enable)
//   4. CCONT          — the power/rail control that actually turns the DSP ON
//                       (diffed from m->ccont[16] so it works for serial AND mmapped CCONT)
// and DSP->MCU coming back: HPI window deltas + the MCU-side interrupts
// (FIQ0=MDIRCV, FIQ1=MDISND, IRQ4=code-block ack).
//
// The boundary is interrupt-synchronised message passing (not per-cycle): the MCU only
// trusts DSP-written cells AFTER an interrupt, so we batch — flush MCU writes + STEP +
// pull deltas/IRQs at sync points (doorbell strobe, or every DSPB_SYNC_PERIOD ticks),
// and the MCU's clock gates on the round-trip (no keep-alive starvation).
//
// DSP_BRIDGE spec (env):  connect:HOST:PORT | listen:PORT | unix:PATH | fd:N
//   plus DSPB_SYNC_PERIOD (MCU ticks between periodic syncs, default 2048).

#ifndef DSP_BRIDGE_H
#define DSP_BRIDGE_H
#include <stdint.h>

// wire message types (1-byte tag, big-endian payloads)
enum {
    DSPB_WRITE  = 0x01,  // M->D  HPI window write   addr:u32 size:u8 val:u32
    DSPB_CTRL   = 0x04,  // M->D  control line       id:u8 val:u32
    DSPB_STEP   = 0x05,  // M->D  advance the DSP     ticks:u32   (server scales by its ratio)
    DSPB_POLL   = 0x06,  // M->D  advance EXACTLY n   n:u32 (no ratio) — fine coherent-read sampling
    DSPB_WINDOW = 0x07,  // D->M  window deltas       count:u16 then count*(addr:u32,val:u16)
    DSPB_IRQ    = 0x08,  // D->M  raise MCU interrupt id:u8
    DSPB_DONE   = 0x09,  // D->M  end of STEP response
    DSPB_PING   = 0x0A,  // M->D  comms check         seq:u8         (no DSP touch)
    DSPB_PONG   = 0x0B,  // D->M  comms check reply    seq:u8 0xA5 0x5A   (echoes seq + marker)
    // --- edge/transactional protocol (DSPB_EDGE=1) — see docs/dct3-dsp-bridge-fbus-scope.md ---
    DSPB_TXN    = 0x0C,  // M->D  one transaction: doorbell + its dirty-window delta, applied THEN signalled
                         //       ctrl_id:u8  count:u16  count*(addr:u32,val:u16)   (ctrl_id = DSPB_CTRL_* to raise after apply)
    DSPB_EVENT  = 0x0D,  // D->M  async push at a DSP edge: irq id + the (barrier-coherent) cell delta
                         //       irq_id:u8  count:u16  count*(addr:u32,val:u16)   (host applies delta THEN raises irq)
    DSPB_HELLO  = 0x0E,  // M->D  enter edge mode (proxy: arm edge push, stop periodic scan). no payload
    DSPB_RESET  = 0x0F,  // M->D  COLD SYSTEM RESTART: proxy asserts IO_CTSI_RST bit2 (== ccont_reboot) —
                         //       full ASIC reset so the real DSP comes up virgin. Phone reboots: NO reply.
    DSPB_IRQLOG = 0x10,  // D->M  diagnostic: raw interrupt latches   fiql:u8 irql:u8 (what the DSP actually raised)
    DSPB_DUMPREQ= 0x11,  // M->D  request the FULL raw HPI window (no shadow-diff, no bus-hold skip) — proxy
                         //       replies with DSPB_WINDOW frames for ALL 0x800 cells then DONE. Terminal-state
                         //       ground truth: the real DSP's view, to compare vs the MCU's filtered mirror.
    DSPB_STATS  = 0x12,  // M->D  request the proxy's SENT-edge counters; reply DSPB_STATS frame:
                         //       mdircv:u32 mdisnd:u32 block:u32 wincells:u32. Diffed vs the host's
                         //       RECEIVED/APPLIED counts to catch wire divergence (dropped edges/cells).
    DSPB_WRSYNC = 0x13,  // (retired v1 — superseded by WRBLOCK) reliable-write checkpoint experiment.
    DSPB_WRBLOCK= 0x14,  // M->D  reliable BATCHED window write: base:u32 count:u16 count*(val:u16). Proxy
                         //       applies the words, replies DSPB_WRBLOCK status:u8(0x01) crc16:u16 over the
                         //       words; host resends on crc-mismatch/timeout (idempotent). Coalesces the
                         //       contiguous boot block pump (512 tiny frames -> ~5 big ones) so the reply
                         //       clears the ESP relay's post-burst echo-blank + every word is CRC-verified.
};

// DSPB_CTRL ids — every line that gates the DSP, INCLUDING the CCONT rail.
enum {
    DSPB_CTRL_RESET   = 0x00, // 0x20002 bit0 : 1 = reset released (run)
    DSPB_CTRL_DSPINT  = 0x01, // 0x30000 bit2 : DSPINT doorbell strobe (INT2)
    DSPB_CTRL_APIMODE = 0x02, // 0x30000 bit1 : HPI-window enable
    DSPB_CTRL_CCONT   = 0x10, // CCONT reg write : val = (reg<<8)|byte
};

// DSPB_IRQ ids — DSP->MCU interrupt lines (map to mad2_raise_fiq line / IRQ).
enum {
    DSPB_IRQ_MDIRCV = 0x00, // FIQ0
    DSPB_IRQ_MDISND = 0x01, // FIQ1
    DSPB_IRQ_BLOCK  = 0x02, // IRQ4 (code-block ack)
};

struct Mad2;
extern const struct DspOps mad2_dsp_bridge;   // src/mad2/dsp_bridge.c

// True + lazily connect if DSP_BRIDGE is set; lets mad2_init pick the backend.
int dsp_bridge_enabled(void);

#endif
