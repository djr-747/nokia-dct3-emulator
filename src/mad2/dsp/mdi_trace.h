// mdi_trace — ONE MDI ring tracer, shared by every DSP engine.
//
// Before this existed each engine logged the ring its own way: `[rom4 pkt] m2d type=..` under
// ROM4_PKT, `[rom6] SIML m2d {..}` under ROM6_LOG, `[siml]`/`[dspvis]` in the legacy mailbox
// body under DSPSIML_LOG/DSPVIS_LOG, and nothing at all for the C54x co-sim. Four formats, four
// knobs, and the one configuration that mattered most — the real DSP under co-sim — was the one
// with no trace. Worse, each logger sat at its own engine's *protocol* site, so it only ever saw
// traffic that engine itself produced.
//
// This traces the TRANSPORT instead: the two hardware rings and their four cursors. Both rings
// are MAD2 HPI shared RAM — identical silicon on every DCT3 — so whoever moves a cursor (HLE
// engine, real C54x, or the MCU) shows up the same way.
//
//   MDISND (m2d, MCU->DSP)   tail advances = MCU produced      head advances = DSP consumed
//   MDIRCV (d2m, DSP->MCU)   tail advances = DSP produced      head advances = MCU consumed
//
// Cursors are WORD indices into their ring; a record is {len,op} in word 0 followed by `len`
// payload bytes, and a record MAY straddle the ring wrap (measured on 3410 v5.46 — the MCU's
// drain reads halfword-by-halfword and wraps mid-record).
//
// The only per-engine difference is HOW to read a ring byte, which is why `peek` is a callback:
// the HLE engines' ring lives in MCU RAM (`m->mem`), while under co-sim it lives in the DSP's
// memory and is reached through the HPI window. Same decode, same format, either way.
//
// Env: MDILOG=1 records (produce side), MDILOG=2 also consume events and empty-window cursor
// moves. Default off -> zero behaviour change, which is why `make guard` is unaffected.
#ifndef MAD2_MDI_TRACE_H
#define MAD2_MDI_TRACE_H

#include <stdint.h>

struct Mad2;

// Reads ONE byte of ring memory at an MCU-space address. `ctx` is the caller's backend handle
// (unused for the plain m->mem reader; the co-sim passes its Dsp54*).
typedef uint8_t (*MdiPeekFn)(const struct Mad2* m, void* ctx, uint32_t mcu_addr);

// Per-observer cursor memory. One instance per viewpoint: the MCU-RAM view and the co-sim's
// HPI view are separate observers of (under co-sim) two different copies of the same rings.
typedef struct MdiTrace {
    uint16_t snd_tail, snd_head;     // MDISND: MCU produce / DSP consume
    uint16_t rcv_tail, rcv_head;     // MDIRCV: DSP produce / MCU consume
    uint8_t  primed;                 // first poll only latches, never reports a false burst
} MdiTrace;

// MDILOG level (0 = off). Parsed once; cheap enough to call per tick.
int mdi_trace_level(void);

// Sample the four cursors and report whatever moved since the last call. `tag` names the
// viewpoint in the output ("mcu", "cosim") so two observers never look like one.
void mdi_trace_poll(const struct Mad2* m, MdiTrace* t, const char* tag,
                    MdiPeekFn peek, void* ctx);

// The plain reader: ring memory straight out of MCU RAM. Used by every HLE engine.
uint8_t mdi_trace_peek_mem(const struct Mad2* m, void* ctx, uint32_t mcu_addr);

#endif
