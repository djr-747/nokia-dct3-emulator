// ROM-4 DSP behaviour for the MAD2 platform — the 5110 / 6110 generation (NSE-1, early-MAD2
// "LEAD" = customised TMS320C54x). See docs/research/5110-.
//
// WHY A SEPARATE MODULE: Nokia shared one DSP ROM across whole
// phone generations. Our HLE responder is therefore organised BY ROM REVISION, not per-model:
//   * ROM 4  -> 5110 / 6110           (this file, mad2_dsp_rom4)
//   * ROM 6  -> 33xx/34xx/82xx/8850   (src/mad2/dsp_default.c, mad2_dsp_default — the 3310-tuned
//                                      0xE4-watchdog / group-0x03 keep-alive model)
// Keeping them in distinct translation units means porting ROM-4 fidelity here can NEVER regress
// the byte-identical 3310 (ROM-6) boot that `make guard` pins.
//
// GROUNDING — the real ROM-4 DSP conversation (captured under DSP54_COSIM=1, the recovered C54x
// image running live; COMMLOG, 2026-06-16):
//   * boot version handshake: the real DSP writes VERSION_A/B (DSP 0x802/0x803 = MCU
//     0x10004/0x10006) = 0x0004. The shared ROM-6 path fakes 6. The MCU serial-bus loader
//     (0x271AF4) only cross-checks [0x10004]==[0x10006] (equality, not the value), so faithful
//     version 4 boots through to the same "Security code" MMI (lcd 4921792d) as ROM-6 did.
//   * run-mode emits 24 HINT (BSCR.b3 -> IRQ4) doorbell edges, a demand-paged overlay upload
//     (UPLOADHEADER descriptors + [0x872]=4 GO), and a PORT1/2/3 mailbox status stream.
//
// CURRENT STATE — this is a faithful-by-revision SCAFFOLD. The MCU-observable boot OUTCOME is
// already correct on the shared mailbox logic (the 5110 reached the identical screen on ROM-6),
// so ROM-4 starts by delegating the mailbox ack / upload-pace / self-test / keep-alive bodies to
// the shared dsp_default_* helpers (non-static, declared in mad2.h) and asserts only the one
// proven ROM-4 divergence: the reported DSP version (carried in the profile as dsp_boot_ready=4).
//
// TODO (port from cosim, each guard-verified): the per-block/overlay HINT cadence, the
// demand-page overlay GO acks, and the run-mode PORT status stream — so the web/non-cosim 5110
// (which cannot link the C54x core) reproduces the real conversation shape, not just the outcome.

#include "mad2/mad2.h"

// ROM-4 read: today identical to the shared DCT3 mailbox model. The ROM-4 version trait (4 vs the
// ROM-6 fake of 6) lives in the 5110 profile's dsp_boot_ready, which dsp_default_read returns on
// the boot-status slots — so delegation already yields the faithful version. Kept as an explicit
// wrapper (not a bare alias) so ROM-4-specific read behaviour has a home to grow into.
static int dsp_rom4_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    return dsp_default_read(m, addr, size, ram_value, out);
}

static int dsp_rom4_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    return dsp_default_write(m, addr, size, value);
}

static void dsp_rom4_tick(Mad2* m) {
    // FAITHFUL ROM-4 pump: the boot-essential bodies (block-ack IRQ4, COBBA consume, self-test
    // responder) are shared with ROM-6 via dsp_default_tick, but the perpetual MDIRCV keep-alive
    // heartbeat is a 3310/ROM-6-ism the real ROM-4 DSP does NOT do — COSIM-GROUNDED: 150M run
    // shows the DSP silent on MDIRCV after ~55M (only MCU CTSI-WDT kicks follow); the 5110 has no
    // 0xE4 MDI-activity watchdog (its reason-0x68 is the EEPROM-fault latch, fixed at EEPROM load).
    // So suppress the keep-alive on the ROM-4 path. See docs/research/5110-.
    m->dsp_no_keepalive = 1;
    dsp_default_tick(m);
}

const DspOps mad2_dsp_rom4 = {
    "rom4", dsp_rom4_read, dsp_rom4_write, dsp_rom4_tick, dsp_hle_tone,
};
