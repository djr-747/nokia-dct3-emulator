// Nokia 6210 (NPE-3) / 6250 (NHM-3) DSP HLE responder — ROM-6 base (mad2_dsp_default) plus the
// self-test-complete ack, modelled on the 7110's (src/models/7110/dsp_7110.c). Both are ROM-6
// (in-flash-gen DSP, F073-fingerprint-confirmed — NOT the 7110's ROM-4), so the boot block-
// upload / mailbox / keep-alive bodies are the shared dsp_default_* helpers; only the self-test-
// complete reply is bespoke here, kept in its own TU so the 3310 (ROM-6) guard boot stays byte-
// identical by construction.
//
// SHARED 6210 + 6250 (like-for-like, 2026-07-14): the 6250 self-test is byte-identical in shape
// (gen_sig: bit2 sequencer 0x30774A, bit2-clear handler 0x3046xx, flag setter 0x42A054 all
// exact-match the 6210's). The ONLY per-model differences are the RAM addresses, which this
// responder reads from the profile: the self-test result flag is m->fw.dsp_uploaded (6210
// 0x16FFE4 / 6250 0x16E474) and the verdict is m->fw.verdict (6210 0x17FD99 / 6250 0x17FD15).
// So one responder serves both — each profile just carries its own addresses.
//
// ── CONTACT-SERVICE GATE CHAIN (traced 2026-07-14; firmware/Nokia 6210 NPE-3 v5.56 A.fls) ────
// Verdict cell [0x17FD99]. Two sub-gates, IDENTICAL in shape to the 7110's:
//   (1) BUILDER gate 0x4AFFB4 returns byte [0x16FFE4]; verdict bit6 kept iff ==1. [0x16FFE4] is
//       set =1 by the firmware's own IRQ4 process stage (0x427322) once the DSP block-ack pump
//       has run — so wiring the profile's dsp_uploaded = 0x16FFE4 lets the shared pump raise IRQ4
//       and the firmware sets it. Verdict advances 40->C0->C4 (bit6 survives). (No poke.)
//   (2) SEQUENCER gate 0x3058DE: `ldrb [0x17FD99]; lsr #3; bcs <clear bit6>` — `lsr #3` puts
//       verdict BIT2 (0x04) into carry; a SET bit2 clears bit6 -> CONTACT SERVICE (C4->84). bit2
//       = "self-test result pending / not yet acknowledged". This is the same instruction-level
//       gate the 7110 has at 0x310D5A. The DSP clears it by posting a group-0x74 sub-13 (0x0D)
//       "self-test complete" record; the ring pump forwards it to the self-test task -> record
//       handler 0x3029D8 -> `and #0xFB` (0x302A60) clears verdict bit2. That is the cmd-13 reply
//       the shared dsp_default responder synthesizes for the ROM-6 siblings (3310/2100/3410).
//
// WHY THE SHARED RESPONDER MISSES THE 6210 (same reason as the 7110): dsp_default's responder
// only fires while the self-test ring is at its initial empty state head==tail==0x80. On the
// 6210 the self-test setup (0x426Dxx) has already loaded a record and advanced the ring (head
// 0x80 / tail 0x82) by the time the builder sets bit2. So the 6210 needs a position-aware post
// at the CURRENT tail — exactly what dsp_6210_ring_push does. FAITHFUL: a healthy DSP asked to
// self-test runs it and reports completion; we model that one reply. We NEVER write the verdict
// ourselves — the MCU's own handler 0x302A60 clears bit2.

#include "mad2/mad2.h"
#include "models/model.h"

// Re-arm state. This is EDGE-BASED, not fire-once-ever: the ack is re-armed every time verdict
// bit2 is CLEAR and posted on the next time it goes SET. A WARM REBOOT re-runs the firmware's
// self-test (verdict re-initialised through a bit2-clear state), which re-arms us — so a rebooted
// 6210/6250 posts the ack again and reaches standby, not CONTACT SERVICE. (A file-static "posted
// once" flag would survive the reboot and never re-fire, since the responder TU isn't reset by
// mad2_init.) File-static is fine: the arming is driven by observed state, not by process lifetime.
static uint8_t st_armed = 1;        // 1 = ready to post the {sub 0x0D} bit2-clear ack this cycle

// Deposit one DSP->MCU record {payload_len, group=0x74, payload...} at the CURRENT ring tail and
// raise FIQ0 — the same delivery the shared dsp_default self-test responder uses, but written at
// the live tail (not forced to 0x80) so it works after the self-test setup has advanced the ring.
// Returns 0 if the ring isn't initialised yet (caller retries next tick).
static int dsp_6210_ring_push(Mad2* m, const uint8_t* payload, int len) {
    uint32_t tp = m->fw.mdircv_tail & m->mem_mask;
    uint32_t hp = m->fw.mdircv_head & m->mem_mask;
    uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
    uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
    if (tail < 0x80 || head < 0x80) return 0;                 // ring not set up yet
    uint16_t words = (uint16_t)(1 + (len + 1) / 2);
    uint16_t pos = tail;
    if ((uint16_t)(pos - 0x80 + words) > 100) return 0;       // would overrun the 100-word buffer
    uint32_t q = (m->fw.mdircv_q & m->mem_mask) + (uint32_t)(pos - 0x80) * 2;
    m->mem[q]     = (uint8_t)len;     // word0 high = payload length
    m->mem[q + 1] = 0x74;             // word0 low  = group 0x74 (self-test class)
    for (int i = 0; i < len; ++i) m->mem[q + 2 + i] = payload[i];
    uint16_t nt = (uint16_t)(pos + words);
    m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
    mad2_raise_fiq(m, 0);             // DSP signals MDIRCV
    return 1;
}

static int dsp_6210_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    return dsp_default_read(m, addr, size, ram_value, out);
}

static int dsp_6210_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    return dsp_default_write(m, addr, size, value);
}

static void dsp_6210_tick(Mad2* m) {
    dsp_default_tick(m);
    if (!m->mem) return;
    // (The in-flash RF-calib checksum repair — profile .calib_cksum_off/val — is applied by the
    // shared platform tick in mad2_timers.c, so it works for ANY model, not just this responder.)

    // Self-test-complete ack: once the builder has set verdict bit2 (result pending) with the
    // DSP-ready flag (m->fw.dsp_uploaded) up, post the group-0x74 sub-13 record that clears bit2.
    // Both addresses are per-model profile data, so this is model-agnostic (6210 + 6250).
    // Edge-based: re-arm whenever bit2 is CLEAR (incl. after a warm reboot re-runs the self-test),
    // post once per bit2 rising edge. This is what makes a rebooted phone reach standby again.
    //
    // DSPVIS: the observed MDISND {0x70,0x0D} run-request (m->dsp_st_req, set per request —
    // so a warm-reboot re-run re-arms naturally) replaces the verdict-bit2 edge, and the
    // internal dsp_running latch replaces the [dsp_uploaded] read. Same reply, no MCU-private
    // RAM. (The 6210/6250 stream the same group-0x70 record sequence; sweep-verified 2026-07-15.)
    if (m->dsp_vis) {
        if (m->dsp_st_req && m->dsp_running) {
            const uint8_t bit2_ack[] = { 0x0D, 0x00 };   // -> handler 0x302A60 `and #0xFB` clears bit2
            if (dsp_6210_ring_push(m, bit2_ack, 2)) m->dsp_st_req = 0;
        }
        return;
    }
    uint32_t vd  = m->fw.verdict & m->mem_mask;
    uint32_t rdy = m->fw.dsp_uploaded & m->mem_mask;
    if (!(m->mem[vd] & 0x04)) {
        st_armed = 1;                                    // bit2 clear -> ready for the next cycle
    } else if (st_armed && m->fw.dsp_uploaded && m->mem[rdy]) {
        const uint8_t bit2_ack[] = { 0x0D, 0x00 };       // -> handler 0x302A60 `and #0xFB` clears bit2
        if (dsp_6210_ring_push(m, bit2_ack, 2)) st_armed = 0;
    }
}

const DspOps mad2_dsp_6210 = {
    "6210", dsp_6210_read, dsp_6210_write, dsp_6210_tick, dsp_hle_tone,
};
