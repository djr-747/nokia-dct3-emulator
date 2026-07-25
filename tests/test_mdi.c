// Unit tests for the shared MDI ring/frame helpers (src/mad2/dsp/mdi.h).
// Pure arithmetic — encodes the campaign's corrected d2m ring + frame byte order.
#include <stdio.h>
#include <assert.h>
#include "mad2/dsp/mdi.h"

int main(void) {
    // Frame word: LOW = opcode, HIGH = length.
    assert(mdi_frame_opcode(0x1874) == 0x74);   // e.g. a 0x74 frame, length 0x18
    assert(mdi_frame_len(0x1874)    == 0x18);
    assert(mdi_frame_word(0x74, 0x18) == 0x1874);

    // Round-trip across the byte space.
    for (int op = 0; op < 256; op += 37)
        for (int ln = 0; ln < 256; ln += 41) {
            uint16_t w = mdi_frame_word((uint8_t)op, (uint8_t)ln);
            assert(mdi_frame_opcode(w) == op && mdi_frame_len(w) == ln);
        }

    // d2m ring occupancy = (producer - consumer) mod 100, including wraparound.
    assert(mdi_ring_avail(5, 2, 100) == 3);
    assert(mdi_ring_avail(2, 2, 100) == 0);      // empty
    assert(mdi_ring_avail(1, 99, 100) == 2);     // wrap: (1 - 99) mod 100 = 2
    assert(mdi_ring_avail(0, 50, 100) == 50);    // wrap
    assert(mdi_ring_avail(99, 0, 100) == 99);    // nearly full

    // --- d2m ring geometry (pinned from disasm; ENGINE-VERIFY §6) --------------------
    // Occupancy is base-invariant: the real cursors are indices 0x80..0xE3, and
    // (producer - consumer) mod 100 cancels the 0x80 offset.
    assert(mdi_ring_avail(0x80, 0x80, 100) == 0);        // empty (both at slot 0)
    assert(mdi_ring_avail(0x90, 0x80, 100) == 16);       // 16 frames queued
    assert(mdi_ring_avail(0x80, 0xE3, 100) == 1);        // producer wrapped 1 ahead of consumer
    assert(mdi_ring_avail(0xE3, 0x80, 100) == 99);       // nearly full (99 = capacity)

    // Index advance wraps 0xE3 -> 0x80 (element wrap 0x101C8 -> 0x10100 at 0x2BACA6).
    assert(mdi_d2m_index_next(0x80) == 0x81);
    assert(mdi_d2m_index_next(0xE2) == 0xE3);
    assert(mdi_d2m_index_next(0xE3) == 0x80);            // wrap
    // Full round-trip over the whole ring returns to the start after exactly 100 steps.
    {
        uint16_t idx = MDI_D2M_FIRST_IDX;
        for (int i = 0; i < 100; ++i) idx = mdi_d2m_index_next(idx);
        assert(idx == MDI_D2M_FIRST_IDX);
    }

    // Reserve-one capacity: post allowed until avail hits 99 (empty-test would alias a
    // full 100 back to empty), then blocked. P2-MUST-PIN #1.
    assert(MDI_D2M_CAPACITY == 99);
    assert(mdi_d2m_can_post(0x80, 0x80));                // empty -> ok
    assert(mdi_d2m_can_post(0xE2, 0x80));                // avail 98 -> ok
    assert(!mdi_d2m_can_post(0xE3, 0x80));               // avail 99 (full) -> blocked
    assert(!mdi_d2m_can_post(0x80, 0x81));               // avail 99 across wrap -> blocked

    // --- APIRAM big-endian access + d2m post (raw buffer; addresses are parameters) -----
    {
        static uint8_t mem[0x400];
        const uint32_t mask = 0x3FFu;
        for (unsigned i = 0; i < sizeof mem; ++i) mem[i] = 0;

        // BE 16-bit: high byte at the lower address.
        mdi_w16be(mem, mask, 0x10, 0xAB12);
        assert(mem[0x10] == 0xAB && mem[0x11] == 0x12);
        assert(mdi_r16be(mem, mask, 0x10) == 0xAB12);
        // Address masking wraps the second byte (0x3FF -> 0x400 & mask = 0x000).
        mdi_w16be(mem, mask, 0x3FF, 0x9C5D);
        assert(mem[0x3FF] == 0x9C && mem[0x000] == 0x5D);
        assert(mdi_r16be(mem, mask, 0x3FF) == 0x9C5D);

        // d2m post: ring_base 0x100, producer cursor @0x08, consumer @0x0A.
        const uint32_t base = 0x100, pcur = 0x08, ccur = 0x0A;
        mdi_w16be(mem, mask, pcur, 0x80);                // producer idx 0x80
        mdi_w16be(mem, mask, ccur, 0x80);                // consumer idx 0x80 (empty)

        assert(mdi_d2m_post(mem, mask, base, pcur, ccur, 0x92, 0x03) == 1);
        assert(mem[0x100] == 0x03 && mem[0x101] == 0x92);   // BE {len,opcode} at slot 0x80
        assert(mdi_r16be(mem, mask, pcur) == 0x81);         // producer advanced

        assert(mdi_d2m_post(mem, mask, base, pcur, ccur, 0x74, 0x18) == 1);
        assert(mem[0x102] == 0x18 && mem[0x103] == 0x74);   // next slot 0x81
        assert(mdi_r16be(mem, mask, pcur) == 0x82);

        // Reserve-one: fill to avail 99, then post must fail (would alias full->empty).
        mdi_w16be(mem, mask, pcur, 0xE2);
        mdi_w16be(mem, mask, ccur, 0x80);                // avail 98
        assert(mdi_d2m_post(mem, mask, base, pcur, ccur, 0x01, 0x00) == 1);  // -> avail 99
        assert(mdi_r16be(mem, mask, pcur) == 0xE3);
        assert(mdi_d2m_post(mem, mask, base, pcur, ccur, 0x02, 0x00) == 0);  // full -> dropped
        assert(mdi_r16be(mem, mask, pcur) == 0xE3);          // producer unchanged on drop

        // Producer index wraps 0xE3 -> 0x80; slot 0xE3 element = base + (0xE3-0x80)*2 = 0x1C6.
        mdi_w16be(mem, mask, pcur, 0xE3);
        mdi_w16be(mem, mask, ccur, 0x81);                // avail 98 -> post ok
        assert(mdi_d2m_post(mem, mask, base, pcur, ccur, 0x88, 0x05) == 1);
        assert(mem[0x1C6] == 0x05 && mem[0x1C7] == 0x88);
        assert(mdi_r16be(mem, mask, pcur) == 0x80);          // wrapped

        // --- mdi_d2m_deposit: faithful empty-ring record (mirrors the rom4-body responders) --
        for (unsigned i = 0; i < sizeof mem; ++i) mem[i] = 0;
        mdi_w16be(mem, mask, pcur, 0x80);                // empty ring at slot 0x80
        mdi_w16be(mem, mask, ccur, 0x80);
        // Exact bytes of the rom4 body's self-test frame: {0x74, [0x0D,0x00]}, len 2 -> 2 words.
        { const uint8_t pl[2] = {0x0D, 0x00};
          assert(mdi_d2m_deposit(mem, mask, base, pcur, ccur, 0x74, pl, 2) == 1); }
        assert(mem[0x100] == 0x02 && mem[0x101] == 0x74);    // word0 {HIGH=len, LOW=opcode}
        assert(mem[0x102] == 0x0D && mem[0x103] == 0x00);    // payload
        assert(mdi_r16be(mem, mask, ccur) == 0x80);          // consumer at record start
        assert(mdi_r16be(mem, mask, pcur) == 0x82);          // producer past it (2 words)

        // Non-empty ring -> refused (caller retries next tick, as the real DSP waits).
        assert(mdi_d2m_deposit(mem, mask, base, pcur, ccur, 0x92, 0, 1) == 0);

        // Straddle relocate: a record that would cross the wrap moves to FIRST_IDX (dsp_rom4.c).
        mdi_w16be(mem, mask, pcur, 0xE3);                // 1 slot before END, empty
        mdi_w16be(mem, mask, ccur, 0xE3);
        assert(mdi_d2m_deposit(mem, mask, base, pcur, ccur, 0x74, 0, 2) == 1);  // words 2, 0xE3+2>0xE3
        assert(mdi_r16be(mem, mask, ccur) == 0x80);          // relocated to ring start
        assert(mdi_r16be(mem, mask, pcur) == 0x82);
        assert(mem[0x100] == 0x02 && mem[0x101] == 0x74);    // written at FIRST_IDX
    }

    printf("test_mdi: OK\n");
    return 0;
}
