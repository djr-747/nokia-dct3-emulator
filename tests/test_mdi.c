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

    // Reserve-one capacity: a record may occupy up to 99 words (the empty-test would alias a
    // full 100 back to empty). P2-MUST-PIN #1. Exercised through mdi_d2m_append below.
    assert(MDI_D2M_CAPACITY == 99);

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

        // --- mdi_d2m_append: the faithful producer -------------------------------------
        // Writes at OUR producer cursor and advances only it. The MCU owns the consumer and
        // we must never write it — see the ownership note in mdi.h.
        { const uint8_t pl[2] = {0x0D, 0x00};            // rom4 self-test frame {0x74,[0D,00]}
          assert(mdi_d2m_append(mem, mask, base, pcur, ccur, 0x74, pl, 2) == 1); }
        assert(mem[0x100] == 0x02 && mem[0x101] == 0x74);   // word0 {HIGH=len, LOW=opcode}
        assert(mem[0x102] == 0x0D && mem[0x103] == 0x00);   // payload
        assert(mdi_r16be(mem, mask, ccur) == 0x80);         // CONSUMER UNTOUCHED
        assert(mdi_r16be(mem, mask, pcur) == 0x82);         // producer past the record (2 words)

        // Several records may be in flight: a non-empty ring does NOT refuse.
        assert(mdi_d2m_append(mem, mask, base, pcur, ccur, 0x92, 0, 1) == 1);
        assert(mdi_r16be(mem, mask, ccur) == 0x80);         // still untouched
        assert(mdi_r16be(mem, mask, pcur) == 0x84);         // 0x82 + 2 words

        // A record MAY straddle the wrap: it is written through the ring end and continues at
        // the start, because the MCU's drain reads halfword-by-halfword and wraps mid-record
        // (3410 v5.46 0x34832A..0x348334). It is NOT relocated, and the consumer is not moved.
        for (unsigned i = 0; i < sizeof mem; ++i) mem[i] = 0;
        mdi_w16be(mem, mask, pcur, 0xE3);                   // last slot, empty ring
        mdi_w16be(mem, mask, ccur, 0xE3);
        { const uint8_t pl[2] = {0xAA, 0xBB};               // 2 words: word0 + 1 payload word
          assert(mdi_d2m_append(mem, mask, base, pcur, ccur, 0x74, pl, 2) == 1); }
        assert(mem[0x1C6] == 0x02 && mem[0x1C7] == 0x74);   // word0 at slot 0xE3 (ring end)
        assert(mem[0x100] == 0xAA && mem[0x101] == 0xBB);   // payload wrapped to slot 0x80
        assert(mdi_r16be(mem, mask, ccur) == 0xE3);         // consumer STILL untouched
        assert(mdi_r16be(mem, mask, pcur) == 0x81);         // producer wrapped past the record

        // Reserve-one: a record that would fill the ring is refused, producer unchanged.
        mdi_w16be(mem, mask, pcur, 0x80);
        mdi_w16be(mem, mask, ccur, 0x81);                   // avail 99 -> no room at all
        assert(mdi_d2m_append(mem, mask, base, pcur, ccur, 0x02, 0, 0) == 0);
        assert(mdi_r16be(mem, mask, pcur) == 0x80);         // unchanged on drop
    }

    printf("test_mdi: OK\n");
    return 0;
}
