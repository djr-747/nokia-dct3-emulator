// Shared MDI ring/frame ABI — the single encoding of the campaign's CORRECTED mailbox
// facts (docs/sim-dsp-groundup/): the d2m ring is a 100-slot ring (word range 0x80..0xE3)
// with the DSP-advanced producer cursor and the MCU-advanced consumer cursor; occupancy
// is (producer - consumer) mod 100. A frame word packs LOW byte = opcode, HIGH byte =
// length (the byte order the old overlay had reversed).
//
// These are pure functions (header-inline) so they are unit-testable with no Mad2
// dependency. The Mad2-facing ring wrappers (which read the cursors from the profile's
// mailbox-window addresses) arrive with the P1 SIML responder that first needs them.
#ifndef MAD2_MDI_H
#define MAD2_MDI_H

#include <stdint.h>

// d2m ring occupancy: (producer - consumer) mod ring_slots. The DSP advances `producer`
// (3310 = [0x101C8]); the MCU advances `consumer` (3310 = [0x101CA]).
static inline unsigned mdi_ring_avail(uint16_t producer, uint16_t consumer, unsigned ring_slots) {
    return (unsigned)(((unsigned)producer + ring_slots - (unsigned)consumer) % ring_slots);
}

// MDI frame word: LOW byte = opcode, HIGH byte = length.
static inline uint8_t  mdi_frame_opcode(uint16_t word) { return (uint8_t)(word & 0xFFu); }
static inline uint8_t  mdi_frame_len(uint16_t word)    { return (uint8_t)(word >> 8); }
static inline uint16_t mdi_frame_word(uint8_t opcode, uint8_t len) {
    return (uint16_t)(((uint16_t)len << 8) | (uint16_t)opcode);
}

// --- d2m ring geometry (3310 v5.79, pinned from disasm; see ENGINE-VERIFY §6) ----------
// Each d2m ring slot holds ONE big-endian descriptor word {HIGH=len, LOW=opcode}; the
// MCU drain (MDIRCV_DEQUEUE 0x2BAC72) consumes exactly one word per frame. The stored
// cursor value is a HALFWORD INDEX, not a raw 0..99 counter: the d2m data ring begins at
// index 0x80 (element addr = window_base 0x10000 + index*2; 0x80 -> mdircv_q 0x10100),
// spanning 100 slots (indices 0x80..0xE3). Occupancy = (producer - consumer) mod 100 is
// base-invariant, so mdi_ring_avail() takes the raw index values directly.
//
// P2-MUST-PIN #1 RESOLVED: the drain's empty-test is producer==consumer (0x2BAC94-98), so
// a full 100 slots would alias to empty. The producer (the engine) therefore RESERVES ONE
// slot — usable capacity is 99, not 100.
#define MDI_RING_SLOTS     100u
#define MDI_D2M_FIRST_IDX  0x80u                        // ring index of d2m slot 0 (-> 0x10100)
#define MDI_D2M_LAST_IDX   (MDI_D2M_FIRST_IDX + MDI_RING_SLOTS - 1u)   // 0xE3
#define MDI_D2M_CAPACITY   (MDI_RING_SLOTS - 1u)        // reserve-one: 99 usable frames

// Advance a d2m cursor index by one slot, wrapping 0xE3 -> 0x80 (mirrors the drain's
// element wrap 0x101C8 -> 0x10100 at 0x2BACA6).
static inline uint16_t mdi_d2m_index_next(uint16_t idx) {
    unsigned rel = ((unsigned)idx - MDI_D2M_FIRST_IDX + 1u) % MDI_RING_SLOTS;
    return (uint16_t)(rel + MDI_D2M_FIRST_IDX);
}

// --- APIRAM access (big-endian, matching the ARM7TDMI-BE core / m->mem storage) ---------
// The DSP HPI window is byte-addressable RAM; the MCU reads/writes it as big-endian
// 16-bit cells (MDIRCV_DEQUEUE 0x2BAC72 does `ldrh` in BE mode). These take the raw backing
// buffer + mask so they are unit-testable with no Mad2 dependency; the Mad2-facing engine
// passes `m->mem`, `m->mem_mask`. `mask` is applied per byte (matches the core's addressing).
static inline uint16_t mdi_r16be(const uint8_t* mem, uint32_t mask, uint32_t addr) {
    return (uint16_t)(((uint16_t)mem[addr & mask] << 8) | (uint16_t)mem[(addr + 1u) & mask]);
}
static inline void mdi_w16be(uint8_t* mem, uint32_t mask, uint32_t addr, uint16_t val) {
    mem[addr & mask]        = (uint8_t)(val >> 8);
    mem[(addr + 1u) & mask] = (uint8_t)(val & 0xFFu);
}

// FAITHFUL producer append: write the record at the CURRENT producer index, wrapping
// halfword-by-halfword at the ring end, and advance ONLY the producer. Contrast
// mdi_d2m_deposit above, which relocates a would-straddle record to the ring start and then
// drags the MCU's consumer after it — a write real silicon cannot make.
//
// A record MAY straddle the wrap. That is not an assumption: the MCU's own drain loop reads
// the record one halfword at a time and wraps mid-record (3410 v5.46 0x34832A..0x348334):
//
//     0x348328  r6 = ring_end (window + 0x1C8)
//     0x34832A  r4 = [r2]              ; read halfword
//     0x34832E  r2 += 2
//     0x348330  cmp r2, r6
//     0x348332  bcc  +0                ; still inside the ring -> keep going
//     0x348334  r2 = r5                ; WRAP to the ring start, mid-record
//
// So the producer simply writes through the wrap and the consumer follows it round. Neither
// side has to touch the other's cursor. The "a record may not wrap" rule we inherited from
// dsp_rom4.c does not hold for this drain.
//
// Returns 1 if appended, 0 if the ring lacks room (caller retries on a later tick).
static inline int mdi_d2m_append(uint8_t* mem, uint32_t mask, uint32_t ring_base,
                                 uint32_t producer_addr, uint32_t consumer_addr,
                                 uint8_t opcode, const uint8_t* payload, uint8_t len) {
    uint16_t p = mdi_r16be(mem, mask, producer_addr);
    uint16_t c = mdi_r16be(mem, mask, consumer_addr);
    unsigned words = 1u + ((unsigned)len + 1u) / 2u;            // word0 + ceil(len/2)
    if (mdi_ring_avail(p, c, MDI_RING_SLOTS) + words > MDI_D2M_CAPACITY) return 0;

    // Write `words` halfwords from the producer index, wrapping exactly as the drain does.
    uint16_t w = p;
    uint16_t hw = mdi_frame_word(opcode, len);
    for (unsigned k = 0; k < words; ++k) {
        uint32_t elem = ring_base + (uint32_t)(uint16_t)(w - MDI_D2M_FIRST_IDX) * 2u;
        if (k == 0) {
            mdi_w16be(mem, mask, elem, hw);                     // word0 = {HIGH=len, LOW=opcode}
        } else {
            unsigned off = (k - 1u) * 2u;                       // payload bytes for this word
            mem[elem & mask]       = (payload && off     < len) ? payload[off]     : 0u;
            mem[(elem + 1u) & mask]= (payload && off + 1u < len) ? payload[off + 1u] : 0u;
        }
        w = mdi_d2m_index_next(w);
    }
    mdi_w16be(mem, mask, producer_addr, w);                     // ONLY the producer moves
    return 1;
}

// ---------------------------------------------------------------------------
// CANONICAL MDI OPCODE REFERENCE (3410 v5.46, static RE — 2026-07-29)
//
// Derived from the firmware, not from observation, so it lists opcodes rom6 has
// never seen as well as the ones it has. Two separate namespaces — do not mix.
//
// (1) d2m (DSP -> MCU): what rom6 must PRODUCE.
//     Dispatched by the MDI receive task (task idx 4, entry 0x3ED764, prio 10,
//     stack 848) on byte[3] of the drained record. MEASURED: the dispatcher is a
//     13-entry jump table at 0x3ED7B0 covering 0x83..0x8F, plus four discrete
//     compares. Anything not listed falls to the shared ignore stub 0x3ED85E.
//
//       0x80 -> 0x3ED84C
//       0x83 -> 0x3ED844      (rom6: RSSI)
//       0x84 -> 0x3ED83C
//       0x85 -> IGNORED (shared stub 0x3ED85E)
//       0x86 -> 0x3ED834
//       0x87 -> 0x3ED82C
//       0x88 -> 0x3ED824      (rom6: nmeas result)
//       0x89 -> 0x3ED81C      (rom6: channel-changed confirm)
//       0x8A -> 0x3ED814
//       0x8B -> 0x3ED80C      (rom6: ALL_RSSI_RESULTS)
//       0x8C -> 0x3ED804
//       0x8D -> IGNORED (shared stub 0x3ED85E)
//       0x8E -> IGNORED (shared stub 0x3ED85E)
//       0x8F -> 0x3ED7FC      (rom6: emitted with 0x02 channel configure)
//       0x99 -> 0x3ED7F4
//       0x9A -> 0x3ED7EC
//       0x9C -> 0x3ED7E4
//
//     0x85/0x8D/0x8E are accepted-and-dropped: sending them is harmless but can
//     never advance firmware state. 0x81/0x82 and >=0x9D are out of range
//     entirely and reach no handler.
//
// (2) m2d (MCU -> DSP): what rom6 must CONSUME.
//     NOT YET ENUMERATED FROM THE IMAGE. The m2d record is built by the MDI send
//     task (task idx 3, entry 0x3E5DC8, prio 11) and pushed into the ring by
//     0x34821E (r0 = total byte length, r1 = record ptr whose byte[0] IS the
//     opcode; word0 = (len<<8)|opcode). 0x34821E has exactly one caller,
//     0x3E5F5C, inside that task. The opcode namespace is therefore the set of
//     record byte[0] values produced by task 3's producers, which is still open.
//     Opcodes rom6 handles today (0x02, 0x0C, 0x0F, 0x11, 0x1B, 0x46, 0x55,
//     0x56, 0x57) are OBSERVED, not enumerated — treat the list as incomplete.
//
// (3) NOT an MDI opcode namespace, despite appearances: 0x348674 is
//     DSP_HPI_FIELD_WRITE(field 0..53, value). Its 54-entry jump table at
//     0x3486A8 selects a bitfield inside the DSP HPI *control registers*
//     0x100A8..0x100C2 (read-modify-write tail at 0x3489A2/0x3489A4), gated on
//     [0x100E0]. These are register writes in shared RAM, NOT ring records, and
//     they are invisible to a ring-only trace. If rom6 ever needs to react to a
//     mode/config change that never shows up as a record, look here.
//
// (4) CSD user data: the opcode is NOT yet identified. Evidence bounding it:
//     no module of the data stack (pdgntb 0x2CA000-0x2CC000, dgntbl2r
//     0x3038B0-0x304B00, DPI 0x308000-0x309400, dgntbif 0x3E3000-0x3E5000)
//     contains ANY literal reference to the shared window 0x10000..0x101FF
//     except one: 0x101D0, referenced from 0x303E18 and 0x30852C. That cell is
//     `hw_dsp_ftd_api_dump[0..9]` — 10 u16 counters sitting in the shared HPI
//     window just past the MDIRCV cursors (0x101C8 tail / 0x101CA head), read
//     and zeroed by the L2RCOP statistics dumps. "FTD" = fax/transparent data.
//     So the CSD data conduit is a shared-window BLOCK at 0x101D0, reached
//     through the MDI send/receive tasks rather than by the data stack directly.
//     This is a LEAD, not a conclusion — the block's layout is unverified.
// ---------------------------------------------------------------------------

#endif // MAD2_MDI_H
