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

// True if the producer may post another frame without the full->empty alias. Pass the raw
// cursor index values (producer = [0x101C8], consumer = [0x101CA]).
static inline int mdi_d2m_can_post(uint16_t producer, uint16_t consumer) {
    return mdi_ring_avail(producer, consumer, MDI_RING_SLOTS) < MDI_D2M_CAPACITY;
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

// Post one d2m descriptor word {HIGH=len, LOW=opcode} at the producer slot and advance the
// producer index (reserve-one full guard). `ring_base` = the d2m data base (slot at index
// 0x80 -> 3310 mdircv_q 0x10100); `producer_addr`/`consumer_addr` = the cursor cells
// ([0x101C8]/[0x101CA]). Returns 1 on success, 0 if the ring is full (frame dropped — the
// caller retries on a later tick, as the real DSP would when the MCU has not yet drained).
// Mirrors the drain's element math (elem = ring_base + (idx - 0x80)*2) and wrap.
static inline int mdi_d2m_post(uint8_t* mem, uint32_t mask, uint32_t ring_base,
                               uint32_t producer_addr, uint32_t consumer_addr,
                               uint8_t opcode, uint8_t len) {
    uint16_t p = mdi_r16be(mem, mask, producer_addr);
    uint16_t c = mdi_r16be(mem, mask, consumer_addr);
    if (!mdi_d2m_can_post(p, c)) return 0;
    uint32_t elem = ring_base + (uint32_t)(uint16_t)(p - MDI_D2M_FIRST_IDX) * 2u;
    mdi_w16be(mem, mask, elem, mdi_frame_word(opcode, len));
    mdi_w16be(mem, mask, producer_addr, mdi_d2m_index_next(p));
    return 1;
}

// Faithful DSP-responder deposit: place ONE record into an EMPTY d2m ring and hand it to the
// MCU. Mirrors the boot self-test / keep-alive / SIML responders in dsp_rom4.c exactly
// (the pattern the real DSP uses): only deposits when the ring is empty (producer==consumer),
// relocates the write position to FIRST_IDX if the record would straddle the wrap (a record
// may not wrap), writes word0 {HIGH=len, LOW=opcode} + `len` payload bytes, then repositions
// the consumer to the record start and the producer just past it (words = 1 + ceil(len/2)).
//
// `len` MUST be non-zero: task-4 (0x2EDC48) treats a drained frame whose high byte is 0 as
// empty / re-fans it to the UI, so a zero-length frame neither routes nor bumps the liveness
// counter. Returns 1 if deposited, 0 if the ring was not empty (the caller retries on a later
// tick, exactly as the real DSP waits for a free window before posting).
static inline int mdi_d2m_deposit(uint8_t* mem, uint32_t mask, uint32_t ring_base,
                                  uint32_t producer_addr, uint32_t consumer_addr,
                                  uint8_t opcode, const uint8_t* payload, uint8_t len) {
    uint16_t p = mdi_r16be(mem, mask, producer_addr);
    uint16_t c = mdi_r16be(mem, mask, consumer_addr);
    if (p != c) return 0;                                       // only into an empty ring
    unsigned words = 1u + ((unsigned)len + 1u) / 2u;            // word0 + ceil(len/2) payload words
    uint16_t w = p;
    if ((unsigned)w + words > (unsigned)MDI_D2M_LAST_IDX)       // record would straddle the wrap
        w = MDI_D2M_FIRST_IDX;                                  // -> relocate to ring start (dsp_rom4.c straddle rule)
    uint32_t elem = ring_base + (uint32_t)(uint16_t)(w - MDI_D2M_FIRST_IDX) * 2u;
    mdi_w16be(mem, mask, elem, mdi_frame_word(opcode, len));    // word0 = {HIGH=len, LOW=opcode}
    for (unsigned i = 0; i < len; ++i)
        mem[(elem + 2u + i) & mask] = payload ? payload[i] : 0u;
    mdi_w16be(mem, mask, consumer_addr, w);                     // MCU reads at w
    mdi_w16be(mem, mask, producer_addr, (uint16_t)(w + words)); // producer past the record
    return 1;
}

#endif // MAD2_MDI_H
