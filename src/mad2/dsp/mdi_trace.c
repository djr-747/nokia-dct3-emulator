// mdi_trace — see mdi_trace.h for why this replaces the per-engine ring logs.
#include "mad2/dsp/mdi_trace.h"
#include "mad2/mad2.h"
#include <stdio.h>
#include <stdlib.h>

int mdi_trace_level(void) {
    static int lvl = -1;
    if (lvl < 0) { const char* e = getenv("MDILOG"); lvl = (e && *e) ? atoi(e) : 0; }
    return lvl;
}

uint8_t mdi_trace_peek_mem(const struct Mad2* m, void* ctx, uint32_t mcu_addr) {
    (void)ctx;
    if (!m->mem) return 0;
    return m->mem[mcu_addr & m->mem_mask];
}

// Cursors are stored big-endian in shared RAM, same as every other MAD2 mailbox word.
static uint16_t peek16(const struct Mad2* m, MdiPeekFn peek, void* ctx, uint32_t addr) {
    return (uint16_t)((peek(m, ctx, addr) << 8) | peek(m, ctx, addr + 1u));
}

// Decode and print the records occupying word range [from, to) of one ring.
//
// `from`/`to` are WORD indices; the range wraps. We walk record-by-record: word 0 of each is
// {len, op} and the payload is the next `len` bytes, so the next record starts `1 + (len+1)/2`
// words along. A malformed/zero-length walk would spin, so the step is floored at one word and
// the loop is bounded by the ring size.
static void dump_range(const struct Mad2* m, MdiPeekFn peek, void* ctx, const char* tag,
                       const char* dir, const char* ring_name,
                       uint32_t base, uint32_t ring_bytes, uint16_t from, uint16_t to) {
    if (!ring_bytes) return;
    const uint32_t ring_words = ring_bytes / 2u;
    if (!ring_words) return;
    uint16_t w = from;
    for (unsigned guard = 0; guard < ring_words && w != to; ++guard) {
        const uint32_t off = (uint32_t)w * 2u;
        const uint8_t  len = peek(m, ctx, base + (off % ring_bytes));
        const uint8_t  op  = peek(m, ctx, base + ((off + 1u) % ring_bytes));
        // A {0,0} word is not a record — it is untouched ring. The cursors legitimately sweep
        // across zeroed ring at init and after a wrap, and reporting those as records buried the
        // real traffic under hundreds of empty lines. Skip them, but keep walking.
        if (!op && !len) { w = (uint16_t)((w + 1u) % ring_words); continue; }
        printf("[mdi] %-5s %s %s op=%02X len=%-3u word=%-3u step=%llu data=",
               tag, dir, ring_name, op, len, (unsigned)w,
               (unsigned long long)m->dsp_steps);
        for (unsigned i = 0; i < len; ++i)
            printf("%02X", peek(m, ctx, base + ((off + 2u + i) % ring_bytes)));
        printf("\n");
        // Advance past word0 + the payload, rounded up to whole words (records are word-aligned).
        uint32_t step = 1u + ((uint32_t)len + 1u) / 2u;
        if (step < 1u) step = 1u;
        w = (uint16_t)((w + step) % ring_words);
    }
}

void mdi_trace_poll(const struct Mad2* m, MdiTrace* t, const char* tag,
                    MdiPeekFn peek, void* ctx) {
    if (!mdi_trace_level() || !m) return;
    const uint32_t snd_q = m->fw.mdisnd_q, snd_tp = m->fw.mdisnd_tail;
    const uint32_t rcv_q = m->fw.mdircv_q, rcv_tp = m->fw.mdircv_tail;
    if (!snd_q || !snd_tp || !rcv_q || !rcv_tp) return;   // addresses not resolved yet
    // The partner cursor sits one word above its tail in every DCT3 layout (3310: MDISND
    // tail 0x100A4 / head 0x100A6, MDIRCV tail 0x101C8 / head 0x101CA), which is the same
    // pairing the co-sim glue reads directly.
    const uint32_t snd_hp = snd_tp + 2u, rcv_hp = rcv_tp + 2u;
    const uint32_t snd_bytes = snd_tp - snd_q, rcv_bytes = rcv_tp - rcv_q;

    const uint16_t snd_tail = peek16(m, peek, ctx, snd_tp), snd_head = peek16(m, peek, ctx, snd_hp);
    const uint16_t rcv_tail = peek16(m, peek, ctx, rcv_tp), rcv_head = peek16(m, peek, ctx, rcv_hp);

    if (!t->primed) {                       // latch only: never report the pre-existing state
        t->primed = 1;
        t->snd_tail = snd_tail; t->snd_head = snd_head;
        t->rcv_tail = rcv_tail; t->rcv_head = rcv_head;
        return;
    }
    const int lvl = mdi_trace_level();
    // Produce sides carry the payload — these are the lines that answer "did anyone reply?".
    if (snd_tail != t->snd_tail)
        dump_range(m, peek, ctx, tag, "m2d", "snd", snd_q, snd_bytes, t->snd_tail, snd_tail);
    if (rcv_tail != t->rcv_tail)
        dump_range(m, peek, ctx, tag, "d2m", "rcv", rcv_q, rcv_bytes, t->rcv_tail, rcv_tail);
    if (lvl >= 2) {                          // consume sides: cursor movement, no payload re-dump
        if (snd_head != t->snd_head)
            printf("[mdi] %-5s m2d snd CONSUMED %u->%u step=%llu\n", tag,
                   (unsigned)t->snd_head, (unsigned)snd_head, (unsigned long long)m->dsp_steps);
        if (rcv_head != t->rcv_head)
            printf("[mdi] %-5s d2m rcv CONSUMED %u->%u step=%llu\n", tag,
                   (unsigned)t->rcv_head, (unsigned)rcv_head, (unsigned long long)m->dsp_steps);
    }
    t->snd_tail = snd_tail; t->snd_head = snd_head;
    t->rcv_tail = rcv_tail; t->rcv_head = rcv_head;
}
