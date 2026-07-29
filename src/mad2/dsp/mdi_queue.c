// mdi_queue — see mdi_queue.h. Extracted verbatim from the ROM-4/ROM-6 copies, which were
// line-for-line equivalent over byte-identical record types.

#include "mad2/dsp/mdi_queue.h"
#include "mad2/dsp/mdi.h"
#include "mad2/mad2.h"
#include "mad2/mad2_internal.h"

#include <string.h>

void mdi_queue_enqueue(MdiQueue* q, uint64_t now,
                       uint8_t op, const uint8_t* payload, uint8_t len) {
    uint8_t nt = (uint8_t)((q->p_tail + 1u) % MDI_PENDING_N);
    if (nt == q->p_head) return;                    // FIFO full: drop (real overflow)
    MdiRec* rec = &q->pending[q->p_tail];
    rec->op = op; rec->len = len;
    if (len > MDI_RCVMAX) len = MDI_RCVMAX, rec->len = len;
    for (uint8_t i = 0; i < len; ++i) rec->bytes[i] = payload ? payload[i] : 0u;
    rec->enq = now; rec->due = now;
    q->p_tail = nt;
}

void mdi_queue_enqueue_at(MdiQueue* q, uint64_t now, uint64_t due,
                          uint8_t op, const uint8_t* payload, uint8_t len) {
    for (unsigned i = 0; i < MDI_DELAYED_N; ++i) {
        MdiRec* rec = &q->delayed[i];
        if (rec->used) continue;
        rec->op = op; rec->len = (len > MDI_RCVMAX) ? MDI_RCVMAX : len;
        for (uint8_t k = 0; k < rec->len; ++k) rec->bytes[k] = payload ? payload[k] : 0u;
        rec->enq = now; rec->due = due; rec->used = 1;
        return;
    }
    // delayed queue full: fall back to immediate (drop the delay rather than the record).
    mdi_queue_enqueue(q, now, op, payload, len);
}

void mdi_queue_pump_delayed(MdiQueue* q, uint64_t now) {
    for (unsigned i = 0; i < MDI_DELAYED_N; ++i) {
        MdiRec* rec = &q->delayed[i];
        if (rec->used && now >= rec->due) {
            mdi_queue_enqueue(q, now, rec->op, rec->bytes, rec->len);
            rec->used = 0;
        }
    }
}

void mdi_queue_expire_stale(MdiQueue* q, uint64_t now, uint64_t ttl) {
    while (q->p_head != q->p_tail) {
        MdiRec* rec = &q->pending[q->p_head];
        if (now - rec->enq <= ttl) break;           // FIFO is roughly time-ordered
        q->p_head = (uint8_t)((q->p_head + 1u) % MDI_PENDING_N);
    }
}

const MdiRec* mdi_queue_peek(const MdiQueue* q) {
    return (q->p_head == q->p_tail) ? 0 : &q->pending[q->p_head];
}

int mdi_queue_post(struct Mad2* m, MdiQueue* q, uint64_t ka_cyc) {
    if (q->p_head == q->p_tail) return 0;           // FIFO empty
    MdiRec* rec = &q->pending[q->p_head];

    // Ring posting: FAITHFUL producer semantics. We write at our own producer cursor and
    // advance only it; the MCU owns the consumer (0x1ca) and we never touch it. Several
    // records may be in flight, and a record may straddle the ring wrap — the MCU's drain
    // reads halfword-by-halfword and wraps MID-RECORD (3410 v5.46 0x34832A..0x348334).
    // See mdi_d2m_append for the full derivation.
    if (!mdi_d2m_append(m->mem, m->mem_mask, m->fw.mdircv_q, m->fw.mdircv_tail,
                        m->fw.mdircv_head, rec->op, rec->bytes, rec->len)) return 0;

    mad2_raise_fiq(m, 0);
    q->p_head = (uint8_t)((q->p_head + 1u) % MDI_PENDING_N);
    // Real d2m traffic feeds the firmware's MDI-activity counter — re-pace the idle telemetry
    // heartbeat so it only ever fills genuine DSP silence.
    m->dsp_hb_next_cyc = m->rtc_mono + ka_cyc;
    return 1;
}
