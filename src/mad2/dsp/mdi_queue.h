// mdi_queue — the DSP-side egress queues that front the single MDIRCV hardware ring.
//
// This is TRANSPORT, not protocol. The ring, its cursors and the software queues in front of
// it are the MAD2 HPI shared-RAM mechanism — identical silicon on every DCT3 — so they belong
// in one place rather than once per DSP engine. What ROM-4 and ROM-6 differ in is the
// vocabulary they speak over it, not how it is carried.
//
// It was previously duplicated: rom4_enqueue/_pump_delayed/_expire_stale/_pump_mdircv and
// rom6_* were the same functions over byte-identical record types. That duplication had a
// cost — the wrap-rule defect (see mdi_d2m_append) had to be found once and then fixed twice,
// and the ROM-4 half was very nearly left behind.
//
// Two queues front the one ring:
//   pending  FIFO of records ready to post
//   delayed  time-ordered slots that mature into the FIFO at their due cycle
// The pump posts ONE record per call into the ring and raises FIQ0. Records that wait unposted
// for more than 2 s are dropped, as the real DSP's would be.
#ifndef MAD2_MDI_QUEUE_H
#define MAD2_MDI_QUEUE_H

#include <stdint.h>

struct Mad2;

#define MDI_RCVMAX     168u   // max d2m payload bytes (0x8B ALL_RSSI_RESULTS = 166)
#define MDI_PENDING_N  48u    // FIFO of ready records
#define MDI_DELAYED_N  48u    // time-ordered delayed queue

// One queued MDIRCV (d2m) record awaiting egress into the single hardware ring.
typedef struct MdiRec {
    uint8_t  op;                 // MDI opcode (frame-word LOW byte)
    uint8_t  len;                // payload byte count (frame-word HIGH byte)
    uint8_t  bytes[MDI_RCVMAX];  // payload (what follows the {len,op} word)
    uint64_t enq;                // monotonic cycle when enqueued (stale-drop reference)
    uint64_t due;                // monotonic event deadline
    uint8_t  used;               // delayed-slot occupancy flag
} MdiRec;

typedef struct MdiQueue {
    MdiRec  pending[MDI_PENDING_N];   // FIFO ring of ready records
    uint8_t p_head, p_tail;           // FIFO cursors (count = (tail - head) mod N)
    MdiRec  delayed[MDI_DELAYED_N];   // time-ordered slots (used-flag array)
} MdiQueue;

// Append a ready record to the FIFO. Silently drops on overflow, as a real queue would.
void mdi_queue_enqueue(MdiQueue* q, uint64_t now,
                       uint8_t op, const uint8_t* payload, uint8_t len);

// Schedule a record to enter the FIFO at `due`. Falls back to immediate enqueue when the
// delayed queue is full (drop the delay rather than the record).
void mdi_queue_enqueue_at(MdiQueue* q, uint64_t now, uint64_t due,
                          uint8_t op, const uint8_t* payload, uint8_t len);

// PumpDelayedMdiRcv: matured delayed records move into the FIFO.
void mdi_queue_pump_delayed(MdiQueue* q, uint64_t now);

// ExpireStale: drop FIFO records that have waited unposted longer than `ttl` cycles.
void mdi_queue_expire_stale(MdiQueue* q, uint64_t now, uint64_t ttl);

// The record at the FIFO head, or NULL when empty. For engines that want to log what is
// about to go out; the pump re-reads it, so this is purely observational.
const MdiRec* mdi_queue_peek(const MdiQueue* q);

// PumpMdiRcv: post ONE record from the FIFO head into the hardware ring, raise FIQ0 and
// re-pace the idle heartbeat by `ka_cyc`. Returns 1 if a record was posted, 0 if the FIFO
// was empty or the ring had no room (caller retries on a later tick).
int mdi_queue_post(struct Mad2* m, MdiQueue* q, uint64_t ka_cyc);

#endif // MAD2_MDI_QUEUE_H
