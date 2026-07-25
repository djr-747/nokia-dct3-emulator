// mad2_dsp_rom6 — clean-room C rebuild of jmacato's reference DCT3 phone-side DSP.
//
// Independent C re-implementation (no C# copied) of the reference GSM-network / DCT3 DSP
// published by github.com/jmacato, studied clean-room. The protocol ground-truth and the
// real-frame-number scheduling model are theirs; none of that project's source is used here.
//
// Selected with ROM6NEW_REF=1 -> m->dsp_override = &mad2_dsp_rom6. STAGE 1 goal: drive the
// 3310 v5.79 firmware to ACQUIRE + SELECT our synthetic cell so it CAMPS (MM leaves state 10).
// Registration / LAPDm / L3 is STAGE 2 (stubbed — search for "STAGE 2" in the .c).
//
// Design: the boot/codeblock/SIML/self-test/keep-alive blocks are lifted VERBATIM (behaviour)
// from the retired rom6_new scaffold — those already worked; they were not redesigned. The NEW parts are the
// reference responder (HandleMdiPacket dispatch + exact builder bytes) and the real-FN
// scheduler (AdvanceTo), which post d2m with COHERENT GSM frame numbers so the firmware's
// cell-acquisition state machine finally locks a serving cell. State lives in Mad2.rom6
// (by value; the core memset builds/destroys it per boot).
#ifndef MAD2_DSP_ROM6_H
#define MAD2_DSP_ROM6_H

#include <stdint.h>

struct Mad2;
struct DspOps;

// Top bring-up phases — mirror the real DSP boot order.
enum {
    ROM6_BOOT = 0,   // power-on: status/version handshake + code-block upload
    ROM6_UPLOAD,     // streaming code blocks (cb req/reply pump)
    ROM6_READY,      // boot-ready latched; steady state
};

// One queued MDIRCV (d2m) record awaiting egress into the single hardware ring. The reference
// posts ONE record at a time (only when the ring is empty), fronted by two software queues:
// a FIFO of ready records and a time-ordered delayed queue.
#define ROM6_RCVMAX     168u   // max d2m payload bytes (0x8B ALL_RSSI = 162)
#define ROM6_PENDING_N  48u    // FIFO of ready records
#define ROM6_DELAYED_N  48u    // time-ordered (EnqueueMdiRcvAfter)
#define ROM6_DEDICATED_N 32u   // LAPDm frames awaiting a scheduled SDCCH downlink slot
#define ROM6_LAPDM_SAPIS 8u
#define ROM6_LAPDM_ACK_N 16u
#define ROM6_L3_MAX      256u
#define ROM6_INCOMING_N  8u
typedef struct Rom6MdiRec {
    uint8_t  op;                 // MDI opcode (frame-word LOW byte)
    uint8_t  len;                // payload byte count (frame-word HIGH byte)
    uint8_t  bytes[ROM6_RCVMAX];// payload (what follows the {len,op} word)
    uint64_t enq;                // monotonic cycle when enqueued (stale-drop reference)
    uint64_t due;                // monotonic event deadline
    uint8_t  used;               // delayed-slot occupancy flag
} Rom6MdiRec;

typedef struct Rom6LapdmLink {
    uint8_t vs;                  // V(S): next downlink I-frame send sequence
    uint8_t vr;                  // V(R): next expected uplink I-frame send sequence
    uint8_t ack_rs[ROM6_LAPDM_ACK_N];
    uint8_t ack_kind[ROM6_LAPDM_ACK_N];
    uint8_t ack_head, ack_count;
    uint8_t seg[ROM6_L3_MAX];  // uplink L3 reassembly (M-bit segmentation)
    uint16_t seg_len;
    uint8_t pending_ua_kind;     // network-originated SABM acknowledgement kind
} Rom6LapdmLink;

typedef struct Rom6IncomingService {
    uint8_t kind;                // 1=MT call, 2=MT SMS
    char address[21];
    char text[121];
} Rom6IncomingService;

// Engine state machine — held by value in struct Mad2, zeroed by the core memset.
typedef struct Rom6Dsp {
    uint8_t  phase;              // ROM6_BOOT .. ROM6_READY (diagnostic)
    uint64_t currentCycles;      // last cycle delivered by DspOps.advance_to
    // --- Serving / measurement / CCCH capture (spec §9) ---
    uint16_t servingArfcn;       // selected serving-cell ARFCN (0 = none selected yet)
    uint8_t  servingBsic;        // serving-cell BSIC
    uint16_t measurementArfcn;   // last measured carrier (0x0F/0x46/0x02 capture)
    uint16_t ccchArfcn;          // ARFCN of the configured CCCH (0x02 logch 0x60)
    uint8_t  ccchBsic;           // BSIC of the configured CCCH
    uint8_t  ccchConfigured;     // 1 once the firmware has configured CCCH (logch 0x60)
    uint8_t  noPswSent;          // 1 once 0x8f NO_PSW_LEFT (search-phase terminator) has been sent
    uint64_t nextBcchBroadcastCycle; // next monotonic BCCH event; UINT64_MAX = disarmed
    uint64_t nextRachTxCycle;    // scheduled handset RACH transmit event
    uint32_t rachRequestFn;      // reduced FN carried by the 0x0C request
    uint8_t  rachReference;      // RA value carried by the 0x0C request

    // --- Dedicated channel (STAGE 2 — SDCCH capture + 51-frame downlink/block-req cadence) ---
    uint16_t dedicatedArfcn;     // dedicated (SDCCH) ARFCN captured from 0x02 logch 0x80
    uint8_t  dedicatedBsic;      // dedicated-cell BSIC
    uint8_t  dedicatedConfigured;// 1 once a dedicated channel is configured (STAGE 2)
    uint8_t  dedicatedReleasePending; // DISC seen: drop the SDCCH once the FIFO drains
    uint64_t nextDedicatedCycle; // next monotonic dedicated DOWNLINK FILL event
    uint64_t nextDedicatedBlkReqCycle; // next monotonic 0x86 BLOCK-REQUEST event
    uint8_t  suppressImsiPaging; // post-registration: page-fill instead of IMSI paging (STAGE 2)
    uint64_t nextIncomingPagingCycle; // identity page at the IMSI paging-group slot
    uint8_t  incomingPagingActive;
    uint8_t  incomingPagingAnswered;
    uint8_t  incomingPagingBursts;
    uint8_t  dedicatedFrames[ROM6_DEDICATED_N][23];
    uint8_t  dedicatedHead, dedicatedTail;

    // --- LAPDm + GSM 04.08 network state (SAPI 0 call control, SAPI 3 SMS) ---
    Rom6LapdmLink lapdm[ROM6_LAPDM_SAPIS];
    uint8_t  regState;           // registration/MM connection FSM
    uint8_t  activeService;      // current CM service type
    uint8_t  nextSmsReference;
    uint8_t  activeIncomingKind;
    char     activeIncomingAddress[21];
    char     activeIncomingText[121];
    Rom6IncomingService incoming[ROM6_INCOMING_N];
    uint8_t  incomingHead, incomingTail;

    // Host-visible network result state. This deliberately records protocol events;
    // it never pokes firmware UI/MM state.
    uint8_t  callState;          // 0=idle, 1=alerting, 2=connected, 3=released
    uint8_t  callDirection;      // 0=none, 1=MO, 2=MT
    uint32_t outgoingCallCount;
    uint32_t outgoingSmsCount;
    char     remoteNumber[21];
    char     lastSmsText[161];

    // --- Paging group (derived from the active EF_IMSI, BS_PA_MFRMS=2) ---
    uint8_t  pagingPhase;        // multiframe phase selected by IMSI mod 1000
    uint8_t  pagingOffset;       // CCCH block selected by IMSI mod 1000
    uint8_t  pagingMobid[8];     // encoded IMSI mobile identity for Paging Request 1

    // --- SIML (local-security) responder sub-state (lifted from the retired scaffold) ---
    uint8_t  siml_want;          // pending d2m replies: bit0=0x34, bit1=0x35, bit2=0x36
    uint8_t  siml_blkidx;        // decoded-record index emitted so far
    uint8_t  siml_msid[13];      // MSID captured from the firmware's 0x70/0x13 request
    uint8_t  siml_block[24];     // SIML ciphertext block captured from 0x70/0x16 (echoed region B)

    // --- MDIRCV egress software queues ---
    Rom6MdiRec pending[ROM6_PENDING_N]; // FIFO ring of ready records
    uint8_t   p_head, p_tail;            // FIFO cursors (count = (tail-head) mod N)
    Rom6MdiRec delayed[ROM6_DELAYED_N]; // time-ordered slots (used-flag array)
} Rom6Dsp;

// Single build/destroy point — called from the mad2 boot/reset path.
void rom6_reset(struct Mad2* m);
int rom6_queue_incoming_call(struct Mad2* m, const char* calling_number);
int rom6_queue_incoming_sms(struct Mad2* m, const char* originator, const char* text);

// The engine vtable (src/mad2/dsp/dsp_rom6.c). ROM6NEW_REF-gated; default off.
extern const struct DspOps mad2_dsp_rom6;

#endif // MAD2_DSP_ROM6_H
