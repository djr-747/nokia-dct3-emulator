// mad2_dsp_rom4 — the network-capable ROM-4 DSP engine (5110/6110/3210/8810 + variants).
//
// Structure: the boot/codeblock/self-test/keep-alive transport skeleton is shared with the
// ROM-6 engine (dsp_rom6.c, jmacato's contributed reference engine — this file began as a
// copy of it). The radio/registration peer is a faithful C port of the ROM-4 protocol
// recovered and verified by **Gareth Davidson (bitplane)** in
// github.com/bitplane/nokia-dct3-re — see the credit block in dsp_rom4.c.
#ifndef MAD2_DSP_ROM4_H
#define MAD2_DSP_ROM4_H

#include <stdint.h>

struct Mad2;
struct DspOps;

// Top bring-up phases — mirror the real DSP boot order.
enum {
    ROM4_BOOT = 0,     // power-on: status/version handshake + code-block upload
    ROM4_UPLOAD,       // streaming code blocks (cb req/reply pump)
    ROM4_READY,        // boot-ready latched; steady state
};

// Radio-peer transaction phases — 1:1 with the recovered ROM-4 registration state machine
// (bitplane/nokia-dct3-re driver/nokia_radio_peer.cpp; 3210-verified through Location
// Updating Accept and steady camp). State numbers are implementation details; packet type,
// request correlation and observable firmware transitions are the contract.
enum {
    ROM4_RP_INACTIVE = 0,
    ROM4_RP_INITIAL_SEARCH,
    ROM4_RP_POST_DEACTIVATE_SEARCH,
    ROM4_RP_CANDIDATE_MEASUREMENT,
    ROM4_RP_CANDIDATE_SYNC,
    ROM4_RP_CANDIDATE_CHANNEL_CHANGE,
    ROM4_RP_CANDIDATE_RA_INFO,
    ROM4_RP_SERVING_BCCH,
    ROM4_RP_SERVING_IDLE_RA,
    ROM4_RP_CANDIDATE_RETRY,
    ROM4_RP_SELECTED_SEARCH,
    ROM4_RP_SERVING_CHANNEL_CHANGE,
    ROM4_RP_SELECTED_CHANNEL_CHANGE,
    ROM4_RP_SELECTED_BCCH,
    ROM4_RP_SELECTED_RA_INFO,
    ROM4_RP_SELECTED_BCCH_CHANNEL_CHANGE,
    ROM4_RP_RANDOM_ACCESS,
    ROM4_RP_ASSIGNED_CHANNEL_CHANGE,
    ROM4_RP_LAPDM_ESTABLISH,
    ROM4_RP_CONTENTION_RESOLUTION,
    ROM4_RP_LOCATION_UPDATE_ACCEPT,
    ROM4_RP_RR_CHANNEL_RELEASE,
    ROM4_RP_RELEASE_DECONFIGURE,
    ROM4_RP_RELEASE_CHANNEL_CHANGE,
    ROM4_RP_COUNT,
};

// One queued MDIRCV (d2m) record awaiting egress into the single hardware ring.
#define ROM4_RCVMAX     168u   // max d2m payload bytes (0x8B ALL_RSSI_RESULTS = 166)
#define ROM4_PENDING_N  48u    // FIFO of ready records
#define ROM4_DELAYED_N  48u    // time-ordered delayed queue
typedef struct Rom4MdiRec {
    uint8_t  op;                  // MDI opcode (frame-word LOW byte)
    uint8_t  len;                 // payload byte count (frame-word HIGH byte)
    uint8_t  bytes[ROM4_RCVMAX];  // payload (what follows the {len,op} word)
    uint64_t enq;                 // monotonic cycle when enqueued (stale-drop reference)
    uint64_t due;                 // monotonic event deadline
    uint8_t  used;                // delayed-slot occupancy flag
} Rom4MdiRec;

// Engine state machine — held by value in struct Mad2, zeroed by the core memset.
typedef struct Rom4Dsp {
    uint8_t  phase;              // ROM4_BOOT .. ROM4_READY (diagnostic)
    uint64_t currentCycles;      // last cycle delivered by DspOps.advance_to/sync_cycle

    // --- Radio-peer transaction state (bitplane's recovered contract, ported 1:1) ---
    uint8_t  radioPhase;         // ROM4_RP_*
    uint8_t  reportsRemaining;   // reports left in the current correlated transaction
    uint8_t  searchMode;         // SEARCH_LIST control byte (payload[0])
    uint8_t  searchRound;        // ALL_RSSI emission counter (drives the RSSI history)
    uint8_t  searchHasArfcn1;    // SEARCH_LIST 512-bit ARFCN set includes ARFCN 1
    uint8_t  searchRequested;    // a newer SEARCH_LIST is queued behind a DEACTIVATE
    uint8_t  selectedReportsRemaining; // suspended selected-search report count
    uint8_t  accessRa;           // CHANNEL REQUEST random-access octet from IDLE_RA form 0
    uint32_t accessFrame;        // frame number reported by 0x84 RA_INFO / echoed by the IA
    uint8_t  contentionL3[64];   // SABM information field (the LU Request) for the UA echo
    uint8_t  contentionLen;
    uint64_t nextReportCycle;    // pacing deadline for the next peer report (0 = emit now)

    // --- SIML (SIM local-security) responder sub-state ---
    // NOT part of bitplane's radio contract: his 3210 lab network used the exempt test PLMN
    // 001-01, so the ROM never ran its local-security startup check and the RE never needed
    // this. Real ROM-4 retail images DO run it, so this responder (shared byte-for-byte with
    // the rom6 engine) is required for a real SIM to be accepted — poke-free: the firmware's
    // own 0x287BE8 table fill accepts from the 0x34/0x35/0x36 records we synthesize.
    uint8_t  siml_want;          // pending d2m replies: bit0=0x34, bit1=0x35, bit2=0x36
    uint8_t  siml_blkidx;        // decoded-record index emitted so far
    uint8_t  siml_msid[13];      // MSID captured from the firmware's 0x70/0x13 request
    uint8_t  siml_block[24];     // security block captured from 0x70/0x16 (echoed as region B)

    // --- MDIRCV egress software queues ---
    Rom4MdiRec pending[ROM4_PENDING_N]; // FIFO ring of ready records
    uint8_t    p_head, p_tail;          // FIFO cursors (count = (tail-head) mod N)
    Rom4MdiRec delayed[ROM4_DELAYED_N]; // time-ordered slots (used-flag array)
} Rom4Dsp;

// Single build/destroy point — called from the mad2 boot/reset path.
void rom4_reset(struct Mad2* m);

// The engine vtable (src/mad2/dsp/dsp_rom4.c) — the ROM-4 profiles' .dsp.
extern const struct DspOps mad2_dsp_rom4;

#endif // MAD2_DSP_ROM4_H
