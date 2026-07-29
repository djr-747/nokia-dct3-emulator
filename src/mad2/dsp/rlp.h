// GSM 04.22 / 3GPP TS 24.022 RLP peer — the emulator side of the 3410's
// non-transparent CSD data link.
//
// On a real DCT3 the DSP owns RLP framing (no CRC-24 table exists in any of the
// 71 firmware images in this repo, including the RLP-proven 6210): the MCU hands
// the DSP a 30-byte frame with the 24-bit FCS slot left zero and the DSP fills
// it on air. In this emulator WE are that peer, so no FCS is computed either —
// the MCU never reads it in either direction.
//
// Frame header, 16 bits, stored LITTLE-endian in both MDI directions (measured;
// decoder RLP_FRAME_HDR_DECODE 0x2CAFA4, encoder tail RLP_FRAME_HDR_STORE
// 0x2CAF88 in the 3410 v5.46 image):
//
//   bit0     C/R          (command = 1, response = 0)
//   bits1-2  S            (supervisory function: 0 RR, 1 REJ, 2 RNR, 3 SREJ)
//   bits3-8  N(S)         (63 = U frame, 62 = S frame, else I+S frame)
//   bit9     P/F
//   bits10-15 N(R), or M for U frames (bits10-14)
//
// U-frame M values, confirmed to map 1:1 onto the firmware's link events:
//   0 UI · 3 DM · 7 SABM · 8 DISC · 12 UA(-> event 0x408) · 15 NULL · 23 XID · 28 TEST
//
// RLP version 0/1 throughout: frame size 30, modulus 62, window max 61.
//
// MDI record payload layout around the header:
//   m2d 0x50 (MS -> us):  [0] no-I-field flag  [1] logch  [2..3] header LE  [4..28] I-field
//   d2m 0x9C (us -> MS):  [0] logch  [1] flags (bit0 must be 0)  [2..3] header LE  [4..28] I-field
#ifndef MAD2_DSP_RLP_H
#define MAD2_DSP_RLP_H

#include <stdint.h>

#define RLP_MODULUS      62u
#define RLP_IFIELD_LEN   25u

// U-frame M values (TS 24.022 §5.3.3)
#define RLP_U_UI    0u
#define RLP_U_DM    3u
#define RLP_U_SABM  7u
#define RLP_U_DISC  8u
#define RLP_U_UA    12u
#define RLP_U_NULL  15u
#define RLP_U_XID   23u
#define RLP_U_TEST  28u

typedef struct RlpPeer {
    uint8_t established;              // 0 = ADM, 1 = link up
    uint8_t vr;                       // receive state variable, mod 62
    uint8_t vs;                       // send state variable, mod 62
    uint8_t peerNr;                   // last N(R) the MS acknowledged
    uint8_t uaPending;                // SABM/DISC seen -> answer UA (F=1)
    uint8_t pollPending;              // S/I command with P=1 seen -> respond F=1
    uint8_t xidPending;               // XID command seen -> echo an XID response
    uint8_t testPending;              // TEST command seen -> echo a TEST response
    uint8_t echoIfield[RLP_IFIELD_LEN]; // I-field to echo for XID/TEST responses
    uint8_t l2rStatusSent;            // one-shot: our L2R-COP status I-frame went out
    uint16_t lastTxHdr;               // last downlink header built (for logging)
    // L2R-COP character pipes. rxq: characters extracted from the MS's in-sequence
    // I-frames (PDU = status octets + chars), drained by the PPP layer. txq:
    // characters queued by the PPP layer, chunked into downlink I-frames.
    uint8_t  rxq[512];
    uint16_t rxqLen;
    uint8_t  txq[4096];
    uint16_t txqLen;
} RlpPeer;

static inline uint16_t rlp_hdr(uint8_t cr, uint8_t s, uint8_t ns, uint8_t pf, uint8_t nr_or_m) {
    return (uint16_t)((cr & 1u) | ((s & 3u) << 1) | ((ns & 0x3Fu) << 3) |
                      ((pf & 1u) << 9) | ((uint16_t)(nr_or_m & 0x3Fu) << 10));
}
static inline uint8_t rlp_hdr_cr(uint16_t h) { return (uint8_t)(h & 1u); }
static inline uint8_t rlp_hdr_ns(uint16_t h) { return (uint8_t)((h >> 3) & 0x3Fu); }
static inline uint8_t rlp_hdr_pf(uint16_t h) { return (uint8_t)((h >> 9) & 1u); }
static inline uint8_t rlp_hdr_m(uint16_t h)  { return (uint8_t)((h >> 10) & 0x1Fu); }
static inline uint8_t rlp_hdr_nr(uint16_t h) { return (uint8_t)((h >> 10) & 0x3Fu); }

void rlp_peer_reset(RlpPeer* p);

// Feed one uplink RLP frame (the payload of an m2d 0x50 record, rec[0..len)).
void rlp_peer_on_uplink(RlpPeer* p, const uint8_t* rec, unsigned len);

// Build the next downlink frame into a d2m 0x9C payload: fills pl[2..3] (header,
// LE) and pl[4..] (I-field, if any) of a caller-zeroed 32-byte payload whose
// pl[0]/pl[1] the caller owns. Returns the header for logging.
uint16_t rlp_peer_next_downlink(RlpPeer* p, uint8_t* pl);

// Queue user characters for downlink I-frames / take characters received in
// uplink I-frames (both are the L2R-COP relay payload, e.g. PPP bytes).
void     rlp_peer_queue_tx(RlpPeer* p, const uint8_t* b, unsigned n);
unsigned rlp_peer_take_rx(RlpPeer* p, uint8_t* out, unsigned max);

#endif
