// Minimal PPP responder — the network side of the WAP-over-CSD dial-up link.
//
// After CONNECT the 3410's browser runs PPP over the RLP/L2R byte pipe (its
// first act, measured: LCP Configure-Request with ACCM=0 + PFC + ACFC, HDLC
// framing 7E..7E with 7D escaping, FCS-16 low-byte-first over FF 03 <proto>
// <payload> — validated byte-for-byte against a live capture). This peer
// negotiates LCP, acks PAP, assigns an IP over IPCP, and hands completed IP
// datagrams up — enough to carry the browser to its WAP gateway traffic.
#ifndef MAD2_DSP_PPP_H
#define MAD2_DSP_PPP_H

#include <stdint.h>
#include "mad2/dsp/wap.h"

// Must hold the largest datagram either side sends: a WSP reply carrying a
// compiled deck (WAP_MAX_DGRAM) plus PPP framing. Undersizing this drops big
// frames silently in ppp_send, which looks exactly like the phone ignoring a
// perfectly good reply.
#define PPP_MRU    (WAP_MAX_DGRAM + 64u)
#define PPP_TXQ    8192u

typedef struct PppPeer {
    // HDLC deframer
    uint8_t  rx[PPP_MRU];
    uint16_t rxLen;
    uint8_t  esc;               // 0x7D seen, XOR next byte with 0x20
    uint8_t  overrun;           // frame too big: discard until next flag
    // negotiation state
    uint8_t  ourLcpSent;        // our LCP Configure-Request is out
    uint8_t  lcpOpen;           // both directions acked
    uint8_t  theirLcpAcked;     // we acked their Configure-Request
    uint8_t  ourIpcpSent;
    uint8_t  ipcpOpen;
    uint8_t  reqId;             // id for requests we originate
    // response byte stream (already HDLC-framed), drained into the RLP tx queue
    uint8_t  tx[PPP_TXQ];
    uint16_t txLen;
    // the WAP gateway serving the phone's WSP traffic once IPCP is open
    WapGw    wap;
} PppPeer;

void ppp_reset(PppPeer* p);

// Feed de-L2R'd characters from the MS. Complete frames are handled internally;
// responses accumulate in the tx stream. log != 0 -> narrate under [rom6].
void ppp_rx_bytes(PppPeer* p, const uint8_t* b, unsigned n, int log);

// Drain up to max bytes of the pending response stream. Returns bytes taken.
unsigned ppp_tx_take(PppPeer* p, uint8_t* out, unsigned max);

#endif
