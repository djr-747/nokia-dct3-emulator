// GSM 04.22 / TS 24.022 RLP peer — see rlp.h for the frame layout and the
// measured mapping onto the 3410's MDI records.
//
// This peer is deliberately minimal: it answers the MS's link establishment and
// keeps the established link acknowledged. It does not (yet) retransmit, REJ,
// or checkpoint — the MS side sends in order over a lossless MDI transport, so
// sequence errors cannot occur here.
#include "mad2/dsp/rlp.h"

#include <string.h>

// L2R-COP status octet (TS 07.02) carried as the first I-field byte: bit 0x40
// clear = connection up, bit 0x20 clear = flow allowed, low five bits = 0x1F =
// "empty PDU" address (no characters follow). Measured: the MS's own first
// I-frame carries exactly this octet, and the firmware's L2RCOP status
// processor (0x3044C8) turns the 1->0 edge of bit 0x40 into AT event 40, the
// only producer of "CONNECT <speed>" in the image.
#define RLP_L2R_STATUS_LINK_UP 0x1Fu

void rlp_peer_reset(RlpPeer* p) {
    memset(p, 0, sizeof *p);
}

// L2R-COP PDU (TS 07.02): a sequence of [status octet][characters...]. The
// status octet's low five bits are an address: 31 = nothing follows, 30 = the
// whole rest of the PDU is characters, k < 30 = k characters then another
// status octet. Measured from the MS's own frames: a full PDU is 0x1E + 24
// chars, a partial one is e.g. 0x0D + 13 chars + 0x1F.
static void rlp_l2r_extract(RlpPeer* p, const uint8_t* ifield, unsigned len) {
    unsigned pos = 0;
    while (pos < len) {
        uint8_t addr = ifield[pos++] & 0x1Fu;
        unsigned take;
        if (addr == 31u) break;
        take = (addr == 30u) ? len - pos : addr;
        if (take > len - pos) take = len - pos;
        for (unsigned i = 0; i < take; i++)
            if (p->rxqLen < sizeof p->rxq) p->rxq[p->rxqLen++] = ifield[pos + i];
        pos += take;
        if (addr == 30u) break;
    }
}

void rlp_peer_queue_tx(RlpPeer* p, const uint8_t* b, unsigned n) {
    for (unsigned i = 0; i < n; i++)
        if (p->txqLen < sizeof p->txq) p->txq[p->txqLen++] = b[i];
}

unsigned rlp_peer_take_rx(RlpPeer* p, uint8_t* out, unsigned max) {
    unsigned n = p->rxqLen < max ? p->rxqLen : max;
    if (!n) return 0;
    memcpy(out, p->rxq, n);
    memmove(p->rxq, p->rxq + n, p->rxqLen - n);
    p->rxqLen = (uint16_t)(p->rxqLen - n);
    return n;
}

void rlp_peer_on_uplink(RlpPeer* p, const uint8_t* rec, unsigned len) {
    if (len < 4u) return;
    uint16_t h = (uint16_t)(rec[2] | ((uint16_t)rec[3] << 8));
    uint8_t ns = rlp_hdr_ns(h);
    if (ns == 63u) {                              // U frame
        switch (rlp_hdr_m(h)) {
        case RLP_U_SABM:
            p->established = 1; p->vr = 0; p->vs = 0; p->peerNr = 0;
            p->uaPending = 1; p->pollPending = 0; p->l2rStatusSent = 0;
            break;
        case RLP_U_DISC:
            p->established = 0; p->uaPending = 1;
            break;
        case RLP_U_XID:
            p->xidPending = 1;
            memcpy(p->echoIfield, rec + 4, len >= 4u + RLP_IFIELD_LEN ? RLP_IFIELD_LEN : len - 4u);
            break;
        case RLP_U_TEST:
            p->testPending = 1;
            memcpy(p->echoIfield, rec + 4, len >= 4u + RLP_IFIELD_LEN ? RLP_IFIELD_LEN : len - 4u);
            break;
        default:                                  // NULL / UI / DM / UA: nothing to do
            break;
        }
        return;
    }
    if (!p->established) return;
    p->peerNr = rlp_hdr_nr(h);
    if (rlp_hdr_cr(h) && rlp_hdr_pf(h)) p->pollPending = 1;
    if (ns < RLP_MODULUS && ns == p->vr) {        // in-sequence I frame
        p->vr = (uint8_t)((p->vr + 1u) % RLP_MODULUS);
        unsigned il = len - 4u;
        if (il > RLP_IFIELD_LEN) il = RLP_IFIELD_LEN;
        rlp_l2r_extract(p, rec + 4, il);
    }
}

uint16_t rlp_peer_next_downlink(RlpPeer* p, uint8_t* pl) {
    uint16_t h;
    if (p->uaPending) {
        p->uaPending = 0;
        h = rlp_hdr(0, 0, 63, 1, RLP_U_UA);                       // 0x33F8
    } else if (p->xidPending) {
        p->xidPending = 0;
        h = rlp_hdr(0, 0, 63, 1, RLP_U_XID);                      // 0x5FF8: accept as offered
        memcpy(pl + 4, p->echoIfield, RLP_IFIELD_LEN);
    } else if (p->testPending) {
        p->testPending = 0;
        h = rlp_hdr(0, 0, 63, 1, RLP_U_TEST);
        memcpy(pl + 4, p->echoIfield, RLP_IFIELD_LEN);
    } else if (p->established && !p->l2rStatusSent) {
        // One I-frame announcing "connection up" at the L2R relay layer. This
        // is what lets the AT layer emit CONNECT and open the data path.
        p->l2rStatusSent = 1;
        h = rlp_hdr(1, 0, p->vs, 0, p->vr);
        pl[4] = RLP_L2R_STATUS_LINK_UP;
        p->vs = (uint8_t)((p->vs + 1u) % RLP_MODULUS);
    } else if (p->established && p->pollPending) {                // final: answer the poll
        p->pollPending = 0;
        h = rlp_hdr(0, 0, 62, 1, p->vr);
    } else if (p->established && p->txqLen) {
        // Data I-frame: one L2R-COP PDU of queued characters. Full PDU is
        // status 0x1E ("rest is chars") + 24 chars; a partial one is
        // status k + k chars + 0x1F terminator.
        unsigned k = p->txqLen < RLP_IFIELD_LEN - 1u ? p->txqLen : RLP_IFIELD_LEN - 1u;
        if (k == RLP_IFIELD_LEN - 1u) {
            pl[4] = 0x1Eu;
            memcpy(pl + 5, p->txq, k);
        } else {
            pl[4] = (uint8_t)k;
            memcpy(pl + 5, p->txq, k);
            pl[5 + k] = RLP_L2R_STATUS_LINK_UP;
        }
        memmove(p->txq, p->txq + k, p->txqLen - k);
        p->txqLen = (uint16_t)(p->txqLen - k);
        h = rlp_hdr(1, 0, p->vs, 0, p->vr);
        p->vs = (uint8_t)((p->vs + 1u) % RLP_MODULUS);
    } else if (p->established) {                                  // keep-alive RR command
        h = rlp_hdr(1, 0, 62, 0, p->vr);
    } else {
        h = rlp_hdr(0, 0, 63, 0, RLP_U_NULL);                     // 0x3DF8: ADM idle
    }
    pl[2] = (uint8_t)(h & 0xFFu);
    pl[3] = (uint8_t)(h >> 8);
    p->lastTxHdr = h;
    return h;
}
