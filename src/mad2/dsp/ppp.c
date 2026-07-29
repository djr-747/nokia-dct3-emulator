// Minimal PPP responder — see ppp.h. RFC 1661/1662 (LCP + HDLC-like framing),
// RFC 1334 (PAP), RFC 1332 (IPCP), trimmed to what the 3410's client needs.
#include "mad2/dsp/ppp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PPP_FLAG 0x7Eu
#define PPP_ESC  0x7Du

#define PROTO_LCP  0xC021u
#define PROTO_PAP  0xC023u
#define PROTO_IPCP 0x8021u
#define PROTO_IP   0x0021u

// LCP/IPCP/PAP code points
#define CODE_CONF_REQ 1u
#define CODE_CONF_ACK 2u
#define CODE_CONF_NAK 3u
#define CODE_CONF_REJ 4u
#define CODE_TERM_REQ 5u
#define CODE_TERM_ACK 6u
#define CODE_PROT_REJ 8u
#define CODE_ECHO_REQ 9u
#define CODE_ECHO_REP 10u

// Our end of the point-to-point link and the address we hand the phone.
static const uint8_t PPP_IP_OURS[4]   = { 10, 6, 6, 1 };
static const uint8_t PPP_IP_THEIRS[4] = { 10, 6, 6, 2 };

// FCS-16, RFC 1662 appendix: reflected 0x1021 (0x8408), init 0xFFFF, ones
// complement, transmitted low byte first. Validated against a live capture
// of the 3410's own LCP frame.
static uint16_t ppp_fcs16(const uint8_t* p, unsigned n) {
    uint16_t f = 0xFFFFu;
    for (unsigned i = 0; i < n; i++) {
        f ^= p[i];
        for (int b = 0; b < 8; b++) f = (f & 1u) ? (uint16_t)((f >> 1) ^ 0x8408u) : (uint16_t)(f >> 1);
    }
    return (uint16_t)~f;
}

void ppp_reset(PppPeer* p) {
    // The host bridge is armed once, before the call; a per-call reset must not
    // clear it, and the memset here would.
    int bridge = wap_host_bridge(&p->wap);
    memset(p, 0, sizeof *p);
    wap_reset(&p->wap);
    wap_set_host_bridge(&p->wap, bridge);
}

static void ppp_tx_byte_escaped(PppPeer* p, uint8_t b) {
    if (p->txLen + 2u > PPP_TXQ) return;                    // drop: peer will retransmit
    // Pre-negotiation default ACCM escapes all of 0x00-0x1F; staying with it
    // after LCP asked for ACCM=0 is always legal (RFC 1662 §7.1).
    if (b == PPP_FLAG || b == PPP_ESC || b < 0x20u) {
        p->tx[p->txLen++] = PPP_ESC;
        p->tx[p->txLen++] = (uint8_t)(b ^ 0x20u);
    } else {
        p->tx[p->txLen++] = b;
    }
}

// Emit one full frame: flag, escaped(FF 03 proto payload fcs), flag.
static void ppp_send(PppPeer* p, uint16_t proto, const uint8_t* pay, unsigned n) {
    uint8_t raw[PPP_MRU];
    if (n + 4u > sizeof raw) {
        if (getenv("ROM6_LOG"))
            fprintf(stderr, "[rom6] PPP: frame too big to send (%u bytes, MRU %u) — dropped\n",
                    n, (unsigned)PPP_MRU);
        return;
    }
    raw[0] = 0xFF; raw[1] = 0x03;
    raw[2] = (uint8_t)(proto >> 8); raw[3] = (uint8_t)proto;
    memcpy(raw + 4, pay, n);
    uint16_t fcs = ppp_fcs16(raw, n + 4u);
    if (p->txLen < PPP_TXQ) p->tx[p->txLen++] = PPP_FLAG;
    for (unsigned i = 0; i < n + 4u; i++) ppp_tx_byte_escaped(p, raw[i]);
    ppp_tx_byte_escaped(p, (uint8_t)(fcs & 0xFFu));
    ppp_tx_byte_escaped(p, (uint8_t)(fcs >> 8));
    if (p->txLen < PPP_TXQ) p->tx[p->txLen++] = PPP_FLAG;
}

// Build a control packet (code, id, options) and send it under proto.
static void ppp_send_cp(PppPeer* p, uint16_t proto, uint8_t code, uint8_t id,
                        const uint8_t* opts, unsigned optLen) {
    uint8_t cp[PPP_MRU];
    unsigned len = 4u + optLen;
    if (len > sizeof cp) return;
    cp[0] = code; cp[1] = id;
    cp[2] = (uint8_t)(len >> 8); cp[3] = (uint8_t)len;
    if (optLen) memcpy(cp + 4, opts, optLen);
    ppp_send(p, proto, cp, len);
}

static void ppp_handle_lcp(PppPeer* p, const uint8_t* cp, unsigned n, int log) {
    if (n < 4u) return;
    uint8_t code = cp[0], id = cp[1];
    unsigned len = ((unsigned)cp[2] << 8) | cp[3];
    if (len > n) return;
    switch (code) {
    case CODE_CONF_REQ:
        // Accept whatever the client asks (measured: ACCM=0, PFC, ACFC — all
        // fine for us since we keep escaping conservatively and always send
        // full address/control/protocol fields).
        ppp_send_cp(p, PROTO_LCP, CODE_CONF_ACK, id, cp + 4, len - 4u);
        p->theirLcpAcked = 1;
        if (log) fprintf(stderr, "[rom6] PPP: LCP Configure-Request id=%u -> Ack\n", id);
        if (!p->ourLcpSent) {                     // our side asks for nothing
            p->ourLcpSent = 1;
            ppp_send_cp(p, PROTO_LCP, CODE_CONF_REQ, ++p->reqId, 0, 0);
            if (log) fprintf(stderr, "[rom6] PPP: our LCP Configure-Request id=%u (no options)\n", p->reqId);
        }
        break;
    case CODE_CONF_ACK:
        p->lcpOpen = 1;
        if (log) fprintf(stderr, "[rom6] PPP: LCP open (our request id=%u acked)\n", id);
        break;
    case CODE_CONF_NAK: case CODE_CONF_REJ:
        // We requested nothing, so a Nak/Rej can only be a client quirk; retry plain.
        ppp_send_cp(p, PROTO_LCP, CODE_CONF_REQ, ++p->reqId, 0, 0);
        break;
    case CODE_ECHO_REQ: {
        uint8_t magic[4] = {0,0,0,0};
        ppp_send_cp(p, PROTO_LCP, CODE_ECHO_REP, id, magic, 4);
        break;
    }
    case CODE_TERM_REQ:
        ppp_send_cp(p, PROTO_LCP, CODE_TERM_ACK, id, 0, 0);
        if (log) fprintf(stderr, "[rom6] PPP: LCP Terminate-Request -> Ack\n");
        break;
    default:
        break;
    }
}

static void ppp_handle_pap(PppPeer* p, const uint8_t* cp, unsigned n, int log) {
    if (n < 4u || cp[0] != 1u) return;            // Authenticate-Request
    uint8_t ok[1] = { 0 };                        // empty message
    ppp_send_cp(p, PROTO_PAP, 2u, cp[1], ok, 1);  // Authenticate-Ack
    if (log) fprintf(stderr, "[rom6] PPP: PAP Authenticate-Request -> Ack\n");
}

static void ppp_handle_ipcp(PppPeer* p, const uint8_t* cp, unsigned n, int log) {
    if (n < 4u) return;
    uint8_t code = cp[0], id = cp[1];
    unsigned len = ((unsigned)cp[2] << 8) | cp[3];
    if (len > n || len < 4u) return;
    switch (code) {
    case CODE_CONF_REQ: {
        // Walk options: reject anything but IP-Address (3); Nak a zero address.
        uint8_t rej[64]; unsigned rejLen = 0;
        uint8_t ipZero = 0, ipOk = 0;
        for (unsigned i = 4; i + 1u < len && i + cp[i + 1] <= len && cp[i + 1] >= 2u; i += cp[i + 1]) {
            if (cp[i] == 3u && cp[i + 1] == 6u) {
                ipZero = !(cp[i+2] | cp[i+3] | cp[i+4] | cp[i+5]);
                ipOk = !ipZero;
            } else if (rejLen + cp[i + 1] <= sizeof rej) {
                memcpy(rej + rejLen, cp + i, cp[i + 1]);
                rejLen += cp[i + 1];
            }
        }
        if (rejLen) {
            ppp_send_cp(p, PROTO_IPCP, CODE_CONF_REJ, id, rej, rejLen);
            if (log) fprintf(stderr, "[rom6] PPP: IPCP Configure-Request id=%u -> Reject (%u option bytes)\n", id, rejLen);
        } else if (ipZero) {
            uint8_t nak[6] = { 3, 6, PPP_IP_THEIRS[0], PPP_IP_THEIRS[1], PPP_IP_THEIRS[2], PPP_IP_THEIRS[3] };
            ppp_send_cp(p, PROTO_IPCP, CODE_CONF_NAK, id, nak, 6);
            if (log) fprintf(stderr, "[rom6] PPP: IPCP -> Nak, offering 10.6.6.2\n");
        } else {
            ppp_send_cp(p, PROTO_IPCP, CODE_CONF_ACK, id, cp + 4, len - 4u);
            if (log) fprintf(stderr, "[rom6] PPP: IPCP Configure-Request id=%u -> Ack%s\n", id, ipOk ? " (address accepted)" : "");
        }
        if (!p->ourIpcpSent) {
            uint8_t opt[6] = { 3, 6, PPP_IP_OURS[0], PPP_IP_OURS[1], PPP_IP_OURS[2], PPP_IP_OURS[3] };
            p->ourIpcpSent = 1;
            ppp_send_cp(p, PROTO_IPCP, CODE_CONF_REQ, ++p->reqId, opt, 6);
            if (log) fprintf(stderr, "[rom6] PPP: our IPCP Configure-Request (10.6.6.1)\n");
        }
        break;
    }
    case CODE_CONF_ACK:
        p->ipcpOpen = 1;
        if (log) fprintf(stderr, "[rom6] PPP: IPCP open — IP layer up (phone 10.6.6.2, us 10.6.6.1)\n");
        break;
    default:
        break;
    }
}

static void ppp_handle_frame(PppPeer* p, const uint8_t* f, unsigned n, int log) {
    if (n < 4u) return;                           // shorter than proto+fcs
    if (ppp_fcs16(f, n) != (uint16_t)~0xF0B8u) {  // RFC 1662 good-FCS residue (we return ~register)
        if (log) fprintf(stderr, "[rom6] PPP: bad FCS on %u-byte frame — dropped\n", n);
        return;
    }
    n -= 2u;                                      // strip FCS
    unsigned i = 0;
    if (n >= 2u && f[0] == 0xFFu && f[1] == 0x03u) i = 2;      // ACFC: fields optional
    if (i >= n) return;
    uint16_t proto;
    if (f[i] & 1u) proto = f[i], i += 1;                        // PFC: 1-byte protocol
    else { if (i + 1u >= n) return; proto = (uint16_t)((f[i] << 8) | f[i + 1]); i += 2; }
    const uint8_t* pay = f + i;
    unsigned payLen = n - i;
    switch (proto) {
    case PROTO_LCP:  ppp_handle_lcp(p, pay, payLen, log); break;
    case PROTO_PAP:  ppp_handle_pap(p, pay, payLen, log); break;
    case PROTO_IPCP: ppp_handle_ipcp(p, pay, payLen, log); break;
    case PROTO_IP: {
        if (log) {
            fprintf(stderr, "[rom6] PPP: IP datagram, %u bytes", payLen);
            if (payLen >= 20u) {
                fprintf(stderr, " %u.%u.%u.%u -> %u.%u.%u.%u proto=%u",
                        pay[12], pay[13], pay[14], pay[15],
                        pay[16], pay[17], pay[18], pay[19], pay[9]);
                unsigned ihl = (pay[0] & 0x0Fu) * 4u;
                if (pay[9] == 17u && payLen >= ihl + 8u)      // UDP
                    fprintf(stderr, " udp %u -> %u len=%u",
                            (pay[ihl] << 8) | pay[ihl + 1],
                            (pay[ihl + 2] << 8) | pay[ihl + 3],
                            (pay[ihl + 4] << 8) | pay[ihl + 5]);
            }
            fprintf(stderr, "  data=");
            for (unsigned i = 0; i < payLen && i < 48u; ++i) fprintf(stderr, "%02X", pay[i]);
            fprintf(stderr, "\n");
        }
        uint8_t rep[WAP_MAX_DGRAM];
        unsigned rn = wap_handle_ip(&p->wap, pay, payLen, rep, sizeof rep, log);
        if (rn) ppp_send(p, PROTO_IP, rep, rn);
        break;
    }
    default: {
        // LCP Protocol-Reject so the client stops offering e.g. CCP.
        uint8_t body[64];
        unsigned c = payLen > 60u ? 60u : payLen;
        body[0] = (uint8_t)(proto >> 8); body[1] = (uint8_t)proto;
        memcpy(body + 2, pay, c);
        ppp_send_cp(p, PROTO_LCP, CODE_PROT_REJ, ++p->reqId, body, c + 2u);
        if (log) fprintf(stderr, "[rom6] PPP: protocol 0x%04X -> Protocol-Reject\n", proto);
        break;
    }
    }
}

void ppp_rx_bytes(PppPeer* p, const uint8_t* b, unsigned n, int log) {
    for (unsigned i = 0; i < n; i++) {
        uint8_t c = b[i];
        if (c == PPP_FLAG) {
            if (p->rxLen && !p->overrun) ppp_handle_frame(p, p->rx, p->rxLen, log);
            p->rxLen = 0; p->esc = 0; p->overrun = 0;
            continue;
        }
        if (c == PPP_ESC) { p->esc = 1; continue; }
        if (p->esc) { c ^= 0x20u; p->esc = 0; }
        if (p->rxLen < sizeof p->rx) p->rx[p->rxLen++] = c;
        else p->overrun = 1;
    }
}

unsigned ppp_tx_take(PppPeer* p, uint8_t* out, unsigned max) {
    // A host-bridged fetch completes asynchronously; its datagram joins the
    // stream here, on the next drain after the host delivered it.
    uint8_t dg[WAP_MAX_DGRAM];
    unsigned dn = wap_take_out(&p->wap, dg, sizeof dg);
    if (dn) ppp_send(p, PROTO_IP, dg, dn);
    unsigned n = p->txLen < max ? p->txLen : max;
    if (!n) return 0;
    memcpy(out, p->tx, n);
    memmove(p->tx, p->tx + n, p->txLen - n);
    p->txLen = (uint16_t)(p->txLen - n);
    return n;
}
