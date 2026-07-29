// Synthetic WAP 1.x gateway — see wap.h.
//
// Layers, outermost first: IP -> UDP -> WTP (WAP-201) -> WSP (WAP-203) -> a
// WBXML-compiled WML 1.1 deck (WAP-192). A WAP 1.x browser never receives text
// WML: the gateway is what compiles it, so the deck below is emitted as WBXML
// with content type application/vnd.wap.wmlc.
#include "mad2/dsp/wap.h"

#include <stdio.h>
#include <string.h>

// ── WTP PDU types (WAP-201 §8.3): octet0 = CON(1) | Type(4) | GTR TTR RID ──
#define WTP_INVOKE 1u
#define WTP_RESULT 2u
#define WTP_ACK    3u
#define WTP_ABORT  4u

// ── WSP PDU types (WAP-203 §8.2) ──
#define WSP_CONNECT       0x01u
#define WSP_CONNECTREPLY  0x02u
#define WSP_REDIRECT      0x03u
#define WSP_REPLY         0x04u
#define WSP_DISCONNECT    0x05u
#define WSP_GET           0x40u
#define WSP_POST          0x60u

#define WSP_STATUS_OK     0x20u   // 200, encoded as WSP status code
#define WSP_CT_WMLC       0x14u   // application/vnd.wap.wmlc, well-known value

// ── The deck, WBXML (WAP-192 §14): WML 1.1, no attributes, no string table ──
//
//   <wml><card><p>...</p></card></wml>
//
// Tag tokens are code page 0 WML 1.1: wml 0x3F, card 0x27, p 0x20, br 0x26.
// 0x40 ORed on a tag means "has content"; 0x01 ends an element; 0x03 is an
// inline NUL-terminated string.
#define WBXML_END   0x01u
#define WBXML_STR_I 0x03u
#define TAG_C(t)    ((uint8_t)((t) | 0x40u))
#define WML_WML     0x3Fu
#define WML_CARD    0x27u
#define WML_P       0x20u
#define WML_BR      0x26u

static unsigned wap_build_deck(uint8_t* d, unsigned max) {
    static const char* const LINES[] = {
        "Hello from the",
        "emulated network.",
        "RLP + PPP + WSP",
        "all the way down.",
    };
    unsigned n = 0;
    if (max < 64u) return 0;
    d[n++] = 0x01;              // WBXML version 1.1
    d[n++] = 0x04;              // public id: -//WAPFORUM//DTD WML 1.1//EN
    d[n++] = 0x6A;              // charset: UTF-8 (MIBenum 106)
    d[n++] = 0x00;              // string table length 0
    d[n++] = TAG_C(WML_WML);
    d[n++] = TAG_C(WML_CARD);
    d[n++] = TAG_C(WML_P);
    for (unsigned i = 0; i < sizeof LINES / sizeof LINES[0]; i++) {
        unsigned l = (unsigned)strlen(LINES[i]);
        if (n + l + 4u >= max) break;
        d[n++] = WBXML_STR_I;
        memcpy(d + n, LINES[i], l); n += l;
        d[n++] = 0x00;
        if (i + 1u < sizeof LINES / sizeof LINES[0]) d[n++] = WML_BR;   // empty element
    }
    d[n++] = WBXML_END;         // </p>
    d[n++] = WBXML_END;         // </card>
    d[n++] = WBXML_END;         // </wml>
    return n;
}

// Multi-byte integer (WAP-203 §8.1.2): 7 bits per octet, high bit = continue.
static unsigned wap_uintvar(uint8_t* d, uint32_t v) {
    uint8_t tmp[5]; unsigned n = 0;
    do { tmp[n++] = (uint8_t)(v & 0x7Fu); v >>= 7; } while (v);
    for (unsigned i = 0; i < n; i++) d[i] = (uint8_t)(tmp[n - 1u - i] | (i + 1u < n ? 0x80u : 0u));
    return n;
}
static uint32_t wap_uintvar_get(const uint8_t* s, unsigned len, unsigned* used) {
    uint32_t v = 0; unsigned i = 0;
    while (i < len && i < 5u) { v = (v << 7) | (s[i] & 0x7Fu); if (!(s[i++] & 0x80u)) break; }
    *used = i;
    return v;
}

static uint16_t wap_cksum(const uint8_t* p, unsigned n, uint32_t seed) {
    uint32_t s = seed;
    for (unsigned i = 0; i + 1u < n; i += 2u) s += ((uint32_t)p[i] << 8) | p[i + 1];
    if (n & 1u) s += (uint32_t)p[n - 1u] << 8;
    while (s >> 16) s = (s & 0xFFFFu) + (s >> 16);
    return (uint16_t)~s;
}

// Wrap a UDP payload in UDP+IP headers, echoing the inbound addresses/ports
// back the way they came.
static unsigned wap_wrap(uint8_t* out, unsigned outMax,
                         const uint8_t* srcIp, const uint8_t* dstIp,
                         uint16_t srcPort, uint16_t dstPort,
                         const uint8_t* pay, unsigned payLen) {
    unsigned total = 28u + payLen;
    if (total > outMax) return 0;
    memset(out, 0, 28u);
    out[0] = 0x45;                                  // IPv4, IHL 5
    out[2] = (uint8_t)(total >> 8); out[3] = (uint8_t)total;
    out[8] = 64;                                    // TTL
    out[9] = 17;                                    // UDP
    memcpy(out + 12, srcIp, 4);
    memcpy(out + 16, dstIp, 4);
    uint16_t hc = wap_cksum(out, 20u, 0);
    out[10] = (uint8_t)(hc >> 8); out[11] = (uint8_t)hc;
    out[20] = (uint8_t)(srcPort >> 8); out[21] = (uint8_t)srcPort;
    out[22] = (uint8_t)(dstPort >> 8); out[23] = (uint8_t)dstPort;
    unsigned ulen = 8u + payLen;
    out[24] = (uint8_t)(ulen >> 8); out[25] = (uint8_t)ulen;
    memcpy(out + 28, pay, payLen);
    // UDP checksum over the pseudo-header + datagram. 0 would be legal ("not
    // computed") but a real gateway computes it, and some stacks drop zero.
    uint32_t seed = 0;
    for (int i = 0; i < 4; i += 2) seed += ((uint32_t)srcIp[i] << 8) | srcIp[i + 1];
    for (int i = 0; i < 4; i += 2) seed += ((uint32_t)dstIp[i] << 8) | dstIp[i + 1];
    seed += 17u; seed += ulen;
    uint16_t uc = wap_cksum(out + 20, ulen, seed);
    out[26] = (uint8_t)(uc >> 8); out[27] = (uint8_t)uc;
    return total;
}

void wap_reset(WapGw* g) {
    int bridge = g->hostBridge;              // survives a per-call reset
    memset(g, 0, sizeof *g);
    g->sessionId = 1;
    g->hostBridge = (uint8_t)bridge;
}

void wap_set_host_bridge(WapGw* g, int mode) {
    g->hostBridge = mode ? 1u : 0u;
    g->relay = (mode == 2) ? 1u : 0u;
}
int  wap_host_bridge(const WapGw* g) { return g->relay ? 2 : (g->hostBridge ? 1 : 0); }

const char* wap_pending_uri(const WapGw* g) {
    return (g->pending && !g->relay) ? g->pendingUri : 0;
}

const uint8_t* wap_pending_relay(const WapGw* g, unsigned* len) {
    if (!g->pending || !g->relay) return 0;
    *len = g->relayReqLen;
    return g->relayReq;
}

static uint8_t* wap_out_slot(WapGw* g) {
    uint8_t next = (uint8_t)((g->outTail + 1u) % WAP_OUTQ);
    if (next == g->outHead) return 0;                  // queue full: drop, peer retransmits
    return g->out[g->outTail];
}
static void wap_out_commit(WapGw* g, unsigned len) {
    if (!len) return;
    g->outLen[g->outTail] = (uint16_t)len;
    g->outTail = (uint8_t)((g->outTail + 1u) % WAP_OUTQ);
}

unsigned wap_take_out(WapGw* g, uint8_t* buf, unsigned max) {
    if (g->outHead == g->outTail) return 0;
    unsigned n = g->outLen[g->outHead];
    if (!n || n > max) { g->outHead = (uint8_t)((g->outHead + 1u) % WAP_OUTQ); return 0; }
    memcpy(buf, g->out[g->outHead], n);
    g->outHead = (uint8_t)((g->outHead + 1u) % WAP_OUTQ);
    return n;
}

// A datagram came back from the real gateway: address it to the phone exactly
// as it addressed us, and queue it. The payload is untouched — WTP/WSP state
// lives end to end between the phone and that gateway, not here.
void wap_relay_deliver(WapGw* g, const uint8_t* pay, unsigned len, int log) {
    g->pending = 0;
    uint8_t* slot = wap_out_slot(g);
    if (!slot || !len) return;
    unsigned n = wap_wrap(slot, WAP_MAX_DGRAM,
                          g->pendingSrcIp, g->pendingDstIp,
                          g->pendingSrcPort, g->pendingDstPort, pay, len);
    wap_out_commit(g, n);
    if (log) fprintf(stderr, "[rom6] WAP: relayed %u bytes back from the gateway\n", len);
}

// Wrap a WSP body in the WTP Result + UDP/IP for the transaction that is
// waiting on it. Shared by the immediate and the deferred (host-fetched) paths.
static void wap_stage_result(WapGw* g, const uint8_t* wsp, unsigned wlen) {
    uint8_t reply[WAP_MAX_DGRAM];
    uint8_t* slot = wap_out_slot(g);
    if (!slot || wlen + 3u > sizeof reply) return;
    reply[0] = (uint8_t)((WTP_RESULT << 3) | 0x02u);
    reply[1] = (uint8_t)((g->pendingTid >> 8) | 0x80u);
    reply[2] = (uint8_t)g->pendingTid;
    memcpy(reply + 3, wsp, wlen);
    wap_out_commit(g, wap_wrap(slot, WAP_MAX_DGRAM,
                               g->pendingSrcIp, g->pendingDstIp,
                               g->pendingSrcPort, g->pendingDstPort,
                               reply, wlen + 3u));
}

void wap_deliver(WapGw* g, uint8_t ct, const uint8_t* body, unsigned len, int log) {
    if (!g->pending) return;
    g->pending = 0;
    uint8_t wsp[WAP_MAX_DGRAM];
    unsigned n = 0;
    if (!len || 4u + len > sizeof wsp) {           // nothing usable came back
        wsp[n++] = WSP_REPLY;
        wsp[n++] = 0x60u;                          // 500 Internal Server Error
        n += wap_uintvar(wsp + n, 1);
        wsp[n++] = (uint8_t)(0x03u | 0x80u);       // text/plain
        if (log) fprintf(stderr, "[rom6] WAP: host fetch failed for \"%s\" -> 500\n", g->pendingUri);
    } else {
        wsp[n++] = WSP_REPLY;
        wsp[n++] = WSP_STATUS_OK;
        n += wap_uintvar(wsp + n, 1);
        wsp[n++] = (uint8_t)(ct | 0x80u);
        memcpy(wsp + n, body, len); n += len;
        g->requests++;
        if (log) fprintf(stderr, "[rom6] WAP: host served \"%s\" — %u bytes, content-type 0x%02X\n",
                         g->pendingUri, len, ct);
    }
    wap_stage_result(g, wsp, n);
}

// Build the WSP payload answering one WSP request. Returns length, 0 = nothing.
static unsigned wap_wsp_reply(WapGw* g, const uint8_t* req, unsigned len,
                              uint8_t* out, unsigned outMax, int log) {
    if (!len) return 0;
    uint8_t pdu = req[0];
    unsigned n = 0;
    if (pdu == WSP_CONNECT) {
        // ConnectReply: session id, then capabilities and headers blocks. We
        // negotiate nothing — an empty capability block means "your proposal
        // stands", which is what a client's own defaults already assume.
        out[n++] = WSP_CONNECTREPLY;
        n += wap_uintvar(out + n, g->sessionId);
        n += wap_uintvar(out + n, 0);              // capabilities length
        n += wap_uintvar(out + n, 0);              // headers length
        g->connected = 1;
        if (log) fprintf(stderr, "[rom6] WAP: WSP Connect -> ConnectReply (session %u)\n", g->sessionId);
        return n;
    }
    if (pdu == WSP_DISCONNECT) {
        g->connected = 0;
        if (log) fprintf(stderr, "[rom6] WAP: WSP Disconnect\n");
        return 0;
    }
    if ((pdu & 0x40u) && pdu != WSP_REPLY) {       // a method PDU: Get/Post/...
        unsigned used = 0;
        uint32_t uriLen = wap_uintvar_get(req + 1, len - 1u, &used);
        const char* uri = (const char*)(req + 1 + used);
        if (uriLen > len - 1u - used) uriLen = len - 1u - used;
        unsigned uriShow = uriLen > 80u ? 80u : uriLen;
        if (g->hostBridge) {
            // Hand it to the host, which can actually reach the network, and
            // hold the transaction until it answers.
            unsigned c = uriLen < WAP_MAX_URI - 1u ? uriLen : WAP_MAX_URI - 1u;
            memcpy(g->pendingUri, uri, c);
            g->pendingUri[c] = 0;
            g->pending = 1;
            if (log) fprintf(stderr, "[rom6] WAP: WSP method 0x%02X uri=\"%.*s\" -> host fetch\n",
                             pdu, (int)uriShow, uri);
            return 0;
        }
        if (log) fprintf(stderr, "[rom6] WAP: WSP method 0x%02X uri=\"%.*s\" -> 200 OK, WML deck\n",
                         pdu, (int)uriShow, uri);
        uint8_t deck[512];
        unsigned dl = wap_build_deck(deck, sizeof deck);
        if (!dl || 4u + dl > outMax) return 0;
        out[n++] = WSP_REPLY;
        out[n++] = WSP_STATUS_OK;
        n += wap_uintvar(out + n, 1);              // headers length: content type only
        out[n++] = (uint8_t)(WSP_CT_WMLC | 0x80u); // well-known content type, short form
        memcpy(out + n, deck, dl); n += dl;
        g->requests++;
        return n;
    }
    if (log) fprintf(stderr, "[rom6] WAP: unhandled WSP PDU 0x%02X (%u bytes)\n", pdu, len);
    return 0;
}

unsigned wap_handle_ip(WapGw* g, const uint8_t* ip, unsigned len,
                       uint8_t* out, unsigned outMax, int log) {
    if (len < 20u || (ip[0] >> 4) != 4u) return 0;
    unsigned ihl = (ip[0] & 0x0Fu) * 4u;
    if (ip[9] != 17u || len < ihl + 8u) return 0;              // UDP only
    const uint8_t* udp = ip + ihl;
    uint16_t sport = (uint16_t)((udp[0] << 8) | udp[1]);
    uint16_t dport = (uint16_t)((udp[2] << 8) | udp[3]);
    unsigned ulen = (unsigned)((udp[4] << 8) | udp[5]);
    if (ulen < 8u || ihl + ulen > len) return 0;
    const uint8_t* w = udp + 8;
    unsigned wlen = ulen - 8u;
    if (dport != 9201u && dport != 9200u) return 0;            // not WSP
    if (!wlen) return 0;

    if (g->relay) {
        // Pure transport: hand the datagram to the host untouched. The phone's
        // WSP session, WTP transactions and WBXML all belong to the real
        // gateway at the far end — we terminate nothing and parse nothing.
        if (wlen > sizeof g->relayReq) return 0;
        memcpy(g->relayReq, w, wlen);
        g->relayReqLen = (uint16_t)wlen;
        memcpy(g->pendingSrcIp, ip + 16, 4);
        memcpy(g->pendingDstIp, ip + 12, 4);
        g->pendingSrcPort = dport; g->pendingDstPort = sport;
        g->pending = 1;
        if (log) fprintf(stderr, "[rom6] WAP: relaying %u bytes to the upstream gateway\n", wlen);
        return 0;
    }

    uint8_t wsp[WAP_MAX_DGRAM], reply[WAP_MAX_DGRAM];
    unsigned rn = 0;

    if (dport == 9200u) {                                      // connectionless WSP
        if (wlen < 2u) return 0;
        unsigned n = wap_wsp_reply(g, w + 1, wlen - 1u, wsp, sizeof wsp, log);
        if (!n) return 0;
        reply[rn++] = w[0];                                    // echo transaction id
        memcpy(reply + rn, wsp, n); rn += n;
    } else {                                                   // WTP
        uint8_t type = (uint8_t)((w[0] >> 3) & 0x0Fu);
        if (type == WTP_ABORT) {
            if (log && wlen >= 4u)
                fprintf(stderr, "[rom6] WAP: WTP Abort tid=0x%04X reason=%u — transaction dropped\n",
                        (w[1] << 8) | w[2], w[3]);
            g->connected = 0;
            return 0;
        }
        if (type == WTP_ACK) return 0;                         // nothing owed
        if (type != WTP_INVOKE || wlen < 4u) return 0;
        uint16_t tid = (uint16_t)((w[1] << 8) | w[2]);
        // Save the transaction context before dispatching: a host-bridged fetch
        // answers later and must address the reply exactly as we would now.
        g->pendingTid = tid;
        memcpy(g->pendingSrcIp, ip + 16, 4);
        memcpy(g->pendingDstIp, ip + 12, 4);
        g->pendingSrcPort = dport; g->pendingDstPort = sport;
        unsigned n = wap_wsp_reply(g, w + 4, wlen - 4u, wsp, sizeof wsp, log);
        if (!n) return 0;
        // Result PDU: CON=0, type=2, TTR=1 (single, complete message). TTR alone
        // marks the last packet; adding GTR to it is a reserved combination.
        // The TID is mirrored with the high bit SET: WTP encodes direction in
        // TID bit 15, so responder-to-initiator PDUs carry tid | 0x8000. A
        // verbatim echo (and a GTR|TTR combo) both draw a Provider Abort
        // PROTOERR from this phone (measured, nav-csd-gw / -gw2).
        reply[rn++] = (uint8_t)((WTP_RESULT << 3) | 0x02u);
        reply[rn++] = (uint8_t)((tid >> 8) | 0x80u);
        reply[rn++] = (uint8_t)tid;
        memcpy(reply + rn, wsp, n); rn += n;
    }

    // Answer to the sender, from the address it addressed — the gateway IP in
    // the phone's profile need not exist anywhere.
    return wap_wrap(out, outMax, ip + 16, ip + 12, dport, sport, reply, rn);
}
