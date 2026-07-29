// Synthetic WAP 1.x gateway — the far end of the phone's dial-up data call.
//
// The 3410's browser, once PPP/IPCP is up, sends WTP/WSP datagrams to whatever
// gateway address its WAP profile names (measured: UDP 9201, the connection-
// oriented WSP port). This module is that gateway: it answers the WSP session
// setup and serves content, so the browser has something to render.
//
// Everything is answered in-place: we reply to whatever destination address the
// datagram carried, so no host networking is involved and the profile's gateway
// IP (127.0.0.1 in this phone) does not need to be reachable or even sane.
//
// Content comes from one of two places. By default the gateway serves a small
// built-in WML deck, which keeps the emulator self-contained. If a host bridge
// is attached (the node harness, which unlike this wasm module can open
// sockets), a request instead becomes a PENDING FETCH: the gateway holds the
// WTP transaction, the host fetches and transcodes the real URL, calls
// wap_deliver(), and the reply goes out on the next pump. WTP allows roughly
// three seconds for this (timer tables 0x4C4F7C), which is ample.
#ifndef MAD2_DSP_WAP_H
#define MAD2_DSP_WAP_H

#include <stdint.h>

// One datagram must hold a whole WSP reply: a compiled deck plus WTP/UDP/IP
// headers. Decks are kept well under this by the gateway's own pagination —
// period phones had a hard deck-size ceiling and gateways split pages with an
// explicit "More" link, which is what our host side does.
#define WAP_MAX_DGRAM  2048u
#define WAP_MAX_URI     256u
#define WAP_OUTQ          4u

typedef struct WapGw {
    uint32_t sessionId;
    uint8_t  connected;
    uint32_t requests;            // WSP method requests served

    // The transaction a pending fetch belongs to, saved so the deferred reply
    // can be addressed exactly like an immediate one would have been.
    uint8_t  pending;             // a fetch is outstanding
    char     pendingUri[WAP_MAX_URI];
    uint16_t pendingTid;
    uint8_t  pendingSrcIp[4];     // our side (what the phone addressed)
    uint8_t  pendingDstIp[4];     // the phone
    uint16_t pendingSrcPort, pendingDstPort;

    // Datagrams built and waiting to be handed to the PPP layer. A relayed
    // reply can arrive as several datagrams (segmented WTP results), so this
    // is a small queue rather than a single slot.
    uint8_t  out[WAP_OUTQ][WAP_MAX_DGRAM];
    uint16_t outLen[WAP_OUTQ];
    uint8_t  outHead, outTail;

    uint8_t  hostBridge;          // host answers requests; else serve built-in
    uint8_t  relay;               // host is a plain UDP relay to a real gateway
    // In relay mode the raw UDP payload is handed over untouched — we do not
    // parse or terminate WSP at all, so the phone's session is genuinely with
    // whatever gateway the host forwards to.
    uint8_t  relayReq[WAP_MAX_DGRAM];
    uint16_t relayReqLen;
} WapGw;

void wap_reset(WapGw* g);

// Handle one inbound IP datagram (full IP header onwards). If an immediate
// reply is due, writes a complete IP datagram to out and returns its length,
// else 0 (which may mean a fetch is now pending).
unsigned wap_handle_ip(WapGw* g, const uint8_t* ip, unsigned len,
                       uint8_t* out, unsigned outMax, int log);

// ── Host bridge ──
// mode 0 = off (serve the built-in deck), 1 = host fetches and transcodes,
// 2 = host relays raw datagrams to a real WAP gateway (we terminate nothing).
void        wap_set_host_bridge(WapGw* g, int mode);
int         wap_host_bridge(const WapGw* g);
// The URI the host should fetch, or NULL when nothing is pending.
const char* wap_pending_uri(const WapGw* g);
// In relay mode: the raw UDP payload awaiting forwarding (NULL if none).
const uint8_t* wap_pending_relay(const WapGw* g, unsigned* len);
// In relay mode: a datagram that came back from the real gateway.
void        wap_relay_deliver(WapGw* g, const uint8_t* pay, unsigned len, int log);
// Supply the fetched body; builds the deferred reply datagram. ct is a WSP
// well-known content-type value (0x14 = application/vnd.wap.wmlc).
void        wap_deliver(WapGw* g, uint8_t ct, const uint8_t* body, unsigned len, int log);
// Drain a datagram the gateway has ready to send. Returns its length or 0.
unsigned    wap_take_out(WapGw* g, uint8_t* buf, unsigned max);

// ── Native host bridge (src/mad2/dsp/wap_host.c) ──
// Arm from the environment (WAPGW / WAPPROXY); returns the selected mode. In
// the web build both are no-ops — the page's JS bridge answers there instead.
int  wap_host_configure(WapGw* g);
// Service a pending request by running the node helpers. Blocking, which is
// what keeps the phone's WTP timers from expiring mid-fetch.
void wap_host_pump(WapGw* g);

#endif
