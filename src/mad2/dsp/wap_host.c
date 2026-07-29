// Native host side of the emulated WAP gateway.
//
// wap.c parks a request and asks for content it cannot fetch itself. In a
// native build (GUI, trace tools) we can simply run the node helpers, which
// already hold the transcoder and the UDP relay — one implementation, shared by
// every front end. The web build has no processes, so there this is a no-op and
// the page's JS bridge answers instead (see dct3_web_wap_* in src/web/main.c).
//
// Blocking here is deliberate: emulated time stops while we are out on the
// network, so the phone's WTP retransmit budget cannot expire.
//
// Enabled by environment, off unless asked for:
//   WAPGW=<url>   serve the phone's home request from <url> (or WAPGW=1 for the
//                 built-in deck's URLs as-is), transcoding and paginating
//   WAPPROXY=host:port   relay raw datagrams to a real WAP gateway instead
#include "mad2/dsp/wap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__EMSCRIPTEN__)

void wap_host_pump(WapGw* g) { (void)g; }
int  wap_host_configure(WapGw* g) { (void)g; return 0; }

#else

#define WAP_HOST_MAX_OUT 8192u

// Single-quote a string for /bin/sh: 'it''s safe'. Returns 0 if it would not
// fit, which is treated as a failed fetch rather than a truncated command.
static int wap_shell_quote(const char* s, char* out, size_t max) {
    size_t n = 0;
    if (max < 3u) return 0;
    out[n++] = '\'';
    for (; *s; ++s) {
        if (*s == '\'') { if (n + 4u >= max) return 0; memcpy(out + n, "'\\''", 4); n += 4; }
        else            { if (n + 2u >= max) return 0; out[n++] = *s; }
    }
    out[n++] = '\'';
    out[n] = 0;
    return 1;
}

// Where the node helpers live. The tools sit next to the repo root; allow an
// override so a packaged build can point elsewhere.
static const char* wap_tools_dir(void) {
    const char* d = getenv("WAPGW_TOOLS");
    return (d && *d) ? d : "tools";
}

static unsigned wap_run(const char* cmd, uint8_t* out, unsigned max) {
    FILE* f = popen(cmd, "r");
    if (!f) return 0;
    unsigned n = 0;
    size_t got;
    while (n < max && (got = fread(out + n, 1, max - n, f)) > 0) n += (unsigned)got;
    pclose(f);
    return n;
}

// Read the bridge configuration once per call setup.
int wap_host_configure(WapGw* g) {
    if (getenv("WAPPROXY")) { wap_set_host_bridge(g, 2); return 2; }
    if (getenv("WAPGW"))    { wap_set_host_bridge(g, 1); return 1; }
    wap_set_host_bridge(g, 0);
    return 0;
}

static void wap_host_fetch(WapGw* g, const char* uri) {
    const char* home = getenv("WAPGW");
    // The phone's configured home points at a gateway that is long gone, so
    // WAPGW=<url> stands in for it (including our "#wpN" continuations).
    char target[WAP_MAX_URI * 2];
    int isHome = (strncmp(uri, "http://a.com", 12) == 0);
    if (home && *home && strcmp(home, "1") != 0 && isHome) {
        // Carry a "?wpg=N" pagination continuation across the substitution.
        // (A "#fragment" would never get here — WML treats one as a card inside
        // the current deck, so the browser resolves it without asking us.)
        const char* pg = strstr(uri, "wpg=");
        if (pg) snprintf(target, sizeof target, "%s%c%s",
                         home, strchr(home, '?') ? '&' : '?', pg);
        else    snprintf(target, sizeof target, "%s", home);
    } else {
        snprintf(target, sizeof target, "%s", uri);
    }

    char q[WAP_MAX_URI * 3], cmd[WAP_MAX_URI * 4];
    if (!wap_shell_quote(target, q, sizeof q)) { wap_deliver(g, 0x14, 0, 0, 1); return; }
    snprintf(cmd, sizeof cmd, "node %s/wapgw-cli.mjs %s 2>/dev/null", wap_tools_dir(), q);

    static uint8_t body[WAP_HOST_MAX_OUT];
    unsigned n = wap_run(cmd, body, sizeof body);
    wap_deliver(g, 0x14, body, n, 1);            // n == 0 makes the gateway answer 500
}

static void wap_host_relay(WapGw* g, const uint8_t* req, unsigned len) {
    const char* proxy = getenv("WAPPROXY");
    char host[128] = "", cmd[1024];
    unsigned port = 9201;
    if (proxy) {
        const char* colon = strrchr(proxy, ':');
        size_t hl = colon ? (size_t)(colon - proxy) : strlen(proxy);
        if (hl >= sizeof host) hl = sizeof host - 1u;
        memcpy(host, proxy, hl); host[hl] = 0;
        if (colon) port = (unsigned)strtoul(colon + 1, 0, 10);
    }
    char hq[160];
    if (!host[0] || !wap_shell_quote(host, hq, sizeof hq)) { wap_relay_deliver(g, 0, 0, 1); return; }

    // Hex-encode the datagram for the helper's argv.
    static char hex[WAP_MAX_DGRAM * 2 + 1];
    if (len > WAP_MAX_DGRAM) { wap_relay_deliver(g, 0, 0, 1); return; }
    for (unsigned i = 0; i < len; i++) snprintf(hex + i * 2u, 3, "%02x", req[i]);
    hex[len * 2u] = 0;

    snprintf(cmd, sizeof cmd, "node %s/wapudp.mjs %s %u %s 49152 2>/dev/null",
             wap_tools_dir(), hq, port, hex);
    static uint8_t outBuf[WAP_HOST_MAX_OUT];
    unsigned n = wap_run(cmd, outBuf, sizeof outBuf - 1u);
    outBuf[n] = 0;
    if (!n) { wap_relay_deliver(g, 0, 0, 1); return; }
    // One hex datagram per line; a reply can be several.
    char* save = 0;
    for (char* line = strtok_r((char*)outBuf, "\n", &save); line; line = strtok_r(0, "\n", &save)) {
        size_t hl = strlen(line);
        static uint8_t dg[WAP_MAX_DGRAM];
        unsigned dn = 0;
        for (size_t i = 0; i + 1u < hl && dn < sizeof dg; i += 2u) {
            char b[3] = { line[i], line[i + 1], 0 };
            dg[dn++] = (uint8_t)strtoul(b, 0, 16);
        }
        if (dn) wap_relay_deliver(g, dg, dn, 1);
    }
}

void wap_host_pump(WapGw* g) {
    if (!wap_host_bridge(g)) return;
    unsigned rlen = 0;
    const uint8_t* relay = wap_pending_relay(g, &rlen);
    if (relay) {
        static uint8_t req[WAP_MAX_DGRAM];
        if (rlen > sizeof req) rlen = sizeof req;
        memcpy(req, relay, rlen);                 // wap_relay_deliver reuses gw state
        wap_host_relay(g, req, rlen);
        return;
    }
    const char* uri = wap_pending_uri(g);
    if (uri) wap_host_fetch(g, uri);
}

#endif
