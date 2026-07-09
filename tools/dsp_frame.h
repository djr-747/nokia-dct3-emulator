// dsp_frame.h — resync-able HDLC-style framing for the DSP bridge over an UNRELIABLE byte stream
// (serial / FBUS). TCP/unix/fd transports stay unframed (reliable); `serial:` wraps each DSPB
// message in a frame so a dropped/flipped byte costs at most one frame, recovered by CRC + resync.
//
//   frame on the wire:  7E  <stuffed( payload .. crc16_lo crc16_hi )>  7E
//     - 0x7E = flag (delimiter). 0x7D = escape: a literal 0x7E/0x7D in the body is sent as
//       0x7D, (byte ^ 0x20). CRC16/CCITT (poly 0x1021, init 0xFFFF) over the raw payload.
//     - the decoder is a byte-at-a-time state machine: a flag ends/starts a frame, CRC mismatch
//       drops the frame, and NO partial state survives a flag — so any single corruption resyncs
//       on the next good frame. Header-only (static inline) so both ends just #include it.
//
// See docs/dct3-dsp-bridge-hw.md for how this fits the real-8210 proxy path.

#ifndef DSP_FRAME_H
#define DSP_FRAME_H
// Freestanding: the proxy FW (MADos/ARM, no libc) defines DSP_FRAME_NO_STDINT and supplies its
// own uint8_t/uint16_t. Host builds use <stdint.h>. The codec uses NO string.h (manual copy).
#ifdef DSP_FRAME_NO_STDINT
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
#else
#include <stdint.h>
#endif

#define DSPFRAME_FLAG 0x7E
#define DSPFRAME_ESC  0x7D
// Max UNSTUFFED payload a frame carries. Sized for ONE chunked DSPB_WINDOW (DSPB_WIN_CHUNK=170
// entries -> 3 + 170*6 = 1023 B) + headroom. Senders split WINDOW into <=170-entry frames so a
// single line glitch costs a ~1 KB frame, not a 12 KB one — and it fits the proxy's tiny RAM.
#define DSPB_WIN_CHUNK 170
#define DSPFRAME_MAX   1200

static inline uint16_t dspframe_crc16(const uint8_t* p, int n) {
    uint16_t c = 0xFFFF; int i, b;             /* C89-style decls: also compiles under MADos gnu89 */
    for (i = 0; i < n; i++) {
        c ^= (uint16_t)p[i] << 8;
        for (b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

static inline void dspframe_putb(uint8_t* out, int* o, int outmax, uint8_t b) {
    if (b == DSPFRAME_FLAG || b == DSPFRAME_ESC) {
        if (*o + 2 <= outmax) { out[(*o)++] = DSPFRAME_ESC; out[(*o)++] = (uint8_t)(b ^ 0x20); }
    } else if (*o < outmax) out[(*o)++] = b;
}

// Encode payload[0..len) into out[]; returns frame length, or -1 if it won't fit.
static inline int dspframe_encode(const uint8_t* payload, int len, uint8_t* out, int outmax) {
    int o = 0, i;
    if (o < outmax) out[o++] = DSPFRAME_FLAG;
    for (i = 0; i < len; i++) dspframe_putb(out, &o, outmax, payload[i]);
    uint16_t crc = dspframe_crc16(payload, len);
    dspframe_putb(out, &o, outmax, (uint8_t)(crc & 0xFF));
    dspframe_putb(out, &o, outmax, (uint8_t)(crc >> 8));
    if (o < outmax) out[o++] = DSPFRAME_FLAG; else return -1;
    return o;
}

// Streaming decoder. Feed one received byte; on a COMPLETE, CRC-valid frame copies the payload to
// out[] and returns its length (>0). Returns 0 for "need more bytes", -1 on a dropped/corrupt frame
// (caller may log; the decoder has already resynced and is ready for the next frame).
typedef struct { uint8_t buf[DSPFRAME_MAX]; int n; int active; int esc; } DspFramer;

static inline int dspframe_feed(DspFramer* f, uint8_t b, uint8_t* out, int outmax) {
    if (b == DSPFRAME_FLAG) {
        int got = f->active ? f->n : -2;          // -2 = idle (no open frame); 0 = empty delimiter
        int was_esc = f->esc;
        f->active = 1; f->n = 0; f->esc = 0;       // this flag also OPENS the next frame
        if (got < 1) return 0;                     // idle flag OR empty inter-frame fill (7E 7E) — not an error
        if (got < 2 || was_esc) return -1;         // truncated frame / dangling escape
        int plen = got - 2;
        uint16_t want = (uint16_t)(f->buf[plen] | ((uint16_t)f->buf[plen + 1] << 8));
        if (dspframe_crc16(f->buf, plen) != want) return -1;
        if (plen > outmax) return -1;
        { int i; for (i = 0; i < plen; i++) out[i] = f->buf[i]; }   // no string.h (freestanding)
        return plen;
    }
    if (!f->active) return 0;                      // junk before the first flag
    if (b == DSPFRAME_ESC) { f->esc = 1; return 0; }
    if (f->esc) { b ^= 0x20; f->esc = 0; }
    if (f->n < DSPFRAME_MAX) f->buf[f->n++] = b; else f->active = 0;  // overrun -> drop, await flag
    return 0;
}

#endif

#ifdef DSP_FRAME_TEST
#include <stdio.h>
#include <string.h>
static int fails = 0;
static void check(int cond, const char* msg) { if (!cond) { printf("FAIL: %s\n", msg); fails++; } }

// Feed a byte buffer through a fresh framer, return the number of payloads delivered + count CRC drops.
static int run(const uint8_t* wire, int n, int* drops) {
    DspFramer f = {0}; uint8_t out[DSPFRAME_MAX]; int delivered = 0; *drops = 0;
    for (int i = 0; i < n; i++) { int r = dspframe_feed(&f, wire[i], out, sizeof out);
        if (r > 0) delivered++; else if (r == -1) (*drops)++; }
    return delivered;
}

int main(void) {
    uint8_t wire[DSPFRAME_MAX], out[DSPFRAME_MAX];

    // 1. round-trip, including payload bytes that need stuffing (0x7E / 0x7D)
    uint8_t p1[] = { 0x01, 0x00, 0x01, 0x7E, 0x7D, 0x05, 0xFF };
    int wl = dspframe_encode(p1, sizeof p1, wire, sizeof wire);
    { DspFramer f = {0}; int delivered = 0, plen = 0;
      for (int i = 0; i < wl; i++) { int r = dspframe_feed(&f, wire[i], out, sizeof out); if (r > 0) { delivered = 1; plen = r; } }
      check(delivered && plen == (int)sizeof p1 && memcmp(out, p1, plen) == 0, "round-trip with stuffing"); }

    // 2. two back-to-back frames (shared flag boundary) both decode
    uint8_t a[] = {0x05,0,0,0,64}, b[] = {0x09};
    uint8_t two[DSPFRAME_MAX]; int o = dspframe_encode(a, sizeof a, two, sizeof two);
    o += dspframe_encode(b, sizeof b, two + o, (int)sizeof two - o);
    int drops; check(run(two, o, &drops) == 2 && drops == 0, "two back-to-back frames");

    // 3. a flipped payload byte -> CRC drop, and the FOLLOWING good frame still resyncs
    uint8_t corrupt[DSPFRAME_MAX]; int co = dspframe_encode(p1, sizeof p1, corrupt, sizeof corrupt);
    corrupt[2] ^= 0x40;                                   // flip a body byte
    co += dspframe_encode(b, sizeof b, corrupt + co, (int)sizeof corrupt - co);
    int d2; int got2 = run(corrupt, co, &d2);
    check(got2 == 1 && d2 == 1, "corrupt frame dropped, next frame resyncs");

    // 4. a dropped byte in the middle -> at most the damaged frame is lost; a later frame recovers
    uint8_t three[DSPFRAME_MAX]; int t = dspframe_encode(a, sizeof a, three, sizeof three);
    t += dspframe_encode(p1, sizeof p1, three + t, (int)sizeof three - t);
    t += dspframe_encode(b, sizeof b, three + t, (int)sizeof three - t);
    uint8_t dropped[DSPFRAME_MAX]; int di = 0;
    for (int i = 0; i < t; i++) { if (i == t/2) continue; dropped[di++] = three[i]; }  // drop one mid byte
    int d3; int got3 = run(dropped, di, &d3);
    check(got3 >= 1, "dropped byte: at least one later frame still recovers");

    printf(fails ? "dsp_frame: %d FAILED\n" : "dsp_frame: all framing tests PASS\n", fails);
    return fails ? 1 : 0;
}
#endif
