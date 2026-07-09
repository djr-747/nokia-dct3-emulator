// dsp_serial.h — raw-tty open + a framed transport wrapper for the DSP bridge.
//
// Both ends of the bridge (src/mad2/dsp_bridge.c, tools/dsp_server.c) speak the same DSPB_*
// messages. Over TCP/unix/fd the byte stream is reliable + in-order, so messages go raw. Over a
// real serial line (the real-8210 payoff) bytes drop/flip, so each message is wrapped in a
// resync-able frame (dsp_frame.h). A DspXport hides that choice: `dspx_send` writes one COMPLETE
// message (raw, or as one frame), `dspx_recv` reads an exact byte count (raw, or pulled from
// decoded frame payloads). The existing incremental recv pattern (read tag, then the per-tag
// body) works unchanged — those reads just drain the current frame's payload.
//
// See docs/dct3-dsp-bridge-hw.md.

#ifndef DSP_SERIAL_H
#define DSP_SERIAL_H
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include "dsp_frame.h"

// Map a numeric baud to its termios speed_t (common rates; default 115200).
static inline speed_t dsp_baud(int b) {
    switch (b) {
        case 9600:   return B9600;   case 19200:  return B19200;
        case 38400:  return B38400;  case 57600:  return B57600;
        case 230400: return B230400; default:     return B115200;
    }
}

// Open a tty (or pty) raw, 8N1, blocking read of >=1 byte. Returns fd or -1.
static inline int dsp_serial_open(const char* path, int baud) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) return -1;
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {          // ptys/ttys: configure; ignore on non-tty
        cfmakeraw(&t);
        speed_t s = dsp_baud(baud);
        cfsetispeed(&t, s); cfsetospeed(&t, s);
        t.c_cflag |= (CLOCAL | CREAD);
        t.c_cflag &= ~CRTSCTS;
        t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &t);
        // Opening a USB-serial tty pulses DTR/RTS, which auto-resets the bridge ESP32. It then
        // needs ~1.5-2s to reboot before it relays; without a settle, the caller starts the DSP
        // cold-reset/upload handshake into a deaf link and desyncs the boot from step 0 (this is
        // what made dsp_ping show 0/20 while a boot-settled ping is 10/10). Wait, then flush the
        // boot-transient line garbage. Tunable via DSPB_OPEN_SETTLE_MS (0 = skip, e.g. for ptys).
        const char* s_env = getenv("DSPB_OPEN_SETTLE_MS");
        int settle_ms = s_env ? atoi(s_env) : 2000;
        if (settle_ms > 0) {
            usleep((useconds_t)settle_ms * 1000);
            tcflush(fd, TCIFLUSH);
        }
    }
    return fd;
}

// Framed transport over a raw fd. `framed` selects serial framing; else pass-through.
typedef struct {
    int     fd;
    int     framed;
    DspFramer rx;                 // inbound frame decoder
    uint8_t pl[DSPFRAME_MAX];     // current decoded frame payload
    int     pl_len, pl_pos;       // its length / read cursor
} DspXport;

static inline void dspx_init(DspXport* x, int fd, int framed) {
    memset(x, 0, sizeof *x); x->fd = fd; x->framed = framed;
}

// Read/write EXACTLY n raw bytes. Returns 0 / -1.
static inline int dsp_raw_rw(int fd, void* p, int n, int wr) {
    char* b = p; int got = 0;
    while (got < n) {
        long r = wr ? write(fd, b + got, n - got) : read(fd, b + got, n - got);
        if (r <= 0) return -1;
        got += (int)r;
    }
    return 0;
}

// Receive EXACTLY n bytes into p. Unframed: straight read. Framed: drain the current frame
// payload, refilling by reading + decoding whole frames (a CRC-dropped frame is skipped).
static inline int dspx_recv(DspXport* x, void* p, int n) {
    if (!x->framed) return dsp_raw_rw(x->fd, p, n, 0);
    uint8_t* b = p; int got = 0;
    while (got < n) {
        if (x->pl_pos >= x->pl_len) {                 // need a fresh frame
            for (;;) {
                uint8_t c;
                if (dsp_raw_rw(x->fd, &c, 1, 0)) return -1;
                int r = dspframe_feed(&x->rx, c, x->pl, (int)sizeof x->pl);
                if (r > 0) { x->pl_len = r; x->pl_pos = 0; break; }
                if (r < 0) { static long d; if ((++d & 0x3F) == 1) fprintf(stderr, "[dspx] frame dropped (crc/resync), n=%ld\n", d); }
            }
        }
        b[got++] = x->pl[x->pl_pos++];
    }
    return 0;
}

// Send one COMPLETE message (len bytes). Unframed: write as-is. Framed: emit one frame.
static inline int dspx_send(DspXport* x, const void* p, int n) {
    if (!x->framed) return dsp_raw_rw(x->fd, (void*)p, n, 1);
    static uint8_t fr[DSPFRAME_MAX * 2 + 8];          // stuffing can ~double; single-threaded
    int fl = dspframe_encode((const uint8_t*)p, n, fr, (int)sizeof fr);
    if (fl < 0) return -1;
    return dsp_raw_rw(x->fd, fr, fl, 1);
}

// Parse a `serial:/dev/ttyX[:baud]` spec -> open + mark framed. Returns fd, sets *framed.
// Returns -1 if `spec` is not a serial: spec (caller falls back to its socket setup).
static inline int dsp_serial_spec(const char* spec, int* framed) {
    if (strncmp(spec, "serial:", 7)) return -1;
    char path[256]; strncpy(path, spec + 7, sizeof path - 1); path[sizeof path - 1] = 0;
    int baud = 115200;
    char* c = strrchr(path, ':');
    if (c && (c[1] >= '0' && c[1] <= '9')) { baud = atoi(c + 1); *c = 0; }   // trailing :baud
    int fd = dsp_serial_open(path, baud);
    if (fd >= 0) { *framed = 1; fprintf(stderr, "[dspx] serial %s @%d (framed)\n", path, baud); }
    return fd;
}

#endif
