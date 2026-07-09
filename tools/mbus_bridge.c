// mbus_bridge — host serial <-> emulated MBUS service bus. See mbus_bridge.h.

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE          /* cfmakeraw */
#endif
#ifndef _XOPEN_SOURCE
#define _XOPEN_SOURCE 600        /* posix_openpt, grantpt, unlockpt, ptsname */
#endif

#include "mbus_bridge.h"
#include "mad2/mad2.h"
#include "mad2/mad2_internal.h"   // mbus_rx_push, mbus_tx_out_pop, mbus_rx_count

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

// Host-side inbound staging: the tool can blast a whole frame faster than the firmware
// drains its 16-byte RX FIFO, so we buffer here and dribble one byte in per emptied FIFO
// (the half-duplex idiom the firmware's per-byte FIQ2 RX ISR expects). Single bridge per
// run, so a file-static ring is fine.
#define HOSTBUF_SZ 1024u
static uint8_t  g_hostbuf[HOSTBUF_SZ];
static unsigned g_hb_head, g_hb_tail;     // count = tail-head
static int      g_echo;
static long     g_log = -1;

static void raw_termios(int fd) {
    struct termios t;
    if (tcgetattr(fd, &t) != 0) return;   // a PTY master may not be a tty in all libcs; best-effort
    cfmakeraw(&t);
    cfsetispeed(&t, B9600);
    cfsetospeed(&t, B9600);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;  // non-blocking reads
    tcsetattr(fd, TCSANOW, &t);
}

// Present a REAL modem-style comport by claiming a tty0tty null-modem pair: the
// emulator opens the even end (tnt0/2/4/6), the service tool connects to the odd
// end (tnt1/3/5/7) which carries genuine DTR/RTS/DSR/DCD/CTS lines. This is what
// lets NokTool & co. drive the MBUS control lines WITHOUT the LD_PRELOAD shim a
// plain PTY needs. Requires the tty0tty module loaded (see ~/tools/bridge). Opens
// the first available pair; returns the emulator-end fd, or -1 if no /dev/tnt*.
static int open_tnt_pair(int echo) {
    for (int k = 0; k < 4; ++k) {
        char emu[32], tool[32];
        snprintf(emu,  sizeof emu,  "/dev/tnt%d", 2 * k);
        int fd = open(emu, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) continue;             // pair absent (module not loaded) or busy — try next
        raw_termios(fd);
        snprintf(tool, sizeof tool, "/dev/tnt%d", 2 * k + 1);
        fprintf(stderr, "[mbus-bridge] tty0tty: emulator=%s  tool COM=%s (real DTR/RTS/DCD — no shim) (echo=%d)\n",
                emu, tool, echo);
        fprintf(stderr, "[mbus-bridge]   Wine:  ln -sf %s ~/.wine/dosdevices/com3   then connect the tool on COM3 (MBUS)\n",
                tool);
        return fd;
    }
    return -1;
}

int mbus_bridge_open(const char* port, int echo) {
    g_echo = echo;
    g_hb_head = g_hb_tail = 0;
    if (g_log < 0) g_log = getenv("MBUSLOG") ? 1 : 0;
    int fd;
    if (port && *port) {
        fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) { fprintf(stderr, "[mbus-bridge] open(%s) failed: %s\n", port, strerror(errno)); return -1; }
        raw_termios(fd);
        fprintf(stderr, "[mbus-bridge] open %s @9600 8N1 (echo=%d) — point the service tool here in MBUS mode\n",
                port, echo);
    } else if (!getenv("MBUSPTY")) {
        // No explicit device and not forced to a raw PTY: prefer a tty0tty pair so the
        // GUI always presents a MODEM-STYLE comport (real control lines, no shim).
        // MBUSPTY=1 opts back into a bare PTY (the LD_PRELOAD-shim path, e.g. noktool.sh).
        fd = open_tnt_pair(echo);
        if (fd >= 0) return fd;
        fprintf(stderr, "[mbus-bridge] no tty0tty pair (/dev/tnt* absent — module not loaded?); "
                        "falling back to a plain PTY (no modem lines; tool needs the LD_PRELOAD shim)\n");
        // fall through to PTY
        fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0) {
            fprintf(stderr, "[mbus-bridge] posix_openpt failed: %s\n", strerror(errno));
            if (fd >= 0) close(fd);
            return -1;
        }
        const char* slave = ptsname(fd);
        raw_termios(fd);
        fprintf(stderr, "[mbus-bridge] PTY ready (echo=%d). Slave = %s\n", echo, slave ? slave : "?");
        fprintf(stderr, "[mbus-bridge]   Wine:  ln -sf %s ~/.wine/dosdevices/com3   then connect the tool on COM3 (MBUS)\n",
                slave ? slave : "<slave>");
    } else {
        fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0 || grantpt(fd) != 0 || unlockpt(fd) != 0) {
            fprintf(stderr, "[mbus-bridge] posix_openpt failed: %s\n", strerror(errno));
            if (fd >= 0) close(fd);
            return -1;
        }
        const char* slave = ptsname(fd);
        raw_termios(fd);
        fprintf(stderr, "[mbus-bridge] PTY ready (echo=%d). Slave = %s\n", echo, slave ? slave : "?");
        fprintf(stderr, "[mbus-bridge]   Wine:  ln -sf %s ~/.wine/dosdevices/com3   then connect the tool on COM3 (MBUS)\n",
                slave ? slave : "<slave>");
    }
    return fd;
}

void mbus_bridge_feed(struct Mad2* m) {
    if (!m) return;
    // staging -> phone: dribble ONE byte per emptied RX FIFO while the receiver is enabled
    // (ctrl bit6), mirroring the MBUSFRAME path byte-for-byte (same FIQ2 assertion). Called
    // every step: the firmware's RX-enable can open only briefly between bytes, and a byte
    // must land the instant the FIFO drains — a periodic (every-N-steps) feed would skip
    // past those windows and the firmware's frame parser would never assemble the frame.
    if (g_hb_head != g_hb_tail && (m->mbus_ctrl & 0x40) && mbus_rx_count(m) == 0) {
        uint8_t b = g_hostbuf[g_hb_head++ % HOSTBUF_SZ];
        mbus_rx_push(m, b);
        if (g_log) fprintf(stderr, "[mbus-bridge]   ->phone 0x%02X (%u staged left)\n",
                           b, (unsigned)(g_hb_tail - g_hb_head));
    }
}

void mbus_bridge_poll(struct Mad2* m, int fd) {
    if (fd < 0 || !m) return;

    // 1) host -> staging: read whatever the tool sent (non-blocking), echo it back on the
    //    single-wire line, and stage it for metered (per-step) delivery via mbus_bridge_feed.
    uint8_t buf[256];
    for (;;) {
        ssize_t n = read(fd, buf, sizeof buf);
        if (n <= 0) break;                        // EAGAIN / nothing pending
        if (g_echo) { ssize_t w = write(fd, buf, (size_t)n); (void)w; }   // MBUS line echo
        for (ssize_t i = 0; i < n; i++) {
            if ((unsigned)(g_hb_tail - g_hb_head) < HOSTBUF_SZ)
                g_hostbuf[g_hb_tail++ % HOSTBUF_SZ] = buf[i];             // else drop (overrun)
        }
        if (g_log) fprintf(stderr, "[mbus-bridge] rx<-tool %zd byte(s)\n", n);
    }

    // 2) phone -> host: forward everything the firmware has shifted out (the 64-byte TX ring
    //    holds ~1 byte / 64 steps, so a 256-step drain never overruns).
    int b;
    while ((b = mbus_tx_out_pop(m)) >= 0) {
        uint8_t ob = (uint8_t)b;
        ssize_t w = write(fd, &ob, 1); (void)w;
        if (g_log) fprintf(stderr, "[mbus-bridge] tool<-phone 0x%02X\n", ob);
    }
}

void mbus_bridge_close(int fd) {
    if (fd >= 0) close(fd);
}
