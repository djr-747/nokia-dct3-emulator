/*
 * sim_bridge - host-side ISO-7816 T=0 driver. See sim_bridge.h.
 */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE          /* cfmakeraw, CRTSCTS (BSD/GNU termios extensions) */
#endif
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L  /* nanosleep, struct timespec */
#endif
#include "sim_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/select.h>
#include <sys/ioctl.h>

/* ---- Bridge framing (must match esp32/simbridge) ---- */
#define REQ_SYNC   0xA5
#define RSP_SYNC   0x5A
#define CMD_PING       0x01
#define CMD_ACTIVATE   0x02
#define CMD_DEACTIVATE 0x03
#define CMD_TRANSFER   0x04
#define CMD_SETBAUD    0x05  /* 4-byte LE baud; switches the bridge's card UART */
#define ST_OK       0x00
#define ST_NOCARD   0x01
#define ST_MUTE     0x02
#define ST_TIMEOUT  0x03
#define ST_BADREQ   0x04

/* Card clock the bridge drives (must match CARD_CLK_HZ in the sketch).
 * Baud after a speed switch = CLK * Di / Fi (ISO 7816-3). */
#define BRIDGE_CLK_HZ 3571200UL

static int   g_fd = -1;
static int   g_log = 0;

void sim_bridge_set_log(int on) { g_log = on; }

static void logbytes(const char* tag, const uint8_t* b, int n) {
    if (!g_log) return;
    fprintf(stderr, "[simbridge] %s (%d):", tag, n);
    for (int i = 0; i < n; ++i) fprintf(stderr, " %02X", b[i]);
    fprintf(stderr, "\n");
}

static int xact(uint8_t cmd, const uint8_t* payload, int plen,
                uint8_t* status, uint8_t* rbuf, int rcap, int* rlen, int rsp_to_ms);

/* ---- serial ---- */
int sim_bridge_open(const char* dev) {
    if (getenv("SIMBRIDGE_LOG")) g_log = 1;
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { fprintf(stderr, "[simbridge] open %s: %s\n", dev, strerror(errno)); return -1; }

    struct termios t;
    if (tcgetattr(fd, &t) != 0) { close(fd); return -1; }
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~CRTSCTS;
    t.c_cflag &= ~HUPCL;   /* don't pulse DTR (reset the ESP32) on close */
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;     /* we do our own select() timeouts */
    if (tcsetattr(fd, TCSANOW, &t) != 0) { close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    g_fd = fd;

    /* Hardware-reset the ESP32 (esptool-style EN pulse via RTS) so every connection
     * starts from a clean boot — this recovers a board wedged/browned-out by a prior
     * run (deasserting alone does NOT reset it). DTR low (IO0 high = run, not
     * download); pulse RTS (EN low) then release. */
    int dtr = TIOCM_DTR, rts = TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &dtr);                        /* DTR off */
    ioctl(fd, TIOCMBIS, &rts);                        /* RTS on  -> EN low (reset held) */
    struct timespec hold = { 0, 150 * 1000 * 1000 };
    nanosleep(&hold, NULL);
    ioctl(fd, TIOCMBIC, &rts);                        /* RTS off -> EN high (run) */

    /* Wait out the bootloader, then PING-sync: discard boot garbage and don't return
     * until the reader answers (or we give up and let the caller see failures). */
    struct timespec ts = { 0, 700 * 1000 * 1000 };   /* 0.7 s boot settle */
    nanosleep(&ts, NULL);
    tcflush(fd, TCIOFLUSH);
    for (int i = 0; i < 20; ++i) {
        uint8_t st, b[64]; int n = 0;
        tcflush(fd, TCIFLUSH);
        if (xact(CMD_PING, NULL, 0, &st, b, sizeof b, &n, 400) == 0 && st == ST_OK) {
            if (g_log) fprintf(stderr, "[simbridge] synced after %d ping(s)\n", i + 1);
            return 0;
        }
    }
    fprintf(stderr, "[simbridge] warning: no PING response after settle\n");
    return 0;
}

void sim_bridge_close(void) {
    if (g_fd >= 0) close(g_fd);
    g_fd = -1;
}

static int read_to(uint8_t* buf, int len, int to_ms) {
    int got = 0;
    while (got < len) {
        fd_set rs; FD_ZERO(&rs); FD_SET(g_fd, &rs);
        struct timeval tv = { to_ms / 1000, (to_ms % 1000) * 1000 };
        int r = select(g_fd + 1, &rs, NULL, NULL, &tv);
        if (r <= 0) return got;                 /* timeout / error */
        int n = read(g_fd, buf + got, len - got);
        if (n <= 0) { if (errno == EAGAIN) continue; return got; }
        got += n;
    }
    return got;
}

/* Send one request frame, read one response frame.
 * resp payload -> rbuf (cap rcap); *rlen set to payload length; *status set.
 * Returns 0 on a well-formed response (any status), <0 on transport error. */
static int xact(uint8_t cmd, const uint8_t* payload, int plen,
                uint8_t* status, uint8_t* rbuf, int rcap, int* rlen, int rsp_to_ms) {
    if (g_fd < 0) return -1;
    uint8_t hdr[4] = { REQ_SYNC, cmd, (uint8_t)(plen & 0xFF), (uint8_t)(plen >> 8) };
    uint8_t ck = cmd ^ hdr[2] ^ hdr[3];
    for (int i = 0; i < plen; ++i) ck ^= payload[i];

    if (write(g_fd, hdr, 4) != 4) return -1;
    if (plen && write(g_fd, payload, plen) != plen) return -1;
    if (write(g_fd, &ck, 1) != 1) return -1;
    if (g_log) { fprintf(stderr, "[simbridge] -> cmd %02X", cmd); logbytes("payload", payload, plen); }

    /* response: sync, status, len(2), payload, ck */
    uint8_t sync = 0;
    int spins = 0;
    do {
        if (read_to(&sync, 1, rsp_to_ms) != 1) { fprintf(stderr, "[simbridge] resp timeout\n"); return -1; }
    } while (sync != RSP_SYNC && ++spins < 512);
    if (sync != RSP_SYNC) return -1;

    uint8_t h[3];
    if (read_to(h, 3, 500) != 3) return -1;
    uint8_t st = h[0];
    int len = h[1] | (h[2] << 8);
    if (len > rcap) { fprintf(stderr, "[simbridge] resp len %d > cap %d\n", len, rcap); return -1; }
    if (len && read_to(rbuf, len, 500) != len) return -1;
    uint8_t rck;
    if (read_to(&rck, 1, 200) != 1) return -1;
    uint8_t calc = st ^ h[1] ^ h[2];
    for (int i = 0; i < len; ++i) calc ^= rbuf[i];
    if (calc != rck) { fprintf(stderr, "[simbridge] resp checksum\n"); return -1; }

    *status = st;
    *rlen = len;
    if (g_log) { fprintf(stderr, "[simbridge] <- st %02X", st); logbytes("payload", rbuf, len); }
    return 0;
}

int sim_bridge_ping(char* id, int cap) {
    uint8_t st, buf[64]; int n = 0;
    if (xact(CMD_PING, NULL, 0, &st, buf, sizeof buf, &n, 1000) < 0) return -1;
    if (st != ST_OK) return -1;
    int k = n < cap - 1 ? n : cap - 1;
    memcpy(id, buf, k); id[k] = 0;
    return 0;
}

/* ---- Post-ATR speed negotiation -------------------------------------------
 * Old GSM SIMs frequently advertise a fast rate in TA1 (e.g. 0x94 = F512/D8 =
 * 55800 baud at 3.5712 MHz) and — pre-ISO'97 "implicit mode" — simply EXPECT the
 * reader at that rate right after ATR: at the default 9600 they ATR fine and then
 * go mute to the first T=0 header (observed live 2026-06-11, card ATR
 * 3B 16 94 71 01 01 09). Compliant negotiable-mode cards instead want a PPS
 * exchange first. Handle both: try PPS (harmless if ignored), then switch the
 * bridge UART to the TA1 rate either way. SIMBRIDGE_NOBAUD=1 disables (pins the
 * old fixed-9600 behavior). */
static const int FI_TAB[16] = {372,372,558,744,1116,1488,1860,0,0,512,768,1024,1536,2048,0,0};
static const int DI_TAB[16] = {0,1,2,4,8,16,32,0,12,20,0,0,0,0,0,0};

static int bridge_setbaud(uint32_t baud) {
    uint8_t p[4] = { (uint8_t)baud, (uint8_t)(baud >> 8),
                     (uint8_t)(baud >> 16), (uint8_t)(baud >> 24) };
    uint8_t st, buf[8]; int n = 0;
    if (xact(CMD_SETBAUD, p, 4, &st, buf, (int)sizeof buf, &n, 1000) < 0) return -1;
    return st == ST_OK ? 0 : -1;
}

/* TA1 -> target baud, or 0 if the default rate is fine / coding is RFU. */
static uint32_t bridge_ta1_baud(const uint8_t* atr, int alen) {
    static int nobaud = -1;
    if (nobaud < 0) nobaud = getenv("SIMBRIDGE_NOBAUD") ? 1 : 0;
    if (nobaud || alen < 3) return 0;
    if (!(atr[1] & 0x10)) return 0;                 /* no TA1 => default speed */
    uint8_t ta1 = atr[2];
    if (ta1 == 0x11 || ta1 == 0x01) return 0;       /* TA1 = default F/D */
    int fi = FI_TAB[(ta1 >> 4) & 0xF], di = DI_TAB[ta1 & 0xF];
    if (!fi || !di) return 0;                       /* RFU coding — leave alone */
    uint32_t baud = (uint32_t)(((uint64_t)BRIDGE_CLK_HZ * (unsigned)di) / (unsigned)fi);
    return baud == BRIDGE_CLK_HZ / 372 ? 0 : baud;
}

/* Discriminator probe: a DEFAULT-parameters PPS (PPSS=FF, PPS0=00 [no PPS1 => F/D
 * stay default], PCK=FF). A negotiable-mode card echoes it and COMMITS TO NOTHING —
 * it keeps talking at the default 9600, which the bench electricals (1k series +
 * 10k pull-up) handle reliably for every card. An implicit-rate card has ALREADY
 * switched to its TA1 rate at ATR end — our 9600 bytes are framing junk to it
 * (observed: a lone garbled 0xFE fragment back) — so silence here identifies it,
 * and the caller COLD-RESETS again and switches the UART silently.
 * Deliberately NOT negotiating fast cards up to their full TA1 rate: a modern card
 * happily confirms e.g. TA1=0x96 => 223 kbaud, which the bench RC can't carry
 * cleanly -> corrupted APDUs -> the firmware rejects a perfectly good card. */
static int bridge_try_pps_default(void) {
    uint8_t pps[3] = { 0xFF, 0x00, 0xFF };
    uint8_t st, rx[16]; int n = 0;
    int ok = xact(CMD_TRANSFER, pps, 3, &st, rx, (int)sizeof rx, &n, 2000) == 0 && st == ST_OK;
    return ok && n >= 3 && memcmp(rx, pps, 3) == 0;
}

int sim_bridge_activate(uint8_t* atr, int* atr_len) {
    int cap = *atr_len;
    /* Cold reset can intermittently come up mute (marginal connection, or a card
     * that needs a couple of resets to wake). Retry a few times and only accept a
     * response that looks like a real ATR (TS=0x3B direct / 0x3F inverse). */
    for (int attempt = 0; attempt < 5; ++attempt) {
        uint8_t st, buf[64]; int n = 0;
        if (xact(CMD_ACTIVATE, NULL, 0, &st, buf, sizeof buf, &n, 2000) == 0
            && st == ST_OK && n >= 2 && (buf[0] == 0x3B || buf[0] == 0x3F)) {
            int k = n < cap ? n : cap;
            memcpy(atr, buf, k);
            *atr_len = k;
            if (g_log && attempt) fprintf(stderr, "[simbridge] activated on attempt %d\n", attempt + 1);
            /* Post-ATR speed negotiation (TA1 != default). Two card populations:
             *  - negotiable-mode (e.g. the TA1=0x96 card): still at 9600, answers
             *    the PPS -> switch after the echo.
             *  - implicit-rate (e.g. TA1=0x94, ATR 3B 16 94 71 01 01 09): already
             *    AT the TA1 rate; the PPS bytes were noise to it -> cold-reset
             *    once more (activate resets the bridge UART to 9600; the ATR is
             *    always at the default rate) and switch SILENTLY before the first
             *    T=0 byte. */
            uint32_t baud = bridge_ta1_baud(atr, k);
            if (baud) {
                uint8_t ta1 = atr[2];
                if (bridge_try_pps_default()) {
                    /* negotiable card, confirmed DEFAULT params — stays at 9600,
                     * the proven-reliable rate for the bench link. No SETBAUD. */
                    if (g_log) fprintf(stderr, "[simbridge] TA1=0x%02X: negotiable card, "
                                       "confirmed default 9600 (not speeding up)\n", ta1);
                } else {
                    uint8_t st2, buf2[64]; int n2 = 0;
                    if (xact(CMD_ACTIVATE, NULL, 0, &st2, buf2, sizeof buf2, &n2, 2000) == 0
                        && st2 == ST_OK && n2 >= 2 && (buf2[0] == 0x3B || buf2[0] == 0x3F)
                        && bridge_setbaud(baud) == 0)
                        fprintf(stderr, "[simbridge] TA1=0x%02X -> %u baud (implicit-rate card: "
                                        "re-reset, switched silently)\n", ta1, (unsigned)baud);
                    else
                        fprintf(stderr, "[simbridge] TA1=0x%02X: %u-baud switch failed (old bridge "
                                        "fw without CMD_SETBAUD? reflash esp32/simbridge) "
                                        "— staying at 9600\n", ta1, (unsigned)baud);
                }
            }
            return 0;
        }
        /* mute / nocard / garbage / transport hiccup — settle, drain, retry */
        struct timespec ts = { 0, 120 * 1000 * 1000 };
        nanosleep(&ts, NULL);
        tcflush(g_fd, TCIFLUSH);
    }
    return -3;   /* persistent mute / absent */
}

int sim_bridge_deactivate(void) {
    uint8_t st, buf[8]; int n = 0;
    if (xact(CMD_DEACTIVATE, NULL, 0, &st, buf, sizeof buf, &n, 1000) < 0) return -1;
    return st == ST_OK ? 0 : -1;
}

/* ---- T=0 TPDU state machine ----------------------------------------------
 * A small byte queue fed by TRANSFER responses. get_byte() pulls the next byte,
 * issuing a read-only TRANSFER (empty payload) to fetch more when the queue
 * empties. A read-only TRANSFER that returns nothing => WWT timeout. */
typedef struct { uint8_t buf[300]; int head, len; } Q;

static int q_refill(Q* q) {
    uint8_t st; int n = 0;
    if (xact(CMD_TRANSFER, NULL, 0, &st, q->buf, (int)sizeof q->buf, &n, 2000) < 0) return -1;
    if (st != ST_OK) return -1;
    q->head = 0; q->len = n;
    return n;                       /* 0 => nothing came back (timeout) */
}

static int q_get(Q* q, uint8_t* out) {
    if (q->head >= q->len) {
        int n = q_refill(q);
        if (n <= 0) return -1;      /* timeout / error */
    }
    *out = q->buf[q->head++];
    return 0;
}

/* TRANSFER `len` bytes, seeding the queue with whatever comes back. */
static int q_send(Q* q, const uint8_t* tx, int len) {
    uint8_t st; int n = 0;
    if (xact(CMD_TRANSFER, tx, len, &st, q->buf, (int)sizeof q->buf, &n, 2000) < 0) return -1;
    if (st != ST_OK) return -1;
    q->head = 0; q->len = n;
    return 0;
}

int sim_bridge_apdu(const uint8_t hdr[5], int dir,
                    const uint8_t* data_in, int data_in_len,
                    uint8_t* data_out, int* out_len,
                    uint8_t* sw1, uint8_t* sw2) {
    Q q = {{0}, 0, 0};
    uint8_t ins = hdr[1];
    int p3 = hdr[4];
    int want = (dir == SIMB_DIR_OUT) ? (p3 ? p3 : 256) : 0;  /* Le (0 => 256) */
    int got_out = 0;
    int out_cap = out_len ? *out_len : 0;
    int sent_in = 0;                                          /* DIR_IN bytes sent */

    if (q_send(&q, hdr, 5) < 0) return -1;

    for (int guard = 0; guard < 1024; ++guard) {
        uint8_t pb;
        if (q_get(&q, &pb) < 0) return -1;

        if (pb == 0x60) continue;                            /* NULL: wait/poll */

        /* SW1: 0x6X or 0x9X (0x60 handled above) */
        if ((pb & 0xF0) == 0x60 || (pb & 0xF0) == 0x90) {
            *sw1 = pb;
            if (q_get(&q, sw2) < 0) return -1;
            if (out_len) *out_len = got_out;
            return 0;
        }

        if (pb == ins) {                                     /* ACK: all remaining */
            if (dir == SIMB_DIR_IN) {
                int rem = data_in_len - sent_in;
                if (rem > 0 && q_send(&q, data_in + sent_in, rem) < 0) return -1;
                sent_in = data_in_len;
            } else if (dir == SIMB_DIR_OUT) {
                while (got_out < want) {
                    uint8_t b;
                    if (q_get(&q, &b) < 0) return -1;
                    if (got_out < out_cap) data_out[got_out] = b;
                    got_out++;
                }
            }
            continue;                                        /* then expect SW */
        }

        if (pb == (uint8_t)(ins ^ 0xFF)) {                   /* ACK: one byte */
            if (dir == SIMB_DIR_IN) {
                if (sent_in < data_in_len) {
                    if (q_send(&q, data_in + sent_in, 1) < 0) return -1;
                    sent_in++;
                }
            } else if (dir == SIMB_DIR_OUT) {
                uint8_t b;
                if (q_get(&q, &b) < 0) return -1;
                if (got_out < out_cap) data_out[got_out] = b;
                got_out++;
            }
            continue;
        }

        /* Unexpected procedure byte. */
        fprintf(stderr, "[simbridge] T=0 unexpected procedure byte %02X\n", pb);
        return -1;
    }
    return -1;
}
