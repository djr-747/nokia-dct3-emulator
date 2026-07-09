// dsp_bridge.c — MCU-side of the remote-DSP transport (see dsp_bridge.h).
//
// Implements the `mad2_dsp_bridge` DspOps backend: it mirrors the HPI window between
// the core RAM and a remote DSP over a byte stream, forwards the DSP control lines
// (0x20002 reset, 0x30000 doorbells, the CCONT rail), and raises the MCU-side
// interrupts the remote reports back. The remote is either another emulator (c54x
// co-sim) or a real phone running proxy FW — same protocol.

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "mad2/mad2.h"
#include "models/model.h"
#include "mad2/dsp_bridge.h"

// The remote-DSP bridge is a HOST transport (serial/socket/termios/poll) — meaningless
// in the browser and unbuildable under emscripten. Compile the whole backend out for
// wasm; the #else at the bottom provides the two symbols mad2.c links against (the
// backend is never selected there because dsp_bridge_enabled() returns 0).
#ifndef __EMSCRIPTEN__

#define HPI_LO   0x00010000u
#define HPI_HI   0x00011000u      // 0x800 words of dual-port DARAM
#define DSP_RST  0x00020002u      // IO_CTSI_DSP reset line
#define DSP_DBL  0x00030000u      // DSP interface doorbell (DSPINT/APIMODE)

// ---- transport (native sockets; inert on wasm) ------------------------------
static int   g_fd = -1;
static int   g_framed;            // 1 = framed serial transport (else raw socket/fd)
static int   g_synperiod = 2048;
static uint8_t g_lastcc[16];
static int   g_doorbell;          // a DSPINT strobe happened since last sync
static uint64_t g_tickn;
static unsigned long g_st_step, g_st_poll, g_st_win, g_st_cells, g_st_rcv, g_st_snd, g_st_blk, g_st_done, g_st_timeout, g_st_shifted;

#ifndef __EMSCRIPTEN__
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include "../../tools/dsp_serial.h"     // raw-tty open + framed DspXport (dsp_frame.h underneath)

static DspXport g_x;                    // transport: framed over serial, raw over socket/fd

static int sock_setup(const char* spec) {
    // serial:/dev/ttyX[:baud] | connect:HOST:PORT | listen:PORT | unix:PATH | fd:N
    char buf[256]; strncpy(buf, spec, sizeof buf - 1); buf[sizeof buf - 1] = 0;
    if (!strncmp(buf, "serial:", 7)) return dsp_serial_spec(buf, &g_framed);   // real-8210 path
    if (!strncmp(buf, "fd:", 3)) return atoi(buf + 3);
    if (!strncmp(buf, "unix:", 5)) {
        int s = socket(AF_UNIX, SOCK_STREAM, 0);
        struct sockaddr_un a; memset(&a, 0, sizeof a); a.sun_family = AF_UNIX;
        strncpy(a.sun_path, buf + 5, sizeof a.sun_path - 1);
        if (connect(s, (struct sockaddr*)&a, sizeof a) == 0) return s;
        close(s); return -1;
    }
    if (!strncmp(buf, "connect:", 8) || !strncmp(buf, "listen:", 7)) {
        int listening = (buf[0] == 'l');
        char* host = listening ? "0.0.0.0" : (buf + 8);
        char* colon = strrchr(host, ':');
        int port = listening ? atoi(buf + 7) : (colon ? atoi(colon + 1) : 0);
        if (colon && !listening) *colon = 0;
        struct sockaddr_in a; memset(&a, 0, sizeof a);
        a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port);
        a.sin_addr.s_addr = listening ? INADDR_ANY : inet_addr(host);
        if (listening) {
            int ls = socket(AF_INET, SOCK_STREAM, 0); int one = 1;
            setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
            if (bind(ls, (struct sockaddr*)&a, sizeof a) || listen(ls, 1)) { close(ls); return -1; }
            fprintf(stderr, "[dspb] listening on :%d ...\n", port);
            int s = accept(ls, NULL, NULL); close(ls);
            return s;
        } else {
            int s = socket(AF_INET, SOCK_STREAM, 0); int one = 1;
            setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            if (connect(s, (struct sockaddr*)&a, sizeof a) == 0) return s;
            close(s); return -1;
        }
    }
    return -1;
}
static int send_all(const void* p, int n) { return g_fd >= 0 ? dspx_send(&g_x, p, n) : -1; }
static int recv_all(void* p, int n)       { return g_fd >= 0 ? dspx_recv(&g_x, p, n) : -1; }
#else
static int sock_setup(const char* s) { (void)s; return -1; }
static int send_all(const void* p, int n) { (void)p; (void)n; return -1; }
static int recv_all(void* p, int n)       { (void)p; (void)n; return -1; }
#endif

static void put32(uint8_t* b, uint32_t v) { b[0]=v>>24; b[1]=v>>16; b[2]=v>>8; b[3]=v; }
static uint32_t get32(const uint8_t* b) { return (uint32_t)b[0]<<24|(uint32_t)b[1]<<16|b[2]<<8|b[3]; }

// ---- enable / connect -------------------------------------------------------
int dsp_bridge_enabled(void) {
    const char* spec = getenv("DSP_BRIDGE");
    if (!spec || !*spec) return 0;
    if (g_fd < 0) {
        const char* sp = getenv("DSPB_SYNC_PERIOD");
        if (sp) g_synperiod = atoi(sp);
        g_fd = sock_setup(spec);
        if (g_fd < 0) { fprintf(stderr, "[dspb] FAILED to open transport '%s'\n", spec); return 0; }
#ifndef __EMSCRIPTEN__
        dspx_init(&g_x, g_fd, g_framed);
#endif
        fprintf(stderr, "[dspb] connected (sync every %d ticks, %s)\n", g_synperiod,
                g_framed ? "framed serial" : "raw stream");
        // COLD SYSTEM RESTART before the MCU boots. We send DSPB_RESET, which the proxy now services as a
        // FULL ASIC reset (IO_CTSI_RST bit2 == ccont_reboot) — NOT the old warm DSP-only 0x20002 toggle that
        // reset just the DSP core and left the HPI/DARAM holding the prior run's half-booted state (dirty
        // window -> undefined DSP boot -> the run-to-run variance). The whole phone reboots, re-runs the
        // proxy from flash, and the real DSP comes up virgin; the MCU's fresh boot then re-seeds the window.
        // The proxy does NOT reply (it's rebooting), so we wait a fixed settle for the reboot + proxy
        // re-init + ESP relay, flush the reboot beacon, then ping-gate to confirm the link before booting the
        // MCU. DSPB_COLD_SETTLE_MS (default 30000) tunes the wait; DSPB_NO_COLD_RESET=1 skips the restart.
        if (g_framed && !getenv("DSPB_NO_COLD_RESET")) {
            uint8_t rst[1] = { DSPB_RESET };
            if (send_all(rst, 1) == 0) {
                int settle = getenv("DSPB_COLD_SETTLE_MS") ? atoi(getenv("DSPB_COLD_SETTLE_MS")) : 30000;
                fprintf(stderr, "[dspb] cold system restart sent (ccont_reboot); waiting %d ms for the phone "
                                "to reboot the proxy...\n", settle);
                for (int waited = 0; waited < settle; waited += 100) usleep(100 * 1000);
                tcflush(g_fd, TCIFLUSH);                          // drop the reboot power-on beacon
                dspx_init(&g_x, g_fd, g_framed);                 // reset the framer after the flush
                struct pollfd pfd; pfd.fd = g_fd; pfd.events = POLLIN; pfd.revents = 0;
                int pong = 0;
                for (int i = 0; i < 20 && !pong; i++) {          // send pings until the rebooted proxy answers
                    uint8_t ping[2] = { DSPB_PING, (uint8_t)i };
                    if (send_all(ping, 2)) break;
                    for (int j = 0; j < 20 && !pong; j++) {      // ~2s per ping for a framed PONG
                        uint8_t tag;
                        if (poll(&pfd, 1, 100) > 0 && recv_all(&tag, 1) == 0 && tag == DSPB_PONG) {
                            uint8_t rest[5]; recv_all(rest, 5);  // seq A5 5A rxhi rxlo
                            pong = 1;
                        }
                    }
                }
                fprintf(stderr, pong ? "[dspb] cold restart OK — proxy answered PING; virgin DSP this run\n"
                                     : "[dspb] cold restart: no PONG after settle (continuing anyway)\n");
            }
        }
    }
    return g_fd >= 0;
}

// ---- forwarders -------------------------------------------------------------
static void fwd_write(uint32_t addr, int size, uint32_t val) {
    uint8_t m[10]; m[0]=DSPB_WRITE; put32(m+1,addr); m[5]=(uint8_t)size; put32(m+6,val);
    send_all(m, 10);
}
// Control edges ride single unacked frames on a link that demonstrably drops small frames (see the
// WRBLOCK retry counters). A lost DSPB_CTRL_RESET(1) leaves the proxy's DSP HELD IN RESET for the
// whole run — total silence (no version token, no acks, no verdict) that masquerades as a dead DSP.
// LEVEL-type controls (RESET run/hold, APIMODE) are idempotent on the proxy (dsp_enable/dsp_disable/
// dsp_setmem are level sets), so deliver them 3x with short gaps; the EDGE-type DSPINT doorbell
// (dsp_genint) must stay single-shot.
static void fwd_ctrl(uint8_t id, uint32_t val) {
    uint8_t m[6]; m[0]=DSPB_CTRL; m[1]=id; put32(m+2,val);
    int sends = (id == DSPB_CTRL_RESET || id == DSPB_CTRL_APIMODE) ? 3 : 1;
    for (int i = 0; i < sends; i++) { if (i) usleep(20000); send_all(m, 6); }
    if (id == DSPB_CTRL_RESET) {
        fprintf(stderr, "[dspb] CTRL RESET=%u fwd x%d @STEP#%lu\n", (unsigned)val, sends, g_st_step);
        fflush(stderr);
    }
}

// ---- reliable batched window writes (DSPB_WRBLOCK) -----------------------------------------------
// The boot block pump (fw 0x2CAD56: 115 x 512-word stride-32 firmware-fingerprint blocks, mailbox
// ping-pong on dsp_mbox0/1 = k7F/k80) is a level-driven lockstep: ONE lost window write — a data
// word OR the mailbox flag — deadlocks it (proven on HW: the MCU's k80=0 block-1 signal was dropped;
// the DSP kept poll-spinning k80 while the MCU spun k7F forever) or silently corrupts the fingerprint
// the DSP checksums. Fire-and-forget of 512 tiny frames also spends ~8s of solid TX per block, and the
// proxy's reply lands inside the ESP relay's post-burst echo-blank -> lost (WRSYNC v1 timed out here).
// FIX: COALESCE the MCU's contiguous ascending strh run into a few big WRBLOCK frames (base,count,words)
// and deliver each request/reply VERIFIED by CRC16 — the proxy applies the words and echoes the CRC; on
// mismatch/timeout the host RE-SENDS the same block (idempotent: the DSP never consumes a buffer before
// its mailbox flag, which is itself a 1-word WRBLOCK sent AFTER the block). ~5 frames/block instead of
// 512, ~150ms bursts instead of 8s, and every word CRC-checked end to end. DSPB_NO_WRBLOCK=1 reverts to
// fire-and-forget. See docs/dct3-dsp-bridge-*; supersedes the WRSYNC v1 ledger.
#define WRBLK_CAP 64                       // words per WRBLOCK: ~128B payload ~150ms TX @9600 (short burst)
static uint16_t g_run[WRBLK_CAP]; static uint32_t g_run_base; static int g_run_n;
static unsigned long g_wb_ok, g_wb_retry, g_wb_fail;
static uint16_t crc16_step(uint16_t c, uint8_t v) {
    c ^= (uint16_t)((uint16_t)v << 8);
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    return c;
}
static void flush_run(Mad2* m);            // defined after apply_msg (needs the reply-drainer)
static void wrblock_append(Mad2* m, uint32_t addr, uint16_t val) {
    if (g_run_n && addr == g_run_base + (uint32_t)(2 * g_run_n)) g_run[g_run_n++] = val;   // contiguous
    else { flush_run(m); g_run_base = addr; g_run[0] = val; g_run_n = 1; }                 // break -> new run
    if (g_run_n == WRBLK_CAP) flush_run(m);
}

// Cell-level integrity trace (DSPB_CELLLOG=1): dump every HPI window cell in BOTH directions,
// bounded to a STEP window (DSPB_CL_FROM..DSPB_CL_TO, default 780..1000 = the self-test/version
// burst) so we can inspect the self-test upload + the DSP's version/verdict reply for byte-swap /
// word-stride / addressing corruption. See docs/dct3-dsp-bridge-ferry-RESUME.md.
static int g_celllog = -1, g_rdlog = -1, g_cl_from, g_cl_to;
static int cl_window(void) {   // one-time init of the log flags + step bounds; returns 1 if in window
    if (g_celllog < 0) {
        g_celllog = getenv("DSPB_CELLLOG") ? 1 : 0;
        g_rdlog   = getenv("DSPB_RDLOG")   ? 1 : 0;
        g_cl_from = getenv("DSPB_CL_FROM") ? atoi(getenv("DSPB_CL_FROM")) : 780;
        g_cl_to   = getenv("DSPB_CL_TO")   ? atoi(getenv("DSPB_CL_TO"))   : 1000;
    }
    return (long)g_st_step >= g_cl_from && (long)g_st_step <= g_cl_to;
}
static int celllog_active(void) { return cl_window() && g_celllog; }

// Boot-pump handshake LEVEL REPAIR (rides DSPB_HLE_BLOCKACK). Under the HLE the MCU produces ahead
// without waiting for the DSP's ack, so a produce-clear can land on k7F/k80 BEFORE the DSP's ack-write
// for the previous buffer — the ack then erases the produce (last-writer-wins on the dual-port), the
// DSP polls its own 1 forever and the pump wedges. HW-caught at the END of run3: the tail's produce
// k80=0 was erased by the DSP's final block-ack k80=1; the DSP parked on its own level, never consumed
// the tail, never wrote the verdict. Repair = when a window report shows the real cell disagreeing
// with the MCU's LAST-WRITTEN level during the pump (gated to after the first HLE'd ack so the 1/1
// init phase is untouched), RE-SEND the MCU's value as a 1-word CRC-acked WRBLOCK. This is pure
// retransmission of a raced write — the DSP's consume/CRC/verdict all stay real.
static uint16_t g_hs_mcu[2];        // MCU's last-written k7F/k80 level
static uint8_t  g_hs_have[2];       // level valid (MCU has written the cell)
static uint8_t  g_hs_fix[2];        // mismatch seen -> re-assert on next tick
static uint64_t g_hleba_n;          // HLE'd block-ack count (0 = pump not started yet)
static unsigned long g_hs_repairs;
// Upload-phase latch (armed at the version HLE, disarmed at the verdict read). Shared by the block-ack
// HLE, the handshake level-repair, and the DSPB_HLE_VERIFY fast path. See the block near bridge_read.
static int g_ba_armed = 0;
// DSPB_HLE_VERIFY experiment: skip driving the REAL DSP through the 115-block compat verification
// entirely — HLE the block-acks (instant), DROP the fingerprint block DATA (don't WRBLOCK-forward it),
// and supply the MEASURED-real verdict at k=1 — so the MCU reaches the post-verdict phase in seconds.
// The final "run block" + warm reset then proceed for REAL (forwarded once the latch disarms). Tests
// whether the DSP's operational state needs the verification or only the final-block+warm-reset boot.
// Value = the verdict to supply (DSPB_HLE_VERIFY_VAL, default 0x1EFF = the real 8210/656545 verdict).
static int g_hle_verify = -1, g_verify_val = 0x1EFF;

// MCU->DSP WRITE side-effect hook: forward window writes + the control doorbells.
static int bridge_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    if (g_fd < 0) return 0;
    if (addr >= HPI_LO && addr < HPI_HI) {           // 1. HPI window
        if (celllog_active()) {
            uint32_t k = (addr - HPI_LO) >> 1;       // DSP word offset
            fprintf(stderr, "[cell] W->DSP  @%08X (k=%03X) sz=%d val=%08X @STEP#%lu\n",
                    addr, k, size, value, g_st_step); fflush(stderr);
        }
        { uint32_t kk0 = (addr - HPI_LO) >> 1;            // track the MCU's intended k7F/k80 level
          if (kk0 == 0x7F || kk0 == 0x80) { g_hs_mcu[kk0 - 0x7F] = (uint16_t)value; g_hs_have[kk0 - 0x7F] = 1; } }
        // DSPB_HLE_VERIFY: during the latched verification phase, DROP the fingerprint block DATA — the
        // real DSP isn't participating (acks + verdict are HLE'd), so forwarding 115x512 words over the
        // wire is pure waste (the 40-min storm). Core still RAM-backs it (local mirror stays coherent).
        // Once the verdict is read the latch disarms and the final "run block" forwards for real below.
        if (g_hle_verify > 0 && g_ba_armed) return 0;
        // Reliable batched path (framed/real-HW only; DSPB_NO_WRBLOCK=1 reverts to fire-and-forget):
        // COALESCE contiguous ascending writes into a run, delivered as CRC-verified WRBLOCK frames
        // (flushed on a contiguity break, at WRBLK_CAP, and at every sync boundary). A 32-bit STM is
        // two BE halfwords at addr / addr+2. The core still RAM-backs the write below (local mirror).
        static int nowb = -1;
        if (nowb < 0) nowb = getenv("DSPB_NO_WRBLOCK") ? 1 : 0;
        if (g_framed && !nowb) {
            if (size >= 4) { wrblock_append(m, addr, (uint16_t)(value >> 16)); wrblock_append(m, addr + 2, (uint16_t)value); }
            else           { wrblock_append(m, addr, (uint16_t)value); }
            return 0;
        }
        fwd_write(addr, size, value);
        return 0;                                    // core still RAM-backs it (local mirror)
    }
    if (addr == DSP_RST) {                            // 2. DSP reset/run line
        flush_run(m);                                 // pending window writes precede the control edge
        fwd_ctrl(DSPB_CTRL_RESET, value & 1u);
        return 0;
    }
    if (addr == DSP_DBL || addr == DSP_DBL + 1) {     // 3. doorbells (0x30000 BE u16)
        uint32_t v = value;
        flush_run(m);                                 // the DSP must see the window before the doorbell
        fwd_ctrl(DSPB_CTRL_APIMODE, (v >> 1) & 1u);
        if ((v >> 2) & 1u) { fwd_ctrl(DSPB_CTRL_DSPINT, 1); g_doorbell = 1; }
        return 0;
    }
    return 0;
}

// Advance the remote DSP by `delta` and apply its window deltas + interrupts into the MCU.
// This is THE clock-gate: the MCU blocks here until the remote DSP has stepped and reported
// back. Returns 0 on success, -1 on transport loss (g_fd cleared).
static uint64_t g_last_sync;

// ---- frame instrumentation (DSPB_LOG=1): prove SND/RCV + window data come back from the phone -----
static int g_log = -1;
static int g_pace = -1;        // DSPB_PACE_MS: ms to wait after each round-trip (back-to-back MBUS is
                               // lossy via the proxy's post-reply deaf window; ~50ms keeps it reliable)
static int g_reply_to = -1;                       // reply timeout (ms); <=0 disables (serial only)
static void bridge_log_stats(const char* why) {
    fprintf(stderr, "[dspb] %s tx{step=%lu poll=%lu} rx{win=%lu cells=%lu MDISND=%lu MDIRCV=%lu BLOCK=%lu done=%lu timeout=%lu shifted=%lu}\n",
            why, g_st_step, g_st_poll, g_st_win, g_st_cells, g_st_snd, g_st_rcv, g_st_blk, g_st_done, g_st_timeout, g_st_shifted);
}

// Apply one DSP->MCU message (tag already read) into the MCU: an IRQ (MDIRCV/MDISND/BLOCK) or a
// WINDOW delta batch. Shared by the STEP/POLL drain AND the edge-mode async drain (drain_pushed),
// so a pushed doorbell/ring lands identically to a polled one. Returns 0 ok, -1 on transport loss.
static int apply_msg(Mad2* m, uint8_t tag) {
    if (tag == DSPB_IRQ) {
        uint8_t id; if (recv_all(&id, 1)) return -1;
        if (id == DSPB_IRQ_MDIRCV)      { if (g_log && !g_st_rcv) bridge_log_stats("FIRST MDIRCV (RCV) from phone!"); g_st_rcv++; mad2_raise_fiq(m, 0); }
        else if (id == DSPB_IRQ_MDISND) { if (g_log && !g_st_snd) bridge_log_stats("FIRST MDISND (SND) from phone!"); g_st_snd++; mad2_raise_fiq(m, 1); }
        else if (id == DSPB_IRQ_BLOCK)  { if (g_log && !g_st_blk) bridge_log_stats("FIRST BLOCK-ack from phone!");    g_st_blk++; m->irq_pending |= (uint8_t)(1u << 4); }  // IRQ4 block-ack
        return 0;
    }
    if (tag == DSPB_WINDOW) {
        uint8_t hdr[2]; if (recv_all(hdr, 2)) return -1;
        int count = (hdr[0] << 8) | hdr[1];
        if (count > 0) { if (g_log && !g_st_cells) bridge_log_stats("FIRST window cells from phone!"); g_st_win++; g_st_cells += (unsigned)count; }
        int cl = celllog_active();
        // Reassembly-corruption detector: the HPI read-pipeline/stride bug makes a bad sync come
        // back shifted +1 word with every value DUPLICATED across two adjacent cells. Measure, over
        // consecutive same-stride word cells that are both non-zero, the fraction that are equal.
        // A clean sync ~0%; a shifted/doubled sync ~100%. Flag + count when the ratio is high.
        uint32_t prev_a = 0xFFFFFFFFu; uint16_t prev_v = 0;
        int nz_pairs = 0, dup_pairs = 0;
        for (int k = 0; k < count; k++) {
            uint8_t e[6]; if (recv_all(e, 6)) return -1;
            uint32_t a = get32(e) & m->mem_mask; uint16_t v = (uint16_t)((e[4] << 8) | e[5]);
            if (m->mem) { m->mem[a] = (uint8_t)(v >> 8); m->mem[a + 1] = (uint8_t)v; }  // BE
            if (a == prev_a + 2 && v != 0 && prev_v != 0) { nz_pairs++; if (v == prev_v) dup_pairs++; }
            prev_a = a; prev_v = v;
            // Handshake level-repair detection: real cell disagrees with the MCU's last-written level
            // mid-pump -> the MCU's write was erased by the DSP's raced ack (or vice versa). Flag it;
            // bridge_tick re-asserts. Gated by the SAME upload-phase latch (g_ba_armed) as the block-ack
            // HLE — NOT by [mdircv_head], which is the always-reported real-DSP cell k=0xE5 whose BIST
            // garbage disabled this repair mid-pump (the tail race then wedged the verdict; see the
            // g_ba_armed rationale). Armed at the version HLE, disarmed at the verdict read.
            { uint32_t kk = (a - HPI_LO) >> 1;
              if ((kk == 0x7F || kk == 0x80) && g_ba_armed && m->mem) {
                  int i = (int)(kk - 0x7F);
                  if (g_hs_have[i] && v != g_hs_mcu[i]) g_hs_fix[i] = 1;
              } }
            if (cl) { uint32_t kk = (a - HPI_LO) >> 1;
                      fprintf(stderr, "[cell] R<-DSP  @%08X (k=%03X) val=%04X @STEP#%lu\n", a, kk, v, g_st_step); }
        }
        if (cl) fflush(stderr);
        if (nz_pairs >= 16 && dup_pairs * 5 >= nz_pairs * 2) {   // >=40% of non-zero word-pairs duplicated
            g_st_shifted++;
            if (g_log) { fprintf(stderr, "[dspb] WINDOW SHIFTED/DOUBLED @STEP#%lu — %d/%d non-zero word-pairs duplicated (HPI read stride/pipeline bug)\n",
                                 g_st_step, dup_pairs, nz_pairs); fflush(stderr); }
        }
        return 0;
    }
    if (tag == DSPB_IRQLOG) {                         // diagnostic: raw DSP interrupt latches
        uint8_t lat[2]; if (recv_all(lat, 2)) return -1;
        fprintf(stderr, "[dspb] LATCH  FIQL=0x%02x  IRQL=0x%02x  (raw sources the DSP raised)\n", lat[0], lat[1]);
        return 0;
    }
    return -1;                                        // unexpected tag = protocol desync
}

// Wait for a WRBLOCK ack (DSPB_WRBLOCK 0x01 crc:u16), applying any stray frames (a stale DONE from a
// timed-out STEP, a pushed WINDOW/IRQ) met on the way. Returns 0 = got ack (crc out), 1 = timeout,
// -1 = transport dead.
static int wrblock_ack(Mad2* m, uint16_t* crc, int ms) {
    struct pollfd pfd; pfd.fd = g_fd; pfd.events = POLLIN; pfd.revents = 0;
    for (int i = 0; i < ms / 100; i++) {
        if (g_x.pl_pos >= g_x.pl_len && poll(&pfd, 1, 100) <= 0) continue;
        uint8_t tag; if (recv_all(&tag, 1)) return -1;
        if (tag == DSPB_WRBLOCK) {
            uint8_t p[3]; if (recv_all(p, 3)) return -1;   // status:u8 crc:u16 (status 0x01 = applied)
            *crc = (uint16_t)((p[1] << 8) | p[2]);
            return 0;
        }
        if (tag == DSPB_DONE) continue;                    // stale DONE from an abandoned STEP reply
        if (apply_msg(m, tag)) return -1;
    }
    return 1;
}

// Flush the pending contiguous write-run as CRC-verified WRBLOCK frame(s): resend a frame until the
// proxy echoes a matching CRC (idempotent — re-applying the same words is harmless). Ordering holds
// because the mailbox flag is itself a later 1-word run flushed after its block. Called on a
// contiguity break, at WRBLK_CAP, and (crucially) at every sync boundary so the flag lands during the
// MCU's block-ack spin. A single run is <= WRBLK_CAP words -> exactly one frame.
static void flush_run(Mad2* m) {
    if (g_run_n <= 0) return;
    int n = g_run_n; uint32_t base = g_run_base; g_run_n = 0;   // clear first: wrblock_ack drains frames
    uint16_t want = 0xFFFF;
    uint8_t f[7 + WRBLK_CAP * 2]; f[0] = DSPB_WRBLOCK; put32(f + 1, base);
    f[5] = (uint8_t)(n >> 8); f[6] = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        f[7 + 2*i] = (uint8_t)(g_run[i] >> 8); f[8 + 2*i] = (uint8_t)g_run[i];
        want = crc16_step(crc16_step(want, (uint8_t)(g_run[i] >> 8)), (uint8_t)g_run[i]);
    }
    for (int attempt = 0; attempt < 8; attempt++) {
        if (send_all(f, 7 + n * 2)) return;                // transport dead
        uint16_t got = 0; int r = wrblock_ack(m, &got, 1500);
        if (r < 0) return;
        if (r == 0 && got == want) { g_wb_ok++;            // verified delivered
            if (g_log > 0 && (g_wb_ok % 8) == 0)           // ~1 log per 512-word block (heartbeat)
                fprintf(stderr, "[dspb] WRBLOCK progress ok=%lu (~block %lu) last base=%05X @STEP#%lu retry=%lu\n",
                        g_wb_ok, g_wb_ok / 8, base, g_st_step, g_wb_retry), fflush(stderr);
            return;
        }
        g_wb_retry++;
        if (g_log > 0) { fprintf(stderr, "[dspb] WRBLOCK retry #%lu @STEP#%lu base=%05X n=%d (%s: want=%04X got=%04X)\n",
                                 g_wb_retry, g_st_step, base, n, r ? "timeout" : "crc-mismatch", want, got); fflush(stderr); }
    }
    g_wb_fail++;
    fprintf(stderr, "[dspb] WRBLOCK FAILED after 8 tries @STEP#%lu base=%05X n=%d\n", g_st_step, base, n);
    fflush(stderr);
}

static int bridge_drain_step(Mad2* m, uint8_t op, uint32_t count) {
    if (g_log < 0) g_log = getenv("DSPB_LOG") ? 1 : 0;
    flush_run(m);                                     // all pending window writes land before the DSP steps
    if (op == DSPB_STEP) g_st_step++; else if (op == DSPB_POLL) g_st_poll++;
    if (g_log && op == DSPB_STEP) {                   // STEPs are the periodic syncs (not floods like POLL)
        fprintf(stderr, "[dspb] -> STEP #%lu (count=%u) awaiting reply...\n", g_st_step, count); fflush(stderr);
    }
    if (g_reply_to < 0) g_reply_to = getenv("DSPB_REPLY_TIMEOUT_MS") ? atoi(getenv("DSPB_REPLY_TIMEOUT_MS")) : 500;
    uint8_t step[5]; step[0]=op; put32(step+1, count ? count : 1);
    if (send_all(step, 5)) { g_fd = -1; return -1; }
    for (;;) {                                       // drain the STEP response until DONE
        uint8_t tag;
        // Reply-timeout guard (framed serial only): each DSPB reply is one atomic frame, so we
        // only ever block on the fd when a FRESH frame is due. The real MBUS link has a residual
        // post-reply "deaf window" that occasionally drops a whole reply frame — with no recovery
        // the host blocks here forever (proxy stays alive + responsive). If the proxy goes silent
        // past the deadline, treat the op as DONE (the proxy already advanced the DSP for it; only
        // the ack was lost) and continue. Worst case is a rare single-step under-advance, vs. the
        // current guaranteed infinite hang. See docs/dct3-dsp-bridge-ferry-RESUME.md.
        if (g_framed && g_reply_to > 0 && g_x.pl_pos >= g_x.pl_len) {
            struct pollfd pfd; pfd.fd = g_fd; pfd.events = POLLIN; pfd.revents = 0;
            int pr = poll(&pfd, 1, g_reply_to);
            if (pr == 0) {                            // silence past deadline = reply lost
                g_st_timeout++;
                if (g_log) { fprintf(stderr, "[dspb] reply TIMEOUT @STEP #%lu (op=0x%02X, %lu total) — lost reply, continuing\n",
                                     g_st_step, op, g_st_timeout); fflush(stderr); }
                // The abandoned reply may still be TRANSMITTING on the half-duplex wire — any write
                // the MCU fires now collides with it (measured: M->D losses cluster after timeouts,
                // and one lost mailbox write deadlocks the boot block pump). Let the in-flight TX
                // drain, then flush the stale partial frame + reset the framer.
                usleep(300 * 1000);
                tcflush(g_fd, TCIFLUSH);
                dspx_init(&g_x, g_fd, g_framed);
                if (g_pace < 0) g_pace = getenv("DSPB_PACE_MS") ? atoi(getenv("DSPB_PACE_MS")) : 0;
                if (g_pace > 0) usleep((useconds_t)g_pace * 1000);
                return 0;                             // treat as completed
            }
        }
        if (recv_all(&tag, 1)) { g_fd = -1; return -1; }
        if (tag == DSPB_DONE) {
            g_st_done++;
            if (g_log && (g_st_done % 200 == 0)) bridge_log_stats("...");
            if (g_pace < 0) g_pace = getenv("DSPB_PACE_MS") ? atoi(getenv("DSPB_PACE_MS")) : 0;
            if (g_pace > 0) usleep((useconds_t)g_pace * 1000);   // space requests out of the deaf window
            return 0;
        }
        if (apply_msg(m, tag)) { g_fd = -1; return -1; }   // IRQ / WINDOW (or desync -> drop link)
    }
}

// ---- edge-driven mode (DSPB_EDGE=1): pace the MCU + drain proxy-PUSHED edges asynchronously ------
// The periodic blocking STEP is retired here; instead the proxy pushes IRQ/WINDOW frames the instant
// the DSP latches HINT / advances a ring, and we (1) throttle the MCU to ~wall-clock so the real DSP
// keeps up, and (2) drain those pushes every tick without a round-trip. See
// docs/dct3-dsp-bridge-edge-driven-DESIGN.md.
static int g_edge = -1;          // DSPB_EDGE: 1 = edge-driven, 0 (default) = periodic-STEP clock
static long g_edge_nspt = -1;    // DSPB_EDGE_NS_PER_TICK: added wall-time per tick (throttle knob)

// Throttle: hold the MCU to roughly (1e9 / ns_per_tick) ticks/sec of wall-clock so the ~21ms MBUS
// RTT is small relative to MCU-perceived time. Batches sleeps to ~1ms granularity for efficiency.
static void bridge_pace(void) {
    static long long acc_ns;
    if (g_edge_nspt <= 0) return;
    acc_ns += g_edge_nspt;
    if (acc_ns >= 1000000) { usleep((useconds_t)(acc_ns / 1000)); acc_ns = 0; }
}

// Non-blocking: apply any frames the proxy has PUSHED since last tick (no STEP sent). Pushed frames
// are IRQ (and optionally WINDOW); there is no DONE. Stops when the fd has nothing immediately ready.
static void bridge_drain_pushed(Mad2* m) {
    for (;;) {
        struct pollfd pfd; pfd.fd = g_fd; pfd.events = POLLIN; pfd.revents = 0;
        if (g_x.pl_pos >= g_x.pl_len && poll(&pfd, 1, 0) <= 0) return;   // nothing buffered or on the wire
        uint8_t tag; if (recv_all(&tag, 1)) { g_fd = -1; return; }
        if (tag == DSPB_DONE) continue;                                  // stray DONE (edge mode) -> ignore
        if (apply_msg(m, tag)) { g_fd = -1; return; }
    }
}

// Coherent dual-port READ (DSPB_SYNCREAD=1). The MCU↔DSP upload/keep-alive handshake rendezvous
// on shared mailbox cells (e.g. 0x87F/0x880) that the DSP sets-and-HOLDS while the concurrently-
// running MCU acks. Our default model samples the window only at STEP boundaries, so the DSP's
// sub-sample mailbox pulses net out and the MCU never sees the level — the rendezvous deadlocks
// (see docs/dsp-bridge-RESUME.md). Here a window read advances the DSP a hair (DSPB_READSTEP, =1
// by default → 1 MCU-poll : 1 DSP-insn) and returns the DSP's CURRENT level, recreating the
// real-dual-port-RAM coincidence: the MCU's poll loop both drives the DSP and observes every
// level it holds. Faithful — it's exactly what the real-8210 proxy FW does (read the live HPI).
static int g_syncread = -1, g_readstep = 1, g_synck2 = -1, g_hle_ver = -1, g_hle_ba = -1;
// Block-ack HLE upload-phase LATCH. The old gate re-tested [mdircv_head] on EVERY k7F/k80 read, but
// mdircv_head (8210 k=0xE5 / 0x101CA) is in the PROXY's always-report list — so the mirror of it is
// continuously overwritten with the REAL DSP's MDIRCV-ring value, which holds nondeterministic BIST/
// ring garbage during the very upload window we need the HLE. Whenever that garbage was nonzero the
// gate read "queue initialised", silently disabled the HLE mid-pump, and the demand-page collapsed to
// the wedging path (window floats, no HINT). Root of the run-to-run pump-vs-"lean" flip: the real-DSP-
// vs-emulated-MCU race (WATCH overhead alone flipped it). FIX: a ONE-WAY latch — arm when the version
// HLE fires (upload is imminent; version is checked at fw 0x2CAD46 immediately before the block loop),
// stay armed through all 115 blocks regardless of E5, disarm only when the REAL verdict is read at k=1
// (fw 0x2CADCE, strictly after the upload) or on reset. Decouples the gate from the always-reported
// real-DSP cell. Requires DSPB_HLE_VERSION (always paired with DSPB_HLE_BLOCKACK in practice).
// (g_ba_armed is declared near the top so apply_msg's level-repair detection can share it.)
static int bridge_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    (void)ram_value;
    if (g_fd < 0) return 0;
    if (g_syncread < 0) {
        g_syncread = getenv("DSPB_SYNCREAD") ? 1 : 0;
        if (getenv("DSPB_READSTEP")) g_readstep = atoi(getenv("DSPB_READSTEP"));
    }
    if (g_synck2 < 0) g_synck2 = getenv("DSPB_SYNCK2") ? 1 : 0;
    if (g_hle_ver < 0) g_hle_ver = getenv("DSPB_HLE_VERSION") ? atoi(getenv("DSPB_HLE_VERSION")) : 0;
    if (g_hle_verify < 0) {
        g_hle_verify = getenv("DSPB_HLE_VERIFY") ? 1 : 0;
        if (getenv("DSPB_HLE_VERIFY_VAL")) g_verify_val = (int)strtol(getenv("DSPB_HLE_VERIFY_VAL"), NULL, 0);
    }

    // DSPB_HLE_VERIFY: short-circuit the whole compat verification. Supply the measured-real verdict at
    // k=1 (0x10002) so the MCU exits the verdict wait 0x2CADCE instantly — the block data was dropped
    // (bridge_write) and the block-acks HLE'd, so the real DSP never participated. Disarms the latch so
    // the post-verdict final block forwards for real. First k=1 read is the verdict wait (k=1 isn't read
    // earlier in the boot), so this is safe. EXPERIMENT: proves whether the real DSP's operational state
    // needs the verification or only the final-block + warm reset. Value = DSPB_HLE_VERIFY_VAL (dflt 0x1EFF).
    if (g_hle_verify && g_ba_armed && addr >= HPI_LO && addr < HPI_HI && ((addr - HPI_LO) >> 1) == 0x01) {
        *out = (uint32_t)(g_verify_val & 0xFFFF);
        g_ba_armed = 0;                               // verdict delivered -> end the HLE-verify phase
        if (g_log) { fprintf(stderr, "[dspb] HLE-VERIFY: verdict k=1 supplied 0x%04x (real DSP skipped) -> final block forwards @step#%lu\n", g_verify_val & 0xFFFF, g_st_step); fflush(stderr); }
        return 1;
    }

    // HLE the DSP version token (DSPB_HLE_VERSION=6). The DSP presents its ROM rev (5/6) at
    // [0x10004] in a brief power-up transient with NO doorbell/edge to sync on, which BIST then
    // clobbers — and we RST the DSP at connect while the MCU doesn't poll until ~1.5min of paced
    // boot later, so the version window is always closed by the time we read (proven: k2 live =
    // 0xFFFF at the poll, 0xa56d after). Supply the known real value so the MCU proceeds to the
    // demand-page upload; EVERYTHING downstream (self-test result, op-code upload, MDI) uses the
    // REAL DSP. Faithful value (this phone's DSP genuinely IS rom6), transport-race workaround only.
    // Self-limits by READ COUNT (not by edge state — early spurious BLOCK/MDI from BIST-garbage
    // ring cells kept tripping the old gate off before the poll). The version poll is a short early
    // burst of k=2 reads (~20-retry cap); we HLE the first N (DSPB_HLE_VER_N, default 1024) and then
    // revert k=2 to the real value so a later boot-status reuse of [0x10004] isn't clobbered.
    if (g_hle_ver > 0 && addr >= HPI_LO && addr < HPI_HI && ((addr - HPI_LO) >> 1) == 0x02) {
        static uint64_t hv_n, hv_last; static long hv_cap = -1;
        if (hv_cap < 0) hv_cap = getenv("DSPB_HLE_VER_N") ? atol(getenv("DSPB_HLE_VER_N")) : 1024;
        if ((long)hv_n < hv_cap) {
            hv_n++;
            g_ba_armed = 1;                                    // arm the block-ack latch: upload is imminent
            *out = (uint32_t)g_hle_ver;
            if (g_st_step != hv_last) { fprintf(stderr, "[dspb] k2 version HLE'd -> %d (real DSP rom rev; wire can't catch the transient) @step#%lu read#%llu\n", g_hle_ver, g_st_step, (unsigned long long)hv_n); fflush(stderr); hv_last = g_st_step; }
            return 1;
        }
    }
    // HLE the DEMAND-PAGE BLOCK-ACK (DSPB_HLE_BLOCKACK=1). The 115-block upload double-buffer (k7F/k80,
    // 0x100FE/0x10100) is a held-level 4-phase handshake: the MCU writes its "produce" marker (0) then
    // spins reading the PARTNER cell for the DSP's ack (!=0). Over the bridge this DEADLOCKS on LATENCY,
    // not loss (RAM-proven: both buffers uploaded, k7F=k80=0, MCU r12=1 @0x2CAD8C) — the µs-fast free-
    // running DSP and the ~400ms/write-paced MCU race on the cell and never register each other's
    // transitions. Break it by HLE'ing ONLY the MCU's ACK-WAIT read (return "acked"): the MCU then does
    // its produce-CLEAR write, which is exactly what the real DSP is waiting for, so the DSP's REAL
    // handshake flows and it consumes each buffer (block DATA still crosses via lossless WRBLOCK). The
    // MCU streams ~3.2s/block (bridge-limited) while the DSP consumes in µs -> no overwrite race. Goal =
    // reach the VERIFICATION: the real DSP checksums this phone's own firmware fingerprint and writes the
    // verdict to k=1 (NOT faked). Same rationale as DSPB_HLE_VERSION — supply only the transport-broken
    // pacing signal. Gated by the g_ba_armed LATCH (armed at the version HLE, disarmed at the verdict
    // read below) so it stays active across the whole 115-block upload and never touches k80's later
    // reuse as the MDIRCV queue word0. (The post-verdict final block "needs no land".)
    if (g_hle_ba < 0) g_hle_ba = getenv("DSPB_HLE_BLOCKACK") ? 1 : 0;
    if (g_hle_ba && g_ba_armed && addr >= HPI_LO && addr < HPI_HI && m->mem) {
        uint32_t kk = (addr - HPI_LO) >> 1;
        if (kk == 0x7F || kk == 0x80) {
            static uint64_t ba_last;
            *out = 1;                                              // "acked" -> unblock the MCU's ack-spin
            if (g_log && g_st_step != ba_last) { fprintf(stderr, "[dspb] block-ack HLE'd (k=%02X->acked) @step#%lu #%llu — DSP gets real data+verdict\n", (unsigned)kk, g_st_step, (unsigned long long)++g_hleba_n); fflush(stderr); ba_last = g_st_step; }
            else g_hleba_n++;
            return 1;
        }
    }
    // read-trace (DSPB_RDLOG=1): log EVERY MCU read of the HPI window in the step window, so we can
    // pin exactly which cells the verdict logic reads (and confirm they're ring-bounded). This is
    // before the syncread filter, so it catches all reads (mirror-served too), not just handshakes.
    if (addr >= HPI_LO && addr < HPI_HI && cl_window() && g_rdlog) {
        uint32_t k = (addr - HPI_LO) >> 1;
        fprintf(stderr, "[rdMCU] read @%08X (k=%03X) sz=%d @STEP#%lu\n", addr, k, size, g_st_step);
        fflush(stderr);
    }
    if (!g_syncread || addr < HPI_LO || addr >= HPI_HI) return 0;
    // Only the DEMAND-PAGE rendezvous cells need the coherent read: req/reply 0x871/0x872
    // (k=71/72) + the ring tail/head (MDISND 0x52/53, MDIRCV E4/E5). NOTE the loader's bulk-upload
    // double-buffer mailboxes 0x87F/0x880 (k=7F/80) are DELIBERATELY excluded: that handshake
    // (DSP loader @0x80xx) works fine on the coarse tick-mirror, and coherent-reading it 1:1
    // deadlocks the upload (DSP spins at 0x8032, MCU stalls pre-timer). Everything else
    // (version/boot-status, upload payload) reads from the tick-synced mirror.
    uint32_t k = (addr - HPI_LO) >> 1;
    // k=1 (0x10002) = the fingerprint-verification VERDICT cell: the MCU parks 0xFFFF there and the
    // verdict wait 0x2CADCE spins `ldrh [k=1]; cmp 0xFFFF; beq .` until the DSP writes its verdict —
    // a ONE-SHOT write with no edge/doorbell, and k=1 is not in the flashed proxy's always-report
    // list, so a stale mirror spins forever even after the real DSP verifies. Coherent-read it like
    // the other rendezvous cells (the DSP is done with its side by then — no k7F/k80-style deadlock).
    int handshake = (k==0x01 || k==0x71 || k==0x72 || k==0x52 || k==0x53 || k==0xE4 || k==0xE5);
    // Version cell k=2 (0x10004): the MCU polls it for the DSP's version token (5/6) to gate the
    // demand-page upload. The coarse tick-mirror can miss a transient present -> upload skipped ->
    // DSP never operational. Coherent-read it (returns + LOGS the live DSP value) — but ONLY until
    // the upload actually starts, signalled by the first BLOCK-ack (IRQ4). We gate on BLOCK (not MDI)
    // because ring-pointer diffs go spurious while the ring cells hold BIST fill, whereas IRQ4 stays
    // genuinely 0 pre-upload. Once a real block-ack lands, k=2 falls back to the mirror. DSPB_SYNCK2=1.
    int k2_early = g_synck2 && (k==0x02) && (g_st_blk==0);
    if (k2_early) handshake = 1;
    if (!handshake) return 0;
    if (bridge_drain_step(m, DSPB_POLL, (uint32_t)g_readstep)) return 0;  // EXACT n insns, no ratio
    g_last_sync = g_tickn;                           // the read just synced; reset the periodic timer
    if (!m->mem) return 0;
    uint32_t a = addr & m->mem_mask, v = 0;
    for (int i = 0; i < size; i++) v = (v << 8) | m->mem[(a + i) & m->mem_mask];  // big-endian
    *out = v;
    if (k == 0x01 && g_ba_armed && (v & 0xFFFF) != 0xFFFF) {   // real verdict landed -> end the upload phase
        g_ba_armed = 0;                                        // disarm the block-ack HLE (k80 now becomes MDI queue)
        if (g_log) { fprintf(stderr, "[dspb] verdict read k=1 = 0x%04x -> block-ack HLE disarmed @step#%lu\n", v & 0xFFFF, g_st_step); fflush(stderr); }
    }
    if (k2_early) {                                  // LOG what the DSP presents at the version cell
        static uint32_t k2_last = 0xFFFFFFFFu, k2_n = 0;
        if (v != k2_last || (k2_n++ & 0x3F) == 0) {
            fprintf(stderr, "[dspb] k2 version-poll: [0x10004] live = 0x%04x  @poll#%lu step#%lu%s\n",
                    v & 0xFFFF, g_st_poll, g_st_step, (v==5||v==6) ? "  <-- VERSION 5/6!" : "");
            fflush(stderr); k2_last = v;
        }
    }
    return 1;
}

// One-shot terminal-state window dump. When DSPB_WINDUMP=<file> and the STEP counter reaches
// DSPB_WINDUMP_AT_STEP, ask the proxy for the FULL raw window (DSPB_DUMPREQ) and save the real DSP's
// view — the ground truth to diff against the MCU mirror at the block-ack stall. Fired at the top of
// bridge_tick (line idle: the previous STEP's DONE is drained), so the only reply is our WINDOW+DONE.
static void maybe_windump(Mad2* m) {
    static int armed = -1; static long at_step = 0; static int fired = 0; static const char* path = 0;
    if (armed < 0) {
        path = getenv("DSPB_WINDUMP");                    // optional: save the raw window to this file
        at_step = getenv("DSPB_WINDUMP_AT_STEP") ? atol(getenv("DSPB_WINDUMP_AT_STEP")) : 0;
        armed = (at_step > 0) ? 1 : 0;                    // trigger on the STEP count; file is optional
    }
    if (!armed || fired || (long)g_st_step < at_step) return;
    fired = 1;
    uint8_t req[1] = { DSPB_DUMPREQ };
    if (send_all(req, 1)) return;
    static uint16_t win[0x800];
    for (;;) {
        uint8_t tag; if (recv_all(&tag, 1)) return;
        if (tag == DSPB_DONE) break;
        if (tag == DSPB_WINDOW) {
            uint8_t hdr[2]; if (recv_all(hdr, 2)) return;
            int count = (hdr[0] << 8) | hdr[1];
            for (int k = 0; k < count; k++) {
                uint8_t e[6]; if (recv_all(e, 6)) return;
                uint32_t a = get32(e); uint16_t v = (uint16_t)((e[4] << 8) | e[5]);
                uint32_t kk = (a - HPI_LO) >> 1; if (kk < 0x800) win[kk] = v;
            }
        } else if (apply_msg(m, tag)) return;         // tolerate a stray pushed IRQ frame
    }
    if (path && *path) {
        FILE* f = fopen(path, "wb");
        if (f) { for (int i = 0; i < 0x800; i++) { uint8_t b[2] = { (uint8_t)(win[i] >> 8), (uint8_t)win[i] }; fwrite(b, 1, 2, f); } fclose(f); }
    }
    fprintf(stderr, "[dspb] WINDUMP @STEP#%lu%s%s (real DSP raw window, 0x800 cells). key cells:\n",
            g_st_step, (path && *path) ? " -> " : "", (path && *path) ? path : "");
    fprintf(stderr, "  k=02 ver[0x10004]=%04X   k=7F ackA[0x100FE]=%04X   k=80 ackB[0x10100]=%04X\n",
            win[0x02], win[0x7F], win[0x80]);
    fprintf(stderr, "  MDISND t/h k52/53=%04X/%04X   MDIRCV t/h kE4/E5=%04X/%04X\n",
            win[0x52], win[0x53], win[0xE4], win[0xE5]);
    fprintf(stderr, "  upload dest k100[0x10200..]= %04X %04X %04X %04X\n",
            win[0x100], win[0x101], win[0x102], win[0x103]);

    // Edge-divergence report: ask the proxy for its SENT counts, diff vs what we RECEIVED/APPLIED.
    // A positive delta = frames the proxy transmitted that never reached us (dropped in the MBUS deaf
    // window) -> the two sides silently disagree, which corrupts the MCU's view of the DSP.
    {
        struct pollfd pfd; pfd.fd = g_fd; pfd.events = POLLIN; pfd.revents = 0;
        int got = 0; uint8_t tag = 0;
        for (int attempt = 0; attempt < 3 && !got; attempt++) {   // the reply is one frame — retry deaf-window drops
            uint8_t sreq[1] = { DSPB_STATS };
            if (send_all(sreq, 1)) break;
            for (int i = 0; i < 10 && !got; i++)                  // ~1s per attempt
                if (poll(&pfd, 1, 100) > 0 && recv_all(&tag, 1) == 0 && tag == DSPB_STATS) got = 1;
        }
        if (got) {
            uint8_t p[16];
            if (recv_all(p, 16) == 0) {
                unsigned long pr_rcv = get32(p), pr_snd = get32(p+4), pr_blk = get32(p+8), pr_cells = get32(p+12);
                fprintf(stderr, "[dspb] EDGE DIVERGENCE (proxy SENT vs host APPLIED):\n");
                fprintf(stderr, "  MDIRCV : sent=%lu  applied=%lu  Δ=%ld%s\n", pr_rcv, g_st_rcv, (long)pr_rcv-(long)g_st_rcv, pr_rcv!=g_st_rcv?"  <-- DROPPED":"");
                fprintf(stderr, "  MDISND : sent=%lu  applied=%lu  Δ=%ld%s\n", pr_snd, g_st_snd, (long)pr_snd-(long)g_st_snd, pr_snd!=g_st_snd?"  <-- DROPPED":"");
                fprintf(stderr, "  BLOCK  : sent=%lu  applied=%lu  Δ=%ld%s\n", pr_blk, g_st_blk, (long)pr_blk-(long)g_st_blk, pr_blk!=g_st_blk?"  <-- DROPPED":"");
                fprintf(stderr, "  WINcell: sent=%lu  applied=%lu  Δ=%ld%s\n", pr_cells, g_st_cells, (long)pr_cells-(long)g_st_cells, pr_cells!=g_st_cells?"  <-- DROPPED":"");
                fprintf(stderr, "  WRBLOCK: ok=%lu retry=%lu fail=%lu  (CRC-verified batched writes M->D)\n", g_wb_ok, g_wb_retry, g_wb_fail);
            }
        } else fprintf(stderr, "[dspb] EDGE DIVERGENCE: no STATS reply after 3 tries (old proxy FW, or link down)\n");
    }
    fflush(stderr);
}

// Per-step pump: forward CCONT changes (the rail) + sync the window/IRQs at boundaries.
static void bridge_tick(Mad2* m) {
    if (g_fd < 0) return;
    // NOTE: do NOT flush_run() here — bridge_tick runs EVERY tick, so flushing here forced every write
    // out as its own n=1 WRBLOCK (no coalescing). The contiguous copy-loop run instead accumulates and
    // auto-flushes at WRBLK_CAP (in wrblock_append); the tail + the mailbox flag flush at the next sync
    // boundary (bridge_drain_step, which fires every DSPB_SYNC_PERIOD ticks — incl. during the block-ack
    // spin) and before every control edge. That gives ~8 big frames/block instead of 512 tiny ones.
    maybe_windump(m);            // one-shot: dump the real DSP window at DSPB_WINDUMP_AT_STEP (diagnostic)

    // Handshake level-repair (flagged by the WINDOW apply path): re-assert the MCU's last-written
    // k7F/k80 level that the DSP's raced ack erased. Deferred to here (line idle) and skipped while
    // a WRBLOCK run is coalescing so we never split a copy-loop burst.
    if ((g_hs_fix[0] || g_hs_fix[1]) && g_framed && g_run_n == 0) {
        for (int i = 0; i < 2; i++) {
            if (!g_hs_fix[i]) continue;
            g_hs_fix[i] = 0;
            uint32_t a = HPI_LO + 2u * (0x7Fu + (uint32_t)i);
            wrblock_append(m, a, g_hs_mcu[i]);
            flush_run(m);
            g_hs_repairs++;
            if (g_log) { fprintf(stderr, "[dspb] hs-repair k=%02X ->%04X (#%lu — DSP ack raced the MCU write) @STEP#%lu\n",
                                 0x7F + i, g_hs_mcu[i], g_hs_repairs, g_st_step); fflush(stderr); }
        }
    }

    // 4. CCONT rail — diff the decoded register file so it works for serial OR mmapped CCONT.
    if (g_log < 0) g_log = getenv("DSPB_LOG") ? 1 : 0;
    for (int i = 0; i < 16; i++) {
        if (m->ccont[i] != g_lastcc[i]) {
            // CCONT-forward trace: reg-3 (regulator) + 6/7/8 are the proxy's own MBUS-TX rails —
            // a forwarded write here can sag the link the proxy replies over (deaf-but-alive, no
            // reboot). Log reg/old->new + the STEP we're at so a TX collapse can be pinned to it.
            if (g_log) {
                int txrail = (i == 0x03 || i == 0x06 || i == 0x07 || i == 0x08);
                fprintf(stderr, "[dspb] CCONT fwd reg%02X %02X->%02X @STEP~%lu%s\n",
                        i, g_lastcc[i], m->ccont[i], g_st_step, txrail ? "  <-- MBUS-TX RAIL" : "");
                fflush(stderr);
            }
            fwd_ctrl(DSPB_CTRL_CCONT, ((uint32_t)i << 8) | m->ccont[i]);
            g_lastcc[i] = m->ccont[i];
        }
    }

    g_tickn++;

    // DSPB_HLE_VERIFY fast path: during the latched verify phase EVERY DSP interaction is HLE'd host-
    // side (block-acks + verdict in bridge_read, block DATA dropped in bridge_write) — the proxy is not
    // needed at all. Skip the per-STEP round-trip + pacing so the MCU runs the 115-block copy loop at
    // NATIVE emulator speed (the ~265 paced STEPs collapse to milliseconds). CCONT rails were already
    // forwarded above (cheap, fire-and-forget — keeps the chip live). Normal sync resumes the instant
    // the verdict read disarms the latch, in time for the dsp_disable -> final-block -> warm-reset
    // sequence to reach the real DSP. Only under HLE_VERIFY: the faithful pump forwards real block data
    // and MUST keep syncing to deliver it + pull the DSP's real handshake.
    if (g_hle_verify > 0 && g_ba_armed) { g_last_sync = g_tickn; return; }

    // Mode select (once): edge-driven push/drain, or the default periodic-STEP clock.
    if (g_edge < 0) {
        g_edge = getenv("DSPB_EDGE") ? atoi(getenv("DSPB_EDGE")) : 0;
        g_edge_nspt = getenv("DSPB_EDGE_NS_PER_TICK") ? atol(getenv("DSPB_EDGE_NS_PER_TICK")) : 34000;
        if (g_edge) fprintf(stderr, "[dspb] EDGE mode: no periodic STEP; pacing %ld ns/tick, draining pushed edges\n", g_edge_nspt);
    }
    if (g_edge) {                    // (1) throttle to wall-clock  (2) apply pushed edges (no round-trip)
        bridge_pace();
        bridge_drain_pushed(m);
        g_doorbell = 0;              // the DSPINT was already forwarded immediately by bridge_write;
        return;                      // the DSP's response arrives as a pushed edge -> drained above.
    }

    // Default: sync on a doorbell strobe, or periodically (to pull spontaneous DSP keep-alive IRQs).
    int due = g_doorbell || (g_synperiod > 0 && (g_tickn - g_last_sync) >= (uint64_t)g_synperiod);
    if (!due) return;
    g_doorbell = 0;

    // STEP carries the REAL number of MCU ticks since the last sync, so the remote DSP
    // advances proportionally (delta * its ratio) and the keep-alive timing stays right.
    uint32_t delta = (uint32_t)(g_tickn - g_last_sync);
    g_last_sync = g_tickn;
    bridge_drain_step(m, DSPB_STEP, delta);           // ratio-scaled by the server (time-pacing)
}

const DspOps mad2_dsp_bridge = {
    .name  = "bridge",
    .read  = bridge_read,          // coherent window reads under DSPB_SYNCREAD (else inert -> RAM)
    .write = bridge_write,
    .tick  = bridge_tick,
};

#else  // __EMSCRIPTEN__ — browser build: no serial/socket transport exists.
// The bridge is never selected here (enabled()==0); these just satisfy the link.
int dsp_bridge_enabled(void) { return 0; }
static int  bridge_off_read(struct Mad2* m, uint32_t a, int sz, uint32_t rv, uint32_t* o) {
    (void)m; (void)a; (void)sz; (void)rv; (void)o; return 0;   // not handled -> falls through to RAM
}
static int  bridge_off_write(struct Mad2* m, uint32_t a, int sz, uint32_t v) {
    (void)m; (void)a; (void)sz; (void)v; return 0;
}
static void bridge_off_tick(struct Mad2* m) { (void)m; }
const DspOps mad2_dsp_bridge = {
    .name = "bridge(off)", .read = bridge_off_read, .write = bridge_off_write, .tick = bridge_off_tick,
};
#endif // __EMSCRIPTEN__
