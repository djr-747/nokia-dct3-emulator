// identity_provision — see identity_provision.h.
//
// The MBUS framing this speaks, RE'd from our own captures (re/mbus-captures/, decoder
// mbusdec.py) since gnokii's fbus-6110.txt documents the reads but not B6/B8/BA:
//
//     1F <dst> <src> <type> <len_hi> <len_lo> <payload...> <seq> <xor>
//     1F <dst> <src> 7F <seq> <xor>                        ACK — the short form
//     <xor> = XOR over every preceding byte of the frame
//
//     payload, tool->phone:  00 01 <cmd> <sub> [data...] <cks>
//     payload, phone->tool:  01 01 <cmd> <sub> [data...] <cks>
//
// Two rules that are easy to get wrong:
//
//  * Do NOT split the byte stream on 0x1F. It occurs inside payloads and silently fragments
//    frames. A candidate frame is accepted only if its XOR checks out, else we skip one byte
//    and resync.
//  * The 0x4F security-record family (B4..BB) carries a trailing CHECKSUM byte after the
//    data: cks = (sum(payload[1:-1]) - 1) & 0xFF, i.e. the sum of every payload byte after
//    the leading destination byte, minus one. Verified against all 58 B4..BB frames in the
//    archived captures. Plain commands (0x64, 0xCD, ...) carry no such byte.
//
// Sequence numbers: the tool sends a command with seq S, the phone ACKs S, then replies with
// seq S+1, and the tool ACKs S+1 — and the tool's NEXT command reuses S+1. One shared
// counter that advances on the phone's reply.

#include "services/identity_provision.h"
#include "services/dct3_calcul.h"
#include "mad2/mad2.h"
#include "mad2/mad2_internal.h"   // mbus_rx_push, mbus_tx_out_pop, mbus_rx_count
#include "models/model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TX_STAGE_SZ 256    // bytes waiting to be dribbled into the phone's RX FIFO
#define RX_ASM_SZ   256    // phone->us byte assembly (frames are <= ~40 bytes)

// Timing. All of these are in ARM cycles (m->rtc_mono, ~13 MHz), NOT steps: what the
// firmware's MBUS receiver cares about is wire time, and an instruction count is not time.
#define ARM_HZ 13000000ull

// Inter-byte spacing is NOT paced. Delivery is window-paced: the firmware opens its MBUS
// receiver only briefly, when it is ready for the next byte, so a byte goes out the moment the
// window opens — exactly what the host serial bridge does. A live Rolis session measures ~420
// emulated cycles/byte, but that is the EMERGENT result of those windows, not a rate to impose
// (imposing it makes us miss windows and the link dies). Nor is a half-duplex turnaround
// needed; both were tried, measured and removed.
//
// Gap between two frames WE send. This one is real and measured: Rolis leaves a median of
// 10455 cycles (min 945) between its ACK and the command that follows it, and comparable gaps
// between every other pair of its frames. We were staging the ACK and the next command
// back-to-back with no gap at all, and the phone's response was to ignore the ACK and
// retransmit its previous reply forever — the exact stall this service kept hitting.
// Inter-BYTE pacing is window-driven (above); inter-FRAME pacing is not, so it must be
// applied explicitly. IDPROVFRAMECYC=<cycles> overrides.
#define INTERFRAME_CYCLES 10455ull

// The firmware only services MBUS from its accessory poll, so allow generous slack.
#define REPLY_TIMEOUT_CYCLES (ARM_HZ * 4ull)    // ~4 s of emulated wire time

// Settle after a write's reply before sending the next command: the firmware commits the
// record to flash/EEPROM in that window, and crowding it is how a real tool gets a NAK.
#define SETTLE_CYCLES (ARM_HZ / 2ull)           // ~500 ms

enum {                      // where we are in the sequence
    ST_IDLE = 0,
    ST_PROBE,               // the session prologue (see PROLOGUE below)
    ST_READ_MSID,           // B4 -> B5, confirm the DSP is reporting the pinned MSID
    ST_WRITE_IMEI,          // B6 -> B7
    ST_WRITE_FAID,          // B8 -> B9
    ST_VERIFY,              // 66 -> 66, read the IMEI back through the firmware
    ST_RESET,               // 65/08, reset phone settings — LAST, see the order note
    ST_SETTLE,              // let the last write commit
    ST_DONE,
    ST_FAILED,
};

// The session prologue, transcribed from the captures. BOTH real tools — NokTool 1.9 and
// Rolis Flasher 4.79 — send exactly this before their first B4, on two different images, and
// jumping straight from 64/02 to B4 gets no B5 back. So it is a session handshake, not tool
// decoration, and we replay it rather than guess which parts matter:
//
//     64/02   status/liveness            -> 64/02
//     D1      version request (frame type 0xD1, not a 0x40 command) -> D2 with the MCU header
//     C8/09   get DSP ROM version        -> C8/09
//     66      get IMEI                   -> 66/01
//     CC/01   get original IMEI          -> CC/01
//     6E/01   get security code          -> 6E/01
//
// `typ` 0 marks the D1 frame, whose payload is a fixed 5 bytes and whose reply is matched on
// the frame type rather than a command byte.
static const struct { uint8_t cmd, sub, expect, typ; } PROLOGUE[] = {
    {0x64, 0x02, 0x64, 0x40},
    {0x00, 0x03, 0x00, 0xD1},   // version: payload 00 01 00 03 00, reply frame type 0xD2
    {0xC8, 0x09, 0xC8, 0x40},
    {0x66, 0x00, 0x66, 0x40},   // sub 0 = "no sub byte", see send_prologue_step
    {0xCC, 0x01, 0xCC, 0x40},
    {0x6E, 0x01, 0x6E, 0x40},
};
#define PROLOGUE_N ((int)(sizeof PROLOGUE / sizeof PROLOGUE[0]))

static struct {
    int      state;
    int      pro_step;           // index into PROLOGUE while ST_PROBE is running
    uint8_t  expect_type;        // frame type of the awaited reply (0x40, or 0xD2 for the
                                 // version request). ZERO = not waiting for anything.
    int      next_state;         // where ST_SETTLE returns to
    int      written;            // records committed so far (0..3)
    uint8_t  seq;
    uint8_t  expect;             // reply command byte we are waiting for. NOT a "waiting"
                                 // flag — the version step legitimately expects 0x00, which
                                 // is why expect_type is the sentinel instead.
    uint64_t deadline;           // rtc_mono cycle at which the wait gives up
    uint64_t settle_until;
    uint64_t tx_next;            // earliest cycle we may start the NEXT frame (see below)
    uint8_t  tx[TX_STAGE_SZ];
    unsigned tx_head, tx_tail;
    uint8_t  rx[RX_ASM_SZ];
    unsigned rx_len;
    char     status[160];
    // The record set for this run, built once at start.
    uint8_t  faid[12], lock1[12], lock2[12], imei[12], imei_bcd[7];
    uint8_t  msid[13];
    char     imei15[16];         // the 15-digit IMEI we asked for, for the read-back check
    char     romver[16];         // DSP ROM version as service C8/09 reported it
    int      verified;           // the phone reported that IMEI back via service cmd 0x66
    int      verify_only;        // read the identity back WITHOUT writing anything
    int      reset_sent;         // the 65/08 settings reset was issued (run complete)
    int      log;                // IDPROVLOG=1 — per-byte wire trace, like MBUSLOG
    uint64_t rx_ctrl_seen;       // steps on which the firmware had MBUS RX enabled
    uint64_t frame_cycles;       // gap between our own frames (INTERFRAME_CYCLES)
    unsigned frame_end[16];      // tx ring positions at which each staged frame ends
    unsigned fe_head, fe_tail;
} G;

static void set_status(const char* s) {
    snprintf(G.status, sizeof G.status, "%s", s);
}

// ---- frame construction ---------------------------------------------------------------

static uint8_t xor_of(const uint8_t* b, unsigned n) {
    uint8_t x = 0;
    while (n--) x ^= *b++;
    return x;
}

static void stage(const uint8_t* b, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
        if ((unsigned)(G.tx_tail - G.tx_head) >= TX_STAGE_SZ) return;   // overrun: drop
        G.tx[G.tx_tail++ % TX_STAGE_SZ] = b[i];
    }
    // Remember where this frame ends so pump_tx can hold the inter-frame gap at the boundary
    // instead of running two frames together.
    if ((unsigned)(G.fe_tail - G.fe_head) < 16u) G.frame_end[G.fe_tail++ % 16u] = G.tx_tail;
}

// Bus addresses, from the raw capture frames (NOT the payload's leading 00 01, which is a
// separate convention inside the payload):
//     tool -> phone   1F 00 10 40 ...      dst 0x00 = phone, src 0x10 = the service tool
//     phone -> tool   1F 10 00 40 ...      dst 0x10 = tool,  src 0x00 = phone
//     phone hello     1F FF 00 D0 ...      dst 0xFF = broadcast (unsolicited, at power-on)
#define ADDR_PHONE 0x00
#define ADDR_TOOL  0x10

// Wrap a payload in a frame and stage it. `typ` is 0x40 for an ordinary command; the
// version request in the prologue is its own frame TYPE (0xD1, answered with a 0xD2).
static void send_payload(const uint8_t* payload, unsigned n, uint8_t seq, uint8_t typ) {
    uint8_t f[64 + 8];
    if (n > 64) return;
    unsigned k = 0;
    f[k++] = 0x1F; f[k++] = ADDR_PHONE; f[k++] = ADDR_TOOL; f[k++] = typ;
    f[k++] = (uint8_t)(n >> 8); f[k++] = (uint8_t)n;
    memcpy(f + k, payload, n); k += n;
    f[k++] = seq;
    f[k] = xor_of(f, k); k++;
    stage(f, k);
}

static void send_ack(uint8_t seq) {
    uint8_t f[6];
    f[0] = 0x1F; f[1] = ADDR_PHONE; f[2] = ADDR_TOOL; f[3] = 0x7F; f[4] = seq;
    f[5] = xor_of(f, 5);
    stage(f, 6);
}

// A plain (non-0x4F) command: 00 01 <cmd> <sub> [arg]. The factory reset is the one caller
// that needs the trailing argument byte, so it is folded in here rather than given its own
// sender; `arg_n` is 0 for every other command.
static void send_cmd_arg(struct Mad2* m, uint8_t cmd, uint8_t sub, uint8_t expect,
                         const uint8_t* arg, unsigned arg_n) {
    uint8_t p[8] = {0x00, 0x01, cmd, sub};
    unsigned k = 4;
    for (unsigned i = 0; i < arg_n && k < sizeof p; ++i) p[k++] = arg[i];
    send_payload(p, k, G.seq, 0x40);
    G.expect = expect;
    G.expect_type = 0x40;
    G.deadline = m->rtc_mono + REPLY_TIMEOUT_CYCLES;
}
static void send_cmd(struct Mad2* m, uint8_t cmd, uint8_t sub, uint8_t expect) {
    send_cmd_arg(m, cmd, sub, expect, NULL, 0);
}

// A 0x4F security-record command, with the family's trailing checksum.
static void send_sec(struct Mad2* m, uint8_t cmd, const uint8_t* data, unsigned n) {
    uint8_t p[64];
    unsigned k = 0;
    p[k++] = 0x00; p[k++] = 0x01; p[k++] = cmd; p[k++] = 0x4F;
    if (n) { memcpy(p + k, data, n); k += n; }
    unsigned sum = 0;
    for (unsigned i = 1; i < k; ++i) sum += p[i];
    p[k++] = (uint8_t)((sum - 1) & 0xFF);
    send_payload(p, k, G.seq, 0x40);
    G.expect = (uint8_t)(cmd + 1);
    G.expect_type = 0x40;                 // B4->B5, B6->B7, B8->B9, BA->BB
    G.deadline = m->rtc_mono + REPLY_TIMEOUT_CYCLES;
}

// Send PROLOGUE[i]. The 0x66 "get IMEI" entry has NO sub byte (payload is just 00 01 66),
// and the version request is a 0xD1-type frame with a fixed 5-byte payload — both verified
// byte-for-byte against the captures.
static void send_prologue_step(struct Mad2* m, int i) {
    const uint8_t cmd = PROLOGUE[i].cmd, sub = PROLOGUE[i].sub;
    uint8_t p[8];
    unsigned k = 0;
    p[k++] = 0x00; p[k++] = 0x01; p[k++] = cmd;
    if (PROLOGUE[i].typ == 0xD1) { p[k++] = sub; p[k++] = 0x00; }   // 00 01 00 03 00
    else if (sub)                 p[k++] = sub;
    send_payload(p, k, G.seq, PROLOGUE[i].typ);
    G.expect = PROLOGUE[i].expect;
    G.expect_type = PROLOGUE[i].typ == 0xD1 ? 0xD2 : 0x40;
    G.deadline = m->rtc_mono + REPLY_TIMEOUT_CYCLES;
}

// ---- the step sequence ------------------------------------------------------------------

static void begin_state(struct Mad2* m, int st) {
    G.state = st;
    switch (st) {
        case ST_PROBE:
            set_status("opening the service session (prologue)");
            G.pro_step = 0;
            send_prologue_step(m, 0);
            break;
        case ST_READ_MSID:
            set_status("reading the MSID (B4)");
            send_sec(m, 0xB4, NULL, 0);
            break;
        case ST_RESET:
            // "Reset phone settings", captured from Rolis and documented in
            // ref/gnokii/fbus-6110.txt: s { 0x65, value, 0x00 } -> r { 0x65, 0x00 }, with
            // value 0x08 = reset UI settings, 0x38 = UI + SCM + call counters. So the
            // trailing 0x00 is part of the command, not an argument we invented, and this is
            // NOT a restart and NOT an EEPROM wipe — it restores the user-interface settings.
            //
            // Not a 0x4F security-record command either, so it carries no family checksum
            // (that rule would want 0x6D here).
            //
            // It runs LAST, and that ordering is forced by measurement, not preference: the
            // phone ACKs the 65/08 frame and then goes PERMANENTLY silent — no 65/00 reply,
            // and nothing answers afterwards, not even a fresh prologue. Rolis's own capture
            // ends at its 65/08 for the same reason. Running it first therefore costs us the
            // session we need for the identity writes. Running it last loses nothing, because
            // 0x65 only resets UI settings and cannot discard the records written before it.
            //
            // Consequently we do NOT wait for a reply here: the frame is sent, the phone acks
            // it, and the run is complete. Treating the missing 65/00 as a failure would
            // report a good provisioning as broken.
            { static const uint8_t arg = 0x00; send_cmd_arg(m, 0x65, 0x08, 0x65, &arg, 1); }
            G.expect = 0; G.expect_type = 0;      // fire-and-forget; see above
            G.reset_sent = 1;
            G.next_state = ST_DONE;
            G.state = ST_SETTLE;                  // let the frame actually reach the phone
            G.settle_until = m->rtc_mono + SETTLE_CYCLES;
            break;
        case ST_WRITE_FAID:
            set_status("writing the FLASH-ID / FAID record (B8)");
            send_sec(m, 0xB8, G.faid, 12);
            break;
        case ST_WRITE_IMEI: {
            // B6 carries the IMEI TWICE: the 12-byte encoded record, then the 7-byte
            // plaintext BCD plus a 0xFF pad. Both are sent because both are what a real
            // NokTool B6/4F write contains; the plaintext is what the firmware echoes back
            // for service command 0x66 "get IMEI", the record is what the security check
            // validates. (We cannot yet prove they land in two different places — read-back
            // over MBUS needs D4, which currently returns length 0. See the handoff doc.)
            uint8_t d[20];
            memcpy(d, G.imei, 12);
            memcpy(d + 12, G.imei_bcd, 7);
            d[19] = 0xFF;
            set_status("writing the IMEI record (B6)");
            send_sec(m, 0xB6, d, 20);
            break;
        }
        case ST_VERIFY:
            // Read the IMEI back the way a real tool does — service command 0x66, answered by
            // the firmware from whatever it actually stored. This is the only end-to-end proof
            // the write landed: we never learn the EEPROM offset, so "the firmware reports the
            // IMEI we asked for" IS the verification. (Rolis issues exactly this after its B6.)
            set_status("reading the IMEI back (66)");
            send_cmd(m, 0x66, 0x00, 0x66);
            break;
        default: break;
    }
}

static void fail(const char* why) {
    G.state = ST_FAILED;
    G.expect = 0; G.expect_type = 0;
    G.tx_head = G.tx_tail = 0;
    G.rx_len = 0;
    char buf[160];
    snprintf(buf, sizeof buf, "%s (%d/2 records written)", why, G.written);
    set_status(buf);
}

// A reply frame for the command we were waiting on.
static void on_reply(struct Mad2* m, const uint8_t* p, unsigned n) {
    switch (G.state) {
        case ST_PROBE:
            // C8/09 "Get DSP Internal SW" answers <status> <chars...> <NUL>, status 0x00 = OK
            // and 0x01 = unavailable. Before the DSP responder answered the MCU's {70 0A},
            // this came back `01 00` — status 1, empty — which is what makes EepromTools say
            // "cannot read ROM version (DSP)". Capture it so a run says which it got.
            if (PROLOGUE[G.pro_step].cmd == 0xC8 && n >= 5) {
                if (p[4] == 0x00) {
                    unsigned k = 0;
                    for (unsigned i = 5; i < n && p[i] && k + 1 < sizeof G.romver; ++i)
                        G.romver[k++] = (char)p[i];
                    G.romver[k] = '\0';
                } else {
                    snprintf(G.romver, sizeof G.romver, "(unavailable)");
                }
                fprintf(stderr, "[idprov] DSP ROM version (C8/09): %s\n", G.romver);
            }
            if (++G.pro_step < PROLOGUE_N) { send_prologue_step(m, G.pro_step); return; }
            begin_state(m, ST_READ_MSID);
            return;
        case ST_READ_MSID: {
            // payload: 01 01 B5 01 4F <13 MSID> <cks>
            char buf[160];
            if (n >= 5 + 13 && !memcmp(p + 5, G.msid, 13)) {
                snprintf(buf, sizeof buf, "MSID confirmed, writing records");
            } else {
                // Not fatal: the records are bound to the PINNED MSID either way, and a
                // mismatch just means the DSP is still echoing (DSPMSID=echo, or no pinned
                // identity reached the responder). Say so loudly — the identity will be
                // internally consistent but will not match what a tool reads back.
                snprintf(buf, sizeof buf,
                         "WARNING: phone reports a different MSID than the pinned one — "
                         "records will not match its COBBA");
            }
            set_status(buf);
            // Verify-only: skip straight to the read-back. Answers "does this image still
            // carry a provisioned identity?" without touching a single record — which is how
            // you check an EEPROM-merged .fls actually kept what was written to it.
            begin_state(m, G.verify_only ? ST_VERIFY : ST_WRITE_IMEI);
            return;
        }
        case ST_RESET:
            // Settle before writing: the firmware is committing its settings reset.
            G.next_state = ST_WRITE_IMEI;
            G.state = ST_SETTLE;
            G.settle_until = m->rtc_mono + SETTLE_CYCLES;
            return;
        case ST_WRITE_IMEI:
            G.written++;
            G.next_state = ST_WRITE_FAID;
            G.state = ST_SETTLE;
            G.settle_until = m->rtc_mono + SETTLE_CYCLES;
            return;
        case ST_WRITE_FAID:
            G.written++;
            G.next_state = ST_VERIFY;
            G.state = ST_SETTLE;
            G.settle_until = m->rtc_mono + SETTLE_CYCLES;
            return;
        case ST_VERIFY: {
            // payload: 01 01 66 01 <15 ASCII digits> 00
            char got[16] = {0};
            if (n >= 4 + 15) memcpy(got, p + 4, 15);
            G.verified = (strcmp(got, G.imei15) == 0);
            char buf[160];
            if (G.verified && G.verify_only)
                snprintf(buf, sizeof buf, "identity present: phone reports IMEI %s", got);
            else if (G.verified)
                snprintf(buf, sizeof buf,
                         "IMEI + FAID written; phone reports IMEI %s", got);
            else if (G.verify_only)
                snprintf(buf, sizeof buf,
                         "no identity: phone reports IMEI '%s', expected %s",
                         got[0] ? got : "(none)", G.imei15);
            else
                // The writes were ACKed but nothing stuck. Seen on the 8890, whose profile
                // never gets past CONTACT SERVICE (its DSP upload handshake does not
                // complete): the firmware answers B7/B9/BB and commits nothing — EEPROM write
                // count stays at 0 and 0x66 still reports the unprovisioned '?' pattern. That
                // is a FAILURE, not a success with a note; without this read-back it would
                // look identical to a good run.
                snprintf(buf, sizeof buf,
                         "wrote the records but the phone reports IMEI '%s', expected %s "
                         "— the firmware ACKed but did not commit",
                         got[0] ? got : "(none)", G.imei15);
            set_status(buf);
            // A good WRITE run ends with the settings reset; a bad one stops, so the failure
            // is not masked by a reset that also ends the session. A verify-only run never
            // sends it at all — the whole point of that mode is that it changes nothing.
            if (G.verify_only)      G.state = G.verified ? ST_DONE : ST_FAILED;
            else if (G.verified)  { begin_state(m, ST_RESET); return; }
            else                    G.state = ST_FAILED;
            fprintf(stderr, "[idprov] %s\n", buf);
            return;
        }
        default:
            return;
    }
}

// ---- wire pump --------------------------------------------------------------------------

// Drain the phone's TX ring into our assembly buffer, then consume every XOR-valid frame.
static void pump_rx(struct Mad2* m) {
    int b;
    while ((b = mbus_tx_out_pop(m)) >= 0) {
        // The phone is driving the single wire: hold off our own transmission until it has
        // been quiet for a turnaround. Re-armed on EVERY received byte, so a multi-byte
        // reply keeps pushing our next transmission out until the frame is complete.
        if (G.log) fprintf(stderr, "[idprov] phone-> 0x%02X\n", (unsigned)b);
        if (G.rx_len < RX_ASM_SZ) G.rx[G.rx_len++] = (uint8_t)b;
        else { memmove(G.rx, G.rx + 1, RX_ASM_SZ - 1); G.rx[RX_ASM_SZ - 1] = (uint8_t)b; }
    }
    unsigned i = 0;
    while (i < G.rx_len) {
        // Three distinct cases, and conflating the first two loses frames: a byte that is
        // not the sync byte is discarded, but a sync byte with the rest of its frame still
        // in flight must be KEPT and waited on. The phone's TX ring dribbles roughly a byte
        // per 64 steps, so a 20-byte reply almost always arrives across many drains — the
        // hello frames only survived a "skip it" here because they land in one burst.
        if (G.rx[i] != 0x1F) { i++; continue; }             // not a frame start — drop
        if (i + 6 > G.rx_len) break;                       // header incomplete — wait
        unsigned n = (G.rx[i + 3] == 0x7F)
                   ? 6u
                   : 6u + (((unsigned)G.rx[i + 4] << 8) | G.rx[i + 5]) + 2u;
        if (n > RX_ASM_SZ) { i++; continue; }               // absurd length — resync
        if (i + n > G.rx_len) break;                       // body incomplete — wait
        if (xor_of(G.rx + i, n - 1) != G.rx[i + n - 1]) { i++; continue; }   // resync

        if (G.rx[i + 3] != 0x7F) {
            const uint8_t* p = G.rx + i + 6;
            unsigned pl = n - 8;
            uint8_t seq = G.rx[i + n - 2];
            if (G.log) fprintf(stderr, "[idprov] frame dst=%02X type=%02X pl=%u cmd=%02X "
                                       "(want %02X)\n", G.rx[i+1], G.rx[i+3], pl,
                                       pl >= 3 ? p[2] : 0, G.expect);
            // ACK only what is addressed to us. The phone's power-on "hello" is a 0xFF
            // broadcast and a real tool does not ack it (verified in the captures) — acking
            // it just injects unexpected frames into the phone's parser.
            if (G.rx[i + 1] == ADDR_TOOL) send_ack(seq);
            // Match on frame TYPE first: the version reply is a 0xD2 frame whose payload is
            // the MCU header string, so there is no command byte to compare — accepting it on
            // the type alone is what the real tools do.
            uint8_t typ = G.rx[i + 3];
            int hit = G.expect_type && typ == G.expect_type &&
                      (G.expect_type == 0xD2 || (pl >= 3 && p[2] == G.expect));
            if (hit) {
                G.seq = seq;                               // shared counter advances here
                G.expect = 0; G.expect_type = 0;
                on_reply(m, p, pl);
            }
        }
        i += n;
    }
    if (i) {                                               // drop what we consumed
        memmove(G.rx, G.rx + i, G.rx_len - i);
        G.rx_len -= i;
    }
}

// Dribble ONE staged byte per emptied RX FIFO while the firmware's receiver is enabled —
// the same pacing mbus_bridge_feed uses for the host serial bridge. The firmware's RX-enable
// opens only briefly between bytes, so this has to run every step: a periodic feed skips
// past those windows and the frame parser never assembles a frame.
static void pump_tx(struct Mad2* m) {
    if (G.tx_head == G.tx_tail) return;
    if (m->rtc_mono < G.tx_next) return;                   // hold the inter-FRAME gap
    if (!(m->mbus_ctrl & 0x40)) return;                    // receiver disabled
    if (mbus_rx_count(m) != 0) return;                     // FIFO not drained yet
    uint8_t b = G.tx[G.tx_head % TX_STAGE_SZ];
    if (mbus_rx_push(m, b)) {
        G.tx_head++;
        if (G.fe_head != G.fe_tail && G.tx_head == G.frame_end[G.fe_head % 16u]) {
            G.fe_head++;                                   // frame complete -> hold the gap
            G.tx_next = m->rtc_mono + G.frame_cycles;
        }
        if (G.log) fprintf(stderr, "[idprov]   ->phone 0x%02X (%u staged left)\n",
                           b, (unsigned)(G.tx_tail - G.tx_head));
    }
}

// ---- public API -------------------------------------------------------------------------

int identity_provision_start(struct Mad2* m) { return identity_provision_run(m, 0); }
int identity_provision_verify(struct Mad2* m) { return identity_provision_run(m, 1); }

int identity_provision_run(struct Mad2* m, int verify_only) {
    if (!m || !m->model) { set_status("no model"); return 0; }
    if (G.state != ST_IDLE && G.state != ST_DONE && G.state != ST_FAILED) {
        set_status("a re-provisioning run is already in flight");
        return 0;
    }
    const uint8_t* msid = m->model->identity.msid;
    const char* imei14  = m->model->identity.imei14;
    int any = 0;
    for (int i = 0; i < 13; ++i) any |= msid[i];
    if (!any || !imei14) {
        char buf[160];
        snprintf(buf, sizeof buf,
                 "model %s has no pinned identity (see tools/dct3_identity.py)",
                 m->model->name ? m->model->name : "?");
        set_status(buf);
        return 0;
    }

    memset(&G, 0, sizeof G);
    G.verify_only = verify_only;
    memcpy(G.msid, msid, 13);
    G.seq = 0x01;
    G.log = getenv("IDPROVLOG") && *getenv("IDPROVLOG") != '0';
    G.frame_cycles = INTERFRAME_CYCLES;

    // Everything below is derived from the pinned MSID, so the whole record set is bound to
    // one COBBA — which is the entire point of pinning it.
    uint8_t crc[4], cobba[4], hash[4];
    if (!calcul_decode_msid(G.msid, crc, cobba, hash)) {
        fail("pinned MSID has an unknown algorithm selector (want 0x81/0x82/0x83)");
        return 0;
    }
    uint8_t plain[12];
    memcpy(plain, crc, 4); memcpy(plain + 4, cobba, 4); memcpy(plain + 8, hash, 4);
    calcul_ppm_crc(plain);
    if (!calcul_encode(plain, G.msid[0], CALCUL_MODE_FLASH, cobba, G.faid) ||
        !calcul_encode(CALCUL_LOCK1_DEF, G.msid[0], CALCUL_MODE_SIMLOCK, cobba, G.lock1) ||
        !calcul_encode(CALCUL_LOCK2_DEF, G.msid[0], CALCUL_MODE_SIMLOCK, cobba, G.lock2) ||
        !calcul_imei_record(imei14, G.msid[0], cobba, G.imei, G.imei_bcd)) {
        fail("could not build the record set from the pinned identity");
        return 0;
    }

    snprintf(G.imei15, sizeof G.imei15, "%s%d", imei14, calcul_luhn(imei14));
    fprintf(stderr, "[idprov] %s %s: COBBA %02X%02X%02X%02X, IMEI %s%d\n",
            verify_only ? "verifying" : "re-provisioning",
            m->model->name ? m->model->name : "?",
            cobba[0], cobba[1], cobba[2], cobba[3], imei14, calcul_luhn(imei14));
    begin_state(m, ST_PROBE);
    return 1;
}

void identity_provision_tick(struct Mad2* m) {
    if (G.state == ST_IDLE || G.state == ST_DONE || G.state == ST_FAILED) return;
    if (!m) return;

    if (G.state == ST_SETTLE) {
        if (m->rtc_mono >= G.settle_until) {
            if (G.next_state == ST_DONE && G.reset_sent) {
                char b2[200];
                snprintf(b2, sizeof b2, "%s; settings reset (65/08) sent", G.status);
                set_status(b2);
                fprintf(stderr, "[idprov] %s\n", G.status);
                G.state = ST_DONE;
                return;
            }
            begin_state(m, G.next_state);
        }
        return;
    }

    if (m->mbus_ctrl & 0x40) G.rx_ctrl_seen++;
    pump_tx(m);
    pump_rx(m);

    if (G.expect_type && m->rtc_mono > G.deadline) {
        // The settings reset is BEST-EFFORT and must not abort the run. Rolis gets a 65/00
        // back from this same emulated 3350, but only somewhere inside a long session we have
        // not yet reproduced — sending 64/01 or 64/02 immediately before it (Rolis's own
        // predecessor) is not enough on its own. Since 0x65 only touches UI settings, skipping
        // it costs nothing that the identity writes depend on, whereas failing here would
        // throw away a provisioning path that demonstrably works. Reported, never silent.
        char why[160];
        // Report whether the firmware's MBUS receiver ever opened. If it never did, the
        // frames were never delivered and the problem is the phone's state, not the frames.
        snprintf(why, sizeof why,
                 "timed out waiting for the 0x%02X reply (%u byte(s) still staged, "
                 "RX-enable open on %llu step(s))",
                 G.expect, (unsigned)(G.tx_tail - G.tx_head),
                 (unsigned long long)G.rx_ctrl_seen);
        fail(why);
    }
}

int identity_provision_state(void) {
    switch (G.state) {
        case ST_IDLE:   return IDPROV_IDLE;
        case ST_DONE:   return IDPROV_DONE;
        case ST_FAILED: return IDPROV_FAILED;
        default:        return IDPROV_RUNNING;
    }
}

const char* identity_provision_status(void) {
    return G.status[0] ? G.status : "idle";
}

int identity_provision_progress(void) { return G.written; }
