// swSIM backend (opt-in via SWSIM=1): route the assembled GSM SIM APDU through the
// vendored swICC/swSIM software card (third_party/swicc + third_party/swsim) instead of
// the hand-rolled synthetic EF table in mad2_sim.c. In-process + synchronous (the 3310
// firmware is the T=0 reader; swICC is the card; this file is the wire), so it stays
// deterministic. Card filesystem/identity comes from third_party/swsim/gsm.json (override
// with SWSIM_FS=<path>). Compiled with -std=gnu11 (swICC needs C11 static_assert).
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "swsim.h"
#include <swicc/swicc.h>

static swsim_st g_sw;
static swicc_st g_icc;
static uint8_t  g_rx[512], g_tx[512];
static int      g_ready = 0;
static unsigned g_writes = 0;       // successful card-mutating APDUs (host auto-save trigger)

// Where swsim_init serialises the parsed gsm.json, and where snapshot/restore stage the
// swICC FS image. MEMFS-writable under emscripten (/tmp exists by default).
#define SWSIM_FS_PATH "/tmp/dct3_swsim.swiccfs"

static int swsim_ensure(void) {
    if (g_ready) return g_ready > 0 ? 1 : 0;
    const char *fs = getenv("SWSIM_FS");
    if (!fs || !*fs) fs = "third_party/swsim/gsm.json";
    if (swsim_init(&g_sw, &g_icc, fs, SWSIM_FS_PATH) != 0) {
        fprintf(stderr, "[swsim] init FAILED (fs=%s)\n", fs);
        g_ready = -1; return 0;
    }
    // Leave SAT/proactive OFF: the 3310 sends TERMINAL PROFILE (now handled via the CLA-gate
    // patch in apduh.c → 9000) but does not need proactive commands. Enabling the default
    // proactive app makes swSIM push a command the phone answers with TERMINAL RESPONSE
    // (A0 14), which hits the same GSM-CLA gap and loops. app_default_enable stays 0 (memset).
    g_sw.proactive.app_default_enable = false;
    g_icc.buf_rx = g_rx; g_icc.buf_rx_len = 0;
    g_icc.buf_tx = g_tx; g_icc.buf_tx_len = sizeof(g_tx);
    if (swicc_mock_reset_cold(&g_icc, true) != SWICC_RET_SUCCESS) {
        fprintf(stderr, "[swsim] reset_cold FAILED\n");
        g_ready = -1; return 0;
    }
    fprintf(stderr, "[swsim] card ready (fs=%s)\n", fs);
    g_ready = 1;
    return 1;
}

// Drive one command APDU (header[5] + optional data) through swICC's T=0 io FSM and
// return response data length, filling out[]/sw1/sw2. The response (data + SW) is the
// LAST non-empty card->reader burst; procedure bytes (INS-echo ACK) arrive earlier.
int swsim_backend_apdu(const uint8_t *apdu, int apdu_len,
                       uint8_t *out, int out_cap, uint8_t *sw1, uint8_t *sw2) {
    if (!swsim_ensure()) { *sw1 = 0x6F; *sw2 = 0x00; return 0; }
    int sent = 0;
    uint8_t last[512]; int last_len = 0;
    for (int guard = 0; guard < 128; guard++) {
        uint16_t want = g_icc.buf_rx_len;
        int give = (int)want;
        if (give > apdu_len - sent) give = apdu_len - sent;
        if (give < 0) give = 0;
        if (give == 0 && sent < 5 && apdu_len >= 5) give = 5 - sent; // prime the 5-byte header
        if (give) { memcpy(g_rx, apdu + sent, (size_t)give); sent += give; }
        g_icc.buf_rx = g_rx; g_icc.buf_rx_len = (uint16_t)give;
        g_icc.buf_tx = g_tx; g_icc.buf_tx_len = sizeof(g_tx);
        swicc_io(&g_icc);
        if (g_icc.buf_tx_len) { memcpy(last, g_tx, g_icc.buf_tx_len); last_len = (int)g_icc.buf_tx_len; }
        swicc_fsm_state_et st; swicc_fsm_state(&g_icc, &st);
        if (st == SWICC_FSM_STATE_CMD_WAIT && sent >= apdu_len) break;
        if (give == 0 && g_icc.buf_tx_len == 0) break; // no progress
    }
    if (last_len < 2) { *sw1 = 0x6F; *sw2 = 0x00; return 0; }
    *sw1 = last[last_len - 2];
    *sw2 = last[last_len - 1];
    int dlen = last_len - 2;
    if (dlen > out_cap) dlen = out_cap;
    if (dlen > 0) memcpy(out, last, (size_t)dlen);
    // Card-state mutation counter. Same contract as dct3_web_eeprom_writes(): the host's
    // auto-save only fires when the card ACTUALLY changed this run, so a broken boot can
    // never overwrite a good save. INCREASE (0x32) counts too — it rewrites EF_ACM.
    if (*sw1 == 0x90 && *sw2 == 0x00 && apdu_len >= 2 &&
        (apdu[1] == 0xDC || apdu[1] == 0xD6 || apdu[1] == 0x32))
        g_writes++;
    return dlen;
}

unsigned swsim_backend_writes(void) { return g_writes; }

// ---- Persistence ------------------------------------------------------------------
// The card FS is snapshotted in swICC's OWN on-disk format (swicc_disk_save), not a
// hand-rolled diff: it round-trips through the same loader swsim_init uses when handed
// a .swiccfs instead of the JSON, and it carries a 16-byte magic that encodes both the
// format and the machine endianness — so a stale or foreign blob is rejected by the
// library rather than silently mounted. The staging file is MEMFS on the web, so the
// bytes only ever leave via the pointer returned here.
//
// g_snap is retained until the next snapshot: the caller (JS) copies it out of the wasm
// heap synchronously, so a single reusable buffer is enough and nothing is leaked.
static uint8_t *g_snap = NULL;
static int      g_snap_len = 0;

// Serialise the LIVE card FS. Returns the byte count (0 on failure) and hands back the
// buffer via *buf. Pure read of the mounted disk — safe to call at any point in a run,
// including mid-APDU, because it never touches the T=0 FSM or the current selection.
int swsim_backend_snapshot(uint8_t **buf, int *len) {
    *buf = NULL; *len = 0;
    if (g_ready != 1) return 0;                  // card never came up -> nothing to save
    if (swicc_disk_save(&g_icc.fs.disk, SWSIM_FS_PATH) != SWICC_RET_SUCCESS) {
        fprintf(stderr, "[swsim] snapshot: disk_save FAILED\n");
        return 0;
    }
    FILE *f = fopen(SWSIM_FS_PATH, "rb");
    if (!f) return 0;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return 0; }
    long n = ftell(f);
    if (n <= 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return 0; }
    uint8_t *p = realloc(g_snap, (size_t)n);
    if (!p) { fclose(f); return 0; }
    g_snap = p;
    size_t rd = fread(g_snap, 1, (size_t)n, f);
    fclose(f);
    if (rd != (size_t)n) return 0;
    g_snap_len = (int)n;
    *buf = g_snap; *len = g_snap_len;
    return g_snap_len;
}

// INTERIM — wipe EF_LOCI (6F7E) back to its virgin value on every mount.
//
// This is deliberately UNFAITHFUL: a real SIM keeps its location info across power
// cycles, and that is exactly what makes re-registration fast. It is here because our
// ROM-4 engine (dsp/dsp_rom4.c) implements registration ONLY as the full Location
// Updating transaction (SABM-with-info -> contention resolution -> LU Accept). A card
// carrying LAI 208-01 / LAC 0001 with location-update-status "updated" — which any
// rom6 session writes — makes the firmware skip the LU on the very cell it then finds,
// so a ROM-4 model camps but never registers and shows no operator name.
//
// Before card persistence existed the bug was self-clearing: swSIM builds the card once
// per PROCESS (the g_ready latch), so any page reload rebuilt it from gsm.json. Persisting
// the card would have made it permanent for a returning visitor. Wiping LOCI on mount
// keeps registration behaving exactly as it does with a non-persistent card while the
// contacts and messages on the card still survive.
//
// REMOVE THIS once dsp_rom4.c handles an already-registered SIM (IMSI attach / no-LU
// path). SWSIM_KEEP_LOCI=1 opts out now, for A/B-ing that work.
// Virgin EF_LOCI per gsm.json: TMSI(4) + LAI(5) + TMSI-time(1) + status(1), all 0xFF —
// status 0xFF is not 0 ("updated"), so the firmware treats the card as never-registered.
int swsim_backend_write_ef(uint16_t df, uint16_t fid, const uint8_t *data, int len);
static void swsim_clear_loci(void) {
    if (getenv("SWSIM_KEEP_LOCI")) return;
    static const uint8_t VIRGIN[11] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    // gsm.json carries an EF_LOCI under each of the three application DFs; which one the
    // firmware selects is model-dependent, so clear all three.
    static const uint16_t DFS[3] = {0x7F20, 0x7F21, 0x7F40};
    for (int i = 0; i < 3; ++i)
        (void)swsim_backend_write_ef(DFS[i], 0x6F7E, VIRGIN, (int)sizeof VIRGIN);
}

// Call at the top of every (re)boot. swSIM builds the card ONCE per process, so a card
// that registered during the previous boot still holds that boot's location info — the
// firmware then skips the Location Update and a ROM-4 model camps without registering.
// (This is why "reboot" never fixed it but a page reload did: only a reload rebuilt the
// card.) No-op until the card exists: a first boot builds it from gsm.json, whose LOCI
// is already virgin, and this must never force the card up early.
void swsim_backend_new_boot(void) {
    if (g_ready != 1) return;
    swsim_clear_loci();
}

// Mount a previously snapshotted card FS in place of the one gsm.json would build.
// Must be called BEFORE the firmware's first APDU (the host calls it right after boot):
// an already-running card is torn down first, which would strand any in-flight command.
// On any failure the card is left un-inited so swsim_ensure() rebuilds it from the JSON —
// a corrupt or stale snapshot degrades to a factory-fresh SIM, never to a dead one.
int swsim_backend_restore(const uint8_t *buf, int len) {
    if (!buf || len < 16) return 0;              // shorter than the swICC FS magic
    FILE *f = fopen(SWSIM_FS_PATH, "wb");
    if (!f) return 0;
    size_t wr = fwrite(buf, 1, (size_t)len, f);
    fclose(f);
    if (wr != (size_t)len) return 0;
    if (g_ready == 1) { swicc_terminate(&g_icc); }
    g_ready = 0;
    // path_json = NULL makes swsim_init LOAD the .swiccfs instead of parsing the JSON.
    if (swsim_init(&g_sw, &g_icc, NULL, SWSIM_FS_PATH) != 0) {
        fprintf(stderr, "[swsim] restore FAILED — falling back to a factory card\n");
        return 0;                                // g_ready stays 0 -> rebuilt from JSON
    }
    g_sw.proactive.app_default_enable = false;
    g_icc.buf_rx = g_rx; g_icc.buf_rx_len = 0;
    g_icc.buf_tx = g_tx; g_icc.buf_tx_len = sizeof(g_tx);
    if (swicc_mock_reset_cold(&g_icc, true) != SWICC_RET_SUCCESS) {
        fprintf(stderr, "[swsim] restore: reset_cold FAILED\n");
        swicc_terminate(&g_icc);
        return 0;
    }
    g_ready = 1;
    g_writes = 0;                                // restored state is the new baseline
    swsim_clear_loci();
    fprintf(stderr, "[swsim] card restored from snapshot (%d bytes)\n", len);
    return 1;
}

// Overwrite a transparent EF from offset 0, addressed as DF/EF under the MF. Used by the
// host to re-apply settings that belong to the CONFIGURATION rather than to the card's
// saved user data — EF_SPN (6F46), whose value comes from the page's operator-name knob
// and must therefore win over whatever the restored snapshot happens to carry.
int swsim_backend_write_ef(uint16_t df, uint16_t fid, const uint8_t *data, int len) {
    if (!swsim_ensure() || !data || len <= 0 || len > 255) return 0;
    uint8_t out[512], sw1, sw2;
    uint8_t sel[7] = { 0xA0, 0xA4, 0x00, 0x00, 0x02, 0, 0 };
    const uint16_t path[3] = { 0x3F00, df, fid };
    for (int i = 0; i < 3; ++i) {
        if (!path[i] || (i == 1 && df == 0x3F00)) continue;
        sel[5] = (uint8_t)(path[i] >> 8); sel[6] = (uint8_t)path[i];
        swsim_backend_apdu(sel, 7, out, (int)sizeof out, &sw1, &sw2);
        if (sw1 != 0x9F && sw1 != 0x90) return 0;      // 9F xx = FCP waiting, 90 00 = done
    }
    uint8_t upd[5 + 255];
    upd[0] = 0xA0; upd[1] = 0xD6; upd[2] = 0x00; upd[3] = 0x00; upd[4] = (uint8_t)len;
    memcpy(upd + 5, data, (size_t)len);
    swsim_backend_apdu(upd, 5 + len, out, (int)sizeof out, &sw1, &sw2);
    return (sw1 == 0x90 && sw2 == 0x00) ? 1 : 0;
}
