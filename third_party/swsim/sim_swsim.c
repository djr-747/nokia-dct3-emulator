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

static int swsim_ensure(void) {
    if (g_ready) return g_ready > 0 ? 1 : 0;
    const char *fs = getenv("SWSIM_FS");
    if (!fs || !*fs) fs = "third_party/swsim/gsm.json";
    if (swsim_init(&g_sw, &g_icc, fs, "/tmp/dct3_swsim.swiccfs") != 0) {
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
    return dlen;
}
