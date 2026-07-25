// ===========================================================================================
// mad2_dsp_rom4 — the network-capable ROM-4 DSP engine (5110 / 6110 / 3210 / 8810 + the
// 5130/5190/5110i/6130/6150/6190 variants).
// ===========================================================================================
//
// CREDIT — the ROM-4 radio/registration protocol in this file is a faithful C port of the
// reverse-engineering work of **Gareth Davidson (bitplane)**, github.com/bitplane/nokia-dct3-re:
// the DSP packet-semantics census, the DSPIF transport contract
// and above all the organic search / cell-selection / random-access /
// LAPDm / Location-Updating registration contract (+
// driver/nokia_radio_peer.cpp + driver/nokia_gsm_network.cpp), recovered from the 3210 (NSE-8)
// v5.01/v6.00 ROMs and verified in MAME through accepted Location Updating, RR release and
// steady camp. The message ids, payload layouts, laboratory-cell data, transaction phases and
// lifecycle rules here follow that reference; the closed-alternative rules it documents
// (no unsolicited confirmations, no replayed terminals, no firmware-RAM pokes) are honored.
// His evidence also independently established the ~32 s reason-0x68 DSP-liveness watchdog and
// the header-only type-0x03 idle indication that our heartbeat models. Thank you, bitplane.
//
// The boot/codeblock/self-test/keep-alive transport skeleton is shared with the ROM-6 engine
// (dsp_rom6.c, jmacato's contributed reference engine — this file began as a copy of it, per
// the owner's direction). Engine state lives in Mad2 (per-boot memset lifecycle); the engine
// acts ONLY through the DSP hardware interface (mailbox rings, HPI registers, FIQ0/IRQ4
// doorbells) — never through MCU-private RAM.
//
// ROM-4 vs ROM-6 protocol differences applied here (bitplane's census):
//   * carrier search is m2d 0x1A SEARCH_LIST (4-byte control + 512-bit ARFCN set), not 0x56;
//   * d2m adds 0x84 RA_INFO and 0x8C IDLE_RA-completion; 0x8B ALL_RSSI_RESULTS is the
//     166-byte forty-record array; 0x83 is the serving-channel scalar (signed RSSI at [2]);
//   * m2d 0x03 DEACTIVATE retires the receiver and cancels queued work;
//   * the 0x70/0x13..0x18 bootstrap records are ONE-WAY publications (no 0x34/0x35/0x36
//     SIML replies on ROM-4); only {0x70,0x0D} "run self-test" gets its 0x74 {0D 00}
//     completion (census: the sole RX 0x74);
//   * registration follows the multi-round search transaction machine below, ending in the
//     laboratory cell: GSM 900 ARFCN 1, BSIC 0x12, reserved test PLMN 001-01, LAC 1, CID 1
//     (matches the ROM-4 models' lock-exempt test IMSI 001-01 — deliberately NOT the rom6
//     engine's REFSIM identity, which would trip handset network-lock provisioning).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mad2/mad2.h"
#include "models/model.h"
#include "mad2/dsp/dsp_rom4.h"
#include "mad2/dsp/mdi.h"

// Single build/destroy point. The core memset already zeroes m->rom4 on init/reset.
void rom4_reset(struct Mad2* m) {
    m->rom4 = (Rom4Dsp){0};
}

static int rom4_log_enabled(void) { return getenv("ROM4_LOG") != 0; }

// ===========================================================================================
// GSM frame/time model. One TDMA frame = 4.615 ms; the emulated clock runs ~13 MHz (rtc_mono
// cycles), so cycles-per-frame defaults to 60000 (ROM4_FNDIV overrides for A/B).
// ===========================================================================================
#define ROM4_CYCLES_PER_FRAME 60000u
#define ROM4_HYPERFRAME       (26u * 51u * 2048u)   // 2,715,648

static uint64_t rom4_cpf(void) {
    static uint64_t v = 0;
    if (!v) { const char* e = getenv("ROM4_FNDIV");
              v = (e && *e) ? strtoull(e, 0, 0) : ROM4_CYCLES_PER_FRAME;
              if (!v) v = ROM4_CYCLES_PER_FRAME; }
    return v;
}
static uint64_t rom4_now(const Rom4Dsp* r) { return r->currentCycles; }
static uint32_t rom4_frame_at(uint64_t cycles) {
    return (uint32_t)((cycles / rom4_cpf()) % ROM4_HYPERFRAME);
}
static uint32_t rom4_fn_now(const struct Mad2* m) {
    return rom4_frame_at(m->rtc_mono);
}

// Unified packet trace (ROM4_PKT=1) — one line per DSP packet, format aligned with bitplane's
// MAME DSPIF verbose log (nokia_dspif.cpp "TX consume" / "RX enqueue") so the two can be diffed:
// direction, type, payload length, GSM frame number, emulated-time stamp, full payload hex.
// "m2d" = MCU->DSP (our observed MDISND request, == MAME's TX consume); "d2m" = DSP->MCU (our
// synthesized MDIRCV report, == MAME's RX enqueue). Time is emulated seconds so it lines up
// with MAME's t=<sec> (both count from boot; absolute values differ, cadence + payloads compare).
static void rom4_pkt_log(struct Mad2* m, const char* dir, uint8_t type,
                         const uint8_t* payload, unsigned len) {
    if (!getenv("ROM4_PKT")) return;
    double t = (double)m->rtc_mono / 13000000.0;   // ~13 MHz rtc_mono -> seconds
    fprintf(stderr, "[rom4 pkt] %s type=%02x payload=%u fn=%u step=%llu t=%.6f data=",
            dir, type, len, rom4_fn_now(m), (unsigned long long)m->dsp_steps, t);
    for (unsigned i = 0; i < len; ++i) fprintf(stderr, "%02x", payload[i]);
    fprintf(stderr, "\n");
}

// ===========================================================================================
// Laboratory cell (bitplane's verified network data — nokia_gsm_network.cpp).
// Minimum broadcast set for a GSM 900 cell on ARFCN 1, BSIC 0x12, reserved test PLMN 001-01,
// LAC 1, cell ID 1. SI1's Cell Channel Description is GSM bitmap-0; ARFCN 1 is bit 0 of the
// final octet of that 16-octet field.
// ===========================================================================================
#define ROM4_BSIC   0x12u
#define ROM4_ARFCN  0x0001u

static const uint8_t ROM4_SI[4][24] = {
    { 0x55, 0x06, 0x19,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01,
      0, 0, 0, 0x2b, 0 },                                        // SI1
    { 0x59, 0x06, 0x1a, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0xff, 0, 0, 0, 0 },                   // SI2
    { 0x49, 0x06, 0x1b, 0x00, 0x01, 0x00, 0xf1, 0x10,
      0x00, 0x01, 0x40, 0, 0, 0, 0, 0, 0, 0, 0,
      0x2b, 0x2b, 0x2b, 0x2b, 0x2b },                            // SI3 (LAC 1, PLMN 001-01, CI 1)
    { 0x31, 0x06, 0x1c, 0x00, 0xf1, 0x10, 0x00, 0x01,
      0, 0, 0, 0, 0, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
      0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b },                      // SI4
};
// TS 45.002 eight-multiframe BCCH schedule: SI by TC slot. SI1 broadcast at TC 0 even though
// optional for this non-hopping cell — the ROM requires the real block when revalidating an
// already-active serving channel.
static const uint8_t ROM4_SI_BY_TC[8] = { 0, 1, 2, 3, 1, 1, 2, 3 };

// A real receiver never returns an identical RSSI forever: the ROM retains the previous sample
// and requires a strict improvement before promoting a background measurement to a usable
// candidate. Deterministic measured-signal variation around -60 dBm.
static int8_t rom4_serving_rssi(unsigned sample) {
    static const int8_t pattern[4] = { -60, -61, -59, -60 };
    return pattern[sample & 3u];
}

// Replace an SI block's LAI PLMN template with the ACTIVE SIM's home PLMN, so the phone
// recognizes our synthetic cell as its OWN network. bitplane's lab used the reserved test
// PLMN 001-01 hardcoded in the SI/LU-Accept, which only matches a 001-01 card; a real /
// swSIM card advertises its own MCC/MNC (e.g. 208-01), so a fixed 001-01 broadcast leaves the
// handset searching for a home network it never sees — it camps but never registers. Only the
// PLMN follows the SIM; the LAC, cell id and everything else stay bitplane's laboratory values.
// (This is the same fix the rom6 engine applies via mad2_sim_current_plmn.)
static void rom4_apply_si_plmn(const struct Mad2* m, uint8_t* si24) {
    uint8_t plmn[3];
    mad2_sim_current_plmn(m, plmn);
    if (si24[0] == 0x49) memcpy(si24 + 5, plmn, 3);       // SI3: LAI PLMN at [5..7]
    else if (si24[0] == 0x31) memcpy(si24 + 3, plmn, 3);  // SI4: LAI PLMN at [3..5]
}

// ===========================================================================================
// MDIRCV egress — single packet at a time. Two software queues front the one hardware ring:
// pending (FIFO) + delayed (time-ordered). The pump posts ONE record via mdi_d2m_deposit ONLY
// when the ring is empty, then raises FIQ0. Stale unposted records (> 2 s) are dropped.
// ===========================================================================================
static void rom4_enqueue(struct Mad2* m, uint8_t op, const uint8_t* payload, uint8_t len) {
    Rom4Dsp* r = &m->rom4;
    uint8_t nt = (uint8_t)((r->p_tail + 1u) % ROM4_PENDING_N);
    if (nt == r->p_head) return;                    // FIFO full: drop (real overflow)
    Rom4MdiRec* rec = &r->pending[r->p_tail];
    rec->op = op; rec->len = len;
    if (len > ROM4_RCVMAX) len = ROM4_RCVMAX, rec->len = len;
    for (uint8_t i = 0; i < len; ++i) rec->bytes[i] = payload ? payload[i] : 0u;
    rec->enq = rom4_now(r); rec->due = rom4_now(r);
    r->p_tail = nt;
}

// PumpDelayedMdiRcv: matured delayed records move into the FIFO in due order.
static void rom4_pump_delayed(struct Mad2* m, uint64_t cycles) {
    Rom4Dsp* r = &m->rom4;
    for (unsigned i = 0; i < ROM4_DELAYED_N; ++i) {
        Rom4MdiRec* rec = &r->delayed[i];
        if (rec->used && cycles >= rec->due) {
            rom4_enqueue(m, rec->op, rec->bytes, rec->len);
            rec->used = 0;
        }
    }
}

// ExpireStale: drop FIFO records that have waited unposted longer than 2 s.
static void rom4_expire_stale(struct Mad2* m, uint64_t cycles) {
    Rom4Dsp* r = &m->rom4;
    uint64_t ttl = rom4_cpf() * 217u * 2u;          // 2 seconds
    while (r->p_head != r->p_tail) {
        Rom4MdiRec* rec = &r->pending[r->p_head];
        if (cycles - rec->enq <= ttl) break;        // FIFO is roughly time-ordered
        r->p_head = (uint8_t)((r->p_head + 1u) % ROM4_PENDING_N);
    }
}

#define ROM4_KA_CYC 25000000u   // idle-telemetry cadence, ~5 s (see rom4_idle_heartbeat)

// PumpMdiRcv: post ONE record from the FIFO head into the (empty) hardware ring + FIQ0.
static void rom4_pump_mdircv(struct Mad2* m) {
    Rom4Dsp* r = &m->rom4;
    if (r->p_head == r->p_tail) return;             // FIFO empty
    Rom4MdiRec* rec = &r->pending[r->p_head];
    if (mdi_d2m_deposit(m->mem, m->mem_mask, m->fw.mdircv_q, m->fw.mdircv_tail,
                        m->fw.mdircv_head, rec->op, rec->bytes, rec->len)) {
        mad2_raise_fiq(m, 0);
        r->p_head = (uint8_t)((r->p_head + 1u) % ROM4_PENDING_N);
        // Real d2m traffic feeds the firmware's MDI-activity counter — re-pace the idle
        // telemetry heartbeat so it only ever fills genuine DSP silence.
        m->dsp_hb_next_cyc = m->rtc_mono + ROM4_KA_CYC;
    }
}

// Perpetual idle MDI telemetry — the real DSP is NEVER silent. bitplane's 3210 evidence
//: a header-only type-0x03 packet is the DSP idle/liveness indication
// the firmware consumes as MDI activity; without periodic DSP activity the firmware enters its
// independent reason-0x68 DSP-watchdog reset after roughly 32 seconds. Same mechanism as the
// ROM-6 0xE4 watchdog (essay in dsp/dsp_rom6.c / the keep-alive block in dsp_mailbox.c).
// NOTE the per-model nuance: the 5110's cosim grounding showed a silent ROM-4 DSP with no
// watchdog — its reason-0x68 is the EEPROM-fault latch — while the 3210 demonstrably HAS the
// watchdog. Emitting the idle indication is the faithful behaviour for the family: a model
// without the watchdog simply consumes the telemetry.
static void rom4_idle_heartbeat(struct Mad2* m) {
    Rom4Dsp* r = &m->rom4;
    int protocol_ready = m->dsp_running &&
        (m->dsp_selftest_replied || m->dsp_selftest_off);
    if (!protocol_ready) { m->dsp_hb_next_cyc = 0; return; }
    if (!m->dsp_hb_next_cyc) { m->dsp_hb_next_cyc = m->rtc_mono + ROM4_KA_CYC; return; }
    if (m->rtc_mono < m->dsp_hb_next_cyc) return;
    if (r->p_head != r->p_tail) return;      // engine has real traffic in flight: it will re-pace
    uint32_t hp = m->fw.mdircv_head & m->mem_mask;
    uint32_t tp = m->fw.mdircv_tail & m->mem_mask;
    uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
    uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
    if (head != tail || head < 0x80) return; // never tramples a live ring entry; retry next tick
    uint32_t q   = m->fw.mdircv_q & m->mem_mask;
    uint32_t off = (uint32_t)(tail - 0x80) * 2u;
    m->mem[q + off]      = 0x03;             // BE even byte = DSP->MCU group 0x03 (idle telemetry)
    m->mem[q + off + 1u] = 0x00;
    uint16_t nt = (uint16_t)(tail + 1u);     // 1 word, no payload
    if (nt >= 0xE4) nt = 0x80;               // ring END -> START wrap, as the firmware's consumer does
    m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1u] = (uint8_t)nt;
    mad2_raise_fiq(m, 0);
    m->dsp_hb_last = m->rtc_mono;
    m->dsp_hb_next_cyc = m->rtc_mono + ROM4_KA_CYC;
    m->dsp_hb_pulses++;
    // Log the idle telemetry beat too, so ROM4_PKT shows the DSP is ALIVE during quiet radio
    // periods (this group-0x03 word is deposited straight into the ring, not via the report
    // path — without this line the trace looks silent when the keep-alive is actually beating).
    { const uint8_t beat[1] = { 0x00 }; rom4_pkt_log(m, "d2m", 0x03, beat, 1); }
}

// DEACTIVATE is one-way and cancels queued work for the retired receiver (lifecycle rule).
// Drop queued radio reports; the 0x74 self-test completion is never radio work.
static void rom4_drop_radio_backlog(Rom4Dsp* r) {
    Rom4MdiRec keep[ROM4_PENDING_N];
    uint8_t count = 0;
    for (uint8_t i = r->p_head; i != r->p_tail; i = (uint8_t)((i + 1u) % ROM4_PENDING_N)) {
        Rom4MdiRec* rec = &r->pending[i];
        int radio = rec->op == 0x80u || rec->op == 0x83u || rec->op == 0x84u ||
                    rec->op == 0x86u || rec->op == 0x87u || rec->op == 0x89u ||
                    rec->op == 0x8Bu || rec->op == 0x8Cu || rec->op == 0x8Fu;
        if (!radio && count + 1u < ROM4_PENDING_N) keep[count++] = *rec;
    }
    for (uint8_t i = 0; i < count; ++i) r->pending[i] = keep[i];
    r->p_head = 0; r->p_tail = count;
    for (unsigned i = 0; i < ROM4_DELAYED_N; ++i) r->delayed[i].used = 0;
}

// ===========================================================================================
// The radio peer — bitplane's recovered ROM-4 transaction machine, ported 1:1.
// Reports are emitted one per pacing deadline (default one TDMA frame apart); the recovered
// 51-frame-multiframe cadence paces BCCH/RSSI streams; a receive-triggered transition defers
// its first report to the next frame (the reference's report_deferred).
// ===========================================================================================
static const char* rom4_rp_name(uint8_t p) {
    static const char* NAMES[ROM4_RP_COUNT] = {
        "inactive", "initial_search", "post_deactivate_search",
        "candidate_measurement", "candidate_sync", "candidate_channel_change",
        "candidate_ra_info", "serving_bcch", "serving_idle_ra", "candidate_retry",
        "selected_search", "serving_channel_change", "selected_channel_change",
        "selected_bcch", "selected_ra_info", "selected_bcch_channel_change",
        "random_access", "assigned_channel_change", "lapdm_establish",
        "contention_resolution", "location_update_accept", "rr_channel_release",
        "release_deconfigure", "release_channel_change"
    };
    return p < ROM4_RP_COUNT ? NAMES[p] : "invalid";
}

static void rom4_rp_defer(struct Mad2* m, uint64_t frames) {
    m->rom4.nextReportCycle = m->rtc_mono + frames * rom4_cpf();
}

// Which report the current phase emits next. 0xff marks phases whose report depends on request
// data or position within a correlated multi-report transaction.
static uint8_t rom4_next_report_type(const Rom4Dsp* r) {
    static const uint8_t FIXED[ROM4_RP_COUNT] = {
        0x87, 0x87, 0x87, 0x8b, 0xff, 0xff, 0x84, 0xff,
        0x8c, 0xff, 0xff, 0x89, 0x89, 0xff, 0x84, 0x89,
        0xff, 0x89, 0x86, 0x80, 0x80, 0x80, 0x87, 0x89
    };
    const uint8_t fixed = r->radioPhase < ROM4_RP_COUNT ? FIXED[r->radioPhase] : 0x87;
    if (fixed != 0xff) return fixed;
    switch (r->radioPhase) {
    case ROM4_RP_CANDIDATE_SYNC:
        return r->reportsRemaining == 2 ? 0x8b : 0x80;
    case ROM4_RP_CANDIDATE_CHANNEL_CHANGE:
        return r->reportsRemaining == 2 ? 0x8f : 0x89;
    case ROM4_RP_SERVING_BCCH:
        return (r->reportsRemaining & 1u) == 0 ? 0x80 : 0x83;
    case ROM4_RP_CANDIDATE_RETRY:
        return r->reportsRemaining == 2 ? 0x8b : 0x87;
    case ROM4_RP_SELECTED_SEARCH:
        // The second selected-search report is either the serving cell's SCH block (0x80,
        // "still there") or the empty terminal (0x87, NO_BCCH_LEFT). bitplane's 3210 recovery
        // keyed that purely on the SEARCH_LIST control byte, because on the 3210 the post-camp
        // verification request always carries mode 0x40/0x50 (or mode 0x00 with ARFCN 1
        // explicitly in the 512-bit set). Other ROM-4 handsets issue the SAME request at the
        // SAME point in the transaction with control byte 0x00 and an ALL-ZERO ARFCN bitmap
        // (measured: 7110 v5.00 `00 81 98 00` + 64 zero bytes, vs the 3210's `40 81 98 00` +
        // 64 zero bytes) — byte-identical apart from that one flag. The physical invariant is
        // not the flag but the position: a search issued while the receiver is CAMPED on a
        // serving cell (selFromServing) is a re-verification of that cell, and a real receiver
        // that still has the carrier reports its SCH block. Answering NO_BCCH_LEFT there tells
        // the firmware the cell it just selected has vanished; it then DEACTIVATEs and the
        // selection restarts from scratch.
        return r->reportsRemaining == 2 ? 0x8b :
                ((r->selFromServing ||
                  (r->searchMode == 0x00 && r->searchHasArfcn1) ||
                  r->searchMode == 0x40 || r->searchMode == 0x50) ? 0x80 : 0x87);
    case ROM4_RP_SELECTED_BCCH:
        return r->reportsRemaining == 1 ? 0x87 :
                ((r->reportsRemaining & 1u) == 0 ? 0x83 : 0x80);
    case ROM4_RP_RANDOM_ACCESS:
        return r->reportsRemaining == 3 ? 0x8c :
                (r->reportsRemaining == 2 ? 0x84 : 0x80);
    default:
        return 0x87;
    }
}

// Post-emission transitions and pacing (the reference's advance_after_report; its wait_ticks
// 59 is the 51-frame-multiframe cadence, its 100 the RA_INFO settling delay — both expressed
// here in TDMA frames of emulated time).
static void rom4_advance_after_report(struct Mad2* m, uint8_t report_type) {
    Rom4Dsp* r = &m->rom4;
    switch (r->radioPhase) {
    case ROM4_RP_CANDIDATE_CHANNEL_CHANGE:
        if (report_type == 0x89) { r->radioPhase = ROM4_RP_CANDIDATE_RA_INFO;
                                   r->reportsRemaining = 1; rom4_rp_defer(m, 100); }
        break;
    case ROM4_RP_CANDIDATE_RA_INFO:
        if (report_type == 0x84) { r->radioPhase = ROM4_RP_SERVING_BCCH;
                                   r->reportsRemaining = 8; rom4_rp_defer(m, 51); }
        break;
    case ROM4_RP_SERVING_BCCH:
        if (report_type == 0x80) rom4_rp_defer(m, 51);   // multiframe pacing; 0x83 follows
        else if (report_type == 0x83 && r->reportsRemaining == 0)
            r->reportsRemaining = 8;                     // a camped cell broadcasts forever
        break;
    case ROM4_RP_SELECTED_SEARCH:
        if (report_type == 0x87 || report_type == 0x8f) {
            r->radioPhase = ROM4_RP_SERVING_BCCH;
            r->reportsRemaining = 8; rom4_rp_defer(m, 51);
        }
        break;
    case ROM4_RP_SERVING_IDLE_RA:
        if (report_type == 0x8c) { r->radioPhase = ROM4_RP_SERVING_BCCH;
                                   r->reportsRemaining = 8; rom4_rp_defer(m, 51); }
        break;
    case ROM4_RP_SERVING_CHANNEL_CHANGE:
        if (report_type == 0x89) { r->radioPhase = ROM4_RP_SERVING_BCCH;
                                   r->reportsRemaining = 8; rom4_rp_defer(m, 51); }
        break;
    case ROM4_RP_SELECTED_CHANNEL_CHANGE:
        if (report_type == 0x89) { r->radioPhase = ROM4_RP_SELECTED_RA_INFO;
                                   r->reportsRemaining = 1; rom4_rp_defer(m, 100); }
        break;
    case ROM4_RP_SELECTED_RA_INFO:
        if (report_type == 0x84) {
            // Validate the selected cell across one complete eight-multiframe BCCH schedule,
            // each block followed by its serving-channel RSSI result. A usable cell does not
            // also produce NO_BCCH_LEFT (contradictory-terminal lifecycle rule).
            r->radioPhase = ROM4_RP_SELECTED_BCCH;
            r->reportsRemaining = 16; rom4_rp_defer(m, 51);
        }
        break;
    case ROM4_RP_SELECTED_BCCH:
        if (report_type == 0x80) rom4_rp_defer(m, 51);
        else if (report_type == 0x83) {
            if (r->reportsRemaining == 0) { r->radioPhase = ROM4_RP_SERVING_BCCH;
                                            r->reportsRemaining = 8; rom4_rp_defer(m, 51); }
        } else if (report_type == 0x87) {
            r->radioPhase = ROM4_RP_SERVING_BCCH;
            r->reportsRemaining = 8; rom4_rp_defer(m, 51);
        }
        break;
    case ROM4_RP_SELECTED_BCCH_CHANNEL_CHANGE:
        if (report_type == 0x89) {
            // The accepted logical-channel change retires the selected-cell scan; replaying
            // the pre-change terminal would make the firmware's next SEARCH_LIST lose
            // ownership and restart selection indefinitely.
            r->radioPhase = ROM4_RP_SERVING_BCCH;
            r->reportsRemaining = 8; r->selectedReportsRemaining = 0; rom4_rp_defer(m, 51);
        }
        break;
    case ROM4_RP_RANDOM_ACCESS:
        if (report_type == 0x80)
            r->reportsRemaining = 0;   // further progress is firmware-owned (SDCCH config + SABM)
        break;
    case ROM4_RP_ASSIGNED_CHANNEL_CHANGE:
        if (report_type == 0x89) { r->radioPhase = ROM4_RP_LAPDM_ESTABLISH;
                                   r->reportsRemaining = 1; rom4_rp_defer(m, 1); }
        break;
    case ROM4_RP_LAPDM_ESTABLISH:
        if (report_type == 0x86) r->reportsRemaining = 0;
        break;
    case ROM4_RP_CONTENTION_RESOLUTION:
        if (report_type == 0x80) { r->radioPhase = ROM4_RP_LOCATION_UPDATE_ACCEPT;
                                   r->reportsRemaining = 1; rom4_rp_defer(m, 1); }
        break;
    case ROM4_RP_LOCATION_UPDATE_ACCEPT:
        if (report_type == 0x80) { r->radioPhase = ROM4_RP_RR_CHANNEL_RELEASE;
                                   r->reportsRemaining = 1; }
        break;
    case ROM4_RP_RR_CHANNEL_RELEASE:
        if (report_type == 0x80) {
            // Firmware owns the LAPDm disconnect and physical-channel teardown which follow.
            r->radioPhase = ROM4_RP_RELEASE_DECONFIGURE;
            r->reportsRemaining = 0;
        }
        break;
    case ROM4_RP_RELEASE_CHANNEL_CHANGE:
        if (report_type == 0x89) { r->radioPhase = ROM4_RP_SERVING_BCCH;
                                   r->reportsRemaining = 8; rom4_rp_defer(m, 51); }
        break;
    default:
        break;
    }
}

// Emit ONE peer report (the reference's emit_report, payloads byte-for-byte).
static void rom4_emit_report(struct Mad2* m) {
    Rom4Dsp* r = &m->rom4;
    uint8_t payload[166];
    memset(payload, 0, sizeof payload);
    const uint8_t report_type = rom4_next_report_type(r);
    if (r->radioPhase == ROM4_RP_LAPDM_ESTABLISH)
        payload[0] = 0x80;   // BLOCK_REQUEST subtype accepted in controller state 6

    if (report_type == 0x8b) {
        // ALL_RSSI_RESULTS: two-byte list header + forty 4-byte records (BE ARFCN, flags,
        // signed RSSI). Only ARFCN 1 exists; two -109 dBm baselines establish the initial
        // acquisition history, then the deterministic signal model varies the measurement.
        payload[0] = 0x00; payload[1] = 0x10;
        for (unsigned i = 0; i < 40; ++i) {
            const int serving = i == 0;
            payload[2 + i * 4] = serving ? 0x00 : 0xff;
            payload[3 + i * 4] = serving ? 0x01 : 0xff;
            payload[5 + i * 4] = serving
                ? (r->searchRound < 2 ? (uint8_t)0x93
                                      : (uint8_t)rom4_serving_rssi(r->searchRound - 2u))
                : 0x81;
        }
    }

    if (report_type == 0x80) {
        // RECEIVED_BLOCK: channel, BSIC, error, frame number, ARFCN, shift, then a 24-byte
        // GSM L2 block at +10. Channel 0x40 = SCH, 0x50 = BCCH, 0x60 = CCCH (the Immediate
        // Assignment), 0x80 = the assigned SDCCH.
        payload[0] = (r->radioPhase >= ROM4_RP_CONTENTION_RESOLUTION &&
                      r->radioPhase <= ROM4_RP_RR_CHANNEL_RELEASE) ? 0x80 :
                     r->radioPhase == ROM4_RP_RANDOM_ACCESS ? 0x60 :
                     ((r->radioPhase < ROM4_RP_CANDIDATE_CHANNEL_CHANGE ||
                       r->radioPhase == ROM4_RP_SELECTED_SEARCH) ? 0x40 : 0x50);
        payload[1] = ROM4_BSIC;
        const uint32_t fn = r->radioPhase == ROM4_RP_RANDOM_ACCESS ? r->accessFrame
                                                                   : rom4_fn_now(m);
        payload[3] = (uint8_t)(fn >> 16);
        payload[4] = (uint8_t)(fn >> 8);
        payload[5] = (uint8_t)fn;
        payload[6] = (uint8_t)(ROM4_ARFCN >> 8);
        payload[7] = (uint8_t)ROM4_ARFCN;

        uint8_t* block = payload + 10;
        if (r->radioPhase == ROM4_RP_CONTENTION_RESOLUTION) {
            // UA with final bit set, echoing the SABM's information field exactly.
            memset(block, 0x2b, 24);
            block[0] = 0x01;
            block[1] = 0x73;
            block[2] = (uint8_t)((r->contentionLen << 2) | 1u);
            memcpy(block + 3, r->contentionL3, r->contentionLen);
        } else if (r->radioPhase == ROM4_RP_LOCATION_UPDATE_ACCEPT) {
            // GSM 04.08 9.2.13: MM header + LAI; with no allocated TMSI the network includes
            // the IMSI mobile-identity IE (copied from the LU Request) so the phone discards
            // any stale TMSI.
            uint8_t accept[17] = { 0x05, 0x02, 0x00, 0xf1, 0x10, 0x00, 0x01,
                                   0x17, 0x08, 0, 0, 0, 0, 0, 0, 0, 0 };
            mad2_sim_current_plmn(m, accept + 2);   // LAI PLMN from the active SIM (see rom4_apply_si_plmn)
            if (r->contentionLen >= 18 && r->contentionL3[9] == 8)
                memcpy(accept + 9, r->contentionL3 + 10, 8);
            memset(block, 0x2b, 24);
            block[0] = 0x03;                     // network-to-mobile command, SAPI 0
            block[1] = 0x00;                     // I frame, N(S)=0, N(R)=0
            block[2] = (uint8_t)((sizeof accept << 2) | 1u);
            memcpy(block + 3, accept, sizeof accept);
        } else if (r->radioPhase == ROM4_RP_RR_CHANNEL_RELEASE) {
            // GSM 04.08 9.1.7 RR Channel Release, cause 0 "normal event".
            static const uint8_t release[3] = { 0x06, 0x0d, 0x00 };
            memset(block, 0x2b, 24);
            block[0] = 0x03;
            block[1] = 0x02;                     // I frame, N(S)=1, N(R)=0
            block[2] = (uint8_t)((sizeof release << 2) | 1u);
            memcpy(block + 3, release, sizeof release);
        } else if (payload[0] == 0x60) {
            // GSM 04.08 9.1.18 Immediate Assignment: SDCCH/8 subchannel 0, timeslot 0 on the
            // non-hopping serving carrier; echo the exact RA octet + reception frame.
            const uint8_t t1p = (uint8_t)((fn / 1326u) & 0x1fu);
            const uint8_t t2  = (uint8_t)(fn % 26u);
            const uint8_t t3  = (uint8_t)(fn % 51u);
            const uint8_t ia[24] = {
                0x2d, 0x06, 0x3f, 0x00,
                0x20, 0x00, 0x01,
                r->accessRa, (uint8_t)((t1p << 3) | (t3 >> 3)), (uint8_t)((t3 << 5) | t2),
                0x00, 0x00,
                0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
                0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b
            };
            memcpy(block, ia, 24);
        } else if (payload[0] == 0x50) {
            memcpy(block, ROM4_SI[ROM4_SI_BY_TC[(fn / 51u) & 7u]], 24);
            rom4_apply_si_plmn(m, block);        // SI3/SI4 LAI PLMN follows the active SIM
        } else {
            memcpy(block, ROM4_SI[2], 24);       // SCH-phase filler block, as the reference
            rom4_apply_si_plmn(m, block);
        }
    }

    if (report_type == 0x83) {
        // Serving-channel scalar RSSI, distinct from the SEARCH_LIST result array.
        payload[2] = (uint8_t)rom4_serving_rssi(r->searchRound);
    }

    if (report_type == 0x84 && r->radioPhase == ROM4_RP_RANDOM_ACCESS) {
        // RA_INFO: the DSP's report of the transmitted random-access burst. Task 10 converts
        // the absolute transmit frame into the request-reference tuple which task 16 later
        // matches against the Immediate Assignment.
        r->accessFrame = rom4_fn_now(m);
        payload[0] = r->accessRa;
        payload[1] = (uint8_t)(r->accessFrame >> 16);
        payload[2] = (uint8_t)(r->accessFrame >> 8);
        payload[3] = (uint8_t)r->accessFrame;
    }

    const uint8_t payload_length = report_type == 0x8b ? 166 : report_type == 0x80 ? 34 : 8;
    rom4_pkt_log(m, "d2m", report_type, payload, payload_length);
    rom4_enqueue(m, report_type, payload, payload_length);

    if (report_type == 0x8b && r->radioPhase >= ROM4_RP_CANDIDATE_MEASUREMENT)
        ++r->searchRound;
    --r->reportsRemaining;
    if (rom4_log_enabled())
        fprintf(stderr, "[rom4] report 0x%02X phase=%s remaining=%u @step=%llu\n",
                report_type, rom4_rp_name(r->radioPhase), r->reportsRemaining,
                (unsigned long long)m->dsp_steps);
    uint64_t before = r->nextReportCycle;
    rom4_advance_after_report(m, report_type);
    if (r->nextReportCycle == before)
        rom4_rp_defer(m, 1);                    // default: one report per TDMA frame
    rom4_pump_mdircv(m);
}

// The scheduler pump: emit the next paced report when its deadline matures.
static void rom4_rp_pump(struct Mad2* m) {
    Rom4Dsp* r = &m->rom4;
    if (r->radioPhase == ROM4_RP_INACTIVE || r->reportsRemaining == 0) return;
    if (m->rtc_mono < r->nextReportCycle) return;
    rom4_emit_report(m);
}

// ===========================================================================================
// m2d request dispatch — the recovered receive-side transitions, ported 1:1.
// ===========================================================================================
static void rom4_handle_mdi(struct Mad2* m, uint8_t op, const uint8_t* buf, unsigned plen) {
    Rom4Dsp* r = &m->rom4;
    const int log = rom4_log_enabled();

    if (op == 0x1a && plen != 0) {
        r->searchMode = buf[0];
        // SEARCH_LIST carries a 512-bit ARFCN set after its four-byte control header; in the
        // ROM-4 wire layout ARFCN 1 is bit 0 of payload byte 65.
        r->searchHasArfcn1 = plen > 65 && (buf[65] & 1u);
        if (log) fprintf(stderr, "[rom4] 0x1A SEARCH_LIST mode=0x%02X arfcn1=%u phase=%s @step=%llu\n",
                         r->searchMode, r->searchHasArfcn1, rom4_rp_name(r->radioPhase),
                         (unsigned long long)m->dsp_steps);
    }

    if (op == 0x1a && r->radioPhase == ROM4_RP_INACTIVE && r->reportsRemaining == 0) {
        // Initial SEARCH_LIST attempts end empty; the firmware narrows its own channel bitmap
        // and publishes the next request.
        r->radioPhase = ROM4_RP_INITIAL_SEARCH;
        r->reportsRemaining = 2; rom4_rp_defer(m, 1);
    }
    else if (op == 0x03 && r->radioPhase == ROM4_RP_INITIAL_SEARCH && r->reportsRemaining == 0) {
        r->radioPhase = ROM4_RP_POST_DEACTIVATE_SEARCH;
        r->reportsRemaining = 2; rom4_rp_defer(m, 1);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_INITIAL_SEARCH && r->reportsRemaining == 0) {
        // A SIM with cached EF_BCCH advances directly to the next bounded search mode instead
        // of deactivating the empty initial scan; same two-terminal completion contract.
        r->radioPhase = ROM4_RP_POST_DEACTIVATE_SEARCH;
        r->reportsRemaining = 2; rom4_rp_defer(m, 1);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_POST_DEACTIVATE_SEARCH && r->reportsRemaining == 0) {
        r->radioPhase = ROM4_RP_CANDIDATE_MEASUREMENT;
        r->reportsRemaining = 1; rom4_rp_defer(m, 1);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_CANDIDATE_MEASUREMENT && r->reportsRemaining == 0) {
        if (r->searchRound >= 3) {
            r->radioPhase = ROM4_RP_CANDIDATE_SYNC;
            r->reportsRemaining = 2; rom4_rp_defer(m, 1);
        } else {
            r->reportsRemaining = 1; rom4_rp_defer(m, 1);
        }
    }
    else if (op == 0x02 &&
             (r->radioPhase == ROM4_RP_CANDIDATE_SYNC || r->radioPhase == ROM4_RP_SELECTED_SEARCH) &&
             r->reportsRemaining == 0) {
        // SCH reception makes the ROM issue CHANNEL_CONFIGURE during both initial acquisition
        // and the later mode-0x40 selection pass; complete the same recovered channel-change
        // transaction while its acceptance window is open.
        const int selected_plmn_search =
            r->radioPhase == ROM4_RP_SELECTED_SEARCH && r->searchMode == 0x50;
        r->radioPhase = selected_plmn_search ? ROM4_RP_SELECTED_CHANNEL_CHANGE
                                             : ROM4_RP_CANDIDATE_CHANNEL_CHANGE;
        r->reportsRemaining = selected_plmn_search ? 1 : 2;
        rom4_rp_defer(m, 1);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_CANDIDATE_SYNC && r->reportsRemaining == 0) {
        // The MCU rejected the measured candidate and requests the next search batch; close
        // that finite scan with the recovered empty-list terminal so MM can select its fallback.
        r->radioPhase = ROM4_RP_CANDIDATE_RETRY;
        r->reportsRemaining = 2; rom4_rp_defer(m, 1);
    }
    else if (op == 0x0c && r->radioPhase == ROM4_RP_SERVING_BCCH) {
        // IDLE_RA form 1 configures the serving-cell receiver; form 0 carries a CHANNEL
        // REQUEST random-access octet at byte 2. Complete the former normally; only the
        // latter starts the network access exchange.
        const int channel_request = plen >= 3 && buf[1] == 0 && buf[2] != 0;
        r->accessRa = channel_request ? buf[2] : 0;
        r->accessFrame = 0;
        r->radioPhase = channel_request ? ROM4_RP_RANDOM_ACCESS : ROM4_RP_SERVING_IDLE_RA;
        r->reportsRemaining = channel_request ? 3 : 1;
        rom4_rp_defer(m, 1);
        if (log) fprintf(stderr, "[rom4] 0x0C IDLE_RA %s RA=0x%02X @step=%llu\n",
                         channel_request ? "CHANNEL-REQUEST" : "receiver-config",
                         r->accessRa, (unsigned long long)m->dsp_steps);
    }
    else if (op == 0x02 && r->radioPhase == ROM4_RP_SERVING_BCCH &&
             plen >= 9 && buf[8] == 0x60) {
        // After serving-cell selection the ROM configures logical channel 0x12, encoded as
        // DSP receive channel 0x60.
        r->radioPhase = ROM4_RP_SERVING_CHANNEL_CHANGE;
        r->reportsRemaining = 1; rom4_rp_defer(m, 1);
    }
    else if (op == 0x02 && r->radioPhase == ROM4_RP_SELECTED_BCCH &&
             plen >= 9 && buf[8] == 0x60) {
        // The channel-change acceptance window closes before the selected search's finite
        // terminal: suspend that search, acknowledge the requested change, then resume.
        r->selectedReportsRemaining = r->reportsRemaining;
        r->radioPhase = ROM4_RP_SELECTED_BCCH_CHANNEL_CHANGE;
        r->reportsRemaining = 1; rom4_rp_defer(m, 1);
    }
    else if (op == 0x02 && r->radioPhase == ROM4_RP_RANDOM_ACCESS &&
             plen >= 9 && buf[8] == 0x80) {
        // A matching Immediate Assignment makes RR configure the assigned SDCCH; complete the
        // same recovered channel-change transaction. Firmware owns the LAPDm establishment.
        r->radioPhase = ROM4_RP_ASSIGNED_CHANNEL_CHANGE;
        r->reportsRemaining = 1; rom4_rp_defer(m, 1);
    }
    else if (op == 0x02 && r->radioPhase == ROM4_RP_RELEASE_DECONFIGURE &&
             plen >= 16 && buf[8] == 0x60 && buf[15] == 0x0f) {
        // RR Channel Release makes the ROM issue the channel-0x60 CHANNEL_CONFIGURE with the
        // recovered deconfiguration flags; confirm it at the DSP boundary.
        r->radioPhase = ROM4_RP_RELEASE_CHANNEL_CHANGE;
        r->reportsRemaining = 1; rom4_rp_defer(m, 1);
    }
    else if (op == 0x03 &&
             (r->radioPhase == ROM4_RP_SERVING_BCCH || r->radioPhase == ROM4_RP_SELECTED_SEARCH)) {
        // DEACTIVATE retires the old receiver and cancels queued work (incl. a pending
        // selected-search terminal — delivering it after reset makes task 4 discard it in the
        // new controller state).
        r->selFromServing = 0;          // the receiver being retired IS the serving cell
        if (r->searchRequested) {
            r->searchRequested = 0;
            r->radioPhase = ROM4_RP_SELECTED_SEARCH;
            r->reportsRemaining = 2; rom4_rp_defer(m, 1);
        } else {
            r->reportsRemaining = 0;
        }
        rom4_drop_radio_backlog(r);
        if (log) fprintf(stderr, "[rom4] 0x03 DEACTIVATE @step=%llu\n",
                         (unsigned long long)m->dsp_steps);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_SERVING_BCCH) {
        // An explicit measurement request preempts the periodic serving-cell stream (delaying
        // it leaves a stale terminal for the firmware's next search).
        r->searchRequested = 0;
        r->selFromServing = 1;          // issued while camped: the serving cell is still there
        r->radioPhase = ROM4_RP_SELECTED_SEARCH;
        r->reportsRemaining = 2; rom4_rp_defer(m, 1);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_SELECTED_SEARCH && r->reportsRemaining == 0) {
        r->selFromServing = 0;          // a re-search after the selected scan already ran
        r->reportsRemaining = 2; rom4_rp_defer(m, 1);
    }
    else if (op == 0x1a && r->radioPhase == ROM4_RP_CANDIDATE_RETRY && r->reportsRemaining == 0) {
        r->radioPhase = ROM4_RP_CANDIDATE_MEASUREMENT;
        r->reportsRemaining = 1; rom4_rp_defer(m, 1);
    }
    else if (op == 0x1b && r->radioPhase == ROM4_RP_LAPDM_ESTABLISH &&
             plen >= 7 && buf[1] == 0x80 && buf[2] == 0x01 && buf[3] == 0x3f) {
        // SEND_BLOCK: two-byte DSP channel header, then LAPDm. A SABM with information invokes
        // contention resolution, whose UA echoes that information field exactly.
        const unsigned l3_length = buf[4] >> 2;
        if (l3_length <= sizeof r->contentionL3 && plen >= 5 + l3_length) {
            memcpy(r->contentionL3, buf + 5, l3_length);
            r->contentionLen = (uint8_t)l3_length;
            r->radioPhase = ROM4_RP_CONTENTION_RESOLUTION;
            r->reportsRemaining = 1; rom4_rp_defer(m, 1);
            if (log) fprintf(stderr, "[rom4] 0x1B SABM info len=%u -> contention resolution @step=%llu\n",
                             l3_length, (unsigned long long)m->dsp_steps);
        }
    }
}

// ===========================================================================================
// Owned DSP-region READ: the boot mailbox handshake (shared skeleton, unchanged).
// ===========================================================================================
static int rom4_read(struct Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    (void)size;
    // These are the LEGACY handshake semantics (dsp_mailbox_read), kept verbatim: the ROM-4
    // serial-bus loaders were brought up on them. A slot reads 0 until the DSP acks
    // (consume-once, unconditional intercept); writing one slot signals the PAIRED slot.
    if (addr == m->fw.dsp_mbox0) { *out = m->dsp_ack[0]; m->dsp_ack[0] = 0; return 1; }
    // dsp_mbox1 doubles as the MDIRCV runtime queue base: the boot-ack ping-pong applies only
    // BEFORE the firmware initialises the MDIRCV queue (head still 0); after that it is the
    // real queue word0 and must read back RAM.
    if (addr == m->fw.dsp_mbox1) {
        uint32_t hp = m->fw.mdircv_head & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        if (head == 0) { *out = m->dsp_ack[1]; m->dsp_ack[1] = 0; return 1; }
        *out = ram_value; return 1;
    }
    // Legacy boot-status word 0x10002: MCU parks 0xFFFF and waits for the DSP to report 0.
    if (addr == 0x00010002u && (ram_value & 0xFFFFu) == 0xFFFFu) { *out = 0; return 1; }
    // Per-profile boot-status slot(s): parked 0xFFFF, answered with the DSP ready/version
    // value. The 5110 serial-bus loader cross-checks a second word ([0x10004]==[0x10006]).
    if (m->fw.dsp_boot_status && addr == m->fw.dsp_boot_status &&
        (ram_value & 0xFFFFu) == 0xFFFFu) { *out = m->fw.dsp_boot_ready; return 1; }
    if (m->fw.dsp_boot_status2 && addr == m->fw.dsp_boot_status2 &&
        (ram_value & 0xFFFFu) == 0xFFFFu) { *out = m->fw.dsp_boot_ready; return 1; }
    return 0;
}

// ===========================================================================================
// Owned DSP-region WRITE: mailbox ack + code-block reply pump + the m2d request observer.
// ===========================================================================================
static void rom4_on_host_interrupt(struct Mad2* m) {
    if (!m->mem || !m->dsp_running) return;

    uint32_t cobba = m->fw.cobba & m->mem_mask;
    if (m->mem[cobba] || m->mem[cobba + 1]) {
        m->mem[cobba] = 0;
        m->mem[cobba + 1] = 0;
    }

    // Short MDI is part of the host-interrupt transaction. Real silicon's DSP clears the
    // command word on accept; the recovered ROM-4 peer synthesizes no reply for it.
    uint32_t sc = 0x000100DCu & m->mem_mask;
    if (m->mem[sc] || m->mem[sc + 1]) {
        m->mem[sc] = 0;
        m->mem[sc + 1] = 0;
    }
    rom4_pump_mdircv(m);
}

static int rom4_write(struct Mad2* m, uint32_t addr, int size, uint32_t value) {
    if (addr == 0x00020002u && (value & 0xFFu) == 0x01u)
        m->dsp_running = 0;                              // MCU put DSP in reset: warm-reboot re-arm
    // Legacy paired-slot boot handshake (see rom4_read): writing one slot makes the DSP
    // signal the paired slot ready (echo the token so the MCU's "!= 0" wait passes).
    if (addr == m->fw.dsp_mbox0) { m->dsp_ack[1] = (uint16_t)value ? (uint16_t)value : 1; m->dsp_acks++; return 1; }
    if (addr == m->fw.dsp_mbox1) {                    // boot-ack phase only (queue head==0)
        uint32_t hp = m->fw.mdircv_head & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        if (head == 0) { m->dsp_ack[0] = (uint16_t)value ? (uint16_t)value : 1; m->dsp_acks++; return 1; }
        return 0;                                     // queue initialised: let the RAM-back stand
    }
    // DSPIF bit2 is the host->DSP interrupt event; all host-side command consumption is tied
    // to this edge.
    if ((addr == 0x00030000u || addr == 0x00030001u) && size >= 1) {
        uint16_t v = size >= 2 ? (uint16_t)value
                   : addr == 0x00030001u ? (uint16_t)(value & 0xFFu)
                   : (uint16_t)((value & 0xFFu) << 8);
        if (v & 0x0004u) rom4_on_host_interrupt(m);
        return 0;
    }
    // The MCU has consumed an MDIRCV record: commit the new consumer pointer, then pump.
    if (m->fw.mdircv_head && m->mem && addr == m->fw.mdircv_head && size >= 2) {
        uint32_t hp = addr & m->mem_mask;
        m->mem[hp] = (uint8_t)(value >> 8);
        m->mem[(hp + 1u) & m->mem_mask] = (uint8_t)value;
        rom4_pump_mdircv(m);
        return 1;
    }
    // MDISND (m2d) write-pointer observer: de-wrap the freshly-enqueued record, then dispatch.
    if (m->fw.mdisnd_tail && m->mem && addr == m->fw.mdisnd_tail && size >= 2) {
        uint16_t prev = m->dsp_mdisnd_prev;
        uint32_t ring = m->fw.mdisnd_tail - m->fw.mdisnd_q;
        uint32_t off0 = (uint32_t)prev * 2;
        if (off0 < ring && (value & 0xFFFFu) != prev) {
            uint32_t b = m->fw.mdisnd_q;
            uint8_t  rlen = m->mem[(b + (off0 % ring)) & m->mem_mask];
            uint8_t  op   = m->mem[(b + ((off0 + 1u) % ring)) & m->mem_mask];
            // MDISND word0 is {payload-length, opcode}; the length does NOT include the opcode.
            uint8_t  buf[192];
            unsigned plen = rlen;
            if (plen > sizeof buf) plen = sizeof buf;
            for (unsigned k = 0; k < plen; ++k)
                buf[k] = m->mem[(b + ((off0 + 2u + k) % ring)) & m->mem_mask];

            Rom4Dsp* r = &m->rom4;
            rom4_pkt_log(m, "m2d", op, buf, plen);
            if (op == 0x70) {
                // {0x70,sub} local-security / bootstrap records. bitplane's census marks the
                // 0x13/0x14/0x15/0x16 bootstrap tables as one-way DSP publications — TRUE for
                // his exempt lab PLMN, where the ROM never runs its local-security startup check.
                // Real ROM-4 retail firmware DOES run it and streams the request records here;
                // capture them so the tick responder can answer with the decoded 0x34/0x35/0x36
                // records the firmware's own 0x287BE8 table-fill accepts from (poke-free). Only
                // {0x70,0x0D} "run self-test" gets its 0x74 {0D 00} completion.
                uint8_t sub = plen ? buf[0] : 0u;
                if (sub == 0x0D) {                       // run self-test
                    m->dsp_st_req = 1;
                } else if (sub == 0x13) {                // MSID setup -> want 0x34
                    for (int i = 0; i < 13 && (unsigned)(1 + i) < plen; ++i) r->siml_msid[i] = buf[1 + i];
                    r->siml_want |= 1u;
                } else if (sub == 0x16) {                // 24-byte SIML block -> want 0x35
                    for (int i = 0; i < 24 && (unsigned)(2 + i) < plen; ++i) r->siml_block[i] = buf[2 + i];
                    r->siml_want |= 2u;
                } else if (sub == 0x17) {                // final validation -> want 0x36
                    r->siml_want |= 4u;
                } else if (rom4_log_enabled()) {
                    fprintf(stderr, "[rom4] 0x70 sub=0x%02X (len=%u) @step=%llu\n",
                            sub, plen, (unsigned long long)m->dsp_steps);
                }
            } else {
                rom4_handle_mdi(m, op, buf, plen);
            }
            if (getenv("ROM4_M2DLOG")) {
                fprintf(stderr, "[rom4 m2d] op=0x%02X len=%u @step=%llu data=",
                        op, rlen, (unsigned long long)m->dsp_steps);
                for (unsigned i = 0; i < plen && i < 16; ++i) fprintf(stderr, "%02X", buf[i]);
                fprintf(stderr, "\n");
            }
        }
        m->dsp_mdisnd_prev = (uint16_t)(value & 0xFFFFu);
        // ProcessMdiSnd owns both queue indices: commit producer and advance the consumer to
        // it after the complete record has been dispatched.
        {
            uint32_t tp = m->fw.mdisnd_tail & m->mem_mask;
            m->mem[tp] = (uint8_t)(value >> 8);
            m->mem[(tp + 1u) & m->mem_mask] = (uint8_t)value;
            m->mem[(tp + 2u) & m->mem_mask] = (uint8_t)(value >> 8);
            m->mem[(tp + 3u) & m->mem_mask] = (uint8_t)value;
        }
        rom4_pump_mdircv(m);
        return 1;
    }
    if (addr == m->fw.dsp_cb_reply) {
        uint16_t v = (uint16_t)(value & 0xFFFFu);
        if (v && m->rom4.phase < ROM4_UPLOAD) m->rom4.phase = ROM4_UPLOAD;
        if (v != 0x0004u)               m->dsp_cb_deadline_cyc = m->rtc_mono + 256u;
        else if (m->dsp_cb_reqblk == 0x14) { m->dsp_cb_reqblk = 0x01; m->dsp_cb_deadline_cyc = m->rtc_mono + 256u; }
        else if (m->dsp_cb_reqblk == 0x01) { m->dsp_cb_reqblk = 0x02; m->dsp_cb_deadline_cyc = m->rtc_mono + 256u; }
        else                            m->dsp_cb_deadline_cyc = 0;
        m->dsp_cb_armed_nz = (v != 0);
        return 1;
    }
    return 0;
}

// ===========================================================================================
// Scheduler surface — sync_cycle / next_wake / advance_to.
// ===========================================================================================
static void rom4_sync_cycle(struct Mad2* m, uint64_t cycles) {
    m->rom4.currentCycles = cycles;
}

static uint64_t rom4_next_wake(struct Mad2* m) {
    Rom4Dsp* r = &m->rom4;
    uint64_t next = UINT64_MAX;
    if (m->dsp_cb_deadline_cyc && m->dsp_cb_deadline_cyc < next)
        next = m->dsp_cb_deadline_cyc;
    if (m->dsp_hb_next_cyc && m->dsp_hb_next_cyc < next)
        next = m->dsp_hb_next_cyc;   // idle telemetry heartbeat (deep-idle must wake for it)
    for (unsigned i = 0; i < ROM4_DELAYED_N; ++i)
        if (r->delayed[i].used && r->delayed[i].due < next) next = r->delayed[i].due;
    if (r->p_head != r->p_tail) {
        uint64_t expiry = r->pending[r->p_head].enq + rom4_cpf() * 217u * 2u;
        if (expiry < next) next = expiry;
    }
    if (r->radioPhase != ROM4_RP_INACTIVE && r->reportsRemaining &&
        r->nextReportCycle < next)
        next = r->nextReportCycle;   // paced peer reports (BCCH cadence etc.)
    return next;
}

static void rom4_advance_to(struct Mad2* m, uint64_t cycles) {
    Rom4Dsp* r = &m->rom4;
    r->currentCycles = cycles;
    rom4_expire_stale(m, cycles);
    rom4_pump_delayed(m, cycles);
    rom4_rp_pump(m);
    rom4_pump_mdircv(m);
}

// ===========================================================================================
// Per-step tick — heartbeat, boot codeblock pump, self-test completion, report scheduler.
// ===========================================================================================
static void rom4_tick(struct Mad2* m) {
    m->dsp_steps++;
    if (m->dsp_hle_quiet) return;
    if (!m->mem) return;

    // Idle telemetry heartbeat (never-silent DSP; see the essay at rom4_idle_heartbeat).
    rom4_idle_heartbeat(m);

    // Boot-loader transport: the legacy HPI codeblock handshake (shared skeleton, unchanged).
    if (m->dsp_cb_deadline_cyc && m->rtc_mono >= m->dsp_cb_deadline_cyc) {
        m->dsp_cb_deadline_cyc = 0;
        uint32_t reply = m->fw.dsp_cb_reply & m->mem_mask;
        uint32_t request = m->fw.dsp_cb_req & m->mem_mask;
        m->mem[reply] = 0;
        m->mem[(reply + 1u) & m->mem_mask] = 0;
        if (m->dsp_running) {
            if (!m->dsp_cb_reqblk) m->dsp_cb_reqblk = 0x14;
            m->mem[request] = 0;
            m->mem[(request + 1u) & m->mem_mask] = m->dsp_cb_reqblk;
        }
        if (!m->dsp_running) mad2_raise_irq(m, 4);
        if (m->dsp_cb_armed_nz) m->dsp_running = 1;
        m->dsp_cb_acks++;
    }

    // Local-security + self-test responder. The firmware's startup checks stream a small
    // request conversation that a healthy DSP answers; we synthesize those answers so the
    // firmware's OWN acceptance path resolves organically — no MCU-RAM poke. Priority
    // 0x34 > 0x35 > 0x36 > self-test-0x0D; one reply enqueued per tick (single-packet egress
    // delivers them one per empty ring window). The 0x34/0x35/0x36 records are needed on real
    // ROM-4 retail images (bitplane's lab used the exempt test PLMN 001-01, so his RE never
    // reached this); the responder is shared byte-for-byte with the rom6 engine.
    {
        Rom4Dsp* r = &m->rom4;
        int want_st = m->dsp_st_req && !m->dsp_selftest_replied && !m->dsp_selftest_off;
        if (r->siml_want || want_st) {
            uint8_t pl[64] = {0}; uint8_t len = 0;
            if (r->siml_want & 1u) {                          // 0x34 MSID reply (msg[11..23])
                pl[0] = 0x34;
                for (int i = 0; i < 13; ++i) pl[3 + i] = r->siml_msid[i];
                len = 16; r->siml_want &= (uint8_t)~1u;
            } else if (r->siml_want & 2u) {                   // 0x35 accepted record + echo
                // Region A (msg[12..35]) = the DECODED, accepted local-security record. Region B
                // (msg[36..59]) = a verbatim echo of the 0x16 security block the firmware
                // integrity-compares against its stored parameter. (Field derivation:
                // docs/sim-dsp-groundup/ local-security notes.)
                static const uint8_t region_a[24] = {
                    0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0x00,0x00, 0x00,0x98,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x08,0x7C,0x00,0x00 };
                pl[0] = 0x35; pl[1] = 0x32; pl[2] = 0x01; pl[3] = 0x00;
                for (int i = 0; i < 24; ++i) pl[4  + i] = region_a[i];
                // Region B is the security block the firmware integrity-compares against its
                // stored parameter (v4.18+: 0x280CAC memcmp region_b vs the stored value). The
                // real C54x DSP derives it by transforming the {70 16} block; our HLE cannot
                // reproduce that transform, so we RETRIEVE the matching block from the RAM-backed
                // parameter flash. Version-agnostic, no hardcoded address: the captured {70 16}
                // block is `[len byte][block data...]`; the full 24-byte block is that data in
                // flash (the MDI record is ~1 byte short). Default = verbatim echo (older builds
                // like v5.79 have no integrity check, so the search below simply no-ops).
                for (int i = 0; i < 24; ++i) pl[28 + i] = r->siml_block[i];
                int check_present = 0;   // stored block exists -> this build runs the integrity check
                {
                    const uint8_t* sig = &r->siml_block[0];
                    // Search the model's upper flash (parameter/NVRAM), not a fixed 2 MB window:
                    // on 4 MB images the flash maps to ARM [flash_base, flash_base+flash_size), so
                    // the stored block can land ~0x5E0046, past the old 0x400000 ceiling. Scan from
                    // just above the code to the end of this model's flash so it is found on every
                    // geometry.
                    uint32_t hi = (m->model ? m->model->mem.flash_base + m->model->mem.flash_size
                                            : 0x400000u);
                    for (uint32_t a = 0x300000u; a + 24u <= hi; ++a) {
                        int ok = 1;
                        for (int k = 0; k < 16; ++k)
                            if (m->mem[(a + (uint32_t)k) & m->mem_mask] != sig[k]) { ok = 0; break; }
                        if (ok) {
                            for (int i = 0; i < 24; ++i) pl[28 + i] = m->mem[(a + (uint32_t)i) & m->mem_mask];
                            check_present = 1;
                            if (rom4_log_enabled())
                                fprintf(stderr, "[rom4] local-security block located in parameter flash @0x%X\n", a);
                            break;
                        }
                    }
                }
                // The MCU already SENT the block in the {70,16} message, so the captured block IS
                // the value to echo (Region B above). A non-blank captured block means this build
                // runs the integrity check even when the flash search came up short (e.g. a model
                // that decodes the record to RAM leaves no flash copy to find).
                if (!check_present)
                    for (int k = 0; k < 24; ++k)
                        if (r->siml_block[k] != 0xFFu) { check_present = 1; break; }
                // On integrity-check builds (v4.18 NHM-5 and kin), region_a[9] steers the firmware's
                // own field classifier so its 0x35 handler takes the direct live-table write
                // (0x280D2C memcpy of region_a into 0x117B10) instead of the staging branch that
                // never commits. 0x78 selects the committing path; the evaluator (0x27FDF0) reads
                // record bytes [8]/[17]/[21], not [9], so the committed accepted record is
                // functionally identical to v5.79's. Benign on non-check builds: their per-build
                // security flag is clear, so their handler never enters the classifier and byte[9]
                // does not steer their already-working direct commit.
                if (check_present) pl[4 + 9] = 0x78;
                len = 52; r->siml_blkidx++; r->siml_want &= (uint8_t)~2u;
            } else if (r->siml_want & 4u) {                   // 0x36 terminal verdict, pass=0 -> accepted
                pl[0] = 0x36; len = 4; r->siml_want &= (uint8_t)~4u;
            } else {                                          // self-test verdict {0x0D,0} pass
                pl[0] = 0x0D; pl[1] = 0x00; len = 2;
                m->dsp_selftest_replied = 1; m->dsp_st_req = 0;
            }
            if (rom4_log_enabled())
                fprintf(stderr, "[rom4] local-security d2m 0x74 sub=0x%02X len=%u @step=%llu\n",
                        pl[0], len, (unsigned long long)m->dsp_steps);
            rom4_enqueue(m, 0x74, pl, len);
            rom4_pump_mdircv(m);
        }
    }

    // Report scheduler (also runs from advance_to for deep-idle wakes).
    rom4_rp_pump(m);
}

const DspOps mad2_dsp_rom4 = {
    .name     = "rom4",
    .read     = rom4_read,
    .write    = rom4_write,
    .tick     = rom4_tick,
    .sync_cycle = rom4_sync_cycle,
    .next_wake = rom4_next_wake,
    .advance_to = rom4_advance_to,
    .hle_tone = dsp_hle_tone,   // shared HLE COBBA tone reader (dsp/dsp_tone.c)
};
