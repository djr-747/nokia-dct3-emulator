// mad2_dsp_rom6 — clean-room C rebuild of jmacato's reference DCT3 phone-side DSP.
//
// ===========================================================================================
// GSM network layer — CREDIT & ATTRIBUTION
// ---------------------------------------------------------------------------------------------
// The reactive-network model this module implements — the GSM 04.08 System-Information / paging /
// immediate-assignment / LAPDm block formats, the DSP<->MCU handshake sequencing, and (the crux)
// the REAL-FRAME-NUMBER scheduler that emits SCH/BCCH/CCCH at coherent GSM frame positions so a
// real DCT3 handset acquires and camps — was derived by studying the reference GSM-network and
// DCT3 DSP work published by github.com/jmacato. None of that project's source is used or copied
// here (it is C#; this is an independent clean-room C re-implementation for the DCT3 emulator).
// The protocol ground-truth and message-flow insight are theirs, and we gratefully credit it.
// ===========================================================================================
//
// When selected (ROM6NEW_REF=1 -> m->dsp_override) this engine IS the DSP: it OWNS read/write/tick.
// Boot/codeblock/SIML/self-test/keep-alive behaviour is a faithful mirror of the retired
// rom6_new scaffold (those
// already work); the NEW parts are the reference responder + the real-FN scheduler. STAGE 1 goal:
// get the firmware to ACQUIRE + SELECT our cell so MM leaves state 10. Registration is STAGE 2.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>
#include "mad2/mad2.h"
#include "models/model.h"
#include "mad2/dsp/dsp_rom6.h"
#include "mad2/dsp/mdi.h"

// Single build/destroy point. The core memset already zeroes m->rom6 on init/reset.
void rom6_reset(struct Mad2* m) {
    m->rom6 = (Rom6Dsp){0};
    m->rom6.nextBcchBroadcastCycle = UINT64_MAX;
    m->rom6.nextDedicatedCycle = UINT64_MAX;
    m->rom6.nextDedicatedBlkReqCycle = UINT64_MAX;
    m->rom6.nextRachTxCycle = UINT64_MAX;
    m->rom6.nextIncomingPagingCycle = UINT64_MAX;
    m->rom6.nextSmsReference = 0x40;
    // The SIM has not necessarily served EF_IMSI yet. Paging identity/group are
    // refreshed from Mad2.sim_imsi when an incoming service is actually paged.
}

// ===========================================================================================
// Real-frame-number timing model — the big difference from a per-tick engine (the real fix).
// ===========================================================================================
// TIMING MODEL (2026-07-23 fix): the frame clock and every cadence are driven by the MCU's real
// monotonic cycle clock `m->rtc_mono` (DCT3_ARM_HZ = 13,000,000 cycles/s), NOT the per-instruction
// `m->dsp_steps` counter. dsp_steps advances once per emulated INSTRUCTION, so its rate vs real time
// jitters with cycles-per-instruction — the DSP frame numbers then drift against the firmware's own
// 13 MHz timers, and the cell-selection SM can measure the cell but never LOCK/SELECT it. Tying the
// frame clock to rtc_mono makes SCH/SI frame tags coherent with the firmware's clock, exactly like the
// reference (CurrentFrameNumber = cycles / CyclesPerTdmaFrame). CyclesPerFrame = 60000 → 13e6/60000 =
// 216.7 frames/s (real GSM rate); CyclesPerSecond = 60000*217 ≈ 13e6 = DCT3_ARM_HZ. env-overridable
// via ROM6_FNDIV. All *Delay/*Period constants (Cps/N = cpf*217/N) then land in real cycle units.
#define ROM6_CYCLES_PER_FRAME 60000u
#define ROM6_HYPERFRAME       (26u * 51u * 2048u)   // 2,715,648

static uint64_t rom6_cpf(void) {
    const char* e = getenv("ROM6_FNDIV");
    uint64_t v = (e && *e) ? strtoull(e, 0, 0) : ROM6_CYCLES_PER_FRAME;
    return v ? v : 1u;
}
// The coherent time base: the MCU's monotonic 13 MHz cycle accumulator (what the firmware's timers use).
static uint64_t rom6_now(const Rom6Dsp* r) { return r->currentCycles; }
static uint32_t rom6_frame_at(uint64_t cycles) {
    return (uint32_t)((cycles / rom6_cpf()) % ROM6_HYPERFRAME);
}
static uint32_t rom6_current_fn(const Rom6Dsp* r) {
    return rom6_frame_at(rom6_now(r));
}

// LastSchFrameNumber: the most recent frame <= fn whose T3 (fn mod 51) is an SCH slot {1,11,21,31,41}.
static uint32_t rom6_last_sch_fn(uint32_t fn) {
    static const int slots[5] = { 1, 11, 21, 31, 41 };
    uint32_t r = fn % 51u, base = (fn / 51u) * 51u;
    int best = -1;
    for (int i = 0; i < 5; ++i) if (slots[i] <= (int)r && slots[i] > best) best = slots[i];
    if (best < 0) { base -= 51u; best = 41; }       // none this multiframe -> take the previous
    return (base + (uint32_t)best) % ROM6_HYPERFRAME;
}

// LastBcchFrameNumber(tc): the most recent frame <= fn with T3 (fn mod 51) == 2 AND (fn/51) mod 8 == tc
// (05.02 §3.3.2.3 BCCH-Normal schedule). Keeps each SI landing in its own TC slot so the RR knows
// which SI it is receiving — coherent, advancing, TC-correct FN is exactly what makes the firmware lock.
static uint32_t rom6_last_bcch_fn(uint32_t fn, uint32_t tc) {
    uint32_t mf = fn / 51u;
    int back = (int)(mf % 8u) - (int)(tc % 8u);
    if (back < 0) back += 8;
    uint32_t m = mf - (uint32_t)back;
    uint32_t out = m * 51u + 2u;
    if (out > fn) { m -= 8u; out = m * 51u + 2u; }  // T3==2 not yet reached this multiframe
    return out % ROM6_HYPERFRAME;
}

// ===========================================================================================
// Host invariants — radio parameters for the reference cell. Subscriber identity
// is not an invariant: paging derives it from the active SIM's EF_IMSI.
// ===========================================================================================
#define ROM6_BSIC        0x00u
#define ROM6_DEFAULT_RSSI 0xD0u   // strong served carrier (signed-dBm byte; gate (int8)lvl+104>=0)
// The carrier this engine models, reported when a blind (untargeted) band search asks the DSP what
// it can hear and no carrier has been captured yet. Same cell as mdi_gsm.c SERVING_ARFCN.
#define ROM6_SEARCH_ARFCN 586u    // 0x024A
// LAI 05 F5 10 00 01 (MCC 505 / MNC 01 / LAC 1) is inlined in the SI2/SI3/SI4 bodies below.
static const uint8_t ROM6_CCCH_OFFSETS[9] = { 6, 12, 16, 22, 26, 32, 36, 42, 46 };

// --- GSM 04.08 L2 System-Information bodies (spec §BUILDER BYTES; LAI/CCD per host invariants) ---
// Structure + SI1/TC-schedule/ATT-bit aligned to bitplane/nokia-dct3-re (nokia_gsm_network.cpp
// SYSTEM_INFORMATION + radio_peer SI_BY_TC), which registers end-to-end. The
// PLMN bytes in SI3/SI4 are templates; rom6_enq_si_at replaces them from the
// active EF_IMSI/EF_AD before each block is queued.
// SI1 (23B): 55 06 19 + Cell-Channel-Description(16, bitmap-0) + RACH(3) + rest. Broadcast at TC=0.
// The BCCH-Norm schedule requires SI1 at TC0; the ROM "requires the real block when revalidating an
// already active serving channel" — the missing block that stalls our cell-selection finalisation.
static const uint8_t ROM6_SI1[23] = {
    0x55, 0x06, 0x19,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,  // CCD bitmap-0
    0x00,0x00,0x00, 0x2B };                         // RACH ctrl + rest octet
// SI2 (23B): 59 06 1A + 16×00 (BA neighbour bitmap) + FF (NCC) + 40 00 00 (RACH ctrl).
static const uint8_t ROM6_SI2[23] = {
    0x59, 0x06, 0x1A,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xFF, 0x40, 0x00, 0x00 };
// SI3 (23B): 49 06 1B 00 01 (cell id) + LAI(5) + Control-Channel-Description(3) + cell opt/sel/RACH
// + 4×2B. CCD octet1 = 0x40 = ATT bit (attach-detach allowed) -> the cell REQUIRES IMSI attach, i.e.
// the phone must perform the Location Update. Ours was 0x00 (attach not required) -> firmware never
// initiated the LU post-camp. This is bitplane's SI3[10]; the byte that drives registration.
static const uint8_t ROM6_SI3[23] = {
    0x49, 0x06, 0x1B, 0x00, 0x01,
    0x00,0xF1,0x10,0x00,0x01,                       // LAI template (PLMN replaced from active SIM)
    0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   // CCD [0x40=ATT] | cell opt | cell sel | RACH
    0x2B,0x2B,0x2B,0x2B };
// SI4 (23B): 31 06 1C + LAI(5) + 5×00 (cell sel | RACH) + 10×2B. (Dsp.cs BuildSystemInformation4:
// a GSM L2 block is 23 octets — the old 22-byte constant was one 0x2B short → wrong block length.)
static const uint8_t ROM6_SI4[23] = {
    0x31, 0x06, 0x1C,
    0x00,0xF1,0x10,0x00,0x01,                       // LAI template (PLMN replaced from active SIM)
    0x00,0x00,0x00,0x00,0x00,
    0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B };

// ===========================================================================================
// MDIRCV egress — single packet at a time (spec §RESPONDER).
// ===========================================================================================
// Two software queues front the one hardware ring: pendingMdiRcv (FIFO) + delayedMdiRcv
// (time-ordered). PumpMdiRcv posts ONE record via mdi_d2m_deposit ONLY when the ring is empty,
// then RaiseFiq0. Stale unposted records (> 2s) are dropped.

static void rom6_enqueue(struct Mad2* m, uint8_t op, const uint8_t* payload, uint8_t len) {
    Rom6Dsp* r = &m->rom6;
    uint8_t nt = (uint8_t)((r->p_tail + 1u) % ROM6_PENDING_N);
    if (nt == r->p_head) return;                    // FIFO full: drop (real overflow)
    Rom6MdiRec* rec = &r->pending[r->p_tail];
    rec->op = op; rec->len = len;
    if (len > ROM6_RCVMAX) len = ROM6_RCVMAX, rec->len = len;
    for (uint8_t i = 0; i < len; ++i) rec->bytes[i] = payload ? payload[i] : 0u;
    rec->enq = rom6_now(r); rec->due = rom6_now(r);
    r->p_tail = nt;
}

static void rom6_enqueue_at(struct Mad2* m, uint64_t due,
                              uint8_t op, const uint8_t* payload, uint8_t len) {
    Rom6Dsp* r = &m->rom6;
    for (unsigned i = 0; i < ROM6_DELAYED_N; ++i) {
        Rom6MdiRec* rec = &r->delayed[i];
        if (rec->used) continue;
        rec->op = op; rec->len = (len > ROM6_RCVMAX) ? ROM6_RCVMAX : len;
        for (uint8_t k = 0; k < rec->len; ++k) rec->bytes[k] = payload ? payload[k] : 0u;
        rec->enq = rom6_now(r); rec->due = due; rec->used = 1;
        return;
    }
    // delayed queue full: fall back to immediate (drop the delay rather than the record).
    rom6_enqueue(m, op, payload, len);
}

static void rom6_enqueue_after(struct Mad2* m, uint64_t delay,
                                 uint8_t op, const uint8_t* payload, uint8_t len) {
    rom6_enqueue_at(m, rom6_now(&m->rom6) + delay, op, payload, len);
}

// PumpDelayedMdiRcv: matured delayed records move into the FIFO in due order.
static void rom6_pump_delayed(struct Mad2* m, uint64_t cycles) {
    Rom6Dsp* r = &m->rom6;
    for (unsigned i = 0; i < ROM6_DELAYED_N; ++i) {
        Rom6MdiRec* rec = &r->delayed[i];
        if (rec->used && cycles >= rec->due) {
            rom6_enqueue(m, rec->op, rec->bytes, rec->len);
            rec->used = 0;
        }
    }
}

// ExpireStale: drop FIFO records that have waited unposted longer than 2s (Cps*2 cycles).
static void rom6_expire_stale(struct Mad2* m, uint64_t cycles) {
    Rom6Dsp* r = &m->rom6;
    uint64_t ttl = rom6_cpf() * 217u * 2u;        // 2 seconds
    while (r->p_head != r->p_tail) {
        Rom6MdiRec* rec = &r->pending[r->p_head];
        if (cycles - rec->enq <= ttl) break;  // FIFO is roughly time-ordered
        r->p_head = (uint8_t)((r->p_head + 1u) % ROM6_PENDING_N);
    }
}

// PumpMdiRcv: post ONE record from the FIFO head into the (empty) hardware ring + RaiseFiq0.
#define ROM6_KA_CYC 25000000u   // idle-telemetry cadence, ~5 s @ 4.93 MHz (see rom6_idle_heartbeat)

static void rom6_pump_mdircv(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    if (r->p_head == r->p_tail) return;             // FIFO empty
    Rom6MdiRec* rec = &r->pending[r->p_head];
    // mdi_d2m_deposit only posts into an EMPTY ring (producer==consumer) — the "one packet at a
    // time" guarantee. Returns 0 if the MCU has not drained the previous frame yet; retry next tick.
    if (mdi_d2m_deposit(m->mem, m->mem_mask, m->fw.mdircv_q, m->fw.mdircv_tail,
                        m->fw.mdircv_head, rec->op, rec->bytes, rec->len)) {
        if (getenv("ROM6_LOG") && rec->op == 0x80 && rec->len >= 13) {
            if (rec->bytes[0] == 0x60 && rec->bytes[12] == 0x3F)
                fprintf(stderr, "[rom6] Immediate Assignment posted FN=%u RA=0x%02X @step=%llu\n",
                        ((unsigned)rec->bytes[3] << 16) | ((unsigned)rec->bytes[4] << 8) | rec->bytes[5],
                        rec->bytes[17], (unsigned long long)m->dsp_steps);
            else if (rec->bytes[0] == 0x80)
                fprintf(stderr,
                        "[rom6] dedicated posted FN=%u addr=%02X ctrl=%02X li=%02X @step=%llu\n",
                        ((unsigned)rec->bytes[3] << 16) | ((unsigned)rec->bytes[4] << 8) | rec->bytes[5],
                        rec->bytes[10], rec->bytes[11], rec->bytes[12],
                        (unsigned long long)m->dsp_steps);
        }
        mad2_raise_fiq(m, 0);
        r->p_head = (uint8_t)((r->p_head + 1u) % ROM6_PENDING_N);
        // Real d2m traffic feeds the firmware's MDI-activity counter — re-pace the idle
        // telemetry heartbeat so it only ever fills genuine DSP silence (see rom6_idle_heartbeat).
        m->dsp_hb_next_cyc = m->rtc_mono + ROM6_KA_CYC;
    }
}

// Perpetual idle MDI telemetry — the real DSP is NEVER silent. The firmware's 0xE4
// DSP-liveness watchdog (soft-timer slot 36) is perpetual and counter-driven: every ~19-21 s
// it demands >=1 non-0xE4 MDIRCV ring entry since the last tick or it stages reason-0x68
// (full RE essay: the keep-alive block in dsp/dsp_rom4.c — measured A/B: a silent DSP ALWAYS
// trips 0x68). The engine's own network stream satisfies this while the firmware drives L1
// (BCCH/CCCH cadence ≪ 5 s), but a model whose network task never engages the DSP (e.g. the
// 3410 at standby) would leave the engine mute -> organic warm-reboot at ~283M cycles. So,
// exactly like the real captured idle stream (v6.33 RAM read: perpetual group-0x03 telemetry),
// deposit one bare group-0x03 ring word per ~5 s of d2m silence (ROM6_KA_CYC, the measured
// real-HW idle report cadence). Same discipline as the engine's egress: only into an empty
// ring, only once protocol-ready, END->START wrap.
static void rom6_idle_heartbeat(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    int protocol_ready = m->dsp_running &&
        (m->dsp_selftest_replied || m->dsp_selftest_off);
    if (!protocol_ready) { m->dsp_hb_next_cyc = 0; return; }
    if (!m->dsp_hb_next_cyc) { m->dsp_hb_next_cyc = m->rtc_mono + ROM6_KA_CYC; return; }
    if (m->rtc_mono < m->dsp_hb_next_cyc) return;
    if (r->p_head != r->p_tail) return;      // engine has real traffic in flight: it will re-pace
    uint32_t hp = m->fw.mdircv_head & m->mem_mask;
    uint32_t tp = m->fw.mdircv_tail & m->mem_mask;
    uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
    uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
    if (head != tail || head < 0x80) return; // never tramples a live ring entry; retry next tick
    uint32_t q   = m->fw.mdircv_q & m->mem_mask;
    uint32_t off = (uint32_t)(tail - 0x80) * 2u;
    m->mem[q + off]      = 0x03;             // BE even byte = DSP->MCU group 0x03 (telemetry)
    m->mem[q + off + 1u] = 0x00;             // odd byte = id/sub (content past the group is moot)
    uint16_t nt = (uint16_t)(tail + 1u);     // 1 word, no payload
    if (nt >= 0xE4) nt = 0x80;               // ring END (idx 0xE4) -> START (idx 0x80), as the
                                             // firmware's own MDIRCV_DEQUEUE wraps its head
    m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1u] = (uint8_t)nt;
    mad2_raise_fiq(m, 0);                    // FIQ0: DSP signalled MDIRCV
    m->dsp_hb_last = m->rtc_mono;
    m->dsp_hb_next_cyc = m->rtc_mono + ROM6_KA_CYC;
    m->dsp_hb_pulses++;
}

// Once the handset retunes to an SDCCH, queued BCCH/CCCH blocks from the old
// idle receive context are no longer receivable.  Keeping them in the software
// FIFO lets stale paging blocks reach the firmware after the dedicated channel
// is active; the firmware then retunes back to CCCH between UA and the first
// network I-frame.  Real RF naturally discards those bursts at the retune edge.
static void rom6_drop_idle_radio_backlog(Rom6Dsp* r) {
    Rom6MdiRec keep[ROM6_PENDING_N];
    uint8_t count = 0;
    for (uint8_t i = r->p_head; i != r->p_tail;
         i = (uint8_t)((i + 1u) % ROM6_PENDING_N)) {
        Rom6MdiRec* rec = &r->pending[i];
        int idle_radio = rec->op == 0x80u && rec->len != 0u &&
                         (rec->bytes[0] == 0x50u || rec->bytes[0] == 0x60u);
        if (!idle_radio && count + 1u < ROM6_PENDING_N)
            keep[count++] = *rec;
    }
    r->p_head = 0;
    r->p_tail = count;
    for (uint8_t i = 0; i < count; ++i) r->pending[i] = keep[i];
    for (unsigned i = 0; i < ROM6_DELAYED_N; ++i) {
        Rom6MdiRec* rec = &r->delayed[i];
        if (rec->used && rec->op == 0x80u && rec->len != 0u &&
            (rec->bytes[0] == 0x50u || rec->bytes[0] == 0x60u))
            rec->used = 0;
    }
}

// ===========================================================================================
// Builders (exact bytes from spec §BUILDER BYTES). Each fills pl[] = the payload that FOLLOWS the
// {len,op} frame word, and returns its length; the caller enqueues {op, pl, len}.
// ===========================================================================================

// ReceivedBlock (op 0x80): pl = [logch][bsic][00][fn>>16][fn>>8][fn][arfcn>>8][arfcn][00][00][L2...].
// len = 10 + |L2|.
static uint8_t rom6_b_received_block(uint8_t* pl, uint8_t logch, uint8_t bsic, uint16_t arfcn,
                                       uint32_t fn, const uint8_t* L2, uint8_t l2len) {
    pl[0] = logch; pl[1] = bsic; pl[2] = 0x00;
    pl[3] = (uint8_t)(fn >> 16); pl[4] = (uint8_t)(fn >> 8); pl[5] = (uint8_t)fn;
    pl[6] = (uint8_t)(arfcn >> 8); pl[7] = (uint8_t)arfcn; pl[8] = 0x00; pl[9] = 0x00;
    for (uint8_t i = 0; i < l2len; ++i) pl[10 + i] = L2[i];
    return (uint8_t)(10 + l2len);
}

// SCH Information (04.08 §9.1.30): 4-byte L2 carried by a ReceivedBlock on logch 0x40.
static uint8_t rom6_b_sch_info(uint8_t* L2, uint8_t bsic, uint32_t fn) {
    uint32_t t1 = (fn / 1326u) % 2048u, t2 = fn % 26u;
    uint32_t t3p = (uint32_t)(((int32_t)(fn % 51u) - 1) / 10);
    L2[0] = (uint8_t)((bsic << 2) | (t1 >> 9));
    L2[1] = (uint8_t)(t1 >> 1);
    L2[2] = (uint8_t)(((t1 & 1u) << 7) | (t2 << 2) | (t3p >> 1));
    L2[3] = (uint8_t)((t3p & 1u) << 7);
    return 4;
}

// PagingRequestType1 (23B L2): 31 06 21 [(chNeeded&3)<<4] 08 <mobid×8> 2B×10.
static uint8_t rom6_b_paging_req1(uint8_t* L2, uint8_t chNeeded,
                                    const uint8_t mobid[8]) {
    L2[0] = 0x31; L2[1] = 0x06; L2[2] = 0x21; L2[3] = (uint8_t)((chNeeded & 3u) << 4); L2[4] = 0x08;
    for (int i = 0; i < 8; ++i) L2[5 + i] = mobid[i];
    for (int i = 13; i < 23; ++i) L2[i] = 0x2B;
    return 23;
}
// PagingFill / no-identity (23B L2): 15 06 21 00 01 F0 2B×17.
static uint8_t rom6_b_paging_fill(uint8_t* L2) {
    L2[0] = 0x15; L2[1] = 0x06; L2[2] = 0x21; L2[3] = 0x00; L2[4] = 0x01; L2[5] = 0xF0;
    for (int i = 6; i < 23; ++i) L2[i] = 0x2B;
    return 23;
}

// RA_INFO (d2m 0x84) — the DSP's report of the random-access burst it has just transmitted:
// payload = [RA][fn>>16][fn>>8][fn], i.e. the CHANNEL REQUEST octet plus the ABSOLUTE GSM frame
// number the burst went out on. Same wire shape as the ROM-4 engine's RA_INFO
// (dsp/dsp_rom4.c, bitplane's census "d2m 0x84 RA_INFO").
//
// This is the message that makes the firmware record its own outstanding request reference — it
// is what retired the old MCU-RAM poke. Verified on
// 3310 v5.79, statically end to end:
//
//   TASK_4_MDI_RECEIVER 0x2EDB04 -> d2m jump table 0x2EDB48 idx (op-0x83)
//     op 0x84 -> 0x2EDBC4 -> bl 0x2C3C9C: gated on [0x110FE0] (GU2_DSP_MEAS_OPER_GATE) == 1,
//        forwards the broker buffer to task 10 via TASK_SEND_TYPED, else HEAP_FREEs it
//   task 10 (0x22E650) -> bl 0x22FB7E -> RA_INFO handler 0x2DB904:
//        fn  = (buf[5]<<16)|(buf[6]<<8)|buf[7]      <- payload[1..3], absolute frame
//        RA  = buf[4]                               <- payload[0]
//        builds a 12-byte broker message id 0x803 with
//        [5]=(fn/1326)&0x1F (T1'), [6]=fn%51 (T3), [7]=fn%26 (T2), [8]=RA
//        and TASK_SEND_TYPEDs it to task 13 (RR)
//   RR (task 13, entry 0x251228) writer at 0x2A67FE:
//        idx = (state[2]+1) % 3; e = 0x111584 + idx*8
//        e[0]=1 (valid) e[1]=RA e[2]=T1' e[3]=T3 e[4]=T2 e[5]=0 (matched)
//   the RR matcher 0x27AF80 later walks those 3 entries against the received Immediate
//   Assignment's Request Reference IE and sets e[5]=1 on a full match.
//
// So the firmware derives T1'/T3/T2 itself from the frame WE report here; the IA built by
// rom6_b_imm_assign() must therefore quote the SAME frame number (it does — both use
// rachRequestFn), or the matcher rejects the grant and RR re-RACHes.
static uint8_t rom6_b_ra_info(uint8_t* pl, uint8_t ra, uint32_t fn) {
    pl[0] = ra;
    pl[1] = (uint8_t)(fn >> 16);
    pl[2] = (uint8_t)(fn >> 8);
    pl[3] = (uint8_t)fn;
    for (int i = 4; i < 8; ++i) pl[i] = 0x00;       // ROM-4 pads the report to 8 octets
    return 8;
}

// ImmediateAssignment (23B L2): 2D 06 3F 00 41 [(tsc<<5)|(arfcn>>8)] [arfcn] [RA]
//   [(t1<<3)|(t3>>3)] [((t3&7)<<5)|t2] 00 00 2B×11. (tsc=0.)
static uint8_t rom6_b_imm_assign(uint8_t* L2, uint16_t arfcn, uint8_t bsic, uint8_t ra, uint32_t fn) {
    fn %= 42432u;                                   // DecodeRequestReferenceFrame modulus (51*26*32)
    uint32_t t3 = fn % 51u, t2 = fn % 26u, t1 = (fn / 1326u) & 0x1Fu;
    uint8_t tsc = bsic & 0x07u;
    arfcn &= 0x03FFu;
    L2[0] = 0x2D; L2[1] = 0x06; L2[2] = 0x3F; L2[3] = 0x00; L2[4] = 0x41;
    L2[5] = (uint8_t)((tsc << 5) | (arfcn >> 8));   // TSC | H0 | spare | ARFCN[9:8]
    L2[6] = (uint8_t)arfcn;
    L2[7] = ra;
    L2[8] = (uint8_t)((t1 << 3) | (t3 >> 3));
    L2[9] = (uint8_t)(((t3 & 7u) << 5) | (t2 & 0x1Fu));
    L2[10] = 0x00; L2[11] = 0x00;
    for (int i = 12; i < 23; ++i) L2[i] = 0x2B;
    return 23;
}

// 0x83 RSSI_RESULTS (6B payload): [arfcn_hi 01 lvl lvl arfcn_hi arfcn_lo].
static uint8_t rom6_b_rssi83(uint8_t* pl, uint16_t arfcn, uint8_t lvl) {
    pl[0] = (uint8_t)(arfcn >> 8); pl[1] = 0x01; pl[2] = lvl; pl[3] = lvl;
    pl[4] = (uint8_t)(arfcn >> 8); pl[5] = (uint8_t)arfcn;
    return 6;
}
// 0x8B ALL_RSSI (162B payload): [00 00 arfcn_hi arfcn_lo 00 D0] then 39×[00 00 00 80].
static uint8_t rom6_b_rssi8b(uint8_t* pl, uint16_t arfcn) {
    pl[0] = 0x00; pl[1] = 0x00; pl[2] = (uint8_t)(arfcn >> 8); pl[3] = (uint8_t)arfcn;
    pl[4] = 0x00; pl[5] = ROM6_DEFAULT_RSSI;
    for (int k = 0; k < 39; ++k) { pl[6+4*k]=0x00; pl[7+4*k]=0x00; pl[8+4*k]=0x00; pl[9+4*k]=0x80; }
    return (uint8_t)162;
}
// 0x88 neighbour-timing-offset (10B payload): [01 fn>>16 fn>>8 fn arfcn_hi arfcn_lo 00 00 00 01].
static uint8_t rom6_b_nbrtim(uint8_t* pl, uint16_t arfcn, uint32_t fn) {
    pl[0] = 0x01; pl[1] = (uint8_t)(fn >> 16); pl[2] = (uint8_t)(fn >> 8); pl[3] = (uint8_t)fn;
    pl[4] = (uint8_t)(arfcn >> 8); pl[5] = (uint8_t)arfcn;
    pl[6] = 0x00; pl[7] = 0x00; pl[8] = 0x00; pl[9] = 0x01;
    return 10;
}

// --- Composite enqueues used by dispatch + scheduler ---

// EnqueueServingCellSchBlock: SCH sync (logch 0x40) at the last SCH frame.
static void rom6_enq_sch_at(struct Mad2* m, uint64_t eventCycles) {
    Rom6Dsp* r = &m->rom6;
    uint8_t pl[64], L2[4];
    uint32_t fn = rom6_last_sch_fn(rom6_frame_at(eventCycles));
    uint8_t l2 = rom6_b_sch_info(L2, r->servingBsic, fn);
    uint8_t len = rom6_b_received_block(pl, 0x40, r->servingBsic, r->servingArfcn, fn, L2, l2);
    rom6_enqueue(m, 0x80, pl, len);
}
static void rom6_enq_sch(struct Mad2* m) {
    rom6_enq_sch_at(m, rom6_now(&m->rom6));
}

// EnqueueOneSystemInformation: one SI (logch 0x50) at the BCCH frame for its TC slot.
static void rom6_enq_si_at(struct Mad2* m, const uint8_t* si, uint8_t silen,
                             uint32_t tc, uint64_t eventCycles) {
    Rom6Dsp* r = &m->rom6;
    uint8_t pl[64], active[23];
    const uint8_t* air = si;
    if (silen == sizeof active && si[1] == 0x06u &&
        (si[2] == 0x1Bu || si[2] == 0x1Cu)) {
        uint8_t plmn[3];
        memcpy(active, si, sizeof active);
        mad2_sim_current_plmn(m, plmn);
        unsigned off = si[2] == 0x1Bu ? 5u : 3u;
        memcpy(active + off, plmn, sizeof plmn);
        air = active;
    }
    uint32_t fn = rom6_last_bcch_fn(rom6_frame_at(eventCycles), tc);
    uint8_t len = rom6_b_received_block(pl, 0x50, r->servingBsic, r->servingArfcn,
                                          fn, air, silen);
    rom6_enqueue(m, 0x80, pl, len);
}
static void rom6_enq_si(struct Mad2* m, const uint8_t* si, uint8_t silen, uint32_t tc) {
    rom6_enq_si_at(m, si, silen, tc, rom6_now(&m->rom6));
}

// EnqueueServingCellSystemInformation: burst SI1(TC0)+SI2(TC1)+SI3(TC2)+SI4(TC3) on capture/config.
static void rom6_enq_si_burst(struct Mad2* m) {
    rom6_enq_si(m, ROM6_SI1, sizeof ROM6_SI1, 0u);
    rom6_enq_si(m, ROM6_SI2, sizeof ROM6_SI2, 1u);
    rom6_enq_si(m, ROM6_SI3, sizeof ROM6_SI3, 2u);
    rom6_enq_si(m, ROM6_SI4, sizeof ROM6_SI4, 3u);
}

// Delayed burst: one SI (logch 0x50) at each TC's BCCH frame, deposited `delay` cycles from now.
// Used so a fresh SCH sync (chan-state advance with chan-state==0) can complete BEFORE the BCCH
// identity decode lands — the real acquisition orders FCCH/SCH sync ahead of the BCCH read, and
// giving both simultaneously makes the firmware enter its camped-monitor (main-state 0x3E, chan-
// state pinned 1) before the cell-select sub-state can reach 3. Env ROM6_SIDELAY_FR (frames, 0=off).
static void rom6_enq_si_after(struct Mad2* m, const uint8_t* si, uint8_t silen, uint32_t tc, uint64_t delay) {
    Rom6Dsp* r = &m->rom6;
    uint8_t pl[64], active[23];
    const uint8_t* air = si;
    if (silen == sizeof active && si[1] == 0x06u &&
        (si[2] == 0x1Bu || si[2] == 0x1Cu)) {
        uint8_t plmn[3];
        memcpy(active, si, sizeof active);
        mad2_sim_current_plmn(m, plmn);
        unsigned off = si[2] == 0x1Bu ? 5u : 3u;
        memcpy(active + off, plmn, sizeof plmn);
        air = active;
    }
    uint32_t fn = rom6_last_bcch_fn(rom6_frame_at(rom6_now(r) + delay), tc);
    uint8_t len = rom6_b_received_block(pl, 0x50, r->servingBsic, r->servingArfcn,
                                          fn, air, silen);
    rom6_enqueue_after(m, delay, 0x80, pl, len);
}
static void rom6_enq_si_burst_delayed(struct Mad2* m) {
    const char* e = getenv("ROM6_SIDELAY_FR");
    uint64_t fr = (e && *e) ? strtoull(e, 0, 0) : 0u;
    if (!fr) { rom6_enq_si_burst(m); return; }            // 0 = original immediate behaviour
    uint64_t d = fr * rom6_cpf();
    rom6_enq_si_after(m, ROM6_SI1, sizeof ROM6_SI1, 0u, d);
    rom6_enq_si_after(m, ROM6_SI2, sizeof ROM6_SI2, 1u, d);
    rom6_enq_si_after(m, ROM6_SI3, sizeof ROM6_SI3, 2u, d);
    rom6_enq_si_after(m, ROM6_SI4, sizeof ROM6_SI4, 3u, d);
}

// EnqueueImmediateAssignment: delayed Cps/10, logch 0x60. The IA's Request Reference MUST echo the
// frame the handset RACHed on (reqfn, from the 0x0C payload) — NOT current_fn — or the firmware rejects
// the grant and re-RACHes (reference EnqueueImmediateAssignment). The block's own FN is a fresh CCCH frame.
static void rom6_enq_imm_assign_at(struct Mad2* m, uint8_t ra, uint32_t reqfn,
                                     uint64_t rachEventCycles) {
    Rom6Dsp* r = &m->rom6;
    uint8_t pl[64], L2[23];
    // Block FN = the next real CCCH/AGCH block frame at/after delivery (reference NextCcchBlockFrameNumber).
    // The firmware validates the block FN against its control-channel schedule (0x32E8D8); a mis-aligned FN
    // is routed to the paging-drop path -> the IA is never accepted -> no dedicated SDCCH -> no LU. Using
    // current_fn (unaligned) was the camp->LU blocker.
    uint64_t delay = (rom6_cpf() * 217u) / 10u;
    uint64_t due = rachEventCycles + delay;
    uint32_t afn = rom6_frame_at(due);
    uint32_t mframe = afn / 51u, t3 = afn % 51u, blkfn; int found = 0;
    for (int i = 0; i < 9 && !found; ++i)
        if (ROM6_CCCH_OFFSETS[i] >= t3) { blkfn = mframe * 51u + ROM6_CCCH_OFFSETS[i]; found = 1; }
    if (!found) blkfn = (mframe + 1u) * 51u + ROM6_CCCH_OFFSETS[0];
    blkfn %= ROM6_HYPERFRAME;
    uint8_t l2 = rom6_b_imm_assign(L2, r->servingArfcn, r->servingBsic, ra, reqfn);   // reqfn -> Request Reference (T1'/T2/T3)
    uint8_t len = rom6_b_received_block(pl, 0x60, r->servingBsic, r->servingArfcn, blkfn, L2, l2);
    rom6_enqueue_at(m, due, 0x80, pl, len);
    if (getenv("ROM6_LOG"))
        fprintf(stderr, "[rom6] Immediate Assignment scheduled blockFN=%u reqFN=%u RA=0x%02X due=%llu\n",
                blkfn, reqfn, ra, (unsigned long long)due);
}

// EnqueuePaging: one CCCH block (logch 0x60) at one CCCH offset of the last complete multiframe.
static void rom6_enq_paging_at(struct Mad2* m, uint32_t offset, int imsi,
                                 uint64_t eventCycles) {
    Rom6Dsp* r = &m->rom6;
    uint8_t pl[64], L2[23];
    uint32_t cur = rom6_frame_at(eventCycles);
    uint32_t base = (cur / 51u) * 51u;
    uint32_t fn = base + (offset % 51u);
    if (fn > cur) { if (base >= 51u) base -= 51u; fn = base + (offset % 51u); }
    // Identity paging requests an SDCCH; periodic non-identity broadcast fill
    // does not request a channel.
    uint8_t l2 = imsi ? rom6_b_paging_req1(L2, 1u, r->pagingMobid)
                      : rom6_b_paging_fill(L2);
    uint8_t len = rom6_b_received_block(pl, 0x60, r->ccchBsic ? r->ccchBsic : r->servingBsic,
                                          r->ccchArfcn ? r->ccchArfcn : r->servingArfcn, fn, L2, l2);
    rom6_enqueue(m, 0x80, pl, len);
}

// ===========================================================================================
// STAGE 2 — SDCCH dedicated channel + LAPDm SAPI-0 link + Location-Update state machine.
// ===========================================================================================
// CREDIT: the LAPDm framing (SABM/UA/DISC/I/RR), the pending-downlink-ack model, and the minimal
// GSM 04.08 Location-Update network side (LU-Accept -> MM-Information + Channel-Release, accepted
// unconditionally with no auth/cipher/TMSI) were derived clean-room from the reference GSM-network
// work published by github.com/jmacato. None of that C# is copied — this is an independent C
// re-implementation. The protocol ground-truth and message-flow insight are gratefully credited.
//
// The MS transmits its LU-Request in the SABM information field (04.08 contention resolution). We
// answer UA immediately, then drive downlink I-frames whose acknowledgements (the MS's N(R))
// advance the network FSM: LU-Accept -> (MS acks) -> MM-Information + Channel-Release -> (MS acks
// Channel-Release) -> Released == REGISTERED. We NEVER advance the FSM before the MS acks a frame.
//
// LU-Accept uses the same active-SIM PLMN as SI3/SI4 and LAC 1. A hardcoded
// test-network LAI here makes a card/IMSI switch fail after otherwise valid camp.

// Network states. Capture of a new dedicated channel resets this connection
// state to IDLE, while the DSP-level registered/paging state survives.
enum {
    REG_IDLE = 0,
    REG_AWAIT_LU_ACK,
    REG_AWAIT_REL_ACK,
    REG_RELEASED,
    REG_AWAIT_CIPHER_ACK,
    REG_AWAIT_CIPHER_COMPLETE,
    REG_MM_ACTIVE
};

enum {
    SVC_NONE = 0,
    SVC_MO_CALL = 1,
    SVC_EMERGENCY = 2,
    SVC_SMS = 4,
    SVC_MT_CALL = 5,
    SVC_MT_SMS = 6
};

// Pending-downlink acknowledgement kinds. KIND_SEGMENT deliberately has no
// network transition; only the final segment owns the L3 message event.
enum {
    KIND_SEGMENT = 1,
    KIND_LU_ACCEPT,
    KIND_MM_INFO,
    KIND_CHAN_REL,
    KIND_CIPHER_COMMAND,
    KIND_MT_CALL_SETUP,
    KIND_SAPI3_ESTABLISH,
    KIND_MT_SMS_CP_DATA,
    KIND_CP_ACK,
    KIND_RP_ACK,
    KIND_CALL_PROCEEDING,
    KIND_ALERTING,
    KIND_CONNECT,
    KIND_CONNECT_ACK,
    KIND_RELEASE
};

enum { INCOMING_CALL = 1, INCOMING_SMS = 2 };

static uint8_t rom6_nitz_bcd(unsigned value) {
    return (uint8_t)(((value % 10u) << 4) | ((value / 10u) % 10u));
}

// Network time is sampled when the MM-Information protocol event is created, as in
// Noks' injected networkLocalTimeProvider. It is never compiled into a radio frame.
// UTC keeps the portable C backend deterministic with respect to host timezone rules;
// the final NITZ octet therefore truthfully advertises a zero quarter-hour offset.
static void rom6_build_mm_information(uint8_t out[10]) {
    time_t stamp = time(NULL);
    struct tm utc = {0};
#if defined(_WIN32)
    gmtime_s(&utc, &stamp);
#else
    {
        struct tm* value = gmtime(&stamp);
        if (value) utc = *value;
    }
#endif
    out[0] = 0x05; out[1] = 0x32; out[2] = 0x47;
    out[3] = rom6_nitz_bcd((unsigned)((utc.tm_year + 1900) % 100));
    out[4] = rom6_nitz_bcd((unsigned)(utc.tm_mon + 1));
    out[5] = rom6_nitz_bcd((unsigned)utc.tm_mday);
    out[6] = rom6_nitz_bcd((unsigned)utc.tm_hour);
    out[7] = rom6_nitz_bcd((unsigned)utc.tm_min);
    out[8] = rom6_nitz_bcd((unsigned)utc.tm_sec);
    out[9] = 0x00;
}

static int rom6_log_enabled(void) {
    return getenv("ROM6_LOG") != 0;
}

static uint64_t rom6_next_paging_group_cycle(const Rom6Dsp* r, uint64_t cycles);
static void rom6_refresh_paging_identity(struct Mad2* m);

static void rom6_copy_digits(char out[21], const char* value) {
    unsigned n = 0;
    if (value) {
        while (*value && n < 20) {
            if (*value >= '0' && *value <= '9') out[n++] = *value;
            value++;
        }
    }
    if (!n) {
        memcpy(out, "12345", 6);
        return;
    }
    out[n] = 0;
}

static void rom6_copy_sms(char out[121], const char* value) {
    unsigned n = 0;
    if (value) {
        while (*value && n < 120) {
            unsigned char c = (unsigned char)*value++;
            out[n++] = (c >= 0x20 && c <= 0x7E) ? (char)c : ' ';
        }
    }
    if (!n) {
        memcpy(out, "Hello from Noks", 16);
        return;
    }
    out[n] = 0;
}

static int rom6_queue_incoming(struct Mad2* m, uint8_t kind,
                                 const char* address, const char* text) {
    Rom6Dsp* r = &m->rom6;
    uint8_t next = (uint8_t)((r->incomingTail + 1u) % ROM6_INCOMING_N);
    if (next == r->incomingHead) return 0;
    Rom6IncomingService* service = &r->incoming[r->incomingTail];
    memset(service, 0, sizeof *service);
    service->kind = kind;
    rom6_copy_digits(service->address, address);
    if (kind == INCOMING_SMS) rom6_copy_sms(service->text, text);
    r->incomingTail = next;
    if (r->suppressImsiPaging && !r->dedicatedConfigured &&
        !r->incomingPagingActive) {
        rom6_refresh_paging_identity(m);
        r->incomingPagingActive = 1;
        r->incomingPagingAnswered = 0;
        r->incomingPagingBursts = 0;
        r->nextIncomingPagingCycle =
            rom6_next_paging_group_cycle(r, rom6_now(r));
    }
    if (rom6_log_enabled())
        fprintf(stderr, "[rom6] incoming %s queued from %s%s%s\n",
                kind == INCOMING_CALL ? "call" : "SMS", service->address,
                kind == INCOMING_SMS ? " text=\"" : "",
                kind == INCOMING_SMS ? service->text : "");
    return 1;
}

int rom6_queue_incoming_call(struct Mad2* m, const char* calling_number) {
    return m && mad2_active_dsp(m) == &mad2_dsp_rom6
        ? rom6_queue_incoming(m, INCOMING_CALL, calling_number, 0) : 0;
}

int rom6_queue_incoming_sms(struct Mad2* m, const char* originator, const char* text) {
    return m && mad2_active_dsp(m) == &mad2_dsp_rom6
        ? rom6_queue_incoming(m, INCOMING_SMS, originator, text) : 0;
}

// Wrap a 23-byte LAPDm L2 block as an 0x80 RECEIVED_BLOCK on the dedicated channel (logch 0x80)
// and enqueue it on the FIFO (single-packet egress paces delivery).
static void rom6_dl_lapdm_at(struct Mad2* m, const uint8_t* L2, uint64_t eventCycles) {
    Rom6Dsp* r = &m->rom6;
    uint8_t pl[64];
    uint32_t fn = rom6_frame_at(eventCycles);
    uint8_t len = rom6_b_received_block(pl, 0x80, r->dedicatedBsic, r->dedicatedArfcn, fn, L2, 23);
    rom6_enqueue(m, 0x80, pl, len);
}

// LAPDm generation is asynchronous. Network/L2 handlers enqueue a raw frame; the
// dedicated-channel scheduler stamps and emits it only when an SDCCH downlink slot fires.
static void rom6_queue_lapdm(struct Mad2* m, const uint8_t* L2) {
    Rom6Dsp* r = &m->rom6;
    uint8_t next = (uint8_t)((r->dedicatedTail + 1u) % ROM6_DEDICATED_N);
    if (next == r->dedicatedHead) return;
    memcpy(r->dedicatedFrames[r->dedicatedTail], L2, 23);
    r->dedicatedTail = next;
}

// Build + queue a downlink I-frame carrying L3 `info`, advance V(S), and register a pending ack
// (its N(R) key = the new V(S), i.e. the sequence the MS will send once it has received the frame).
static Rom6LapdmLink* rom6_link(Rom6Dsp* r, uint8_t sapi) {
    return &r->lapdm[sapi & 7u];
}

static void rom6_link_reset(Rom6LapdmLink* link) {
    memset(link, 0, sizeof *link);
}

static void rom6_reset_links(Rom6Dsp* r) {
    for (unsigned sapi = 0; sapi < ROM6_LAPDM_SAPIS; ++sapi)
        rom6_link_reset(&r->lapdm[sapi]);
}

static void rom6_record_ack(Rom6LapdmLink* link, uint8_t kind) {
    if (link->ack_count >= ROM6_LAPDM_ACK_N) return;
    uint8_t idx = (uint8_t)((link->ack_head + link->ack_count) % ROM6_LAPDM_ACK_N);
    link->ack_rs[idx] = link->vs;
    link->ack_kind[idx] = kind;
    link->ack_count++;
}

static void rom6_send_iframes(struct Mad2* m, uint8_t sapi,
                                const uint8_t* info, uint16_t infolen,
                                uint8_t nr, uint8_t final_kind) {
    Rom6Dsp* r = &m->rom6;
    Rom6LapdmLink* link = rom6_link(r, sapi);
    uint16_t offset = 0;
    do {
        uint16_t remaining = infolen - offset;
        uint8_t count = (uint8_t)(remaining > 20u ? 20u : remaining);
        int more = remaining > 20u;
        uint8_t L2[23];
        memset(L2, 0x2B, sizeof L2);
        L2[0] = (uint8_t)((sapi << 2) | 0x03u);
        L2[1] = (uint8_t)(((nr & 7u) << 5) | ((link->vs & 7u) << 1));
        L2[2] = (uint8_t)((count << 2) | (more ? 0x03u : 0x01u));
        if (count) memcpy(L2 + 3, info + offset, count);
        rom6_queue_lapdm(m, L2);
        link->vs = (uint8_t)((link->vs + 1u) & 7u);
        rom6_record_ack(link, more ? KIND_SEGMENT : final_kind);
        offset = (uint16_t)(offset + count);
    } while (offset < infolen);
}

static void rom6_send_sabm(struct Mad2* m, uint8_t sapi, uint8_t ack_kind) {
    Rom6Dsp* r = &m->rom6;
    Rom6LapdmLink* link = rom6_link(r, sapi);
    uint8_t L2[23];
    memset(L2, 0x2B, sizeof L2);
    rom6_link_reset(link);
    L2[0] = (uint8_t)((sapi << 2) | 0x03u);
    L2[1] = 0x3Fu;
    L2[2] = 0x01u;
    link->pending_ua_kind = ack_kind;
    rom6_queue_lapdm(m, L2);
}

static uint8_t rom6_encode_digits(uint8_t* out, const char* digits) {
    uint8_t count = 0;
    while (digits[count] && count < 20u) count++;
    uint8_t bytes = (uint8_t)((count + 1u) / 2u);
    memset(out, 0, bytes);
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t digit = (uint8_t)(digits[i] - '0');
        if (i & 1u) out[i / 2u] |= (uint8_t)(digit << 4);
        else out[i / 2u] = digit;
    }
    if (count & 1u) out[bytes - 1u] |= 0xF0u;
    return bytes;
}

static int rom6_decode_digits(char out[21], const uint8_t* encoded,
                                unsigned bytes, unsigned digits) {
    if (!digits || digits > 20 || bytes < (digits + 1u) / 2u) return 0;
    for (unsigned i = 0; i < digits; ++i) {
        uint8_t nibble = (i & 1u) ? encoded[i / 2u] >> 4 : encoded[i / 2u] & 0x0Fu;
        // GSM semi-octet strings use F in the final high nibble when the
        // address has an odd number of digits.  The IE length counts octets,
        // not digits, so stop at that terminal filler.
        if (nibble == 0x0Fu && i + 1u == digits) {
            out[i] = 0;
            return i != 0u;
        }
        if (nibble > 9u) return 0;
        out[i] = (char)('0' + nibble);
    }
    out[digits] = 0;
    return 1;
}

static uint8_t rom6_pack_gsm7(uint8_t* out, unsigned cap, const char* text) {
    uint8_t septets = (uint8_t)strlen(text);
    if (septets > 120u) septets = 120u;
    unsigned bytes = ((unsigned)septets * 7u + 7u) / 8u;
    if (bytes > cap) return 0;
    memset(out, 0, bytes);
    for (unsigned i = 0; i < septets; ++i) {
        uint8_t septet = (uint8_t)text[i] & 0x7Fu;
        unsigned bit = i * 7u, byte = bit / 8u, shift = bit % 8u;
        out[byte] |= (uint8_t)(septet << shift);
        if (shift > 1u && byte + 1u < bytes)
            out[byte + 1u] |= (uint8_t)(septet >> (8u - shift));
    }
    return septets;
}

static void rom6_unpack_gsm7(char out[161], const uint8_t* packed,
                               unsigned packed_len, unsigned septets) {
    if (septets > 160u) septets = 160u;
    unsigned n = 0;
    for (; n < septets; ++n) {
        unsigned bit = n * 7u, byte = bit / 8u, shift = bit % 8u;
        if (byte >= packed_len) break;
        unsigned value = packed[byte] >> shift;
        if (shift > 1u && byte + 1u < packed_len)
            value |= (unsigned)packed[byte + 1u] << (8u - shift);
        value &= 0x7Fu;
        out[n] = (value >= 0x20u && value <= 0x7Eu) ? (char)value : ' ';
    }
    out[n] = 0;
}

static uint16_t rom6_build_mt_call_setup(uint8_t out[64], const char* number) {
    static const uint8_t prefix[] = {
        0x03,0x05, 0x04,0x04,0x60,0x02,0x00,0x81, 0x34,0x01
    };
    memcpy(out, prefix, sizeof prefix);
    uint8_t digits[10];
    uint8_t bytes = rom6_encode_digits(digits, number);
    out[10] = 0x5C;
    out[11] = (uint8_t)(bytes + 1u);
    out[12] = 0x81;
    memcpy(out + 13, digits, bytes);
    return (uint16_t)(13u + bytes);
}

static uint16_t rom6_build_mt_sms(uint8_t out[ROM6_L3_MAX],
                                    const char* originator, const char* text,
                                    uint8_t reference) {
    uint8_t tpdu[180], rpdu[220], user[120], digits[10], sca[6];
    uint8_t origin_digits = (uint8_t)strlen(originator);
    if (origin_digits > 20u) origin_digits = 20u;
    uint8_t origin_bytes = rom6_encode_digits(digits, originator);
    uint8_t septets = rom6_pack_gsm7(user, sizeof user, text);
    unsigned user_bytes = ((unsigned)septets * 7u + 7u) / 8u;
    unsigned t = 0;
    tpdu[t++] = 0x04; tpdu[t++] = origin_digits; tpdu[t++] = 0x81;
    memcpy(tpdu + t, digits, origin_bytes); t += origin_bytes;
    tpdu[t++] = 0x00; tpdu[t++] = 0x00;
    {
        time_t stamp = time(NULL);
        struct tm utc = {0};
#if defined(_WIN32)
        gmtime_s(&utc, &stamp);
#else
        struct tm* value = gmtime(&stamp);
        if (value) utc = *value;
#endif
        tpdu[t++] = rom6_nitz_bcd((unsigned)((utc.tm_year + 1900) % 100));
        tpdu[t++] = rom6_nitz_bcd((unsigned)(utc.tm_mon + 1));
        tpdu[t++] = rom6_nitz_bcd((unsigned)utc.tm_mday);
        tpdu[t++] = rom6_nitz_bcd((unsigned)utc.tm_hour);
        tpdu[t++] = rom6_nitz_bcd((unsigned)utc.tm_min);
        tpdu[t++] = rom6_nitz_bcd((unsigned)utc.tm_sec);
        tpdu[t++] = 0x00;
    }
    tpdu[t++] = septets;
    memcpy(tpdu + t, user, user_bytes); t += user_bytes;

    sca[0] = 0x91;
    uint8_t sca_bytes = rom6_encode_digits(sca + 1, "1234567890");
    unsigned p = 0;
    rpdu[p++] = 0x01; rpdu[p++] = reference;
    rpdu[p++] = (uint8_t)(sca_bytes + 1u);
    memcpy(rpdu + p, sca, sca_bytes + 1u); p += sca_bytes + 1u;
    rpdu[p++] = 0x00;
    rpdu[p++] = (uint8_t)t;
    memcpy(rpdu + p, tpdu, t); p += t;

    out[0] = 0x09; out[1] = 0x01; out[2] = (uint8_t)p;
    memcpy(out + 3, rpdu, p);
    return (uint16_t)(p + 3u);
}

static int rom6_get_rpdu(const uint8_t* info, uint16_t len,
                           const uint8_t** rpdu, uint16_t* rpdu_len) {
    if (len < 4u) return 0;
    unsigned offset = (len >= 5u && info[2] == 0x01u) ? 4u : 3u;
    unsigned count = info[offset - 1u];
    if (!count || offset + count > len) return 0;
    *rpdu = info + offset;
    *rpdu_len = (uint16_t)count;
    return 1;
}

static void rom6_decode_mo_call(Rom6Dsp* r, const uint8_t* info, uint16_t len) {
    r->remoteNumber[0] = 0;
    for (unsigned i = 2; i + 2u < len; ++i) {
        if (info[i] != 0x5Eu) continue;
        unsigned count = info[i + 1u];
        if (count < 2u || i + 2u + count > len) continue;
        if (rom6_decode_digits(r->remoteNumber, info + i + 3u,
                                 count - 1u, (count - 1u) * 2u)) {
            size_t n = strlen(r->remoteNumber);
            if (n && r->remoteNumber[n - 1u] == '0' &&
                ((info[i + 2u + count - 1u] >> 4) & 0x0Fu) == 0x0Fu)
                r->remoteNumber[n - 1u] = 0;
            return;
        }
    }
    memcpy(r->remoteNumber, "unknown", 8);
}

static uint8_t rom6_decode_mo_sms(Rom6Dsp* r, const uint8_t* info, uint16_t len) {
    const uint8_t* rpdu; uint16_t rpdu_len;
    if (!rom6_get_rpdu(info, len, &rpdu, &rpdu_len) ||
        rpdu_len < 2u || (rpdu[0] & 7u) != 0u) return 0;
    uint8_t reference = rpdu[1];
    unsigned offset = 2;
    for (int field = 0; field < 2; ++field) {
        if (offset >= rpdu_len) return reference;
        unsigned count = rpdu[offset++];
        if (offset + count > rpdu_len) return reference;
        offset += count;
    }
    if (offset >= rpdu_len) return reference;
    unsigned tpdu_len = rpdu[offset++];
    if (tpdu_len < 7u || offset + tpdu_len > rpdu_len) return reference;
    const uint8_t* tpdu = rpdu + offset;
    unsigned digits = tpdu[2], digit_bytes = (digits + 1u) / 2u;
    if (digits && digits <= 20u && 4u + digit_bytes + 3u <= tpdu_len)
        rom6_decode_digits(r->remoteNumber, tpdu + 4, digit_bytes, digits);
    offset = 4u + digit_bytes;
    if (offset + 3u > tpdu_len) return reference;
    offset += 2u; // PID + DCS
    switch ((tpdu[0] >> 3) & 3u) {
    case 2: offset += 1u; break;
    case 1: case 3: offset += 7u; break;
    default: break;
    }
    if (offset >= tpdu_len) return reference;
    unsigned septets = tpdu[offset++];
    rom6_unpack_gsm7(r->lastSmsText, tpdu + offset,
                       tpdu_len - offset, septets);
    return reference;
}

// Queue a UA response (addr=(sapi<<2)|1, ctrl=0x63|P) echoing the command's length + information.
static void rom6_send_ua(struct Mad2* m, uint8_t sapi, uint8_t pbit, uint8_t li,
                           const uint8_t* info, uint8_t infolen) {
    uint8_t L2[23];
    for (int i = 0; i < 23; ++i) L2[i] = 0x2B;
    L2[0] = (uint8_t)((sapi << 2) | 0x01u);
    L2[1] = (uint8_t)(0x63u | (pbit & 0x10u));
    L2[2] = li;
    for (uint8_t i = 0; i < infolen && i < 20; ++i) L2[3 + i] = info[i];
    rom6_queue_lapdm(m, L2);
}

// Queue an RR supervisory response (addr=(sapi<<2)|1 BSS response, ctrl=(N(R)<<5)|(final?0x11:0x01)).
static void rom6_send_rr(struct Mad2* m, uint8_t sapi, uint8_t nr, int final) {
    uint8_t L2[23];
    for (int i = 0; i < 23; ++i) L2[i] = 0x2B;
    L2[0] = (uint8_t)((sapi << 2) | 0x01u);
    L2[1] = (uint8_t)(((nr & 7u) << 5) | (final ? 0x11u : 0x01u));
    L2[2] = 0x01;
    rom6_queue_lapdm(m, L2);
}

static void rom6_queue_channel_release(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    static const uint8_t channel_release[3] = { 0x06, 0x0D, 0x00 };
    r->regState = REG_AWAIT_REL_ACK;
    rom6_send_iframes(m, 0, channel_release, sizeof channel_release,
                        rom6_link(r, 0)->vr, KIND_CHAN_REL);
}

static uint64_t rom6_next_paging_group_cycle(const Rom6Dsp* r, uint64_t cycles) {
    uint64_t cpf = rom6_cpf();
    uint64_t currentFrame = cycles / cpf;
    uint64_t currentMultiframe = currentFrame / 51u;
    uint64_t deltaMultiframes =
        (r->pagingPhase + 2u - (currentMultiframe % 2u)) % 2u;
    uint64_t targetFrame =
        (currentMultiframe + deltaMultiframes) * 51u + r->pagingOffset;
    uint64_t targetCycle = targetFrame * cpf;
    if (targetCycle <= cycles) targetCycle += 2u * 51u * cpf;
    return targetCycle;
}

static void rom6_refresh_paging_identity(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    char imsi[16];
    mad2_sim_current_imsi(m, imsi);
    size_t nd = strlen(imsi);
    if (nd < 5 || nd > 15) return;

    memset(r->pagingMobid, 0xFF, sizeof r->pagingMobid);
    r->pagingMobid[0] = (uint8_t)(((imsi[0] - '0') << 4) |
                                  ((nd & 1u) ? 0x09u : 0x01u));
    for (size_t i = 1; i < nd; ++i) {
        uint8_t digit = (uint8_t)(imsi[i] - '0');
        size_t bi = (i + 1u) / 2u;
        if (i & 1u)
            r->pagingMobid[bi] = (uint8_t)((r->pagingMobid[bi] & 0xF0u) | digit);
        else
            r->pagingMobid[bi] = (uint8_t)((r->pagingMobid[bi] & 0x0Fu) | (digit << 4));
    }

    unsigned mod1000 = 0;
    for (size_t i = nd > 3 ? nd - 3 : 0; i < nd; ++i)
        mod1000 = mod1000 * 10u + (unsigned)(imsi[i] - '0');
    unsigned group = mod1000 % (9u * 2u);            // BS_PA_MFRMS=2
    r->pagingPhase = (uint8_t)(group / 9u);
    r->pagingOffset = ROM6_CCCH_OFFSETS[group % 9u];

    if (getenv("ROM6_LOG"))
        fprintf(stderr,
                "[rom6] paging IMSI=%s group=%u phase=%u offset=%u\n",
                imsi, group, r->pagingPhase, r->pagingOffset);
}

static void rom6_start_next_incoming_paging(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    if (!r->suppressImsiPaging || r->dedicatedConfigured ||
        r->incomingHead == r->incomingTail || r->incomingPagingActive)
        return;
    rom6_refresh_paging_identity(m);
    r->incomingPagingActive = 1;
    r->incomingPagingAnswered = 0;
    r->incomingPagingBursts = 0;
    r->nextIncomingPagingCycle =
        rom6_next_paging_group_cycle(r, rom6_now(r));
}

static void rom6_queue_cipher_command(struct Mad2* m, uint8_t service) {
    Rom6Dsp* r = &m->rom6;
    static const uint8_t cipher[3] = { 0x06, 0x35, 0x01 };
    r->activeService = service;
    r->regState = REG_AWAIT_CIPHER_ACK;
    rom6_send_iframes(m, 0, cipher, sizeof cipher,
                        rom6_link(r, 0)->vr, KIND_CIPHER_COMMAND);
}

static void rom6_queue_mt_sms(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    uint8_t info[ROM6_L3_MAX];
    uint16_t len = rom6_build_mt_sms(info, r->activeIncomingAddress,
                                       r->activeIncomingText, r->nextSmsReference++);
    rom6_send_iframes(m, 3, info, len, rom6_link(r, 3)->vr,
                        KIND_MT_SMS_CP_DATA);
    if (rom6_log_enabled())
        fprintf(stderr, "[rom6] MT SMS CP-DATA queued from %s text=\"%s\"\n",
                r->activeIncomingAddress, r->activeIncomingText);
}

// Network step: a downlink I-frame we sent has just been acknowledged by the MS -> advance the FSM.
static void rom6_net_on_ack(struct Mad2* m, uint8_t sapi, uint8_t kind) {
    Rom6Dsp* r = &m->rom6;
    int log = rom6_log_enabled();
    if (kind == KIND_LU_ACCEPT && r->regState == REG_AWAIT_LU_ACK) {
        // MM INFORMATION (empty network name -> omit 0x43 IE): 05 32 47 + live NITZ.
        uint8_t mminfo[10];
        static const uint8_t chanrel[3] = { 0x06, 0x0D, 0x00 };   // CHANNEL RELEASE (RR)
        rom6_build_mm_information(mminfo);
        r->regState = REG_AWAIT_REL_ACK;
        rom6_send_iframes(m, 0, mminfo, sizeof mminfo,
                            rom6_link(r, 0)->vr, KIND_MM_INFO);
        rom6_send_iframes(m, 0, chanrel, sizeof chanrel,
                            rom6_link(r, 0)->vr, KIND_CHAN_REL);
        if (log) fprintf(stderr, "[rom6] LU-Accept acked -> MM-Information + Channel-Release queued\n");
    } else if (kind == KIND_CHAN_REL && r->regState == REG_AWAIT_REL_ACK) {
        r->regState = REG_RELEASED;                              // == REGISTERED
        r->suppressImsiPaging = 1;                               // post-reg: no-identity paging fill
        r->activeService = SVC_NONE;
        r->activeIncomingKind = 0;
        if (r->callState && r->callState != 3u) r->callState = 3;
        r->incomingPagingActive = 0;
        r->incomingPagingAnswered = 0;
        r->nextIncomingPagingCycle = UINT64_MAX;
        rom6_start_next_incoming_paging(m);
        if (log) fprintf(stderr, "[rom6] Channel-Release acked -> REGISTERED (Released)\n");
    } else if (kind == KIND_CIPHER_COMMAND && r->regState == REG_AWAIT_CIPHER_ACK) {
        r->regState = REG_AWAIT_CIPHER_COMPLETE;
        if (log) fprintf(stderr, "[rom6] Ciphering-Mode-Command acknowledged\n");
    } else if (kind == KIND_SAPI3_ESTABLISH && sapi == 3u &&
               r->regState == REG_MM_ACTIVE && r->activeService == SVC_MT_SMS) {
        rom6_queue_mt_sms(m);
    }
}

// AcknowledgePendingDownlinkFrames(N(R)): if any queued I-frame is covered by the MS's N(R), dequeue
// pending acks in order (running the network step per kind) up to and including that frame.
static void rom6_ack_pending(struct Mad2* m, uint8_t sapi, uint8_t nr) {
    Rom6Dsp* r = &m->rom6;
    Rom6LapdmLink* link = rom6_link(r, sapi);
    int found = 0;
    for (uint8_t i = 0; i < link->ack_count; ++i) {
        uint8_t idx = (uint8_t)((link->ack_head + i) % ROM6_LAPDM_ACK_N);
        if (link->ack_rs[idx] == nr) { found = 1; break; }
    }
    if (!found) return;                                          // N(R) covers nothing -> not an ack
    while (link->ack_count > 0) {
        uint8_t idx = link->ack_head;
        uint8_t rs = link->ack_rs[idx], kind = link->ack_kind[idx];
        link->ack_head = (uint8_t)((link->ack_head + 1u) % ROM6_LAPDM_ACK_N);
        link->ack_count--;
        rom6_net_on_ack(m, sapi, kind);                       // may append new pending acks
        if (rs == nr) break;
    }
}

static int rom6_pop_incoming(Rom6Dsp* r) {
    if (r->incomingHead == r->incomingTail) return 0;
    Rom6IncomingService* service = &r->incoming[r->incomingHead];
    r->activeIncomingKind = service->kind;
    memcpy(r->activeIncomingAddress, service->address, sizeof service->address);
    memcpy(r->activeIncomingText, service->text, sizeof service->text);
    r->incomingHead = (uint8_t)((r->incomingHead + 1u) % ROM6_INCOMING_N);
    return 1;
}

static void rom6_net_established(struct Mad2* m, const uint8_t* info, uint16_t len) {
    Rom6Dsp* r = &m->rom6;
    int log = rom6_log_enabled();
    if (len >= 2 && (info[0] & 0x0Fu) == 0x05u && info[1] == 0x08u && r->regState == REG_IDLE) {
        uint8_t luacc[7] = { 0x05, 0x02, 0, 0, 0, 0x00, 0x01 };
        mad2_sim_current_plmn(m, luacc + 2);
        r->regState = REG_AWAIT_LU_ACK;
        rom6_send_iframes(m, 0, luacc, sizeof luacc,
                            rom6_link(r, 0)->vr, KIND_LU_ACCEPT);
        if (log) fprintf(stderr, "[rom6] LU-Request accepted -> LU-Accept I-frame queued\n");
    } else if (len >= 3u && (info[0] & 0x0Fu) == 0x05u &&
               info[1] == 0x24u && r->regState == REG_IDLE) {
        uint8_t service = info[2] & 0x0Fu;
        if (service == SVC_MO_CALL || service == SVC_EMERGENCY || service == SVC_SMS) {
            rom6_queue_cipher_command(m, service);
            if (log) fprintf(stderr, "[rom6] CM-Service-Request service=%u -> cipher command\n", service);
        }
    } else if (len >= 2u && (info[0] & 0x0Fu) == 0x06u &&
               info[1] == 0x27u && r->regState == REG_IDLE) {
        if (rom6_pop_incoming(r)) {
            uint8_t service = r->activeIncomingKind == INCOMING_CALL ? SVC_MT_CALL : SVC_MT_SMS;
            rom6_queue_cipher_command(m, service);
            if (log) fprintf(stderr, "[rom6] Paging-Response -> cipher command service=%u\n", service);
        }
    }
}

static void rom6_queue_call_message(struct Mad2* m, uint8_t ti_pd,
                                      uint8_t type, uint8_t kind) {
    Rom6Dsp* r = &m->rom6;
    uint8_t info[2] = { (uint8_t)(ti_pd ^ 0x80u), type };
    rom6_send_iframes(m, 0, info, sizeof info, rom6_link(r, 0)->vr, kind);
}

static void rom6_net_active(struct Mad2* m, uint8_t sapi,
                              const uint8_t* info, uint16_t len) {
    Rom6Dsp* r = &m->rom6;
    int log = rom6_log_enabled();
    if (len < 2u) return;
    uint8_t pd = info[0] & 0x0Fu;
    uint8_t type = info[1];
    uint8_t cc_type = type & 0xBFu;

    if (r->regState == REG_AWAIT_CIPHER_COMPLETE) {
        if (pd == 0x06u && type == 0x32u) {
            uint8_t mm_info[10];
            r->regState = REG_MM_ACTIVE;
            rom6_build_mm_information(mm_info);
            rom6_send_iframes(m, 0, mm_info, sizeof mm_info,
                                rom6_link(r, 0)->vr, KIND_MM_INFO);
            if (r->activeService == SVC_MT_CALL) {
                uint8_t setup[64];
                uint16_t setup_len = rom6_build_mt_call_setup(
                    setup, r->activeIncomingAddress);
                r->callState = 1; r->callDirection = 2;
                memcpy(r->remoteNumber, r->activeIncomingAddress,
                       sizeof r->activeIncomingAddress);
                rom6_send_iframes(m, 0, setup, setup_len,
                                    rom6_link(r, 0)->vr, KIND_MT_CALL_SETUP);
            } else if (r->activeService == SVC_MT_SMS) {
                rom6_send_sabm(m, 3, KIND_SAPI3_ESTABLISH);
            }
            if (log) fprintf(stderr, "[rom6] Ciphering-Mode-Complete -> MM connection active\n");
        }
        return;
    }
    if (r->regState != REG_MM_ACTIVE) return;

    if (pd == 0x03u && cc_type == 0x05u && r->activeService == SVC_MO_CALL) {
        rom6_decode_mo_call(r, info, len);
        r->outgoingCallCount++;
        r->callDirection = 1; r->callState = 1;
        rom6_queue_call_message(m, info[0], 0x02, KIND_CALL_PROCEEDING);
        rom6_queue_call_message(m, info[0], 0x01, KIND_ALERTING);
        rom6_queue_call_message(m, info[0], 0x07, KIND_CONNECT);
        if (log) fprintf(stderr, "[rom6] MO CALL SETUP to %s -> PROCEEDING/ALERTING/CONNECT\n",
                         r->remoteNumber);
    } else if (pd == 0x03u && cc_type == 0x0Eu &&
               r->activeService == SVC_EMERGENCY) {
        memcpy(r->remoteNumber, "emergency", 10);
        r->outgoingCallCount++;
        r->callDirection = 1; r->callState = 1;
        rom6_queue_call_message(m, info[0], 0x02, KIND_CALL_PROCEEDING);
        rom6_queue_call_message(m, info[0], 0x07, KIND_CONNECT);
    } else if (pd == 0x03u && cc_type == 0x08u &&
               r->activeService == SVC_MT_CALL) {
        r->callState = 1;
        if (log) fprintf(stderr, "[rom6] MT CALL CONFIRMED received\n");
    } else if (pd == 0x03u && cc_type == 0x01u &&
               r->activeService == SVC_MT_CALL) {
        r->callState = 1;
        if (log) fprintf(stderr, "[rom6] MT CALL ALERTING received\n");
    } else if (pd == 0x03u && cc_type == 0x07u &&
               r->activeService == SVC_MT_CALL) {
        r->callState = 2;
        rom6_queue_call_message(m, info[0], 0x0F, KIND_CONNECT_ACK);
        if (log) fprintf(stderr, "[rom6] MT CALL answered -> CONNECT ACKNOWLEDGE\n");
    } else if (pd == 0x03u && cc_type == 0x0Fu) {
        r->callState = 2;
        if (log) fprintf(stderr, "[rom6] CALL connected\n");
    } else if (pd == 0x03u && cc_type == 0x25u) {
        r->callState = 3;
        rom6_queue_call_message(m, info[0], 0x2D, KIND_RELEASE);
        if (log) fprintf(stderr, "[rom6] CALL DISCONNECT -> RELEASE\n");
    } else if (pd == 0x03u && cc_type == 0x2Au) {
        r->callState = 3;
        rom6_queue_channel_release(m);
        if (log) fprintf(stderr, "[rom6] CALL RELEASE COMPLETE -> channel release\n");
    } else if (pd == 0x09u && type == 0x01u && r->activeService == SVC_SMS) {
        const uint8_t* rpdu = 0; uint16_t rpdu_len = 0;
        uint8_t reference = rom6_decode_mo_sms(r, info, len);
        uint8_t cp_ack[2] = { (uint8_t)(info[0] ^ 0x80u), 0x04 };
        rom6_send_iframes(m, sapi, cp_ack, sizeof cp_ack,
                            rom6_link(r, sapi)->vr, KIND_CP_ACK);
        if (rom6_get_rpdu(info, len, &rpdu, &rpdu_len) &&
            rpdu_len >= 2u && (rpdu[0] & 7u) == 0u) {
            uint8_t rp_ack[5] = {
                (uint8_t)(info[0] ^ 0x80u), 0x01, 0x02, 0x03, reference
            };
            r->outgoingSmsCount++;
            rom6_send_iframes(m, sapi, rp_ack, sizeof rp_ack,
                                rom6_link(r, sapi)->vr, KIND_RP_ACK);
        }
        if (log) fprintf(stderr, "[rom6] MO SMS to %s text=\"%s\" -> CP/RP ACK\n",
                         r->remoteNumber, r->lastSmsText);
    } else if (pd == 0x09u && type == 0x04u) {
        if (r->activeService == SVC_SMS) rom6_queue_channel_release(m);
    } else if (pd == 0x09u && type == 0x01u && r->activeService == SVC_MT_SMS) {
        uint8_t cp_ack[2] = { (uint8_t)(info[0] ^ 0x80u), 0x04 };
        rom6_send_iframes(m, sapi, cp_ack, sizeof cp_ack,
                            rom6_link(r, sapi)->vr, KIND_CP_ACK);
        rom6_queue_channel_release(m);
        if (log) fprintf(stderr, "[rom6] MT SMS RP response -> CP ACK + channel release\n");
    }
}

// SABM (with optional LU-Request info): reset the link, queue UA, run establishment if info present.
static void rom6_lapdm_sabm(struct Mad2* m, const uint8_t* L2, unsigned l2len) {
    Rom6Dsp* r = &m->rom6;
    uint8_t sapi = (uint8_t)((L2[0] >> 2) & 7u);
    uint8_t ctrl = L2[1], li = L2[2];
    uint8_t infolen = (uint8_t)(li >> 2);
    if (3u + infolen > l2len) infolen = (uint8_t)(l2len > 3 ? l2len - 3 : 0);
    rom6_link_reset(rom6_link(r, sapi));
    uint8_t info[20];
    for (uint8_t i = 0; i < infolen && i < 20; ++i) info[i] = L2[3 + i];
    rom6_send_ua(m, sapi, ctrl, li, info, infolen);
    if (infolen > 0) rom6_net_established(m, info, infolen);
}

// HandleMdiSendBlock -> LAPDm uplink dispatch (spec §REGISTRATION L3). L2 = 23-byte block.
static void rom6_lapdm_uplink(struct Mad2* m, uint8_t logch, const uint8_t* L2, unsigned l2len) {
    Rom6Dsp* r = &m->rom6;
    int log = getenv("ROM6_LOG") != 0;
    (void)logch;
    if (l2len < 3) return;
    uint8_t addr = L2[0], ctrl = L2[1], li = L2[2];
    uint8_t sapi = (uint8_t)((addr >> 2) & 7u);
    Rom6LapdmLink* link = rom6_link(r, sapi);
    if ((addr & 1u) == 0) return;                               // EA bit must be set

    // SABM: ctrl&0xEF==0x2F, LI odd, M=0.
    if ((ctrl & 0xEFu) == 0x2Fu && (li & 1u) && !(li & 2u)) {
        if (log) fprintf(stderr, "[rom6] LAPDm SABM sapi=%u info=%u -> UA\n", sapi, li >> 2);
        rom6_lapdm_sabm(m, L2, l2len);
        return;
    }
    // DISC: ctrl&0xEF==0x43, LI==0x01. Queue UA, then release the SDCCH once drained.
    if ((ctrl & 0xEFu) == 0x43u && li == 0x01u) {
        if (log) fprintf(stderr, "[rom6] LAPDm DISC sapi=%u -> UA + release\n", sapi);
        rom6_send_ua(m, sapi, ctrl, 0x01u, 0, 0);
        r->dedicatedReleasePending = 1;
        return;
    }
    // I-frame: ctrl bit0==0, LI odd.
    if ((ctrl & 1u) == 0 && (li & 1u)) {
        uint8_t ns = (uint8_t)((ctrl >> 1) & 7u), nr = (uint8_t)((ctrl >> 5) & 7u);
        uint8_t moreData = (uint8_t)((li & 2u) != 0), infolen = (uint8_t)(li >> 2);
        int pollFinal = (ctrl & 0x10u) != 0;
        if (3u + infolen > l2len) infolen = (uint8_t)(l2len > 3 ? l2len - 3 : 0);
        uint8_t complete[ROM6_L3_MAX]; int haveComplete = 0; uint16_t completeLen = 0;
        if (ns == link->vr) {                                    // in-sequence -> accept
            if (moreData) {
                for (uint8_t i = 0; i < infolen && link->seg_len < sizeof link->seg; ++i)
                    link->seg[link->seg_len++] = L2[3 + i];
            } else {
                for (uint16_t i = 0; i < link->seg_len && completeLen < sizeof complete; ++i)
                    complete[completeLen++] = link->seg[i];
                for (uint8_t i = 0; i < infolen && completeLen < sizeof complete; ++i)
                    complete[completeLen++] = L2[3 + i];
                link->seg_len = 0; haveComplete = 1;
            }
            link->vr = (uint8_t)((link->vr + 1u) & 7u);
        }
        if (log) fprintf(stderr, "[rom6] LAPDm I sapi=%u ns=%u nr=%u m=%u info=%u\n",
                         sapi, ns, nr, moreData, infolen);
        rom6_ack_pending(m, sapi, nr);
        rom6_send_rr(m, sapi, link->vr, pollFinal);
        if (haveComplete) {
            if (r->regState == REG_IDLE) rom6_net_established(m, complete, completeLen);
            else rom6_net_active(m, sapi, complete, completeLen);
        }
        return;
    }
    // RR supervisory: ctrl low nibble 0x01, LI==0x01 -> pure ack.
    if ((ctrl & 0x0Fu) == 0x01u && li == 0x01u) {
        uint8_t nr = (uint8_t)((ctrl >> 5) & 7u);
        if (log) fprintf(stderr, "[rom6] LAPDm RR sapi=%u nr=%u\n", sapi, nr);
        rom6_ack_pending(m, sapi, nr);
        return;
    }
    // UA response to a network-originated SABM (used to establish SMS SAPI 3).
    if ((addr & 3u) == 3u && (ctrl & 0xEFu) == 0x63u && li == 0x01u) {
        uint8_t kind = link->pending_ua_kind;
        link->pending_ua_kind = 0;
        if (log) fprintf(stderr, "[rom6] LAPDm UA sapi=%u\n", sapi);
        if (kind) rom6_net_on_ack(m, sapi, kind);
    }
}

// ===========================================================================================
// HandleMdiPacket — the reference dispatch (spec §DISPATCH). op + a clean, de-wrapped payload
// buffer buf[0..plen). Enqueues d2m replies (immediate FIFO or time-ordered delayed).
// ===========================================================================================
static void rom6_handle_mdi(struct Mad2* m, uint8_t op, const uint8_t* buf, unsigned plen) {
    Rom6Dsp* r = &m->rom6;
    int log = getenv("ROM6_LOG") != 0;
    switch (op) {
    case 0x56: {                                    // carrier search -> PostSchBlock (Dsp.cs)
        // Reference PostSchBlock: scan the ARFCN-pair list, skip 0000/FFFF, take the first real
        // carrier as the serving cell, emit SCH + SI. (Was: only buf[0..1], one-shot guard.)
        for (unsigned i = 0; i + 1 < plen; i += 2) {
            uint16_t arfcn = (uint16_t)(((uint16_t)buf[i] << 8) | buf[i + 1]);
            if (arfcn == 0x0000u || arfcn == 0xFFFFu) continue;
            r->measurementArfcn = arfcn;
            r->servingArfcn = arfcn;
            rom6_enq_sch(m);                       // SCH first; SI burst may be delayed (SIDELAY_FR)
            rom6_enq_si_burst_delayed(m);
            r->nextBcchBroadcastCycle = rom6_now(r) + (rom6_cpf() * 217u) / 4u;
            if (log) fprintf(stderr, "[rom6] 0x56 search -> serving ARFCN=%u @step=%llu\n",
                             arfcn, (unsigned long long)m->dsp_steps);
            break;
        }
        break;
    }
    case 0x02: {                                    // channel configure -> (0x8f) 0x89 + captures
        uint8_t confirm = (plen > 6) ? (uint8_t)(buf[6] & 1u) : 0u;
        if (plen >= 12) r->measurementArfcn = (uint16_t)(((uint16_t)buf[10] << 8) | buf[11]);
        uint8_t logch = (plen > 8) ? buf[8] : 0u;
        // Search-phase terminator (bitplane/nokia-dct3-re camp sequence:
        // 0x02 CHANNEL_CONFIGURE -> 0x8f NO_PSW_LEFT -> 0x89 CHANNEL_CHANGED_CNF). NO_PSW_LEFT
        // "closes the initial power-scan work list", telling the firmware to stop searching and
        // commit to the serving cell — the downlink our cell-selection wall was missing. One-shot:
        // "unsolicited or replayed confirmations are forbidden". Payload is not inspected on the
        // controller state-1 path; the arrival is what transitions the controller state.
        if (logch == 0x50 && !r->noPswSent) {
            uint8_t pl8f[1] = { 0x00 };
            rom6_enqueue(m, 0x8F, pl8f, 1);       // 0x8F NO_PSW_LEFT (search-phase terminator)
            r->noPswSent = 1;
            if (log) fprintf(stderr, "[rom6] 0x8F NO_PSW_LEFT (search done, commit serving cell) @step=%llu\n",
                             (unsigned long long)m->dsp_steps);
        }
        uint8_t pl89[1] = { confirm };
        rom6_enqueue(m, 0x89, pl89, 1);           // 0x89 CHANNEL_CHANGED_CNF (RESPONSE)
        if (logch == 0x60) {                        // CCCH configured
            r->ccchConfigured = 1;
            if (plen >= 12) r->ccchArfcn = (uint16_t)(((uint16_t)buf[10] << 8) | buf[11]);
            r->ccchBsic = (plen > 1) ? buf[1] : ROM6_BSIC;
            rom6_enq_sch(m); rom6_enq_si_burst(m);
        } else if (logch == 0x50) {                 // BCCH / serving cell
            // Dsp.cs CaptureServingCell: BSIC = payload[1], ARFCN = payload[10..11], meas = serving.
            r->servingBsic = (plen > 1) ? buf[1] : r->servingBsic;
            if (plen >= 12) { r->servingArfcn = (uint16_t)(((uint16_t)buf[10] << 8) | buf[11]);
                              r->measurementArfcn = r->servingArfcn; }
            rom6_enq_si_burst(m);
        } else if (logch == 0x80) {                 // dedicated (SDCCH) — STAGE 2 capture
            rom6_drop_idle_radio_backlog(r);
            r->dedicatedConfigured = 1;
            r->dedicatedReleasePending = 0;
            if (plen >= 12) r->dedicatedArfcn = (uint16_t)(((uint16_t)buf[10] << 8) | buf[11]);
            r->dedicatedBsic = (plen > 1) ? buf[1] : r->servingBsic;
            // Arm the 51-frame downlink-fill + block-request cadence so the firmware transmits its
            // uplink LAPDm (the LU-Request) on the SDCCH; reset the LAPDm link + LU FSM.
            uint64_t cpf = rom6_cpf();
            uint64_t period = 51u * cpf;
            uint64_t frameStart = (rom6_now(r) / cpf) * cpf;
            uint32_t t3 = rom6_current_fn(r) % 51u;
            uint32_t fillDelta = (51u - t3) % 51u;
            uint32_t requestDelta = (15u + 51u - t3) % 51u;
            r->nextDedicatedCycle = frameStart + (uint64_t)fillDelta * cpf;
            if (r->nextDedicatedCycle <= rom6_now(r)) r->nextDedicatedCycle += period;
            r->nextDedicatedBlkReqCycle = frameStart + (uint64_t)requestDelta * cpf;
            if (r->nextDedicatedBlkReqCycle <= rom6_now(r)) r->nextDedicatedBlkReqCycle += period;
            if (r->nextDedicatedBlkReqCycle < r->nextDedicatedCycle)
                r->nextDedicatedBlkReqCycle += period;
            r->dedicatedHead = r->dedicatedTail = 0;
            rom6_reset_links(r);
            r->regState = REG_IDLE;
            r->activeService = SVC_NONE;
            r->activeIncomingKind = 0;
            if (log) fprintf(stderr, "[rom6] 0x02 dedicated (SDCCH logch 0x80) captured ARFCN=%u @step=%llu\n",
                             r->dedicatedArfcn, (unsigned long long)m->dsp_steps);
        }
        if (log) fprintf(stderr, "[rom6] 0x02 channel-configure logch=0x%02X confirm=%u @step=%llu\n",
                         logch, confirm, (unsigned long long)m->dsp_steps);
        break;
    }
    case 0x0C: {                                    // RACH -> Immediate Assignment (delayed Cps/10)
        uint8_t ra = (plen > 2) ? buf[2] : 0u;
        // Request-reference frame the handset RACHed on (reference formula): if payload[1]==0x01 the
        // reduced FN is carried at [4..5]; else derive from the current frame + the [3] offset. mod 42432.
        uint32_t reqfn = (plen >= 6 && buf[1] == 0x01)
                       ? (uint32_t)(((uint32_t)buf[4] << 8) | buf[5])
                       : (rom6_current_fn(r) + (plen >= 4 ? buf[3] : 0u)) % 42432u;
        r->rachReference = ra;
        r->rachRequestFn = reqfn;
        if (r->incomingPagingActive) {
            r->incomingPagingAnswered = 1;
            r->nextIncomingPagingCycle = UINT64_MAX;
        }
        // MDISND 0x0C is the already-observed handset RACH event. `reqfn` is
        // the request-reference label the later IA must echo; it is not a
        // second host deadline. Schedule the network response from this event,
        // exactly as Noks does.
        //
        // First report the transmitted burst back uplink-side: d2m 0x84 RA_INFO {RA, absolute
        // FN}. That is the ONLY thing the firmware needs to record its own outstanding request
        // reference — its own task-10 -> task-13 chain converts our frame number into the
        // {T1',T3,T2} tuple its RR matcher compares the Immediate Assignment against (full
        // trace in rom6_b_ra_info above). Reporting the burst instead of forging the MCU's
        // record of it is what let the MCU-RAM poke be deleted.
        {
            uint8_t pl[8];
            uint8_t len = rom6_b_ra_info(pl, ra, reqfn);
            rom6_enqueue(m, 0x84, pl, len);
        }
        rom6_enq_imm_assign_at(m, ra, reqfn, rom6_now(r));
        if (log) fprintf(stderr,
                         "[rom6] RACH observed RA=0x%02X reqFN=%u @cycle=%llu step=%llu\n",
                         ra, reqfn, (unsigned long long)rom6_now(r),
                         (unsigned long long)m->dsp_steps);
        break;
    }
    case 0x0F:                                      // neighbour list / MSI -> 0x83 RSSI
    case 0x46: {
        // Dsp.cs RssiArfcn: servingArfcn, else measurementArfcn, else 0x03EC.
        uint16_t rssi_arfcn = r->servingArfcn ? r->servingArfcn
                            : (r->measurementArfcn ? r->measurementArfcn : 0x03ECu);
        uint8_t pl[8]; uint8_t len = rom6_b_rssi83(pl, rssi_arfcn, ROM6_DEFAULT_RSSI);
        rom6_enqueue(m, 0x83, pl, len);
        break;
    }
    case 0x11: {                                    // nmeas instructions -> 0x88 (delayed Cps/217)
        // Dsp.cs EnqueueNeighbourTimingOffset: carrier from request[6..7] first, else meas/serving.
        uint16_t carrier = (plen >= 8) ? (uint16_t)(((uint16_t)buf[6] << 8) | buf[7]) : r->measurementArfcn;
        if (carrier == 0) carrier = r->measurementArfcn ? r->measurementArfcn
                                   : (r->servingArfcn ? r->servingArfcn : r->servingArfcn);
        uint8_t pl[16]; uint8_t len = rom6_b_nbrtim(pl, carrier, rom6_current_fn(r));
        rom6_enqueue_after(m, rom6_cpf(), 0x88, pl, len);   // Cps/217 == cpf
        break;
    }
    case 0x55: {                                    // blind band search -> 0x8B ALL_RSSI_RESULTS
        // 0x55 is the *untargeted* counterpart of 0x56. The MCU's cell-selection sub-FSM picks
        // between the two by walking its 3x9-byte candidate-carrier count table: any group non-empty
        // -> targeted search (0x56, a 160-byte SEARCH_LIST of known carriers); ALL groups empty ->
        // blind band search (0x55). The 3310 v5.79 ships that table with one populated group, so it
        // only ever emits 0x56; the 3410 v5.46 and 5210 v5.40 ship it all-zero and open with 0x55.
        // Same code, different data — which is why a 0x56-only responder walls those two models.
        // Payload is 2 bytes {band index, attempts} (both observed emitting {03 05}); it is NOT an
        // ARFCN, so there is nothing carrier-specific to honour — the DSP is being asked "sweep the
        // band and tell me everything you can hear".
        //
        // The answer is a full power scan: 0x8B ALL_RSSI_RESULTS (reference EnqueueAllRssiResults,
        // reused verbatim via rom6_b_rssi8b — the same 162-byte body the 0x4B MORE_RSSI path emits).
        // Of the handlers the FSM can reach while parked on soft-timer 36, 0x80/0x8A are gated on a
        // flag that is still zero here and 0x87/0x89 loop back into the wait, so 0x8B is the only
        // reply that advances it unconditionally. Once it lands the MCU has >=1 candidate carrier,
        // its count table goes non-zero, the selector flips to the targeted arm and the model
        // rejoins the 0x56 -> 0x02 -> SCH/BCCH camp path the 3310 already walks.
        //
        // The reported carrier is the cell the rest of this engine models (mdi_gsm.c SERVING_ARFCN
        // 586 / PLMN from the active SIM) so the blind sweep hands back the very cell the following
        // 0x56/0x02 exchange will configure; every other slot reports 0x80 = nothing heard.
        uint16_t arfcn = r->servingArfcn ? r->servingArfcn
                       : (r->measurementArfcn ? r->measurementArfcn : ROM6_SEARCH_ARFCN);
        r->measurementArfcn = arfcn;                // RssiArfcn() coherence for the follow-up 0x0F/0x46
        uint8_t pl[ROM6_RCVMAX];
        uint8_t len = rom6_b_rssi8b(pl, arfcn);
        rom6_enqueue(m, 0x8B, pl, len);
        if (log) fprintf(stderr,
                         "[rom6] 0x55 blind band search band=%u tries=%u -> 0x8B ALL_RSSI ARFCN=%u @step=%llu\n",
                         plen > 0 ? buf[0] : 0u, plen > 1 ? buf[1] : 0u, arfcn,
                         (unsigned long long)m->dsp_steps);
        break;
    }
    case 0x57:                                      // serving -> SCH
        rom6_enq_sch(m);
        break;
    case 0x1B: {                                    // SEND_BLOCK (uplink LAPDm) -> L3 — STAGE 2
        // payload[1]=logch, payload[2..]=23-byte uplink LAPDm block.
        uint8_t logch = (plen > 1) ? buf[1] : 0x80u;
        const uint8_t* L2 = (plen > 2) ? buf + 2 : buf;
        unsigned l2len = (plen > 2) ? plen - 2 : 0;
        if (log) fprintf(stderr, "[rom6] 0x1B SEND_BLOCK logch=0x%02X l2len=%u @step=%llu\n",
                         logch, l2len, (unsigned long long)m->dsp_steps);
        rom6_lapdm_uplink(m, logch, L2, l2len);
        break;
    }
    default:
        break;
    }
}

// ===========================================================================================
// Owned DSP-region READ (verbatim behaviour from the retired rom6_new scaffold): the boot mailbox handshake.
// ===========================================================================================
static int rom6_read(struct Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    (void)size;
    if (addr == m->fw.dsp_mbox0) { *out = m->dsp_ack[0]; m->dsp_ack[0] = 0; return 1; }
    if (addr == m->fw.dsp_mbox1) {                       // doubles as MDIRCV queue word0 base
        uint32_t hp = m->fw.mdircv_head & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        if (head == 0) { *out = m->dsp_ack[1]; m->dsp_ack[1] = 0; return 1; }
        *out = ram_value; return 1;                      // queue initialised: real word0
    }
    if (addr == 0x00010002u && (ram_value & 0xFFFF) == 0xFFFF) { *out = 0; return 1; }
    if (m->fw.dsp_boot_status && addr == m->fw.dsp_boot_status &&
        (ram_value & 0xFFFF) == 0xFFFF) { *out = m->fw.dsp_boot_ready; return 1; }
    if (m->fw.dsp_boot_status2 && addr == m->fw.dsp_boot_status2 &&
        (ram_value & 0xFFFF) == 0xFFFF) { *out = m->fw.dsp_boot_ready; return 1; }
    return 0;
}

// ===========================================================================================
// Owned DSP-region WRITE: mailbox ack + code-block reply pump (verbatim from the retired scaffold), plus the
// MDISND (m2d) observer that de-wraps the record and calls HandleMdiPacket / the SIML capture.
// ===========================================================================================
static void rom6_on_host_interrupt(struct Mad2* m) {
    if (!m->mem || !m->dsp_running) return;

    uint32_t cobba = m->fw.cobba & m->mem_mask;
    if (m->mem[cobba] || m->mem[cobba + 1]) {
        m->mem[cobba] = 0;
        m->mem[cobba + 1] = 0;
    }

    // Short MDI is part of the host-interrupt transaction, not a polled timer.
    uint32_t sc = 0x000100DCu & m->mem_mask;
    if (m->mem[sc] || m->mem[sc + 1]) {
        uint8_t param = m->mem[sc], type = m->mem[sc + 1];
        Rom6Dsp* r = &m->rom6;
        uint16_t arfcn = r->measurementArfcn ? r->measurementArfcn : r->servingArfcn;
        m->mem[sc] = 0;
        m->mem[sc + 1] = 0;
        if ((type == 0x45 && param != 0) || type == 0x4B) {
            uint8_t pl[168];
            uint8_t len = rom6_b_rssi83(pl, arfcn, ROM6_DEFAULT_RSSI);
            rom6_enqueue(m, 0x83, pl, len);
            if (type == 0x4B) {
                len = rom6_b_rssi8b(pl, arfcn);
                rom6_enqueue(m, 0x8B, pl, len);
            }
            // 0x84 with no random-access burst to report: an all-zero RA_INFO, exactly as the
            // ROM-4 engine emits outside its RANDOM_ACCESS phase. It MUST still be a full
            // 8-octet report — the firmware's RA_INFO handler (0x2DB904, see rom6_b_ra_info)
            // unconditionally reads payload[0..3], and the MCU allocates its broker buffer from
            // the record's own length byte, so a short record makes it read past that buffer.
            // RA 0 / frame 0 simply never matches a real Immediate Assignment.
            len = rom6_b_ra_info(pl, 0x00, 0u);
            rom6_enqueue(m, 0x84, pl, len);
        }
    }
    rom6_pump_mdircv(m);
}

static int rom6_write(struct Mad2* m, uint32_t addr, int size, uint32_t value) {
    if (addr == 0x00020002u && (value & 0xFFu) == 0x01u)
        m->dsp_running = 0;                              // MCU put DSP in reset: warm-reboot re-arm
    if (addr == m->fw.dsp_mbox0) { m->dsp_ack[1] = (uint16_t)value ? (uint16_t)value : 1; m->dsp_acks++; return 1; }
    if (addr == m->fw.dsp_mbox1) {
        uint32_t hp = m->fw.mdircv_head & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        if (head == 0) { m->dsp_ack[0] = (uint16_t)value ? (uint16_t)value : 1; m->dsp_acks++; return 1; }
        return 0;
    }
    // DSPIF bit2 is the host->DSP interrupt event. All host-side command
    // consumption is tied to this edge; no timer loop polls command registers.
    if ((addr == 0x00030000u || addr == 0x00030001u) && size >= 1) {
        uint16_t v = size >= 2 ? (uint16_t)value
                   : addr == 0x00030001u ? (uint16_t)(value & 0xFFu)
                   : (uint16_t)((value & 0xFFu) << 8);
        if (v & 0x0004u) rom6_on_host_interrupt(m);
        return 0;
    }
    // The MCU has consumed an MDIRCV record. Commit the new consumer pointer
    // before pumping, exactly like Noks' OnSharedWrite(0x1CA).
    if (m->fw.mdircv_head && m->mem && addr == m->fw.mdircv_head && size >= 2) {
        uint32_t hp = addr & m->mem_mask;
        m->mem[hp] = (uint8_t)(value >> 8);
        m->mem[(hp + 1u) & m->mem_mask] = (uint8_t)value;
        rom6_pump_mdircv(m);
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
            // Copy the payload out of the ring into a flat buffer (handles the ring wrap).
            uint8_t  buf[192];
            // MDISND word0 is {payload-length, opcode}; the length does NOT include the opcode.
            // Treating it as a whole-record length truncated every request by one byte (notably
            // SEND_BLOCK's final LAPDm octet) and diverged from both silicon traces and Dsp.cs.
            unsigned plen = rlen;
            if (plen > sizeof buf) plen = sizeof buf;
            for (unsigned k = 0; k < plen; ++k)
                buf[k] = m->mem[(b + ((off0 + 2u + k) % ring)) & m->mem_mask];

            Rom6Dsp* r = &m->rom6;
            if (op == 0x70) {                            // local-security handshake (SIML) — CAPTURE
                uint8_t sub = plen ? buf[0] : 0u;
                if (getenv("ROM6_LOG"))
                    fprintf(stderr, "[rom6] SIML m2d {%02X 70 %02X} plen=%u body=%02X %02X %02X %02X @step=%llu\n",
                            rlen, sub, plen, plen>1?buf[1]:0, plen>2?buf[2]:0, plen>3?buf[3]:0, plen>4?buf[4]:0,
                            (unsigned long long)m->dsp_steps);
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
                }
            } else {                                     // network / L1 opcode -> reference dispatch
                rom6_handle_mdi(m, op, buf, plen);
            }
            if (getenv("ROM6_M2DLOG")) {
                fprintf(stderr, "[rom6 m2d] op=0x%02X len=%u @step=%llu data=",
                        op, rlen, (unsigned long long)m->dsp_steps);
                for (unsigned _i = 0; _i < plen && _i < 16; ++_i) fprintf(stderr, "%02X", buf[_i]);
                fprintf(stderr, "\n");
            }
        }
        m->dsp_mdisnd_prev = (uint16_t)(value & 0xFFFFu);
        // ProcessMdiSnd owns both queue indices: commit producer and advance the
        // consumer to it after the complete record has been dispatched.
        {
            uint32_t tp = m->fw.mdisnd_tail & m->mem_mask;
            m->mem[tp] = (uint8_t)(value >> 8);
            m->mem[(tp + 1u) & m->mem_mask] = (uint8_t)value;
            m->mem[(tp + 2u) & m->mem_mask] = (uint8_t)(value >> 8);
            m->mem[(tp + 3u) & m->mem_mask] = (uint8_t)value;
        }
        rom6_pump_mdircv(m);
        return 1;
    }
    if (addr == m->fw.dsp_cb_reply) {
        uint16_t v = (uint16_t)(value & 0xFFFFu);
        if (v && m->rom6.phase < ROM6_UPLOAD) m->rom6.phase = ROM6_UPLOAD;
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
// AdvanceTo(cycles) — asynchronous RF/GSM event dispatcher.
// ===========================================================================================
static void rom6_sync_cycle(struct Mad2* m, uint64_t cycles) {
    m->rom6.currentCycles = cycles;
}

static uint64_t rom6_next_wake(struct Mad2* m) {
    Rom6Dsp* r = &m->rom6;
    uint64_t next = UINT64_MAX;
    if (m->dsp_cb_deadline_cyc && m->dsp_cb_deadline_cyc < next)
        next = m->dsp_cb_deadline_cyc;
    if (m->dsp_hb_next_cyc && m->dsp_hb_next_cyc < next)
        next = m->dsp_hb_next_cyc;   // idle telemetry heartbeat (deep-idle must wake for it)
    for (unsigned i = 0; i < ROM6_DELAYED_N; ++i)
        if (r->delayed[i].used && r->delayed[i].due < next) next = r->delayed[i].due;
    if (r->p_head != r->p_tail) {
        uint64_t expiry = r->pending[r->p_head].enq + rom6_cpf() * 217u * 2u;
        if (expiry < next) next = expiry;
    }
    if (r->servingArfcn && r->nextBcchBroadcastCycle < next)
        next = r->nextBcchBroadcastCycle;
    if (r->nextRachTxCycle < next)
        next = r->nextRachTxCycle;
    if (r->incomingPagingActive && !r->incomingPagingAnswered &&
        r->nextIncomingPagingCycle < next)
        next = r->nextIncomingPagingCycle;
    if (r->dedicatedConfigured) {
        if (r->nextDedicatedCycle < next) next = r->nextDedicatedCycle;
        if (r->nextDedicatedBlkReqCycle < next) next = r->nextDedicatedBlkReqCycle;
    }
    return next;
}

static void rom6_advance_to(struct Mad2* m, uint64_t cycles) {
    Rom6Dsp* r = &m->rom6;
    r->currentCycles = cycles;

    if (cycles >= r->nextRachTxCycle) {
        uint64_t eventCycles = r->nextRachTxCycle;
        uint8_t ra = r->rachReference;
        uint32_t reqfn = r->rachRequestFn;
        r->nextRachTxCycle = UINT64_MAX;
        rom6_enq_imm_assign_at(m, ra, reqfn, eventCycles);
        if (getenv("ROM6_LOG"))
            fprintf(stderr, "[rom6] RACH transmitted RA=0x%02X reqFN=%u @cycle=%llu\n",
                    ra, reqfn, (unsigned long long)eventCycles);
    }

    rom6_expire_stale(m, cycles);
    rom6_pump_delayed(m, cycles);

    if (r->incomingPagingActive && !r->incomingPagingAnswered &&
        !r->dedicatedConfigured && cycles >= r->nextIncomingPagingCycle) {
        uint64_t eventCycles = r->nextIncomingPagingCycle;
        rom6_enq_paging_at(m, r->pagingOffset, 1, eventCycles);
        r->incomingPagingBursts++;
        if (getenv("ROM6_LOG"))
            fprintf(stderr,
                    "[rom6] incoming paging block FN=%u T3=%u burst=%u\n",
                    rom6_frame_at(eventCycles),
                    rom6_frame_at(eventCycles) % 51u,
                    r->incomingPagingBursts);
        if (r->incomingPagingBursts >= 8u) {
            r->incomingPagingActive = 0;
            r->nextIncomingPagingCycle = UINT64_MAX;
        } else {
            r->nextIncomingPagingCycle += 2u * 51u * rom6_cpf();
        }
    }

    // Dedicated LAPDm and block requests are independent scheduled events. Downlink
    // frames are kept raw until this point, so their GSM FN comes from the slot that
    // actually emitted them, never from the MCU write that happened to create them.
    if (r->dedicatedConfigured) {
        uint64_t period = 51u * rom6_cpf();
        if (cycles >= r->nextDedicatedCycle) {
            uint64_t eventCycles = r->nextDedicatedCycle;
            if (r->dedicatedHead != r->dedicatedTail) {
                if (getenv("ROM6_LOG")) {
                    const uint8_t* frame = r->dedicatedFrames[r->dedicatedHead];
                    fprintf(stderr,
                            "[rom6] dedicated downlink addr=%02X ctrl=%02X li=%02X @cycle=%llu data=",
                            frame[0], frame[1], frame[2],
                            (unsigned long long)eventCycles);
                    for (unsigned i = 0; i < 23; ++i)
                        fprintf(stderr, "%02X", frame[i]);
                    fputc('\n', stderr);
                }
                rom6_dl_lapdm_at(m, r->dedicatedFrames[r->dedicatedHead], eventCycles);
                r->dedicatedHead = (uint8_t)((r->dedicatedHead + 1u) % ROM6_DEDICATED_N);
            } else {
                static const uint8_t fill[23] = {
                    0x03, 0x03, 0x01,
                    0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,
                    0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B,0x2B };
                rom6_dl_lapdm_at(m, fill, eventCycles);
            }
            r->nextDedicatedCycle += period;
            if (r->dedicatedReleasePending && r->dedicatedHead == r->dedicatedTail) {
                r->dedicatedConfigured = 0;
                r->dedicatedReleasePending = 0;
                r->nextDedicatedCycle = UINT64_MAX;
                r->nextDedicatedBlkReqCycle = UINT64_MAX;
                // A host service may have arrived while registration still owned
                // the SDCCH.  Paging cannot start until the release is physically
                // complete, so arm it from this transition instead of relying on
                // a later per-instruction poll.
                rom6_start_next_incoming_paging(m);
            }
        }
        if (r->dedicatedConfigured && cycles >= r->nextDedicatedBlkReqCycle) {
            uint8_t blkreq[1] = { 0x80 };
            rom6_enqueue(m, 0x86, blkreq, 1);
            r->nextDedicatedBlkReqCycle += period;
        }
    }

    // Periodic BCCH broadcast: one SI (rotated by TC) every Cps/4, plus paging at every CCCH
    // block offset of the current multiframe once CCCH is configured (pre-reg = IMSI paging).
    if (r->servingArfcn != 0 && cycles >= r->nextBcchBroadcastCycle) {
        uint64_t eventCycles = r->nextBcchBroadcastCycle;
        uint32_t fn = rom6_frame_at(eventCycles);
        uint32_t tc = (fn / 51u) % 8u;
        // bitplane BCCH-Norm schedule SI_BY_TC = {SI1,SI2,SI3,SI4,SI2,SI2,SI3,SI4} — SI1 at TC0.
        static const uint8_t* const SI_TBL[8] = {
            ROM6_SI1, ROM6_SI2, ROM6_SI3, ROM6_SI4,
            ROM6_SI2, ROM6_SI2, ROM6_SI3, ROM6_SI4 };
        static const uint8_t SI_LEN[8] = {
            sizeof ROM6_SI1, sizeof ROM6_SI2, sizeof ROM6_SI3, sizeof ROM6_SI4,
            sizeof ROM6_SI2, sizeof ROM6_SI2, sizeof ROM6_SI3, sizeof ROM6_SI4 };
        rom6_enq_si_at(m, SI_TBL[tc], SI_LEN[tc], tc, eventCycles);
        if (r->ccchConfigured && !r->dedicatedConfigured) {
            // A broadcast cadence is not itself an incoming-service event. Emit
            // no-identity paging fill here; the incoming call/SMS queue owns IMSI
            // paging and will schedule it only when a real host request exists.
            int imsi = 0;
            for (int i = 0; i < 9; ++i)
                rom6_enq_paging_at(m, ROM6_CCCH_OFFSETS[i], imsi, eventCycles);
        }
        r->nextBcchBroadcastCycle += (rom6_cpf() * 217u) / 4u;
    }
    rom6_pump_mdircv(m);
}

// ===========================================================================================
// Per-step DSP pump is limited to boot/security work. Runtime RF/GSM work lives
// exclusively in the asynchronous scheduler above.
// ===========================================================================================
static void rom6_tick(struct Mad2* m) {
    m->dsp_steps++;
    if (m->dsp_hle_quiet) return;

    // Idle telemetry heartbeat (never-silent DSP; see the essay at rom6_idle_heartbeat).
    if (m->mem) rom6_idle_heartbeat(m);

    // Boot-loader transport is intentionally separate from the RF/GSM scheduler:
    // this is the legacy HPI codeblock handshake, not radio or frame timing.
    if (m->dsp_cb_deadline_cyc && m->rtc_mono >= m->dsp_cb_deadline_cyc && m->mem) {
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

    // SIML local-security + self-test responder — VERBATIM byte behaviour from the retired scaffold. The
    // engine supplies the DECODED "unlocked" record; it does not decrypt (C54x is out of scope, the
    // MCU is a verbatim courier). Priority 0x34 > 0x35 > 0x36 > self-test-0x0D; one reply enqueued
    // per tick (the single-packet egress delivers them one per empty ring window, same cadence).
    if (m->mem) {
        Rom6Dsp* r = &m->rom6;
        int want_st = m->dsp_st_req && !m->dsp_selftest_replied && !m->dsp_selftest_off;
        if (r->siml_want || want_st) {
            uint8_t pl[64] = {0}; uint8_t len = 0;
            if (r->siml_want & 1u) {                          // 0x34 MSID reply (msg[11..23])
                pl[0] = 0x34;
                for (int i = 0; i < 13; ++i) pl[3 + i] = r->siml_msid[i];
                len = 16; r->siml_want &= (uint8_t)~1u;
            } else if (r->siml_want & 2u) {                   // 0x35 decoded record + ciphertext echo
                // Region A = the DECODED unlocked record (msg[12..35]); Region B (msg[36..59]) = a
                // verbatim echo of the 0x16 ciphertext the firmware integrity-checks against EEPROM.
                // (Derivation of region_a's classifier/gate bytes: docs/sim-dsp-groundup/ SIML notes.)
                static const uint8_t region_a[24] = {
                    0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0x00,0x00, 0x00,0x98,0x00,0x00,
                    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x08,0x7C,0x00,0x00 };
                pl[0] = 0x35; pl[1] = 0x32; pl[2] = 0x01; pl[3] = 0x00;
                for (int i = 0; i < 24; ++i) pl[4  + i] = region_a[i];
                // Region B = the SIM-lock CIPHERTEXT the firmware integrity-checks against the PMM
                // (v4.18+: 0x280CAC memcmp region_b vs EEPROM param). The real C54x ROM6 derives it by
                // decrypt/re-encrypt of the {70 16} block; our HLE cannot compute that, so we RETRIEVE
                // the real ciphertext from the RAM-backed PMM flash. VERSION-AGNOSTIC + no hardcoded
                // address: the {70 16} block we captured is `[len byte][ciphertext data...]`; the full
                // 24-byte ciphertext is that data in flash (the MDI record is ~1 byte short of it). We
                // locate the data signature (block bytes after the leading length) in the upper flash
                // (PMM/parameter/NVRAM area) and read the 24-byte ciphertext there. Default = verbatim
                // echo (older builds like v5.79 have no crypto check, and the search simply no-ops).
                for (int i = 0; i < 24; ++i) pl[28 + i] = r->siml_block[i];
                // The captured block IS the ciphertext prefix (siml_block[0]=ciphertext[0]); the MDI
                // record is ~1 byte short of the full 24, so we locate the block's 16-byte signature in
                // the upper flash (PMM/parameter/NVRAM) and read the complete 24-byte ciphertext there.
                int crypto_present = 0;   // PMM ciphertext exists -> this build gates on the crypto check
                if (m->mem) {
                    const uint8_t* sig = &r->siml_block[0];
                    // Search the model's upper flash (PMM/parameter/EEPROM), not a fixed 2 MB
                    // window: on 4 MB images (3330 etc.) the flash maps to ARM [flash_base,
                    // flash_base+flash_size) so the PMM/EEPROM ciphertext lands ~0x5E0046, well
                    // past the old hardcoded 0x400000 ceiling. Scan from just above the code up
                    // to the end of this model's flash so the block is found on every geometry.
                    uint32_t hi = (m->model ? m->model->mem.flash_base + m->model->mem.flash_size
                                            : 0x400000u);
                    for (uint32_t a = 0x300000u; a + 24u <= hi; ++a) {
                        int ok = 1;
                        for (int k = 0; k < 16; ++k)
                            if (m->mem[(a + (uint32_t)k) & m->mem_mask] != sig[k]) { ok = 0; break; }
                        if (ok) {
                            for (int i = 0; i < 24; ++i) pl[28 + i] = m->mem[(a + (uint32_t)i) & m->mem_mask];
                            crypto_present = 1;
                            if (getenv("ROM6_LOG"))
                                fprintf(stderr, "[rom6] SIML ciphertext located in PMM @0x%X\n", a);
                            break;
                        }
                    }
                }
                // The MCU already SENDS the SIM-lock ciphertext in the {70,16} message, so the
                // captured block IS the ciphertext (Region B above already echoes it verbatim) —
                // a non-blank captured block means this is a crypto build, no flash copy required.
                // The finder above only matters for builds whose message capture came up short;
                // when it misses but the block is real (e.g. the 7110 decodes the record to RAM,
                // so no flash copy exists to find), gate on the captured block itself.
                if (!crypto_present)
                    for (int k = 0; k < 24; ++k)
                        if (r->siml_block[k] != 0xFFu) { crypto_present = 1; break; }
                // Route the decode-commit for CRYPTO builds (v4.18 NHM-5 and kin, where the security
                // block is populated in PMM) into the firmware's own live-table write.
                //
                // The v4.18 0x35 handler (0x2809E8) has two commit paths that memcpy region_a into the
                // live SIM-lock table 0x117B10: the streaming write (0x280A90, gated [0x117AEB]==1, never
                // armed for a first record) and the else-write (0x280D2C, gated r5==1). A per-build
                // security flag ([0x11FE59] bit7, set on v4.18) makes the field-classifier at 0x280AF6
                // fall through to 0x280B92 which sets r7=1; with r7==1 the handler takes the crypto branch
                // (0x280C8A: integrity-memcmp region_b vs the PMM ciphertext, then 0x27F7B6 which only
                // STAGES region_a to 0x117AF0 and emits {70 17}) and clears r5 at 0x280CA6 -> the live
                // table is never written and the evaluator (0x27FDF0, reads 0x117B10) keeps rejecting.
                //
                // The classifier routes on region_a[9]: 0x98 (>=0x80) reaches 0x280B92 which sets r7=1
                // (crypto branch); a value in [0x78,0x80) instead reaches 0x280B42 where region_a[20] bit3
                // (0x08, set) makes 0x280B48 branch to 0x280BCC WITHOUT setting r7. With r7==0 and r5
                // still 1 (region_a[10..11]/[22..23] match the state-2 comparand 0), 0x280C86 falls to
                // 0x280D08 -> 0x280D0A -> 0x280D26 (r5==1) -> 0x280D2C: the firmware's OWN code memcpy's
                // region_a into the live table 0x117B10. region_a[9] is not read by the evaluator
                // (0x27FDF0 checks record bytes [8]/[17]/[21]), so the committed unlocked record is
                // functionally identical to v5.79's (wildcard compare + cleared enable/status).
                //
                // The non-v4.18 reference builds (v5.79 3310, 8850, 8210) ALSO populate a PMM security
                // block, so crypto_present is set for them too and they likewise receive region_a[9]=0x78.
                // That is benign: their per-build security flag ([0x11FE59]-analog bit7) is CLEAR, so
                // their 0x35 handler never enters the crypto-routing classifier at all and byte[9] does
                // not steer their (already-working) direct commit; the evaluator ignores byte[9] either
                // way. Verified: v5.79 still reaches Set-time, 8850/8210 still reach Security-code.
                if (crypto_present) pl[4 + 9] = 0x78;
                len = 52; r->siml_blkidx++; r->siml_want &= (uint8_t)~2u;
            } else if (r->siml_want & 4u) {                   // 0x36 terminal verdict, pass=0 -> unlocked
                pl[0] = 0x36; len = 4; r->siml_want &= (uint8_t)~4u;
            } else {                                          // self-test verdict {0x0D,0} pass
                pl[0] = 0x0D; pl[1] = 0x00; len = 2;
                m->dsp_selftest_replied = 1; m->dsp_st_req = 0;
            }
            if (getenv("ROM6_LOG"))
                fprintf(stderr, "[rom6] SIML d2m 0x74 sub=0x%02X len=%u -> %02X %02X %02X %02X %02X %02X @step=%llu\n",
                        pl[0], len, len>0?pl[0]:0, len>1?pl[1]:0, len>2?pl[2]:0, len>3?pl[3]:0, len>4?pl[4]:0, len>5?pl[5]:0,
                        (unsigned long long)m->dsp_steps);
            rom6_enqueue(m, 0x74, pl, len);
            rom6_pump_mdircv(m);
        }
    }
}


const DspOps mad2_dsp_rom6 = {
    .name     = "rom6",
    .read     = rom6_read,
    .write    = rom6_write,
    .tick     = rom6_tick,
    .sync_cycle = rom6_sync_cycle,
    .next_wake = rom6_next_wake,
    .advance_to = rom6_advance_to,
    .hle_tone = dsp_hle_tone,   // shared HLE COBBA tone reader (dsp/dsp_tone.c)
};
