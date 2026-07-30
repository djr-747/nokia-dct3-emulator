// GSM cell-search bridge — 3310 v5.79 (§9 Phase A of).
//
// PURPOSE. The RR cell-selection FSM (0x22E650, state var [0x111742]) issues a carrier
// search then blocks forever waiting for the DSP's radio result. Our HLE DSP has no RF,
// so nothing answers and a real (non-test) SIM ends at "SIM card not accepted". This
// module synthesizes the DSP's radio responses (MDI 0x80 RECEIVED_BLOCK records) and
// pushes them into the MDIRCV ring + FIQ0 exactly as silicon's DSP would, so the
// firmware's OWN receive path drives the FSM forward. It NEVER pokes FSM state, the
// SIM-status word, the verdict, or any registration cell — faithful message injection only.
//
// OPT-IN. Entirely gated by env GSMBRIDGE=1 (default OFF). When off, mdi_gsm_tick() returns
// immediately, so every guarded boot stays byte-identical. GSMLOG=1 traces each event.
//
// ─────────────────────────────────────────────────────────────────────────────────────
// CORRECTED MECHANISM (this session — supersedes the handoff's "0x56 via MDISND 0x2BABA4"):
//
//  * The 0x56 carrier search is NOT an MDISND ring record. Builder 0x2A70AC mallocs a
//    164-byte block {hw[0]=0x0002, [2]=0xA0(len 160), [3]=0x56, [4..163]=0xFF x160
//    carrier-list template} and 0x2A70DA dispatches it via TASK_SEND_TYPED(task 3, block)
//    — an internal RTOS message to the L1 task, invisible to the MDISND-tail hook
//    (the rom4 body watches [0x100A4], which froze at 0x33 after the boot SIML burst). So
//    "observe the runtime GSM MDI" means watching the FSM/L1 RAM side-effects, not the ring.
//
//  * The FSM waits (TASK_RECV_BLOCKING at 0x232EEE) for a TASK message whose halfword[0]
//    == 0x1802 and byte[3] ∈ {0x80,0x87,0x89,0x8B}. That message is produced by the L1
//    receive path from a DSP 0x80 RECEIVED_BLOCK — so answering the DSP side is faithful.
//
//  * DSP→MCU delivery (the mechanism we drive here): write records into the MDIRCV ring
//    (base .mdircv_q=0x10100, head [0x101CA], tail [0x101C8]; word idx 0x80..0xE3) and
//    raise FIQ0. TASK_4_MDI_RECEIVER (0x2EDB04) → BROKER_DECODE (0x2BACEC) → group jumptable
//    (byte[3]==0x80 → 0x2EDBD4 → L1 block processor 0x2C3BEC). A record is:
//        word0   = { len(HIGH byte) , type(LOW byte) }      -> heap block[2]=len, block[3]=type
//        word1.. = block body                               -> heap block[4..], copied by
//                                                              BROKER_DECODE's word-copy loop
//                                                              (0x2BAD42) from the ring words
//                                                              FOLLOWING word0.
//    The 0x80 block body (block[4..], per jmacato (github.com/jmacato) §9.1) = [logch, BSIC, status, fn[3],
//    ARFCN[2], status[2], Layer2...]. Logical channels: SCH 0x40, BCCH 0x50, CCCH 0x60,
//    SDCCH 0x80. The L1 processor gate is [0x110FE0]==1 (SCH-search armed; true at ~11.85M).
//
//  * The RR FSM runs in TASK 10. The L1 SCH branch (0x2C3C82) forwards the heap block via
//    TASK_SEND_TYPED(task 10, block) — and because the dequeue stamps block[0..1]={0x18,0x02},
//    that block's halfword[0] IS 0x1802, block[3]=type. So the FSM's recv (0x232EEE) reads it
//    directly. Post-0x56 the FSM (state at 0x232F12) dispatches on the sub/type byte block[3]:
//    0x80 SCH → checks block[4]==0x40 & gate [0x110ABC]!=0 → SCH accept handler 0x2320AC;
//    0x87/0x89 → 0x2300C; 0x8A → SI path.
//
// VERIFIED THIS SESSION (GSMBRIDGE=1, real IMSI 505/01): the FSM blocks on its recv at step
// 11,852,037; the injected SCH block is received (type 0x1802 matched at 0x232F12), dispatched
// to the SCH accept handler 0x2320AC (step 11,901,349, gate [0x110ABC]=1), processed, and the
// FSM RE-BLOCKS at step 11,902,015 awaiting the next block — i.e. the permanent stall is broken
// and the cell-selection sequence now progresses. Per §9.2 the DSP streams SI2/SI3/SI4 (0x80
// blocks) after one 0x56, so the next records to deliver are BCCH (logch 0x50) System
// Information carrying the LAI that matches the SIM's PLMN.
//
// GROWTH POINT (Phase 1, not yet built): after SCH, stream SI3/SI2/SI4 (BCCH 0x50 blocks with
// the real LAI) → 0x02 CHANNEL_CONFIGURE / 0x89 confirm → 0x0C RACH + Immediate Assignment →
// SDCCH (0x80) → LAPDm SABM LOCATION UPDATING REQUEST → UA + LOCATION UPDATING ACCEPT →
// registered idle, SIM-status word [0x10A5DE]=0x0FFC. Needs a small LAPDm engine (23-byte
// frames). Each stage = another mdircv_push() keyed off the FSM re-blocking on its recv.

#include "mad2/mad2.h"
#include <stdlib.h>   // getenv
#include <stdio.h>    // printf (GSMLOG)
#include <string.h>

// ── RAM anchors (3310 v5.79; runtime = fls + 0x200000) ───────────────────────────────
#define L1_SCH_ARMED   0x00110FE0u   // ==1 while L1 SCH search is armed (0x2C3BF4). MCU-private:
                                     // used ONLY as an opt-in safety interlock, not a DSP trigger.
#define NETMON_CSEARCH 0x0011FDB8u   // carrier-search counter, ++'d right after each 0x56 emit
#define FSM_STATE      0x00111742u   // cell-sel FSM (0x22E650) state var

// ── MDI types / logical channels (jmacato (github.com/jmacato) §9.1) ──────────────────────────────────────
#define MDI_RECEIVED_BLOCK 0x80u
#define MDI_RSSI_RESULTS   0x83u   // per-carrier RSSI (handler 0x2C3690 -> RR task 10)
#define MDI_CHANNEL_CHANGED_CNF 0x89u // channel-change confirm (handler 0x2C3D4C -> FSM task 10)
#define MDI_MEAS_RESULT    0x8Au   // serving-cell measurement report (FSM state-20 -> 0x23212A -> MM task 13)
#define MDI_ALL_RSSI       0x8Bu   // full-scan RSSI array (handler 0x2C3FC4)
#define LOGCH_SCH   0x40u
#define LOGCH_BCCH  0x50u
#define LOGCH_CCCH  0x60u
#define LOGCH_SDCCH 0x80u
#define RSSI_STRONG 0xD0u          // jmacato (github.com/jmacato) §9.3 traced strong/default measurement byte
#define RSSI_NONE   0x80u          // jmacato (github.com/jmacato) §9.3 no-signal placeholder (ALL_RSSI_RESULTS)
#define SERVING_ARFCN 586u         // 0x024A — the candidate cell staged at 0x117834

// ── module state (file-static; the 3310 has one DSP instance, mirrors dsp_7110.c) ─────
static int      g_en   = -1;   // GSMBRIDGE cached (-1 = unread, 0/1 = off/on)
static int      g_log  = 0;    // GSMLOG cached
static int      g_noguard = 0; // GSMBRIDGE_NOGUARD: drop the MCU-side safety interlock
static int      g_stage = 0;   // 0 = await carrier search, 1 = SCH delivered / SI broadcasting
static uint64_t g_si_next = 0; // explicit deadline for the next SI block
static int      g_si_idx  = 0; // SI broadcast cursor (0=SI3,1=SI2,2=SI4, cycling)
static uint64_t g_si_cad  = 0; // GSMBRIDGE_SI_CAD: mono cycles between SI blocks (0 = unread)
static int      g_rssi    = -1;// GSMBRIDGE_RSSI cached (answer the 0x56 power scan)
static int      g_rssi_done = 0; // one-shot: RSSI results delivered before the SCH
static uint8_t  g_rssi_type = 0; // GSMBRIDGE_RSSI_TYPE: 0x83 RSSI_RESULTS (default) or 0x8B ALL
static int      g_si_stov = -1;// GSMBRIDGE_SI_STATUS override (-1 = auto: first 3 -> status 1)
static int      g_si     = 1;  // SI broadcast sub-feature (default ON under GSMBRIDGE; =0 opts out)
static int      g_cnf     = 1;  // channel-change-confirm (0x89) sub-feature (GSMBRIDGE_CNF=0 opts out)
static int      g_cnf_done = 0; // one-shot: 0x89 CHANNEL_CHANGED_CNF delivered
static int      g_meas    = 0;  // continuous 0x8A measurement stream (GSMBRIDGE_MEAS=1, opt-in test)
static uint64_t g_meas_next = 0;// explicit deadline for the next 0x8A measurement
static uint8_t  g_plmn[3] = {0x00,0xF1,0x10}; // refreshed from the active EF_IMSI/EF_AD

// Sub-feature gate for the ONE master knob GSMBRIDGE: each faithful piece defaults ON when the
// master is set; opt one out for A/B with <NAME>=0 (mirrors the 5110 DSP54_COSIM sub-knob pattern).
static int gsm_sub_on(const char* name) { const char* v = getenv(name); return v ? (atoi(v) != 0) : 1; }

// Push one DSP->MCU MDIRCV record { len(high), type(low) } + body words into the ring, at
// the current tail (requires an EMPTY ring, head==tail, so real MDI is never trampled), then
// raise FIQ0. Body bytes become heap block[4..] via BROKER_DECODE's word-copy. Returns 1 on
// delivery, 0 if the ring is not initialised/drained (caller retries next tick).
static int mdircv_push(Mad2* m, uint8_t type, const uint8_t* body, int bodylen) {
    uint32_t hp = m->fw.mdircv_head & m->mem_mask, tp = m->fw.mdircv_tail & m->mem_mask;
    uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
    uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
    if (head < 0x80 || tail < 0x80 || head != tail) return 0;   // not init, or pending data
    int words = 1 + (bodylen + 1) / 2;                          // word0 + ceil(body/2)
    uint16_t pos = tail;
    if ((uint16_t)(pos - 0x80 + words) > 100) pos = 0x80;       // wrap before ring end (idx 0xE4)
    uint32_t q = (m->fw.mdircv_q & m->mem_mask) + (uint32_t)(pos - 0x80) * 2;
    m->mem[q]     = (uint8_t)bodylen;   // word0 HIGH = length -> heap block[2]
    m->mem[q + 1] = type;               // word0 LOW  = MDI type -> heap block[3]
    for (int i = 0; i < bodylen; ++i) m->mem[(q + 2 + i) & m->mem_mask] = body[i];
    if (bodylen & 1) m->mem[(q + 2 + bodylen) & m->mem_mask] = 0;   // pad odd body to word
    uint16_t nt = (uint16_t)(pos + words);
    m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;  // advance tail only
    mad2_raise_fiq(m, 0);
    return 1;
}

// GSM frame clock: FN advances 1 per TDMA frame (~4.615 ms = ~60000 ARM cycles @ 13 MHz),
// wrapping at the hyperframe (26*51*2048 = 2715648). Derive it from rtc_mono so downlink
// blocks carry a coherent, monotonically-advancing FN exactly as silicon's DSP would stamp.
#define GSM_CYCLES_PER_FRAME  60000ull
#define GSM_FN_MAX            2715648ull
static uint32_t gsm_frame_number(Mad2* m) {
    return (uint32_t)((m->rtc_mono / GSM_CYCLES_PER_FRAME) % GSM_FN_MAX);
}

// Build + push an SCH-sync RECEIVED_BLOCK. Body = block[4..] per §9.1: the L1 processor
// (0x2C3BEC) reads logch(block[4]), status(block[6]), frame number(block[7..9]). SCH sync
// carries BSIC + reduced frame number; a minimal plausible cell.
static int push_sch(Mad2* m) {
    uint8_t body[16] = {0};
    body[0] = LOGCH_SCH;   // block[4]  logical channel = SCH
    body[1] = 0x00;        // block[5]  BSIC
    body[2] = 0x00;        // block[6]  status (0x2C3C0C: !=1 -> run 0x2C36AC)
    body[3] = 0x00;        // block[7]  frame number hi
    body[4] = 0x00;        // block[8]  frame number mid
    body[5] = 0x00;        // block[9]  frame number lo
    body[6] = 0x02;        // block[10] ARFCN hi \  = 0x024A (586): MUST match the candidate cell
    body[7] = 0x4A;        // block[11] ARFCN lo /  the firmware staged at 0x117834 (record[6]),
                           //                       else the ARFCN lookup 0x28832E returns NULL.
    body[8] = 0x00;        // block[12] status hi
    body[9] = 0x00;        // block[13] status lo
    return mdircv_push(m, MDI_RECEIVED_BLOCK, body, 10);
}

// ── RSSI scan results (§9.2 step 1 answer / §9.3; mirrors Noks EnqueueRssiResults / SetDspRssi) ──
// The 0x56 carrier search is a power scan; the DSP replies with RSSI results the firmware radio task
// uses to rank carriers, set the serving cell's measured level, and pass the C1 suitability test so
// MM can camp. Two MDI carriers (jmacato (github.com/jmacato) §9.1): 0x83 RSSI_RESULTS (per serving carrier; handler
// 0x2C3690 forwards it to RR task 10 when SCH is armed) and 0x8B ALL_RSSI_RESULTS (the full scan
// array; handler 0x2C3FC4). Measurement byte 0xD0 = strong, 0x80 = no-signal (jmacato (github.com/jmacato) §9.3).

// Build + push a 0x83 RSSI_RESULTS block for the serving carrier. Body follows the 0x80 radio prefix
// {logch, BSIC, status, fn[3], ARFCN[2], status[2]} so the RR task reads a consistent (ARFCN, level)
// pair; block[16] (= body[12]) carries the raw measurement byte (0xD0 strong) the capture path reads.
static int push_rssi_results(Mad2* m, uint8_t type, uint8_t meas) {
    uint8_t body[16] = {0};
    uint32_t fn = gsm_frame_number(m);
    body[0] = LOGCH_SCH;                  // block[4]  serving carrier's logical channel
    body[1] = 0x00;                       // block[5]  BSIC
    body[2] = 0x01;                       // block[6]  status (valid result)
    body[3] = (uint8_t)(fn >> 16);        // block[7..9] frame number
    body[4] = (uint8_t)(fn >> 8);
    body[5] = (uint8_t)(fn & 0xFF);
    body[6] = (uint8_t)(SERVING_ARFCN >> 8);   // block[10..11] ARFCN 586
    body[7] = (uint8_t)(SERVING_ARFCN & 0xFF);
    body[8] = 0x00; body[9] = 0x00;       // block[12..13] status
    body[12] = meas;                      // block[16] raw RSSI measurement byte (0xD0 = strong)
    return mdircv_push(m, type, body, 13);
}

// ── Frame-scheduled BCCH System-Information broadcast (§9.2 step 2; mirrors Noks'
// EnqueueServingCellSystemInformation / BuildSystemInformation2/3/4) ───────────────────
// A real BTS continuously broadcasts SI2/SI3/SI4 on BCCH, each block tagged with the TDMA
// frame number at which it aired. The MCU L1 (0x2C3BEC) extracts block[7..9] -> [0x110FB4]
// and the BCCH handler (0x2C3B8E) forwards each to MM (task 13) as signal 0x0809 while the
// block counter 0x2C3B5C ([0x110FE8], caps at 3 = SI2/3/4) accrues and, on completion,
// notifies RR (task 10). So the faithful model is: after SCH sync, air SI3->SI2->SI4 on a
// steady cadence with a free-running frame number, forever — NOT one static SI3.

// Assemble a BCCH RECEIVED_BLOCK: block prefix {logch 0x50, BSIC, status, fn[3], ARFCN[2],
// status[2]} + the 23-byte L2 SI frame at block[14..], stamped with the live frame number.
// status(block[6])=0 makes the BCCH handler skip its BSIC/ARFCN dup-check (0x2C3BA2) so each
// SI always forwards to MM. ARFCN 586 (0x024A) matches the serving cell staged at 0x117834.
static int push_bcch_si(Mad2* m, const uint8_t* l2, int l2len, uint8_t status) {
    uint8_t body[10 + 23] = {0};
    uint32_t fn = gsm_frame_number(m);
    body[0] = LOGCH_BCCH;                 // block[4]  logical channel = BCCH
    body[1] = 0x00;                       // block[5]  BSIC (matches SCH)
    body[2] = status;                     // block[6]  status: 0 skips dup-check + forwards to MM;
                                          //           !=0 drives the BCCH block counter (0x2C3B5C)
                                          //           down toward 0 -> notifies RR task 10
    body[3] = (uint8_t)(fn >> 16);        // block[7]  frame number hi
    body[4] = (uint8_t)(fn >> 8);         // block[8]  frame number mid
    body[5] = (uint8_t)(fn & 0xFF);       // block[9]  frame number lo
    body[6] = 0x02; body[7] = 0x4A;       // block[10..11] ARFCN 0x024A = 586
    if (l2len > 23) l2len = 23;
    for (int i = 0; i < l2len; ++i) body[10 + i] = l2[i];   // block[14..] L2 SI frame
    return mdircv_push(m, MDI_RECEIVED_BLOCK, body, 10 + l2len);
}

// GSM 04.08 System Information Type 3 (§9.1.35) — the camping-critical one: Cell Identity,
// LAI (505/01 = PLMN 05 F5 10, same nibble packing as EF_IMSI; LAC 0x0001), Control Channel
// Description, Cell Options, Cell Selection Parameters, RACH Control Parameters, rest octets.
static const uint8_t SI3_L2[23] = {
    0x49,             // L2 pseudo-length (18 mandatory octets)
    0x06, 0x1B,       // RR PD (skip=0, PD=6) + message type SI3
    0x00, 0x01,       // Cell Identity = 0x0001
    0x05, 0xF5, 0x10, // LAI PLMN 505/01 (MCC 505, MNC 01)
    0x00, 0x01,       // LAI LAC = 0x0001
    0x00, 0x00, 0x00, // Control Channel Description (CCCH-not-combined, minimal)
    0x00,             // Cell Options (BCCH): PWRC/DTX/RADIO-LINK-TIMEOUT minimal
    0x00, 0x00,       // Cell Selection Parameters (CELL-RESELECT-HYST, MS-TXPWR, RXLEV-ACCESS-MIN)
    0x00, 0x00, 0x00, // RACH Control Parameters (max retrans, tx-integer, cell-bar-access)
    0x2B, 0x2B, 0x2B, // SI3 rest octets (spare padding 0x2B)
};

// SI Type 2 (§9.1.32) — Neighbour Cell Description (empty BA bitmap), NCC-permitted, RACH ctl.
static const uint8_t SI2_L2[23] = {
    0x59,             // L2 pseudo-length (22 mandatory octets)
    0x06, 0x1A,       // RR PD + message type SI2
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // Neighbour Cell Description bitmap (16 oct):
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   //   no neighbours (isolated serving cell)
    0x00,             // NCC permitted (all barred except our BCC=0)
    0x00, 0x00, 0x00, // RACH Control Parameters
};

// SI Type 4 (§9.1.36) — LAI, Cell Selection Parameters, RACH Control Parameters, rest octets.
static const uint8_t SI4_L2[23] = {
    0x49,             // L2 pseudo-length
    0x06, 0x1C,       // RR PD + message type SI4
    0x05, 0xF5, 0x10, // LAI PLMN 505/01
    0x00, 0x01,       // LAI LAC = 0x0001
    0x00, 0x00,       // Cell Selection Parameters
    0x00, 0x00, 0x00, // RACH Control Parameters
    0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B,  // SI4 rest octets
};

// Air the next SI in the SI3 -> SI2 -> SI4 broadcast cycle, stamping the GENUINE home PLMN (g_plmn,
// decoded from EF_IMSI) into the LAI so the broadcast cell matches the actual SIM. Returns 1 pushed.
static int push_next_si(Mad2* m, int idx, uint8_t status) {
    uint8_t l2[23]; int plmn_off;   // PLMN offset within L2: SI3 l2[5..7], SI4 l2[3..5], SI2 none
    switch (idx % 3) {
        case 0:  memcpy(l2, SI3_L2, sizeof l2); plmn_off = 5;  break;
        case 1:  memcpy(l2, SI2_L2, sizeof l2); plmn_off = -1; break;
        default: memcpy(l2, SI4_L2, sizeof l2); plmn_off = 3;  break;
    }
    if (plmn_off >= 0) { l2[plmn_off] = g_plmn[0]; l2[plmn_off+1] = g_plmn[1]; l2[plmn_off+2] = g_plmn[2]; }
    return push_bcch_si(m, l2, (int)sizeof l2, status);
}

// ── Channel-change confirm (§9.2 step 3: 0x89 CHANNEL_CHANGED_CNF) ─────────────────────
// After the SCH + SI broadcast, the RR cell-selection FSM (0x22E650) advances to state 11
// (8->20->11, ~step 12M) and BLOCKS in TASK_RECV_BLOCKING (0x22E944) awaiting a 0x1802 message
// of type 0x89 — the DSP's confirmation that it locked the serving-cell channel the firmware just
// configured. Our HLE DSP does not model the L1<->DSP channel-configure handshake (the 0x02 config
// reaches L1 handler 0x207864 and is processed internally, writing no DSP-observable command and
// leaving the pending-request slot [0x11161C]=0), so silicon's 0x89 never comes and the FSM parks
// forever in state 11 — no camp, no location update, no registration. This synthesizes that confirm.
//
// ROUTING (RE'd): type 0x89 -> TASK_4 d2m jumptable 0x2EDB48[0x89] -> thunk 0x2EDBA4 -> handler
// 0x2C3D4C. Gate [0x110FE0]==1 (L1 armed; already true). With no pending request ([0x11161C]==0,
// the always-taken path here since the firmware never records one) it takes 0x2C3DCC, which resets
// the channel-setup markers and forwards the block to the FSM via TASK_SEND_TYPED(task 10) @0x2C3DEC.
// The FSM state-11 handler (0x2321DA) then matches id 0x1802 + type 0x89, checks [0x110F96] net
// status, and (status != 0x50/0x51/0x52) issues the NEXT radio command to L1 (task 3) @0x232206 —
// i.e. it unblocks and the cell-selection sequence proceeds past the camp wall.
static int push_channel_cnf(Mad2* m) {
    uint8_t body[16] = {0};
    uint32_t fn = gsm_frame_number(m);
    body[0] = LOGCH_BCCH;                       // block[4]  the camped serving-cell logical channel
    body[1] = 0x00;                             // block[5]  BSIC (matches SCH)
    body[2] = 0x01;                             // block[6]  status = confirmed
    body[3] = (uint8_t)(fn >> 16);              // block[7..9] frame number
    body[4] = (uint8_t)(fn >> 8);
    body[5] = (uint8_t)(fn & 0xFF);
    body[6] = (uint8_t)(SERVING_ARFCN >> 8);    // block[10..11] ARFCN 586 (the configured cell)
    body[7] = (uint8_t)(SERVING_ARFCN & 0xFF);
    return mdircv_push(m, MDI_CHANNEL_CHANGED_CNF, body, 10);
}

// ── Serving-cell measurement report (§9.2: continuous 0x8A) ────────────────────────────
// Per the converged camp analysis ("SESSION 2026-07-17"): the RR
// cell-selection FSM stays in the state-20 dispatcher and NEVER commits our cell as the serving
// cell ([0x10B6D4] stays 0) → never advances to the camp/config-issue states (9/10 -> 21/28) →
// never issues the 0x02 config that would set the pending request [0x11161C] itself. A real DSP
// continuously reports serving-cell measurements; the state-20 dispatcher routes type 0x8A to the
// measurement handler 0x23212A (-> 0x293F1C -> MM task 13). Streaming strong 0x8A measurements for
// the serving ARFCN is the faithful lever to drive the serving-cell commit + camp advance. body =
// radio prefix {logch, BSIC, status, fn[3], ARFCN[2], status[2]} + measurement level at block[16].
static int push_meas_result(Mad2* m, uint8_t meas) {
    uint8_t body[16] = {0};
    uint32_t fn = gsm_frame_number(m);
    body[0]  = LOGCH_BCCH;                       // block[4]  serving-cell logical channel
    body[1]  = 0x00;                             // block[5]  BSIC
    body[2]  = 0x01;                             // block[6]  status = valid measurement
    body[3]  = (uint8_t)(fn >> 16);              // block[7..9] frame number
    body[4]  = (uint8_t)(fn >> 8);
    body[5]  = (uint8_t)(fn & 0xFF);
    body[6]  = (uint8_t)(SERVING_ARFCN >> 8);    // block[10..11] ARFCN 586
    body[7]  = (uint8_t)(SERVING_ARFCN & 0xFF);
    body[8]  = 0x00; body[9] = 0x00;             // block[12..13] status
    body[12] = meas;                             // block[16] measured level (0xD0 strong)
    return mdircv_push(m, MDI_MEAS_RESULT, body, 13);
}

// SIM-lock acceptance belongs to the rom4/rom6 local-security handshake
// (0x70/0x13/0x16/0x17 -> 0x74/0x34/0x35/0x36), which lets the FIRMWARE's own decode write its
// live table. Nothing in this file may patch that table.

// Per-step GSM bridge pump. Called from dsp_rom4_tick (gated). Never touches FSM/status.
void mdi_gsm_tick(Mad2* m) {
    if (g_en < 0) {
        // ONE master knob: GSMBRIDGE=1 turns on the whole network bring-up (RSSI scan answer +
        // SCH + SI2/3/4 broadcast + the DSP-radio responder in dsp_rom4.c). Each faithful piece
        // defaults ON; opt one out for A/B with <NAME>=0.
        g_en = getenv("GSMBRIDGE") ? 1 : 0; g_log = getenv("GSMLOG") ? 1 : 0;
        g_noguard = getenv("GSMBRIDGE_NOGUARD") ? 1 : 0;
        const char* cad = getenv("GSMBRIDGE_SI_CAD");
        g_si_cad = (cad && *cad) ? (uint64_t)strtoull(cad, 0, 0) : 250000ull;
        const char* st = getenv("GSMBRIDGE_SI_STATUS");
        g_si_stov = (st && *st) ? (int)strtoul(st, 0, 0) : -1;
        g_rssi = gsm_sub_on("GSMBRIDGE_RSSI");   // sub-features default ON; =0 opts out
        g_si   = gsm_sub_on("GSMBRIDGE_SI");
        // 0x89 confirm is default-OFF (opt-in GSMBRIDGE_CNF=1): with no pending channel-change
        // request at [0x11161C] (the firmware never issues the 0x02 config — L1<->DSP unmodelled),
        // it takes the handler's ABORT path (0x2C3DCC) which sets net-status [0x110F96]=0x50
        // (channel-setup-failed), NOT a successful camp. Kept as a scaffold + A/B knob, not a
        // default, because on its own it aborts rather than advances registration.
        g_cnf  = getenv("GSMBRIDGE_CNF") ? (atoi(getenv("GSMBRIDGE_CNF")) != 0) : 0;
        g_meas = getenv("GSMBRIDGE_MEAS") ? (atoi(getenv("GSMBRIDGE_MEAS")) != 0) : 0;
        const char* rt = getenv("GSMBRIDGE_RSSI_TYPE");
        g_rssi_type = (rt && *rt) ? (uint8_t)strtoul(rt, 0, 0) : MDI_RSSI_RESULTS;
        mad2_sim_current_plmn(m, g_plmn);
    }
    if (!g_en || !m->mem) return;
    // swSIM/bridged cards can replace the selected default identity after the
    // bridge was initialised. The cell follows what the firmware actually read.
    mad2_sim_current_plmn(m, g_plmn);

    // The compatibility bridge is armed by state transitions, never by an
    // instruction-count approximation: the DSP must be running and the L1 search
    // interlock must report an active SCH acquisition.  The ring itself provides
    // back-pressure and mdircv_push retries until the consumer is ready.
    if (g_stage == 0) {
        if (!m->dsp_running) return;                            // DSP code live (DSP-observable)
        if (!g_noguard && m->mem[L1_SCH_ARMED & m->mem_mask] != 1) return; // opt-in MCU safety interlock
        // §9.2 step 1: answer the 0x56 power scan with RSSI results FIRST (GSMBRIDGE_RSSI=1) — a
        // strong (0xD0) measurement for the serving ARFCN — so the firmware's radio task ranks the
        // carrier and stamps the serving cell's level before the SCH sync arrives. One-shot; the
        // next tick pushes the SCH.
        if (g_rssi && !g_rssi_done) {
            if (push_rssi_results(m, g_rssi_type, RSSI_STRONG)) {
                g_rssi_done = 1;
                if (g_log)
                    printf("[gsm] RSSI type 0x%02X pushed (ARFCN %u, meas 0x%02X) @mono=%llu\n",
                           g_rssi_type, SERVING_ARFCN, RSSI_STRONG, (unsigned long long)m->rtc_mono);
            }
            return;
        }
        if (push_sch(m)) {
            g_stage = 1;
            g_si_next = m->rtc_mono + g_si_cad;
            g_meas_next = m->rtc_mono + g_si_cad;
            if (g_log)
                printf("[gsm] SCH RECEIVED_BLOCK pushed (logch 0x40) @mono=%llu fsm_state=0x%02X\n",
                       (unsigned long long)m->rtc_mono, m->mem[FSM_STATE & m->mem_mask]);
        }
        return;
    }
    // Stage 1 — CONTINUOUS frame-scheduled SI broadcast (GSMBRIDGE_SI=1). Per jmacato (github.com/jmacato) §9.2 step 2
    // the DSP streams SI2/SI3/SI4 as 0x80 BCCH blocks after the SCH; a real BTS re-broadcasts them
    // forever. Air the next SI in the SI3->SI2->SI4 cycle every GSMBRIDGE_SI_CAD mono cycles
    // (default 250k), each stamped with the live frame number, whenever the MDIRCV ring is drained.
    // Each routes to MM (task 13) as signal 0x0809 and drives the BCCH block counter (0x2C3B5C /
    // [0x110FE8]). Stays in stage 1 (perpetual) — never one-shot — so the cell stays "on air".
    // §9.2 step 3 — one-shot channel-change confirm (0x89). Deliver it once the cell-selection FSM
    // (0x22E650) has parked in state 11 (the state whose handler 0x2321DA blocks awaiting a 0x1802
    // type-0x89), which is the reliable proxy for "the firmware has configured the serving-cell
    // channel and is waiting for the DSP's lock confirmation." Reading the FSM state ([0x111742]
    // BE halfword == 0x000B) is an MCU-side interlock (same discipline as L1_SCH_ARMED) — the
    // faithful trigger would be the DSP observing the 0x02 config, but that L1<->DSP path is
    // unmodelled. One-shot: the confirm is a single event; the FSM advances out of state 11.
    if (g_stage == 1 && g_cnf && !g_cnf_done) {
        uint16_t st = (uint16_t)((m->mem[FSM_STATE & m->mem_mask] << 8)
                                 | m->mem[(FSM_STATE + 1) & m->mem_mask]);
        if (st == 0x000B) {
            if (push_channel_cnf(m)) {
                g_cnf_done = 1;
                if (g_log)
                    printf("[gsm] 0x89 CHANNEL_CHANGED_CNF pushed (FSM state 11 -> confirm) @mono=%llu\n",
                           (unsigned long long)m->rtc_mono);
            }
            return;
        }
    }
    // Continuous 0x8A serving-cell measurement stream (GSMBRIDGE_MEAS=1, opt-in). Drives the RR
    // serving-cell commit that the state-20 dispatcher otherwise never performs (the camp gap).
    // Own cadence (reuse g_si_cad); strong level so the serving carrier ranks top.
    if (g_stage == 1 && g_meas) {
        if (g_meas_next && m->rtc_mono >= g_meas_next) {
            if (push_meas_result(m, RSSI_STRONG)) {
                g_meas_next = m->rtc_mono + g_si_cad;
                if (g_log)
                    printf("[gsm] 0x8A MEAS pushed (ARFCN %u, meas 0x%02X) @mono=%llu\n",
                           SERVING_ARFCN, RSSI_STRONG, (unsigned long long)m->rtc_mono);
                return;
            }
        }
    }
    if (g_stage == 1 && g_si) {
        if (g_si_next && m->rtc_mono < g_si_next) return;
        // status: env override applies to every block; otherwise AUTO — the first 3 SIs carry
        // status 1 to walk the BCCH block counter (0x2C3B5C / [0x110FE8]) 3->0 exactly once, which
        // notifies RR (task 10) that the BCCH read is complete; subsequent blocks carry status 0
        // (pure MM forward) so the counter never underflows (the firmware does not re-arm it).
        uint8_t status = (g_si_stov >= 0) ? (uint8_t)g_si_stov : (g_si_idx < 3 ? 1 : 0);
        if (push_next_si(m, g_si_idx, status)) {
            if (g_log)
                printf("[gsm] SI%c BCCH pushed (LAI PLMN %02X%02X%02X, fn=%u, st=%u) @mono=%llu\n",
                       g_si_idx % 3 == 0 ? '3' : (g_si_idx % 3 == 1 ? '2' : '4'),
                       g_plmn[0], g_plmn[1], g_plmn[2],
                       gsm_frame_number(m), status, (unsigned long long)m->rtc_mono);
            g_si_idx++;
            g_si_next = m->rtc_mono + g_si_cad;
        }
        return;
    }
    // ── FRONTIER: camp is blocked by the unmodelled L1<->DSP channel-config handshake.
    // The cell-sel FSM (0x22E650) parks in state 11 awaiting a 0x1802/type-0x89 CHANNEL_CHANGED_CNF
    // (push_channel_cnf above delivers it under GSMBRIDGE_CNF=1). But the 0x89 handler 0x2C3D4C
    // correlates the confirm against a PENDING channel-change request at [0x11161C], which is NEVER
    // written in our boot because the firmware's 0x02 CHANNEL_CONFIGURE goes to L1 handler 0x207864
    // and is processed internally with no DSP-observable command (the radio-interface gap). With
    // [0x11161C]==0 the handler takes its abort path (0x2C3DCC), setting net-status [0x110F96]=0x50
    // (channel-setup-failed) and advancing the FSM 11->20 but NOT camping — so MM never initiates the
    // location update (RR establishment 0x2C2D34 = 0), [0x110F97] (the REGISTRATION status the device
    // state machine 0x2EB1FE reads; needs ~0xB0/0xC0/0xD2 for state-1 registered) stays 0, and
    // [0x11FD09] LOGGED_INTO_NETWORK stays 0.
    //
    // NEXT BUILD (the real camp fix): model the L1<->DSP channel-config so [0x11161C] is populated
    // with a valid pending request (block[4]&1 == [ptr+2], [ptr+3]==1) before the 0x89 — i.e. find
    // where silicon's L1 records the pending channel-change (hook the firmware's 0x02 config path /
    // L1 0x207864) so the 0x89 takes the SUCCESS path (not abort). Only then does the FSM camp, MM
    // trigger the LU, and the rest of §9.2 (RACH → Immediate Assignment → SDCCH → LAPDm SABM →
    // LOCATION UPDATING REQUEST/ACCEPT → [0x110F97] registered) become reachable. See
    // "SESSION 2026-07-17".
}

uint64_t mdi_gsm_next_wake(Mad2* m) {
    if (g_en < 0) return m->rtc_mono;  // one event performs lazy configuration
    if (!g_en || !m->mem) return UINT64_MAX;
    if (g_stage == 0) {
        if (!m->dsp_running) return UINT64_MAX;
        if (!g_noguard && m->mem[L1_SCH_ARMED & m->mem_mask] != 1) return UINT64_MAX;
        return m->rtc_mono;             // RSSI/SCH event, ring back-pressure decides delivery
    }
    if (g_cnf && !g_cnf_done) {
        uint16_t st = (uint16_t)((m->mem[FSM_STATE & m->mem_mask] << 8)
                                 | m->mem[(FSM_STATE + 1) & m->mem_mask]);
        if (st == 0x000Bu) return m->rtc_mono;
    }
    uint64_t next = UINT64_MAX;
    if (g_meas && g_meas_next && g_meas_next < next) next = g_meas_next;
    if (g_si && g_si_next && g_si_next < next) next = g_si_next;
    return next;
}

void mdi_gsm_advance_to(Mad2* m, uint64_t cycles) {
    (void)cycles;
    mdi_gsm_tick(m);
}
