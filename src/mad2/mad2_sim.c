// MAD2 — SIM card: SIMI UART block + ISO-7816 T=0 transport + GSM 11.11 / TS
// 51.011 EF tree + real-card shadow bridge. Extracted from mad2.c; see
// mad2_internal.h. Bus-facing entries (sim_read/sim_write/sim_tick) are
// declared there; everything else is private to this file.

#include "mad2/mad2_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// SIM bridge (real-card shadow comparison). Native-only: the host-side ISO-7816
// T=0 driver uses POSIX serial, so it's compiled out of the wasm build. Activated
// at runtime by the SIMBRIDGE=<serial-dev> env var. See docs/sim-bridge-protocol.md.
#ifndef __EMSCRIPTEN__
#include "sim_bridge.h"
#endif

// --- SIM card (SIMI UART block + ISO-7816 T=0 + GSM 11.11 EF tree) -----------
// The firmware drives the SIM reader UART over I/O 0x36-0x3F (see mad2.h). We model
// a present GSM SIM: deliver a canned ATR on activate, run the T=0 transport, and
// answer the boot-path APDUs (SELECT / READ BINARY / READ RECORD / GET RESPONSE /
// STATUS / VERIFY-CHV) from an in-memory EF tree. This lets the firmware's own SIM
// driver detect the card, read its EFs, and clear the disp49 SIM gate faithfully.
// No crypto: RUN GSM ALGORITHM is the network-auth wall and out of scope.

// SIMLOG=1 (native env knob): trace each assembled SIM APDU + its response + the ATR.
// Quiet by default; mirrors the other mad2 debug knobs (CCVERBOSE, MBUSLOG, ...).
static int sim_log_on(void) { static int v = -1; if (v < 0) v = getenv("SIMLOG") ? 1 : 0; return v; }

// SIMI offsets within the 0x20000 window.

// SIMI_UART_INT bits, decoded from the firmware's FIQ6 ISR (0x2D490E). The ISR tests
// each bit with `lsr #n; bcc` (carry = bit n-1) and dispatches:
//   lsr #6 -> bit5 -> post SM "status 6"            (desc 0x32E748)
//   lsr #5 -> bit4 -> sim_advance 0x2D46E4 (TX-empty -> SM "status 7")
//   lsr #7 -> bit6 -> sim_receive 0x2D487C (RX data ready)
//   lsr #2 -> bit1 -> post SM "status 16"           (desc 0x32E770)
//   lsr #8 -> bit7 -> post SM "status 15"           (desc 0x32E768)
// So bit6 = RX-ready and bit4 = TX-empty/threshold (the one that yields status 7).
#define SIM_INT_RXRDY    0x40u   // bit6: RX data available
#define SIM_INT_TXEMPTY  0x10u   // bit4: TX FIFO drained (drives sim_advance -> status 7)

// ISO-7816 status words.
#define SW_OK_HI 0x90
#define SW_OK_LO 0x00
// "61 xx" = OK, xx bytes available via GET RESPONSE (T=0 chaining for SELECT etc.).
// GSM/3GPP TS 51.011 SIMs answer SELECT with "9F XX" (XX = length of the FCP, fetched
// next with GET RESPONSE / read by STATUS), not the ISO generic "61 XX". The DCT3 SIM
// driver specifically follows the 9F path (a 61 SW1 drops it into a reset/retry loop).
#define SW_GR_HI 0x9F

// --- RX FIFO helpers ---------------------------------------------------------
static int sim_rx_count(const Mad2* m) {
    return (int)((uint8_t)(m->sim_rx_tail - m->sim_rx_head));
}
static void sim_rx_push(Mad2* m, uint8_t b) {
    if ((uint8_t)(m->sim_rx_tail - m->sim_rx_head) >= sizeof(m->sim_rx)) return; // full: drop
    m->sim_rx[m->sim_rx_tail % sizeof(m->sim_rx)] = b;
    m->sim_rx_tail++;
}
static uint8_t sim_rx_pop(Mad2* m) {
    if (m->sim_rx_head == m->sim_rx_tail) return 0xFF;
    return m->sim_rx[m->sim_rx_head++ % sizeof(m->sim_rx)];
}
// Make the firmware notice queued RX bytes: report them via SIMI_RXD_QUE, set the
// RX-ready UART-int bit, and assert FIQ6. (Idempotent; called whenever bytes are
// pushed and re-armed from the timer tick while the FIFO is non-empty.)
static void sim_rx_signal(Mad2* m) {
    if (sim_rx_count(m) > 0) {
        m->sim_uart_int |= SIM_INT_RXRDY;
        mad2_raise_fiq(m, 6);          // FIQ6 = SIM-UART
    }
}
// Signal "TX FIFO drained" (bit4). The firmware's sim_advance reads the bytes-sent
// counter vs. the command length: when all bytes are out it posts SM status 7 (the
// "ACK, continue" the SELECT/data sends gate on); otherwise it transmits more. Our
// TX FIFO drains instantly (TXD_QUE reads 0), so we raise this right after a flush.
static void sim_txempty_signal(Mad2* m) {
    m->sim_uart_int |= SIM_INT_TXEMPTY;
    mad2_raise_fiq(m, 6);              // FIQ6 = SIM-UART (bit4 = TX-empty)
}

// --- GSM 11.11 / TS 51.011 EF tree (canned, plausible) -----------------------
// Each entry is one transparent or linear-fixed EF the boot/MMI path reads. The
// data are minimal-but-valid: PIN disabled (no CHV prompt), GSM phase 2, a test
// IMSI/ICCID, English language. NO real Ki — RUN GSM ALGORITHM is out of scope.
typedef struct {
    uint16_t fid;        // file id (e.g. 0x6F07 EF_IMSI)
    uint8_t  type;       // 0 = transparent, 1 = linear-fixed (records)
    uint8_t  rec_len;    // record length (linear-fixed)
    uint8_t  recs;       // record count (linear-fixed)
    uint16_t size;       // body size (transparent)
    const uint8_t* body; // file body
} SimEf;

// EF_ICCID (2FE2, MF transparent, 10 bytes BCD, swapped nibbles). 20-digit ICCID
// 89490100000000000014 (89=telecom, 49=test country, issuer 01, serial, Luhn check
// digit 4). The previous value (89010100000000000090) FAILED the Luhn check (sum 27),
// which a strict SIM-recognition path treats as "card not accepted". This one passes.
static const uint8_t EF_ICCID[10]   = {0x98,0x16,0x10,0x04,0x00,0x30,0x56,0x64,0x52,0x36};  // real fixture card ICCID
// EF_LP (6F05, language preference): 0x00 = German per TS 51.011 Annex; use English
// (0x01)? GSM 11.11 EF_LP holds language codes (0=German,1=English,...). Offer English.
static const uint8_t EF_LP[4]       = {0x01,0xFF,0xFF,0xFF};  // real card EF_LP is 4 bytes
// EF_ELP (2F05, extended language preference, under MF): real fixture card = "en".
static const uint8_t EF_ELP[6]      = {0x65,0x6E,0xFF,0xFF,0xFF,0xFF};
// EF_IMSI (6F07, transparent 9 bytes): len(1) + IMSI TBCD. Test IMSI 001010123456789.
static const uint8_t EF_IMSI[9]     = {0x08,0x09,0x10,0x10,0x32,0x54,0x76,0x98,0x10};
// EF_AD (6FAD, administrative data, 4 bytes): normal operation, OFM off, len(MNC)=2.
static const uint8_t EF_AD[4]       = {0x00,0xFF,0xFF,0x02};  // real fixture card (normal op, MNC len 2)
// EF_PHASE (6FAE, 1 byte): GSM phase. 0=Phase 1, 2=Phase 2, 3=Phase 2+. Phase 2
// triggers stricter FCP conformance checks in the firmware (CHV1 status parsing,
// SST service-bit validation). Declaring Phase 1 makes the firmware use the older,
// looser SIM-init state machine — useful for getting past PIN-required init when
// the rest of our SIM model isn't 100% Phase-2 compliant. Toggle via SIMPHASE env.
uint8_t EF_PHASE[1]           = {0x03};  // real fixture card = Phase 2+ (SIMPHASE overrides)
// EF_SST (6F38, SIM service table): 2 bits per service (bit0 allocated, bit1 activated;
// service N at byte (N-1)/4, nibble-pair ((N-1)%4)*2). Per GSM 11.11 §10.3.7 Table 10:
//   svc1 CHV1-disable, svc2 ADN, svc3 FDN, svc4 SMS,
//   svc5 AOC, svc6 CCP, svc7 PLMN Selector, svc8 RFU,
//   svc9 MSISDN, svc12 SMSP, svc13 LND, ...
// PLMN Selector (svc7) MUST be advertised on Phase 2 SIMs or some firmware stacks
// reject the SIM at recognition time. Byte1 now advertises services 5/6/7 (AOC/CCP/
// PLMN-selector) as allocated+activated, leaving svc8 (RFU) at 0.
//   byte0 0xFF = svc1-4 allocated+activated
//   byte1 0x3F = svc5 (alloc+act 0x3) | svc6 (alloc+act 0x3<<2) | svc7 (alloc+act 0x3<<4) | svc8 0
//   byte2 0xC3 = svc9 (MSISDN) + svc12 (SMSP)
//   byte3 0x03 = svc13 (LND)
static const uint8_t EF_SST[15]     = {0xCF,0x30,0xCF,0x0F,0x03,0x00,0xDC,0x03,0x00,0x0C,0x00,0xC0,0x00,0x00,0x00};  // real fixture card SST
// EF_SPN (6F46, service provider name, 17 bytes): display-condition + "Service Provider".
// (Name matches Nokia's own test-SIM fixture sim_init.sd@0x3511 — see docs/3330-sim-fixture.md.)
static const uint8_t EF_SPN[17]     = {0x00,'A','L','D','I','m','o','b','i','l','e',0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};  // real fixture card SPN
// EF_LOCI (6F7E, location information, 11 bytes): TMSI invalid, LAI, "not updated".
static const uint8_t EF_LOCI[11]    = {0xFF,0xFF,0xFF,0xFF,0x05,0xF5,0x10,0xFF,0xFE,0x00,0x03};  // real fixture card LOCI
// EF_BCCH (6F74) / EF_KC (6F20) / EF_FPLMN (6F7B): plausible empty/neutral.
static const uint8_t EF_KC[9]       = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x07};
static const uint8_t EF_FPLMN[12]   = {0x05,0xF5,0x20,0x05,0xF5,0x30,0x05,0xF5,0x60,0xFF,0xFF,0xFF};  // real fixture card FPLMN
// EF_ACC (6F78, access control class, 2 bytes): access class 0 enabled, permissive — a SIM
// with no class bits set bars itself from the network, so light bit0 (don't self-bar).
static const uint8_t EF_ACC[2]      = {0x00,0x10};  // real fixture card access class
// EF_BCCH (6F74, broadcast control channel list, 16 bytes): empty (no cached neighbours).
static const uint8_t EF_BCCH[16]    = {0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xFF,0xFF,0xFE,0x00,0x00};  // real fixture card BCCH
// EF_ECC (6FB7, emergency call codes): linear-fixed, 3-byte swapped-BCD records (F=pad).
// 112 (universal), 999 (UK/legacy), 911 (NA). Matches the choice in Nokia's own 3330 test SIM
// fixture sim_init.sd@0x34F5+ (ASCII "112"/"999"/"911"; see docs/3330-sim-fixture.md). The
// prior set used 000 (Australia/NZ) instead of 911; aligning to Nokia's defaults so any MMI
// path that probes for canonical Nokia ECC content sees what the same MMI on 3330 expected.
static const uint8_t EF_ECC[15]     = {0x00,0xF0,0xFF, 0x11,0xF2,0xFF, 0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF};  // real fixture card ECC (transparent, 15B)
// EF_MSISDN (6F40), EF_ADN (6F3A), EF_SMSP (6F42): linear-fixed record files modelled with
// real record geometry but blank (NULL) bodies — record content is subscriber-private (spec section 6).
// EF_PLMNsel (6F30): transparent, 48 bytes, no preferred PLMN (FF). Operator HPLMN list
// here is card identity (spec section 6); kept generic.
static const uint8_t EF_PLMNSEL[48] = {0x05,0xF5,0x10, [3 ... 47] = 0xFF};  // real fixture card preferred PLMN
// New EFs the real card provisions (geometry only; private content stays blank).
static const uint8_t EF_HPLMN[1] = {0x05};  // real fixture card HPLMN search period
static const uint8_t EF_SMSS[2]  = {0x01,0xFF};
// EF_ACM/ACMmax/PUCT (charge counters): linear-fixed, blank records.

static const SimEf SIM_EFS[] = {
    {0x2FE2, 0, 0, 0, sizeof EF_ICCID, EF_ICCID},   // MF: ICCID
    {0x2F05, 0, 0, 0, sizeof EF_ELP,   EF_ELP},      // MF: extended language preference
    {0x6F05, 0, 0, 0, sizeof EF_LP,    EF_LP},       // DF_GSM: language preference
    {0x6F07, 0, 0, 0, sizeof EF_IMSI,  EF_IMSI},     // IMSI
    {0x6F20, 0, 0, 0, sizeof EF_KC,    EF_KC},       // Kc (transparent, 9 bytes; FF×8 + key-seq 07)
    {0x6F30, 0, 0, 0, sizeof EF_PLMNSEL, EF_PLMNSEL},// PLMNsel (transparent, 48 bytes)
    {0x6F38, 0, 0, 0, sizeof EF_SST,   EF_SST},      // SIM service table
    {0x6F46, 0, 0, 0, sizeof EF_SPN,   EF_SPN},      // service provider name
    {0x6F7B, 0, 0, 0, sizeof EF_FPLMN, EF_FPLMN},    // FPLMN
    {0x6F7E, 0, 0, 0, sizeof EF_LOCI,  EF_LOCI},     // location information
    {0x6FAD, 0, 0, 0, sizeof EF_AD,    EF_AD},       // administrative data
    {0x6FAE, 0, 0, 0, sizeof EF_PHASE, EF_PHASE},    // phase
    {0x6F78, 0, 0, 0, sizeof EF_ACC,   EF_ACC},      // access control class
    {0x6F74, 0, 0, 0, sizeof EF_BCCH,  EF_BCCH},     // BCCH list
    {0x6FB7, 0, 0, 0, sizeof EF_ECC,   EF_ECC},      // emergency call codes (transparent, 15 bytes)
    {0x6F31, 0, 0, 0, sizeof EF_HPLMN, EF_HPLMN},    // HPLMN search period (1 byte)
    {0x6F3A, 1, 32, 50, 0,             NULL},        // ADN phonebook (50 x 32-byte records, blank)
    {0x6F42, 1, 46, 1, 0,              NULL},        // SMSP (1 x 46-byte record, blank)
    {0x6F43, 0, 0, 0, sizeof EF_SMSS,  EF_SMSS},     // SMSS SMS status (2 bytes)
    {0x6F40, 1, 38, 3, 0,              NULL},        // MSISDN (DF_TELECOM, 3 x 38-byte records, blank)
};
#define SIM_EF_COUNT (int)(sizeof(SIM_EFS)/sizeof(SIM_EFS[0]))

static const SimEf* sim_find_ef(uint16_t fid) {
    for (int i = 0; i < SIM_EF_COUNT; ++i) if (SIM_EFS[i].fid == fid) return &SIM_EFS[i];
    return NULL;
}

// Build the SELECT response (GSM 11.11 §9.2.1 "Response data of SELECT"). We return
// the legacy GSM (phase 1/2) FCP, which the DCT3 firmware understands:
//   [0..1] RFU, [2..3] file size (BE), [4..5] file id (BE), [6] file type
//   (1=MF,2=DF,4=EF), [7..12] RFU, [13] length of following data, [14..] file
//   characteristics. For an EF: [13]=2 then [14] structure (0=transparent,1=linear),
//   ... For simplicity we emit the minimum the firmware parses (size+id+type+
//   structure+record length + access conditions "always/CHV1 disabled").
static uint8_t sim_build_select_resp(Mad2* m, const SimEf* ef, uint16_t fid, int is_df) {
    uint8_t* r = m->sim_gr_buf;
    memset(r, 0, sizeof m->sim_gr_buf);
    uint16_t fsize = ef ? (ef->type ? (uint16_t)(ef->rec_len * ef->recs) : ef->size) : 0;
    r[0] = 0x00; r[1] = 0x00;                       // RFU
    r[2] = (uint8_t)(fsize >> 8); r[3] = (uint8_t)fsize;  // file size
    r[4] = (uint8_t)(fid >> 8);   r[5] = (uint8_t)fid;    // file id
    r[6] = is_df ? (fid == 0x3F00 ? 0x01 : 0x02) : 0x04;  // 1=MF,2=DF,4=EF
    r[7] = 0x00; r[8] = 0x00; r[9] = 0x00; r[10] = 0x00; r[11] = 0x00;  // bytes 8-12: RFU (5)
    if (is_df) {
        // MF/DF SELECT response, GSM 11.11 §9.2.1 (r[] 0-indexed; spec byte N == r[N-1]):
        //   bytes1-2 RFU, 3-4 available memory, 5-6 file ID, 7 type, 8-12 RFU(5),
        //   byte13 (r[12]) = LENGTH of the following GSM-specific data (12),
        //   byte14 file characteristics, 15 #child-DFs, 16 #child-EFs, 17 #codes, 18 RFU,
        //   19 CHV1 status, 20 CHV1-unblock, 21 CHV2 status, 22 CHV2-unblock, 23-25 RFU/admin.
        // This is the GENERIC GSM directory status — it carries NO operator identity; the
        // Telstra/PLMN identity lives in the EFs (IMSI 6F07, LOCI 6F7E, PLMNsel 6F30, ...).
        // PRIOR BUG: the length byte was at r[13] + an invented 2-byte "GSM memory" field
        // shifted #EFs/CHV-status by 2 -> the firmware misparsed STATUS and re-polled forever.
        r[12] = 0x15;                               // byte13: length of following GSM data (21 -> total 34)
        // byte14 file characteristic (GSM 11.11 §10.1.1.1.1): bit0 = CHV1-disabled flag
        // (1=disabled, 0=enabled), bit7 = clock-stop allowed. (Bit 5 has no defined
        // meaning here — the older "+0x20" comment was misreading the spec; the
        // PIN-off path worked anyway because bit0 happened to be set in 0xA1.)
        r[13] = m->sim_pin_enabled ? 0x80 : 0x81;   // 0x80 = clock-stop only (CHV1 req'd); 0x81 = + CHV1-disabled
        r[14] = 0x00;                               // byte15: # child DFs
        r[15] = 0x16;                               // byte16: # child EFs (22, per real GSM SIM)
        r[16] = 0x03;                               // byte17: # CHV/unblock/admin codes
        r[17] = 0x00;                               // byte18: RFU
        r[18] = (uint8_t)(0x80 | (m->sim_pin_tries & 0x0F));  // byte19: CHV1 status (initialised+tries; disable signalled in file char)
        r[19] = (uint8_t)(0x80 | (m->sim_puk_tries & 0x0F));  // byte20: CHV1 unblock status
        r[20] = 0x83;                               // byte21: CHV2 status (initialised, 3)
        r[21] = 0x8A;                               // byte22: CHV2 unblock status (initialised, 10)
        r[22] = 0x00; r[23] = 0x00; r[24] = 0x00;   // bytes 23-25: RFU / admin
        // bytes 26-34 (r[25..33]) left 0 by memset; real GSM SIMs return the full 34-byte DF FCP.
        return 34;
    }
    // EF SELECT response (GSM 11.11 §9.3): byte9-11 (r[8..10]) access conditions
    // (each nibble: 0=ALW, 1=CHV1, 2=CHV2, ... ; byte9 hi=READ/SEEK lo=UPDATE),
    // byte12 (r[11]) file status, byte13 (r[12]) length of following, byte14 (r[13])
    // structure (0=transparent,1=linear-fixed,3=cyclic), byte15 (r[14]) record length.
    // When CHV1 is enabled the protected EFs require CHV1 to READ/UPDATE so the firmware
    // knows the PIN is needed; ICCID/LP stay readable to allow basic recognition.
    // GSM 11.11 standard access-condition bytes per EF, matching the real fixture card
    // (advertisement only; readability is governed by the alw/PIN gate below).
    uint8_t ac8 = 0x11, ac9 = 0xFF, ac10 = 0x55;
    switch (fid) {
        case 0x2FE2: ac8 = 0x0F; ac10 = 0xFF; break;
        case 0x6F05: case 0x2F05: ac8 = 0x01; break;
        case 0x6F46: ac8 = 0x00; break;
        case 0x6FAE: case 0x6FAD: case 0x6FB7: ac8 = 0x05; break;
        case 0x6F07: ac8 = 0x15; ac10 = 0x15; break;
        case 0x6F31: case 0x6F38: case 0x6F3E: case 0x6F3F:
        case 0x6F78: ac8 = 0x15; break;
        case 0x6F7E: ac10 = 0x15; break;
        case 0x6F3A: ac10 = 0x22; break;
    }
    r[8] = ac8; r[9] = ac9; r[10] = ac10;
    r[11] = 0x01;                                   // file status: not invalidated, readable
    r[12] = 0x02;                                   // length of following (structure + rec len)
    r[13] = ef ? (ef->type ? 0x01 : 0x00) : 0x00;   // structure: 0=transparent,1=linear-fixed
    r[14] = ef ? ef->rec_len : 0x00;                 // record length (linear-fixed)
    return 15;
}

// Is INS a "command with incoming data" (the firmware sends P3 data bytes TO the
// card after the procedure-byte ACK)? GSM/ISO case-3 commands.
static int sim_ins_has_indata(uint8_t ins) {
    switch (ins) {
    case 0x10:  // TERMINAL PROFILE (SAT) — Lc bytes of profile data
    case 0x14:  // TERMINAL RESPONSE (SAT) — Lc bytes of response data
    case 0xC2:  // ENVELOPE (SAT) — Lc bytes of command (SMS-PP / menu select / ...)
    case 0xA4:  // SELECT (file id)
    case 0xA2:  // SEEK / SEARCH RECORD (search pattern in; rec# via GET RESPONSE)
    case 0x20:  // VERIFY CHV
    case 0x24:  // CHANGE CHV
    case 0x26:  // DISABLE CHV
    case 0x28:  // ENABLE CHV
    case 0x2C:  // UNBLOCK CHV
    case 0xDC:  // UPDATE RECORD
    case 0xD6:  // UPDATE BINARY
    case 0x32:  // INCREASE (amount in; new value via GET RESPONSE)
    case 0x88:  // RUN GSM ALGORITHM (RAND in; SRES+Kc via GET RESPONSE)
        return 1;
    // Data-out (B0/B2/C0/F2/12 FETCH) and no-data (04 INVALIDATE/44 REHABILITATE/
    // FA SLEEP) commands return 0 — no incoming data phase.
    default: return 0;
    }
}

// Compare a supplied 8-byte CHV (ASCII digits, 0xFF-padded) to a stored code.
static int sim_chv_match(const uint8_t* supplied, uint16_t n, const uint8_t* stored) {
    if (n < 8) return 0;
    for (int i = 0; i < 8; ++i) if (supplied[i] != stored[i]) return 0;
    return 1;
}

// --- A3/A8 (RUN GSM ALGORITHM) — synthetic operator authentication --------------
//
// GSM A3/A8 is OPERATOR-DEFINED: the network's AuC and the SIM share a 128-bit secret
// Ki and an agreed A3/A8 function; given a 16-byte RAND challenge they each derive
// SRES (4 bytes, returned to the network) and Kc (8 bytes, the A5 cipher key). We are a
// synthetic operator, so we bake OUR OWN Ki and OUR OWN deterministic A3/A8 here. This is
// a self-consistent test SIM (same as sysmoSIM/OsmocomBB test cards use a known Ki) — it
// authenticates against any network programmed with the same Ki + this algorithm. It does
// NOT and CANNOT reproduce a real card's auth: a real Ki is fused in hardware and never
// leaves the card (real SRES/Kc are only obtainable by forwarding 0x88 over the bridge).
//
// NOTE: SIM_KI is a SYNTHETIC TEST KEY, not a real subscriber secret. It is never readable
// over the SIM interface (there is no EF for Ki) — it only ever feeds this function.
static const uint8_t SIM_KI[16] = {
    0x4E,0x4F,0x4B,0x49,0x41,0x44,0x43,0x54,0x33,0x45,0x4D,0x55,0x4B,0x49,0x30,0x31  // "NOKIADCT3EMUKI01"
};

// splitmix64-style table-free finalizer — strong avalanche, fully deterministic.
static uint64_t sim_mix64(uint64_t x) {
    x ^= x >> 30; x *= 0xBF58476D1CE4E5B9ULL;
    x ^= x >> 27; x *= 0x94D049BB133111EBULL;
    x ^= x >> 31; return x;
}

// Synthetic A3/A8: (Ki[16], RAND[16]) -> SRES[4] + Kc[8]. Table-free, deterministic.
// Kc has its low 10 bits cleared, matching the GSM convention (A5 uses 54 significant bits).
static void sim_a3a8(const uint8_t* ki, const uint8_t* rand, uint8_t* sres, uint8_t* kc) {
    uint64_t a = 0x6A09E667F3BCC908ULL, b = 0xBB67AE8584CAA73BULL;
    for (int i = 0; i < 16; ++i) {
        a = sim_mix64(a ^ ((uint64_t)ki[i] << ((i & 7) * 8)) ^ (uint64_t)rand[i]);
        b = sim_mix64(b + (uint64_t)ki[i] * 131u + (uint64_t)rand[i] * 1099511628211ULL + (uint64_t)i);
    }
    for (int r = 0; r < 8; ++r) { a = sim_mix64(a ^ b ^ (uint64_t)r); b = sim_mix64(b ^ a); }
    uint64_t o = a ^ b;
    for (int i = 0; i < 4; ++i) sres[i] = (uint8_t)(o >> (56 - 8 * i));   // top 32 bits
    uint64_t kk = sim_mix64(a + b);
    for (int i = 0; i < 8; ++i) kc[i] = (uint8_t)(kk >> (56 - 8 * i));
    kc[7] = 0x00; kc[6] = (uint8_t)(kc[6] & 0xFC);                        // clear low 10 bits
}

// SIMNOAUTH=1 reverts 0x88 to the faithful "no Ki" stub (98 04) for A/B testing.
static int sim_noauth(void) { static int v = -1; if (v < 0) v = getenv("SIMNOAUTH") ? 1 : 0; return v; }

// Public entry (tests + any future network-attach path): A3/A8 with the baked Ki.
void sim_run_gsm_algorithm(const uint8_t* rand16, uint8_t* sres4, uint8_t* kc8) {
    sim_a3a8(SIM_KI, rand16, sres4, kc8);
}

// Compute the SYNTHETIC response to a fully-assembled T=0 command APDU (header +
// any case-3 data): fills out[0..*np) + *sw1p/*sw2p, with NO FIFO push and no
// asm_len reset (so it can also be used as the shadow reference when comparing
// against a real card via the SIM bridge). Mirrors GSM 11.11.
static void sim_compute_apdu(Mad2* m, uint8_t* out, int* np, int out_cap,
                             uint8_t* sw1p, uint8_t* sw2p) {
    uint8_t ins = m->sim_asm[1];
    uint8_t p1  = m->sim_asm[2];
    uint8_t p2  = m->sim_asm[3];
    uint8_t p3  = m->sim_asm[4];
    const uint8_t* data = m->sim_asm + 5;
    uint16_t data_len   = (uint16_t)(m->sim_asm_len > 5 ? m->sim_asm_len - 5 : 0);
    m->sim_apdus++;
    if (sim_log_on()) {
        printf("[sim] APDU CLA=%02X INS=%02X P1=%02X P2=%02X P3=%02X len=%u :",
               m->sim_asm[0], ins, p1, p2, p3, (unsigned)m->sim_asm_len);
        for (uint16_t i = 0; i < m->sim_asm_len && i < 32; ++i) printf(" %02X", m->sim_asm[i]);
        printf("\n");
    }

    // GSM CLA is 0xA0. Some commands echo the INS as a procedure byte first; the
    // DCT3 driver tolerates the response with or without it, but a clean T=0 reply
    // is: [procedure ACK][outgoing data][SW1 SW2]. We send the data + SW only,
    // which matches what the firmware's byte engine consumes (it counts P3 bytes
    // then the 2 status bytes).
    int n = 0;
    uint8_t sw1 = SW_OK_HI, sw2 = SW_OK_LO;

    switch (ins) {
    case 0xA4: {  // SELECT  (P3=2, data = file id)
        uint16_t fid = data_len >= 2 ? (uint16_t)((data[0] << 8) | data[1]) : 0;
        int is_df = (fid == 0x3F00 || fid == 0x7F20 || fid == 0x7F10);  // only DFs the card provisions
        const SimEf* ef = is_df ? NULL : sim_find_ef(fid);
        if (is_df) {
            m->sim_sel_df = fid; m->sim_sel_file = fid;
            m->sim_gr_len = sim_build_select_resp(m, NULL, fid, 1);
            sw1 = SW_GR_HI; sw2 = m->sim_gr_len;       // 61 xx: GET RESPONSE available
        } else if (ef) {
            m->sim_sel_file = fid;
            m->sim_gr_len = sim_build_select_resp(m, ef, fid, 0);
            sw1 = SW_GR_HI; sw2 = m->sim_gr_len;
        } else {
            // Unknown file: report "file not found" (94 04) so the firmware skips it
            // rather than hanging on a missing EF.
            sw1 = 0x94; sw2 = 0x04;
        }
        break;
    }
    case 0xC0: {  // GET RESPONSE (P3 = length) -> deliver the pending SELECT FCP
        int want = p3 ? p3 : m->sim_gr_len;
        if (want > m->sim_gr_len) want = m->sim_gr_len;
        for (int i = 0; i < want && n < out_cap; ++i) out[n++] = m->sim_gr_buf[i];
        sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        break;
    }
    case 0xB0: {  // READ BINARY (P1:P2 = offset, P3 = length) -> transparent EF
        const SimEf* ef = sim_find_ef(m->sim_sel_file);
        uint16_t off = (uint16_t)((p1 << 8) | p2);
        int want = p3 ? p3 : 0;
        // CHV1 gate: with the PIN enabled but not yet verified, CHV1-protected EFs
        // are refused (98 04 "no CHV value"). ALWays-readable EFs (per GSM 11.11
        // §10.2.x) stay readable so the firmware can complete recognition.
        int alw = 0;
        switch (m->sim_sel_file) {
            case 0x2FE2: case 0x6F05: case 0x6FAE: case 0x6FAD:
            case 0x6F38: case 0x6F46: case 0x6FB7:
                alw = 1; break;
        }
        if (m->sim_pin_enabled && !m->sim_pin_verified && !alw) {
            sw1 = 0x98; sw2 = 0x04; break;
        }
        if (ef && ef->type == 0) {
            for (int i = 0; i < want && n < out_cap; ++i) {
                uint8_t b = (ef->body && off + i < ef->size) ? ef->body[off + i] : 0xFF;
                out[n++] = b;
            }
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else { sw1 = 0x94; sw2 = 0x08; }            // not a transparent EF
        break;
    }
    case 0xB2: {  // READ RECORD (P1 = record#, P2 = mode, P3 = length) -> linear-fixed
        const SimEf* ef = sim_find_ef(m->sim_sel_file);
        int rec = p1 ? p1 - 1 : 0;
        int want = p3 ? p3 : (ef ? ef->rec_len : 0);
        int alw = 0;
        switch (m->sim_sel_file) {
            case 0x2FE2: case 0x6F05: case 0x6FAE: case 0x6FAD:
            case 0x6F38: case 0x6F46: case 0x6FB7:
                alw = 1; break;
        }
        if (m->sim_pin_enabled && !m->sim_pin_verified && !alw) {
            sw1 = 0x98; sw2 = 0x04; break;
        }
        if (ef && ef->type == 1) {
            for (int i = 0; i < want && n < out_cap; ++i) {
                int idx = rec * ef->rec_len + i;
                uint8_t b = (ef->body && rec < ef->recs && i < ef->rec_len)
                            ? ef->body[idx % (ef->rec_len * ef->recs)] : 0xFF;
                out[n++] = b;
            }
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else {
            // Treat a READ RECORD on a transparent (or unknown) EF as blank record.
            for (int i = 0; i < want && n < out_cap; ++i) out[n++] = 0xFF;
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        }
        break;
    }
    case 0xF2: {  // STATUS -> return current DF FCP (like SELECT of the current DF)
        m->sim_gr_len = sim_build_select_resp(m, NULL, m->sim_sel_df ? m->sim_sel_df : 0x3F00, 1);
        int want = p3 ? p3 : m->sim_gr_len;
        if (want > m->sim_gr_len) want = m->sim_gr_len;
        for (int i = 0; i < want && n < out_cap; ++i) out[n++] = m->sim_gr_buf[i];
        sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        break;
    }
    case 0x20: {  // VERIFY CHV (P2 = CHV#: 01 = CHV1/PIN). Data = 8-byte CHV.
        if (p2 != 0x01 || !m->sim_pin_enabled) { sw1 = SW_OK_HI; sw2 = SW_OK_LO; break; }
        if (m->sim_pin_tries == 0) { sw1 = 0x98; sw2 = 0x40; break; }     // CHV1 blocked
        if (sim_chv_match(data, data_len, m->sim_pin)) {
            m->sim_pin_verified = 1; m->sim_pin_tries = 3;                // OK -> verified
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else {
            if (m->sim_pin_tries) m->sim_pin_tries--;
            if (m->sim_pin_tries == 0) { sw1 = 0x98; sw2 = 0x40; }       // now blocked
            else { sw1 = 0x63; sw2 = (uint8_t)(0xC0 | m->sim_pin_tries); } // 63 CX: X left
        }
        break;
    }
    case 0x2C: {  // UNBLOCK CHV (P2 = CHV#). Data = 8-byte PUK + 8-byte new CHV.
        if (p2 != 0x00 && p2 != 0x01) { sw1 = 0x6B; sw2 = 0x00; break; }
        if (m->sim_puk_tries == 0) { sw1 = 0x98; sw2 = 0x40; break; }    // PUK blocked
        if (data_len >= 16 && sim_chv_match(data, data_len, m->sim_puk)) {
            memcpy(m->sim_pin, data + 8, 8);                              // set new PIN
            m->sim_pin_tries = 3; m->sim_pin_verified = 1; m->sim_puk_tries = 10;
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else {
            if (m->sim_puk_tries) m->sim_puk_tries--;
            if (m->sim_puk_tries == 0) { sw1 = 0x98; sw2 = 0x40; }
            else { sw1 = 0x63; sw2 = (uint8_t)(0xC0 | m->sim_puk_tries); }
        }
        break;
    }
    case 0x24: {  // CHANGE CHV (P2). Data = 8-byte old CHV + 8-byte new CHV.
        if (p2 != 0x01 || !m->sim_pin_enabled) { sw1 = SW_OK_HI; sw2 = SW_OK_LO; break; }
        if (m->sim_pin_tries == 0) { sw1 = 0x98; sw2 = 0x40; break; }
        if (data_len >= 16 && sim_chv_match(data, data_len, m->sim_pin)) {
            memcpy(m->sim_pin, data + 8, 8);
            m->sim_pin_tries = 3; m->sim_pin_verified = 1;
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else {
            if (m->sim_pin_tries) m->sim_pin_tries--;
            if (m->sim_pin_tries == 0) { sw1 = 0x98; sw2 = 0x40; }
            else { sw1 = 0x63; sw2 = (uint8_t)(0xC0 | m->sim_pin_tries); }
        }
        break;
    }
    case 0x26: {  // DISABLE CHV. Data = 8-byte CHV (must match to disable).
        if (sim_chv_match(data, data_len, m->sim_pin)) {
            m->sim_pin_enabled = 0; m->sim_pin_verified = 1; m->sim_pin_tries = 3;
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else {
            if (m->sim_pin_tries) m->sim_pin_tries--;
            if (m->sim_pin_tries == 0) { sw1 = 0x98; sw2 = 0x40; }
            else { sw1 = 0x63; sw2 = (uint8_t)(0xC0 | m->sim_pin_tries); }
        }
        break;
    }
    case 0x28: {  // ENABLE CHV. Data = 8-byte CHV (must match to enable).
        if (sim_chv_match(data, data_len, m->sim_pin)) {
            m->sim_pin_enabled = 1; m->sim_pin_verified = 1; m->sim_pin_tries = 3;
            sw1 = SW_OK_HI; sw2 = SW_OK_LO;
        } else {
            if (m->sim_pin_tries) m->sim_pin_tries--;
            if (m->sim_pin_tries == 0) { sw1 = 0x98; sw2 = 0x40; }
            else { sw1 = 0x63; sw2 = (uint8_t)(0xC0 | m->sim_pin_tries); }
        }
        break;
    }
    case 0x10:    // TERMINAL PROFILE (SAT) — accept; no proactive command pending
    case 0x14:    // TERMINAL RESPONSE (SAT) — accept
    case 0xC2:    // ENVELOPE (SAT) — accept (synthetic has no proactive session)
    case 0xA2:    // SEEK — accept (synthetic: no match data returned)
    case 0x32:    // INCREASE
    case 0xDC:    // UPDATE RECORD
    case 0xD6:    // UPDATE BINARY
        sw1 = SW_OK_HI; sw2 = SW_OK_LO;               // accept writes (no persistence)
        break;
    case 0x88:    // RUN GSM ALGORITHM (A3/A8): 16-byte RAND in -> SRES(4)+Kc(8) out.
        // The synthetic operator algorithm computes a self-consistent SRES/Kc from our
        // baked Ki. SIMNOAUTH=1 restores the old "no Ki" stub (98 04).
        if (sim_noauth()) { sw1 = 0x98; sw2 = 0x04; break; }
        if (data_len >= 16) {
            uint8_t sres[4], kc[8];
            sim_a3a8(SIM_KI, data, sres, kc);
            memcpy(m->sim_gr_buf, sres, 4);
            memcpy(m->sim_gr_buf + 4, kc, 8);
            m->sim_gr_len = 12;
            sw1 = SW_GR_HI; sw2 = 0x0C;                // 9F 0C: 12 bytes (SRES+Kc) via GET RESPONSE
        } else {
            sw1 = 0x67; sw2 = 0x00;                     // wrong length (RAND must be 16 bytes)
        }
        break;
    default:
        sw1 = 0x6D; sw2 = 0x00;                        // INS not supported
        break;
    }

    if (sim_log_on()) {
        printf("[sim]   -> resp(%d):", n);
        for (int i = 0; i < n && i < 32; ++i) printf(" %02X", out[i]);
        printf("  SW=%02X%02X\n", sw1, sw2);
    }
    (void)p1; (void)p2;
    *np = n; *sw1p = sw1; *sw2p = sw2;
}

// Push a computed T=0 response (data + SW) into the RX FIFO and signal the firmware.
// T=0 case-2 (data-out: STATUS/READ/GET RESPONSE) framing: the card echoes INS as a
// procedure byte BEFORE the outgoing data -> [INS][data][SW1 SW2]; the firmware's byte
// SM reads that ACK first. We previously pushed [data][SW] only, so the firmware read
// data[0] where it expected the INS ACK -> misframe -> STATUS re-poll loop (the
// SIM-recognition stall). (The case-3 data-IN procedure byte is emitted in sim_run_apdu;
// case-1/case-3 responses carry no outgoing data so n==0 -> unchanged.) SIMT0NOPROC=1 reverts.
static void sim_push_resp(Mad2* m, uint8_t ins, const uint8_t* out, int n,
                          uint8_t sw1, uint8_t sw2) {
    static int t0noproc = -1; if (t0noproc < 0) t0noproc = getenv("SIMT0NOPROC") ? 1 : 0;
    if (!t0noproc && n > 0) sim_rx_push(m, ins);   // procedure byte (INS echo) before data-out
    for (int i = 0; i < n; ++i) sim_rx_push(m, out[i]);
    sim_rx_push(m, sw1);
    sim_rx_push(m, sw2);
    m->sim_asm_len = 0;
    sim_rx_signal(m);
}

// Synthetic SIM: compute + push (the non-bridge default path).
static void sim_process_apdu(Mad2* m) {
    uint8_t out[64]; int n = 0; uint8_t sw1, sw2;
    sim_compute_apdu(m, out, &n, (int)sizeof out, &sw1, &sw2);
    sim_push_resp(m, m->sim_asm[1], out, n, sw1, sw2);
}

#ifndef __EMSCRIPTEN__
// ---- SIM bridge: real-card shadow comparison -------------------------------
// When SIMBRIDGE=<dev> is set, every assembled APDU is run against BOTH the
// synthetic SIM and a real card on the ESP32, a MATCH/DIFF line is logged, and
// (by default) the REAL response is fed back so the boot follows the authentic
// path. SIMBRIDGE_FEED=synth feeds the synthetic response instead (still logging
// the real diff) — useful for testing synthetic fixes against live real answers.
// Goal: drive the synthetic SIM to faithfulness by eliminating the DIFFs.
static int g_simbridge = -1;          // -1 uninit, 0 off, 1 active
static int g_simbridge_feed_synth = 0;
static uint8_t g_real_atr[33];        // real card ATR (for complete pass-through)
static int     g_real_atr_len = 0;

// Cold-reset the real card and cache its ATR in g_real_atr. Returns 0 on success.
// On failure the cached ATR is CLEARED — a card pulled from the bridge mid-session
// must read as ABSENT at the next warm reset, not replay its stale ATR.
static int simbridge_refresh_atr(void) {
    uint8_t atr[33]; int al = (int)sizeof atr;
    if (sim_bridge_activate(atr, &al) == 0 && al >= 2) {
        memcpy(g_real_atr, atr, (size_t)al);
        g_real_atr_len = al;
        return 0;
    }
    g_real_atr_len = 0;
    return -1;
}

// Faithful card-absent semantics (2026-06-11): with the bridge ACTIVE but the card
// MUTE (no ATR), the firmware must see an EMPTY TRAY ("Insert SIM card") — not the
// synthetic SIM. The old behavior silently fell back to synth on a mute card, so
// "no card in the ESP32" booted past the insert-SIM screen (misleading at the
// bench). SIMBRIDGE_FALLBACK=synth restores the old fallback (compare-tool mode).
static int simbridge_mute_means_absent(void) {
    static int v = -1;
    if (v < 0) { const char* s = getenv("SIMBRIDGE_FALLBACK");
                 v = (s && !strcmp(s, "synth")) ? 0 : 1; }
    return v;
}

static void simbridge_init_once(void) {
    if (g_simbridge >= 0) return;
    const char* dev = getenv("SIMBRIDGE");
    if (!dev || !*dev) { g_simbridge = 0; return; }
    if (sim_bridge_open(dev) != 0) {
        fprintf(stderr, "[simcmp] bridge open failed on %s; using synthetic SIM\n", dev);
        g_simbridge = 0; return;       // fall back to synthetic
    }
    const char* feed = getenv("SIMBRIDGE_FEED");
    g_simbridge_feed_synth = (feed && !strcmp(feed, "synth"));
    if (simbridge_refresh_atr() == 0) {
        printf("[simcmp] real card ATR:");
        for (int i = 0; i < g_real_atr_len; ++i) printf(" %02X", g_real_atr[i]);
        printf("  (passed through to firmware)\n");
    } else {
        fprintf(stderr, "[simcmp] warning: no ATR from real card (mute/absent)\n");
    }
    printf("[simcmp] bridge active on %s; feeding %s to firmware\n",
           dev, g_simbridge_feed_synth ? "SYNTH" : "REAL");
    g_simbridge = 1;
}

static void sim_compare_log(const uint8_t* apdu, int alen,
                            const uint8_t* so, int sn, uint8_t ssw1, uint8_t ssw2,
                            int real_ok, const uint8_t* ro, int rn,
                            uint8_t rsw1, uint8_t rsw2) {
    int match = real_ok && sn == rn && ssw1 == rsw1 && ssw2 == rsw2
                && (sn == 0 || memcmp(so, ro, (size_t)sn) == 0);
    printf("[simcmp] APDU");
    for (int i = 0; i < alen && i < 16; ++i) printf(" %02X", apdu[i]);
    printf(" | synth:");
    for (int i = 0; i < sn && i < 32; ++i) printf(" %02X", so[i]);
    printf(" SW%02X%02X | real:", ssw1, ssw2);
    if (!real_ok) printf(" <bridge-fail>");
    else { for (int i = 0; i < rn && i < 32; ++i) printf(" %02X", ro[i]);
           printf(" SW%02X%02X", rsw1, rsw2); }
    printf(" | %s\n", match ? "MATCH" : "DIFF");
}

// Re-cold-reset the real card so its selection state tracks a firmware warm reset
// (and refresh the cached ATR for pass-through).
static void simbridge_reactivate(void) {
    if (g_simbridge != 1) return;
    (void)simbridge_refresh_atr();
}

// Per-file feed override for bisecting which EF gates SIM acceptance: while feeding
// REAL globally, SIMBRIDGE_SYNTH_FILES="6F38,6F3E" forces those files to the synthetic
// response (and SIMBRIDGE_REAL_FILES forces real while feeding synth globally). FIDs are
// 4-hex-digit, comma/space separated. The decision uses the synth-tracked current file.
static int simbridge_fid_in_env(const char* envname, uint16_t fid) {
    const char* s = getenv(envname);
    if (!s || !*s) return 0;
    while (*s) {
        if (*s == ',' || *s == ' ') { s++; continue; }
        char* end; unsigned long v = strtoul(s, &end, 16);
        if (end == s) { s++; continue; }
        if ((uint16_t)v == fid) return 1;
        s = end;
    }
    return 0;
}

// MITM IMSI rewrite (proof tool): when SIMBRIDGE_IMSI=<digits> is set, rewrite the
// MCC (and optionally more) of the REAL card's EF_IMSI READ BINARY response on the way
// to the firmware — same physical card, only the locked field changed. Proves a
// SIMLOCK/IMSI gate: if the card is rejected as-is but accepted after the MCC is
// rewritten to a passing value, the IMSI is the gate (and the DSP is not involved).
// Digits are the full IMSI (<=15); we re-pack TBCD into the response, preserving the
// real length byte and parity nibble layout. Only touches a READ BINARY (B0) response
// while EF_IMSI (6F07) is selected and the data looks like a TBCD IMSI (len byte 0x08).
static void simbridge_rewrite_imsi(Mad2* m, uint8_t ins, uint8_t* rout, int rn) {
    if (m->sim_sel_file != 0x6F07 || ins != 0xB0 || rn < 9) return;
    const char* want = getenv("SIMBRIDGE_IMSI");
    if (!want || !*want) return;
    // Collect up to 15 decimal digits.
    char d[16]; int nd = 0;
    for (const char* p = want; *p && nd < 15; ++p) if (*p >= '0' && *p <= '9') d[nd++] = *p;
    if (nd < 5) return;                       // need at least MCC+MNC to be meaningful
    // EF_IMSI byte0 = length of the IMSI value field (TBCD bytes that follow). Keep the
    // real length; re-pack our digits TBCD with the GSM parity nibble in byte1 low.
    int valbytes = rout[0];                   // typically 8
    uint8_t old1 = rout[1];
    uint8_t packed[8]; for (int i = 0; i < 8; ++i) packed[i] = 0xFF;
    // parity nibble (low of byte1): bit0 = odd(1)/even(0) digit count; preserve type bits.
    uint8_t parity = (uint8_t)((old1 & 0x0E) | (nd & 1));
    packed[0] = (uint8_t)((d[0] - '0') << 4 | parity);
    for (int i = 1; i < nd; ++i) {
        int bi = (i + 1) / 2, hi = (i + 1) & 1;   // digit i lands in byte (i+1)/2
        uint8_t nib = (uint8_t)(d[i] - '0');
        if (hi) packed[bi] = (uint8_t)((packed[bi] & 0x0F) | (nib << 4));
        else    packed[bi] = (uint8_t)((packed[bi] & 0xF0) | nib);
    }
    if (valbytes > 8) valbytes = 8;
    for (int i = 0; i < valbytes && 1 + i < rn; ++i) rout[1 + i] = packed[i];
    static int announced = 0;
    if (!announced) { announced = 1;
        fprintf(stderr, "[simcmp] IMSI REWRITE active: real EF_IMSI -> %s (MITM proof; "
                        "same card, MCC/MNC changed)\n", d); }
}

static void sim_dispatch_apdu(Mad2* m) {
    simbridge_init_once();
    if (g_simbridge != 1) { sim_process_apdu(m); return; }

    // Synthetic reference (advances synth state: selected file, GET RESPONSE buf, ...).
    uint8_t sout[64]; int sn = 0; uint8_t ssw1, ssw2;
    sim_compute_apdu(m, sout, &sn, (int)sizeof sout, &ssw1, &ssw2);

    // Real card via the bridge.
    uint8_t ins = m->sim_asm[1];
    uint8_t hdr[5]; memcpy(hdr, m->sim_asm, 5);
    int dir = sim_ins_has_indata(ins) ? SIMB_DIR_IN : (hdr[4] ? SIMB_DIR_OUT : SIMB_DIR_NONE);
    const uint8_t* din = m->sim_asm + 5;
    int dinlen = m->sim_asm_len > 5 ? (int)m->sim_asm_len - 5 : 0;
    uint8_t rout[256]; int rn = (int)sizeof rout; uint8_t rsw1 = 0, rsw2 = 0;
    int real_ok = (sim_bridge_apdu(hdr, dir, din, dinlen, rout, &rn, &rsw1, &rsw2) == 0);
    if (!real_ok) rn = 0;

    // MITM IMSI rewrite (proof tool) — patch before logging + feeding so the trace
    // shows exactly what the firmware receives.
    if (real_ok) simbridge_rewrite_imsi(m, ins, rout, rn);

    sim_compare_log(m->sim_asm, (int)m->sim_asm_len, sout, sn, ssw1, ssw2,
                    real_ok, rout, rn, rsw1, rsw2);

    // Effective feed: global mode, overridden per current file for bisection.
    uint16_t curfid = m->sim_sel_file;
    int feed_synth = g_simbridge_feed_synth;
    if (simbridge_fid_in_env("SIMBRIDGE_SYNTH_FILES", curfid)) feed_synth = 1;
    if (simbridge_fid_in_env("SIMBRIDGE_REAL_FILES",  curfid)) feed_synth = 0;

    if (!feed_synth && real_ok)
        sim_push_resp(m, ins, rout, rn, rsw1, rsw2);   // feed real (authentic path)
    else
        sim_push_resp(m, ins, sout, sn, ssw1, ssw2);   // feed synth (or real failed)
}
#else
#define sim_dispatch_apdu sim_process_apdu
#define simbridge_reactivate() ((void)0)
#endif

// Handle a TX flush (SIMI_TXD_FL -> 0): the firmware has queued a chunk of command
// bytes (sim_tx) and flushed the FIFO. T=0 over the DCT3 SIM UART is chunked: the
// firmware sends the 5-byte header, then (for case-3 commands like SELECT/VERIFY) the
// P3 data bytes one at a time, and after EACH chunk it expects a TX-empty interrupt
// (bit4 -> sim_advance -> SM "status 7" = ACK/continue). It does NOT want a procedure
// byte echoed during the send (that would be classified as a response data byte and
// abort the command). So we: (1) accumulate the chunk into the APDU assembly buffer,
// (2) raise TX-empty so the firmware advances, (3) once the whole command (+ case-3
// data) is in, process it and push the real response (data + SW) to the RX FIFO.
static void sim_run_apdu(Mad2* m) {
    // Append this chunk to the assembly buffer.
    for (uint16_t i = 0; i < m->sim_tx_len; ++i)
        if (m->sim_asm_len < sizeof m->sim_asm) m->sim_asm[m->sim_asm_len++] = m->sim_tx[i];
    m->sim_tx_len = 0;
    // TX FIFO drained -> raise the TX-empty interrupt so sim_advance posts status 7
    // (the firmware then sends the next chunk, or proceeds to read the response).
    sim_txempty_signal(m);

    // PPS (Protocol & Parameter Selection), ISO-7816-3 §9. Sent right after an ATR
    // that advertises non-default speed (TA1) — e.g. the real card's TA1=0x96. It is
    // NOT a T=0 APDU: the card confirms by ECHOING the PPS frame. Detect PPSS=0xFF,
    // wait for the full frame (PPSS, PPS0, optional PPS1-3 per PPS0 bits 4-6, PCK),
    // then echo it back. We confirm the proposed params as-is (the emulated UART link
    // and the fixed-rate bridge both run default timing regardless). Without this the
    // negotiation fails and the firmware rejects the card.
    if (m->sim_asm_len >= 1 && m->sim_asm[0] == 0xFF) {
        if (m->sim_asm_len < 2) return;                       // need PPS0
        uint8_t pps0 = m->sim_asm[1];
        int need = 2 + ((pps0 & 0x10) ? 1 : 0) + ((pps0 & 0x20) ? 1 : 0)
                     + ((pps0 & 0x40) ? 1 : 0) + 1;           // +PCK
        if (m->sim_asm_len < need) return;                    // wait for full PPS frame
        if (sim_log_on()) {
            printf("[sim] PPS request:");
            for (int i = 0; i < need; ++i) printf(" %02X", m->sim_asm[i]);
            printf(" -> echo (confirm)\n");
        }
        for (int i = 0; i < need; ++i) sim_rx_push(m, m->sim_asm[i]);  // card echoes PPS
        m->sim_asm_len = 0;
        sim_rx_signal(m);
        return;
    }

    // Decide whether the command is complete. Need at least the 5-byte header.
    if (m->sim_asm_len < 5) return;
    uint8_t ins = m->sim_asm[1];
    uint8_t p3  = m->sim_asm[4];
    if (sim_ins_has_indata(ins)) {
        // case-3 (SELECT/VERIFY/...): the card requests the P3 data with a T=0
        // procedure byte (= INS, "ACK send the next byte"); the firmware's byte SM
        // classifies that as SM status 11, which is what makes it transmit the next
        // data byte. So after the header (and after each non-final data byte) we send
        // ONE procedure byte. When all P3 data bytes are in, we send the response (SW).
        uint16_t need = (uint16_t)(5 + p3);
        if (m->sim_asm_len < need) {
            sim_rx_push(m, ins);               // procedure byte = ACK -> SM status 11
            sim_rx_signal(m);
            return;
        }
        sim_dispatch_apdu(m);                  // all data received -> SW response
    } else {
        // case-2 / case-1 (READ, GET RESPONSE, STATUS...): no data phase, the 5-byte
        // header is the whole command. The card now returns data + SW.
        sim_dispatch_apdu(m);
    }
}

// Deliver the ATR after a card activate (SIMI_CTRL bit7). The DCT3 driver auto-
// detects the convention from the first byte: direct = 0x3B, inverse = 0x03 (it
// then sets SIMI_CLK_CTRL |= 0x0C and remaps). We emit a short direct-convention
// T=0 ATR: TS=0x3B, T0=0x00 (no interface bytes, no historical bytes). A minimal
// "3B 00" is a valid T=0 ATR; we add a couple of historical bytes to be safe.
static void sim_deliver_atr(Mad2* m) {
    if (!m->sim_present) return;
    // A common, widely-compatible GSM (phase 2) T=0 ATR, direct convention:
    //   TS=3B  T0=65 (TA1+TB1+TC1 absent, TD1 absent? actually 0x65: Y1=0110 ->
    //   TB1+TC1 present, 5 historical bytes). To keep the firmware's interface-byte
    //   parser happy with a clean default-timing T=0 card, emit:
    //     TS=3B, T0=0x60 (Y1=0110: TB1,TC1 present; 0 historical), TB1=0x00, TC1=0x00
    //   -> default Fi/Di, T=0, no historical bytes, no TCK (T=0 only). This is a
    //   minimal but fully-specified ATR (no ambiguous default-fill the firmware must
    //   guess), which the DCT3 driver accepts. (Inverse-convention 0x03 path is
    //   handled by the firmware via SIMI_CLK_CTRL|=0x0C; we send direct 0x3B.)
    static const uint8_t CANNED_ATR[] = { 0x3B, 0x60, 0x00, 0x00 };
    const uint8_t* atr = CANNED_ATR;
    int atrlen = (int)sizeof CANNED_ATR;
#ifndef __EMSCRIPTEN__
    // SIM bridge: complete pass-through. Cold-reset the real card (syncs its selection
    // state to this warm reset) and feed the firmware the REAL card's ATR instead of
    // our canned one — a true end-to-end test of whether the firmware accepts the
    // physical card. (mad2 delivers ATR bytes via the RX FIFO, so any non-default
    // TA1 baud the real ATR advertises has no effect on the emulated UART link.)
    simbridge_init_once();
    simbridge_reactivate();
    if (g_simbridge == 1 && g_real_atr_len >= 2) { atr = g_real_atr; atrlen = g_real_atr_len; }
    else if (g_simbridge == 1 && simbridge_mute_means_absent()) {
        // Bridge up, card mute: faithful empty tray — deliver NO ATR so the firmware
        // shows "Insert SIM card" (same presentation as SIMABSENT). The old synth
        // fallback is available via SIMBRIDGE_FALLBACK=synth.
        static int warned = 0;
        if (!warned) { warned = 1;
            fprintf(stderr, "[simcmp] card mute -> presenting SIM ABSENT "
                            "(SIMBRIDGE_FALLBACK=synth for the old synth fallback)\n"); }
        return;
    }
#endif
    if (sim_log_on()) printf("[sim] deliver ATR (%d bytes)\n", atrlen);
    for (int i = 0; i < atrlen; ++i) sim_rx_push(m, atr[i]);
    m->sim_reset_count++;
    m->sim_asm_len = 0; m->sim_t0_state = 0;            // abandon any in-flight APDU
    m->sim_sel_df = 0x3F00; m->sim_sel_file = 0x3F00;  // reset selection to MF
    sim_rx_signal(m);
}

// SIMI register read.
uint32_t sim_read(Mad2* m, uint8_t off) {
    switch (off) {
    case IO_SIM_RXD: {            // pop one received byte
        uint8_t b = sim_rx_pop(m);
        if (sim_rx_count(m) == 0) m->sim_uart_int &= (uint8_t)~SIM_INT_RXRDY;
        return b;
    }
    case IO_SIM_UART_INT: return m->sim_uart_int;
    case IO_SIM_CTRL:
        // SIMI_CTRL read mixes the firmware's last write with hardware STATUS bits.
        // bit6 (0x40) = card active/clock-running (the firmware's transmit gate
        // 0x2D46B8 does `lsr #7` -> carry=bit6, requiring it set before sending an
        // APDU). The UART asserts it once a present card is powered (after the bit7
        // activate write); we set it whenever a card is present and was activated.
        return (uint8_t)((m->sim_ctrl & ~0x40u)
                         | ((m->sim_present && (m->sim_ctrl & 0x80)) ? 0x40u : 0u));
    case IO_SIM_CLK_CTRL: return m->sim_clk_ctrl;
    case IO_SIM_TXD_LWM:  return m->sim_txd_lwm;
    case IO_SIM_RXD_QUE:  return (uint8_t)sim_rx_count(m);   // bytes available to read
    case IO_SIM_RXD_FL:   return m->sim_rxd_fl;
    case IO_SIM_TXD_FL:   return m->sim_txd_fl;
    case IO_SIM_TXD_QUE:  return 0x00;       // TX FIFO bytes used = 0 (always empty:
                                             // sim_transmit computes free = 15 - QUE,
                                             // so 0 => full 15-byte room, drains instantly)
    case IO_SIM_TXD:      return 0x00;
    default:              return 0x00;
    }
}

// SIMI register write.
void sim_write(Mad2* m, uint8_t off, uint8_t v) {
    switch (off) {
    case IO_SIM_TXD:                          // firmware queued a TX byte (APDU)
        if (m->sim_tx_len < sizeof m->sim_tx) m->sim_tx[m->sim_tx_len++] = v;
        break;
    case IO_SIM_UART_INT:                     // write-1-to-clear the pending int lines
        m->sim_uart_int &= (uint8_t)~v;
        // FIQ6 stays asserted only while a SIM-UART int line is still pending (RX-ready
        // or TX-empty). The ISR write-backs clear the serviced lines.
        if ((m->sim_uart_int & (SIM_INT_RXRDY | SIM_INT_TXEMPTY)) == 0)
            mad2_ack_fiq(m, 6);
        break;
    case IO_SIM_CTRL: {
        uint8_t prev = m->sim_ctrl;
        m->sim_ctrl = v;
        // Activate / warm-reset edge: bit7 (0x80) rising = clock on, card emits ATR.
        // (sim_reset writes bit0 then bit7; we deliver the ATR on the bit7 edge.)
        if ((v & 0x80) && !(prev & 0x80)) {
            m->sim_rx_head = m->sim_rx_tail = 0;   // flush stale RX
            m->sim_tx_len = 0; m->sim_asm_len = 0; // abandon any partial APDU
            m->sim_atr_pending = 1;                // deliver shortly (paced by the tick)
        }
        break;
    }
    case IO_SIM_CLK_CTRL: m->sim_clk_ctrl = v; break;
    case IO_SIM_TXD_LWM:  m->sim_txd_lwm = v;  break;
    case IO_SIM_RXD_FL:   m->sim_rxd_fl = v;   break;
    case IO_SIM_TXD_FL:
        // SIMI_TXD_FL: the firmware sets 0x04 to begin a send and 0x00 to flush. On
        // the flush (->0) hand the accumulated TX chunk to the T=0 APDU engine.
        if (v == 0x00 && m->sim_txd_fl != 0x00 && m->sim_tx_len > 0) sim_run_apdu(m);
        m->sim_txd_fl = v;
        break;
    case IO_SIM_RXD: case IO_SIM_RXD_QUE: case IO_SIM_TXD_QUE:
    default: break;            // read-only / no-effect-on-write SIMI regs
    }
}

// Per-tick SIM housekeeping: deliver a pending ATR, re-arm FIQ6 while RX bytes wait
// (the firmware masks/clears FIQ6 each ISR; a level-style re-raise keeps draining),
// edge-detect card insert/remove for FIQ7, and (when SIMWWT enabled) raise the
// SIMI_UART_INT bit 5 / FIQ6 WWT-timeout when the SIM line has been idle longer
// than the configured threshold.
void sim_tick(Mad2* m) {
    if (m->sim_atr_pending) { m->sim_atr_pending = 0; sim_deliver_atr(m); }
    // Card insert/remove edge -> FIQ7 (card-detect). The firmware re-probes on this.
    if (m->sim_present != m->sim_present_seen) {
        m->sim_present_seen = m->sim_present;
        mad2_raise_fiq(m, 7);          // FIQ7 = SIM card detect
    }
    // NOTE: an HLE band-aid that clamped [0x11FD08] to suppress the SIM-recognition
    // timeout was tried + reverted. The byte is OVERLOADED by firmware: the SIM SM
    // @0x2AF238 uses it as a 0..185 retry counter, but TASK_5_UI's dispatcher
    // @0x2E4708 reads the same byte as an index into DISPATCHERS_TABLE 0x321EA0 (8B
    // stride). Any host-side clamp misdirects the dispatcher when task 5 runs
    // between SIM-SM ticks — slot 0 = NULL → `bx 0` → PC=0x4 crash; non-zero slots
    // route msgs to wrong handlers and SIM APDU exchange stops. The proper fix
    // requires modeling the upstream signal that resets the counter in firmware-
    // native code (a message from the recognition-complete msg handler). See
    // docs/handoff-sim-dispatcher-decoded.md cont(2)/(3) for the kill-chain decode.
    // Keep FIQ6 asserted while unread RX bytes remain and the line is unmasked-ready.
    if (sim_rx_count(m) > 0 && (m->sim_uart_int & SIM_INT_RXRDY))
        mad2_raise_fiq(m, 6);
    // WWT timeout (SIMI_UART_INT bit 5). Real ISO-7816 WWT only ticks against an ACTIVE
    // card running the T=0 protocol — before activation (SIMI_CTRL bit7) and ATR delivery
    // the line is electrically dead and no WWT exists. We gate on:
    //  - SIM is present (the firmware wouldn't enable WWT for an empty slot)
    //  - SIM is activated (sim_ctrl bit7 set; firmware has powered the card)
    //  - ATR has been delivered + at least one APDU has flowed (proves the driver finished
    //    the activation handshake and is in steady-state byte exchange — only THEN does
    //    WWT firing match firmware expectations; spamming it during early init is what
    //    causes a FIQ6 storm that locks the CPU in FIQ mode before the SIM driver runs)
    //  - line is idle (no RX queued, no TX assembled, no in-flight APDU, no pending ATR)
    if (m->sim_wwt_threshold_cyc > 0 && m->sim_present && (m->sim_ctrl & 0x80)
        && m->sim_reset_count > 0 && m->sim_apdus > 0
        && sim_rx_count(m) == 0 && m->sim_tx_len == 0
        && m->sim_asm_len == 0 && !m->sim_atr_pending) {
        if (m->sim_wwt_last_active_cyc == 0) {
            m->sim_wwt_last_active_cyc = m->rtc_mono;     // first idle since SIM became active
        } else if (m->rtc_mono - m->sim_wwt_last_active_cyc >= m->sim_wwt_threshold_cyc) {
            m->sim_uart_int |= 0x20u;             // bit 5 = WWT timeout
            mad2_raise_fiq(m, 6);         // FIQ6 = SIM-UART
            m->sim_wwt_last_active_cyc = m->rtc_mono;
        }
    } else {
        // Pre-active or in-flight: reset the idle anchor; next steady-idle restarts the WWT clock.
        m->sim_wwt_last_active_cyc = m->rtc_mono;
    }
}
