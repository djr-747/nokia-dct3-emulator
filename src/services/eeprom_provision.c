// eeprom_provision — external-EEPROM factory-provisioning service. See eeprom_provision.h.
//
// This owns ALL load-time provisioning of the external 24Cxx EEPROM so the MAD2 load path
// (ext_eeprom.c) stays pure transport. The checksum/latch fix-ups were moved here verbatim
// from ext_eeprom.c (byte-identical behaviour); the FAID identity provisioning is new.
//
// FAID / factory-identity: reverse-engineered and first solved for the 3210 by Gareth Davidson
// (bitplane), github.com/bitplane/nokia-dct3-re — +
// tools/make_eeprom_profile.py. The firmware transforms, offsets, checksum algorithms and the
// IMEI check-digit routine are his; this is a C port into the emulator's load path, driven by a
// per-model ModelProfile.eeprom_faid descriptor so any external-EEPROM model can opt in.

#include "mad2/mad2.h"
#include "models/model.h"
#include "services/eeprom_provision.h"
#include "services/dct3_calcul.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- FAID identity provisioning ------------------------------------------------------------

// Firmware 0x265244 (3210 v6.00): the standard Luhn check digit over the first 14 IMEI digits.
static unsigned faid_imei_check_digit(const char* first14) {
    unsigned total = 0;
    for (int i = 0; i < 14; ++i) {
        unsigned d = (unsigned)(first14[i] - '0');
        unsigned v = (i & 1) ? d * 2u : d;
        total += v / 10u + v % 10u;
    }
    return (10u - (total % 10u)) % 10u;
}

// The security-level byte the firmware installs itself when the stored setting reads
// "not configured" (5110 v5.30 0x270318: `mov r0,#17` -> level cell -> recompute the
// stored checksum -> write it back as record 0x070A). The identity checksum has to be
// summed against the SAME value or the validator rejects it on the next boot.
#define FAID_SECLEVEL_DEFAULT 0x11u

// Expand the 14 BCD identity digits at identity_off into the 15-character ASCII IMEI the
// firmware hashes, and return its 16-bit byte sum. This mirrors firmware 0x258798 selector
// 3 (BCD -> ASCII digits, then the Luhn check digit at index 14) feeding 0x28019C (byte 15
// zeroed) and the plain 16-bit sum 0x283194 — the 5110 v5.30 chain; every ROM-4 build has
// the same three functions. Returns 0 when no identity slot is declared.
static unsigned faid_identity_sum(const struct Mad2* m, const EepromFaid* f) {
    if (!f->identity_off) return 0;
    const uint8_t* id = &m->i2c_eeprom[f->identity_off];
    char imei[16];
    for (int i = 0; i < 7; ++i) {
        imei[i * 2]     = (char)('0' + (id[i] >> 4));
        imei[i * 2 + 1] = (char)('0' + (id[i] & 0x0F));
    }
    imei[14] = (char)('0' + faid_imei_check_digit(imei));
    imei[15] = '\0';
    unsigned s = 0;
    for (int i = 0; i < 15; ++i) s += (uint8_t)imei[i];
    return s & 0xFFFFu;
}

// Re-derive the stored security-settings checksum (secstate bytes 6-7, record 0x070A) from
// the identity ALREADY in the EEPROM, for models whose blob carries a real factory identity
// and only needs the security block made self-consistent again.
//
// The gate this feeds is the security-settings validator (5110 v5.30 0x28CC8C, 6110 v5.48
// 0x29FBAC — the same shape in every ROM-4 build):
//     (sum16(identity ASCII) + [security-level byte]) & 0xFFFF  ==  stored BE16
// On a mismatch the firmware treats the security block as foreign and demands the security
// code at power-up. The NokiX virgin blobs are repair templates: their stored checksum was
// baked for a DIFFERENT handset's identity (6110: stored 0x2449 vs derived 0x030D), so a
// factory-fresh boot stops on "Security code" no matter what the rest of the EEPROM says.
static void faid_refresh_security_cksum(struct Mad2* m, const EepromFaid* f) {
    unsigned s = (faid_identity_sum(m, f) + FAID_SECLEVEL_DEFAULT) & 0xFFFFu;
    m->i2c_eeprom[f->secstate_off + 6] = (uint8_t)(s >> 8);
    m->i2c_eeprom[f->secstate_off + 7] = (uint8_t)(s & 0xFF);
    if (getenv("I2CLOG"))
        printf("[faid] security-settings checksum @0x%X = 0x%04X (identity sum + level 0x%02X)\n",
               f->secstate_off + 6, s, FAID_SECLEVEL_DEFAULT);
}

// Provision the identity + derived security state (bitplane's provision_security_identity):
//   EEPROM[identity_off] = 14 IMEI digits, high-nibble-first BCD (+ 1 zero byte)
//   EEPROM[seccode_off]  = 5-digit security code, BCD
//   EEPROM[secstate_off] = 4-byte "encrypted" code, 2 zero, BE16 identity checksum
// The stored identity checksum (secstate bytes 6-7) is sum16(identity_ascii(15)) + the
// security-level byte 0x11 — the value the validator above recomputes and compares.
static void faid_provision_identity(struct Mad2* m, const EepromFaid* f) {
    uint8_t* ee = m->i2c_eeprom;
    const char* prefix = f->imei_prefix;
    const char* code = f->security_code ? f->security_code : "12345";

    // 15-digit IMEI as ASCII (14 supplied + computed check digit).
    char imei[16];
    for (int i = 0; i < 14; ++i) imei[i] = prefix[i];
    imei[14] = (char)('0' + faid_imei_check_digit(prefix));
    imei[15] = '\0';

    // identity: 7 BCD bytes (14 digits, high nibble first) + 1 zero byte.
    for (int i = 0; i < 7; ++i)
        ee[f->identity_off + i] = (uint8_t)(((prefix[i * 2] - '0') << 4) | (prefix[i * 2 + 1] - '0'));
    ee[f->identity_off + 7] = 0;

    // security code: BCD pairs {d0d1, d2d3, d4<<4}.
    uint8_t code_bcd[3] = {
        (uint8_t)(((code[0] - '0') << 4) | (code[1] - '0')),
        (uint8_t)(((code[2] - '0') << 4) | (code[3] - '0')),
        (uint8_t)((code[4] - '0') << 4),
    };
    if (f->seccode_off) {
        ee[f->seccode_off + 0] = code_bcd[0];
        ee[f->seccode_off + 1] = code_bcd[1];
        ee[f->seccode_off + 2] = code_bcd[2];
    }

    if (f->secstate_off) {
        // encrypted[i] = packed_code[i] ^ imei_ascii[11+i] ^ xor[i], xor = {00,FF,FF,FF}.
        const uint8_t packed_code[4] = { 5, code_bcd[0], code_bcd[1], code_bcd[2] };
        static const uint8_t xor4[4] = { 0x00, 0xFF, 0xFF, 0xFF };
        for (int i = 0; i < 4; ++i)
            ee[f->secstate_off + i] =
                (uint8_t)(packed_code[i] ^ (uint8_t)imei[11 + i] ^ xor4[i]);
        ee[f->secstate_off + 4] = 0;
        ee[f->secstate_off + 5] = 0;
        // identity checksum = sum16(imei_ascii[0..14]) + the security-level byte, stored BE16.
        unsigned s = 0;
        for (int i = 0; i < 15; ++i) s += (uint8_t)imei[i];
        s += FAID_SECLEVEL_DEFAULT;
        s &= 0xFFFFu;
        ee[f->secstate_off + 6] = (uint8_t)(s >> 8);
        ee[f->secstate_off + 7] = (uint8_t)(s & 0xFF);
    }
}

// Finalize the two identity integrity checksums (firmware 0x264c56 tune/security, 0x234588
// contact/config). Recomputed AFTER the identity is written, so writing the identity into the
// summed regions cannot break them.
static void faid_finalize_checksums(struct Mad2* m, const EepromFaid* f) {
    uint8_t* ee = m->i2c_eeprom;
    if (f->tunesec_cksum_off && f->tunesec_sum_end) {
        // Zero the 4-byte store, sum [0, tunesec_sum_end), write the 16-bit result as BE32
        // (the two overlap bytes at the store are therefore zero, as the firmware expects).
        ee[f->tunesec_cksum_off + 0] = 0; ee[f->tunesec_cksum_off + 1] = 0;
        ee[f->tunesec_cksum_off + 2] = 0; ee[f->tunesec_cksum_off + 3] = 0;
        unsigned s = 0;
        for (unsigned i = 0; i < f->tunesec_sum_end; ++i) s += ee[i];
        s &= 0xFFFFu;
        ee[f->tunesec_cksum_off + 2] = (uint8_t)(s >> 8);
        ee[f->tunesec_cksum_off + 3] = (uint8_t)(s & 0xFF);
        if (getenv("I2CLOG")) printf("[faid] tune/security checksum @0x%X = 0x%04X\n",
                                     f->tunesec_cksum_off, s);
    }
    if (f->config_start && f->config_cksum_off) {
        // sum [config_start, config_cksum_off) minus the two correction bytes, stored BE16.
        unsigned s = 0;
        for (unsigned i = f->config_start; i < f->config_cksum_off; ++i) s += ee[i];
        if (f->config_corr0) s -= ee[f->config_corr0];
        if (f->config_corr1) s -= ee[f->config_corr1];
        s &= 0xFFFFu;
        ee[f->config_cksum_off + 0] = (uint8_t)(s >> 8);
        ee[f->config_cksum_off + 1] = (uint8_t)(s & 0xFF);
        if (getenv("I2CLOG")) printf("[faid] contact/config checksum @0x%X = 0x%04X\n",
                                     f->config_cksum_off, s);
    }
}

// ---- 5110 (NSE-1 v5.30) FAID — record-derive variant --------------------------------------
// The 5110 factory-ID check (fw 0x25F378, verdict [0x10B09C], gate = bit5 of the return)
// derives an 8-byte value from the phone identity and compares it to EEPROM records:
//   derived[i] = identity[1+i] ^ serial[8+i] ^ key[i]      (fw derive 0x25F130)
// where identity is the 8-byte blob at EEPROM 0x0C (the SAME identity slot the 3210 uses),
// serial is the "record 3" sub-field of 0x0C ("9019871\0" on the NokiX blob; fw 0x258798),
// and key is the 8-byte constant at flash 0x2AADB4. The result must equal:
//   rec 0x706 @ EE 0x32C (layer1 memcmp) AND rec 0x704 @ EE 0x304 (layer2 table entry),
// with rec 0x705 @ EE 0x334 != 0 (layer2 presence marker) and rec 0x707 @ EE 0x335 != 0.
// Record offsets RE'd from the static flash directory 0x2A6438. This provisions the coherent
// record set so the firmware's own FAID check passes organically (no verdict poke).
static void simlock_provision_5110(struct Mad2* m);   // defined just below
static void record_area_cksum_5110(struct Mad2* m);

static void faid_provision_5110(struct Mad2* m) {
    uint8_t* ee = m->i2c_eeprom;
    // Serial sub-field of the identity blob (fw 0x258798 selector 3). Deterministic from 0x0C;
    // on the NokiX nse-1 blob it decodes to "9019871\0".
    static const uint8_t serial[8] = { '9','0','1','9','8','7','1','\0' };
    // Key @ flash 0x2AADB4 (read from mapped flash; fall back to the RE'd constant if unmapped).
    static const uint8_t key_fallback[8] = { 0x5d,0x09,0x16,0x58,0x05,0x0c,0x1d,0x18 };
    const uint8_t* key = key_fallback;
    if (m->mem) key = &m->mem[0x2AADB4u & m->mem_mask];
    // identity[1+i] = the 8 identity bytes at EEPROM 0x0C (after the loaded length byte).
    const uint8_t* id = &ee[0x0C];
    uint8_t derived[8];
    for (int i = 0; i < 8; ++i) derived[i] = (uint8_t)(id[i] ^ serial[i] ^ key[i]);
    for (int i = 0; i < 8; ++i) { ee[0x32C + i] = derived[i]; ee[0x304 + i] = derived[i]; }
    ee[0x334] = 0x01;   // rec 0x705 presence marker
    ee[0x335] = 0x01;   // rec 0x707 marker
    if (getenv("I2CLOG")) {
        printf("[faid] 5110 FAID derived =");
        for (int i = 0; i < 8; ++i) printf(" %02X", derived[i]);
        printf("  -> rec706@0x32C rec704@0x304 rec705@0x334=1\n");
    }
    simlock_provision_5110(m);
    record_area_cksum_5110(m);   // must follow every record write
}

// ---- 5110 SIMlock records (EE 0x20..0x37) --------------------------------------------------
// MEASURED on the v5.30 ring, not guessed. The MCU ships the whole record set to the DSP at
// boot and the offsets fall straight out of it (`DSP54_COSIM=1 MDILOG=1`, m2d op=70):
//
//   {70 14} 12B = EE 0x14..0x1F   FLASH-ID record
//   {70 15} 20B = EE 0x00..0x0B   IMEI record  + EE 0x0C..0x13 identity block
//   {70 16} 24B = EE 0x20..0x37   SIMlock parts 1 and 2, XOR-masked in transit
//
// The mask is a fixed 12-byte constant applied to each half. Measured directly: stage 24 zero
// bytes at 0x20 and the {70 16} payload comes back as SIML_XOR twice, exactly.
//
// The DSP decodes each half at PROM 0x7F2D — one of six entry points into the same cipher block
// the MSID uses (0x7F22 is the matching ENCODE entry). 0x7F2D selects the LOCK table at data
// 0xB6DF and mixes the COBBA words (0x1F0C:0x1F0D) into its first four bytes via 0x8015, which
// is literally our sel_alg(algo, MODE_SIMLOCK, cobba); the round core is the same codec. Verified
// with spike/dsp54/siml_probe.c: the key the ROM builds at 0x13DC and the plaintext it produces
// are byte-identical to calcul_encode/decode.
//
// So a record set for our pinned identity is exactly:
//     EE[0x20] = calcul_encode(LOCK1_DEF, algo, MODE_SIMLOCK, cobba) ^ SIML_XOR
//     EE[0x2C] = calcul_encode(LOCK2_DEF, algo, MODE_SIMLOCK, cobba) ^ SIML_XOR
// which reproduces, byte for byte, what NokTool writes into this EEPROM over MBUS — an
// independent confirmation of the whole chain from a real service tool.
//
// LOCK1_DEF/LOCK2_DEF are the published Calcul UNLOCKED defaults, so this provisions a
// coherent no-lock pair bound to this handset's COBBA. It is ordinary factory provisioning
// through the real record format at the real offsets — not a bypass: the firmware still runs
// its own SIMlock check, it just now has a self-consistent record to check instead of the
// donor blob's, which belongs to a different COBBA and decodes to noise.
// The transport pad, DERIVED — ported from the MCU routine that builds it (5110 v5.30
// 0x25792C..0x2579A2, reached from the {70 16} assembler at 0x25792D; the final XOR loop is the
// PC that shows up if you RAMWATCH the message buffer). It is a function of the IMEI RECORD and
// of nothing else, which is why rewriting the FLASH-ID record leaves it untouched.
//
//   stage 1 (0x25792C)  the 12-byte IMEI record, laid down TWICE to fill 24 bytes, is walked in
//                       byte PAIRS and each pair replaced by the 16-bit product's halves:
//                       (a,b) -> (lo(a*b), hi(a*b)).
//   stage 2 (0x25794C)  key[j] = bitrev(~X[23-j]) — the inner loop shifts the COMPLEMENT of each
//                       source bit, LSB-first, into an accumulator it shifts left, so the byte
//                       comes out bit-reversed and inverted, and the buffer comes out reversed.
//                       (The same shape as the record codec's own rev_buf, plus the NOT.)
//   stage 3 (0x25798E)  block[i] ^= key[i] over 24 bytes.
//
// Feeding the record twice is what makes the pad repeat with period 12, so only 12 bytes are
// produced here. Verified against both measured pads:
//     3AE6978A9961875C1B7B6F1B (donor) -> 2F52CF60F3DE63607599D3C7
//     E37069457C7457265D647BC9 (ours)  -> F936DBD5CFA8E3F3C74D39F5
// Computed, not measured — so provisioning stays correct for ANY pinned identity.
static void siml_pad_from_imei(const uint8_t imei_rec[12], uint8_t pad[12]) {
    uint8_t x[24];
    for (int i = 0; i < 12; ++i) { x[i] = imei_rec[i]; x[12 + i] = imei_rec[i]; }
    for (int i = 0; i < 12; ++i) {                       // stage 1
        unsigned p = (unsigned)x[2 * i] * (unsigned)x[2 * i + 1];
        x[2 * i]     = (uint8_t)(p & 0xFF);
        x[2 * i + 1] = (uint8_t)((p >> 8) & 0xFF);
    }
    for (int j = 0; j < 12; ++j) {                       // stage 2
        uint8_t s = (uint8_t)~x[23 - j], r = 0;
        for (int b = 0; b < 8; ++b)
            if (s & (1u << b)) r |= (uint8_t)(1u << (7 - b));
        pad[j] = r;
    }
}
#define SIML_P1_OFF 0x20u
#define SIML_P2_OFF 0x2Cu

static void simlock_provision_5110(struct Mad2* m) {
    if (!m->model) return;
    { const char* off = getenv("EE5110_NOSIML");        // A/B opt-out
      if (off && *off && *off != '0') return; }
    const uint8_t* msid = m->model->identity.msid;
    int pinned = 0;
    for (int i = 0; i < 13; ++i) if (msid[i]) { pinned = 1; break; }
    if (!pinned) return;                    // no pinned identity -> nothing coherent to write

    uint8_t crc[4], cobba[4], hash[4];
    if (!calcul_decode_msid(msid, crc, cobba, hash)) return;

    uint8_t* ee0 = m->i2c_eeprom;

    // --- the other two records, so the whole set is bound to ONE identity -------------------
    // Both of the blob's are the donor handset's, encoded against a COBBA that is not ours:
    // decoded under our key they are noise (IMEI record -> 6106AB17…, FLASH-ID -> 35E438D0…).
    // The MCU ships both to the DSP verbatim at boot ({70 15} = 0x00..0x13, {70 14} =
    // 0x14..0x1F — read straight off the ring), so leaving them donor-owned means the DSP is
    // handed an identity that contradicts the MSID we pin.
    const char* imei14 = m->model->identity.imei14;
    if (imei14) {
        uint8_t imei_rec[12], imei_bcd[7];
        if (calcul_imei_record(imei14, msid[0], cobba, imei_rec, imei_bcd))
            memcpy(&ee0[0x00], imei_rec, 12);
    }
    {   // FLASH-ID record: the decoded MSID fields put through ppm_crc, then mode-1 encoded.
        uint8_t plain[12], flid[12];
        memcpy(plain, crc, 4); memcpy(plain + 4, cobba, 4); memcpy(plain + 8, hash, 4);
        calcul_ppm_crc(plain);
        if (calcul_encode(plain, msid[0], CALCUL_MODE_FLASH, cobba, flid))
            memcpy(&ee0[0x14], flid, 12);
    }

    // The transport pad follows from the IMEI record we just wrote — computed, so this tracks
    // any pinned identity with no re-measurement.
    uint8_t SIML_XOR[12];
    siml_pad_from_imei(&ee0[0x00], SIML_XOR);

    // The PLAINTEXTS are NOT the published Calcul LOCK1_DEF/LOCK2_DEF. Those are the generic
    // "unlocked" defaults of the record codec; this firmware's local-security profile is a
    // different object, and feeding it LOCK1_DEF (which decodes to 0010 1000 0000 0010 0079)
    // gets the record REJECTED and the phone takes its one-shot retry reboot (reason-4
    // @0x258D2E).
    //
    // What it accepts is a WILDCARD-compare profile: part 1's leading FF FF FF FF FF match any
    // IMSI, which is what "no operator lock" means here. These 24 bytes are exactly the
    // region_a the ROM-4 HLE hands the firmware (src/mad2/dsp/dsp_rom4.c) — the known-accepted
    // record. The difference is that the HLE FABRICATES it, whereas provisioning it here makes
    // the real C54x mask ROM decode it out of the EEPROM and report it itself.
    //
    // Both carry the 0x54C2 trailer: the handler verifies each decoded part's last word against
    // data[0xB703] (= 0x54C2) at PROM 0x4B8E/0x4B92 before zeroing it, which is also why both
    // published LOCK defaults end in 54C2.
    //
    // Verified end to end: with these records and DSP54_SELFTEST_MEAS=0 (i.e. NO output stub),
    // the co-sim boots past "SIM card not accepted" to standby — lcd 1d1dee9b…, the same screen
    // the HLE reaches.
    static const uint8_t SIML_P1_PLAIN[12] = {
        0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x78,0x54,0xC2 };
    static const uint8_t SIML_P2_PLAIN[12] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x7C,0x54,0xC2 };
    uint8_t p1[12], p2[12];
    if (!calcul_encode(SIML_P1_PLAIN, msid[0], CALCUL_MODE_SIMLOCK, cobba, p1) ||
        !calcul_encode(SIML_P2_PLAIN, msid[0], CALCUL_MODE_SIMLOCK, cobba, p2))
        return;

    uint8_t* ee = m->i2c_eeprom;
    for (int i = 0; i < 12; ++i) {
        ee[SIML_P1_OFF + i] = (uint8_t)(p1[i] ^ SIML_XOR[i]);
        ee[SIML_P2_OFF + i] = (uint8_t)(p2[i] ^ SIML_XOR[i]);
    }
    if (getenv("I2CLOG")) {
        printf("[faid] 5110 SIMlock provisioned (COBBA %02X%02X%02X%02X) p1@0x%02X=",
               cobba[0], cobba[1], cobba[2], cobba[3], SIML_P1_OFF);
        for (int i = 0; i < 12; ++i) printf("%02X", ee[SIML_P1_OFF + i]);
        printf(" p2@0x%02X=", SIML_P2_OFF);
        for (int i = 0; i < 12; ++i) printf("%02X", ee[SIML_P2_OFF + i]);
        printf("\n");
    }
}

// The whole record area 0x00..0x3D is covered by a 16-bit big-endian sum at 0x3E. Writing any
// record without refreshing it fails the firmware's integrity check and the phone comes up on
// CONTACT SERVICE — measured: provisioning SIMlock alone dropped the 5110 guard from
// `23a09224…` to the CONTACT SERVICE hash `b86d6d5c…`, and refreshing this restored it.
// The rule is confirmed against a real tool: recomputing it over NokTool's own bake (FLASH-ID +
// both SIMlock records rewritten) reproduces the 0x1874 NokTool stored there, and the virgin
// blob's 0x1A17 is the sum of its own untouched records. Idempotent — safe to run always.
#define REC_CKSUM_OFF 0x3Eu
static void record_area_cksum_5110(struct Mad2* m) {
    uint8_t* ee = m->i2c_eeprom;
    unsigned s = 0;
    for (unsigned i = 0; i < REC_CKSUM_OFF; ++i) s += ee[i];
    s &= 0xFFFFu;
    ee[REC_CKSUM_OFF]     = (uint8_t)(s >> 8);
    ee[REC_CKSUM_OFF + 1] = (uint8_t)(s & 0xFF);
    if (getenv("I2CLOG"))
        printf("[faid] 5110 record-area checksum @0x%02X = 0x%04X (sum 0x00..0x%02X)\n",
               REC_CKSUM_OFF, s, REC_CKSUM_OFF - 1);
}

static void faid_provision(struct Mad2* m) {
    if (m->model && m->model->name &&
        (strcmp(m->model->name, "5110") == 0 || strcmp(m->model->name, "5110i") == 0)) {
        // Bespoke identity path (record-derive, not the 3210 IMEI-BCD one); the descriptor
        // below still applies for the parts that ARE shared (the security-level setting).
        faid_provision_5110(m);
    }
    const EepromFaid* f = m->model ? m->model->eeprom_faid : NULL;
    if (!f) return;
    if (f->imei_prefix && f->identity_off) {
        // Blank identity slot (3210): write a whole coherent identity + security state.
        faid_provision_identity(m, f);
    } else if (f->identity_off && f->secstate_off) {
        // Real factory identity already in the blob: only make the security block agree
        // with it again (the template's stored checksum belongs to another handset).
        faid_refresh_security_cksum(m, f);
    }
    // Reset the stored security-level user setting to the erased factory default (0xFF =
    // off) — a real dump may carry the previous owner's "code at power-up" setting (0x00),
    // which prompts on every boot. Done before the checksum finalize so the config sum
    // covers the normalized byte.
    if (f->seclevel_off) {
        m->i2c_eeprom[f->seclevel_off] = 0xFF;
        if (getenv("I2CLOG"))
            printf("[faid] security-level setting @0x%X reset to 0xFF (factory default: no "
                   "code at power-up)\n", f->seclevel_off);
    }
    faid_finalize_checksums(m, f);
    if (getenv("I2CLOG"))
        printf("[faid] EEPROM identity/security provisioned (%s)\n", m->model->name);
}

// ---- Per-revision checksum / latch fix-ups (moved verbatim from ext_eeprom.c) --------------

static void legacy_checksum_fixups(struct Mad2* m) {
    // The tune-checksum offsets + DSP-fault-latch record are 5110 (24C16) specific. Other
    // serial-EEPROM models (24C64/128) have a different layout, so gate to the 2K device.
    int is_24c16 = !(m->model && m->model->i2c_eeprom_size > 2048);
    if (is_24c16) {
        unsigned s = 0;
        for (int i = 0x40; i < 0x11E; i++) s += m->i2c_eeprom[i];
        s = (s - m->i2c_eeprom[0x74] - m->i2c_eeprom[0x75]) & 0xFFFF;
        m->i2c_eeprom[0x11E] = (uint8_t)(s >> 8);
        m->i2c_eeprom[0x11F] = (uint8_t)(s & 0xFF);
        if (getenv("I2CLOG")) printf("[i2c] tune checksum finalized @0x11E = 0x%04X\n", s);
        // DSP-fault latch (record 0x607, 1 byte @0x29E on the 24C16): an erased 0xFF reads as
        // "fault latched" -> deterministic reason-0x68 SWDSP reset ~283M after cold boot. A
        // factory phone ships 0x00 (no fault); provision only the virgin state.
        if (m->i2c_eeprom[0x29E] == 0xFF) {
            m->i2c_eeprom[0x29E] = 0x00;
            if (getenv("I2CLOG")) printf("[i2c] DSP-fault latch (rec 0x607) provisioned @0x29E = 0x00\n");
        }
    } else {
        // 24C64/128/256 (6110-family + 8810): same DSP-fault latch at a different record offset
        // (8810 v6.02: rec 0x607 -> {0x3F2,1}). Provision the virgin 0xFF state only.
        if (m->model->i2c_eeprom_size > 0x3F2 && m->i2c_eeprom[0x3F2] == 0xFF) {
            m->i2c_eeprom[0x3F2] = 0x00;
            if (getenv("I2CLOG")) printf("[i2c] DSP-fault latch (rec 0x607) provisioned @0x3F2 = 0x00\n");
        }
    }

    // NSB calibration-record checksum finalize (5190/6190): stored[off] == (Σ EEPROM[beg..beg+len)
    // - adj_hi - adj_lo) & 0xFFFF, adj = word@nsb_cksum_adj. Requires 2-byte addressing.
    if (m->model && m->model->nsb_cksum_off) {
        uint32_t off = m->model->nsb_cksum_off;
        uint32_t beg = m->model->nsb_cksum_beg;
        uint32_t len = m->model->nsb_cksum_len;
        uint32_t adjo = m->model->nsb_cksum_adj;
        unsigned s = 0;
        for (uint32_t i = beg; i < beg + len; i++) s += m->i2c_eeprom[i & 0x7FFF];
        unsigned adj = ((unsigned)m->i2c_eeprom[adjo & 0x7FFF] << 8) | m->i2c_eeprom[(adjo + 1) & 0x7FFF];
        s = (s - (adj >> 8) - (adj & 0xFF)) & 0xFFFF;
        m->i2c_eeprom[off & 0x7FFF]       = (uint8_t)(s >> 8);
        m->i2c_eeprom[(off + 1) & 0x7FFF] = (uint8_t)(s & 0xFF);
        if (getenv("I2CLOG")) printf("[i2c] NSB calib checksum finalized @0x%X = 0x%04X\n", off, s);
    }
    // Second NSB checksum — self-test result item 18 (validator 0x26CC4C): 16-bit byte-sum over
    // EEPROM[0..off) stored as a BE u32 at [off] (outside its own sum, upper 2 bytes zeroed).
    if (m->model && m->model->nsb_cksum2_off) {
        uint32_t off = m->model->nsb_cksum2_off;
        unsigned s = 0;
        for (uint32_t i = 0; i < off; i++) s += m->i2c_eeprom[i & 0x7FFF];
        s &= 0xFFFF;
        m->i2c_eeprom[off       & 0x7FFF] = 0;
        m->i2c_eeprom[(off + 1) & 0x7FFF] = 0;
        m->i2c_eeprom[(off + 2) & 0x7FFF] = (uint8_t)(s >> 8);
        m->i2c_eeprom[(off + 3) & 0x7FFF] = (uint8_t)(s & 0xFF);
        if (getenv("I2CLOG")) printf("[i2c] NSB calib checksum#2 finalized @0x%X = 0x%04X\n", off, s);
    }
}

// ---- Service entry -------------------------------------------------------------------------

void eeprom_provision(struct Mad2* m) {
    if (getenv("EE5110_RAW")) return;   // opt out of all provisioning (checksum-fault A/B)
    legacy_checksum_fixups(m);          // per-revision tune/calib checksums + DSP-fault latch
    faid_provision(m);                  // FAID identity + identity checksums (models that opt in)
}
