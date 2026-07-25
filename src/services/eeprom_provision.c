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

// Provision the identity + derived security state (bitplane's provision_security_identity):
//   EEPROM[identity_off] = 14 IMEI digits, high-nibble-first BCD (+ 1 zero byte)
//   EEPROM[seccode_off]  = 5-digit security code, BCD
//   EEPROM[secstate_off] = 4-byte "encrypted" code, 2 zero, BE16 identity checksum
// The stored identity checksum (secstate bytes 6-7) is sum16(identity_ascii(15) + callback 0x11),
// the exact word selector 0x2ae61a compares against.
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
        // identity checksum = sum16(imei_ascii[0..14] + callback state 0x11), stored BE16.
        unsigned s = 0;
        for (int i = 0; i < 15; ++i) s += (uint8_t)imei[i];
        s += 0x11u;
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
}

static void faid_provision(struct Mad2* m) {
    if (m->model && m->model->name &&
        (strcmp(m->model->name, "5110") == 0 || strcmp(m->model->name, "5110i") == 0)) {
        faid_provision_5110(m);
        return;
    }
    const EepromFaid* f = m->model ? m->model->eeprom_faid : NULL;
    if (!f) return;
    if (f->imei_prefix && f->identity_off) faid_provision_identity(m, f);
    // Reset the stored security-level user setting to the erased factory default (0xFF =
    // off) — a real dump may carry the previous owner's "code at power-up" setting (0x00),
    // which prompts on every boot. Done before the checksum finalize so the config sum
    // covers the normalized byte.
    if (f->seclevel_off) m->i2c_eeprom[f->seclevel_off] = 0xFF;
    faid_finalize_checksums(m, f);
    if (getenv("I2CLOG"))
        printf("[faid] identity provisioned (%s) — Security-code prompt cleared, "
               "service-present retained\n", m->model->name);
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
