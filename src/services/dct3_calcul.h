// dct3_calcul — the DCT3 security-record codec (MSID / FLASH-ID / SIMlock / IMEI).
//
// C port of the published "Calcul" unit (Salami1_1, 2003) that DCT3 service tools use to
// turn a phone's 13-byte MSID into the EEPROM records its firmware validates against. The
// Python reference implementation is tools/dct3_calcul.py — it carries the full derivation
// notes, the inverse round function (used offline to MINT an MSID) and a self-test against
// a real captured NHM-5 MSID. This port is the ENCODE direction only, which is all the
// in-emulator provisioning service needs: the MSID itself is minted offline and pinned in
// the model profile (ModelProfile.identity), so nothing here has to invert anything.
//
// The shape of it:
//
//     msid[0]     -> algorithm selector 0x81 / 0x82 / 0x83
//     msid[1..12] -> decode -> [0..3] flash CRC | [4..7] COBBA s/n | [8..11] hash
//
// The COBBA serial is therefore not stored anywhere — it is derived from the MSID — and
// every other record is encoded with tables XOR-ed against that COBBA. One MSID, one
// self-consistent identity.
#ifndef DCT3_CALCUL_H
#define DCT3_CALCUL_H

#include <stdint.h>

// Record modes (the `mode` argument to calcul_encode).
enum {
    CALCUL_MODE_MSID   = 0,   // MSID itself (used by calcul_decode_msid)
    CALCUL_MODE_FLASH  = 1,   // FLASH ID / FAID record
    CALCUL_MODE_SIMLOCK = 2,  // SIMlock parts 1 and 2
    CALCUL_MODE_IMEI   = 3,   // IMEI record
};

// Decode a 13-byte MSID into its three 4-byte fields. `flash_crc`, `cobba` and `hash` may
// each be NULL if not wanted. Returns 0 if msid[0] is not a known selector (0x81/82/83).
int calcul_decode_msid(const uint8_t msid[13], uint8_t flash_crc[4],
                       uint8_t cobba[4], uint8_t hash[4]);

// Encode a 12-byte plaintext into a 12-byte EE record. `cobba` is the 4-byte COBBA serial
// (the codec pads it to 12 internally). Returns 0 on a bad selector/mode.
int calcul_encode(const uint8_t plain[12], uint8_t algo, int mode,
                  const uint8_t cobba[4], uint8_t out[12]);

// The FLASH-ID plaintext transform: un-mask the stored flash CRC, byte-complement it into
// bytes 4..7 and zero 8..11. `buf` is the decoded MSID (crc|cobba|hash), modified in place.
void calcul_ppm_crc(uint8_t buf[12]);

// Build the mode-3 IMEI record from 14 IMEI digits. `out` gets the 12-byte record and
// `bcd` the 7-byte straight-BCD plaintext the B6 frame also carries. Returns 0 if the
// digits are not 14 characters of '0'..'9'.
int calcul_imei_record(const char* imei14, uint8_t algo, const uint8_t cobba[4],
                       uint8_t out[12], uint8_t bcd[7]);

// The 15th (check) digit of an IMEI, standard Luhn over the first 14. Returns -1 on bad
// input, else 0..9.
int calcul_luhn(const char* imei14);

// The published Calcul defaults for an UNLOCKED SIMlock pair (plaintexts, not records).
extern const uint8_t CALCUL_LOCK1_DEF[12];
extern const uint8_t CALCUL_LOCK2_DEF[12];

#endif // DCT3_CALCUL_H
