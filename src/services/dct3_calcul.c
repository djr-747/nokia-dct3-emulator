// dct3_calcul — DCT3 security-record codec. See dct3_calcul.h and, for the full derivation
// notes + the offline inverse used to mint an MSID, tools/dct3_calcul.py.
//
// Every constant table below is transcribed from the published Calcul unit, with ONE
// correction recovered by inverse-decoding a real NokTool B6/4F IMEI write: IMEI_DEF byte 9
// is 0xFF, not 0x00 (the published source does not reproduce a real record).

#include "services/dct3_calcul.h"

#include <string.h>

static const uint8_t ENCOD_TABL[12] = {0xB1,0x73,0xE6,0x5A,0xAB,0x47,0x8E,0x0D,0x1A,0x34,0x68,0x0B};
static const uint8_t DECOD_TABL[12] = {0xD0,0x16,0x2C,0x58,0xB0,0x71,0xE2,0xD5,0x5A,0x67,0xCE,0x8D};

// Per-selector tables. Index 0/1/2 = selector 0x81/0x82/0x83.
static const uint8_t MSID_DEC[3][12] = {
    {0x83,0xCB,0x74,0x24,0xDF,0xF1,0xE9,0x97,0x68,0x71,0xDB,0x44},
    {0x9F,0x7A,0xAD,0x34,0xE7,0x79,0x27,0x73,0x4E,0x41,0x0D,0x26},
    {0x50,0xF3,0x65,0x25,0xD2,0xB1,0xC1,0xB6,0x09,0xAE,0xFF,0x4C},
};
static const uint8_t FLID_ENC[3][12] = {
    {0x2C,0x03,0x6F,0x34,0x5D,0x23,0xC6,0x58,0x03,0x09,0x52,0xC4},
    {0x4F,0xB9,0x6C,0x27,0x02,0xDD,0x77,0xDD,0x06,0xE3,0xE2,0xDF},
    {0xB8,0x33,0x80,0x04,0xB8,0x48,0x85,0x1A,0x2D,0xCE,0xA1,0x28},
};
// The 0x81 Lock/Imei tables are flagged "bad table ... do not use" in the source; they are
// carried verbatim (same bytes as 0x83) so the table indexing stays honest.
static const uint8_t LOCK_ENC[3][12] = {
    {0x7B,0xB4,0xD0,0xEF,0x9E,0xB2,0x0A,0xBE,0x73,0xDA,0xD3,0x35},
    {0xD1,0xA2,0x5F,0xEB,0x4F,0xF0,0x2D,0x7B,0x0F,0x4C,0xE1,0xC3},
    {0x7B,0xB4,0xD0,0xEF,0x9E,0xB2,0x0A,0xBE,0x73,0xDA,0xD3,0x35},
};
static const uint8_t IMEI_ENC[3][12] = {
    {0xCA,0x63,0x5A,0xE7,0x19,0xFA,0xA6,0x4C,0x0D,0x78,0xFC,0x16},
    {0x0B,0xAE,0xFC,0x8C,0xC7,0xFC,0x79,0x2A,0xC1,0x94,0xC2,0x37},
    {0xCA,0x63,0x5A,0xE7,0x19,0xFA,0xA6,0x4C,0x0D,0x78,0xFC,0x16},
};

const uint8_t CALCUL_LOCK1_DEF[12] = {0x00,0x10,0x10,0x00,0x00,0x00,0x00,0x10,0x00,0x79,0x54,0xC2};
const uint8_t CALCUL_LOCK2_DEF[12] = {0x00,0x00,0x00,0x00,0x29,0x00,0x00,0x00,0x00,0x00,0x54,0xC2};
// IMEI 000000-00-000000. Byte 9 = 0xFF, see the file header.
static const uint8_t IMEI_DEF[12] = {0x79,0x29,0,0,0,0,0,0,0,0xFF,0,0};

static int algo_index(uint8_t algo) {
    return (algo == 0x81) ? 0 : (algo == 0x82) ? 1 : (algo == 0x83) ? 2 : -1;
}

// Mode 0 returns the MSID decode table as-is; modes 1..3 XOR their encode table with the
// COBBA serial, which is what binds every derived record to one handset.
static int sel_alg(uint8_t algo, int mode, const uint8_t cobba12[12], uint8_t out[12]) {
    int a = algo_index(algo);
    if (a < 0) return 0;
    const uint8_t* tab;
    switch (mode) {
        case CALCUL_MODE_MSID:    memcpy(out, MSID_DEC[a], 12); return 1;
        case CALCUL_MODE_FLASH:   tab = FLID_ENC[a]; break;
        case CALCUL_MODE_SIMLOCK: tab = LOCK_ENC[a]; break;
        case CALCUL_MODE_IMEI:    tab = IMEI_ENC[a]; break;
        default: return 0;
    }
    for (int i = 0; i < 12; ++i) out[i] = (uint8_t)(tab[i] ^ cobba12[i]);
    return 1;
}

static uint8_t rev_byt(uint8_t b) {
    uint8_t r = 0;
    for (int i = 0; i < 8; ++i) if (b & (1u << i)) r |= (uint8_t)(1u << (7 - i));
    return r;
}

// Reverse the buffer AND the bits of every byte.
static void rev_buf(uint8_t buf[12]) {
    uint8_t c[12];
    memcpy(c, buf, 12);
    for (int i = 0; i < 12; ++i) buf[i] = rev_byt(c[11 - i]);
}

// Rotate the big-endian 32-bit word at buf[p..p+3] right k times.
static void crr32k(uint8_t buf[12], int p, int k) {
    uint32_t x = ((uint32_t)buf[p] << 24) | ((uint32_t)buf[p+1] << 16)
               | ((uint32_t)buf[p+2] << 8) | buf[p+3];
    k %= 32;
    if (k) x = (x >> k) | (x << (32 - k));
    buf[p]   = (uint8_t)(x >> 24);
    buf[p+1] = (uint8_t)(x >> 16);
    buf[p+2] = (uint8_t)(x >> 8);
    buf[p+3] = (uint8_t)x;
}

// Xn := Xn xor (Xn+4 or not Xn+8), indices wrapping at 12, all from a snapshot.
static void xor_or_b(uint8_t buf[12]) {
    uint8_t c[12];
    memcpy(c, buf, 12);
    for (int i = 0; i < 12; ++i)
        buf[i] = (uint8_t)(c[i] ^ (c[(i + 4) % 12] | (uint8_t)~c[(i + 8) % 12]));
}

// XOR in the algorithm table; lanes 2,3,8,9 also take the round key byte.
static void scr_stt(uint8_t buf[12], const uint8_t tab1[12], const uint8_t key[12], int p) {
    for (int i = 0; i < 12; ++i) buf[i] ^= tab1[i];
    buf[2] ^= key[p]; buf[3] ^= key[p]; buf[8] ^= key[p]; buf[9] ^= key[p];
}

// Permut: each source lane feeds a fixed set of destination lanes. Transcribed in the SAME
// order as the Pascal, because the first touch of a lane is a plain assignment and later
// touches are XORs — reordering changes the result. `seen` tracks first-touch per lane.
static const uint8_t PERM[12][5] = {
    { 1, 4, 8, 9,11}, { 5, 6,10, 8, 9}, { 3, 1, 6,10,11}, { 0, 7, 8,10,11},
    { 0, 1, 3, 5, 8}, { 2, 0, 1, 9,10}, { 2, 3, 5, 7,10}, { 0, 2, 3, 4,11},
    { 0, 4, 5, 7, 9}, { 1, 2, 4, 5, 6}, { 2, 6, 7, 9,11}, { 3, 4, 6, 7, 8},
};

static void permut(uint8_t buf[12]) {
    uint8_t w[12] = {0}, seen[12] = {0}, x = 0;
    for (int src = 0; src < 12; ++src) {
        uint8_t v = buf[src];
        x ^= v;
        for (int j = 0; j < 5; ++j) {
            int d = PERM[src][j];
            w[d] = seen[d] ? (uint8_t)(v ^ w[d]) : v;
            seen[d] = 1;
        }
    }
    for (int i = 0; i < 12; ++i) buf[i] = (uint8_t)(w[i] ^ x);
}

// 11 full rounds + a final half round. The same routine serves both directions of the
// stored form — only the `key` schedule differs (DECOD_TABL vs ENCOD_TABL).
static void codec_n(uint8_t buf[12], const uint8_t tab[12], const uint8_t key[12]) {
    for (int i = 0; i < 11; ++i) {
        scr_stt(buf, tab, key, i);
        permut(buf);
        crr32k(buf, 0, 10);
        crr32k(buf, 8, 31);
        xor_or_b(buf);
        crr32k(buf, 8, 10);
        crr32k(buf, 0, 31);
    }
    scr_stt(buf, tab, key, 11);
    permut(buf);
    rev_buf(buf);
}

int calcul_decode_msid(const uint8_t msid[13], uint8_t flash_crc[4],
                       uint8_t cobba[4], uint8_t hash[4]) {
    uint8_t tab[12], buf[12], zero[12] = {0};
    if (!sel_alg(msid[0], CALCUL_MODE_MSID, zero, tab)) return 0;
    memcpy(buf, msid + 1, 12);
    codec_n(buf, tab, DECOD_TABL);
    if (flash_crc) memcpy(flash_crc, buf + 0, 4);
    if (cobba)     memcpy(cobba,     buf + 4, 4);
    if (hash)      memcpy(hash,      buf + 8, 4);
    return 1;
}

int calcul_encode(const uint8_t plain[12], uint8_t algo, int mode,
                  const uint8_t cobba[4], uint8_t out[12]) {
    uint8_t tab[12], cobba12[12] = {0};
    memcpy(cobba12, cobba, 4);
    if (!sel_alg(algo, mode, cobba12, tab)) return 0;
    memcpy(out, plain, 12);
    codec_n(out, tab, ENCOD_TABL);
    return 1;
}

void calcul_ppm_crc(uint8_t buf[12]) {
    static const uint8_t mask[4] = {0x17, 0xCA, 0x60, 0x89};
    for (int i = 0; i < 4; ++i) buf[i] ^= mask[i];
    for (int i = 0; i < 4; ++i) buf[4 + i] = (uint8_t)(buf[i] ^ 0xFF);
    for (int i = 8; i < 12; ++i) buf[i] = 0;
}

int calcul_luhn(const char* imei14) {
    if (!imei14) return -1;
    int total = 0;
    for (int i = 0; i < 14; ++i) {
        char ch = imei14[i];
        if (ch < '0' || ch > '9') return -1;
        int d = ch - '0';
        if (i & 1) { d *= 2; if (d > 9) d -= 9; }   // double every second digit from the left
        total += d;
    }
    if (imei14[14] != '\0' && (imei14[14] < '0' || imei14[14] > '9')) { /* 15-char input ok */ }
    return (10 - total % 10) % 10;
}

int calcul_imei_record(const char* imei14, uint8_t algo, const uint8_t cobba[4],
                       uint8_t out[12], uint8_t bcd[7]) {
    if (calcul_luhn(imei14) < 0) return 0;
    uint8_t packed[7], plain[12];
    for (int i = 0; i < 7; ++i)                       // straight BCD, high nibble first
        packed[i] = (uint8_t)(((imei14[2*i] - '0') << 4) | (imei14[2*i + 1] - '0'));
    memcpy(plain, IMEI_DEF, 12);
    memcpy(plain + 2, packed, 7);
    plain[9] = 0xFF;                                  // see the file header
    if (bcd) memcpy(bcd, packed, 7);
    return calcul_encode(plain, algo, CALCUL_MODE_IMEI, cobba, out);
}
