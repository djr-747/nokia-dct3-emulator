// GSM 05.02/05.03 downlink bit-level encoders — see gsm_dl.h. Pure functions.
//
// Conventions: octet bits are taken MSB-first into the 05.03 info-bit stream (the
// osmocom gsm0503 convention); if the 5110 DSP's own decoder disagrees on packing we
// flip HERE, in one place, against the observed d2m sync/BCCH reports.

#include "mad2/gsm_dl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- convolutional encoder, GSM 05.03 rate 1/2, K=5: G0=1+D3+D4, G1=1+D+D3+D4 -------
static void conv_half(const uint8_t *u, unsigned n, uint8_t *c) {
    unsigned s = 0;
    for (unsigned k = 0; k < n; k++) {
        s = ((s << 1) | (u[k] & 1u)) & 0x1Fu;          // s holds u(k), u(k-1).. u(k-4)
        c[2*k]   = (uint8_t)(((s >> 4) ^ (s >> 3) ^ s) & 1u);            // G0: D0+D3+D4
        c[2*k+1] = (uint8_t)(((s >> 4) ^ (s >> 3) ^ (s >> 1) ^ s) & 1u); // G1: D0+D1+D3+D4
    }
}

// --- SCH ------------------------------------------------------------------------------
// 25 info bits use the TS 45.003 packed SB_INFO layout, expanded LSB-first:
// byte0 = BSIC[5:0]<<2 | T1[10:9], byte1 = T1[8:1],
// byte2 = T1[0]<<7 | T2[4:0]<<2 | T3'[2:1], byte3 bit0 = T3'[0].
// Parity = 10-bit CRC, g(D)=D10+D8+D6+D5+D4+D2+1, transmitted inverted.
// Then 4 zero tails, conv 1/2 -> 78.
static void sch_info_bits(uint32_t fn, unsigned bsic, uint8_t a[25]) {
    uint32_t t1 = (fn / 1326u) % 2048u;      // superframe count (51x26 frames)
    uint32_t t2 = fn % 26u;
    uint32_t t3 = fn % 51u;
    uint32_t t3p = t3 / 10u;                 // SCH frames: t3 in {1,11,21,31,41}
    uint8_t packed[4];
    packed[0] = (uint8_t)(((bsic & 0x3fu) << 2) | ((t1 & 0x600u) >> 9));
    packed[1] = (uint8_t)((t1 & 0x1feu) >> 1);
    packed[2] = (uint8_t)(((t1 & 1u) << 7) |
                          ((t2 & 0x1fu) << 2) |
                          ((t3p & 6u) >> 1));
    packed[3] = (uint8_t)(t3p & 1u);
    for (unsigned i = 0; i < 25; i++)
        a[i] = (uint8_t)((packed[i >> 3] >> (i & 7u)) & 1u);
}

static void sch_parity(const uint8_t a[25], uint8_t p[10]) {
    // LFSR division of a(0..24) * D^10 by g; remainder inverted.
    unsigned reg = 0;
    const unsigned g = 0x175u;               // D10+(D8+D6+D5+D4+D2+1) low bits: 1 0111 0101
    for (unsigned k = 0; k < 25; k++) {
        unsigned fb = ((reg >> 9) ^ a[k]) & 1u;
        reg = ((reg << 1) & 0x3FFu) ^ (fb ? g : 0u);
    }
    for (unsigned k = 0; k < 10; k++)
        p[k] = (uint8_t)(((reg >> (9 - k)) & 1u) ^ 1u);
}

// GSM 05.02 §5.2.5 extended training sequence (64 bits)
static const uint8_t sch_ets[64] = {
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
    0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1
};

void gsm_sch_uncoded(uint32_t fn, unsigned bsic, uint8_t out[35]) {
    sch_info_bits(fn, bsic, out);
    sch_parity(out, out + 25);
}

void gsm_sch_burst(uint32_t fn, unsigned bsic, uint8_t out[148]) {
    uint8_t u[39], c[78];
    gsm_sch_uncoded(fn, bsic, u);
    memset(u + 35, 0, 4);
    conv_half(u, 39, c);
    if (getenv("DSP54_SCHBITS")) {
        static unsigned n;
        if (n++ < 4) {
            fprintf(stderr, "[rf] SCHCODE fn=%u bsic=%u", fn, bsic);
            for (unsigned i = 0; i < 78; i++) fprintf(stderr, " %u", c[i]);
            fputc('\n', stderr);
        }
    }
    memset(out, 0, 3);                        // tail
    memcpy(out + 3, c, 39);                   // e(0..38)
    memcpy(out + 42, sch_ets, 64);
    memcpy(out + 106, c + 39, 39);            // e(39..77)
    memset(out + 145, 0, 3);                  // tail
}

// --- BCCH -----------------------------------------------------------------------------
// FIRE code g(x) = (x23+1)(x17+x3+1) = x40+x26+x23+x17+x3+1; parity inverted.
static void fire_parity(const uint8_t d[184], uint8_t p[40]) {
    uint8_t reg[40];
    memset(reg, 0, sizeof reg);
    static const uint8_t taps[5] = { 26, 23, 17, 3, 0 };  // exponents below 40
    for (unsigned k = 0; k < 184; k++) {
        uint8_t fb = (uint8_t)(reg[39] ^ d[k]);
        memmove(reg + 1, reg, 39);
        reg[0] = 0;
        if (fb) for (unsigned t = 0; t < 5; t++) reg[taps[t]] ^= 1u;
    }
    for (unsigned k = 0; k < 40; k++)
        p[k] = (uint8_t)(reg[39 - k] ^ 1u);
}

void gsm_bcch_encode(const uint8_t l2[23], uint8_t out[4][114]) {
    uint8_t d[184], u[228], c[456];
    for (unsigned k = 0; k < 184; k++)
        d[k] = (uint8_t)((l2[k / 8] >> (7 - (k % 8))) & 1u);   // MSB-first per octet
    memcpy(u, d, 184);
    fire_parity(d, u + 184);
    memset(u + 224, 0, 4);
    conv_half(u, 228, c);
    // interleave (05.03 §4.1.4): B = k mod 4, j = 2*((49k) mod 57) + ((k mod 8) div 4)
    uint8_t i[4][114];
    memset(i, 0, sizeof i);
    for (unsigned k = 0; k < 456; k++)
        i[k % 4][2u * ((49u * k) % 57u) + ((k % 8u) / 4u)] = c[k];
    memcpy(out, i, sizeof i);
}

// GSM 05.02 §5.2.3 training sequences (26 bits, TSC 0..7)
static const uint8_t tsc_tab[8][26] = {
    {0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1},
    {0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1},
    {0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0},
    {0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0},
    {0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1},
    {0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0},
    {1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1},
    {1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0}
};

void gsm_normal_burst(const uint8_t e[114], unsigned tsc, uint8_t out[148]) {
    memset(out, 0, 3);
    memcpy(out + 3, e, 57);
    /* xCCH maps both stealing positions as one (gsm0503_xcch_encode does
     * likewise).  They are part of the normal-burst control-channel mapping,
     * even though this is not FACCH stealing a traffic burst. */
    out[60] = 1;                              // hl
    memcpy(out + 61, tsc_tab[tsc & 7u], 26);
    out[87] = 1;                              // hu
    memcpy(out + 88, e + 57, 57);
    memset(out + 145, 0, 3);
}

void gsm_fcch_burst(uint8_t out[148]) { memset(out, 0, 148); }

void gsm_gmsk_phase_steps(const uint8_t bits[148], int8_t out[148], uint8_t *state) {
    uint8_t prev = *state & 1u;
    for (unsigned k = 0; k < 148; k++) {
        uint8_t d = (uint8_t)(bits[k] ^ prev);
        prev = bits[k] & 1u;
        out[k] = d ? -1 : 1;                  // MSK: +90deg for d=0, -90deg for d=1
    }
    *state = prev;
}

char gsm_ts0_sched(uint32_t f) {
    f %= 51u;
    if (f == 50u) return 'I';
    if (f % 10u == 0u) return 'F';
    if (f % 10u == 1u) return 'S';
    if (f >= 2u && f <= 5u) return 'B';
    return 'C';
}
