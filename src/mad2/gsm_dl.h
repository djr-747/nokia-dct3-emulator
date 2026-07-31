// GSM downlink C0/TS0 bit-level encoders (GSM 05.02/05.03) — pure functions, no
// emulator dependencies. Feed the COBBA RX model (cobba.c) with real burst content so
// the 5110 DSP's own demodulator decodes SCH/BCCH organically. The I/Q sample synthesis
// (rate, scaling, GMSK pulse shape) lives with the consumer; this header is bits only.
#ifndef MAD2_GSM_DL_H
#define MAD2_GSM_DL_H

#include <stdint.h>

// SCH: 25 info bits (BSIC 6 + RFN T1 11 / T2 5 / T3' 3) + 10 parity + 4 tail ->
// rate-1/2 conv -> 78 coded bits. Output: the full 148-symbol SB burst bit pattern
// (3 tail, 39 data, 64 extended training seq, 39 data, 3 tail).
void gsm_sch_burst(uint32_t fn, unsigned bsic, uint8_t out_bits[148]);

// BCCH (or any CCCH) block: 184 info bits -> FIRE(40) -> 4 tail -> conv 1/2 -> 456
// bits -> interleave over 4 bursts of 114. out_bits[4][114] in transmit order.
void gsm_bcch_encode(const uint8_t l2_23bytes[23], uint8_t out_bits[4][114]);

// Assemble a normal burst (NB): 3 tail, 57 data, 1 steal, 26 TSC, 1 steal, 57 data,
// 3 tail = 148 symbols. tsc = training sequence code 0..7 (BCC for CCCH).
void gsm_normal_burst(const uint8_t data114[114], unsigned tsc, uint8_t out_bits[148]);

// FCCH burst: 148 zero bits (the pure +67.7 kHz tone after GMSK).
void gsm_fcch_burst(uint8_t out_bits[148]);

// Differential encode per GSM 05.04 (d[i] = a[i] XOR a[i-1]) then map to GMSK phase
// steps: out_qsteps[i] = +1/-1 quarter-turn rotation per symbol (MSK approximation of
// GMSK — adequate for a detector/demod built for real GMSK; refine if the DSP's
// matched filter rejects it). state carries a[-1] across calls.
void gsm_gmsk_phase_steps(const uint8_t bits[148], int8_t out_qsteps[148], uint8_t *state);

// 51-multiframe TS0 schedule for C0 downlink (GSM 05.02): returns the burst type for
// frame (fn mod 51): 'F' FCCH (0,10,20,30,40), 'S' SCH (1,11,21,31,41),
// 'B' BCCH (2-5), 'C' CCCH (6-9,12-19,22-29,32-39,42-49), 'I' idle (50).
char gsm_ts0_sched(uint32_t fn51);

#endif
