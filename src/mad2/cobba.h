// COBBA (analog baseband codec) chip model — RX RF front-end sample source.
//
// On DCT3 the DSP never sees raw RF: the RF module downconverts the tuned channel to
// analog baseband I/Q and the COBBA's RX ADCs digitize and stream the words to the MAD
// DSP (port 0x27 on the 5110/MAD2 co-sim). This module models that stream: a per-channel
// signal source whose consumer passes in the currently tuned channel. It is deliberately
// C54x-free — the DSP bridge owns the port dispatch and the tuned-channel tap; this file
// owns the signal. (The COBBA serial/parallel REGISTER models still live in the bridge,
// third_party/c54x/mad2_dsp_c54x.c — planned to migrate here with the knob-registry work.)
#ifndef MAD2_COBBA_H
#define MAD2_COBBA_H

#include <stdint.h>

// Nonzero when the synthetic RX source is enabled (DSP54_RFMODEL). Lazy-inits the model.
int cobba_rf_enabled(void);

// Next RX sample component on the currently tuned channel (interleaved I,Q — one signed
// 16-bit component per call, matching the DSP's port-0x27 read granularity).
// `cell_arfcn` is the bridge's DSP-scratch-cell guess at the tuned channel; it is used
// only while DSP54_RF_CHTAP is not `synth`. With `DSP54_RF_CHTAP=synth` the channel comes
// from cobba_rf_synth() below — the physically right source — but that is NOT the default
// yet: the DSP's own tune path (block03 0x99D / 0xA85 -> ROM 0x3FFB -> ports 0x31/0x32) is
// dormant in the co-sim (MEASURED: one port-0x31 write in a 300M-step run, at 0xA23A from
// the boot-time queue template), so a synth-only channel would never leave the noise floor.
uint16_t cobba_rf_sample(uint16_t cell_arfcn);

// Reconcile receiver reads with the continuously running RF/GSM air-time clock at a
// hardware timing event. `quarter_symbols` uses the same one-read-per-quarter-symbol
// timebase as cobba_rf_sample().
void cobba_rf_advance(uint32_t quarter_symbols);
uint32_t cobba_rf_time(void);
void cobba_rf_sync_advance(uint32_t quarter_symbols);
void cobba_rf_sync_fcch(uint32_t quarter_symbols_until_burst);
void cobba_rf_align_sch(void);
void cobba_rf_arm_xcch(void);
int cobba_rf_xcch_pending(void);
void cobba_rf_align_next_xcch(void);
void cobba_rf_align_frame(unsigned frame51);
void cobba_rf_inject_fcch(uint32_t quarter_symbols);
char cobba_rf_last_burst(void);
uint32_t cobba_rf_last_frame(void);
uint32_t cobba_rf_last_pos(void);

// RF synthesizer programming tap — the physically faithful "what frequency is the radio
// on" signal. The 5110 DSP builds a 32-bit PLL word in ROM fn 0x3FFB (RX) / 0x4006 (TX)
// and ships it out as a low/high pair on DSP I/O ports 0x31 / 0x32 (either directly from
// 0x4020/0x4025 when the caller passes AR2=49, or queued at 0x402A and replayed by the
// RF sequencer at 0x370C/0xA23A). Call this on the port-0x32 write that completes a pair.
//   lo/hi  : the two port words
//   cfg    : DSP DARAM [0x216A] — bit1 selects the 0x4049 packing, bit2 the 0x402E one
//   off    : DSP DARAM [0x216B] — the ARFCN -> divider offset (measured 0x13A6 = 5030,
//            i.e. LO = 935.2 MHz + 0.2*(ARFCN-1) + 71.0 MHz IF at a 200 kHz raster)
void cobba_rf_synth(uint16_t lo, uint16_t hi, uint16_t cfg, uint16_t off);

#endif
