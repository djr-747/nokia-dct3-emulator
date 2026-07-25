// dsp_hle_tone — the shared HLE COBBA tone reader (DspOps.hle_tone; declared in
// models/model.h). The MCU programs oscillator frequencies (1/4-Hz units) + amplitude
// into the HPI mailbox window; with no real DSP to play them, we report the active tone
// so emu_audio synthesizes it into the PCM stream. Registers are model-invariant (the
// HPI base .cobba = 0x100E0 everywhere), which is why this lives in its own neutral TU:
// BOTH ROM-revision engines (dsp_rom4.c, dsp_rom6.c) point .hle_tone here, while the
// c54x co-sim leaves .hle_tone NULL and plays tones through the codec DXR instead.
// Kept in the DSP layer (not the mixer) because tone generation is a DSP function.

#include "mad2/mad2.h"

int dsp_hle_tone(Mad2* m, int* f1_hz, int* f2_hz) {
    if (!m->mem) return 0;
    uint32_t amp_a = DCT3_TONE_AMP  & m->mem_mask;
    if (((m->mem[amp_a] << 8) | m->mem[(amp_a + 1) & m->mem_mask]) == 0) return 0;  // amplitude gate
    uint32_t o1 = DCT3_TONE_OSC1 & m->mem_mask, o2 = DCT3_TONE_OSC2 & m->mem_mask;
    int f1 = (int)(((m->mem[o1] << 8) | m->mem[(o1 + 1) & m->mem_mask])) >> 2;       // reg is 1/4 Hz
    if (f1 <= 0) return 0;
    int f2 = (int)(((m->mem[o2] << 8) | m->mem[(o2 + 1) & m->mem_mask])) >> 2;
    *f1_hz = f1;
    *f2_hz = (f2 > 0) ? f2 : 0;
    return 1;
}
