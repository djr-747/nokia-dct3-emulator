// Unified audio mixer — buzzer + DSP HLE tone -> the shared pcm_sink PCM stream.
//
// THE PROBLEM this solves: the audio voices used to be synthesized inside each frontend
// by POLLING device/RAM state at frame cadence (SDL ~21 ms audio callback, web ~16.7 ms
// rAF), and each frontend did it DIFFERENTLY — the SDL GUI had only the PWM buzzer (as a
// device-clock square), the web page had the buzzer AND a pair of Web Audio sine
// oscillators for the DSP tone (keypad beep / DTMF), read from the COBBA tone registers.
// So the GUI had no keypad beep at all on non-cosim models, and a short buzzer chirp that
// enabled+disabled inside one frame-burst was rendered late, softly, or missed entirely.
//
// THE FIX: render every voice into PCM at EMULATED-TIME resolution, in ONE shared place.
// The buzzer onset is sample-accurate because emu_audio_render() is called at the TOP of
// every buzzer register write (before the state mutates), flushing the stream up to that
// exact rtc_mono at the old state; the DSP HLE tone is polled at the per-frame flush (a beep
// spans many frames, so frame-granular onset is inaudible). Both voices are mixed and
// delivered on the SAME pcm_sink channel the 5110 DSP codec uses — so every frontend plays
// ONE PCM stream and never branches on DSP type or model.
//
// VOICE OWNERSHIP: the buzzer is a MAD2 ASIC peripheral (PWM, I/O 0x15) and is synthesized
// HERE. The tone is a DSP function, so its HPI-register logic lives in the DSP layer, not
// here: the active DSP backend's DspOps.hle_tone reports the commanded frequencies (the HLE
// backends read the COBBA regs; the cosim backend leaves hle_tone NULL and plays the tone
// itself through the codec DXR tap). The mixer only asks "what tone?" and renders+mixes it.
// pcm_codec_seen additionally stands the whole mixer down once the real codec has spoken, so
// a cosim run is never doubled.
//
// Clocked off rtc_mono (the 13 MHz monotonic cycle accumulator) at DCT3_CODEC_HZ. No-op
// when pcm_sink is NULL, so headless boots (make guard / make test) stay byte-identical.

#include "mad2/mad2_internal.h"
#include "mad2/emu_host.h"

// Buzzer square-wave amplitude (matches the legacy frontend synth level; below full-scale
// so the buzzer + tone sum without clipping: 8000 + 2*2600 = 13200 < 32767).
#define EMU_BUZZ_AMP  8000

// Buzzer tone-shaping low-pass coefficient (Q8, one-pole y += a*(x-y), two cascaded stages).
// a=96/256 -> fc ~= 3.6 kHz per stage at EMU_AUDIO_HZ (~12 dB/oct combined). Models the piezo
// disc + case grille rolling off the PWM square's high harmonics: ringtone fundamentals
// (<= ~2 kHz) pass intact, the hard edges round off — the real phone never sounded like a
// mathematically perfect square.
#define EMU_BUZZ_LP_A 96
// DSP HLE tone amplitude PER oscillator (~0.08 full-scale, matching the web sine gain).
#define EMU_TONE_AMP  2600

// 256-point sine table (int16, ±EMU_TONE_AMP), built once via the Bhaskara I approximation
// sin(x deg) ≈ 4x(180-x) / (40500 - x(180-x)) — accurate to ~0.2%, and no libm dependency
// (emu_audio has none, matching the GUI WSOLA path's libm-free constraint).
static int16_t g_sin[256];
static int     g_sin_ready;
static void emu_sin_init(void) {
    for (int i = 0; i < 256; i++) {
        double d = 360.0 * i / 256.0;                    // phase in degrees
        double s;
        if (d <= 180.0) { double u = d * (180.0 - d);        s =  4.0 * u / (40500.0 - u); }
        else            { double e = d - 180.0, u = e*(180.0-e); s = -4.0 * u / (40500.0 - u); }
        g_sin[i] = (int16_t)(s * EMU_TONE_AMP);
    }
    g_sin_ready = 1;
}

void emu_audio_render(Mad2* m) {
    // Off unless a host sink is attached; the real DSP codec owns the sink once it speaks.
    if (!m->pcm_sink || m->pcm_codec_seen) return;
    m->pcm_rate = (double)EMU_AUDIO_HZ;   // publish the HLE rate (codec DXR overrides on takeover)

    // Absolute sample index on the mixer grid at "now". rtc_mono*EMU_AUDIO_HZ stays well
    // within uint64 for any realistic run (hours * 13e6 * 48000 << 2^64).
    uint64_t target = (m->rtc_mono * (uint64_t)EMU_AUDIO_HZ) / DCT3_ARM_HZ;
    uint64_t pos    = m->audio_last_cyc;
    if (target <= pos) { m->audio_last_cyc = target; return; }   // nothing new (or clock rebased back)
    uint64_t n = target - pos;
    // Clamp a giant first burst (sink attached mid-run, or a long fast-forward) to ~1 s so
    // we never flood the ring; steady-state per-frame n is only a few hundred samples.
    if (n > (uint64_t)EMU_AUDIO_HZ) n = (uint64_t)EMU_AUDIO_HZ;

    // --- Voice 1: PWM buzzer (I/O 0x15 bit5 + divider) -----------------------
    int      buzz_on  = (m->buzz_on && m->buzz_div);
    uint32_t buzz_inc = 0;
    if (buzz_on) {
        uint32_t freq = 13000000u / m->buzz_div;                             // 13 MHz / divider
        buzz_inc = (uint32_t)(((uint64_t)freq << 16) / EMU_AUDIO_HZ);        // Q16 cycle per sample
    }

    // --- Voice 2: DSP HLE tone — the ACTIVE DSP backend decides (not the mixer) ----
    // Ask whichever DSP is bound what tone the MCU has commanded. HLE backends report it
    // from the COBBA regs; the cosim backend has no hle_tone (plays it via the codec DXR).
    int tone_on = 0; uint32_t t1_inc = 0, t2_inc = 0;
    const DspOps* d = m->dsp_override ? m->dsp_override : (m->model ? m->model->dsp : NULL);
    int f1 = 0, f2 = 0;
    if (d && d->hle_tone && d->hle_tone(m, &f1, &f2) && f1 > 0) {
        tone_on = 1;
        if (!g_sin_ready) emu_sin_init();
        t1_inc = (uint32_t)(((uint64_t)(unsigned)f1 << 16) / EMU_AUDIO_HZ);
        if (f2 > 0) t2_inc = (uint32_t)(((uint64_t)(unsigned)f2 << 16) / EMU_AUDIO_HZ);
    }

    // CONSTANT STREAM: emit every sample on the codec grid, INCLUDING silence. The stream
    // never stops between tones, so the consumer's buffer level stays stable and it primes
    // exactly once — no stop/underrun/re-prime transient at every tone boundary (that
    // start/stop cycling was audible as scratch between tones). Idle silence is real samples.
    //
    // ENVELOPE: the firmware gates a voice on/off mid-phase (amp register cut), which is a
    // hard step = a click at every tone edge. A short Q8 attack/release (EMU_ENV_STEP/sample,
    // full swing ~64 samples ≈ 3.4 ms) smooths both edges; the release keeps the waveform
    // ringing down at its CACHED pitch (audio_c_*) after the registers already gate it off.
    int voiced = (buzz_on || tone_on);
    uint32_t c_buzz, c_t1, c_t2;
    if (voiced) {
        c_buzz = buzz_on ? buzz_inc : 0;
        c_t1   = tone_on ? t1_inc   : 0;
        c_t2   = tone_on ? t2_inc   : 0;
        m->audio_c_buzz = c_buzz; m->audio_c_t1 = c_t1; m->audio_c_t2 = c_t2;
    } else {                                               // release: ring down the last voice
        c_buzz = m->audio_c_buzz; c_t1 = m->audio_c_t1; c_t2 = m->audio_c_t2;
    }
    #define EMU_ENV_STEP 4
    for (uint64_t i = 0; i < n; i++) {
        if (voiced) { if (m->audio_env < 256) m->audio_env += EMU_ENV_STEP; }
        else        { if (m->audio_env > 0)   m->audio_env -= EMU_ENV_STEP; }
        // Buzzer voice: square -> two cascaded one-pole low-passes (tone shaping). The filter
        // runs EVERY sample (input 0 when the voice is off) so its state rings down naturally
        // and never carries a stale DC step into the next tone.
        int32_t bin = 0;
        if (c_buzz) { m->audio_buzz_phase += c_buzz; bin = (m->audio_buzz_phase & 0x8000) ? -EMU_BUZZ_AMP : EMU_BUZZ_AMP; }
        m->audio_buzz_lp1 += (EMU_BUZZ_LP_A * (bin - m->audio_buzz_lp1)) >> 8;
        m->audio_buzz_lp2 += (EMU_BUZZ_LP_A * (m->audio_buzz_lp1 - m->audio_buzz_lp2)) >> 8;
        if (m->audio_env == 0) { m->pcm_sink(m, 1, 0); continue; }   // idle: pure silence
        int32_t s = m->audio_buzz_lp2;
        if (c_t1)   { m->audio_tone_ph1  += c_t1;   s += g_sin[(m->audio_tone_ph1 >> 8) & 0xFF]; }
        if (c_t2)   { m->audio_tone_ph2  += c_t2;   s += g_sin[(m->audio_tone_ph2 >> 8) & 0xFF]; }
        s = (s * (int32_t)m->audio_env) >> 8;
        if (s >  32767) s =  32767;
        if (s < -32768) s = -32768;
        m->pcm_sink(m, 1, (int16_t)s);                     // ch1 = earpiece (shared with the codec)
    }
    m->audio_last_cyc = target;
}
