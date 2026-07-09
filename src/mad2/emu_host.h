// EmuHost — the single typed surface a harness binds to (docs/hal-spec.md).
//
// Every harness (web src/web/main.c, SDL tools/gui_sdl.c, headless tools/boot_trace.c,
// node tools/nav.mjs) observes the emulated phone through THESE accessors instead of
// reaching into Mad2 fields. The point is decoupling: a harness depends on this stable,
// enumerated contract — not the Mad2 struct layout — and a NEW harness reads one header
// to learn the complete hardware-facing surface. It is MODEL-AGNOSTIC: geometry, LED
// colour, keypad layout and capabilities all come from the active profile, so a harness
// drives any DCT3 model without hardcoding the 3310.
//
// Scope = the HARDWARE surface (display / LEDs / audio / vibra / lifecycle + input
// primitives). It is NOT the debug-instrumentation surface (trace rings, heap curve,
// wrwatch, …) — those stay harness/diagnostic-specific. It is NOT the control surface
// (the ~knob registry is a separate, still-pending consolidation).
//
// INPUT NOTE — timing is harness POLICY, not part of this contract. The KK_*->matrix
// resolution is shared (emu_keyline), but how a press is driven over time differs by
// harness by design: the web binding enqueues a logical key for an auto-release
// sequencer playing it over emulated time; the SDL GUI asserts/releases on real key
// up/down events; boot_trace injects at the raw-matrix level on a step schedule. So the
// façade exposes the key LOOKUP + the special-scan primitive, and each harness owns its
// down/hold/up timing.

#ifndef DCT3_EMU_HOST_H
#define DCT3_EMU_HOST_H

#include "mad2/mad2.h"   // Mad2, ModelProfile, mad2_lcd_px, mad2_keypad_irq

// ---- Display (PCD8544 framebuffer) ------------------------------------------
static inline int      emu_lcd_w(const Mad2* m)     { return m->model ? m->model->lcd.width  : 84; }
static inline int      emu_lcd_h(const Mad2* m)     { return m->model ? m->model->lcd.height : 48; }
static inline int      emu_lcd_banks(const Mad2* m) { return m->model ? m->model->lcd.banks  : 6; }
static inline const uint8_t* emu_lcd_fb(const Mad2* m) { return m->fb; }   // raw DDRAM (bulk readers)
static inline int      emu_lcd_pixel(const Mad2* m, int x, int y) { return mad2_lcd_px(m, x, y); }
static inline int      emu_lcd_mode(const Mad2* m)  { return m->lcd_mode; }  // 0 blank/1 all-on/2 normal/3 inverse
static inline uint64_t emu_lcd_dirty(const Mad2* m) { return m->lcd_data_writes; }  // dirty hint

// ---- Backlight LEDs ----------------------------------------------------------
static inline int emu_led_lcd(const Mad2* m) { return m->led_lcd ? 1 : 0; }
static inline int emu_led_kbd(const Mad2* m) { return m->led_kbd ? 1 : 0; }
// Packed state: bit0 = LCD backlight, bit1 = keypad backlight.
static inline int emu_led_bits(const Mad2* m) { return emu_led_lcd(m) | (emu_led_kbd(m) << 1); }
// Period-correct backlight colour, 0xRRGGBB (0 = harness's classic yellow-green default).
// which: 0 = LCD glow, 1 = keypad buttons (falls back to the LCD colour).
static inline uint32_t emu_led_rgb(const Mad2* m, int which) {
    if (!m->model) return 0;
    uint32_t c = (which == 1 && m->model->led.kbd_rgb) ? m->model->led.kbd_rgb : m->model->led.lcd_rgb;
    return c;
}

// ---- Audio: buzzer (PWM tone) + DSP/COBBA PCM --------------------------------
static inline int      emu_buzzer_on(const Mad2* m)  { return m->buzz_on; }
static inline uint16_t emu_buzzer_div(const Mad2* m) { return m->buzz_div; }   // freq = 13 MHz / div
static inline uint8_t  emu_buzzer_vol(const Mad2* m) { return m->buzz_vol; }
// Sub-frame chirp latch (so a beep between two frame polls is never missed): rising-edge
// count (low byte) packed with the divider-at-edge (high 16); READS AND CLEARS the count.
static inline int      emu_buzzer_chirp_drain(Mad2* m) {
    int v = (m->buzz_edges & 0xFF) | ((m->buzz_div_at_edge & 0xFFFF) << 8);
    m->buzz_edges = 0;
    return v;
}
static inline int      emu_vibra_on(const Mad2* m)   { return m->vibra_on; }
static inline uint8_t  emu_vibra_ctrl(const Mad2* m) { return m->vibra_ctrl; }
// PCM earpiece channel: register a sink to receive every DSP codec DAC sample (ch1 =
// earpiece, ch0 = secondary) as it is emitted; emu_pcm_rate is the producer sample rate.
static inline void     emu_set_pcm_sink(Mad2* m, void (*sink)(Mad2*, int, int16_t)) { m->pcm_sink = sink; }
static inline double   emu_pcm_rate(const Mad2* m)   { return m->pcm_rate ? m->pcm_rate : (double)DCT3_CODEC_HZ; }
// Unified audio mixer (emu_audio.c): render the PWM buzzer voice into the pcm_sink stream
// up to the current rtc_mono, at DCT3_CODEC_HZ, so the buzzer and the DSP codec arrive on
// ONE PCM channel. The device model calls this at the top of every buzzer register write
// (before the state mutates) so a tone's onset/length land on the exact emulated-time
// sample; a harness calls it once per frame as a flush so a held tone keeps streaming.
// No-op when pcm_sink is NULL or the DSP codec owns the sink (pcm_codec_seen) — headless
// boots stay byte-identical.
void emu_audio_render(Mad2* m);

// ---- Lifecycle ---------------------------------------------------------------
static inline int emu_powered_off(const Mad2* m)  { return (int)m->power_off; }
static inline int emu_reset_reason(const Mad2* m) { return (int)m->reset_last_reason; }

// ---- Input primitives (timing is harness policy — see header note) -----------
// The model's matrix line for a logical key (NULL if the key is absent on this model);
// `.special` != 0 = a power/special-scan key. emu_key_present(m,id) is the boolean form.
const KeyLine* emu_keyline(const ModelProfile* prof, int key_id);   // src/models/model.c
static inline int emu_key_present(const Mad2* m, int key_id) {
    return m->model && emu_keyline(m->model, key_id) != NULL;
}
// Assert/release a power/special-scan key (PWR) + pulse the keypad IRQ. Normal matrix
// keys are harness-timed (queue/sequencer or immediate up/down), so they are NOT here.
static inline void emu_key_special(Mad2* m, uint8_t mask, int down) {
    if (down) m->kbd_special_cols |= mask;
    else      m->kbd_special_cols &= (uint8_t)~mask;
    mad2_keypad_irq(m);
}

#endif // DCT3_EMU_HOST_H
