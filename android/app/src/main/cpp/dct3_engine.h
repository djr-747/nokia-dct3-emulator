// DCT3 Android engine — plain-C driver around the vendored nokia-dct3-emulator core.
//
// This is the native front-end for the Nokia 3410 (NHM-2) model only: it boots one
// fixed model profile, runs it behind the shared harness (fault-detect + reset-
// recovery — see nokia-dct3-emulator/src/harness/), and exposes just what an Android
// UI needs to drive it: boot, advance time, read the LCD framebuffer, and inject
// keypad matrix edges. No jni.h here on purpose — this file is host-buildable/
// testable on its own; dct3_jni.c is the thin JNI marshaling layer on top of it.
//
// Thread-safety: every entry point below takes an internal lock, so the render/step
// loop (a background thread) and key-event delivery (the UI thread) can call in
// concurrently without the caller managing any locking itself.

#ifndef DCT3AND_ENGINE_H
#define DCT3AND_ENGINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Boot (or re-boot) the emulator from a raw .fls flash image at `path`. Always the
// Nokia 3410 profile — see model.c's DCT3_MODEL_3410_ONLY registry.
// Returns 0 on success, or a negative error code:
//   -1 file not found / can't open   -2 empty file   -3 short read   -4 core alloc failed
int dct3and_boot(const char* path);

// Advance the emulated clock by ~n_cycles CPU cycles (DCT3_ARM_HZ = 13 MHz), the same
// real-time pacing dct3_web_run_cycles uses. No-op if not booted, already faulted, or
// cleanly powered off. Safe to call from a dedicated background thread at a fixed
// cadence (e.g. n_cycles = 13000000/30 for a 30 Hz tick).
void dct3and_run_cycles(int n_cycles);

// Active LCD geometry (valid after a successful boot; 0 before). The 3410 = 96x65.
int dct3and_lcd_width(void);
int dct3and_lcd_height(void);

// Decode the current LCD DDRAM into `out` (caller-owned, width*height ints) as packed
// 0xAARRGGBB, one entry per pixel in row-major (y*width+x) order — `on_argb` where the
// firmware lit the pixel, `off_argb` otherwise (mad2_lcd_px already applies the
// controller's blank/all-on/normal/inverse display-mode transform). No-op before boot.
void dct3and_render_pixels(uint32_t* out, int out_len, uint32_t on_argb, uint32_t off_argb);

// Drive one logical key (KeyId from nokia-dct3-emulator/src/models/model.h — KK_*) up
// or down as a REAL matrix edge (no artificial hold/auto-release timer: Android key
// events already carry genuine down/up timing, so this mirrors the native SDL GUI's
// key_apply / the web's dct3_web_key_logical_raw). Returns 1 if the key exists on the
// 3410's matrix, 0 if it doesn't (caller should not treat the input as consumed).
int dct3and_key_event(int key_id, int down);

// Drain up to [max_samples] queued PCM samples (mono, 16-bit, [dct3and_audio_rate] Hz)
// into [out] — the buzzer + DSP HLE tone (keypad beeps, ringtones; the 3410 has no
// C54x codec) rendered by the shared mad2 audio mixer. Returns the count actually
// written (0 if none queued yet or not booted). Call periodically from whichever
// thread feeds your audio output; safe to call from a different thread than the one
// driving dct3and_run_cycles.
int dct3and_read_audio(int16_t* out, int max_samples);

// The fixed sample rate dct3and_read_audio's samples are at (EMU_AUDIO_HZ = 48000).
int dct3and_audio_rate(void);

// True once the firmware has cleanly powered itself off (CCONT regulators cut) past
// the boot-settle window. The engine parks (stops executing) at that point.
int dct3and_is_powered_off(void);

// True once the shared harness caught an unrecoverable fault (wild-PC / spin / heap
// guard / ...) and halted the CPU. The postmortem was already logged (logcat tag
// "dct3engine"); the UI should show a "crashed, reboot to retry" state.
int dct3and_is_faulted(void);

// Release the emulator core. Safe to call even if never booted.
void dct3and_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // DCT3AND_ENGINE_H
