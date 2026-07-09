// Compile-gated native SDL2 GUI overlay for the DCT3 boot-trace frontend.
//
// This is a THIN, OPTIONAL overlay on tools/boot_trace.c — the SOLE native
// frontend. ALL boot/IO/harness/run-loop infrastructure stays in boot_trace.c;
// the GUI only mirrors the LCD framebuffer, renders a diag panel, and injects
// keypad events using the EXISTING mad2 keypad idiom (kbd_norm_cols + irq_pending).
// It reimplements NOTHING from the core / mad2 / harness / models.
//
// Build modes:
//   DCT3_SDL undefined  -> every entry point is a static-inline NO-OP, so
//                          boot_trace.c compiles byte-identically with zero SDL
//                          dependency (the headless build/dct3_boot_trace).
//   DCT3_SDL defined    -> tools/gui_sdl.c provides the real implementations.
//
// The window stays alive (driven from boot_trace's post-loop wait) after a fault/
// halt so the crash/halt LCD + reason stay visible until the user closes it.

#ifndef DCT3_GUI_H
#define DCT3_GUI_H

#include <stdint.h>   // int16_t — gui_pcm_sink sample type

// Forward declarations — the real struct definitions come from the headers
// boot_trace.c already includes (mad2.h / model.h / dct3_core.h). gui_sdl.c
// includes those headers itself; here we only need the names for the prototypes.
struct Mad2;
struct ARMCore;
struct ModelProfile;

// Per-frame input/control returned to the run loop. All fields are edge/level
// signals the loop acts on minimally (see boot_trace.c, DCT3_SDL-guarded block):
//   quit   — window closed / Esc: break the loop, then shut down.
//   paused — Space toggles; the loop spins on gui_frame() while paused.
//   reboot — R key, or a 30-second HOLD of the on-screen PWR button (our own hard
//            reset, distinct from the firmware's own power-off): the loop performs an
//            in-process warm reboot (reset core+mad2, re-run boot, keep flash) via
//            gui_reset(). Also honoured from the post-halt wait, so a 30s hold can
//            hard-reset a powered-off / frozen phone.
typedef struct {
    int quit;
    int paused;
    int reboot;
} GuiInput;

#ifdef DCT3_SDL

// Real overlay (tools/gui_sdl.c).
void     gui_init(const struct ModelProfile* prof);
GuiInput gui_frame(struct Mad2* m, struct ARMCore* cpu, long long step,
                   const char* halt_reason);
// Reset per-boot overlay state (held keys, the boot power-hold one-shot, the PWR
// hold timer) so an in-process warm reboot starts clean. Called by boot_trace's
// run loop when it acts on gi.reboot.
void     gui_reset(void);
void     gui_shutdown(void);
// HAL PCM channel consumer: install as m->pcm_sink so the DSP codec earpiece
// samples (ch1) play out the host audio device opened in gui_init. ch0/other
// channels are ignored. Safe to call before the device opens (drops samples).
void     gui_pcm_sink(struct Mad2* m, int ch, int16_t sample);
// DIAGNOSTIC: if GUI_PCMTEST=<file> is set, play that raw codec-rate PCM capture
// through the consumption path with NO core running, and return 1 (caller exits).
// Returns 0 when GUI_PCMTEST is unset (normal boot). See gui_sdl.c.
int      gui_pcm_selftest(void);

#else

// NO-OP stubs: boot_trace.c compiles unchanged, no SDL linkage. The GUI=1 env
// path in boot_trace is itself DCT3_SDL-guarded, so these are never reached in
// the headless build — they exist only to keep the single source compiling.
static inline void gui_init(const struct ModelProfile* prof) { (void)prof; }
static inline GuiInput gui_frame(struct Mad2* m, struct ARMCore* cpu,
                                 long long step, const char* halt_reason) {
    (void)m; (void)cpu; (void)step; (void)halt_reason;
    GuiInput gi = {0, 0, 0};
    return gi;
}
static inline void gui_reset(void) {}
static inline void gui_shutdown(void) {}
static inline void gui_pcm_sink(struct Mad2* m, int ch, int16_t sample) {
    (void)m; (void)ch; (void)sample;
}
static inline int gui_pcm_selftest(void) { return 0; }

#endif // DCT3_SDL

#endif // DCT3_GUI_H
