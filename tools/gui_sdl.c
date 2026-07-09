// SDL2 overlay implementation for the DCT3 boot-trace GUI (tools/gui.h).
//
// This is the ONLY SDL-touching translation unit. It is compiled (and linked
// against SDL2) ONLY for the build/dct3_boot_trace_gui target, under -DDCT3_SDL.
// The headless build/dct3_boot_trace never sees this file.
//
// What it does (and ONLY this — no boot/IO/harness/run-loop logic lives here):
//   * Open a window sized for the LCD (scaled ~7x) plus a diag side panel.
//   * Each gui_frame(): present the persistent mad2 LCD DDRAM (m->fb) as the
//     classic Nokia mono panel (ON = dark, OFF = light), tinted greenish when the
//     LCD backlight LED is on; draw a small text panel of live counters; poll SDL
//     events and translate physical-keyboard keys into mad2 keypad matrix edges
//     using the SAME (row,col) mapping as web/main.js (KMAP, keymap 0x32E718).
//   * Keypad injection mirrors src/web/main.c dct3_web_key_raw: on key DOWN set
//     m->kbd_norm_cols[row] |= 1<<col; on UP clear it; pulse m->irq_pending |= 0x01
//     on every change (the keypad is wake-on-keypress).
//
// It reimplements NOTHING from src/core, src/mad2, src/harness, src/models — it
// only reads mad2 state and pokes the same fields the existing keypad path pokes.

#ifdef DCT3_SDL

#include <SDL.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "gui.h"
#include "mad2/mad2.h"        // struct Mad2: fb, kbd_norm_cols, irq_pending, counters
#include "mad2/emu_host.h"    // EmuHost — shared HAL accessors (LED colour, pixel, ...)
#include "models/model.h"     // struct ModelProfile: lcd.width/height/banks
#include "harness/harness.h"  // heap_shadow_used_d() — the clean heap-used accessor
#include <mgba/internal/arm/arm.h>  // struct ARMCore: cycles

// Window title. The release/beta build (make gui-release) passes -DDCT3_ATTRIB="..."
// (author attribution) and optionally -DDCT3_LICENSEE="..." (the named beta tester —
// a per-recipient watermark). The dev build keeps the plain title.
#ifndef DCT3_WIN_TITLE
#  if defined(DCT3_ATTRIB) && defined(DCT3_LICENSEE)
#    define DCT3_WIN_TITLE "Nokia 3310 Emulator — " DCT3_ATTRIB " · Licensed to " DCT3_LICENSEE
#  elif defined(DCT3_ATTRIB)
#    define DCT3_WIN_TITLE "Nokia 3310 Emulator — " DCT3_ATTRIB
#  else
#    define DCT3_WIN_TITLE "DCT3 emulator (boot_trace GUI)"
#  endif
#endif

// --- Layout ----------------------------------------------------------------
#define SCALE       7
#define PANEL_W     260      // diag panel width (px) to the right of the LCD
#define MARGIN      8
#define KEYPAD_TOP  12       // gap (px) between the LCD bottom and the keypad
#define KEY_W       54       // on-screen keypad button cell width (px)
#define KEY_H       40       // on-screen keypad button cell height (px)
#define KEY_GAP     6        // gap between buttons (px)

static SDL_Window*   g_win = NULL;
static SDL_Renderer* g_ren = NULL;
static int g_lcd_w = 84, g_lcd_h = 48, g_lcd_banks = 6;
static int g_win_w = 0, g_win_h = 0;

// --- PCM earpiece audio (HAL PCM channel -> SDL) ----------------------------
// The emulator's pcm_sink (gui_pcm_sink) writes ch1 codec samples into a single-
// producer/single-consumer ring; SDL's audio thread drains it in audio_cb. The
// GUI is ~realtime-paced (vsync), so production ~= consumption; on underrun we
// emit silence, on overrun we drop (never block the emulator step loop). The
// device opens at DCT3_CODEC_HZ (the firmware-pinned ~18.6 kHz codec rate) so playback
// pitch is faithful. GUIAUDIO=0 disables; a missing audio device degrades to
// silent (video unaffected).
#define PCMRING_SZ   32768u           // ~1.8 s at ~18 kHz; power of two
#define PCMRING_MASK (PCMRING_SZ - 1u)
static int16_t g_pcm_ring[PCMRING_SZ];
static volatile unsigned g_pcm_head, g_pcm_tail;   // head=producer, tail=consumer
static SDL_AudioDeviceID g_audio_dev = 0;
// The device opens at a NATIVE host rate (48 kHz — universally supported), NOT the
// codec's ~18.6 kHz (which forces the OS into a low-quality internal resample = the
// scratch). The codec stream is converted to the device rate HERE by a feedback
// resampler (audio_cb). The buzzer is synthesized at the device clock, so its phase
// increment uses AUDIO_DEV_HZ too.
#define AUDIO_DEV_HZ 48000u

// The live emulator (set each gui_frame) — the audio thread reads its buzzer STATE.
// Benign cross-thread read of a couple of scalars; audio doesn't need a lock here.
static const Mad2* g_audio_m = NULL;

// WSOLA TIME-STRETCH + RESAMPLER (codec ~18.6 kHz -> device 48 kHz).
//
// The DSP codec stream is clean and gap-free (verified: uniform 900 Hz). The artefacts are
// CONSUMER-side and have TWO causes: (1) the old device opened at the non-standard 18.6 kHz
// (forcing an OS resample = scratch) — fixed by opening at native 48 kHz; (2) while a tone
// plays the DSP is busy synthesizing it, so the cosim runs ~0.44x realtime and produces codec
// samples ~2.3x too slowly for realtime playback — the ring starves -> dashes. (2) is a genuine
// throughput deficit: you cannot play more audio than the source makes without either dropping
// pitch (naive stretch) or gaps. WSOLA solves it by PITCH-PRESERVING time-stretch: it fills the
// realtime gap by overlap-adding waveform-similar segments, so a 24 ms tone produced over 55 ms
// of wall-time plays as a clean continuous 24-ms-of-content tone stretched to 55 ms, same pitch.
//
// Pipeline: g_pcm_ring (codec) --WSOLA(stretch alpha)--> g_inter (stretched codec)
//           --linear resample (codec/dev)--> 48 kHz device.
// alpha is driven by the INPUT-ring fill (the deficit signal): ring draining -> alpha up (stretch
// more, consume input slower); ring filling -> alpha down (toward 1.0, no stretch when the source
// keeps up, e.g. a captured-file feed). WSOLA's similarity search keeps every overlap-add seam
// phase-continuous, so there is no warble or buzz. Validated in sim: clean 900 Hz at a 2.3x
// deficit (steady-state pitch 900 Hz, stdev 0.47). Triangular window (50% overlap sums to unity;
// no libm dependency). The BUZZER is a separate state-driven voice mixed at the device clock.
#define WS_L        128                       // WSOLA frame length (codec samples) ~2 periods of the lowest tone
#define WS_HS       64                        // synthesis hop (= L/2, 50% overlap)
#define WS_OV       (WS_L - WS_HS)            // overlap region
#define WS_SEARCH   80                        // waveform-similarity search radius (codec samples)
#define WS_PRIME    320u                      // codec samples buffered before WSOLA starts (>= L+SEARCH)
#define PLAIN_PRIME 2048u                     // ring cushion before the plain (HLE) resampler starts (~43 ms
                                              //   @48k) — must exceed a frame period + scheduling jitter, or
                                              //   the buffer dips each vsync and fade/re-prime cycles = crackle
#define WS_TARGET   1200u                     // input-ring fill the alpha control aims to hold (~64 ms)
#define INTER_SZ    8192u                     // stretched-codec FIFO (power of two)
#define INTER_MASK  (INTER_SZ - 1u)

static float    g_ws_win[WS_L];               // triangular OLA window (init once)
static int      g_ws_win_init = 0;
static float    g_ws_tail[WS_OV];             // windowed second-half carry for overlap-add
static float    g_ws_ref[WS_OV];              // expected continuation (similarity target)
static int      g_ws_have_ref = 0;
static unsigned g_ws_ana = 0;                 // analysis cursor into g_pcm_ring (== consumer position)
static double   g_ws_ana_frac = 0.0;
static int16_t  g_inter[INTER_SZ];            // WSOLA output (stretched codec) FIFO
static unsigned g_inter_head = 0, g_inter_tail = 0;
static double   g_ws_rphase = 0.0;            // resample read position within g_inter (frac)

// Produce ONE WSOLA synthesis frame (WS_HS stretched-codec samples appended to g_inter) from the
// input ring. Returns 0 if not enough input is buffered yet (caller treats as underrun).
static int wsola_frame(unsigned head) {
    if ((head - g_ws_ana) < (unsigned)(WS_L + WS_SEARCH)) return 0;   // need a full frame + search slack
    unsigned best = g_ws_ana;
    if (g_ws_have_ref) {
        double bc = -1e300;
        for (int d = -WS_SEARCH; d <= WS_SEARCH; d++) {
            long s = (long)g_ws_ana + d;
            if (s < 0 || (head - (unsigned)s) < (unsigned)WS_OV) continue;
            double c = 0.0;                                            // cross-correlate with the expected continuation
            for (int k = 0; k < WS_OV; k++)
                c += (double)g_pcm_ring[((unsigned)s + k) & PCMRING_MASK] * g_ws_ref[k];
            if (c > bc) { bc = c; best = (unsigned)s; }
        }
    }
    // Overlap-add: first half blends with the previous frame's windowed tail -> emit WS_HS samples.
    for (int k = 0; k < WS_HS; k++) {
        float v = g_ws_tail[k] + (float)g_pcm_ring[(best + k) & PCMRING_MASK] * g_ws_win[k];
        int iv = (int)v; if (iv > 32767) iv = 32767; if (iv < -32768) iv = -32768;
        g_inter[g_inter_head & INTER_MASK] = (int16_t)iv; g_inter_head++;
    }
    for (int k = 0; k < WS_OV; k++)                                   // carry the windowed second half
        g_ws_tail[k] = (float)g_pcm_ring[(best + WS_HS + k) & PCMRING_MASK] * g_ws_win[WS_HS + k];
    for (int k = 0; k < WS_OV; k++)                                   // continuation that the next frame should match
        g_ws_ref[k] = (float)g_pcm_ring[(best + WS_HS + k) & PCMRING_MASK];
    g_ws_have_ref = 1;
    // Advance the analysis cursor by Ha = Hs/alpha; alpha rises as the input ring drains.
    unsigned fill = head - g_ws_ana;
    double e = ((double)WS_TARGET - (double)fill) / (double)WS_TARGET;
    if (e >  2.5) e =  2.5;
    if (e < -0.5) e = -0.5;
    double alpha = 1.0 + 1.6 * e;
    if (alpha < 0.5) alpha = 0.5;
    g_ws_ana_frac += (double)WS_HS / alpha;
    unsigned st = (unsigned)g_ws_ana_frac; g_ws_ana_frac -= st; g_ws_ana += st;
    return 1;
}

// SDL audio thread, steady 48 kHz wall-clock. The single unified PCM stream (emu_audio.c
// buzzer + DSP HLE tone, OR the cosim DSP codec) is resampled to the device rate. Which
// resampler depends on the SOURCE'S PACING:
//   * cosim DSP codec — produced SLOWER than realtime while the DSP computes a tone, so the
//     ring runs at a throughput deficit -> WSOLA time-stretch fills the gap pitch-preserving;
//   * HLE buzzer/tone (emu_audio) — produced at EXACTLY wall-clock rate, no deficit -> a
//     plain linear resample is exact. Running this shallow-buffered realtime stream through
//     WSOLA makes the alpha control read the low fill as a deficit and stretch ~2x, splicing
//     the tone continuously (= scratchy). pcm_codec_seen picks the path.
static void audio_cb(void* ud, Uint8* stream, int len) {
    (void)ud;
    int16_t* out = (int16_t*)stream;
    int n = len / (int)sizeof(int16_t);
    unsigned head = g_pcm_head;                          // producer snapshot (published with the data)
    static int16_t last   = 0;                           // last emitted codec sample (click-free underrun fade)
    static int     primed = 0;                           // WSOLA started (avoids start-into-empty-ring scratch)

    const Mad2* m = g_audio_m;
    double in_rate = (m && m->pcm_rate > 0.0) ? m->pcm_rate : (double)DCT3_CODEC_HZ;
    double rr = in_rate / (double)AUDIO_DEV_HZ;          // input samples per device sample (~0.388)

    // --- Realtime HLE source (buzzer / DSP tone): plain resample, no WSOLA ---------------
    if (!m || !m->pcm_codec_seen) {
        static int     pprimed = 0;
        static int16_t plast   = 0;
        static double  prph    = 0.0;
        if (!pprimed) {                                  // small cushion so per-frame burst jitter can't underrun
            if ((unsigned)(head - g_pcm_tail) >= PLAIN_PRIME) pprimed = 1;
            else { for (int i = 0; i < n; i++) out[i] = 0; return; }
        }
        for (int i = 0; i < n; i++) {
            if ((unsigned)(head - g_pcm_tail) >= 2u) {
                int16_t a = g_pcm_ring[g_pcm_tail & PCMRING_MASK];
                int16_t b = g_pcm_ring[(g_pcm_tail + 1u) & PCMRING_MASK];
                int32_t s = (int32_t)(a + (int32_t)((double)(b - a) * prph));
                plast = (int16_t)s; out[i] = (int16_t)s;
                prph += rr;
                while (prph >= 1.0 && (unsigned)(head - g_pcm_tail) >= 2u) { prph -= 1.0; g_pcm_tail++; }
            } else {                                     // stall (source streams constantly, so an empty
                plast = (int16_t)((plast * 31) / 32);    // ring = emulator paused/behind): fade + re-prime
                out[i] = plast;
                if (plast == 0) { pprimed = 0; prph = 0.0; }
            }
        }
        return;
    }

    if (!g_ws_win_init) {                                // triangular (Bartlett) window — 50% overlap sums to ~unity
        g_ws_win_init = 1;
        for (int i = 0; i < WS_L; i++) {
            float t = (float)i / (float)(WS_L - 1);      // 0..1
            g_ws_win[i] = 1.0f - (t < 0.5f ? (1.0f - 2.0f * t) : (2.0f * t - 1.0f));
        }
    }

    // Priming: hold the voice silent until enough input is buffered for WSOLA to start
    // cleanly. Re-armed after a full drain. The buzzer is no longer synthesized here — it
    // arrives through the ring as PCM (emu_audio_render), unified with the DSP codec voice.
    if (!primed) {
        if ((head - g_ws_ana) >= WS_PRIME) primed = 1;
        else {
            for (int i = 0; i < n; i++) out[i] = 0;
            g_pcm_tail = g_ws_ana;                       // let the producer see consumption
            return;
        }
    }

    for (int i = 0; i < n; i++) {
        int32_t s;
        while ((g_inter_head - g_inter_tail) < 2u) {     // ensure two stretched samples to interpolate
            if (!wsola_frame(head)) break;               // producer ran dry -> underrun below
        }
        if ((g_inter_head - g_inter_tail) >= 2u) {
            int16_t a = g_inter[g_inter_tail & INTER_MASK];
            int16_t b = g_inter[(g_inter_tail + 1u) & INTER_MASK];
            s = (int32_t)(a + (int32_t)((double)(b - a) * g_ws_rphase));
            last = (int16_t)s;
            g_ws_rphase += rr;
            while (g_ws_rphase >= 1.0 && (g_inter_head - g_inter_tail) >= 2u) { g_ws_rphase -= 1.0; g_inter_tail++; }
        } else {
            // genuine underrun (tone ended, producer stopped): fade to silence, re-arm priming/WSOLA
            last = (int16_t)((last * 31) / 32);
            s = last;
            if (last == 0) { primed = 0; g_ws_have_ref = 0; g_ws_rphase = 0.0; }
        }
        if (s >  32767) s =  32767;
        if (s < -32768) s = -32768;
        out[i] = (int16_t)s;
    }
    g_pcm_tail = g_ws_ana;                               // publish consumption to the producer
}

static void audio_open(void) {
    const char* e = getenv("GUIAUDIO");
    if (e && atoi(e) == 0) return;                   // explicitly disabled
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        fprintf(stderr, "GUI: audio init failed (%s) — silent\n", SDL_GetError());
        return;
    }
    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = (int)AUDIO_DEV_HZ;   // native 48 kHz; resample codec->device in audio_cb
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = 1024;
    want.callback = audio_cb;
    g_audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!g_audio_dev) {
        fprintf(stderr, "GUI: no audio device (%s) — silent\n", SDL_GetError());
        return;
    }
    SDL_PauseAudioDevice(g_audio_dev, 0);             // start pulling
    fprintf(stderr, "GUI: audio %d Hz mono S16 (PCM earpiece)\n", have.freq);
}

void gui_pcm_sink(struct Mad2* m, int ch, int16_t sample) {
    (void)m;
    if (ch != 1 || !g_audio_dev) return;             // earpiece (ch1) only
    unsigned head = g_pcm_head;
    if ((unsigned)(head - g_pcm_tail) >= PCMRING_SZ - 1u) return;   // full -> drop
    g_pcm_ring[head & PCMRING_MASK] = sample;
    g_pcm_head = head + 1u;                           // publish after the write
}

// DIAGNOSTIC: GUI_PCMTEST=<file.pcm> plays a raw int16-LE codec-rate capture through the
// EXACT consumption path (ring -> feedback resampler -> SDL device) with NO emulator core
// running. It isolates "is the artefact in the GUI consumption, or in the core's bursty
// vsync-paced production?" The file is looped continuously. Production is emulated by a
// wall-clock feeder at the codec rate (emu_pcm_rate fallback DCT3_CODEC_HZ):
//   STEADY (default)        — smooth feed: ideal production. Clean here => consumption is fine.
//   GUI_PCMTEST_BURST=1     — push a ~16 ms chunk then idle 16 ms: mimics the GUI run-loop
//                             stepping ~a frame then blocking on vsync (the suspected starver).
// Returns 1 if the test ran (caller should exit without booting), 0 if GUI_PCMTEST is unset.
int gui_pcm_selftest(void) {
    const char* path = getenv("GUI_PCMTEST");
    if (!path || !*path) return 0;
    if (!g_audio_dev) { fprintf(stderr, "PCMTEST: no audio device — cannot run\n"); return 1; }
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "PCMTEST: cannot open %s\n", path); return 1; }
    fseek(f, 0, SEEK_END); long bytes = ftell(f); fseek(f, 0, SEEK_SET);
    size_t N = (bytes > 0) ? (size_t)bytes / 2u : 0u;
    int16_t* buf = (N > 0) ? (int16_t*)malloc(N * sizeof(int16_t)) : NULL;
    if (!buf || fread(buf, 2, N, f) != N) { fprintf(stderr, "PCMTEST: read failed\n"); fclose(f); free(buf); return 1; }
    fclose(f);
    int burst = getenv("GUI_PCMTEST_BURST") ? 1 : 0;
    double rate = (double)DCT3_CODEC_HZ;                 // codec/producer rate
    g_audio_m = NULL;                                    // no buzzer voice in the test
    fprintf(stderr, "PCMTEST: %zu samples from %s, looping at %.0f Hz%s. ESC / close window to stop.\n",
            N, path, rate, burst ? "  [BURST: mimics vsync-paced core]" : "  [STEADY production]");
    size_t pos = 0;
    double t0 = (double)SDL_GetTicks();
    double pushed = 0.0;                                 // samples fed so far
    int running = 1;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = 0;
        }
        double now = (double)SDL_GetTicks();
        double due = (now - t0) * rate / 1000.0;         // samples that should have been produced by now
        long want = (long)(due - pushed);
        for (long i = 0; i < want; i++) {                // feed (one burst per wake in BURST mode)
            unsigned head = g_pcm_head;
            if ((unsigned)(head - g_pcm_tail) < PCMRING_SZ - 1u) {
                g_pcm_ring[head & PCMRING_MASK] = buf[pos];
                g_pcm_head = head + 1u;
            }
            pos = (pos + 1u) % N;
        }
        if (want > 0) pushed += (double)want;
        SDL_Delay(burst ? 16u : 2u);                     // STEADY: smooth; BURST: ~one vsync frame
    }
    free(buf);
    fprintf(stderr, "PCMTEST: stopped.\n");
    return 1;
}

// Track held keys so we only pulse the IRQ on a real edge (and release on keyup).
// Indexed by SDL scancode-ish small table built from KMAP below.

// --- Embedded 8x8 ASCII bitmap font (public domain, Daniel Hepper's font8x8,
// "basic" Latin set 0x20-0x7F). Each glyph is 8 bytes; bit set (LSB = leftmost
// column) = a lit pixel. Only the printable range we need is included. -------
static const unsigned char FONT8X8[96][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // U+0020 (space)
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // U+0021 (!)
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // U+0022 (")
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // U+0023 (#)
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // U+0024 ($)
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // U+0025 (%)
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // U+0026 (&)
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // U+0027 (')
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // U+0028 (()
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // U+0029 ())
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // U+002A (*)
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // U+002B (+)
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, // U+002C (,)
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // U+002D (-)
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // U+002E (.)
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // U+002F (/)
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // U+0030 (0)
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // U+0031 (1)
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // U+0032 (2)
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // U+0033 (3)
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // U+0034 (4)
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // U+0035 (5)
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // U+0036 (6)
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // U+0037 (7)
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // U+0038 (8)
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // U+0039 (9)
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // U+003A (:)
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, // U+003B (;)
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // U+003C (<)
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // U+003D (=)
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // U+003E (>)
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // U+003F (?)
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // U+0040 (@)
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // U+0041 (A)
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // U+0042 (B)
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // U+0043 (C)
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // U+0044 (D)
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // U+0045 (E)
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // U+0046 (F)
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // U+0047 (G)
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // U+0048 (H)
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // U+0049 (I)
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // U+004A (J)
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // U+004B (K)
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // U+004C (L)
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // U+004D (M)
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // U+004E (N)
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // U+004F (O)
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // U+0050 (P)
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // U+0051 (Q)
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // U+0052 (R)
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // U+0053 (S)
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // U+0054 (T)
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // U+0055 (U)
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // U+0056 (V)
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // U+0057 (W)
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // U+0058 (X)
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // U+0059 (Y)
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // U+005A (Z)
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // U+005B ([)
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, // U+005C (\)
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // U+005D (])
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, // U+005E (^)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // U+005F (_)
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, // U+0060 (`)
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, // U+0061 (a)
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, // U+0062 (b)
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, // U+0063 (c)
    {0x38,0x30,0x30,0x3e,0x33,0x33,0x6E,0x00}, // U+0064 (d)
    {0x00,0x00,0x1E,0x33,0x3f,0x03,0x1E,0x00}, // U+0065 (e)
    {0x1C,0x36,0x06,0x0f,0x06,0x06,0x0F,0x00}, // U+0066 (f)
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, // U+0067 (g)
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, // U+0068 (h)
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, // U+0069 (i)
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, // U+006A (j)
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, // U+006B (k)
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // U+006C (l)
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, // U+006D (m)
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, // U+006E (n)
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, // U+006F (o)
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, // U+0070 (p)
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, // U+0071 (q)
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, // U+0072 (r)
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, // U+0073 (s)
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, // U+0074 (t)
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, // U+0075 (u)
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, // U+0076 (v)
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // U+0077 (w)
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, // U+0078 (x)
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, // U+0079 (y)
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, // U+007A (z)
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, // U+007B ({)
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // U+007C (|)
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, // U+007D (})
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, // U+007E (~)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // U+007F (del)
};

// --- Keypad model (single source of truth) ---------------------------------
// One logical keypad button = a label, a matrix (row,col), the SDL keys that
// drive it from the physical keyboard, and an on-screen rectangle (filled in at
// init). (row,col) mirror web/index.html data-row/data-col + web/main.js keymap
// table 0x32E718; verified against the handoff: Menu=(4,3), Up=(0,1),
// Down=(1,1), C/Clear=(0,4). Keyboard AND on-screen clicks share THIS table, so
// the two input surfaces can never diverge.
#define MAX_KEYS_PER_BTN 3
typedef struct {
    const char*  label;                  // panel/keypad glyph(s)
    int          row, col;               // mad2 keypad matrix position (normal scan)
    SDL_Keycode  keys[MAX_KEYS_PER_BTN]; // physical-keyboard bindings (0 = unused)
    SDL_Rect     rect;                   // on-screen hitbox (init-filled)
    int          held;                   // 1 while pressed (mouse or keyboard)
    uint8_t      special;                // if !=0: drive kbd_special_cols w/ this mask
                                         // (the power key, special scan) not the matrix
} KeyBtn;

// Per-KeyId GUI metadata: the on-screen glyph + host-keyboard bindings. The matrix
// (row,col)/special come from the MODEL profile (model->keypad.lines), NOT here, so
// the same key can sit on different lines for different models. Family B (3310)
// relabels SOFT1/SOFT2 to "MENU"/"C" in glyph_for().
#define GRID_COLS 3
typedef struct { const char* glyph; SDL_Keycode keys[MAX_KEYS_PER_BTN]; } KeyMeta;
// Digit rows are FLIPPED to match the phone's spatial layout (same convention as the
// web KMAP): a keyboard numpad/number-row has 7-8-9 on top and 1-2-3 on the bottom, but
// the phone is the reverse — so host 7/8/9 drive the phone's TOP row (1 2 3) and host
// 1/2/3 drive the phone's BOTTOM row (7 8 9). Middle row (4 5 6) and 0/*/# unchanged.
static const KeyMeta KEYMETA[KK_COUNT] = {
    [KK_0]={"0",{SDLK_0,SDLK_KP_0,0}},   [KK_1]={"1",{SDLK_7,SDLK_KP_7,0}},
    [KK_2]={"2",{SDLK_8,SDLK_KP_8,0}},   [KK_3]={"3",{SDLK_9,SDLK_KP_9,0}},
    [KK_4]={"4",{SDLK_4,SDLK_KP_4,0}},   [KK_5]={"5",{SDLK_5,SDLK_KP_5,0}},
    [KK_6]={"6",{SDLK_6,SDLK_KP_6,0}},   [KK_7]={"7",{SDLK_1,SDLK_KP_1,0}},
    [KK_8]={"8",{SDLK_2,SDLK_KP_2,0}},   [KK_9]={"9",{SDLK_3,SDLK_KP_3,0}},
    [KK_STAR]={"*",{SDLK_ASTERISK,SDLK_KP_MULTIPLY,0}},
    [KK_HASH]={"#",{SDLK_HASH,SDLK_KP_HASH,0}},
    [KK_UP]={"UP",{SDLK_UP,0,0}},        [KK_DOWN]={"DN",{SDLK_DOWN,0,0}},
    [KK_SOFT1]={"SL",{SDLK_RETURN,SDLK_F1,0}},     // left soft / 3310 Menu
    [KK_SOFT2]={"SR",{SDLK_BACKSPACE,SDLK_F2,0}},  // right soft / 3310 Names-C
    [KK_SEND]={"SND",{SDLK_F3,SDLK_HOME,0}},       // green / call
    [KK_END]={"END",{SDLK_ESCAPE,SDLK_F4,0}},      // red / hang-up
    [KK_VOLUP]={"V+",{SDLK_EQUALS,SDLK_KP_PLUS,SDLK_PAGEUP}},
    [KK_VOLDOWN]={"V-",{SDLK_MINUS,SDLK_KP_MINUS,SDLK_PAGEDOWN}},
    // Power: SPECIAL scan, held through boot to clear the power-on gate (auto-released
    // at GUIPWRHOLD); a press-and-HOLD re-asserts it -> firmware powers off, faithfully.
    [KK_PWR]={"PWR",{SDLK_p,0,0}},
};

// Per-family on-screen placement (matches the real phone): PWR above the LCD, the
// nav band + numeric grid below it, and (Family A only) the volume keys stacked to
// the LEFT of the LCD. The matrix (row,col) for each key still comes from the model
// profile; this only decides where the button is drawn. KK_NONE = empty pad cell.
typedef struct {
    const uint8_t* top;            int n_top;     // above the LCD (centered row)
    const uint8_t* side;           int n_side;    // left of the LCD (stacked, Family A)
    const uint8_t (*pad)[GRID_COLS]; int pad_rows; // below the LCD (3-wide grid)
} FamilyLayout;

static const uint8_t TOP_PWR[]   = { KK_PWR };
static const uint8_t SIDE_VOL[]  = { KK_VOLUP, KK_VOLDOWN };

static const uint8_t PAD_3310[][GRID_COLS] = {   // Family B: Menu/UP, C/DOWN, digits
    {KK_SOFT1, KK_UP,   KK_NONE},   // MENU  UP
    {KK_SOFT2, KK_DOWN, KK_NONE},   // C     DOWN
    {KK_1, KK_2, KK_3}, {KK_4, KK_5, KK_6},
    {KK_7, KK_8, KK_9}, {KK_STAR, KK_0, KK_HASH},
};
static const uint8_t PAD_AC[][GRID_COLS] = {     // Family A & C nav band + digits
    {KK_SOFT1, KK_UP,   KK_SOFT2},  // LSK  UP   RSK
    {KK_SEND,  KK_DOWN, KK_END},    // SND  DN   END
    {KK_1, KK_2, KK_3}, {KK_4, KK_5, KK_6},
    {KK_7, KK_8, KK_9}, {KK_STAR, KK_0, KK_HASH},
};

static FamilyLayout family_layout(KeypadFamily fam) {
    FamilyLayout L = {0};
    L.top = TOP_PWR; L.n_top = 1;
    switch (fam) {
        case KP_FAMILY_8210:   // A: volume on the side
            L.side = SIDE_VOL; L.n_side = 2;
            L.pad = PAD_AC; L.pad_rows = (int)(sizeof(PAD_AC)/sizeof(PAD_AC[0]));
            break;
        case KP_FAMILY_3410:   // C: like A but no volume
            L.pad = PAD_AC; L.pad_rows = (int)(sizeof(PAD_AC)/sizeof(PAD_AC[0]));
            break;
        default:               // B: 3310
            L.pad = PAD_3310; L.pad_rows = (int)(sizeof(PAD_3310)/sizeof(PAD_3310[0]));
            break;
    }
    return L;
}

// Fallback matrix for a model profile that carries no keypad lines (legacy, e.g.
// 7110): use the 3310 Family-B mapping so the GUI still works.
static const KeyLine FALLBACK_LINES_3310[] = {
    {KK_1,1,2,0},{KK_2,1,3,0},{KK_3,4,1,0},{KK_4,2,4,0},{KK_5,2,3,0},{KK_6,2,2,0},
    {KK_7,3,4,0},{KK_8,3,3,0},{KK_9,3,2,0},{KK_STAR,4,4,0},{KK_0,0,2,0},{KK_HASH,4,2,0},
    {KK_SOFT1,4,3,0},{KK_SOFT2,0,4,0},{KK_UP,0,1,0},{KK_DOWN,1,1,0},{KK_PWR,0,0,0x02},
};

// Runtime keypad, built per-model at gui_init() from the family layout + model lines.
#define MAX_KEYS  24
static KeyBtn KEYPAD[MAX_KEYS];
static int    KEYPAD_N = 0;
// LCD + diag-panel origins (computed in build_layout from the keypad geometry).
static int    g_lcd_x0 = MARGIN, g_lcd_y0 = MARGIN, g_panel_x0 = 0;
// LCD destination size on screen (classic = lcd*SCALE; shell = the photo's screen cutout,
// which is NOT an integer multiple — the real 3310 LCD has tall pixels). The renderer maps
// LCD (x,y) -> dest with integer bresenham-style division, exact in both modes.
static int    g_lcd_dw = 0, g_lcd_dh = 0;

// --- Device shell (photo body, same assets as the web: web/shells/<model>/) ---------
// The shell definitions mirror web/shells/<model>/shell.js (design-space coordinates);
// zone rects are the bounding boxes of the editor-exported click paths. The PNG photo is
// decoded with the vendored stb_image (no SDL2_image dependency) and the LCD framebuffer
// is blitted into the photo's screen cutout. GUISHELL=0 opts back into the classic grid;
// a missing/unreadable PNG degrades to the classic grid with one stderr note.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG                 // the shells are PNG; trim every other decoder
#define STBI_NO_FAILURE_STRINGS
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"   // trimmed decoders leave dead helpers
#include "../third_party/stb/stb_image.h"
#pragma GCC diagnostic pop

typedef struct { KeyId id; SDL_Rect r; } ShellZone;
typedef struct {
    const char* model;          // ModelProfile.name this shell applies to
    const char* png;            // photo path (repo-relative, shared with the web)
    int         dw, dh;         // design-space size (zone/lcd coords live in this space)
    SDL_Rect    lcd;            // LCD content rect in design space (canvasW/H of shell.js)
    ShellZone   zones[20];
    int         n_zones;
} ShellDef;

static const ShellDef SHELLS[] = {
    { "3310", "web/shells/3310/nokia-3310.png", 384, 875,
      { 70, 242, 252, 166 },     // lcd box (69,235,254,181) inset to the 252x166 canvas
      {
        { KK_PWR,   {104,   0, 153, 20} },
        { KK_SOFT1, {121, 452, 151, 43} },   // Menu (bbox of the editor path anchors)
        { KK_SOFT2, { 61, 472,  72, 70} },   // C
        { KK_UP,    {270, 470,  58, 58} },
        { KK_DOWN,  {213, 506,  63, 55} },
        { KK_1,     { 52, 573,  65, 39} }, { KK_2, {157, 588, 71, 35} }, { KK_3, {268, 571, 65, 39} },
        { KK_4,     { 59, 632,  65, 40} }, { KK_5, {157, 649, 71, 35} }, { KK_6, {265, 632, 68, 41} },
        { KK_7,     { 58, 693,  69, 41} }, { KK_8, {157, 711, 71, 35} }, { KK_9, {258, 694, 68, 40} },
        { KK_STAR,  { 64, 753,  70, 44} }, { KK_0, {157, 773, 71, 35} }, { KK_HASH, {256, 755, 67, 45} },
      }, 17 },
};

static const ShellDef* g_shell = NULL;        // active shell def (NULL = classic grid)
static SDL_Texture*    g_shell_tex = NULL;    // photo texture (created after the renderer)
static unsigned char*  g_shell_px = NULL;     // decoded RGBA pixels (freed post-texture)
static int             g_shell_pw = 0, g_shell_ph = 0;

// Shell display scale (percent). The ShellDef tables stay in canonical design coords
// (matching web/shells/*.js verbatim); every on-screen use goes through shpx(). 75% puts
// the 3310 at 288x656 — life-size-ish on a typical desktop instead of towering.
#define SHELL_PCT 75
static int shpx(int v) { return (v * SHELL_PCT) / 100; }

// Decode the shell photo for this model (before window creation — layout needs the dims).
static void shell_try_load(const struct ModelProfile* prof) {
    g_shell = NULL;
    const char* e = getenv("GUISHELL");
    if ((e && atoi(e) == 0) || !prof || !prof->name) return;
    for (size_t i = 0; i < sizeof(SHELLS)/sizeof(SHELLS[0]); ++i) {
        if (strcmp(SHELLS[i].model, prof->name) != 0) continue;
        int n = 0;
        g_shell_px = stbi_load(SHELLS[i].png, &g_shell_pw, &g_shell_ph, &n, 4);
        if (!g_shell_px) {
            fprintf(stderr, "GUI: shell photo %s not found — classic grid\n", SHELLS[i].png);
            return;
        }
        g_shell = &SHELLS[i];
        return;
    }
}

// Family-aware glyph: Family B relabels the soft keys to the 3310's Menu / C.
static const char* glyph_for(KeypadFamily fam, KeyId id) {
    if (fam == KP_FAMILY_3310) {
        if (id == KK_SOFT1) return "MENU";
        if (id == KK_SOFT2) return "C";
    }
    return KEYMETA[id].glyph;
}

// Find a model's matrix line for a logical key (NULL if the model lacks it).
static const KeyLine* line_for(const KeyLine* lines, int n, KeyId id) {
    for (int i = 0; i < n; ++i) if (lines[i].id == id) return &lines[i];
    return NULL;
}

// Place one logical key at (x,y) with its model matrix line. Returns 1 if added.
static int place_key(const KeyLine* lines, int n, KeypadFamily fam, KeyId id, int x, int y) {
    if (id == KK_NONE || KEYPAD_N >= MAX_KEYS) return 0;
    const KeyLine* kl = line_for(lines, n, id);
    if (!kl) return 0;                       // model lacks this key -> skip
    KeyBtn* b = &KEYPAD[KEYPAD_N++];
    b->label = glyph_for(fam, id);
    b->row = kl->row; b->col = kl->col; b->special = kl->special;
    for (int j = 0; j < MAX_KEYS_PER_BTN; ++j) b->keys[j] = KEYMETA[id].keys[j];
    b->held = 0;
    b->rect = (SDL_Rect){ x, y, KEY_W, KEY_H };
    return 1;
}

// Build the runtime keypad + on-screen geometry (rects, LCD origin, window size).
static void build_layout(const struct ModelProfile* prof) {
    KeypadFamily fam = prof ? prof->keypad.family : KP_FAMILY_3310;
    const KeyLine* lines = (prof && prof->keypad.lines) ? prof->keypad.lines : FALLBACK_LINES_3310;
    int n_lines = (prof && prof->keypad.lines) ? prof->keypad.n_lines
                  : (int)(sizeof(FALLBACK_LINES_3310)/sizeof(FALLBACK_LINES_3310[0]));

    if (g_shell) {
        // Shell mode: the photo IS the layout. Zones give the clickable rects (design
        // coords = window coords, the photo renders at (0,0) dw x dh); the LCD blits into
        // the photo's screen cutout. The diag panel sits to the right of the photo.
        KEYPAD_N = 0;
        for (int i = 0; i < g_shell->n_zones; ++i) {
            const ShellZone* z = &g_shell->zones[i];
            if (place_key(lines, n_lines, fam, z->id, z->r.x, z->r.y))
                KEYPAD[KEYPAD_N - 1].rect = (SDL_Rect){ shpx(z->r.x), shpx(z->r.y),
                                                        shpx(z->r.w), shpx(z->r.h) };
        }
        g_lcd_x0 = shpx(g_shell->lcd.x);  g_lcd_y0 = shpx(g_shell->lcd.y);
        g_lcd_dw = shpx(g_shell->lcd.w);  g_lcd_dh = shpx(g_shell->lcd.h);
        g_panel_x0 = shpx(g_shell->dw) + MARGIN;
        g_win_w = g_panel_x0 + PANEL_W + MARGIN;
        g_win_h = shpx(g_shell->dh);
        if (g_win_h < 360) g_win_h = 360;
        if (getenv("GUIKPDUMP"))
            fprintf(stderr, "GUI keypad: model=%s SHELL (%d zones)\n",
                    prof && prof->name ? prof->name : "?", KEYPAD_N);
        return;
    }

    FamilyLayout L = family_layout(fam);

    int side_w = L.n_side ? (KEY_W + KEY_GAP) : 0;   // left gutter for volume keys
    int top_h  = L.n_top  ? (KEY_H + KEYPAD_TOP) : 0; // header for the power key
    g_lcd_x0 = MARGIN + side_w;
    g_lcd_y0 = MARGIN + top_h;
    int lcd_w = g_lcd_w * SCALE, lcd_h = g_lcd_h * SCALE;
    g_lcd_dw = lcd_w; g_lcd_dh = lcd_h;
    int pad_w = GRID_COLS * KEY_W + (GRID_COLS - 1) * KEY_GAP;

    KEYPAD_N = 0;

    // Top band: PWR centered above the LCD (or above the pad if the pad is wider).
    int top_span = (lcd_w > pad_w ? lcd_w : pad_w);
    int top_tot  = L.n_top * KEY_W + (L.n_top - 1) * KEY_GAP;
    int tx = g_lcd_x0 + (top_span - top_tot) / 2, ty = MARGIN;
    for (int i = 0; i < L.n_top; ++i) { place_key(lines, n_lines, fam, L.top[i], tx, ty); tx += KEY_W + KEY_GAP; }

    // Side band: volume keys stacked to the left, vertically centered on the LCD.
    int side_tot = L.n_side * KEY_H + (L.n_side - 1) * KEY_GAP;
    int sy = g_lcd_y0 + (lcd_h - side_tot) / 2;
    for (int i = 0; i < L.n_side; ++i) { place_key(lines, n_lines, fam, L.side[i], MARGIN, sy); sy += KEY_H + KEY_GAP; }

    // Pad: nav band + numeric grid below the LCD.
    int pad_x0 = g_lcd_x0, pad_y0 = g_lcd_y0 + lcd_h + KEYPAD_TOP;
    for (int r = 0; r < L.pad_rows; ++r)
        for (int c = 0; c < GRID_COLS; ++c)
            place_key(lines, n_lines, fam, (KeyId)L.pad[r][c],
                      pad_x0 + c * (KEY_W + KEY_GAP), pad_y0 + r * (KEY_H + KEY_GAP));

    // Window geometry.
    int lcd_right = g_lcd_x0 + lcd_w, pad_right = pad_x0 + pad_w;
    int content_right = (lcd_right > pad_right ? lcd_right : pad_right);
    g_panel_x0 = content_right + MARGIN;
    g_win_w = g_panel_x0 + PANEL_W + MARGIN;
    int pad_bottom = pad_y0 + L.pad_rows * KEY_H + (L.pad_rows - 1) * KEY_GAP;
    g_win_h = pad_bottom + MARGIN;
    if (g_win_h < 360) g_win_h = 360;

    if (getenv("GUIKPDUMP")) {
        fprintf(stderr, "GUI keypad: model=%s family=%d (%d keys)\n",
                prof && prof->name ? prof->name : "?", (int)fam, KEYPAD_N);
        for (int i = 0; i < KEYPAD_N; ++i) {
            KeyBtn* b = &KEYPAD[i];
            if (b->special) fprintf(stderr, "  %-4s special=0x%02X  @%d,%d\n", b->label, b->special, b->rect.x, b->rect.y);
            else            fprintf(stderr, "  %-4s (r%d,c%d)  @%d,%d\n", b->label, b->row, b->col, b->rect.x, b->rect.y);
        }
    }
}

static int g_paused = 0;
static int g_mouse_btn = -1;                   // KEYPAD[] index the mouse is holding
static long long g_gui_step = 0;               // latest step (for key-edge logging)
static int g_keylog = -1;                      // GUIKEYLOG=1: trace every keypad edge to stderr
// Power-button state (all per-boot; cleared by gui_reset on warm reboot):
static int       g_pwr_released   = 0;         // momentary power-ON press done (one-shot/boot)
static long long g_pwr_release_step = -2;      // GUIPWRHOLD cfg (-2 unread, -1 = hold forever)
static uint32_t  g_pwr_down_ms    = 0;         // SDL_GetTicks at PWR press (0 = not held)
static int       g_reboot_fired   = 0;         // 30s-hold reboot already signalled (one-shot)
static int       g_pwr_edge_down  = 0;         // PWR went down this frame (consumed in gui_frame)
#define PWR_REBOOT_HOLD_MS 30000u              // 30 s hold of PWR -> our own warm reboot

// Apply a press/release of one logical button to the mad2 keypad matrix, exactly
// mirroring src/web/main.c dct3_web_key_raw: set/clear the matrix bit and pulse
// the keypad-wake IRQ on a REAL edge only. Tracks held state so a key cannot get
// stuck (and so a lost KEYUP can be force-released — see focus handling).
static void key_apply(struct Mad2* m, KeyBtn* b, int down) {
    if (b->held == down) return;          // no edge -> no mutation, no IRQ
    b->held = down;
    int row = b->row & 7, col = b->col & 7;
    if (b->special) {                     // power key: special scan, not the matrix
        if (down) {
            m->kbd_special_cols |=  b->special;
            g_pwr_down_ms = SDL_GetTicks(); if (!g_pwr_down_ms) g_pwr_down_ms = 1;
            g_reboot_fired = 0;           // arm the 30s-hold reboot timer for this press
            g_pwr_edge_down = 1;          // power-on-from-off edge (consumed in gui_frame)
        } else {
            m->kbd_special_cols &= (uint8_t)~b->special;
            // Single (tap) vs held classification — purely informational; the firmware
            // itself distinguishes a momentary power edge from a sustained power-off hold.
            uint32_t held_ms = g_pwr_down_ms ? (SDL_GetTicks() - g_pwr_down_ms) : 0;
            g_pwr_down_ms = 0;            // released -> stop the hold timer
            if (g_keylog < 0) g_keylog = getenv("GUIKEYLOG") ? 1 : 0;
            if (g_keylog) fprintf(stderr, "[gui-key] PWR %s press (held %ums)\n",
                                  held_ms < 600 ? "single" : "held", held_ms);
        }
    } else {
        if (down) m->kbd_norm_cols[row] |=  (uint8_t)(1u << col);
        else      m->kbd_norm_cols[row] &= (uint8_t)~(1u << col);
    }
    // Keypad wake (any edge). For a NORMAL matrix key, route through mad2_keypad_irq so
    // 8850-class models also get the matrix interrupt-source bit; the power/special edge
    // stays a plain IRQ0 (it isn't a matrix-source event).
    if (b->special) m->irq_pending |= 0x01;
    else            mad2_keypad_irq(m);
    if (g_keylog < 0) g_keylog = getenv("GUIKEYLOG") ? 1 : 0;
    if (g_keylog) {
        // List every key still held after this edge, so a stuck key (DOWN with no
        // matching UP before a power-off) is visible in the post-run log.
        char held[64]; int n = 0; held[0] = 0;
        for (int i = 0; i < KEYPAD_N; ++i)
            if (KEYPAD[i].held) n += snprintf(held + n, sizeof held - n, "%s%s",
                                              n ? "," : "", KEYPAD[i].label);
        if (b->special)
            fprintf(stderr, "[gui-key] %-4s %-4s  (special) spec=%02X  held={%s}  @step %lld\n",
                    b->label, down ? "DOWN" : "UP", m->kbd_special_cols, held, g_gui_step);
        else
            fprintf(stderr, "[gui-key] %-4s %-4s  (r%d c%d) cols[%d]=%02X  held={%s}  @step %lld\n",
                    b->label, down ? "DOWN" : "UP", row, col, row,
                    m->kbd_norm_cols[row], held, g_gui_step);
    }
}

// Force-release every held key (called on focus loss / window leave so a KEYUP
// that SDL never delivers can't leave a phantom key pressed -> a stuck matrix bit
// would perturb the firmware, the exact non-faithful behaviour we must avoid).
static void keys_release_all(struct Mad2* m) {
    for (int i = 0; i < KEYPAD_N; ++i)
        if (KEYPAD[i].held) key_apply(m, &KEYPAD[i], 0);
}

// Reset per-boot overlay state for an in-process warm reboot. mad2 is re-initialised
// by boot_trace (it re-asserts the boot power-hold), so here we only clear the GUI's
// own latches: held-key flags, the power one-shot, and the PWR hold/reboot timers. The
// GUIPWRHOLD config (g_pwr_release_step) and GUIKEYLOG flag persist across reboots.
void gui_reset(void) {
    for (int i = 0; i < KEYPAD_N; ++i) KEYPAD[i].held = 0;
    g_mouse_btn     = -1;
    g_pwr_released  = 0;
    g_pwr_down_ms   = 0;
    g_reboot_fired  = 0;
    g_pwr_edge_down = 0;
}

// --- text helpers ----------------------------------------------------------
static void draw_glyph(SDL_Renderer* r, int x, int y, char ch, int scale) {
    if (ch < 0x20 || ch > 0x7F) ch = '?';
    const unsigned char* g = FONT8X8[(int)ch - 0x20];
    for (int row = 0; row < 8; ++row) {
        unsigned char bits = g[row];
        for (int col = 0; col < 8; ++col) {
            if (bits & (1u << col)) {
                SDL_Rect px = { x + col * scale, y + row * scale, scale, scale };
                SDL_RenderFillRect(r, &px);
            }
        }
    }
}
static void draw_text(SDL_Renderer* r, int x, int y, const char* s, int scale) {
    for (; *s; ++s) { draw_glyph(r, x, y, *s, scale); x += 8 * scale + scale; }
}

// Humanize a large count into a short K/M/B string (reentrant — caller buf).
static const char* hnum(double v, char* buf, size_t n) {
    if      (v >= 1e9) snprintf(buf, n, "%.2fB", v / 1e9);
    else if (v >= 1e6) snprintf(buf, n, "%.1fM", v / 1e6);
    else if (v >= 1e3) snprintf(buf, n, "%.1fK", v / 1e3);
    else               snprintf(buf, n, "%.0f", v);
    return buf;
}

// --- public API ------------------------------------------------------------
void gui_init(const struct ModelProfile* prof) {
    if (prof) {
        g_lcd_w = prof->lcd.width;
        g_lcd_h = prof->lcd.height;
        g_lcd_banks = prof->lcd.banks;
    }
    // Model-aware: build the keypad (family layout + per-model matrix lines), place
    // all buttons (PWR top, volume on the side for Family A, nav+digits below the LCD),
    // and compute the LCD origin + window size. A shell photo (web/shells/) replaces the
    // classic grid when this model has one (GUISHELL=0 opts out).
    shell_try_load(prof);
    build_layout(prof);

#ifdef DCT3_LICENSEE
    // Confidential-beta watermark: name the licensed tester on launch (also in the
    // window title). A leaked copy is therefore traceable to its recipient.
    fprintf(stderr, "Confidential beta — Licensed to %s. Not for redistribution.\n",
            DCT3_LICENSEE);
#endif
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "GUI: SDL_Init failed: %s\n", SDL_GetError());
        return;
    }
    audio_open();   // best-effort PCM earpiece output (graceful if no device)
    g_win = SDL_CreateWindow(DCT3_WIN_TITLE,
                             SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                             g_win_w, g_win_h, SDL_WINDOW_SHOWN);
    if (!g_win) { fprintf(stderr, "GUI: CreateWindow failed: %s\n", SDL_GetError()); return; }
    g_ren = SDL_CreateRenderer(g_win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!g_ren) g_ren = SDL_CreateRenderer(g_win, -1, SDL_RENDERER_SOFTWARE);
    if (!g_ren) { fprintf(stderr, "GUI: CreateRenderer failed: %s\n", SDL_GetError()); return; }
    if (g_shell && g_shell_px) {
        // Photo -> texture (stb gives tightly-packed RGBA = ABGR8888 little-endian).
        // Linear filtering: the photo renders below native size (SHELL_PCT), and
        // nearest-neighbour downscaling shimmers on the body's edges and print.
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        g_shell_tex = SDL_CreateTexture(g_ren, SDL_PIXELFORMAT_ABGR8888,
                                        SDL_TEXTUREACCESS_STATIC, g_shell_pw, g_shell_ph);
        if (g_shell_tex) {
            SDL_UpdateTexture(g_shell_tex, NULL, g_shell_px, g_shell_pw * 4);
            SDL_SetTextureBlendMode(g_shell_tex, SDL_BLENDMODE_BLEND);
        } else {
            fprintf(stderr, "GUI: shell texture failed (%s) — classic grid\n", SDL_GetError());
            g_shell = NULL;
            build_layout(prof);   // rebuild classic geometry (window is already sized; harmless)
        }
        stbi_image_free(g_shell_px); g_shell_px = NULL;
    }
    fprintf(stderr, "GUI: window %dx%d, LCD %dx%d x%d%s\n",
            g_win_w, g_win_h, g_lcd_w, g_lcd_h, SCALE, g_shell ? " (shell)" : "");
}

GuiInput gui_frame(struct Mad2* m, struct ARMCore* cpu, long long step,
                   const char* halt_reason) {
    GuiInput gi = {0, 0, 0};
    if (!g_ren || !m) { gi.quit = 1; return gi; }
    g_audio_m = m;             // keep the codec resampler's rate source live
    emu_audio_render(m);       // flush the buzzer voice into the PCM ring up to now (held tones
                               // keep streaming; onset was already stamped at the register write)
    g_gui_step = step;

    // REALTIME THROTTLE — hold emulated time to wall-clock, independent of vsync.
    // The driver calls us once per ~1/60 s of EMULATED time and we then present with
    // PRESENTVSYNC; that was the ONLY pacing, on the assumption the compositor honours
    // ~60 Hz vsync. When it does NOT (software renderer, occluded/!focused window, VRR,
    // vsync off), the step loop runs FLAT OUT, so a single physical key-hold spans
    // millions of emulated steps and the firmware's hold-auto-repeat turns one tap into a
    // menu-scroll storm (it re-emits the scroll per animation frame past its release
    // grace — see src/web/main.c). A keyboard/mouse press is asserted on the real key
    // up/down edges, so its faithfulness depends ENTIRELY on emulated time tracking real
    // time. Here we sleep whenever emulated time has run AHEAD of wall-clock, capping it
    // at realtime; vsync (if it throttles) just makes the sleep ~zero. A model that can't
    // reach realtime (5110 DSP co-sim ~0.5x) never sleeps and we re-anchor so a transient
    // catch-up can't bank credit and then sprint. Disabled under headless automation
    // (SDL_VIDEODRIVER=dummy — GUISHOT/CI) and via GUIREALTIME=0.
    {
        static int rt_mode = -1;
        static Uint32 rt_t0 = 0; static uint64_t rt_cyc0 = 0; static int rt_anchored = 0;
        if (rt_mode < 0) {
            const char* drv = getenv("SDL_VIDEODRIVER");
            const char* e   = getenv("GUIREALTIME");
            int headless = drv && !strcmp(drv, "dummy");
            rt_mode = e ? (atoi(e) != 0) : !headless;
        }
        if (rt_mode) {
            Uint32 now = SDL_GetTicks();
            if (!rt_anchored) { rt_t0 = now; rt_cyc0 = m->rtc_mono; rt_anchored = 1; }
            else {
                double emu_ms  = (double)(m->rtc_mono - rt_cyc0) / 13000.0;  // 13 MHz -> ms
                double wall_ms = (double)(Uint32)(now - rt_t0);
                double ahead   = emu_ms - wall_ms;
                if (ahead > 1.0) {                       // ahead of real time: sleep the surplus
                    if (ahead > 250.0) ahead = 250.0;    // clamp one sleep (paranoia)
                    SDL_Delay((Uint32)ahead);
                } else if (ahead < -500.0) {             // >0.5 s behind (co-sim): re-anchor
                    rt_t0 = now; rt_cyc0 = m->rtc_mono;
                }
            }
        }
    }

    // FAITHFUL POWER BUTTON: a real user powers the phone on with a MOMENTARY press —
    // held through the power-on-reason gate (~2M steps) then RELEASED. The boot model
    // holds the virtual power key (kbd_special_cols) so that gate clears; but if it is
    // NEVER released, the firmware reads every later keypress as "a key while POWER is
    // held" and does a clean WDT=0 power-off (~6M steps after the press). So with the
    // button stuck down, ANY on-screen/keyboard key powers the phone off instead of
    // driving the UI. Release it once, just past the gate (mirrors boot_trace KEYRELEASE:
    // clear special cols + pulse the keypad-wake IRQ0). GUIPWRHOLD=<step> overrides the
    // release point; GUIPWRHOLD=-1 holds forever (old behaviour, for A/B).
    if (g_pwr_release_step == -2) {
        const char* e = getenv("GUIPWRHOLD");
        g_pwr_release_step = (e && *e) ? atoll(e) : 5000000;
    }
    if (!g_pwr_released && g_pwr_release_step >= 0 && step >= g_pwr_release_step) {
        m->kbd_special_cols = 0;       // release the virtual power button
        m->irq_pending |= 0x01;        // keypad wake-on-change (ISR 0x2E8C10)
        g_pwr_released = 1;
        if (g_keylog < 0) g_keylog = getenv("GUIKEYLOG") ? 1 : 0;
        if (g_keylog) fprintf(stderr, "[gui-key] power button RELEASED (momentary press done) @step %lld\n", step);
    }
    // OUR OWN REBOOT on a 30-second HOLD of PWR (wall-clock): a hard-reset escape
    // hatch, distinct from (and surviving past) the firmware's own ~0.45s power-off.
    // Fires once per hold; boot_trace acts on gi.reboot with an in-process warm reboot
    // (works from the post-halt wait too, so it resets a powered-off/frozen phone).
    if (g_pwr_down_ms && !g_reboot_fired &&
        (SDL_GetTicks() - g_pwr_down_ms) >= PWR_REBOOT_HOLD_MS) {
        g_reboot_fired = 1;
        gi.reboot = 1;
        if (g_keylog < 0) g_keylog = getenv("GUIKEYLOG") ? 1 : 0;
        if (g_keylog) fprintf(stderr, "[gui-key] PWR held 30s -> REQUESTING WARM REBOOT @step %lld\n", step);
    }
    // POWER ON from the OFF state. When the phone is powered off (firmware wrote CCONT
    // WDT=0, e.g. a service-tool shutdown), boot_trace's post-halt wait calls gui_frame
    // with a non-NULL halt_reason. A real user turns the phone back on with a MOMENTARY
    // power-key tap (not the 30s hard-reset hold) — so any PWR press-edge while off boots
    // it via the warm-reboot path (core + mad2 re-init re-asserts the powerup latch;
    // flash/EEPROM in RAM persist, exactly like a hardware power-on). During a normal run
    // (halt_reason == NULL) a PWR tap is NOT a reboot — it's the firmware's own power key.
    if (halt_reason && g_pwr_edge_down) {
        gi.reboot = 1;
        if (g_keylog < 0) g_keylog = getenv("GUIKEYLOG") ? 1 : 0;
        if (g_keylog) fprintf(stderr, "[gui-key] PWR tap while OFF -> power ON (warm reboot) @step %lld\n", step);
    }
    g_pwr_edge_down = 0;   // consume the power-on edge each frame

    // GUIREBOOTAT=<step>: test hook — force exactly one warm reboot at a step, so the
    // warm-reboot path can be verified headlessly (SDL_VIDEODRIVER=dummy) without input.
    { static long long ra = -2; static int ra_fired = 0;
      if (ra == -2) { const char* e = getenv("GUIREBOOTAT"); ra = (e && *e) ? atoll(e) : -1; }
      if (ra >= 0 && !ra_fired && step >= ra) { ra_fired = 1; gi.reboot = 1; } }

    // --- events: keypad + mouse + control keys ---
    // FAITHFULNESS RULE (Task 4): when no key is pressed the overlay must NOT
    // mutate mad2 state. Keypad bits + irq_pending are touched ONLY on a tracked
    // press/release edge (key_apply), and ALL held keys are force-released on
    // focus loss so a dropped KEYUP can't leave a phantom matrix bit set.
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) { gi.quit = 1; }
        else if (ev.type == SDL_WINDOWEVENT) {
            // Lost keyboard focus or pointer left: release everything held so the
            // matrix returns to idle (0x1F) and the run stays unperturbed.
            if (ev.window.event == SDL_WINDOWEVENT_FOCUS_LOST ||
                ev.window.event == SDL_WINDOWEVENT_LEAVE ||
                ev.window.event == SDL_WINDOWEVENT_MINIMIZED ||
                ev.window.event == SDL_WINDOWEVENT_HIDDEN) {
                if (g_keylog < 0) g_keylog = getenv("GUIKEYLOG") ? 1 : 0;
                if (g_keylog) fprintf(stderr, "[gui-key] window event %d -> force-release @step %lld\n",
                                      ev.window.event, g_gui_step);
                keys_release_all(m);
            }
        }
        else if (ev.type == SDL_KEYDOWN || ev.type == SDL_KEYUP) {
            int down = (ev.type == SDL_KEYDOWN);
            SDL_Keycode k = ev.key.keysym.sym;
            if (down && ev.key.repeat) continue;   // ignore OS autorepeat (one logical press)
            if (down && k == SDLK_ESCAPE) { gi.quit = 1; continue; }
            if (down && k == SDLK_SPACE)  { g_paused = !g_paused; continue; }
            if (down && k == SDLK_r)      { gi.reboot = 1; continue; }
            // keypad matrix edge (mirror src/web/main.c dct3_web_key_raw idiom)
            for (int i = 0; i < KEYPAD_N; ++i) {
                for (int j = 0; j < MAX_KEYS_PER_BTN && KEYPAD[i].keys[j]; ++j)
                    if (KEYPAD[i].keys[j] == k) { key_apply(m, &KEYPAD[i], down); break; }
            }
        }
        else if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
            SDL_Point p = { ev.button.x, ev.button.y };
            for (int i = 0; i < KEYPAD_N; ++i)
                if (SDL_PointInRect(&p, &KEYPAD[i].rect)) {
                    key_apply(m, &KEYPAD[i], 1); g_mouse_btn = i; break;
                }
        }
        else if (ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT) {
            if (g_mouse_btn >= 0) { key_apply(m, &KEYPAD[g_mouse_btn], 0); g_mouse_btn = -1; }
        }
    }
    gi.paused = g_paused;

    // POWER ON FROM OFF: when the phone is halted/powered-off (boot_trace passes the
    // halt reason here; it is NULL during a live run), a single PWR press turns it back
    // on — i.e. our warm reboot. During a LIVE run a PWR tap is just a momentary power
    // edge (the firmware decides), so this only fires in the halted state.
    if (halt_reason && *halt_reason && g_pwr_edge_down) gi.reboot = 1;
    g_pwr_edge_down = 0;   // consume the edge each frame

    // --- render ---
    // Background (panel) charcoal.
    SDL_SetRenderDrawColor(g_ren, 18, 20, 24, 255);
    SDL_RenderClear(g_ren);

    // LCD panel. ON = dark, OFF = light; when the backlight LED is on, the glow
    // colour comes from the EmuHost facade (period-correct per profile: 5210
    // orange, 8250 blue, ...). 0 = no profile colour -> classic yellow-green.
    int lit = emu_led_lcd(m);
    uint32_t glow = emu_led_rgb(m, 0);
    if (!glow) glow = 0x96C878;
    Uint8 off_r = lit ? (Uint8)(glow >> 16)        : 120,
          off_g = lit ? (Uint8)(glow >> 8)         : 130,
          off_b = lit ? (Uint8)glow                : 110;
    Uint8 on_r = 20, on_g = lit ? 40 : 25, on_b = 25;
    // Shell mode: the photo body first — the LCD + key highlights render on top of it.
    if (g_shell && g_shell_tex) {
        SDL_Rect dst = { 0, 0, shpx(g_shell->dw), shpx(g_shell->dh) };
        SDL_RenderCopy(g_ren, g_shell_tex, NULL, &dst);
    }

    int lx = g_lcd_x0, ly = g_lcd_y0;
    SDL_SetRenderDrawColor(g_ren, off_r, off_g, off_b, 255);
    SDL_Rect lcd_bg = { lx, ly, g_lcd_dw, g_lcd_dh };
    SDL_RenderFillRect(g_ren, &lcd_bg);
    SDL_SetRenderDrawColor(g_ren, on_r, on_g, on_b, 255);
    // Shared unpack (mad2_lcd_px): DDRAM bit + the display-control transform —
    // the old inline unpack skipped blank/all-on/inverse, rendering stale DDRAM
    // during screen transitions (web/main.js applied it; the GUI didn't).
    // Dest mapping by integer division: exact in classic mode (dw = w*SCALE) and
    // handles the shell cutout's non-integer scale (the 3310's tall LCD pixels).
    for (int y = 0; y < g_lcd_h; ++y) {
        int dy0 = (y * g_lcd_dh) / g_lcd_h, dy1 = ((y + 1) * g_lcd_dh) / g_lcd_h;
        for (int x = 0; x < g_lcd_w; ++x) {
            if (mad2_lcd_px(m, x, y)) {
                int dx0 = (x * g_lcd_dw) / g_lcd_w, dx1 = ((x + 1) * g_lcd_dw) / g_lcd_w;
                SDL_Rect px = { lx + dx0, ly + dy0, dx1 - dx0, dy1 - dy0 };
                SDL_RenderFillRect(g_ren, &px);
            }
        }
    }

    // --- on-screen keypad (clickable; same matrix as the keyboard) ---
    if (g_shell) {
        // Shell mode: the photo already shows the keys — only mark a HELD key, with a
        // soft translucent highlight over its zone.
        SDL_SetRenderDrawBlendMode(g_ren, SDL_BLENDMODE_BLEND);
        for (int i = 0; i < KEYPAD_N; ++i) {
            if (!KEYPAD[i].held) continue;
            SDL_SetRenderDrawColor(g_ren, 255, 235, 130, 90);
            SDL_RenderFillRect(g_ren, &KEYPAD[i].rect);
        }
        SDL_SetRenderDrawBlendMode(g_ren, SDL_BLENDMODE_NONE);
    } else
    for (int i = 0; i < KEYPAD_N; ++i) {
        SDL_Rect* rc = &KEYPAD[i].rect;
        // button face: brighter when held
        if (KEYPAD[i].held) SDL_SetRenderDrawColor(g_ren, 90, 130, 90, 255);
        else                SDL_SetRenderDrawColor(g_ren, 46, 50, 58, 255);
        SDL_RenderFillRect(g_ren, rc);
        SDL_SetRenderDrawColor(g_ren, 90, 96, 104, 255);   // border
        SDL_RenderDrawRect(g_ren, rc);
        // centered label (8x8 font, scale 2)
        const int kfs = 2;
        int lw = (int)strlen(KEYPAD[i].label) * (8 * kfs + kfs) - kfs;
        int tx = rc->x + (rc->w - lw) / 2;
        int ty = rc->y + (rc->h - 8 * kfs) / 2;
        SDL_SetRenderDrawColor(g_ren, 220, 228, 236, 255);
        draw_text(g_ren, tx, ty, KEYPAD[i].label, kfs);
    }

    // Diag panel: right of the LCD/keypad content (origin computed in build_layout).
    int px0 = g_panel_x0;
    int py = MARGIN;
    const int fs = 2;                 // font scale
    const int lh = 8 * fs + 4;        // line height
    char line[128], hb[24];
    // True cycle total is mad2.rtc_mono (cpu->cycles is rebased/wrapped to dodge 32-bit
    // overflow, so it is NOT the running total). Derive emulated time from it.
    unsigned long long cyc = (unsigned long long)m->rtc_mono;
    double secs = (double)m->rtc_mono / 13.0e6;
    double heap_used = heap_shadow_used_d(m);

    SDL_SetRenderDrawColor(g_ren, 210, 220, 230, 255);
    draw_text(g_ren, px0, py, "DCT3 BOOT-TRACE GUI", fs); py += lh + 4;

    snprintf(line, sizeof line, "step %s", hnum((double)step, hb, sizeof hb));
    draw_text(g_ren, px0, py, line, fs); py += lh;
    snprintf(line, sizeof line, "cyc  %s", hnum((double)cyc, hb, sizeof hb));
    draw_text(g_ren, px0, py, line, fs); py += lh;
    snprintf(line, sizeof line, secs >= 60.0 ? "time %.1fm" : "time %.1fs",
             secs >= 60.0 ? secs / 60.0 : secs);
    draw_text(g_ren, px0, py, line, fs); py += lh;
    snprintf(line, sizeof line, "fiqs %s", hnum((double)m->fiqs_raised, hb, sizeof hb));
    draw_text(g_ren, px0, py, line, fs); py += lh;
    snprintf(line, sizeof line, "irqs %s", hnum((double)m->irqs_raised, hb, sizeof hb));
    draw_text(g_ren, px0, py, line, fs); py += lh;
    snprintf(line, sizeof line, "heap %s", hnum(heap_used, hb, sizeof hb));
    draw_text(g_ren, px0, py, line, fs); py += lh;
    snprintf(line, sizeof line, "lcd  %s", hnum((double)m->lcd_data_writes, hb, sizeof hb));
    draw_text(g_ren, px0, py, line, fs); py += lh + 4;

    // Output-device state row: backlight / keypad-LED / buzzer / vibra. Each glyph lights
    // green when its HAL line is asserted (emu_host facade), so every modelled output is
    // visible at a glance (the LCD panel itself also tints for the backlight). The buzzer
    // adds its divider-derived tone Hz; vibra shows when the ringer motor line is driven.
    { int has_kl = !(m->model && m->model->led.no_kbd_led);   // model has a keypad LED?
      int has_vb = !(m->model && m->model->led.no_vibra);     // model has a vibra motor?
      struct { const char* l; int on; int show; } outs[] = {
        { "BL", emu_led_lcd(m),   1      },   // LCD/EL backlight
        { "KL", emu_led_kbd(m),   has_kl },   // keypad LED (absent on the serial-keypad 5110)
        { "BZ", emu_buzzer_on(m), 1      },   // PWM buzzer/ringer
        { "VB", emu_vibra_on(m),  has_vb },   // vibra motor (absent on the 5110 NSE-1)
      };
      int ox = px0;
      for (int i = 0; i < 4; ++i) {
          if (!outs[i].show) continue;        // omit a glyph the model physically lacks
          if (outs[i].on) SDL_SetRenderDrawColor(g_ren, 120, 220, 120, 255);
          else            SDL_SetRenderDrawColor(g_ren,  70,  76,  84, 255);
          draw_text(g_ren, ox, py, outs[i].l, fs);
          ox += 3 * (8 * fs + fs);
      }
      py += lh;
      if (emu_buzzer_on(m) && emu_buzzer_div(m)) {
          SDL_SetRenderDrawColor(g_ren, 150, 200, 150, 255);
          snprintf(line, sizeof line, "buzz %uHz", 13000000u / emu_buzzer_div(m));
          draw_text(g_ren, px0, py, line, fs);
      }
      py += lh + 4;
    }

    if (g_paused) {
        SDL_SetRenderDrawColor(g_ren, 255, 220, 80, 255);
        draw_text(g_ren, px0, py, "** PAUSED **", fs);
    } else {
        SDL_SetRenderDrawColor(g_ren, 120, 200, 120, 255);
        draw_text(g_ren, px0, py, "running", fs);
    }
    py += lh + 4;

    SDL_SetRenderDrawColor(g_ren, 150, 160, 170, 255);
    draw_text(g_ren, px0, py, "click keypad", fs); py += lh;
    draw_text(g_ren, px0, py, "or kbd keys", fs);  py += lh;
    draw_text(g_ren, px0, py, "SPACE pause", fs);  py += lh;
    draw_text(g_ren, px0, py, "R reboot", fs);     py += lh;
    draw_text(g_ren, px0, py, "ESC quit", fs);     py += lh + 6;

    if (halt_reason && *halt_reason) {
        // Task 3: show boot_trace's ACTUAL reason string (no heuristic), and
        // colour-code by severity. A clean power-off (CCONT WDT=0) is a normal
        // run-end, NOT a fault — render it neutral; everything else (wild-PC,
        // CCONT-reset, spin, harness fault) is a red fault. We classify purely
        // from the reason text the harness produced (see src/harness/fault.c),
        // never re-deriving it from emulator state.
        int clean = (strstr(halt_reason, "POWER-OFF") != NULL) ||
                    (strstr(halt_reason, "powered off") != NULL) ||
                    (strstr(halt_reason, "budget") != NULL) ||
                    (strstr(halt_reason, "window closed") != NULL);
        if (clean) {
            SDL_SetRenderDrawColor(g_ren, 150, 200, 150, 255);   // neutral/green
            draw_text(g_ren, px0, py, "STOPPED:", fs);
        } else {
            SDL_SetRenderDrawColor(g_ren, 255, 70, 70, 255);     // red fault
            draw_text(g_ren, px0, py, "FAULT:", fs);
        }
        py += lh;
        // wrap the reason at ~16 chars per line
        char buf[64]; size_t n = strlen(halt_reason);
        for (size_t i = 0; i < n; i += 16) {
            size_t len = n - i < 16 ? n - i : 16;
            memcpy(buf, halt_reason + i, len); buf[len] = 0;
            draw_text(g_ren, px0, py, buf, fs); py += lh;
        }
        // step + cyc, so the halt context is fully readable on-screen.
        SDL_SetRenderDrawColor(g_ren, 200, 205, 215, 255);
        snprintf(line, sizeof line, "@step %s", hnum((double)step, hb, sizeof hb));
        draw_text(g_ren, px0, py, line, fs); py += lh;
        snprintf(line, sizeof line, "@%s (%.1fs)", hnum((double)cyc, hb, sizeof hb), secs);
        draw_text(g_ren, px0, py, line, fs); py += lh;
    }

    // GUISHOT=<step>: capture the rendered window to a BMP once at that step (default
    // 6M when set to "1"). Lets the GUI be screenshotted headlessly / when the display
    // is locked (SDL_RenderReadPixels grabs the backbuffer regardless of visibility),
    // then converted to PNG and viewed. GUISHOT_OUT overrides the path.
    {
        static long long shot_at = -2; static int shot_done = 0;
        if (shot_at == -2) {
            const char* e = getenv("GUISHOT");
            shot_at = (e && *e) ? atoll(e) : -1;
            if (shot_at == 1) shot_at = 6000000;
        }
        if (shot_at >= 0 && !shot_done && step >= shot_at) {
            int w = 0, h = 0; SDL_GetRendererOutputSize(g_ren, &w, &h);
            SDL_Surface* s = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_ARGB8888);
            if (s && SDL_RenderReadPixels(g_ren, NULL, SDL_PIXELFORMAT_ARGB8888, s->pixels, s->pitch) == 0) {
                const char* out = getenv("GUISHOT_OUT"); if (!out || !*out) out = "/tmp/gui_shot.bmp";
                SDL_SaveBMP(s, out);
                fprintf(stderr, "GUI: screenshot @step %lld -> %s\n", step, out);
            }
            if (s) SDL_FreeSurface(s);
            shot_done = 1;
        }
    }

    SDL_RenderPresent(g_ren);
    // When paused/halt-wait, don't spin the CPU at 100% — a small delay is fine
    // (boot_trace calls us in a tight wait-loop in those states).
    if (g_paused || (halt_reason && *halt_reason)) SDL_Delay(16);
    return gi;
}

void gui_shutdown(void) {
    if (g_audio_dev) { SDL_CloseAudioDevice(g_audio_dev); g_audio_dev = 0; }
    if (g_ren) { SDL_DestroyRenderer(g_ren); g_ren = NULL; }
    if (g_win) { SDL_DestroyWindow(g_win); g_win = NULL; }
    SDL_Quit();
}

#endif // DCT3_SDL
