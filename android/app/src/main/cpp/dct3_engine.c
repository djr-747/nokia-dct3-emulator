// DCT3 Android engine — see dct3_engine.h. Boot sequence mirrors
// nokia-dct3-emulator/src/web/main.c's dct3_web_boot(); the per-step loop mirrors
// web_step_once() / dct3_web_run_cycles() trimmed to the non-debug essentials; the key
// primitive mirrors dct3_web_key_logical_raw() / tools/gui_sdl.c's key_apply() — a real
// matrix edge per Android KeyEvent, no artificial auto-release timer (that timer exists
// in the web driver only because a browser click can't carry real down/up timing; a
// physical KeyEvent already can). Per the project's "no reimplementation" convention,
// fault-detection + reset-recovery are NOT redone here — they go through the shared
// src/harness/ orchestrator exactly like the web and native (boot_trace/SDL) drivers.

#include "dct3_engine.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "core/dct3_core.h"
#include "mad2/mad2.h"
#include "mad2/emu_host.h"
#include "models/model.h"
#include "harness/harness.h"

#ifdef __ANDROID__
#include <android/log.h>
#define LOG_TAG "dct3engine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...) fprintf(stderr, "[dct3engine] " __VA_ARGS__)
#define LOGE(...) fprintf(stderr, "[dct3engine] " __VA_ARGS__)
#endif

// Declared (not exported via any public header) exactly as src/web/main.c and
// tools/boot_trace.c do — the vendored ARM core's interrupt entry points.
void ARMRaiseIRQ(struct ARMCore* cpu);
void ARMRaiseFIQ(struct ARMCore* cpu);

// The core periodically rebases cpu->cycles down by this amount (see DCT3_EVENT_SLICE
// in src/core/dct3_core.c) to keep the signed int32 counter from overflowing; undo it
// here when summing true elapsed cycles. Same constant the web driver uses.
#define DCT3_CYCLE_REBASE 0x20000000

// The power key is held from mad2_init() (models the user holding PWR through a cold
// boot) so the power-on-reason gate sees it; auto-release after the gate has settled,
// same window + rationale as src/web/main.c's PWR_RELEASE_STEP.
#define PWR_RELEASE_STEP 12000000LL

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

// --- PCM audio (mad2's shared HAL pcm_sink -> a ring the UI thread drains) --------
// mad2/emu_audio.c's unified mixer renders the PWM buzzer + the DSP HLE tone (keypad
// beeps, ringtones — the 3410 has no C54x codec, so this is the only audio source)
// into ch1 (earpiece) samples at EMU_AUDIO_HZ (48 kHz), driven off m->rtc_mono, which
// mad2_timers_tick already advances every step. All ring access here happens either
// from inside a g_lock-held region (the producer, called from step_once) or takes
// g_lock itself (the consumer), so no separate lock is needed.
#define AUDIO_RING_SZ  16384u          // power of two; ~0.34 s at 48 kHz
#define AUDIO_RING_MASK (AUDIO_RING_SZ - 1u)
static int16_t  g_audio_ring[AUDIO_RING_SZ];
static unsigned g_audio_head = 0, g_audio_tail = 0;

static void pcm_sink_cb(struct Mad2* m, int ch, int16_t sample) {
    (void)m;
    if (ch != 1) return;   // earpiece channel only, matches every other frontend
    unsigned head = g_audio_head;
    if ((unsigned)(head - g_audio_tail) >= AUDIO_RING_SZ - 1u) return;   // full -> drop
    g_audio_ring[head & AUDIO_RING_MASK] = sample;
    g_audio_head = head + 1u;
}

static DCT3Core*            g_core   = NULL;
static Mad2                 g_mad2;
static const ModelProfile*  g_model  = NULL;
static HarnessConfig        g_hc;
static uint64_t             g_cycles64 = 0;
static int64_t              g_step     = 0;
static int                  g_booted   = 0;
static int                  g_faulted  = 0;
static int                  g_pwr_auto     = 1;
static int                  g_pwr_released = 0;

// True once the firmware has cleanly powered off AND we're past the boot-settle
// window — mirrors src/web/main.c's web_parked(). Caller must hold g_lock.
static int engine_parked(void) {
    return g_mad2.power_off && g_step > PWR_RELEASE_STEP;
}

// One emulated instruction plus per-step housekeeping (timer tick, FIQ/IRQ delivery,
// true-cycle accounting, the power-key auto-release) — mirrors web_step_once() with
// every debug/trace/inject knob stripped out. Caller must hold g_lock.
static void step_once(void) {
    struct ARMCore* cpu = &g_core->cpu;

    HarnessStatus hs = harness_observe(&g_hc, &g_mad2, g_core, g_step);
    if (hs == HARNESS_FAULT_HALT) {
        harness_fault_report(&g_hc, &g_mad2, g_core, g_hc.fault_reason);
        LOGE("FAULT HALT: %s\n%s", g_hc.fault_reason ? g_hc.fault_reason : "?", g_mad2.postmortem);
        g_faulted = 1;
        return;
    }

    int32_t cyc_before = cpu->cycles;
    dct3_step(g_core);
    int32_t cyc_delta = cpu->cycles - cyc_before;
    if (cyc_delta < 0) cyc_delta += DCT3_CYCLE_REBASE;   // undo the periodic rebase
    g_cycles64 += (uint32_t)cyc_delta;
    mad2_timers_tick(&g_mad2, (uint32_t)cpu->cycles);

    // Canonical ordering (converged on in src/web/main.c): apply a pending recover
    // AFTER the step + tick, BEFORE the FIQ/IRQ raise.
    harness_post_step(&g_hc, &g_mad2, g_core);

    int raised = 0;
    if (!cpu->cpsr.f) {
        int fiq = mad2_fiq_poll(&g_mad2);
        if (fiq >= 0) { ARMRaiseFIQ(cpu); g_mad2.fiqs_raised++; raised = 1; }
    }
    if (!raised && !cpu->cpsr.i) {
        int irq = mad2_irq_poll(&g_mad2);
        if (irq >= 0) { ARMRaiseIRQ(cpu); g_mad2.irqs_raised++; }
    }

    if (g_pwr_auto && !g_pwr_released && g_step >= PWR_RELEASE_STEP) {
        g_mad2.kbd_special_cols = 0;   // release the held power key
        g_mad2.irq_pending |= 0x01;    // keypad IRQ so the up-edge is seen
        g_pwr_released = 1;
    }
    g_step++;
}

int dct3and_boot(const char* path) {
    pthread_mutex_lock(&g_lock);

    if (!g_core) g_core = dct3_core_create();
    else         dct3_core_reset(g_core);
    if (!g_core) { pthread_mutex_unlock(&g_lock); return -4; }
    // Cold power-on: a reboot must start from the same zero RAM state as the very
    // first boot (dct3_core_create callocs to zero) — else boot-gating flags that
    // should read 0 are stale and the firmware never comes up.
    memset(g_core->ram, 0, DCT3_RAM_SIZE);

    FILE* f = fopen(path, "rb");
    if (!f) { pthread_mutex_unlock(&g_lock); return -1; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n <= 0) { fclose(f); pthread_mutex_unlock(&g_lock); return -2; }
    uint8_t* b = (uint8_t*)malloc((size_t)n);
    if (!b || fread(b, 1, (size_t)n, f) != (size_t)n) {
        free(b); fclose(f); pthread_mutex_unlock(&g_lock); return -3;
    }
    fclose(f);

    g_model = model_default();   // DCT3_MODEL_3410_ONLY registry -> always the 3410
    dct3_write_bytes(g_core, g_model->mem.flash_base, b, (size_t)n);
    free(b);
    dct3_fix_mcu_checksum(g_core);

    memset(&g_mad2, 0, sizeof g_mad2);
    mad2_init(&g_mad2, g_model);
    // mad2_init() already defaults sim_present=1 with a synthetic ISO-7816/GSM-11.11
    // SIM (genuine ATR, SELECT/GET_RESPONSE/READ, A3/A8 auth — see src/mad2/mad2_sim.c)
    // and sim_pin_enabled=0 (no PIN prompt) — leave that alone so the phone actually
    // reaches standby/menus. (Only src/web/main.c's browser driver forces sim_present=0
    // for anonymous first-time visitors; that choice doesn't apply to a real device.)
    g_mad2.mem = g_core->ram;
    g_mad2.mem_mask = DCT3_RAM_MASK;
    g_mad2.pcm_sink = pcm_sink_cb;
    g_audio_head = g_audio_tail = 0;

    model_resolve(g_model, g_core->ram, DCT3_RAM_MASK, &g_mad2.fw);

    dct3_set_io_hooks(g_core, &g_mad2, g_model->mem.io_lo, g_model->mem.io_hi, mad2_read, mad2_write);
    dct3_set_io_range2(g_core, g_model->mem.flash_base,
                       g_model->mem.flash_base + g_model->mem.flash_size);
    dct3_boot_at(g_core, g_model->mem.boot_entry);

    g_step = 0;
    g_cycles64 = 0;
    g_pwr_auto = 1;
    g_pwr_released = 0;
    g_faulted = 0;

    memset(&g_hc, 0, sizeof g_hc);
    // recovery=1 (mirrors the web default — the phone is a live device here, so it
    // should warm-recover a recoverable reset rather than halting); wild_pc_after=16
    // steps (the boot-ROM HLE de-pipe warmup, same as every other driver).
    harness_init(&g_hc, &g_mad2, /*recovery=*/1, /*spin_limit=*/0, /*wild_pc_after=*/16);

    g_booted = 1;
    LOGI("booted model=%s flash=%ld bytes lcd=%dx%d", g_model->name, n,
         g_model->lcd.width, g_model->lcd.height);

    pthread_mutex_unlock(&g_lock);
    return 0;
}

void dct3and_run_cycles(int n_cycles) {
    pthread_mutex_lock(&g_lock);
    if (!g_booted || g_faulted || n_cycles <= 0) { pthread_mutex_unlock(&g_lock); return; }
    uint64_t until = g_cycles64 + (uint64_t)n_cycles;
    // Safety cap, same rationale/value as dct3_web_run_cycles: a pathological low-cpi
    // tight loop shouldn't spin the batch unbounded.
    int guard = 20000000;
    while (g_cycles64 < until && guard-- > 0 && !g_faulted && !engine_parked())
        step_once();
    // Flush the buzzer/tone mixer up to the current rtc_mono (mirrors dct3_web_run /
    // dct3_web_run_cycles calling emu_audio_render once per batch — the buzzer's own
    // onset is already sample-accurate from being flushed at every register write
    // inside mad2, this just catches a tone that's still playing).
    emu_audio_render(&g_mad2);
    pthread_mutex_unlock(&g_lock);
}

int dct3and_read_audio(int16_t* out, int max_samples) {
    pthread_mutex_lock(&g_lock);
    unsigned avail = g_audio_head - g_audio_tail;
    unsigned n = (unsigned)max_samples < avail ? (unsigned)max_samples : avail;
    for (unsigned i = 0; i < n; i++) out[i] = g_audio_ring[(g_audio_tail + i) & AUDIO_RING_MASK];
    g_audio_tail += n;
    pthread_mutex_unlock(&g_lock);
    return (int)n;
}

int dct3and_audio_rate(void) {
    return EMU_AUDIO_HZ;   // the HLE mixer's fixed output rate (mad2.h)
}

int dct3and_lcd_width(void) {
    pthread_mutex_lock(&g_lock);
    int w = g_model ? g_model->lcd.width : 0;
    pthread_mutex_unlock(&g_lock);
    return w;
}

int dct3and_lcd_height(void) {
    pthread_mutex_lock(&g_lock);
    int h = g_model ? g_model->lcd.height : 0;
    pthread_mutex_unlock(&g_lock);
    return h;
}

void dct3and_render_pixels(uint32_t* out, int out_len, uint32_t on_argb, uint32_t off_argb) {
    pthread_mutex_lock(&g_lock);
    if (!g_booted || !g_model || !out) { pthread_mutex_unlock(&g_lock); return; }
    int w = g_model->lcd.width, h = g_model->lcd.height;
    if (out_len < w * h) { pthread_mutex_unlock(&g_lock); return; }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            out[y * w + x] = mad2_lcd_px(&g_mad2, x, y) ? on_argb : off_argb;
    pthread_mutex_unlock(&g_lock);
}

int dct3and_key_event(int key_id, int down) {
    pthread_mutex_lock(&g_lock);
    if (!g_booted || !g_model) { pthread_mutex_unlock(&g_lock); return 0; }
    const KeyLine* k = emu_keyline(g_model, key_id);
    if (!k) { pthread_mutex_unlock(&g_lock); return 0; }   // key not present on the 3410
    if (k->special) {
        emu_key_special(&g_mad2, k->special, down);   // PWR-class special-scan
    } else {
        int row = k->row & 7, col = k->col & 7;
        if (down) g_mad2.kbd_norm_cols[row] |=  (uint8_t)(1u << col);
        else      g_mad2.kbd_norm_cols[row] &= (uint8_t)~(1u << col);
        mad2_keypad_irq(&g_mad2);
    }
    pthread_mutex_unlock(&g_lock);
    return 1;
}

int dct3and_is_powered_off(void) {
    pthread_mutex_lock(&g_lock);
    int off = g_booted && engine_parked();
    pthread_mutex_unlock(&g_lock);
    return off;
}

int dct3and_is_faulted(void) {
    pthread_mutex_lock(&g_lock);
    int f = g_faulted;
    pthread_mutex_unlock(&g_lock);
    return f;
}

void dct3and_shutdown(void) {
    pthread_mutex_lock(&g_lock);
    if (g_core) { dct3_core_destroy(g_core); g_core = NULL; }
    g_booted = 0;
    g_model = NULL;
    pthread_mutex_unlock(&g_lock);
}
