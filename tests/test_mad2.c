// MAD2 device-model unit tests.
//
// The mad2.c split into per-subsystem TUs (mad2_<sub>.c) made the device model
// directly drivable WITHOUT the mGBA ARM core: build a Mad2 + a real ModelProfile,
// poke registers through each peripheral's entry points, and assert on the result.
// These run in milliseconds and pin per-subsystem behaviour (flash CFI FSM, CCONT
// RTC/ADC, PCD8544 addressing, keypad scan, MBUS FIFO) so a refactor that changes a
// state transition fails HERE with a precise message, long before a boot regression.
//
// Complements the coarse boot golden-master (tools/.. + the regression check): this
// is the fine-grained per-module layer.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "mad2/mad2.h"
#include "mad2/mad2_internal.h"   // ccont_*, flash_*, lcd_*, kbd_cols, mbus_rx_*, wdt_kick
#include "mad2/emu_host.h"        // emu_audio_render — unified buzzer PCM mixer
#include "models/model.h"          // model_by_name

static int g_pass, g_fail;
#define CHECK(desc, cond) do { if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL %s\n", desc); } } while (0)
#define CHECK_EQ(desc, got, want) do { \
    uint64_t G=(uint64_t)(got), W=(uint64_t)(want); \
    if (G==W) { g_pass++; } \
    else { g_fail++; printf("  FAIL %-44s got=0x%llX want=0x%llX\n", desc, \
            (unsigned long long)G, (unsigned long long)W); } } while (0)

// --- shared test fixture ----------------------------------------------------
#define TEST_RAM_SIZE 0x00400000u          // 4 MiB (covers 3310 flash + eeprom @0x3D0000)
static Mad2 g_m;
static uint8_t* g_mem;

// Fresh device on the 3310 profile, RAM-backed. mad2_init memsets + sets the boot
// defaults (adc, masks, flash_vpp=1, recovery policy, ...); we bind RAM afterwards.
static const ModelProfile* fixture(void) {
    const ModelProfile* prof = model_by_name("3310");
    if (!prof) { printf("  FATAL: model_by_name(3310) returned NULL\n"); exit(2); }
    mad2_init(&g_m, prof);
    if (!g_mem) g_mem = calloc(1, TEST_RAM_SIZE);
    memset(g_mem, 0, TEST_RAM_SIZE);
    g_m.mem      = g_mem;
    g_m.mem_mask = TEST_RAM_SIZE - 1u;
    return prof;
}

// CCONT is a two-byte transaction: an address byte (reg# in bits 7:3) then a value.
static void ccont_sel(uint8_t reg)            { ccont_byte(&g_m, (uint8_t)(reg << 3)); }
static void ccont_write_reg(uint8_t reg, uint8_t v) { ccont_sel(reg); ccont_byte(&g_m, v); }
static uint8_t ccont_read_reg(uint8_t reg)    { ccont_sel(reg); return ccont_read(&g_m); }

// --- Flash: Intel/Sharp CFI command FSM -------------------------------------
static void test_flash(void) {
    printf("[flash] Intel/Sharp CFI command FSM\n");
    const ModelProfile* p = fixture();
    uint32_t ee = p->mem.eeprom_base;            // 0x3D0000 — the FFS/EEPROM partition

    // read-ID (0x90): mfr=Intel 0x0089 @offset0, device 0x8890 @offset2 (the 3410's
    // flash-chip detector requires this or it installs a no-op driver).
    flash_write(&g_m, ee, 0x90, 2);
    CHECK_EQ("read-id mfr",    flash_read(&g_m, ee + 0, 2, 0), 0x0089);
    CHECK_EQ("read-id device", flash_read(&g_m, ee + 2, 2, 0), 0x8890);
    // read-array reset (0xFF): reads return the array (ram_value) again.
    flash_write(&g_m, ee, 0xFF, 2);
    CHECK_EQ("read-array passthrough", flash_read(&g_m, ee + 0, 2, 0xBEEF), 0xBEEF);

    // program: NOR can only clear 1->0, big-endian, Vpp must be on (init set it).
    g_m.mem[ee & g_m.mem_mask] = 0xFF;
    flash_write(&g_m, ee, 0x40, 1);              // program-setup
    flash_write(&g_m, ee, 0x0F, 1);              // data
    CHECK_EQ("program ANDs (0xFF & 0x0F)", g_m.mem[ee & g_m.mem_mask], 0x0F);
    CHECK_EQ("program counted", g_m.flash_programs, 1);
    // busy->ready edge: SR.7 low for FLASH_BUSY_POLLS reads, then high.
    CHECK("busy poll 1 not-ready", (flash_read(&g_m, ee, 1, 0) & 0x80) == 0);
    CHECK("busy poll 2 not-ready", (flash_read(&g_m, ee, 1, 0) & 0x80) == 0);
    CHECK("then ready (SR.7)",     (flash_read(&g_m, ee, 1, 0) & 0x80) == 0x80);

    // Vpp gate: with Vpp off a program FAILS (SR.3 set) and the array is unchanged.
    flash_write(&g_m, ee, 0xFF, 1);
    g_m.mem[ee & g_m.mem_mask] = 0xFF;
    g_m.flash_vpp = 0; g_m.flash_sr = 0;
    flash_write(&g_m, ee, 0x40, 1);
    flash_write(&g_m, ee, 0x00, 1);
    CHECK_EQ("Vpp-off array unchanged", g_m.mem[ee & g_m.mem_mask], 0xFF);
    CHECK("Vpp-off sets SR.3", (g_m.flash_sr & 0x08) != 0);
    g_m.flash_vpp = 1;

    // erase: 0x20 setup + 0xD0 confirm wipes the 64 KB block to 0xFF.
    g_m.mem[ee & g_m.mem_mask] = 0x00;
    flash_write(&g_m, ee, 0x20, 1);
    flash_write(&g_m, ee, 0xD0, 1);
    CHECK_EQ("erase -> 0xFF", g_m.mem[ee & g_m.mem_mask], 0xFF);
    // (the web-only EEPROM write-block gate was removed 2026-06-19 — it dropped the
    // firmware's FFS init writes; flash now always programs, matching native/HW.)
}

// --- CCONT: power/RTC/ADC controller ----------------------------------------
static void test_ccont(void) {
    printf("[ccont] power/RTC/ADC + watchdog\n");
    fixture();

    // chip-ID self-test: reg3 high bits must satisfy (v & 0xFC) == 0xB0 or boot
    // shows CONTACT SERVICE.
    CHECK_EQ("reg3 chip-ID & 0xFC", ccont_read_reg(0x03) & 0xFC, 0xB0);

    // ADC: reg0 enables A/D + selects a channel; reg2 returns that channel's value.
    ccont_write_reg(0x00, 0x08 | (2u << 4));      // enable + channel 2 (Vbatt)
    CHECK_EQ("ADC ch2 low byte (vbatt 0x220)", ccont_read_reg(0x02), 0x20);

    // RTC per-counter write: writing the DAY counter must NOT zero the minute (the
    // clock-freeze bug — independent free-running counters + sub-second phase).
    ccont_write_reg(0x08, 45);                     // minute = 45
    ccont_write_reg(0x0A, 0);                      // day reset = 0
    CHECK_EQ("minute survives day-reset", ccont_read_reg(0x08), 45);

    // reg5 = 0x00 is ccont_poweroff (clean shutdown), not a kick.
    ccont_write_reg(0x05, 0x00);
    CHECK("reg5=0 -> power_off", g_m.power_off == 1);

    // watchdog kick table: 0x3F disables (pauses starvation), 0x31 re-arms.
    wdt_kick(&g_m, 0x3F, 0); CHECK("wdt 0x3F disables", g_m.wdt_disabled == 1);
    wdt_kick(&g_m, 0x31, 0); CHECK("wdt 0x31 re-arms",  g_m.wdt_disabled == 0);
    CHECK_EQ("wdt window = value * ARM_HZ", g_m.wdt_window_cyc, 0x31ull * DCT3_ARM_HZ);
}

// --- PCD8544 LCD controller -------------------------------------------------
static void test_lcd(void) {
    printf("[lcd] PCD8544 controller + framebuffer\n");
    const ModelProfile* p = fixture();
    int w = p->lcd.width, banks = p->lcd.banks;

    lcd_command(&g_m, 0x20);                       // function-set: basic (H=0,V=0)
    lcd_command(&g_m, 0x0C);                       // display control: D=1,E=0 -> normal
    CHECK_EQ("display mode normal", g_m.lcd_mode, 2);

    // set-Y bank: the 4-bit mask must NOT alias bank 8 to bank 0 (top-row flicker
    // bug). Bank 8 clamps to banks-1, it does not wrap to 0.
    lcd_command(&g_m, 0x48);                       // set-Y bank 8
    CHECK_EQ("bank-8 clamps (no alias)", g_m.lcd_y, banks - 1);

    // set-X col: clamps to width-1.
    lcd_command(&g_m, 0x80 | 90);                  // col 90 (> width 84)
    CHECK_EQ("col clamps to width-1", g_m.lcd_x, w - 1);

    // data write + horizontal auto-advance.
    lcd_command(&g_m, 0x40);                       // y = 0
    lcd_command(&g_m, 0x80);                       // x = 0
    lcd_data(&g_m, 0xAB);
    CHECK_EQ("fb[0] stored", g_m.fb[0], 0xAB);
    CHECK_EQ("x auto-advanced", g_m.lcd_x, 1);

    // px() applies the display transform: blank mode -> nothing lit.
    lcd_command(&g_m, 0x08);                       // display blank (D=0,E=0)
    CHECK_EQ("blank mode", g_m.lcd_mode, 0);
}

// --- Keypad matrix ----------------------------------------------------------
static void test_keypad(void) {
    printf("[keypad] matrix scan + IRQ\n");
    fixture();
    // idle: columns active-low, nothing pressed -> 0x1F.
    CHECK_EQ("idle cols", kbd_cols(&g_m), 0x1F);
    // press col0 on row0: that column reads low (3310 drives all rows).
    g_m.kbd_norm_cols[0] = 0x01;
    CHECK_EQ("col0 pressed", kbd_cols(&g_m), 0x1E);
    // a keypad edge raises IRQ0.
    g_m.irq_pending = 0;
    mad2_keypad_irq(&g_m);
    CHECK("keypad raises IRQ0", (g_m.irq_pending & 0x01) != 0);
}

// --- MBUS receive FIFO ------------------------------------------------------
static void test_mbus(void) {
    printf("[mbus] receive FIFO\n");
    fixture();
    CHECK_EQ("empty count", mbus_rx_count(&g_m), 0);
    CHECK_EQ("empty reads 0xFF (idle line)", mbus_rx_pop(&g_m), 0xFF);
    g_m.mbus_rx[0] = 0xAB; g_m.mbus_rx[1] = 0xCD;
    g_m.mbus_rx_head = 0; g_m.mbus_rx_tail = 2;
    CHECK_EQ("count after fill", mbus_rx_count(&g_m), 2);
    CHECK_EQ("pop 1", mbus_rx_pop(&g_m), 0xAB);
    CHECK_EQ("pop 2", mbus_rx_pop(&g_m), 0xCD);
    CHECK_EQ("drained reads 0xFF", mbus_rx_pop(&g_m), 0xFF);
}

// --- Model profile HAL data (docs/hal-spec.md) -------------------------------
// Capability query + period-correct LED colours: profile DATA, so pin it here —
// a profile edit that drops a key or a colour shows up as a test failure.
static void test_profile_hal(void) {
    printf("[profile] key capabilities + LED colours\n");
    const ModelProfile* p3310 = model_by_name("3310");
    const ModelProfile* p8250 = model_by_name("8250");
    const ModelProfile* p5210 = model_by_name("5210");
    CHECK_EQ("3310 has soft1 (Menu)", model_key_present(p3310, KK_SOFT1), 1);
    CHECK_EQ("3310 has no SEND (Family B)", model_key_present(p3310, KK_SEND), 0);
    CHECK_EQ("3310 has no volume", model_key_present(p3310, KK_VOLUP), 0);
    CHECK_EQ("8250 has SEND (Family A)", model_key_present(p8250, KK_SEND), 1);
    CHECK_EQ("8250 has volume", model_key_present(p8250, KK_VOLUP), 1);
    CHECK_EQ("no model maps the wheel yet", model_key_present(p3310, KK_WHEEL_UP), 0);
    CHECK_EQ("out-of-range id", model_key_present(p3310, KK_COUNT), 0);
    CHECK_EQ("3310 LED colour = classic default (0)", (int)p3310->led.lcd_rgb, 0);
    CHECK_EQ("5210 LED colour = orange", (int)p5210->led.lcd_rgb, 0xFFA040);
    CHECK_EQ("8250 LED colour = blue", (int)p8250->led.lcd_rgb, 0x6FA8FF);
}

// --- Synthetic A3/A8 (RUN GSM ALGORITHM) -------------------------------------
// The synth SIM runs its own operator A3/A8 over a baked test Ki. Pin: determinism,
// avalanche, GSM Kc low-10-bits-zero, and a fixed regression vector.
static void test_sim_auth(void) {
    printf("[sim] A3/A8 synthetic authentication\n");
    uint8_t rand0[16]; for (int i = 0; i < 16; ++i) rand0[i] = (uint8_t)i;
    uint8_t rand1[16]; memcpy(rand1, rand0, 16); rand1[15] ^= 0x01;   // flip one bit
    uint8_t s0[4], k0[8], s0b[4], k0b[8], s1[4], k1[8];
    sim_run_gsm_algorithm(rand0, s0, k0);
    sim_run_gsm_algorithm(rand0, s0b, k0b);
    sim_run_gsm_algorithm(rand1, s1, k1);
    CHECK("A3/A8 deterministic (SRES)", memcmp(s0, s0b, 4) == 0);
    CHECK("A3/A8 deterministic (Kc)",   memcmp(k0, k0b, 8) == 0);
    CHECK("A3/A8 avalanche: 1-bit RAND flip changes SRES", memcmp(s0, s1, 4) != 0);
    CHECK("A3/A8 avalanche: 1-bit RAND flip changes Kc",   memcmp(k0, k1, 8) != 0);
    CHECK("Kc low 10 bits zero (GSM A5)", k0[7] == 0x00 && (k0[6] & 0x03) == 0);
    // Pinned vector for RAND = 00 01 .. 0F with Ki "NOKIADCT3EMUKI01".
    static const uint8_t SRES_VEC[4] = {0x8C,0x0E,0x08,0x73};
    static const uint8_t KC_VEC[8]   = {0xE1,0x71,0x2D,0xEC,0x96,0xA7,0xD8,0x00};
    CHECK("A3/A8 SRES regression vector", memcmp(s0, SRES_VEC, 4) == 0);
    CHECK("A3/A8 Kc regression vector",   memcmp(k0, KC_VEC, 8) == 0);
}

// --- Audio: unified buzzer -> PCM mixer (emu_audio.c) -----------------------
// Capture sink: count total + non-zero (tone) samples and the first tone index.
static long  g_au_total, g_au_tone, g_au_first = -1;
static void au_sink(struct Mad2* m, int ch, int16_t s) {
    (void)m;
    if (ch != 1) return;
    if (s != 0 && g_au_first < 0) g_au_first = g_au_total;
    if (s != 0) g_au_tone++;
    g_au_total++;
}
static void test_audio_mixer(void) {
    printf("[audio] unified buzzer -> PCM mixer (sample-accurate onset)\n");
    fixture();
    g_m.pcm_sink = au_sink;
    g_au_total = g_au_tone = 0; g_au_first = -1;

    // A ~30 ms chirp at divider 13000 (freq = 13 MHz / 13000 = 1 kHz). Timeline in rtc_mono
    // cycles (13 MHz): boot to 100k, divider set, buzzer ON, +390k cyc (30 ms) ON, then OFF.
    // The mixer streams CONSTANTLY (idle = silence samples), so the pre-ON span is zeros.
    const uint64_t ARM = 13000000ull, CODEC = EMU_AUDIO_HZ;   // mixer renders at 48 kHz
    g_m.rtc_mono = 100000;
    mad2_write(&g_m, 0, MMIO_BASE + 0x1C, 2, 13000);   // divider (word write) -> 1 kHz
    mad2_write(&g_m, 0, MMIO_BASE + 0x15, 1, 0x20);    // PUP bit5 = buzzer ON (stamps onset here)
    CHECK_EQ("no tone emitted before the ON edge", g_au_tone, 0);
    long onset_expect = g_au_total;                    // stream position at the ON write

    uint64_t on_cyc = 390000;                          // ~30 ms
    g_m.rtc_mono += on_cyc;
    mad2_write(&g_m, 0, MMIO_BASE + 0x15, 1, 0x00);    // OFF -> flushes the exact ON span as PCM
    long want = (long)((on_cyc * CODEC) / ARM);        // 1440 samples @48k
    long span = g_au_total - onset_expect;             // stream positions are exact (constant stream)
    CHECK("ON span matches duration on the stream grid (+/-1)",
          span >= want - 1 && span <= want + 1);
    // The low-passed square crosses zero, so count nonzero loosely: most of the span voiced.
    CHECK("waveform present for most of the ON span", g_au_tone > want / 2);
    CHECK("onset lands exactly at the ON-write stream position", g_au_first == onset_expect);

    // The stream keeps flowing after the OFF edge (constant stream, no stop): a short
    // release ramp (<= ~64 samples, the anti-click envelope ring-down) then pure silence.
    g_m.rtc_mono += 400000;
    long tone_before = g_au_tone, total_before = g_au_total;
    emu_audio_render(&g_m);
    CHECK("post-OFF: only the release ramp is non-silent (<= 70 samples)",
          g_au_tone - tone_before >= 1 && g_au_tone - tone_before <= 70);
    CHECK("stream continues (silence) after the tone", g_au_total > total_before);

    // Gate: when the DSP codec owns the sink, the buzzer mixer stands down entirely.
    fixture();
    g_m.pcm_sink = au_sink; g_m.pcm_codec_seen = 1;
    g_au_total = g_au_tone = 0; g_au_first = -1;
    g_m.rtc_mono = 100000;
    mad2_write(&g_m, 0, MMIO_BASE + 0x1C, 2, 13000);
    mad2_write(&g_m, 0, MMIO_BASE + 0x15, 1, 0x20);
    g_m.rtc_mono += 390000;
    mad2_write(&g_m, 0, MMIO_BASE + 0x15, 1, 0x00);
    CHECK_EQ("codec-owned sink: buzzer mixer emits nothing", g_au_total, 0);

    // DSP HLE tone producer: writing the COBBA tone registers (amp + osc1 freq in RAM)
    // makes the mixer synthesize a sine into the same PCM stream — the keypad beep the GUI
    // previously had no voice for. 900 Hz = reg 0x0E10 (1/4-Hz units) at amp != 0.
    fixture();
    g_m.pcm_sink = au_sink;
    g_au_total = g_au_tone = 0; g_au_first = -1;
    g_m.mem[DCT3_TONE_AMP]      = 0x00; g_m.mem[DCT3_TONE_AMP + 1]  = 0x40;  // amp != 0 (BE)
    g_m.mem[DCT3_TONE_OSC1]     = 0x0E; g_m.mem[DCT3_TONE_OSC1 + 1] = 0x10;  // 0x0E10 -> 900 Hz
    g_m.rtc_mono = 100000;
    emu_audio_render(&g_m);                            // sync cursor (no output yet)
    long tone_total0 = g_au_total;
    g_m.rtc_mono += 390000;                            // ~30 ms of tone
    emu_audio_render(&g_m);
    long want_tone = (long)((390000ull * (uint64_t)EMU_AUDIO_HZ) / 13000000ull);
    CHECK("HLE tone emits PCM for the tone duration",
          g_au_total - tone_total0 >= want_tone - 2 && g_au_total - tone_total0 <= want_tone + 2);
    CHECK("HLE tone is a non-trivial waveform (not all zero)", g_au_tone > 0);

    // Amplitude gate: amp == 0 -> the stream is pure silence (samples flow, all zero).
    fixture();
    g_m.pcm_sink = au_sink;
    g_au_total = g_au_tone = 0; g_au_first = -1;
    g_m.mem[DCT3_TONE_OSC1] = 0x0E; g_m.mem[DCT3_TONE_OSC1 + 1] = 0x10;      // freq set, amp = 0
    g_m.rtc_mono = 100000; emu_audio_render(&g_m);
    g_m.rtc_mono += 390000; emu_audio_render(&g_m);
    CHECK_EQ("HLE tone gated off when amplitude == 0 (silence only)", g_au_tone, 0);
    CHECK("constant stream still flows while gated", g_au_total > 0);
}

// --- dbgcon: emulator-only developer console port ----------------------------
static void test_dbgcon(void) {
    printf("[dbgcon] fake MMIO developer console @0x%05X\n", DCT3_DBGCON_CHAR);
    fixture();
    // CHAR port: byte writes assemble a line, '\n' flushes (prints above the checks).
    const char* msg = "OK\n";
    for (const char* p = msg; *p; ++p) mad2_write(&g_m, 0, DCT3_DBGCON_CHAR, 1, (uint8_t)*p);
    CHECK_EQ("CHAR writes counted", g_m.dbgcon_writes, 3);
    CHECK_EQ("newline flushed the line buffer", g_m.dbgcon_len, 0);
    // 32-bit write appends MSB-first (BE store order): 'A','B','C','\n' in one str.
    mad2_write(&g_m, 0, DCT3_DBGCON_CHAR, 4, 0x4142430Au);
    CHECK_EQ("word write assembled+flushed MSB-first", g_m.dbgcon_len, 0);
    // HEX port prints immediately; must not disturb the CHAR buffer.
    mad2_write(&g_m, 0, DCT3_DBGCON_HEX, 4, 0xCAFE);
    CHECK_EQ("HEX write counted", g_m.dbgcon_writes, 5);
    // Presence probe: reads return 0xDEADBEEF truncated per size (BE order).
    CHECK_EQ("probe read (word)", mad2_read(&g_m, 0, DCT3_DBGCON_CHAR, 4, 0), 0xDEADBEEFu);
    CHECK_EQ("probe read (half)", mad2_read(&g_m, 0, DCT3_DBGCON_CHAR, 2, 0), 0xDEADu);
    CHECK_EQ("probe read (byte)", mad2_read(&g_m, 0, DCT3_DBGCON_CHAR, 1, 0), 0xDEu);
}

int main(void) {
    printf("=== mad2 device-model unit tests ===\n");
    test_flash();
    test_ccont();
    test_lcd();
    test_keypad();
    test_mbus();
    test_profile_hal();
    test_sim_auth();
    test_audio_mixer();
    test_dbgcon();
    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
