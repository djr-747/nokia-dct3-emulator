// MAD2 internal cross-module interface.
//
// mad2.c is being split into per-subsystem translation units (mad2_<sub>.c).
// Helpers that were file-static in the monolith but are shared across the bus
// router and the peripheral modules are declared here (and lose `static`).
// The public API and the shared Mad2 state struct stay in mad2.h; this header
// is private to the device model and is NOT installed/used outside src/mad2/.
#ifndef MAD2_INTERNAL_H
#define MAD2_INTERNAL_H

#include "mad2/mad2.h"

// Clock rates for the derived timer counters: Timer0/Timer1 tick at 33055/(div+1) Hz,
// the MAD ARM core at ~13 MHz. Shared by the CCONT/watchdog, the timer tick, and the
// bus I/O handlers (the IO_T0_DIV continuity offset + mad2_timers_tick advance).
#define DCT3_ARM_HZ   13000000ULL
#define DCT3_T0_HZ    33055ULL

// Lazily-computed timer counters (derived from the monotonic cycle clock). Read ON DEMAND
// by the bus I/O handlers instead of recomputing every instruction in the tick — the per-
// instruction hot path then carries only a cached-target compare for the timer EDGES. These
// are the exact formulas the tick used to evaluate per step (so reads are byte-identical:
// rtc_mono is constant within an instruction).
static inline uint16_t mad2_t1_value(const Mad2 *m) {
    return (uint16_t)((m->rtc_mono * (uint64_t)m->t1_hz) / DCT3_ARM_HZ);
}
static inline uint16_t mad2_t0_value(const Mad2 *m) {
    uint64_t d = (uint64_t)m->t0_div + 1u;
    return (uint16_t)((m->rtc_mono * DCT3_T0_HZ) / (DCT3_ARM_HZ * d) + m->t0_offset);
}

// MAD2 I/O window (the 0x20000 MMIO page) + register offsets within it. Shared by
// the bus dispatcher (mad2_bus.c), the timer tick (mad2_timers.c), and the GENSIO
// peripherals. The DSP shared-RAM mailbox addresses are per-model and live in m->fw.
#define MMIO_BASE 0x00020000u
#define MMIO_END  0x00020100u

// CTSI interrupt controller + Timer0 (offsets within the 0x20000 window).
#define IO_CTSI_WDT  0x03   // CTSI watchdog shadow (kick value; 0x00 = power-off)
#define IO_SLEEP_CTR 0x04   // 16-bit sleep/Timer1 counter (0x04/0x05)
#define IO_PUP_FIQ8  0x16   // FIQ8 + Timer1 enable: bit0 EN, bit1 ACT, bit2 MSK
#define IO_FIQ_ACT   0x08
#define IO_IRQ_ACT   0x09
#define IO_FIQ_MASK  0x0A
#define IO_IRQ_MASK  0x0B
#define IO_INT_CTRL  0x0C
#define IO_T0_DIV    0x0F
#define IO_T0_VAL    0x10   // 16-bit Timer0 value (0x10/0x11)
#define IO_T0_DEST   0x12   // 16-bit Timer0 destination (0x12/0x13)

// GENSIO-attached register offsets within the I/O window.
#define IO_GENSIO_CCONT_W   0x2C   // CCONT: write reg# then value
#define IO_LCD_DATA         0x2E   // PCD8544 data byte
#define IO_LCD_CMD          0x6E   // PCD8544 command byte (set-X 0x80|x, set-Y 0x40|y,
                                   // function-set/temp/bias/Vop) over GENSIO
#define IO_GENSIO_START     0x2D   // write 0x25 to begin a transaction
#define IO_GENSIO_CCONT_R   0x6C   // CCONT readback
#define IO_GENSIO_STATUS    0x6D   // bit0 wr-ready, bit1 txn-ready, bit2 rd-ready

// --- MBUS receive (mad2_mbus.c) ---------------------------------------------
int     mbus_rx_count(const Mad2* m);
uint8_t mbus_rx_pop(Mad2* m);
void    mbus_rx_signal(Mad2* m);   // accessory injection (no caller yet)
int     mbus_rx_push(Mad2* m, uint8_t b);   // host serial bridge: feed an RX byte (+FIQ2)
int     mbus_tx_out_pop(Mad2* m);           // host serial bridge: drain a shifted-out TX byte (-1 empty)

// --- PCD8544 LCD (mad2_lcd.c) -----------------------------------------------
// mad2_render_ascii / mad2_save_pgm are public (mad2.h).
void lcd_command(Mad2* m, uint8_t b);
void lcd_data(Mad2* m, uint8_t b);

// --- Keypad matrix (mad2_keypad.c) ------------------------------------------
// mad2_keypad_irq / mad2_slide_set are public (mad2.h).
uint8_t kbd_cols(const Mad2* m);

// --- Flash chip / CFI (mad2_flash.c) ----------------------------------------
uint32_t flash_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value);
void     flash_write(Mad2* m, uint32_t addr, uint32_t value, int size);

// --- CCONT power/RTC/ADC + watchdog (mad2_ccont.c) --------------------------
void    cc_int_update(Mad2* m);
void    wdt_kick(Mad2* m, uint8_t value, int cc);
uint8_t ccont_read(Mad2* m);
void    ccont_byte(Mad2* m, uint8_t b);

// External 24C16 EEPROM (I2C on GENIO ports 0x20/0x24, early-MAD2) — see ext_eeprom.c.
// off = 0x20 (bit0 SDA, bit2 SCL) or 0x24 (bit0 SDA direction, 1=MCU drives).
void    ext_eeprom_init(Mad2* m);                     // eager load (mad2_init); idempotent
void    ext_eeprom_write(Mad2* m, uint8_t off, uint8_t v);
int     ext_eeprom_sda(Mad2* m);
void    ext_eeprom_save(Mad2* m, const char* path);   // dump the 24C16 buffer (EE5110SAVE persist)

// --- Diagnostics (mad2_diag.c) ----------------------------------------------
// All exports (mad2_record_*, mad2_render_postmortem, mad2_intercept_panic_early)
// are public in mad2.h; the file owns the symbols_3310_v579 table for LR labels.

// --- SIM (mad2_sim.c) -------------------------------------------------------
// SIMI UART offsets within the 0x20000 window — shared by the SIM module and
// the bus dispatcher (mad2_read/mad2_write address-decode).
#define IO_SIM_TXD       0x36
#define IO_SIM_RXD       0x37
#define IO_SIM_UART_INT  0x38
#define IO_SIM_CTRL      0x39
#define IO_SIM_CLK_CTRL  0x3A
#define IO_SIM_TXD_LWM   0x3B
#define IO_SIM_RXD_QUE   0x3C
#define IO_SIM_RXD_FL    0x3D
#define IO_SIM_TXD_FL    0x3E
#define IO_SIM_TXD_QUE   0x3F
// EF_PHASE (GSM phase byte) is defined in mad2_sim.c, externed in mad2.c for the
// SIMPHASE env override in mad2_init.
uint32_t sim_read(Mad2* m, uint8_t off);
void     sim_write(Mad2* m, uint8_t off, uint8_t v);
void     sim_tick(Mad2* m);
// Run the synthetic A3/A8 with the baked test Ki: RAND[16] -> SRES[4] + Kc[8].
void     sim_run_gsm_algorithm(const uint8_t* rand16, uint8_t* sres4, uint8_t* kc8);

#endif // MAD2_INTERNAL_H
