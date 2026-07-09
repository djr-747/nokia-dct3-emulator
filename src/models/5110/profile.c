// Nokia 5110 model profile — NSE-1, an EARLY MAD2 DCT3 (1998). It is all MAD2; the early
// models just differ in how peripherals attach. Two hard differences from the 2 MB family:
//
//   1. 1 MB flash (MCU+PPM only) — NO in-flash EEPROM partition. The EEPROM is an EXTERNAL
//      24C16 serial chip, and the CCONT power ASIC is ALSO reached over a bit-banged serial
//      bus rather than the later memory-mapped GENSIO. Modelled by the serial BusOps
//      (mad2_bus_serial) + the gensio/keypad/lcd port DATA below — no platform fork.
//   2. The MCU version header is NOT at the canonical flash 0x1FC (it sits at file
//      0x94070 / addr 0x294070). flash[0x1FC] is code, so model_detect falls back to a
//      whole-image search for the ident string — 1 MB + "NSE-1" is unambiguous.
//
// The DCT3 RTOS/MMI core is shared with the 3310 (the sigs resolve), so our ARM core runs
// it; RAM is shifted ~0x10000 down vs the 3310 (sim_gate 0x10FD22 vs ~0x11Fxxx). Boots to
// "Security code" under DSP54_COSIM=1 (tools/check_5110_boot.sh).

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images

// Family B (3310-style) keypad matrix. The 5110 ALSO has dedicated screen-side volume
// buttons (up/down on the left edge) that the candybar 3310 lacks — they sit on their own
// I/O lines (TBD: not in the scan matrix); modelled later once the bus barrier is cleared.
// Calibrated against the live 5110 firmware (rawkey = row*5+col; scan ISR 0x290C7E drives
// DIR_R=1<<row, reads col 0x30; decode store 0x290D0A -> [0x10B6C8]). The matrix is a regular
// grid: keypad columns 1/2/3 sit on scan cols 2/3/4, keypad rows 1..4 on scan rows 1..4 (scan
// row 0 unused). The col-1 nav cluster decodes to rawkeys 6/11/16/21 (rows 1..4) — ALL FOUR
// verified via RAMWATCH=0x10B6C8 on injected presses (each cell -> its row*5+col rawkey).
// (1,1)=Names/C and (4,1)=scroll-up are also Dan-confirmed in-GUI; (2,1)=Menu and (3,1)=scroll
// -down follow the same cluster (rawkey 11/16) — full in-menu confirmation is gated behind the
// FAID boot-lock ("Security code") on a virgin EEPROM.
static const KeyLine keylines_5110[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,1,4,0},
    {KK_4,2,2,0}, {KK_5,2,3,0}, {KK_6,2,4,0},
    {KK_7,3,2,0}, {KK_8,3,3,0}, {KK_9,3,4,0},
    {KK_STAR,4,2,0}, {KK_0,4,3,0}, {KK_HASH,4,4,0},
    {KK_SOFT2,1,1,0},   // right softkey "Names" / C (clear) — rawkey 6 -> keycode 0x1A
    {KK_UP,4,1,0},      // scroll up — rawkey 21 -> keycode 0x17, confirmed in-GUI
    {KK_SOFT1,2,1,0},   // left softkey "Menu" — rawkey 11 -> keycode 0x19
    // scroll down: keycode 0x18 lives at rawkey 15 = (row3,col0), NOT (3,1). The keymap
    // table @0x2AB4FC[16] (=(3,1)) is 0x3E (NO KEY) — the earlier (3,1) guess was a dead
    // cell, so the down key did nothing. Verified against the table (idx15=(3,0)=0x18, the
    // 4th nav key alongside up=0x17/Menu=0x19/Names=0x1A); 6110 differs (down IS at (3,1)).
    {KK_DOWN,3,0,0},    // scroll down — rawkey 15 -> keycode 0x18, keymap-confirmed
    {KK_PWR,0,0,0x02},  // special-scan power
};

const ModelProfile model_5110 = {
    .name = "5110",
    .description = "Nokia 5110 (NSE-1, early MAD2, PCD8544 84x48) — external EEPROM / serial bus",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00100000u,   // 1 MB (MCU+PPM only — no in-flash EEPROM partition)
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // EXTERNAL EEPROM (serial chip) — there is no flash EEPROM region. eeprom_base 0
        // signals "no in-flash partition"; the EEPROM lives on the I/O 0x28-0x2D serial bus.
        .eeprom_base = 0,
        .eeprom_size = 0,
    },
    .lcd = {
        // 84x48 PCD8544. The early-MAD2 5110 LCD ports DIFFER (MADos ioports.h PHONE_5110):
        // GENSIO_LCD_DATA 0x2B, GENSIO_LCD_CMD 0x2C (vs the later 0x2E/0x6E). io_data/io_cmd
        // are LOAD-BEARING: mad2_bus routes LCD writes by these per-model ports..
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2B, .io_cmd = 0x2C,
    },
    .keypad = {
        // serial-bus keypad ports differ from MAD2 (MADos PHONE_5110): KPD_C 0x30, KPD_R 0x31,
        // DIR_R 0x2F (vs MAD2 0x2A/0x28/0xA8 — which on serial-bus are GENSIO CC_WR/CFG!). These
        // are now profile DATA (col/row/dir_port) routed by mad2_bus, not hardcoded cases.
        // The matrix decode below is the Family-B layout. + screen-side volume buttons.
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_3310,
        .lines   = keylines_5110,
        .n_lines = (int)(sizeof(keylines_5110) / sizeof(keylines_5110[0])),
        .scan = KP_SCAN_SERIAL,   // dir-aware; 0xE0 is an upper-row scan phase, not special-scan
        .reg33_im_c = 1,          // I/O 0x33 is IM_C (keypad int-mask), not the keypad-LED line
        .hold_insns = 400000,     // serial debounce FSM decodes 197-293k after the down-edge
        .col_port = 0x30, .row_port = 0x31, .dir_port = 0x2F,
    },
    .led = {
        // Iconic 5110 green EL backlight = PUP_GENIO (I/O 0x20) BIT6 (0x40 = PUP_GENIO_LED),
        // per MADos led scheme 2 (apps/main.c led=2; hw/led.c case 2; ioports.h PHONE_5110).
        // NOT bit3 (MAD2) and NOT bit5 (0x20 = PUP_GENIO_DISP, the LCD-module power line — the
        // earlier bit5 mapping was wrong). I/O 0x20 is muxed with the I2C bit-bang, so led_lcd
        // is latched in ext_eeprom_write (the serial 0x20 handler), not mad2_bus case 0x20
        // (which never runs for the serial family). Off at idle, lit on user activity.
        // The serial-keypad 5110 has NO separate keypad LED. It DOES vibrate — via an
        // accessory vibra BATTERY pack (not an internal motor), on the same channel as the
        // 3310 (I/O 0x15 bit4 -> vibra_on); the firmware drives it, so model it (no no_vibra).
        .lcd_rgb = 0x55C955, .lcd_mask = 0x40, .no_kbd_led = 1,
    },
    .battery = {
        // Placeholder — the serial-bus battery A/D is read over the serial CCONT, not the MAD2
        // GENSIO ADC; values here are inert until the bus is modelled.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    // Bit-banged serial CCONT ports (vs the later memory-mapped GENSIO 0x2C/0x6C/0x2D).
    // CC_RD 0x2D is a READ; it does not collide with the later START 0x2D (a write)
    // — ccont_r matches on reads only. The serial bus self-frames via CFG 0x28 (no START).
    .gensio = { .ccont_w = 0x2A, .ccont_r = 0x2D, .start = 0 },
    // Early-MAD2 serial-attached transport: external 24C16 EEPROM I2C + GENSIO bus
    // status (BusOps, not a flag —). The CCONT/keypad/LCD PORTS are
    // the gensio/keypad/lcd spec data above; this owns the EEPROM lines + status.
    .bus = &mad2_bus_serial,
    .asic = {
        .irq_sources = 8,
    },
    .fw = {
        // DSP self-test verdict / DSP-upload-done flag — gen_sig-ported from 3310 v5.79
        // (5110 is NSE-1, same NSM/MMI+RTOS core). CONTACT SERVICE here is the DSP verdict
        // gate, identical in shape to 3310/8210/8250/8850 — NOT the external EEPROM (the
        // 24C16 I2C bus is never driven before standby).
        //
        // verdict = 0x0010FDDD (analog of 3310 0x11FF15): the MMI-ready / self-test verdict
        // byte. 3310 site 0x24389A `ldr =0x11FF15; ldrb; lsr #7` (read bit7 = MMI ready)
        // gen_sig-ports 89% to 5110 0x23115A `ldr =0x10FDDD; ldrb; lsr #7` — same bit7 gate.
        // dsp_uploaded = 0x0010A9E4 (analog of 3310 0x11038C): the DSP-code-block-done flag
        // the IRQ4 block-ack ISR sets to 1. 3310 setter 0x2BB050 ports 96% to 5110 0x271D1C
        // (`ldr =0x10A9E4; strb`); two more sites (0x272144/0x286462) agree on 0x10A9E4.
        // The shared mad2 DSP responder/pump drives both organically (no pokes/trampolines).
        .verdict = 0x0010FDDDu, .sim_gate = 0, .dsp_uploaded = 0x0010A9E4u,
        // DSP shared-RAM mailbox / MDIRCV queue (HPI layout) — hardware-fixed at 0x100E0,
        // shared across DCT3 (identical to 3310/8850).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // get_string (PPM nokstr resolver) — gen_sig-ported from 3310 0x2BBFAC (96%) /
        // 0x2BBCB8 (wide). Enables the GETSTR tracer (maps on-screen strings -> caller).
        .get_string = 0x00277B98u, .w_get_string = 0x0027790Eu,
        .faid_cksum = 0, .faid_cksum_val = 0,
        // DSP boot handshake: firmware's DSP-readiness step parks the HPI status word
        // 0x10004 at 0xFFFF and polls it for the DSP's ready/version reply; the ROM-4 HLE
        // responder (mad2_dsp_rom4) returns dsp_boot_ready so the firmware proceeds to
        // upload the DSP code blocks (block-ack IRQ4 sets dsp_uploaded). 0x10004 is the
        // hardware-fixed HPI status slot, shared with 8210/8850.
        // VERSION = 4 is COSIM-GROUNDED: the real C54x DSP writes VERSION_A/B = 0x0004 at boot
        // (docs/research/5110-). The old shared path faked 6 (a ROM-6
        // value); the loader (0x271AF4) only cross-checks [0x10004]==[0x10006], so 4 boots faithfully.
        .dsp_boot_status = 0x00010004u, .dsp_boot_ready = 4,
        // serial-bus loader (0x271AF4) cross-checks [0x10004]==[0x10006] before accepting the DSP
        // version reply; return dsp_boot_ready for the second word too so the branch table
        // validates and the firmware sets dsp_uploaded itself (clears MISMATCH). See §7e/§8e.
        .dsp_boot_status2 = 0x00010006u,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    .sigs = MAD2_SIGS,            // shared DCT3 RTOS sigs still resolve over the 5110 flash
    .n_sigs = MAD2_N_SIGS,        // (enables the post-mortem labelling)
    .boot = {
        .skip_seclock_default = 0,
        .pin_verdict_default  = 0,
    },
    .ident = {
        // String-only match: the harness calls model_detect with a fixed 0x400000 length
        // (boot_trace.c), so a 1 MB flash_size would never equal it. "NSE-1" is unique to
        // the 5110 (header isn't at 0x1FC -> whole-image search finds it in the first 1 MB).
        .match = "NSE-1",
        .flash_size = 0,
    },
    // Real C54x co-sim backend natively (runs re/dsp-5110/raw/dsp_full.bin alongside the
    // MCU; pass-through to the legacy model unless DSP54_COSIM=1 — see third_party/c54x/).
    // The wasm build has no C54x core, so it keeps the shared default.
#ifndef __EMSCRIPTEN__
    .dsp = &mad2_dsp_c54x,           // native: real C54x co-sim (falls through to ROM-4 HLE)
#else
    .dsp = &mad2_dsp_rom4,           // wasm (no C54x core): ROM-4 HLE responder
#endif
    // External 24C16 I2C EEPROM — the NokiX virgin nse-1 image, baked in (auto-selected here).
    // ext_eeprom.c loads this into m->i2c_eeprom (or an EE5110= file override); the web build has
    // no MEMFS file, so the baked blob is what makes the web 5110 boot past CONTACT SERVICE.
    .i2c_eeprom_default = EE_NSE1,
    .i2c_eeprom_size    = EE_NSE1_LEN,
};
