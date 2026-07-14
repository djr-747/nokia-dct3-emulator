// Nokia 5130 model profile — NSK-1, the 900/1800-band Xpress-on sibling of the 5110 (NSE-1).
// Same board family: 1 MB flash (MCU+PPM only, NO in-flash EEPROM), external serial 24C16
// EEPROM + CCONT on the bit-banged serial bus, PCD8544 84x48, 5110 keypad. Clone of
// src/models/5110/profile.c with per-build differences:
//
//   1. Ident "NSK-1" (header not at flash 0x1FC; whole-image string match, like the 5110).
//      NOTE: the NSK-1 image embeds a stray "NSE-3" string, so this profile MUST be
//      registered BEFORE the 6110 in model.c (first-match-wins).
//   2. ROM-4 HLE DSP on BOTH native and web — the C54x co-sim backend loads the *5110's*
//      recovered DSP image (re/dsp-5110), which is only canonical for the 5110 v5.30 build
//, so the 5130 uses mad2_dsp_rom4.
//   3. External EEPROM = the 5110's NokiX virgin nse-1 blob (EE_NSE1) — NokiX ships no
//      nsk-1 image; the NSE-1 24C16 layout is the analogue.
//
// Per-build RAM/code addresses below were gen_sig-ported from 5110 v5.30 to NSK-1 v5.30
// (firmware/Nokia 5130 NSK-1 v5.30 A.fls) and literal-confirmed in disfw — see .fw comments.
//
// STATE (2026-07-14 first boot): detects, self-test passes organically, boots to the
// "Security code" (FAID) lock — the same canonical state as the 5110/6110.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_5110.h"  // shared 5110-family matrix (5110/5130/5190)

const ModelProfile model_5130 = {
    .name = "5130",
    .description = "Nokia 5130 (NSK-1, early MAD2, PCD8544 84x48) — external EEPROM / serial bus",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00100000u,   // 1 MB (MCU+PPM only — no in-flash EEPROM partition)
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0,             // external EEPROM (serial bus); no in-flash partition
        .eeprom_size = 0,
    },
    .lcd = {
        // 84x48 PCD8544 on the early-MAD2 serial-bus LCD ports (== 5110): data 0x2B, cmd 0x2C.
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2B, .io_cmd = 0x2C,
    },
    .keypad = {
        // Serial-bus keypad ports + Family-B matrix, all == 5110 (same board family).
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_3310,
        .lines   = keylines_5110,
        .n_lines = (int)(sizeof(keylines_5110) / sizeof(keylines_5110[0])),
        .scan = KP_SCAN_SERIAL,
        .reg33_im_c = 1,
        .hold_insns = 400000,
        .col_port = 0x30, .row_port = 0x31, .dir_port = 0x2F,
    },
    .led = {
        // 5110-family EL backlight: PUP_GENIO (I/O 0x20) bit6, latched in ext_eeprom_write.
        .lcd_rgb = 0x55C955, .lcd_mask = 0x40, .no_kbd_led = 1,
    },
    .battery = {
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .gensio = { .ccont_w = 0x2A, .ccont_r = 0x2D, .start = 0 },
    .bus = &mad2_bus_serial,
    .asic = {
        .irq_sources = 8,
    },
    .fw = {
        // verdict 0x10FDE5 — gen_sig of the 5110 v5.30 MMI-ready gate (0x23115A `ldr =0x10FDDD;
        // ldrb; lsr #7`) → NSK-1 v5.30 0x22E6CE (86%), same 3-insn gate, `ldr r5,=0x10FDE5`.
        // dsp_uploaded 0x10B23C — gen_sig of the 5110 setter 0x271D1C → EXACT match 0x27331C
        // (`ldr r0,=0x10B23C; strb r4,[r0]`).
        .verdict = 0x0010FDE5u, .sim_gate = 0, .dsp_uploaded = 0x0010B23Cu,
        // HW-fixed HPI-window cells — identical across DCT3.
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        .mdisnd_q     = 0x00010000u,    // MCU->DSP request queue (HPI layout-invariant; see 3310)
        .mdisnd_tail  = 0x000100A4u,    // verified by the 2026-07-15 per-model RAMWATCH sweep
        // get_string 0x27C2DC (96% fuzzy) / w_get_string 0x27C05E (EXACT) — gen_sig-ported
        // from the 5110's 0x277B98/0x27790E.
        .get_string = 0x0027C2DCu, .w_get_string = 0x0027C05Eu,
        .faid_cksum = 0, .faid_cksum_val = 0,
        // 5110-family DSP boot handshake: ROM-4 version reply on both HPI status words.
        .dsp_boot_status = 0x00010004u, .dsp_boot_ready = 4,
        .dsp_boot_status2 = 0x00010006u,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .boot = {
        .skip_seclock_default = 0,
    },
    .ident = {
        .match = "NSK-1",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web (no 5110-image C54x cosim)
    // External 24C16 I2C EEPROM — the 5110's NokiX virgin nse-1 blob (no nsk-1 image exists).
    .i2c_eeprom_default = EE_NSE1,
    .i2c_eeprom_size    = EE_NSE1_LEN,
};
