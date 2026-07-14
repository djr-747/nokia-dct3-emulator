// Nokia 5110i model profile — NSE-2, the cost-reduced 5110 refresh. Same board family as the
// 5110 (NSE-1): external serial 24C16 EEPROM + CCONT on the bit-banged serial bus, PCD8544
// 84x48, 5110 keypad — but a 2 MB flash (vs the NSE-1's 1 MB). Clone of
// src/models/5110/profile.c with per-build differences:
//
//   1. Ident "NSE-2" (header not at flash 0x1FC; whole-image string match). NOTE: the NSE-2
//      image embeds a stray "NSE-1" string, so this profile MUST be registered BEFORE the
//      5110 in model.c (first-match-wins). A genuine 5110 image carries no "NSE-2".
//   2. flash_size 2 MB.
//   3. ROM-4 HLE DSP on BOTH native and web — the C54x co-sim backend loads the *5110's*
//      recovered DSP image, canonical only for the 5110 v5.30 build, so the 5110i uses
//      mad2_dsp_rom4.
//   4. External EEPROM = the 5110's NokiX virgin nse-1 blob (EE_NSE1) — NokiX ships no
//      nse-2 image; the NSE-1 24C16 layout is the analogue.
//
// Per-build RAM/code addresses below were gen_sig-ported from 5110 v5.30 and literal-confirmed
// in disfw against BOTH library builds. The profile pins v5.53 (the newest, what `./run 5110i`
// resolves); v5.51 values noted inline where they differ:
//   verdict       — 0x10FDDD in BOTH builds (identical to the 5110 itself; gate v5.53
//                   0x2357FE 98% / v5.51 0x23573A 92%)
//   dsp_uploaded  — v5.53 0x10B9F4 (setter 0x278458 EXACT) / v5.51 0x10B9E8 (0x278314 EXACT)
//   get_string    — v5.53 0x27D996 / v5.51 0x27D852 (EXACT)
//   w_get_string  — v5.53 0x27D70C / v5.51 0x27D5C8 (EXACT)
//
// STATE (2026-07-14 first boot, BOTH v5.51 + v5.53): detects and boots into the MMI to
// CONTACT SERVICE with the borrowed 5110 nse-1 EEPROM. The blob DOES load + finalize (I2CLOG:
// tune checksum @0x11E + DSP-fault latch @0x29E both applied) — so the 5110's tune-checksum
// gate passes. But the 5110i self-test has an ADDITIONAL record-based element the nse-1 virgin
// blob doesn't satisfy: at v5.53 0x2333F2 it reads EEPROM record 0x31E (reader 0x23315C -> r10)
// and compares it against the record's own stored halfword (bne 0x23346C = FAIL), then OR-checks
// record 0x290 (beq FAIL) — verdict bit6 is cleared at 0x233476 (40->C0->C4->84). Reaching the
// "Security code" lock (like the 5110/5130) needs an nse-2 dump or provisioning records 0x31E/
// 0x290 in the blob — deferred RE (same class as the NSB judged self-test). So: BOOTS, at CS.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_5110.h"  // shared 5110-family matrix (5110/5110i/5130/5190)

const ModelProfile model_5110i = {
    .name = "5110i",
    .description = "Nokia 5110i (NSE-2, early MAD2, 2 MB, PCD8544 84x48) — external EEPROM / serial bus",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB (MCU+PPM — still external EEPROM, no in-flash partition)
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
        // v5.53 values — see the port table in the header (verdict is build-invariant).
        .verdict = 0x0010FDDDu, .sim_gate = 0, .dsp_uploaded = 0x0010B9F4u,
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
        .get_string = 0x0027D996u, .w_get_string = 0x0027D70Cu,
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
        .match = "NSE-2",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web (no 5110-image C54x cosim)
    // External 24C16 I2C EEPROM — the 5110's NokiX virgin nse-1 blob (no nse-2 image exists).
    .i2c_eeprom_default = EE_NSE1,
    .i2c_eeprom_size    = EE_NSE1_LEN,
};
