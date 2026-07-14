// Nokia 6190 model profile — NSB-3, the US 1900 MHz sibling of the 6110 (NSE-3). Same board
// family: 1 MB flash (MCU+PPM only, NO in-flash EEPROM), external serial-bus EEPROM + CCONT
// on the bit-banged serial bus, PCD8544 84x48, NSM Family-A key layout. Clone of
// src/models/6110/profile.c with per-build differences:
//
//   1. Ident "NSB-3" (header not at flash 0x1FC; whole-image string match, like the 6110).
//      NOTE: the NSB-3 image embeds a stray "NSE-3" string, so this profile MUST be
//      registered BEFORE the 6110 in model.c (first-match-wins).
//   2. External EEPROM = the 6110's NokiX virgin nse-3 blob (EE_NSE3, 8 KB 24C64) — NokiX
//      ships no nsb-3 image; the NSE-3 layout is the analogue.
//
// Per-build RAM/code addresses below were gen_sig-ported from 6110 v5.48 / 5110 v5.30 to
// NSB-3 v6.13 (firmware/Nokia 6190 NSB-3 v6.13 A.fls) and literal-confirmed in disfw.
//
// STATE (2026-07-14 first boot): detects + boots clean into the MMI, rests at CONTACT
// SERVICE — same NSB-build self-test FAIL as the 5190 (see src/models/5190/profile.c:
// bit6 cleared by a judged expected-nonzero check before the group-0x74 reply lands).
// The non-US 6110 siblings (6130/6150/8810) pass organically to "Security code".

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (== 6110/8210/8850)

const ModelProfile model_6190 = {
    .name = "6190",
    .description = "Nokia 6190 (NSB-3, early MAD2) — external 24C64 EEPROM / serial bus",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00100000u,   // 1 MB (MCU+PPM only — external EEPROM, no in-flash partition)
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0,             // external EEPROM (serial bus); no in-flash partition
        .eeprom_size = 0,
    },
    .lcd = {
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2B, .io_cmd = 0x2C,
    },
    .keypad = {
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,   // Family-A layout on the serial-bus scan HW (== 6110)
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_SERIAL,
        .reg33_im_c = 1,
        .hold_insns = 400000,
        .col_port = 0x30, .row_port = 0x31, .dir_port = 0x2F,
    },
    .led = {
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
        // verdict 0x10FDE9 — gen_sig of the 6110 v5.48 MMI-ready gate (0x239CCA) → NSB-3 v6.13
        // 0x23F60E (94%), same 3-insn gate, `ldr r5,=0x10FDE9`.
        // dsp_uploaded 0x10AD08 — gen_sig of the 5110 setter 0x271D1C → 0x2946EA (96%); the
        // matched site writes through r8, loaded two insns up at 0x2946A0 `ldr r0,=0x10AD08;
        // mov r8,r0` (same register-allocated variant as the 5190 v6.71 build).
        .verdict = 0x0010FDE9u, .sim_gate = 0, .dsp_uploaded = 0x0010AD08u,
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
        // get_string 0x29DBD8 / w_get_string 0x29D982 — EXACT gen_sig matches from the 5110.
        .get_string = 0x0029DBD8u, .w_get_string = 0x0029D982u,
        .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0x00010004u, .dsp_boot_ready = 4,       // ROM-4 DSP reports version 4
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
        .match = "NSB-3",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web
    // External 24C64 (8 KB) I2C EEPROM — the 6110's NokiX virgin nse-3 blob (no nsb-3 image).
    .i2c_eeprom_default = EE_NSE3,
    .i2c_eeprom_size    = EE_NSE3_LEN,
};
