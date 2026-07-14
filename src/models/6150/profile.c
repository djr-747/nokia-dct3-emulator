// Nokia 6150 model profile — NSM-1, the dual-band (900/1800) sibling of the 6110 (NSE-3).
// Same serial-bus board family — external serial-bus EEPROM + CCONT, PCD8544 84x48, NSM
// Family-A key layout — but a 2 MB flash (the image is 2 MB, RAM sits ~0x11xxxx like the
// 3310, vs the 1 MB 6110's ~0x10Fxxx). Clone of src/models/6110/profile.c with:
//
//   1. Ident "NSM-1" (header not at flash 0x1FC; whole-image string match, like the 6110).
//      Registered with the other serial-family variants BEFORE the 6110 in model.c.
//   2. flash_size 2 MB.
//   3. External EEPROM = the NokiX virgin **nsm-1** blob (EE_NSM1, 16 KB 24C128) — the 6150
//      has its OWN NokiX repair image, so no analogue blob is needed.
//
// Per-build RAM/code addresses below were gen_sig-ported from 6110 v5.48 / 5110 v5.30 to
// NSM-1 v5.23 (firmware/Nokia 6150 NSM-1 v5.23 A.fls) and literal-confirmed in disfw.
//
// STATE (2026-07-14 first boot): detects, self-test passes organically (verdict
// 08->48->C8->CC->C8, bit6 kept), boots to the "Security code" (FAID) lock like the 6110.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (== 6110/8210/8850)

const ModelProfile model_6150 = {
    .name = "6150",
    .description = "Nokia 6150 (NSM-1, early MAD2, 2 MB) — external 24C128 EEPROM / serial bus",
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
        // verdict 0x11FDD1 — gen_sig of the 6110 v5.48 MMI-ready gate (0x239CCA) → NSM-1 v5.23
        // 0x2422FE (97%), same 3-insn gate, `ldr r5,=0x11FDD1` (2 MB build → 3310-like RAM top).
        // dsp_uploaded 0x111A94 — gen_sig of the 5110 setter 0x271D1C → 0x2A4A14 (96%),
        // `ldr r0,=0x111A94; strb r4,[r0]`.
        .verdict = 0x0011FDD1u, .sim_gate = 0, .dsp_uploaded = 0x00111A94u,
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
        // get_string 0x2A98D4 / w_get_string 0x2A9662 — EXACT gen_sig matches from the 5110.
        .get_string = 0x002A98D4u, .w_get_string = 0x002A9662u,
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
        .match = "NSM-1",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web
    // External 24C128 (16 KB) I2C EEPROM — the 6150's OWN NokiX virgin nsm-1 blob.
    .i2c_eeprom_default = EE_NSM1,
    .i2c_eeprom_size    = EE_NSM1_LEN,
};
