// Nokia 8810 model profile — NSE-6, the 1998 luxury slider. Electrically a 6110-family
// serial-bus phone (closest analogue = 6110): external serial-bus EEPROM + CCONT
// on the bit-banged serial bus, PCD8544 84x48, NSM Family-A key layout — but a 2 MB flash
// (RAM top ~0x13FDxx, above even the 3310's ~0x11FFxx). Clone of src/models/6110/profile.c:
//
//   1. Ident "NSE-6" (header not at flash 0x1FC; whole-image string match, like the 6110).
//      NOTE: the NSE-6 image embeds a stray "NSE-3" string, so this profile MUST be
//      registered BEFORE the 6110 in model.c (first-match-wins).
//   2. flash_size 2 MB.
//   3. External EEPROM = the NokiX virgin **nse-6** blob (EE_NSE6, 32 KB 24C256) — the 8810
//      has its OWN NokiX repair image, so no analogue blob is needed.
//   4. The sliding keypad cover is NOT modelled yet (has_slide=0) — the 8855/8890 slide
//      wiring (cover line I/O 0x28 bit0) is a later-MAD2 mechanism; whether the 8810's
//      serial-bus scan exposes a cover line is an open RE item for bring-up.
//
// Per-build RAM/code addresses below were gen_sig-ported from 6110 v5.48 / 5110 v5.30 to
// NSE-6 v6.02 (firmware/Nokia 8810 NSE-6 v6.02 A.fls) and literal-confirmed in disfw.
//
// STATE (2026-07-14 first boot): detects, self-test passes organically, boots to the
// "Security code" (FAID) lock — the same canonical state as the 5110/6110.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (== 6110/8210/8850)

const ModelProfile model_8810 = {
    .name = "8810",
    .description = "Nokia 8810 (NSE-6, early MAD2, 2 MB, slider) — external 24C256 EEPROM / serial bus",
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
        // NSE-6 keypad ISR (0x2DE292) is later than the 6110's: it won't scan on a bare IRQ0,
        // it first reads the matrix interrupt-SOURCE at I/O 0x34 (bits[4:0]) and only proceeds
        // when a column bit is set. Without this the keypad is dead (IRQ0 delivered, ISR runs,
        // reads 0x34=0, bails). See the platform's irq_src34 handling. RE'd 2026-07-14.
        .irq_src34 = 1,
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
        // verdict 0x13FDE1 — gen_sig of the 6110 v5.48 MMI-ready gate (0x239CCA) → NSE-6 v6.02
        // 0x2432AE (97%), same 3-insn gate, `ldr r5,=0x13FDE1`.
        // dsp_uploaded 0x1205C0 — gen_sig of the 5110 setter 0x271D1C → EXACT match 0x2B63AC
        // (`ldr r0,=0x1205C0; strb r4,[r0]`).
        .verdict = 0x0013FDE1u, .sim_gate = 0, .dsp_uploaded = 0x001205C0u,
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
        // get_string 0x2C4A3E / w_get_string 0x2C47D6 — EXACT gen_sig matches from the 5110.
        .get_string = 0x002C4A3Eu, .w_get_string = 0x002C47D6u,
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
        .match = "NSE-6",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web
    // External 24C256 (32 KB) I2C EEPROM — the 8810's OWN NokiX virgin nse-6 blob (fills the
    // Mad2 i2c_eeprom buffer exactly; see mad2.h).
    .i2c_eeprom_default = EE_NSE6,
    .i2c_eeprom_size    = EE_NSE6_LEN,
};
