// Nokia 6110 model profile — NSE-3, an EARLY MAD2 DCT3 (1998), same HW family as the 5110
// (NSE-1): 1 MB flash (MCU+PPM, NO in-flash EEPROM), an EXTERNAL serial-bus EEPROM, and the
// CCONT power ASIC on the bit-banged serial bus. Per Dan: "same HW profile as the 5110, just a
// different keypad." So this is a near-clone of src/models/5110/profile.c with three changes:
//
//   1. Ident "NSE-3" (header not at flash 0x1FC; whole-image string match, like the 5110).
//   2. ROM-4 HLE DSP on BOTH native and web — the C54x co-sim backend (mad2_dsp_c54x) loads the
//      *5110's* recovered DSP image (re/dsp-5110), which is wrong for the 6110, so the 6110 uses
//      the revision-correct ROM-4 HLE responder (mad2_dsp_rom4; see [[hle-dsp-by-rom-revision]]).
//   3. External EEPROM = the NokiX virgin **nse-3** image — an **8 KB 24C64** (2-byte word
//      addressing), vs the 5110's 2 KB 24C16. (firmware/nse-3.bin, baked as EE_NSE3.)
//
// The DCT3 RTOS/MMI core is shared with the 3310/5110, so MAD2_SIGS resolves the per-build
// firmware addresses over the 6110 flash. The build-specific .fw fallbacks below START as the
// 5110 v5.30 values (closest analog) and are corrected from the sig-resolution / boot post-mortem
// during bring-up; the HW-fixed HPI-window cells (cobba/mailbox/MDIRCV/boot-status) are identical
// across DCT3 and are correct as-is.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (== 8210/8850)

// 6110 keypad — shares the 5110's serial-bus scan HW LINES ([[dct3-keypad-families]]) but the
// key LAYOUT is NSM Family-A, not the 5110's. PROVEN: the 6110's position->keycode keymap
// (@0x2BFB28) is BYTE-IDENTICAL to the 8210/8850 nsm_a keymap (soft1=(1,1)/0x19, up=(2,1)/0x17,
// down=(3,1)/0x18, soft2=(4,1)/0x1A, send=(2,0)/0x0E, end=(3,0)/0x0F, volup=(4,0)/0x10,
// voldown=(1,0)/0x11), so it uses the shared keylines_nsm_a verbatim. (The earlier 5110-cloned
// keylines had soft1/soft2/up mis-placed — soft2 at (1,1), up at (4,1).)

const ModelProfile model_6110 = {
    .name = "6110",
    .description = "Nokia 6110 (NSE-3, early MAD2) — external 24C64 EEPROM / serial bus",
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
        // Start with the 5110's PCD8544 84x48 + serial-bus LCD ports (Dan: same HW profile). If the
        // 6110 panel/controller differs, the boot logic is unaffected — fix rendering post-standby.
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2B, .io_cmd = 0x2C,
    },
    .keypad = {
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,   // Family-A: 2 soft keys, up/down, send/end, vol — keymap-confirmed
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_SERIAL,
        .reg33_im_c = 1,
        .hold_insns = 400000,
        .col_port = 0x30, .row_port = 0x31, .dir_port = 0x2F,
    },
    .led = {
        // Vibrates via an accessory vibra BATTERY pack (not an internal motor), same
        // channel as the 3310 (I/O 0x15 bit4 -> vibra_on).
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
        // Build-specific RAM/code cells — START as the 5110 v5.30 fallbacks; MAD2_SIGS resolves the
        // 6110 v5.48 addresses over its flash (post-mortem/sigdump shows what resolves vs falls back).
        // verdict 0x10FDE1 — located via gen_sig of the 5110 MMI-ready gate (5110 0x23115A
        // `ldr =0x10FDDD`) → 6110 0x239CCA `ldr =0x10FDE1` (93% sig match). RAMWATCH confirms the
        // 40→C0→C4→(timeout clears bit6)→CONTACT SERVICE evolution; the HLE self-test responder
        // must watch THIS cell to post its PASS before the ~8.5M timeout (PC 0x23A53E).
        .verdict = 0x0010FDE1u, .sim_gate = 0, .dsp_uploaded = 0x0010A9E4u,
        // HW-fixed HPI-window cells — identical across DCT3, correct as-is.
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        .get_string = 0x00277B98u, .w_get_string = 0x0027790Eu,   // 5110 fallback; sig-resolve for 6110
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
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NSE-3",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web (no 5110-image C54x cosim)
    // External 24C64 (8 KB) I2C EEPROM — NokiX virgin nse-3, baked in (firmware/nse-3.bin).
    .i2c_eeprom_default = EE_NSE3,
    .i2c_eeprom_size    = EE_NSE3_LEN,
};
