// Nokia 5190 model profile — NSB-1, the US 1900 MHz sibling of the 5110 (NSE-1). Same board
// family: 1 MB flash (MCU+PPM only, NO in-flash EEPROM), external serial 24C16 EEPROM + CCONT
// on the bit-banged serial bus, PCD8544 84x48, 5110 keypad. Clone of src/models/5110/profile.c
// with per-build differences:
//
//   1. Ident "NSB-1" (header not at flash 0x1FC; whole-image string match, like the 5110).
//      NOTE: the NSB-1 image embeds a stray "NSE-3" string, so this profile MUST be
//      registered BEFORE the 6110 in model.c (first-match-wins).
//   2. ROM-4 HLE DSP on BOTH native and web — the C54x co-sim backend loads the *5110's*
//      recovered DSP image, canonical only for the 5110 v5.30 build, so the 5190 uses
//      mad2_dsp_rom4.
//   3. External EEPROM = the 5110's NokiX virgin nse-1 blob (EE_NSE1) — NokiX ships no
//      nsb-1 image; the NSE-1 24C16 layout is the analogue.
//
// Per-build RAM/code addresses below were gen_sig-ported from 5110 v5.30 to NSB-1 v6.71
// (firmware/Nokia 5190 NSB-1 v6.71 A.fls) and literal-confirmed in disfw — see .fw comments.
//
// STATE: the DSP SELF-TEST is now SOLVED — all 24 self-test result items pass and
// the verdict settles at 0x40 (bit6 kept), byte-identical to the working 5130. Root cause of the
// former CONTACT SERVICE was a two-part external-EEPROM fault:
//   (1) ADDRESSING: NSB-1 v6.71 drives a 2-byte word address (24C32+ protocol) even though we bake
//       the 2K 5110 analogue blob. With 1-byte (24C16) addressing every calibration read misaligned
//       → the self-test checksums read garbage → false CS. Fixed via .i2c_two_byte_addr (ext_eeprom.c).
//   (2) CHECKSUMS: two self-test result items read an external-EEPROM calibration checksum the
//       analogue blob isn't self-consistent for — item 12 (gate 0x239FF4: BE u16 @0x1FE over
//       [0x120..0x1FE) − adj@0x154) and item 18 (validator 0x26CC4C: BE u32 @0x11C = 16-bit sum
//       over [0..0x11C)). Both finalized on load (.nsb_cksum_* / .nsb_cksum2_*, ext_eeprom.c).
// REMAINING (OPEN): the phone STILL rests at CONTACT SERVICE from a SEPARATE cause — NOT the verdict
// (identical to the 5130 which reaches "Insert SIM card"), NOT a reset (clean boot), NOT SIM (CS with
// SIM present too). Most likely a deeper band-specific calibration-VALUE check that the wrong-band
// 5110 (900 MHz) analogue data can't satisfy — a faithful fix needs a real NSB-1 (1900 MHz) EEPROM
// dump, which NokiX doesn't ship. The sibling NSB-3 v6.13 (6190) shares the self-test fault; the
// non-US builds (5130/6130/6150/8810) pass organically to "Security code".

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images
#include "models/keylines_5110.h"  // shared 5110-family matrix (5110/5130/5190)

const ModelProfile model_5190 = {
    .name = "5190",
    .description = "Nokia 5190 (NSB-1, early MAD2, PCD8544 84x48) — external EEPROM / serial bus",
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
        // verdict 0x10FDE5 — gen_sig of the 5110 v5.30 MMI-ready gate (0x23115A) → NSB-1 v6.71
        // 0x23C54A (87%), same 3-insn gate, `ldr r5,=0x10FDE5` (same cell as the 5130).
        // dsp_uploaded 0x10AD2C — gen_sig of the 5110 setter 0x271D1C → 0x28FC42 (96%); the
        // matched site writes through r8, loaded two insns up at 0x28FBF8 `ldr r0,=0x10AD2C;
        // mov r8,r0` (register-allocated variant of the 5110's direct `ldr;strb`).
        .verdict = 0x0010FDE5u, .sim_gate = 0, .dsp_uploaded = 0x0010AD2Cu,
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
        // get_string 0x297612 / w_get_string 0x2973B2 — EXACT gen_sig matches from the 5110.
        .get_string = 0x00297612u, .w_get_string = 0x002973B2u,
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
        .match = "NSB-1",
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_rom4,            // ROM-4 HLE on native + web (no 5110-image C54x cosim)
    // External I2C EEPROM — the 5110's NokiX virgin nse-1 blob (no nsb-1 image exists). The
    // NSB-1 v6.71 firmware drives a 2-byte word address (24C32+ protocol) even though we bake
    // the 2K analogue, so force 2-byte addressing — without it every calibration read misaligns
    // and the self-test checksum reads garbage → false CONTACT SERVICE. See ext_eeprom.c.
    .i2c_eeprom_default = EE_NSE1,
    .i2c_eeprom_size    = EE_NSE1_LEN,
    .i2c_two_byte_addr  = 1,
    // NSB calibration-record checksums (RE'd 2026-07-15 against NSB-1 v6.71). Two self-test
    // result items each read an external-EEPROM calibration checksum: item 12 (gate 0x239FF4)
    // and item 18 (validator 0x26CC4C). Both are false-failed by the non-self-consistent
    // analogue blob; finalize each so the self-test result array is all-pass.
    .nsb_cksum_off = 0x1FE, .nsb_cksum_beg = 0x120, .nsb_cksum_len = 0xDE, .nsb_cksum_adj = 0x154,
    .nsb_cksum2_off = 0x11C,
};
