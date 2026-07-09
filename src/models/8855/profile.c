// Nokia 8855 model profile — BOOTS TO STANDBY (2026-06-06, EEPROM image). NSM-4, 4 MB MAD2
// SLIDE phone — the updated 8850; same NSM Family-A class (shared keypad matrix + BLB-2
// battery) and a slide cover, but a 4 MB image (8850/8210/8250 are 2 MB). verdict/dsp_uploaded
// are RE'd below and the DSP upload completes natively (acks ~3589) so the self-test RUNS.
// The stock image ships a BLANK EEPROM partition (0x580000-0x5FFFFF all 0xFF) → the calibration
// self-test fails → CONTACT SERVICE. This is the 5210 case: generate a default EEPROM via the
// firmware's own init — `tools/eeprom_reset 8855 --full --capture` (the 8855 is in the case
// table: RESET_FN 0x2613F4 / EWB 0x3DC2F8 / VERDICT 0x17FDE7 / PART_RT 0x580000 / CKFIX
// 10b0011c:0000187d — the same calib checksum as the 5210). Boot the generated image
// `firmware/Nokia 8855 NSM-4 v5.13 P (EEPROM).fls`. See docs/eeprom-fix-howto.md +
// docs/8210-8250-bringup.md. Firmware on hand: firmware/Nokia 8855 NSM-4 v5.13 P.fls.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (shared with the 8850)

const ModelProfile model_8855 = {
    .name = "8855",
    .description = "Nokia 8855 (NSM-4, MAD2, PCD8544 84x48) — boots to standby (EEPROM image)",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB (8855 is 4 MB, unlike the 2 MB 8850/8210/8250)
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0x00580000u,   // 4 MB layout (same as the 5210 — verify per build)
        .eeprom_size = 0x00080000u,
    },
    .lcd = {
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        // NSM Family-A keypad, shared with the 8850. 8855 is a slide (like the 8850).
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,
        .has_slide = 1,               // NSM-4 slide cover (reed switch), like the 8850
    },
    .battery = {
        // Shared NSM Li-ion (BLB-2) battery-type window (== 8850/5210).
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 16,
    },
    .fw = {
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build DSP scratch — RE'd from 8855 v5.13 (ported from 8850 v5.31 sigs).
        // verdict = 0x17FDE5 (MMI_READY base 0x17FD7C + 0x69 — the 4 MB builds use 0x17FD7C
        // like the 5210, vs the 2 MB 0x13FD78). Bit2-writer 0x25F420 (prologue r5=ldr=0x17FD7C,
        // r6=#0x69, write [r6,r5]). dsp_uploaded = 0x13A210: DSP-block-done flag the IRQ4 ISR
        // 0x33D578 (98% fuzzy of 8850 0x2CB418) sets to 1 (at 0x33D5B0, when [0x100E4]==0).
        // The 8855's DSP upload completes natively (DSP acks ~3589) so no dsp_boot_status override.
        .verdict = 0x0017FDE5u, .sim_gate = 0, .dsp_uploaded = 0x0013A210u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
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
        .match = "NSM-4",            // 8855 product code (0x1FC header)
        .flash_size = 0x00400000u,   // 4 MB
    },
    .dsp = &mad2_dsp_default,
};
