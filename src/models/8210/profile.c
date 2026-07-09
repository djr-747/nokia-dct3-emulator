// Nokia 8210 model profile — BOOTS TO STANDBY (2026-06-06). NSM-3, 2 MB MAD2 candybar
// of the same NSM Family-A class as the 8850 (shared keypad matrix + BLB-2 battery
// window). 8210 v5.31 is the SAME build version as the 8850 v5.31, so the 8850 DSP
// sigs port exactly (verdict/dsp_uploaded/dsp_boot_status below). Firmware on hand:
// firmware/Nokia 8210 NSM-3 v5.31 A.fls (v5.11–v5.30 variants also present).
//
// EEPROM CAVEAT: the stock dump's DSP-calibration record carries a STALE stored checksum
// (logical 0x0254 = 0x7B26) that mismatches the firmware-computed checksum (0x77E2) -> the
// DSP self-test fails -> CONTACT SERVICE. This is NOT a blank EEPROM (the partition is
// populated) — only one checksum is stale. Boot the pre-fixed image
// `firmware/Nokia 8210 NSM-3 v5.31 A (EEPROM).fls` (signature-located CKFIX: framed journal
// header `08 f7 02 54`, stored BE u16 -> 0x77E2). See docs/8210-8250-bringup.md.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (shared with the 8850)

const ModelProfile model_8210 = {
    .name = "8210",
    .description = "Nokia 8210 (NSM-3, MAD2, PCD8544 84x48) — boots to standby",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0x003D0000u,   // 2 MB layout (same as 8850 — verify per build)
        .eeprom_size = 0x00030000u,
    },
    .lcd = {
        // 84x48 PCD8544 — the de-facto DCT3 panel. (No x_mirror: only the 5210's
        // panel has reversed segment order among this family.)
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        // NSM Family-A keypad, shared with the 8850. 8210 is a candybar (no slide).
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,
    },
    .battery = {
        // Shared NSM Li-ion (BLB-2) battery-type window (== 8850/5210): BSI in
        // [0x13E,0x172] AND temp in [0x133,0x158). 3310 NiMH defaults fall outside.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 16,   // NSM Family-A 16-source IRQ model (like the 8850)
    },
    .fw = {
        // Shared MAD2 HPI mailbox / MDIRCV queue layout (verify per build).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build DSP scratch — RE'd from 8210 v5.31 (same build version as the 8850,
        // so the 8850 sigs port exactly). dsp_uploaded = 0x135774: the DSP-code-block-done
        // flag the IRQ4 ISR 0x2CB338 sets to 1 (at 0x2CB370, when reply reg [0x100E4]==0)
        // once the upload completes — ported from the 8850 ISR 0x2CB418 (exact sig match;
        // 8850 flag 0x135664). The shared mad2 DSP pump raises the block-ack IRQ4 only
        // while [dsp_uploaded]==0. verdict = 0x13FDE1 (DSP self-test / MMI-ready byte =
        // MMI_READY base 0x13FD78 + 0x69) — IDENTICAL address to the 8850 (the bit2-writer
        // 0x240B54 ported exactly from 8850 0x240B60; prologue r4=ldr=0x13FD78, r5=#0x69).
        // bit2(0x04)="DSP self-test pending"; the shared mad2 responder posts the group-0x74
        // reply + FIQ0 only while [verdict]&0x04 — without this it read m->mem[0], never
        // replied -> bit2 stayed pending -> CONTACT SERVICE.
        .verdict = 0x0013FDE1u, .sim_gate = 0, .dsp_uploaded = 0x00135774u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        // DSP boot handshake (ported from 8850 v5.31 — same build): the startup launcher
        // 0x2CAC80 parks HPI mailbox slot [0x10004] at 0xFFFF and polls it (fn 0x2CAD46)
        // for a DSP ready/version reply of 5 or 6. Returning dsp_boot_ready lets the boot
        // upload the DSP code blocks (handshaked via dsp_mbox0/1), which writes dsp_cb_reply
        // -> the mad2 pump raises the block-ack IRQ4 -> ISR 0x2CB338 sets [dsp_uploaded]=1.
        // Without it the poll times out, no upload, dsp_uploaded stays 0 -> the verdict gate
        // 0x2F69A4 (reads [dsp_uploaded]) skips the DSP self-test -> CONTACT SERVICE.
        .dsp_boot_status = 0x00010004u, .dsp_boot_ready = 6,
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
        .match = "NSM-3",            // 8210 product code (0x1FC header). NOTE: prefix of
        .flash_size = 0,             // "NSM-3D" (8250) — register 8250 BEFORE 8210. String-
                                     // only match (like 8850); NSM-3 is unique to 8210/8250.
    },
    .dsp = &mad2_dsp_default,
};
