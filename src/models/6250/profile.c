// Nokia 6250 model profile — NHM-3, the ruggedized sibling of the 6210 (NPE-3): a 4 MB MAD2
// DCT3 (2000) with an in-flash EEPROM partition, 96x65 graphic display, ROM-6 DSP. Clone of
// src/models/6210/profile.c (closest analogue) with per-build differences:
//
//   1. Ident "NHM-3" (no canonical 0x1FC header in the image; whole-image string match).
//   2. verdict re-located for v5.00 (the 6210 v5.56 gate sig doesn't match this older build;
//      found by a raw scan for the gate SHAPE — see .fw comment).
//
// DSP self-test = LIKE-FOR-LIKE with the 6210: dsp_uploaded=0x16E474 + the shared
// mad2_dsp_6210 responder drive it identically (block-ack pump sets the flag → verdict bit6
// survives gate-1; the group-0x74 sub-13 ack clears verdict bit2 → gate-2 passes). Both verified
// firing (bit2 clears @1.915M). But the 6250 does NOT yet reach standby: it has ONE EXTRA gate
// the 6210 passes — a CALIBRATION-RECORD CHECKSUM at 0x304520 (the 6210 has the byte-identical
// gate at 0x30282C and passes it). It recomputes a calib checksum (r6 = 0x3042B4(), = 0x6D57 in
// our model) and compares it to stored EEPROM record 0x254 (= 0x7095); the mismatch clears verdict
// bit6 @0x30454A (~1.911M, BEFORE the DSP bit2-ack lands and independent of it). So this v5.00
// image's stored calibration checksum is inconsistent with its own calib data (cf. the 5110's tune
// checksum, but here in the IN-FLASH EEPROM partition). Resolving it (recompute record 0x254, or
// fix whatever calib input our model feeds 0x3042B4) is the 6250's remaining barrier — the DSP
// self-test transfer itself is complete and correct. Rests at CONTACT SERVICE until then.
//
// Flash layout scan: f0f0 param headers at 0x5FA000/0x5FC000 (the generic
// top-64K parameter blocks, same as the 6210) + one at 0x534000; content dense to ~0x5A0000.
// eeprom_base below stays the 6210's ESTIMATE — verify against a dump map during bring-up.
//
// STATE (2026-07-14 first boot): detects and boots into the MMI to CONTACT SERVICE (the
// 6210-class resting state). Two barriers cleared on the way:
//   * a re-reset poll (0x4E7DC2: clear [0x20002] bits3+2, wait bit4-low) spun on a stale
//     synthesized status latched into RAM — fixed generically in mad2_bus.c (bit4 masked
//     low while the release bit is clear);
//   * with reg 0x0E = 0 the v5.00 boot classifies "no power-on cause" and powers straight
//     off (WDT=0 @1.4M) — unlike the 6210, which never reads the cause back. Fixed by
//     ccont_poweron_int below (3210-style).

#include "models/model.h"
#include "models/mad2_sigs.h"
#include "models/keylines_nsm_a.h"

const ModelProfile model_6250 = {
    .name = "6250",
    .description = "Nokia 6250 (NHM-3, MAD2, 4 MB, 96x65) — in-flash EEPROM",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // In-flash EEPROM/NVRAM partition — the 6210's ESTIMATE, carried over (same top-64K
        // param blocks + same dense-to-0x5A0000 shape in the v5.00 image). Verify vs a dump map.
        .eeprom_base = 0x00580000u,
        .eeprom_size = 0x00080000u,
    },
    .lcd = {
        // 96x65 graphic display (6210/3410/7110 class), PCD8544-compatible decode over the
        // MAD2 GENSIO offsets — carried from the 6210; verify controller/wiring at bring-up.
        .controller = LCD_PCD8544,
        .width = 96, .height = 65, .banks = 9,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        // 6210 keypad model carried over (shared MAD2 matrix keymap + DIR-aware scan +
        // IM_C source-decode). The 6250's own keymap table should be verified at bring-up.
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,    // GUI visual layout (2 soft keys + send/end + vol)
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,
    },
    .led = { .lcd_rgb = 0x55C955, .kbd_rgb = 0x55C955 },
    .battery = {
        // Standard MAD2 CCONT A/D wiring assumed (== 6210 v5.56, verified there); the 6250
        // shares the 6210 board family. Values sit inside the 6210's accept windows.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,
        // DSP release/ready model carried from the 6210 v5.56 RE (release bit2, poll bit4,
        // status read-back 0x53). Same NPE-3/NHM-3 board family — verify if the v5.00 boot
        // spins at the DSP-release poll.
        .dsp_reset_running = 0x53,
        .dsp_release_mask  = 0x04,
    },
    // Power-on cause: UNLIKE the 6210 (which reads CCONT reg 0x0E once and never uses it),
    // the 6250 v5.00 boot classifies the power-on reason and does a CLEAN POWER-OFF (WDT=0
    // @1.4M) when reg 0x0E reads 0 — the 3210-style "no valid power-on cause" path. Latch
    // bit1 ("powered on by PWR key") at cold reset, as on the 3210 (a cold boot = the user
    // holding PWR).
    .ccont_poweron_int = 0x02,
    .fw = {
        // HW-fixed MAD2 HPI mailbox / MDIRCV (shared across MAD2 DCT3).
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
        // verdict 0x17FD15 — the 6210 v5.56 gate sig does NOT match v5.00, so the gate was
        // re-located by scanning for its exact SHAPE (`ldr rX,=LIT; ldrb r0,[rX]; lsr r0,#7;
        // bcs`, literal in RAM): unique MMI-ready-class hit at 0x306CF2 `ldr r4,=0x17FD15`,
        // with the identical strb-[r5,#10..] tail as the 6210's gate 0x30501A (verdict
        // 0x17FD99 there).
        // dsp_uploaded = 0x16E474 — the self-test result flag, LIKE-FOR-LIKE with the 6210's
        // 0x16FFE4 (RE'd 2026-07-14; gen_sig of the 6210's self-test-pass leaf 0x4AFFB4 → 6250
        // 0x4C0F28 `ldr =0x16E474; ldrb; bx lr`, and setter 0x42A054 `ldr =0x16E474`). The shared
        // ROM-6 self-test responder (mad2_dsp_6210) drives it exactly as on the 6210: the block-
        // ack pump raises IRQ4 → firmware sets [0x16E474]=1 (gate-1) → responder posts the
        // group-0x74 sub-13 ack → handler clears verdict bit2 (gate-2) → standby.
        .verdict = 0x0017FD15u, .sim_gate = 0, .dsp_uploaded = 0x0016E474u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        // Reset/reboot path: unlike the 6210 (v5.56-pinned constants), no v5.00 values are
        // RE'd yet — leave 0 and let the shared MAD2_SIGS resolve them (sigdump shows misses).
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0,
        .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .boot = {
        .skip_seclock_default = 1,
    },
    .ident = {
        .match = "NHM-3",
        .flash_size = 0x00400000u,
    },
    .dsp = &mad2_dsp_6210,            // shared ROM-6 self-test-complete responder (like-for-like 6210)
    // Repair this v5.00 library image's inconsistent RF-calibration checksum: the self-test at
    // 0x304520 recomputes sum(EEPROM calib 0x120..0x133) = 0x6D57 and compares it to the stored
    // checksum record (EEPROM offset 0x254) = 0x7095 — mismatch clears verdict bit6. The FFS entry
    // {0x0254, 0x7095} lives at flash 0x5FAAEC; write the correct sum into its value halfword
    // (flash 0x5FAAEE) so a factory-consistent calibration passes. See model.h calib_cksum_*.
    .calib_cksum_off = 0x005FAAEEu,
    .calib_cksum_val = 0x6D57u,
};
