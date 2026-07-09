// Nokia 8250 model profile — BOOTS TO STANDBY (2026-06-06). NSM-3D, 2 MB MAD2 candybar —
// the 8210's twin (same NSM Family-A class: shared keypad matrix + BLB-2 battery). v6.02
// is a LATER build than the 8850/8210 v5.31, so the 8850 DSP sigs port only FUZZY (the
// addresses shift) — verdict/dsp_uploaded below were re-pinned against this image. Unlike
// the 8210, the stock 8250 EEPROM dump is self-consistent (no stale-checksum fix needed),
// so it boots straight to standby from the unmodified image. Firmware on hand:
// firmware/Nokia 8250 NSM-3D v6.02 J.fls. See docs/8210-8250-bringup.md.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (shared with the 8850)

const ModelProfile model_8250 = {
    .name = "8250",
    .description = "Nokia 8250 (NSM-3D, MAD2, PCD8544 84x48) — boots to standby",
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
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        // NSM Family-A keypad, shared with the 8850. 8250 is a candybar (no slide).
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,
    },
    // Period-correct backlight: the blue glow (Dan, 2026-06-12). 0xRRGGBB,
    // consumed by the harness tints (web palette / gui_sdl LCD bg).
    .led = { .lcd_rgb = 0x6FA8FF, .kbd_rgb = 0x6FA8FF },
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
        // Per-build DSP scratch — RE'd from 8250 v6.02 (ported from the 8850 v5.31 sigs;
        // v6.02 shifts addresses vs v5.31 so these are fuzzy-matched, not exact).
        // dsp_uploaded = 0x12F33C: the DSP-code-block-done flag the IRQ4 ISR 0x2CBD30 sets
        // to 1 (at 0x2CBD68, when reply reg [0x100E4]==0) — ported from the 8850 ISR
        // 0x2CB418 (98% fuzzy). verdict = 0x13FDD9 (DSP self-test / MMI-ready byte =
        // MMI_READY base 0x13FD70 + 0x69; the v6.02 struct sits 8 bytes below the v5.31
        // 0x13FD78). Located via the bit2-writer 0x2449EC (96% fuzzy of 8850 0x240B60):
        // prologue r6=#0x69, r5=ldr=0x13FD70, write [r6,r5]=[0x69+0x13FD70]=0x13FDD9.
        // bit2(0x04)="DSP self-test pending"; the shared mad2 responder posts the group-0x74
        // reply + FIQ0 only while [verdict]&0x04. (8250 v6.02 uploads the DSP code via the
        // mad2-default 0x10002 boot slot — DSP acks already fire — so no dsp_boot_status
        // override is needed, unlike the 8210/8850 v5.31 which park 0x10004.)
        .verdict = 0x0013FDD9u, .sim_gate = 0, .dsp_uploaded = 0x0012F33Cu,
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
        .match = "NSM-3D",           // 8250 product code (0x1FC header) — more specific
        .flash_size = 0,             // than 8210's "NSM-3"; MUST be registered first.
                                     // String-only match (like 8850).
    },
    .dsp = &mad2_dsp_default,
};
