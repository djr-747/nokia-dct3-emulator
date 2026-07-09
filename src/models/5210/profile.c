// Nokia 5210 model profile — SCAFFOLD (bring-up in progress, following the 3410
// method in docs/3410-bringup.md / docs/5210-bringup.md).
//
// Firmware on hand: firmware/Nokia 5210 NSM-5 v5.40 A.fls (flat 4 MB .fls, already
// assembled). Header @0x1FC: "V  5.40 / 11-10-03 / NSM-5 / (c) NMP", fw-id TBD.
// Product code NSM-5, 4 MB image.
//
// Memory map: docs/dct3-flash-partition-map.md, 5210 row (4 MB):
//   MCU 0x200000-0x48FFFF   PPM 0x490000-0x57FFFF   EEPROM 0x580000-0x5FFFFF
// (Note the EEPROM start is 0x580000 — LATER than the 3410's 0x570000 because the
//  5210 carries a larger PPM. This is the one hard difference from the 3410 map.)
//
// Like the 3410 scaffold, the .fw per-build RAM/code addresses are all 0 (the
// 3310-v5.79 constants are WRONG for this v5.40 build). The shared MAD2 RTOS
// signatures (src/models/mad2_sigs.c) locate them at runtime over the 5210's own
// flash region. mad2 degrades gracefully on the unresolved 0s. See the bring-up
// doc for which barriers (DSP-release / CONTACT SERVICE) the boot hits and the
// per-build address resolutions that clear them — the same S5-class pattern as the
// 3410 (dsp_release_mask / dsp_uploaded / verdict).

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h" // shared NSM Family-A keypad matrix (== 8850)

const ModelProfile model_5210 = {
    .name = "5210",
    .description = "Nokia 5210 (NSM-5, MAD2, 4 MB) — SCAFFOLD (bring-up in progress)",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // 5210 (4 MB): EEPROM 0x580000-0x5FFFFF (MCU+PPM below). EEPROM start is
        // 0x580000 — 0x10000 higher than the 3410's 0x570000 (larger 5210 PPM).
        .eeprom_base = 0x00580000u,
        .eeprom_size = 0x00080000u,   // 0x580000..0x600000
        // param_base/param_block_size left 0: the generic MAD2 top-boot default
        // (top 64 KB = 8x 8 KB parameter blocks) applies — here 0x5F0000..0x600000.
        // See mad2_flash.c flash_erase_blk.
    },
    .lcd = {
        // 84x48 PCD8544 — same panel as the 3310 (confirmed on-screen, Dan 2026-06-05;
        // was a 96x65/102x72 placeholder copied from the 3410). ctrl_* dropped so the
        // controller RAM == the visible 84x48. io_data/io_cmd = 3310 GENSIO offsets.
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
        .x_mirror = 1,   // 5210 panel has REVERSED segment order (text renders backwards
                         // without this) — the one way its LCD differs from other DCT3.
    },
    .keypad = {
        // NSM Family-A keypad, SHARED with the 8850 (per Dan: 5210/82xx/8855 all match
        // the 8850 matrix). The 5210 is a candybar, not a slide -> has_slide stays 0.
        .power_special_cols = 0x02,   // PWR special-scan bit1
        .family  = KP_FAMILY_8210,    // Family A: soft keys, up/down, send/end, volume
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,                 // matrix IRQ-source decode (8850-class)
    },
    // Period-correct backlight: the orange glow (Dan, 2026-06-12). 0xRRGGBB,
    // consumed by the harness tints (web palette / gui_sdl LCD bg).
    .led = { .lcd_rgb = 0xFFA040, .kbd_rgb = 0xFFA040 },
    .battery = {
        // 5210 battery-type detection (RE 2026-06-05): the pre-MMI boot verdict
        // 0x3C8B52 (boot-cause reason 10) reads CCONT A/D ch3 (BSI) + ch4 (temp) via
        // 0x3E7B58 and stores a type to [0x13E960]; type 0 -> deliberate clean
        // power-off (WDT=0 at 0x3E42C8 via the teardown at 0x2DDAD4). Empirically the
        // valid windows are BSI in [0x13E,0x172] AND temp in [0x133,0x158) — IDENTICAL
        // to the 8850 (NSM-2, fn 0x2FF716 mode 10). This battery-type window + values are
        // SHARED across the NSM Li-ion (BLB-2) family: 5210 / 8210 / 8250 / 8850 (Dan,
        // 2026-06-05) — reuse these for those bring-ups. The 3310 NiMH defaults
        // (bsi 0x100, temp 0x200) fall OUTSIDE both -> type 0 -> power-off. These
        // mid-window values give a valid type and let the boot proceed past ~1.8M.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,           // MAD2 default
        .asic_version = 0,          // unknown for 5210 -> read falls through to RAM
        // DSP-release quirk UNKNOWN for the 5210. Start at the defaults (3310 bit0
        // release, 0x01). If the boot hangs in a tight pre-LCD spin polling [0x20002]
        // (as the 3410 did at 0x3CBADC on bit4), RE the release RMW and set
        // dsp_reset_running / dsp_release_mask to match — see bring-up §"DSP release".
        .dsp_reset_running = 0,
        .dsp_release_mask  = 0,     // 0 -> mad2 default 0x01 (bit0), like 3310/8850/7110
    },
    .fw = {
        // DSP shared-RAM mailbox / MDIRCV queue: the MAD2 HPI layout is assumed shared
        // across MAD2 DCT3 (verify against the 5210 image before relying on it).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build scratch + reset/heap/task-14 addresses UNKNOWN for this v5.40 build.
        // 0 = unresolved (mad2 degrades gracefully). The shared MAD2 RTOS signatures
        // (.sigs below) locate what they can over the 5210's own flash region; the
        // boot-critical ones (dsp_uploaded, verdict — and on the 3410, sim_gate) must
        // be RE'd per-build (the 3310/3410 values do NOT transfer; the early RAM-init
        // clobbers them). See bring-up doc for the resolution method.
        // dsp_uploaded = the 5210 DSP-ready gate byte [0x13B528+0] (RE 2026-06-05).
        // The MMI startup sequencer (0x3B2940) reads it at 0x3B29A2: ==0 -> fallback
        // dialog set -> fatal spin 0x3E42C8. It is set to 1 (once, sticky) by the DSP
        // ISR DSP_READY_HANDLER 0x345DFC (store at 0x345E34), which mad2's shared
        // dsp_default IRQ4 pump invokes. Wiring it here un-gates the IRQ4 raise during
        // the code-block upload (mad2 raises IRQ4 while [dsp_uploaded]==0) so the ISR
        // runs and sets the gate -- the same per-build dsp_uploaded fix as the 3410
        // (0x12BA10) / 8850. Direct analogue of the 3310's [0x11038C].
        // verdict = 0x17FDE5 (5210-specific; analog of 8850 0x13FDE1 / 3310 0x11FF15):
        // DSP self-test verdict / MMI-ready byte = MMI_READY struct base 0x17FD7C + 0x69
        // (same 0x69 invariant offset as 3310/3410/8850). Located by porting the 8850
        // verdict-bit2 writer 0x240B60 (91% fuzzy sig -> 5210 0x25D34C) whose function
        // prologue at 0x25D24C sets r6=#105(0x69) and r5=ldr=0x17FD7C, so [r6,r5]=0x17FDE5.
        // bit2(0x04)="DSP self-test pending" set by writer 0x25D34C right after streaming
        // the request to the DSP (bl 0x2B5518 = 8850's 0x284E28); bit6/7 by 0x25D2DC/E4.
        // The shared mad2 DSP responder posts the group-0x74 reply + FIQ0 only while
        // [verdict]&0x04; without this it read m->mem[0], never replied -> bit2 stayed
        // pending. RAMWATCH (before this fix): 00->40->C0->C4 @1.96M then 0x84 STALL
        // (bit6 cleared, bit2 never cleared) -> CONTACT SERVICE.
        .verdict = 0x0017FDE5u, .sim_gate = 0, .dsp_uploaded = 0x0013B528u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        // NOTE: the 8850's DSP-boot fields (dsp_boot_status=0x10004 / dsp_boot_ready=6)
        // do NOT port to the 5210 — A/B tested (no effect on the verdict path). The 8850
        // polls an HPI mailbox boot-version slot [0x10004] (real MCU literal @0x337444),
        // whereas the 5210 MCU code has NO word-aligned [0x10004] literal in its code pool
        // and gates verdict bit6 on a DIFFERENT mechanism: an EEPROM-record self-test
        // result comparison (verdict fn 0x25D24C reads expected from EEPROM rec 0x6509 via
        // 0x25D10C, streams the request from EEPROM rec 10 via 0x2B5518, then at 0x25D37E
        // compares DSP reply [sp+4] vs expected r9 — mismatch -> 0x25D3F4 clears bit6).
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    // Shared MAD2 RTOS signatures locate the per-build addresses over the 5210's own
    // flash region (sig hits OVERRIDE the 0 fallbacks in .fw above). The [sigdump] line
    // at boot reports which resolved (MISS = still needs locating).
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    // 5210 overlay: signature-resolve verdict + dsp_uploaded per build. The .fw constants
    // above are the v5.40 values; these sigs override them with the build's own addresses,
    // so every 5210 firmware (v5.13..v5.40) comes up — not just v5.40. A v5.22 dump stalled
    // pre-MMI precisely because its dsp_uploaded is 0x13B150, not the hardcoded 0x13B528.
    .sigs2 = MAD2_SIGS_5210,
    .n_sigs2 = MAD2_N_SIGS_5210,
    .boot = {
        .skip_seclock_default = 1,   // mirror 3310/3410 web default
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NSM-5",            // 5210 product code (0x1FC version header)
        .flash_size = 0x00400000u,   // 4 MB — with the NSM-5 string, unambiguous
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour (verify for 5210)
};
