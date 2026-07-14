// Nokia 3350 model profile — SCAFFOLD (bring-up in progress).
//
// the 3350 is HW-identical to the 3410 — SAME LCD (96x65 visible / 102x72
// controller PCD8544-compatible) and SAME keypad (Family C: 2 soft keys, up/down,
// send/end, no volume). It differs from the 3410 only in the memory map (its own 4 MB
// partition layout) and the per-build firmware addresses (sig-located at runtime).
//
// Firmware on hand: firmware/Nokia 3350 NHM-9 v5.22 A.fls (flat 4 MB .fls). Header
// @0x1FC: "V 05.22 / 25-06-02 / NHM-9 / (c) NMP".
//
// Memory map:, 3350 row (4 MB) — CONFIRMED against the
// WinTesla FIASCO block addresses (Nhm9nx05.220 MCU 0x200000..0x470000; Nhm9nx05.22a PPM
// 0x470000..0x530000):
//   MCU 0x200000-0x46FFFF   PPM 0x470000-0x52FFFF   EEPROM 0x530000-0x5FFFFF

#include "models/model.h"
#include "models/mad2_sigs.h"   // shared MAD2 RTOS firmware-address signatures

// 3350 keypad matrix = the 3410's (Family C) ("3410 & 3350 same" keys). RE'd
// 3410 table reused verbatim; treat the green/red orientation as inherited-inferred.
static const KeyLine keylines_3350[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,4,1,0},
    {KK_4,2,4,0}, {KK_5,2,3,0}, {KK_6,2,2,0},
    {KK_7,3,4,0}, {KK_8,3,3,0}, {KK_9,3,2,0},
    {KK_STAR,4,4,0}, {KK_0,0,2,0}, {KK_HASH,4,2,0},
    {KK_UP,4,3,0}, {KK_DOWN,0,3,0},
    {KK_SOFT1,0,4,0}, {KK_SOFT2,0,1,0},   // left / right soft key
    {KK_SEND,1,4,0}, {KK_END,1,1,0},      // green / red
    {KK_PWR,0,0,0x02},                     // special-scan bit1
};

const ModelProfile model_3350 = {
    .name = "3350",
    .description = "Nokia 3350 (NHM-9, MAD2, 96x65/102x72 LCD, 4 MB) — 3410 HW, bring-up scaffold",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // 3350 (4 MB): EEPROM 0x530000-0x5FFFFF (CONFIRMED — PPM ends 0x530000). NB this
        // is LOWER than the 3410's 0x570000 (the 3350's MCU+PPM are smaller).
        .eeprom_base = 0x00530000u,
        .eeprom_size = 0x000D0000u,   // 0x530000..0x600000
        // param_base/param_block_size left 0: generic MAD2 top-boot default
        // (top 64 KB = 8x 8 KB parameter blocks) at 0x5F0000..0x600000.
    },
    .lcd = {
        // IDENTICAL to the 3410 — 96x65 visible, 102x72 controller RAM, PCD8544-compatible
        // (same command set as the 3310; only the geometry differs). See the 3410 profile
        // for the two mad2.c geometry caveats (set-Y bank mask, ctrl_width stride).
        .controller = LCD_PCD8544,
        .width = 96, .height = 65, .banks = 9,      // visible
        .io_data = 0x2E, .io_cmd = 0x6E,
        .ctrl_width = 102, .ctrl_height = 72, .ctrl_banks = 9,  // controller RAM
    },
    .keypad = {
        .power_special_cols = 0x02,   // PWR special bit1 (3410 mechanism)
        .family  = KP_FAMILY_3410,    // Family C (3410): soft keys, up/down, send/end, no vol
        .lines   = keylines_3350,
        .n_lines = (int)(sizeof(keylines_3350) / sizeof(keylines_3350[0])),
    },
    .battery = {
        // 3310/3410 NiMH wedge-clean defaults; revisit once the 3350 reads its CCONT A/D.
        .vbatt = 0x220, .bsi = 0x100, .temp = 0x200, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,           // MAD2 default
        .asic_version = 0,          // -> read falls through to RAM (verify for 3350)
        // DSP-release quirk: the 3350 shares the 3410's HW. The 3410 (v5.46) releases the
        // DSP via [0x20002] bit2 then polls bit4 (dsp_release_mask=0x04). Whether the 3350
        // (v5.22) firmware uses the same release bit is BUILD-dependent — start at the
        // defaults and RE the release RMW during bring-up; set 0x04 if it hangs at the
        // bit4 poll (the 3410's §5 barrier).
        .dsp_reset_running = 0,
        .dsp_release_mask  = 0,     // 0 -> mad2 default 0x01 (bit0); revisit (3410 = 0x04)
    },
    .fw = {
        // DSP shared-RAM mailbox / MDIRCV queue (MAD2 HPI layout, shared across DCT3).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build scratch/reset addresses UNKNOWN for v5.22. 0 = unresolved; the shared
        // MAD2 sigs locate what they can. The boot-critical verdict/dsp_uploaded cells
        // are resolved by the NHM family-line signatures (.sigs2 = MAD2_SIGS_3310 below):
        // both anchors hit UNIQUELY on v5.22 (verified 2026-07-14: verdict=0x13FE61,
        // dsp_uploaded=0x120DD4; the latter cross-checked against this build's DSP
        // message pump at 0x31E248 — an exact gen_sig port of the 3310's 0x2BB430 —
        // which does the same `strb 1,[0x120DD4]` gated on cb_reply [0x100E4]==0).
        .verdict = 0, .sim_gate = 0, .dsp_uploaded = 0,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .sigs2 = MAD2_SIGS_3310,      // NHM-line verdict + dsp_uploaded self-heal (per-build RAM cells)
    .n_sigs2 = MAD2_N_SIGS_3310,
    .boot = {
        .skip_seclock_default = 1,   // mirror 3410 web default
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NHM-9",            // 3350 product code (0x1FC version header)
        .flash_size = 0x00400000u,   // 4 MB — with NHM-9, unambiguous
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour (verify for 3350)
};
