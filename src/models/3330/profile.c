// Nokia 3330 model profile — SCAFFOLD (bring-up in progress).
//
// the 3330 is HW-identical to the 3310 — SAME LCD (84x48 PCD8544) and SAME
// keypad (Family B: Menu/Names(C), up/down, digits) — it is essentially "a 4 MB 3310".
// The only differences from the 3310 are the memory map (4 MB image, EEPROM pushed
// late) and the per-build firmware addresses (located by signature at runtime).
//
// Firmware on hand: firmware/Nokia 3330 NHM-6 v4.16 A.fls and v4.50 A.fls (both flat
// 4 MB .fls). Header @0x1FC: "V 04.16 / NHM-6" (and v4.50). The "NHM-6" + 4 MB ident
// serves both builds; the sigs resolve each build's own addresses.
//
// Memory map:, 3330 row (4 MB) — CONFIRMED against the
// WinTesla FIASCO block addresses (NHM6NX04.160 MCU 0x200000..0x436400; NHM6NX04.16A PPM
// 0x490000..0x550000):
//   MCU 0x200000-0x48FFFF   PPM 0x490000-0x54FFFF   EEPROM 0x550000-0x5FFFFF

#include "models/model.h"
#include "models/mad2_sigs.h"   // shared MAD2 RTOS firmware-address signatures

// 3330 keypad matrix = byte-identical to the 3310 (Family B). RE'd 3310 table reused
// verbatim ("3310 & 3330 same" keys).
static const KeyLine keylines_3330[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,4,1,0},
    {KK_4,2,4,0}, {KK_5,2,3,0}, {KK_6,2,2,0},
    {KK_7,3,4,0}, {KK_8,3,3,0}, {KK_9,3,2,0},
    {KK_STAR,4,4,0}, {KK_0,0,2,0}, {KK_HASH,4,2,0},
    {KK_SOFT1,4,3,0},   // "Menu"
    {KK_SOFT2,0,4,0},   // "Names" / C (clear)
    {KK_UP,0,1,0}, {KK_DOWN,1,1,0},
    {KK_PWR,0,0,0x02},  // special-scan
};

const ModelProfile model_3330 = {
    .name = "3330",
    .description = "Nokia 3330 (NHM-6, MAD2, 84x48 LCD, 4 MB) — 3310 HW, bring-up scaffold",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // 3330 (4 MB): EEPROM 0x550000-0x5FFFFF (CONFIRMED — PPM ends 0x550000).
        .eeprom_base = 0x00550000u,
        .eeprom_size = 0x000B0000u,   // 0x550000..0x600000
        // param_base/param_block_size left 0: generic MAD2 top-boot default
        // (top 64 KB = 8x 8 KB parameter blocks) at 0x5F0000..0x600000.
    },
    .lcd = {
        // IDENTICAL to the 3310 — 84x48 monochrome PCD8544, GENSIO data/cmd. No
        // controller-RAM split (controller == visible), so ctrl_* stay 0.
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        .power_special_cols = 0x02,   // power key = special-scan column 1 (3310)
        .family  = KP_FAMILY_3310,    // Family B (3310): Menu/Names(C), up/down, digits
        .lines   = keylines_3330,
        .n_lines = (int)(sizeof(keylines_3330) / sizeof(keylines_3330[0])),
    },
    .battery = {
        // 3310 NiMH wedge-clean defaults (same HW family as the 3310).
        .vbatt = 0x220, .bsi = 0x100, .temp = 0x200, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,           // MAD2 default
        // The 3330 is 3310 HW (identical except the memory map), so the ASIC /
        // DSP-release read-backs are the SAME silicon values as the 3310. Leaving these at
        // 0 made MMIO 0x20000 (ASIC ver) fall through to RAM (bogus ASIC ID into the DSP
        // shared-mem slot 0x101E2) and MMIO 0x20002 never report the post-release DSP
        // clock/ready/API status (0x53) — so the firmware's DSP-ready poll stalls, the
        // self-test verdict never lands, and the MMI latches CONTACT SERVICE.
        .asic_version = 0xA1,       // MAD2 ASIC version @0x20000 — 3310 real-HW value
        .dsp_reset_running = 0x53,  // I/O 0x20002 read-back after DSP release — 3310 real-HW
        .dsp_release_mask  = 0,     // 0 -> mad2 default 0x01 (bit0), like the 3310
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
        // Per-build scratch/reset addresses UNKNOWN for v4.16/v4.50 (the 3310 v5.79
        // constants would mis-fire). 0 = unresolved; the shared MAD2 sigs locate what
        // they can over the 3330's own flash region.
        // The boot-critical verdict/dsp_uploaded cells are resolved per-build by the
        // NHM family-line signatures (.sigs2 = MAD2_SIGS_3310 below): both anchors hit
        // UNIQUELY on v4.16 AND v4.50 (verified 2026-07-14: verdict=0x13FE59 on both;
        // dsp_uploaded v4.50=0x122F10, v4.16=0x12305C — genuinely per-build). The old
        // hardcoded pair here (verdict=0x11FF15 / dsp_uploaded=0x110360) was borrowed
        // from the 3310 v5.57 line and NEVER matched these builds — that mis-aim is why
        // the upload latch/verdict chain "RE" stalled at Contact Service.
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
        .skip_seclock_default = 1,   // mirror 3310 web default
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match  = "NHM-6",           // 3330 product code (0x1FC version header)
        // Cooked 4 MB builds (e.g. the "3330_10.01n" ToolKit image) carry the 3310's
        // NHM-5 code instead of NHM-6 but are 3310 HW on the 4 MB map — this profile is
        // the right home for them. Accepted here (not by the 2 MB-pinned 3310) because
        // the size guard below (4 MB) keeps it off real 2 MB 3310 dumps.
        .match2 = "NHM-5",
        .flash_size = 0x00400000u,   // 4 MB — with NHM-6/NHM-5 + 4 MB, unambiguous
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour (verify for 3330)
};
