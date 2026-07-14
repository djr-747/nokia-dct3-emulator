// Nokia 5510 model profile — SCAFFOLD (bring-up in progress).
//
// Built "4 MB 3310-based": the boot/platform scaffold reuses the 3310 HW base
// (84x48 PCD8544 LCD, Family B keypad) with the 5510's own 4 MB memory map. NB the real
// 5510 hardware is physically unusual for a DCT3 — a landscape "music" phone with a full
// QWERTY keyboard and a wide display — so the LCD geometry and (especially) the keypad
// MATRIX below are the 3310 scaffold, NOT the 5510's true panel/keyboard. They do not
// gate boot; refine them (RE the 5510 keymap + LCD wiring) once it boots far enough to
// drive the display/keys.
//
// Firmware on hand: firmware/Nokia 5510 NPM-5 v3.50 A.fls and v3.53 A.fls (both flat
// 4 MB .fls). Header @0x1FC: "V 03.50 / NPM-5" (and v3.53). The "NPM-5" + 4 MB ident
// serves both builds; the sigs resolve each build's own addresses.
//
// Memory map:, 5510 row (4 MB) — CONFIRMED against the
// WinTesla FIASCO block addresses (npm5nx03.500 MCU 0x200000..0x44BC00; npm5nx03.50a PPM
// 0x490000..0x550000) AND the assembled image (PPM\0 header @0x490000; 0x550000 = 0xFF):
//   MCU 0x200000-0x48FFFF   PPM 0x490000-0x54FFFF   EEPROM 0x550000-0x5FFFFF
// (Same partition layout as the 3330.)

#include "models/model.h"
#include "models/mad2_sigs.h"   // shared MAD2 RTOS firmware-address signatures

// 5510 keypad matrix — PLACEHOLDER (3310 Family B scaffold). The real 5510 has a full
// QWERTY keyboard with a different matrix that has NOT been RE'd. These lines only affect
// key DECODE, never boot; revisit once the phone boots and the 5510 keymap is located.
static const KeyLine keylines_5510[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,4,1,0},
    {KK_4,2,4,0}, {KK_5,2,3,0}, {KK_6,2,2,0},
    {KK_7,3,4,0}, {KK_8,3,3,0}, {KK_9,3,2,0},
    {KK_STAR,4,4,0}, {KK_0,0,2,0}, {KK_HASH,4,2,0},
    {KK_SOFT1,4,3,0},   // "Menu"
    {KK_SOFT2,0,4,0},   // "Names" / C (clear)
    {KK_UP,0,1,0}, {KK_DOWN,1,1,0},
    {KK_PWR,0,0,0x02},  // special-scan
};

const ModelProfile model_5510 = {
    .name = "5510",
    .description = "Nokia 5510 (NPM-5, MAD2, 4 MB) — 3310-based scaffold (LCD/keypad TBD)",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // 5510 (4 MB): EEPROM 0x550000-0x5FFFFF (CONFIRMED — PPM ends 0x550000; same as 3330).
        .eeprom_base = 0x00550000u,
        .eeprom_size = 0x000B0000u,   // 0x550000..0x600000
        // param_base/param_block_size left 0: generic MAD2 top-boot default
        // (top 64 KB = 8x 8 KB parameter blocks) at 0x5F0000..0x600000.
    },
    .lcd = {
        // 3310 scaffold geometry (84x48 PCD8544). The real 5510 panel is wider/landscape
        // — UNVERIFIED; refine once it renders. ctrl_* stay 0 (controller == visible here).
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        .power_special_cols = 0x02,   // PWR special bit1 (3310 mechanism, assumed)
        .family  = KP_FAMILY_3310,    // 3310 Family B scaffold (5510 is really QWERTY — TBD)
        .lines   = keylines_5510,
        .n_lines = (int)(sizeof(keylines_5510) / sizeof(keylines_5510[0])),
    },
    .battery = {
        // 3310 NiMH wedge-clean defaults; revisit once the 5510 reads its CCONT A/D.
        .vbatt = 0x220, .bsi = 0x100, .temp = 0x200, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,           // MAD2 default
        .asic_version = 0,          // -> read falls through to RAM (verify for 5510)
        // DSP-release quirk UNKNOWN. Start at the 3310 defaults (bit0 release). If boot
        // hangs in a pre-LCD spin polling [0x20002] (like the 3410's bit4 wait), RE the
        // release RMW and set the mask — see the 3410 bring-up §5 method.
        .dsp_reset_running = 0,
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
        .mdisnd_q     = 0x00010000u,    // MCU->DSP request queue (HPI layout-invariant; see 3310)
        .mdisnd_tail  = 0x000100A4u,    // base/tail verified (init + group-0x05 records decode); the
        // 0x70 self-test stream is never reached — 5510 bring-up stalls earlier (verdict/dsp_uploaded also un-RE'd)
        // Per-build scratch/reset addresses UNKNOWN for v3.50/v3.53. 0 = unresolved; the
        // shared MAD2 sigs locate what they can over the 5510's own flash region. The
        // boot-critical ones (dsp_uploaded, verdict) must be RE'd per-build (3310 values
        // do NOT transfer — the early RAM-init clobbers them; see 3410 bring-up §8.3).
        .verdict = 0, .sim_gate = 0, .dsp_uploaded = 0,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .boot = {
        .skip_seclock_default = 1,   // mirror 3310 web default
    },
    .ident = {
        .match = "NPM-5",            // 5510 product code (0x1FC version header)
        .flash_size = 0x00400000u,   // 4 MB — with NPM-5, unambiguous (serves v3.50 + v3.53)
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour (verify for 5510)
};
