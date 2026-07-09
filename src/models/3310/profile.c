// Nokia 3310 model profile. Captures the constants the platform + shells used to
// hardcode (see git history of src/mad2/mad2.c, src/web/main.c, tools/boot_trace.c).
// This is the reference profile; behaviour must stay identical to pre-refactor.

#include "models/model.h"
#include "models/mad2_sigs.h"   // shared MAD2 RTOS firmware-address signatures

// 3310 keypad matrix (Family B). (row,col) verified against the firmware keymap table
// 0x32E718 + the long-standing GUI/web mapping; PWR is special-scan col 1. The 3310
// has Menu (left) + Names/C (right) rather than two soft keys + send/end.
static const KeyLine keylines_3310[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,4,1,0},
    {KK_4,2,4,0}, {KK_5,2,3,0}, {KK_6,2,2,0},
    {KK_7,3,4,0}, {KK_8,3,3,0}, {KK_9,3,2,0},
    {KK_STAR,4,4,0}, {KK_0,0,2,0}, {KK_HASH,4,2,0},
    {KK_SOFT1,4,3,0},   // "Menu"
    {KK_SOFT2,0,4,0},   // "Names" / C (clear)
    {KK_UP,0,1,0}, {KK_DOWN,1,1,0},
    {KK_PWR,0,0,0x02},  // special-scan
};

const ModelProfile model_3310 = {
    .name = "3310",
    .description = "Nokia 3310 (NHM-5, MAD2, PCD8544 84x48)",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0x003D0000u,   // NVRAM/EEPROM partition (MCU+PPM below)
        .eeprom_size = 0x00030000u,   // 0x3D0000..0x400000
    },
    .lcd = {
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        .power_special_cols = 0x02,   // power key = special-scan column 1
        .family  = KP_FAMILY_3310,    // Family B: Menu/Names(C), up/down, digits
        .lines   = keylines_3310,
        .n_lines = (int)(sizeof(keylines_3310) / sizeof(keylines_3310[0])),
    },
    .battery = {
        // Must sit in a narrow wedge-clean window: <=0x210 enters the firmware's
        // low-battery path which corrupts the soft-timer queue (head [0x11182C])
        // ~570M steps in and wedges the CPU at a garbage PC; >=0x228 scales (coeff
        // ~1.008, offset ~0 — vbatt scaler 0x2D727C) above the "battery full"
        // threshold (>556) where TASK_17_CHARGE asserts 0x5C0A. 0x220 is below
        // "full" (charger animates the bars) yet clear of the low-battery wedge.
        // The wide-open root fix is real EEPROM vbatt calibration (Tune 0x0205).
        //
        // vbatt stays at the wedge-clean 0x220: A/B'd on a long native run, 0x2C0
        // scales to ~0x2C5 (>556) and trips the charge/broker-fault assert chain
        // (BROKER_FAULT_NOTIFIER 0x2D724C → code 0x5B06 → reason-5 reset ~28M cyc),
        // even with no charger. 0x220 runs clean. BSI restored to the original 0x100
        // (recognised-pack battery-type path) — the combination that gave the early
        // emu its healthy full-battery boot without the assert.
        .vbatt = 0x220,
        .bsi   = 0x100,   // recognised pack (<=284 passes the battery-type check)
        .temp  = 0x200,   // room temp (in the charge window)
        .charger = 0x000, // no charger connected
    },
    .asic = {
        .irq_sources = 8,
        .asic_version = 0xA1,   // MAD2 ASIC version @0x20000 — real-HW confirmed (Dan)
        .dsp_reset_running = 0x53,  // I/O 0x20002 read-back after DSP release — real-HW (Dan)
    },
    .fw = {
        // Boot/self-test scratch (per-build; signatures may override).
        .verdict      = 0x0011FF15u,
        .sim_gate     = 0x0011FD1Bu,
        .dsp_uploaded = 0x0011038Cu,
        // DSP shared-RAM mailbox / MDIRCV queue (HPI layout).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Shell-side helpers.
        .get_string    = 0x002BBFACu,   // v6.39
        .w_get_string  = 0x002BBCB8u,   // v6.39
        .faid_cksum    = 0x00111B8Au,   // FuBu v6.39
        .faid_cksum_val = 0x03B7u,      // FuBu v6.39
        // Reset/reboot path — sig-first; fallbacks pinned to 3310 v5.79.
        // See docs/watchdog-reset-3310.md §1 and the REBOOT_PAT signature (mad2_sigs.c).
        .reboot_fn     = 0x002EEBAEu,
        .reboot_reason = 0x0011FF94u,
        .reboot_save   = 0x0011FF88u,
        .fatal_handler = 0x002F0BA4u,
        .assert_log    = 0x002EDD4Au,
        .reason_setter = 0x002E0ED8u,
        .malloc_fail   = 0x00299B26u,   // blocking-allocator NULL-return / block-retry PC
        // Task-14 cancel state (READ-ONLY by the DSP keep-alive; sig-first via STATE_FANOUT).
        .task14_state  = 0x0010AE61u,   // task-14 state byte ([0x10AE58]+9)
        .task14_status = 0x0010B1FCu,   // per-state status table base
    },
    .sigs = MAD2_SIGS,            // shared MAD2 RTOS signatures (src/models/mad2_sigs.c)
    .n_sigs = MAD2_N_SIGS,
    .sigs2 = MAD2_SIGS_3310,      // NHM-5 verdict + dsp_uploaded self-heal (per-build RAM cells)
    .n_sigs2 = MAD2_N_SIGS_3310,
    .boot = {
        .skip_seclock_default = 1,   // web default ("FAID Pass" on)
        .pin_verdict_default  = 0,   // organic verdict via the DSP responder
    },
    .ident = {
        .match = "NHM-5",            // 3310 product code (present in the dump)
        // Pin to 2 MB: the 3310 is always a 2 MB dump, and the string alone already
        // disambiguates from the 2 MB 8850. The size guard stops a 4 MB image that also
        // carries the NHM-5 code (cooked 3330-class builds, e.g. the "10.01n" ToolKit
        // image) from matching the 2 MB 3310 here — those fall through to the 3330.
        .flash_size = 0x00200000u,   // 2 MB
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour
};
