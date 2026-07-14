// Nokia 8290 model profile — PARTIAL (bring-up triage 2026-06-06). NSB-7, 2 MB MAD2
// candybar — the US-band (1900 MHz) sibling of the 8210 (NSM-3); same NSM Family-A class
// (shared keypad matrix + BLB-2 battery + 84x48 PCD8544). Autodetects, boots clean into
// the MMI, rests at CONTACT SERVICE. verdict + dsp_uploaded RE'd and VERIFIED below.
// REMAINING BARRIER: the DSP-code upload handshake does NOT complete under mad2 (DSP acks
// stay ~4, dsp_uploaded never reaches 1) so the verdict gate skips the self-test and clears
// bit6. Unlike the 8210/8850 the launcher parks [0x10004] but polls a RAM ready slot
// [0x12DF08]==6 (a different mechanism); neither dsp_boot_status=0x10004 nor 0x12DF08 drives
// it. The 8290's DSP-boot upload sequence needs dedicated RE. Firmware: Nokia 8290 NSB-7
// v5.22 A.fls. Without an ident match it would fall through to the 3310 default.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (shared with the 8210/8850)

const ModelProfile model_8290 = {
    .name = "8290",
    .description = "Nokia 8290 (NSB-7, MAD2, PCD8544 84x48) — SCAFFOLD",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0x003D0000u,   // 2 MB layout (same as 8210/8850 — verify per build)
        .eeprom_size = 0x00030000u,
    },
    .lcd = {
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,
    },
    .battery = {
        // Shared NSM Li-ion (BLB-2) battery-type window (== 8210/8250/8850/5210).
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
        .mdisnd_q     = 0x00010000u,    // MCU->DSP request queue (HPI layout-invariant; see 3310)
        .mdisnd_tail  = 0x000100A4u,    // layout default — UNVERIFIED (boot does not reach DSP init, 2026-07-15 sweep)
        // Per-build DSP scratch — RE'd from 8290 v5.22 (ported from 8850 v5.31 sigs).
        // verdict = 0x13FDE1 (MMI_READY base 0x13FD78 + 0x69; the base is stable across the
        // family). Bit2-writer 0x2488F6 (prologue r5=ldr=0x13FD78, r6=#0x69, write [r6,r5]).
        // dsp_uploaded = 0x12DF00: DSP-block-done flag the IRQ4 ISR 0x2CB990 (exact sig match
        // of 8850 0x2CB418) sets to 1 (at 0x2CB9C8, when reply reg [0x100E4]==0).
        .verdict = 0x0013FDE1u, .sim_gate = 0, .dsp_uploaded = 0x0012DF00u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,  // DSP-boot upload handshake differs from 8210/8850 — RE pending
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
        .match = "NSB-7",            // 8290 product code (0x1FC header) — unique to the 8290.
        .flash_size = 0,
    },
    .dsp = &mad2_dsp_default,
};
