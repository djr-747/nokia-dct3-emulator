// Nokia 8850 model profile — STUB. Proves the multi-model abstraction loads a
// second 2 MB candybar-class DCT3. Boot is not expected yet: the firmware-build
// scratch addresses (verdict, dsp_uploaded, etc.) are unknown and must be located
// by signature/RE before 8850 can boot. See docs/multi-model-architecture.md.
//
// Firmware on hand: firmware/FuBu8850 v5.31 (PPM C).fls (product code NSM-2).

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h" // NSM Family-A keypad matrix (RE'd here; now shared
                                    // with 5210/8210/8250/8855 — single source of truth)

const ModelProfile model_8850 = {
    .name = "8850",
    .description = "Nokia 8850 (NSM-2, MAD2, PCD8544 84x48) — STUB",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0x003D0000u,   // 2 MB layout — assumed same as 3310 (verify)
        .eeprom_size = 0x00030000u,
    },
    .lcd = {
        // 8850 drives an 84x48 PCD8544 like the 3310 (per Dan) — our existing LCD
        // decode applies directly.
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        .power_special_cols = 0x02,   // boot-hold (works); RE says faithful PWR special idx=4
        .family  = KP_FAMILY_8210,    // Family A: soft keys, up/down, send/end, volume
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,                 // NSM-2 IRQ0 dispatcher decodes the matrix source bit
                                      // (I/O 0x2B bit4); without it keypresses never scan.
        .has_slide = 1,               // NSM-2 is a slide phone: reed-switch cover sensor on
                                      // I/O 0x28 bit0 / interrupt source 0x29 bit0 (mad2 models).
    },
    .battery = {
        // 8850 battery-type detection (fn 0x2FF716, mode 10): reads CCONT A/D ch3
        // (BSI) + ch4 (temp) and only returns a VALID type (non-zero -> boot; 0 ->
        // the boot halts at 0x309DA2) when BSI is in [318,370] (0x13E-0x172) AND temp
        // is in [307,348] (0x133-0x15C). The 3310's NiMH defaults (bsi 0x026, temp
        // 0x200) fall OUTSIDE both windows -> battery type 0 -> halt. These in-window
        // values give battery type 5 and let the 8850 boot + draw. (Li-ion BLB-2.)
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 16,   // 8850 noted as needing the 16-source IRQ model
    },
    .fw = {
        // Reset/recovery (reboot_fn/reboot_reason/reason_setter) resolve via the shared
        // sigs — the bl-less REBOOT8 prologue variant + reason_setter SIG_CODE_VLIT in
        // mad2_sigs.c. No hardcoded constants here (SIGDUMP confirms resolution).
        // DSP shared-RAM mailbox / MDIRCV queue: the MAD2 HPI layout is assumed
        // shared across MAD2 DCT3 (verify against the 8850 image).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build scratch + helpers (RE'd from the 8850 v5.31 image).
        // dsp_uploaded = 0x135664 (8850-specific; analog of 3310 0x11038C / 3410
        // 0x12BA10): the DSP-code-block-done flag the IRQ4 ISR 0x2CB418 sets to 1
        // (at 0x2CB44E, when reply reg [0x100E4]==0) once the upload completes. The
        // shared mad2 DSP pump raises the block-ack IRQ4 only while [dsp_uploaded]==0,
        // so the gate fires a handful of times then stops — faithfully, replacing the
        // old IRQ4_PERIOD=20000 hack in (deleted) src/models/8850/dsp.c. Verified
        // RAMWATCH: starts 0, set 0->1 @PC 0x2CB44E ~step 2.2M (real boot step, not an
        // early-RAM-init clobber).
        //
        // verdict = 0x13FDE1 (8850-specific; analog of 3310 0x11FF15 / 3410 0x17FE3D):
        // the DSP self-test verdict / MMI-ready byte = MMI_READY struct base 0x13FD78 +
        // 0x69 (same 0x69 offset as 3310/3410). bit2(0x04)="DSP self-test pending" set
        // by writer 0x240B60 right after streaming the request to the DSP (bl 0x284E28);
        // bit7(0x80)="MMI ready" read by CONTACT_SERVICE gates 0x285616/0x242FE6. The
        // shared mad2 DSP responder posts the group-0x74 reply + FIQ0 only while
        // [verdict]&0x04, so without this it read m->mem[0], never replied -> bit2 stayed
        // pending (C4) until the readiness task gave up @9.42M -> CONTACT SERVICE. RAMWATCH:
        // 00->40->C0->C4 @~2.57M (real boot step). sim_gate still 0 = unresolved.
        .verdict = 0x0013FDE1u, .sim_gate = 0, .dsp_uploaded = 0x00135664u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        // DSP boot handshake: the startup launcher's DSP-readiness step (fn 0x2CAD60)
        // posts a command to the HPI mailbox 0x100F6, pulses the DSP (0x20002 bit0),
        // parks 0x10004 at 0xFFFF and polls it for the DSP's ready/version reply. A
        // reply of 5 or 6 (fn 0x2CAE26) makes the firmware upload the 115 DSP code
        // blocks (handshaked via dsp_mbox0/1) and keeps the DSP-ready flag 0x135665
        // set. Without it the poll times out -> minimal task startup -> CONTACT SERVICE.
        // (Flag 0x135664 still needs IRQ4; see docs/8850.md.)
        .dsp_boot_status = 0x00010004u, .dsp_boot_ready = 6,
    },
    .sigs = MAD2_SIGS,            // shared MAD2 RTOS sigs locate reboot_fn/fatal_handler/
    .n_sigs = MAD2_N_SIGS,        // reboot_reason etc. at runtime (enables RESET_EARLY/recovery)
    .boot = {
        .skip_seclock_default = 0,
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NSM-2",
        .flash_size = 0,
    },
    // DSP behaviour: the shared faithful mad2 default — same as 3310 and 3410. The
    // block-ack IRQ4 (flag 0x135664) and the self-test FIQ0 reply are driven by the
    // resolved per-build addresses above (dsp_uploaded / verdict), NOT a custom hack.
    // (Old src/models/8850/dsp.c IRQ4_PERIOD=20000 injector deleted.)
    .dsp = &mad2_dsp_default,
};
