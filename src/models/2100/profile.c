// Nokia 2100 model profile — SCAFFOLD (registered 2026-06-17).
//
// "essentially the 5210, but with the LCD of the 3410." So this profile
// reuses the 5210's NSM-class personality — NSM Family-A keypad (== 8850/5210),
// BLB-2 Li-ion battery window, orange backlight, shared MAD2 DSP responder — but
// swaps in the 3410's larger graphical panel (96x65 visible / 102x72 controller
// RAM) and a 2 MB memory map (the 5210 is 4 MB; the 2100 is a 2 MB part).
//
// Firmware on hand: firmware/Nokia 2100 NAM-2 v5.84.fls (flat 2 MB .fls, assembled
// from WinTesla NAM205.840 (MCU) + NAM205.84E (PPM) via tools/fls_assemble.py).
// Header @0x1FC: "V 05.84 / 19-05-04 / NAM-2". Product code NAM-2, 2 MB image.
//
// Memory map:, 2100 row (2 MB):
//   MCU 0x200000-0x35FFFF   PPM 0x360000-0x3EFFFF   EEPROM 0x3F0000-0x3FFFFF
// (Note the EEPROM is only the top 64 KB — the 2100 carries a small PPM and no
//  dedicated EEPROM partition like the larger NSM parts; the top boot blocks hold
//  the parameter records.)
//
// Like the 5210/3410 scaffolds, the per-build .fw RAM/code addresses are all 0 (the
// 5210-v5.40 constants are WRONG for this NAM-2 v5.84 build, and live in a 4 MB
// address space besides). The shared MAD2 RTOS signatures (src/models/mad2_sigs.c)
// locate what they can at runtime over the 2100's own flash region; the boot-critical
// per-build addresses (dsp_uploaded / verdict / sim_gate) must be RE'd per-build
// before this gets past the S5-class barriers — same method as the 5210 bring-up.

#include "models/model.h"
#include "models/mad2_sigs.h"       // shared MAD2 RTOS firmware-address signatures
#include "models/keylines_nsm_a.h"  // shared NSM Family-A keypad matrix (== 5210/8850)

const ModelProfile model_2100 = {
    .name = "2100",
    .description = "Nokia 2100 (NAM-2, MAD2, 2 MB) — SCAFFOLD (5210 personality, 3410 LCD)",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // 2100 (2 MB): EEPROM is the top 64 KB (0x3F0000-0x3FFFFF). MCU+PPM fill the
        // region below (0x200000-0x3EFFFF). param_base/param_block_size left 0: the
        // generic MAD2 top-boot default (top 64 KB = 8x 8 KB parameter blocks) applies.
        .eeprom_base = 0x003F0000u,
        .eeprom_size = 0x00010000u,   // 0x3F0000..0x400000
    },
    .lcd = {
        // 3410 graphical display: 96x65 visible, 102x72 controller RAM. PCD8544-
        // compatible controller (per the 3410 profile). io_data/io_cmd = 3310 GENSIO
        // offsets (shared MAD2 bus).
        .controller = LCD_PCD8544,
        .width = 96, .height = 65, .banks = 9,                 // visible
        .io_data = 0x2E, .io_cmd = 0x6E,
        .ctrl_width = 102, .ctrl_height = 72, .ctrl_banks = 9, // controller RAM
        .x_mirror = 1,   // 2100 panel has REVERSED segment order, like the 5210 — text renders backwards without this.
    },
    .keypad = {
        // NSM Family-A keypad, SHARED with the 5210/8850. Candybar -> has_slide stays 0.
        .power_special_cols = 0x02,   // PWR special-scan bit1
        .family  = KP_FAMILY_8210,    // Family A: soft keys, up/down, send/end, volume
        .lines   = keylines_nsm_a,
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,                 // matrix IRQ-source decode (8850-class)
    },
    // Period-correct orange backlight (shared with the 5210).
    .led = { .lcd_rgb = 0xFFA040, .kbd_rgb = 0xFFA040 },
    .battery = {
        // NSM Li-ion (BLB-2) battery-type window — SHARED across the NSM family
        // (5210 / 8210 / 8250 / 8850). Mid-window values give a valid battery type so
        // the pre-MMI boot verdict doesn't read a clean power-off and tear down. See the
        // 5210 profile for the full RE narrative. (2100 battery type unconfirmed; reuse
        // the family window as the scaffold default.)
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,           // MAD2 default
        .asic_version = 0,          // unknown -> read falls through to RAM
        // DSP-release quirk UNKNOWN for the 2100. Start at the MAD2 defaults (3310 bit0
        // release, 0x01). If the boot hangs in a tight pre-LCD spin polling [0x20002],
        // RE the release RMW per the 3410/5210 bring-up notes.
        .dsp_reset_running = 0,
        .dsp_release_mask  = 0,     // 0 -> mad2 default 0x01 (bit0)
    },
    .fw = {
        // DSP shared-RAM mailbox / MDIRCV queue: the MAD2 HPI layout is assumed shared
        // across MAD2 DCT3 (verify against the 2100 image before relying on it).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build scratch + reset/heap/task-14 + verdict/dsp_uploaded/sim_gate addresses
        // are UNKNOWN for this NAM-2 v5.84 build. 0 = unresolved (mad2 degrades gracefully;
        // the shared MAD2 RTOS signatures below locate what they can). The 5210's resolved
        // values (verdict=0x17FDE5, dsp_uploaded=0x13B528) are v5.40/4MB-specific and do
        // NOT transfer — they must be RE'd per-build before this clears the S5 barriers.
        //
        // verdict = 0x13FDB3 = MMI-ready struct base 0x13FD46 + 0x6D (RE 2026-06-17).
        // The 5210/3310/8850 use +0x69; this v5.84 build shifted the field to +0x6D.
        // Verdict fn = 0x256720 (NOT sig-located — gen_sig fuzzy from the 5210 fn 0x25D24C
        // failed <65%; located via match_skel.py skeleton match, 68% MED, big gap to next).
        // Confirmed two ways: (a) structural — 0x256720 has ldr=0x13FD46 + offset 0x6D and
        // the identical verdict bit choreography (orr#1 / and#0xEF / orr#0x40 / orr#0x80 /
        // and#0xBF) as the 5210; (b) runtime — RAMWATCH 0x13FDB3 transitions 00->40->C0->80
        // exactly at the verdict-fn PCs (0x2567B0/0x2567BC/0x256816).
        // CONTACT SERVICE root cause: verdict fn gates on the DSP self-test RESULT flag
        // [0x10EC50] (read via accessor 0x2F80B0; cmp #1 at 0x256804). ==1 -> set bit2 +
        // read EEPROM rec 0x6901; !=1 -> `and #0xBF` clears bit6 at 0x256816 -> CONTACT
        // SERVICE. [0x10EC50] is a DSP self-test result that our HLE doesn't yet set (runner
        // 0x2D0730 kicks the self-test via the 0x10000 mailbox). Self-test responder WIP.
        // dsp_uploaded == the self-test RESULT gate byte [0x10EC50] (RE 2026-06-17).
        // On this NAM-2 v5.84 build the DSP-ready ISR and the self-test result are the
        // SAME byte: handler 0x2D0C54 (structural twin of the 5210 DSP_READY_HANDLER
        // 0x345DFC) stores 1 to [0x10EC50] at 0x2D0C82 iff [0x10EC50]==0 && [0x100E4]==0;
        // the verdict fn 0x256720 reads the same byte via accessor 0x2F80B0 (cmp #1 at
        // 0x256804). Wiring it here un-gates mad2's IRQ4 block-ack pump (raised while
        // [dsp_uploaded]==0) so the 0x2D0C54 ISR runs, consumes the DSP self-test reply,
        // and sets [0x10EC50]=1 -> verdict keeps bit6 (no CONTACT SERVICE). The runner
        // 0x2D0724 kicks the exchange: streams the request to HPI mailbox 0x100F6, writes
        // cb cmd [0x100E4]=2 (drives dsp_cb_delay), arms [0x10EC50]=0. FAITHFUL: IRQ4 is
        // the real DSP "block/result done" signal; the ISR sets the flag organically once
        // (no DSPIRQ force, no poke of the result byte).
        // Security-code oracle (RE 2026-06-17): the boot security-code lock verifies the
        // entered code by encrypt(entered)==stored. encrypt 0x2FE780 (xortable 0x33E7A8),
        // verify 0x2FE992, stored code at [0x10FE78] (derived at ~step 4.75M). The real code
        // IS 12345 (firmware plaintext @0x112108); it only fails to unlock under emulation
        // because the keypad first-keypress char-null bug nulls byte0 of the entry string verify
        // reads -> strlen=0 -> reject. The web "unlock" hook repairs verify's input (r0) to
        // "12345" at the verify call site so the firmware's own accept path runs.
        .seccode_encrypt = 0x002FE780u, .seccode_verify = 0x002FE992u, .seccode_store = 0x0010FE78u,
        .verdict = 0x0013FDB3u, .sim_gate = 0, .dsp_uploaded = 0x0010EC50u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    // Shared MAD2 RTOS signatures locate the per-build addresses over the 2100's own
    // flash region (sig hits OVERRIDE the 0 fallbacks in .fw above). The [sigdump] line
    // at boot reports which resolved (MISS = still needs locating).
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .boot = {
        .skip_seclock_default = 1,   // mirror 3310/3410/5210 web default
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NAM-2",            // 2100 product code (0x1FC version header)
        .flash_size = 0x00200000u,   // 2 MB
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour (verify for 2100)
};
