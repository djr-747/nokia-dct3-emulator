// Nokia 6210 model profile — NPE-3, a 4 MB MAD2 DCT3 (2000). NOT an NSM/Family-A phone: the 6210
// is its own design that STRADDLES the 5110 and the 3310 (Dan, 2026-06-20). From the 3310 side it
// is ROM-6 DSP (in-flash generation, mad2_dsp_default — confirmed by DSP-block F073-target finger-
// print) with an in-flash EEPROM partition (NOT the 5110/6110 external serial EEPROM). From the
// 5110 side it shares the SAME battery A/D window {0x2C0,0x150,0x140,0x000}. The 96x65 graphic
// display is the larger screen (3410/7110 class). MAD2_SIGS resolves the per-build v5.56 addresses
// over the 6210 flash. (The 4 MB memory-map skeleton was seeded from the 5210 bring-up template,
// but the 6210 is a distinct phone — do not treat shared tables below as an NSM-family designation.)
//
// STATE: first bring-up. verdict 0x17FD99 located by gen_sig of the 5110 MMI-ready gate
// (0x231150 → 6210 0x305010, 90% fuzzy; gate 0x30501A `ldr =0x17FD99`). The 5210/6210 self-test
// gates verdict bit6 on an EEPROM-RECORD comparison (not the HPI mailbox), so the in-flash EEPROM
// partition (eeprom_base) + the self-test record reads matter — expect a 5210-style CONTACT SERVICE
// pass needing the right partition + dsp_uploaded. eeprom_base is an ESTIMATE pending a dump map.

#include "models/model.h"
#include "models/mad2_sigs.h"
#include "models/keylines_nsm_a.h"

const ModelProfile model_6210 = {
    .name = "6210",
    .description = "Nokia 6210 (NPE-3, MAD2, 4 MB, 96x65) — in-flash EEPROM",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // In-flash EEPROM/NVRAM partition near the top of the 4 MB flash. ESTIMATE (5210 = 0x580000);
        // the 6210 PPM is large (dense to ~0x590000) so this likely sits higher — verify vs a dump map.
        .eeprom_base = 0x00580000u,
        .eeprom_size = 0x00080000u,
    },
    .lcd = {
        // 96x65 graphic display (same larger screen class as the 3410/7110, per Dan). PCD8544-
        // compatible decode (as the 3410) over the MAD2 GENSIO offsets. Verify controller/wiring.
        .controller = LCD_PCD8544,
        .width = 96, .height = 65, .banks = 9,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        // Keypad (row,col) decode is firmware-VERIFIED v5.56 (2026-06-20), NOT an NSM-family
        // assumption: the 6210 KEYMAP_NORM table @0x2869B8 (decode fn 0x4FACFA, raw=row*5+col) is
        // byte-identical in content to the shared MAD2 matrix keymap, so the 6210 reuses that
        // (row,col) table — this is shared MAD2 keypad silicon decode, not a product-family tag.
        // Scan = DIR_AWARE: RAW_READ 0x4F84C8 writes DIR 0xE0 to I/O 0xA8, reads col 0x2A, isolates
        // the active row. uif_irq=1: the ISR saves/acks IM_C (I/O 0x6B) — the source-decode hallmark.
        // The Navi-roller is a separate rotary encoder (not in the matrix) — KK_WHEEL_* unmapped.
        // power_special_cols: the firmware SPEC table @0x2869D4 decodes POWER (0x0D) at special-scan
        // bit4 (raw 0x84 → mask 0x10). Kept at 0x02 (the 5110/3310 neighbour value) because cold
        // power-on is driven by the reason chain, NOT this special-scan, so 0x02 is not load-bearing;
        // 0x10 is the strictly-faithful mask if the held-power special-scan path is ever exercised.
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_8210,    // GUI visual layout only (2 soft keys + send/end + vol present)
        .lines   = keylines_nsm_a,    // shared MAD2 (row,col) table — content == 6210 KEYMAP_NORM
        .n_lines = (int)(sizeof(keylines_nsm_a) / sizeof(keylines_nsm_a[0])),
        .scan = KP_SCAN_DIR_AWARE,
        .uif_irq = 1,
    },
    .led = { .lcd_rgb = 0x55C955, .kbd_rgb = 0x55C955 },
    .battery = {
        // CCONT A/D — STANDARD MAD2 wiring (VERIFIED v5.56, 2026-06-20): the per-quantity
        // channel-map table at flash 0x2869DC = {00 04 05 06 07 03 01 02} is byte-identical to
        // the 3310's (0x32E73C) → VBATT=ch2, BSI=ch3, BTEMP=ch4, VCHAR=ch5; m->adc[] indexed by
        // that convention. NOT the 3210 oddball, so adc_route stays all-zero (identity). A/D
        // dispatcher 0x4F3962, channel reader 0x504100 (builds reg-0 select (ch<<4)|8). Values
        // sit cleanly inside every v5.56 accept window (60M clean boot, no batt/charge/temp assert):
        //   VBATT 0x2C0 — scaler 0x4B0094; scaled-value validity window [0x708..0x157C] @0x3D72A8
        //   BSI   0x150 — battery-type window [0x13E..0x172] @0x4DD296/0x4DD2A4 (classifier 0x4DD24A)
        //   BTEMP 0x140 — accept (<50 || [0x134..0x15D)) @0x4DD2AE/0x4DD2BC/0x4DD2CA; 0x140 in range
        //   VCHAR 0x000 — charger-present gate 0x4F26F2 (cmp #100; bgt) → 0<100 = no charger
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,
        // I/O 0x20002 (DSP reset ctrl): v5.56 boot releases the DSP at 0x4DC0E8 by writing
        // 0x40 then 0x44 (bit6, then bit2) and then polls bit4 for DSP-ready (lsr #5; bcc).
        // Same MAD2 ASIC family as the 3310 -> real-HW status read-back 0x53 (bit4 set). The
        // release bit here is bit2 (0x04 — the last bit set before the poll), NOT the 3310's
        // bit0. Without this the boot spins forever reading 0x44 at 0x4DC0FA. asic_version
        // left 0 (RAM fallthrough) pending a real-HW 0x20000 capture.
        .dsp_reset_running = 0x53,
        .dsp_release_mask  = 0x04,
    },
    // Power-on reason: ccont_poweron_int stays UNSET (0). The 6210 is 5210-class (keypad-driven
    // power-on reason), NOT a 3210-style CCONT-PWRONX oddball. VERIFIED v5.56 (2026-06-20): over a
    // full boot, CCONT reg 0x0E is read exactly once (-> 0x00) at 0x4EC7C4, and only as part of the
    // generic register-snapshot loop (stores into [0x1742EC+reg]); the result [0x1742FA] is never
    // read back, so there is no PWRONX power-off classifier (contrast 3210 0x2AF0AE). Boots cleanly
    // with reg0E=0 — no power-straight-off risk.
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
        // verdict 0x17FD99 — gen_sig of the 5110 MMI-ready gate → 6210 0x30501A `ldr =0x17FD99`
        // (90% fuzzy). dsp_uploaded UNKNOWN for v5.56 (5210 analog 0x13B528) — 0 until RE'd; the
        // self-test is EEPROM-record-based (see header), so expect to RE the record gate.
        .verdict = 0x0017FD99u, .sim_gate = 0, .dsp_uploaded = 0,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        // Reset/reboot path — all six resolve via the shared MAD2_SIGS for v5.56 (VERIFIED
        // 2026-06-20: sig-resolved AND disasm-confirmed). Pinned here as documented fallbacks
        // (matching the 3310 profile) so the values are explicit if a future build shifts a sig.
        //   reboot_fn 0x50096A   — universal reboot entry; at 0x500996 strb reason -> [0x17FE48],
        //                          then [0x20001]|=4 (MCU reset edge); b .
        //   fatal_handler 0x5076D4 (ARM) — ldr =0x17FE3C; stmia {LR,SPSR,CPSR} (the 3-word save)
        //   reason_setter 0x4B0424 — canonical 3-insn staged-reason setter (strb r0,[=0x17FE48])
        .reboot_fn = 0x0050096Au, .reboot_reason = 0x0017FE48u, .reboot_save = 0x0017FE3Cu,
        .fatal_handler = 0x005076D4u, .assert_log = 0x004FB12Eu, .reason_setter = 0x004B0424u,
        .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .boot = {
        .skip_seclock_default = 1,
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NPE-3",
        .flash_size = 0x00400000u,
    },
    .dsp = &mad2_dsp_default,         // ROM-6-class (in-flash gen) shared DSP responder
};
