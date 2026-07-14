// Nokia 3410 model profile — SCAFFOLD. Proves the multi-model abstraction loads a
// 4 MB DCT3 and pins the correct memory map, but the 3410 will NOT boot or draw yet:
//
//   1. The .fw fallbacks are all 0 (the 3310-v5.79 constants are WRONG for v5.46), but
//      the shared MAD2 RTOS signatures (src/models/mad2_sigs.c) locate them at runtime
//      over the 3410's own flash region. 7 of 10 resolve: sim_gate, reboot_fn,
//      reboot_reason, reboot_save, fatal_handler, assert_log, reason_setter. malloc_fail
//      and task14_state/status still miss (restructured in v5.46 — need RE; both are
//      post-boot, so non-blocking). mad2 degrades gracefully on the remaining 0s.
//
//   2. The LCD controller is the SAME PCD8544-compatible family as the 3310 —
//      same command set, so mad2.c's existing LCD decode path applies directly. Only the
//      geometry differs: 102x72 controller RAM, 96x65 visible. Two small mad2.c geometry
//      tweaks are still needed for FULL fidelity (deferred — scaffold only):
//        (a) set-Y bank mask `b & 0x07` (mad2.c:945) reaches only banks 0-7; the 3410's
//            9-bank/72-row RAM needs `b & 0x0F` to address bank 8 (visible row 64).
//            Without it the bottom visible row is unreachable (minor 1-row artifact).
//        (b) the fb stride + render read use lcd.width (visible 96); to hold the full
//            102-col controller RAM faithfully, stride by lcd.ctrl_width then crop to 96.
//      With controller = LCD_PCD8544 and the visible geometry below, banks 0-7 (rows
//      0-63) already render with today's decoder.
//
// Firmware on hand: assemble the WinTesla split files into a flat 4 MB .fls first:
//   tools/fls_assemble.py -o 3410_v5.46.fls NHM2NX05.460 NHM2NX05.46E "3410 virgin eeprom.pmm"
// Product code NHM-2, MCU header "V 05.46 / 23-12-04 / NHM-2".
//
// Memory map:, 3410 "04.26 <" row (v5.46 qualifies):
//   MCU    0x200000-0x4CFFFF   PPM 0x4D0000-0x56FFFF   EEPROM 0x570000-0x5FFFFF

#include "models/model.h"
#include "models/mad2_sigs.h"   // shared MAD2 RTOS firmware-address signatures

// 3410 keypad matrix (Family C: 2 soft keys, up/down, send/end, NO volume).
// RE'd from keymap table 0x4C5130 (lookup fn 0x3E496E, index = row*5 + col). The
// digit/*/# block is byte-identical to the 3310; the nav/soft region is relocated and
// adds green/red (codes 0x0E/0x0F) — REFUTES MADos's "3410 == 3310 matrix" claim.
// SEND/END green-vs-red orientation inferred (medium); cells are certain. PWR special
// bit1 (0x02), same mechanism as the 3310.
static const KeyLine keylines_3410[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,4,1,0},
    {KK_4,2,4,0}, {KK_5,2,3,0}, {KK_6,2,2,0},
    {KK_7,3,4,0}, {KK_8,3,3,0}, {KK_9,3,2,0},
    {KK_STAR,4,4,0}, {KK_0,0,2,0}, {KK_HASH,4,2,0},
    {KK_UP,4,3,0}, {KK_DOWN,0,3,0},
    {KK_SOFT1,0,4,0}, {KK_SOFT2,0,1,0},   // KEY_SOFT_A(0x19) / 0x1A
    {KK_SEND,1,4,0}, {KK_END,1,1,0},      // green / red — swapped (RE guess was inverted)
    {KK_PWR,0,0,0x02},                     // special-scan bit1 (faithful)
};

const ModelProfile model_3410 = {
    .name = "3410",
    .description = "Nokia 3410 (NHM-2, MAD2, 96x65/102x72 LCD) — SCAFFOLD (no boot/LCD yet)",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // 4 MB layout pushes the EEPROM/NVRAM partition late (vs 0x3D0000 on the 2 MB
        // 3310). 3410 v5.46 (>=04.26): EEPROM 0x570000-0x5FFFFF (MCU+PPM below).
        .eeprom_base = 0x00570000u,
        .eeprom_size = 0x00090000u,   // 0x570000..0x600000
        // param_base/param_block_size left 0: the generic MAD2 top-boot default
        // (top 64 KB = 8x 8 KB parameter blocks) applies — here 0x5F0000..0x600000,
        // where the FFS rotates its active "EEPROM" record blocks (in-image f0f0
        // headers at 0x5FA000/0x5FC000/0x5FE000). See mad2_flash.c flash_erase_blk.
    },
    .lcd = {
        // 3410 graphical display: 96x65 visible, 102x72 controller RAM. Same PCD8544-
        // compatible controller as the 3310 — reuse the existing decode path;
        // see the geometry caveats (a)/(b) in the header comment. io_data/io_cmd are the
        // 3310 GENSIO offsets (shared MAD2 bus — verify the 3410 wiring).
        .controller = LCD_PCD8544,
        .width = 96, .height = 65, .banks = 9,      // visible
        .io_data = 0x2E, .io_cmd = 0x6E,
        .ctrl_width = 102, .ctrl_height = 72, .ctrl_banks = 9,  // controller RAM
    },
    .keypad = {
        .power_special_cols = 0x02,   // RE-confirmed: PWR special bit1 (table 0x4C50CC)
        .family  = KP_FAMILY_3410,    // Family C: soft keys, up/down, send/end, no volume
        .lines   = keylines_3410,
        .n_lines = (int)(sizeof(keylines_3410) / sizeof(keylines_3410[0])),
    },
    .battery = {
        // PLACEHOLDER — 3310 NiMH wedge-clean defaults. The 3410 battery-type detection
        // window is unverified; revisit once it boots far enough to read CCONT A/D.
        .vbatt = 0x220, .bsi = 0x100, .temp = 0x200, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,           // MAD2 default
        .asic_version = 0,          // unknown for 3410 -> read falls through to RAM
        // The 3410 releases the DSP via [0x20002] bit2 (RMW @0x3CBAC8) then spins polling
        // bit4 for DSP-ready. Synthesise the same 0x53 status the 3310 reads (bit4 set),
        // gated on the 3410's bit2 release. Without this it hangs at 0x3CBADC pre-LCD.
        .dsp_reset_running = 0x53,
        .dsp_release_mask  = 0x04,  // 3410 release bit = bit2 (3310 = bit0)
    },
    .fw = {
        // DSP shared-RAM mailbox / MDIRCV queue: the MAD2 HPI layout is assumed shared
        // across MAD2 DCT3 (verify against the 3410 image before relying on it).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        .mdisnd_q     = 0x00010000u,    // MCU->DSP request queue — layout identical to the
        .mdisnd_tail  = 0x000100A4u,    // 3310's (RAMWATCH-pinned 2026-07-15; enqueue 0x348200)
        // Per-build scratch + reset/heap/task-14 addresses UNKNOWN for the v5.46 build —
        // 3310 values would mis-fire. 0 = unresolved (mad2 degrades gracefully). Locate
        // by re-running the shared MAD2 RTOS signatures over 0x200000..0x4D0000.
        // dsp_uploaded = 0x12BA10 (3410-specific; NOT the 3310's 0x11038C, which the 3410
        // clobbers to 0x6F early → gate never fires). The mad2 DSP pump raises the boot
        // block-ack IRQ4 only while [dsp_uploaded]==0; the 3410 DSP message-pump (0x348A20,
        // writer 0x348A4A) sets [0x12BA10]=1 once the upload completes, and the subsystem-
        // starter 0x3B3B9E bails (→0x3B3C16) while it's !=1 (then mov r0,#22 = msg 0x16
        // advance). Located by RE of the pump/starter analogs; [0x12BA10] is 0 through the
        // stall and not clobbered. This is THE fix for the 3410 CONTACT-SERVICE stall — the
        // DSP-block-ack IRQ4 now fires (gated) → DSP pump runs → 0x4C/0x4D handshake → boot
        // advances. (3410 bring-up §7.13/§7.14.)
        // verdict = 0x17FE3D (= MMI_READY_FLAG byte, base 0x17FDD0+0x6D): bit7=ready,
        // bit2="DSP self-test pending" (firmware sets it @1.886M via 0x25CFDA, holds blocked
        // on the DSP reply). The mad2 DSP self-test responder fires (posts group-0x74 + FIQ0)
        // only while [verdict]&0x04; with verdict=0 it read m->mem[0] and never replied, so
        // bit2 stayed set, readiness never completed, and READINESS_REEVAL re-demoted at ~9.5M
        // = the SECOND CONTACT SERVICE. 3310 uses 0x11FF15 (clobbered to 0xB8 on the 3410).
        // (3410 bring-up §7.15.)
        // sim_gate = 0x17FBD0: the 3410 disp49 SIM-gate selector (gate fn 0x300264, struct base
        // CALL_STATUS 0x17FB96 + 0x3A). NB the 3410 reads it as a HALFWORD (3310 reads a BYTE at
        // 0x11FD1B = base+0x1D), so the no-SIM bypass value 0x06 lands in the LOW byte 0x17FBD1
        // (POKE 0x17FBD1=0x06). The 3310's 0x11FD1B is in the early-clobbered 0x11xxxx range;
        // 0x17FBD0 is in the live MMI-state region (like verdict 0x17FE3D), 0 in no-SIM. Caveat:
        // boot_trace's TRACE=sim `sim_gate-0x1D` math + its hardcoded-0x11FD1B bypass are
        // 3310-specific (offset is 0x3A here, halfword) — treat as 3310-only. (§7.16.)
        .verdict = 0x0017FE3Du, .sim_gate = 0x0017FBD0u, .dsp_uploaded = 0x0012BA10u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0, .dsp_boot_ready = 0,
        .reboot_fn = 0, .reboot_reason = 0, .reboot_save = 0,
        .fatal_handler = 0, .assert_log = 0, .reason_setter = 0, .malloc_fail = 0,
        .task14_state = 0, .task14_status = 0,
    },
    // Shared MAD2 RTOS signatures locate the per-build addresses over the 3410's own
    // flash region (sig hits OVERRIDE the 0 fallbacks in .fw above). On v5.46: 7 of 10
    // resolve; malloc_fail + task14_state/status miss (restructured — need RE). See the
    // header comment and mad2_sigs.h.
    .sigs = MAD2_SIGS,
    .n_sigs = MAD2_N_SIGS,
    .boot = {
        .skip_seclock_default = 1,   // mirror 3310 web default
    },
    .ident = {
        .match = "NHM-2",            // 3410 product code (in the PPM header of the dump)
        .flash_size = 0x00400000u,   // 4 MB — with the NHM-2 string, unambiguous
    },
    .dsp = &mad2_dsp_default,        // shared/legacy DSP behaviour (verify for 3410)
};
