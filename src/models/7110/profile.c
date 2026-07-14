// Nokia 7110 model profile — STUB. Specifically exercises the 4 MB-class memory
// map (the case the 3330 also hits): a larger flash pushes the EEPROM/NVRAM
// partition LATE in the address space, and the device-owned flash range extends
// past 0x400000. Also a different (96x65 Navi) LCD. Boot is not expected yet —
// firmware-build scratch addresses need RE/signatures. See
// docs/multi-model-architecture.md.
//
// Firmware on hand: flash/7110 v5.01 Converted MCU+PPM D.fls (product code NSE-5;
// dump is 0x390000 = MCU+PPM only, no EEPROM partition).

#include "models/model.h"
#include "models/mad2_sigs.h"   // shared MAD2 RTOS firmware-address signatures

// 7110 keypad matrix — RE'd from the firmware keymap table @0x28DC94 (My 7110 v5.00).
// The scanner KEYPAD_RAW_SCAN 0x47397C builds a 5x5 rawkey = row*5 + col; the keypad
// task 0x4CF5D6 indexes the 25-byte table with it (0x5A = dead cell). Each cell's
// firmware keycode was mapped to a function by cross-referencing the 5110's verified
// keymap (both are ROM-4 sharing the same keycode space): keycode 0x01-0x09 = digits
// 1-9, 0x0A = 0, 0x0B = #, 0x0C = *, 0x19 = left soft (Menu), 0x1A = right soft (C/Back).
// (row,col) below is what the emulator drives; the firmware's own scan+keymap then
// decodes the intended key. Digit wiring is physically scattered — trust the table.
// The three 7110-only keycodes 0x0E/0x0F/0x12 (no 5110 analog) are the send/end/roller
// cluster — assigned provisionally, pending per-key confirmation (the Navi roller's
// rotation is a separate encoder, not a matrix key —).
static const KeyLine keylines_7110[] = {
    {KK_1,2,1,0}, {KK_2,3,1,0}, {KK_3,4,1,0},   // keycodes 0x01/0x02/0x03
    {KK_4,2,3,0}, {KK_5,3,2,0}, {KK_6,1,4,0},   // 0x04/0x05/0x06
    {KK_7,2,4,0}, {KK_8,3,3,0}, {KK_9,3,4,0},   // 0x07/0x08/0x09
    {KK_STAR,0,4,0}, {KK_0,1,2,0}, {KK_HASH,1,3,0},   // 0x0C/0x0A/0x0B
    {KK_SOFT1,0,2,0},   // left soft "Menu" — keycode 0x19
    {KK_SOFT2,0,3,0},   // right soft "C"/Back — keycode 0x1A
    {KK_SEND,0,1,0},          // keycode 0x0E (provisional: green/call)
    {KK_END,1,1,0},           // keycode 0x0F (provisional: red/hang-up)
    {KK_WHEEL_PRESS,2,2,0},   // keycode 0x12 (provisional: Navi roller select)
    {KK_PWR,0,0,0x02},  // special-scan power (boot-hold)
};

const ModelProfile model_7110 = {
    .name = "7110",
    .description = "Nokia 7110 (NSE-5, MAD2, 4 MB, 96x65 Navi) — STUB",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00400000u,   // 4 MB chip (flash spans 0x200000..0x600000)
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        // EEPROM/FFS partition base — determined from the full-dump (FuBu) FFS structure,
        // not estimated: the lowest FFS block header (F0F0FFFx) in My 7110 v5.00 is at
        // 0x590000 (0x580000 below is all-0xFF, the PPM boundary); blocks run 0x590000..
        // 0x5E0000 incl. 'PMM' @0x5C0000/0x5D0000 + 'EEPROM' @0x5FC000. Matches the 3310
        // rule (eeprom_base = lowest FFS block). (This is exactly where the truncated
        // "Converted MCU+PPM" image ended — i.e. it shipped with no EEPROM at all.)
        .eeprom_base = 0x00590000u,
        .eeprom_size = 0x00070000u,
    },
    .lcd = {
        .controller = LCD_NAVI,            // SED1565 on-glass controller (NOT PCD8544)
        .width = 96, .height = 65, .banks = 9,
        .io_data = 0x2E, .io_cmd = 0x6E,
        .x_mirror = 1,                     // panel segments wired reversed (XOR'd with the SED1565 ADC select)
        .col_offset = 18,                  // 96-col glass centered in the SED1565's 132-col DDRAM (fw writes cols 18..113)
    },
    .keypad = {
        // MAD2 PLAIN 5x5 scan (row 0x28 / dir 0xA8 / col 0x2A defaults; verified via
        // KEYPAD_RAW_SCAN 0x47397C). Matrix lines RE'd into keylines_7110 above.
        .power_special_cols = 0x02,
        .family  = KP_FAMILY_7110,   // dedicated: soft keys + send/end + Navi roller (no up/down or vol)
        .lines   = keylines_7110,
        .n_lines = (int)(sizeof(keylines_7110) / sizeof(keylines_7110[0])),
    },
    .battery = {
        // 7110 self-test ADC gate (fn 0x49F5C0, latch [0x16BC7C], firmware/My 7110 v5.00):
        // reads CCONT A/D ch3 (BSI, = adc[3]) and ch4 (BTEMP, = adc[4]) via the bit-banged
        // GENSIO ADC at 0x4EC1E8 (cmd [0x2002C], status [0x2006D], result [0x2006C]). The
        // latch drives verdict [0x17FE15] bit0: the self-test builder (0x30DE18) reads
        // [0x16BC7C] and sets bit0 *iff* it is 0 (0x30DE26). [0x16BC7C] inits to 0 and is
        // forced to 2 (FAIL) at 0x49F61A only when the BSI read fails BOTH the battery
        // window AND the full-scale check. The gate has two healthy outcomes:
        //   (A) ch3 (BSI) in [318,370] (0x13E-0x172) AND ch4 (BTEMP) < 50  -> [0x16BC7C]=0,
        //       posts the normal battery-OK event (0x49D478 -> 0x7B06); OR
        //   (B) ch3 (BSI) >= 0x3FA (full-scale)                           -> the FAIL latch
        //       at 0x49F61A is never written, so [0x16BC7C] stays 0 (path 0x49F620/0x4E8E8A).
        // The 3310 NiMH defaults (bsi 0x026, temp 0x200) hit NEITHER -> ch3=38 fails both
        // checks -> [0x16BC7C]=2 -> bit0 never set -> CONTACT SERVICE.
        // Path (B) is NOT a usable continuation: ch3>=0x3FA routes to 0x4E8E8A, a
        // charger/power-management bit-bang that busy-waits on an unmodelled hardware line
        // (0x4E8ED4 `cmp r5,#1; bne .`) and never reaches the display. So the faithful
        // working-hardware reading is path (A): a real BLB-2 pack presents BSI in the
        // detection window (bsi=0x150, the same in-window value the 5110/6110/8850 family
        // profiles use) AND the 7110 reads its BTEMP line low here (ch4 < 50), which posts
        // the battery-OK event (0x49D478 -> 0x7B06) with [0x16BC7C]=0 -> verdict bit0 set.
        // ADC AUDIT REVISED temp 0x028 -> 0x0140; REVERTED 2026-07-13. The audit's
        // "bit0 is not the standby gate" is REFUTED: the self-test subop DISPATCHER 0x3106E4
        // gates EVERY subop except the 0x64 begin-handshake on verdict bit0 (0x31070E reads
        // [0x17FDAC+0x69] bit0; bit0-clear fallback 0x310B88 accepts only 0x72/0x7C/0xC8) — so
        // with bit0 clear the step engine (subop-0x80) and completion (subop-0xA4) can never
        // run and verdict bit3 never clears = CONTACT SERVICE. The bit0 gate (0x49F5C0) needs
        // raw ch4 < 50 at self-test time; the charge classifier 0x4C83F6 wants ch4 in [307,347]
        // only when charging (no charger modeled -> moot). Boot-time BTEMP reading low is the
        // plausible real behavior (NTC bias off until the charger path enables it); if a charger
        // model lands, ch4 needs to become time/state-dependent rather than a constant.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x0160, .charger = 0x000,
    },
    .asic = {
        .irq_sources = 8,   // TODO: verify (88xx are 16; 7110 unconfirmed)
    },
    .fw = {
        // MAD2 DSP HPI mailbox layout assumed shared (verify against the image).
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        // Per-build scratch + helpers UNKNOWN for 7110 — 0 = unresolved.
        .verdict = 0x0017FE15u,         // self-test/MMI screen gate, bit6 (traced: lifecycle + POKE-confirmed)
        // dsp_uploaded: the 7110's DSP self-test RESULT flag is [0x167030] (sibling pattern: 2100 used
        // [0x10EC50], 3410 [0x12BA10]; builder leaf 0x49F548 returns it, bit6 kept iff ==1). The shared
        // dsp_default IRQ4 block-ack pump (gated on [dsp_uploaded]==0) invokes the MCU's process stage
        // 0x432EE0 once (cb_reply==0) → [0x167030]=1 → builder keeps verdict bit6.
        // HELD AT 0 (default = clean CONTACT SERVICE). Setting it to 0x00167030u DOES set verdict
        // bit6 (web-verified: [0x167030] 00→01 @2.2M, verdict ends 0x49), but bit3 ("self-test not
        // complete") stays set → the verdict is INCONSISTENT and the web MMI parks in the tally
        // recompute loop 0x47304A → BLANK screen, worse than the honest CONTACT SERVICE
        // (RE-CONFIRMED 2026-07-13: nav.mjs renders all-white with dsp_uploaded=0x167030).
        // 2026-07-13 RESOLVED: dsp_uploaded=0x167030 keeps the DSP-ready flag up so the builder
        // leaves the verdict at 0xCD (bit6 kept). The CONTACT-SERVICE gate is then verdict BIT2
        // (the sequencer 0x310D5A does `lsr #3` -> carry = bit2, NOT bit3), which the 7110 DSP
        // responder clears by posting the group-0x74 sub-13 self-test-complete ack (handler 0x30E0EA
        // `and #0xFB`), exactly like the sibling models' cmd-13. See src/models/7110/dsp_7110.c.
        .sim_gate = 0, .dsp_uploaded = 0x00167030u,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
    },
    .sigs = MAD2_SIGS,      // shared MAD2 RTOS signatures — resolves the reboot/fatal
    .n_sigs = MAD2_N_SIGS,  // chain so mad2 EARLY-intercepts firmware self-resets
                            // (without this, a reset is only caught at the [0x20001]
                            // reset-request write, after the fw runs its full reboot path)
    .boot = {
        .skip_seclock_default = 0,
        .pin_verdict_default  = 0,
    },
    .ident = {
        .match = "NSE-5",
        .flash_size = 0,
    },
    // 7110 DSP is ROM-4 (verified, tools/dsp/rom_fingerprint.py /:
    // F073 PROM-target range 0x0926-0xF870, ~0% related to the 3310's ROM-6). So it gets the ROM-4
    // wiring, identical in shape to the 5110: native = the real C54x co-sim (DSP54_COSIM=1; the 7110
    // uploads its OWN ROM-4 blocks onto the shared ROM-4 PROM via DSP54_REALUP — proven 2026-06-17),
    // pass-through to the HLE otherwise; wasm = the ROM-4 HLE (no C54x core). Was mis-bound to
    // mad2_dsp_default (the ROM-6 HLE).
    // The native C54x co-sim CANNOT boot the 7110 DSP: it runs the 5110's resident PROM
    // (re/dsp-5110/raw/dsp_full.bin) while the 7110 uploads its OWN blocks (only 36%
    // F073-overlapped), so the 7110's code far-branches into incompatible PROM routines and
    // wedges (DSP stuck polling api_ram[0x1200] @PC 0x00ed, IMR=0x0001, never services the
    // cmd-0x70 self-test). Same limit the 3210 hits. So use the revision-correct ROM-4 HLE
    // responder on BOTH native and web (like the 6110); cosim stays a research-only opt-in
    // (bind mad2_dsp_c54x + DSP54_COSIM=1 to study the real DSP)
    // 7110-SPECIFIC ROM-4 responder (src/models/7110/dsp_7110.c): same revision as the
    // 5110/6110/3210 (mad2_dsp_rom4) but its own DSP self-test reply model — the phones differ.
    .dsp = &mad2_dsp_7110,           // ROM-4 HLE responder, 7110-specific (native + web)
};
