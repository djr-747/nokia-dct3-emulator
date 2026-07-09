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
    },
    .keypad = {
        .power_special_cols = 0x02,   // TODO: verify 7110 keypad scan
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
        // ADC AUDIT: temp REVISED 0x028 -> 0x0140 (320). The bit0 gate's "ch4<50"
        // is NON-PHYSICAL: ch4 (BTEMP) is an NTC, and the AUTHORITATIVE charge classifier
        // 0x4C83F6 needs raw ch4 in [307,347] (0x133..0x15B) = a real room-temperature reading.
        // A real 7110 at room temp reads BTEMP~320, FAILS the bit0 gate (>=50), takes the
        // 0x49F60E path ([0x16BC7C]=2, bit0 not set) and STILL reaches standby — so bit0 is NOT
        // the standby gate (the wall is verdict bit3/bit5, see dsp_7110.c). temp=0x028 forced a
        // moot bit0 while making the charge controller read "battery too cold" (state 5).
        // 0x0140 is faithful (mid charge-temp window) and benign (verdict still CONTACT SERVICE,
        // verified). bsi=0x150 stays in [318,370] (valid BLB-2 type); vbatt=0x2C0 >=438 gate;
        // charger=0 = clean "absent". Per-model BatterySpec → guard stays byte-identical.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x0140, .charger = 0x000,
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
        // [0x10EC50], 3410 [0x12BA10]; builder leaf 0x49F548 returns it, bit6 kept iff ==1). Setting
        // .dsp_uploaded=0x00167030u DOES work the sibling-standard way — the shared dsp_default IRQ4
        // block-ack pump (gated on [dsp_uploaded]==0) invokes the MCU's process stage 0x432EE0 once
        // (cb_reply==0) → [0x167030]=1 → builder keeps verdict bit6 (verified: [0x167030] 00→01 @2.2M).
        // HELD AT 0 ON PURPOSE: unlike the 2100/3410, the 7110 verdict has an EXTRA gate — bit3
        // (sequencer 0x310D5A "self-test not complete") — which the sibling mechanism does NOT clear
        // (bit3's only setter-path is gated on verdict bit5, and bit5 has NO setter in this build;
        // see src/models/7110/dsp_7110.c GATE B). With bit6 set but bit3 still set the verdict is
        // INCONSISTENT and the web MMI parks recomputing the tally → BLANK screen, which is worse than
        // the honest, stable CONTACT SERVICE. So we keep dsp_uploaded=0 (clean CONTACT SERVICE) until
        // the bit3/bit5 self-test-completion gate is resolved. (DSP7110_GATEA=1 enables the same
        // [0x167030] mechanism for native A/B.)
        .sim_gate = 0, .dsp_uploaded = 0,
        .get_string = 0, .w_get_string = 0, .faid_cksum = 0, .faid_cksum_val = 0,
    },
    .sigs = NULL,
    .n_sigs = 0,
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
