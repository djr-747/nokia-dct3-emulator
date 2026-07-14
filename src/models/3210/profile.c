// Nokia 3210 model profile — NSE-8 (v6.00), an early MAD2 DCT3 (1999), same
// peripheral attachment family as the 5110 (NSE-1) and 6110 (NSE-3): the CCONT power ASIC
// and an EXTERNAL serial-bus EEPROM hang off a BIT-BANGED SERIAL bus (mad2_bus_serial), not
// the later memory-mapped GENSIO + in-flash EEPROM. "hw is 5110, but may differ;
// rom4 should work with cosim." So this starts as a near-clone of src/models/5110/profile.c
// with the 3210's three real deltas:
//
//   1. 2 MB flash (vs the 5110/6110's 1 MB). Still NO in-flash EEPROM partition — the
//      NVRAM is the external serial chip. eeprom_base/size stay 0.
//   2. External EEPROM = the NokiX virgin **nse-8** image — a **16 KB 24C128** (2-byte word
//      addressing, like the 6110's 24C64), vs the 5110's 2 KB 24C16. (firmware/nse-8.bin,
//      baked as EE_NSE8.) ext_eeprom.c is device-size-aware (two_byte path for >2 KB).
//   3. Ident "NSE-8" — the MCU version header is NOT at the canonical flash 0x1FC (0x1FC is
//      code: `b530 1c02…`), so model_detect falls back to a whole-image string search, like
//      the 5110/6110. "NSE-8" is unambiguous.
//
// DSP: ROM-4 (NSE generation), same revision as the 5110/6110. Uses the revision-correct
// ROM-4 HLE responder (mad2_dsp_rom4) on BOTH native and web — NOT the C54x co-sim backend:
// mad2_dsp_c54x loads the *5110's* recovered DSP image (re/dsp-5110/raw/dsp_full.bin), but the
// 3210 uploads its OWN DSP code blocks from its own flash, so co-sim drives that static image
// into a degenerate PC=0x0000 bootstub loop and never produces the boot reply. Same call the
// 6110 makes. (A faithful 3210 co-sim would need the 3210's own
// recovered ROM-4 image — out of scope for bring-up.)
//
// The DCT3 RTOS/MMI core is shared with the 3310/5110/6110, so MAD2_SIGS resolves the
// per-build firmware addresses over the 3210 flash. The build-specific .fw fallbacks below
// START as the 6110/5110 values (closest analogs) and are corrected from the sig-resolution
// (sigdump) / boot post-mortem during bring-up; the HW-fixed HPI-window cells
// (cobba/mailbox/MDIRCV/boot-status) are identical across DCT3 and are correct as-is.

#include "models/model.h"
#include "models/mad2_sigs.h"      // shared DCT3 RTOS firmware-address signatures
#include "mad2/ext_eeprom_blobs.h" // baked NokiX virgin external-I2C-EEPROM images

// 3210 keypad — candybar Family B (Menu / Names-C / up / down / digits): same PHYSICAL keys
// as the 3310, but the matrix wiring is the 3210's own. RE'd 2026-07-14 from the firmware
// keymap table @ flash 0x2E2D58 (25 bytes, keycode = table[drive*5 + readbit]; 0x3E = none;
// right next to the ADC channel-map table 0x2E2D74) + the matrix scanner (bitplane
// `keypad_matrix_scan` 0x2B2F90: drives lines 0..3 via I/O 0x28/0xA8, reads 5 bits on 0x2A
// active-low; scancode = drive*5 + bit; specials = 0x80+bit on the all-driven pass, mapped
// via the appended 5-byte block [20..24] = {0x0D PWR,-,-,-,-}). Live-verified: the old
// 5110-placeholder presses 1,2,3,4,5,menu typed "0#6341" at the security prompt — exactly
// what this table predicts for those (row,col) cells.
static const KeyLine keylines_3210[] = {
    {KK_1,2,1,0}, {KK_2,3,1,0}, {KK_3,2,2,0},   // keycodes 0x01/0x02/0x03
    {KK_4,2,3,0}, {KK_5,3,2,0}, {KK_6,1,4,0},   // 0x04/0x05/0x06
    {KK_7,2,4,0}, {KK_8,3,3,0}, {KK_9,3,4,0},   // 0x07/0x08/0x09
    {KK_STAR,0,4,0}, {KK_0,1,2,0}, {KK_HASH,1,3,0},   // 0x0C/0x0A/0x0B
    {KK_SOFT1,0,1,0},   // left key "Menu" — keycode 0x19
    {KK_SOFT2,0,2,0},   // right key "Names" / C — keycode 0x1A
    {KK_UP,0,3,0},      // scroll up   — 0x17
    {KK_DOWN,1,1,0},    // scroll down — 0x18
    // Power = special-scan bit0 (keymap special block [20]=0x0D). NOTE the 3210's power
    // BUTTON is wired to the CCONT (PWRONX, .ccont_poweron_int) — this matrix line is the
    // firmware's secondary/service view of it.
    {KK_PWR,0,0,0x01},
};

const ModelProfile model_3210 = {
    .name = "3210",
    .description = "Nokia 3210 (NSE-8, early MAD2, PCD8544 84x48) — external 24C128 EEPROM / serial bus",
    .mem = {
        .flash_base  = 0x00200000u,
        .flash_size  = 0x00200000u,   // 2 MB (MCU+PPM only — external EEPROM, no in-flash partition)
        .boot_entry  = 0x00200040u,
        .io_lo       = 0x00010000u,
        .io_hi       = 0x00100000u,
        .mmio_base   = 0x00020000u,
        .mmio_size   = 0x00000100u,
        .eeprom_base = 0,             // external EEPROM (serial bus); no in-flash partition
        .eeprom_size = 0,
    },
    .lcd = {
        // 84x48 PCD8544 on the MAD2 memory-mapped GENSIO (NOT the 5110's serial-bus LCD ports).
        // The 3210 is a HYBRID: serial-attached external EEPROM, but a memory-mapped MAD2 GENSIO
        // for the CCONT + LCD (RE'd: the LCD driver 0x2B1D72/0x2B1D82 writes the standard GENSIO
        // LCD ports). Use the MAD2 defaults (== 3310: data 0x2E / cmd 0x6E). The earlier 5110
        // serial ports (0x2B/0x2C) were wrong — 0x2C collided with the CCONT reg-select write and
        // the real LCD writes (0x6E/0x2E) fell through, so nothing ever drew. (data/cmd vs the
        // 0x6E/0x2E the driver disasm showed may still need a swap — verify rendering post-standby.)
        .controller = LCD_PCD8544,
        .width = 84, .height = 48, .banks = 6,
        .io_data = 0x2E, .io_cmd = 0x6E,
    },
    .keypad = {
        // MAD2 memory-mapped keypad (NOT the 5110's serial-bus keypad): the 3210 writes the MAD2
        // scan ports 0x2A/0x28/0xA8 (col/row/dir) and never the serial 0x30 — same hybrid as its
        // CCONT/LCD. So 3310-class PLAIN scan (0xE0 = power special-scan): with the serial config
        // the power-key special pass was never delivered → the firmware saw no power key held and
        // cleanly powered off after the boot logo. Default MAD2 ports (col/row/dir = 0 → 0x2A/0x28/0xA8).
        .power_special_cols = 0x01,   // special block [20]=0x0D → power on special bit0
        .family  = KP_FAMILY_3310,   // Family B (Menu/Names) — candybar, no soft/send/end/vol
        .lines   = keylines_3210,    // (row,col) still placeholder — RE the MAD2 matrix post-standby
        .n_lines = (int)(sizeof(keylines_3210) / sizeof(keylines_3210[0])),
        .scan = KP_SCAN_PLAIN,       // 3310-class: 0xE0 = power special-scan
    },
    .led = {
        // Green EL backlight = PUP_GENIO (I/O 0x20) bit6 (0x40), like the 5110. Vibrates via
        // an accessory vibra battery pack on the 3310's channel (I/O 0x15 bit4), no internal motor.
        .lcd_rgb = 0x55C955, .lcd_mask = 0x40, .no_kbd_led = 1,
    },
    .battery = {
        // CCONT A/D readings (live: fed to the fw via mad2_ccont + .adc_route below). The 3210 uses a
        // ~2.6V NiMH pack (BMC-3 class), unlike the ~3.6V Nokias the 5110 placeholders assumed.
        //   .vbatt  0x2C0 — in the VBATT accept window [0x2BE..0x314] (gate 0x2708A6). The window is a
        //                   FIXED fw constant; the board divider lands a healthy 2.6V pack there, so do
        //                   NOT rescale by 2.6/3.6 (that drops below 0x2BE and fails the window).
        //   .bsi    0x150 — battery-type/size indicator. The fw RECOGNISES a pack by interpolating
        //                   BSI(+BTEMP) (reader 0x2A68C6 -> table chain 0x2B64FC/0x2B59E0/0x2B5446/
        //                   0x2B63CC); only a RECOGNISED pack lets the analog subsystem complete and
        //                   report ready. 0x150 is a placeholder, NOT yet a verified recognised pack.
        //   .temp   0x140 — BTEMP placeholder, PENDING pack-recognition RE. Two facts pin the target:
        //                   (1) CONTACT SERVICE gate (charge fn 0x2A90AC: read ch4; `cmp #26; blt`)
        //                       wants ch4 < 26 to PROCEED ([0x112448]=0) — so the real value is <26.
        //                   (2) readiness completion needs a RECOGNISED (bsi,temp) PAIR — value-specific
        //                       & non-monotonic (only isolated temps completed it under a given bsi) =
        //                       pack recognition, NOT a threshold. No single temp tried satisfies BOTH.
        //                   Left at the 5110 placeholder 0x140 (boot stays at CONTACT SERVICE = the
        //                   faithful "uncharacterised battery" state) until the pack-recognition table
        //                   yields the (vbatt,bsi,temp) for a real 2.6V pack with temp<26. (temp=0x14
        //                   cleared CONTACT SERVICE but fed an UNRECOGNISED pack -> readiness stalled
        //                   blank; reverted 2026-06-20.) 3310's BTEMP gate is INVERTED (cmp#50 bge=
        //                   normal) — do not port its 0x200.
        //   .charger 0    — VCHAR; present-test 0x2B0866 `cmp #100; bgt` → 0 = no charger connected.
        .vbatt = 0x2C0, .bsi = 0x150, .temp = 0x140, .charger = 0x000,
    },
    // CCONT on the MAD2 memory-mapped GENSIO (NOT the 5110's bit-banged serial CCONT). The 3210
    // is a hybrid: its CCONT driver (0x2AFB94 reg-select->I/O 0x2C, poll status 0x6D bit2, read
    // data<-0x6C) uses the standard GENSIO ports, so use the MAD2 defaults. The earlier 5110 serial
    // ports (ccont_w 0x2A / ccont_r 0x2D) meant CCONT reg-selects landed on the LCD-cmd port and
    // reads fell through to garbage -> CCONT-dependent readiness/init never completed -> the
    // power-up sequencer (0x2A92D2) spun forever. Only the EXTERNAL EEPROM is serial (mad2_bus_serial
    // owns I/O 0x20/0x24); CCONT/LCD fall through it to the shared MMIO GENSIO path.
    .gensio = { .ccont_w = 0x2C, .ccont_r = 0x6C, .start = 0x2D },
    .bus = &mad2_bus_serial,
    .asic = {
        .irq_sources = 8,
    },
    .fw = {
        // Build-specific RAM/code cells — START as the 6110/5110 fallbacks; MAD2_SIGS resolves the
        // 3210 v6.00 addresses over its flash (sigdump shows what resolves vs falls back). The
        // verdict (MMI-ready / self-test gate) is the boot-critical per-build cell and must be
        // RE'd for v6.00 (gen_sig-port from the 5110 0x10FDDD / 6110 0x10FDE1 MMI-ready gate);
        // 0x10FDE1 is the v6-era closest guess until located.
        // verdict = 0x11FED1 — the MMI startup-screen display gate, RE'd 2026-06-19 (was the
        // unlocated 6110 fallback 0x10FDE1, a dead cell never written on v6.00). The "CONTACT
        // SERVICE" decision is at 0x237B52: `ldr r5,=0x11FE68; ldrb r0,[0x69+r5]` (=[0x11FED1])
        // `lsr #7; bcs` — bit7 SET = standby, CLEAR = draw the inline "CONTACT SERVICE" string
        // (@flash 0x237CC4). NOTE: this is a STABLE startup-incomplete screen, NOT a DSP self-test
        // verdict / reset (boot runs clean to 80M, no halt). The CONTACT SERVICE text itself is
        // gated on the BTEMP A/D gate now cleared by .battery.temp (proceed-eval 0x29BC70, gate
        // [0x112448]==0 @0x29BC92). Past that the phone is alive but BLANK: a deeper readiness
        // sequencer (0x2A92FC) spins on check-3 0x28FF14 needing [0x111C93]!=0 — set by a
        // broker-driven measurement task (sub-id 0x4943) that never runs on the 3210 (the
        // "0x290D60/0x2D7346 task-18 launch" theory was DISPROVEN: a poked-3310 sets the flag with
        // that launch firing 0x). This cell is for telemetry/post-mortem labelling, not a boot fix.
        // dsp_uploaded = 0x110C2C — located for the 3210 v6.00: the DSP-block-loader setter
        // ports 92% (5110 0x271D1C) / 96% (3310 0x2BB050) to 3210 0x290C9A, which does
        // `strb r4,[r8]` with r8=0x110C2C (WATCH 0x290C9C r0=0x110C2C, written @step 1.4M as
        // the upload completes). The 5110 fallback 0x10A9E4 is a dead cell here — pointing the
        // responder at it made it believe the upload never finished, so it re-drove the IRQ4
        // block-ack loop forever and never advanced task-14 to the SWDSP-ready state that the
        // power-up readiness gate (0x28FF14, SWDSP_STATE_RECORDS) polls.
        .verdict = 0x0011FED1u, .sim_gate = 0, .dsp_uploaded = 0x00110C2Cu,
        // HW-fixed HPI-window cells — identical across DCT3, correct as-is.
        .cobba        = 0x000100E0u,
        .dsp_cb_req   = 0x000100E2u,
        .dsp_cb_reply = 0x000100E4u,
        .dsp_mbox0    = 0x000100FEu,
        .dsp_mbox1    = 0x00010100u,
        .mdircv_q     = 0x00010100u,
        .mdircv_head  = 0x000101CAu,
        .mdircv_tail  = 0x000101C8u,
        .mdisnd_q     = 0x00010000u,    // MCU->DSP request queue (HPI layout-invariant; see 3310)
        .mdisnd_tail  = 0x000100A4u,    // verified by the 2026-07-15 per-model RAMWATCH sweep
        .get_string = 0x00277B98u, .w_get_string = 0x0027790Eu,   // 5110 fallback; sig-resolve for 3210
        .faid_cksum = 0, .faid_cksum_val = 0,
        .dsp_boot_status = 0x00010004u, .dsp_boot_ready = 4,       // ROM-4 DSP reports version 4
        .dsp_boot_status2 = 0x00010006u,
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
        // String-only match: header not at 0x1FC (it's code) -> whole-image search for "NSE-8".
        .match = "NSE-8",
        .flash_size = 0,
    },
#ifndef __EMSCRIPTEN__
    .dsp = &mad2_dsp_c54x,            // native: real C54x co-sim available (DSP54_COSIM=1); else HLE fallthrough
#else
    .dsp = &mad2_dsp_rom4,            // wasm (no C54x core): ROM-4 HLE responder
#endif
    // External 24C128 (16 KB) I2C EEPROM — NokiX virgin nse-8, baked in (firmware/nse-8.bin).
    .i2c_eeprom_default = EE_NSE8,
    .i2c_eeprom_size    = EE_NSE8_LEN,
    // 3210 oddball: its DSP bulk code-upload window is aliased +0x1000 (firmware writes
    // loader1/blocks to MCU 0x11xxx, e.g. loader1 -> 0x11E00 = DSP word 0xF00), while the
    // control/mailbox cells stay at 0x100xx like every other DCT3. The C54x co-sim window
    // is anchored at 0x10000, so declare the alias base and the backend folds 0x11xxx
    // accesses back into the 0x10000 window. (NATIVE cosim only; see model.h.)
    .dsp_hpi_alias_base = 0x00011000u,
    // 3210 oddball #2: its external-EEPROM I2C clock is bit-banged on I/O 0x20 bit3, not the
    // early-MAD2 default bit2 (RE'd: bit-bang routine 0x2B0332 sets bit3 as SCL; bit2 is held high).
    // With the default bit2 reader the FSM saw a constant clock-high → 53k START/STOP, zero
    // reads → the EEPROM never provisioned. SDA stays bit0. (5110/6110 keep SCL=bit2.)
    .i2c_sda_bit = 0,
    .i2c_scl_bit = 3,
    // 3210 oddball #3: the power button is wired to the CCONT (PWRONX), NOT the keypad matrix
    // — which is why delivering the power key via the keypad special-scan had no effect. The
    // 3210's power-on classifier (0x2AF0AE, reached with cold-boot reason 10) reads ONLY CCONT
    // reg 0x0E (select 0x74, field 0x10 → descriptor 0x70 = reg 0x0E << 3) and maps bit7→mode3
    // (RTC-alarm), bit2→mode1, bit1→mode2 (normal PWR-key boot); no bit set → "no valid
    // power-on reason" → it sends msg 0x282 to task 18 and powers off (the deterministic 2.2M
    // ccont reg5=0 shutdown at 0x2714CA→0x2B4E16). bit1 is the only cause that sustains a
    // normal power-on (bit2's mode-1 falls back through the same shutdown on the next message
    // cycle). A cold boot = the user holding PWR, so latch bit1 at reset. With it the phone
    // powers on normally and reaches CONTACT SERVICE (the separate DSP self-test/readiness gate).
    .ccont_poweron_int = 0x02,
    // 3210 oddball #4: its CCONT A/D MUX wires VBATT to channel 0, not the MAD2-standard
    // channel 2. RE'd: the boot battery reader 0x2A84B0 reads CCONT A/D channel 0 (5-sample
    // average) and the task-18 analog handler 0x270848 gates it to the VBATT window
    // [0x2BE..0x314] (=702..788) that brackets the modelled VBATT 0x2C0; the firmware's
    // per-quantity channel-map table (flash 0x2E2D74 = {00 04 05 06 07 03 02 01}, vs the
    // 3310's {00 04 05 06 07 03 01 02}) and the literal channel selects (ch0/1/3/4/5/6 read,
    // ch2 NEVER read) confirm VBATT lives on ch0 here. Route ch0 -> adc[2] so the modelled
    // battery voltage is returned on the channel the 3210 actually reads. (ch3/4/5 keep the
    // standard BSI/BTEMP/VCHAR identity mapping — those the 3210 reads on their usual lines.)
    // ch0 -> vbatt (above). ch1 -> vbatt TOO (corrected 2026-07-14; was [1]=3=bsi, WRONG): CCONT ch1
    // on the 3210 is a SECOND battery-VOLTAGE input, not BSI. Proof: (a) the per-quantity channel-map
    // (flash 0x2E2D74) puts measurement QUANTITY 7 on ch1 where the 3310's table (0x32E73C) puts the
    // same quantity on ch2 = its VBATT channel; (b) the quantity-7 classifier input (0x27CC74) and the
    // direct ch1 reader (0x2A68C6, previously mis-labelled BSI_READER) both apply the SAME
    // BATT_CHAR_STRUCT [0x11FDE0]+52/+56 voltage calibration, and the classifier scales it *1500/313
    // to MILLIVOLTS for the source-7 undervoltage guard (0x27D5FC: <2100 mV → power_off 0x2B4E16) —
    // a 2.1V floor is exactly a 2.6V-NiMH-pack cutoff; (c) BSI is genuinely on ch3 (reader 0x2A90B4),
    // which mad2 already identity-maps to adc[3]=.bsi. The old [1]=3 fed the voltage guard the BSI
    // code 0x150 → ~1560 mV → deterministic clean power-off @~948K steps. With [1]=2 the guard reads
    // .vbatt=0x2C0 → ~3280 mV → passes. (ch3/4/5 keep identity BSI/BTEMP/VCHAR mapping.)
    .adc_route = { [0] = 2, [1] = 2 },
};
