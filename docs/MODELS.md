# Model support roster

Single source of truth for which DCT3 models boot and how far. The registry lives in
`src/models/model.c`; per-model facts in `src/models/<model>/profile.c`. Model and version are
auto-detected from the flash header — nothing is hardcoded per image, so any matching firmware
you supply is picked up automatically.

**Legend** — ✅ boots (standby, or the normal Security-code lock) · 🟡 partial (reaches Contact
Service, or a lock it can't clear faithfully) · 🔴 scaffold (stalls early).

| Model | Code / ver | Status | Notes |
|---|---|---|---|
| **2100** | NAM-2 v5.84 | 🟡 partial | 5210-personality profile; boots to the Security-code (FAID) screen but rejects the EEPROM-baked code (12345) — see issue #3 |
| **3210** | NSE-8 v6.00 | ✅ interactive | boots to the "Security codes" prompt (SIM-present; no-SIM → "Insert SIM card") and **accepts the factory code 12345 → "Code accepted"**. Two root fixes: CCONT ch1 is a battery-VOLTAGE input (`adc_route[1]=2`; the source-7 undervolt guard reads ch1 as mV, 2100mV floor), and the real keypad matrix RE'd from the firmware keymap table @0x2E2D58 (wiring differs from the 3310 despite identical physical keys). Open: a reason-0x68 SWDSP fault fires after ~60s idle at the prompt (5110-class DSP-fault latch) |
| **3310** | NHM-5 v5.79 | ✅ standby | reference baseline; kept byte-identical by `make guard` |
| **3330** | NHM-6 v4.50 | ✅ interactive | boots to the "Security codes" prompt and **accepts the factory code 12345 → "Code accepted" → the first-boot Time wizard** (web, SIM-present). Root fix: the old hardcoded verdict/dsp_uploaded pair was borrowed from the wrong build (3310 v5.57) and the HLE pump's write to that live RAM cell derailed the boot — now self-healed per-build by `.sigs2 = MAD2_SIGS_3310` (v4.16 + v4.50 both resolve uniquely). Known quirk: the 3410-family recoverable reason-0x73 reset (~283M cycles, auto-recovered) |
| **3350** | NHM-9 v5.22 | ✅ interactive | same fix + flow as the 3330 (`.sigs2` family line; 12345 → "Code accepted" → Time wizard) — but this library image ships an **empty EEPROM partition**, and the factory config record (whose checksum the verdict-bit6 gate at 0x2540B8 compares) is never self-created. Graft a donor NHM FFS 'EEPROM' block first: `tools/graft_eeprom_block.py "<3350>.fls" "<3330>.fls"` (validated with the 3330 v4.50 donor) |
| **3410** | NHM-2 v5.46 | ✅ standby | minor set-time clock-tick gap tracked separately, not a boot blocker |
| **5110** | NSE-1 v5.30 | ✅ locked | external-EEPROM layout (no in-flash partition). Boots into the MMI: the **HLE DSP** (web build) reaches PIN/standby; the **C54x DSP co-sim** (`DSP54_COSIM=1`, native GUI) reaches the faithful "Security code" lock. |
| **5110i** | NSE-2 v5.53 | 🟡 partial | 2 MB 5110 refresh; boots into the MMI to Contact Service on the borrowed 5110 EEPROM (tune-checksum passes; a 5110i-specific record self-test — EEPROM records 0x31E/0x290 — isn't provisioned in the nse-1 blob) |
| **5130** | NSK-1 v5.30 | ✅ locked | 5110 sibling (Xpress-on); boots to the "Security code" (FAID) lock |
| **5190** | NSB-1 v6.71 | 🟡 partial | Contact Service; US NSB build fails an extra judged self-test element (expected-nonzero result pair) before the shared DSP reply lands — RE pending, like the other NSB variants |
| **5210** | NSM-5 v5.40 | ✅ standby | |
| **5510** | NPM-5 v3.50 | 🔴 scaffold | clean power-off early; map/scratch unresolved |
| **6110** | NSE-3 v5.48 | ✅ locked | boots to the "Security code" (FAID) lock |
| **6130** | NSK-3 v5.61 | ✅ locked | 6110 sibling; boots to the "Security code" (FAID) lock |
| **6150** | NSM-1 v5.23 | ✅ locked | 6110 sibling (2 MB, own NokiX 24C128 EEPROM blob); boots to the "Security code" (FAID) lock |
| **6190** | NSB-3 v6.13 | 🟡 partial | Contact Service; same US NSB judged self-test FAIL as the 5190 |
| **6210** | NPE-3 v5.56 | ✅ standby | reaches the idle screen (signal/battery/operator). Self-test resolved via the sibling pattern: `dsp_uploaded=0x16FFE4` (block-ack pump sets it → verdict bit6) + the ROM-6 self-test-complete responder posting the group-0x74 sub-13 ack that clears verdict bit2 (`src/models/6210/dsp_6210.c`, modelled on the 7110). |
| **6250** | NHM-3 v5.00 | ✅ standby | 6210 sibling, resolved like-for-like: shared `mad2_dsp_6210` responder + `dsp_uploaded=0x16E474`, plus a repair of this library image's inconsistent RF-calibration checksum (EEPROM record 0x254 @ flash 0x5FAAEE: stored 0x7095 → correct sum 0x6D57, via `calib_cksum_*`). Also needed the 0x20002 DSP-in-reset bit4 model + 3210-style CCONT power-on cause. |
| **7110** | NSE-5 v5.00 | ✅ standby | interactive: SED1565 display (132-col DDRAM offset), RE'd 5×5 keypad matrix + **Navi roller** (3-phase optical encoder — mouse-wheel / arrows / on-screen buttons scroll menus, press to select). SIM-present boots to a drawn, key-navigable standby; a ~2-min DSP-temp watchdog is warm-recovered. Self-test completion is HLE'd (sibling-style DSP ack; no RAM/PC pokes). |
| **8210** | NSM-3 v5.31 | ✅ standby | |
| **8250** | NSM-3D v6.02 | ✅ standby | |
| **8290** | NSB-7 v5.22 | 🟡 partial | Contact Service; US-band DSP-upload boot handshake RE pending |
| **8810** | NSE-6 v6.02 | ✅ locked | 6110-family slider (2 MB, own NokiX 24C256 EEPROM blob); boots to the "Security code" (FAID) lock, keypad interactive (digits register), no spurious reset. Its ISR gates on the I/O 0x34 matrix interrupt-source (profile `irq_src34`), unlike the 6110; its nse-6 DSP-fault latch (record 0x607 @ EEPROM 0x3F2) is provisioned like the 5110's. Slide cover not modelled. |
| **8850** | NSM-2 v5.31 | ✅ standby | NSM Family-A reference |
| **8855** | NSM-4 v5.13 | ✅ standby | |
| **8890** | NSB-6 v12.16 | 🟡 partial | Contact Service; same as 8290 (US-band 8850) |

**Boots to a usable state: 19 of 26 registered** — standby (3310, 3410, 5210, 6210, 6250, 7110, 8210, 8250, 8850, 8855), interactive security-code entry (3210, 3330, 3350 — factory 12345 accepted), or the Security-code lock (5110, 5130, 6110, 6130, 6150, 8810).

## How to check a model yourself

```bash
# Detect the model + see where it stops
./build/dct3_boot_trace "<your-image>.fls" 30000000 | grep -iE 'model *:|stopped'

# Render the screen at the end of the run
./build/dct3_boot_trace "<your-image>.fls" 60000000 | grep -A50 "LCD framebuffer"
```

The always-on post-mortem block (printed at every halt/reset) names the cause, the PC, and the
reset reason — read it top-to-bottom when a model stalls.
