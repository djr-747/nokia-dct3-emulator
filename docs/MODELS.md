# Model support roster

Single source of truth for which DCT3 models boot and how far. The registry lives in
`src/models/model.c`; per-model facts in `src/models/<model>/profile.c`. Model and version are
auto-detected from the flash header — nothing is hardcoded per image, so any matching firmware
you supply is picked up automatically.

**Legend** — **Boot**: ✅ reaches a usable screen (standby, or a normal interactive prompt) ·
🟡 partial (reaches Contact Service, or a lock it can't clear faithfully) · 🔴 scaffold (stalls
early). **Net**: ✅ registers on the synthetic network (operator name + signal bars) · 🔴 boots
but does not register · ❓ untested · **—** n/a (doesn't boot far enough).

The old "boots to the Security-code (FAID) lock" verdict is **gone** from the ROM-4 roster: that
prompt was never a FAID failure but an identity-sum vs stored-checksum mismatch (record
`0x0701`+6), and with the external EEPROM provisioned offline — coherent IMEI, FLASH-ID/FAID
record, both SIMlock parts, security code, and every checksum the boot validator compares — the
3210, 6110, 6150 and 8810 reach a registered standby instead. The security-level user setting is
provisioned to its erased factory default, so no code is asked for at power-up; 12345 is still
accepted if the setting is re-enabled.

Net values below were measured with a headless `dct3_boot_trace` run of 250M steps per model in
the default configuration (`LCDOUT` set per run, so no shared-screen cross-talk).

| Model | Code / ver | Boot | Net | Notes |
|---|---|---|---|---|
| **2100** | NAM-2 v5.84 | 🟡 partial | ❓ | 5210-personality profile; boots to the Security-code screen but rejects the EEPROM-baked code (12345), so nothing past it is tested — see issue #3 |
| **3210** | NSE-8 v6.00 | ✅ standby | ✅ | registered standby (SIM-present; no-SIM → "Insert SIM card"). Three root fixes: CCONT ch1 is a battery-VOLTAGE input (`adc_route[1]=2`; the source-7 undervolt guard reads ch1 as mV, 2100mV floor), the real keypad matrix RE'd from the firmware keymap table @0x2E2D58 (wiring differs from the 3310 despite identical physical keys), and the CCONT persistent-ready bit modelled so the startup readiness check passes organically (was Contact Service). Open: a reason-0x68 SWDSP fault fires after ~60s idle (5110-class DSP-fault latch) |
| **3310** | NHM-5 v5.79 | ✅ standby | ✅ | reference baseline; kept byte-identical by `make guard`. Supports calls + SMS; this firmware has no WAP browser |
| **3330** | NHM-6 v4.50 | ✅ standby | ✅ | on the provisioned EEPROM image it reaches a registered standby; on a bare image it stops at the "Security codes" prompt and **accepts the factory code 12345 → "Code accepted" → the first-boot Time wizard**. Root fix: the old hardcoded verdict/dsp_uploaded pair was borrowed from the wrong build (3310 v5.57) and the HLE pump's write to that live RAM cell derailed the boot — now self-healed per-build by `.sigs2 = MAD2_SIGS_3310` (v4.16 + v4.50 both resolve uniquely). Calls, SMS, and WAP browsing over CSD all work. Known quirk: the 3410-family recoverable reason-0x73 reset (~283M cycles, auto-recovered) |
| **3350** | NHM-9 v5.22 | ✅ standby | ✅ | **registers once it is given a good identity** (a sound PPM + EEPROM). This library image ships an **empty EEPROM partition**, and the factory config record (whose checksum the verdict-bit6 gate at 0x2540B8 compares) is never self-created, so on the bare image it stops at the "Security codes" prompt (12345 → "Code accepted" → Time wizard) and shows no signal bars. Graft a donor NHM FFS 'EEPROM' block first: `tools/graft_eeprom_block.py "<3350>.fls" "<3330>.fls"` (validated with the 3330 v4.50 donor). Same `.sigs2` family-line fix as the 3330 |
| **3410** | NHM-2 v5.46 | ✅ standby | ✅ | the handset the CSD / WAP / OTA-settings reverse engineering was done on: calls, SMS, WAP browsing and OTA settings provisioning all work. Minor set-time clock-tick gap tracked separately, not a boot blocker |
| **5110** | NSE-1 v5.30 | ✅ standby | 🔴 | external-EEPROM layout (no in-flash partition). **Passes local security faithfully under the C54x co-sim** (`DSP54_COSIM=1`): the provisioned SIMlock, IMEI and FLASH-ID records are decoded and judged by the real mask ROM, which takes it past "SIM card not accepted" to standby — the ROM verifies each decoded part's trailer against `data[0xB703]` (= 0x54C2) at PROM 0x4B8E/0x4B92 and the verdict lands in `data[0x1F11]` bit 2. The HLE DSP (web build) also reaches standby. Does not register yet — work to do |
| **5110i** | NSE-2 v5.53 | 🟡 partial | — | 2 MB 5110 refresh; boots into the MMI to Contact Service on the borrowed 5110 EEPROM (tune-checksum passes; a 5110i-specific record self-test — EEPROM records 0x31E/0x290 — isn't provisioned in the nse-1 blob) |
| **5130** | NSK-1 v5.30 | ✅ standby | 🔴 | 5110 sibling (Xpress-on); reaches standby, no registration yet — same gap as the 5110 |
| **5190** | NSB-1 v6.71 | ✅ standby | 🔴 | reaches standby (was Contact Service); no registration yet — same gap as the 5110 |
| **5210** | NSM-5 v5.40 | ✅ standby | ✅ | calls + SMS work; WAP untested |
| **5510** | NPM-5 v3.50 | 🔴 scaffold | — | clean power-off early; map/scratch unresolved |
| **6110** | NSE-3 v5.48 | ✅ standby | ✅ | registered standby, where it used to sit on the Security-code prompt: the stored security-settings checksum is re-derived from the blob's own identity (record 0x0701 @ EEPROM 0x0380, settings level @ 0x03A7) |
| **6130** | NSK-3 v5.61 | ✅ standby | 🔴 | 6110 sibling (settings level @ 0x03D7). Unlike the 6110 and 6150 it reaches standby **without** registering — no signal bars, no operator name, on the same default config where its siblings both register. Unexplained; being chased |
| **6150** | NSM-1 v5.23 | ✅ standby | ✅ | 6110 sibling (2 MB, own NokiX 24C128 EEPROM blob, settings level @ 0x03F3); registered standby |
| **6190** | NSB-3 v6.13 | 🟡 partial | — | Contact Service; same US NSB judged self-test FAIL as the other NSB variants |
| **6210** | NPE-3 v5.56 | ✅ standby | ✅ | reaches the registered idle screen (signal/battery/operator); calls, SMS and WAP browsing over CSD all work. Self-test resolved via the sibling pattern: `dsp_uploaded=0x16FFE4` (block-ack pump sets it → verdict bit6) + the ROM-6 self-test-complete responder posting the group-0x74 sub-13 ack that clears verdict bit2 (`src/models/6210/dsp_6210.c`, modelled on the 7110). |
| **6250** | NHM-3 v5.00 | ✅ standby | ✅ | 6210 sibling, resolved like-for-like: shared `mad2_dsp_6210` responder + `dsp_uploaded=0x16E474`, plus a repair of this library image's inconsistent RF-calibration checksum (EEPROM record 0x254 @ flash 0x5FAAEE: stored 0x7095 → correct sum 0x6D57, via `calib_cksum_*`). Also needed the 0x20002 DSP-in-reset bit4 model + 3210-style CCONT power-on cause. Calls, SMS and WAP browsing work |
| **7110** | NSE-5 v5.00 | ✅ standby | ✅ | interactive: SED1565 display (132-col DDRAM offset), RE'd 5×5 keypad matrix + **Navi roller** (3-phase optical encoder — mouse-wheel / arrows / on-screen buttons scroll menus, press to select). **Slow to boot**, then a drawn, key-navigable registered standby; a ~2-min DSP-temp watchdog is warm-recovered. Self-test completion is HLE'd (sibling-style DSP ack; no RAM/PC pokes). Nokia's first WAP handset, but it binds the ROM-4 engine, so there is no CSD bearer for it yet. Note its indicators live in a title bar, not the left column, so the signal-bar check used for the other models doesn't read it |
| **8210** | NSM-3 v5.31 | ✅ standby | ✅ | boots to the Security-code screen; **registers once the EEPROM security settings are correct**, then supports calls + SMS. No WAP browser in this firmware |
| **8250** | NSM-3D v6.02 | ✅ standby | ✅ | as the 8210 |
| **8290** | NSB-7 v5.22 | 🟡 partial | — | Contact Service; US-band DSP-upload boot handshake RE pending |
| **8810** | NSE-6 v6.02 | ✅ standby | ✅ | 6110-family slider (2 MB, own NokiX 24C256 EEPROM blob); registered standby, keypad interactive (digits register), no spurious reset. Its ISR gates on the I/O 0x34 matrix interrupt-source (profile `irq_src34`), unlike the 6110; its nse-6 DSP-fault latch (record 0x607 @ EEPROM 0x3F2) is provisioned like the 5110's. Slide cover not modelled. |
| **8850** | NSM-2 v5.31 | ✅ standby | ✅ | NSM Family-A reference; as the 8210 |
| **8855** | NSM-4 v5.13 | ✅ standby | ❓ | reaches standby; radio untested — the headless boot shows no signal bars |
| **8890** | NSB-6 v12.16 | 🟡 partial | — | Contact Service; same as 8290 (US-band 8850) |

**Boots to a usable state: 19 of 26 registered**, and **15 of those register** on the synthetic
network: 3210, 3310, 3330, 3350, 3410, 5210, 6110, 6150, 6210, 6250, 7110, 8210, 8250, 8810,
8850. The four that boot without registering are the 5110 family (5110, 5130, 5190) and the
6130; the 8855 and 2100 are untested.

Calls and SMS come with the **ROM-6** engine (3310, 3330, 3350, 3410, 5210, 6210, 6250, 8210,
8250, 8850); the **ROM-4** engine (3210, 5110 family, 6110 family, 7110, 8810) does camp +
registration only. **WAP over CSD** additionally needs the handset's own firmware to carry a WAP
browser and a settings receiver — confirmed on the 3330, 3410, 6210 and 6250.

## How to check a model yourself

```bash
# Detect the model + see where it stops
./build/dct3_boot_trace "<your-image>.fls" 30000000 | grep -iE 'model *:|stopped'

# Render the screen at the end of the run
./build/dct3_boot_trace "<your-image>.fls" 60000000 | grep -A50 "LCD framebuffer"
```

The always-on post-mortem block (printed at every halt/reset) names the cause, the PC, and the
reset reason — read it top-to-bottom when a model stalls.
