# Model support roster

Single source of truth for which DCT3 models boot and how far. Registry lives in
`src/models/model.c`; per-model facts in `src/models/<model>/profile.c`. Bring-up method:
`docs/firmware-bringup-playbook.md`. Last verified: **2026-06-06**.

**Legend** — ✅ standby (clock/icons) · 🟡 partial (boots into MMI, one barrier left) ·
🔴 doesn't boot (scaffold) · ⚫ no profile yet.

| Model | Code / ver | Image used | Status | Notes |
|---|---|---|---|---|
| **3310** | NHM-5 v5.79 | `Factory Reset 3310 NR1 v5.79.fls` | ✅ **standby** | baseline; must stay byte-identical |
| **5210** | NSM-5 v5.40 | `Nokia 5210 NSM-5 v5.40 A (EEPROM).fls` | ✅ **standby** | needs the **(EEPROM)** image (blank-EEPROM gen) |
| **8210** | NSM-3 v5.31 | `Nokia 8210 NSM-3 v5.31 A (EEPROM).fls` | ✅ **standby** | needs the **(EEPROM)** image (stale-checksum CKFIX) |
| **8250** | NSM-3D v6.02 | `Nokia 8250 NSM-3D v6.02 J.fls` | ✅ **standby** | stock image (EEPROM self-consistent) |
| **8850** | NSM-2 v5.31 | `Nokia 8850 NSM-2 v5.31 A.fls` | ✅ **standby** | NSM Family-A reference |
| **8855** | NSM-4 v5.13 | `Nokia 8855 NSM-4 v5.13 P (EEPROM).fls` | ✅ **standby** | needs the **(EEPROM)** image (blank-EEPROM gen) |
| **3410** | NHM-2 v5.46 | `Nokia 3410 NHM-2 v5.46 (assembled).fls` | ✅ **standby** | boots to standby; known minor clock-tick gap (set-time) tracked separately, not a boot blocker |
| **8290** | NSB-7 v5.22 | `Nokia 8290 NSB-7 v5.22 A.fls` | 🟡 partial | CONTACT SERVICE; verdict+dsp_uploaded RE'd, DSP-upload handshake RE pending |
| **8890** | NSB-6 v12.16 | `Nokia 8890 NSB-6 v12.16 A.fls` | 🟡 partial | same as 8290 (US-band 8850) |
| **3330** | NHM-6 v4.50 | `Nokia 3330 NHM-6 v4.50 A.fls` | 🔴 scaffold | CONTACT SERVICE. EEPROM populated (56 KB) + DSP upload works (acks 3397); needs MMI verdict/dsp_uploaded RE (NHM-6 / 3310-class — 8850 verdict-writer sig does not port). eeprom_reset entry added (RESET_FN 0x2514B0). |
| **3350** | NHM-9 v5.22 | `Nokia 3350 NHM-9 v5.22 A.fls` | 🔴 scaffold | CONTACT SERVICE. EEPROM BLANK; eeprom_reset entry added (RESET_FN 0x2566A8) but the init under-populates (~496 B vs ~35 KB) AND verdict/dsp_uploaded need RE (NHM-9 / 3410-class). |
| **5510** | NPM-5 v3.50 | `Nokia 5510 NPM-5 v3.50 A.fls` | 🔴 scaffold | clean POWER-OFF early; map/scratch unresolved |
| **7110** | NSE-5 v5.01 | `Nokia 7110 NSE-5 v5.01 A.fls` | 🔴 scaffold | WILD-PC (crash); 4 MB map/scratch unresolved |
| **5110** | NSE-1 v5.30 | `Nokia 5110 NSE-1 v5.30 A.fls` | 🟡 partial | **MAD1 / external EEPROM** (1 MB, no in-flash partition), **native only** — the web build doesn't link the C54x core. Boots into the MMI: with the **C54x DSP co-sim** (`DSP54_COSIM=1`) it passes Contact Service to the faithful "Security code" (FAID) screen; with the **HLE DSP** it reaches PIN/standby. reason-0x68 idle-reset and the DSP self-test (cmd-0x70 / COBBA loopback) are resolved. SSOT: `docs/5110-DSP-CANONICAL-STATE.md`. |
| **8810** | NSE-6 v6.02 | `Nokia 8810 NSE-6 v6.02 A.fls` | ⚫ MAD1-family | NSE-6 — same MAD1 layout as the 5110 (header at 0x170004 = block+4, not 0x1FC). MAD1 external-EEPROM/serial-bus; not yet brought up. |

**Booting to standby: 7 of 13 registered** (3310, 3410, 5210, 8210, 8250, 8850, 8855).

## Repaired-EEPROM images (the `(EEPROM)` files)
Three models ship a blank or stale EEPROM and need a fixed image to reach standby. The fixed
images live in `firmware/` but are **gitignored** (all `*.fls` are excluded — firmware is never
committed). Regenerate any time:

```bash
tools/eeprom_reset 5210 --full --capture "firmware/Nokia 5210 NSM-5 v5.40 A (EEPROM).fls"
tools/eeprom_reset 8855 --full --capture "firmware/Nokia 8855 NSM-4 v5.13 P (EEPROM).fls"
# 8210 is a one-byte stale-checksum CKFIX, not a full regen — see docs/8210-8250-bringup.md §8210.4
```

Current `(EEPROM)` images on disk: 5210, 8210, 8855.

## Open bring-up items
1. **8290 / 8890** — RE the US-band DSP-code upload boot handshake (the IRQ4 pump never starts;
   they park `[0x10004]` but poll RAM `[0x12DF08]==6`). One barrier from standby.
2. **3410** — standby works; minor clock-tick (set-time) gap tracked in its memory tags.
3. **3330 / 3350** — NHM scaffolds at CONTACT SERVICE; RE verdict/dsp_uploaded (not yet 8850-class).
4. **5510 / 7110** — early POWER-OFF / WILD-PC; need map + DSP-scratch RE.
5. **8810** — investigate the non-standard flash layout before a profile is meaningful.

## How to check a model yourself
```bash
./build/dct3_boot_trace "<image>" 60000000 | grep -A50 "LCD framebuffer"   # render the screen
./build/dct3_boot_trace "<image>" 30000000 | grep -iE 'model  :|stopped'   # detect + stop reason
```
References: `docs/8210-8250-bringup.md` (NSM 8-series), `docs/5210-bringup-HANDOFF.md`,
`docs/eeprom-fix-howto.md`, `docs/firmware-bringup-playbook.md`.
