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
| **3210** | NSE-8 v6.00 | 🟡 partial | powers on to Contact Service (CS = MMI-block launch) |
| **3310** | NHM-5 v5.79 | ✅ standby | reference baseline; kept byte-identical by `make guard` |
| **3330** | NHM-6 v4.50 | 🟡 partial | Contact Service; DSP upload works, MMI verdict RE outstanding |
| **3350** | NHM-9 v5.22 | 🟡 partial | Contact Service; profile/RE incomplete |
| **3410** | NHM-2 v5.46 | ✅ standby | minor set-time clock-tick gap tracked separately, not a boot blocker |
| **5110** | NSE-1 v5.30 | ✅ locked | external-EEPROM layout (no in-flash partition). Boots into the MMI: the **HLE DSP** (web build) reaches PIN/standby; the **C54x DSP co-sim** (`DSP54_COSIM=1`, native GUI) reaches the faithful "Security code" lock. |
| **5210** | NSM-5 v5.40 | ✅ standby | |
| **5510** | NPM-5 v3.50 | 🔴 scaffold | clean power-off early; map/scratch unresolved |
| **6110** | NSE-3 v5.48 | ✅ locked | boots to the "Security code" (FAID) lock |
| **6210** | NPE-3 v5.56 | 🟡 partial | boots to Contact Service (DSP-gated); 4 MB map |
| **7110** | NSE-5 v5.01 | 🟡 partial | renders (SED1565 controller); Contact Service |
| **8210** | NSM-3 v5.31 | ✅ standby | |
| **8250** | NSM-3D v6.02 | ✅ standby | |
| **8290** | NSB-7 v5.22 | 🟡 partial | Contact Service; US-band DSP-upload boot handshake RE pending |
| **8850** | NSM-2 v5.31 | ✅ standby | NSM Family-A reference |
| **8855** | NSM-4 v5.13 | ✅ standby | |
| **8890** | NSB-6 v12.16 | 🟡 partial | Contact Service; same as 8290 (US-band 8850) |

**Boots to a usable state: 9 of 18 registered** — standby (3310, 3410, 5210, 8210, 8250, 8850, 8855) or the Security-code lock (5110, 6110).

## Not yet profiled

These DCT3 models have firmware in the wild but no `src/models/<model>/profile.c` yet, so the
emulator can't boot them until a profile is written. Most are close relatives of models above, so
several should be quick bring-ups:

| Model | Variant | Closest profiled relative |
|---|---|---|
| **5130** | NSK-1 | 5110-class |
| **5190** | NSB-1 | US-band (like 8290 / 8890) |
| **6130** | NSK-3 | 6110-class |
| **6150** | NSM-1 | rugged 6110-type |
| **6190** | NSB-3 | US-band |
| **6250** | NHM-3 | rugged 6210-type |
| **8810** | NSE-6 | external-EEPROM (5110-class) |

## How to check a model yourself

```bash
# Detect the model + see where it stops
./build/dct3_boot_trace "<your-image>.fls" 30000000 | grep -iE 'model *:|stopped'

# Render the screen at the end of the run
./build/dct3_boot_trace "<your-image>.fls" 60000000 | grep -A50 "LCD framebuffer"
```

The always-on post-mortem block (printed at every halt/reset) names the cause, the PC, and the
reset reason — read it top-to-bottom when a model stalls.
