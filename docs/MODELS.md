# Model support roster

Single source of truth for which DCT3 models boot and how far. The registry lives in
`src/models/model.c`; per-model facts in `src/models/<model>/profile.c`. Model and version are
auto-detected from the flash header — nothing is hardcoded per image, so any matching firmware
you supply is picked up automatically.

**Legend** — ✅ standby (clock/icons) · 🟡 partial (boots into the MMI, one barrier left) ·
🔴 scaffold (profile exists, stalls early).

| Model | Code / ver | Status | Notes |
|---|---|---|---|
| **3310** | NHM-5 v5.79 | ✅ standby | reference baseline; kept byte-identical by `make guard` |
| **3410** | NHM-2 v5.46 | ✅ standby | minor set-time clock-tick gap tracked separately, not a boot blocker |
| **2100** | NAM-2 v5.84 | ✅ standby | 5210-personality profile |
| **5210** | NSM-5 v5.40 | ✅ standby | |
| **8210** | NSM-3 v5.31 | ✅ standby | |
| **8250** | NSM-3D v6.02 | ✅ standby | |
| **8850** | NSM-2 v5.31 | ✅ standby | NSM Family-A reference |
| **8855** | NSM-4 v5.13 | ✅ standby | |
| **5110** | NSE-1 v5.30 | 🟡 partial | external-EEPROM layout (no in-flash partition). Boots into the MMI: the **HLE DSP** (web build) reaches PIN/standby; the **C54x DSP co-sim** (`DSP54_COSIM=1`, native GUI) passes Contact Service to the faithful "Security code" (FAID) screen. |
| **6110** | NSE-3 v5.48 | 🟡 partial | boots to the "Security code" (FAID) screen |
| **6210** | NPE-3 v5.56 | 🟡 partial | boots to Contact Service (DSP-gated); 4 MB map |
| **7110** | NSE-5 v5.01 | 🟡 partial | renders (SED1565 controller); Contact Service |
| **3210** | NSE-8 v6.00 | 🟡 partial | powers on to Contact Service (CS = MMI-block launch) |
| **8290** | NSB-7 v5.22 | 🟡 partial | Contact Service; US-band DSP-upload boot handshake RE pending |
| **8890** | NSB-6 v12.16 | 🟡 partial | Contact Service; same as 8290 (US-band 8850) |
| **3330** | NHM-6 v4.50 | 🟡 partial | Contact Service; DSP upload works, MMI verdict RE outstanding |
| **3350** | NHM-9 v5.22 | 🟡 partial | Contact Service; profile/RE incomplete |
| **5510** | NPM-5 v3.50 | 🔴 scaffold | clean power-off early; map/scratch unresolved |

**Booting to standby: 8 of 18 registered** (2100, 3310, 3410, 5210, 8210, 8250, 8850, 8855).

## How to check a model yourself

```bash
# Detect the model + see where it stops
./build/dct3_boot_trace "<your-image>.fls" 30000000 | grep -iE 'model *:|stopped'

# Render the screen at the end of the run
./build/dct3_boot_trace "<your-image>.fls" 60000000 | grep -A50 "LCD framebuffer"
```

The always-on post-mortem block (printed at every halt/reset) names the cause, the PC, and the
reset reason — read it top-to-bottom when a model stalls.
