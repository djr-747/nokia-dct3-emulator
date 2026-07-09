# Multi-model architecture — decoupling the platform from the 3310

How the emulator stops being a 3310 emulator and becomes a *DCT3* emulator that
boots the 3310 first. Goal: support 8850, 7110, 3210, 3330, … without per-model
`#ifdef`s and without breaking the model that already boots. Read
[`architecture-3310.md`](architecture-3310.md) first for the core/platform/model
split this builds on.

## The invariant (from)

```
core      never knows what a phone is        (src/core — already true)
platform  never knows what a 3310 is         (src/mad2 — mostly true, a few leaks)
model     never knows about canvases         (src/models — new; was empty)
```

Today the "model" layer is empty and 3310 facts have leaked up into the platform
(`mad2.c`) and out into the shells (`web/main.c`, `tools/boot_trace.c`). This work
introduces a **runtime `ModelProfile`** that carries every per-model fact, so the
platform and shells read it instead of hardcoding 3310.

## What actually varies per model (the audit)

| Category | Varies? | Examples |
|---|---|---|
| ARM7 core | no | mGBA, model-agnostic already |
| MAD2 ASIC register map (`0x20000` window) | no¹ | CTSI/timers/IRQ, GENSIO/CCONT, flash CFI, SIMI, MBUS — shared by all MAD2 DCT3 |
| **Memory map** | **yes** | flash size (2 MB 3310/8850 vs ~4 MB 7110/3330) → **EEPROM/NVRAM base shifts late**; device-owned flash range |
| **LCD** | **yes** | PCD8544 84×48 (3310) vs 96×65 candybar/Navi (8850/7110); controller + geometry + I/O offsets |
| **Keypad matrix** | **yes** | row/col → keycode mapping |
| **Battery / ADC defaults** | **yes** | BSI pack code, temp window |
| **IRQ source count** | **yes** | 3310 = 8-source; 8850 noted as needing a 16-source model |
| **Firmware addresses** | **yes (per build)** | verdict byte, DSP flags/mailbox/queue, COBBA, `get_string`, FAID sum, dispatcher/msg-send PCs |
| **Boot/HLE config** | **yes** | spike pokes + timing, skip-seclock default, power-release timing |

¹ ASIC *variants* exist (e.g. IRQ width); carried as `PlatformQuirks`, not a fork.

> Out of scope here: DCT4 (5510 = UPP/UEMK, a different ASIC entirely). The seam is
> designed so a future `Platform` abstraction *could* sit beside MAD2, but we build
> only MAD2 DCT3 now.

## Firmware addresses: signature-first (NokiX `LOCATE`/`SMARTFIND`)

The firmware-build-dependent addresses (verdict, DSP flags, `get_string`, FAID, …)
move when the firmware is recompiled, so hardcoding them per (model, build) is
brittle. The robust approach — already prototyped in `main.c`'s `resolve_spike()`
and `sig_find()` — is **masked byte-signature search**: match a function body with
its operands/addresses wildcarded, locating it regardless of where a build moved
it. This mirrors NokiX's `find`/`findfunc`/`smartfind` macros
(`ref/NokiX-scripts/.../macros/`), which are a deep, ready-made source of portable
signatures (`LOCATE_SPECIAL.rx`, `SMARTFIND.rx`, …).

Two signature flavours cover every case we have:

- **`SIG_CODE`** — the match *is* the wanted code address (e.g. a handler entry).
- **`SIG_LITERAL`** — the match locates an instruction that loads a RAM base
  literal; the wanted RAM address = `extracted_literal + addend`. (This is exactly
  how `resolve_spike` finds the SIM-gate byte: locate `sim_func`, read its
  `ldr r1,=<base>`, add 29.)

Each firmware address is a **signature with a constant fallback**: if the signature
misses, use the per-profile constant (today's hardcoded value). So 3310 behaviour is
unchanged on day one, and new models add signatures incrementally.

## The profile (shape)

See `src/models/model.h` for the authoritative definition. Sketch:

```
ModelProfile
  name, description, ident          // "3310"; product-code/version match for autodetect
  MemMap        mem                  // flash/ram/io/mmio/eeprom bases + sizes
  LcdSpec       lcd                  // controller, w/h/banks, data/cmd I/O offsets
  KeypadSpec    keypad               // matrix → keycode map
  BatterySpec   battery             // ADC reset defaults (vbatt/bsi/temp/charger)
  PlatformQuirks quirks             // irq_sources (8/16), ASIC revision flags
  FwAddrs       fw                   // constant fallbacks for all firmware addresses
  SigResolve[]  sigs                 // signature overrides (signature-first), table-driven
  BootConfig    boot                 // spike pokes + timing, skip-seclock default
```

Resolution at boot: `model_resolve(profile, flash, len, &fw_out)` copies the profile
constants into `fw_out`, then runs each `SigResolve` against the loaded flash,
overwriting the field it targets on a hit. The platform (`mad2`) and shells then read
`fw_out` + the static profile — no literals.

### LCD framebuffer sizing

`mad2.h` currently has a static `fb[LCD_BANKS][LCD_W]` (6×84). Going runtime-variable
without heap churn: size the static array to the **max** geometry
(`LCD_MAX_W`/`LCD_MAX_BANKS`, e.g. 96×9) and use `profile->lcd.{width,banks}` for the
active region. The render/scan paths clamp to the active geometry.

## Selection

1. **Explicit** — web model selector / a CLI/env name → `model_by_name("8850")`.
2. **Autodetect** — `model_detect(flash, len)` matches `FwIdent` (product-code or
   version string in the dump, plus flash size) and returns the best profile. The
   web firmware-picker uses this so dropping in a dump "just works".
3. **Fallback** — default to 3310 if nothing matches (today's behaviour).

The existing web model dropdown is currently cosmetic (nav-key layout only); it gets
wired to real selection.

## Phasing & validation

| Step | Deliverable | Gate |
|---|---|---|
| 1 | This doc + `model.h` interface | review |
| 2 | `model.c` resolver + registry | `make test` 41/41 |
| 3 | 3310 profile from current constants | — |
| 4 | `mad2` consumes `ModelProfile` (geometry, fw addresses) | `make test` 41/41 |
| 5 | Wire shells (main.c, boot_trace.c) to select + resolve | **3310 boot byte-identical before/after** |
| 6 | 8850 stub (2 MB candybar, 16-src IRQ) | loads + resolves; full boot not required |
| 7 | 7110 stub (4 MB memmap, late EEPROM, 96×65) | proves the 4 MB / LCD-geometry path |

**The hard constraint:** steps 1–5 must not change 3310 behaviour. Verified by
`make test` (41/41) and a before/after native `boot_trace` comparison of the v5.79
boot (dispatcher progression + framebuffer hash at fixed step counts).

## Test matrix (firmware on hand)

| Model | Flash dump | Size | Proves |
|---|---|---|---|
| 3310 | `flash/My 3310 NR1 v5.79.fls` | 2 MB | baseline, must stay identical |
| 8850 | `firmware/FuBu8850 v5.31 (PPM C).fls` | 2 MB | candybar LCD, 16-source IRQ variant |
| 7110 | `flash/7110 v5.01 Converted MCU+PPM D.fls` | ~4 MB | 4 MB memmap, late EEPROM, 96×65 LCD |
