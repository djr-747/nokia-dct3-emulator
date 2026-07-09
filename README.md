# Nokia DCT3 Emulator

An open-source emulator and architectural-analysis toolkit for Nokia DCT3-era phones —
the 3310, 3410, 5210, 8210, 8250, 8850 and their siblings. It boots **real firmware** in
the browser and in a native SDL GUI, faithfully modelling the MAD2/MAD1 platform ASIC, the
CCONT power controller, the TMS320C54x DSP, the PCD8544/SED-class LCD, the keypad matrix,
the SIM, and the internal + external EEPROM.

This is a **historical software-preservation** project: it documents and re-hosts the
behaviour of obsolete, ~25-year-old hardware for offline educational analysis. The design
rule throughout is **faithfulness** — behaviour is reproduced by modelling the silicon, not
by patching firmware to "look right." No forced self-test verdicts, no trampolines, no
poking the boot-lock check.

---

## Firmware — bring your own

> This repository contains **no** copyrighted firmware, flash dumps, or EEPROM images, and a
> pre-commit hook + CI check keep it that way. You must supply your own legitimately-obtained
> `.fls` image.

Drop a flash image in and point a target at it:

```bash
# Native
./build/dct3_boot_trace_gui "Nokia 3310 NR1 v5.79.fls"

# Web — either rebuild the module with your image baked in:
make all WEB_FW="Nokia 3310 NR1 v5.79.fls"
# ...or leave the module firmware-free and load an image from the web UI at runtime.
```

### Bootability by model

How far each model gets today. Legend: ✅ standby (clock + idle screen) · 🟡 partial (into
the MMI, one barrier left) · 🔴 scaffold (profile exists, stalls early). Model detection and
version are read from the flash header — nothing is hardcoded per image.

| Model | Code / ver | Status | Notes |
|---|---|---|---|
| **3310** | NHM-5 v5.79 | ✅ standby | reference baseline (kept byte-identical by `make guard`) |
| **3410** | NHM-2 v5.46 | ✅ standby | minor set-time clock-tick gap tracked separately |
| **5210** | NSM-5 v5.40 | ✅ standby | needs a repaired-EEPROM image (blank-EEPROM regen) |
| **8210** | NSM-3 v5.31 | ✅ standby | needs a one-byte stale-checksum EEPROM fix |
| **8250** | NSM-3D v6.02 | ✅ standby | stock image, EEPROM self-consistent |
| **8850** | NSM-2 v5.31 | ✅ standby | NSM Family-A reference |
| **8855** | NSM-4 v5.13 | ✅ standby | needs a repaired-EEPROM image |
| **8290 / 8890** | NSB v5.22 / v12.16 | 🟡 partial | reach Contact Service; DSP-upload handshake RE pending |
| **3330 / 3350** | NHM v4.50 / v5.22 | 🔴 scaffold | Contact Service; verdict/dsp-upload RE outstanding |
| **5510 / 7110** | NPM / NSE | 🔴 scaffold | early power-off / wild-PC; map + DSP scratch unresolved |
| **5110** | NSE-1 v5.30 | 🟡 partial | **MAD1 / external-EEPROM**, native only (the web build doesn't link the C54x core). Boots to the MMI: the **C54x DSP co-sim** passes Contact Service to the "Security code" (FAID) screen faithfully; the **HLE DSP** reaches PIN/standby. See `docs/5110-DSP-CANONICAL-STATE.md`. |
| **8810** | NSE-6 v6.02 | 🔴 scaffold | same MAD1 external-EEPROM layout as the 5110; not yet brought up |

Seven models boot to standby today. Per-model detail lives in [`docs/MODELS.md`](docs/MODELS.md).

---

## Features

**Emulation core**
- Vendored **mGBA ARM** interpreter (big-endian-corrected for DCT3), driving the real firmware
  instruction-for-instruction — no HLE of the application layer.
- **MAD2 platform model** (`src/mad2/`): the ASIC bus, interrupt/FIQ controller, timers, RTC,
  Intel/Sharp CFI flash FSM, internal + I²C external EEPROM, and the MBUS/FBUS USART.
- **CCONT** power/RTC/ADC controller with the interrupt→event measurement protocol.
- **20 model profiles** (`src/models/`) selecting memory map, LCD controller, battery/ADC
  windows, keypad matrix, and DSP variant at runtime from the flash header.

**DSP**
- Faithful **TMS320C54x co-simulation** via a vendored qemu-calypso-lineage interpreter
  (`third_party/c54x/`), running the recovered DSP image with demand-paged overlay uploads —
  the MCU↔DSP HPI mailbox, boot handshake, self-test and keep-alive modelled as on real silicon.
- High-level DSP responders split by ROM revision for models where the co-sim isn't wired.

**Peripherals & I/O**
- **LCD**: PCD8544-class framebuffer, rendered to the browser canvas, the SDL window, or PNG.
- **Keypad**: matrix scan with IRQ-driven debounce and auto-repeat modelling.
- **Audio**: unified buzzer→PCM mixer with sample-accurate tone onset (organic keypad beeps
  synthesised by the DSP tone path, not canned samples).
- **SIM**: synthetic card with genuine ATR, ISO-7816 PPS, and GSM 11.11 T=0 (SELECT / GET_RESPONSE
  / READ) plus A3/A8 authentication. An optional **real-SIM bridge** (`tools/simprobe.c` +
  `tools/sim_bridge.c`, driving the ESP32 thin reader in `esp32/simbridge/`) lets the emulator talk
  to a physical SIM card over serial — see `docs/sim-bridge-protocol.md`.
- **Battery, charger, LEDs, vibra** exposed to both front-ends.

**Front-ends**
- **Web** (`web/`): cycle-paced WASM build, per-model phone-shell UI, runs in any browser.
- **Native SDL GUI**: desktop shell with the phone photo, live keypad, and LCD.
- **Headless harnesses**: `dct3_boot_trace` (instrumented native run with an always-on
  post-mortem that labels every halt PC, message and reset reason) and `tools/nav.mjs`
  (deterministic browser-paced harness that plays a key script over emulated time and renders
  the framebuffer to PNG).

**Analysis toolchain**
- **Symbol store** (`tools/symbols/`): per-firmware-build address/name/message database with a
  lookup CLI and a scratch→promote workflow.
- **`disfw`**: an annotated disassembler that reads the symbol store and decodes effective
  addresses, cmp-gates, jump tables, and message IDs inline.
- **NokiX-compatible script runner** (`tools/nokix/`) for applying diagnostic overlays and
  auto-locating symbols.

---

## Quick start

```bash
# Web  (committed WASM core runs as-is; rebuilding needs emscripten)
make all && make serve            # serves web/ on http://localhost:8000

# Native SDL GUI   (needs SDL2)
make gui
./build/dct3_boot_trace_gui "your-firmware.fls"

# Headless boot observation
make trace
./build/dct3_boot_trace "your-firmware.fls" 250000000

# Annotated disassembler
make disfw
./build/disfw "your-firmware.fls" 0x002EEBAE 12

# Real-SIM bridge (optional; needs the ESP32 reader from esp32/simbridge/)
make simprobe
./build/simprobe /dev/ttyUSB0 atr

# Tests
make test        # native regression suite (68 checks + dbgcon)
make guard       # byte-identical boot guard (needs your firmware images)
```

---

## Acknowledgements

This project stands on two decades of open Nokia DCT3 reverse-engineering. It would not exist
without the people and projects that mapped this hardware first:

- **blacksphere** — the collective of hackers who reverse-engineered the DCT3 debug interfaces,
  firmware and hardware, and crucially the **DSP↔CPU interface** (the GSM L1/L2 boundary) that our
  DSP co-simulation models. The foundational work that made all of this tractable.
- **g3gg0** (Georg Hofstetter) — **MADos**, the alternative open-source DCT3 firmware/OS
  (LGPL-2.1). Ground-truth for the DSP block layout and the MCU↔DSP mailbox; our DSP blocks are
  byte-identical to its `dspblocks/`.
- **Vitaly Nevzorov** (original author, 2002) and **[Yak]** (2005) — **NokiX**, the DCT3
  firmware-modification SDK: it patches an original flash image via Rexx scripts and can **compile
  new C functionality and inject it into the firmware**, emitting a modified flash. Our
  `tools/nokix/` script runner and symbol auto-locator descend from its LOCATE scripts.
  (NokiX modifies firmware; it is not a flasher.)
- **AlexD** — the **5110 DSP ROM4 dump**. The recovered TMS320C54x DSP image that our C54x
  co-simulation actually executes traces back to this work.
- **osmocom-bb** — the open Calypso DSP API, the Rosetta stone for the MCU↔DSP mailbox.
- **gnokii** / **Gammu** (and `dct3trac`) — the FBUS/MBUS service-protocol RE and the NHM-5 trace
  dictionary that names our broker events.

Vendored third-party code (see `third_party/` for licenses):

- **mGBA** ARM core — Vicki Pfau (endrift), adapted to DCT3's big-endian ARM.
- **qemu-calypso / bbaranoff** C54x — the TMS320C54x interpreter our DSP co-sim is built on (GPL-2).
- **stb_image** — Sean Barrett.

Any errors or misattributions here are ours, not theirs — corrections welcome.

---

## Licensing

Released under the **GNU General Public License v2** (see `LICENSE`). The native builds
statically link the GPL-2 C54x DSP interpreter under `third_party/c54x/`, which makes GPL-2 the
governing license for the combined native binaries.
