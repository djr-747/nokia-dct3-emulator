# Nokia DCT3 Emulator

An open-source emulator and architectural-analysis toolkit for Nokia DCT3-era phones —
the 3310, 3210, 3410, 5210, 6210, 6250, 7110, 8210, 8250, 8810, 8850 and their siblings. It boots **real firmware** in
the browser and in a native SDL GUI, faithfully modelling the MAD2 platform ASIC, the
CCONT power controller, the TMS320C54x DSP, the PCD8544/SED-class LCD, the keypad matrix,
the SIM, and the internal + external EEPROM.

Beyond the phone UI:

- GSM registration on a synthetic network; voice calls; SMS send and receive.
- WAP browsing over a circuit-switched-data bearer, with settings delivered by binary SMS.
- Offline provisioning of the factory identity records (IMEI, FAID, SIMlock, security code) into
  the emulated EEPROM.
- An MBUS service bus, usable by period Nokia service tools over a null-modem COM port.

A historical software-preservation project: it documents and re-hosts the behaviour of obsolete,
~25-year-old hardware for offline educational analysis.

**Implementation constraint.** Behaviour is modelled at the silicon level; firmware is not
patched. There are no forced self-test verdicts and no trampolines. Every write into MCU RAM is a
mailbox write: the DSP↔MCU message ring, its head and tail pointers, the COBBA staging cell, the
codeblock request/reply cells. Provisioned identity records are decoded and judged by firmware
code, and on the 5110 by the real C54x mask ROM running the DCT3 security cipher.

**Known deviation.** `EF_LOCI` is cleared from the SIM on every boot. ROM-4 implements
registration only as a full Location Update, so a card holding a valid location camps without
registering. Contacts and messages are unaffected. `SWSIM_KEEP_LOCI=1` disables the clear. This
may account for the models below that reach standby without registering. Everything else
outstanding is under [Open items](#open-items).

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

### Bootability and radio capability by model

How far each model gets today. Model and version are read from the flash header, so nothing is
hardcoded per image.

**Boot**: ✅ reaches a usable screen (standby, or a normal interactive prompt such as the factory
Security-code entry) · 🟡 reaches Contact Service, or a lock it can't clear faithfully · 🔴 stalls
early.

**Net / Calls / SMS / WAP**: ✅ works · 🟡 partial · ❓ untested · **—** not applicable. Not
applicable covers two cases: the handset's own firmware has no such feature (only models with a WAP
browser can browse), or its DSP engine doesn't implement it. The GSM stack lives in the ROM-6
engine. ROM-4 does camp and registration only, with no calls, SMS or circuit-switched data.

| Model | Code / ver | DSP | Boot | Net | Calls | SMS | WAP | Notes |
|---|---|---|---|---|---|---|---|---|
| **2100** | NAM-2 v5.84 | ROM-6 | 🟡 | ❓ | ❓ | ❓ | ❓ | Security-code screen; rejects the EEPROM-baked code (12345). Nothing past it tested. See #3 |
| **3210** | NSE-8 v6.00 | ROM-4 | ✅ | ✅ | — | — | — | Registered standby. Requires the CCONT persistent-ready bit model and the security-level record provisioned to its erased default; 12345 still accepted if re-enabled. No-SIM boots to "Insert SIM card". Keypad matrix RE'd from the firmware keymap table (wiring differs from the 3310) |
| **3310** | NHM-5 v5.79 | ROM-6 | ✅ | ✅ | ✅ | ✅ | — | Reference baseline, pinned byte-identical by `make guard`. No WAP browser in this firmware |
| **3330** | NHM-6 v4.50 | ROM-6 | ✅ | ✅ | ✅ | ✅ | ✅ | Factory code 12345 → first-boot Time wizard. Verdict/upload cells resolve per-build via the NHM family signatures. WAP over CSD and OTA settings SMS both confirmed |
| **3350** | NHM-9 v5.22 | ROM-6 | ✅ | ✅ | ✅ | ✅ | — | Requires a valid identity: the common library image ships an empty EEPROM partition. Graft a donor NHM 'EEPROM' block first (`tools/graft_eeprom_block.py`, 3330 donor validated). Otherwise as the 3330 |
| **3410** | NHM-2 v5.46 | ROM-6 | ✅ | ✅ | ✅ | ✅ | ✅ | CSD / WAP / OTA reference target. Open: minor set-time clock-tick gap |
| **5110** | NSE-1 v5.30 | ROM-4 | ✅ | 🔴 | — | — | — | Local security passes under the C54x co-sim (`DSP54_COSIM=1`): the mask ROM decodes and judges the provisioned SIMlock, IMEI and FLASH-ID records, reaching standby. Web HLE DSP also reaches standby. Open: no registration |
| **5110i** | NSE-2 v5.53 | ROM-4 | 🟡 | — | — | — | — | 2 MB 5110 refresh. Contact Service on the borrowed 5110 EEPROM; a 5110i-specific record self-test is not provisioned |
| **5130** | NSK-1 v5.30 | ROM-4 | ✅ | 🔴 | — | — | — | 5110 sibling (Xpress-on). Standby. Open: no registration |
| **5190** | NSB-1 v6.71 | ROM-4 | ✅ | 🔴 | — | — | — | Standby. Open: no registration |
| **5210** | NSM-5 v5.40 | ROM-6 | ✅ | ✅ | ✅ | ✅ | ❓ | WAP untested |
| **5510** | NPM-5 v3.50 | ROM-6 | 🔴 | — | — | — | — | Early power-off. Memory map unresolved |
| **6110** | NSE-3 v5.48 | ROM-4 | ✅ | ✅ | — | — | — | Registered standby. Requires the stored security-settings checksum re-derived from the blob's own identity |
| **6130** | NSK-3 v5.61 | ROM-4 | ✅ | 🔴 | — | — | — | 6110 sibling. Standby without registering, unlike the 6110 and 6150 on the same config. Open, cause unknown |
| **6150** | NSM-1 v5.23 | ROM-4 | ✅ | ✅ | — | — | — | 6110 sibling (2 MB, own external-EEPROM blob). Registered standby |
| **6190** | NSB-3 v6.13 | ROM-4 | 🟡 | — | — | — | — | Contact Service. The US NSB build fails an extra judged self-test element. RE pending |
| **6210** | NPE-3 v5.56 | ROM-6 | ✅ | ✅ | ✅ | ✅ | ✅ | Self-test resolves via the DSP block-ack pump setting the upload flag plus a ROM-6 self-test-complete responder posting the firmware's own ack. WAP over CSD confirmed |
| **6250** | NHM-3 v5.00 | ROM-6 | ✅ | ✅ | ✅ | ✅ | ✅ | 6210 sibling. Additionally needs a repair of this library image's inconsistent RF-calibration checksum and the DSP-in-reset status-bit model |
| **7110** | NSE-5 v5.00 | ROM-4 | ✅ | ✅ | — | — | — | SED1565 display; RE'd keypad and Navi roller (mouse-wheel or arrows scroll, press to select). Slow boot, then key-navigable registered standby. Has a WAP browser but binds ROM-4, so no CSD bearer |
| **8210** | NSM-3 v5.31 | ROM-6 | ✅ | ✅ | ✅ | ✅ | — | Security-code screen at boot. Registers with correct EEPROM security settings |
| **8250** | NSM-3D v6.02 | ROM-6 | ✅ | ✅ | ✅ | ✅ | — | As the 8210 |
| **8290** | NSB-7 v5.22 | ROM-6 | 🟡 | — | — | — | — | Contact Service. DSP-upload handshake RE pending |
| **8810** | NSE-6 v6.02 | ROM-4 | ✅ | ✅ | — | — | — | 6110-family slider (2 MB, own external-EEPROM blob). Registered standby, keypad interactive. Slide cover not modelled |
| **8850** | NSM-2 v5.31 | ROM-6 | ✅ | ✅ | ✅ | ✅ | — | NSM Family-A reference. As the 8210 |
| **8855** | NSM-4 v5.13 | ROM-6 | ✅ | ❓ | ❓ | ❓ | ❓ | Standby. Radio untested |
| **8890** | NSB-6 v12.16 | ROM-6 | 🟡 | — | — | — | — | Contact Service (US-band 8850) |

Nineteen of the 26 registered models boot to a usable state, and fifteen of those register on a
synthetic test network with an operator name and signal bars.

---

## Features

**Emulation core**
- Vendored **mGBA ARM** interpreter (big-endian-corrected for DCT3), driving the real firmware
  instruction-for-instruction — no HLE of the application layer.
- **MAD2 platform model** (`src/mad2/`): the ASIC bus, interrupt/FIQ controller, timers, RTC,
  Intel/Sharp CFI flash FSM, internal + I²C external EEPROM, and the MBUS/FBUS USART.
- **CCONT** power/RTC/ADC controller with the interrupt→event measurement protocol.
- **26 model profiles** (`src/models/`) selecting memory map, LCD controller, battery/ADC
  windows, keypad matrix, and DSP variant at runtime from the flash header.

**DSP**
- Faithful **TMS320C54x co-simulation** via a vendored qemu-calypso-lineage interpreter
  (`third_party/c54x/`), running the recovered DSP image with demand-paged overlay uploads. The
  MCU↔DSP HPI mailbox, boot handshake, self-test and keep-alive are modelled as on real silicon.
  On the 5110 the mask ROM runs the DCT3 local-security path, decoding the provisioned SIMlock
  records and reporting its own verdict. Required C54x interpreter fixes: `RSBX`/`SSBX` decode
  (`F6Bx`/`F7Bx`, previously swallowing `0xF0B0` `OR A,-16,A`), `MVDP` routed through `prog_write`
  so mask ROM rejects writes, `dmad` auto-increment under `RPT` for `MVDK`/`MVKD`/`MVDM`/`MVMD`,
  and three ALU/control-flow corrections used by the identity cipher.
- High-level DSP responders split by ROM revision for models where the co-sim isn't wired.

**Peripherals & I/O**
- **LCD**: PCD8544-class framebuffer, rendered to the browser canvas, the SDL window, or PNG.
- **Keypad**: matrix scan with IRQ-driven debounce and auto-repeat modelling.
- **Audio**: a unified mixer over the buzzer and a PCM channel. Keypad tones and DTMF come from
  the DSP: synthesised from the mailbox tone oscillators under the HLE DSP, and delivered as PCM
  samples from the DSP itself under the C54x co-sim.
- **SIM**: a software card driven through the modelled SIMI UART, with a real ATR, ISO-7816 PPS,
  and GSM 11.11 T=0 down to the byte pacing and the procedure-byte handshake. See below.
- **Battery, charger, LEDs, vibra** exposed to both front-ends.
- **Radio**: a synthetic GSM cell driven over the DSP↔MCU mailbox (System Information, paging,
  immediate assignment, LAPDm) carrying registration, voice calls, SMS, and a minimal
  circuit-switched-data bearer.

**GSM network, calls, SMS and WAP**
- **Registration**: every ROM-6 model that boots registers on the synthetic network; ROM-4
  models camp and register too, apart from the 5110 family.
- **Calls and SMS** on the ROM-6 engine, including CP/RP, SMS-DELIVER, `EF_SMS` storage and
  Smart-Messaging ringtones.
- **CSD and WAP**: an RLP peer, PPP, and a WAP gateway that fetches real pages and compiles WML
  1.1 to WBXML so forms work. It isn't keyed to a model: it lives in the shared ROM-6 engine and
  arms on the decoded UDI bearer in the handset's own `SETUP`. The limit is whether a given
  firmware has a WAP browser and a settings receiver. Confirmed on the 3330, 3410, 6210 and 6250.
- **OTA settings provisioning**: WAP settings arrive over binary SMS rather than being typed in.

**Factory provisioning and the DCT3 security codec**
- **Offline provisioning of the I²C external EEPROM** for the ROM-4 phones (3210, 5110, 5110i,
  5130, 5190, 6110, 6130, 6150, 8810). No donor handset or service cable. Writes a coherent IMEI,
  the FLASH-ID/FAID record, both SIMlock parts, the security code and every checksum the boot
  validator compares; firmware reads, decodes and judges them. This is what takes the 3210, 6110,
  6150 and 8810 to a registered standby. The Security-code prompt on those models is an
  identity-sum vs stored-checksum mismatch, not a FAID failure.
- **The security codec is open** (`src/services/dct3_calcul.c`): the record codec for all four
  modes (MSID, FLASH-ID, Lock, IMEI). Tables transcribed, with one published-source error
  corrected: `IMEI_DEF` byte 9 is `0xFF`, recovered by inverse-decoding a real service-tool write.
  `tools/dct3_calcul.py` holds the offline inverse used to mint an MSID.
- **The SIMlock transport pad is derived, not measured**, from the firmware routine that builds
  it (5110 v5.30 `0x25792C`..`0x2579A2`: pairwise 16-bit products, a bit-reversed complement key,
  then a 24-byte XOR). Valid for any pinned identity.
- **A pinned MSID under both HLE engines.** ROM-4 and ROM-6 responders answer the local-security
  identity query (`{74 34}`) from a per-model MSID carrying the correct COBBA signature, selected
  by DSP generation.
- **The emulator issues the MBUS provisioning commands itself**; no external tool required.
  `src/services/identity_provision.c` builds the `B8`/`BA`/`B6` (selector `0x4F`) frames a Windows
  service tool sends, byte-identical in form to captured NokTool traffic, and feeds them into the
  MBUS RX FIFO by the same path the host serial bridge uses. Firmware service handlers decode them
  and write the EEPROM. No EEPROM offsets are poked, so records land wherever the build keeps them.
  Run it from the web UI button or with `REPROVISION=1` (`REPROVISION_AT=` sets the trigger step,
  `REPROVISION=verify` reads the identity back without writing). Requires the phone past startup in
  normal mode. A run with no reply times out rather than half-writing.

**Front-ends**
- **Web** (`web/`): cycle-paced WASM build, per-model phone-shell UI, runs in any browser.
- **Native SDL GUI**: desktop shell with the phone photo, live keypad, and LCD.
- **Android** (`android/`): native JNI build for real feature-phone-shaped Android
  handsets (e.g. the HMD Terra M) — no phone-shell chrome, since the hardware already
  looks like one. Nokia 3410 only for now; see `android/README.md`.
- **A null-modem COM port**: the emulator can expose its MBUS service bus as a real serial port,
  so period Nokia service tools drive the handset unmodified. NokTool, Rolis, Koci, NFREE and the
  DCT3 EEPROM tools run under Wine and talk to the phone over MBUS as they would to hardware on a
  service cable. See below.
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

# Real-SIM bridge (optional; needs an ESP32 flashed with esp32/simbridge/)
make simprobe
./build/simprobe /dev/ttyUSB0 atr

# Tests
make test        # native regression suite (68 checks + dbgcon)
make guard       # byte-identical boot guard (needs your firmware images)
```

---

## Driving the emulator with period service tools

The emulated MBUS service bus can be presented to the host as a serial port. Unmodified Nokia
service software (NokTool, Rolis, Koci, NFREE, the DCT3 EEPROM tools) then connects to the
handset, reads its IMEI, dumps and rewrites its EEPROM, and runs the service commands it would
against hardware on a service cable.

Optional: provisioning does not require it. Use it to drive the phone with period software, or to
cross-check provisioning against a genuine tool.

```bash
MBUSBRIDGE=1 MBUSLOG=1 ./build/dct3_boot_trace_gui "your-firmware.fls"
# [mbus-bridge] tty0tty: emulator=/dev/tnt0  tool COM=/dev/tnt1 (real DTR/RTS/DCD)
```

**Requirements**

- **A null-modem tty pair with real modem control lines.** `MBUSBRIDGE=1` claims the first free
  [tty0tty](https://github.com/freemed/tty0tty) pair: the emulator opens the even end
  (`/dev/tnt0`), the tool connects to the odd end (`/dev/tnt1`). The service tools drive
  DTR/RTS/DSR/DCD/CTS, so real lines are what remove the need for an `LD_PRELOAD` shim. Build the
  module once (`cd tty0tty/module && make`), then `sudo insmod tty0tty.ko && sudo chmod 666
  /dev/tnt*` per boot. `MBUSPORT=<dev>` uses a specific device; `MBUSPTY=1` falls back to a bare
  PTY, which has no modem lines and does need the shim.
- **Wine, with the tool end mapped to a COM port.** Point `COM3` at the odd end, with either
  `wine reg add 'HKLM\Software\Wine\Ports' /v COM3 /d /dev/tnt1 /f` or
  `ln -sf /dev/tnt1 ~/.wine/dosdevices/com3`, then connect the tool on COM3 / MBUS.
- **The right Wine prefix per tool.** NokTool needs a true 32-bit (Win9x) prefix, since it uses
  LDT/win9x facilities WoW64 doesn't provide. The DCT3 EEPROM tool is VB6 and needs `msvbvm60`
  plus a registered `mscomctl.ocx`. Rolis, Koci and NFREE are plain Win32 PE32 apps and run in a
  normal prefix. `faid.exe` is a console tool, so launch it via `wineconsole` for a window.
- **A powered-off start.** `GUIPWROFF=1` boots the GUI with the phone off, which is what the
  tools expect; tap `p` to power on once connected.

`MBUSLOG=1` prints every byte in both directions, which is the quickest way to see whether a
tool's handshake is landing.

---

## Persistence

Non-volatile storage splits into two mutually exclusive hardware families. A handset has one or
the other, never both:

| Family | Store | Holds | Models |
|---|---|---|---|
| External I²C 24Cxx EEPROM | a separate 2 KB device, no in-flash PMM partition | factory identity and RF calibration | 3210, 5110, 5110i, 5130, 5190, 6110, 6130, 6150, 6190, 8810 |
| In-flash PMM partition | a partition inside the `.fls` image; the offset is per-model, not a constant | settings, clock, contacts, SMS, the J2ME game store | everything else |

The I²C family is the ROM-4 models with one exception: the **7110 is ROM-4 but flash-PMM based**,
so it sits in the second group.

The SIM card filesystem (contacts, SMS, `Kc`, `FPLMN`) is a third store, independent of that split
and present on every model.

In the browser all of them persist automatically.

| UI | Store | Key | Default |
|---|---|---|---|
| Phone shell (`web/next/`) | IndexedDB | per model + image | on |
| Diagnostic (`web/`) | localStorage | per image | off |

IndexedDB rather than localStorage: the 3410's NVRAM partition alone is 576 KB, and base64 in
UTF-16 puts several models past the ~5 MB localStorage origin quota. The diagnostic UI defaults
off so boots stay reproducible.

Two behaviours to be aware of:

- **The SIM card is keyed per origin, not per image**, so it follows you across model switches.
  The PMM and I²C EEPROM snapshots are keyed per image.
- **A region is saved only if the firmware programmed it** this session. Auto-save runs every few
  seconds, on tab hide and on unload. DCT3 holds many settings in a working-RAM shadow and flushes
  on a real trigger, so an uncommitted change has not moved the write counter. The Save button
  forces a write.

**Wipe saved settings** (`dct3ResetEeprom()`) clears the NVRAM, EEPROM and SIM snapshots. Uploaded
games and UI preferences are separate and survive. A provisioned identity is an ordinary EEPROM
change and persists like any other write.

Natively nothing persists unless asked; a `boot_trace` run discards its flash writes at exit.

```bash
EE5110SAVE=out.bin ./build/dct3_boot_trace "<fw>.fls" 40000000   # dump the external EEPROM
EE5110=out.bin     ./build/dct3_boot_trace "<fw>.fls" 250000000  # ...and load it back
node tools/nav.mjs "<fw>.fls" --simsave card.bin                 # snapshot the SIM at exit
node tools/nav.mjs "<fw>.fls" --simload card.bin                 # mount it at boot
```

The in-flash PMM partition has no native save knob. Manipulate it offline with `eeprom_tool`:
`inspect`, `extract`, `merge`, `provision`, set the IMEI or security code, list or add J2ME games.
It reads the real store format, selecting the active block by sequence number rather than position
and replaying the append journal that trails the flat image. Readers that skip either step report
stale values.

Working RAM does not persist; a reboot is modelled as a fresh power-on. Neither do the
device-model flags, so the front-ends re-apply SIM presence and PIN settings after a reboot.

---

## The SIM

**Electrical and transport.** The SIMI UART (MAD2 registers `0x36`–`0x3F`) is modelled with the
FIQ6 receive and transmit-empty interrupts and the FIQ7 card-detect edges; bit assignments were
taken from the firmware's FIQ6 handler. Bytes are paced at ~960 characters/second, one per
~1.04 ms (`SIMPACE=0` for instant). Work-waiting time is modelled. PPS is answered by echoing the
frame. T=0 reassembles transmit chunks into whole APDUs and acknowledges case-3 commands with the
procedure byte once per command; acknowledging per chunk desynchronises the link (a 176-byte SMS
record update arrives as twelve chunks) and surfaces later as a spurious "Insert SIM card".

**Two card backends.** The default is a software SIM on the vendored swSIM/swICC stack: a full
GSM 11.11 filesystem across `DF_TELECOM` and three application DFs, with COMP128 for
`RUN GSM ALGORITHM`. Local to this project: the electrical layer, the T=0 transport, and five
patches to the upstream card required for DCT3 SIM init — TERMINAL PROFILE and
READ RECORD / UPDATE RECORD under GSM class byte `A0` (upstream returned `6D00`), a SELECT that
reports record length so the phone can learn ADN and SMS geometry, and SELECT-by-file-id resolving
current-DF-first so `EF_LOCI` does not collapse across the three application DFs. `SWSIM=0`
selects the synthetic EF table instead, 21 files with geometry taken from a captured real card.

**Presence and identity.** A SIM is inserted by default; `SIMABSENT=1` (or `--sim 0` headless)
models an empty tray. The IMSI is selected by DSP engine: ROM-6 uses the reference identity
208-01, ROM-4 the lock-exempt test identity 001-01. The two therefore show different operator
names in the table above. The synthetic cell broadcasts the fitted card's PLMN rather than a fixed
one.

**PIN.** CHV1 is fully modelled on the synthetic backend: enable, disable, change, verify,
unblock, try counters, the always-readable file whitelist, and the file-characteristics bits the
phone reads to determine whether a PIN is required (`SIMPIN`, `SIMPUK`, `SIMPINON`). Upstream
swSIM does not implement PINs, so the PIN controls are inert on the default backend; `SWSIM=0`
exercises that path.

**Writable files.** Contacts (`EF_ADN`), messages (`EF_SMS`), SMS parameters and service centre,
`MSISDN`, and any transparent file. On the software card these are filesystem writes surviving a
reboot and, in the browser, a page reload. On the synthetic backend they go to a copy-on-write RAM
overlay wiped every boot; an unwritten file keeps no overlay, which is what holds the
byte-identical boot guard stable. `EF_LOCI` is the exception noted at the top of this README.

### Talking to a physical SIM card

An optional bridge lets the emulated phone use a real SIM. The tap is at APDU level: the emulator
hands each command to `tools/sim_bridge.c`, which owns the T=0 state machine and drives a thin
ESP32 reader over USB (sketch in `esp32/simbridge/`). The reader clocks the card at 3.5712 MHz and
speaks ISO 7816. Protocol decisions stay on the host, so they can be logged and changed without
reflashing.

Every APDU is shadow-compared: the emulator computes the synthetic answer, runs the real card,
logs `MATCH` or `DIFF`, then feeds one to the firmware. This is the mechanism for aligning the
software card with silicon. A mute card reads as absent rather than falling back to the synthetic
one, so a reader with no card fitted gives "Insert SIM card". `SIMBRIDGE_IMSI=<digits>` rewrites
the IMSI in the real card's response in transit, isolating whether a rejection is IMSI-driven.

`make simprobe` builds a standalone CLI (`ping`, `atr`, `iccid`, `imsi`, `apdu <hex>`) for
hardware bring-up. Limits: only the direct convention is supported, so an inverse-convention
(`TS=0x3F`) card will not talk; and the reference wiring powers the card from the ESP32's 3.3 V
rail through a series resistor — adequate for
bench work, but a proper 3.0 V load switch and an open-drain buffer are wanted for anything
beyond it.

---

## Open items

Filed in the issue tracker:

- Clock does not advance after being set, all models ([#1](../../issues/1)).
- Charger connected: no animating charge bars, and the phone may show "Not Charging"
  ([#2](../../issues/2)).
- 2100: the Security-code screen rejects the EEPROM-baked code 12345 ([#3](../../issues/3)).

Accessory and backlight modelling:

- **Headset accessory detection is incomplete.** Standby shows "Headset" on the 5110, 5130, 5190,
  6110, 6130, 6150, 6210, 6250 and 8810, i.e. the accessory reads as permanently connected.
- **Backlight control lines are not mapped on the 3210, 7110 and 8810.** These models drive the
  keypad and LCD backlights from a single control line; later models split the two for independent
  control, which is what makes the rhythmic-backlight effects possible.

Radio and SIM:

- `EF_LOCI` is cleared on every boot, because ROM-4 implements registration only as a full
  Location Update.
- The 6130 reaches standby without registering, unlike the 6110 and 6150 on the same config.
- The 5110 family (5110, 5130, 5190) reaches standby without registering.
- Radio is untested on the 8855 and the 2100.
- The real-SIM bridge supports only the direct convention; inverse-convention (`TS=0x3F`) cards
  will not talk.

Not modelled, and wanted:

- IrDA ([#7](../../issues/7)).
- A more generic SIM layer, e.g. onomondo or osmo-remsim ([#6](../../issues/6)).
- A real GSM connection via Osmocom rather than the synthetic cell ([#5](../../issues/5)).

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
- **Jumar Macato** ([jmacato](https://github.com/jmacato)) — **direct contributor**: built the
  phone-side DSP engine behind the GSM network emulation (registration, SMS, incoming calls) and
  the event-driven DSP runtime. The **GSM signalling** (RR/MM/CC over LAPDm and the MDI ring
  format), the DSP↔MCU camp/registration handshake our faithful DSP engine models, and the
  consolidated NHM-5 register-and-protocol reference for **RTC** and **SMS** (CP/RP, SMS-DELIVER,
  EF_SMS storage, Smart-Messaging ringtones) are the ground-truth this network layer is built on.
- **bitplane** — [nokia-dct3-re](https://github.com/bitplane/nokia-dct3-re): DCT3 reverse
  engineering and a MAME driver, whose 3210 DSP message-format documentation informed the DSP↔MCU
  modelling and the external-EEPROM FAID provisioning here.

Vendored third-party code (see `third_party/` for licenses):

- **mGBA** ARM core — Vicki Pfau (endrift), adapted to DCT3's big-endian ARM (MPL-2.0).
- **qemu-calypso / bbaranoff** C54x — the TMS320C54x interpreter our DSP co-sim is built on (GPL-2).
- **swSIM / swICC** — software SIM stack, © 2024 Tomasz Lisowski (BSD-3-Clause; `third_party/swsim/`,
  `third_party/swicc/`).
- **stb_image** — Sean Barrett.

Any errors or misattributions here are ours, not theirs — corrections welcome.

---

## Licensing

Released under the **GNU General Public License v2** (see `LICENSE`). The native builds
statically link the GPL-2 C54x DSP interpreter under `third_party/c54x/`, which makes GPL-2 the
governing license for the combined native binaries.
