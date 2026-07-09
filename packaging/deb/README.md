# Nokia 3310 Emulator

This is a native Linux build that boots a real Nokia 3310 firmware image to a
live on-screen phone. Free software under the GNU GPL v2.

## 1. Install

```sh
sudo apt install ./dct3-emu_*.deb
```

That pulls in the only dependency (`libsdl2-2.0-0`). To remove later:
`sudo apt remove dct3-emu`.

## 2. Add firmware (once)

Firmware is **not** included — it's copyrighted Nokia code, so you supply your
own 3310 `.fls` dump. Either:

- pass it on the command line: `dct3-emu /path/to/your-3310.fls`, **or**
- drop a `*.fls` into `~/.local/share/dct3-emu/` and just run `dct3-emu`.

### The image must be COMPLETE — MCU + PPM + PMM

A working image contains all three flash regions:

- **MCU** — the firmware code,
- **PPM** — the language pack,
- **PMM (EEPROM)** — the per-phone NVRAM: calibration, IMEI, settings.

**Use a Full Backup.** The safest source is a **Full Factory / Full Backup**
`.fls` read from the phone (or a service tool's "Full Backup"), which includes
the PMM/EEPROM. A **2 MB** `NHM-5` image is the full 3310 flash.

⚠️ **A code-only dump will not boot.** Many shared `.fls` files are **MCU+PPM
only** (~1.81 MB / `0x1D0000`) with the EEPROM stripped out — they have no
IMEI/calibration and will stall or show a blank screen. If your file is under
2 MB, it's almost certainly missing the PMM; get a Full Backup instead.

## 3. Run

From the app menu (**Nokia 3310 Emulator**) or the terminal:

```sh
dct3-emu
```

The phone boots in a few seconds. A factory image lands on the **"Security
code:"** lock screen — that's a normal, fully-booted phone asking for its code.

## Controls

| Input | Action |
|-------|--------|
| On-screen keypad / number keys | dial / menu |
| Arrow keys, Enter | navigation / select |
| **PWR button (hold 30 s)** | hard power-cycle a hung phone |
| **R** | quick warm reset |
| **Space** | pause / resume |
| **S** | save a screenshot (PNG) of the LCD |
| **Esc** / close window | quit |

Earpiece audio (keypad tones etc.) plays through your default sound device.

## Reporting a bug

Please include:

- what you did (which keys, in order),
- a **screenshot** (press **S**),
- the firmware version string (the `V xx.xx … NHM-5` header of your `.fls`).

## Notes / limitations

- **3310 only.** Other DCT3 models and the DSP co-simulation are removed from
  this build.
- The window is wall-clock paced, so it runs at roughly real phone speed.
- Needs a graphical session (X11 or Wayland) and a working audio device.
