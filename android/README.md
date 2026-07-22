# Android front-end

A native Android app that JNI-wraps this repo's C core directly (no browser, no wasm)
for real Android handsets that are already shaped like an old Nokia — e.g. the
[HMD Terra M](https://www.hmd.com/) ("Cupra"), which is what this was built and tested
against. Because the target hardware already looks like a feature phone, the app draws
**no phone-shell chrome** — just the emulated LCD, scaled up and centered on a black
field, and the phone's own physical keys drive input directly.

Currently **Nokia 3410 only**. See "Scope" below for why, and what it'd take to widen
that.

## Build & run

Standard Android Studio project — open `android/` as the project root, or from the
command line:

```bash
cd android
./gradlew installDebug
```

The native build (`app/src/main/cpp/CMakeLists.txt`) compiles this repo's own
`src/core`, `src/mad2`, `src/harness`, `src/models` (3410 profile only) and
`third_party/mgba-arm` sources directly — no copying/vendoring, no external checkout.
It resolves the repo root by relative path from its own location, so this directory
must stay at `android/app/src/main/cpp/` relative to the repo root.

Ships `armeabi-v7a` only by default (`app/build.gradle.kts`'s `ndk.abiFilters`) to
match the 32-bit-only test device; widen that if you're targeting a 64-bit handset.

### Firmware

Same policy as the rest of the repo: **bring your own** `.fls`, nothing copyrighted is
bundled. At first launch the app looks for a raw NHM-2 (3410) image in its private
external-files dir and, if it isn't there, prompts with a plain Storage Access
Framework file picker — no storage permission needed. Or push one directly:

```bash
adb push "Nokia 3410 NHM-2 v5.46.fls" \
  /sdcard/Android/data/<applicationId>/files/firmware.fls
```

## Scope: why 3410-only

This app adds exactly one change to the shared core, in `src/models/model.c`: a new
`DCT3_MODEL_3410_ONLY` build-mode macro that mirrors the existing
`DCT3_MODEL_3310_ONLY`, so `model_default()` resolves to a single-model registry. That
keeps the APK from linking all 26 phone profiles plus the ~635 KB C54x DSP co-simulator
(only needed by the serial-bus profiles — 5110/6110/3210/7110; the 3410 uses the HLE
DSP already). Everything else the app links is unmodified upstream source.

Widening to other models is mostly a matter of adding another `DCT3_MODEL_<N>_ONLY`
mode (or a runtime model switch, which the registry doesn't currently support) and
wiring up that model's keypad layout in `MainActivity.mapKey()`.

## Architecture

- `app/src/main/cpp/dct3_engine.c` / `.h` — the Android driver loop, deliberately kept
  as plain C with no `jni.h` dependency (host-gcc-testable in isolation), mirroring
  `src/web/main.c`'s boot/step/key-inject shape. `dct3_jni.c` is a thin JNI marshaling
  layer on top of it — all logic lives in the plain-C layer.
- Key input is injected as **real matrix down/up edges** (`dct3and_key_event`), unlike
  the web front-end's driver, which runs an artificial auto-release timer to work
  around browser click events not carrying real press/release timing. Android
  `KeyEvent`s already have real timing, so this app doesn't need that workaround.
- A single `pthread_mutex_t` serializes the background stepping thread against
  UI-thread key events, taken once per `runCycles()` batch (not per instruction).
- Kotlin (`MainActivity.kt`) drives real-time pacing itself: a 30 Hz tick loop calls
  `runCycles(ArmHz / 30)` per tick and self-corrects instead of accumulating sleep
  debt if a tick runs long.
- Audio reuses the core's existing `emu_audio_render()` HLE mixer (buzzer + DSP-HLE
  tone) unmodified; `dct3_engine.c` only adds a small ring buffer between it and a
  `pcm_sink` callback, which `MainActivity.kt` drains into an `AudioTrack` in
  `MODE_STREAM`.

## Device notes (HMD Terra M / "Cupra")

This was the only hardware this frontend has been tested against, and a couple of
things are specific enough to it that they're worth flagging for anyone building
against different hardware:

- **The two physical buttons below the display (Menu / Back) send non-standard raw
  `KeyEvent` codes** — `414` for Menu, `411` for Back — with no named
  `KeyEvent.KEYCODE_*` constant. `adb shell input keyevent KEYCODE_MENU` (symbolic
  injection) reaches the app fine and proves nothing about what the physical button
  itself sends — only reading `KeyEvent.getKeyCode()` from a real press caught this.
  `MainActivity.mapKey()` matches these two raw ints directly, alongside (not instead
  of) the named constants, on the assumption this is a device-specific keylayout
  remap that won't generalize — worth re-checking with `onKeyDown` logging on any
  other hardware before trusting the raw values.
- DCT3 has no literal "back" key — `KK_END` (red hang-up) always jumps straight to
  standby. The real "step back one menu level" action is the contextual right soft key
  (`SOFT2`), which is what the physical Back button is wired to here, not `END`.

## Performance

This is CPU-bound on the test device's SoC (Qualcomm QCM2290, weak in-order
armeabi-v7a cores). Measured on-device, cycle-accurate against the emulated 13 MHz ARM
clock:

| Build flags | Throughput |
|---|---|
| `-O2` | ~34–36% of real-time |
| `-O2 -flto=thin` | no measurable change |
| `-O3 -flto` (full LTO, current default) | ~41–44% of real-time |

CPU governor was confirmed pinned at max frequency throughout (not thermal
throttling). This repo's Makefile notes `-flto` roughly doubling throughput and `-O3`
*regressing* it from I-cache pressure on the native x86 build — neither claim
transferred to this ARM target as-is, which is why both were re-measured empirically
here rather than assumed. Even with `-O3 -flto`, this is still well under real-time on
this specific device; it looks like a genuine single-core decode/dispatch throughput
ceiling rather than something another compiler flag closes, and getting further would
likely mean a fundamentally different execution strategy (e.g. a JIT) rather than more
tuning — out of scope for this change.

## Known gotcha if you're adapting this

`src/web/main.c` deliberately forces `sim_present = 0` for anonymous web visitors.
Copying that line verbatim into a real-device driver (an early mistake here) means the
phone shows "Insert SIM card" forever and is unusable — `mad2_init()`'s real default
(`sim_present = 1`, synthetic ISO-7816/GSM-11.11 SIM, no PIN) is what a real-device
build should leave in place.
