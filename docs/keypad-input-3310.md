# Keypad input + tap timing (DCT3 3310)

How DOM/host key events are delivered to the firmware's keypad scanner, and the
tap-timing model that makes navigation robust.

## Hardware

The keypad is a matrix on the MAD UIF block: rows driven on `0x20028`
(signal) / `0x200A8` (direction, `0xE0` = the "special" scan that reads the
power key), columns read on `0x2002A` (active-low: a 0 bit = pressed). The
firmware scans the matrix (v6.39 scan `0x290A28`, repeat `0x290A64`, no-key
reset `0x290A90`) and is wake-on-keypress via **IRQ0**.

> Gap (from the MADos comparison): mad2 does **not** raise an organic keypad
> IRQ0 on a matrix change — key delivery is driven by the web shim
> (`dct3_web_key`) which raises IRQ0 itself. `IO_KEY_IRQC (0x6B)` column→IRQ
> enable is ignored (acceptable HLE). See §7 of the gap analysis.

## The tap-timing model (`dct3_web_key`)

**Problem (a reported bug):** "if the numeric buttons come too fast after the
first menu hit, the phone goes back to home." Two failure modes:
- keys arriving **too fast** after the first Menu press → phone **stuck on the
  home screen** (the next press stomped the previous one mid-hold, so the press
  was lost);
- keys arriving **too slow** → the long fixed hold read as a **long-press**,
  whose exit/repeat semantics **reverted to home**.

**Root cause:** the old `dct3_web_key` drove the matrix immediately and held it
for a fixed ~5M-instruction window, but JS delivers presses at **wall-clock**
pace, which is decoupled from emulated time. So consecutive taps overlapped or
mis-timed. The working window was tiny — the idle→menu transition alone takes
~6.6M instructions to register, vs the 5M auto-release.

**Fix (`src/web/main.c`):** `dct3_web_key` now **enqueues** taps into a small
ring (`KEY_Q_N`); a state machine in the run loop plays each tap **atomically
over emulated time**:

```
clean release of previous key + settle (KEY_SETTLE_INSNS)
  → key-down + IRQ0
  → hold past the firmware repeat-grace (g_key_hold_insns)
  → clean key-up + IRQ0
  → settle
  → next queued tap
```

The tap result is now **independent of how fast/slow the JS calls arrive**: a
0-gap burst of Menu→6→2→1 reliably reaches the language screen (`disp=184`) and
commits NVRAM (verified `eepromWrites` grows), where before it stuck on home.

**Faithful idle-timeout preserved:** genuinely long gaps (15M+ instructions
between keys) still let the menu auto-close (~32M instructions of inactivity) —
that is correct firmware behavior (a real 3310 returns to idle after ~2.5 s).

`dct3_web_key_raw` (manual up/down edges) and the `oneshot` mode are preserved
for tests/harnesses that need raw control.

## History

This supersedes the session-10/12 fixed-`KEY_HOLD_INSNS` window approach
(which decoupled the emulated hold from wall-clock but still used a single fixed
window). The atomic tap-queue is the robust replacement.

## Harness rule

When writing nav/repro harnesses, **render the framebuffer before each key
press** (verify the screen, don't press blind). Reference harnesses:
`tools/menu_roll.mjs`, `tools/snake_play.mjs`.
