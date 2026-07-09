# DCT3 emulator — architecture & fidelity (what's real vs HLE)

How the emulator is built, and an honest map of where it's silicon-accurate vs where it
takes high-level shortcuts. Read this first to understand *what kind* of emulator this is.

## Shape

A **from-the-silicon, full-system emulator** of a Nokia DCT3 phone (MAD2 ASIC = ARM7 MCU +
TI C54x DSP), running the **stock Nokia flash firmware**, compiled to **WASM and executing
in the browser**, all the way to a **drawn, interactive UI** on the PCD8544 LCD.

```
  stock firmware (.fls)  ──runs on──>  [ ARM7 core ] + [ MAD2 hardware model ]  ──draws──> canvas
                                        (mGBA, BE)       (ours, from scratch)
```

## The CPU core — real hardware accuracy (not HLE, not poked)

The ARM7 is **mGBA's ARM7TDMI core** (`third_party/mgba-arm/`: `isa-arm.c`, `isa-thumb.c`,
the decoders, `arm.c`) — a genuine instruction-accurate ARMv4T interpreter (full ARM+Thumb
semantics, barrel shifter, multiply timing, `LDM`/`STM`, banked registers, CPSR/SPSR,
exception/IRQ/FIQ vectors, PC+8/PC+4 prefetch). mGBA is one of the most accurate GBA
emulators in existence. This is why stock firmware stays in lockstep for **billions** of
instructions (verified stable past **3.7 billion cycles ≈ 1.37 B instructions**).

**We forked it to big-endian** (GBA is little-endian; DCT3 runs the ARM7 in BE). The flash
stays byte-exact; the core was adapted.

**Caveat — cycle *timing* is approximate, semantics are exact.** Per-access cycle counts
come from mGBA's memory-timing model (tuned for GBA waitstates), not measured DCT3 silicon.
We calibrated the clock (13 MHz) and the effective ~2.7 cycles/instruction empirically so the
firmware's *relative* timing (timers ↔ IRQs ↔ scheduler) holds. So: **instruction semantics
= silicon-accurate; absolute cycle timing = faithful-relative, approximate-absolute.**

## The boot ROM — HLE'd to a single instruction

On real silicon the ARM7 resets into a **mask ROM inside the MAD2** (clock/power/memory init,
MBUS/FBUS flashing-mode check, then a branch to flash). That ROM is **on-chip — it is NOT in
the `.fls` dump**, so we don't have it and can't run it. Our entire boot ROM is
(`dct3_boot_at`, `src/core/dct3_core.c`):

```c
dct3_write32(core, 0, 0xEA000000u | branch_off);  // plant `B 0x200040` at the reset vector
dct3_core_reset(core);                            // ... then reset
```

i.e. **plant one ARM branch to the flash firmware entry (`BOOT_ENTRY = 0x200040`) and reset.**
Zeroed RAM = fresh power-on; `mad2_init` supplies the MAD2 reset-default register state. We
emulate the boot ROM's *exit state*, not its steps (like an HLE BIOS). Everything observed —
self-tests, DSP handshake, PMM, UI — is the **real flash firmware bootstrapping itself** from
flash entry; it re-initialises essentially all its own hardware on the way up, which is why a
one-instruction stub suffices. (Any state the real boot ROM leaves that the firmware *assumes*
rather than re-inits is a candidate source for the few remaining edge pokes.)

## The MAD2 hardware model — a spectrum (faithful → HLE)

Built from scratch (`src/mad2/mad2.c`). The whole "faithful vs poke" theme of the project is
about moving boxes from the right column to the left:

**Faithful hardware models:** CCONT (power/RTC chip, GENSIO bus), PCD8544 LCD, keypad matrix,
Timer0/Timer1 + FIQ/IRQ controller, flash/CFI + EEPROM/NVRAM (persisted), SIM/SIMI (ATR, T=0,
CHV1/PIN), buzzer/vibra, charger + battery ADC, and — as of the DSP milestone — the **DSP
mailbox + self-test handshake**.

**Still HLE / shortcut (shrinking):** the boot ROM (above); the SIM-manager scheduling and
FAID checks (the `Bypass SIM` / `FAID Pass` toggles); **RF/HAGAR + the GSM stack + COBBA audio
path are unmodeled.**

## Counters — no practical overflow

`g_step` (`int64_t`), `g_cycles64` / `rtc_mono` (`uint64_t`) → ~45,000 years at 13 MHz. The
core's `cpu->cycles` is 32-bit but **auto-rebased every `0x20000000` (~41 s)**; the 64-bit
accumulators undo each rebase. Only the JS-reported counts (doubles) lose exact precision past
2^53 (~22 yr), cosmetically. **Leave it running indefinitely.**

## Web layer

`src/web/main.c` (the WASM exports + boot + real-time cycle-paced run loop), `web/main.js`
(UI wiring, LCD canvas, audio, diagnostics), `web/index.html` + `web/style.css` (3-panel UI,
phone-model selector, line keys). Default firmware: `flash/My 3310 NR1 v5.79.fls` (Makefile
`WEB_FW`). Boot defaults are now organic (no verdict spike) thanks to the DSP responder.
