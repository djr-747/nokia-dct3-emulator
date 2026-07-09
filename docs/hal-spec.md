# Canonical peripheral/host interface (HAL) — spec

**Status:** ACTIVE milestone spec (2026-06-13). Guardrail gate: `make guard`
(three byte-identical boots — 3310 `82610e5e`, 3410 `5a284ea4`, 5110 cosim
`4921792d` — plus the unit suites). **Every step of this work runs the gate
before and after; any drift stops the step.**

## Goal

Every model is a *parts list* behind one canonical interface, so each harness
(web `src/web/main.c`, SDL `tools/gui_sdl.c`, headless `tools/boot_trace.c`,
`tools/nav.mjs`) is a thin adapter and a new model is mostly a profile file.

## The two axes (the core design rule)

Variation lives on exactly two axes; conflating them is how HALs bloat:

1. **Implementation (ops vtable)** — only when *behaviour* differs.
   `DspOps` (`src/models/model.h`) is the proven prototype: HLE mailbox
   (`src/mad2/dsp_default.c`) vs real C54x co-sim
   (`third_party/c54x/mad2_dsp_c54x.c`), dispatched per model from
   `mad2_bus.c`/`mad2_timers.c`.
2. **Configuration (spec struct)** — when only *parameters* differ.
   `LcdSpec`, `KeypadSpec`, `BatterySpec`, `FwAddrs` already carry this.

**Rule of thumb: new struct values are free, new implementations are
expensive.** Design every slot so geometry/colours/maps/windows are data.

### Finding (2026-06-12): the DCT3 family varies far less than a naive HAL assumes

- **LCD = ONE implementation.** All models speak the PCD8544 command set;
  `mad2_lcd.c` has zero controller branching today. Per-model data: geometry
  (84×48 vs 96×65), controller RAM > visible (3410 102×72), `x_mirror`
  (5210, 3410), GENSIO offsets. The `LcdController` enum is vestigial — never
  read; keep as a descriptive tag only, never branch on it. Open item: verify
  the 7110 Navi panel (io offsets TODO in its profile).
- **Genuine implementation splits are a short list:** MAD1 serial transport
  (5110 keypad/CCONT/external-I2C-EEPROM, `quirks.mad1_serial` +
  `mad1_i2c.c`) and HLE-vs-cosim DSP. Everything else is expected to be one
  implementation + per-model spec structs.

## Tracks

| Track | What | Status |
|---|---|---|
| 0 | Guardrails (`make guard`, `tools/check_guardrails.sh`) | ✅ done (38e181a) |
| 1 | C54x consolidation + licensing fence | ✅ done (0f42d1e) |
| 3 | Host interface contract (this doc) + slices | ✅ done (LED colour, PCM/SDL audio, logical keys + capability query, shared fb unpack) |
| 2 | Per-model peripheral behaviour: addressing as data, transport as impl, no quirks | ✅ done (LCD/CCONT/keypad ports as data; BusOps; interrupt fabric API; de-quirked; `mad1` purged) |

Track 3 before Track 2 so each peripheral is moved **once**, directly behind
its final interface.

### Track 1 outcome (decided, do not re-open)

The entire GPL-2 C54x surface — core *and* project-written glue — stays in
`third_party/c54x/` (policy: `third_party/c54x/README.md`). Native dev
binaries only; never in the shipped wasm. `src/` never includes a
`third_party/c54x` header; the single seam is the link-time symbol
`extern const DspOps mad2_dsp_c54x` (`model.h`).

## The host interface (Track 3 contract)

> **Realized as `src/mad2/emu_host.h` (2026-06-13, `84e931b`).** The contract below
> is now an actual header — the single typed surface a harness binds to: inline
> accessors for the observation surface + shared input primitives (`emu_keyline`,
> `emu_key_special`, `emu_key_present`). Web routed through it; gui_sdl adopting
> incrementally (LED glow done, `19a89af`). **Found while building it:** input
> *timing* is harness POLICY, not part of the contract — web enqueues a logical key
> for its auto-release sequencer, the SDL GUI drives real up/down, boot_trace injects
> at raw-matrix level — so the façade shares the key LOOKUP + special-scan primitive,
> and each harness owns down/hold/up. The CONTROL surface (knob registry) is the one
> contract axis still unconsolidated.

The proven interaction model in this codebase is **deterministic polling +
monotonic edge counters** (e.g. `buzz_edges`), not host callbacks: the step
loop stays reentrancy-free, wasm/JS polls per frame, native harnesses read
state at their own cadence. v1 keeps that, with one exception (PCM, below).

### Outputs (emu → harness)

| Channel | State | Edge/event | Notes |
|---|---|---|---|
| LCD | `m->fb` + `LcdSpec` geometry | `lcd_data_writes`/`lcd_cmd_writes` counters as dirty hint | ✅ shared unpack = `mad2_lcd_px()` (DDRAM bit + display-control transform; gui_sdl now uses it — its inline copy had skipped blank/inverse). web JS mirrors it (inherent to the wasm boundary). `x_mirror` is homed at WRITE time (`lcd_data()` stores un-mirrored) |
| LED: LCD backlight | `m->led_lcd` on/off | — | **colour comes from `ModelProfile.led.lcd_rgb`** (see below) |
| LED: keypad | `m->led_kbd` on/off | — | colour from `led.kbd_rgb`; MAD1 note: I/O 0x33 is IM_C, not LEDs |
| Buzzer | `buzz_on/div/vol` | `buzz_edges` + `buzz_div_at_edge` | freq = 13 MHz / div |
| Vibra | `vibra_on/ctrl` | — | |
| PCM audio | `pcm_rate` (=`DCT3_CODEC_HZ` 18 kHz) | **sink callback** `Mad2.pcm_sink(m, ch, sample)` | ✅ cosim feed + **SDL audio binding** (gui_sdl SPSC ring → SDL pull callback @ 18 kHz; `./run 5110 gui` plays the real codec earpiece). Codec rate measured ~18 kHz two ways (zero-cross + 794 samples/44 ms). Browser audio is ALREADY covered by parametric resynthesis (buzzer square + `tone_hz`/`tone_hz2` sines) — the browser runs HLE (no cosim to feed real PCM), so parametric IS the right HLE approach there. `boot_trace` `PCMSINK=1` = the counting demo |
| Reset/fault | `reset_request`, reason byte, post-mortem buf | reset counter | already shared via `src/harness/` |
| Power-off | `power_off` latch | — | |

**LED colour is profile data** (period-correct per model): 3310/most =
classic yellow-green (the 0 default), 5210 = orange, 8250 = blue; 6210 blue /
2100 white when those models gain profiles. Harnesses must stop hardcoding
green (`web/main.js` palette, `gui_sdl.c` tint).

### Inputs (harness → emu)

- **Logical keys (`KeyId`/`KK_*`, `model.h`)** — already wired end-to-end:
  profiles carry `KeyLine[]` (KeyId → row/col or special-scan), web exports
  `dct3_web_key_id`, gui_sdl drives the same lines. Harnesses never touch
  (row,col) directly.
- **Wheel events** — `KK_WHEEL_UP` / `KK_WHEEL_DOWN` / `KK_WHEEL_PRESS`,
  appended after `KK_PWR` (the JS `KK` map hardcodes the numbering — never
  renumber existing ids). A roller detent = a momentary key tap, so the
  existing KeyLine machinery absorbs it; no new event type. How the 7110
  hardware presents the roller to firmware is an **open RE item** owned by
  the 7110 profile.
- **Capability query** — `model_key_present(prof, id)`: a key exists iff it's
  in the profile's `lines[]` (+ `has_slide` for the reed switch). Harnesses
  render/accept only what exists; a nav script pressing a missing key fails
  loudly. Deliberately NO aliasing (no wheel→arrow translation) — helpful
  aliases hide model differences, the opposite of preservation.
- **Environment** — battery (vbatt/BSI/temp), charger, SIM present, MBUS
  frame injection, RTC: present as web setters/env knobs; fold into the knob
  registry below.

### Emu controls — the knob registry (open)

~168 native env knobs + 139 `DSP54_*` + web `set_*` exports + nav `--flags`
are four hand-maintained views of the same set. Target: one typed table
`{name, type, default, scope: run-config|debug-tap, help}` that generates env
parsing, `./run` validation/help, nav flags, and the wasm setter surface.
`DSP54_*` classification: canonical-faithful defaults / A/B opt-outs /
`DSP54_ALLOW_STATIC` retired / diagnostics.

## Track 2 (after the contract): peripheral slots

Lift each chip behind a slot at **module granularity** — the bus switch is the
hot path of 300M-step runs; one indirect call per region (like DspOps today)
is fine, finer is not. One change per step, `make guard` between.

### Three principles (settled 2026-06-13)

**1. Peripherals depend on shared *fabrics*, never on each other.** The
MAD2/MAD1 ASIC (the `Mad2` struct) is the hub that owns the cross-cutting
fabrics; a peripheral signals by calling the hub's API (e.g. raise IRQ line
N), never by poking another peripheral. If chip A needs chip B's value it goes
through a fabric (shared RAM window, a bus read). `DspOps` already proves this:
the DSP slot raises FIQ0 + uses the RAM mailbox, and touches no other chip.
The fabrics here: **CTSI interrupt controller** (pending/mask, IRQ/FIQ lines),
the **GENSIO/serial bus**, **GPIO registers**, **reset/watchdog/power**.

**2. Boundary rule — chip → slot, fabric → hub.** A thing is a slot if it's a
discrete chip with its own register model reached *through* the bus (LCD,
CCONT, SIM UART, flash, DSP). It stays at the hub if it's a shared fabric. The
hard cases resolve via this rule, not against it: I/O `0x33` (flash-Vpp bit0 +
keypad-LED bit1) is a **GPIO register the ASIC owns** and fans bits out from —
not owned by flash or keypad; CCONT reg `0x05` vs CTSI `0x03` are two views of
the **reset/watchdog fabric**; the MAD1 bus `0x29` ready/status belongs to the
**bus**, not any chip on it. Tick ordering is the hub's job (deterministic
order), so peripherals never tick each other.

**3. Addressing is per-model DATA, never in peripheral code.** Split "where"
(routing) from "what" (behaviour). The peripheral implementation takes
*semantic* ops (`lcd_data(b)`, `kbd_col_read()`), never a raw address; the bus
hub maps `(model, io_offset) → (chip, op)` from profile data. This is
**mandatory, not tidy**: offsets *collide* across models — `0x2A` is the
keypad column-read on MAD2 but `CC_WR` (CCONT write) on MAD1 — which a static
switch cannot express (hence today's `quirks.mad1_serial` branches). Two forms
by scale:
- **Few ports, memory-mapped (MAD2 LCD, keypad):** individual spec fields
  (`LcdSpec.io_data/io_cmd`; add `KeypadSpec.col_port/row_port/dir_port`).
- **Shared bus, many chips (MAD1 GENSIO: CCONT+LCD+keypad+EEPROM multiplexed):**
  a per-model **bus port-map** table `{offset → (chip, op)}` owned by the bus
  fabric. The map *is* the addressing, in one place, and the natural home for
  the cross-model collisions.

### There is no "MAD1" — and quirk-flags are an anti-pattern (2026-06-13, Dan)

Two corrections that reshape Track 2:

1. **It is all MAD2.** There is no separate "MAD1" platform/ASIC in this
   architecture. The EARLY MAD2 models (5110/6110/3210) simply have differences
   — chiefly peripherals on a bit-banged serial bus + an external EEPROM. Purge
   "MAD1"/`mad1_*` naming; name things by the actual difference (serial-attached
   transport, external EEPROM), not a fictional generation.

2. **No quirk-flags for behaviour.** `if (model->quirks.X)` branches in shared
   code are the residue of the old anti-pattern. Split by the two-axis rule:
   - **Pure data mislabelled as quirks** (`asic_version`, `dsp_reset_running`,
     `dsp_release_mask`, `irq_sources`) — these are per-model VALUES the shared
     code reads. Keep as data, but move them out of a struct called "quirks"
     into an honest config home.
   - **Behavioural flags** (`mad1_serial`, `keypad.uif_irq`, `keypad.has_slide`)
     gate genuinely different behaviour → become model IMPLEMENTATIONS (vtables
     like `DspOps`), not flags. Dan's call: push the doctrine all the way,
     including the small keypad deltas.

De-quirking — ✅ COMPLETE (2026-06-13):
- (1) **BusOps** (`507d87a`) — serial transport → `mad2_bus_serial` model impl;
  the `mad1_serial` bus branches gone.
- (2) **Keypad behaviour as capability data** (`0ecc6c5`) — the keypad deltas
  compose orthogonally (scan-style × irq-source × slide × `0x33`-role × hold),
  so the honest factoring is composable capability DATA on `KeypadSpec`
  (`KeypadScan` enum + `reg33_im_c` + `hold_insns`; `uif_irq`/`has_slide` kept
  as the honest capabilities they already were), NOT a 4-way vtable that would
  duplicate the shared scan. With the bus on BusOps this left `mad1_serial` with
  zero users → **removed**. (The genuinely *structural* difference — the bus —
  correctly stayed a vtable; the two-axis rule says vtable for structure, data
  for parameters.)
- (3) **renames** — `mad1_i2c`→`ext_eeprom` (`8525473`); `PlatformQuirks`→
  `AsicConfig`, `quirks`→`asic` (`0518c96`); all `MAD1`/`quirks` shorthand purged
  from src + tools comments/strings (`035ea2b`). **src/ + tools/ now contain
  zero `mad1`/`quirks` references** (the archived `dsp-5110-mad1.md` doc path and
  the GPL-fenced `third_party/c54x` DSP-RE comments are intentionally left).

### CCONT is the peripheral; battery is not

Correcting an earlier framing: **battery/charger/temp are ADC channel *values*
(`BatterySpec` data) presented through CCONT reg 0x02/0x03 — not peripherals.**
CCONT is the single peripheral; its register map (ADC, RTC sec/min/hr/day, WDT
0x05, interrupt cascade 0x0E/0x0F) is the same silicon across DCT3 (one
implementation, `mad2_ccont.c` is already generic). What varies is the
*transport* to reach it — MAD2 memory-mapped GENSIO (`0x2C`) vs MAD1
bit-banged (`CC_WR 0x2A`/`CC_RD 0x2D`/`STATUS 0x29`) — i.e. addressing (data),
handled per principle 3.

### Slot order

0. ✅ **Interrupt fabric API (2026-06-13, 5301c81)** — `mad2_raise_fiq/irq` +
   `mad2_ack_fiq/irq` inlines in `mad2.h`; all device-model raise/single-line-ack
   sites routed through them. Bulk mask-clears in `IO_*_ACT` stay raw (firmware
   ack, a fabric op). Every slot depends on this — done first.
0b. ✅ **Bus port-map: CCONT transport (2026-06-13, 6be9a7f)** — `GensioSpec
   {ccont_w, ccont_r, start}` in the profile; CCONT routed by those fields, not
   hardcoded literals + the `mad1_serial` branch. Collisions resolve by access
   DIRECTION (`0x2D` = MAD2 START-write vs MAD1 CC_RD-read). **Design call:
   named profile fields, NOT a generic `{offset→op}` table** — matches the LCD
   pattern, avoids a per-access lookup on the hot path, lower risk. STATUS read
   stays in the dispatcher (its value differs per model — not pure routing).
   The `mad1_serial` branch now carries only genuine transport behaviour (the
   bit-banged 24C16 EEPROM). Keypad ports still pending (folded into the keypad
   lift, step 3).
1. ✅ **LCD (2026-06-13)** — one impl, routing made data-driven via
   `LcdSpec.io_data/io_cmd` (was hardcoded twice: switch 0x2E/0x6E + a
   `mad1_serial` branch 0x2B/0x2C). The 5110's "informational" ports became
   load-bearing. A vtable is added only for a genuinely different controller
   (7110 Navi = the open question).
2. CCONT (chip impl already generic; transport ports now data — see 0b. The
   chip's register model needs no further lift unless we vtable it).
3. ✅ **keypad (2026-06-13, 9d06a1d)** — scan ports → `KeypadSpec.col/row/dir_port`
   (MAD2 defaults; 5110 sets 0x30/0x31/0x2F); the `mad1_serial` keypad cases
   retired. col=read, row/dir=write, so no collision with the CCONT/GENSIO
   ports. 8850 uif/slide overlays (0x2B/0x6B/0x28-bit0/0x29/0x69) stay (MAD2
   feature gates, not addressing). Functionally verified (MAD1 beep repro).
4. buzzer/vibra · MBUS/IrDA · SIM · flash/EEPROM (mostly single-impl + values;
   few/no MAD1 port deltas — lighter than CCONT/keypad).
   DSP already vtable'd.

## Status: the milestone's structural work is COMPLETE (2026-06-13)

All four tracks done. The model-variation story is settled and proven byte-
identical across 3310 / 3410 / 5110 the whole way: each peripheral is one
implementation parameterized by profile DATA, the one genuinely-structural
transport difference is a `BusOps` model implementation, peripherals signal
through the interrupt fabric API (never each other), and there are zero quirk-
flags or `mad1` references left in src/ + tools/. A new model is a profile file.

## Open items (follow-ons, not blockers)

- **EmuHost façade** — ✅ delivered (`emu_host.h`); web routed through it; gui_sdl
  adopting incrementally (LED glow done). Remaining adoption: gui_sdl's input path
  (emu_keyline/emu_key_special — interactive-only, wants a manual smoke-test) and
  boot_trace's HAL reads. Pure decoupling; no behaviour change.
- **Unified audio** — Stage 1 done (`d36dd11`): the buzzer is synthesized into the
  one `pcm_sink` PCM stream by `emu_audio.c` (emulator-side, clocked off the tick), so
  the SDL GUI plays buzzer + codec through one channel with no DSP-type branching.
  Stage 2 (not done, wants ear-verification): migrate web to consume the unified PCM
  stream (drop its JS buzzer/tone oscillators) + fold the HLE DSP tone into the mixer.
- **Knob registry** — the remaining ergonomics win (single-source ~168 env knobs
  + 139 `DSP54_*` + web setters + nav flags into one typed table). Independent of
  the peripheral/de-quirk work; sizeable, with scope calls. Not started.
- **7110**: Navi panel verification (likely PCD8544-compatible — one-impl story
  holds unless it genuinely differs); scroll-roller HW wiring RE. `KK_WHEEL_*`
  already reserved.
- **LED colours** for models beyond 5210/8210/8250 as confirmed (6210 blue, 2100
  white have no profiles yet) — pure profile data.
- Done this milestone (for reference): guardrails · C54x consolidation + GPL fence ·
  LED colour as profile data · PCM channel + SDL audio · logical keys + capability
  query · shared `mad2_lcd_px()` unpack · LCD/CCONT/keypad addressing as data ·
  interrupt fabric API · `BusOps` serial transport · full de-quirk + `mad1` purge.
