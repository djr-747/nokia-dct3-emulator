# 5110 (MAD1) DSP Co-Sim — Faithful Boot Model (CANONICAL)

> **Single source of truth** for the 5110/MAD1 DSP co-simulation. This WINS over any archived doc
> or older 5110-dsp memory. The recipe-era predecessor (knob soup, Recipe A/B/C) is archived at
> `docs/archive/5110-dsp/5110-DSP-CANONICAL-STATE-recipe-era-2026-06-10.md` — history only.
>
> **Firmware is ALWAYS** `firmware/Nokia 5110 NSE-1 v5.30 A.fls`. **Native only** (`build/dct3_boot_trace`);
> the web build does NOT link the C54x core. The DSP is a Nokia "LEAD" = customised TMS320C54x.

> ★★★ **MILESTONE ACHIEVED (2026-06-12) — base `DSP54_COSIM=1` now boots THROUGH CONTACT SERVICE to
> the "Security code" MMI, faithfully, with NO knobs.** The cmd-0x70 DSP self-test gate is closed:
> its verdict cell `data[0x6F9]` is a two-phase COBBA codec loopback (serial walking-bit + BSP
> DXR→DRR), and the missing piece was the **COBBA BSP digital loopback** (commit `9348d36`) plus
> **promoting the COBBA model to a faithful default-on sub-knob** (so it engages under bare cosim).
> VERIFIED: `tools/check_5110_boot.sh` (no extra env) → lcd md5 **`4921792d…`** (Security code, was
> CONTACT SERVICE `b86d6d5c…`); `DSP54_COBBA=0` opts out → CONTACT SERVICE; `make test` 41+16+35;
> 3310 byte-identical. **The ⚠️ "base does NOT boot past CONTACT SERVICE" banner below is now
> SUPERSEDED — it documents the pre-fix state; kept for history.** Full RE chain:
> `docs/research/5110-selftest-protocol.md` + `5110-selftest-cobba-fix.md`.

> ★★ **reason-0x68 RESOLVED on the 5110 (2026-06-12 PM):** the deterministic idle reset
> (staged ~283.5M) was the **unprovisioned external-EEPROM DSP-fault latch** — record `0x607`
> = 1 byte @ `EE[0x29E]`, virgin `0xFF` reads bit0 = "SWDSP fault latched"; a periodic
> (~94M-step ≈ 7.2 s) watchdog check (msg-29 → handler `0x291C1C` → checker `0x288962`) enforces
> it as reboot reason `0x68`. NOT MDI-activity starvation (that is the 3310's separate mechanism).
> Fixed by virgin-only factory provisioning at EEPROM load (`mad1_i2c.c`; `EE5110_RAW=1` opts out).
> 320M soak: 3 healthy checks, no reset. Full story: `docs/5110-happy-dsp-HANDOFF.md` §1.
> Also settled there: the keypad beep chain works organically end-to-end (900 Hz, ~440 samples/press
> via `DSP54_PCMCAP`); beeps are MMI-fixed-duration; `[0xAA]` is call-audio, not the beep latch.

> ⚠️ **CORRECTION (2026-06-11 — SUPERSEDED 2026-06-12 by the milestone banner above; pre-fix history):
> base cosim does NOT boot past CONTACT SERVICE.** An earlier version of this doc claimed `DSP54_COSIM=1` "passes CONTACT SERVICE,
> byte/step-identical to the HLE." That claim was **measured at a 3M cutoff** — *before the screen
> renders any content (still the black splash at 3M) and ~5.6M steps before the DSP self-test times
> out.* **The verification standard is 30M, not 3M.** At 30M the truth is:
> - **`DSP54_COSIM=1` (faithful) → CONTACT SERVICE, latched.** The cmd-0x70 DSP self-test never
>   delivers an organic PASS, so it **times out at step 8.63M** (PC `0x2319FA`), which clears verdict
>   `[0x10FDDD]` bit6 (`0xC4→0x84→0x00`) → the MMI latches CONTACT SERVICE by ~9M and holds it.
> - **`DSP54_HLE=1` (legacy fake) → normal MMI (PIN/standby).** The HLE self-test *responder* fakes a
>   PASS so the verdict survives. Cosim silences that responder (`dsp_hle_quiet`) → no fake → timeout.
> - So cosim and HLE **diverge at the screen** — they are NOT byte/step-identical. The genuinely-
>   grounded part below (DSP reaches a healthy run-mode idle, balanced doorbells, 230M no-halt soak)
>   is about **DSP internal health** and remains true; it does **not** imply the MMI boots through.
>
> **The real open item is the cmd-0x70 self-test measurement source** (model it so the real DSP
> returns a valid PASS before 8.63M) — see Open/uncertain §1. Until then, faithful cosim stops at
> CONTACT SERVICE. **Always verify 5110 boot at ≥30M.**

Last grounded: 2026-06-11. **This is the BASE STATE for the DSP co-sim — a healthy DSP boot, NOT a
through-boot to standby (see CORRECTION above).** Verified with a scrubbed environment
(`env -i ... DSP54_COSIM=1`): one knob, no others, the DSP reaches the permanently healthy run-mode
idle (`0x31A4`, INTM=0, IMR=0x52FD, IFR=0, SP=0x1EC5 stable), every MCU doorbell serviced with
exactly balanced frames, **zero HLE-faked signals**, 230M soak clean (no halt / wild-PC / reason-0x68),
make test 41+16+35 green, **3310/non-cosim boot byte-identical (untouched)**. The 5110 MMI itself
still latches CONTACT SERVICE at 30M (DSP self-test timeout). (commits `373b3a6`..`9e5815c`)

---

## TL;DR — ONE knob

**`DSP54_COSIM=1` IS the faithful path — and the whole recipe.** It implies the entire faithful
load path AND chip model (`dsp54_faithful()` + `dsp_hle_quiet`): held-at-reset boot, real MCU
upload, single-store DARAM, real loader1 CRC-verify, MCU rehome, C542 `no_xpc` interrupt frames,
on-chip timer, host doorbell on vec 25, all HLE DSP signals silenced. No flash seeds, no HLE
block-scatter, no extra knobs.

```bash
FW="firmware/Nokia 5110 NSE-1 v5.30 A.fls"
# Verify at 30M — PAST the 8.63M self-test timeout. (3M is useless: the screen is still the black
# splash and the timeout hasn't fired, so a 3M run looks "clean" while the boot is NOT through.)
DSP54_COSIM=1 ./build/dct3_boot_trace "$FW" 30000000       # faithful DSP boot — MMI ends on CONTACT SERVICE
DSP54_COSIM=1 DSP54_COMMLOG=1 ./build/dct3_boot_trace "$FW" 30000000  # + the whole MCU<->DSP conversation
# verdict gate (watch it clear at 8.63M → that's the CONTACT SERVICE cause):
RAMWATCH=0x10FDDD RAMWATCHLEN=1 DSP54_COSIM=1 ./build/dct3_boot_trace "$FW" 30000000
```

An explicit `DSP54_X=0` still opts a single faithful knob out (A/B), and `DSP54_HLE=1` re-enables
the legacy HLE model. The run-mode frame crutches (`DSP54_MMR58PER` / `DSP54_FRAMEINT` /
`DSP54_FRAMEPER`) are **obsolete for reaching idle** (the vec-25 + decoder fixes removed the need)
— they remain as explicit-only A/B probes.

---

## The faithful boot architecture

Grounded end-to-end against the real firmware + the recovered DSP image. The cold boot runs to a
clean **hold** at ~865k dsp_steps, the MCU rehomes run-mode code, then a warm reset enters it and a
**doorbell handshake** drives the DSP into run-mode (tasks enabled, producer alive). See Grounded.

| stage | when (dsp_steps) | who | what |
|-------|------------------|-----|------|
| **0. Stage loader1** | `@28k` | MCU | MCU's boot routine STM-copies block25 (loader1) into the HPI window at DSP `0xF00`. The copy is `0x271A66 → 0x2936E4` (ARM `bx pc` → `stmia r1!,{r3-r12}`, 32-bit STM) → MCU `0x10E00`. Then MCU sets `0x20002` bit0 (0→1) = **cold reset release**. |
| **1. Reset → loader1** | `@28k` | DSP | Hardware reset vector `0xFF80` (`stm #0xFFA8,pmst` → `b 0x0F00`) enters loader1 at `0xF00`. (SPRU131G: the reset vector for HW resets *always* resides at `0xFF80`.) |
| **2. Stream + CRC-verify** | `28k–865k` | MCU + loader1 | MCU streams the DSP code blocks through the **bufA/bufB double buffer** (`0x10200`/`0x10600` = DSP `0x900`/`0xB00`), flash source `0x200040+` (+`0x4000`/block). **loader1 only CRC-*verifies*** each buffer in place (`call 0x8023` is a checksum, NOT a copy/relocate). The MCU is paced by the real **mailbox poll** (`0x271B2A`/`0x271B3C`), synced to loader1's MBOX acks. |
| **3. Hold** | `@865k` | MCU | MCU clears `0x20002` bit0 (1→0) = **DSP HELD in reset**. The CRC pass is done; the DSP freezes (loader1 parked at `idle 1`, PC `0x0F6A`). |
| **4. Clear + rehome** | `905k–934k` | MCU | While the DSP is held, the MCU **zero-clears the entire shared window** (k=`0x000`–`0x7FF`, incl. loader1 at `0xF00`) then **rehomes run-mode code via 32-bit STM**: the **run-mode bootstrap ("loader2")** to DSP `0xF00` (overwriting the wiped loader1), the **IVT** to `0xD00` (`F073 26xx` = branches to run-mode handlers), and MDIRCV ring ptrs. Confirmed by `DSP54_F00DUMP`/`HOLDLOG` — these are the MCU's STM writes, not DSP stores. |
| **5. Warm reset → loader2 → run-mode** | `@934k+` | MCU → DSP | MCU re-releases reset; **RS forces PC=`0xFF80`** (`stm #0xFFA8,pmst` → `b 0x0F00`), which now holds the **rehomed loader2**, not loader1. loader2 inits (SP/CPL/clock, clears scratch + run RAM `0x1000–0x2FFF`, relocates the `0xD00` IVT into prog-mem), **clears `[0x872]`**, signals ready (`[0x871]=0x12` + pulses **BSCR `0x29` bit3 = HINT doorbell → MCU IRQ4**), then parks at `0xF3D` polling `[0x872]≠0`. The MCU's IRQ4 mailbox handler (`0x2BCEF0`) delivers the run-mode **go-command `[0x872]=0x0002`**; loader2 wakes → **`b 0x0D80` → run-mode** (`0x3000` superloop; tasks `[0x6E4]` enabled; producer `0x37CE`). |

**Key reframes that make this faithful (all grounded):**
- **loader1 is a CRC verifier, not a relocator.** It streams + checksums, then *parks at `idle 1`* (INTM=1, so only RS can leave it — DSPINT-wake was ruled out). It is *meant* to be discarded, not to branch to run-mode itself.
- **The MCU arranges the blocks**, via real HPI-window writes while the DSP is held — not an emulator HLE. (We retired the `seed_block_from_flash` SEEDBASE scatter; it was an unfounded shortcut that also *corrupted loader1* — the IVT seed overwrote loader1's own `0xF33–0xF43`.)
- **The warm boot IS a hardware reset to `0xFF80`** (RS, non-maskable) — `dsp54_warm_reset(0xFF80)` is **faithful**, NOT an artifact. It does not re-enter loader1: the MCU **overwrote `0xF00` with loader2** during clear+rehome (stage 4). The resident reset vector is unconditional (`b 0x0F00` always) — there is no cold/warm branch.
- **The wake is a doorbell handshake, not DSPINT.** loader2 signals ready via the **BSCR `0x29` bit3 HINT doorbell → MCU IRQ4** (handler `0x2BCEF0`); the MCU answers with the go-command `[0x872]`. The MCU's *pre-release* `[0x872]=2` write (`0x271D1A`, inside the inline warm-boot fn `0x271BC8`) is a doomed default — loader2's own `0xF2A` clears it after release — so the **responsive** write MUST come from the IRQ4 handler *after* the doorbell. Wired via `DSP54_HINTIRQ4` (faithful default). Without it the doorbell is invisible and the DSP spins at `0xF3D` forever.

---

## Memory model — single-store DARAM (faithful)

On the C54x, on-chip DARAM is **one physical dual-access memory**; with `PMST.OVLY=1` it maps into
both program and data space at the same address, and the **HPI RAM is a slice of that DARAM the host
shares** (SPRU131G §3/§8). The co-sim now models exactly this:

- **`api_ram` IS the DARAM** for `[0x800,0x2800)`. A program fetch (gated by the LIVE `PMST.OVLY` bit
  in `ovly_cell`), a DSP data access, and the MCU HPI window (`MCU 0x10000+2k ↔ DSP 0x800+k`) all
  resolve to the **same cell**. No `data[]`/`api_ram[]` split, no `UNIDARAM`/`SHAREDWIN`/`HPIAPI`
  knobs (retired as no-ops over the coherent store). (`data[]` remains the single store for the
  DARAM below the window, `[0x80,0x800)` — two adjacent stores, one cell per address.)
- **Ghost-shadow purge (2026-06-10 PM).** The lift's **Calypso boot-ROM model** (`c54x_reset`
  copies `prog[0x7080+] → DARAM` — real mask-ROM behavior on Calypso, NOT on MAD1) planted 716
  stale words into `data[0x800-0x2800)` that the (api_ram) fetch could never see — pure analysis
  poison (it mimicked "resident scheduler code that doesn't execute"). Now gated off under
  `no_xpc` (the C542/MAD1 chip mode) + scrubbed at `dsp54_create`. MAD1 DARAM powers up CLEAN and
  is populated only by the MCU's real HPI upload, the DSP's own stores, and `SEEDDARAM`.
- **ARM-core STM fix** (`src/core/dct3_core.c`): `mem_storeMultiple` now dispatches each word to the
  MMIO handler like `mem_store32`. STM stores previously bypassed MMIO → the MCU's 32-bit STM upload
  of loader1 silently hit raw RAM instead of the DSP window. This was THE reason the seed was needed.

**Chip mode — `no_xpc` (C542-class, faithful default `DSP54_NOXPC`):** the MAD1 LEAD has a single
64K program page and **no XPC**. Per SPRU131G: an interrupt pushes ONLY the 16-bit PC, and
**RETE pops only PC** ("load the PC with the return address at the top of the stack") — only the
F-prefixed far ops (FCALL/FCALA/FRET/FRETE) move XPC. The lift's C548/549-style 2-word {PC,XPC}
frames broke the firmware's task-level `call …; rete` idiom (the sleep/wake sequencer tail
`0x90BE`) by one word → "return" into the empty page 1 → zero-walk → MMR spray.

**POPD implemented (was a stub-NOP).** `0x8B = POPD Smem` (pop TOS→Smem, SP++). The vec-25
doorbell ISR's long path pairs `pshd *(0x60)/*(0x61)` (entry 0x3785/87) with `popd` (exit
0x37AE/B0); the stub leaked +2 SP words per pass → the ISR's RETE popped a data word → ret-storm
into zeros. With the fix every doorbell cycle is exactly balanced (IRQPAIR-verified: entry pushes
ra=0x31A5, RETE pops ra=0x31A5).

Authoritative contract: `docs/dsp-c54x-memory-model.md`.

---

## What's GROUNDED vs OPEN

### Grounded (verified this session)
- loader1 = CRC-only; never rehomes (`0x8023` = checksum).
- The MCU stages loader1 (STM, `0x271A66`), streams+CRCs, **holds** (`@865k`), **rehomes** the
  verified blocks itself (`@923k`, DSP held), then re-releases (`@934k`).
- Reset vector `0xFF80` → loader1 `0xF00`, unconditional in the resident ROM.
- The block placement is the MCU's real HPI-window writes (not an HLE) — `PLACELOG`/`HOMELOG`/`F00DUMP`.
- **Warm boot wired + reaches run-mode (this session).** Warm RS `0xFF80`→`0xF00` enters the rehomed
  loader2; the **BSCR `0x29` bit3 doorbell is wired to MCU IRQ4** (`DSP54_HINTIRQ4`, faithful default —
  the core counts the 1-instruction-wide bit3 rising edge so it isn't missed at RATIO>1); the MCU's
  IRQ4 handler (`0x2BCEF0`) delivers `[0x872]=2`; loader2 → `0x0D80` → run-mode. **Verified:** DSP
  reaches idle, **`taskbmp[0x6E4]=0x0349`** (tasks enabled, was `0x0000`), the producer **re-pulses the
  doorbell `@1840k`** (DSP→MCU run-mode traffic = no longer dormant), 230M steps clean (no halt / no
  reason-0x68 / no wild-PC), and the LCD is unchanged (doorbell ON == OFF — purely internal advance).
- DSPINT-wake-from-IDLE (SPRU131G §8.6.4) was **ruled out**: loader1 holds INTM=1 to its `idle` park
  (no `rsbx intm` anywhere), so a DSPINT could only resume it inline, never vector it. The warm entry
  is RS, and the rehomed code at `0xF00` is loader2, not loader1.
- `make test` 41+16+35 green throughout; default (non-cosim) boot byte-identical (3310).

### Grounded 2026-06-10 (PM session — run-mode steady state cracked)
- **C54x decoder bug fixed (the idle-deafness root).** A bogus `(op & 0xFCE0) == 0xF4A0` "SFTL"
  catch-all (two copies) swallowed the whole `F[4-7]A0–BF` window — **RSBX/SSBX ST0/ST1, CMPR,
  LD #k,ARP** all executed as a harmless accumulator shift (same catch-all bug class as the
  decode-length fixes, 1-word form). In particular the run-mode idle loop's `rsbx st1,intm`
  @`0x31A3` silently NO-OPed → INTM stayed 1 → `idle 1` was woken-but-never-serviced, forever.
  Fixed in `calypso_c54x.c` (binutils-verified decode; RSBX/SSBX fall through to their dedicated
  handlers). Run-mode now parks at **`idle 1` @`0x31A4` with INTM=0, IMR=0x52FD, SP healthy**.
- **The run-mode steady-state location** (live-dump disasm): inner idle loop `0x319D–0x31A5`
  (`ssbx intm; bitf [0x866],#2; bc work; rsbx intm; idle 1`), work dispatcher `0x390A` → MDISND
  command processor `0x3924` (30+ handler returns), producer/enqueue `0x37CE`. The **outer
  superloop iteration (`0x311A–0x3151`) has an autonomous d2m heartbeat enqueue at `0x314D`**
  (2-word msg from `[0x1867]` staged at `[0x1200]`), gated by `[0x6E3]`bit0 / `[0x1949]`bit7.
  Idle-exit = per-task bit test `bitt [0x6D1]` (bit = 15−AR2) at `0x31AE`.
- **The TI on-chip timer is NOT the heartbeat.** Timer counting was added to the core
  (`DSP54_TIMER`, faithful default; SPRU131G §8.8 incl. free-run-from-reset) — and promptly proved
  the firmware **stops** it: loader2's first insn `0xD80: stm #0x10,TCR` (TSS=1) is the **only**
  timer-register write in the entire image. Vec 19 / IMR bit3 must be a MAD1 system-logic source.
- **DSPINT lands on vec 25 — the real run-mode host doorbell (MAJOR).** Run-mode IMR=`0x52FD`
  masks INT1 (the old vec-17 default left IFR bit1 pending forever; its rationale was measured
  under the broken decoder). The IVT routes **vec 25 (`0xFFE4`) → ISR `0x3772`**, which is the
  whole conversation in one ISR: `orm #2,[0x866]` (the idle work-flag!), processes the `[0x870]`
  mailbox via `0xA51B` (cmd word `[0x85D]&0x1F`), and acks with the **BSCR.b3 HINT pulse**.
  `DSP54_INTVEC=25` is now the faithful default. **Result (COMMLOG-grounded):** the MCU's boot
  host-commands (`[0x872]=4` overlay-upload requests with UPLOADHEADER descriptors) are processed
  and HINT-acked, port1/port2 mailbox replies flow, and the demand-paged overlays LAND (dest
  `0x930–0xC7E` 838/846 words, `0xD80–0xEA2` 282/290 — verified in the live DARAM at idle).
  ⚠ EVIDENCE CORRECTION (2026-06-11): an earlier claim that "the producer enqueues organically,
  MDIRCV tail `0x80→0x84`" was measured BEFORE the no_xpc/POPD fixes — i.e. in the corrupted-stack
  regime — and does NOT reproduce on the clean build. **Clean-build truth: the upload protocol
  acks via PORTs + HINT only; the MDIRCV ring is born empty (head==tail==0x80), stays empty, and
  the MCU never reads it at this boot state** (zero ring-pointer reads in 40M, COMMLOG). That is
  protocol-faithful: nothing posts MDI at no-SIM standby, and — unlike the 3310, whose 0xE4
  DSP-liveness watchdog trips at 203M on exactly this silence — the 5110 shows NO armed consumer.

### Open / uncertain
0a. ⚠ **DISPATCHER-WAKE RESOLVED on branch `dsp-page-chain-decode-fix` (2026-06-12 PM).** The
   `DSP54_TICKVEC=25` "dispatcher-wake stand-in" is now **RETIRED** — once the demand-page sleep
   overlay (id 8) lands at `0x1CA4` (READA/WRITA PAR-seed + `INTR k` decode fixes, commit
   `8ab4a07`), the faithful `0x30A7` sleep loop self-pumps the ring drain. MEASURED: base (no
   knobs) and `DSP54_TICKVEC=25` are **byte-identical** (same MDIRCV organic drain at the same
   steps, same 25 CMDLEVEL edges, same verdict timeline) even though vec25 demonstrably fires.
   So the LAST gap below ("the dispatcher WAKE") is closed — but the through-boot blocker MOVED one
   layer deeper: the organic DSP now posts a SECOND self-test report (~3.2M) with a garbage
   measurement field that `DSP54_SELFTEST_MEAS` doesn't cover, so the MCU validator clears verdict
   bit6 (`0x22F082`) → CONTACT SERVICE. See the `5110-decode-bugs` HANDOFF §4a-NEW. (This is BRANCH
   state; `main` still uses the TICKVEC crutch.)
   - **2026-06-12 (PERIPHMAP session): COBBA-serial hypothesis REFUTED + TWO inherited claims
     corrected.** Built `DSP54_PERIPHMAP` (DSP peripheral-I/O surface map — ports + the `0x00-0x5F`
     MMR band, STRUCTURAL "modeled?" flag; default OFF, read-only). `DSP54_DSPHALT` halts at each
     candidate PC then DISPROVED the prior story: (1) **`0x250B` is RESIDENT PROM, not a missing
     overlay** — reading all 27 demand-page block headers from flash (table @ RAM `0x10AA04` → flash
     `0x294xxx`, flash base `0x200000`), NO block dest covers `0x250B` (it's in resident space
     between the `0x2286` blocks' top `0x23C3` and id-1's `0x25B4`); it holds `RETD; NOP` from boot,
     never written. So "acquisition overlay not on hand / never demand-paged" is WRONG — nothing is
     requested/sent/clobbered. (2) **`0x4B73`+`call 0x250B` build report #1 (@1520k, PASSES), NOT
     report #2.** The failing 3.27M report #2 is transform **`0x3900`** → enqueue **`0x4A00`**
     (@3266k). Its leading fields ARE the modeled COBBA nominals (`0x010`/`0x160`); words 8–12
     (`5dcf c017 3100 a325 2a7a`) are stale/uninit from an unmodeled DSP-internal source `0x3900`
     reads (`[0x853]`/`[0x866]`/ring cells). Toggling COBBA does not move the verdict → report #2 is
     COBBA-independent. **Corrected fix target = trace what `0x3900` reads into staging words 8–12**
     (a measurement INPUT, not a missing block). `DSP54_SELFTEST_MEAS` stays the stand-in. Full
     evidence + tool how-to: `docs/5110-peripheral-surface-HANDOFF.md`.
   - **★ GATE RE-IDENTIFIED 2026-06-12 (multi-agent fan-out RE, poke-proven) —
     `docs/research/5110-selftest-protocol.md` is now AUTHORITATIVE for the self-test.** The screen
     gate is NOT the loopback report band-check (that plane does not gate; report #2 PASSES, its
     result is discarded). The gate is the cmd-0x70 self-test's **`0x0D` verdict record `byte[9]`**
     (`[0x101441]`): `[0x10FDDD]` bit6 clears at `0x22F082`/`0x22F06C` iff `byte[9] & 0x03 != 0`.
     `POKE=0x101441=0x00 POKEAFTER=2000000` → "Security code" (`4921792d`) — VERIFIED. Set at 1.525M,
     before the sub-0x16 validator runs. `byte[9]` = the DSP's COBBA-scored 2-bit fail field
     (`DSP54_COBBA=0`: `0x02→0x01`, still fail; need `0x00`). **Faithful fix = make the DSP emit
     `0x0D00` via an in-band COBBA/CCONT measurement.** Open RE: pin the DSP PC that scores the COBBA
     read into the `0x0Dxx` field + the exact COBBA cell.
   - **★★ GATE CLOSED 2026-06-12 (commit `9348d36`, VERIFIED) — COBBA BSP digital loopback.** The
     `data[0x6F9]` verdict cell is folded by loader2 (block 18) via a TWO-PHASE COBBA codec self-test:
     phase 1 = the COBBA SERIAL walking-bit loopback (ports 0x2C/0x2D — already passes), phase 2 =
     the **BSP loopback** (writes DXR `MMR 0x21`=0x0AAA, polls DRR `MMR 0x20` for the echo, BSPC0
     `MMR 0x22`=0xC008/0xC0C8 = codec loopback mode). Our core never drove DRR → poll failed →
     `0x6F9=0x01` → CONTACT SERVICE. **Fix (`mad2_dsp_c54x.c`): model the codec's faithful digital
     loopback — mirror DXR(0x21)→DRR(0x20) while COBBA present** (the echoed word, not an arbitrary
     constant; gated `DSP54_COBBA`, opt-out `DSP54_COBBA_NOBSPLOOP=1`). VERIFIED:
     `DSP54_COSIM=1 DSP54_COBBA=1 tools/check_5110_boot.sh` → **Security code `4921792d`** (the target);
     A/B `NOBSPLOOP=1` → CONTACT SERVICE (sole lever); make test 41+16+35; 3310 byte-identical.
     **REMAINING for a no-knob through-boot:** COBBA is still opt-in (getenv-presence); base
     `DSP54_COSIM=1` alone is still CONTACT SERVICE. Promote COBBA to a faithful default-on sub-knob
     (like the others) + verify the 230M healthy-boot/audio paths don't regress = the final step.
0. ⚠ **PARTIALLY RESOLVED 2026-06-11 PM (see the new Grounded section below): the self-test now
   executes, ships a valid nominal report and PASSES the validator under `DSP54_CMDLEVEL` +
   `DSP54_SELFTEST_MEAS` (both faithful defaults) + a dispatcher-wake (`DSP54_TICKVEC=25`
   stand-in) — boot reaches the normal "Security code" MMI @30M. The remaining blocker is ONLY
   the faithful dispatcher-wake source.** Original analysis (still the correct background):
   **★ BLOCKER — cmd-0x70 self-test never PASSes → CONTACT SERVICE @30M.** This is THE reason
   faithful cosim doesn't boot through. The MCU issues the DSP self-test (cmd 0x70, MDISND ring);
   the real DSP never returns a valid measurement report (see "self-test report is garbage-sourced"
   below), so the MCU's wait **times out at step 8.63M** (`0x2319FA`). That timeout clears verdict
   `[0x10FDDD]` bit6 → MMI latches **CONTACT SERVICE** (~9M, held through 30M, proven). The HLE only
   "passes" because its self-test responder fakes the verdict — silenced under cosim. **Fix: model
   the cmd-0x70 measurement source** so the DSP returns a valid PASS report before 8.63M (real HW
   fills it with ADC/signal data; ours reads uninitialised state). This is the next milestone for
   "faithful 5110 boots to standby." Until then, the canonical cosim state = healthy-DSP + CONTACT
   SERVICE. **Always check at ≥30M.**
1. **Steady-state d2m heartbeat / 0x68 milestone validation.** ⚠ The earlier "`[0x85D]` cmd
   protocol gap / vocoder-strands-on-uninit-state" reading is **SUPERSEDED** — that strand was TWO
   core bugs, both fixed (see Memory model below): the C548-style 2-word XPC interrupt frames
   (→ `no_xpc` chip mode) and the stub-NOP **POPD** (→ implemented). With both fixed the DSP
   parks at a permanently-healthy armed idle (`0x31A4`, INTM=0, IMR=0x52FD, IFR=0, SP rock-stable)
   and services every MCU doorbell with exactly balanced frames. `[0x85D]`=0 at standby is
   protocol-faithful ("no audio task"); the handler's cmd=0 path is `0xA4CB` and returns cleanly.
   REMAINING: at standby the MDIRCV ring rests (producer enqueues only on command traffic) — does
   the 5110 analogue of the 3310 reason-0x68 DSP-watchdog get fed (superloop heartbeat at `0x314D`,
   gates `[0x6E3]`bit0/`[0x1949]`bit7), or does the 5110 even arm one? Validate over ≥97M windows.
2. **`0x20002` bit1 / `0x20003` semantics.** Observed: `0x20002 <= 0x03` (bit0+bit1) `@1686k`,
   `0x20003 <= 0x31` `@5874k`/`@22697k`. MCU-side MAD1 control bits around the warm sequence. The
   inline warm-boot fn (`0x271BC8`) drives reset via `0x20008`–`0x2000B`; our release model keys on
   `0x20002` bit0 — reconcile the two (benign today: the warm sequence fires correctly).
3. **HPI window extent.** We expose 8K (`0x800–0x2800`) to the MCU window; the real MAD1 HPI RAM is
   **2K** (`0x800–0xFFF`), with `0x1000+` DSP-private. Benign today (all shared traffic is < `0xFFF`).

### Grounded 2026-06-10 (eve) — MCU→DSP MDISND command delivery + the self-test report

Investigating the SIM-inserted CONTACT-SERVICE path surfaced the **MCU→DSP command ring (MDISND)**
delivery mechanism — distinct from the DSP→MCU MDIRCV reply ring above (which rests empty at
standby). Findings:

- **The reason-4 reboot is a KNOB ARTIFACT, not faithful behaviour.** Only `DSP54_UNIDOORBELL=1`
  **and** `DSP54_FIQ0OUT=2` *together* trip it (@4.758M); base `DSP54_COSIM=1` and each knob alone
  run CLEAN (no reboot). Do NOT promote those knobs.
- ⚠️ **CORRECTED (2026-06-11): base config does NOT pass CONTACT SERVICE.** The prior text here
  ("passes CONTACT SERVICE — verdict bit6 set 1.44M→8.63M, byte/step-identical to the HLE") was a
  **3M-cutoff artifact**. Re-verified at 30M: verdict `[0x10FDDD]` bit6 is set 1.44M→**8.63M** and
  then **clears at the 8.63M self-test timeout** (`0xC4→0x84→0x00` @PC `0x2319FA`) → the MMI latches
  **CONTACT SERVICE** by ~9M and holds through 30M. HLE does NOT clear the verdict (its self-test
  responder fakes a PASS) → HLE reaches a normal MMI screen. **Cosim ≠ HLE at the screen.** The DSP
  *internal* boot is still clean; the MMI gate is not satisfied. Fix = model the cmd-0x70 measurement
  (Open §1), not a knob.
- **The self-test (cmd 0x70) is an MDISND ring command.** MCU enqueue fn (`0x2717F0-0x271858`)
  advances tail `[0x100A4]` then strobes **only `[0x020008]=2`** (no `[0x30000]`). The DSP consumes
  it only when BOTH fire: **vec25** sets `[0x866]` bit1/mask2 (wakes `idle 1` @`0x319D`→dispatcher)
  AND **vec18** host-cmd path (`0x3621`) sets `[0x866]` bit0/mask1 (arms dequeue `0x394F→0x3860`).
  Base `[0x20008]→vec25` only → woken-but-never-armed → ring (tail 0x33) never consumed → self-test
  **times out @8.63M** (`0x2319FA`), validator `0x258B5E` never called. (HLE does the identical
  optimistic-verdict-then-timeout — it never runs a DSP self-test either.) `DSP54_INTVEC=18` alone
  also fails (wakes but never sets mask2). UNIDOORBELL fires both → clean delivery, cmd-0x70
  dispatched, replies enqueued.
- **The 52-byte report is garbage-sourced.** cmd 0x70 → builder `0x4B73` copies a circular source +
  `0x7f17` scramble into a high-entropy buffer (transfers **byte-identical** DSP→MCU, NOT a ring
  bug). `byte9=0x32` coincidentally passes the validator type-gate; the BCD/range checks then fail
  (`byte21=0xA6`). It's the DSP's diagnostic MEASUREMENT report — real HW fills it with valid
  ADC/signal data, ours reads unmodelled/uninitialised state. Only reaches the MCU under FIQ0OUT.

**FAITHFUL fix — NOT a frame timer (corrected 2026-06-10 eve).** The dequeue-arm is the issue, and
the IVT decode resolves the interrupt map: **vec16=INT0** (external frame, ISR `0x3204` reads
`pa33`), **vec18=INT2** (host-cmd), **vec25=HPINT** (host-port doorbell, ISR `0x3772` — sets
`[0x866]` bit1 wake + services the `[0x870]` mailbox via `0xA51B`; it does NOT touch MDISND).
`MMR[0x58]` is **CLKMD** (clock/PLL), NOT a frame timer — the `0x7EBE/0x7ECC` "two-phase poll" is a
PLL-lock wait inside an `idle 3` low-power routine (`0x7EB7`) that isn't even reached at standby.
The old keepalive memory's "model MMR[0x58] frame timer" path was already tried (06-08) and the
frame loop ran 17848× with the **dequeue still at 0** — because the MDISND dequeue arms only on
`[0x866]` bit0, set by the vec18 host-cmd-bit1 handler `0x3621`. The ring-post `[0x20008]` strobe
fires HPINT/vec25 (wake) but never re-arms. UNIDOORBELL works by ALSO firing vec18, which
re-evaluates the latched `port1=0x900F` (bit1 set) → `0x3621` → re-arm. **So the real open question
is the `[0x20008]`/`[0x30000]` doorbell→interrupt mapping (does the ring-post assert INT2/vec18 on
MAD1?), not a periodic timer.** See memory `5110-dsp-faithful-boot-model` cont.4/cont.5.

### Grounded 2026-06-10 (night) — DOORBELL MODEL SETTLED + standby is the faithful terminal state

The MCU↔DSP doorbell/ring model is now brought to faithfulness (commits d4b762e, c0b840e, 7cbf316,
60c7c16), grounded on MADos (`ref/Nok-MADos-master/hw/{dsp,mdi}.c`, `include/hw/int.h`) + Dan-confirmed
HPINT.

**The settled interrupt map (MCU→DSP):**
- **HPINT = `[0x30000]` bit2 → vec25 (0x3772)** — the ONLY MCU-pulsed host doorbell. Sets the idle
  work-bit `[0x866]`bit1 (dispatcher wake) + services the `[0x870]` mailbox (0xA51B). `g_hpivec=25`
  is the faithful default (was 18 — a mis-route to the cmd-word ISR). `dsp_genint()` in MADos = this.
- **INT2 = vec18 (0x3598/0x35B9)** — the port cmd-word channel; bit1→`0x3621` arms the MDISND dequeue
  (`[0x866]`bit0). **NOT MCU-pulsed** (the poster 0x272018 writes only `[0x30000]`+`[0x20008]`) →
  it is a separate **periodic timer-poll** of the latched `port1=0x900F` (validated: `DSP54_CMDPOLL`
  fires vec18 → reaches 0x35B9 → arms bit0).
- **`[0x20008]` = the MCU's OWN ARM FIQ register** (IO_FIQ_ACT), never reaches the DSP. The old
  `[0x20008]→DSP-vec` mapping (and `DSP54_UNIDOORBELL`) were HACKS standing in for `[0x30000]→vec25`;
  RETIRED (`DSP54_RINGDOORBELL`, default OFF).

**The DSP→MCU map (RINGFIQ, faithful default):** ring-pointer advances raise the MCU FIQ —
MDIRCV-tail advance → FIQ0 (`FIQ_MDIRCV`), MDISND-head advance → FIQ1 (`FIQ_MDISND`). NOT port-write
events/values (the DSP's pa1 writes are status words 0x0700/0x0100/0x0000).

**Consumption chain (decomposed + proven):** the DSP drains the MDISND ring when BOTH (a) the dequeue
is ARMED — vec18/INT2 timer-poll → `[0x866]`bit0, and (b) the dispatcher RUNS. The dispatcher `0x390A`
is called UNCONDITIONALLY from the superloop at `0x3138`, so (b) holds whenever the superloop iterates.
The superloop iterates vs drops to `idle 1` at junction `0x3198` → `bitt [0x6D1]` (per-task READY bit):
ready → iterate (→ dispatcher → drain), none → idle (only HPINT breaks it). `CMDPOLL + RINGDOORBELL`
proves it: head drains 0x06→…→0x33, cmd-0x70 0x4951 runs.

**STANDBY IS THE FAITHFUL TERMINAL STATE.** Over 40M steps base-cosim, `[0x6D1]` is written only in
early init (3× `0x0000`) and NEVER after — **no task is ever posted, the DSP idles forever, 0x4951
never runs**. This is correct: the DSP's standby work source is the **GSM baseband** (frame sync
INT0/vec16 + RF burst data on the `0x30–0x40` serial group), and our cosim has **no RF** → no frame
strobe, no burst data → no task → idle. The self-test command timing out @8.63M is part of this
correct DSP idle — but ⚠️ that timeout is ALSO what clears verdict `[0x10FDDD]` bit6 → **the MMI
latches CONTACT SERVICE @30M** (it does NOT pass; HLE only "passes" by faking the verdict — see Open
§0). The UNIDOORBELL "consumption" (+ its garbage 52-byte self-test report → reason-4 reboot) was the
NON-faithful artifact.

**Peripheral map (DSP I/O the cosim does NOT drive — all dormant at no-RF/no-call standby):**
McBSP0 codec (MMR 0x20/0x21 DRR/DXR, vec20 BRINT0) = voice PCM, **call-only** (NOT expected at
standby); baseband/RF serial (MMR 0x30–0x40); on-chip timer (0x24–0x26, firmware stops it); frame
status pa33 (returns 0); analog/RF control ports (pa 0x36/0x38/0x3C, write-only sinks). **"Active
mode" is not a missing peripheral model — it is a missing RF/MMI STIMULUS.**

**NEXT (Dan-directed): keypad tones** — the active-mode trigger that needs NO RF. A key press →
MCU sends a tone command to the DSP → DSP posts a tone-generator task (spins the superloop) → emits
the tone via MDIRCV → the web HTML harness detects the DSP messages and renders the sound. This gives
"the shape" of how the DSP responds to a real, RF-free stimulus.

### Grounded 2026-06-11 (PM) — INT2 LEVEL COMPARATOR (DSP54_CMDLEVEL) + self-test validated end-to-end

**The INT2/vec18 trigger is SOLVED: a MAD-glue LEVEL comparator over the request/mask latch,
edge-sampled at the pin.** Grounding chain (docs/research/dsp-vec18-int2-trigger.md + this session):

- **The DSP's on-chip timer is NEVER re-enabled — definitively closed.** Raw-opcode + disasm sweep
  of the ENTIRE image (PROM + DROM + every uploaded block/overlay, all store forms stm/stlm/mvdm/
  direct): exactly ONE access to TIM/PRD/TCR exists — loader2's stop `0xD80: stm #0x10,TCR`. So the
  chronological source is NOT DSP-internal. (Old observation re-verified under the corrected ISA.)
- **vec19's ISR `0x241A` = `retf; nop`** — an INERT acknowledge stub in the resident overlay
  (patchable trampoline table 0x2400-0x243F). IMR bit3 is armed for a wake-only tick; it can't arm
  or drain anything.
- **The firmware's ISR-exit idiom proves level-generated hardware.** C54x INT pins are EDGE-SAMPLED
  (SPRS039C 2-FF synchronizer). The 0x3598 ISR exit (0x35F7) pulses port2=0xFFFF then restores —
  forcing the line to deassert and RE-EDGE if requests remain. That sequence is meaningless against
  any hardware except a combinational `(~port2)&port1 != 0` level source feeding an edge-sampled pin
  (the 8259-class request/mask convention; Nokia's own MCU-side FIQ_ACT block uses the same idiom).
- **bit1 of the request word = the glue's MDISND ring-state (tail!=head), not a latched memory bit.**
  This kills the 06-10 ping-pong (empty ring drops the request, so the dequeue-disarm's port2-bit1
  reopen can't re-fire; the next MCU ring post re-edges). It is the DSP-side view of the SAME DSPIF
  ring-pointer comparator that already faithfully raises the MCU FIQs (RINGFIQ).

**Model (`DSP54_CMDLEVEL`, faithful default ON):** assert INT2 (vec18) on the 0->1 transition of
`((~api[0x855]) & (api[0x854] with bit1 := MDISND tail!=head)) & 0x17FF`; deassert on 0. Evaluated
event-driven (DSP port2 writes = mask/ack + exit pulse; MCU window writes to k=0x52..0x55) + sampled
per tick (catches the DSP's own head-advance deassert). **Validated:** exactly 2 edges to drain the
boot backlog organically, no ping-pong, no DARAM corruption; ring head 0x00->0x33; cmd-0x70 reaches
the real builder.

**The self-test chain then validated END-TO-END for the first time — with one correction:** the
builder tail (`call 0x3900` + 2x `0x7f2d` + checksum-zeroing of [0x1207]/[0x120D]) **rewrites
staging words 2+ in place** (the old "0x7f2d writes 0x13DC, not the report" reading was WRONG —
live A/B: staged 0010/0/0/0/0160 left the DSP as BC4E/803B/E300/A5F2/4C1B). On real HW that tail
inserts the live measurement; `DSP54_SELFTEST_MEAS` (faithful default ON) therefore now stages the
nominal idle reading as FINAL WIRE BYTES post-transform ([0x1202..0x1206] in the 0x4B8C-0x4BA7
window, before the MDIRCV enqueue `dcall 0x37CE @0x4BA8`).

**The reason-4 reboot is the firmware's REAL one-shot self-test retry protocol** (RE'd this
session): the self-test post writes marker `[0x10FDDE]=0x5A` (@0x22EC76); the validator's REJECT
path falls into the marker check `0x258CEA` — 0x5A seen -> write 0xA5 + `SET_REBOOT_REASON(4)`
warm reboot (retry); on the rebooted pass the marker is !=0x5A -> write 0x55 -> continue (->
CONTACT SERVICE if the retry also fails). The validator ACCEPT path branches around it (0x258CE8
`bne 0x258D3C`) and writes verdict C4->C0 (@0x22F044, bit2 clear — the organic analogue of the
HLE's faked cmd-13 PASS).

**RESULT — first faithful-path boot through CONTACT SERVICE:**
```bash
# base (one knob) — UNCHANGED at the screen: CONTACT SERVICE @30M (md5 b86d6d5c...), no reset.
DSP54_COSIM=1 ./build/dct3_boot_trace "$FW" 30000000
# + the dispatcher-wake stand-in -> PASSES: "Security code" normal MMI @30M (the FAID boot-lock
#   screen, same state the HLE reaches; md5 4921792d...). tools/check_5110_boot.sh PASS.
DSP54_TICKVEC=25 DSP54_TICKPER=400000 DSP54_COSIM=1 ./build/dct3_boot_trace "$FW" 30000000
```
Timeline under the PASS recipe: edge#1 arms @1.12M -> drain @1.6M -> builder + MEAS @1.63M ->
validator ACCEPT @1.77M (verdict C4->C0, bit6 kept, no marker reboot) -> normal MMI. make test
41+16+35 green; 3310 byte-identical (md5-proven); `DSP54_CMDLEVEL=0` restores the old base exactly.

**Remaining open (the LAST gap to a one-knob through-boot): the dispatcher WAKE.**
> ✅ **CLOSED on branch `dsp-page-chain-decode-fix` (2026-06-12 PM) — see item 0a above.** The
> faithful `0x30A7` sleep loop self-pumps the drain once id 8 is resident; `DSP54_TICKVEC` is
> retired (base == TICKVEC byte-identical). The paragraph below is the pre-resolution analysis
> (correct background for why the stand-in was needed on `main`).

The drain needs
the dispatcher to RUN after the arm; at standby only `[0x866]`bit1 (vec25/HPINT) breaks `idle 1`,
and the MCU's last organic doorbell (1044k) precedes the cmd-0x70 post (1.44M) — base therefore
drains only at the 10.6M retry doorbell, after the 8.63M timeout -> CONTACT SERVICE persists.
`DSP54_TICKVEC=25` (periodic vec25) is an UNFAITHFUL stand-in. ⚠ **UPDATE 2026-06-12
(`docs/research/dsp-dispatcher-wake-findings.md` — full session findings): the "CTSI frame tick"
route is REFUTED.** Exhaustive MMIO audit (0x20000/0x30000/0x40000 blocks, `DSP54_DSPIFAUDIT`)
proves the MCU never programs ANY periodic DSP interrupt; bare ticks on vec16/19/20 FAIL the 30M
oracle (vec19 = retf stub even at runtime, live-dump-verified); RATIO 1..16 changes nothing. The
designed steady-state wake is the **SLEEP loop `0x30A7`** (INTM=1 idle via trampoline
`0x2464→0x1CA4` + UNCONDITIONAL dispatcher call `0x30BA` on every inline interrupt resume — any
armed vector, stub or not, pumps the dispatcher there) — but the sleep overlay at `0x1CA4` is
EMPTY at boot (never demand-paged) and `0x30A7` is never reached in 30M. The boot park is the
restart-sequencer inner idle (`0x31A4`, bit1-only gate). Remaining faithful candidates:
(1) more MCU flag=1 API-command traffic in the boot window on real boots (each = vec25 doorbell;
the sender `DSP_APICMD_SEND 0x271D70` delivers flag=1 via [0x870]+doorbell, flag=0 via the
POLLED MDISND ring — nothing is "dropped", the ring is polled by design), (2) a longer
demand-page chain on real HW (each [0x872]=4 GO posts the only resident task → busy-poll window
whose mask-3h gate `0x308F` drains on the CMDLEVEL arm alone). ALSO open: warm-reboot DSP
re-staging (boot#2 after a firmware warm reboot never re-stages the DSP -> splash hang; visible
under CMDLEVEL with MEAS=0).

---

## Key addresses (5110 NSE-1 v5.30 A)

**Boot blocks** (flash descriptor `[6-word BE: entry,ctrl,size,dest,len,0][payload @desc+0xC]`;
run addr = DSP `0x800+dest`):

| block | flash descriptor | dest | DSP run addr | payload[0] | role |
|-------|------------------|------|--------------|-----------|------|
| 25 loader1 | `0x002996F8` | `0x700` | `0xF00` | `0xF7BB` | CRC-gate / upload verifier |
| 0 IVT | `0x002948EC` | `0x500` | `0xD00` | `0xFC00` | vectors / branch table |
| 18 loader2 | `0x00298934` | `0x580` | `0xD80` | `0x7726` | stage-2 |

**MCU side:**
- DSP reset control: `0x20002` bit0 (0→1 release). Adjacent: `0x20003`. The inline warm-boot fn
  `0x271BC8` (holds → clears+rehomes → writes the doomed pre-release `[0x872]=2` @`0x271D1A`) drives
  reset via `0x20008`–`0x2000B`.
- DSPINT doorbell (host→DSP): MCU `0x30000` bit2 = HPIC DSPINT (firmware-verified PC `0x272048`).
- **HINT doorbell (DSP→host): DSP `BSCR 0x29` bit3 toggle → MCU IRQ4** (`irq_pending` bit4), handler
  `0x2BCEF0` (DSP mailbox) — delivers the responsive run-mode go-command `[0x872]=2`.
- loader1 stage-0 copy: PC `0x271A66` → ARM copy `0x2936E4` → window `0x10E00`.
- Bulk upload loop: `0x271B00` (block counter `0..63`), bufA `0x10200`/bufB `0x10600`, flash src `0x200040+`.

**Warm-boot bootstrap (MCU-rehomed into the wiped loader1 slot):** DSP `0xF00` (payload[0]=`0x7718`
`stm #..,sp`) — CPU init + IVT relocate + ready-doorbell; parks at `0xF3D` (poll `[0x872]`); on
go-command branches `0xF3B: b 0x0D80` → loader2 proper (`0xD80`, block 18) → run-mode `0x3000`.

**HPI window** (`MCU 0x10000+2k ↔ DSP 0x800+k`):

| field | DSP | MCU | dir |
|-------|-----|-----|-----|
| version / boot-status | `0x802`/`0x803` | `0x10004`/`0x10006` | D↔M |
| UPLOADHEADER (5-word) | `0x87B–0x87F` | `0x100F6–0x100FE` | M→D |
| MBOX0 / MBOX1 | `0x87F` / `0x880` | `0x100FE` / `0x10100` | D↔M |
| bufA / bufB | `0x900` / `0xB00` | `0x10200` / `0x10600` | M→D |

**DSP run-mode:** reset vector `0xFF80`; superloop entry `0x3000`; **steady-state idle `0x31A4`**
(`idle 1`; inner loop `0x319D–0x31A5`, work flag `[0x866]`bit1, idle-exit task test `bitt [0x6D1]`
@`0x31AE`); superloop heartbeat enqueue `0x314D`; work dispatcher `0x390A` → MDISND processor
`0x3924`; MDIRCV producer/enqueue `0x37CE`; CRC routine `0x8023`. (`0x408C`/`0x407F` were the
*broken-path* idle — pre-SEEDDARAM artifact.)

**Run-mode interrupt map (IVT 0xFF80, IPTR=0x1FF; live IMR=`0x52FD`):**

| vec | vector addr | ISR | role |
|-----|-------------|-----|------|
| 16 (INT0) | `0xFFC0` | `0x3204` | codec/audio FIR frame ISR (reads pa33/pa39; modes `[0xAA]`/`[0xAC]`) |
| 18 (INT2) | `0xFFC8` | `0x3598` | legacy/boot host-cmd ISR (`0x35B9` dispatch) |
| 19 | `0xFFCC` | `0x241A` | DARAM dispatcher (IMR bit3 enabled; source = MAD1 system logic, NOT the TI timer — fw stops it) |
| 20 | `0xFFD0` | `0x3416` | serial/RINT |
| **25** | **`0xFFE4`** | **`0x3772`** | **the run-mode host doorbell: `[0x866]\|=2` + `[0x870]` mailbox (`0xA51B`, cmd `[0x85D]&0x1F`) + HINT ack** |

---

## Env knobs

**Faithful (default ON under `DSP54_COSIM`, override with `=0`):** **`CMDLEVEL`** (2026-06-11: the
MAD-glue INT2 level comparator — `(~port2)&port1` with bit1 = MDISND tail!=head, edge-delivered;
THE organic MDISND dequeue-arm), **`SELFTEST_MEAS`** (nominal idle measurement staged post-transform
as final wire bytes — the un-stimulated front-end's no-signal reading), `REALUP`, `RATIO=4`, `REALUPLOAD`,
`DSPNOSELFTEST`, **`HINTIRQ4`** (the BSCR `0x29` bit3 doorbell → MCU IRQ4 wake), **`SEEDDARAM`**
(reload the resident DARAM overlay `prog[0x2000–0x2800]` at run-mode entry — the SP-leak fix),
**`TIMER`** (C54x on-chip timer counting + TINT, SPRU131G §8.8 — fw stops it at loader2, so inert
in practice; kept for fidelity), **`NOXPC`** (C542-class 1-word IRQ frames), **`DSP54_INTVEC`
default 25** (the run-mode host doorbell ISR `0x3772`; vec 17/18 are wrong — INT1 is IMR-masked in
run-mode), **`REALSTATUS`** (version/boot-status reads from the real DSP — the old HLE exception
retired) (+ the now-no-op `UNIDARAM`/`SHAREDWIN`/`HPIAPI`).

**HLE fully silenced under cosim (2026-06-11, audited):** `dsp_hle_quiet` (set automatically under
`DSP54_COSIM`; `DSP54_HLE=1` opts the legacy model back in for A/B) returns `dsp_default_tick`
right after the `dsp_steps` counter — the block-ack pump + its **IRQ4 raise**, the Cobba command
auto-consume, the self-test responder, the DSPMSG injector and the keep-alive **FIQ0 stream** are
all dead. Signal census after the audit: the ONLY DSP→MCU signal paths are the real HINT doorbell
(`dsp54_hint_edges` → IRQ4, handler `0x2BCEF0`) and env-gated diagnostics (`DSP54_FIQ0OUT`,
`T0IRQ4` — both default OFF). The HLE write path survives as passive bookkeeping only (mbox echo
counters; invisible — all window reads serve from api_ram). **Every signal the MCU sees from the
DSP is now generated by the real core.**

**Diagnostics (read-only, off by default):**
- **`DSP54_COMMLOG=1`** — unified bidirectional MCU↔DSP comms trace: one decoded, PC-tagged line per
  signalling event (reset/hold/release, DSPINT + HINT doorbells, MDISND/MDIRCV rings, mailbox/version
  handshake). `M->D` carries the exact **MCU PC**; `M<-D` the **DSP PC**. `=2` also dumps the bulk
  code-upload + MBOX ping-pong + ring payload. The fastest way to *see the whole conversation*.
- `DSP54_HSLOG` — MCU↔DSP mailbox state machine (MBOX/status/cmd/version, both views, coherence).
- `DSP54_HSHALT=0xDSPPC` (+`DSP54_HSSTOP`) — freeze + dump combined state at a DSP PC.
- `DSP54_PLACELOG` — tag GLUE writes to block run-homes (proves the MCU, not an HLE, places blocks).
- `DSP54_HOMELOG` — timestamp MCU writes to the run-homes (proves the `@923k` rehome).
- `DSP54_HOLDLOG` (+`HOLDLO`/`HOLDHI`) — every shared-window write in the hold→release window.
- `DSP54_F00DUMP` — ground-truth dump of the rehomed `0xF00` (loader2) + `0xD00` (IVT) at warm reset.
- `DSP54_GOTEST[=val]` — A/B probe: re-deliver `[0x872]` at the `0xF3D` park (proves the wait responds).
- `DSP54_PCTRACE=<dsp_step>` (+`PCTRACEN`) — dump N consecutive DSP PCs from a step.
- `DSP54_LOG` — reset/release/hold/warm events + the WARM-BOOT block-homes peek + `HINT→IRQ4` strobes.

**COBBA chip model (default OFF; 2026-06-12):** `DSP54_COBBA` = serial register interface (port
0x2C reg-select bit4=R/W, port 0x2D 12-bit value; nominal reg5=0x160/reg6=0x010,
`DSP54_COBBA_REG<n>` overrides) + codec frame clock + codec ports. **`DSP54_COBBAPAR`** = the
parallel **MFI** half (implied by `DSP54_COBBA`, `=0` opts out): mmr 0x22/0x32 `0xC0xx` control
frames (4-bit COBBA reg addr + 12-bit data) decoded via the core hook `g_c54x_mfi_write_cb`,
plus the datapath ports — port(0Dh) 14-bit AGC ADC (nominal 0x160, `DSP54_COBBA_AGC`), 0x38
status=ready, 0x39 RX I/Q (`DSP54_COBBA_IQ`), 0x3A/0x3B meas readback (`DSP54_COBBA_MEASA/B`),
0x30-0x3C write latch. `DSP54_COBBALOG` traces both halves. The sub-0x13 measurement tail
(serial regs 5/6 + the parallel AGC at DSP `0x4B0D`, packed `0xAA00|AGC&0xFF` → MDIRCV) runs
end-to-end against the model under `DSP54_INJTONE=1`. COBBA does NOT gate boot (sub-0x16 is the
boot path; see the handoff §3 corrections).

**Peripheral surface map (`DSP54_PERIPHMAP`, default OFF, read-only; 2026-06-12):** the reusable
"what lives where + what we silently fake" tool. Instruments BOTH DSP peripheral surfaces at the
choke points — I/O port space (`PORTR`/`PORTW`, the `mad2_dsp_c54x.c` hooks) and the memory-mapped
band `data[0x00-0x5F]` (CPU MMRs + on-chip peripherals: timer, BSCR `0x29`, MFI `0x22/0x32`, McBSP
`0x38/0x39`, DMA, CLKMD `0x58`). Each row = `addr | R/W | first_cycle | hits | modeled? | values |
top PCs`, dumped sorted at exit. **`modeled?` is STRUCTURAL** (`yes`/`NO`/`part`): an explicit
handler case sets the flag; backing-store fall-through = `NO` (a real reg reading 0 is not "fake").
`DSP54_PERIPHMAP=1` = aggregate; `DSP54_PERIPHMAP_LO/_HI=<insn>` and/or `DSP54_PERIPHMAP_PA=0xNN`
add a raw per-access log inside a window. The base run flags the un-modeled ports (the COBBA
parallel `0x04-0x2B` bring-up writes, the `0x24` TX burst) — useful for spotting the next CLKMD-
class hidden register before it costs a session. Full how-to: `docs/5110-peripheral-surface-HANDOFF.md`.

**Run-mode crutches (⚠ temporary, explicit only):** `DSP54_MMR58PER` (toggle the unmodelled MMR[0x58]
frame clock), `DSP54_FRAMEINT=16` / `DSP54_FRAMEPER` (synthetic codec frame INT0 → ISR `0x3204`).
`DSP54_WARMENTRY=0xNNNN` — override the warm-reset PC (default `0xFF80`, the faithful RS vector);
A/B-only now that the faithful warm path reaches run-mode.

---

## Hardware architecture — grounded against the 5110 service manual (2026-06-11)

Source: `ref/nokia_5110_mobil_telefon_service_manual/` (NSE-1 PAMS, 03/98) + the LC542 datasheet
(SPRS039C) + SPRU131G. These **correct several standing project assumptions**:

- **It is MAD2, not "MAD1".** The NSE-1 service manual (`03sys.pdf` p3-1) states the baseband =
  **four ASICs: CHAPS, CCONT, COBBA-GJ, and MAD2**, and that *"the MCU, DSP and Logic control
  functions [are] in [the] MAD ASIC"* — i.e. **MCU + DSP are integrated on one die**, not discrete.
  (Our "MAD1" label is informal; the DSP is a C542-class LEAD core inside MAD.) Engine-schematic
  confirmation (`up8sa3.pdf` net list): there is **no `DSPINT`/`HINT`/`INTn` net on the board** — a
  discrete DSP would have to expose them. The only DSP-adjacent external net is **`DSPXF`** (the C54x
  XF *output* flag, to COBBA); all DSP interrupts are generated **on-die** (by CTSI — see below).
- **The DSP interrupt source is therefore NOT in any repair doc** — it's internal to MAD2. The MAD
  System Logic block that generates it is **CTSI (Clock, Timing, Sleep and Interrupt control)**;
  **DSPIF** = the MCU↔DSP interface; **API** memory = the shared `api_ram` window + the `0xD00` DSP
  IVT; **MFI** = the COBBA AD/DA datapath. (Block list from the MAD2WD1 sibling doc, NHM-5NX
  `sysmod.pdf` p29; same architecture family.) We model CTSI's **MCU-side** timers (`mad2_timers.c`,
  FIQ5/FIQ8, 33055 Hz) but **not** a CTSI→DSP frame interrupt — that is the missing-stimulus gap.
- **COBBA↔MAD is external + documented** (`03sys.pdf` p3-2): *"a parallel connection for high-speed
  signalling and a serial connection for PCM coded audio"* (the 4-wire PCM serial = the DSP McBSP
  frame path → vec16/`0x3204` audio ISR). Schematic nets: `COBBACLK/COBBARDX/COBBAWRX/COBBADAX/
  COBBARSTX` (strobed parallel control bus) + `DSPXF`. The baseband/frame *timing into MAD* is on the
  schematic; how CTSI routes it to a DSP vector is on-die.
- **RF = PLUSSA (VHF/PLL) + CRFU (front end)** on the 5110 — **not Hagar** (a later-model RF ASIC).
- **Keypad tones are MAD-generated → COBBA** (*"Keypad tones, DTMF and other audio tones are
  generated and encoded by the MAD and transmitted to the COBBA"*) — confirms the RF-free stimulus
  path to spin the DSP superloop.
- **CCONT register map** (vs MADos `hw/ccont.c` + our `mad2_ccont.c`): we semantically model 0x00
  (ADC sel), 0x02/0x03 (AD value + chip-ID `0xB0`), 0x05 (WDT), 0x07-0x0A (RTC), 0x0E/0x0F (int
  pending/mask). Unmodelled **semantics**: 0x01 (charger PWM), 0x04/0x06 (control; MADos inits
  `0x20`/`0x54`), 0x00 bit7 (charger enable) — all **charger/regulator domain**. A generic
  `m->ccont[reg]` shadow gives all 16 regs read/write-back, so firmware I/O is consistent; only the
  *analog side effects* (charge current, regulator enables) are unmodelled — none on the boot path.

**Consequence:** the DSP frame/INT2 source question is **docs-exhausted** — repair manuals + schematics
can't show an on-die signal. The two remaining routes are RE: (a) trace how the MCU firmware programs
the CTSI/DSPIF block (`0x20000` MMIO) to arm the DSP interrupt(s); (b) empirically test which DSP
vector a CTSI-paced tick must hit to make the superloop iterate and drain MDISND (the `DSP54_TICKVEC`
probe, understood as a *CTSI-frame* stand-in — NOT a fake doorbell). Recall the MDISND queue is
**polled** (back-pressure-only `FIQ_MDISND`); the doorbell (`[0x30000]`/HPINT/vec25) is for urgent
API commands. See `docs/research/dsp-{vec16-frame,vec18-int2-trigger,superloop-taskready,frame-cadence-reference}.md`.

---

## TI references (on hand)

- **SPRU131G** *C54x CPU and Peripherals* — §3 memory/OVLY, §8 HPI (HPIC `0x2C`, DSPINT/HINT),
  §6.11 IDLE/INTM semantics (INTM=1 → an interrupt resumes IDLE *inline*, never vectors — this is
  why loader1's INTM=1 park can't be DSPINT-woken). Reset vector (RS) always `0xFF80`.
  Note: the §8.6.4 host-int-vector indirection is NOT this build's wake path — the wake is RS into
  the MCU-rehomed loader2 + the BSCR `0x29` bit3 → IRQ4 doorbell handshake.
- **SPRU288** *C548/C549 Bootloader* — boot-mode selection (HPI boot = INT2 → branch to HPI RAM
  start; other modes read entry point XPC+PC from the boot table's last 2 words).
- `ref/tms320lc542.pdf` (SPRS039C). MAD1-specific values are RE-verified inline.
