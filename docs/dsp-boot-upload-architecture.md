# Nokia DCT3 DSP boot & code-upload architecture (TMS320C54x "LEAD")

> ⚠️ **PARTIALLY SUPERSEDED (2026-06-08).** The "loader1 consumes the upload / scatters blocks to
> dest" framing in places below is the **wrong premise** — loader1 is a CRC-gate over a fixed
> double-buffer (`0x900`/`0xB00`); it does not scatter or relocate. For the corrected, verified boot
> choreography read **`docs/5110-dsp-boot-sequence.md`** + the CORRECTED BOOT MODEL in
> **`docs/5110-DSP-CANONICAL-STATE.md`**. Kept here for the RE detail (disasm, addresses, provenance).

> **Firmware pinned:** Nokia 5110 NSE-1 v5.30 A (MAD1). The MCU↔DSP protocol is shared across
> DCT3 (3310/MAD2 reconciles — see `docs/dsp-blocks-re-3310.md`); addresses below are 5110 v5.30
> unless noted. **Status: reverse-engineered.** This document is the authoritative end-to-end
> account of how the MCU boots the DSP. Most of it is not documented anywhere public — see
> **Provenance** at the end for what is prior art vs. newly RE'd here.

The Nokia DCT3 baseband DSP is a TI TMS320C54x core (Nokia "LEAD"). It has a small **resident
mask-ROM/PROM** (dumped: program `0x3000-0x7FFF` + high vectors `0xE000-0xFFFF`, plus an on-chip
DROM) but its **application code is not resident** — the MCU **uploads** it into the DSP's on-chip
DARAM over the HPI shared-memory window at every boot. The upload is **demand-paged**: far more
code blocks exist than DARAM can hold at once, so blocks that share a run-address are loaded on
demand as the DSP switches task/codec.

---

## 1. The shared-memory window (HPI)

The MCU sees the DSP through a 0x1000-byte / **0x800-word** window at **MCU `0x10000`**
(byte address = `0x10000 + word*2`). The DSP sees the same cells at **DSP data `0x800 + word`**
(PMST.OVLY=1 aliases program/data for `[0x80,0x2800)`, so uploaded code both lands in and executes
from this region). Key cells (word index | MCU addr):

| symbol | word | MCU addr | dir | meaning |
|--------|------|----------|-----|---------|
| `MDISND_QUEUE` | 0x000 | 0x10000 | M→D | send ring (size 0x52) |
| `MDISND_TAIL/HEAD` | 0x052/0x053 | 0x100A4/0x100A6 | — | send ring ptrs |
| `UPLOADREQUEST` | 0x071 | 0x100E2 | D→M | "(ready / block #)" |
| `UPLOADREPLY` | 0x072 | 0x100E4 | M→D | `0x0002`=MORE, `0x0004`=FINISHED |
| `UPLOADHEADER` | 0x07B–0x07F | 0x100F6–0x100FE | M→D | **5-word block descriptor** (see §3) |
| staging buffer | 0x100… | 0x10200… | M→D | **block payload landing zone** (§4) |
| `MDIRCV_QUEUE` | 0x080 | 0x10100 | D→M | receive ring (size 0x64) |
| `MDIRCV_TAIL/HEAD` | 0x0E4/0x0E5 | 0x101C8/0x101CA | — | receive ring ptrs |

Doorbells: **MCU→DSP** = write `[0x30000] |= 4` (DSPINT, bit2) after staging a command;
**DSP→MCU** = `*(0x29) |= #8` then `&= #0xFFF7` (BSCR bit3 repurposed = HINT → MCU IRQ4).

The `dsp_uploaded` complete-flag is **`[0x10A9E4]`** (3310 analog `[0x11038C]`); the boot
subsystem-starter polls it and bails (app/MMI never launches) if still 0.

---

## 2. Reset & boot entry

The DSP reset vector lives in resident ROM (dumped):

```
0xFF80  PMST = #0xFFA8        ; OVLY=1, IPTR etc.   (#-88 = 0xFFA8)
0xFF82  goto 0xFF85
0xFF85  goto 0x0F00           ; -> loader1
```

So reset jumps straight to **loader1 @ `0x0F00`** — but **`0x0F00` is itself an uploaded block**
(block25). On silicon the **HPI hardware boot** loads the base set into DARAM *before* the DSP is
released from reset; only then does the `goto 0x0F00` land on real code. (In the dump, `0x0F00`
reads as zero — loader1 had been freed by capture time; the resident idle is `0x408C = idle(1)`.)

**Co-sim consequence:** releasing the modelled DSP at `0x0F00` *before* the base set is uploaded
makes it slide through empty DARAM into the resident ROM and bounce between `idle(1)` and the
bist/vocoder region. The faithful model **holds the DSP until the boot upload completes, seeds the
base set, then enters `0x0F00`.**

---

## 3. The block descriptor (6 words; first 5 == UPLOADHEADER)

The firmware holds a table of **6-word descriptors** (one per block; pointer chain from
`[0x10A9E4]` → table `0x293B2C` → e.g. block0 desc `@0x2940AC`; payloads at separate flash
addresses, "our_mcu" in `re/dsp-5110/block_map.json`). Format **verified** by matching the live
`UPLOADHEADER` writes (`DSP54_HDRLOG`) to the table:

```
word0 = entry   ; run address (post-relocation) the block executes at
word1 = ctrl    ; control/flags
word2 = size    ; payload length in words
word3 = dest    ; HPI-window word offset it uploads to  (DSP addr = 0x800 + dest)
word4 = len     ; MCU-side byte/record length
word5 = 0x0000  ; terminator
```

Worked examples (`dsp_blocks.txt`, confirmed against `block_map.json` + HDRLOG):

| block | entry | ctrl | size | dest | DSP addr (0x800+dest) | role |
|-------|-------|------|------|------|------------------------|------|
| 0  | 0xFD00 | 0xFF80 | 0x244 | 0x500 | 0x0D00 | IVT + **branch-table source** |
| 25 | 0x0F00 | 0x0000 | 0x0BA | 0x700 | 0x0F00 | **loader1** |
| 18 | 0x0D80 | 0x1000 | 0x122 | 0x580 | 0x0D80 | **loader2** |
| 3  | 0x0926 | 0x0B39 | 0x451 | 0x126 | 0x0926 | **frame scheduler** |
| 1  | 0x25B4 | 0x1F80 | 0x34E | 0x130 | 0x0930 | code (relocates) |

When `entry == 0x800 + dest` the block **runs in place** in the window (loader1, loader2,
scheduler). When they differ (block0 → IVT `0xFD00`; block1 → `0x25B4`) the block is **relocated**
from the window to `entry` by the loaders.

---

## 4. The wire protocol — two phases (measured)

Observed live on a clean boot via `DSP54_UPLOADLOG` / `DSP54_HDRLOG` / `DSP54_WRSCAN`:

**Phase 1 — bulk payload DMA (~dsp_steps 28k–324k).** ~33 transfers, each prefixed by a header
`[offset=0x100, len=0x300, 0, flags=0x8000, seq]` then **768 words streamed to the staging buffer
at word `0x100`**. The payloads are real C54x code (`0xE3A0 0xE10F 0xE129 …`). All traffic rides the
`0x10000-0x10FFF` window — **nothing is written above `0x11000`** (`DSP54_HIWIN` = silent), so the
window, not a wide aperture, is the whole upload channel. The resident loader scatters each chunk
out of staging to its destination.

**Phase 2 — finalize descriptors (~369k–398k).** A few real block descriptors posted to
`UPLOADHEADER` (block0, block1…) with `UPLOADREPLY` stepping `0x00 → 0x02 (MORE) → 0x04 (FINISHED)`.

**It is MCU-push and self-terminating, NOT DSP-pull at boot.** A one-off experiment (since
reverted) that forced `UPLOADREQUEST` through `0,1,2,…` instead of the HLE's fixed `0x01` produced
**byte-identical** output — the firmware ignores the request word and writes `FINISHED` on its own
after the boot set. So the boot upload is **complete, not truncated**: it loads the **base set**
and stops. (The runtime on-demand pulls in §5 are a separate, post-boot mechanism.)

---

## 5. Demand-paged overlays (the key architectural finding)

The 28 blocks do **not** all fit in DARAM, and many **share a run address**. Grouping the
descriptors by `entry`:

| run addr (entry) | blocks | nature |
|---|---|---|
| `0xF00` / `0xD80` / `0xFD00` | 25 / 18 / 0 | loader1 / loader2 / IVT — unique |
| `0x25B4` / `0x1536` / `0x1CA4` / `0x9AA` | 1 / 6 / 8 / 10 | unique code/data |
| **`0x926`** | **3,4,5,7,9,12,13,14,15** | **9 alternates** — one resident at a time |
| **`0x2286`** | **19,20,21,22,23,24** | **6 alternates** |
| **`0x216A`** | **2,11,26** | **3 alternates** |

You **physically cannot** hold all 9 `0x926` blocks at once — they are **paged overlays**, almost
certainly the alternate GSM speech-codec / algorithm modes, each pulled on demand (via
`UPLOADREQUEST`) by the **running** DSP when it switches task/codec (e.g. when a call starts). This
is why a single static DARAM image always "tops out," and why the boot upload loads only a base set.

**Boot needs only the base set** (loaders + IVT + one scheduler page + data); the alternate pages
load later, at runtime, during a call. Reaching the DSP's idle/scheduler — our current goal —
therefore does **not** require modelling runtime paging.

---

## 6. End-to-end boot sequence (synthesis)

```
MCU power-on
  │  HPI hardware boot: stream base-set blocks into DARAM window (Phase 1 bulk DMA),
  │     loaders scatter staging(0x100) -> each block's dest
  │  post finalize descriptors (Phase 2), UPLOADREPLY=FINISHED, set [0x10A9E4]
  ▼
DSP released from reset -> 0xFF80 -> 0x0F00 (loader1, now present)
  loader1: set SP/PMST/SWWSR, clear DARAM, BUILD branch table by copying the
           branch-table SOURCE (block0 @ DSP 0xD00) -> 0x23C4, relocate IVT -> 0xFD00,
           signal ready [0x871]=0x12, wait on UPLOADREPLY [0x872]
  ▼
loader2 @ 0xD80 (run-mode init: IMR, vectors)
  ▼
run-mode dispatch -> branch table -> frame scheduler @ 0x926 -> idle(1) @ 0x408C
  ▼
runtime: DSP pulls codec/overlay pages on demand (UPLOADREQUEST=block#),
         produces MDIRCV traffic, services host commands (DSPINT/IRQ4/FIQ0)
```

---

## 6a. loader1 IS the upload consumer — how we model it faithfully (`DSP54_REALUP`)

**Do not relitigate this.** The DSP-side upload consumer is **loader1 itself** (block25), running on
the real C54x core — *not* something the emulator scatters. Verified by disassembling `block25.bin`
(the real loader; full.s54's `loader1.s54` is a different/inaccurate reconstruction — block25
starts `f7bb (INTM=1), 7718 1ec6 (SP), 771d ffa8 (PMST), …`). At `0xF00` it sets up, calls a
subroutine, then runs its consume loop:

```
0xF15  f074 0f6d   call 0xF6D
0xF17  10f8 087f   A = *(0x87F)         ; poll UPLOADHEADER word 0x7F (DSP 0x87F = api_ram[0x7F])
0xF19  f844 0f17   if(cond) goto 0xF17  ; wait for the MCU to post the next block
```

So the faithful co-sim model (knob `DSP54_REALUP=1`, default-OFF, native co-sim only) is:

1. **Seed ONLY loader1** (`block25.bin` → DSP `0xF00`) at init — models the HPI hardware boot that
   places the loader into DARAM before the DSP is released from reset. Nothing else is pre-placed.
2. **Do NOT hold the DSP** and **do NOT manually scatter** — loader1 runs from t=0 and does the
   whole base-set scatter itself in real DSP code.
3. **Bridge the MCU's upload-region writes into `api_ram`**: words `0x71-0x7F`
   (UPLOADREQUEST/REPLY/HEADER) and `0x100-0x7FF` (staging). The DSP reads the window via `api_ram`
   but MCU writes default to `data[]`; without this bridge loader1 polls a stale `api_ram[0x7F]`
   forever. Scoped to the upload region so it doesn't disturb the MDISND/MDIRCV rings.

**A held-then-FINISHED-then-manual-scatter model is WRONG** (an earlier attempt): it starts loader1
*after* the MCU has finished pushing, so loader1's consume loop waits forever. loader1 must run
*during* the upload. This is settled — do not re-attempt manual scatter or DSP-hold.

### 6b. The MCU drives DSP reset via IO `0x20002` — gate the co-sim DSP on it (BREAKTHROUGH)

**The MCU holds the DSP in reset during upload and releases it via IO when upload completes.** The
DSP-reset-control register is **MMIO `0x20002`, bit0 = release** (read-back `0x53` running on 3310;
the same routine that uploads blocks, `0x271D70`, owns this constant). Measured timing
(`DSP54_REALUP DSP54_LOG`):
```
@28k   0x20002<=0x01   release (Phase-1 bulk upload begins, DSP runs)
@327k  0x20002<=0x00   RESET asserted (DSP held)
@397k  0x20002<=0x01   RELEASE  == upload FINISHED  ← the real boot moment
@3454k 0x20002<=0x03   post-boot
```
So the faithful co-sim model gates the DSP on this real signal: **held (not stepped) while bit0=0;
on the 0->1 release edge enter the RESET VECTOR `0xFF80`** (NOT `0xF00` directly — `0xFF80` runs
`PMST=#0xFFA8`, OVLY=1, then `goto 0xF00`; jumping straight to `0xF00` leaves PMST stale and the
DSP diverges). **The DSP knows its entry purely from the reset vector — it does not choose; every
release re-enters `0xFF80`→loader1.** The warm reset is a PC change only — **DARAM (the uploaded
blocks) is preserved** across it (verified: loader1 `0xF00`=`f7bb 7718` intact at 396k/398k/600k),
faithful to the C54x (reset does not clear on-chip RAM).

**RESULT (2026-06-08) — first fully faithful boot to idle.** `DSP54_REALUP=1` (+ `DSP54_HPIAPI=1`):
real MCU upload → `0x20002` release → reset vector → loader1 → **resident idle `0x408C` ~95%**,
`bist` ~3% (was 41%+), producer task bits `[0x6E4]=0xF468` set (incl. bit6/0x40, the MDIRCV
keystone). No static overlay, no LOADERGO, no manual scatter, no hold hack. `0x408C` is the
resident host-wait idle — the boot-complete steady state.

**Host-command interrupt = INT1 (vec 17), NOT INT2 (vec 18) — 2026-06-08.** Under the faithful
boot the DSP idles with `IMR=0x7122` (bit1/INT1 enabled, **bit2/INT2 masked**), so the DSPINT
doorbell must be delivered as **INT1 (vec 17)**; vec 18 is never serviced (`hostISR3598=0` forever,
no matter how long you run). Baked into `DSP54_REALUP` (`g_intvec` defaults to 17). With it the DSP
**services host commands** (`hostISR3598=6 hostCmd35B9=6`). (Found by sweeping the `IMR`-enabled
vectors: 17 services; 18 = 0; 21 diverges = the codec int; 24 idles but doesn't service.)

**Current steady state (faithful boot + vec 17): the DSP is ALIVE and busy — not stuck.** Boots →
services host commands → settles into a periodic state: idle `0x408C` ~32%, working ~68%. A 20M-step
PC histogram makes `0x44E4-0x44EB` look like a hot loop (1.83M each), but a live instruction trace
at 8M shows **varied work** (`0x1200-0x1219`, `0x44xx`, …) — so `0x44E4` is just *frequently
revisited*, not a tight spin; the "hot loop" was 16-bit-PC histogram aliasing (XPC banking — only
16 port reads occur all run, none at `0x44E4`). The DSP cycles through real processing. Running to
100M adds nothing — steady state by ~10M. (`Dsp54Status` now carries `xpc` for future banked-PC RE.)

**Post-upload command activity + the boot self-test (2026-06-08, faithfully resolved).** The MCU
DOES send post-upload commands — both a DSPINT host-command stream AND an **MDISND ring stream**
(MCU→DSP, `word0=0x0A05` = size 10 / type 5, from ~577k). The DSP **services the DSPINT host-cmds**
(vec 17, `isr35B9`) but **never dequeues the MDISND ring** (`dequeue3860=0`). The **boot self-test
DID run and emit 2 real MDIRCV messages** — confirmed real (not the `dsp_default` HLE) by
`DSPNOSELFTEST=1`: `isr35B9` drops 6→1 but `enq37CE=2` is unchanged. The producer keystone
`[0x6E4]&0x40` (and `[0x866]&1`) are set by the DSP's OWN boot init (`h3621=0 h363C=0` — NOT by host
commands).

**CORRECTION (2026-06-08) — the real DSP does NOT post to the MDIRCV ring at boot.** Earlier text
said "the boot self-test emits 2 real MDIRCV messages." That was a MISREAD: `enq37CE=2` is PC hits
at the enqueue routine, NOT ring posts. A/B with `DSPNOSELFTEST=1` proves it: the MCU drains the
ring (`MDIRCV_HEAD 0x80→0x83`) ONLY with the HLE self-test responder ON; with it OFF the MDIRCV
**tail `0x101C8` stays `0x80`** (never advances) and the MCU consumes nothing (`head` stays `0x80`,
`r=0`). So the messages the MCU consumes are entirely `dsp_default`'s HLE, not the real DSP. The
real DSP does write `port(1)=0x0100 @PC 0xB7E1`, but it is NOT a validated doorbell (raising FIQ0 on
it hits an empty ring). **Real DSP→MCU MDIRCV delivery is an open gap** — consistent with the
producer/task-scheduler being dormant at idle. (The cluster-reframe's `port(1)=1 @0x3807` doorbell
was a forced-frame artifact; the idle boot path uses neither.)

**Conclusion (R1 faithfully answered).** Boot → self-test (2 MDIRCV) → **idle quiescence**.
Everything up to the boot self-test works. Sustained idle MDIRCV is NOT a pokeable bug: the
producer/MDISND-consumer lives in the uploaded **task scheduler `0x926`**, which is **dormant at
idle by design** (`idleGate0A0C=0`) — it runs on frame/codec activity (a live call), not elapsed
time, and not on the host commands the DSP services. The keystone bits are armed; nothing drives the
consumer at idle. This reproduces `5110-dsp-cluster-reframe` on a fully faithful substrate: the
reason-0x68 keep-alive (sustained idle MDIRCV) requires **call/frame activation**. The decisive next
test is to activate a call and confirm the scheduler then runs → dequeues MDISND → sustains MDIRCV.

## 7. Provenance (what is prior art vs. RE'd here)

- **Prior art (external):** the raw DSP image + IDA database + `dsp_blocks.txt` descriptor decode +
  `Task info.txt` RTOS notes — from the ~2003–2004 `sss` analyst dump (`re/dsp-5110/raw/`,
  `~/Downloads/sss_extract/`). The descriptor *table contents* are theirs.
- **Prior art (adjacent):** MADos (open-source, DCT4/Calypso era — a *different* DSP generation)
  gave the symbol names for the protocol contract; reconciled in `docs/dsp-blocks-re-3310.md`.
- **Reverse-engineered in this project:** that the protocol applies to 5110/MAD1; the live two-phase
  wire behaviour; that `UPLOADHEADER` == the 5-word descriptor; the fixed staging at word `0x100`;
  **MCU-push / self-terminating / boot-complete-not-truncated**; the **demand-paged overlay
  architecture** (blocks sharing run-slots); the base-set vs on-demand split; the reset→loader1
  entry and the held-DSP boot model. None of this end-to-end account is, to our knowledge, public.

## 8. How to reproduce the observations

```bash
FW="firmware/Nokia 5110 NSE-1 v5.30 A.fls"
# where payloads land + the descriptor headers (the protocol, live):
DSP54_COSIM=1 DSP54_HPIAPI=1 DSP54_LOG=1 DSP54_UPLOADLOG=1 DSP54_HDRLOG=1 \
  ./build/dct3_boot_trace "$FW" 30000000 2>&1 | grep -E 'HDRLOG|FIRST-TOUCH'
# descriptor table / payloads:  re/dsp-5110/block_map.json + block%02d.bin
# (push-vs-pull was settled by a reverted experiment that advanced UPLOADREQUEST 0,1,2,… ->
#  byte-identical output -> MCU-push; re-add that probe to dsp_default.c if you need to re-verify)
```

See also: `docs/dsp-blocks-re-3310.md` (protocol contract), `docs/dsp-5110-mad1.md`
(chronological RE log §8), memory `5110-dsp-staged-upload-pivot`.
