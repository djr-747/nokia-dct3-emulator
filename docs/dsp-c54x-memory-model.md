# DSP C54x memory model — the faithful contract (TI-grounded)

> **Purpose.** Ground the 5110/MAD1 DSP co-sim's memory model in the **TI TMS320C54x
> specification** (SPRU131G *CPU and Peripherals*; SPRS039C *TMS320LC54x datasheet*) instead of
> the empirically-accreted three-store + bridge model. This document is the **contract** the
> co-sim memory should implement. Where it disagrees with a knob (`DSP54_SHAREDWIN`,
> `DSP54_HPIAPI`, `DSP54_CMDBRIDGE`, the `data[]`/`api_ram[]` split), **this wins** — those knobs
> are compensations for not modelling the single dual-port DARAM the silicon actually has.
>
> Firmware pinned: **Nokia 5110 NSE-1 v5.30 A** (MAD1, Nokia "LEAD" = customised TMS320C54x).
> Sources: `TI_SPRU131G_C54x_CPU_and_Peripherals.pdf` (§3 Memory, §8 On-Chip Peripherals),
> `ref/tms320lc542.pdf` (SPRS039C). MAD1-specific values are RE-verified (cited inline).

---

## 0. TL;DR — the one structural fact

On the C54x, **on-chip DARAM is a single physical dual-access memory.** With `PMST.OVLY=1` it is
mapped into **both program and data space at the same addresses** — an instruction *fetch*, a data
*read*, and a data *write* of a DARAM address all hit **one cell** (SPRU131G §3, "DARAM can be
mapped into program/data memory space by setting the OVLY bit"). The **HPI RAM is a 2K-word block
of that same DARAM**, *dual-ported* so the host (MCU) and the DSP see the **same cells**
(shared-access mode, SPRU131G §8 HPI).

⇒ The co-sim must model the HPI/OVLY region as **ONE store** shared by `prog_fetch`, DSP
data-access, and the MCU window. Today it is three (`data[]` for fetch, `api_ram[]` for DSP data,
MCU `mem[]` for the window) glued by bridges — which is why uploaded blocks "never land" (the host
writes one copy, the DSP fetches another) and why every coherence knob exists.

---

## 1. Memory map (SPRU131G §3, SPRS039C)

| region | role | notes |
|--------|------|-------|
| `0x0000–0x007F` | MMRs + reserved | `0x00–0x1F` CPU MMRs, `0x20–0x5F` peripheral MMRs (data page 0) |
| `0x0080–…`      | **on-chip DARAM** | `OVLY=1`: also program space (executable). `'542` = 10K words, 5×2K blocks |
| HPI RAM (2K)    | **dual-port DARAM** | stock `'542` = `0x1000–0x17FF`; **MAD1 relocates to `0x0800–0x0FFF`** (§3 here) |
| `0x9000–0xFF7F` | on-chip ROM | resident PROM/DROM (the recovered `dsp_full.bin` image) |
| `0xFF80–0xFFFF` | interrupt vectors | `IPTR<<7 + vec*4`; reset `0xFF80`, INT1 `0xFFC4`, INT2 `0xFFC8` |

**OVLY (PMST bit):** when set, DARAM appears in program space; fetch and data access alias the same
cell. MAD1 boots with `PMST=0xFFA8` (OVLY=1, IPTR=0x1FF).

---

## 2. The HPI — dual-port shared RAM (SPRU131G §8 Host-Port Interface)

> "Information is exchanged between the DSP and the host through on-chip memory accessible by both…
> HPI memory is a **2K-word DARAM block** … and can also be used as **program** DARAM."
> "In **shared-access mode (SAM)**, both the DSP and the host can access HPI memory" (host wins a
> same-cycle conflict; DSP waits one cycle). Host indexes the RAM with **`HPIA & 0x7FF`** (only the
> 11 LSBs address the 2K block).

So the HPI RAM is **not a separate buffer** — it is a slice of the DSP's DARAM that the host can
also reach. Code uploaded into it executes in place (OVLY). There is **no "MCU copy"** to keep
coherent; the window *is* the DARAM.

### 2.1 HPI control register — HPIC @ MMR `0x2C` (SPRU131G Table 8-23)

| bit | writable by | function |
|-----|-------------|----------|
| **DSPINT** | host only | host writes 1 → **interrupt the DSP** (the doorbell). Not readable; always reads 0. |
| **HINT**   | DSP sets / host clears | DSP writes 1 → HINT pin low → **interrupt the host**; host clears by writing 1. |
| **SMOD**   | DSP | `0` at reset = host-only mode (**DSP denied HPI-RAM access**); DSP sets `1` after reset to enable SAM. |
| BOB        | host | first-byte significance (byte order). |

---

## 3. MAD1-specific values (RE-verified — Nokia "LEAD" ≠ stock '542)

| fact | value | evidence |
|------|-------|----------|
| HPI RAM base (DSP side) | **DSP `0x0800`** (`word k → 0x800+k`) | DSP code: ready `[0x871]`, hdr `[0x87F]`, ring tail `[0x8E4]`; loader1 `dest+0x800` |
| HPI RAM size | **2K words** (`0x800–0xFFF`) | matches `'542` HPI RAM; MCU window `0x10000–0x10FFF` |
| HPI RAM in MCU space | **`0x10000`** (`word k → MCU 0x10000+2k`) | `src/models/5110/profile.c` (mailbox "hardware-fixed at 0x100E0") |
| host→DSP doorbell | **MCU `0x30000` bit2** = HPIC DSPINT | **firmware-verified**: PC `0x272048` `ldr r1,=0x30000; mov r0,#4; strh r0,[r1]` (also `0x2721D2`) |
| DSP→host | HPIC `0x2C` **HINT** (TI) — co-sim watches **BSCR `0x29`** bit3 ⚠ verify | arch doc RE; **`0x29` is BSCR per TI**, so this attribution is unconfirmed |
| DSP reset control | **MCU `0x20002` bit0** (0→1 = release = upload done) | `docs/dsp-boot-upload-architecture.md` §6b |
| `dsp_uploaded` flag | MCU `0x10A9E4` (MCU RAM, outside window) | profile |

### 3.1 HPI-window field map (window word → DSP addr `0x800+word` → MCU `0x10000+2·word`)

| field | word | DSP | MCU | dir |
|-------|------|-----|-----|-----|
| MDISND ring / tail / head | `0x00 / 0x52 / 0x53` | `0x800 / 0x852 / 0x853` | `0x10000 / 0x100A4 / 0x100A6` | M→D |
| UPLOADREQUEST / ready (`=0x12`) | `0x71` | `0x871` | `0x100E2` | D↔M |
| UPLOADREPLY | `0x72` | `0x872` | `0x100E4` | M→D |
| UPLOADHEADER (5-word descriptor) | `0x7B–0x7F` | `0x87B–0x87F` | `0x100F6–0x100FE` | M→D |
| MDIRCV ring / tail / head | `0x80 / 0xE4 / 0xE5` | `0x880 / 0x8E4 / 0x8E5` | `0x10100 / 0x101C8 / 0x101CA` | D→M |

The uploaded **overlay run addresses (`0x926`, `0xD00`, `0xD80`, `0xF00`) fall inside the 2K HPI
RAM `0x800–0xFFF`** — i.e. the MCU uploads code into the HPI RAM and the DSP executes it there
(OVLY). `loader1` reads `dest=[0x87B]`, `+0x800` → run address, and **checksums** it in place
(`call 0x8023`); it does *not* copy. So the payload must arrive at `0x800+dest` directly — which it
does only if the window write and the DSP fetch share one store.

### 3.2 MMR map (SPRU131G Table 8-2, '542) and the Nokia-custom frame clock

CPU MMRs `0x00–0x1F` (IMR/IFR/ST0/ST1/AL..BG/T/TRN/AR0–7/SP/BK/BRC/RSA/REA/PMST). Peripheral MMRs:
`0x20–0x23` BSP codec (BDRR0/BDXR0/BSPC0/BSPCE0), `0x24–0x26` timer, `0x28` SWWSR, **`0x29` BSCR**,
**`0x2C` HPIC**, `0x30–0x35` TDM, `0x38–0x3B` ABU (AXR0/BKX0/ARR0/BKR0). **`0x3C–0x5F` reserved.**

⇒ **MMR `0x58` is reserved on a stock '542** — the run-mode poll at DSP `0x7EBC` (`mmr[0x58]=…;
spin on bit0`) targets a **Nokia-custom peripheral**, almost certainly the **GSM TDMA frame /
sample clock**. It is *not* findable in TI docs; the `DSP54_MMR58PER` stand-in is legitimately
modelling silicon TI does not define. The faithful long-term model is the **BSP + ABU** codec path
(`0x20–0x23` + `0x38–0x3B`) with serial frame-sync (XINT/RINT) interrupts, not an injected timer.

---

## 4. Current co-sim deviations → fix list

| co-sim (today) | spec | symptom it causes |
|----------------|------|-------------------|
| `prog_fetch→data[]` **+** DSP-data→`api_ram[]` **+** MCU→`mem[]`, glued by `SHAREDWIN`/`HPIAPI`/`CMDBRIDGE` | **one** dual-port DARAM (OVLY + HPI SAM) | uploaded blocks never resident; host ISR `call 0x2414` derails into garbage; producer dormant |
| `dsp54_hpi_write`: window word `k` → `data[k]` | window word `k` = DSP `0x800+k` (one cell) | off-by-`0x800`; payloads land where the DSP never fetches |
| `api_ram` = `0x800`, 8K (Calypso geometry) | HPI RAM = `0x800`, **2K** (`0x800–0xFFF`); `0x1000+` is DSP-private DARAM | conflates shared HPI RAM with private DARAM |
| DSP→host watched at BSCR `0x29` | HPIC `0x2C` HINT | may be watching the wrong register |
| codec frame via injected INT0 + `MMR58PER` | BSP/ABU + serial frame-sync; `0x58` = Nokia frame clock | diagnostic stand-in, not faithful |

## 4a. PMST control-bit coverage (SPRU131G §4.1, Table 4-3)

PMST bits (`SST0 SMUL1 CLKOFF2 DROM3 APTS4 OVLY5 MP/MC6 IPTR15-7`) are defined correctly in
`calypso_c54x.h` and MAD1 boots `PMST=0xFFA8` ⇒ `IPTR=0x1FF, OVLY=1, DROM=1, MP/MC=0`.

| bit | spec meaning | co-sim status |
|-----|--------------|---------------|
| **OVLY** | DARAM appears in **program *and* data space at the same address** = one cell | bit **tested** at every program-space site (`prog_fetch`/`prog_read`/`prog_write`, `0x80–0x2800`→`data[]`) ✓ — **but the aliasing is NOT implemented**: program-space `0x926`→`data[]`, data-space `0x926`→`api_ram[]` = two cells ✗ |
| IPTR | vector base `IPTR<<7` | honored ✓ (reset `0xFF80`, INT1 `0xFFC4`) |
| DROM | on-chip ROM in data space | DROM loaded to `data[0xB000+]` **unconditionally** (not `PMST.DROM`-gated) — OK only because MAD1 DROM=1 fixed |
| MP/MC | on-chip ROM in program space | ROM `>0x2800` always from `prog[]` (not `MP/MC`-gated) — OK only because MAD1 MP/MC=0 fixed |
| data-space DARAM | always in data space (OVLY-independent) | correctly **not** OVLY-gated ✓ |

**Key point:** OVLY is *checked* but its *effect* (unify program & data over `[0x800,0x2800)`) is not
implemented — program fetch lands in `data[]`, data access in `api_ram[]`. So "is OVLY covered?" —
the *bit* yes, the *behaviour* no. The §5 fix is precisely to make OVLY mean what the spec says.

## 5. The single-store contract (what to implement)

1. **One DARAM array** backs `[0x80, 0x2800)`. `prog_fetch` (under OVLY), DSP data read/write, and
   the MCU window (`0x10000–0x10FFF` → DARAM `0x800–0xFFF`) all alias it. No `data[]`/`api_ram[]`
   duplication; no per-region bridge.
2. **MCU window write to `0x10000+2k` ⇒ DARAM[`0x800+k`]** (the single cell). MCU read symmetric.
3. **DSPINT** = MCU `0x30000` bit2 → DSP INT (HPIC DSPINT). **HINT** = DSP sets HPIC `0x2C` → MCU
   IRQ4/FIQ0 (verify vs the `0x29` attribution).
4. **`SMOD`**: before the DSP sets SMOD=1 (post-reset), the host owns the RAM (HOM); model the
   reset→release ordering already captured by `0x20002`.
5. Retire `DSP54_SHAREDWIN` / `HPIAPI` / `CMDBRIDGE` / `LIVE` once the single store lands — they
   become no-ops over a coherent memory.

**Expected result:** the base set lands at its run addresses → loader1's `0x8023` checksum passes →
the branch table at `0x23C4` is built → the host-cmd ISR's `call 0x2414` reaches the real dispatch
`0x35B9` → host commands serviced → gates `[0x866]`/`[0x6E4]` set → MDISND drained / MDIRCV produced.

### 5.1 Implementation status (2026-06-08) — `DSP54_UNIDARAM=1`

Step 1 (the single store) is **implemented and shipped behind `DSP54_UNIDARAM=1`** (default OFF):
`c54x_set_ovly_unified()` routes program-fetch/`prog_read`/`prog_write` of `[0x800,0x2800)` to
`api_ram[]` (the `ovly_cell()` helper), and `dsp54_hpi_write/read` route the whole MCU window to
`api_ram[k]`. **Verified:** `make test` = 41+16+35 green; default boot byte-identical; Recipe B LCD
**byte-identical** with/without the flag (`md5 b86d6d5c…`), so it does not diverge the MCU.

**Result: necessary but not yet sufficient.** With UNIDARAM the shared cells are now coherent
(`[0x6E1]` moves `0x0000→0xF0EC` — the DSP reads real shared values), but boot still does **not**
complete: `bist≈111M (~97%)`, `idle408C=0`, `hostISR3598=0`. The DSP is **stuck in the resident
PROM BIST/vocoder grind (`0xD800–0xF200`) *before* it ever reaches the uploaded scheduler** — an
*upstream* blocker independent of memory coherence. The memory fix is the correct foundation; the
**next blocker is the BIST self-test not terminating** in the lift (consistent with the long-running
"BIST limit-cycle" theme). NEXT: RE the BIST exit condition (what makes the resident self-test
finish and drop to the host-ready idle `0x408C` on silicon) — that is what gates reaching the now-
coherent uploaded code.

## 6. Open items to verify against firmware/silicon

- **HINT register**: confirm DSP→host is HPIC `0x2C` (TI) vs the RE'd BSCR `0x29` bit3.
- **MMR `0x58`**: characterise the Nokia frame clock (period/reload `0xFFFD`/`0x3146`) and whether
  it is the BSP frame-sync or a separate TDMA counter.
- **`0x1000+` DARAM**: confirm it is DSP-private (not host-visible) — the MCU window is only 2K.
