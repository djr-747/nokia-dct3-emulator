# Forensic + cross-build toolkit (RE workflow)

Tools added during the multi-model bring-up for answering "**why did it stop here?**"
without manual computation, and for porting the 3310's names to new builds. All the
runtime knobs are **default-off and NULL-guarded** — the 3310 reference boot stays
byte-identical when they're unused.

## 1. Capture a black box at a halt — `build/dct3_boot_trace`

One run snapshots full machine state + the data/control trail leading in. Trigger the
dump at a step, a PC, or a RAM byte-change; all gated by a cycle window.

```bash
RAMDUMP=/tmp/x RAMDUMPPC=0x003E42C8 RAMDUMPSTOP=1 \
MEMLOG=1 MEMLOGRD=1 CALLLOG=1 \
  ./build/dct3_boot_trace "firmware/Nokia 5210 NSM-5 v5.40 A.fls" 5000000
```

Produces alongside `/tmp/x`:

| file | contents |
|---|---|
| `/tmp/x`         | 16 MB RAM snapshot (flat, indexed by masked address) |
| `/tmp/x.regs`    | r0–r15 + cpsr at the trigger |
| `/tmp/x.memlog`  | last N memory accesses — writes `old -> new @pc`, reads `value @pc` |
| `/tmp/x.calllog` | last N taken BL/BLX/BX — `pc -> target lr m=mode` |

**Dump triggers** (pick one): `RAMDUMPAT=<step>` · `RAMDUMPPC=0xADDR` ·
`RAMDUMPWATCH=0xADDR` (fires when that byte changes). `RAMDUMPAFTER=<step>` arms only
after a cycle window. `RAMDUMPSTOP=1` ends the run after the dump.

**Log knobs:** `MEMLOG=1` (writes only) + `MEMLOGRD=1` (also reads), `MEMLOGN=<n>`
(ring size, default 512). `CALLLOG=1` + `CALLLOGN=<n>` (default 256). Both rings are
flushed at the dump.

## 2. Read it back, fully annotated — `build/disfw`

disfw now does lightweight register value-tracking (linear sweep), so the disasm states
what it used to leave to inference. With `--ram` it shows **live values** from the
snapshot.

```bash
# annotated disasm with live values from the snapshot
./build/disfw "firmware/Nokia 5210 NSM-5 v5.40 A.fls" 0x003B29A0 5 --ram /tmp/x
```
yields, e.g.:
```
ldr  r5, =0x0013B528   ; GLOBAL_BUFFER+0x7E8
ldrb r0, [r5, #0]      ; [0x13B528] -> 0x00
cmp  r0, #0            ; gate: [0x13B528] == 0x0
```

Annotations: **effective address + symbol** (`ldrb r1,[r0,#1] ; [0x120E21 = TASKS_INIT_TABLE+0x1]`,
propagated through `add/sub/mov/ADR`), **cmp-gate** (`gate: [ADDR] == K`), **jump/dispatch
table peek** (`[BASE + rIdx]  table: FN0, FN1, ...`), **indirect `bx/blx`** target, and the
**live value** (`-> 0xVV`, sized by the access) with `--ram`. The sidecar auto-loads by
fw-id (globs `tools/symbols/*.symbols.txt`; `SYMDIR` overrides).

> Tracking is basic-block-local: the gate/effective-addr annotation needs the
> `ldr rX,=BASE` pointer-load **in the same disasm window** as the access.

### Struct field map — `--struct`

Sweeps the whole image with the tracker and lists every setter (store) and reader (load)
of a struct's fields, grouped by offset, with PCs and (with `--ram`) live values:

```bash
./build/disfw "firmware/Nokia 5210 NSM-5 v5.40 A.fls" --struct 0x0013B528 --structsize 0x40 --ram /tmp/x
#   +0x00  byte  [=0x00]  W:2 0x345A2C, 0x345E34   R:5 0x345E24, 0x3B29A2, ...
#   +0x24  word  [=0x0040BAEC]  W:0   R:1 0x3459F0
```

This is the auto version of "trace a struct by hand across sessions". `--structsize`
sets the window (default 0x80).

## 3. Port names to a new build — `tools/symbols/`

```bash
# bulk-locate via the NokiX LOCATE_*.rx masked sigs -> <model>-<ver>.nokix.yaml + sidecar
REGINA_MACROS=$(realpath ref/NokiX-scripts/NokiX/scripts/scripts/macros) \
  ./tools/symbols/dump_nokix.py "firmware/Nokia 5210 NSM-5 v5.40 A.fls" --model 5210
./tools/symbols/gen_symbols_txt.py 5210-v540        # refresh the disfw sidecar

# auto-derive a masked byte signature from a named 3310 fn; find it in another build
./tools/symbols/gen_sig.py "<3310.fls>" 0x2EEBAE --find "<dst.fls>"          # exact (close builds)
./tools/symbols/gen_sig.py "<3310.fls>" 0x2EEBAE --fuzzy --find "<dst.fls>"  # ranked (distant)

# call/opcode SKELETON match (build-invariant) — name a fn via its 3310 analogue
./tools/symbols/lookup.py 3310-v579 list > /tmp/named.txt
./tools/symbols/match_skel.py "<dst.fls>" 0x3E4236 "<3310.fls>" --named /tmp/named.txt
```

`gen_sig` exact-matches close builds (3330/5510) and verifies known fns on distant ones;
`match_skel` crosses generations (HIGH confidence on call-rich fns, self-flags
short/leaf). NokiX `dump_nokix.py` is the most reliable for the ~345 known primitives.

## Worked example — the 5210 `[0x13B528+0]` halt gate
1. `--struct 0x13B528 --ram` → field map: `+0x00` gate (writers `0x345A2C`/`0x345E34`,
   reader `0x3B29A2`); `+0x24` = the command-table pointer.
2. `RAMDUMPPC=0x345E34 MEMLOG=1 CALLLOG=1` → capture the writer's state to see what value
   it stores and the condition that chose it.
3. `disfw … 0x345E34 --ram /tmp/x` → read the writer with live values + the gate it's on.
