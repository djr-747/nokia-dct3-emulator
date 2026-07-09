# MCU↔DSP init baseline — the canonical signal protocol (HLE)

> **The authoritative MCU-side DSP init/upload protocol**, captured against the **known-good HLE
> DSP** (`dsp_default`) — the config that boots clean to standby. The MCU performs the same init
> regardless of the DSP model (it only needs the handshake answered), so this is the *proper baseline*
> the co-sim (C54x core) must reproduce. Firmware: **Nokia 5110 NSE-1 v5.30 A** (MAD1). Times are MCU
> steps ("instruction#"). Captured with `DSP54_INITLOG=1` (+ `WRWATCH`/`WATCH`), native, HLE (no
> `DSP54_COSIM`). Companion: `docs/dsp-c54x-memory-model.md` (memory/HPI), `docs/dsp-boot-upload-architecture.md`.

Reproduce:
```bash
FW="firmware/Nokia 5110 NSE-1 v5.30 A.fls"
DSP54_INITLOG=1 ./build/dct3_boot_trace "$FW" 8000000 2>&1 | grep '^\[init\]'
WATCH=0x00271D70 ./build/dct3_boot_trace "$FW" 8000000 2>&1 | grep '\[watch\]'   # runtime cmd dispatch
```

## Address map (window word k = MCU `0x10000+2k` = DSP `0x800+k`)

| signal | window word | MCU addr | dir | role |
|--------|------|----------|-----|------|
| MDISND ring / tail / head | `0x00 / 0x52 / 0x53` | `0x10000 / 0x100A4 / 0x100A6` | M→D | MCU→DSP send ring |
| boot_status / status2 | `0x02 / 0x03` | `0x10004 / 0x10006` | D→M | version/ready reply |
| UPLOADREQUEST / ready(`0x12`) | `0x71` | `0x100E2` | D↔M | DSP block-request / loader ready |
| UPLOADREPLY | `0x72` | `0x100E4` | M→D | `0x02`=MORE, `0x04`=FINISHED |
| UPLOADHEADER (5-word descr) | `0x7B–0x7F` | `0x100F6–0x100FE` | M→D | per-transfer / per-block descriptor |
| PORT2 / mailbox | `0x80` | `0x10100` | M↔D | host-cmd handshake word |
| PORT1 / cmd | `0x81` | `0x10102` | M→D | host-cmd bits |
| MDIRCV ring / tail / head | `0x80 / 0xE4 / 0xE5` | `0x10100 / 0x101C8 / 0x101CA` | D→M | DSP→MCU receive ring |
| DSP reset-ctrl | — | `0x20002` (bit0) | M→D | hold(0)/release(1) |
| DSPINT doorbell | — | `0x30000` (bit2) | M→D | MCU→DSP interrupt (= HPIC DSPINT) |
| DSP→MCU interrupt | — | FIQ/IRQ4 | D→M | HINT (HLE raises `fiq_pending`) |

---

# SECTION 1 — MCU → DSP

### 1.1 IO reset-control `0x20002` (bit0 = release)
| step | write | PC | meaning |
|------|-------|-----|---------|
| 28371 | `0x01` | `0x271AA8` | **release** — Phase-1 bulk upload begins (DSP runs) |
| 327832 | `0x00` | `0x271BE2` | reset asserted (DSP held) |
| 368108 | `0x00` | — | (re-assert during Phase-2 setup) |
| 397479 | `0x01` | `0x271D2A` | **release** — upload FINISHED (the real boot moment) |
| ~later | `0x03` | `0x28DC3C` | post-boot (bit0=1 + bit1) |

### 1.2 DSPINT doorbell `0x30000` bit2 (MCU→DSP INT → HPIC DSPINT)
Strobed `value=0x04` at PC `0x272048` / `0x2721D2`. First fires `@398219` (right after upload-done),
then `@504731`, `@505611` (post-boot command notifications).

### 1.3 Block upload — Phase 1 (bulk DMA to staging)
`@28335` post header → release → ~33 transfers of **768 words** into the **staging buffer** (window
`0x100` = MCU `0x10200`), each ~4624 steps apart, each ending with a `[0x7F]`/`[0x80]=0` handshake:
```
UPLOADHEADER = { off=0x100, len=0x300(768), 0, flags=0x8000, seq }   ; offset=staging, not per-block
```
The staging words `0x10200…` are each written **33×** (one per transfer) — the whole base-set payload
streams through one staging window.

### 1.4 Block upload — Phase 2 (real block descriptors) + reply
`@~397418` the MCU posts the genuine block descriptors to `UPLOADHEADER[0x7B–0x7F] =
[entry, ctrl, size, dest, len]`, with `UPLOADREPLY[0x72]` stepping `0x00 → 0x02 (MORE) @397458 →
0x04 (FINISHED) @398216`:

| block | entry | ctrl | size | dest | len | step |
|-------|-------|------|------|------|-----|------|
| 0 (IVT/branch) | `0xFD00` | `0xFF80` | `0x0244` | `0x0500` | `0x0078` | 397418 |
| 1 (code)       | `0x25B4` | `0x1F80` | `0x034E` | `0x0130` | `0x044C` | 397843 |
| — terminator   | `0x0000` | `0x0000` | `0x0000` | `0x0000` | `0x0000` | — |

**Only block0 and block1 are finalized via descriptors at boot** — NOT all 18. The loaders
(block25/loader1, block18/loader2) are pre-resident (HPI hardware boot), and the `0x926` codec
overlays (block3/4/5/9) are **demand-paged during a call**. See block-presence table below.

### 1.5 Host-command mailbox (version handshake)
`@28352 PORT2[0x80]=0x0001`, `@28356 PORT1[0x81]=0x0200` → DSP cmd `=(~PORT2)&PORT1 = 0x200`
(version). (DSP-side ISR `0x3598` decodes; see memory-model doc §2.1.)

### 1.6 Runtime command dispatch `0x271D70(r0=cmd, r1=param)` — POST-boot, not block upload
`r0` is a 0–46 command index (jump-table at `0x271DA4`). Observed (periodic, `@504k` then `@7.3M`):
`r0 = 0x08, 0x09, 0x20, 0x24, 0x25, 0x28, 0x2D, 0x2E` with data in `r1`. These are runtime DSP
control commands (codec/config), *not* the boot block upload.

### 1.7 MDISND ring (MCU→DSP send)
Ring/ptrs cleared `@369310` (`tail 0x52`, `head 0x53` ← 0). At runtime the MCU posts messages
(`word0=0x0A05`…) and advances `MDISND_TAIL[0x100A4]`; the DSP polls/dequeues from its scheduler.

---

# SECTION 2 — DSP → MCU

### 2.1 boot_status / version reply
`@28199 boot_status[0x10004]: 0000 → FFFF` — the DSP-model's version/ready reply slot (the MCU's
loader `0x271AF4` polls `[0x10004]`; profile cross-checks `[0x10004]==[0x10006]`).

### 2.2 MDIRCV ring init (the empty-ring baseline)
`@396837 MDIRCV_TAIL[0x101C8]: 0000 → 0080` and `MDIRCV_HEAD[0x101CA] → 0080` — the DSP initialises
the receive ring to the empty state `0x80/0x80` at upload-finish. (Coherent-ring target the co-sim
must match.)

### 2.3 DSP→MCU interrupt (HINT = FIQ/IRQ4)
`fiq_pending` raised: `@0` `=0x80` (initial), then post-boot `@564087+` `=0x84 / 0x88 / 0x8C` — the
HLE's MDIRCV-post notifications. (On silicon: DSP sets HPIC `0x2C` HINT bit → MCU IRQ4; the BSCR
`0x29` attribution is unconfirmed — see memory-model doc §6.)

### 2.4 UPLOADREPLY pacing (note: M→D write, D-gated)
`UPLOADREPLY[0x72]` is written by the MCU (`0x02`/`0x04`) but paced by the DSP's per-transfer ack
(`UPLOADREQUEST[0x71]`/`[0x7F]` handshake) — the DSP gates progress by clearing the handshake word.

---

## Block presence in FW (descriptor table `@0x293B2C`)

**Present (18, real payload):** 0,1,2,3,4,5,6,8,9,11,18,19,20,21,22,24,25,26.
**Absent (9, `size=0` placeholders):** 7,10,12,13,14,15,16,17,23.
Base set = 0 (IVT/branch→`0xD00`), 18 (loader2→`0xD80`), 25 (loader1→`0xF00`). The nine `0x926`
entries (3,4,5,7,9,12–15) are demand-paged codec alternates (only 3/4/5/9 carry payload).

## Init timeline (summary)
```
@0       DSP FIQ=0x80 (initial)
@28199   boot_status -> 0xFFFF (DSP ready/version)
@28335   UPLOADHEADER {0x100,0x300,0x8000,seq}  + PORT2=1/PORT1=0x200 (version cmd)
@28371   RESET-CTRL release (bit0=1)  → Phase-1 bulk: ~33×768w → staging 0x100
@327832  RESET-CTRL assert
@369310  MDISND ring clear ; @371354 MDIRCV ring clear
@396837  MDIRCV ring init -> 0x80/0x80 (empty)
@397418  Phase-2 descriptors: block0, block1 ; UPLOADREPLY 0x02(MORE)
@397479  RESET-CTRL release (bit0=1)  → upload FINISHED
@398216  UPLOADREPLY 0x04(FINISHED) ; @398219 DSPINT doorbell
@504k+   DSPINT + runtime cmds (0x271D70: 8,9,0x20,0x24,0x25,0x28,0x2D,0x2E)
@564k+   DSP FIQ 0x84/0x88/0x8C (MDIRCV posts)
```

> Captured via `DSP54_INITLOG=1` (default-OFF; logs in the HLE path, outside the cosim gate).
