# Vendored: mGBA ARM7TDMI core

The CPU for the DCT3 emulator, lifted from [mGBA](https://github.com/mgba-emu/mgba).

- **Upstream:** https://github.com/mgba-emu/mgba
- **Commit:** `9a36d65767a0ea21616b4d602e2b31eb875b2f18` (see `MGBA_COMMIT.txt`)
- **License:** MPL-2.0 (see `LICENSE`). Every source file retains its original
  MPL header. **Two deliberate modifications** from upstream — see "Local changes".

## Local changes

- `src/arm/arm.c` + `include/.../arm/arm.h` — added **`ARMRaiseFIQ`** (upstream
  only exposed `ARMRaiseIRQ`). It mirrors `ARMRaiseIRQ` but enters `MODE_FIQ` at
  `BASE_FIQ` (0x1C) and masks both F and I. The MAD2 firmware drives the MBUS
  UART (and sleep clock) via FIQ, so the platform must be able to deliver them.

- `include/mgba/internal/arm/macros.h` — the `LOAD_*`/`STORE_*` macros are pointed
  at the **big-endian** variants (`LOAD_32BE`, …) instead of upstream's
  little-endian. The DCT3/MAD2 ARM7TDMI runs big-endian, and the CPU's instruction
  fetch reads memory through these macros (bypassing the `ARMMemory` vtable), so the
  fetch path itself must be big-endian. A simple byte-swap of the flash image can't
  work because the firmware mixes 32-bit ARM and 16-bit Thumb fetches from the same
  bytes. Keeping this the only change lets the flash dump stay byte-for-byte
  identical to physical hardware. The `.c`/`.h` execution and decode sources are
  unmodified.

## What's here

`src/arm/`
| file              | role |
|-------------------|------|
| `arm.c`           | core lifecycle: `ARMInit`/`ARMReset`/`ARMRun`, mode switching, IRQ entry/exit |
| `isa-arm.c`       | ARM instruction implementations (execution) |
| `isa-thumb.c`     | Thumb instruction implementations (execution) |
| `decoder.c`       | `ARMDisassemble` — text disassembly; a bring-up / boot-trace aid |
| `decoder-arm.c`   | ARM decode tables → `ARMInstructionInfo` |
| `decoder-thumb.c` | Thumb decode tables → `ARMInstructionInfo` |

`include/` — the minimal header closure needed to compile the above:
- `mgba/internal/arm/*.h` — arm core, isa, decoder, emitter X-macro tables, macros
- `mgba/core/cpu.h` — `mCPUComponent`, `mMemoryAccessSource` (referenced by `arm.h`)
- `mgba/internal/debugger/symbols.h` — symbol-lookup decl used by the disassembler
- `mgba-util/{common,dllexports,macros,string}.h` — base types + platform macros

## Integration notes

- The CPU reaches memory **only** through the `struct ARMMemory` vtable in
  `arm.h` (`load`/`store` 8·16·32, `loadMultiple`/`storeMultiple`,
  `setActiveRegion`). The platform (`src/mad2/`) implements that vtable; the
  core never learns what a phone is.
- `decoder.c` references `mDebuggerSymbolReverseLookup`; we supply a no-symbols
  stub returning `NULL` (`src/core/dct3_debug_stub.c`) so it links. The core is
  built with `-DENABLE_DEBUGGERS` to compile in the disassembler.
- To update: re-copy `src/arm/` + this header closure from a newer mGBA commit
  and bump `MGBA_COMMIT.txt`.
