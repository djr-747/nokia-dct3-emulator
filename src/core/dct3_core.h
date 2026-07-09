// DCT3 core — a thin wrapper around the vendored mGBA ARM7TDMI.
//
// Phase 1 goal: bring the CPU to life over a flat RAM model, decoupled from any
// notion of "phone". The core reaches memory only through the ARMMemory vtable
// (see third_party/mgba-arm/.../arm.h); this file implements that vtable over a
// single contiguous buffer. Phase 2 swaps the flat buffer for the MAD2 MMIO
// router without the CPU core noticing.

#ifndef DCT3_CORE_H
#define DCT3_CORE_H

#include <stdint.h>
#include <stddef.h>

#include <mgba/internal/arm/arm.h>

// Flat address space for bring-up. 16 MiB comfortably covers the 3310's 2 MB
// flash plus RAM, and the power-of-two size makes masking a single AND.
#define DCT3_RAM_SIZE 0x01000000u          // 16 MiB
#define DCT3_RAM_MASK (DCT3_RAM_SIZE - 1u)  // 0x00FFFFFF

// Phase 2 I/O hooks. Accesses with io_lo <= addr < io_hi are routed through
// these while memory stays RAM-backed: io_read returns the value delivered to
// the CPU (given the RAM-backed value, so it can override status registers);
// io_write observes stores. Both may be NULL. `pc` is the faulting instruction.
typedef uint32_t (*dct3_io_read_fn)(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t ram_value);
typedef void     (*dct3_io_write_fn)(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t value);

typedef struct DCT3Core {
    struct ARMCore cpu;   // MUST be first: vtable callbacks cast ARMCore* -> DCT3Core*
    uint8_t* ram;         // DCT3_RAM_SIZE bytes, big-endian (BE-32 target)

    // Bring-up instrumentation, written by the interrupt-handler vtable.
    uint32_t last_swi;      // immediate of the most recent SWI
    uint32_t last_stub;     // opcode of the most recent unimplemented instruction
    uint32_t last_illegal;  // opcode of the most recent illegal instruction
    int swi_count;
    int stub_count;
    int illegal_count;

    // I/O trace/intercept (see typedefs above). Zero-initialised = disabled.
    void* io_ctx;
    uint32_t io_lo, io_hi;
    dct3_io_read_fn  io_read;
    dct3_io_write_fn io_write;

    // Second I/O range, *device-owned*: stores in [io2_lo,io2_hi) are NOT written
    // to RAM by the core — the handler owns the backing (used for the flash chip,
    // whose command writes must not hit the array). Reads are still RAM-backed and
    // passed to io_read so the handler can override (e.g. status-register reads).
    uint32_t io2_lo, io2_hi;

    // Memory access trace hook (harness/debug; zero-initialised = disabled). Fired on
    // every store with the pre-write value (old) and the stored value (new); also on
    // loads when mem_trace_reads != 0. Lets a tool keep a rolling read/write log
    // (addr, old->new, pc) for a forensic window before a snapshot. NULL = no overhead.
    void*  mem_trace_ctx;
    void (*mem_trace)(void* ctx, uint32_t pc, uint32_t addr, int size,
                      uint32_t oldv, uint32_t newv, int is_write);
    uint8_t mem_trace_reads;     // also fire on loads (default 0 = writes only)
} DCT3Core;

// Lifecycle.
DCT3Core* dct3_core_create(void);
void     dct3_core_destroy(DCT3Core* core);
void     dct3_core_reset(DCT3Core* core);   // ARMReset: PC=0, ARM/SYSTEM mode

// Host-side memory access (big-endian, bounds-masked, no LDR rotation).
void     dct3_write_bytes(DCT3Core* core, uint32_t addr, const void* data, size_t len);
void     dct3_write32(DCT3Core* core, uint32_t addr, uint32_t value);
void     dct3_write16(DCT3Core* core, uint32_t addr, uint16_t value);
uint32_t dct3_read32(DCT3Core* core, uint32_t addr);
// Repair the MCU flash-region checksum after a file-level firmware patch so the
// boot integrity check passes (else reset reason 4). No-op on an unpatched image.
// Returns 1 if it repaired a mismatch, 0 otherwise.
int dct3_fix_mcu_checksum(DCT3Core* core);

// Execution.
void     dct3_step(DCT3Core* core);        // execute one instruction
void     dct3_run(DCT3Core* core, int n);  // execute n instructions

// Phase 2 bring-up helpers.
void dct3_set_io_hooks(DCT3Core* core, void* ctx, uint32_t lo, uint32_t hi,
                       dct3_io_read_fn read, dct3_io_write_fn write);
// Set the device-owned second range (e.g. flash). Uses the same io_read/io_write.
void dct3_set_io_range2(DCT3Core* core, uint32_t lo, uint32_t hi);
// Install the memory access trace hook (NULL cb = disabled). reads != 0 also traces loads.
void dct3_set_mem_trace(DCT3Core* core, void* ctx,
                        void (*cb)(void*, uint32_t, uint32_t, int, uint32_t, uint32_t, int),
                        int reads);
void dct3_boot_at(DCT3Core* core, uint32_t entry);  // HLE boot ROM: enter ARM at `entry`

// Register / status helpers (gpr index 0..15; r15 == PC).
static inline uint32_t dct3_reg(const DCT3Core* core, int i) { return (uint32_t)core->cpu.gprs[i]; }
static inline uint32_t dct3_cpsr(const DCT3Core* core) { return (uint32_t)core->cpu.cpsr.packed; }
static inline int      dct3_mode(const DCT3Core* core) { return core->cpu.privilegeMode; }

// Force PC + CPSR — the resume primitive used to recover from a firmware-self-reset
// (mad2's reset-reason discriminator; see docs/watchdog-reset-3310.md). `cpsr` encodes
// privilege mode (low 5 bits) and Thumb (bit5 .t) of the resume context. ARM/Thumb is
// switched correctly and the prefetch pipeline is refilled, so execution continues at
// `pc` exactly as if the exception or panic never happened.
//
// `pc` may have bit0 set to request Thumb (a function-pointer / saved-LR convention);
// if `cpsr.t` is already set bit0 is ignored. The PC is masked to the instruction width.
// If `sp` is non-zero, it's written to gprs[13] of the resume mode AFTER the CPSR/mode
// switch — used by multi-frame stack-pop recovery to skip teardown fns. Pass 0 to keep
// the current SP (the common case for plain exception-return).
void dct3_core_force_pc_cpsr(DCT3Core* core, uint32_t pc, uint32_t cpsr, uint32_t sp);

#endif // DCT3_CORE_H
