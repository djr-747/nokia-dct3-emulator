// DCT3 core — ARMMemory vtable over a flat RAM buffer (Phase 1).
//
// The vendored mGBA core (third_party/mgba-arm) talks to memory two ways:
//   * Instruction fetch reads `activeRegion` directly (a raw pointer + mask),
//     bypassing the vtable — so activeRegion must point at a contiguous buffer.
//   * Data access (LDR/STR/LDM/STM, SWP) goes through the function pointers in
//     struct ARMMemory.
// Both are satisfied here by a single big-endian byte buffer (the DCT3/MAD2 MCU
// is big-endian; the vendored fetch macros are set to BE accordingly).

#include "core/dct3_core.h"

#include <stdlib.h>
#include <string.h>

#include <mgba/internal/arm/isa-inlines.h>

// --- Raw RAM access: big-endian, bounds-masked, alignment-forced -------------
// The DCT3/MAD2 ARM7TDMI runs big-endian (BE-32, word-invariant), so words and
// halfwords are assembled MSB-first; byte addressing is natural. This matches
// the vendored core, whose fetch macros are set to the BE variants (arm/macros.h),
// and lets the flash image stay byte-for-byte identical to the physical dump.

static inline uint32_t ram_rd32(DCT3Core* c, uint32_t a) {
    a = (a & DCT3_RAM_MASK) & ~3u;
    const uint8_t* p = c->ram + a;
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static inline uint16_t ram_rd16(DCT3Core* c, uint32_t a) {
    a = (a & DCT3_RAM_MASK) & ~1u;
    const uint8_t* p = c->ram + a;
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline uint8_t ram_rd8(DCT3Core* c, uint32_t a) {
    return c->ram[a & DCT3_RAM_MASK];   // byte addressing is natural in BE-32
}
static inline void ram_wr32(DCT3Core* c, uint32_t a, uint32_t v) {
    a = (a & DCT3_RAM_MASK) & ~3u;
    uint8_t* p = c->ram + a;
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static inline void ram_wr16(DCT3Core* c, uint32_t a, uint16_t v) {
    a = (a & DCT3_RAM_MASK) & ~1u;
    uint8_t* p = c->ram + a;
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}
static inline void ram_wr8(DCT3Core* c, uint32_t a, uint8_t v) {
    c->ram[a & DCT3_RAM_MASK] = v;
}

// The vtable callbacks receive struct ARMCore*, which is the first member of
// DCT3Core, so the cast recovers the owning struct.
#define CORE(cpu) ((DCT3Core*)(cpu))

// --- ARMMemory vtable --------------------------------------------------------

// Address of the instruction currently executing (PC runs two fetches ahead).
static inline uint32_t instr_pc(struct ARMCore* cpu) {
    return (uint32_t)cpu->gprs[ARM_PC] - (cpu->cpsr.t ? 4u : 8u);
}
// Route reads/writes in [io_lo,io_hi) or the device-owned [io2_lo,io2_hi)
// through the platform hooks (if installed).
static inline int in_io(DCT3Core* c, uint32_t a) {
    return (a >= c->io_lo && a < c->io_hi) || (a >= c->io2_lo && a < c->io2_hi);
}
// True for the device-owned range, where the core must NOT write RAM itself.
static inline int in_io2(DCT3Core* c, uint32_t a) {
    return c->io2_hi && a >= c->io2_lo && a < c->io2_hi;
}
static inline uint32_t io_rd(DCT3Core* c, uint32_t addr, int size, uint32_t v) {
    if (c->io_read && in_io(c, addr))
        return c->io_read(c->io_ctx, instr_pc(&c->cpu), addr, size, v);
    return v;
}
static inline void io_wr(DCT3Core* c, uint32_t addr, int size, uint32_t v) {
    if (c->io_write && in_io(c, addr))
        c->io_write(c->io_ctx, instr_pc(&c->cpu), addr, size, v);
}

// Fire the read trace hook (loads only, opt-in) with the delivered value.
static inline void trace_rd(DCT3Core* c, uint32_t addr, int size, uint32_t value) {
    if (c->mem_trace && c->mem_trace_reads)
        c->mem_trace(c->mem_trace_ctx, instr_pc(&c->cpu), addr, size, value, value, 0);
}
static uint32_t mem_load32(struct ARMCore* cpu, uint32_t address, int* cycles) {
    if (cycles) *cycles += 1;
    DCT3Core* c = CORE(cpu);
    uint32_t value = ram_rd32(c, address);
    unsigned rot = (address & 3u) << 3;   // ARMv4 unaligned LDR rotate
    if (rot) value = (value >> rot) | (value << (32 - rot));
    value = io_rd(c, address, 4, value);
    trace_rd(c, address, 4, value);
    return value;
}
static uint32_t mem_load16(struct ARMCore* cpu, uint32_t address, int* cycles) {
    if (cycles) *cycles += 1;
    DCT3Core* c = CORE(cpu);
    uint32_t value = io_rd(c, address, 2, ram_rd16(c, address));
    trace_rd(c, address, 2, value);
    return value;
}
static uint32_t mem_load8(struct ARMCore* cpu, uint32_t address, int* cycles) {
    if (cycles) *cycles += 1;
    DCT3Core* c = CORE(cpu);
    uint32_t value = io_rd(c, address, 1, ram_rd8(c, address));
    trace_rd(c, address, 1, value);
    return value;
}
static void mem_store32(struct ARMCore* cpu, uint32_t address, int32_t value, int* cycles) {
    (void)cycles;   // ARM7TDMI STR = 2N. The core's THUMB_STORE_POST_BODY already charges
                    // the non-sequential access, so adding a store cycle here over-counted
                    // (STR was 3 cycles; now the correct 2). Loads keep their +1 (LDR = 3).
    DCT3Core* c = CORE(cpu);
    // VALWATCH=0xVAL (TEMP diag): log the first stores of a specific 32-bit VALUE anywhere
    // (find where a pointer is parked). Cheap cached gate; default OFF.
    { static long vw = -2; static int vn = 0;
      if (vw == -2) { const char* e = getenv("VALWATCH"); vw = e ? (long)strtoul(e, 0, 0) : -1; }
      if (vw >= 0 && (uint32_t)value == (uint32_t)vw && vn < 80) {
          fprintf(stderr, "[valw] str32 [%08X]=%08X pc=%06X\n",
                  address, (uint32_t)value, instr_pc(&c->cpu) & 0xFFFFFF); vn++; } }
    if (c->mem_trace)
        c->mem_trace(c->mem_trace_ctx, instr_pc(&c->cpu), address, 4,
                     ram_rd32(c, address), (uint32_t)value, 1);
    if (!in_io2(c, address)) ram_wr32(c, address, (uint32_t)value);  // flash: handler owns the store
    io_wr(c, address, 4, (uint32_t)value);
}
static void mem_store16(struct ARMCore* cpu, uint32_t address, int16_t value, int* cycles) {
    (void)cycles;   // ARM7TDMI STRH = 2N (see mem_store32).
    DCT3Core* c = CORE(cpu);
    if (c->mem_trace)
        c->mem_trace(c->mem_trace_ctx, instr_pc(&c->cpu), address, 2,
                     ram_rd16(c, address), (uint16_t)value, 1);
    if (!in_io2(c, address)) ram_wr16(c, address, (uint16_t)value);
    io_wr(c, address, 2, (uint16_t)value);
}
static void mem_store8(struct ARMCore* cpu, uint32_t address, int8_t value, int* cycles) {
    (void)cycles;   // ARM7TDMI STRB = 2N (see mem_store32).
    DCT3Core* c = CORE(cpu);
    if (c->mem_trace)
        c->mem_trace(c->mem_trace_ctx, instr_pc(&c->cpu), address, 1,
                     ram_rd8(c, address), (uint8_t)value, 1);
    if (!in_io2(c, address)) ram_wr8(c, address, (uint8_t)value);
    io_wr(c, address, 1, (uint8_t)value);
}

// Starting address for an LDM/STM block. The lowest-numbered register always
// maps to the lowest address; the mode just shifts where the block begins.
static uint32_t lsm_start(uint32_t base, int count, enum LSMDirection dir) {
    switch (dir) {
        case LSM_IA: return base;
        case LSM_IB: return base + 4;
        case LSM_DA: return base - (uint32_t)count * 4 + 4;
        case LSM_DB: return base - (uint32_t)count * 4;
        default:     return base;
    }
}
// Writeback value after the transfer (base ± count*4; bit1 of dir = decrement).
static uint32_t lsm_writeback(uint32_t base, int count, enum LSMDirection dir) {
    return (dir & LSM_D) ? base - (uint32_t)count * 4 : base + (uint32_t)count * 4;
}

static uint32_t mem_loadMultiple(struct ARMCore* cpu, uint32_t base, int mask,
                                 enum LSMDirection dir, int* cycles) {
    DCT3Core* c = CORE(cpu);
    int count = __builtin_popcount((unsigned)(mask & 0xFFFF));
    uint32_t addr = lsm_start(base, count, dir);
    for (int i = 0; i < 16; ++i) {
        if (mask & (1 << i)) {
            cpu->gprs[i] = (int32_t)ram_rd32(c, addr);  // PC reload handled by the ISA's POST_BODY
            addr += 4;
            if (cycles) *cycles += 1;
        }
    }
    return lsm_writeback(base, count, dir);
}

static uint32_t mem_storeMultiple(struct ARMCore* cpu, uint32_t base, int mask,
                                  enum LSMDirection dir, int* cycles) {
    DCT3Core* c = CORE(cpu);
    int count = __builtin_popcount((unsigned)(mask & 0xFFFF));
    uint32_t addr = lsm_start(base, count, dir);
    for (int i = 0; i < 16; ++i) {
        if (mask & (1 << i)) {
            // STM is N×STR: each word must take the SAME path as mem_store32 — write RAM (unless the
            // device owns it, in_io2) AND dispatch to the platform io_write. The old code wrote RAM
            // ONLY, so an STM into a device register never reached the handler. That silently lost the
            // MCU's 32-bit STM upload of the DSP loader into the HPI window (PC 0x2936F4
            // `stmia r1!,{r3-r12}` → 0x10E00): the bytes hit raw RAM, not the DSP DARAM the window
            // maps to. (STR/STRH already dispatched, which is why only the STM-staged loader was lost.)
            if (!in_io2(c, addr)) ram_wr32(c, addr, (uint32_t)cpu->gprs[i]);
            io_wr(c, addr, 4, (uint32_t)cpu->gprs[i]);
            addr += 4;
            if (cycles) *cycles += 1;
        }
    }
    return lsm_writeback(base, count, dir);
}

static int32_t mem_stall(struct ARMCore* cpu, int32_t wait) {
    (void)cpu;
    return wait;  // flat RAM: no wait states
}

static void mem_setActiveRegion(struct ARMCore* cpu, uint32_t address) {
    (void)address;  // single region covers the whole space
    DCT3Core* c = CORE(cpu);
    cpu->memory.activeRegion = (const uint32_t*)c->ram;  // byte base for LOAD_* macros
    cpu->memory.activeMask = DCT3_RAM_MASK;              // bit1 set so Thumb fetch works
    // ARM7TDMI cycle model. The mGBA core charges every instruction a base of
    // (1 + activeSeqCycles) cycles for the sequential prefetch; activeSeqCycles is
    // the memory's *added* wait states.
    //
    // CALIBRATION (contested — settle against the real 3310): seq=0 gives cpi ~1.64,
    // which matches the BARE ARM7TDMI core (NOP=1S, LDR=3) running from 0-wait memory.
    // But the firmware executes IN PLACE from wait-stated NOR flash, so the real
    // effective cpi is higher. Two independent lines of evidence say ~2.7:
    //   - the v6.39 Snake ran ~1.8x too fast at the old assumed cpi 1.5 (=> ~2.7);
    //   - instruction-paced firmware logic (busy-poll retry counters, menu/backlight
    //     reset windows) outruns the cycle-paced timers (Timer0/1 = cycles*33055/13e6)
    //     by ~1.6x at cpi 1.64 -> timeouts feel early, the recognition retry budget
    //     burns out, the menu-shortcut window expires between keys.
    // The run loop now paces to a real 13 MHz *cycle* clock, so seq=1 (one flash wait
    // state, cpi ~2.64) makes instruction throughput ~5M insns/s = real silicon, which
    // is what corrects the "too fast" timer feel. (An earlier session set seq=0 calling
    // 1.64 "accurate to the core" and 2.7 "too slow"; that predates pinning the symptoms
    // to the instruction-vs-cycle skew. VERIFY against the real phone — time the
    // backlight-off / screensaver / a stopwatch tick — and adjust the wait state here.)
    // TODO: a per-region model (wait-stated flash, 0-wait SRAM) would be exact; the RAM-
    // resident flash-program loop is the only hot code that runs wait-state-free.
    cpu->memory.activeSeqCycles32 = 1;
    cpu->memory.activeNonseqCycles32 = 1;
    cpu->memory.activeSeqCycles16 = 1;
    cpu->memory.activeNonseqCycles16 = 1;
}

// --- Interrupt-handler vtable ------------------------------------------------

static void irq_reset(struct ARMCore* cpu) { (void)cpu; }

// No scheduled devices yet: push the next event far out so ARMRun's event loop
// (`while (cycles >= nextEvent) processEvents`) terminates after one call.
// cpu->cycles is a signed int32 with no real events to rebase it, so it grows
// unbounded — and `cycles + SLICE` overflows int32 (signed UB -> a wasm trap,
// observed ~398M instructions) once cycles passes the slice. Use a half-int32
// slice and rebase cycles down each window so `cycles` stays in [0, SLICE] and
// `cycles + SLICE` <= 0x40000000 < INT32_MAX. The only cycle-derived consumers
// (mad2 FIQ4 / Timer0) edge-detect and tolerate the periodic rebase (~one
// extra benign tick per window, ~every 40 s of emulated time).
#define DCT3_EVENT_SLICE 0x20000000

static void irq_processEvents(struct ARMCore* cpu) {
    if (cpu->cycles >= DCT3_EVENT_SLICE) cpu->cycles -= DCT3_EVENT_SLICE;
    cpu->nextEvent = cpu->cycles + DCT3_EVENT_SLICE;
}
static void irq_swi16(struct ARMCore* cpu, int imm)        { CORE(cpu)->last_swi = (uint32_t)imm; CORE(cpu)->swi_count++; }
static void irq_swi32(struct ARMCore* cpu, int imm)        { CORE(cpu)->last_swi = (uint32_t)imm; CORE(cpu)->swi_count++; }
static void irq_hitIllegal(struct ARMCore* cpu, uint32_t op){ CORE(cpu)->last_illegal = op; CORE(cpu)->illegal_count++; }
static void irq_bkpt16(struct ARMCore* cpu, int imm)       { (void)cpu; (void)imm; }
static void irq_bkpt32(struct ARMCore* cpu, int imm)       { (void)cpu; (void)imm; }
static void irq_readCPSR(struct ARMCore* cpu)              { (void)cpu; }
static void irq_hitStub(struct ARMCore* cpu, uint32_t op)  { CORE(cpu)->last_stub = op; CORE(cpu)->stub_count++; }

// --- Master CPU component (mGBA requires one for ARMInit) --------------------

static void master_init(void* cpu, struct mCPUComponent* component) { (void)cpu; (void)component; }
static struct mCPUComponent g_master = { 0, master_init, NULL };

// --- Lifecycle ---------------------------------------------------------------

DCT3Core* dct3_core_create(void) {
    DCT3Core* core = (DCT3Core*)calloc(1, sizeof(DCT3Core));
    if (!core) return NULL;
    core->ram = (uint8_t*)calloc(1, DCT3_RAM_SIZE);
    if (!core->ram) { free(core); return NULL; }

    struct ARMCore* cpu = &core->cpu;
    cpu->memory.load32 = mem_load32;
    cpu->memory.load16 = mem_load16;
    cpu->memory.load8 = mem_load8;
    cpu->memory.store32 = mem_store32;
    cpu->memory.store16 = mem_store16;
    cpu->memory.store8 = mem_store8;
    cpu->memory.loadMultiple = mem_loadMultiple;
    cpu->memory.storeMultiple = mem_storeMultiple;
    cpu->memory.stall = mem_stall;
    cpu->memory.setActiveRegion = mem_setActiveRegion;
    cpu->memory.accessSource = mACCESS_PROGRAM;
    mem_setActiveRegion(cpu, 0);

    cpu->irqh.reset = irq_reset;
    cpu->irqh.processEvents = irq_processEvents;
    cpu->irqh.swi16 = irq_swi16;
    cpu->irqh.swi32 = irq_swi32;
    cpu->irqh.hitIllegal = irq_hitIllegal;
    cpu->irqh.bkpt16 = irq_bkpt16;
    cpu->irqh.bkpt32 = irq_bkpt32;
    cpu->irqh.readCPSR = irq_readCPSR;
    cpu->irqh.hitStub = irq_hitStub;

    ARMSetComponents(cpu, &g_master, 0, NULL);
    ARMInit(cpu);
    return core;
}

void dct3_core_destroy(DCT3Core* core) {
    if (!core) return;
    free(core->ram);
    free(core);
}

void dct3_core_reset(DCT3Core* core) {
    ARMReset(&core->cpu);
}

// --- Host-side memory helpers ------------------------------------------------

void dct3_write_bytes(DCT3Core* core, uint32_t addr, const void* data, size_t len) {
    const uint8_t* src = (const uint8_t*)data;
    for (size_t i = 0; i < len; ++i) {
        core->ram[(addr + (uint32_t)i) & DCT3_RAM_MASK] = src[i];
    }
}
void dct3_write32(DCT3Core* core, uint32_t addr, uint32_t value) { ram_wr32(core, addr, value); }
void dct3_write16(DCT3Core* core, uint32_t addr, uint16_t value) { ram_wr16(core, addr, value); }
uint32_t dct3_read32(DCT3Core* core, uint32_t addr) { return ram_rd32(core, addr); }

// Recompute & repair the MCU flash-region checksum that the firmware's boot
// integrity check verifies (5110 v5.30 = 0x276E3C; the structure is general
// across DCT3 — 3310/8850/5110 all carry it). A file-level firmware patch
// (e.g. NokiX kill_faid_check, or any FAID/feature poke) changes flash content
// without recomputing this checksum, so the verifier mismatches and the firmware
// warm-reboots (reset reason 4). A real flasher recomputes the MCU checksum on
// write; this does the same at load time so a patched image boots.
//
// Layout (big-endian flash, base 0x200000): a 16-bit stored checksum at 0x200022,
// then a region table at 0x200024 of 6-byte records {start[3], end[3]} terminated
// by start==0xFFFFFF. Checksum = 16-bit sum of BE halfwords over each region
// [start, end). Verified to reproduce the stock value on 3310/8850/5110.
//
// SAFE BY DESIGN: only rewrites when the recomputed value differs from the stored
// one, so an unpatched image is byte-identical (make guard stays green); and it
// only runs when the table is well-formed (rec0.start == 0x200024, the table's own
// self-referential anchor on every known image), so a model without this structure
// is skipped. Returns 1 if it repaired a mismatch, 0 otherwise.
int dct3_fix_mcu_checksum(DCT3Core* core) {
    const uint32_t CKSUM = 0x200022u, TBL = 0x200024u, BASE = 0x200000u;
    // Byte-accurate flash access: dct3_read32 applies ARM align+rotate semantics,
    // which mangles halfwords at non-4-aligned addresses — read bytes directly.
    #define FB(a) ((uint32_t)core->ram[(a) & DCT3_RAM_MASK])
    #define FH(a) ((FB(a) << 8) | FB((a) + 1))            // big-endian halfword
    if (((FB(TBL) << 16) | (FB(TBL+1) << 8) | FB(TBL+2)) != TBL) return 0;  // anchor gate
    uint32_t sum = 0, rec = TBL;
    for (int i = 0; i < 2; ++i) {                 // firmware sums at most 2 regions
        uint32_t start = (FB(rec)   << 16) | (FB(rec+1)   << 8) | FB(rec+2);
        if (start == 0xFFFFFFu) break;
        uint32_t end   = (FB(rec+3) << 16) | (FB(rec+4)   << 8) | FB(rec+5);
        if (start < BASE || end <= start || end > BASE + 0x400000u) return 0;  // implausible
        for (uint32_t a = start; a < end; a += 2) sum = (sum + FH(a)) & 0xFFFFu;
        rec += 6;
    }
    uint16_t stored = (uint16_t)FH(CKSUM);
    if ((uint16_t)sum == stored) return 0;        // already consistent — no-op
    core->ram[CKSUM       & DCT3_RAM_MASK] = (uint8_t)(sum >> 8);
    core->ram[(CKSUM + 1) & DCT3_RAM_MASK] = (uint8_t)(sum & 0xFF);
    // Some builds keep a REDUNDANT COPY of the MCU checksum that the boot calibration
    // self-test cross-checks against [0x200022]. Observed at 0x2FFFFA on the 5110
    // (sub-test fn 0x276B32) and 0x56FFFA on the 3410 — in both cases parked 6 bytes
    // below a 64 KiB boundary (the top of the MCU code/data partition). If only
    // 0x200022 is repaired (to satisfy the late region verifier 0x276E3C, the reason-4
    // gate) the copy mismatches -> the early verdict clears its OK bit -> CONTACT
    // SERVICE: the "checksum maze". Sync every such copy too, but ONLY where it
    // currently mirrors the OLD stored value (proving it IS that copy, not unrelated
    // data), so unpatched/other images stay byte-identical. Scan window spans the
    // largest DCT3 flash (4 MiB) + margin; calloc'd RAM beyond the loaded image reads
    // 0x0000 and never matches a nonzero checksum.
    if (stored != 0) {
        for (uint32_t blk = BASE; blk < BASE + 0x600000u; blk += 0x10000u) {
            uint32_t ref = blk + 0xFFFAu;
            if ((uint16_t)FH(ref) != stored) continue;
            core->ram[ref       & DCT3_RAM_MASK] = (uint8_t)(sum >> 8);
            core->ram[(ref + 1) & DCT3_RAM_MASK] = (uint8_t)(sum & 0xFF);
        }
    }
    return 1;
    #undef FB
    #undef FH
}

// --- Execution ---------------------------------------------------------------

void dct3_step(DCT3Core* core) { ARMRun(&core->cpu); }
void dct3_run(DCT3Core* core, int n) { for (int i = 0; i < n; ++i) ARMRun(&core->cpu); }

// --- Phase 2 bring-up helpers ------------------------------------------------

void dct3_set_io_hooks(DCT3Core* core, void* ctx, uint32_t lo, uint32_t hi,
                       dct3_io_read_fn read, dct3_io_write_fn write) {
    core->io_ctx = ctx;
    core->io_lo = lo;
    core->io_hi = hi;
    core->io_read = read;
    core->io_write = write;
}

void dct3_set_io_range2(DCT3Core* core, uint32_t lo, uint32_t hi) {
    core->io2_lo = lo;
    core->io2_hi = hi;
}

void dct3_set_mem_trace(DCT3Core* core, void* ctx,
                        void (*cb)(void*, uint32_t, uint32_t, int, uint32_t, uint32_t, int),
                        int reads) {
    core->mem_trace_ctx = ctx;
    core->mem_trace = cb;
    core->mem_trace_reads = (uint8_t)(reads ? 1 : 0);
}

// Force PC + CPSR — the resume primitive for reset-reason recovery (see mad2's
// recover_pending path and docs/watchdog-reset-3310.md). Used to land back at the
// instruction the watchdog/fatal handler interrupted (reason 5: PC from *0x11FF88,
// CPSR from *0x11FF8C). Replays the same steps the core uses on an MSR CPSR + branch:
// write CPSR (which banks SP/LR + sets ARM/Thumb), write PC, refill the prefetch pair.
void dct3_core_force_pc_cpsr(DCT3Core* core, uint32_t pc, uint32_t cpsr, uint32_t sp) {
    struct ARMCore* cpu = &core->cpu;
    cpu->cpsr.packed = cpsr;
    _ARMReadCPSR(cpu);                                 // banks SP/LR + sets execution mode
    if (sp) cpu->gprs[ARM_SP] = sp;                    // multi-frame pop: skip teardown frames
    cpu->gprs[ARM_PC] = pc & ~1u;                      // drop the Thumb-bit from a saved-LR convention
    if (cpu->cpsr.t) cpu->cycles += ThumbWritePC(cpu); // refill the prefetch pair
    else             cpu->cycles += ARMWritePC(cpu);
    cpu->halted = 0;                                   // a halted core (WFI etc.) resumes too
}

// HLE the MAD boot ROM: drop a single ARM branch at the reset vector (0) that
// jumps to the flash entry, then reset so the core's own pipeline priming runs.
void dct3_boot_at(DCT3Core* core, uint32_t entry) {
    uint32_t off = ((entry - 8u) >> 2) & 0x00FFFFFFu;   // 24-bit ARM branch offset
    dct3_write32(core, 0, 0xEA000000u | off);
    dct3_core_reset(core);
}
