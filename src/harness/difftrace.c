// DIFFTRACE — native(gcc) vs wasm(emcc) lockstep guest-state differential trace
// (Phase 9, plan 09-02, the H4 DIFFTRACE-PIN step). Observation-only; gated OFF by
// default; the disarmed path is byte-identical (env probe = no-op when unset).
//
// PURPOSE. The two prior RE docs (09-02-H4-VERDICT / -SCHED-RE) pinned the no-SIM
// reason-0x6C reset to a self-sustaining message storm at ~54.6M cyc that occurs on
// the wasm build but never on native — classified as a guest-execution divergence in
// the vendored ARM core (src/core/dct3_core.c) between gcc/x86-64 and emcc/wasm32.
// The storm is a LATE symptom; the FIRST instruction where the two builds compute a
// different {PC, register, CPSR} value is the real target. Both builds start from an
// identical boot state, take identical input, and step exactly one dct3_step per the
// harness `step` counter (boot_trace.c:1118+1130, main.c:589+803), so absent a UB/
// width bug they are byte-for-byte lockstep. The FIRST mismatch IS the bug.
//
// METHOD. Each step folds {pc, r0..r15, cpsr.packed} into a rolling FNV-1a-64 hash.
// Every DIFFTRACE_EVERY steps (default 1,000,000) the (step, rolling-hash) checkpoint
// is printed to stderr as a `DIFFTRACE chk` line. Running both builds and diffing the
// checkpoint streams locates the first divergent WINDOW cheaply (no 54M-step full
// trace). Then a second pass with DIFFTRACE_LO/HI set to that window prints a full
// per-step `DIFFTRACE row` {step, pc, r0..r15, cpsr} so the two builds diff to the
// FIRST mismatching step + instruction.
//
// Observation-only invariant: reads ONLY cpu->gprs + cpu->cpsr.packed (read-only);
// NEVER writes guest RAM, cpu->gprs, or cpu->cycles. No host allocation in the hook.

#include "harness/harness.h"
#include <mgba/internal/arm/arm.h>
#include <stdio.h>
#include <stdlib.h>

// --- env-probe gating (gated-OFF idiom, mirrors telemetry.c:tlm_probe) -------
static int      dt_on        = -2;          // -2 unprobed; 0 off; 1 on
static int64_t  dt_every     = 1000000;     // checkpoint cadence (steps)
static int64_t  dt_lo        = 0;           // full-row window [lo, hi] (0,0 = none)
static int64_t  dt_hi        = 0;
static uint64_t dt_hash      = 1469598103934665603ULL;  // FNV-1a-64 offset basis
static int64_t  dt_next_chk  = 0;

static void dt_probe(void) {
    if (dt_on != -2) return;
    const char* e = getenv("DIFFTRACE");
    if (!e) { dt_on = 0; return; }
    dt_on = 1;
    const char* ev = getenv("DIFFTRACE_EVERY");
    if (ev) { long long v = strtoll(ev, NULL, 0); if (v > 0) dt_every = (int64_t)v; }
    const char* lo = getenv("DIFFTRACE_LO");
    const char* hi = getenv("DIFFTRACE_HI");
    if (lo) dt_lo = (int64_t)strtoll(lo, NULL, 0);
    if (hi) dt_hi = (int64_t)strtoll(hi, NULL, 0);
}

int difftrace_armed(void) {
    dt_probe();
    return dt_on ? 1 : 0;
}

// Web cwrap arm: the dev panel / node harness flips the same static gate. on=1 resets
// the rolling hash so a run starts clean. every>0 sets the cadence; lo/hi set the
// optional full-row window (0,0 = checkpoints only).
void difftrace_arm(int on, double every, double lo, double hi) {
    dt_probe();
    dt_on = on ? 1 : 0;
    if (every > 0) dt_every = (int64_t)every;
    dt_lo = (int64_t)lo;
    dt_hi = (int64_t)hi;
    if (dt_on) { dt_hash = 1469598103934665603ULL; dt_next_chk = 0; }
}

static inline void dt_fold(uint32_t v) {
    // FNV-1a-64 over 4 bytes (endianness-stable: we feed the host-order u32 byte by
    // byte high-to-low, identical arithmetic on x86-64 and wasm32).
    dt_hash ^= (uint64_t)((v >> 24) & 0xFF); dt_hash *= 1099511628211ULL;
    dt_hash ^= (uint64_t)((v >> 16) & 0xFF); dt_hash *= 1099511628211ULL;
    dt_hash ^= (uint64_t)((v >>  8) & 0xFF); dt_hash *= 1099511628211ULL;
    dt_hash ^= (uint64_t)( v        & 0xFF); dt_hash *= 1099511628211ULL;
}

// Called from harness_observe (gated). Folds the current pre-step guest state into the
// rolling hash, prints a checkpoint at the cadence, and (if a window is armed) prints a
// full per-step row. `pc` is the de-piped PC the caller already computed.
void difftrace_step(struct ARMCore* cpu, uint32_t pc, int64_t step) {
    dt_probe();
    if (!dt_on || !cpu) return;

    // Fold {pc, r0..r15, cpsr.packed} — the complete architectural guest state that a
    // faithful core MUST reproduce identically. (gprs[15] is the live PC; we fold both
    // the de-piped pc and the raw gprs[15] so a pipeline-state divergence also shows.)
    dt_fold(pc);
    for (int i = 0; i < 16; ++i) dt_fold((uint32_t)cpu->gprs[i]);
    dt_fold(cpu->cpsr.packed);

    // Checkpoint at the cadence (a compact diffable stream).
    if (step >= dt_next_chk) {
        dt_next_chk = step + dt_every;
        fprintf(stderr, "DIFFTRACE chk step=%lld hash=%016llx pc=%08x\n",
                (long long)step, (unsigned long long)dt_hash, pc);
    }

    // Full per-step row inside the armed window (the bisect's final pass).
    if (dt_hi > dt_lo && step >= dt_lo && step <= dt_hi) {
        fprintf(stderr,
            "DIFFTRACE row %lld pc=%08x cpsr=%08x "
            "r0=%08x r1=%08x r2=%08x r3=%08x r4=%08x r5=%08x r6=%08x r7=%08x "
            "r8=%08x r9=%08x r10=%08x r11=%08x r12=%08x sp=%08x lr=%08x pc15=%08x\n",
            (long long)step, pc, cpu->cpsr.packed,
            (uint32_t)cpu->gprs[0],  (uint32_t)cpu->gprs[1],  (uint32_t)cpu->gprs[2],
            (uint32_t)cpu->gprs[3],  (uint32_t)cpu->gprs[4],  (uint32_t)cpu->gprs[5],
            (uint32_t)cpu->gprs[6],  (uint32_t)cpu->gprs[7],  (uint32_t)cpu->gprs[8],
            (uint32_t)cpu->gprs[9],  (uint32_t)cpu->gprs[10], (uint32_t)cpu->gprs[11],
            (uint32_t)cpu->gprs[12], (uint32_t)cpu->gprs[13], (uint32_t)cpu->gprs[14],
            (uint32_t)cpu->gprs[15]);
    }
}
