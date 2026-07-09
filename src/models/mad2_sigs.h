// Shared MAD2 RTOS firmware-address signatures. These masked byte-patterns identify
// the universal OS functions (reboot sink, fatal handler, assert logger, reason setter,
// blocking allocator, SIM gate, task-14 state machine) by code SHAPE, with operands and
// pool addresses wildcarded — so they locate the same function across MAD2 DCT3 builds
// regardless of where the linker placed it. Hoisted out of src/models/3310/profile.c so
// every MAD2 model (3310, 3410, 8850, 7110, ...) resolves from one table.
//
// Search windows are left as 0/0 in the table so model_resolve() defaults each to the
// profile's own flash region. sig_find() returns the LOWEST match, and code only lives
// in the MCU partition (low addresses) — so a wider ceiling (PPM/EEPROM above) never
// changes the first match. The 3310's results are therefore identical to when these
// carried explicit 0x200000..0x340000 windows.
//
// Coverage note (empirically): the 3410 (NHM-2 v5.46 = a newer 3310, same base OS)
// resolves 7 of 10 fields — sim_gate / reboot_fn / reboot_reason / reboot_save /
// fatal_handler / assert_log / reason_setter (the last after wildcarding its trailing
// classifier's drifted register bits). malloc_fail and the task14 STATE_FANOUT still
// miss: their code shape was RESTRUCTURED for v5.46 (not just register drift), so they
// need real RE to re-anchor — both are post-boot (heap telemetry / DSP-keepalive gate),
// so non-blocking for bring-up.

#ifndef DCT3_MAD2_SIGS_H
#define DCT3_MAD2_SIGS_H

#include "models/model.h"

extern const SigResolve MAD2_SIGS[];
// Macro (not a const int) so it is a compile-time constant usable in the static
// ModelProfile initializers (.n_sigs = MAD2_N_SIGS). mad2_sigs.c static-asserts that
// it equals the actual array length.
#define MAD2_N_SIGS 16

// 5210-specific overlay table (wired via ModelProfile.sigs2). Signature-resolves the two
// per-build gate RAM variables — verdict and dsp_uploaded — whose code shape recurs across
// the 5210 build line (v5.13..v5.40) but whose VALUE drifts per build (dsp_uploaded:
// v5.13=0x13A8EC, v5.18-22=0x13B150, v5.25-40=0x13B528). Kept OUT of the shared MAD2_SIGS
// because the dsp_uploaded writer shape also recurs on 8210/8850/8855 where the faithful
// site differs — see model.h ModelProfile.sigs2.
extern const SigResolve MAD2_SIGS_5210[];
#define MAD2_N_SIGS_5210 2

// 3310/NHM-5 overlay table (wired via ModelProfile.sigs2). Signature-resolves verdict +
// dsp_uploaded for the 3310 build line — the 5210 sigs above don't match the 3310's code
// shapes, so the NHM-5 line carries its own anchors. Verified on v5.79 (== old constants,
// byte-identical boot), v6.39 and the v5.57 mod (each a distinct true cell). See mad2_sigs.c.
extern const SigResolve MAD2_SIGS_3310[];
#define MAD2_N_SIGS_3310 2

#endif // DCT3_MAD2_SIGS_H
