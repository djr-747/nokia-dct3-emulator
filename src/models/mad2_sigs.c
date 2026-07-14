// Shared MAD2 RTOS firmware-address signatures (see mad2_sigs.h). Moved verbatim from
// src/models/3310/profile.c; the per-entry search windows are now 0/0 (model_resolve
// defaults them to each profile's flash region — first-match is unchanged, see header).

#include "models/mad2_sigs.h"

#include <stddef.h>   // offsetof

// disp49 (sim_func) prologue signature → the SIM-gate bypass byte. Locates the
// function regardless of firmware version; the gate byte is the RAM base literal it
// loads (ldr r1,=<base> at +0x0E) + 29.
//   PUSH{lr}; cmp r0,=0x366; beq; cmp r0,=0x367; beq; ldr r1,=<base>; ...
static const uint8_t SIM_PAT[16] = {0xB5,0x00,0x49,0x30,0x42,0x88,0xD0,0x23,
                                    0x49,0x2D,0x42,0x88,0xD0,0x20,0x49,0x20};
static const uint8_t SIM_MSK[16] = {0xFF,0xFF,0xFF,0x00,0xFF,0xFF,0xF8,0x00,
                                    0xFF,0x00,0xFF,0xFF,0xF8,0x00,0xFF,0x00};

// Universal reboot fn prologue (0x2EEBAE on 3310 v5.79). The fn is a noreturn sink: every
// reset path bl's it with a reason in r0. Distinctive opening: it saves the reason to
// r4, calls a notify helper, then dispatches on r4 ∈ {3,4,2} with a tight cmp/beq chain.
//   mov r4,r0; bl ???; cmp r4,#3; beq; cmp r4,#4; beq; cmp r4,#2; bne ???
// SIG_LITERAL piggybacks: at offset 0x30 from the prolog sits `ldr rX, =reboot_reason`.
// The save block lives 0x0C bytes earlier (struct: 12 bytes {saved_LR,SPSR,CPSR} then the
// reason byte) so we sig the same ldr twice — addend 0 for the reason, -0x0C for the save.
static const uint8_t REBOOT_PAT[18] = {
    0x1C,0x04,  0xF0,0x00,0xF8,0x00,  0x2C,0x03,  0xD0,0x00,
    0x2C,0x04,  0xD0,0x00,  0x2C,0x02,  0xD1,0x00
};
static const uint8_t REBOOT_MSK[18] = {
    0xFF,0xFF,  0xF8,0x00,0xF8,0x00,  0xFF,0xFF,  0xFF,0x00,
    0xFF,0xFF,  0xFF,0x00,  0xFF,0xFF,  0xFF,0x00
};

// reboot_fn prologue VARIANT without the early `bl notify` (NSM-2 8850 v5.31):
//   1C04 mov r4,r0 ; 2C03 cmp r4,#3 ; D0xx beq ; 2C04 cmp #4 ; D0xx beq ; 2C02 cmp #2 ; D1xx bne
// The 3310/3410 REBOOT_PAT has a `bl` between mov r4,r0 and the cmp chain; this one does
// not, so it is a separate pattern. The ldr=reboot_reason sits 4 bytes earlier than the
// 3310's (shorter prologue) → lit_off 0x2C instead of 0x30. first-hit-wins keeps the
// 3310/3410 on REBOOT_PAT (this variant doesn't match their bl prologue anyway).
static const uint8_t REBOOT8_PAT[14] = {
    0x1C,0x04,  0x2C,0x03,  0xD0,0x00,  0x2C,0x04,  0xD0,0x00,  0x2C,0x02,  0xD1,0x00
};
static const uint8_t REBOOT8_MSK[14] = {
    0xFF,0xFF,  0xFF,0xFF,  0xFF,0x00,  0xFF,0xFF,  0xFF,0x00,  0xFF,0xFF,  0xFF,0x00
};

// reason-stamp leaf VARIANT (8850): a minimal `49xx ldr r1,=X ; 7008 strb r0,[r1] ; 46F7
// mov pc,lr`. This shape recurs (function tails), so SIG_CODE_VLIT accepts only the match
// whose loaded literal == reboot_reason. lit_off 0 (the ldr is the first insn).
static const uint8_t RSET8_PAT[6] = { 0x49,0x00, 0x70,0x08, 0x46,0xF7 };
static const uint8_t RSET8_MSK[6] = { 0xFF,0x00, 0xFF,0xFF, 0xFF,0xFF };

// ARM fatal-exception handler entry (0x2F0BA4 on 3310 v5.79). The four fatal vectors all
// B here. ARM mode, starts with `ldr r0,=<saved-state-base>` + a distinctive save+mode-flip:
//   E59F00xx ; ldr r0,=save  | E1A0100E ; mov r1,lr | E14F2000 ; mrs r2,spsr
//   E10F3000 ; mrs r3,cpsr   | E880000E ; stmia r0,{r1-r3} | E28F0001 ; add r0,pc,#1
//   E12FFF10 ; bx r0  (-> Thumb). SIG_CODE: the match offset IS the address.
static const uint8_t FATAL_PAT[28] = {
    0xE5,0x9F,0x00,0x00,  0xE1,0xA0,0x10,0x0E,
    0xE1,0x4F,0x20,0x00,  0xE1,0x0F,0x30,0x00,
    0xE8,0x80,0x00,0x0E,  0xE2,0x8F,0x00,0x01,
    0xE1,0x2F,0xFF,0x10,
};
static const uint8_t FATAL_MSK[28] = {
    0xFF,0xFF,0x00,0x00,  0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,  0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,  0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,
};

// Generic assertion logger entry (0x2EDD4A on 3310 v5.79). Prologue: push lr + 1-word slot
// + strh r1 + a 3-way cmp/beq chain against three special codes (rest fall through):
//   B500 push{lr}; B081 sub sp,#4; 466A mov r2,sp; 8011 strh r1,[r2];
//   49xx ldr r1,=0x5C12; 4288 cmp r0,r1; D0xx beq; (x3 against 0x5C12/0x5C0F/0x5C13)
static const uint8_t ASSERT_PAT[22] = {
    0xB5,0x00, 0xB0,0x81, 0x46,0x6A, 0x80,0x11,
    0x49,0x00, 0x42,0x88, 0xD0,0x00,
    0x49,0x00, 0x42,0x88, 0xD0,0x00,
    0x49,0x00,
};
static const uint8_t ASSERT_MSK[22] = {
    0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF,
    0xFF,0x00, 0xFF,0xFF, 0xFF,0x00,
    0xFF,0x00, 0xFF,0xFF, 0xFF,0x00,
    0xFF,0x00,
};

// Reason-byte setter (0x2E0ED8 on 3310 v5.79). ldr r1,=reason; strb r0,[r1]; bx lr — the
// 3-byte body matches ~15 setters, so the sig extends through the adjacent boot-state
// classifier's opener (state read + cmp chain against {1,6,11,104}):
//   49xx ldr r1,=reason; 7008 strb r0,[r1]; 46F7 mov pc,lr;
//   2x00 mov rA,#0; 4xxx ldr rB,=state; 78xx ldrb rB,[rB]; 2x01 cmp rB,#1
// The trailing classifier's register allocation drifts between builds (3310 v5.79 uses
// r1/r0; 3410 v5.46 swaps to r0/r1), so the rd/rn bits are wildcarded (0xF8 on the
// mov/ldr/cmp opcodes, 0x80 on the ldrb offset byte). Resolves 3310 0x2E0ED8 + 3410
// 0x3B3D9A, each uniquely.
static const uint8_t REASON_SETTER_PAT[14] = {
    0x49,0x00, 0x70,0x08, 0x46,0xF7,
    0x20,0x00, 0x48,0x00, 0x78,0x00, 0x28,0x01,
};
static const uint8_t REASON_SETTER_MSK[14] = {
    0xFF,0x00, 0xFF,0xFF, 0xFF,0xFF,
    0xF8,0xFF, 0xF8,0x00, 0xFF,0x80, 0xF8,0xFF,
};

// Heap allocation-failure / block-retry PC (0x299B26 on 3310 v5.79). On NULL from the heap
// core the blocking allocator branches here: zero a flag, strb busy-state, park on a memory
// semaphore, restore state, retry. SIG_CODE anchored AT the fail block:
//   4648 mov r0,r9; 7006 strb r6,[r0]; 4640 mov r0,r8; 7328 strb r0,[r5,#12];
//   F0..F8.. bl <sema>; 732F strb r7,[r5,#12]; 1C20 mov r0,r4; F0..F8.. bl <heap>;
//   2800 cmp r0,#0; D0.. beq <fail>
static const uint8_t MALLOC_FAIL_PAT[24] = {
    0x46,0x48, 0x70,0x06, 0x46,0x40, 0x73,0x28,
    0xF0,0x00,0xF8,0x00,  0x73,0x2F, 0x1C,0x20,
    0xF0,0x00,0xF8,0x00,  0x28,0x00, 0xD0,0x00,
};
static const uint8_t MALLOC_FAIL_MSK[24] = {
    0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF,
    0xF8,0x00,0xF8,0x00,  0xFF,0xFF, 0xFF,0xFF,
    0xF8,0x00,0xF8,0x00,  0xFF,0xFF, 0xFF,0x00,
};

// Task-14 readiness STATE_FANOUT entry (0x265588 on 3310 v5.79). Loads the task-14 state
// base (ldr r4,=base; state = [base+9]) and the per-state status table (ldr r6,=status).
// Both are what the DSP keep-alive READS to gate the 0xE4 cancel.
//   B5F0 push{r4-r7,lr}; 4641 mov r1,r8; B402 push{r1}; 4680 mov r8,r0;
//   4Cxx ldr r4,=state_base; 7A60 ldrb r0,[r4,#9]; 2158 mov r1,#88; 4341 mul r1,r0;
//   4Axx ldr r2,=struct_base; 1855 add r5,r2,r1; 4Exx ldr r6,=status_base
static const uint8_t STATE_FANOUT_PAT[22] = {
    0xB5,0xF0, 0x46,0x41, 0xB4,0x02, 0x46,0x80,
    0x4C,0x00, 0x7A,0x60, 0x21,0x58, 0x43,0x41,
    0x4A,0x00, 0x18,0x55, 0x4E,0x00,
};
static const uint8_t STATE_FANOUT_MSK[22] = {
    0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF,
    0xFF,0x00, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF,
    0xFF,0x00, 0xFF,0xFF, 0xFF,0x00,
};

// 3410 v5.46 STATE_FANOUT variant (0x2871C8). SAME readiness dispatcher (state*88
// struct stride, status table indexed, cmp #17 jump table) as the 3310 entry above,
// but a different prologue and state-byte offset, so the 22-byte pattern misses:
//   3310: B5F0 4641 B402 4680 (saves r8) ; ldrb r0,[r4,#9]
//   3410: B5F0 B081 1C07      (push;sub sp,#4;mov r7,r0) ; ldrb r0,[r4,#2]
//   B5F0 push{r4-r7,lr}; B081 sub sp,#4; 1C07 mov r7,r0;
//   4Cxx ldr r4,=state_base; 78A0 ldrb r0,[r4,#2]; 2158 mov r1,#88; 4341 mul r1,r0;
//   4Axx ldr r2,=struct_base; 1855 add r5,r2,r1; 4Exx ldr r6,=status_base
// → task14_state = state_base(0x121A38)+2 = 0x121A3A ; task14_status = 0x121DD4.
static const uint8_t STATE_FANOUT2_PAT[20] = {
    0xB5,0xF0, 0xB0,0x81, 0x1C,0x07, 0x4C,0x00,
    0x78,0xA0, 0x21,0x58, 0x43,0x41, 0x4A,0x00,
    0x18,0x55, 0x4E,0x00,
};
static const uint8_t STATE_FANOUT2_MSK[20] = {
    0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0x00,
    0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0x00,
    0xFF,0xFF, 0xFF,0x00,
};

// --- 5210 per-build gate writers (overlay table MAD2_SIGS_5210, ModelProfile.sigs2) ---
//
// VERDICT writer prologue (5210 v5.40 @0x25D274 / v5.22 @0x25D268). The MMI-ready/self-test
// verdict byte = base 0x17FD7C + 0x69 (the 0x69 offset is the 3310/3410/8850 invariant). The
// fn sets `mov r6,#0x69 ; ldr r5,=<base> ; mov r0,r5 ; mov r1,#imm` — sig the ldr (lit_off 2)
// and add 0x69. Unique single hit across every 5210 build (and the wider DCT3 family it
// resolves correctly: 7110→0x17FE15, 8250→0x13FDD9, ... — hence family-wide-safe but kept
// 5210-scoped because dsp_uploaded below is NOT).
static const uint8_t VERDICT5_PAT[8] = { 0x26,0x69, 0x4D,0x00, 0x1C,0x28, 0x21,0x00 };
static const uint8_t VERDICT5_MSK[8] = { 0xFF,0xFF, 0xFF,0x00, 0xFF,0xFF, 0xFF,0x00 };

// DSP_UPLOADED setter (5210 v5.40 @0x345E1C / v5.22 @0x345D7C). The DSP-ready ISR's sticky
// gate byte: `mov r0,#16 ; strb r0,[r5,#9] ; ldr r4,=<gate> ; mov r9,r5 ; ldrb r0,[r4] ; cmp
// r0,#0`. Sig the ldr (lit_off 4), addend 0. The drifting value this resolves: v5.13=0x13A8EC,
// v5.18-22=0x13B150, v5.25-40=0x13B528 — the constant 0x13B528 fallback only fit v5.25+, which
// is why a v5.22 dump stalled pre-MMI. Distinctive 12-byte shape, unique per 5210 build.
static const uint8_t DSPUP5_PAT[12] = { 0x20,0x10, 0x72,0x68, 0x4C,0x00, 0x46,0xA9, 0x78,0x20, 0x28,0x00 };
static const uint8_t DSPUP5_MSK[12] = { 0xFF,0xFF, 0xFF,0xFF, 0xFF,0x00, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF };

// --- 3310/NHM-5 per-build gate sites (overlay table MAD2_SIGS_3310, ModelProfile.sigs2) ---
//
// Same job as the 5210 overlay below, but for the NHM-5 (3310) build line. The 5210's
// VERDICT5/DSPUP5 writer shapes DO NOT occur in the 3310 firmware (verified: they fall back
// to the profile constant on every 3310 build), so the 3310 line needs its own anchors.
// Both resolve UNIQUELY and correctly on v5.79 + v6.39 + a community v5.57 mod — and each
// build genuinely places these RAM cells differently (dsp_uploaded: v5.79=0x11038C,
// v6.39=0x11079C, v5.57=0x110360), which is exactly why a single hardcoded constant was
// silently wrong off-v5.79.
//
// VERDICT site (3310 v5.79 @0x241276): the self-test verdict-byte bit3 gate —
//   `4D.. ldr r5,=verdict ; 7828 ldrb r0,[r5] ; 08C0 lsr r0,#3 ; D3.. bcc ; 201B mov r0,#27`.
// The verdict byte IS the loaded literal (lit_off 0, addend 0). r5/r0 + the #3 shift + the
// mov #27 follow-on make it a single hit family-line-wide; the bcc displacement is wildcarded.
static const uint8_t VERDICT3_PAT[10] = { 0x4D,0x00, 0x78,0x28, 0x08,0xC0, 0xD3,0x00, 0x20,0x1B };
static const uint8_t VERDICT3_MSK[10] = { 0xFF,0x00, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0x00, 0xFF,0xFF };

// DSP_UPLOADED site (3310 v5.79 @0x2BAF82): the upload-complete flag STRUCT initializer.
// `48.. ldr r0,=dsp_uploaded ; 2101 mov #1 ; 03C9 lsl #15 ; 8041 strh[r0,#2] ; 2109 mov #9 ;
//  0309 lsl #12 ; 8081 strh[r0,#4] ; 2104 mov #4 ; 81C1 strh[r0,#14]`. The `#1<<15, #9<<12,
// #4` constant payload is highly distinctive (single hit per build); the loaded literal is the
// flag base the gate reads (lit_off 0, addend 0). Only the ldr's pool-offset byte is wildcarded.
static const uint8_t DSPUP3_PAT[18] = { 0x48,0x00, 0x21,0x01, 0x03,0xC9, 0x80,0x41, 0x21,0x09,
                                        0x03,0x09, 0x80,0x81, 0x21,0x04, 0x81,0xC1 };
static const uint8_t DSPUP3_MSK[18] = { 0xF8,0x00, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF,
                                        0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF };

// MMI message poster (send_message): `lsl r0,#16; stmdb sp!,{r0-r3}; stmdb sp!,{r4,r5,r7,lr};
// add r7,sp,#16`. The shift-msgid-into-high-half + 4-arg shadow-save + push is a distinctive,
// build-invariant prologue — verified UNIQUE on 3310 v5.79 (0x2E8A1A), 3410 v5.46 (0x39FF9C),
// 8850 v5.31 (0x301564). Fully fixed bytes (no operand drift in the prologue).
static const uint8_t MMI_SEND_PAT[8] = { 0x04,0x00, 0xB4,0x0F, 0xB5,0xB0, 0xAF,0x04 };
static const uint8_t MMI_SEND_MSK[8] = { 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF };

// lo/hi = 0,0 → model_resolve() uses [flash_base, flash_base+flash_size) per profile.
const SigResolve MAD2_SIGS[] = {
    { "sim_gate", offsetof(FwAddrs, sim_gate),
      { SIG_LITERAL, SIM_PAT, SIM_MSK, 16, /*lit_off*/0x0E, /*addend*/29, 0, 0 } },
    { "reboot_fn", offsetof(FwAddrs, reboot_fn),
      { SIG_CODE, REBOOT_PAT, REBOOT_MSK, 18, 0, 0, 0, 0 } },
    { "reboot_reason", offsetof(FwAddrs, reboot_reason),
      { SIG_LITERAL, REBOOT_PAT, REBOOT_MSK, 18, /*lit_off*/0x30, /*addend*/0, 0, 0 } },
    { "reboot_save", offsetof(FwAddrs, reboot_save),
      { SIG_LITERAL, REBOOT_PAT, REBOOT_MSK, 18, /*lit_off*/0x30, /*addend*/-0x0C, 0, 0 } },
    { "fatal_handler", offsetof(FwAddrs, fatal_handler),
      { SIG_CODE, FATAL_PAT, FATAL_MSK, 28, 0, 0, 0, 0 } },
    { "assert_log", offsetof(FwAddrs, assert_log),
      { SIG_CODE, ASSERT_PAT, ASSERT_MSK, 22, 0, 0, 0, 0 } },
    { "reason_setter", offsetof(FwAddrs, reason_setter),
      { SIG_CODE, REASON_SETTER_PAT, REASON_SETTER_MSK, 14, 0, 0, 0, 0 } },
    { "malloc_fail", offsetof(FwAddrs, malloc_fail),
      { SIG_CODE, MALLOC_FAIL_PAT, MALLOC_FAIL_MSK, 24, 0, 0, 0, 0 } },
    { "task14_state", offsetof(FwAddrs, task14_state),
      { SIG_LITERAL, STATE_FANOUT_PAT, STATE_FANOUT_MSK, 22, /*lit_off*/0x08, /*addend*/9, 0, 0 } },
    { "task14_status", offsetof(FwAddrs, task14_status),
      { SIG_LITERAL, STATE_FANOUT_PAT, STATE_FANOUT_MSK, 22, /*lit_off*/0x14, /*addend*/0, 0, 0 } },
    // --- prologue-variant fallbacks (NSM-2 8850 etc.): only fire when the primary
    //     pattern above missed (model_resolve is first-hit-wins per field). ---
    { "reboot_fn", offsetof(FwAddrs, reboot_fn),
      { SIG_CODE, REBOOT8_PAT, REBOOT8_MSK, 14, 0, 0, 0, 0 } },
    { "reboot_reason", offsetof(FwAddrs, reboot_reason),
      { SIG_LITERAL, REBOOT8_PAT, REBOOT8_MSK, 14, /*lit_off*/0x2C, /*addend*/0, 0, 0 } },
    { "reboot_save", offsetof(FwAddrs, reboot_save),
      { SIG_LITERAL, REBOOT8_PAT, REBOOT8_MSK, 14, /*lit_off*/0x2C, /*addend*/-0x0C, 0, 0 } },
    // reason_setter VLIT must come AFTER reboot_reason (verifies against it).
    { "reason_setter", offsetof(FwAddrs, reason_setter),
      { SIG_CODE_VLIT, RSET8_PAT, RSET8_MSK, 6, /*lit_off*/0, 0, 0, 0, offsetof(FwAddrs, reboot_reason) } },
    // task14 STATE_FANOUT variant (3410 v5.46 prologue): only fires when the primary
    // 22-byte pattern above missed. ldr r4,=state_base at byte 6 (+2 = state byte);
    // ldr r6,=status_base at byte 18.
    { "task14_state", offsetof(FwAddrs, task14_state),
      { SIG_LITERAL, STATE_FANOUT2_PAT, STATE_FANOUT2_MSK, 20, /*lit_off*/6, /*addend*/2, 0, 0 } },
    { "task14_status", offsetof(FwAddrs, task14_status),
      { SIG_LITERAL, STATE_FANOUT2_PAT, STATE_FANOUT2_MSK, 20, /*lit_off*/18, /*addend*/0, 0, 0 } },
    // MMI message poster — SIG_CODE (match offset IS the fn entry). Diagnostic-only
    // (SENDLOG monitor); not boot-critical, so a miss on an untested build is harmless.
    { "mmi_send", offsetof(FwAddrs, mmi_send),
      { SIG_CODE, MMI_SEND_PAT, MMI_SEND_MSK, 8, 0, 0, 0, 0 } },
};
// Build-time guard: the MAD2_N_SIGS macro (mad2_sigs.h) must match the table length.
// (Portable C99 static assert via a negative array size on mismatch.)
typedef char mad2_n_sigs_check[(sizeof(MAD2_SIGS) / sizeof(MAD2_SIGS[0]) == MAD2_N_SIGS) ? 1 : -1];

// 5210 overlay (ModelProfile.sigs2): the two per-build gate writers above. Layered on top
// of MAD2_SIGS so the 5210 self-heals verdict + dsp_uploaded for any build in its line,
// without forcing those resolutions onto the rest of the family. See mad2_sigs.h.
const SigResolve MAD2_SIGS_5210[] = {
    { "verdict", offsetof(FwAddrs, verdict),
      { SIG_LITERAL, VERDICT5_PAT, VERDICT5_MSK, 8, /*lit_off*/2, /*addend*/0x69, 0, 0 } },
    { "dsp_uploaded", offsetof(FwAddrs, dsp_uploaded),
      { SIG_LITERAL, DSPUP5_PAT, DSPUP5_MSK, 12, /*lit_off*/4, /*addend*/0, 0, 0 } },
};
typedef char mad2_n_sigs_5210_check[(sizeof(MAD2_SIGS_5210) / sizeof(MAD2_SIGS_5210[0]) == MAD2_N_SIGS_5210) ? 1 : -1];

// 3310/NHM-5 overlay (ModelProfile.sigs2): self-heals verdict + dsp_uploaded for any build in
// the 3310 line, so neither is a per-version hardcode. On v5.79 both resolve to the existing
// profile constants (0x11FF15 / 0x11038C) → byte-identical boot; on v6.39 / v5.57 they resolve
// to that build's true cells. See mad2_sigs.h.
const SigResolve MAD2_SIGS_3310[] = {
    { "verdict", offsetof(FwAddrs, verdict),
      { SIG_LITERAL, VERDICT3_PAT, VERDICT3_MSK, 10, /*lit_off*/0, /*addend*/0, 0, 0 } },
    { "dsp_uploaded", offsetof(FwAddrs, dsp_uploaded),
      { SIG_LITERAL, DSPUP3_PAT, DSPUP3_MSK, 18, /*lit_off*/0, /*addend*/0, 0, 0 } },
};
typedef char mad2_n_sigs_3310_check[(sizeof(MAD2_SIGS_3310) / sizeof(MAD2_SIGS_3310[0]) == MAD2_N_SIGS_3310) ? 1 : -1];
