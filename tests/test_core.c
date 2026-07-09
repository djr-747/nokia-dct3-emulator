// DCT3 core — Phase 1 validation harness (native, gcc).
//
// Each test loads a hand-encoded ARM/Thumb program into the flat RAM, runs it,
// and checks register/memory results. Opcodes are hand-encoded; each is
// cross-checked by mGBA's own disassembler (printed via dump_arm) — the decoder
// and executor share emitter-arm.h, so a correct disassembly implies the
// executor sees the intended instruction.

#include <stdio.h>
#include <stdint.h>

#include "core/dct3_core.h"
#include <mgba/internal/arm/decoder.h>

static int g_pass, g_fail;

static void check_eq_(const char* desc, uint32_t got, uint32_t want) {
    if (got == want) { g_pass++; }
    else { g_fail++; printf("  FAIL %-24s got=0x%08X want=0x%08X\n", desc, got, want); }
}
#define CHECK_EQ(desc, got, want) check_eq_(desc, (uint32_t)(got), (uint32_t)(want))
#define CHECK(desc, cond) do { if (cond) { g_pass++; } else { g_fail++; printf("  FAIL %s\n", desc); } } while (0)

static void load_arm(DCT3Core* core, const uint32_t* prog, int n, uint32_t base) {
    for (int i = 0; i < n; ++i) dct3_write32(core, base + (uint32_t)i * 4, prog[i]);
}

// Disassemble a block so hand-encoded opcodes can be eyeballed against intent.
static void dump_arm(DCT3Core* core, const uint32_t* prog, int n, uint32_t base) {
    struct ARMInstructionInfo info;
    char buf[80];
    for (int i = 0; i < n; ++i) {
        ARMDecodeARM(prog[i], &info);
        // ARMDisassemble wants the executing PC (instruction + 8 in ARM state)
        // so branch targets resolve correctly.
        ARMDisassemble(&info, &core->cpu, NULL, base + (uint32_t)i * 4 + 8, buf, sizeof buf);
        printf("    %08X: %08X  %s\n", base + (uint32_t)i * 4, prog[i], buf);
    }
}

// --- ALU: mov / add / orr / sub / shifted move -------------------------------
static void test_alu(void) {
    printf("[alu] mov/add/orr/sub/lsl\n");
    static const uint32_t prog[] = {
        0xE3A00005, // mov  r0, #5
        0xE2801003, // add  r1, r0, #3
        0xE3A02010, // mov  r2, #0x10
        0xE3823001, // orr  r3, r2, #1
        0xE0414000, // sub  r4, r1, r0
        0xE1A05200, // mov  r5, r0, lsl #4
        0xEAFFFFFE, // b    .
    };
    DCT3Core* c = dct3_core_create();
    load_arm(c, prog, 7, 0);
    dct3_core_reset(c);
    dump_arm(c, prog, 7, 0);
    dct3_run(c, 6);
    CHECK_EQ("r0", dct3_reg(c, 0), 5);
    CHECK_EQ("r1", dct3_reg(c, 1), 8);
    CHECK_EQ("r2", dct3_reg(c, 2), 0x10);
    CHECK_EQ("r3", dct3_reg(c, 3), 0x11);
    CHECK_EQ("r4", dct3_reg(c, 4), 3);
    CHECK_EQ("r5", dct3_reg(c, 5), 0x50);
    dct3_core_destroy(c);
}

// --- Single data transfer: ldr / str / ldrb / strb ---------------------------
static void test_ldrstr(void) {
    printf("[mem] ldr/str/ldrb/strb\n");
    static const uint32_t prog[] = {
        0xE3A00C02, // mov   r0, #0x200
        0xE3A010AB, // mov   r1, #0xAB
        0xE5801000, // str   r1, [r0]
        0xE5902000, // ldr   r2, [r0]
        0xE3A03012, // mov   r3, #0x12
        0xE5C03004, // strb  r3, [r0, #4]
        0xE5D04004, // ldrb  r4, [r0, #4]
        0xE5905004, // ldr   r5, [r0, #4]
        0xEAFFFFFE, // b     .
    };
    DCT3Core* c = dct3_core_create();
    load_arm(c, prog, 9, 0);
    dct3_core_reset(c);
    dump_arm(c, prog, 9, 0);
    dct3_run(c, 8);
    CHECK_EQ("r2 (ldr)", dct3_reg(c, 2), 0xAB);
    CHECK_EQ("r4 (ldrb)", dct3_reg(c, 4), 0x12);
    // BE-32: a byte stored at address A is the most-significant byte of the
    // word at A, so reading the word back gives 0x12 in the top lane.
    CHECK_EQ("r5 (BE byte lane)", dct3_reg(c, 5), 0x12000000);
    CHECK_EQ("mem[0x200]", dct3_read32(c, 0x200), 0xAB);
    dct3_core_destroy(c);
}

// --- Block transfer: stmia / ldmia with writeback ----------------------------
static void test_ldmstm(void) {
    printf("[lsm] stmia/ldmia + writeback\n");
    static const uint32_t prog[] = {
        0xE3A00011, // mov   r0, #0x11
        0xE3A01022, // mov   r1, #0x22
        0xE3A02033, // mov   r2, #0x33
        0xE3A03044, // mov   r3, #0x44
        0xE3A0DC04, // mov   r13, #0x400
        0xE8AD000F, // stmia r13!, {r0-r3}
        0xE24D6010, // sub   r6, r13, #16
        0xE8B60780, // ldmia r6!, {r7-r10}
        0xEAFFFFFE, // b     .
    };
    DCT3Core* c = dct3_core_create();
    load_arm(c, prog, 9, 0);
    dct3_core_reset(c);
    dump_arm(c, prog, 9, 0);
    dct3_run(c, 8);
    CHECK_EQ("mem[0x400]", dct3_read32(c, 0x400), 0x11);
    CHECK_EQ("mem[0x404]", dct3_read32(c, 0x404), 0x22);
    CHECK_EQ("mem[0x408]", dct3_read32(c, 0x408), 0x33);
    CHECK_EQ("mem[0x40C]", dct3_read32(c, 0x40C), 0x44);
    CHECK_EQ("r13 writeback", dct3_reg(c, 13), 0x410);
    CHECK_EQ("r7", dct3_reg(c, 7), 0x11);
    CHECK_EQ("r8", dct3_reg(c, 8), 0x22);
    CHECK_EQ("r9", dct3_reg(c, 9), 0x33);
    CHECK_EQ("r10", dct3_reg(c, 10), 0x44);
    CHECK_EQ("r6 writeback", dct3_reg(c, 6), 0x410);
    dct3_core_destroy(c);
}

// --- Status registers + SP banking across mode switches ----------------------
static void test_psr(void) {
    printf("[psr] mrs/msr + SP banking\n");
    static const uint32_t prog[] = {
        0xE3A0DC05, // mov   r13, #0x500   (SYS sp)
        0xE10F0000, // mrs   r0, cpsr
        0xE321F0D2, // msr   cpsr_c, #0xD2  -> IRQ mode, I/F set
        0xE10F1000, // mrs   r1, cpsr
        0xE3A0DC06, // mov   r13, #0x600   (IRQ sp, banked)
        0xE321F0DF, // msr   cpsr_c, #0xDF  -> SYS mode
        0xE10F2000, // mrs   r2, cpsr
        0xE1A0300D, // mov   r3, r13       (should read restored SYS sp)
        0xEAFFFFFE, // b     .
    };
    DCT3Core* c = dct3_core_create();
    load_arm(c, prog, 9, 0);
    dct3_core_reset(c);
    dump_arm(c, prog, 9, 0);
    dct3_run(c, 8);
    CHECK_EQ("reset mode = SYSTEM", dct3_reg(c, 0) & 0x1F, 0x1F);
    CHECK_EQ("cpsr after msr 0xD2", dct3_reg(c, 1) & 0xFF, 0xD2);
    CHECK_EQ("cpsr after msr 0xDF", dct3_reg(c, 2) & 0xFF, 0xDF);
    CHECK_EQ("SYS sp restored", dct3_reg(c, 3), 0x500);
    CHECK_EQ("final mode SYSTEM", dct3_mode(c), 0x1F);
    dct3_core_destroy(c);
}

// --- IRQ entry (vector 0x18, banking, SPSR, I-bit) and exit (subs pc,lr) ------
static void test_irq(void) {
    printf("[irq] raise -> vector/bank/spsr; subs pc,lr,#4 returns\n");
    static const uint32_t prog[] = {
        /*0x00*/ 0xE321F012, // msr  cpsr_c, #0x12   IRQ mode, I clear
        /*0x04*/ 0xE3A0DC07, // mov  r13, #0x700      IRQ sp (banked)
        /*0x08*/ 0xE321F01F, // msr  cpsr_c, #0x1F    SYS mode, I clear
        /*0x0C*/ 0xE3A08000, // mov  r8, #0
        /*0x10*/ 0xEA000002, // b    0x20             jump over IRQ vector
        /*0x14*/ 0xE1A00000, // nop
        /*0x18*/ 0xE3A090BB, // mov  r9, #0xBB         <- BASE_IRQ handler
        /*0x1C*/ 0xE25EF004, // subs pc, lr, #4        return from IRQ
        /*0x20*/ 0xE2888001, // add  r8, r8, #1        <- main loop
        /*0x24*/ 0xEAFFFFFD, // b    0x20
    };
    DCT3Core* c = dct3_core_create();
    load_arm(c, prog, 10, 0);
    dct3_core_reset(c);
    dump_arm(c, prog, 10, 0);

    dct3_run(c, 5 + 6);  // 5 setup insns, then 3 loop iterations
    CHECK("r8 incrementing pre-IRQ", dct3_reg(c, 8) >= 1);
    CHECK_EQ("pre-IRQ mode SYSTEM", dct3_mode(c), 0x1F);
    uint32_t r8_before = dct3_reg(c, 8);
    uint32_t cpsr_before = dct3_cpsr(c);

    ARMRaiseIRQ(&c->cpu);
    CHECK_EQ("IRQ mode entered", dct3_mode(c), 0x12);
    CHECK("I bit set on entry", (dct3_cpsr(c) & 0x80) != 0);
    CHECK_EQ("spsr = prior cpsr", (uint32_t)c->cpu.spsr.packed, cpsr_before);
    CHECK("PC at IRQ vector", dct3_reg(c, 15) >= 0x18 && dct3_reg(c, 15) <= 0x20);

    dct3_step(c);  // mov r9, #0xBB
    CHECK_EQ("handler ran (r9)", dct3_reg(c, 9), 0xBB);
    dct3_step(c);  // subs pc, lr, #4 -> return + restore cpsr
    CHECK_EQ("returned to SYSTEM", dct3_mode(c), 0x1F);
    CHECK("I bit restored", (dct3_cpsr(c) & 0x80) == 0);

    dct3_run(c, 4);  // resume the loop
    CHECK("loop resumed (r8 advanced)", dct3_reg(c, 8) > r8_before);
    CHECK_EQ("r9 retained", dct3_reg(c, 9), 0xBB);
    dct3_core_destroy(c);
}

// --- Thumb: BX into Thumb state, then Thumb ALU -------------------------------
static void test_thumb(void) {
    printf("[thumb] bx into thumb; movs/adds/lsls\n");
    static const uint32_t arm_part[] = {
        0xE28F0001, // add r0, pc, #1   (pc=0x08 -> r0=0x09)
        0xE12FFF10, // bx  r0           (-> Thumb at 0x08)
    };
    static const uint16_t thumb_part[] = {
        0x2107, // movs r1, #7
        0x2202, // movs r2, #2
        0x188B, // adds r3, r1, r2
        0x008C, // lsls r4, r1, #2
        0xE7FE, // b .
    };
    DCT3Core* c = dct3_core_create();
    load_arm(c, arm_part, 2, 0);
    for (int i = 0; i < 5; ++i) dct3_write16(c, 0x08 + (uint32_t)i * 2, thumb_part[i]);
    dct3_core_reset(c);
    dump_arm(c, arm_part, 2, 0);
    dct3_run(c, 6);  // add, bx, movs, movs, adds, lsls
    CHECK("entered Thumb (T bit)", (dct3_cpsr(c) & 0x20) != 0);
    CHECK_EQ("r1", dct3_reg(c, 1), 7);
    CHECK_EQ("r2", dct3_reg(c, 2), 2);
    CHECK_EQ("r3", dct3_reg(c, 3), 9);
    CHECK_EQ("r4", dct3_reg(c, 4), 0x1C);
    dct3_core_destroy(c);
}

int main(void) {
    printf("DCT3 core — Phase 1 validation\n\n");
    test_alu();
    test_ldrstr();
    test_ldmstm();
    test_psr();
    test_irq();
    test_thumb();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
