// DCT3 boot-trace harness (Phase 2/3, native).
//
// Loads a raw 2 MB flash image at physical 0x200000, HLEs the MAD boot ROM by
// entering ARM at 0x200040, and runs the firmware behind the src/mad2 device
// model (GENSIO/CCONT/PCD8544). Every peripheral access is histogrammed; CCONT
// and LCD transactions are decoded. On exit the LCD framebuffer is rendered.
//
//   make trace                 # default flash image
//   ./build/dct3_boot_trace <flash.bin> [budget]
//
// Expects a RAW flash dump (e.g. the .fls). FIASCO (.390) unwrapping is separate.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "core/dct3_core.h"
#include "mad2/mad2.h"
#include "models/model.h"
#include "harness/harness.h"   // shared fault-detect + reset-recovery (Phase 8 H0)
#include "harness/seccode.h"   // SECCODE=1 validation knob (firmware-oracle code reader)
#include "gui.h"               // optional SDL overlay (DCT3_SDL); NO-OP stubs otherwise
#include "mbus_bridge.h"       // optional host-serial MBUS service bridge (MBUSPORT/MBUSPTY)
#include "trace_names.h"       // BROKERLOG: name broker trace events via dct3trac dict
#include <time.h>              // MBUS-bridge wall-clock pacing
#include <mgba/internal/arm/decoder.h>
#include <mgba/internal/arm/isa-inlines.h>   // ThumbWritePC/_ARMSetMode for the CALL primitive

// --- DCT3_RELEASE: beta/GUI build — compile out the debug/trace/inject/bridge knobs ---
// This TU (boot_trace.c) is the native frontend and carries ~130 env knobs used for
// forensics (RAMDUMP/CALLLOG/TRACE/WATCH/POKE/MBUS-bridge/key-inject/...). For the beta
// GUI shared with a tester, `make gui-release` defines DCT3_RELEASE, which neutralises
// every env knob here EXCEPT the short whitelist below (GUI mode, GUI power-off, harness
// recovery). The kept GUI FEATURE knobs (audio, screenshot, keypad-log, PWR-hold warm
// reset) live in tools/gui_sdl.c — a separate TU this macro does not touch, so they stay
// live. The headless dev build (no DCT3_RELEASE) is byte-identical: `make guard` unaffected.
#ifdef DCT3_RELEASE
static inline char* dct3_release_getenv(const char* n) {
    // Only these knobs are honoured in the release binary; every other getenv() → NULL,
    // so its debug block is inert. (getenv here is the real libc one — this function is
    // textually BEFORE the #define, so it is not itself remapped.)
    static const char* const allow[] = { "GUI", "GUIPWROFF", "RESET_RECOVER", 0 };
    for (int i = 0; allow[i]; i++) if (strcmp(n, allow[i]) == 0) return getenv(n);
    return (char*)0;
}
#define getenv(n) dct3_release_getenv(n)
#endif

#define FLASH_BASE     0x00200000u
#define BOOT_ENTRY     0x00200040u
#define IO_LO          0x00010000u   // DSP mailbox / MMIO / BusC
#define IO_HI          0x00100000u   // RAM proper starts here
#define HIST_SIZE      0x00100000u

#define DEFAULT_FW     "flash/My 3310 NR1 v5.79.fls"
#define DEFAULT_BUDGET 20000000L

typedef struct {
    Mad2 mad2;
    uint32_t* rd;       // read/write counts + last value per address
    uint32_t* wr;
    uint32_t* last;
    long verbose;       // remaining verbose I/O lines
} Harness;

static const char* sz(int size) { return size == 4 ? "w" : size == 2 ? "h" : "b"; }

// Humanize a large count into a readable K/M/B string (writes into caller's buf so it
// is reentrant — safe to use several times in one printf). e.g. 1722700000 -> "1.72 B".
static const char* hnum(double v, char* buf, size_t n) {
    if      (v >= 1e9) snprintf(buf, n, "%.2f B", v / 1e9);
    else if (v >= 1e6) snprintf(buf, n, "%.1f M", v / 1e6);
    else if (v >= 1e3) snprintf(buf, n, "%.1f K", v / 1e3);
    else               snprintf(buf, n, "%.0f", v);
    return buf;
}

// MBUS register access logger (env MBUSLOG): control 0x18, status 0x19, data
// 0x1A, FIQ pending 0x08, FIQ mask 0x0A. Capped so it shows the transmit window.
static int mbus_log_addr(uint32_t a) {
    return a == 0x20018 || a == 0x20019 || a == 0x2001A;  // control/status/data only (late, rare)
}
static long g_mbus_log_cap = 0;
// SREAD=<addr>: log PC + flash-return call-stack on the first reads of <addr>
// (used to find who renders a given flash string/resource).
static struct DCT3Core* g_core = NULL;
static uint32_t g_sread = 0;
static long g_sread_cap = 0;
static uint32_t g_sread_len = 8;   // SREADLEN: window size (default 8; widen to watch a region)
static uint32_t rd32(DCT3Core* c, uint32_t a);

static uint32_t on_read(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t ram_value) {
    Harness* h = (Harness*)ctx;
    uint32_t v = mad2_read(&h->mad2, pc, addr, size, ram_value);
    if (g_sread && g_core && addr >= g_sread && addr < g_sread + g_sread_len && g_sread_cap > 0) {
        printf("[sread] RD%s [%06X]=%0*X pc=%06X  stack:", sz(size), addr, size * 2, v, pc);
        uint32_t sp = (uint32_t)g_core->cpu.gprs[13];
        for (int k = 0; k < 16; ++k) {
            uint32_t w = rd32(g_core, sp + (uint32_t)k * 4);
            if (w >= 0x200000 && w < 0x500000) printf(" %06X", w & ~1u);  // 4 MB MCU range (3410)
        }
        printf("\n"); g_sread_cap--;
    }
    if (addr < HIST_SIZE) { h->rd[addr]++; h->last[addr] = v; }
    if (g_mbus_log_cap > 0 && mbus_log_addr(addr)) {
        printf("[mbus] RD [%05X]=%02X  pc=%06X\n", addr, v & 0xFF, pc); g_mbus_log_cap--;
    }
    if (h->verbose > 0) { printf("  %08X  RD%s [%08X] = %08X\n", pc, sz(size), addr, v); h->verbose--; }
    return v;
}
static void on_write(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t value) {
    Harness* h = (Harness*)ctx;
    { // WRWATCH=<periph addr>: log the writing PC for writes. WRWATCH_MAX raises the
      // default 24-write cap so the trace runs through a full crash window. For the
      // interrupt-control register the prior latched value is shown (prev->new) so a
      // partial write like 0x0A->0x40 (gate-clobber) is visible directly.
        static long wrw = -2, wrmax = 24; static long wrn = 0;
        if (wrw == -2) {
            wrw = getenv("WRWATCH") ? (long)strtoul(getenv("WRWATCH"), NULL, 0) : -1;
            if (getenv("WRWATCH_MAX")) wrmax = atol(getenv("WRWATCH_MAX"));
        }
        if (wrw >= 0 && addr == (uint32_t)wrw && wrn < wrmax) {
            printf("[wrw] WR [%05X] %02X->%02X  pc=%06X (#%ld)\n",
                   addr, h->mad2.int_ctrl, value & 0xFF, pc, wrn); wrn++;
        }
    }
    if (g_mbus_log_cap > 0 && mbus_log_addr(addr)) {
        printf("[mbus] WR [%05X]=%02X  pc=%06X\n", addr, value & 0xFF, pc); g_mbus_log_cap--;
    }
    // FLASHLOG=<n>: log the first n writes into the flash region (CFI commands +
    // program data) with the writing PC — for tracing the NVRAM/record-store commit
    // path (which settings reach the 0x3D0000 EEPROM partition, and when).
    { static long fl = -2; static int fln = 0;
      if (fl == -2) fl = getenv("FLASHLOG") ? atol(getenv("FLASHLOG")) : -1;
      if (fl > 0 && addr >= 0x200000 && addr < 0x400000 && fln < fl) {
          const char* k = h->mad2.flash_prog ? "PROGDATA" :
                          ((value&0xFF)==0x40||(value&0xFF)==0x10) ? "PROG-SET" :
                          ((value&0xFF)==0x20) ? "ERASE-SET" : ((value&0xFF)==0xD0) ? "CONFIRM" :
                          ((value&0xFF)==0x50) ? "CLR-STAT" : ((value&0xFF)==0x70) ? "RD-STAT" :
                          ((value&0xFF)==0x90) ? "RD-ID" : ((value&0xFF)==0xFF||(value&0xFF)==0xF0) ? "RD-ARR" : "?";
          printf("[flash] WR%s [%06X]=%04X %-9s pc=%06X (#%d)\n", sz(size), addr, value & 0xFFFF, k, pc, fln); fln++;
      } }
    { static int tlog = -1; if (tlog < 0) tlog = getenv("TLOG") ? 1 : 0;   // Timer0 dest writes (cached)
      if (tlog && (addr == 0x20012 || addr == 0x20013)) {
        static int n = 0;
        if (n++ < 30) printf("[tlog] %08X t0_dest<-%04X (cnt=%04X div=%02X) pc=%08X\n",
                             addr, (unsigned)value, h->mad2.t0_counter, h->mad2.t0_div & 0xFF, pc);
    } }
    mad2_write(&h->mad2, pc, addr, size, value);
    if (addr < HIST_SIZE) { h->wr[addr]++; h->last[addr] = value; }
    if (h->verbose > 0) { printf("  %08X  WR%s [%08X] = %08X\n", pc, sz(size), addr, value); h->verbose--; }
}

static uint32_t rd32(DCT3Core* c, uint32_t a) {
    const uint8_t* p = c->ram + (a & DCT3_RAM_MASK);
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void disasm(DCT3Core* c, uint32_t addr, int count, int thumb) {
    struct ARMInstructionInfo info;
    char buf[80];
    for (int i = 0; i < count; ++i) {
        if (thumb) {
            uint16_t op  = (uint16_t)((c->ram[addr & DCT3_RAM_MASK] << 8) | c->ram[(addr + 1) & DCT3_RAM_MASK]);
            uint16_t op2 = (uint16_t)((c->ram[(addr + 2) & DCT3_RAM_MASK] << 8) | c->ram[(addr + 3) & DCT3_RAM_MASK]);
            struct ARMInstructionInfo i1, i2, comb;
            ARMDecodeThumb(op, &i1);
            ARMDecodeThumb(op2, &i2);
            if (ARMDecodeThumbCombine(&i1, &i2, &comb)) {       // 32-bit Thumb (BL/BLX)
                ARMDisassemble(&comb, &c->cpu, NULL, addr + 4, buf, sizeof buf);
                printf("    %08X: %04X %04X %s\n", addr, op, op2, buf); addr += 4;
            } else {
                ARMDisassemble(&i1, &c->cpu, NULL, addr + 4, buf, sizeof buf);
                printf("    %08X: %04X      %s\n", addr, op, buf); addr += 2;
            }
        } else {
            uint32_t op = rd32(c, addr);
            ARMDecodeARM(op, &info);
            ARMDisassemble(&info, &c->cpu, NULL, addr + 8, buf, sizeof buf);
            printf("    %08X: %08X  %s\n", addr, op, buf); addr += 4;
        }
    }
}

static uint8_t* slurp(const char* path, long* nout) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return NULL; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t* b = malloc((size_t)n);
    if (!b || fread(b, 1, (size_t)n, f) != (size_t)n) { fprintf(stderr, "read failed\n"); fclose(f); free(b); return NULL; }
    fclose(f); *nout = n; return b;
}

// Load a Nokia firmware file into the core. Nokia .390/.39x/.pmm images are
// FIASCO-wrapped: a chain of blocks, each a 9-byte header
//   0x0b <addr:3 BE> <flag:1> <len:2 LE> <pad:1> <cksum:1>  followed by <len>
// data bytes; we place each block's data at its target address. (Block stride is
// 9 + len.) A leading byte other than 0x0b means a raw image, written verbatim at
// raw_base (e.g. a .fls). The 9-byte header was confirmed by matching the v6.39
// MCU boot code at 0x200040 against the raw v5.79 dump.
// Returns the number of flash bytes the image occupies from `raw_base` (raw .fls =
// file size; FIASCO = span to the top loaded address), so the caller can size-detect
// the model. 0 on load failure.
static long fw_load(DCT3Core* c, const char* path, uint32_t raw_base) {
    long sz; uint8_t* b = slurp(path, &sz);
    if (!b) return 0;
    long loaded;
    if (sz > 9 && b[0] == 0x0b) {
        long off = 0; int n = 0; uint32_t lo = 0xFFFFFFFFu, hi = 0;
        while (off + 9 <= sz && b[off] == 0x0b) {
            uint32_t addr = ((uint32_t)b[off+1] << 16) | ((uint32_t)b[off+2] << 8) | b[off+3];
            uint32_t len  = (uint32_t)b[off+5] | ((uint32_t)b[off+6] << 8);
            if (off + 9 + (long)len > sz) len = (uint32_t)(sz - off - 9);
            dct3_write_bytes(c, addr, b + off + 9, len);
            if (addr < lo) lo = addr;
            if (addr + len > hi) hi = addr + len;
            off += 9 + (long)len; n++;
        }
        printf("  FIASCO %s: %d blocks 0x%06X-0x%06X\n", path, n, lo, hi);
        loaded = (hi > raw_base) ? (long)(hi - raw_base) : sz;
    } else {
        dct3_write_bytes(c, raw_base, b, (size_t)sz);
        printf("  raw    %s: -> 0x%06X (%ld bytes)\n", path, raw_base, sz);
        loaded = sz;
    }
    free(b);
    return loaded;
}

// --- SWEEP=1 mode: DSP MDI group-code brute-force sweep ---
// Once boot has reached SWEEP_AT, fork one child per candidate 8-bit group code,
// inject a single MDIRCV message into the DSP->MCU queue, run the child forward
// SWEEP_AFTER more steps, and record outcomes (msg 0x47 to task 17 = "GSM init
// DONE", [0x10FE52] flag set, recog counter reset, reboot caught). The boot
// loop above is the validated one; this mode just runs after it reaches the
// chosen snapshot step.

// Inject one MDIRCV message at the queue's current tail and raise FIQ0.
// Mirrors the self-test reply path in src/mad2/dsp_default.c.
static void sweep_inject_mdircv(Harness* h, DCT3Core* c, uint8_t group,
                                const uint8_t* payload, uint8_t payload_len)
{
    uint32_t q  = h->mad2.fw.mdircv_q   & DCT3_RAM_MASK;
    uint32_t tp = h->mad2.fw.mdircv_tail & DCT3_RAM_MASK;
    uint16_t tail = (uint16_t)((c->ram[tp] << 8) | c->ram[tp + 1]);
    uint32_t q_off = (uint32_t)(tail - 0x80) * 2;   // each "word" slot is 2 bytes
    uint32_t qp = q + q_off;
    c->ram[qp]     = payload_len;
    c->ram[qp + 1] = group;
    for (int i = 0; i < payload_len && i < 32; ++i) c->ram[qp + 2 + i] = payload[i];
    uint16_t words = (uint16_t)(1 + (payload_len + 1) / 2);
    uint16_t nt = (uint16_t)(tail + words);
    c->ram[tp] = (uint8_t)(nt >> 8); c->ram[tp + 1] = (uint8_t)nt;
    h->mad2.fiq_pending |= 0x01;
}

typedef struct {
    int  msg47_to_task17;
    int  gsm_done_flag_set;
    int  counter_reset;
    int  reboot_caught;
    long step_msg47;
    long step_gsm_done;
    uint8_t counter_initial;
    uint8_t counter_final;
    uint8_t counter_min;
    uint32_t final_pc;
} SweepOutcome;

// Per-child instruction tracing (opt-in via SWEEP_TRACE_STEPS + SWEEP_TRACE_GROUPS).
// Returns 1 if this child's group is in the comma-list of groups to trace.
static int sweep_trace_group_match(uint8_t group, const char* list) {
    if (!list || !*list) return 0;
    const char* p = list;
    while (*p) {
        while (*p == ',' || *p == ' ') p++;
        if (!*p) break;
        char* end = NULL;
        long v = strtol(p, &end, 0);
        if (end == p) break;
        if ((uint8_t)v == group) return 1;
        p = end;
    }
    return 0;
}

static void sweep_child_run(DCT3Core* c, Harness* h, long max_steps, SweepOutcome* out,
                            uint8_t trace_group, const uint8_t* trace_payload,
                            int trace_payload_len, const char* trace_outdir)
{
    out->step_msg47 = -1;
    out->step_gsm_done = -1;
    out->counter_initial = c->ram[0x11FD08u & DCT3_RAM_MASK];
    out->counter_min     = out->counter_initial;
    fprintf(stderr, "[pid %d] child_run start: PC=0x%08X cycles=%lld max_steps=%ld\n",
            (int)getpid(), (uint32_t)c->cpu.gprs[15], (long long)c->cpu.cycles, max_steps);
    uint8_t gsm_done_initial = c->ram[0x10FE52u & DCT3_RAM_MASK];
    unsigned long long initial_reset_total = h->mad2.reset_total;

    // Per-step trace setup. No-op unless SWEEP_TRACE_STEPS>0 and group matches.
    long trace_steps = getenv("SWEEP_TRACE_STEPS") ? atol(getenv("SWEEP_TRACE_STEPS")) : 0;
    const char* trace_groups_env = getenv("SWEEP_TRACE_GROUPS");
    FILE* trace_fp = NULL;
    if (trace_steps > 0 && sweep_trace_group_match(trace_group, trace_groups_env)) {
        char tpath[512];
        snprintf(tpath, sizeof tpath, "%s/trace_%03x.txt",
                 trace_outdir ? trace_outdir : "/tmp/dsp_sweep", trace_group);
        trace_fp = fopen(tpath, "w");
        if (trace_fp) {
            setvbuf(trace_fp, NULL, _IOLBF, 0);
            fprintf(trace_fp, "# SWEEP_AT=%s group=0x%02X payload=",
                    getenv("SWEEP_AT") ? getenv("SWEEP_AT") : "0", trace_group);
            for (int i = 0; i < trace_payload_len; ++i) {
                fprintf(trace_fp, "%s%02X", i ? "," : "", trace_payload[i]);
            }
            fprintf(trace_fp, " trace_steps=%ld\n", trace_steps);
        }
    }

    for (long s = 0; s < max_steps; ++s) {
        struct ARMCore* cpu = &c->cpu;
        uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);

        if (trace_fp && s < trace_steps) {
            fprintf(trace_fp,
                    "%6ld pc=0x%08x r0=%08x r1=%08x r2=%08x r3=%08x lr=%08x cpsr=%08x\n",
                    s, pc,
                    (uint32_t)cpu->gprs[0], (uint32_t)cpu->gprs[1],
                    (uint32_t)cpu->gprs[2], (uint32_t)cpu->gprs[3],
                    (uint32_t)cpu->gprs[ARM_LR], (uint32_t)cpu->cpsr.packed);
        }

        // SEND_TASK_MESSAGE (0x29921C): r0 = task id, r1 = msg id.
        if (pc == 0x29921Cu) {
            uint32_t task = (uint32_t)cpu->gprs[0] & 0xFFu;
            uint32_t msg  = (uint32_t)cpu->gprs[1] & 0xFFFFu;
            if (task == 17 && msg == 0x47 && !out->msg47_to_task17) {
                out->msg47_to_task17 = 1;
                out->step_msg47 = s;
            }
        }

        // Pre-step reset-recovery hooks (mirror of the main loop's inline hooks).
        if (h->mad2.fw.reboot_fn && pc == (h->mad2.fw.reboot_fn & ~1u)) {
            h->mad2.reboot_entry_lr     = (uint32_t)cpu->gprs[ARM_LR];
            h->mad2.reboot_entry_reason = (uint32_t)cpu->gprs[0];
            h->mad2.reboot_entry_cpsr   = cpu->cpsr.packed;
            h->mad2.reboot_entry_sp     = (uint32_t)cpu->gprs[ARM_SP];
            h->mad2.reboot_entry_seen   = 1;
        }
        if (h->mad2.fw.fatal_handler && pc == (h->mad2.fw.fatal_handler & ~1u)
            && !cpu->cpsr.t) {
            mad2_record_fatal(&h->mad2, (uint8_t)cpu->cpsr.priv,
                              (uint32_t)cpu->gprs[ARM_LR], cpu->spsr.packed);
        }
        if (h->mad2.fw.assert_log && pc == (h->mad2.fw.assert_log & ~1u)) {
            mad2_record_assertion(&h->mad2,
                                  (uint16_t)(cpu->gprs[0] & 0xFFFFu),
                                  (uint16_t)(cpu->gprs[1] & 0xFFFFu),
                                  (uint32_t)cpu->gprs[ARM_LR]);
        }
        if (h->mad2.fw.reason_setter && pc == (h->mad2.fw.reason_setter & ~1u)) {
            mad2_record_stage(&h->mad2,
                              (uint8_t)(cpu->gprs[0] & 0xFFu),
                              (uint32_t)cpu->gprs[ARM_LR]);
        }
        if (h->mad2.fw.malloc_fail && pc == (h->mad2.fw.malloc_fail & ~1u))
            mad2_record_heap_fail(&h->mad2, (uint32_t)cpu->gprs[ARM_LR]);

        dct3_step(c);
        mad2_timers_tick(&h->mad2, (uint32_t)cpu->cycles);
        if (h->mad2.recover_pending) {
            dct3_core_force_pc_cpsr(c, h->mad2.recover_pc, h->mad2.recover_cpsr,
                                    h->mad2.recover_sp);
            h->mad2.recover_pending = 0; h->mad2.recover_sp = 0;
        }
        // Deliver pending FIQ/IRQ so the injected MDIRCV's FIQ0 actually fires.
        if (!cpu->cpsr.f) {
            int fiq = mad2_fiq_poll(&h->mad2);
            if (fiq >= 0) { ARMRaiseFIQ(cpu); h->mad2.fiqs_raised++; }
        }
        if (!cpu->cpsr.i) {
            int irq = mad2_irq_poll(&h->mad2);
            if (irq >= 0) { ARMRaiseIRQ(cpu); h->mad2.irqs_raised++; }
        }

        uint8_t ctr = c->ram[0x11FD08u & DCT3_RAM_MASK];
        if (ctr < out->counter_min) out->counter_min = ctr;

        uint8_t g = c->ram[0x10FE52u & DCT3_RAM_MASK];
        if (g == 1 && gsm_done_initial != 1 && !out->gsm_done_flag_set) {
            out->gsm_done_flag_set = 1;
            out->step_gsm_done = s;
        }

        if (h->mad2.reset_total > initial_reset_total && !out->reboot_caught) {
            out->reboot_caught = 1;
        }
    }
    out->counter_final = c->ram[0x11FD08u & DCT3_RAM_MASK];
    if (out->counter_final < out->counter_initial || out->counter_min < out->counter_initial)
        out->counter_reset = 1;
    out->final_pc = (uint32_t)c->cpu.gprs[15];
    fprintf(stderr, "[pid %d] child_run end:   PC=0x%08X cycles=%lld\n",
            (int)getpid(), (uint32_t)c->cpu.gprs[15], (long long)c->cpu.cycles);
    if (trace_fp) fclose(trace_fp);
}

static int sweep_parse_payload(const char* spec, uint8_t* out, int max) {
    if (!spec || !*spec) { out[0] = 0; out[1] = 0; return 2; }
    int n = 0;
    const char* p = spec;
    while (*p && n < max) {
        while (*p == ',' || *p == ' ') p++;
        if (!*p) break;
        out[n++] = (uint8_t)strtoul(p, (char**)&p, 0);
    }
    return n;
}

static void do_sweep(DCT3Core* c, Harness* h, int group_lo, int group_hi,
                     int parallel, long sweep_after,
                     const char* payload_spec, const char* outdir)
{
    uint8_t payload[32];
    int payload_len = sweep_parse_payload(payload_spec, payload, sizeof payload);
    if (payload_len > 32) payload_len = 32;
    mkdir(outdir, 0755);

    uint8_t ctr0 = c->ram[0x11FD08u & DCT3_RAM_MASK];
    uint8_t g0   = c->ram[0x10FE52u & DCT3_RAM_MASK];
    uint32_t mq  = h->mad2.fw.mdircv_q   & DCT3_RAM_MASK;
    uint32_t mtp = h->mad2.fw.mdircv_tail & DCT3_RAM_MASK;
    uint32_t mhp = h->mad2.fw.mdircv_head & DCT3_RAM_MASK;
    uint16_t mt = (uint16_t)((c->ram[mtp] << 8) | c->ram[mtp + 1]);
    uint16_t mh = (uint16_t)((c->ram[mhp] << 8) | c->ram[mhp + 1]);
    fprintf(stderr, "SWEEP: snapshot reached  PC=0x%08X  ctr[0x11FD08]=%u  gsm_done[0x10FE52]=%u\n"
                    "       mdircv_q=0x%05X  head=0x%04X  tail=0x%04X  (empty=%s)\n"
                    "       groups=%d..%d payload_len=%d parallel=%d outdir=%s sweep_after=%ld\n",
            (uint32_t)c->cpu.gprs[15], ctr0, g0, mq, mh, mt, (mh == mt) ? "yes" : "NO",
            group_lo, group_hi, payload_len, parallel, outdir, sweep_after);
    if (mh != mt)
        fprintf(stderr, "SWEEP: WARNING — MDIRCV queue not empty at snapshot. Injection may overflow.\n");

    int active = 0;
    for (int g = group_lo; g <= group_hi; ++g) {
        while (active >= parallel) {
            int st; pid_t pid = wait(&st);
            if (pid > 0) active--;
        }
        pid_t pid = fork();
        if (pid < 0) { perror("fork"); break; }
        if (pid == 0) {
            sweep_inject_mdircv(h, c, (uint8_t)g, payload, (uint8_t)payload_len);
            SweepOutcome out = {0};
            sweep_child_run(c, h, sweep_after, &out,
                            (uint8_t)g, payload, payload_len, outdir);
            char outpath[512];
            snprintf(outpath, sizeof outpath, "%s/%03d.txt", outdir, g);
            FILE* f = fopen(outpath, "w");
            if (f) {
                fprintf(f, "group=0x%02X msg0x47=%d gsm_done=%d counter_reset=%d reboot=%d "
                           "step_msg47=%ld step_gsm_done=%ld "
                           "counter=%u->%u(min=%u) final_pc=0x%08X\n",
                        g, out.msg47_to_task17, out.gsm_done_flag_set,
                        out.counter_reset, out.reboot_caught,
                        out.step_msg47, out.step_gsm_done,
                        out.counter_initial, out.counter_final, out.counter_min,
                        out.final_pc);
                fclose(f);
            }
            _exit(0);
        }
        active++;
    }
    while (active > 0) {
        int st; pid_t pid = wait(&st);
        if (pid > 0) active--;
    }

    fprintf(stderr, "\nSWEEP: complete. Interesting outcomes:\n");
    int interesting = 0;
    for (int g = group_lo; g <= group_hi; ++g) {
        char p[512]; snprintf(p, sizeof p, "%s/%03d.txt", outdir, g);
        FILE* f = fopen(p, "r"); if (!f) continue;
        char line[512];
        if (fgets(line, sizeof line, f)) {
            if (strstr(line, "msg0x47=1") || strstr(line, "gsm_done=1") ||
                strstr(line, "counter_reset=1") || strstr(line, "reboot=1")) {
                fputs(line, stderr);
                interesting++;
            }
        }
        fclose(f);
    }
    if (!interesting) fprintf(stderr, "  (none — every group code left counter and GSM-done flag unchanged.)\n");
    fprintf(stderr, "Full per-group results in %s/\n", outdir);
}

// CFTRACE: short CPU-mode tag for the control-flow / interrupt trace.
static const char* cf_mode_str(uint32_t m) {
    switch (m) {
        case 0x10: return "usr"; case 0x11: return "fiq"; case 0x12: return "irq";
        case 0x13: return "svc"; case 0x17: return "abt"; case 0x1B: return "und";
        case 0x1F: return "sys"; default: return "???";
    }
}

// RAMDUMP: snapshot the full 16 MB RAM (flash + RAM + I/O, indexed by masked address) so
// disfw --ram can resolve RAM-resident table contents + live values. Writes <path> plus a
// <path>.regs sidecar (r0..r15 + cpsr) for an optional concrete-replay seed.
static void ramdump_write(DCT3Core* c, struct ARMCore* cpu, long steps, uint32_t pc,
                          const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "[ramdump] cannot open %s\n", path); return; }
    size_t w = fwrite(c->ram, 1, DCT3_RAM_SIZE, f);
    fclose(f);
    char rp[1100];
    snprintf(rp, sizeof rp, "%s.regs", path);
    FILE* rf = fopen(rp, "w");
    if (rf) {
        fprintf(rf, "# ramdump regs @ step %ld pc=0x%06X\n", steps, pc & 0xFFFFFFu);
        for (int i = 0; i < 16; i++)
            fprintf(rf, "r%d=0x%08X\n", i, (uint32_t)cpu->gprs[i]);
        fprintf(rf, "cpsr=0x%08X\n", (uint32_t)cpu->cpsr.packed);
        fclose(rf);
    }
    fprintf(stderr, "[ramdump] wrote %s (%zu bytes) + .regs @ step %ld pc=0x%06X\n",
            path, w, steps, pc & 0xFFFFFFu);
}

// MEMLOG: a rolling ring of the last N memory accesses (writes always; reads with
// MEMLOGRD=1), flushed with the RAM dump as the "what happened just before the halt"
// window — each write as `old -> new @pc`, each read as `value @pc`.
typedef struct { uint32_t pc, addr, oldv, newv; uint8_t size, wr; } MemRec;
static MemRec* g_memring = NULL;
static long g_memring_n = 0, g_memring_head = 0, g_memring_count = 0;
static void memlog_cb(void* ctx, uint32_t pc, uint32_t addr, int size,
                      uint32_t oldv, uint32_t newv, int is_write) {
    (void)ctx;
    if (!g_memring) return;
    MemRec* r = &g_memring[g_memring_head];
    r->pc = pc; r->addr = addr; r->oldv = oldv; r->newv = newv;
    r->size = (uint8_t)size; r->wr = (uint8_t)is_write;
    g_memring_head = (g_memring_head + 1) % g_memring_n;
    if (g_memring_count < g_memring_n) g_memring_count++;
}
static void memlog_flush(const char* dumppath) {
    if (!g_memring || g_memring_count == 0) return;
    char p[1100];
    snprintf(p, sizeof p, "%s.memlog", dumppath);
    FILE* f = fopen(p, "w");
    if (!f) return;
    fprintf(f, "# last %ld memory accesses before dump (oldest -> newest)\n", g_memring_count);
    long start = (g_memring_head - g_memring_count + g_memring_n) % g_memring_n;
    for (long i = 0; i < g_memring_count; i++) {
        MemRec* r = &g_memring[(start + i) % g_memring_n];
        int w = r->size * 2;
        if (r->wr)
            fprintf(f, "WR%d [%06X] 0x%0*X -> 0x%0*X  pc=%06X\n",
                    r->size, r->addr, w, r->oldv, w, r->newv, r->pc & 0xFFFFFFu);
        else
            fprintf(f, "RD%d [%06X] = 0x%0*X  pc=%06X\n",
                    r->size, r->addr, w, r->newv, r->pc & 0xFFFFFFu);
    }
    fclose(f);
    fprintf(stderr, "[memlog] wrote %s (%ld records)\n", p, g_memring_count);
}

// CALLLOG: rolling ring of taken BL/BLX/BX (calls + indirect jumps, not local `b`)
// with target, LR, and CPU mode — the control-flow companion to MEMLOG, flushed with
// the dump so the exact call path into the halt is visible alongside the data trail.
typedef struct { uint32_t pc, target, lr; uint8_t kind, mode; } CallRec;
static CallRec* g_callring = NULL;
static long g_callring_n = 0, g_callring_head = 0, g_callring_count = 0;
static void callring_push(uint32_t pc, uint32_t target, uint32_t lr, uint8_t kind, uint8_t mode) {
    if (!g_callring) return;
    CallRec* r = &g_callring[g_callring_head];
    r->pc = pc; r->target = target; r->lr = lr; r->kind = kind; r->mode = mode;
    g_callring_head = (g_callring_head + 1) % g_callring_n;
    if (g_callring_count < g_callring_n) g_callring_count++;
}
static void callring_flush(const char* dumppath) {
    if (!g_callring || g_callring_count == 0) return;
    char p[1100];
    snprintf(p, sizeof p, "%s.calllog", dumppath);
    FILE* f = fopen(p, "w");
    if (!f) return;
    static const char* MK[] = { "", "BL ", "BLX", "BX " };
    fprintf(f, "# last %ld calls/jumps before dump (oldest -> newest)\n", g_callring_count);
    long start = (g_callring_head - g_callring_count + g_callring_n) % g_callring_n;
    for (long i = 0; i < g_callring_count; i++) {
        CallRec* r = &g_callring[(start + i) % g_callring_n];
        fprintf(f, "%s %06X -> %06X  lr=%06X  m=%s\n",
                MK[r->kind], r->pc & 0xFFFFFFu, r->target & 0xFFFFFFu,
                r->lr & 0xFFFFFFu, cf_mode_str(r->mode));
    }
    fclose(f);
    fprintf(stderr, "[calllog] wrote %s (%ld records)\n", p, g_callring_count);
}

// --- PCMSINK=1: HAL PCM channel demo binding ---------------
// Counts samples per channel + tracks ch1 peak; summary at exit. The point is a
// zero-dependency consumer of mad2.pcm_sink so the channel has a native proof.
static long g_pcmsink_n[2];
static int  g_pcmsink_pk;
static void pcmsink_count(Mad2* m, int ch, int16_t sample) {
    (void)m;
    if (ch == 0 || ch == 1) g_pcmsink_n[ch]++;
    if (ch == 1) { int a = sample < 0 ? -sample : sample; if (a > g_pcmsink_pk) g_pcmsink_pk = a; }
}


int main(int argc, char** argv) {
    // Probe mode:  dis <hexaddr> <count>  -- disassemble Thumb in flash and exit.
    if (argc >= 4 && strcmp(argv[1], "dis") == 0) {
        long n; uint8_t* img = slurp(DEFAULT_FW, &n);
        if (!img) return 1;
        DCT3Core* dc = dct3_core_create();
        dct3_write_bytes(dc, FLASH_BASE, img, (size_t)n);
        int thumb_mode = !(argc >= 5 && strcmp(argv[4], "arm") == 0);
        disasm(dc, (uint32_t)strtoul(argv[2], NULL, 0), atoi(argv[3]), thumb_mode);
        return 0;
    }
    const char* path = argc > 1 ? argv[1] : DEFAULT_FW;
    long budget = argc > 2 ? atol(argv[2]) : DEFAULT_BUDGET;
#ifdef DCT3_SDL
    // Optional SDL overlay: GUI=1 env or a --gui arg. When on, run effectively
    // unbounded (until window-close / fault); headless behaviour is unchanged.
    int gui_on = getenv("GUI") ? atoi(getenv("GUI")) : 0;
    for (int ai = 1; ai < argc; ++ai) if (strcmp(argv[ai], "--gui") == 0) gui_on = 1;
    if (gui_on) budget = 0x7FFFFFFFFFFFFFFFLL;   // run until close/fault
#endif

    printf("DCT3 boot trace\n  entry  : 0x%08X (ARM, BE)\n  budget : %ld insns\n  images :\n",
           BOOT_ENTRY, budget);

    DCT3Core* c = dct3_core_create();
    // Firmware images (FIASCO-unwrapped or raw). MCU (raw .fls or FIASCO .390),
    // then an optional separate PPM language pack, then an optional EEPROM/PMM.
    // The blank EEPROM partition in a raw .fls fails the NVRAM self-test, so a
    // virgin EEPROM is usually needed:  EEPROM=<.pmm>  PPM=<.39x>
    long mcu_sz = fw_load(c, path, FLASH_BASE);

    // Select the model profile from the loaded MCU image: MODEL=<name> override,
    // else autodetect by product code + real image size, else the 3310 default.
    const ModelProfile* prof = getenv("MODEL") ? model_by_name(getenv("MODEL")) : NULL;
    if (!prof) prof = model_detect(c->ram + FLASH_BASE, (uint32_t)mcu_sz);
    if (!prof) prof = model_default();
    printf("  model  : %s — %s\n", prof->name, prof->description);

    if (getenv("PPM"))    fw_load(c, getenv("PPM"),    0x340000);
    if (getenv("EEPROM")) fw_load(c, getenv("EEPROM"), prof->mem.eeprom_base);
    // Repair the MCU flash-region checksum so a file-patched image (e.g. NokiX
    // kill_faid_check, or any FAID/feature poke) passes the boot integrity check
    // instead of warm-rebooting (reset reason 4). No-op on an unpatched image, so
    // stock boots stay byte-identical. FLASHCKSUM_RAW=1 opts out (A/B the fault).
    if (!getenv("FLASHCKSUM_RAW") && dct3_fix_mcu_checksum(c))
        printf("  cksum  : MCU flash-region checksum repaired (patched image @0x200022)\n");
    printf("\n");

    Harness h = {0};
    mad2_init(&h.mad2, prof);
    if (getenv("CCONT0E")) h.mad2.ccont[0x0E] = (uint8_t)strtoul(getenv("CCONT0E"), NULL, 0);
    // CCONT A/D overrides (diagnostic): VBATT=adc2 BSI=adc3(battery type) BTEMP=adc4 CHARGER=adc5.
    // The 8850 uses a Li-ion BLB-2 pack — a different BSI than the 3310 NiMH default.
    if (getenv("VBATT"))   h.mad2.adc[2] = (uint16_t)strtoul(getenv("VBATT"), NULL, 0);
    if (getenv("BSI"))     h.mad2.adc[3] = (uint16_t)strtoul(getenv("BSI"), NULL, 0);
    if (getenv("BTEMP"))   h.mad2.adc[4] = (uint16_t)strtoul(getenv("BTEMP"), NULL, 0);
    if (getenv("CHARGER")) h.mad2.adc[5] = (uint16_t)strtoul(getenv("CHARGER"), NULL, 0);
    if (getenv("KEYSPECIAL")) h.mad2.kbd_special_cols = (uint8_t)strtoul(getenv("KEYSPECIAL"), NULL, 0);
    if (getenv("KEYROW") && getenv("KEYCOL"))
        h.mad2.kbd_norm_cols[strtoul(getenv("KEYROW"),NULL,0) & 7] = 1u << (strtoul(getenv("KEYCOL"),NULL,0) & 7);
    // Quieting: the end-of-run peripheral histogram (~2k lines, the bulk of the noise) is now
    // OPT-IN via IOHIST=1 / VERBOSE=1. The small startup traces (CCVERBOSE decoded CCONT/LCD,
    // MEMTRACE first-N mem accesses) keep their original defaults — they must NOT be suppressed
    // here: the 3410 boot has a wall-clock-timing dependence (a real-clock RNG seed) and
    // removing those printf's flips its outcome (see docs / the reason-0x73 reset). Fixing that
    // determinism bug is the prerequisite to quieting them; tracked separately.
    int verbose_all = getenv("VERBOSE") ? 1 : 0;
    h.mad2.verbose = getenv("CCVERBOSE") ? (int)atol(getenv("CCVERBOSE")) : 60;
    h.rd = calloc(HIST_SIZE, sizeof(uint32_t));
    h.wr = calloc(HIST_SIZE, sizeof(uint32_t));
    h.last = calloc(HIST_SIZE, sizeof(uint32_t));
    h.verbose = getenv("MEMTRACE") ? (int)atol(getenv("MEMTRACE")) : 120;
    h.mad2.mem = c->ram;             // flash device programs into the core's backing
    h.mad2.mem_mask = DCT3_RAM_MASK;
    // Resolve firmware-build addresses (signatures over the loaded image, profile
    // fallbacks otherwise) into h.mad2.fw.
    model_resolve(prof, c->ram, DCT3_RAM_MASK, &h.mad2.fw);
    dct3_set_io_hooks(c, &h, IO_LO, IO_HI, on_read, on_write);
    dct3_set_io_range2(c, prof->mem.flash_base,
                       prof->mem.flash_base + prof->mem.flash_size);  // flash chip (device-owned)

    // DSPTRAMP: install a Thumb trampoline at 0x1F0000 (unused RAM) that posts a
    // DSP-response message (sub-type 6 at [buf+4]) to task id19, so the DSP responder
    // can drive id19's response loop to set barrier flag 2 (0x10B20C). It mallocs an
    // 8-byte msg via 0x299AF8, sets [+4]=6, and posts via 0x299134(r0=19, r1=buf).
    // Call it from CALLFNLIST=...,0x1F0001,... (Thumb).
    if (getenv("DSPTRAMP")) {
        // ARMv4T Thumb (NO blx — that is ARMv5). Direct BL to malloc/post, encoded for
        // this fixed load address 0x1F0000. Posts a DSP-response message to id19 with
        // sub-type = r0 (passed by the CALL), so a script can send the exact sub-type
        // sequence (sub-type 1 enters id19's response loop; 4x sub-type 6 -> flag 2):
        //   push{r4,lr}; r4=r0(sub-type); r0=8; bl 0x299AF8(malloc);
        //   [r0+4]=r4; r1=r0(buf); r0=19; bl 0x299134(post); pop{r4,pc}
        static const uint8_t tramp[] = {
            0xB5,0x10, 0x1C,0x04, 0x20,0x08, 0xF0,0xA9, 0xFD,0x77,  // push{r4,lr};r4=r0;r0=8;bl malloc
            0x71,0x04, 0x1C,0x01, 0x20,0x13, 0xF0,0xA9, 0xF8,0x90,  // [r0+4]=r4;r1=r0;r0=19;bl post
            0xBD,0x10,                                              // pop{r4,pc}
        };
        dct3_write_bytes(c, 0x001F0000u, tramp, sizeof tramp);
        printf("DSPTRAMP installed at 0x001F0000 (call via 0x001F0001)\n");
    }

    // T14INJECT: diagnostic trampoline at 0x1F0000 that posts a crafted message to the
    // SWDSP task-14 (mailbox task id 14) via FUN_00299134, to test whether feeding the
    // SWDSP state machine its entry message cancels the 0xE4 watchdog. CALLR0 = the 16-bit
    // msg id (e.g. 0xFCD = state-0 entry, 0x1004 = state-6/col-1 = the watchdog cancel).
    //   push{r4,lr}; r4=r0(id); r0=0x40; bl malloc(0x299AF8); strh r4,[r0,#0]; r1=r0;
    //   r0=14; bl post(0x299134); pop{r4,pc}
    // Fire via CALL=0x001F0001 CALLR0=0x1004. Real heap buffer => task-14's HEAP_FREE is safe.
    if (getenv("T14INJECT")) {
        uint8_t t14[] = {
            0xB5,0x10, 0x1C,0x04, 0x20,0x40, 0xF0,0xA9, 0xFD,0x77,  // push;r4=r0;r0=0x40;bl malloc
            0x80,0x04, 0x1C,0x01, 0x20,0x0E, 0xF0,0xA9, 0xF8,0x90,  // buf[0]=r4;r1=buf;r0=<task>;bl post
            0xBD,0x10,                                              // pop{r4,pc}
        };
        int injtask = getenv("INJTASK") ? atoi(getenv("INJTASK")) : 14;  // control: INJTASK=19
        t14[15] = (uint8_t)injtask;  // patch the `mov r0,#<task>` immediate
        dct3_write_bytes(c, 0x001F0000u, t14, sizeof t14);
        printf("T14INJECT trampoline at 0x001F0000 -> task %d (CALL=0x001F0001 CALLR0=<msgid>)\n", injtask);
    }

    printf("=== boot entry ===\n");
    disasm(c, BOOT_ENTRY, 8, 0);
    printf("\n=== execution ===\n");
    dct3_boot_at(c, BOOT_ENTRY);

    long steps = 0;
    // Spin/wild-PC detection moved into the shared harness (src/harness/fault.c). The
    // spin-window state now lives in HarnessConfig hc (initialised below); SPIN_LIMIT
    // is still read here and handed to harness_init.
    long SPIN_LIMIT = getenv("SPINLIMIT") ? atol(getenv("SPINLIMIT")) : 16000000;  // ~3.3s emu (was 2M ~0.4s, too eager on bounded compute loops)
    long dsp_fiq_period = getenv("DSPFIQ") ? atol(getenv("DSPFIQ")) : 0;
    int  dsp_fiq_bit    = getenv("DSPBIT") ? (int)atol(getenv("DSPBIT")) : 0;
    // IRQINJECT=mask[,period][,after] (experimental): periodically OR `mask` into
    // irq_pending from step `after`. Used to discover which IRQ source(s) the 8850
    // waits on (it unmasks IRQ0/5/6 but our model only asserts 2/4). e.g.
    // IRQINJECT=0x01,30000 = pulse IRQ0 every 30k steps.
    unsigned irqinj_mask = 0; long irqinj_period = 0, irqinj_after = 0;
    if (getenv("IRQINJECT")) {
        const char* p = getenv("IRQINJECT");
        irqinj_mask = (unsigned)strtoul(p, (char**)&p, 0);
        if (*p == ',') { p++; irqinj_period = strtol(p, (char**)&p, 0); }
        if (*p == ',') { p++; irqinj_after  = strtol(p, (char**)&p, 0); }
        if (irqinj_period <= 0) irqinj_period = 30000;
        printf("IRQINJECT: mask=0x%02X every %ld steps from step %ld\n",
               irqinj_mask, irqinj_period, irqinj_after);
    }
    const char* reason = "budget reached";
    uint32_t watch = (uint32_t)strtoul(getenv("WATCH") ? getenv("WATCH") : "0", NULL, 0);
    int watch_hits = 0;
    // BROKERLOG=1: name every trace event the firmware publishes. Taps BROKER_ROUTER
    // (0x2EACAA) and direct BROKER_DELIVER (0x2EABE8) callers; r1 = 16-bit event-id,
    // decoded via the dct3trac NHM-5 dictionary. BROKERLOG=1 is pure observation (reads
    // regs at fn entry, before the [0x11FF28] enable gate). BROKERLOG=2 also forces the
    // broker enable flag each step so MDI/LOC sites that self-gate on it (e.g. the
    // MDI:m2d/d2m local handshake) publish — equivalent to POKE=0x11FF28=1 but built in.
    // BROKERLOG_MAX caps lines (def 400).
    int brokerlog = getenv("BROKERLOG") ? atoi(getenv("BROKERLOG")) : 0;
    long brokerlog_cap = getenv("BROKERLOG_MAX") ? atol(getenv("BROKERLOG_MAX")) : 400;
    long brokerlog_n = 0;
    if (brokerlog) trace_names_load();
    // WDOGLOG=1: TASK_4 DSP-watchdog correlation. Taps the 0xE4-watchdog decision (0x2EDC52)
    // which reads the MDI-activity counter [0x111862] (bumped by every non-0xE4 MDI msg
    // received); counter==0 at the tick -> SWDSP_STAGER(1) = reason 0x68. Prints each tick's
    // counter, the inter-tick step gap, and flags the 0x68 trip. Needs no POKE/broker — the
    // watchdog runs in core TASK_4. Run alongside BROKERLOG=2 to see which d2m fed the counter.
    int wdoglog = getenv("WDOGLOG") ? atoi(getenv("WDOGLOG")) : 0;
    long wdog_last = 0;
    // WATCHTAIL=<n>: instead of logging the first 12 hits of WATCH, keep a ring of
    // the LAST n hits (regs + step) and dump them on exit — for seeing what state a
    // watched PC is in right when the boot settles (e.g. a sweep stall point).
    long wt_n = getenv("WATCHTAIL") ? atol(getenv("WATCHTAIL")) : 0;
    struct { uint32_t lr,r0,r1,r2,r5,r6; long step; } *wt_ring = NULL;
    long wt_w = 0;
    if (wt_n > 0) wt_ring = calloc((size_t)wt_n, sizeof(*wt_ring));
    g_mbus_log_cap = getenv("MBUSLOG") ? atol(getenv("MBUSLOG")) : 0;
    g_core = c;
    g_sread = (uint32_t)strtoul(getenv("SREAD") ? getenv("SREAD") : "0", NULL, 0);
    g_sread_cap = getenv("SREADN") ? atol(getenv("SREADN")) : 6;
    g_sread_len = getenv("SREADLEN") ? (uint32_t)strtoul(getenv("SREADLEN"), NULL, 0) : 8;
    // POKE=addr=val,addr=val,... : force RAM bytes every step (diagnostic HLE of a
    // readiness/self-test flag the firmware never sets on its own).
    uint32_t poke_addr[64]; uint8_t poke_val[64]; int poke_n = 0;
    if (getenv("POKE")) {
        const char* p = getenv("POKE");
        while (*p && poke_n < 64) {
            uint32_t a = (uint32_t)strtoul(p, (char**)&p, 0);
            if (*p == '=') p++;
            uint32_t v = (uint32_t)strtoul(p, (char**)&p, 0);
            poke_addr[poke_n] = a; poke_val[poke_n] = (uint8_t)v; poke_n++;
            if (*p == ',') p++;
        }
        printf("POKE: %d byte(s) forced each step\n", poke_n);
    }
    // SPIKE=1: shorthand for the standard FuBu v6.39 boot spike — pin the self-test
    // verdict (0x11FF15=0xC8) + SIM-gate bypass (0x11FD1B=0x06), applied from
    // POKEAFTER (defaults to 1.5M when SPIKE is set). Mirrors the web "Boot spike"
    // toggle. SPIKE unset/0 = organic boot (no verdict pin) — the DSP bring-up
    // milestone. Composes with any explicit POKE.
    if (getenv("SPIKE") && atoi(getenv("SPIKE")) != 0 && poke_n + 2 <= 64) {
        poke_addr[poke_n] = 0x11FF15; poke_val[poke_n] = 0xC8; poke_n++;
        poke_addr[poke_n] = 0x11FD1B; poke_val[poke_n] = 0x06; poke_n++;
        printf("SPIKE: verdict 0x11FF15=0xC8 + SIM-gate 0x11FD1B=0x06 (v6.39 boot spike)\n");
    }
    // SOFTWDT_KEEPALIVE=1 (research knob, default OFF): periodically clear the
    // software-watchdog supervisor counter at [0x11FEAC + 0x6E] = [0x11FF1A] —
    // the byte TASK_0_IDLE's SOFT_WDT_SUPERVISOR (0x002C1996) clears when the
    // idle path runs, and that the task-2 broker WDT-kick increments each tick.
    //
    // Useful for: genuine IDLE-task starvation scenarios — if some task hangs
    // such that IDLE never gets scheduled, the supervisor counter would overflow
    // (~15 ticks) and the broker would call REBOOT_FN itself with a reason
    // staged by some failure-flag fan-out in SOFT_WDT_SUPERVISOR (typically
    // REASON_FACTORY_RESET=4). This knob simulates "IDLE keeps running" from
    // outside, so you can observe whether downstream behavior depends on the
    // overflow reset path actually firing.
    //
    // NOT useful for: the SWDSP/DSP-reset reason-0x68 chain. That one is msg-driven
    // (SWDSP_STAGER stages [0x11FF94]=0x68, posts msg 0xDC to task-2 broker,
    // broker's msg-0xDC arm at 0x2444C4 re-reads the persisted reason and calls
    // REBOOT_FN(0x68) at 0x2444C8 — counter never overflows in that path). Use
    // the reboot_fn-intercept recovery framework for those.
    //
    // SOFTWDT_KEEPALIVE_PERIOD=N (default 1000000 cycles ≈ ~77 ms emu) sets the
    // clear cadence. Real firmware clears the counter many times per second via
    // IDLE; 1 M cycles is well under the 15-tick overflow window.
    long softwdt_keepalive = getenv("SOFTWDT_KEEPALIVE") ? atol(getenv("SOFTWDT_KEEPALIVE")) : 0;
    uint64_t softwdt_period = getenv("SOFTWDT_KEEPALIVE_PERIOD")
        ? (uint64_t)strtoull(getenv("SOFTWDT_KEEPALIVE_PERIOD"), NULL, 0) : 1000000ull;
    uint64_t softwdt_last_cyc = 0;
    if (softwdt_keepalive)
        printf("SOFTWDT_KEEPALIVE: clearing [0x0011FF1A] every %llu cycles "
               "(research knob; reroutes around the SWDSP/DSP-reset reason-0x68 chain)\n",
               (unsigned long long)softwdt_period);
    // POKE2 / POKEAFTER2: a SECOND independently-timed poke set. Needed when one poke
    // must take effect at a different time than another (e.g. v6.39: force the verdict
    // 0x11FF15=0xC8 early so GSM/PON completes, but defer the SIM-gate bypass
    // 0x11FD1B=0x06 until AFTER the system is ready, so the post-SIM setup chain runs
    // late like v5.79 instead of stalling on undrained startup queues).
    uint32_t poke2_addr[64]; uint8_t poke2_val[64]; int poke2_n = 0;
    if (getenv("POKE2")) {
        const char* p = getenv("POKE2");
        while (*p && poke2_n < 64) {
            uint32_t a = (uint32_t)strtoul(p, (char**)&p, 0);
            if (*p == '=') p++;
            uint32_t v = (uint32_t)strtoul(p, (char**)&p, 0);
            poke2_addr[poke2_n] = a; poke2_val[poke2_n] = (uint8_t)v; poke2_n++;
            if (*p == ',') p++;
        }
        printf("POKE2: %d byte(s) forced each step (from POKEAFTER2)\n", poke2_n);
    }
    // CALL=<fn>[ CALLR0=.. CALLR1=.. CALLR2=.. CALLR3=..] : once boot has settled
    // (step >= CALLAFTER, default 6M) and the CPU is in a safe (System) mode, hijack
    // execution to invoke a firmware Thumb function with the given args, returning to
    // wherever we were. Then run CALLSTEPS more (default 3M) and dump the LCD. This is
    // a live in-emulator function call: lets us drive a single dispatcher/draw routine
    // (e.g. a screen handler with MSG_D_INIT) without perpetually pinning state via POKE.
    uint32_t call_fn = (uint32_t)strtoul(getenv("CALL") ? getenv("CALL") : "0", NULL, 0);
    uint32_t call_r[4] = {
        (uint32_t)strtoul(getenv("CALLR0") ? getenv("CALLR0") : "0", NULL, 0),
        (uint32_t)strtoul(getenv("CALLR1") ? getenv("CALLR1") : "0", NULL, 0),
        (uint32_t)strtoul(getenv("CALLR2") ? getenv("CALLR2") : "0", NULL, 0),
        (uint32_t)strtoul(getenv("CALLR3") ? getenv("CALLR3") : "0", NULL, 0),
    };
    long call_after = getenv("CALLAFTER") ? atol(getenv("CALLAFTER")) : 6000000;
    long call_steps = getenv("CALLSTEPS") ? atol(getenv("CALLSTEPS")) : 3000000;
    // CALLN=<n> CALLEVERY=<steps>: fire the call <n> times, spaced <steps> apart
    // (default n=1). Used to drive a blocking receive loop (e.g. the DSP task id19
    // waking on each posted response) — see the DSP responder below.
    long call_n     = getenv("CALLN") ? atol(getenv("CALLN")) : 1;
    long call_every = getenv("CALLEVERY") ? atol(getenv("CALLEVERY")) : 200000;
    long call_count = 0; long call_fire_step = -1;
    int  call_fired = 0;  // set once the first call has fired
    // CALLR0LIST=a,b,c : on fire k use the k-th value as r0 (overrides CALLR0). Lets one
    // CALL= drive a fn over a list of args, e.g. resume a set of tasks via 0x298CE8.
    uint32_t call_r0list[32]; int call_r0n = 0;
    if (getenv("CALLR0LIST")) {
        const char* p = getenv("CALLR0LIST");
        while (*p && call_r0n < 32) {
            call_r0list[call_r0n++] = (uint32_t)strtoul(p, (char**)&p, 0);
            if (*p == ',') p++;
        }
        if (call_n < call_r0n) call_n = call_r0n;
        if (getenv("CALLEVERY") == NULL) call_every = 3000;  // default tighter spacing for a list
    }
    // CALLFNLIST=a,b,c : on fire k call the k-th fn (overrides CALL=). Pairs with
    // CALLR0LIST so one script can drive several different firmware fns in sequence
    // (e.g. resume tasks via 0x298CE8, then post via a trampoline).
    uint32_t call_fnlist[32]; int call_fnn = 0;
    if (getenv("CALLFNLIST")) {
        const char* p = getenv("CALLFNLIST");
        while (*p && call_fnn < 32) {
            call_fnlist[call_fnn++] = (uint32_t)strtoul(p, (char**)&p, 0);
            if (*p == ',') p++;
        }
        if (call_n < call_fnn) call_n = call_fnn;
        if (!call_fn) call_fn = call_fnlist[0];
        if (getenv("CALLEVERY") == NULL) call_every = 50000;
    }
    // Control-flow edge ring: record (from -> to, lr) on each non-sequential PC
    // transition (calls/branches/returns). Dump the last EDGE_N on exit so the
    // exact path into a stop point is visible.  Enable with EDGES=<n>.
    long edge_n = (long)strtoul(getenv("EDGES") ? getenv("EDGES") : "0", NULL, 0);
    if (edge_n > 4000) edge_n = 4000;
    // break the instant PC leaves flash (>=0x400000) after this step,
    // to catch the exact jump-into-garbage of the FIQ-mode wedge.
    long wedgetrap = getenv("WEDGETRAP") ? atol(getenv("WEDGETRAP")) : 0;
    struct { uint32_t from, to, lr; } *edges = edge_n ? calloc((size_t)edge_n, sizeof(*edges)) : NULL;
    long edge_w = 0; uint32_t prev_pc = 0;
    long edge_from = getenv("EDGE_FROM") ? atol(getenv("EDGE_FROM")) : 0;
    long edge_to   = getenv("EDGE_TO")   ? atol(getenv("EDGE_TO"))   : 0x7FFFFFFF;
    // RAM byte-change watch: log PC + new value whenever any byte in
    // [RAMWATCH, RAMWATCH+RAMWATCHLEN) changes (len defaults to 1).
    uint32_t ramw = (uint32_t)strtoul(getenv("RAMWATCH") ? getenv("RAMWATCH") : "0", NULL, 0);
    uint32_t ramw_len = getenv("RAMWATCHLEN") ? (uint32_t)strtoul(getenv("RAMWATCHLEN"), NULL, 0) : 1;
    if (ramw_len > 256) ramw_len = 256;
    uint8_t ramw_prevb[256];
    for (uint32_t k = 0; ramw && k < ramw_len; ++k) ramw_prevb[k] = c->ram[(ramw + k) & DCT3_RAM_MASK];
    int ramw_hits = 0;
    int ramw_cap = getenv("RAMWATCHCAP") ? (int)atol(getenv("RAMWATCHCAP")) : 60;
    long ramw_from = getenv("RAMWATCHFROM") ? atol(getenv("RAMWATCHFROM")) : 0;  // TEMP: gate RAMWATCH to steps >= this

    // RAMDUMP=<path>: snapshot full RAM (for disfw --ram). Trigger by step (RAMDUMPAT),
    // PC hit (RAMDUMPPC), or a byte change at RAMDUMPWATCH — all gated by RAMDUMPAFTER
    // (cycle window). RAMDUMPSTOP=1 ends the run after the dump.
    const char* ramdump_path  = getenv("RAMDUMP");
    long     ramdump_at       = getenv("RAMDUMPAT")    ? atol(getenv("RAMDUMPAT")) : -1;
    uint32_t ramdump_pc       = (uint32_t)strtoul(getenv("RAMDUMPPC")    ? getenv("RAMDUMPPC")    : "0", NULL, 0);
    uint32_t ramdump_watch    = (uint32_t)strtoul(getenv("RAMDUMPWATCH") ? getenv("RAMDUMPWATCH") : "0", NULL, 0);
    long     ramdump_after    = getenv("RAMDUMPAFTER") ? atol(getenv("RAMDUMPAFTER")) : 0;
    int      ramdump_done     = 0;
    uint8_t  ramdump_watch_pv = ramdump_watch ? c->ram[ramdump_watch & DCT3_RAM_MASK] : 0;

    // MEMLOG: install the rolling memory-access ring (flushed with the dump).
    if (getenv("MEMLOG")) {
        g_memring_n = getenv("MEMLOGN") ? atol(getenv("MEMLOGN")) : 512;
        if (g_memring_n < 1) g_memring_n = 512;
        g_memring = calloc((size_t)g_memring_n, sizeof(MemRec));
        dct3_set_mem_trace(c, &h, memlog_cb, getenv("MEMLOGRD") ? 1 : 0);
    }
    // CALLLOG: install the rolling call/jump ring (flushed with the dump).
    if (getenv("CALLLOG")) {
        g_callring_n = getenv("CALLLOGN") ? atol(getenv("CALLLOGN")) : 256;
        if (g_callring_n < 1) g_callring_n = 256;
        g_callring = calloc((size_t)g_callring_n, sizeof(CallRec));
    }
    long poke_after = getenv("POKEAFTER") ? atol(getenv("POKEAFTER"))
                    : (getenv("SPIKE") && atoi(getenv("SPIKE")) ? 1500000 : 0);  // SPIKE defaults POKEAFTER=1.5M
    long poke_after2 = getenv("POKEAFTER2") ? atol(getenv("POKEAFTER2")) : 0; // apply POKE2s only from this step
    // KEYRELEASE=<step>: release the held power key (kbd_special_cols=0) at this step,
    // modelling a real momentary power-button press->release. The boot needs PWR held
    // through the power-on-reason gate (~2M); the startup handler may wait for the
    // key-up before drawing the idle screen. -1 (default) = never release (hold).
    long key_release = getenv("KEYRELEASE") ? atol(getenv("KEYRELEASE")) : -1;
    int key_released = 0;
    // KEYPRESS=<step>[ KEYPRESSRC=<row>,<col> KEYPRESSDUR=<steps>]: model a real
    // momentary press of a NORMAL matrix key. At <step> set kbd_norm_cols[row] bit
    // <col> (key down) + raise keypad IRQ0; after KEYPRESSDUR steps clear it (key up)
    // + raise IRQ0 again. The keypad is wake-on-keypress (any change -> IRQ bit0 ->
    // ISR 0x2E8C10 -> scan 0x2EBE80 -> keycode via table 0x32E718). Default key =
    // (row 1,col 2) = KEY_1 (a digit -> disp49 number-entry draw).
    long key_press = getenv("KEYPRESS") ? atol(getenv("KEYPRESS")) : -1;
    long key_press_dur = getenv("KEYPRESSDUR") ? atol(getenv("KEYPRESSDUR")) : 400000;
    int kp_row = 1, kp_col = 2;
    if (getenv("KEYPRESSRC")) sscanf(getenv("KEYPRESSRC"), "%d,%d", &kp_row, &kp_col);
    kp_row &= 7; kp_col &= 7;
    int key_down = 0, key_up = 0;
    // PCMSINK=1: bind the HAL PCM channel (mad2.h pcm_sink) to a counting sink —
    // proves the host audio channel end-to-end without a file capture (the cosim
    // DXR tap feeds it; cf. DSP54_PCMCAP which writes the same stream to disk).
    // Default OFF; summary prints at exit next to the LCD block.
    if (getenv("PCMSINK")) h.mad2.pcm_sink = pcmsink_count;
    // SLIDEOPEN=<step>: open the slide-cover (reed switch) at this step on a slide phone;
    // SLIDECLOSE=<step>: close it. Drives mad2_slide_set (cover IRQ + open/closed message).
    long slide_open_at  = getenv("SLIDEOPEN")  ? atol(getenv("SLIDEOPEN"))  : -1;
    long slide_close_at = getenv("SLIDECLOSE") ? atol(getenv("SLIDECLOSE")) : -1;
    int slide_opened = 0, slide_closed = 0;

    // --- REPLAY=<json|file>: nav-style step-keyed replay (H1; Phase 8 plan 05) ---------
    // Accepts the minimal canonical form nav.mjs uses (tools/nav.mjs:127-188): a
    // step-keyed ARRAY  [{"key":"c","step":9692410261},{"key":"c","step":10321084655}]
    // — keys fired at an exact emulated INSTRUCTION-count step (the monotonic `steps`
    // counter, NOT raw cpu->cycles; cadence is step-based per Pitfall §"cadence must be
    // step-based, not cycles"). The value is either inline JSON or a path to a .json file.
    // Each record drives the SAME keypad-inject idiom KEYPRESS uses (set kbd_norm_cols[row]
    // bit + raise IRQ0 on DOWN, clear + raise on UP after REPLAYDUR steps) — no reimpl.
    //
    // KEYS[] vocabulary mirrors nav.mjs's 3310 keypad matrix EXACTLY (the [row,col] pairs
    // from web/index.html data-row/data-col), so a macro recorded against nav reproduces
    // byte-for-byte here. Unmapped names ("pwr"/"off"/"wait") are accepted but no-op the
    // matrix inject (pwr/off are power events the CLI models via KEYRELEASE / the powered
    // -off latch; "wait" is a pure delay) — they advance the cursor without a keypad edge.
    typedef struct { const char* name; int row; int col; int matrix; } ReplayKey;
    static const ReplayKey KEYS[] = {
        {"0",0,2,1},{"1",1,2,1},{"2",1,3,1},{"3",4,1,1},{"4",2,4,1},{"5",2,3,1},
        {"6",2,2,1},{"7",3,4,1},{"8",3,3,1},{"9",3,2,1},
        {"*",4,4,1},{"#",4,2,1},{"menu",4,3,1},{"up",0,1,1},{"down",1,1,1},{"c",0,4,1},
        {"pwr",0,0,0},{"off",0,0,0},{"wait",0,0,0},
    };
    typedef struct { long step; int row; int col; int matrix; char name[8]; } ReplayEvt;
    ReplayEvt* replay_evts = NULL; int replay_n = 0, replay_i = 0;
    int replay_down = 0; long replay_down_step = 0; int replay_down_row = 0, replay_down_col = 0;
    long replay_dur = getenv("REPLAYDUR") ? atol(getenv("REPLAYDUR")) : key_press_dur;
    if (getenv("REPLAY")) {
        const char* spec = getenv("REPLAY");
        char* buf = NULL;
        // If the spec names a readable file, slurp it; else treat it as inline JSON.
        FILE* rf = fopen(spec, "rb");
        if (rf) {
            fseek(rf, 0, SEEK_END); long rl = ftell(rf); fseek(rf, 0, SEEK_SET);
            buf = (char*)malloc((size_t)rl + 1);
            if (buf && rl > 0) { size_t got = fread(buf, 1, (size_t)rl, rf); buf[got] = 0; }
            else if (buf) buf[0] = 0;
            fclose(rf);
        }
        const char* json = buf ? buf : spec;
        // Minimal hand-rolled scan for the array-of-{key,step} form — no JSON lib. Walk the
        // string finding each "key":"<name>" and the nearest "step":<int> in the same record
        // (either order). Robust to whitespace; ignores any extra fields (cyc, etc.).
        int cap = 8; replay_evts = (ReplayEvt*)malloc((size_t)cap * sizeof(ReplayEvt));
        const char* p = json;
        while ((p = strstr(p, "\"key\"")) != NULL) {
            const char* q = strchr(p, ':'); if (!q) break;
            while (*q && (*q == ':' || *q == ' ' || *q == '\t')) q++;
            char kname[8] = {0};
            if (*q == '"') {
                q++; int kn = 0;
                while (*q && *q != '"' && kn < 7) kname[kn++] = *q++;
                kname[kn] = 0;
            }
            // Find the step in this record: search both directions to the record braces.
            // Simplest robust approach: look for the nearest "step" after this "key" but
            // before the next "key" (records are objects; nav emits key then step or vice
            // versa — scan a bounded window from the enclosing '{').
            const char* rec_start = p; while (rec_start > json && *rec_start != '{') rec_start--;
            const char* rec_end = strchr(p, '}'); if (!rec_end) rec_end = json + strlen(json);
            long stepv = -1;
            const char* sp = strstr(rec_start, "\"step\"");
            if (sp && sp < rec_end) {
                const char* sq = strchr(sp, ':');
                if (sq) { sq++; while (*sq == ' ' || *sq == '\t') sq++; stepv = atol(sq); }
            }
            if (stepv >= 0 && kname[0]) {
                int row = 0, col = 0, matrix = 0, known = 0;
                for (size_t ki = 0; ki < sizeof(KEYS)/sizeof(KEYS[0]); ++ki)
                    if (strcmp(KEYS[ki].name, kname) == 0) {
                        row = KEYS[ki].row; col = KEYS[ki].col; matrix = KEYS[ki].matrix; known = 1; break;
                    }
                // Explicit row/col in the record override the name table (lets a GUIKEYLOG
                // capture replay any model's matrix verbatim — e.g. 3410 SL/DN/UP nav keys).
                { const char* rp = strstr(rec_start, "\"row\"");
                  const char* cp = strstr(rec_start, "\"col\"");
                  if (rp && rp < rec_end && cp && cp < rec_end) {
                    const char* rq = strchr(rp, ':'); const char* cq = strchr(cp, ':');
                    if (rq && cq) { row = atoi(rq + 1); col = atoi(cq + 1); matrix = 1; known = 1; } } }
                if (!known) fprintf(stderr, "[replay] WARN unknown key '%s' @step %ld — cursor-advance only\n", kname, stepv);
                if (replay_n >= cap) { cap *= 2; replay_evts = (ReplayEvt*)realloc(replay_evts, (size_t)cap * sizeof(ReplayEvt)); }
                replay_evts[replay_n].step = stepv;
                replay_evts[replay_n].row = row; replay_evts[replay_n].col = col;
                replay_evts[replay_n].matrix = matrix;
                snprintf(replay_evts[replay_n].name, sizeof(replay_evts[replay_n].name), "%s", kname);
                replay_n++;
            }
            p = rec_end;
        }
        free(buf);
        printf("REPLAY: %d step-keyed event(s) parsed (cadence = INSTRUCTION count, dur=%ld); "
               "KEYS matrix matches nav.mjs\n", replay_n, replay_dur);
        for (int e = 0; e < replay_n; ++e)
            printf("  [replay] @step %ld  key=%s  (row %d,col %d)%s\n",
                   replay_evts[e].step, replay_evts[e].name, replay_evts[e].row,
                   replay_evts[e].col, replay_evts[e].matrix ? "" : "  [non-matrix: cursor-advance]");
    }

    // --- TRACE=<subsystems>: diffable event trace for real-vs-emulated comparison -----
    // Emits compact, timing-stripped `[T] <TAG> ... @<step>` records for the SIM and DSP
    // handshakes — the same events the on-device monitor will stream over MBUS, so a real
    // capture diffs 1:1 against the emulator (strip the `@<step>` suffix; compare the
    // event sequence). for the probe table + on-device mirror.
    //   TRACE=dsp  DSP boot/runtime handshake (verdict, mbox acks, code-blocks, MDIRCV, IRQ4)
    //   TRACE=sim  SIM recognition (disp49 gate eval, gate var, recognition timeout) + APDUs
    //   TRACE=all  both.  All addresses are per-build via h.mad2.fw; SIM gate var is
    //              fw.sim_gate-0x1D and the struct offsets hang off it (v5.79-derived).
    unsigned trace = 0;
    if (getenv("TRACE")) {
        const char* t = getenv("TRACE");
        if (strstr(t, "all")) trace = ~0u;
        if (strstr(t, "dsp")) trace |= 1u;
        if (strstr(t, "sim")) trace |= 2u;
        if ((trace & 2u) && !getenv("SIMLOG")) setenv("SIMLOG", "1", 1);  // APDU lines join the SIM trace
    }
    const uint32_t TGATE = (h.mad2.fw.sim_gate - 0x1D) & DCT3_RAM_MASK;  // gate var [0x11FCFE]
    uint8_t  tr_verdict  = c->ram[h.mad2.fw.verdict      & DCT3_RAM_MASK];
    uint8_t  tr_uploaded = c->ram[h.mad2.fw.dsp_uploaded & DCT3_RAM_MASK];
    uint8_t  tr_gate     = c->ram[TGATE];
    uint64_t tr_acks = h.mad2.dsp_acks, tr_cbacks = h.mad2.dsp_cb_acks;
    uint16_t tr_mdtail = 0; int tr_irq4 = 0, tr_selftest = 0;
    uint32_t tr_disp49 = 0xFFFFFFFFu;   // last (gate<<16|sub<<8|sel) emitted, to dedup re-evals
    // print N firmware PCs after each IRQ2 (CCONT cascade) delivery, to
    // map the cascade-dispatch path (vector -> common IRQ -> CCONT int service -> send msg).
    // IRQ2TRACE_AFTER gates to IRQ2s raised after a given step.
    long irq2trace = getenv("IRQ2TRACE") ? atol(getenv("IRQ2TRACE")) : 0;
    long irq2trace_after = getenv("IRQ2TRACE_AFTER") ? atol(getenv("IRQ2TRACE_AFTER")) : 0;
    long irq2trace_left = 0; int irq2_seen = 0;
    // when PC first hits TRACEFROM after step TRACEAFTER, print the next
    // TRACEN PCs (decode a routine's path). Reuses the irq2trace print.
    uint32_t tracefrom = (uint32_t)strtoul(getenv("TRACEFROM") ? getenv("TRACEFROM") : "0", NULL, 0);
    long traceafter = getenv("TRACEAFTER") ? atol(getenv("TRACEAFTER")) : 0;
    long tracen = getenv("TRACEN") ? atol(getenv("TRACEN")) : 200;
    int trace_seen = 0;
    // SWEEP=1 mode: at SWEEP_AT, fork per group code in [SWEEP_LO,SWEEP_HI],
    // inject MDIRCV, run SWEEP_AFTER more steps per child. See do_sweep().
    int  sweep_enabled = getenv("SWEEP") ? atoi(getenv("SWEEP")) : 0;
    long sweep_at      = getenv("SWEEP_AT")     ? atol(getenv("SWEEP_AT"))     : 0;
    long sweep_after   = getenv("SWEEP_AFTER")  ? atol(getenv("SWEEP_AFTER"))  : 30000000;
    int  sweep_lo      = getenv("SWEEP_LO")     ? (int)strtoul(getenv("SWEEP_LO"), NULL, 0) : 0;
    int  sweep_hi      = getenv("SWEEP_HI")     ? (int)strtoul(getenv("SWEEP_HI"), NULL, 0) : 255;
    int  sweep_par     = getenv("SWEEP_PARALLEL") ? atoi(getenv("SWEEP_PARALLEL")) : 8;
    const char* sweep_payload_spec = getenv("SWEEP_PAYLOAD");
    const char* sweep_outdir       = getenv("SWEEP_OUTDIR") ? getenv("SWEEP_OUTDIR") : "/tmp/dsp_sweep";
    if (sweep_enabled && sweep_at == 0) sweep_at = 4500000;
    int sweep_done = 0;

    // --- Shared harness (Phase 8 H0): fault-detect + reset-recovery orchestration.
    // The per-step reboot/fatal/assert/stage/heap-fail snapshots, the recover-apply,
    // and the wild-PC/spin/CCONT-reset/power-off classification all live ONCE in
    // src/harness/ now (no CLI-vs-web reimplementation). boot_trace is a thin input
    // (keypad/replay) + output (PNG/stdout) driver calling harness_observe() per step.
    //
    // D-10: recovery DEFAULTS OFF in the CLI driver — an honest harness HALTS on a
    // firmware self-reset (renders the post-mortem) instead of silently warm-recovering
    // and running to budget. Opt back IN with RESET_RECOVER (any value re-enables the
    // master gate; per-reason RESET_RECOVER=N,M still tunes the recover list in mad2).
    int recovery_default = getenv("RESET_RECOVER") ? 1 : 0;
    HarnessConfig hc = {0};
    // wild_pc_after: WEDGETRAP=N delays the wild-PC arm to step N. Default to a short
    // 16-step warmup so the boot-ROM HLE pipeline priming (the reset-vector branch at
    // step 0 leaves a transient out-of-range de-piped PC) doesn't false-fire before the
    // firmware's first real instruction runs.
    int64_t wild_after = wedgetrap > 0 ? (int64_t)wedgetrap : 16;
    harness_init(&hc, &h.mad2, recovery_default, SPIN_LIMIT, wild_after);
    printf("HARNESS: recovery %s (D-10 default OFF; RESET_RECOVER opts in), "
           "spin_limit=%ld, wild-PC armed @step %lld\n",
           recovery_default ? "ON" : "OFF", SPIN_LIMIT, (long long)wild_after);

#ifdef DCT3_SDL
    if (gui_on) {
        gui_init(prof);
        // DIAGNOSTIC: GUI_PCMTEST=<file> plays a PCM capture through the audio path with
        // NO core running (isolates consumption vs core-production artefacts). Exits after.
        if (gui_pcm_selftest()) { gui_shutdown(); return 0; }
        // Route the HAL PCM channel to the GUI's SDL audio device so the DSP
        // codec earpiece (e.g. the 5110 cosim keypad beep) is audible live.
        // Wins over the PCMSINK demo counter when the GUI is up.
        h.mad2.pcm_sink = gui_pcm_sink;
        // GUIPWROFF=1: start in the OFF state. Render the (blank) panel and wait for a
        // power-key tap before the firmware runs — mirroring a real phone that boots only
        // when you press POWER. A PWR tap sets gi.reboot (gui_frame off-state path); we
        // then fall through to the run loop and boot the already-initialised mad2 normally.
        // Only on the initial launch (before gui_run_start) so a later power-on just runs.
        if (getenv("GUIPWROFF")) {
            fprintf(stderr, "[gui] GUIPWROFF: phone is OFF — press P to power on\n");
            GuiInput gpi = {0, 0, 0};
            while (!gpi.quit && !gpi.reboot)
                gpi = gui_frame(&h.mad2, &c->cpu, 0, "powered off");
            if (gpi.quit) { gui_shutdown(); return 0; }
        }
    }
gui_run_start:   // in-process warm-reboot target (PWR held 30s); GUI build only
#endif

    // CFTRACE: control-flow + interrupt trace. Logs every taken BL/BLX/BX (pc->target, lr)
    // and every FIQ/IRQ entry (with the raised line), each tagged with CPU mode — for diffing
    // 3310-vs-3410 early boot. Bounded by CFTRACE_AFTER/CFTRACE_UNTIL/CFTRACE_CAP. Addresses
    // are post-annotated to symbols via tools/cfannotate.py <build>.
    long cft_on    = getenv("CFTRACE") ? 1 : 0;
    long cft_after = getenv("CFTRACE_AFTER") ? atol(getenv("CFTRACE_AFTER")) : 0;
    long cft_until = getenv("CFTRACE_UNTIL") ? atol(getenv("CFTRACE_UNTIL")) : 0;  // 0 = no upper bound
    long cft_cap   = getenv("CFTRACE_CAP")   ? atol(getenv("CFTRACE_CAP"))   : 500000;
    // INTFIRE: focused interrupt-fire trace (no BL/BLX spam). Logs each FIQ/IRQ delivery
    // with source, interrupted PC, mode-before (preemption tag), pending/mask, and the
    // 0x2000C master gate (int_ctrl) at fire time — to see which line leaks into a
    // guarded section before a wild-PC. INTFIRE_AFTER gates by step.
    long intfire       = getenv("INTFIRE") ? 1 : 0;
    long intfire_after = getenv("INTFIRE_AFTER") ? atol(getenv("INTFIRE_AFTER")) : 0;
    long intfire_cap   = getenv("INTFIRE_CAP")   ? atol(getenv("INTFIRE_CAP"))   : 4000000;

    // MBUS service bridge: present the phone's MBUS on a host serial port / PTY so a real
    // service tool (WinTesla/EepromTools/NokiX, e.g. Wine) drives the firmware's own service
    // handlers over the genuine protocol. MBUSPORT=<dev> uses that device; MBUSBRIDGE=1 or
    // MBUSTNT=1 auto-claims a tty0tty pair so the port presents REAL modem lines (DTR/RTS/DCD —
    // no shim); MBUSPTY=1 forces a bare PTY (the LD_PRELOAD-shim path). MBUSECHO=0 disables the
    // single-wire line echo. When active we pace the loop to wall-clock so the firmware's timers
    // + frame latency match what the tool expects (and we don't burn the step budget in seconds).
    // Pass a large step budget for an interactive session.
    int mbus_fd = -1;
    {
        const char* mp = getenv("MBUSPORT");
        int want = (mp && *mp) || getenv("MBUSPTY") || getenv("MBUSBRIDGE") || getenv("MBUSTNT");
        if (want) {
            int echo = getenv("MBUSECHO") ? atoi(getenv("MBUSECHO")) : 1;
            mbus_fd = mbus_bridge_open(mp, echo);
        }
    }
    struct timespec mbus_anchor = {0, 0}; uint64_t mbus_anchor_cyc = 0; int mbus_anchored = 0;

    for (; steps < budget; ++steps) {
        struct ARMCore* cpu = &c->cpu;
        uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);

        // Headless keypad injection (calibration/testing): KEYINJECT=ROW,COL presses that
        // matrix cell at KEYINJECTAT (default 4M) for KEYHOLD steps (default 300k) then
        // releases — same path the GUI uses (set kbd_norm_cols + mad2_keypad_irq).
        {
            static int ki = -2, kr = 0, kc = 0, kst = 0; static long kat = 0, khold = 0;
            if (ki == -2) { const char* e = getenv("KEYINJECT");
                if (e && sscanf(e, "%d,%d", &kr, &kc) == 2) {
                    kat = getenv("KEYINJECTAT") ? atol(getenv("KEYINJECTAT")) : 4000000;
                    khold = getenv("KEYHOLD") ? atol(getenv("KEYHOLD")) : 300000; ki = 1;
                    printf("[keyinj] row=%d col=%d at=%ld hold=%ld\n", kr, kc, kat, khold);
                } else ki = 0; }
            if (ki == 1) {
                if (kst == 0 && steps >= kat) {
                    h.mad2.kbd_norm_cols[kr] |= (uint8_t)(1u << kc); mad2_keypad_irq(&h.mad2);
                    kst = 1; printf("[keyinj] PRESS (%d,%d) @step %ld\n", kr, kc, steps);
                } else if (kst == 1 && steps >= kat + khold) {
                    h.mad2.kbd_norm_cols[kr] &= (uint8_t)~(1u << kc); mad2_keypad_irq(&h.mad2);
                    kst = 2; printf("[keyinj] RELEASE @step %ld\n", steps);
                }
            }
        }

        // MBUS frame injection (MBUSFRAME=<hex>): feed an MBus service frame to the
        // running firmware as if a tool were talking on the 1-wire bus. Bytes are paced
        // one at a time — pushed only when the RX FIFO has drained AND the firmware has
        // RX enabled (ctrl bit6 of 0x20018) — each push raising FIQ2, the accessory path
        // mbus_rx_signal documents. MBUSAT gates the start; MBUSLOG logs. This drives the
        // firmware's OWN MBUS service dispatch (B-path: the phone parses + writes its FFS).
        {
            // MBUSFRAME may carry a SEQUENCE of frames separated by ';' (e.g. a two-shot
            // service protocol). MBUSAT is a matching comma-list of per-frame inject steps;
            // a frame starts only once the previous one is fully delivered AND its own step
            // is reached — so a stateful command (write-after-arm) can be paced after the
            // firmware has processed the prior frame (handlers run asynchronously, ~1.5M
            // steps later). Single frame + single MBUSAT still works as before.
            #define MBF_MAXF 8
            static int mbi = -2, mblog = 0;
            static uint8_t mbf[MBF_MAXF][256]; static int mbn[MBF_MAXF]; static long mbat[MBF_MAXF];
            static int mbnf = 0, mbcur = 0, mbp = 0; static int mbcur_done = -1;
            if (mbi == -2) {
                const char* hx = getenv("MBUSFRAME");
                const char* at = getenv("MBUSAT");
                mblog = getenv("MBUSLOG") ? 1 : 0;
                while (hx && *hx && mbnf < MBF_MAXF) {                 // parse ';'-separated frames
                    int n = 0;
                    while (*hx && *hx != ';' && n < (int)sizeof(mbf[0])) {
                        while (*hx==' '||*hx==',') hx++;
                        if (*hx == ';' || !*hx) break;
                        unsigned v; if (sscanf(hx,"%2x",&v)==1){mbf[mbnf][n++]=(uint8_t)v;hx+=2;} else break;
                    }
                    if (n > 0) { mbn[mbnf] = n; mbat[mbnf] = 0; mbnf++; }
                    if (*hx == ';') hx++;
                }
                for (int k = 0; k < mbnf; k++) {                      // parse ','-separated inject steps
                    if (at && *at) { mbat[k] = atol(at); const char* c = strchr(at, ','); at = c ? c+1 : at+strlen(at); }
                    else if (k > 0) mbat[k] = mbat[k-1];              // missing -> reuse previous
                }
                if (mbnf) printf("[mbus] %d frame(s), inject steps:", mbnf);
                for (int k = 0; k < mbnf; k++) if (mbnf) printf(" [%d]=%ldB@%ld", k, (long)mbn[k], mbat[k]);
                if (mbnf) printf(" (RX-enable gated)\n");
                mbi = (mbnf > 0);
            }
            if (mbi == 1 && mbcur < mbnf) {
                if (mbp < mbn[mbcur] && steps >= mbat[mbcur] && (h.mad2.mbus_ctrl & 0x40)
                    && (uint8_t)(h.mad2.mbus_rx_tail - h.mad2.mbus_rx_head) == 0) {
                    uint8_t b = mbf[mbcur][mbp++];
                    h.mad2.mbus_rx[h.mad2.mbus_rx_tail++ % sizeof(h.mad2.mbus_rx)] = b;
                    h.mad2.mbus_rxdrdy = 1; h.mad2.fiq_pending |= (1u << 2);   // FIQ2 = MBUS RX
                    if (mblog) printf("[mbus] frame %d inject [%d/%d] 0x%02X @step %ld\n", mbcur, mbp, mbn[mbcur], b, steps);
                } else if (mbp == mbn[mbcur] && mbcur_done != mbcur) {
                    mbcur_done = mbcur;
                    printf("[mbus] frame %d delivered: fw read %llu MBUS bytes total @step %ld\n",
                           mbcur, (unsigned long long)h.mad2.mbus_rx_bytes, steps);
                    mbcur++; mbp = 0;                                 // advance to next frame
                }
            }
        }

        // MBUS service bridge. RX must be fed PER STEP (mbus_bridge_feed) so a byte lands the
        // instant the firmware drains the previous one — the firmware's RX-enable opens only
        // briefly between bytes, and a periodic feed skips past those windows so the frame
        // never assembles. The fd I/O + TX forward + wall-clock pacing stay batched (every
        // 256 steps) to keep syscalls off the hot path.
        if (mbus_fd >= 0) mbus_bridge_feed(&h.mad2);
        if (mbus_fd >= 0 && (steps & 0xFF) == 0) {
            mbus_bridge_poll(&h.mad2, mbus_fd);
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            if (!mbus_anchored) { mbus_anchor = now; mbus_anchor_cyc = h.mad2.rtc_mono; mbus_anchored = 1; }
            else {
                double emu_s  = (double)(h.mad2.rtc_mono - mbus_anchor_cyc) / 13.0e6;  // 13 MHz
                double wall_s = (now.tv_sec - mbus_anchor.tv_sec) + (now.tv_nsec - mbus_anchor.tv_nsec) / 1e9;
                double ahead  = emu_s - wall_s;
                if (ahead > 0.001) {                       // ahead of real time -> sleep the surplus
                    if (ahead > 0.05) ahead = 0.05;        // cap a single sleep (stay responsive to the fd)
                    struct timespec ts = { (time_t)ahead, (long)((ahead - (double)(time_t)ahead) * 1e9) };
                    nanosleep(&ts, NULL);
                } else if (ahead < -0.5) {                 // fell behind (cosim) -> re-anchor, never sprint
                    mbus_anchor = now; mbus_anchor_cyc = h.mad2.rtc_mono;
                }
            }
        }
#ifdef DCT3_SDL
        // Render one frame per 1/60 s of EMULATED time (rtc_mono is in ARM cycles;
        // 13 MHz / 60 ~= 216667 cyc). gui_frame ALSO sleeps to hold emulated time to
        // wall-clock (the realtime throttle in gui_sdl.c) so the GUI clock can't outrun
        // real time when vsync fails to throttle -- without that the step loop runs flat
        // out, a single physical key-hold spans millions of emulated steps, and the
        // firmware's hold-auto-repeat turns one tap into a menu-scroll storm. (vsync, when
        // honoured, makes that sleep ~zero; the throttle is the robust floor.)
        static uint64_t gui_last_frame_cyc = 0;
        if (gui_on && (h.mad2.rtc_mono - gui_last_frame_cyc) >= 216667u) {
            gui_last_frame_cyc = h.mad2.rtc_mono;
            GuiInput gi = gui_frame(&h.mad2, &c->cpu, (long long)steps, NULL);
            if (gi.reboot) goto gui_warm_reboot;
            while (gi.paused && !gi.quit) gi = gui_frame(&h.mad2, &c->cpu, (long long)steps, NULL);
            if (gi.quit) { reason = "GUI window closed"; break; }
        }
#endif
        if (ramdump_path && !ramdump_done && steps >= ramdump_after) {
            int trig = 0;
            if (ramdump_at >= 0 && steps >= ramdump_at) trig = 1;
            else if (ramdump_pc && pc == ramdump_pc) trig = 1;
            else if (ramdump_watch) {
                uint8_t v = c->ram[ramdump_watch & DCT3_RAM_MASK];
                if (v != ramdump_watch_pv) trig = 1;
                ramdump_watch_pv = v;
            }
            if (trig) {
                ramdump_write(c, &c->cpu, steps, pc, ramdump_path);
                memlog_flush(ramdump_path);
                callring_flush(ramdump_path);
                ramdump_done = 1;
                if (getenv("RAMDUMPSTOP")) { reason = "RAMDUMP done"; break; }
            }
        }
        if (tracefrom && pc == tracefrom && steps >= traceafter && trace_seen < 1) {
            trace_seen++; irq2trace_left = tracen;
            printf("[trc] >>> hit %06X @step %ld\n", pc & 0xFFFFFFu, steps);
        }
        if (irq2trace_left > 0) { printf("[irq2] %06X\n", pc & 0xFFFFFFu); irq2trace_left--; }
        // log RTOS message sends — entry of send 0x29921C (task ctx) and the
        // IRQ-ctx twin 0x2997B0. r0=target task id, r1=message. Shows the live message graph
        // (which tasks ever get messages). SENDLOG_AFTER gates by step.
        {
            static long sl = -2, sla = 0; static long sln = 0; static long sl_max = 4000;
            static uint32_t sl_pc = 0x29921Cu, sl_pc2 = 0x2997B0u;  // 3310 defaults; override per model
            if (sl == -2) {
                sl = getenv("SENDLOG") ? 1 : 0; sla = getenv("SENDLOG_AFTER") ? atol(getenv("SENDLOG_AFTER")) : 0;
                if (getenv("SENDLOG_PC"))  sl_pc  = (uint32_t)strtoul(getenv("SENDLOG_PC"), NULL, 0);
                if (getenv("SENDLOG_PC2")) sl_pc2 = (uint32_t)strtoul(getenv("SENDLOG_PC2"), NULL, 0);
                if (getenv("SENDLOG_MAX")) sl_max = atol(getenv("SENDLOG_MAX"));
            }
            if (sl && steps >= sla && (pc == sl_pc || pc == sl_pc2) && sln < sl_max) {
                sln++;
                printf("[send] %s task=%lu msg=%04lX LR=%06lX @%ld\n",
                       pc == sl_pc ? "snd" : "irq",
                       (unsigned long)(cpu->gprs[0] & 0xFF), (unsigned long)(cpu->gprs[1] & 0xFFFF),
                       (unsigned long)(cpu->gprs[14] & 0xFFFFFF), steps);
            }
        }
        if (key_release >= 0 && !key_released && steps >= key_release) {
            h.mad2.kbd_special_cols = 0; key_released = 1;
            // The keypad is wake-on-keypress: a key state change raises IRQ bit0
            // (handler 0x2E8C10 -> scan 0x2EBD76). Releasing the key without the
            // interrupt is invisible to the firmware, so assert bit0 here.
            h.mad2.irq_pending |= 0x01;
            printf("KEYRELEASE: power key released + keypad IRQ0 raised at step %ld\n", steps);
        }
        if (slide_open_at >= 0 && !slide_opened && steps >= slide_open_at) {
            mad2_slide_set(&h.mad2, 1); slide_opened = 1;
            printf("SLIDEOPEN: reed switch -> open + cover IRQ at step %ld\n", steps);
        }
        if (slide_close_at >= 0 && !slide_closed && steps >= slide_close_at) {
            mad2_slide_set(&h.mad2, 0); slide_closed = 1;
            printf("SLIDECLOSE: reed switch -> closed + cover IRQ at step %ld\n", steps);
        }
        if (key_press >= 0 && !key_down && steps >= key_press) {
            h.mad2.kbd_norm_cols[kp_row] |= (uint8_t)(1u << kp_col);
            mad2_keypad_irq(&h.mad2); key_down = 1;   // matrix edge (8850-class source bit too)
            printf("KEYPRESS: key DOWN (row %d,col %d) + keypad IRQ0 at step %ld\n", kp_row, kp_col, steps);
        }
        if (key_down && !key_up && steps >= key_press + key_press_dur) {
            h.mad2.kbd_norm_cols[kp_row] &= (uint8_t)~(1u << kp_col);
            mad2_keypad_irq(&h.mad2); key_up = 1;
            printf("KEYPRESS: key UP + keypad IRQ0 at step %ld\n", steps);
        }
        // REPLAY (H1): fire the next step-keyed event when the INSTRUCTION-count `steps`
        // counter reaches its step. DOWN drives the SAME keypad-inject idiom KEYPRESS uses
        // (set kbd_norm_cols bit + raise IRQ0); UP clears it + raises IRQ0 after replay_dur
        // steps. A non-matrix key ("pwr"/"off"/"wait") just advances the cursor (no edge).
        if (replay_down && steps >= replay_down_step + replay_dur) {
            h.mad2.kbd_norm_cols[replay_down_row] &= (uint8_t)~(1u << replay_down_col);
            mad2_keypad_irq(&h.mad2); replay_down = 0;
            printf("REPLAY: key UP (row %d,col %d) + keypad IRQ0 at step %ld\n",
                   replay_down_row, replay_down_col, steps);
        }
        if (!replay_down && replay_i < replay_n && steps >= replay_evts[replay_i].step) {
            ReplayEvt* ev = &replay_evts[replay_i];
            if (ev->matrix) {
                h.mad2.kbd_norm_cols[ev->row] |= (uint8_t)(1u << ev->col);
                mad2_keypad_irq(&h.mad2);
                replay_down = 1; replay_down_step = steps;
                replay_down_row = ev->row; replay_down_col = ev->col;
                printf("REPLAY: key DOWN %s (row %d,col %d) + keypad IRQ0 at step %ld\n",
                       ev->name, ev->row, ev->col, steps);
            } else {
                printf("REPLAY: non-matrix event %s at step %ld (cursor-advance only)\n",
                       ev->name, steps);
            }
            replay_i++;
        }
        // CHARGERAT=<step>: simulate plugging a charger (set charger-voltage ADC ch5)
        // so the CCONT charger-detect (CContINT3 -> IRQ2) edge fires. Verifies the
        // firmware's IRQ2 cascade handler responds (reads CCONT 0x0E, runs charge UI).
        { static long chg_at = -2; static int chg_done = 0;
          if (chg_at == -2) chg_at = getenv("CHARGERAT") ? atol(getenv("CHARGERAT")) : -1;
          if (chg_at >= 0 && !chg_done && steps >= chg_at) {
              h.mad2.adc[5] = 0x2C0; chg_done = 1;
              printf("CHARGERAT: charger connected (adc5=0x2C0) at step %ld\n", steps);
          } }
        if (steps >= poke_after)
            for (int pk = 0; pk < poke_n; ++pk) c->ram[poke_addr[pk] & DCT3_RAM_MASK] = poke_val[pk];
        if (getenv("POKE2") && steps >= poke_after2)
            for (int pk = 0; pk < poke2_n; ++pk) c->ram[poke2_addr[pk] & DCT3_RAM_MASK] = poke2_val[pk];
        // A hijacked call is "active" between fire and its return to the hijack PC; we
        // save the full register/flag context on fire and restore it on return, so the
        // call can be fired mid-execution (e.g. resuming tasks from any boot context)
        // without corrupting the interrupted code's registers (only RAM effects persist).
        static int call_active = 0; static uint32_t call_ret_pc = 0;
        static int32_t call_save_gpr[15]; static uint32_t call_save_cpsr = 0;
        if (call_active && pc == call_ret_pc) {                 // call returned: restore context
            for (int i = 0; i < 15; ++i) cpu->gprs[i] = call_save_gpr[i];
            cpu->cpsr.packed = call_save_cpsr;
            call_active = 0;
        }
        if (call_fn && !call_active && call_count < call_n && steps >= call_after + call_count * call_every
                && cpu->privilegeMode == MODE_SYSTEM) {
            for (int i = 0; i < 15; ++i) call_save_gpr[i] = (int32_t)cpu->gprs[i];
            call_save_cpsr = cpu->cpsr.packed; call_ret_pc = pc;
            uint32_t r0v = (call_count < call_r0n) ? call_r0list[call_count] : call_r[0];
            uint32_t fnv = (call_count < call_fnn) ? call_fnlist[call_count] : call_fn;
            cpu->gprs[0] = r0v; cpu->gprs[1] = call_r[1];
            cpu->gprs[2] = call_r[2]; cpu->gprs[3] = call_r[3];
            cpu->gprs[ARM_LR] = pc | (cpu->cpsr.t ? 1u : 0u);  // return to hijack point, same ISA
            cpu->gprs[ARM_PC] = fnv & ~1u;
            _ARMSetMode(cpu, MODE_THUMB); cpu->cpsr.t = 1;
            cpu->cycles += ThumbWritePC(cpu);
            printf("CALL[%ld]: 0x%08X(r0=%08X r1=%08X r2=%08X r3=%08X)  fired @step %ld  (was PC=%08X SP=%08X)\n",
                   call_count, fnv, r0v, call_r[1], call_r[2], call_r[3], steps, pc, (uint32_t)cpu->gprs[ARM_SP]);
            call_fired = 1; call_count++; call_fire_step = steps;
            hc.spin_anchor = 0xFFFFFFFFu; hc.spin_count = 0;   // fresh basic-block window
            continue;
        }
        if (call_fired && !call_active && call_count >= call_n && steps - call_fire_step >= call_steps) {
            reason = "CALL window elapsed"; break;
        }
        // HEAPSAMPLE / HEAPGUARD / HEAPSHADOW moved into the shared core
        // (src/harness/heap_shadow.c, D-07) — dispatched from harness_observe below
        // so BOTH drivers inherit the allocator shadow + poison-free trap. The env
        // knobs (HEAPSAMPLE=<N> / HEAPGUARD=1 / HEAPSHADOW=1 / HEAPGUARD_NOSTOP=1)
        // are unchanged; a HEAPGUARD bad-free now returns HARNESS_FAULT_HALT.
        if (ramw) {
            for (uint32_t k = 0; k < ramw_len; ++k) {
                uint8_t v = c->ram[(ramw + k) & DCT3_RAM_MASK];
                if (v != ramw_prevb[k] && ramw_hits < ramw_cap && steps >= ramw_from) {
                    printf("[ramw] [%08X] %02X -> %02X  at PC=%08X LR=%08X r0=%08X r4=%08X r5=%08X (step %ld)\n",
                           ramw + k, ramw_prevb[k], v, prev_pc, (uint32_t)cpu->gprs[14], (uint32_t)cpu->gprs[0], (uint32_t)cpu->gprs[4], (uint32_t)cpu->gprs[5], steps);
                    ramw_hits++;
                }
                ramw_prevb[k] = v;
            }
        }
        if (edges && steps >= edge_from && steps <= edge_to) {
            uint32_t d = pc - prev_pc;
            if (d != 2 && d != 4) {   // non-sequential => control-flow edge
                long last = (edge_w - 1) % edge_n;
                if (edge_w == 0 || edges[last].from != prev_pc || edges[last].to != pc) {
                    edges[edge_w % edge_n].from = prev_pc;   // skip repeats (tight loops)
                    edges[edge_w % edge_n].to   = pc;
                    edges[edge_w % edge_n].lr   = (uint32_t)cpu->gprs[14];
                    edge_w++;
                }
            }
        }
        prev_pc = pc;
        // GETSTR=1: trace get_string (0x2BBFAC) — log the caller, the decoded nokstr (PPM
        // string ID, or ascii), and the resolved text (read at the function's return). Maps
        // any on-screen string to the code that requests it. GETSTRN caps the number logged.
        if (getenv("GETSTR")) {
            static uint32_t gs_ret = 0, gs_lr = 0, gs_in = 0; static int gs_wide = 0;
            static long gs_cap = -1;
            if (gs_cap < 0) gs_cap = getenv("GETSTRN") ? atol(getenv("GETSTRN")) : 120;
            uint32_t gs_pc  = h.mad2.fw.get_string   ? h.mad2.fw.get_string   : 0x002BBFACu;
            uint32_t gs_wpc = h.mad2.fw.w_get_string ? h.mad2.fw.w_get_string : 0x002BBCB8u;
            if ((pc == gs_pc || pc == gs_wpc) && gs_ret == 0 && gs_cap > 0) {
                gs_in = (uint32_t)cpu->gprs[0];
                gs_lr = (uint32_t)cpu->gprs[14];
                gs_ret = gs_lr & ~1u;
                gs_wide = (pc == gs_wpc);
            } else if (gs_ret && pc == gs_ret) {
                uint32_t res = (uint32_t)cpu->gprs[0];
                uint8_t b0 = c->ram[gs_in & DCT3_RAM_MASK];
                uint8_t b1 = c->ram[(gs_in + 1) & DCT3_RAM_MASK];
                uint8_t b2 = c->ram[(gs_in + 2) & DCT3_RAM_MASK];
                char txt[64]; int n = 0, zeros = 0;          // skip 0 bytes (UCS-2), stop on 00 00
                for (int i = 0; i < 120 && n < 60; ++i) {
                    uint8_t ch = c->ram[(res + (uint32_t)i) & DCT3_RAM_MASK];
                    if (ch == 0) { if (++zeros >= 2) break; continue; }
                    zeros = 0; txt[n++] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : '.';
                }
                txt[n] = 0;
                const char* tag = gs_wide ? "w" : " ";
                if (b0 == 0x04 && b1 != 0xFF)
                    printf("[getstr%s] id=%04X caller=%06X -> \"%s\"\n", tag, (b1 << 8) | b2, gs_lr, txt);
                else
                    printf("[getstr%s] ascii  caller=%06X -> \"%s\"\n", tag, gs_lr, txt);
                gs_cap--; gs_ret = 0;
            }
        }
        if (watch && pc == watch && wt_ring) {     // WATCHTAIL: ring of last n hits
            static long wvalr = -1;
            if (wvalr == -1) wvalr = getenv("WATCHVAL") ? (long)strtoul(getenv("WATCHVAL"),NULL,0) : -2;
            if (wvalr == -2 || (uint32_t)cpu->gprs[0] == (uint32_t)wvalr) {
                long i = wt_w % wt_n;
                wt_ring[i].lr = (uint32_t)cpu->gprs[14]; wt_ring[i].r0 = (uint32_t)cpu->gprs[0];
                wt_ring[i].r1 = (uint32_t)cpu->gprs[1]; wt_ring[i].r2 = (uint32_t)cpu->gprs[2];
                wt_ring[i].r5 = (uint32_t)cpu->gprs[5]; wt_ring[i].r6 = (uint32_t)cpu->gprs[6];
                wt_ring[i].step = steps; wt_w++;
            }
        }
        if (watch && pc == watch && !wt_ring && watch_hits < 12) {
            static long wval = -1, wval1 = -3;
            if (wval == -1) wval = getenv("WATCHVAL") ? (long)strtoul(getenv("WATCHVAL"),NULL,0) : -2;
            if (wval1 == -3) wval1 = getenv("WATCHVAL1") ? (long)strtoul(getenv("WATCHVAL1"),NULL,0) : -2;  // r1 filter
            if ((wval  == -2 || (uint32_t)cpu->gprs[0] == (uint32_t)wval) &&
                (wval1 == -2 || (uint32_t)cpu->gprs[1] == (uint32_t)wval1)) {
                printf("[watch] %08X  LR=%08X r0=%08X r1=%08X r2=%08X r5=%08X r6=%08X r7=%08X r9=%08X (step %ld)\n",
                       pc, (uint32_t)cpu->gprs[14], (uint32_t)cpu->gprs[0],
                       (uint32_t)cpu->gprs[1], (uint32_t)cpu->gprs[2],
                       (uint32_t)cpu->gprs[5],
                       (uint32_t)cpu->gprs[6], (uint32_t)cpu->gprs[7],
                       (uint32_t)cpu->gprs[9], steps);
                uint32_t sp = (uint32_t)cpu->gprs[13];
                printf("        stack:");
                for (int k = 0; k < 12; ++k) {
                    uint32_t w = rd32(c, sp + (uint32_t)k*4);
                    if (w >= 0x200000 && w < 0x400000) printf(" %08X", w);   // flash return addrs
                }
                printf("\n");
                watch_hits++;
            }
        }
        if (brokerlog >= 2) {                                        // force broker enable + full subscription bitmap
            c->ram[0x0011FF28u & DCT3_RAM_MASK] = 1;                 //   master enable flag
            memset(&c->ram[0x0011FF4Cu & DCT3_RAM_MASK], 0xFF, 18);  //   subscribe cats 0x09..0x84 ([0x11FF4D..5C]); stays clear of seq [0x11FF6C] + reboot_save [0x11FF88]
        }
        if (brokerlog && brokerlog_n < brokerlog_cap &&
            (pc == 0x002EACAAu ||                                   // BROKER_ROUTER entry (gated leaf events)
             (pc == 0x002EABE8u && (uint32_t)cpu->gprs[14] != 0x002EACC7u))) {  // direct BROKER_DELIVER callers
            uint16_t ev  = (uint16_t)cpu->gprs[1];                  // r1 = event-id at both entries
            uint32_t len = (uint32_t)cpu->gprs[0];                  // r0 = payload byte count
            uint32_t pp  = (uint32_t)cpu->gprs[2];                  // r2 = payload pointer
            const char* nm = trace_name_lookup(ev);
            char pbuf[48]; pbuf[0] = 0;                             // hex of payload (first up to 16 bytes)
            if (len && len <= 0x400u && pp >= IO_HI) {              // RAM payload only
                int show = len < 16u ? (int)len : 16, o = 0;
                for (int i = 0; i < show; i++)
                    o += snprintf(pbuf + o, sizeof(pbuf) - (size_t)o, "%02X", c->ram[(pp + (uint32_t)i) & DCT3_RAM_MASK]);
                if (len > (uint32_t)show) snprintf(pbuf + o, sizeof(pbuf) - (size_t)o, "..");
            }
            printf("[broker] step=%-10ld id=%04X len=%-3u %-7s %-44s%s%s LR=%06X\n",
                   steps, ev, len, (pc == 0x002EACAAu ? "ROUTER" : "DELIVER"),
                   nm ? nm : "(unnamed)", pbuf[0] ? " p=" : "", pbuf, (uint32_t)cpu->gprs[14]);
            brokerlog_n++;
        }
        if (wdoglog && pc == 0x002EDC52u) {   // TASK_4 0xE4-watchdog decision: counter just before verdict
            uint32_t ca = 0x00111862u & DCT3_RAM_MASK;
            int ctr = (c->ram[ca] << 8) | c->ram[(ca + 1) & DCT3_RAM_MASK];
            printf("[wdog] step=%-11ld dt=%-10ld counter=%-4d %s\n",
                   steps, wdog_last ? steps - wdog_last : 0, ctr,
                   ctr == 0 ? "<<< counter==0 -> SWDSP reset 0x68 STAGED" : "healthy (DSP alive, counter->0)");
            wdog_last = steps;
        }
        if (trace & 2u) {   // SIM recognition PC-events (v5.79 PCs; gate offsets off TGATE)
            if (pc == 0x0029EAC0u) {      // disp49 SIM/PIN gate evaluates (dedup identical re-evals)
                uint8_t g = c->ram[TGATE], sb = c->ram[(TGATE + 0x1D) & DCT3_RAM_MASK],
                        sl = c->ram[(TGATE + 0x23) & DCT3_RAM_MASK];
                uint32_t pk = (uint32_t)(g << 16) | (uint32_t)(sb << 8) | sl;
                if (pk != tr_disp49) {    // [11FCFE]/[11FD1B]/[11FD21]
                    printf("[T] SIM.DISP49 gate=%02X sub=%02X sel=%02X @%ld\n", g, sb, sl, steps);
                    tr_disp49 = pk;
                }
            }
            else if (pc == 0x002AF240u)   // recognition retry/timeout fires (-> "SIM rejected")
                printf("[T] SIM.RECOG_TIMEOUT ctr=%02X @%ld\n",
                       c->ram[(TGATE + 0x0A) & DCT3_RAM_MASK], steps);   // [11FD08]
        }
        // Shared harness: the pre-step de-pipe + reboot-entry snapshot + the four
        // model-record hooks (reboot/fatal/assert/stage/heap-fail) + the wild-PC / spin
        // / CCONT-reset / power-off classify all live in src/harness/ now (no CLI-vs-web
        // reimplementation). One call, returning a per-step verdict.
        HarnessStatus hstatus = harness_observe(&hc, &h.mad2, c, steps);
        if (hstatus == HARNESS_FAULT_HALT) {
            // Honest halt: the model already rendered the post-mortem for a caught reset;
            // harness_fault_report covers fault classes the model didn't (wild-PC/spin).
            harness_fault_report(&hc, &h.mad2, c, hc.fault_reason);
            reason = hc.fault_reason ? hc.fault_reason : "harness fault";
            break;
        }
        if (hstatus == HARNESS_POWERED_OFF) {
            reason = hc.fault_reason ? hc.fault_reason : "powered off";
            break;   // graceful run-end, NO fault dump
        }
        // CFTRACE: classify the instruction about to execute (BL/BLX/BX), pre-step.
        uint32_t cft_site = pc; uint32_t cft_mode = cpu->cpsr.priv;
        int cft_kind = 0, cft_size = cpu->cpsr.t ? 2 : 4;   // kind: 0 none 1 BL 2 BLX 3 BX
        int cft_print = cft_on && steps >= cft_after && cft_cap > 0 && (!cft_until || steps <= cft_until);
        if (cft_print || g_callring) {
            uint32_t a = cft_site & DCT3_RAM_MASK;
            if (cpu->cpsr.t) {
                uint16_t op = (uint16_t)((c->ram[a] << 8) | c->ram[(a + 1) & DCT3_RAM_MASK]);
                if ((op & 0xF800u) == 0xF000u) {           // Thumb BL/BLX imm (32-bit)
                    uint16_t op2 = (uint16_t)((c->ram[(a + 2) & DCT3_RAM_MASK] << 8) | c->ram[(a + 3) & DCT3_RAM_MASK]);
                    cft_size = 4;
                    if ((op2 & 0xF800u) == 0xF800u) cft_kind = 1;        // BL
                    else if ((op2 & 0xF800u) == 0xE800u) cft_kind = 2;   // BLX imm
                } else if ((op & 0xFF80u) == 0x4700u) {    // Thumb BX / BLX reg
                    cft_kind = (op & 0x80u) ? 2 : 3;
                }
            } else {
                uint32_t ins = ((uint32_t)c->ram[a] << 24) | ((uint32_t)c->ram[(a + 1) & DCT3_RAM_MASK] << 16)
                             | ((uint32_t)c->ram[(a + 2) & DCT3_RAM_MASK] << 8) | c->ram[(a + 3) & DCT3_RAM_MASK];
                if ((ins & 0x0F000000u) == 0x0B000000u) cft_kind = 1;        // ARM BL
                else if ((ins & 0xFE000000u) == 0xFA000000u) cft_kind = 2;   // ARM BLX imm
                else if ((ins & 0x0FFFFFF0u) == 0x012FFF10u) cft_kind = 3;   // ARM BX
                else if ((ins & 0x0FFFFFF0u) == 0x012FFF30u) cft_kind = 2;   // ARM BLX reg
            }
        }
        dct3_step(c);
        // CFTRACE / CALLLOG: log/record the branch if taken (post-step PC diverged).
        if (cft_kind) {
            uint32_t newpc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
            if (newpc != cft_site + (uint32_t)cft_size) {
                if (cft_print) {
                    static const char* MK[] = { "", "BL ", "BLX", "BX " };
                    printf("[cf] %s %06X->%06X lr=%06lX m=%s @%ld\n",
                           MK[cft_kind], cft_site & 0xFFFFFFu, newpc & 0xFFFFFFu,
                           (unsigned long)(cpu->gprs[14] & 0xFFFFFFu), cf_mode_str(cft_mode), steps);
                    cft_cap--;
                }
                if (g_callring)
                    callring_push(cft_site, newpc, (uint32_t)cpu->gprs[14],
                                  (uint8_t)cft_kind, (uint8_t)cft_mode);
            }
        }
        mad2_timers_tick(&h.mad2, (uint32_t)cpu->cycles);
        // SOFTWDT_KEEPALIVE: clear the supervisor counter at the configured cadence.
        // Done at the harness level (mad2_timers_tick has no RAM pointer). Uses the
        // monotonic cycle accumulator so SLICE rebases don't trip the period check.
        if (softwdt_keepalive && (h.mad2.rtc_mono - softwdt_last_cyc) >= softwdt_period) {
            c->ram[0x0011FF1Au & DCT3_RAM_MASK] = 0;
            softwdt_last_cyc = h.mad2.rtc_mono;
        }
        // Post-step recover apply (shared): mad2_timers_tick already ran above; the
        // harness now applies a pending recover (gated by recovery policy — a no-op when
        // recovery is OFF, the D-10 default) BEFORE the FIQ/IRQ raise, converging on the
        // canonical web ordering. It also resets the shared spin window after a recover.
        harness_post_step(&hc, &h.mad2, c);
        if (trace & 1u) {   // DSP handshake state transitions (sampled post-step/tick)
            uint8_t v = c->ram[h.mad2.fw.verdict & DCT3_RAM_MASK];
            if (v != tr_verdict) { printf("[T] DSP.VERDICT %02X->%02X @%ld\n", tr_verdict, v, steps); tr_verdict = v; }
            uint8_t u = c->ram[h.mad2.fw.dsp_uploaded & DCT3_RAM_MASK];
            if (u != tr_uploaded) {     // upload complete: report the mailbox/code-block totals
                printf("[T] DSP.UPLOADED %02X->%02X (mbox=%llu cblk=%llu) @%ld\n", tr_uploaded, u,
                       (unsigned long long)h.mad2.dsp_acks, (unsigned long long)h.mad2.dsp_cb_acks, steps);
                tr_uploaded = u;
            }
            // The mailbox/code-block handshakes ping hundreds of times during the upload;
            // emit only the first of each (structure, not flood) — totals land on UPLOADED.
            if (h.mad2.dsp_acks   != tr_acks)   { if (tr_acks   == 0) printf("[T] DSP.MBOX_BEGIN @%ld\n", steps);   tr_acks   = h.mad2.dsp_acks; }
            if (h.mad2.dsp_cb_acks != tr_cbacks) { if (tr_cbacks == 0) printf("[T] DSP.CBLOCK_BEGIN @%ld\n", steps); tr_cbacks = h.mad2.dsp_cb_acks; }
            if (h.mad2.dsp_selftest_replied && !tr_selftest) { printf("[T] DSP.SELFTEST_REPLY @%ld\n", steps); tr_selftest = 1; }
            uint32_t tp = h.mad2.fw.mdircv_tail & DCT3_RAM_MASK;          // MDIRCV (DSP->MCU) queue
            uint16_t mt = (uint16_t)((c->ram[tp] << 8) | c->ram[tp + 1]);
            if (mt != tr_mdtail && mt > 0x80) {                          // skip the empty-init (->0x80)
                uint32_t q = h.mad2.fw.mdircv_q & DCT3_RAM_MASK;
                printf("[T] DSP.MDIRCV tail=%04X w0=%02X%02X p0=%02X p1=%02X @%ld\n",
                       mt, c->ram[q], c->ram[q + 1], c->ram[q + 2], c->ram[q + 3], steps);
            }
            if (mt != tr_mdtail) tr_mdtail = mt;
            int i4 = (h.mad2.irq_pending & 0x10) ? 1 : 0;                // DSP block-done IRQ4
            if (i4 && !tr_irq4) printf("[T] DSP.IRQ4 @%ld\n", steps);
            tr_irq4 = i4;
        }
        if (trace & 2u) {   // SIM recognition gate-var transition
            uint8_t g = c->ram[TGATE];
            if (g != tr_gate) { printf("[T] SIM.GATE %02X->%02X @%ld\n", tr_gate, g, steps); tr_gate = g; }
        }
        // DSP->MCU interrupt test (DSPFIQ=<period> DSPBIT=<0|1>): the DSP signals
        // the MCU via FIQ0/FIQ1 (handlers 0x2BAB82/0x2BAB64) to deliver responses.
        // We don't run the DSP, so model a periodic DSP event interrupt.
        if (dsp_fiq_period && (steps % dsp_fiq_period) == 0)
            h.mad2.fiq_pending |= (uint8_t)(1u << dsp_fiq_bit);
        if (irqinj_mask && steps >= irqinj_after && (steps % irqinj_period) == 0)
            h.mad2.irq_pending |= (uint8_t)irqinj_mask;
        int raised = 0;
        if (!cpu->cpsr.f) {                       // FIQs enabled? (higher priority)
            int fiq = mad2_fiq_poll(&h.mad2);
            if (fiq >= 0) {
                if (g_mbus_log_cap > 0) { printf("[mbus] >>> deliver FIQ%d (pending=%02X mask=%02X)\n",
                                                 fiq, h.mad2.fiq_pending, h.mad2.fiq_mask); g_mbus_log_cap--; }
                if (cft_on && steps >= cft_after && cft_cap > 0) {
                    uint32_t ipc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
                    printf("[cf] FIQ%d @%06X m=%s->fiq line=%02X @%ld\n", fiq, ipc & 0xFFFFFFu,
                           cf_mode_str(cpu->cpsr.priv), h.mad2.fiq_pending, steps); cft_cap--;
                }
                if (intfire && steps >= intfire_after && intfire_cap > 0) {
                    uint32_t ipc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
                    printf("[int] FIQ%d @%06X %s->fiq  fact=%02X fmsk=%02X  iact=%02X imsk=%02X  ictrl=%02X f8=%02X @%ld\n",
                           fiq, ipc & 0xFFFFFFu, cf_mode_str(cpu->cpsr.priv),
                           h.mad2.fiq_pending, h.mad2.fiq_mask, h.mad2.irq_pending, h.mad2.irq_mask,
                           h.mad2.int_ctrl, h.mad2.fiq8_ctrl, steps); intfire_cap--;
                }
                ARMRaiseFIQ(cpu); h.mad2.fiqs_raised++; raised = 1;
                telemetry_event_irq(&h.mad2, fiq, 1, steps);   // IRQ/CCONT event log (gated)
                hc.spin_count = 0; hc.spin_anchor = 0xFFFFFFFFu;   // interrupt = fresh block
            }
        }
        if (!raised && !cpu->cpsr.i) {            // IRQs globally enabled?
            int irq = mad2_irq_poll(&h.mad2);
            if (irq >= 0) {
                if (irq2trace && irq == 2 && steps >= irq2trace_after && irq2_seen < 3) {
                    printf("[irq2] >>> IRQ2 raised @step %ld lines=%02X mask=%02X (pc-before=%06X)\n",
                           steps, h.mad2.cc_int_lines, h.mad2.cc_int_mask, pc & 0xFFFFFFu);
                    irq2trace_left = irq2trace; irq2_seen++;
                }
                if (cft_on && steps >= cft_after && cft_cap > 0) {
                    uint32_t ipc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
                    printf("[cf] IRQ%d @%06X m=%s->irq line=%02X @%ld\n", irq, ipc & 0xFFFFFFu,
                           cf_mode_str(cpu->cpsr.priv), h.mad2.irq_pending, steps); cft_cap--;
                }
                if (intfire && steps >= intfire_after && intfire_cap > 0) {
                    uint32_t ipc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
                    printf("[int] IRQ%d @%06X %s->irq  fact=%02X fmsk=%02X  iact=%02X imsk=%02X  ictrl=%02X f8=%02X @%ld\n",
                           irq, ipc & 0xFFFFFFu, cf_mode_str(cpu->cpsr.priv),
                           h.mad2.fiq_pending, h.mad2.fiq_mask, h.mad2.irq_pending, h.mad2.irq_mask,
                           h.mad2.int_ctrl, h.mad2.fiq8_ctrl, steps); intfire_cap--;
                }
                ARMRaiseIRQ(cpu); h.mad2.irqs_raised++;
                telemetry_event_irq(&h.mad2, irq, 0, steps);   // IRQ/CCONT event log (gated)
                hc.spin_count = 0; hc.spin_anchor = 0xFFFFFFFFu;   // interrupt = fresh block
            }
        }
        if (h.mad2.fiqs_raised > 5000000) { reason = "FIQ storm (TxD not re-masked)"; break; }
        if (sweep_enabled && sweep_at && steps + 1 == sweep_at) {
            // Boot has reached SWEEP_AT (post-step bookkeeping done). Fork+inject+collect,
            // then skip the rest of boot_trace's exit/dump path — sweep output is in OUTDIR.
            do_sweep(c, &h, sweep_lo, sweep_hi, sweep_par, sweep_after,
                     sweep_payload_spec, sweep_outdir);
            sweep_done = 1;
            reason = "SWEEP complete";
            break;
        }
    }

    if (sweep_done) {
        dct3_core_destroy(c);
        free(h.rd); free(h.wr); free(h.last);
        return 0;
    }

    // SECCODE=1 (validation knob): recover the security code via the firmware oracle on
    // the now-booted core. Addresses default to 3310 v5.79; override per build via
    // SECCODE_GET / SECCODE_VERIFY (hex). Proves seccode_solve before the web wiring.
    if (getenv("SECCODE")) {
        SeccodeAddrs sa = {
            .get_fn     = getenv("SECCODE_GET")     ? (uint32_t)strtoul(getenv("SECCODE_GET"), 0, 0)     : 0x287476u,
            .decrypt_fn = getenv("SECCODE_DECRYPT") ? (uint32_t)strtoul(getenv("SECCODE_DECRYPT"), 0, 0) : 0x2E7D9Eu,
            .block      = getenv("SECCODE_BLOCK")   ? (uint32_t)strtoul(getenv("SECCODE_BLOCK"), 0, 0)   : 0x00111814u,
            .verify_fn  = getenv("SECCODE_VERIFY")  ? (uint32_t)strtoul(getenv("SECCODE_VERIFY"), 0, 0)  : 0x2E7ED4u,
        };
        SeccodeResult sr;
        seccode_solve(c, &sa, 0, &sr);
        printf("[seccode] plaintext=%s  authoritative=%s%s%s\n",
               sr.have_plaintext ? sr.plaintext : "(none)",
               sr.have_auth ? sr.authoritative : "(unresolved)",
               sr.verified ? " [firmware-verified]" : "",
               sr.diverged ? " [DIVERGED from plaintext]" : "");
    }

    struct ARMCore* cpu = &c->cpu;
    int thumb = cpu->cpsr.t;
    uint32_t pc = (uint32_t)cpu->gprs[15] - (thumb ? 4u : 8u);

    // True total cycles live in mad2.rtc_mono (cpu->cycles is rebased down by
    // 0x20000000 to avoid 32-bit overflow, so it wraps — printing it gave a bogus
    // cyc/insn and emulated-time). Use rtc_mono for cycles/time/cpi, and humanize the
    // big counts so the line is readable.
    uint64_t tcyc = h.mad2.rtc_mono;
    double   tsec = (double)tcyc / 13.0e6;
    char hb1[24], hb2[24];
    printf("\n=== stopped: %s ===\n  steps=%ld (%s)  PC=0x%08X (%s)  mode=0x%02X  stub/ill=%d/%d\n",
           reason, steps, hnum((double)steps, hb1, sizeof hb1),
           pc, thumb ? "Thumb" : "ARM", cpu->privilegeMode, c->stub_count, c->illegal_count);
    printf("  cycles=%llu (%s)  ~%.1f s (%.2f min) emulated @ 13 MHz  (%.2f cyc/insn)\n",
           (unsigned long long)tcyc, hnum((double)tcyc, hb2, sizeof hb2),
           tsec, tsec / 60.0, steps ? (double)tcyc / (double)steps : 0.0);
    printf("  CCONT: %llu rd / %llu wr   LCD: %llu data / %llu cmd\n",
           (unsigned long long)h.mad2.ccont_reads, (unsigned long long)h.mad2.ccont_writes,
           (unsigned long long)h.mad2.lcd_data_writes, (unsigned long long)h.mad2.lcd_cmd_writes);
    printf("  IRQs raised: %llu  FIQs raised: %llu   irq_pending=%02X irq_mask=%02X fiq_mask=%02X  t0_dest=%04X t0_cnt=%04X  DSP acks=%llu\n",
           (unsigned long long)h.mad2.irqs_raised, (unsigned long long)h.mad2.fiqs_raised,
           h.mad2.irq_pending, h.mad2.irq_mask, h.mad2.fiq_mask,
           h.mad2.t0_dest, h.mad2.t0_counter, (unsigned long long)h.mad2.dsp_acks);
    printf("  FIQ5/Timer1 overflows: %llu (t1_hz=%u)   FIQ8/ct_timer ticks: %llu (fiq8_hz=%u)\n",
           (unsigned long long)h.mad2.t1_overflows, h.mad2.t1_hz,
           (unsigned long long)h.mad2.fiq8_ticks, h.mad2.fiq8_hz);
    printf("  FLASH: %llu programs (%llu in EEPROM partition) / %llu erases   last=%06X sub-EEPROM write=%08X %s\n",
           (unsigned long long)h.mad2.flash_programs, (unsigned long long)h.mad2.flash_eeprom_programs,
           (unsigned long long)h.mad2.flash_erases, h.mad2.flash_last_cmd_addr,
           h.mad2.flash_codewrite, h.mad2.flash_codewrite ? "(non-EEPROM region — legit for a custom FFS)" : "(none)");

    if (getenv("IOHIST") || verbose_all) {   // ~2k lines: every I/O address touched — opt-in
        printf("\n=== peripheral histogram ===\n");
        long distinct = 0;
        for (uint32_t a = IO_LO; a < IO_HI; ++a)
            if (h.rd[a] || h.wr[a]) { printf("  %08X : r=%-7u w=%-7u last=%08X\n", a, h.rd[a], h.wr[a], h.last[a]); distinct++; }
        printf("  (%ld distinct)\n", distinct);
    }

    printf("\n=== exception vectors / IRQ state ===\n");
    printf("  cpsr I(rq-masked)=%d F=%d  mode=0x%02X\n",
           (int)((cpu->cpsr.packed >> 7) & 1), (int)((cpu->cpsr.packed >> 6) & 1), cpu->privilegeMode);
    printf("  @0x000000:"); for (int i = 0; i < 8; ++i) printf(" %08X", rd32(c, (uint32_t)i * 4)); printf("\n");
    printf("  @0x100000:"); for (int i = 0; i < 8; ++i) printf(" %08X", rd32(c, 0x100000u + (uint32_t)i * 4)); printf("\n");

    if (edges) {
        long total = edge_w < edge_n ? edge_w : edge_n;
        long start = edge_w < edge_n ? 0 : edge_w;
        printf("\n=== control-flow edges (last %ld) ===\n", total);
        for (long i = 0; i < total; ++i) {
            long idx = (start + i) % edge_n;
            printf("  %08X -> %08X   (lr=%08X)\n", edges[idx].from, edges[idx].to, edges[idx].lr);
        }
    }

    if (wt_ring) {
        long total = wt_w < wt_n ? wt_w : wt_n;
        long start = wt_w < wt_n ? 0 : wt_w;
        printf("\n=== WATCHTAIL %08X (last %ld of %ld hits) ===\n", watch, total, wt_w);
        for (long i = 0; i < total; ++i) {
            long idx = (start + i) % wt_n;
            printf("  step %-10ld LR=%08X r0=%08X r1=%08X r2=%08X r5=%08X r6=%08X\n",
                   wt_ring[idx].step, wt_ring[idx].lr, wt_ring[idx].r0, wt_ring[idx].r1,
                   wt_ring[idx].r2, wt_ring[idx].r5, wt_ring[idx].r6);
        }
    }

    if (getenv("DUMP")) {
        uint32_t da = (uint32_t)strtoul(getenv("DUMP"), NULL, 0);
        printf("\n=== RAM dump @%08X ===\n", da);
        for (int r = 0; r < 8; ++r) {
            printf("  %08X:", da + (uint32_t)r*16);
            for (int b = 0; b < 16; ++b) printf(" %02X", c->ram[(da + (uint32_t)r*16 + (uint32_t)b) & DCT3_RAM_MASK]);
            printf("\n");
        }
    }

    printf("\n=== disasm around stop PC ===\n");
    disasm(c, (pc & ~1u) - 48u, 44, thumb);

    if (h.mad2.lcd_data_writes) {
        printf("\n=== LCD framebuffer (%llu data bytes) ===\n",
               (unsigned long long)h.mad2.lcd_data_writes);
        mad2_render_ascii(&h.mad2);
        if (mad2_save_pgm(&h.mad2, "build/lcd.pgm") == 0) printf("(saved build/lcd.pgm)\n");
    } else {
        printf("\n(no LCD data written yet)\n");
    }

    if (h.mad2.pcm_sink == pcmsink_count)
        printf("\n[pcmsink] ch1=%ld samples (peak %d) ch0=%ld (rate %.0f Hz)\n",
               g_pcmsink_n[1], g_pcmsink_pk, g_pcmsink_n[0],
               h.mad2.pcm_rate ? h.mad2.pcm_rate : 0.0);

    // hexdump a RAM region at exit (16 bytes/line). For task
    // census (TCB table 0x106130, pending counts 0x104884).
    // NOTE: RAMDUMP is ALSO the snapshot-path knob (line ~887). Only treat it as an
    // addr-hexdump when it parses as a number — a path like "/tmp/w" parses nothing
    // (endptr unmoved) and must NOT trigger this (else it spams "0x000000..0x000040").
    if (getenv("RAMDUMP")) {
        const char* p = getenv("RAMDUMP");
        const char* start = p;
        uint32_t da = (uint32_t)strtoul(p, (char**)&p, 0);
        uint32_t dl = (*p == ',') ? (uint32_t)strtoul(p + 1, NULL, 0) : 64;
        if (p == start) goto skip_ramdump_hexdump;  // RAMDUMP=<path>, not an address
        printf("\n=== RAMDUMP 0x%06X..0x%06X ===\n", da, da + dl);
        for (uint32_t i = 0; i < dl; i += 16) {
            printf("  %06X:", da + i);
            for (uint32_t j = 0; j < 16 && i + j < dl; ++j)
                printf(" %02X", c->ram[(da + i + j) & DCT3_RAM_MASK]);
            printf("\n");
        }
        skip_ramdump_hexdump: ;
    }

    // dump fall-through MMIO access table.
    mad2_mmio_audit_dump(&h.mad2);

#ifdef DCT3_SDL
    // Keep the window alive after the run ends so the crash/halt LCD + reason stay
    // visible. Pass the halt `reason` boot_trace already computed; close to exit. A
    // 30s PWR hold here (gi.reboot) hard-resets a powered-off / wild-PC'd phone.
    if (gui_on) {
        GuiInput gi = {0, 0, 0};
        while (!gi.quit && !gi.reboot) gi = gui_frame(&h.mad2, &c->cpu, (long long)steps, reason);
        if (gi.reboot) goto gui_warm_reboot;
        gui_shutdown();
    }
#endif

    // EE5110SAVE=<file>: persist the serial-bus external 24C16 EEPROM (5110/6110/3210 have no in-flash
    // EEPROM partition) — captures any runtime write (e.g. a SETFAID bake) for re-load via EE5110=.
    if (getenv("EE5110SAVE")) ext_eeprom_save(&h.mad2, getenv("EE5110SAVE"));
    if (mbus_fd >= 0) mbus_bridge_close(mbus_fd);
    dct3_core_destroy(c);
    free(h.rd); free(h.wr); free(h.last);
    free(replay_evts);
    return 0;

#ifdef DCT3_SDL
gui_warm_reboot:
    // Our own (GUI/harness) warm reboot — distinct from the firmware's power-off.
    // Reset the CPU (dct3_boot_at) + mad2 device state (mad2_init re-asserts the boot
    // power-hold at kbd_special_cols) + the fault harness; FLASH/EEPROM in c->ram
    // persist, exactly like a real hardware reset. Then re-enter the run loop.
    printf("\n=== GUI: warm reboot (PWR / R) — core+mad2 reset, flash persists ===\n");
    fflush(stdout);
    // A real hardware reset clears VOLATILE RAM (everything below flash, incl. the DSP
    // shared mailbox at 0x10000) while flash/EEPROM persist. Without this the firmware
    // re-boots on a stale mailbox (head != 0) and the DSP code-block boot-ack handshake
    // never engages -> spin at MDIRCV_HEAD_INIT. calloc gave the cold boot zeroed RAM,
    // so match it: zero [0, flash_base). (ARMReset already zeroes cpu->cycles.)
    memset(c->ram, 0, prof->mem.flash_base);
    mad2_init(&h.mad2, prof);
    // mad2_init memset the whole struct, so REBIND everything the initial boot set up
    // after it (lines ~498-517) — most critically m->mem (the core RAM pointer): without
    // it the next guest access NULL-derefs. Mirror the original env overrides + resolve.
    if (getenv("CCONT0E"))    h.mad2.ccont[0x0E]   = (uint8_t)strtoul(getenv("CCONT0E"), NULL, 0);
    if (getenv("VBATT"))      h.mad2.adc[2]        = (uint16_t)strtoul(getenv("VBATT"), NULL, 0);
    if (getenv("BSI"))        h.mad2.adc[3]        = (uint16_t)strtoul(getenv("BSI"), NULL, 0);
    if (getenv("BTEMP"))      h.mad2.adc[4]        = (uint16_t)strtoul(getenv("BTEMP"), NULL, 0);
    if (getenv("CHARGER"))    h.mad2.adc[5]        = (uint16_t)strtoul(getenv("CHARGER"), NULL, 0);
    if (getenv("KEYSPECIAL")) h.mad2.kbd_special_cols = (uint8_t)strtoul(getenv("KEYSPECIAL"), NULL, 0);
    h.mad2.verbose  = getenv("CCVERBOSE") ? (int)atol(getenv("CCVERBOSE")) : (getenv("VERBOSE") ? 60 : 0);
    h.mad2.mem      = c->ram;                  // <- the fix: rebind core RAM (memset nulled it)
    h.mad2.mem_mask = DCT3_RAM_MASK;
    model_resolve(prof, c->ram, DCT3_RAM_MASK, &h.mad2.fw);
    dct3_boot_at(c, BOOT_ENTRY);
    harness_init(&hc, &h.mad2, recovery_default, SPIN_LIMIT, wild_after);
    gui_reset();
    steps  = 0;
    reason = "budget reached";
    goto gui_run_start;
#endif
}
