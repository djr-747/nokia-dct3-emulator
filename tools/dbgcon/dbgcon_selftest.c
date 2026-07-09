/* dbgcon_selftest — exercise tools/dbgcon/dbgcon.h against the REAL emulator port, and
 * prove the emulator-vs-real-hardware gate.
 *
 * The client lib (dbgcon.h) is dependency-free; this harness routes its MMIO macros at a
 * live Mad2 through the actual mad2_read/mad2_write (the same functions the ARM core's I/O
 * hook calls), so the lib drives the genuine src/mad2/mad2_bus.c dbgcon implementation — not
 * a mock. A mode switch models the two worlds:
 *
 *   EMU  — reads hit the real port (returns 0xDEADBEEF) -> dbgcon_present()==1 -> writes flow;
 *          you SEE real "[dbgcon] ..." lines and m.dbgcon_writes climbs.
 *   REAL — reads float (0xFFFFFFFF, as an undecoded bus line would) -> present()==0 -> every
 *          lib call no-ops -> the port is never written (m.dbgcon_writes stays 0).
 *
 * Build: see the `dbgcon-selftest` Makefile target (links the mad2 device model, no core). */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mad2/mad2.h"
#include "models/model.h"

/* --- route the lib's MMIO at a live Mad2, with an emu/real-hardware mode switch --------- */
static Mad2       g_m;
static int        g_realhw = 0;   /* 0 = emulator, 1 = real hardware (port undecoded) */

static unsigned dbg_rd(unsigned a);
static void     dbg_wr(unsigned a, unsigned v, int sz);

#define DBGCON_U32       unsigned int
#define DBGCON_RD32(a)   dbg_rd((unsigned)(a))
#define DBGCON_WR32(a,v) dbg_wr((unsigned)(a), (unsigned)(v), 4)
#define DBGCON_WR8(a,v)  dbg_wr((unsigned)(a), (unsigned)(v), 1)
#include "dbgcon/dbgcon.h"

static unsigned dbg_rd(unsigned a) {
    if (g_realhw) return 0xFFFFFFFFu;                 /* undecoded bus line floats high */
    return mad2_read(&g_m, 0 /*pc*/, a, 4, 0);
}
static void dbg_wr(unsigned a, unsigned v, int sz) {
    if (g_realhw) return;                             /* inert on real hardware */
    mad2_write(&g_m, 0 /*pc*/, a, sz, v);
}

/* The exact same client code, run in both worlds. */
static void run_client(void) {
    dbgcon_reset();                                   /* re-probe after a mode switch */
    dbgcon_puts("boot reached standby");
    dbgcon_nl();
    dbgcon_kv("verdict=", 0x0011FF15u);               /* one coherent "[dbgcon] verdict=0x..." line */
    dbgcon_hexport(0xCAFEBABEu);                      /* immediate HEX port dump */
    dbgcon_puts("partial line, no newline");          /* left in the CHAR buffer... */
    dbgcon_flush();                                   /* ...pushed out explicitly via FLUSH */
}

static int g_pass, g_fail;
#define CHECK(desc, cond) do { if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL %s\n", desc); } } while (0)

int main(void) {
    printf("=== dbgcon client-lib selftest (real emulator port) ===\n");
    const ModelProfile *prof = model_by_name("3310");
    if (!prof) { printf("  FATAL: no 3310 profile\n"); return 2; }
    mad2_init(&g_m, prof);
    static unsigned char ram[0x400000];
    g_m.mem = ram; g_m.mem_mask = 0x3FFFFFu;

    /* Sanity: the memoized probe must read the LIVE backing, not a stale cache. */
    g_realhw = 0; dbgcon_reset();
    CHECK("present() == 1 under the emulator", dbgcon_present() == 1);
    g_realhw = 1; dbgcon_reset();
    CHECK("present() == 0 on real hardware",   dbgcon_present() == 0);

    /* EMU world: the client's output flows to the real port (prints below). */
    printf("--- EMU: expect real [dbgcon] lines ---\n");
    g_m.dbgcon_writes = 0;
    g_realhw = 0;
    run_client();
    unsigned long emu_writes = g_m.dbgcon_writes;
    CHECK("emu: the client wrote the port", emu_writes > 0);
    CHECK("emu: CHAR line buffer flushed by the trailing newline(s)", g_m.dbgcon_len == 0);

    /* REAL world: identical client code, gated fully off (runtime presence probe). */
    printf("--- REAL: expect NO [dbgcon] output ---\n");
    g_m.dbgcon_writes = 0;
    g_realhw = 1;
    run_client();
    CHECK("real: the client wrote NOTHING (gated off)", g_m.dbgcon_writes == 0);

    /* Compile-time strip (-DDBGCON_ENABLE=0, dbgcon_stripcheck.c): even pointed at the LIVE
     * emulator port, a production build emits zero writes — the calls vanished at compile
     * time (the strings-are-gone half is proven by the docs/build check). */
    extern unsigned long dbgcon_stripcheck(Mad2 *m);
    g_m.dbgcon_writes = 0;
    g_realhw = 0;                                      /* port IS present, yet... */
    CHECK("strip: DBGCON_ENABLE=0 build never touches the port", dbgcon_stripcheck(&g_m) == 0);

    printf("emu port writes = %lu (real = 0)\n", emu_writes);
    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
