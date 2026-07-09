/* dbgcon.h — freestanding client for the DCT3 emulator debug console (docs/dbgcon.md).
 *
 * A patch author (NokiX overlay, injected test code, hand-assembled stub) includes this
 * ONE header and calls dbgcon_puts()/dbgcon_kv()/... to print on the HOST console while the
 * firmware runs under the emulator. On REAL hardware the presence probe fails and every call
 * is a no-op — nothing to strip for a production build, no serial bridge, no PC receiver.
 *
 * DEPENDENCY-FREE: no libc, no system headers, no allocations, no statics that need a CRT.
 * Access goes through the DBGCON_RD32/WR32/WR8 macros, which default to direct volatile MMIO
 * (correct on the DCT3 ARM target). A host harness overrides them to route at a device model
 * so the SAME source can be exercised off-target — see tools/dbgcon/dbgcon_selftest.c.
 *
 * The port contract mirrors src/mad2/mad2.h (DCT3_DBGCON_*) exactly.
 *
 * TWO-LAYER GATING:
 *   compile-time  DBGCON_ENABLE (default 1). Build a PRODUCTION / real-hardware flash with
 *                 -DDBGCON_ENABLE=0 and every call below expands to nothing — the debug
 *                 STRINGS become unreferenced and drop out of the image, zero bytes, zero
 *                 cycles. (Caveat, like assert(): args are NOT evaluated when stripped, so
 *                 never put a needed side effect inside a dbgcon_*() call.)
 *   run-time      When DBGCON_ENABLE=1, calls are still gated by dbgcon_present() so a debug
 *                 build stays silent if it is ever run on real hardware. Belt and braces. */
#ifndef DBGCON_H
#define DBGCON_H

#ifndef DBGCON_ENABLE
#define DBGCON_ENABLE 1
#endif

/* 32-bit unsigned on the DCT3 ARM; overridable for a host build with a different int width. */
#ifndef DBGCON_U32
typedef unsigned int dbgcon_u32;
#else
typedef DBGCON_U32   dbgcon_u32;
#endif

#define DBGCON_CHAR   0x000DEAD0u   /* byte buffer; '\n' / 127 chars / FLUSH prints the line */
#define DBGCON_HEX    0x000DEAD4u   /* immediate: "[dbgcon] pc=.. val=.. (dec)"              */
#define DBGCON_FLUSH  0x000DEAD8u   /* flush a partial CHAR line                             */
#define DBGCON_MAGIC  0xDEADBEEFu   /* read DBGCON_CHAR == this  <=>  running under the emu  */

#if DBGCON_ENABLE

#ifndef DBGCON_RD32
#define DBGCON_RD32(a)    (*(volatile dbgcon_u32*)(a))
#endif
#ifndef DBGCON_WR32
#define DBGCON_WR32(a,v)  (*(volatile dbgcon_u32*)(a) = (dbgcon_u32)(v))
#endif
#ifndef DBGCON_WR8
#define DBGCON_WR8(a,v)   (*(volatile unsigned char*)(a) = (unsigned char)(v))
#endif

/* Presence, memoized: -1 unknown, 0 real hardware, 1 emulator. One MMIO read, then cached. */
static int dbgcon__present = -1;

static int dbgcon_present(void) {
    if (dbgcon__present < 0)
        dbgcon__present = (DBGCON_RD32(DBGCON_CHAR) == DBGCON_MAGIC) ? 1 : 0;
    return dbgcon__present;
}
/* Re-probe (after a warm reboot, or when a harness swaps the backing device). */
static void dbgcon_reset(void) { dbgcon__present = -1; }

/* --- CHAR line ---------------------------------------------------------------------- */
static void dbgcon_putc(char c) {
    if (dbgcon_present()) DBGCON_WR8(DBGCON_CHAR, (unsigned char)c);
}
static void dbgcon_puts(const char *s) {
    if (!dbgcon_present()) return;
    while (*s) DBGCON_WR8(DBGCON_CHAR, (unsigned char)*s++);
}
static void dbgcon_nl(void)    { dbgcon_putc('\n'); }                  /* flushes the line */
static void dbgcon_flush(void) { if (dbgcon_present()) DBGCON_WR32(DBGCON_FLUSH, 0u); }

/* Append "0x" + 8 upper-case hex digits into the CHAR line (coherent, single-line output). */
static void dbgcon_puthex32(dbgcon_u32 v) {
    static const char H[16] = { '0','1','2','3','4','5','6','7',
                                '8','9','A','B','C','D','E','F' };
    int i;                                    /* C89: declaration before statements */
    if (!dbgcon_present()) return;
    DBGCON_WR8(DBGCON_CHAR, '0'); DBGCON_WR8(DBGCON_CHAR, 'x');
    for (i = 28; i >= 0; i -= 4) DBGCON_WR8(DBGCON_CHAR, H[(v >> i) & 0xFu]);
}
/* "<key><hex>\n" as one line, e.g. dbgcon_kv("verdict=", x) -> "[dbgcon] verdict=0x0011FF15". */
static void dbgcon_kv(const char *key, dbgcon_u32 v) {
    if (!dbgcon_present()) return;
    dbgcon_puts(key); dbgcon_puthex32(v); dbgcon_nl();
}

/* --- HEX port (immediate, one-shot; the emulator prepends the writing PC) ------------ */
static void dbgcon_hexport(dbgcon_u32 v) {
    if (dbgcon_present()) DBGCON_WR32(DBGCON_HEX, v);
}

#else  /* !DBGCON_ENABLE — production strip: every call vanishes, strings included ------ */

#define dbgcon_present()      (0)
#define dbgcon_reset()        ((void)0)
#define dbgcon_putc(c)        ((void)0)
#define dbgcon_puts(s)        ((void)0)
#define dbgcon_nl()           ((void)0)
#define dbgcon_flush()        ((void)0)
#define dbgcon_puthex32(v)    ((void)0)
#define dbgcon_kv(k, v)       ((void)0)
#define dbgcon_hexport(v)     ((void)0)

#endif /* DBGCON_ENABLE */

#endif /* DBGCON_H */
