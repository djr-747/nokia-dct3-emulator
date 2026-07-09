/* dbgcon_stripcheck — compiled with -DDBGCON_ENABLE=0 (production strip). Proves that even
 * when pointed at a LIVE emulator Mad2, a stripped build touches the port ZERO times: the
 * calls (and their strings) vanish at compile time. Linked into the selftest; the assertion
 * lives there. Kept in its own TU because DBGCON_ENABLE is a per-translation-unit choice. */
#include "mad2/mad2.h"

#define DBGCON_ENABLE 0
#include "dbgcon/dbgcon.h"

/* Run the same client calls a debug build would; return how many port writes resulted.
 * With the strip active this must be 0 regardless of whether m is the emulator or not. */
unsigned long dbgcon_stripcheck(struct Mad2 *m) {
    unsigned long before = m->dbgcon_writes;
    dbgcon_puts("this string must not survive the strip");
    dbgcon_nl();
    dbgcon_kv("verdict=", 0x0011FF15u);
    dbgcon_hexport(0xCAFEBABEu);
    dbgcon_flush();
    return m->dbgcon_writes - before;
}
