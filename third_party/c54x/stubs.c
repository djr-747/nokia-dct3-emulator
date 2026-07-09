/* Spike stubs: the few QEMU/Calypso externals calypso_c54x.c links against. */
#include "hw/arm/calypso/calypso_full_pcb.h"
#include <stdlib.h>
#include <stdint.h>

QemuMutex calypso_pcb_daram_lock;
QemuMutex calypso_pcb_api_ram_lock;

/* === Memoized env lookups (2026-06-09 perf) ===
 * calypso_debug_enabled() / cdbg_env() were a raw getenv() per call. The C54x
 * core calls them from inside its per-instruction loop (PCEQ, PCWIN, SNAPSHOT,
 * DECODE-AUDIT, AR/A-traces, ...) — several env scans PER INSTRUCTION, over
 * billions of boot instructions = a dominant, pure-overhead cost. The
 * environment never changes during a run, so memoize. Direct-mapped cache keyed
 * by the (stable, string-literal) name POINTER; collisions just recompute, so
 * correctness is independent of the keying. Falls back to getenv on a miss. */
struct dbg_cache_ent { const char *n; const char *v; int known; };

const char *cdbg_env(const char *name)
{
    if (!name) return 0;
    static struct dbg_cache_ent cache[1024];
    unsigned i = (unsigned)(((uintptr_t)name >> 4) & 1023u);
    if (cache[i].known && cache[i].n == name) return cache[i].v;
    const char *v = getenv(name);
    cache[i].n = name; cache[i].v = v; cache[i].known = 1;
    return v;
}

int calypso_debug_enabled(const char *name)
{
    return cdbg_env(name) != 0;
}
