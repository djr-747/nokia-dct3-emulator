/* Spike stub for calypso_debug.h — cdbg_env maps to getenv so the core's
 * env-gated debug probes still work; cdbg() logging is a no-op. */
#ifndef HW_ARM_CALYPSO_DEBUG_H
#define HW_ARM_CALYPSO_DEBUG_H

#include <stdlib.h>
const char *cdbg_env(const char *name);
int calypso_debug_enabled(const char *name);

/* C54_LOG is defined inside calypso_c54x.c; C54_DBG is not — stub it out. */
#define C54_DBG(...) ((void)0)

#endif
