// Minimal debugger-symbol provider.
//
// The vendored disassembler (decoder.c, compiled with -DENABLE_DEBUGGERS) calls
// mDebuggerSymbolReverseLookup to annotate addresses with symbol names. We don't
// load a symbol table, so this stub returns NULL and the disassembler prints raw
// addresses — enough for bring-up tracing without dragging in mGBA's full
// debugger subsystem.

#include <mgba/internal/debugger/symbols.h>

const char* mDebuggerSymbolReverseLookup(const struct mDebuggerSymbols* syms, int32_t value, int segment) {
    (void)syms; (void)value; (void)segment;
    return NULL;
}
