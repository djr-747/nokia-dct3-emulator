#!/bin/sh
# dct3-emu — launch the Nokia 3310 (DCT3) emulator GUI.
#
# Firmware is NOT bundled (it's copyrighted Nokia code). Supply your own 3310
# .fls dump, either as the first argument or dropped into the data directory:
#     dct3-emu /path/to/3310.fls
#     # ...or place a *.fls in ~/.local/share/dct3-emu/ and just run:
#     dct3-emu
set -eu

DATADIR="${XDG_DATA_HOME:-$HOME/.local/share}/dct3-emu"
BIN="/usr/lib/dct3-emu/dct3-emu"
STEPS="${DCT3_STEPS:-400000000}"     # ~wall-clock run budget; GUI is realtime-paced

FW="${1:-}"
if [ -n "$FW" ]; then shift; fi
if [ -z "$FW" ]; then
    # No path given: pick the first .fls in the data dir, if any.
    FW="$(ls "$DATADIR"/*.fls 2>/dev/null | head -n1 || true)"
fi

if [ -z "$FW" ] || [ ! -f "$FW" ]; then
    msg="No Nokia 3310 firmware found.

Supply a .fls dump one of two ways:
  1. dct3-emu /path/to/your-3310.fls
  2. put a *.fls in:  $DATADIR/   then run: dct3-emu

(Firmware is not included — it is copyrighted Nokia code.)"
    printf '%s\n' "$msg" >&2
    if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v zenity >/dev/null 2>&1; then
        zenity --error --title="Nokia 3310 Emulator" --width=380 --text="$msg" 2>/dev/null || true
    fi
    exit 1
fi

# GUI on, harness recovery on (auto-recover firmware self-resets). All other
# debug/co-sim knobs are compiled out of this build.
exec env GUI=1 RESET_RECOVER=1 "$BIN" "$FW" "$STEPS" "$@"
