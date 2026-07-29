#!/usr/bin/env python3
"""Count and locate Thumb BL call sites targeting given addresses in a DCT3 flash image.

Complements build/disfw: disfw prints the callers of one function as part of its header,
this answers "how many callers does each of these N addresses have" in one pass, which is
how you tell a real API (hundreds of callers) from a shared epilogue, a dead wrapper, or a
function only ever reached through a pointer.

    python3 tools/symbols/blscan.py <flash.fls> 0xVA [0xVA ...]

Two traps this encodes, both of which have cost real time on this firmware:

  * The image stores instruction halfwords BIG-ENDIAN, so a BL pair must be read as
    two big-endian halfwords, not as a little-endian word.
  * The flash maps at virtual base 0x200000 (file offset = VA - BASE). Forget it and
    every computed target lands 2 MB low, so every function reports zero callers —
    which reads exactly like "this code is dead" and is not.

A genuine zero here means no *direct* BL reaches the address. The function may still be
live via a pointer table; check with a big-endian search for the address and address|1
before concluding anything is dead.
"""
import sys

BASE = 0x200000


def bl_targets(data, base=BASE):
    """Yield (file_offset, target_va) for every Thumb BL in the image."""
    for off in range(0, (len(data) & ~1) - 2, 2):
        hi = (data[off] << 8) | data[off + 1]
        if (hi & 0xF800) != 0xF000:          # BL prefix (high halfword)
            continue
        lo = (data[off + 2] << 8) | data[off + 3]
        if (lo & 0xF800) != 0xF800:          # BL suffix (low halfword)
            continue
        disp = ((hi & 0x7FF) << 12) | ((lo & 0x7FF) << 1)
        if disp & 0x400000:                  # sign-extend the 23-bit displacement
            disp -= 0x800000
        yield off, (off + base + 4 + disp) & 0xFFFFFFFF


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    data = open(sys.argv[1], 'rb').read()
    targets = [int(a, 16) for a in sys.argv[2:]]
    hits = {t: [] for t in targets}
    for off, target in bl_targets(data):
        if target in hits:
            hits[target].append(off + BASE)
    for t in targets:
        print(f"0x{t:06X}: {len(hits[t])} bl callers")
        for va in hits[t]:
            print(f"    bl @ 0x{va:06X}")


if __name__ == '__main__':
    main()
