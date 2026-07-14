#!/usr/bin/env python3
"""Graft an FFS 'EEPROM' config block from a donor DCT3 flash image into a target image.

Why: some library .fls images ship with an EMPTY (all-0xFF) EEPROM partition — e.g. the
3350 NHM-9 v5.22 set. On first boot the firmware self-formats a PMM FFS block, but the
factory CONFIG record (the store whose 16-bit checksum the startup fault-status builder
compares — 3350: computed via 0x253E40 vs the 2-byte record read at 0x25409C — the
verdict-bit6 gate) is factory-provisioned, never self-created, so every boot fails the
config-checksum gate and latches CONTACT SERVICE.

Fix (the classic donor-graft, cf. the "3310 v5.57 bounce (v5.79 EEPROM graft)" image):
copy the 64 KB FFS 'EEPROM' block from a sibling image that ships one (e.g. 3330 NHM-6
v4.50, byte offset 0x3F0000 = flash 0x5F0000) into the same flash address of the target.
The NHM family shares the FFS + config-record scheme, so the donor store passes the
checksum gate and the firmware then adopts/rewrites it as its own. Verified 2026-07-14:
3350 v5.22 + 3330 v4.50 donor boots to "Security code" -> factory 12345 accepted ->
"Code accepted" -> first-boot Time wizard.

Usage:
    tools/graft_eeprom_block.py <target.fls> <donor.fls> [-o out.fls]

The FFS 'EEPROM' block is located in the donor by its header: F0 F0 FF F8 <seq:2> "EEPROM".
It is written at the SAME flash address in the target (both must be same-size DCT3 images
with the EEPROM partition in the top of flash). Output defaults to
"<target stem> (EEPROM graft).fls". No firmware ships with this repo — bring your own.
"""
import argparse
import os
import re
import sys

BLOCK = 0x10000                      # DCT3 flash erase-block / FFS block size (64 KB)
HDR = re.compile(rb"\xF0\xF0\xFF\xF8..EEPROM\x00", re.S)


def find_eeprom_block(img: bytes, name: str) -> int:
    hits = [m.start() for m in HDR.finditer(img)]
    hits = [h for h in hits if h % BLOCK == 0]   # FFS blocks are block-aligned
    if not hits:
        sys.exit(f"error: no FFS 'EEPROM' block header in {name}")
    if len(hits) > 1:
        print(f"note: {len(hits)} 'EEPROM' blocks in {name}; using the last (top of flash)")
    return hits[-1]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("target")
    ap.add_argument("donor")
    ap.add_argument("-o", "--out")
    args = ap.parse_args()

    tgt = bytearray(open(args.target, "rb").read())
    don = open(args.donor, "rb").read()
    if len(tgt) != len(don):
        print(f"warn: image sizes differ (target {len(tgt):#x}, donor {len(don):#x}); "
              "grafting at the donor's block offset anyway")

    off = find_eeprom_block(don, args.donor)
    if off + BLOCK > len(tgt):
        sys.exit("error: donor block offset lies outside the target image")

    old = tgt[off:off + BLOCK]
    if any(b != 0xFF for b in old):
        print(f"note: target block @0x{off:X} (flash 0x{off + 0x200000:X}) is not blank — replacing it")
    tgt[off:off + BLOCK] = don[off:off + BLOCK]

    out = args.out or f"{os.path.splitext(args.target)[0]} (EEPROM graft).fls"
    open(out, "wb").write(tgt)
    print(f"grafted 'EEPROM' block @ flash 0x{off + 0x200000:X} from {os.path.basename(args.donor)}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
