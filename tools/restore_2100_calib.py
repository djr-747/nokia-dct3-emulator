#!/usr/bin/env python3
# restore_2100_calib.py — repair a Nokia 2100 (NAM-2) flash/EEPROM dump whose CALIBRATION
# block was wiped, so it boots past CONTACT SERVICE again.
#
# WHY THIS EXISTS
# ---------------
# The 2100's CONTACT SERVICE screen is the DSP self-test verdict gate (verdict fn 0x256720,
# byte [0x13FDB3]). The verdict aggregates 24 self-test items; three of them read the phone's
# CALIBRATION records, which live in the FIXED upper 16 KB of the EEPROM partition
# (device 0x3FC000-0x3FFFFF — the firmware reads these by absolute address, NOT via the
# relocatable FFS that holds settings/contacts/the security code):
#   - item 18 -> calib record 0x3FC026 (0x120 B, trailing 16-bit additive-sum checksum at +0x11C,
#                fw checker 0x2F113C / predicate 0x291E64). This is the record provision_2100_
#                eeprom.py recalculates when grafting the v5.21 EEPROM onto the v5.84 MCU+PPM.
#   - items 4 & 12 -> other calibration records in the same 0x3FC000 block.
# If ANY of these reads a fail-code, the verdict clears bit6 -> CONTACT SERVICE.
#
# Running the firmware's "Restore factory settings" on the grafted image (or any operation that
# erases the calibration block to 0xFF) makes items 4/12/18 fail -> the saved .fls then boots to
# CONTACT SERVICE even though the rest of the FFS (settings, the security code) is fine. A real
# factory reset does NOT touch calibration; ours does because the grafted v5.21 FFS confuses the
# v5.84 reset. This tool puts the calibration block back.
#
# WHAT IT DOES
# -----------
# Detects a wiped/inconsistent calibration block (item-18 checksum mismatch or an all-0xFF record)
# and restores the whole fixed calibration block (0x3FC000-0x400000) from a known-good reference —
# by default the provisioned image that provision_2100_eeprom.py produces. The FFS BELOW 0x3FC000
# (including the security code, FFS record 0x701) is left untouched, so a changed security code or
# other user NVRAM survives the repair.
#
#   tools/restore_2100_calib.py "<broken>.fls"                 # -> <broken>_calib-restored.fls
#   tools/restore_2100_calib.py "<broken>.fls" -o fixed.fls
#   tools/restore_2100_calib.py "<broken>.fls" --check         # report only, write nothing
#   tools/restore_2100_calib.py "<broken>.fls" --ref other.fls # use a different reference dump
#
# Reference (gitignored) defaults, first that exists:
#   firmware/Nokia 2100 NAM-2 v5.84 A (EEPROM).fls   (full 2 MB provisioned image)
#   firmware/Nokia 2100 NAM-2 EEPROM.bin             (raw 64 KB provisioned partition)

import argparse
import os
import sys

FLASH_BASE = 0x200000
EE_LO      = 0x3F0000        # EEPROM partition start (FFS lives here)
CALIB_LO   = 0x3FC000        # fixed calibration block (self-test items 4/12/18 read here)
CALIB_HI   = 0x400000        # top of the 2 MB image
CALIB_REC  = 0x3FC026        # item-18 record
CALIB_LEN  = 0x11C           # bytes summed; checksum is the big-endian u32 at +0x11C

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF_DEFAULTS = [
    os.path.join(ROOT, "firmware", "Nokia 2100 NAM-2 v5.84 A (EEPROM).fls"),
    os.path.join(ROOT, "firmware", "Nokia 2100 NAM-2 EEPROM.bin"),
]


def calib_status(buf):
    """Return (ok, computed, stored) for the item-18 checksum record in a 2 MB image buffer."""
    ro = CALIB_REC - FLASH_BASE
    computed = sum(buf[ro:ro + CALIB_LEN]) & 0xFFFF
    stored = int.from_bytes(buf[ro + CALIB_LEN:ro + CALIB_LEN + 4], "big") & 0xFFFF
    return (computed == stored, computed, stored)


def ref_slice(ref_path):
    """Read the calibration block (0x3FC000-0x400000) out of a reference .fls or .bin."""
    ref = open(ref_path, "rb").read()
    if len(ref) >= CALIB_HI - FLASH_BASE:          # full 2 MB image
        base = FLASH_BASE
    elif len(ref) == CALIB_HI - EE_LO:             # 64 KB raw EEPROM partition
        base = EE_LO
    else:
        sys.exit("reference %s is %d bytes — not a 2 MB .fls or 64 KB EEPROM partition"
                 % (ref_path, len(ref)))
    return ref[CALIB_LO - base:CALIB_HI - base]


def main():
    ap = argparse.ArgumentParser(description="restore a wiped 2100 calibration block")
    ap.add_argument("image", help="2100 .fls dump to repair (2 MB)")
    ap.add_argument("-o", "--out", help="output path (default <image>_calib-restored.fls)")
    ap.add_argument("--ref", help="known-good reference .fls / EEPROM .bin to copy calib from")
    ap.add_argument("--check", action="store_true", help="report calib status only; write nothing")
    ap.add_argument("--force", action="store_true", help="restore even if the calib looks healthy")
    args = ap.parse_args()

    img = bytearray(open(args.image, "rb").read())
    if len(img) < CALIB_HI - FLASH_BASE:
        sys.exit("%s is %d bytes — expected a 2 MB 2100 image" % (args.image, len(img)))

    ok, comp, stored = calib_status(img)
    ro = CALIB_REC - FLASH_BASE
    blank = all(b == 0xFF for b in img[ro:ro + CALIB_LEN])
    print("calib record 0x%06X: computed=0x%04X stored=0x%04X  %s%s"
          % (CALIB_REC, comp, stored, "OK" if ok else "*** MISMATCH ***",
             "  (record erased to 0xFF)" if blank else ""))

    if args.check:
        return 0 if ok else 1
    if ok and not args.force:
        print("calib block is consistent — nothing to do (use --force to restore anyway)")
        return 0

    ref_path = args.ref
    if not ref_path:
        ref_path = next((p for p in REF_DEFAULTS if os.path.exists(p)), None)
    if not ref_path or not os.path.exists(ref_path):
        sys.exit("no reference image found; pass --ref <provisioned 2100 .fls or EEPROM .bin>\n"
                 "  (regenerate one with tools/provision_2100_eeprom.py)")

    img[CALIB_LO - FLASH_BASE:CALIB_HI - FLASH_BASE] = ref_slice(ref_path)
    ok2, comp2, stored2 = calib_status(img)
    print("restored calib block 0x%06X-0x%06X from %s" % (CALIB_LO, CALIB_HI, os.path.basename(ref_path)))
    print("  now: computed=0x%04X stored=0x%04X  %s" % (comp2, stored2, "OK" if ok2 else "STILL BAD"))
    if not ok2:
        sys.exit("reference calib is itself inconsistent — wrong reference image?")

    out = args.out or (os.path.splitext(args.image)[0] + "_calib-restored.fls")
    open(out, "wb").write(img)
    print("wrote %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
