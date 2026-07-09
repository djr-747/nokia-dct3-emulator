#!/usr/bin/env python3
# provision_2100_eeprom.py — build the Nokia 2100 (NAM-2) provisioned EEPROM image.
#
# The WinTesla v5.84 2100 split (NAM205.840 MCU + NAM205.84E PPM) ships with a BLANK
# (0xFF) EEPROM partition. The separate "2100sharp.pmm" is a complete v5.21 flash dump
# whose top-64 KB EEPROM partition (0x3F0000-0x3FFFFF) carries the real factory NVRAM
# records. We graft that EEPROM onto the v5.84 MCU+PPM (EEPROMs are HW-specific,
# firmware-VERSION-agnostic — same call as the 3310 v6.39/v5.79 graft).
#
# ONE cross-version fixup is needed: the v5.21 EEPROM's calibration record at device
# 0x3FC026 (0x120 bytes, trailing 16-bit additive-sum checksum at +0x11C) does NOT
# reconcile under the v5.84 self-test's checksum (fw fn 0x2F113C — a plain 16-bit byte
# sum). The v5.84 boot self-test reads this record, sums its first 0x11C bytes, and
# compares to the stored checksum (verdict fn 0x256720 item 18, predicate 0x291E64);
# a mismatch clears verdict bit6 -> CONTACT SERVICE. A real v5.84-calibrated phone's
# record is self-consistent, so we RECALC the stored checksum to match the content
# (faithful calib-checksum recalc — same fix class as the 5210 / 8210 bring-ups). This
# does NOT force a self-test pass: the record becomes genuinely self-consistent and the
# firmware's own check passes on its own.
#
#   tools/provision_2100_eeprom.py            # regenerate from the source dumps
#
# Inputs (gitignored, copyrighted — must be present locally):
#   ~/Downloads/NAM-2_extracted/{NAM205.840,NAM205.84E,2100sharp.pmm}   (rar contents)
# Outputs (gitignored):
#   firmware/Nokia 2100 NAM-2 v5.84 A (EEPROM).fls   (full 2 MB, EEPROM grafted + fixed)
#   firmware/Nokia 2100 NAM-2 EEPROM.bin             (raw 64 KB partition, fixed)

import os
import struct
import subprocess
import sys

HOME = os.path.expanduser("~")
SRC = os.path.join(HOME, "Downloads", "NAM-2_extracted")
MCU = os.path.join(SRC, "NAM205.840")
PPM = os.path.join(SRC, "NAM205.84E")
SHARP = os.path.join(SRC, "2100sharp.pmm")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT_FLS = os.path.join(ROOT, "firmware", "Nokia 2100 NAM-2 v5.84 A (EEPROM).fls")
OUT_BIN = os.path.join(ROOT, "firmware", "Nokia 2100 NAM-2 EEPROM.bin")

FLASH_BASE = 0x200000
EE_LO = 0x3F0000            # 2100 EEPROM partition (partition-map 2100 row)
EE_HI = 0x400000            # top of the 2 MB image
CALIB_REC = 0x3FC026        # calib record validated by v5.84 self-test item 18
CALIB_LEN = 0x11C           # bytes summed (record is 0x120; checksum is the last 4)


def fiasco_to_flat(*paths, size=0x200000):
    """Assemble FIASCO streams into a flat image via the existing fls_assemble."""
    out = os.path.join("/tmp", "_2100_assemble.fls")
    subprocess.run(
        [sys.executable, os.path.join(HERE, "fls_assemble.py"), "-o", out, *paths],
        check=True, stdout=subprocess.DEVNULL,
    )
    return bytearray(open(out, "rb").read())


def main():
    for p in (MCU, PPM, SHARP):
        if not os.path.exists(p):
            sys.exit("missing source dump: %s\n(extract NAM-2 v.05.84 2100.rar there)" % p)

    split = fiasco_to_flat(MCU, PPM)                  # v5.84 MCU+PPM, blank EEPROM
    sharp = fiasco_to_flat(SHARP)                     # v5.21 full image (has EEPROM)

    # graft the v5.21 EEPROM partition onto the v5.84 image
    split[EE_LO - FLASH_BASE:EE_HI - FLASH_BASE] = sharp[EE_LO - FLASH_BASE:EE_HI - FLASH_BASE]

    # faithful calib-checksum recalc for the item-18 record (16-bit additive sum)
    ro = CALIB_REC - FLASH_BASE
    cks = sum(split[ro:ro + CALIB_LEN]) & 0xFFFF
    old = int.from_bytes(split[ro + CALIB_LEN:ro + CALIB_LEN + 4], "big")
    split[ro + CALIB_LEN:ro + CALIB_LEN + 4] = struct.pack(">I", cks)
    print("calib record 0x%06X: checksum 0x%04X -> 0x%04X (16-bit sum over 0x%X bytes)"
          % (CALIB_REC, old & 0xFFFF, cks, CALIB_LEN))

    open(OUT_FLS, "wb").write(split)
    open(OUT_BIN, "wb").write(split[EE_LO - FLASH_BASE:EE_HI - FLASH_BASE])
    print("wrote %s" % OUT_FLS)
    print("wrote %s (64 KB raw EEPROM partition)" % OUT_BIN)


if __name__ == "__main__":
    main()
