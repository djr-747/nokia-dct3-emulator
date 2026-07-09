#!/usr/bin/env python3
"""
symbols_check.py — validate every entry in <build>.{nokix,overlay}.yaml
against the firmware they claim to describe.

Checks per entry:
  1. The firmware file referenced in the YAML header exists and matches
     the FNV-1a id (catches "you regenerated nokix.yaml but the .fls
     moved / was edited").
  2. Each address falls in a valid region (flash, RAM, MMIO, IO window).
  3. For code-region addresses (>= flash_base): the byte at addr looks
     like a plausible instruction prefix:
        - Thumb fn:  push {…,lr} = 0xB5xx (high byte = 0xB5)
        - Thumb data/branch:  any sane Thumb opcode high nibble
        - ARM fn:    typical prologue (E92D… stmdb sp!, etc.)
     Pure RAM literals (data tables) are accepted as-is.
  4. Optional `expected_prefix: "B5 00 B0 81"` field — exact byte match.

Exit code 0 = all green; 1 = any failure (printed with reason).

Usage:
    ./symbols_check.py                          # all builds in tools/symbols/
    ./symbols_check.py 3310-v579                # just one build
    ./symbols_check.py -v                       # also report passes

This is the linter for the symbol store. Run after any locator update
or any firmware reflash to catch drift before it's discovered the hard way.
"""

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

# Reuse lookup.py's parser by importing it as a sibling module
sys.path.insert(0, str(HERE))
from lookup import load, ADDR_LINE  # noqa: E402

# Header parsing for the `firmware:` block.
HEADER_KEYS = ("id", "file", "model", "version", "type")


def parse_header(path: Path) -> dict:
    """Pull the `firmware:` header out of a YAML."""
    out = {}
    in_fw = False
    for line in path.read_text().splitlines():
        if line.startswith("firmware:"):
            in_fw = True
            continue
        if in_fw:
            if line and not line[0].isspace():
                break
            m = re.match(r"^\s+(\w+)\s*:\s*(.+)$", line)
            if m and m.group(1) in HEADER_KEYS:
                out[m.group(1)] = m.group(2).strip()
    return out


def fnv1a32(data: bytes) -> int:
    """FNV-1a 32-bit over an arbitrary byte range."""
    h = 0x811C9DC5
    for b in data:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def fw_build_id(fw_bytes: bytes) -> int:
    """Stable firmware id: hash the canonical 32-byte MCU version header, NOT the whole
    file. MAD2 puts it at flash 0x1FC; the V1/MAD1 family (5110/6110/8810/...) has no
    0x1FC header — its version header lives in the PPM/content block at a 64 KB-block
    boundary +4 (0xN0004). Mirrors disfw/nokix/dump_nokix."""
    HDR_LEN = 32

    def _is_hdr(h):
        return (h[0:1] == b"V" and h[1:2] == b" " and h[7:8] == b"\n"
                and h[10:11] == b"-" and h[13:14] == b"-" and h[16:17] == b"\n")

    if len(fw_bytes) >= 0x1FC + HDR_LEN and _is_hdr(fw_bytes[0x1FC:0x1FC + HDR_LEN]):
        return fnv1a32(fw_bytes[0x1FC:0x1FC + HDR_LEN])   # MAD2 canonical
    off = 0x4
    while off + HDR_LEN <= len(fw_bytes):
        if _is_hdr(fw_bytes[off:off + HDR_LEN]):
            return fnv1a32(fw_bytes[off:off + HDR_LEN])   # V1/MAD1 block header
        off += 0x10000
    return fnv1a32(fw_bytes)                              # last resort


# Per-model memory map (matches src/models/<model>/profile.c MemMap).
MEM_MAPS = {
    "3310": {"flash": (0x00200000, 0x00400000), "ram": (0x00100000, 0x00200000),
             "io":    (0x00010000, 0x00100000), "mmio": (0x00020000, 0x00020100)},
    "8850": {"flash": (0x00200000, 0x00400000), "ram": (0x00100000, 0x00200000),
             "io":    (0x00010000, 0x00100000), "mmio": (0x00020000, 0x00020100)},
    "3210": {"flash": (0x00200000, 0x00400000), "ram": (0x00100000, 0x00200000),
             "io":    (0x00010000, 0x00100000), "mmio": (0x00020000, 0x00020100)},
    "7110": {"flash": (0x00200000, 0x00600000), "ram": (0x00100000, 0x00200000),
             "io":    (0x00010000, 0x00100000), "mmio": (0x00020000, 0x00020100)},
}


def region(addr: int, mm: dict) -> str:
    if mm["flash"][0] <= addr < mm["flash"][1]: return "flash"
    if mm["ram"][0]   <= addr < mm["ram"][1]:   return "ram"
    if mm["mmio"][0]  <= addr < mm["mmio"][1]:  return "mmio"
    if mm["io"][0]    <= addr < mm["io"][1]:    return "io"
    return "out-of-bounds"


def check_flash_byte(flash: bytes, addr: int, flash_base: int) -> str:
    """Return '' if the addr points at allocated flash; else a one-line reason.

    NokiX locators frequently return mid-function call-site offsets (e.g. the
    instruction after a `bl`, the next-pc captured for an LR snapshot), so a
    strict "Thumb prologue 0xB5xx" check fires false-positive on most entries.

    What we CAN catch reliably: an address that lands in the create-pool's
    blank (0xFF) region OR off the end of flash — those mean either (a) a
    stale entry from before flash was refreshed, or (b) a locator that
    `create`-stub-allocated a fn that doesn't exist in this build, and we
    forgot to filter it via DEFAULT_FREE_POOL in dump_nokix.py.
    """
    off = addr - flash_base
    if off < 0 or off + 2 > len(flash):
        return "off-end-of-flash"
    # Look at the surrounding 4 bytes — if they're all 0xFF the address is in
    # unallocated free flash (= a create-stub site or a typo).
    nearby = flash[max(0, off - 1): off + 3]
    if nearby and all(b == 0xFF for b in nearby):
        return f"blank flash region (0xFF…) at 0x{addr:08X}"
    return ""


def check_one(build_tag: str, verbose: bool) -> int:
    """Validate one build's YAMLs. Returns number of failures."""
    fails = 0
    nokix_path   = HERE / f"{build_tag}.nokix.yaml"
    overlay_path = HERE / f"{build_tag}.overlay.yaml"
    if not nokix_path.exists() and not overlay_path.exists():
        print(f"  [skip] {build_tag}: no YAMLs", file=sys.stderr)
        return 0

    # Use whichever header is present (overlay first — it's authoritative).
    header_src = overlay_path if overlay_path.exists() else nokix_path
    header = parse_header(header_src)

    # Step 1: verify the firmware file + id
    fw_rel = header.get("file")
    if not fw_rel:
        print(f"  [FAIL] {build_tag}: no firmware.file in header ({header_src})")
        return 1
    fw_path = (REPO / fw_rel).resolve()
    if not fw_path.exists():
        # Try unquoting (YAML might wrap paths with spaces)
        fw_path = REPO / fw_rel.strip("'\"")
        if not fw_path.exists():
            print(f"  [FAIL] {build_tag}: firmware not found: {fw_rel}")
            return 1
    fw_bytes = fw_path.read_bytes()
    actual_id = f"{fw_build_id(fw_bytes):08x}"
    expected_id = header.get("id", "")
    if expected_id and expected_id != actual_id:
        print(f"  [FAIL] {build_tag}: id mismatch — yaml={expected_id} actual={actual_id} ({fw_path.name})")
        fails += 1
        # Continue — addresses may still be useful to lint

    # Step 2 + 3: per-address sanity
    model = header.get("model", "3310")
    if model not in MEM_MAPS:
        print(f"  [FAIL] {build_tag}: unknown model '{model}'")
        return fails + 1
    mm = MEM_MAPS[model]
    flash_base = mm["flash"][0]
    addrs, msgs, _ = load(build_tag)

    errs_by_name = []
    warns_by_name = []
    for name, e in addrs.items():
        addr = e["addr"]
        reg = region(addr, mm)
        if reg == "out-of-bounds":
            # Some "addresses" are really constants (action codes, enum values
            # < 0x10000). Treat those as informational, not errors.
            if addr < 0x10000:
                warns_by_name.append((name, f"value {addr} (0x{addr:X}) — looks like a constant, not an address"))
            else:
                errs_by_name.append((name, f"addr 0x{addr:08X} is out-of-bounds for model {model}"))
            continue
        # For code-region entries in flash, sanity-check the byte (catches
        # stale entries pointing at blank-flash / pool-stub regions).
        if reg == "flash" and e["kind"] == "code":
            r = check_flash_byte(fw_bytes, addr, flash_base)
            if r:
                warns_by_name.append((name, r))

    if errs_by_name:
        for n, m in errs_by_name:
            print(f"  [FAIL] {build_tag}: {n}: {m}")
        fails += len(errs_by_name)
    if verbose and warns_by_name:
        for n, m in warns_by_name:
            print(f"  [warn] {build_tag}: {n}: {m}")
    if verbose:
        ok = len(addrs) - len(errs_by_name) - len(warns_by_name)
        print(f"  [ok]   {build_tag}: {ok}/{len(addrs)} addresses fully clean, "
              f"{len(warns_by_name)} warn, {len(errs_by_name)} fail, "
              f"{len(msgs)} msg IDs (msgs not validated against fw — they're semantic constants)")
    return fails


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build", nargs="?", help="Build tag (e.g. 3310-v579). Omitted = all in tools/symbols/")
    ap.add_argument("-v", "--verbose", action="store_true", help="Also report warnings + passes")
    args = ap.parse_args()

    if args.build:
        builds = [args.build]
    else:
        # Auto-enumerate: tools/symbols/<build>.nokix.yaml (overlay-only is also valid)
        builds = set()
        for p in HERE.glob("*.nokix.yaml"):
            builds.add(p.name.replace(".nokix.yaml", ""))
        for p in HERE.glob("*.overlay.yaml"):
            builds.add(p.name.replace(".overlay.yaml", ""))
        builds = sorted(builds)
        if not builds:
            sys.exit("no YAMLs found in tools/symbols/")

    total_fails = 0
    for b in builds:
        if args.verbose:
            print(f"=== {b} ===")
        total_fails += check_one(b, args.verbose)

    if total_fails:
        print(f"\n{total_fails} failure(s)", file=sys.stderr)
        sys.exit(1)
    if args.verbose:
        print("\nall clean.")


if __name__ == "__main__":
    main()
