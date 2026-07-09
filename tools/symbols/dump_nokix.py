#!/usr/bin/env python3
"""
Dump every NokiX locator name against a firmware → canonical symbols YAML.

Walks the LOCATE_*.rx files in `ref/NokiX-scripts/.../macros/` for every
`if func="NAME"` it can find (~500 names on the current 3310 macro set), runs
`tools/nokix/nokix locate_one.nrx NAME` on each, and emits a YAML file:

    firmware:
      id: 3bdfac2d
      file: flash/My 3310 NR1 v5.79.fls
      version: v5.79
      model: 3310
    addresses:
      DIALOG_OPEN:      { addr: 0x0028D278, source: nokix-locate }
      NETMONITOR_STATUS:{ addr: 0x0011FE07, source: nokix-locate }
      ...
    unresolved:        # names NokiX knows about but doesn't find in this fw
      - PHONE_MODEL    # (returns text, not addr — wrong kind for this lookup)
      - ...

This is the *generated* half of the symbol store. A sibling `<model>-<ver>.overlay.yaml`
holds hand-curated entries NokiX doesn't know about (msg IDs, RAM addresses we've RE'd
ourselves, fns like task_4_dispatcher that don't have a LOCATE macro).

Usage:
    tools/symbols/dump_nokix.py <firmware.fls> [--model MODEL]
        [--out tools/symbols/<model>-<ver>.nokix.yaml]
        [--names PATH]          # override the auto-derived name list
        [--workers N]           # parallel nokix processes (default 8)
        [--limit N]             # only the first N names — useful for smoke tests

Runs ~500 nokix processes (~17 ms each ≈ 9 s wall-clock with -j 8).
"""

import argparse
import concurrent.futures
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO     = Path(__file__).resolve().parent.parent.parent
NOKIX    = REPO / "tools" / "nokix" / "nokix"
LOC_ONE  = REPO / "tools" / "nokix" / "locate_one.nrx"
MACROS   = REPO / "ref" / "NokiX-scripts" / "NokiX" / "scripts" / "scripts" / "macros"

LOCATE_FILES = [
    "LOCATE_BUILTIN.rx", "LOCATE_OWN.rx", "LOCATE_SPECIAL.rx",
    "LOCATE_DATA.rx", "LOCATE_AUTO.rx",
    # LOCATE_MATH.rx and LOCATE_OBSOLETE.rx omitted on purpose:
    # MATH locators *unconditionally* `create` a Thumb blob in free flash
    # (NokiX uses them to inject math fns into firmware that lacks them) — every
    # call returns the free-pool base, not a real address. OBSOLETE is legacy
    # aliasing that returns through other locators. Neither helps symbol discovery.
]

# Names whose LOCATE handler returns text or has side effects we don't want.
# Skip these wholesale rather than firing nokix at them.
TEXT_OR_META = {
    "PHONE_MODEL", "FIRMWARE_TYPE", "FIRMWARE_VERSION", "FIRMWARE_DATE",
    "FIRMWARE_ID",
}

# Default free-flash pool base for 3310 v5.79 (auto-detected by nokix as the
# longest 0xFF run; ~240 KB at this address). Any locate() that returns this
# address is a `create`-allocated stub, NOT a real firmware symbol — filter it
# out. Other builds may have a different pool base; the wrapper's -v flag is the
# fastest way to discover it for a new fw (look for "[nokix] free flash pool …").
DEFAULT_FREE_POOL = 0x00393EC0


def harvest_names(extra_files=()):
    """Grep `if func="NAME"` from every LOCATE_*.rx macro."""
    names = set()
    pat = re.compile(r'^\s*if\s+func\s*=\s*"([^"]+)"', re.MULTILINE)
    for rel in list(LOCATE_FILES) + list(extra_files):
        path = MACROS / rel
        if not path.exists():
            print(f"  [warn] {rel} not found at {path}", file=sys.stderr)
            continue
        for m in pat.finditer(path.read_text()):
            names.add(m.group(1))
    return sorted(names - TEXT_OR_META)


def fnv1a32(data: bytes) -> int:
    """FNV-1a 32-bit over an arbitrary byte range."""
    h = 0x811C9DC5
    for b in data:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def fw_build_id(fw_bytes: bytes) -> int:
    """Stable firmware id: hash the canonical 32-byte MCU version header at
    flash offset 0x1FC, NOT the whole file. The EEPROM/FFS partition at the
    tail mutates per session, so a whole-file hash gives the same code-identical
    firmware different ids (e.g. flash/My ... vs firmware/Factory Reset ...).
    MAD2 puts the header at 0x1FC; the V1/MAD1 family (5110/6110/8810/...) has no 0x1FC
    header — its version header lives in the PPM/content block at a 64 KB-block boundary +4
    (0xN0004), so scan boundaries before the whole-file last resort.
    Matches the algorithm in tools/disfw.c (fw_build_id) and tools/nokix/nokix.c.
    """
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


def run_one(fw, model, name, free_pool, timeout=10):
    """Spawn nokix for a single locator. Returns (name, addr_int, reason).

    Suppresses the patched-flash write by redirecting `-o` to /dev/null — some
    locators (and any `create` side effects) would otherwise write a 2 MB
    .patched.fls per invocation.
    """
    env = os.environ.copy()
    env["REGINA_MACROS"] = str(MACROS)
    try:
        r = subprocess.run(
            [str(NOKIX), "-m", model, "-o", "/dev/null", str(fw), str(LOC_ONE), name],
            env=env, capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return name, None, "timeout"
    if r.returncode != 0:
        return name, None, "fail"
    # Parse the one expected line: NAME=DEC
    for line in r.stdout.splitlines():
        if "=" not in line:
            continue
        k, _, v = line.partition("=")
        if k.strip() != name:
            continue
        try:
            addr = int(v.strip())
        except ValueError:
            return name, None, "parse"
        if addr == free_pool:
            return name, None, "stub-allocated"
        return name, addr, None
    return name, None, "nooutput"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("firmware", help="Path to .fls image")
    ap.add_argument("--model", default="3310")
    ap.add_argument("--out", help="Output YAML path (default: tools/symbols/<model>-<ver>.nokix.yaml)")
    ap.add_argument("--names", help="File with one name per line (override the LOCATE_*.rx harvest)")
    ap.add_argument("--workers", "-j", type=int, default=8)
    ap.add_argument("--limit", type=int, default=0, help="Process only the first N names")
    ap.add_argument("--free-pool", type=lambda s: int(s, 0), default=DEFAULT_FREE_POOL,
                    help=f"Free-flash pool base (any locate() returning this is a stub; default 0x{DEFAULT_FREE_POOL:08X})")
    args = ap.parse_args()

    if not NOKIX.exists():
        sys.exit(f"nokix binary missing — build via tools/nokix/Makefile first ({NOKIX})")
    if not LOC_ONE.exists():
        sys.exit(f"locate_one.nrx missing — expected at {LOC_ONE}")

    fw = Path(args.firmware).resolve()
    if not fw.exists():
        sys.exit(f"firmware not found: {fw}")

    fw_bytes = fw.read_bytes()
    fw_id = f"{fw_build_id(fw_bytes):08x}"

    if args.names:
        names = [ln.strip() for ln in Path(args.names).read_text().splitlines() if ln.strip() and not ln.startswith("#")]
    else:
        names = harvest_names()
    if args.limit:
        names = names[: args.limit]

    print(f"[dump_nokix] fw={fw.name} id={fw_id} model={args.model} names={len(names)} workers={args.workers}", file=sys.stderr)

    results = {}
    skipped = {}    # name -> reason
    t0 = time.time()
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(run_one, fw, args.model, n, args.free_pool) for n in names]
        for i, f in enumerate(concurrent.futures.as_completed(futures), 1):
            name, addr, reason = f.result()
            if addr is not None:
                results[name] = addr
            else:
                skipped[name] = reason
            if i % 50 == 0:
                elapsed = time.time() - t0
                print(f"  [dump_nokix] {i}/{len(names)} done  ok={len(results)}  skip={len(skipped)}  {elapsed:.1f}s", file=sys.stderr)

    elapsed = time.time() - t0
    print(f"[dump_nokix] total: ok={len(results)}  skip={len(skipped)}  in {elapsed:.1f}s", file=sys.stderr)

    # Parse the canonical DCT3 MCU version header at fixed offset 0x1FC. Same
    # format nokix.c uses now (32 bytes: "V VV.VV\nDD-MM-YY\n<TYPE>\n(c) NMP.").
    # This is model-agnostic (any DCT3 build) — relying on the regex previously
    # only matched NHM-5/3310. Returns ("05.79", "NHM-5") or ("", "") on miss.
    fw_str, fw_type = "", ""
    if len(fw_bytes) >= 0x1FC + 32:
        h = fw_bytes[0x1FC:0x1FC + 32]
        if h[0:1] == b"V" and h[1:2] == b" " and h[7:8] == b"\n" and h[10:11] == b"-" and h[13:14] == b"-" and h[16:17] == b"\n":
            v0 = b"0" if h[2:3] == b" " else h[2:3]
            fw_str = (v0 + h[3:4] + b"." + h[5:7]).decode("ascii", "ignore")
            # type follows the date \n at offset 17, terminated by next \n
            end = h.find(b"\n", 17)
            if end > 17:
                fw_type = h[17:end].decode("ascii", "ignore")

    out = args.out
    if not out:
        # Drop the leading zero so "05.79" -> "v579" (matches our doc conventions).
        ver_tag = f"v{fw_str.replace('.', '').lstrip('0')}" if fw_str else fw_id[:6]
        out = REPO / "tools" / "symbols" / f"{args.model}-{ver_tag}.nokix.yaml"
    out = Path(out)

    # Hand-roll YAML (no PyYAML dependency).
    lines = [
        "# Generated by tools/symbols/dump_nokix.py — DO NOT EDIT by hand.",
        "# Re-run the script to refresh. Hand-curated symbols go in the sibling overlay.yaml.",
        "firmware:",
        f"  id: {fw_id}",
        f"  file: {fw.relative_to(REPO) if fw.is_relative_to(REPO) else fw}",
        f"  model: {args.model}",
    ]
    if fw_str:
        lines.append(f"  version: {fw_str}")
    if fw_type:
        lines.append(f"  type: {fw_type}")
    lines.append("addresses:")
    for name in sorted(results):
        addr = results[name]
        # NokiX returns RAM addresses for some objects (e.g. NETMONITOR_STATUS).
        # We don't try to classify here — every entry is "what locate() returned".
        lines.append(f"  {name}: {{ addr: 0x{addr:08X}, source: nokix-locate }}")
    if skipped:
        lines.append("unresolved:")
        # Group by reason so we can spot patterns (most will be 'fail' = locator unknown for this build)
        reasons = {}
        for n, r in skipped.items():
            reasons.setdefault(r, []).append(n)
        for r, ns in sorted(reasons.items()):
            lines.append(f"  # {r} ({len(ns)})")
            for n in sorted(ns):
                lines.append(f"  - {n}")

    out.write_text("\n".join(lines) + "\n")
    print(f"[dump_nokix] wrote {out}", file=sys.stderr)


if __name__ == "__main__":
    main()
