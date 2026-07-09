#!/usr/bin/env python3
"""
scratch.py — session RE scratchpad: append findings during a session, promote
to overlay.yaml when ready.

The scratchpad is a single plain-text file per build at
    tools/symbols/<build>.scratch.txt

Format — same `<hex_addr> <NAME>  [# note]` shape as symbols.txt's [addresses]
section, so disfw can load it directly. Lines starting with `#` are comments.

Lifecycle:
    add      append a new finding (hex addr + name + optional note)
    list     show pending findings
    drop     remove an entry (typo / wrong name)
    promote  merge all entries into <build>.overlay.yaml AND regenerate
             symbols.txt + symbols_*.h; archive the scratch.

The disfw runner reads the scratchpad automatically (in addition to the
.symbols.txt sidecar). Adding here makes the next disfw run pick up the new
label without a manual regen step.

Usage:
    ./scratch.py <build> add 0x002D11F4 SWDSP_STAGER scenario maps to reason
    ./scratch.py <build> add 0x002E0F20 RESET_POSTER
    ./scratch.py <build> list
    ./scratch.py <build> drop SWDSP_STAGER
    ./scratch.py <build> promote
"""

import argparse
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

# `<hex> <NAME> [# note]`  — note runs from `#` to end of line.
ENTRY = re.compile(r"^\s*(0x[0-9A-Fa-f]+)\s+([A-Za-z_][A-Za-z_0-9]*)\s*(?:#\s*(.*))?\s*$")


def scratch_path(build: str) -> Path:
    return HERE / f"{build}.scratch.txt"


def archive_path(build: str) -> Path:
    return HERE / f"{build}.scratch.archive.txt"


def overlay_path(build: str) -> Path:
    return HERE / f"{build}.overlay.yaml"


def canonical_path(build: str) -> Path:
    return HERE / f"{build}.canonical.yaml"


# `<name>: { category: <cat> }` — canonical entries (address-less stubs).
CANON_LINE = re.compile(r"^\s+([A-Za-z_][A-Za-z_0-9]*)\s*:\s*\{\s*category:\s*([A-Za-z_][A-Za-z_0-9]*)\s*\}")


def load_canonical(build: str) -> dict:
    """Return {name: category} for all address-less canonical names."""
    p = canonical_path(build)
    if not p.exists():
        return {}
    out = {}
    for line in p.read_text().splitlines():
        m = CANON_LINE.match(line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


def remove_from_canonical(build: str, names: list) -> int:
    """Remove the given names from canonical.yaml. Returns count removed."""
    if not names:
        return 0
    p = canonical_path(build)
    if not p.exists():
        return 0
    name_set = set(names)
    removed = 0
    keep = []
    for line in p.read_text().splitlines(keepends=True):
        m = CANON_LINE.match(line)
        if m and m.group(1) in name_set:
            removed += 1
            continue
        keep.append(line)
    if removed:
        p.write_text("".join(keep))
    return removed


def ensure_scratch(build: str) -> Path:
    p = scratch_path(build)
    if not p.exists():
        p.write_text(
            f"# Scratchpad for {build} — session RE findings, pending promotion to overlay.\n"
            f"# Format: <hex_addr> <NAME>  [# note]\n"
            f"# `./scratch.py {build} promote` to merge into overlay.yaml + regen artifacts.\n"
            f"# NB: [addresses] header is required so disfw's load_symbols picks entries up.\n"
            "\n"
            "[addresses]\n"
        )
    elif "[addresses]" not in p.read_text():
        # Existing scratch from before the header convention — splice it in.
        body = p.read_text()
        # Insert [addresses] after the comment block (before the first non-#/non-blank line)
        lines = body.splitlines(keepends=True)
        insert_at = 0
        for i, ln in enumerate(lines):
            if ln.strip() and not ln.lstrip().startswith("#"):
                insert_at = i; break
        else:
            insert_at = len(lines)
        lines.insert(insert_at, "[addresses]\n")
        p.write_text("".join(lines))
    return p


def load_entries(path: Path):
    """Parse the scratchpad. Returns list of (addr_int, name, note)."""
    out = []
    if not path.exists():
        return out
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if line.lstrip().startswith("["):
            # Section marker — load_symbols (in disfw) needs `[addresses]` but
            # scratch.py only deals with addresses, so accept-and-skip.
            continue
        m = ENTRY.match(line)
        if not m:
            print(f"  [warn] malformed line ignored: {line!r}", file=sys.stderr)
            continue
        addr = int(m.group(1), 16)
        name = m.group(2)
        note = (m.group(3) or "").strip()
        out.append((addr, name, note))
    return out


# ----- subcommands -----


def cmd_add(args):
    """add <hex_addr> <NAME> [optional note words...]"""
    if len(args.rest) < 2:
        sys.exit("usage: add <hex_addr> <NAME> [note...]")
    addr_str, name = args.rest[0], args.rest[1]
    note = " ".join(args.rest[2:]).strip()
    try:
        addr = int(addr_str, 0)
    except ValueError:
        sys.exit(f"can't parse address '{addr_str}'")
    if not re.match(r"^[A-Za-z_][A-Za-z_0-9]*$", name):
        sys.exit(f"name '{name}' must be a C-identifier (letters/digits/underscore)")
    p = ensure_scratch(args.build)
    # Idempotent — don't add the same (addr, name) pair twice.
    for a, n, _ in load_entries(p):
        if a == addr and n == name:
            print(f"  [info] already in scratch: 0x{addr:08X} {name}")
            return
    with open(p, "a") as f:
        ts = datetime.now().strftime("%H:%M")
        line = f"0x{addr:08X} {name}"
        if note:
            line += f"  # {note} [{ts}]"
        else:
            line += f"  # [{ts}]"
        f.write(line + "\n")
    # Hint when resolving a canonical (3330-vocabulary) name — that's the happy path.
    canon = load_canonical(args.build)
    if name in canon:
        print(f"  [scratch] added 0x{addr:08X} {name}  ← resolves canonical [{canon[name]}]")
    else:
        print(f"  [scratch] added 0x{addr:08X} {name}")


def cmd_list(args):
    p = scratch_path(args.build)
    entries = load_entries(p)
    if not entries:
        print(f"  [scratch] empty: {p}")
        return
    print(f"  [scratch] {len(entries)} pending in {p}")
    for addr, name, note in sorted(entries, key=lambda x: x[0]):
        print(f"    0x{addr:08X}  {name}{'  — ' + note if note else ''}")


def cmd_drop(args):
    """drop <NAME or 0xADDR>"""
    if not args.rest:
        sys.exit("usage: drop <NAME or 0xADDR>")
    q = args.rest[0]
    p = ensure_scratch(args.build)
    keep, dropped = [], []
    for line in p.read_text().splitlines():
        m = ENTRY.match(line)
        if not m:
            keep.append(line)
            continue
        addr = int(m.group(1), 16)
        name = m.group(2)
        match = False
        try:
            match = (int(q, 0) == addr)
        except ValueError:
            match = (q == name)
        if match:
            dropped.append((addr, name))
        else:
            keep.append(line)
    p.write_text("\n".join(keep) + "\n")
    if dropped:
        for a, n in dropped:
            print(f"  [scratch] dropped 0x{a:08X} {n}")
    else:
        print(f"  [scratch] no match for {q!r}")


def cmd_promote(args):
    p = scratch_path(args.build)
    entries = load_entries(p)
    if not entries:
        print("  [scratch] nothing to promote")
        return

    # Build an overlay block. Append into the `addresses:` section preserving
    # everything that's already there. Insert before the next top-level key
    # (`ram:` / `messages:` / EOF).
    op = overlay_path(args.build)
    if not op.exists():
        sys.exit(f"overlay file not found: {op} — create it manually first")
    src = op.read_text().splitlines(keepends=True)

    # Find the `addresses:` block and its end.
    addr_start, addr_end = -1, len(src)
    for i, line in enumerate(src):
        if line.startswith("addresses:"):
            addr_start = i
        elif addr_start >= 0 and line and not line[0].isspace() and ":" in line and not line.startswith("#"):
            addr_end = i
            break
    if addr_start < 0:
        sys.exit(f"no `addresses:` section in {op} — can't promote into a missing section")

    # Check for name collisions inside addresses block — refuse to silently overwrite.
    existing_names, existing_addrs = set(), set()
    for line in src[addr_start:addr_end]:
        m = re.match(r"^\s+([A-Za-z_][A-Za-z_0-9]*)\s*:\s*\{\s*addr:\s*(0x[0-9A-Fa-f]+)", line)
        if m:
            existing_names.add(m.group(1))
            existing_addrs.add(int(m.group(2), 16))

    # Cross-reference canonical.yaml — promoted names that have a canonical entry
    # are resolving 3330-vocabulary stubs and get their category tagged.
    canon = load_canonical(args.build)

    new_lines = []
    resolved_canon = []   # names we resolved a canonical for (removed at end)
    skip = []
    for addr, name, note in entries:
        if name in existing_names:
            skip.append((addr, name, "name already in overlay"))
            continue
        if addr in existing_addrs:
            skip.append((addr, name, f"addr 0x{addr:08X} already in overlay"))
            continue
        role = note if note else "promoted from scratch"
        # If the name is in canonical.yaml, tag the role with its category so the
        # 3330-derived classification (timer/task/event/etc.) survives the move.
        if name in canon:
            cat = canon[name]
            role = (f"{role}  [{cat}]" if role != "promoted from scratch"
                                       else f"resolves canonical [{cat}]")
            resolved_canon.append(name)
        # Quote role if it has commas or quotes that'd break our parser.
        if '"' in role:
            role = role.replace('"', "\\\"")
        new_lines.append(
            f"  {name + ':':33s} {{ addr: 0x{addr:08X}, source: overlay, role: \"{role}\" }}\n"
        )

    if not new_lines:
        if skip:
            print(f"  [scratch] no promotable entries — {len(skip)} skipped:")
            for a, n, why in skip:
                print(f"    0x{a:08X} {n}  ({why})")
        return

    # Insert just before addr_end with a banner so promoted entries are easy to
    # find later if a re-name is needed.
    banner = f"  # ---- promoted from scratch on {datetime.now():%Y-%m-%d %H:%M} ----\n"
    insert = [banner] + new_lines
    new_src = src[:addr_end] + insert + src[addr_end:]
    op.write_text("".join(new_src))
    print(f"  [scratch] promoted {len(new_lines)} entries into {op}")
    if skip:
        for a, n, why in skip:
            print(f"    skip: 0x{a:08X} {n}  ({why})")

    # Resolve canonical stubs — remove now-addressed names from canonical.yaml so
    # the wishlist shrinks as RE progresses (idempotent if file absent / no hits).
    if resolved_canon:
        n_removed = remove_from_canonical(args.build, resolved_canon)
        print(f"  [scratch] resolved {n_removed} canonical (3330) stubs:")
        for nm in sorted(resolved_canon):
            print(f"    - {nm}")

    # Archive the scratch — keep an append-only log of what was promoted, then
    # zero the live scratch so the next session starts fresh.
    ap = archive_path(args.build)
    with open(ap, "a") as f:
        f.write(f"\n# ---- archived on {datetime.now():%Y-%m-%d %H:%M} ----\n")
        f.write(p.read_text().split("\n", 3)[-1] if p.exists() else "")
    p.write_text(
        f"# Scratchpad for {args.build} — session RE findings, pending promotion to overlay.\n"
        f"# Format: <hex_addr> <NAME>  [# note]\n"
        f"# Cleared on {datetime.now():%Y-%m-%d %H:%M} after promotion. See {ap.name} for archive.\n"
        "\n"
    )

    # Regenerate downstream artifacts so disfw, mad2 post-mortem, and the
    # lookup CLI all pick up the new names in the next session.
    print(f"  [scratch] regenerating downstream artifacts…")
    try:
        subprocess.run([str(HERE / "gen_symbols_txt.py"), args.build], check=True, cwd=REPO)
        subprocess.run([str(HERE / "gen_c_table.py"),    args.build], check=True, cwd=REPO)
        subprocess.run([str(HERE / "symbols_check.py"),  args.build], check=False, cwd=REPO)
        print("  [scratch] done. New symbols are live in disfw + mad2 post-mortem.")
    except subprocess.CalledProcessError as e:
        print(f"  [warn] regen failed: {e}", file=sys.stderr)


# ----- main -----

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("build", help="Build tag (e.g. 3310-v579)")
    ap.add_argument("cmd", choices=["add", "list", "drop", "promote"])
    ap.add_argument("rest", nargs=argparse.REMAINDER)
    args = ap.parse_args()
    {"add": cmd_add, "list": cmd_list, "drop": cmd_drop, "promote": cmd_promote}[args.cmd](args)


if __name__ == "__main__":
    main()
