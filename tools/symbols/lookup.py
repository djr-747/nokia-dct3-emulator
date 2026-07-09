#!/usr/bin/env python3
"""
Symbol-store lookup CLI — name/addr/msg-id resolver across all generated
+ overlay YAMLs for a given firmware build.

Sections recognised:
- addresses: code addresses (NokiX-located + hand-overlay)
- ram:       RAM byte addresses (same shape as addresses; kept separate for org)
- messages:  per-msg-ID semantic table ({ id: 0xNNNN, role: "..." })

Usage:
    ./lookup.py <build-tag> <name-or-addr-or-msg>
    ./lookup.py 3310-v579 DIALOG_OPEN              # name -> 0x0028DB38
    ./lookup.py 3310-v579 0x002EEB94               # addr -> CCONT_RESET_WDT
    ./lookup.py 3310-v579 DSP_WATCHDOG_TIMER       # canonical name, addr unknown
    ./lookup.py 3310-v579 msg 0x366                # msg id -> MSG_KEY_DOWN
    ./lookup.py 3310-v579 msg MSG_DIALOG_DRAW      # msg name -> 0x057C
    ./lookup.py 3310-v579 list                     # dump every name=addr
    ./lookup.py 3310-v579 list-messages            # dump every msg
    ./lookup.py 3310-v579 list-canonical [CAT]     # dump every canonical name [optionally
                                                   #   filtered by category, e.g. timer]
"""

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# `{ addr: 0xHHHHHHHH, source: TAG[, role: "..."] }` — addresses + ram identical
ADDR_LINE = re.compile(r"^\s+([A-Za-z_][A-Za-z_0-9]*)\s*:\s*\{\s*addr:\s*(0x[0-9A-Fa-f]+)\s*,\s*source:\s*([^,}]+?)(?:\s*,\s*role:\s*\"?([^\"}]*)\"?)?\s*\}")
# `{ id: 0xHHHH[, role: "..."] }` — messages
MSG_LINE  = re.compile(r"^\s+([A-Za-z_][A-Za-z_0-9]*)\s*:\s*\{\s*id:\s*(0x[0-9A-Fa-f]+|\d+)(?:\s*,\s*role:\s*\"?([^\"}]*)\"?)?\s*\}")
# `{ category: CAT }` — canonical (address-less) names from re/3330/symbols/
CANON_LINE = re.compile(r"^\s+([A-Za-z_][A-Za-z_0-9]*)\s*:\s*\{\s*category:\s*([A-Za-z_][A-Za-z_0-9]*)\s*\}")


def load(build_tag):
    """Return (addresses, messages, canonical) — name -> dict each.

    addresses entries: {addr: int, source: str, role: str, kind: 'code'|'ram'}
    messages entries:  {id: int, role: str}
    canonical entries: {category: str, source: 'canonical-3330'}  — address-less
                       (= names known to exist in 3330 firmware but not yet RE'd
                        to a 3310 address). Resolved when scratch.py promotes the
                        name into overlay.yaml.

    Overlay wins on name conflict (loaded after nokix). Names addressed in
    overlay/nokix shadow the canonical entry (the canonical is "resolved").
    """
    addrs = {}
    msgs = {}
    canon = {}
    for suffix in ("nokix", "overlay", "canonical"):
        path = HERE / f"{build_tag}.{suffix}.yaml"
        if not path.exists():
            continue
        section = None
        for line in path.read_text().splitlines():
            # Section boundary — any top-level key resets the parse mode.
            stripped = line.lstrip()
            if line and not line[0].isspace() and stripped.endswith(":") and ":" in line:
                key = stripped.rstrip(":").strip()
                if key in ("addresses", "ram", "messages", "canonical"):
                    section = key
                    continue
                else:
                    section = None
                    continue
            if section in ("addresses", "ram"):
                m = ADDR_LINE.match(line)
                if m:
                    name, hex_addr, src, role = m.groups()
                    addrs[name] = {
                        "addr": int(hex_addr, 16),
                        "source": src.strip(),
                        "role": (role or "").strip(),
                        "kind": "ram" if section == "ram" else "code",
                    }
            elif section == "messages":
                m = MSG_LINE.match(line)
                if m:
                    name, val, role = m.groups()
                    msgs[name] = {
                        "id": int(val, 0),
                        "role": (role or "").strip(),
                    }
            elif section == "canonical":
                m = CANON_LINE.match(line)
                if m:
                    name, cat = m.groups()
                    # Shadow canonical entry if name already addressed elsewhere.
                    if name in addrs:
                        continue
                    canon[name] = {
                        "category": cat,
                        "source": "canonical-3330",
                    }
    return addrs, msgs, canon


def fmt_canon_hit(name, e):
    return f"{name:32s}  CANONICAL  category={e['category']}  ({e['source']} — addr unknown)"


def fmt_addr_hit(name, e):
    role = f"  — {e['role']}" if e["role"] else ""
    kind = f" [{e['kind']}]"
    return f"{name:32s} 0x{e['addr']:08X}  ({e['source']}){kind}{role}"


def fmt_msg_hit(name, e):
    role = f"  — {e['role']}" if e["role"] else ""
    return f"{name:32s} id=0x{e['id']:04X}{role}"


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__.strip())
    build = sys.argv[1]
    args = sys.argv[2:]
    addrs, msgs, canon = load(build)
    if not addrs and not msgs and not canon:
        sys.exit(f"no symbols loaded for build tag '{build}' — looked for "
                 f"{HERE}/{build}.{{nokix,overlay,canonical}}.yaml")

    # `list` / `list-messages` / `list-canonical` modes
    if args[0] == "list":
        for n, e in sorted(addrs.items(), key=lambda kv: kv[1]["addr"]):
            print(fmt_addr_hit(n, e))
        return
    if args[0] == "list-messages":
        for n, e in sorted(msgs.items(), key=lambda kv: kv[1]["id"]):
            print(fmt_msg_hit(n, e))
        return
    if args[0] == "list-canonical":
        # Optional category filter: `list-canonical timer`
        cat_filter = args[1] if len(args) > 1 else None
        for n, e in sorted(canon.items()):
            if cat_filter and e["category"] != cat_filter:
                continue
            print(fmt_canon_hit(n, e))
        return

    # `msg <name-or-id>` mode
    if args[0] == "msg":
        if len(args) < 2:
            sys.exit("usage: msg <NAME> | msg 0xNNNN")
        q = args[1]
        if q in msgs:
            print(fmt_msg_hit(q, msgs[q]))
            return
        try:
            target = int(q, 0)
        except ValueError:
            # substring match on name
            hits = [(n, e) for n, e in msgs.items() if q.lower() in n.lower()]
            if not hits:
                sys.exit(f"no msg matching '{q}'")
            for n, e in sorted(hits):
                print(fmt_msg_hit(n, e))
            return
        hits = [(n, e) for n, e in msgs.items() if e["id"] == target]
        if not hits:
            sys.exit(f"no msg with id=0x{target:04X}")
        for n, e in sorted(hits):
            print(fmt_msg_hit(n, e))
        return

    # Generic single-arg lookup
    query = args[0]

    # Address lookup
    if query.lower().startswith("0x") or query.replace("_", "").isdigit():
        try:
            target = int(query, 0)
        except ValueError:
            sys.exit(f"can't parse '{query}' as an address")
        hits = [(n, e) for n, e in addrs.items() if e["addr"] == target]
        if hits:
            for n, e in sorted(hits):
                print(fmt_addr_hit(n, e))
            return
        # No exact hit — show nearest.
        nearest = min(addrs.items(), key=lambda kv: abs(kv[1]["addr"] - target))
        n, e = nearest
        delta = target - e["addr"]
        print(f"no exact match for 0x{target:08X}")
        print(f"  nearest: {fmt_addr_hit(n, e)}  ({'+' if delta>=0 else ''}{delta:+d} bytes)")
        sys.exit(1)

    # Name lookup (exact then case-insensitive substring across addrs+msgs+canonical)
    if query in addrs:
        print(fmt_addr_hit(query, addrs[query]))
        return
    if query in msgs:
        print(fmt_msg_hit(query, msgs[query]))
        return
    if query in canon:
        print(fmt_canon_hit(query, canon[query]))
        return
    matches = []
    for n, e in addrs.items():
        if query.lower() in n.lower():
            matches.append(("addr", n, e))
    for n, e in msgs.items():
        if query.lower() in n.lower():
            matches.append(("msg", n, e))
    for n, e in canon.items():
        if query.lower() in n.lower():
            matches.append(("canon", n, e))
    if not matches:
        sys.exit(f"no symbol matching '{query}' in {build}")
    print(f"# {len(matches)} match(es) for '{query}':")
    for kind, n, e in sorted(matches, key=lambda x: x[1]):
        if kind == "addr":
            print("  " + fmt_addr_hit(n, e))
        elif kind == "msg":
            print("  " + fmt_msg_hit(n, e))
        else:
            print("  " + fmt_canon_hit(n, e))


if __name__ == "__main__":
    main()
