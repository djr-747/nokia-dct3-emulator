#!/usr/bin/env python3
"""
import_3330_canonical.py — bulk-import the canonical symbol vocabulary
from re/3330/symbols/ into tools/symbols/<build>.canonical.yaml.

The 3330 source has ~1500 named OS objects (tasks, timers, mailboxes, event
ids, APPL states, etc.) extracted from debug-trace strings + lookup tables.
The 3310 firmware shares much of this vocabulary (same RTOS + same MMI core)
but at DIFFERENT addresses, which we resolve via RE one symbol at a time.

The canonical.yaml file holds NAMES we expect to find in 3310 firmware, with
no addresses. Whenever a RE session locates a name's actual 3310 address,
`scratch.py add 0xADDR NAME …` followed by `promote` MOVES the entry from
canonical.yaml → overlay.yaml (now address-resolved). The remaining canonical
entries are the still-pending "wishlist".

Idempotent + re-runnable: skips names that are already addressed in
overlay.yaml or nokix.yaml.

Usage:
    ./import_3330_canonical.py [3310-v579]
"""

import re
import sys
from pathlib import Path
from datetime import datetime

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
SRC  = REPO / "re" / "3330" / "symbols"

# File → category. None = skip (not symbol names — printf templates, English text, etc.).
CATEGORIES = {
    "01_appl_states.txt":      "appl_state",
    "02_tasks.txt":            "task",
    "03_timers.txt":           "timer",
    "04_resources.txt":        "resource",
    "05_semaphores.txt":       "semaphore",
    "06_mailboxes.txt":        "mailbox",
    "07_source_files.txt":     "source_file",
    "08_OS_primitives.txt":    "os_primitive",
    "09_msg_ids_REQ_IND.txt":  "msg_id",
    "10_event_signals.txt":    "event_signal",
    "11_simP_simAT.txt":       "sim_primitive",
    "12_UI_TRACE_strings.txt": "ui_trace",
    "13_GSM_constants.txt":    "gsm_const",
    "14_WAP_WSP_WTP.txt":      "wap",
    "15_HW_states.txt":        "hw_state",
    "16_hw_functions.txt":     "hw_func",
    "17_func_traces.txt":      "func_trace",
    "18_state_names.txt":      "state_name",
    "20_format_strings.txt":   None,  # Skip — printf format templates, not symbol names
    "21_misc_upper_snake.txt": "event",   # Mixed bucket of event/enum/msg names
    "22_misc_lower_snake.txt": "fn",      # Function names from debug traces
    "23_ascii_messages.txt":   None,      # English UI strings — captured separately if useful
    "24_function_trace.txt":   "fn",      # function names extracted from FUNCTION_TRACE: prefix strings
}

VALID_NAME = re.compile(r"^[A-Za-z_][A-Za-z_0-9]*$")

ADDR_LINE_RE = re.compile(
    r"^\s+([A-Za-z_][A-Za-z_0-9]*)\s*:\s*\{\s*addr:\s*(0x[0-9A-Fa-f]+)"
)


def existing_named(build: str) -> set:
    """Names already addressed in overlay.yaml or nokix.yaml — skip on import."""
    out = set()
    for suffix in ("nokix", "overlay"):
        path = HERE / f"{build}.{suffix}.yaml"
        if not path.exists():
            continue
        for line in path.read_text().splitlines():
            m = ADDR_LINE_RE.match(line)
            if m:
                out.add(m.group(1))
    return out


def main():
    build = sys.argv[1] if len(sys.argv) > 1 else "3310-v579"
    out_path = HERE / f"{build}.canonical.yaml"

    already = existing_named(build)
    by_category = {}  # category -> set of names
    by_file_count = {}

    for fname, cat in sorted(CATEGORIES.items()):
        if cat is None:
            continue
        p = SRC / fname
        if not p.exists() or p.stat().st_size == 0:
            continue
        n_in_file = 0
        for line in p.read_text().splitlines():
            line = line.strip()
            if not line:
                continue
            if not VALID_NAME.match(line):
                continue  # skip strings with spaces / non-identifier chars
            n_in_file += 1
            by_category.setdefault(cat, set()).add(line)
        by_file_count[fname] = (cat, n_in_file)

    # Dedup across categories — first occurrence wins. Order categories so the
    # most specific (task, timer, mailbox) come before the generic (event, fn).
    PRIORITY = [
        "task", "mailbox", "semaphore", "resource", "timer",
        "os_primitive", "msg_id", "event_signal", "sim_primitive",
        "appl_state", "state_name", "hw_state", "hw_func",
        "gsm_const", "wap", "ui_trace", "source_file", "func_trace",
        "event", "fn",   # generic buckets last
    ]
    chosen = {}  # name -> category
    for cat in PRIORITY:
        for n in sorted(by_category.get(cat, [])):
            if n in chosen:
                continue
            chosen[n] = cat

    # Drop names that already have an address in overlay/nokix.
    pending = {n: c for n, c in chosen.items() if n not in already}
    skipped_resolved = len(chosen) - len(pending)

    by_cat_pending = {}
    for n, c in pending.items():
        by_cat_pending.setdefault(c, []).append(n)
    for c in by_cat_pending:
        by_cat_pending[c].sort()

    # Write the file. Stable per-category ordering for diffability.
    with open(out_path, "w") as f:
        f.write(
            f"# Canonical 3330 symbol names — vocabulary we expect to find in 3310.\n"
            f"# Address-less stubs. When scratch.py adds an address for a name in here,\n"
            f"# `promote` moves the entry into overlay.yaml AND removes it from this file.\n"
            f"#\n"
            f"# Source: re/3330/symbols/ (extracted from 3330.exe debug-trace strings +\n"
            f"# lookup tables). Auto-generated by tools/symbols/import_3330_canonical.py.\n"
            f"# Last imported: {datetime.now():%Y-%m-%d %H:%M}\n"
            f"# Total pending: {len(pending)}   (skipped {skipped_resolved} already addressed)\n"
            f"#\n"
            f"# Format identical to overlay.yaml's `addresses:` (so lookup.py can re-use\n"
            f"# the same parser) but with `addr: 0` sentinel + `category:` annotation.\n"
            "\n"
            f"firmware:\n"
            f"  id: b2d2c20e\n"
            f"  model: 3310\n"
            f"  version: 05.79\n"
            "\n"
            f"canonical:\n"
        )
        for cat in PRIORITY:
            names = by_cat_pending.get(cat, [])
            if not names:
                continue
            # Find source file(s) for this category for the comment header.
            srcs = [k for k, (c, _n) in by_file_count.items() if c == cat]
            src_str = ", ".join(srcs)
            f.write(f"\n  # ---- {cat} ({len(names)} pending; src: {src_str}) ----\n")
            for name in names:
                f.write(f"  {name+':':40s} {{ category: {cat} }}\n")

    print(f"[import_3330_canonical] wrote {len(pending)} canonical names to")
    print(f"  {out_path}")
    print(f"  ({skipped_resolved} names already addressed in overlay/nokix — skipped)")
    print("  per-category breakdown:")
    for cat in PRIORITY:
        n = len(by_cat_pending.get(cat, []))
        if n:
            print(f"    {cat:18s} {n:4d}")


if __name__ == "__main__":
    main()
