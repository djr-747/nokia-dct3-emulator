#!/usr/bin/env python3
"""gen_trace_names.py — regenerate the DCT3 broker/MDI trace-event name table.

Extracts the bare 16-bit-id -> functional-name FACTS for the NHM-5 (3310) family
broker trace and normalizes them into our own table (tools/symbols/trace-names-nhm5.txt),
consumed by tools/trace_names.h (BROKERLOG). The output carries no GPL: only the
factual id->name mapping is transcribed; the source file's text, banners, module/
offset hints, and per-index annotations are dropped. See the output header for the
full provenance rationale.

Usage:
  tools/symbols/gen_trace_names.py [INPUT] [OUTPUT]
    INPUT  default: ref/dct3trac/nhm5_587.txt  (local, gitignored)
    OUTPUT default: tools/symbols/trace-names-nhm5.txt
"""
import sys, os, re

IN_DEFAULT  = "ref/dct3trac/nhm5_587.txt"
OUT_DEFAULT = "tools/symbols/trace-names-nhm5.txt"

HEADER = """\
# DCT3 broker / MDI trace-event names — NHM-5 (3310 family)
#
# Maps a 16-bit DCT3 trace event-id (the value the firmware hands BROKER_ROUTER /
# BROKER_DELIVER) to its functional name, e.g.  1802  MDI:m2d/CHANNEL_CONFIGURE.
# Consumed by tools/trace_names.h (BROKERLOG) — native tooling only.
#
# PROVENANCE / LICENSE: these are FACTS — the firmware's own Nokia subsystem and
# GSM/protocol message labels. They are cross-referenced against the public
# gammu/dct3trac NHM-5 dictionary (GPLv2, github.com/gammu/gammu), but ONLY the
# bare id->name mapping is transcribed and normalized here; gammu's file text,
# structure, category banners, module/offset hints and per-index annotations are
# NOT reproduced. A list of short functional names keyed by id is not copyrightable
# expression (cf. Feist v. Rural), so this file carries no GPL obligation. Where our
# own RE has independently established a name, ours takes precedence.
#
# Regenerate:  tools/symbols/gen_trace_names.py [INPUT] [OUTPUT]
# Format:  <id:4-hex> <space> <CATEGORY:sub/NAME>   (banners / comments start non-hex)
"""

# A data line is 4 hex digits, a space, then "CATEGORY:sub/NAME [annotations]".
# Category BANNER lines are "NNXX CAT:sub/..._TRA" (non-hex 'X' in the id) — skipped
# by the \A[0-9A-Fa-f]{4} anchor. We keep the NAME token only: everything up to the
# first space or '(' after the id (this drops gammu's " 0"/" 1" index suffixes and
# "(mod/off)" hints, neither of which is a fact we model).
LINE = re.compile(r"^([0-9A-Fa-f]{4}) ([^\s(]+)")

def main():
    inp = sys.argv[1] if len(sys.argv) > 1 else IN_DEFAULT
    out = sys.argv[2] if len(sys.argv) > 2 else OUT_DEFAULT
    if not os.path.exists(inp):
        sys.exit(f"gen_trace_names: input not found: {inp}\n"
                 f"  (the source dict is local/gitignored under ref/; nothing to do)")
    seen = {}
    with open(inp, encoding="utf-8", errors="replace") as f:
        for ln in f:
            m = LINE.match(ln)
            if not m:
                continue
            idv = int(m.group(1), 16)
            name = m.group(2)
            seen.setdefault(idv, name)          # first wins (file is id-ordered)
    with open(out, "w", encoding="utf-8") as f:
        f.write(HEADER)
        for idv in sorted(seen):
            f.write(f"{idv:04X} {seen[idv]}\n")
    print(f"gen_trace_names: wrote {len(seen)} id->name facts to {out}")

if __name__ == "__main__":
    main()
