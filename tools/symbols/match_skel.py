#!/usr/bin/env python3
"""Cross-build function matcher by instruction/call SKELETON (for distant builds).

Byte signatures (gen_sig.py) break across firmware *generations* because the compiler
picks different instructions/registers — not just different operands. But the *shape*
of a function survives: the ordered sequence of CALLS, the branch CONDITIONS, the
load/store mix, the return. So we tokenize a function entry -> `pop {..,pc}`/`bx lr`
into operand-free instruction classes and match by sequence similarity. This is
build-invariant by construction (registers, literal offsets, and call targets are all
dropped), so it locates analogues across e.g. 3310 (NHM-5) <-> 5210 (NSM-5).

Token alphabet (one per instruction; operands discarded):
  CALL          BL/BLX (32-bit)
  B<cc>         conditional branch, by condition code (cc is build-stable logic)
  JMP           unconditional branch
  LDRPC / ADR   PC-relative literal load / address
  RET           pop {..,pc} / bx lr   (walk stops)
  xNN           any other op, classed by its top opcode bits (operands dropped)

Matching: difflib ratio over the token lists, with a call-count prefilter (the number
of calls is a strong, cheap invariant). Candidate function entries in the target build
are found by their `push {..,lr}` prologue.

Use it to port names across generations:
  # name a 5210 function via its 3310 analogue:
  match_skel.py 5210.fls 0x3E4272 3310.fls --top 5
  # then lookup.py 3310-v579 <top hit> for the name

  --anchors FILE   optional "src_hex dst_hex" lines (known pairs); a candidate whose
                   call-target sequence maps consistently through these anchors is
                   boosted — turns raw shape-similarity into call-graph evidence.
"""
import sys, argparse, difflib

FLASH_BASE = 0x200000

def load(p):
    with open(p, "rb") as f:
        return f.read()

def be16(d, o):
    return (d[o] << 8) | d[o + 1]

def bl_target(d, o):
    """Decode a 32-bit Thumb BL/BLX at file offset o -> absolute target, or None."""
    h1, h2 = be16(d, o), be16(d, o + 2)
    if (h1 & 0xF800) != 0xF000 or (h2 & 0xE800) != 0xE800:
        return None
    off = ((h1 & 0x7FF) << 12) | ((h2 & 0x7FF) << 1)
    if off & (1 << 22):
        off -= (1 << 23)
    return (FLASH_BASE + o + 4 + off) & 0xFFFFFFFE

def tokens(d, addr, max_hw=80):
    """Tokenize entry->return into (token_list, [(tok_index, bl_target), ...])."""
    o = addr - FLASH_BASE
    toks, calls = [], []
    n = 0
    while n < max_hw and o + 2 <= len(d):
        h = be16(d, o)
        nxt = be16(d, o + 2) if o + 4 <= len(d) else 0
        # BL/BLX (32-bit)
        if (h & 0xF800) == 0xF000 and (nxt & 0xE800) == 0xE800:
            calls.append((len(toks), bl_target(d, o)))
            toks.append("CALL")
            o += 4; n += 2; continue
        # return -> stop
        if (h & 0xFF00) == 0xBD00 or h == 0x4770:
            toks.append("RET")
            break
        if (h & 0xF000) == 0xD000:
            cc = (h >> 8) & 0xF
            toks.append("SWI" if cc >= 0xE else "B%X" % cc)
        elif (h & 0xF800) == 0xE000:
            toks.append("JMP")
        elif (h & 0xF800) == 0x4800:
            toks.append("LDRPC")
        elif (h & 0xF800) == 0xA000:
            toks.append("ADR")
        else:
            toks.append("x%02X" % (h >> 10))   # top 6 opcode bits, operands dropped
        o += 2; n += 1
    return toks, calls

def entries(d):
    """Candidate function entries: `push {..,lr}` (B5xx) prologues."""
    out = []
    for o in range(0, len(d) - 1, 2):
        if (be16(d, o) & 0xFF00) == 0xB500:
            out.append(FLASH_BASE + o)
    return out

def load_anchors(path):
    m = {}
    with open(path) as f:
        for ln in f:
            ln = ln.split("#", 1)[0].split()
            if len(ln) >= 2:
                m[int(ln[0], 16)] = int(ln[1], 16)
    return m

def anchor_bonus(src_calls, cand_calls, anchors):
    """Fraction of src calls that (a) hit a known anchor and (b) appear, mapped, among
    the candidate's call targets — call-graph evidence on top of raw shape."""
    mapped = [anchors[t] for (_, t) in src_calls if t in anchors]
    if not mapped:
        return 0.0
    cand_t = {t for (_, t) in cand_calls}
    hit = sum(1 for t in mapped if t in cand_t)
    return hit / len(mapped)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("addr"); ap.add_argument("dst")
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--maxhw", type=int, default=80)
    ap.add_argument("--callslop", type=int, default=3, help="prefilter: max call-count diff")
    ap.add_argument("--anchors", default=None)
    ap.add_argument("--named", default=None,
                    help="`lookup.py <build> list` output; score ONLY these named fns and print names")
    args = ap.parse_args()

    named = {}   # addr -> name
    if args.named:
        with open(args.named) as f:
            for ln in f:
                parts = ln.split()
                if len(parts) >= 2 and parts[1].startswith("0x"):
                    try:
                        a = int(parts[1], 16)
                    except ValueError:
                        continue
                    if a >= FLASH_BASE and "[code]" in ln:
                        named.setdefault(a, parts[0])

    ds = load(args.src); dd = load(args.dst)
    addr = int(args.addr, 16)
    s_tok, s_calls = tokens(ds, addr, args.maxhw)
    s_ncall = sum(1 for t in s_tok if t == "CALL")
    anchors = load_anchors(args.anchors) if args.anchors else {}

    print("# src 0x%06X  %d tokens, %d calls" % (addr, len(s_tok), s_ncall))
    print("# skeleton: %s" % " ".join(s_tok[:28]) + (" ..." if len(s_tok) > 28 else ""))

    cand_addrs = sorted(named) if named else entries(dd)
    sm = difflib.SequenceMatcher(None, s_tok, None, autojunk=False)
    scored = []
    for e in cand_addrs:
        t_tok, t_calls = tokens(dd, e, args.maxhw)
        nc = sum(1 for t in t_tok if t == "CALL")
        if abs(nc - s_ncall) > args.callslop:
            continue
        sm.set_seq2(t_tok)
        r = sm.ratio()
        if anchors:
            r = 0.75 * r + 0.25 * anchor_bonus(s_calls, t_calls, anchors)
        scored.append((r, e, nc))
    scored.sort(reverse=True)

    # Confidence: the call-skeleton discriminates only when the function is call-rich
    # and the top score stands clear of the runner-up. Short/leaf fns match generically.
    top = scored[0][0] if scored else 0.0
    gap = top - (scored[1][0] if len(scored) > 1 else 0.0)
    if s_ncall >= 4 and len(s_tok) >= 14 and top >= 0.80 and gap >= 0.12:
        conf = "HIGH"
    elif s_ncall >= 2 and top >= 0.65 and gap >= 0.08:
        conf = "MED"
    else:
        conf = "LOW (short/leaf or no clear winner — treat as a hint, verify)"
    print("# confidence: %s   (top=%.0f%% gap=%.0f%% calls=%d tokens=%d)" % (
        conf, top * 100, gap * 100, s_ncall, len(s_tok)))
    print("# top %d %s in %s:" % (args.top, "named matches" if named else "candidates",
                                  args.dst.split("/")[-1]))
    for r, e, nc in scored[:args.top]:
        nm = ("  " + named[e]) if e in named else ""
        print("  0x%06X  %.0f%%  (%d calls)%s" % (e, r * 100, nc, nm))

if __name__ == "__main__":
    main()
