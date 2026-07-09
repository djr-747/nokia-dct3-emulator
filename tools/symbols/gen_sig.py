#!/usr/bin/env python3
"""Auto-derive a masked Thumb function signature (NokiX-LOCATE style) and locate the
analogous function in other builds.

The premise (Dan): every Thumb opcode splits into build-STABLE bits (the opcode +
register fields) and build-VARIABLE bits (PC-relative literal-pool offsets and BL/BLX
call targets — these move when the image is recompiled). So we can mechanically emit a
(pattern, mask) for a function by walking its instructions from the entry up to the
terminating `pop {..,pc}` / `bx lr`, wildcarding only the variable operand bits. The
result drops straight into model.h's `Sig` (same encoding as the hand-rolled
src/models/mad2_sigs.c) and into sig_find().

Masking rules (match the existing hand-written sigs):
  BL/BLX (32-bit)            F8 00 F8 00   wildcard the 22-bit call offset
  LDR Rd,[PC,#imm8]          FF 00         wildcard the literal-pool offset (pool moves)
  ADD Rd,PC,#imm8 (ADR)      FF 00         "
  B<cond> #imm8              FF 00         wildcard the branch displacement (layout drift)
  B #imm11 (uncond)          F8 00         "
  everything else            FF FF         opcode + registers are build-stable
Terminators (kept full, walk stops): POP{..,pc} (BDxx), BX LR (4770), POP{..} not-pc
does NOT stop. An early conditional `pop pc` before --min halfwords is treated as an
early-out and the walk continues.

Why this helps: the 3310 build has a rich symbol store (named functions); the 4 MB
bring-up builds (3410/5210/3330/5510) do not. Sig a named 3310 function, scan the new
build, and you've ported the name — the same trick mad2_sigs.c uses for ~10 functions,
now mechanical for any of the ~1000.

Usage:
  gen_sig.py <src.fls> <addr> [--max N] [--min N] [--loose] [--c NAME]
  gen_sig.py <src.fls> <addr> --find <dst.fls> [--find <dst2.fls> ...]

  <addr>     hex function entry in <src.fls> (e.g. 0x2BAC72)
  --max N    cap the walk at N halfwords (default 40)
  --min N    don't stop at a `pop pc` before N halfwords (early-out; default 6)
  --loose    also wildcard the low register nibble of data-processing ops (survives
             register-allocation drift — see the 3410 reason_setter r0/r1 swap note)
  --c NAME   also print C `pat[]`/`mask[]` arrays named NAME for pasting into a sigs TU
  --find F   after emitting, scan F for the signature and print every match address
"""
import sys, argparse

FLASH_BASE = 0x200000

def load(path):
    with open(path, "rb") as f:
        return f.read()

def be16(d, off):
    return (d[off] << 8) | d[off + 1]

def is_bl_prefix(h):      return (h & 0xF800) == 0xF000           # 11110 xxxxxxxxxxx
def is_bl_suffix(h):      return (h & 0xE800) == 0xE800           # 11111 (BL) / 11101 (BLX)
def is_pcrel_ldr(h):      return (h & 0xF800) == 0x4800           # LDR Rd,[PC,#imm8]
def is_adr(h):            return (h & 0xF800) == 0xA000           # ADD Rd,PC,#imm8
def is_cond_branch(h):    return (h & 0xF000) == 0xD000 and (h & 0x0F00) < 0x0E00  # B<cc>, not SWI/UDF
def is_uncond_branch(h):  return (h & 0xF800) == 0xE000           # B #imm11
def is_pop_pc(h):         return (h & 0xFF00) == 0xBD00           # POP {..,pc}
def is_bx_lr(h):          return h == 0x4770                      # BX LR
def is_dataproc(h):       return (h & 0xC000) == 0x0000           # shift/add/sub/mov-imm/ALU block

def hw_mask(h, nxt, loose):
    """Return (size_in_halfwords, [mask_hw...]) for the instruction at this halfword."""
    if is_bl_prefix(h) and nxt is not None and is_bl_suffix(nxt):
        return 2, [0xF800, 0xF800]
    if is_pcrel_ldr(h) or is_adr(h) or is_cond_branch(h):
        return 1, [0xFF00]
    if is_uncond_branch(h):
        return 1, [0xF800]
    if loose and is_dataproc(h):
        return 1, [0xFFF8]                 # keep opcode, wildcard low reg nibble (Rd/Rm drift)
    return 1, [0xFFFF]

def walk(d, start_addr, max_hw, min_hw, loose):
    off = start_addr - FLASH_BASE
    if off < 0 or off + 2 > len(d):
        sys.exit("address out of range for this image")
    pat, msk = [], []
    nhw = 0
    while nhw < max_hw and off + 2 <= len(d):
        h = be16(d, off)
        nxt = be16(d, off + 2) if off + 4 <= len(d) else None
        term = is_pop_pc(h) or is_bx_lr(h)
        sz, masks = hw_mask(h, nxt, loose)
        for i in range(sz):
            hw = be16(d, off + 2 * i)
            pat += [(hw >> 8) & 0xFF, hw & 0xFF]
            m = masks[i]
            msk += [(m >> 8) & 0xFF, m & 0xFF]
        off += 2 * sz
        nhw += sz
        if term and nhw >= min_hw:
            break
    return pat, msk

def scan(d, pat, msk, lo=FLASH_BASE, hi=None):
    if hi is None:
        hi = FLASH_BASE + len(d)
    n = len(pat)
    hits = []
    a = (lo - FLASH_BASE) & ~1
    end = (hi - FLASH_BASE)
    while a + n <= end and a + n <= len(d):
        ok = True
        for i in range(n):
            if (d[a + i] & msk[i]) != (pat[i] & msk[i]):
                ok = False
                break
        if ok:
            hits.append(FLASH_BASE + a)
        a += 2
    return hits

def fuzzy(d, pat, msk, thresh, top):
    """Rank every halfword-aligned position by the fraction of FIXED bytes that match.
    For distant builds where instruction SELECTION drifts (insertions/register realloc),
    exact masked match fails but the best structural candidate still scores high. Only
    counts mask==0xFF bytes (the build-stable opcode/register bytes)."""
    n = len(pat)
    fixed_idx = [i for i in range(n) if msk[i] == 0xFF]
    if not fixed_idx:
        return []
    nf = len(fixed_idx)
    scored = []
    a = 0
    while a + n <= len(d):
        m = 0
        for i in fixed_idx:
            if d[a + i] == pat[i]:
                m += 1
        frac = m / nf
        if frac >= thresh:
            scored.append((frac, FLASH_BASE + a))
        a += 2
    scored.sort(reverse=True)
    return scored[:top]

def hexbytes(b):
    return " ".join("%02X" % x for x in b)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("addr")
    ap.add_argument("--max", type=int, default=40)
    ap.add_argument("--min", type=int, default=6)
    ap.add_argument("--loose", action="store_true")
    ap.add_argument("--c", metavar="NAME", default=None)
    ap.add_argument("--find", action="append", default=[], metavar="DST")
    ap.add_argument("--fuzzy", action="store_true", help="rank best candidates in --find builds")
    ap.add_argument("--thresh", type=float, default=0.65, help="fuzzy min fixed-byte fraction")
    ap.add_argument("--top", type=int, default=8, help="fuzzy: show top N candidates")
    args = ap.parse_args()

    addr = int(args.addr, 16)
    d = load(args.src)
    pat, msk = walk(d, addr, args.max, args.min, args.loose)
    nbytes = len(pat)
    fixed = sum(1 for m in msk if m == 0xFF)

    print("# sig for 0x%06X in %s  (%d bytes, %d/%d fixed%s)" % (
        addr, args.src.split("/")[-1], nbytes, fixed, nbytes,
        ", loose" if args.loose else ""))
    print("PAT: " + hexbytes(pat))
    print("MSK: " + hexbytes(msk))

    # self-match sanity
    self_hits = scan(d, pat, msk)
    tag = "UNIQUE" if self_hits == [addr] else ("%d hits incl. self" % len(self_hits)
          if addr in self_hits else "SELF-MISS(!)")
    print("# self-scan: %s  %s" % (tag, [hex(h) for h in self_hits[:6]]))

    if args.c:
        N = args.c
        def cfmt(b):
            rows = ["    " + ", ".join("0x%02X" % x for x in b[i:i+8]) for i in range(0, len(b), 8)]
            return ",\n".join(rows)
        print("\nstatic const uint8_t %s_PAT[%d] = {\n%s };" % (N, nbytes, cfmt(pat)))
        print("static const uint8_t %s_MSK[%d] = {\n%s };" % (N, nbytes, cfmt(msk)))

    for dst in args.find:
        dd = load(dst)
        hits = scan(dd, pat, msk)
        print("\n# --find %s" % dst.split("/")[-1])
        if hits:
            for h in hits[:12]:
                print("  exact 0x%06X" % h)
            if len(hits) > 12:
                print("  ... %d more" % (len(hits) - 12))
        elif args.fuzzy:
            cands = fuzzy(dd, pat, msk, args.thresh, args.top)
            if not cands:
                print("  no exact match; no fuzzy candidate >= %.0f%%" % (args.thresh * 100))
            for frac, h in cands:
                print("  fuzzy 0x%06X  %.0f%%" % (h, frac * 100))
        else:
            print("  no match (try --fuzzy)")

if __name__ == "__main__":
    main()
