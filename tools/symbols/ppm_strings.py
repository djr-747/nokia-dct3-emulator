#!/usr/bin/env python3
"""Extract the PPM (language pack) string table from a flat DCT3 .fls.

The PPM is a chunked resource (LPCS/GSMC/FONT/TEXT/...). The TEXT chunk holds one
subchunk per language plus a leading COMM (common) subchunk; each subchunk is a
length-prefixed string blob. Two on-flash encodings occur:

  * sentinel  — `<length_array> 0xFF <strings>` (COMM and other flags==0 chunks).
  * flat      — `<length_array> <strings>` with NO 0xFF sentinel; N (the string
                count) is the unique split where sum(lengths[0:N]) + N == datalen.
                Language subchunks (flags bit 0x8) use this.

Firmware string IDs (the `sid` in NokiX get_string / a STRPTR of `id + 0x1000`)
address the COMM subchunk first, then the selected language:

    sid < comm_count            -> COMM[sid]
    sid >= comm_count           -> ENGL[sid - comm_count]

so a language string's id is its subchunk index PLUS comm_count. comm_count is
NOT a fixed constant (3310=51, 3410=37, ...) — it is derived from the COMM
subchunk here, matching NokiX GET_STRING.rx / PPMAN.rx exactly.

Output: `<id>\t<text>` per line, COMM ids first then the language. LPCS decoding
is applied when the LPCS chunk is present (English is near-identity ASCII).

Usage: ppm_strings.py <flash.fls> [--lang ENGL] [--out FILE] [--lookup ID|TEXT]
"""
import sys, argparse, re

BASE = 0x200000

def be32(d, a):
    o = a - BASE
    return (d[o] << 24) | (d[o+1] << 16) | (d[o+2] << 8) | d[o+3]

def find_ppm(d):
    """The real PPM partition: a `PPM\\0` marker in/after the PPM region whose
    version header begins "V " (the bogus `PPMVNEXT` next-pointer does not)."""
    cands = []
    for m in re.finditer(b'PPM\x00', d):
        a = m.start() + BASE
        if a < 0x300000:                     # past MCU code for every DCT3 layout
            continue
        o = m.start()
        if d[o+4:o+6] == b'V ':              # real version string "V 0x.yy\n..."
            cands.append(a)
    if not cands:
        raise SystemExit("PPM partition (with 'V ' version header) not found")
    return cands[0]

def lpcs_table(d, base):
    """The LPCS chunk's 512-byte byte->codepoint table (BE u16 per byte), or None."""
    a = base + 44
    for _ in range(24):
        ln = be32(d, a + 4); typ = d[a+8-BASE:a+12-BASE].rstrip(b'\0')
        if not typ or not typ.isalpha(): break
        if typ == b'LPCS':
            o = (a + 36) - BASE               # LPCS byte->codepoint table (PPMAN.rx a1+0x24)
            return d[o:o+512]
        a += ln + (ln & 1)
    return None

def text_subchunks(d, base):
    """Yield (name, data_addr, data_len) for every TEXT subchunk in flash order."""
    a = base + 44
    for _ in range(24):
        ln = be32(d, a + 4); typ = d[a+8-BASE:a+12-BASE].rstrip(b'\0')
        if not typ or not typ.isalpha(): break
        if typ == b'TEXT':
            a1 = a + 20
            for _ in range(80):
                t = be32(d, a1); ln1 = be32(d, a1 + 4)
                nm = d[a1+8-BASE:a1+12-BASE].rstrip(b'\0')
                if ln1 < 16 or ln1 > 0x100000: break     # terminator / junk
                if t != 0 and nm.isalpha():
                    yield nm.decode('latin1'), a1 + 16, ln1 - 16
                a1 += ln1 + ((4 - ln1 % 4) % 4)
            return
        a += ln + (ln & 1)
    raise SystemExit("TEXT chunk not found")

def split_strings(sub, dlen):
    """Split a subchunk blob into strings, auto-detecting sentinel vs flat."""
    # sentinel: first 0xFF where sum(lengths)+idx+1 == dlen
    running = 0
    for k in range(len(sub)):
        if sub[k] == 0xFF and running + k + 1 == dlen:
            lens, pos = sub[:k], k + 1
            break
        running += sub[k]
    else:
        # flat: unique N where sum(lengths[0:N]) + N == dlen
        running = 0; N = -1
        for k in range(len(sub)):
            if running + k == dlen:
                N = k; break
            running += sub[k]
        if N < 0:
            raise SystemExit("length-array split not found")
        lens, pos = sub[:N], N
    out = []
    for L in lens:
        out.append(sub[pos:pos+L]); pos += L
    return out

def decode(b, lpcs):
    out = []
    i = 0
    while i < len(b):
        c = b[i]
        if c == 0x04 and i + 2 < len(b):     # UCS-2 escape: 04 hi lo
            i += 3; continue
        cp = c
        if lpcs:
            v = (lpcs[c*2] << 8) | lpcs[c*2+1]
            if v: cp = v
        if cp == 0x0a: out.append('\\n')
        elif 0x20 <= cp < 0x7f: out.append(chr(cp))
        elif cp >= 0x80: out.append('?')
        i += 1
    return ''.join(out).strip()

def build_table(d, lang="ENGL"):
    base = find_ppm(d)
    lpcs = lpcs_table(d, base)
    subs = {nm: (a, l) for nm, a, l in text_subchunks(d, base)}
    if "COMM" not in subs:
        raise SystemExit("COMM subchunk not found")
    if lang not in subs:
        raise SystemExit("language subchunk %s not found (have: %s)"
                         % (lang, " ".join(subs)))
    def load(nm):
        a, l = subs[nm]
        blob = d[a - BASE:a - BASE + l]
        return [decode(s, lpcs) for s in split_strings(blob, l)]
    comm = load("COMM")
    langs = load(lang)
    table = {}                                # id -> text
    for i, s in enumerate(comm):
        table[i] = s
    for i, s in enumerate(langs):
        table[len(comm) + i] = s
    return table, len(comm), base

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fls")
    ap.add_argument("--lang", default="ENGL")
    ap.add_argument("--out", default=None)
    ap.add_argument("--lookup", default=None,
                    help="an id (decimal/0x) -> text, or a substring -> matching ids")
    args = ap.parse_args()
    d = open(args.fls, "rb").read()
    table, comm_count, base = build_table(d, args.lang)

    if args.lookup is not None:
        q = args.lookup
        try:
            i = int(q, 0)
            s = table.get(i)
            strptr = i + 0x1000
            print("id %d (0x%x)  STRPTR 0x%x  ->  %r" % (i, i, strptr, s))
        except ValueError:
            ql = q.lower()
            for i in sorted(table):
                if ql in table[i].lower():
                    print("id %d (0x%x)  STRPTR 0x%x  ->  %r"
                          % (i, i, i + 0x1000, table[i]))
        return

    lines = ["%d\t%s" % (i, table[i]) for i in sorted(table) if table[i]]
    txt = "\n".join(lines) + "\n"
    if args.out:
        open(args.out, "w").write(txt)
        sys.stderr.write("[ppm_strings] wrote %s (%d strings, lang=%s, comm_count=%d, ppm@0x%x)\n"
                         % (args.out, len(lines), args.lang, comm_count, base))
    else:
        sys.stdout.write(txt)

if __name__ == "__main__":
    main()
