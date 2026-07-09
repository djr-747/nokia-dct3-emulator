#!/usr/bin/env python3
"""Extract the PPM (language pack) string table from a flat DCT3 .fls.

The PPM is a chunked resource (LPCS/GSMC/FONT/TEXT/...). The TEXT chunk holds one
subchunk per language (ENGL/POLI/HUNG/...); each is `<length_array> 0xFF <strings>`,
the strings stored as LPCS-codepage bytes (byte -> Unicode codepoint via a 512-byte
table). For English the LPCS is near-identity, so the strings read as plain ASCII.
This is the same format our tools/nokix/nokix.c `dumplang` command implements; ported
here as a self-contained static extractor so disfw can annotate get_string() calls.

Output: `<id>\t<text>` per line. id = (string index within the language subchunk) +
--base (default 51 — get_string maps `id<51` specially, else `id-51` into this table;
PPM_STRING_LOOKUP 0x2BBF00). Verify the base against a known site if in doubt.

Usage: ppm_strings.py <flash.fls> [--lang ENGL] [--base 51] [--out FILE]
"""
import sys, argparse, re

BASE = 0x200000

def be32(d, a):
    o = a - BASE
    return (d[o] << 24) | (d[o+1] << 16) | (d[o+2] << 8) | d[o+3]

def chunk_data(d, a):
    return d[a - BASE:]

def find_ppm(d):
    # PPM partition header marker; take the first in/after the MCU region.
    for m in re.finditer(b'PPM\x00', d):
        a = m.start() + BASE
        if a >= 0x300000:        # past MCU code for every DCT3 layout
            return a
    raise SystemExit("PPM partition marker not found")

def lpcs_table(d, base):
    """The LPCS chunk's 512-byte byte->codepoint table (BE u16 per byte), or None."""
    a = base + 44
    for _ in range(16):
        ln = be32(d, a + 4); typ = d[a+8-BASE:a+12-BASE].rstrip(b'\0')
        if not typ.isalpha(): break
        if typ == b'LPCS':
            t = a + 12                          # after the 12-byte subchunk header
            o = t - BASE
            return d[o:o+512]
        a += ln + (ln & 1)
    return None

def lang_subchunk(d, base, lang):
    a = base + 44
    for _ in range(16):
        ln = be32(d, a + 4); typ = d[a+8-BASE:a+12-BASE].rstrip(b'\0')
        if not typ.isalpha(): break
        if typ == b'TEXT':
            a1 = a + 20
            for _ in range(64):
                sid = be32(d, a1)
                if sid == 0: break
                ln1 = be32(d, a1 + 4); typ1 = d[a1+8-BASE:a1+12-BASE].rstrip(b'\0')
                if typ1 == lang.encode():
                    return a1 + 16, ln1 - 16     # data start, len (after 16B header: id/len/type/flags)
                al = ln1 + ((4 - ln1 % 4) % 4)
                a1 += al
        a += ln + (ln & 1)
    raise SystemExit("language subchunk %s not found" % lang)

def decode(b, lpcs):
    """Decode one string's LPCS bytes to text (printable; \\n kept as space)."""
    out = []
    i = 0
    while i < len(b):
        c = b[i]
        if c == 0x04 and i + 2 < len(b):     # UCS-2 escape: 04 hi lo (rare in UI text)
            i += 3; continue
        cp = c
        if lpcs:
            v = (lpcs[c*2] << 8) | lpcs[c*2+1]
            if v: cp = v
        if cp == 0x0a: out.append(' ')
        elif 0x20 <= cp < 0x7f: out.append(chr(cp))
        elif cp >= 0x80: out.append('?')
        i += 1
    return ''.join(out).strip()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("fls")
    ap.add_argument("--lang", default="ENGL")
    ap.add_argument("--base", type=int, default=51)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    d = open(args.fls, "rb").read()
    base = find_ppm(d)
    # English decodes near-identity (plain ASCII); pass-through. LPCS wiring optional.
    lpcs = None
    da, dl = lang_subchunk(d, base, args.lang)
    data = d[da - BASE: da - BASE + dl]
    # length array up to the 0xFF sentinel at position N where sum(d[0:N])+N+1 == dl
    sent, running = -1, 0
    for k in range(len(data)):
        if data[k] == 0xFF and running + k + 1 == dl:
            sent = k; break
        running += data[k]
    if sent < 0:
        raise SystemExit("length-array sentinel not found")
    lens = data[:sent]
    pos = sent + 1
    lines = []
    for i, L in enumerate(lens):
        s = decode(data[pos:pos+L], lpcs)
        pos += L
        if s:
            lines.append("%d\t%s" % (args.base + i, s))
    txt = "\n".join(lines) + "\n"
    if args.out:
        open(args.out, "w").write(txt)
        sys.stderr.write("[ppm_strings] wrote %s (%d strings, lang=%s, lpcs=%s)\n"
                         % (args.out, len(lines), args.lang, "yes" if lpcs else "no"))
    else:
        sys.stdout.write(txt)

if __name__ == "__main__":
    main()
