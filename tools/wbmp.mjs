// GIF -> WBMP, because the handset does not really do GIF.
//
// The 3410 advertises image/gif (0x1D) in its WSP Accept, and passing a GIF
// through on the strength of that produces a rectangle of solid black —
// measured, with two different 2-colour 72x14 operator logos from
// wap.blamba.de. It renders WBMP perfectly, so the fix is the one a period
// gateway used: gateways transcoded images to WBMP precisely because phones'
// own decoders were absent or unreliable.
//
// Scope is deliberately narrow: enough GIF to handle the small paletted images
// WAP sites serve. No animation (first frame only), no local colour tables
// beyond the first frame's, and anything unexpected returns null so the caller
// can fall back rather than ship a broken image.

// LZW as GIF uses it: LSB-first bit packing, variable code width growing from
// minCodeSize+1, with explicit clear and end codes.
function lzwDecode(data, minCodeSize, expected) {
  const clearCode = 1 << minCodeSize;
  const endCode = clearCode + 1;
  let codeSize = minCodeSize + 1;
  let dict = [];
  const resetDict = () => {
    dict = new Array(clearCode + 2);
    for (let i = 0; i < clearCode; i++) dict[i] = [i];
    codeSize = minCodeSize + 1;
  };
  resetDict();

  const out = [];
  let bitPos = 0, prev = null;
  const readCode = () => {
    let v = 0;
    for (let i = 0; i < codeSize; i++) {
      const byte = bitPos >> 3;
      if (byte >= data.length) return -1;
      v |= ((data[byte] >> (bitPos & 7)) & 1) << i;
      bitPos++;
    }
    return v;
  };

  for (;;) {
    const code = readCode();
    if (code < 0 || code === endCode) break;
    if (code === clearCode) { resetDict(); prev = null; continue; }

    let entry;
    if (code < dict.length && dict[code]) entry = dict[code];
    else if (prev) entry = prev.concat(prev[0]);          // the KwKwK case
    else return null;

    for (const px of entry) out.push(px);
    if (out.length > expected * 2) return null;           // runaway: malformed

    if (prev) {
      dict.push(prev.concat(entry[0]));
      if (dict.length === (1 << codeSize) && codeSize < 12) codeSize++;
    }
    prev = entry;
  }
  return out;
}

// Multi-byte integer, as WBMP uses for the dimensions (same encoding as WSP's
// uintvar): 7 bits per octet, high bit set on all but the last.
function uintvar(v) {
  const tmp = [];
  do { tmp.unshift(v & 0x7f); v >>= 7; } while (v);
  for (let i = 0; i < tmp.length - 1; i++) tmp[i] |= 0x80;
  return tmp;
}

// Pack 1-bit rows into WBMP type 0. Bit set = white, clear = black, rows
// padded to a byte boundary — the convention the phone's renderer expects.
function packWbmp(w, h, isInk) {
  const out = [0x00, 0x00, ...uintvar(w), ...uintvar(h)];
  const stride = (w + 7) >> 3;
  for (let y = 0; y < h; y++) {
    const row = new Uint8Array(stride).fill(0xff);        // start all white
    for (let x = 0; x < w; x++) {
      if (isInk(x, y)) row[x >> 3] &= ~(0x80 >> (x & 7)); // ink -> clear the bit
    }
    out.push(...row);
  }
  return Buffer.from(out);
}

// Decode the first frame of a GIF and re-encode it as WBMP. Returns null if
// the image is anything this does not confidently understand.
export function gifToWbmp(buf) {
  try {
    if (buf.length < 14) return null;
    const magic = buf.toString('latin1', 0, 6);
    if (magic !== 'GIF87a' && magic !== 'GIF89a') return null;

    const flags = buf[10];
    const gctSize = flags & 0x80 ? 2 ** ((flags & 7) + 1) : 0;
    let i = 13;
    let palette = [];
    for (let k = 0; k < gctSize; k++, i += 3) palette.push([buf[i], buf[i + 1], buf[i + 2]]);

    // Walk blocks to the first image descriptor, stepping over extensions.
    for (;;) {
      if (i >= buf.length) return null;
      const block = buf[i];
      if (block === 0x2c) break;                          // image descriptor
      if (block === 0x3b) return null;                    // trailer: no image
      if (block === 0x21) {                               // extension
        i += 2;                                           // marker + label
        while (i < buf.length && buf[i] !== 0) i += buf[i] + 1;
        i++;
        continue;
      }
      return null;                                        // not something we know
    }

    const iw = buf[i + 5] | (buf[i + 6] << 8);
    const ih = buf[i + 7] | (buf[i + 8] << 8);
    const lflags = buf[i + 9];
    const interlaced = !!(lflags & 0x40);
    i += 10;
    if (lflags & 0x80) {                                  // local colour table wins
      const n = 2 ** ((lflags & 7) + 1);
      palette = [];
      for (let k = 0; k < n; k++, i += 3) palette.push([buf[i], buf[i + 1], buf[i + 2]]);
    }
    if (!iw || !ih || !palette.length) return null;
    if (iw > 640 || ih > 640) return null;                // far past anything this screen shows

    const minCodeSize = buf[i++];
    if (minCodeSize < 2 || minCodeSize > 8) return null;
    const parts = [];
    while (i < buf.length && buf[i] !== 0) {
      const n = buf[i++];
      parts.push(buf.subarray(i, i + n));
      i += n;
    }
    const pixels = lzwDecode(Buffer.concat(parts), minCodeSize, iw * ih);
    if (!pixels || pixels.length < iw * ih) return null;

    // Which palette entry is ink? Darker by luminance. Deciding per image
    // rather than assuming index 0 keeps inverted-palette images the right way
    // up — these logos come both ways round.
    const lum = ([r, g, b]) => 0.299 * r + 0.587 * g + 0.114 * b;
    const lums = palette.map(lum);
    const mid = (Math.min(...lums) + Math.max(...lums)) / 2;

    // Interlaced GIFs store rows in four passes; undo that before packing.
    let rowOf = (y) => y;
    if (interlaced) {
      const order = [];
      for (let y = 0; y < ih; y += 8) order.push(y);
      for (let y = 4; y < ih; y += 8) order.push(y);
      for (let y = 2; y < ih; y += 4) order.push(y);
      for (let y = 1; y < ih; y += 2) order.push(y);
      const map = new Array(ih);
      order.forEach((dst, src) => { map[dst] = src; });
      rowOf = (y) => map[y];
    }

    return packWbmp(iw, ih, (x, y) => {
      const idx = pixels[rowOf(y) * iw + x];
      return idx < lums.length && lums[idx] <= mid;
    });
  } catch {
    return null;                                          // malformed: caller falls back
  }
}
