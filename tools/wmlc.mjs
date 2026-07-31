// WML 1.1 -> WBXML compiler.
//
// A WAP 1.x handset never receives text WML: the gateway compiles it. When an
// upstream serves us real WML (our transcoding proxy does — it hands back
// WML 1.1 rather than a modern page), compiling it directly preserves what the
// HTML->node->deck path in wapfetch.mjs cannot: <input> fields, <anchor><go>
// with <postfield>, and $(variable) substitution. That is the difference
// between a browsable portal and an empty card, because the era's portals are
// built almost entirely out of forms.
//
// PROVENANCE OF THE TOKEN TABLES — this matters more than the code.
// The tables below are WML 1.1 code page 0, taken from libwbxml 0.11.10
// (src/wbxml_tables.c, sv_wml11_tag_table / sv_wml11_attr_table), which is a
// reference implementation of WAP-191/WAP-192. They are NOT reconstructed from
// memory or inferred. Cross-check against the values this project had already
// proven against the real handset (see wapfetch.mjs): a=0x1C, p=0x20, b=0x24,
// big=0x25, br=0x26, card=0x27, wml=0x3F, href=0x4A, href"http://"=0x4B,
// href"https://"=0x4C — all eleven agree exactly.
//
// The one correction the cross-check produced: wapfetch.mjs carried
// `title: 0x3b`, but 0x3B is type="password"; the title attribute is 0x36.
// That token was declared and never emitted, which is why nothing broke.
//
// SAFETY RULE, applied throughout: never emit a token we cannot evidence.
// An unknown element is skipped but its children are still compiled, and an
// unknown attribute is dropped. A wrong token does not degrade gracefully —
// the phone Provider-Aborts the whole transaction — so silence beats a guess.

// ── Tag tokens, WML 1.1 code page 0 ──
const T = {
  a: 0x1c, td: 0x1d, tr: 0x1e, table: 0x1f, p: 0x20, postfield: 0x21,
  anchor: 0x22, access: 0x23, b: 0x24, big: 0x25, br: 0x26, card: 0x27,
  do: 0x28, em: 0x29, fieldset: 0x2a, go: 0x2b, head: 0x2c, i: 0x2d,
  img: 0x2e, input: 0x2f, meta: 0x30, noop: 0x31, prev: 0x32, onevent: 0x33,
  optgroup: 0x34, option: 0x35, refresh: 0x36, select: 0x37, small: 0x38,
  strong: 0x39, template: 0x3b, timer: 0x3c, u: 0x3d, setvar: 0x3e, wml: 0x3f,
};

// ── Attribute-start tokens. `v` is a value PREFIX carried by the token
// itself; the rest of the value follows as inline string. Entries without `v`
// are the generic form (whole value inline).
const ATTRS = [
  { n: 'accept-charset', tok: 0x05 },
  { n: 'align', v: 'bottom', tok: 0x06 }, { n: 'align', v: 'center', tok: 0x07 },
  { n: 'align', v: 'left', tok: 0x08 }, { n: 'align', v: 'middle', tok: 0x09 },
  { n: 'align', v: 'right', tok: 0x0a }, { n: 'align', v: 'top', tok: 0x0b },
  { n: 'align', tok: 0x52 },
  { n: 'alt', tok: 0x0c },
  { n: 'class', tok: 0x54 },
  { n: 'columns', tok: 0x53 },
  { n: 'content', v: 'application/vnd.wap.wmlc;charset=', tok: 0x5c },
  { n: 'content', tok: 0x0d },
  { n: 'domain', tok: 0x0f },
  { n: 'emptyok', v: 'false', tok: 0x10 }, { n: 'emptyok', v: 'true', tok: 0x11 },
  { n: 'format', tok: 0x12 },
  { n: 'forua', v: 'false', tok: 0x56 }, { n: 'forua', v: 'true', tok: 0x57 },
  { n: 'height', tok: 0x13 },
  { n: 'href', v: 'https://', tok: 0x4c }, { n: 'href', v: 'http://', tok: 0x4b },
  { n: 'href', tok: 0x4a },
  { n: 'hspace', tok: 0x14 },
  { n: 'http-equiv', v: 'Content-Type', tok: 0x5b },
  { n: 'http-equiv', v: 'Expires', tok: 0x5d },
  { n: 'http-equiv', tok: 0x5a },
  { n: 'id', tok: 0x55 },
  { n: 'iname', tok: 0x16 }, { n: 'ivalue', tok: 0x15 },
  { n: 'label', tok: 0x18 },
  { n: 'localsrc', tok: 0x19 },
  { n: 'maxlength', tok: 0x1a },
  { n: 'method', v: 'get', tok: 0x1b }, { n: 'method', v: 'post', tok: 0x1c },
  { n: 'mode', v: 'nowrap', tok: 0x1d }, { n: 'mode', v: 'wrap', tok: 0x1e },
  { n: 'multiple', v: 'false', tok: 0x1f }, { n: 'multiple', v: 'true', tok: 0x20 },
  { n: 'name', tok: 0x21 },
  { n: 'newcontext', v: 'false', tok: 0x22 }, { n: 'newcontext', v: 'true', tok: 0x23 },
  { n: 'onenterbackward', tok: 0x25 }, { n: 'onenterforward', tok: 0x26 },
  { n: 'onpick', tok: 0x24 }, { n: 'ontimer', tok: 0x27 },
  { n: 'optional', v: 'false', tok: 0x28 }, { n: 'optional', v: 'true', tok: 0x29 },
  { n: 'ordered', v: 'true', tok: 0x33 }, { n: 'ordered', v: 'false', tok: 0x34 },
  { n: 'path', tok: 0x2a },
  { n: 'scheme', tok: 0x2e },
  { n: 'sendreferer', v: 'false', tok: 0x2f }, { n: 'sendreferer', v: 'true', tok: 0x30 },
  { n: 'size', tok: 0x31 },
  { n: 'src', v: 'https://', tok: 0x59 }, { n: 'src', v: 'http://', tok: 0x58 },
  { n: 'src', tok: 0x32 },
  { n: 'tabindex', tok: 0x35 },
  { n: 'title', tok: 0x36 },
  { n: 'type', v: 'accept', tok: 0x38 }, { n: 'type', v: 'delete', tok: 0x39 },
  { n: 'type', v: 'help', tok: 0x3a }, { n: 'type', v: 'password', tok: 0x3b },
  { n: 'type', v: 'onpick', tok: 0x3c }, { n: 'type', v: 'onenterbackward', tok: 0x3d },
  { n: 'type', v: 'onenterforward', tok: 0x3e }, { n: 'type', v: 'ontimer', tok: 0x3f },
  { n: 'type', v: 'options', tok: 0x45 }, { n: 'type', v: 'prev', tok: 0x46 },
  { n: 'type', v: 'reset', tok: 0x47 }, { n: 'type', v: 'text', tok: 0x48 },
  { n: 'type', tok: 0x37 },
  { n: 'value', tok: 0x4d },
  { n: 'vspace', tok: 0x4e },
  { n: 'width', tok: 0x4f },
  { n: 'xml:lang', tok: 0x50 },
];

const END = 0x01, STR_I = 0x03, CONTENT = 0x40, ATTRS_F = 0x80;
// Inline variable references. The conversion a variable carries is expressed
// by WHICH token is used, so $(x:noesc) must not be emitted as $(x:escape) —
// that would change what the page does. A bare $(x) has no conversion in the
// source; escape is the right default in the URL context these appear in (a
// search term with a space has to arrive percent-encoded), and libwbxml's
// encoder has no opinion because it never implemented variables, so that
// default is our call and is recorded here deliberately.
const EXT_I = { escape: 0x40, unesc: 0x41, noesc: 0x42 };

const ENT = { amp: '&', lt: '<', gt: '>', quot: '"', apos: "'", nbsp: ' ' };
const decodeEntities = (s) => s.replace(/&(#?\w+);/g, (m, e) => {
  if (ENT[e]) return ENT[e];
  if (e[0] === '#') {
    const n = e[1] === 'x' || e[1] === 'X' ? parseInt(e.slice(2), 16) : parseInt(e.slice(1), 10);
    return Number.isFinite(n) ? String.fromCharCode(n) : m;
  }
  return m;
});

// The 3410's font is Latin-1. Fold the common typographic characters rather
// than dropping them (a missing apostrophe mid-word reads as a typo), then
// discard anything still outside the repertoire. Mirrors wapfetch.mjs.
const toLatin1 = (s) => s.replace(/[‘’‚′]/g, "'")
                         .replace(/[“”„]/g, '"')
                         .replace(/[–—]/g, '-')
                         .replace(/…/g, '...')
                         .replace(/[   ]/g, ' ')
                         .replace(/[^\x20-\x7e\xa0-\xff]/g, '');

const inlineStr = (s) => [STR_I, ...Buffer.from(s, 'utf8'), 0x00];

// Split a string into inline-string and variable-reference items. WBXML lets a
// text run or an attribute value be a SEQUENCE of these, which is exactly how
// "...?q=$(query)" is represented.
function textItems(raw) {
  const s = toLatin1(decodeEntities(raw));
  const out = [];
  let lit = '';
  for (let i = 0; i < s.length; i++) {
    if (s[i] === '$' && s[i + 1] === '$') { lit += '$'; i++; continue; }
    if (s[i] === '$' && s[i + 1] === '(') {
      const close = s.indexOf(')', i + 2);
      if (close > 0) {
        // $(name:conv) — the conversion is carried by the token choice, so
        // only the bare name goes on the wire.
        const [rawName, conv] = s.slice(i + 2, close).split(':');
        const name = rawName.trim();
        const tok = EXT_I[(conv || '').trim().toLowerCase()] ?? EXT_I.escape;
        if (name) {
          if (lit) { out.push(...inlineStr(lit)); lit = ''; }
          out.push(tok, ...Buffer.from(name, 'utf8'), 0x00);
          i = close;
          continue;
        }
      }
    }
    lit += s[i];
  }
  if (lit) out.push(...inlineStr(lit));
  return out;
}

// href/src have to be made absolute, because the phone does not know where the
// deck came from. With the home-URL substitution (`--wapgw <url>`) it believes
// it is browsing http://a.com, so a relative src="logo.wbmp" is resolved
// against a.com and fetches a stranger's 403 instead of the image. The HTML
// path has always absolutised; WML source needs the same treatment.
//
// A value carrying a $(variable) is left alone: URL parsing would percent-encode
// the "$(" and break the substitution the phone is meant to perform.
const URL_ATTRS = new Set(['href', 'src']);
function absolutise(value, baseUrl) {
  if (!baseUrl || !value || value.includes('$(')) return value;
  if (/^[a-z][a-z0-9+.-]*:/i.test(value) || value.startsWith('#')) return value;
  try { return new URL(value, baseUrl).href; } catch { return value; }
}

function encodeAttr(name, value, baseUrl) {
  const lname = name.toLowerCase();
  if (URL_ATTRS.has(lname)) value = absolutise(value, baseUrl);
  let best = null;
  for (const e of ATTRS) {
    if (e.n !== lname) continue;
    if (e.v === undefined) { if (!best) best = { e, rest: value }; continue; }
    if (value.startsWith(e.v) && (!best || !best.e.v || e.v.length > best.e.v.length)) {
      best = { e, rest: value.slice(e.v.length) };
    }
  }
  if (!best) return null;                      // unknown attribute: drop it
  return [best.e.tok, ...(best.rest ? textItems(best.rest) : [])];
}

// ── A deliberately small XML reader. WML from a gateway is machine-generated
// and regular; this handles what such output contains (declaration, doctype,
// comments, elements, attributes, text, entities) and nothing more.
function parse(src) {
  const root = { tag: '#root', attrs: [], kids: [] };
  const stack = [root];
  let i = 0;
  const text = (s) => { if (s) stack[stack.length - 1].kids.push({ text: s }); };
  while (i < src.length) {
    const lt = src.indexOf('<', i);
    if (lt < 0) { text(src.slice(i)); break; }
    text(src.slice(i, lt));
    if (src.startsWith('<!--', lt)) { const e = src.indexOf('-->', lt); i = e < 0 ? src.length : e + 3; continue; }
    if (src.startsWith('<?', lt))   { const e = src.indexOf('?>', lt);  i = e < 0 ? src.length : e + 2; continue; }
    if (src.startsWith('<!', lt))   { const e = src.indexOf('>', lt);   i = e < 0 ? src.length : e + 1; continue; }
    const gt = src.indexOf('>', lt);
    if (gt < 0) break;
    let raw = src.slice(lt + 1, gt).trim();
    const selfClose = raw.endsWith('/');
    if (selfClose) raw = raw.slice(0, -1).trim();
    if (raw.startsWith('/')) {                                    // closing tag
      const tag = raw.slice(1).trim().toLowerCase();
      for (let k = stack.length - 1; k > 0; k--) {                // tolerate unclosed children
        if (stack[k].tag === tag) { stack.length = k; break; }
      }
      i = gt + 1;
      continue;
    }
    const m = raw.match(/^([\w:-]+)/);
    if (!m) { i = gt + 1; continue; }
    const node = { tag: m[1].toLowerCase(), attrs: [], kids: [] };
    const attrRe = /([\w:-]+)\s*=\s*("([^"]*)"|'([^']*)'|([^\s"'>]+))/g;
    let am;
    while ((am = attrRe.exec(raw.slice(m[1].length))) !== null) {
      node.attrs.push([am[1], am[3] ?? am[4] ?? am[5] ?? '']);
    }
    stack[stack.length - 1].kids.push(node);
    if (!selfClose) stack.push(node);
    i = gt + 1;
  }
  return root;
}

// Whitespace in a WML deck is layout, not content: the proxy indents its
// markup, and emitting those runs verbatim wastes a tight deck budget and
// pushes text off a 96x65 screen. Collapse runs to one space, the same thing
// the browser would do.
const collapse = (s) => s.replace(/\s+/g, ' ');

function emit(node, out, baseUrl) {
  for (const kid of node.kids) {
    if (kid.text !== undefined) {
      const t = collapse(kid.text);
      if (t.trim() === '' && !/^ $/.test(t)) continue;
      const items = textItems(t);
      if (items.length) out.push(...items);
      continue;
    }
    const tok = T[kid.tag];
    if (tok === undefined) { emit(kid, out, baseUrl); continue; }   // unknown element: keep the children
    const attrs = [];
    for (const [n, v] of kid.attrs) {
      const enc = encodeAttr(n, v, baseUrl);
      if (enc) attrs.push(...enc);
    }
    const hasKids = kid.kids.length > 0;
    out.push(tok | (hasKids ? CONTENT : 0) | (attrs.length ? ATTRS_F : 0));
    if (attrs.length) { out.push(...attrs, END); }
    if (hasKids) { emit(kid, out, baseUrl); out.push(END); }
  }
}

// Compile WML source to a WBXML deck. Returns a Buffer, or null when the
// source has no <wml> root (i.e. it is not WML at all — the caller should fall
// back to transcoding it as HTML).
export function compileWml(src, baseUrl) {
  const root = parse(src);
  const findWml = (n) => {
    for (const k of n.kids || []) {
      if (k.tag === 'wml') return k;
      const deep = k.tag ? findWml(k) : null;
      if (deep) return deep;
    }
    return null;
  };
  const wml = findWml(root);
  if (!wml) return null;
  // Header: WBXML 1.1, public id 0x04 (WML 1.1), charset 0x6A (UTF-8), no
  // string table. MEASURED on this handset (see wapfetch.mjs): declaring 0x04
  // as the charset instead yields "file format unknown", and text must be real
  // UTF-8 under this declaration — one invalid sequence fails the document.
  const out = [0x01, 0x04, 0x6a, 0x00];
  const body = [];
  emit({ kids: [wml] }, body, baseUrl);
  return Buffer.from(out.concat(body));
}

export const WML_TAGS = T;
