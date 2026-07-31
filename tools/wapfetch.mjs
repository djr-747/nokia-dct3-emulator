// Host side of the emulated WAP gateway: fetch a real URL and hand the phone
// something its 2002 browser can render.
//
// The gateway inside the emulator (src/mad2/dsp/wap.c) cannot open sockets from
// wasm, so when the browser asks for a URL the gateway parks the WTP
// transaction and exposes the URI here. We fetch it, transcode whatever comes
// back into WML 1.1, compile that to WBXML, and hand the bytes back. This is
// exactly the job a real WAP gateway did: phones spoke WSP/WBXML, the gateway
// spoke HTTP and compiled on the fly.
//
// Scope: enough transcoding to browse simple pages — headings, paragraphs,
// links, list items. Not a full HTML renderer, and deliberately not one.

import { execFileSync } from 'child_process';
import { readFileSync, unlinkSync } from 'fs';
import { tmpdir } from 'os';
import { join } from 'path';
import { compileWml } from './wmlc.mjs';

export const WSP_CT_WMLC = 0x14;   // application/vnd.wap.wmlc

// WSP well-known content types (WAP-230) the 3410 advertises in its Connect
// Accept: wmlc, wmlscriptc, wbmp, gif, text/plain. Anything NOT in this table
// still ships — it goes out as a textual content type instead (see
// wap_deliver_ct), which is the only way to serve a vendor type like
// application/vnd.nokia.ringing-tone, since WSP never assigned it a number.
export const WSP_CT = {
  'text/plain':                     0x03,
  'text/vnd.wap.wml':               0x08,
  'application/vnd.wap.wmlc':       0x14,
  'application/vnd.wap.wmlscriptc': 0x15,
  'image/gif':                      0x1d,
  'image/jpeg':                     0x1e,
  'image/png':                      0x20,
  'image/vnd.wap.wbmp':             0x21,
};

// Content we must not "transcode": it is already what the phone asked for.
// Anything textual/markup goes through the WML path; everything else — images,
// ringtones, whatever a WAP site serves — is handed over byte for byte.
export const isOpaqueType = (ct) =>
  !!ct && !/^text\/|\bhtml\b|\bxml\b|vnd\.wap\.wml($|[^c])/i.test(ct);

// ── WBXML (WAP-192 §14), WML 1.1 code page 0 ──
const T = { wml: 0x3f, card: 0x27, p: 0x20, br: 0x26, a: 0x1c, big: 0x25, b: 0x24 };
// Attribute-start tokens, WML 1.1 code page 0. The three href forms sit at
// 0x4A/0x4B/0x4C — mirroring src at 0x35/0x36/0x37. Getting these wrong is
// quiet: the anchor still renders (the label is element content) but carries no
// usable target, so activating it does nothing at all.
// title is 0x36. It was recorded here as 0x3b for a long time, which is
// actually type="password" — the token is never emitted by compileDeck (the
// deck title is rendered as bold text), so nothing ever broke. Corrected
// against libwbxml's WML 1.1 table; see tools/wmlc.mjs for the provenance.
const A = { href: 0x4a, hrefHttp: 0x4b, hrefHttps: 0x4c, title: 0x36 };
const CONTENT = 0x40, ATTRS = 0x80, END = 0x01, STR_I = 0x03;

// Inline string, encoded as UTF-8 to match the charset the deck declares. The
// text has already been folded to the phone's Latin-1 repertoire, so this is
// ASCII plus the occasional two-byte accented character.
const str = (s) => [STR_I, ...Buffer.from(s, 'utf8'), 0x00];

// Attribute value: the token table has prefixes for the two common schemes, so
// "http://x" costs one byte plus the remainder rather than the whole string.
function hrefValue(url) {
  if (url.startsWith('http://'))  return [A.hrefHttp,  ...str(url.slice(7))];
  if (url.startsWith('https://')) return [A.hrefHttps, ...str(url.slice(8))];
  return [A.href, ...str(url)];
}

// Period phones had a hard ceiling on deck size, so gateways and portals split
// long pages server-side and left an explicit "More" link for the next chunk —
// that visible pagination is what users of the era actually saw, not any
// protocol-level splitting (WTP segmentation is invisible when it happens).
// The ceiling here is deliberately conservative: one deck, one datagram.
export const DECK_LIMIT = 1100;

// Split a node list into pages that each compile to under DECK_LIMIT bytes,
// leaving room for the "More" link the page will carry.
export function paginate(title, nodes, limit = DECK_LIMIT) {
  const pages = [];
  let cur = [];
  const size = (ns) => compileDeck(title, ns).length;
  for (const n of nodes) {
    cur.push(n);
    if (size(cur) > limit - 60) {          // 60B headroom for the More link
      const spill = cur.pop();
      if (!cur.length) cur = [spill];      // a single oversized node: let it ride
      else { pages.push(cur); cur = [spill]; }
    }
  }
  if (cur.length) pages.push(cur);
  return pages.length ? pages : [[{ t: 'text', v: '(empty page)' }]];
}

// nodes: {t:'text',v} | {t:'br'} | {t:'link',href,label}
export function compileDeck(title, nodes) {
  // WBXML 1.1, WML 1.1 public id, UTF-8 (MIBenum 106), no string table.
  // MEASURED: this phone renders decks declaring 0x6A and answers 0x04
  // (ISO-8859-1) with "file format unknown", despite listing ISO-8859-1 in its
  // own Accept-Charset — that header is about HTTP content, not the WBXML
  // charset field. Text is therefore encoded as real UTF-8 below; emitting
  // single-byte text under this declaration is what broke rendering before,
  // since one invalid sequence fails the entire document.
  const out = [0x01, 0x04, 0x6a, 0x00];
  out.push(T.wml | CONTENT);
  out.push(T.card | CONTENT);
  out.push(T.p | CONTENT);
  if (title) { out.push(T.b | CONTENT, ...str(title), END, T.br); }
  for (const n of nodes) {
    if (n.t === 'text') out.push(...str(n.v));
    else if (n.t === 'br') out.push(T.br);
    else if (n.t === 'link') {
      out.push(T.a | CONTENT | ATTRS);
      out.push(...hrefValue(n.href));
      out.push(END);                            // end of attribute list
      out.push(...str(n.label));
      out.push(END);                            // </a>
    }
  }
  out.push(END, END, END);                      // </p></card></wml>
  return Buffer.from(out);
}

const ENT = { amp: '&', lt: '<', gt: '>', quot: '"', apos: "'", nbsp: ' ', '#39': "'", '#160': ' ' };
const decodeEntities = (s) => s.replace(/&(#?\w+);/g, (m, e) => {
  if (ENT[e]) return ENT[e];
  if (e[0] === '#') return String.fromCharCode(parseInt(e.slice(1), 10) || 32);
  return m;
});
// The 3410's font is Latin-1. Fold the common typographic characters to their
// ASCII equivalents rather than dropping them (losing an apostrophe mid-word
// reads as a typo), then discard anything still outside the encoding.
const toLatin1 = (s) => s.replace(/[‘’‚′]/g, "'")
                         .replace(/[“”„]/g, '"')
                         .replace(/[–—]/g, '-')
                         .replace(/…/g, '...')
                         .replace(/[   ]/g, ' ')
                         .replace(/[^\x20-\x7e\xa0-\xff]/g, '');

// Turn HTML into the small node list the deck compiler understands.
export function htmlToNodes(html, baseUrl, maxChars = 1400) {
  const body = html.replace(/<!--[\s\S]*?-->/g, '')
                   .replace(/<![^>]*>/g, '')          // doctype and friends
                   .replace(/<\?[\s\S]*?\?>/g, '')    // xml prolog
                   .replace(/<(script|style|head)[\s\S]*?<\/\1>/gi, '');
  const titleM = html.match(/<title[^>]*>([\s\S]*?)<\/title>/i);
  const title = titleM ? toLatin1(decodeEntities(titleM[1])).trim().slice(0, 40) : '';

  const nodes = [];
  let chars = 0, pendingBreak = false;
  const pushText = (t) => {
    t = toLatin1(decodeEntities(t)).replace(/\s+/g, ' ');
    if (!t.trim()) return;
    if (chars + t.length > maxChars) t = t.slice(0, Math.max(0, maxChars - chars));
    if (!t) return;
    if (pendingBreak) { nodes.push({ t: 'br' }); pendingBreak = false; }
    nodes.push({ t: 'text', v: t });
    chars += t.length;
  };

  // Walk tags in order so links keep their place in the text flow.
  const re = /<(\/?)(\w+)([^>]*)>|([^<]+)/g;
  let m, linkHref = null, linkText = '';
  while ((m = re.exec(body)) && chars < maxChars) {
    const [, close, tagRaw, attrs, text] = m;
    if (text !== undefined) { if (linkHref !== null) linkText += text; else pushText(text); continue; }
    const tag = tagRaw.toLowerCase();
    if (tag === 'a' && !close) {
      const h = attrs.match(/href\s*=\s*["']?([^"'\s>]+)/i);
      // Entity-decode the URL too: "&amp;" between query parameters is an HTML
      // escape, and passing it through literally gives the phone a dead link.
      if (h) { linkHref = decodeEntities(h[1]); linkText = ''; }
    } else if (tag === 'a' && close && linkHref !== null) {
      const label = toLatin1(decodeEntities(linkText)).replace(/\s+/g, ' ').trim().slice(0, 40);
      if (label) {
        let href = linkHref;
        try { href = new URL(linkHref, baseUrl).href; } catch { /* keep as written */ }
        if (pendingBreak) { nodes.push({ t: 'br' }); pendingBreak = false; }
        nodes.push({ t: 'link', href, label });
        chars += label.length;
        pendingBreak = true;
      }
      linkHref = null;
    } else if (/^(br|p|div|tr|li|h[1-6]|table|ul|ol)$/.test(tag)) {
      pendingBreak = true;
    }
  }
  return { title, nodes };
}

// Fetch a URL. Returns {ct, body} when the content is already phone-ready
// (pre-compiled WBXML) or body null on failure; otherwise {title, nodes, url}
// for the caller to paginate and compile.
//
// Deliberately SYNCHRONOUS. The emulator's frame loop is synchronous, and
// blocking it freezes emulated time — which means the phone's WTP retransmit
// budget (~3 s of its own clock) cannot expire while we are out on the network,
// however slow the real fetch is.
export function fetchForPhone(uri, log = () => {}) {
  let url = uri;
  if (!/^https?:\/\//i.test(url)) url = 'http://' + url.replace(/^\/+/, '');
  const hdrFile = join(tmpdir(), `wapgw-${process.pid}.hdr`);
  const bodyFile = join(tmpdir(), `wapgw-${process.pid}.bin`);
  let headers = '', buf = Buffer.alloc(0);
  // Ask for the least demanding thing a server is willing to give us:
  //  - a period User-Agent and the phone's real UAProf URL, which is the
  //    signal WAP-aware sites and transcoding proxies key on;
  //  - WML ahead of HTML in Accept;
  //  - Save-Data, which some modern CDNs honour with a lighter variant.
  // WAPGW_VIA=host:port routes through a transcoding proxy — e.g. the WML
  // proxy at 176.103.221.33:3132, which hands back real WML 1.1 (already
  // paginated) instead of a modern page we would have to strip ourselves.
  const via = process.env.WAPGW_VIA || '';
  const args = ['-sL', '--max-time', via ? '25' : '10',
                '-A', 'Nokia3410/1.0 (05.46) UP.Link/1.0',
                '-H', 'Accept: text/vnd.wap.wml;q=1.0, application/vnd.wap.wmlc;q=1.0, ' +
                      'text/plain;q=0.8, text/html;q=0.5, */*;q=0.1',
                '-H', 'x-wap-profile: "http://nds1.nds.nokia.com/uaprof/N3410r100.xml"',
                '-H', 'Save-Data: on',
                '-H', 'Accept-Charset: utf-8, iso-8859-1'];
  if (via) args.push('-x', via);
  try {
    execFileSync('curl', [...args, '-D', hdrFile, '-o', bodyFile, url], { stdio: 'ignore' });
    headers = readFileSync(hdrFile, 'latin1');
    buf = readFileSync(bodyFile);
  } catch (e) {
    log(`[wapgw] fetch failed for ${url}: ${e.message}`);
    return { ct: WSP_CT_WMLC, body: null };
  } finally {
    for (const f of [hdrFile, bodyFile]) { try { unlinkSync(f); } catch { /* already gone */ } }
  }
  const status = (headers.match(/HTTP\/[\d.]+ (\d+)/g) || []).pop() || '???';
  const ctM = headers.match(/^content-type:\s*([^\r\n]+)/im);
  const ct = (ctM ? ctM[1] : '').toLowerCase();
  log(`[wapgw] ${status} ${ct || 'no content-type'} ${buf.length}B <- ${url}`);
  if (!buf.length) return { ct: WSP_CT_WMLC, body: null };
  // Already-compiled WBXML from a real WAP site: pass it straight through.
  if (ct.includes('vnd.wap.wmlc')) return { ct: WSP_CT_WMLC, body: buf };
  // Anything that is not markup is content the phone asked for by URL — a wbmp
  // logo, a GIF, an RTTTL ringtone — and transcoding it would be nonsense.
  // Hand the bytes over untouched under their own content type. Before this,
  // every reply claimed to be a WML deck no matter what had been fetched,
  // which is why images and tone downloads could not work at all.
  const bare = ct.split(';')[0].trim();
  if (isOpaqueType(bare)) {
    const wk = WSP_CT[bare];
    if (wk !== undefined) {
      log(`[wapgw] passing ${bare} through untranscoded — ${buf.length}B`);
      return { ct: wk, ctText: bare, body: buf };
    }
    // No well-known token, so the type could only go out as text — and this
    // browser refuses a textual content type outright, dropping the whole
    // session (measured; see wap_deliver_ct). Losing the browser is far worse
    // than not showing the file, so say so on screen instead. WAPGW_TEXT_CT=1
    // sends it anyway, for anyone picking that investigation back up.
    if (process.env.WAPGW_TEXT_CT) {
      log(`[wapgw] WAPGW_TEXT_CT: sending ${bare} as a textual content type (expect a Disconnect)`);
      return { ct: 0, ctText: bare, body: buf };
    }
    log(`[wapgw] ${bare} has no well-known WSP token — reporting it instead of dropping the session`);
    return { ct: WSP_CT_WMLC, body: compileDeck('Not supported', [
      { t: 'text', v: `This phone cannot be sent "${bare}" over WAP.` },
    ]) };
  }
  // Decode with the charset the server declared (or the document's own <meta>)
  // before touching the text. Reading UTF-8 bytes as Latin-1 and passing them
  // through would emit mojibake and, worse, split multi-byte sequences.
  let enc = (ct.match(/charset=([\w-]+)/) || [])[1]
         || (buf.toString('latin1', 0, 2048).match(/charset=["']?([\w-]+)/i) || [])[1]
         || 'utf-8';
  let text;
  try { text = new TextDecoder(enc).decode(buf); }
  catch { text = buf.toString('latin1'); }        // unknown label: assume 8-bit
  // Real WML (a WAP site, or our transcoding proxy, which returns WML 1.1):
  // compile it as-is. The node transcoder below models a page as text +
  // links, so it would silently drop <input>/<anchor><go>/<postfield> — i.e.
  // every form, which is most of what a period portal consists of. Only take
  // this path when the compiled deck fits one datagram; otherwise fall
  // through, because the node path is the one that can paginate.
  if (ct.includes('vnd.wap.wml') || /<wml[\s>]/i.test(text.slice(0, 512))) {
    const deck = compileWml(text, url);
    if (deck && deck.length <= DECK_LIMIT) {
      log(`[wapgw] compiled WML directly — ${deck.length}B deck (forms preserved)`);
      return { ct: WSP_CT_WMLC, body: deck };
    }
    if (deck) log(`[wapgw] compiled WML is ${deck.length}B (> ${DECK_LIMIT}) — paginating as text instead`);
  }
  const { title, nodes } = htmlToNodes(text, url);
  if (!nodes.length) nodes.push({ t: 'text', v: '(no renderable content)' });
  return { title, nodes, url };
}

// Pagination state: the phone asks for "<url>" then "<url>?wpg=2"… and each
// request serves one deck-sized slice of the same transcoded page.
//
// This MUST be a query parameter, not a "#fragment": in WML a fragment names a
// card WITHIN the current deck, so a "#wp2" link never reaches the network —
// the browser just fails to find that card and reports the page as missing.
export const PAGE_PARAM = 'wpg';
const pageUrl = (base, n) => base + (base.includes('?') ? '&' : '?') + PAGE_PARAM + '=' + n;
export function splitPage(uri) {
  const m = uri.match(new RegExp(`^(.*?)[?&]${PAGE_PARAM}=(\\d+)$`));
  return m ? { base: m[1], page: parseInt(m[2], 10) } : { base: uri, page: 1 };
}

const pageCache = new Map();

// Serve one page of a URI, fetching (and caching) the underlying document as
// needed. Returns {ct, ctText, body} ready for the emulator, body null on
// failure. ctText is set only for types with no well-known WSP token, and the
// caller must then deliver via the textual-content-type path.
export function servePage(uri, log = () => {}, limit = DECK_LIMIT) {
  const { base, page } = splitPage(uri);

  let entry = pageCache.get(base);
  if (!entry) {
    const got = fetchForPhone(base, log);
    if (got.body === null) return { ct: WSP_CT_WMLC, body: null };
    // Pre-compiled deck, or opaque content passed through: either way it is
    // already exactly what the phone should receive, so no pagination.
    if (got.body) return { ct: got.ct, ctText: got.ctText, body: got.body };
    entry = { title: got.title, pages: paginate(got.title, got.nodes, limit), url: got.url };
    pageCache.set(base, entry);
    log(`[wapgw] transcoded into ${entry.pages.length} deck${entry.pages.length > 1 ? 's' : ''}`);
  }
  const idx = Math.min(Math.max(page, 1), entry.pages.length) - 1;
  const nodes = entry.pages[idx].slice();
  if (idx + 1 < entry.pages.length) {
    nodes.push({ t: 'br' });
    nodes.push({ t: 'link', href: pageUrl(base, idx + 2), label: `More (${idx + 2}/${entry.pages.length})` });
  }
  const title = entry.pages.length > 1 ? `${entry.title} [${idx + 1}/${entry.pages.length}]` : entry.title;
  const body = compileDeck(title, nodes);
  log(`[wapgw] serving page ${idx + 1}/${entry.pages.length} of ${base} — ${body.length}B`);
  return { ct: WSP_CT_WMLC, body };
}
