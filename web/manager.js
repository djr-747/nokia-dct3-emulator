// DCT3 Firmware Manager — front-end over the harvested FIASCO source base.
//
// Data: fiasco/index.json (built by tools/fwlib_index.py) groups deduped FIASCO
// components by model -> version -> {mcu, ppm, eeprom}. We fetch only the chosen
// components, assemble a flat .fls CLIENT-SIDE (the same splat tools/fls_assemble.py
// does: each FIASCO block written at runtime_addr - 0x200000 into a 0xFF image),
// then either download it or hand it to the emulator (index.html) to boot-test.
//
// Faithful-only: we never patch bytes — only select + relocate components exactly
// as harvested. A mismatched MCU<->PPM pairing is allowed to fail (CONTACT SERVICE),
// because that's the truth the source base preserves.

const FLASH_BASE = 0x200000;
const HDR = 9;                  // FIASCO block header length
// Emulator's IndexedDB handoff (see web/main.js): write {name,bytes} then open
// index.html and the emulator restores + boots it on load. No changes to main.js.
const IDB_DB = "dct3", IDB_STORE = "fw", IDB_KEY = "lastfw";

const state = { index: null, model: null, version: null,
                mcu: null, ppm: null, eeprom: null, image: null, name: null };

const $ = (id) => document.getElementById(id);

// ---- FIASCO parse + assemble (mirrors tools/fls_assemble.py) ---------------- //
function parseFiasco(buf) {
  if (buf.length < HDR || buf[0] !== 0x0B) return null;
  const blocks = [];
  let off = 0;
  while (off + HDR <= buf.length && buf[off] === 0x0B) {
    const addr = (buf[off + 1] << 16) | (buf[off + 2] << 8) | buf[off + 3];
    const blen = buf[off + 5] | (buf[off + 6] << 8);   // little-endian length
    const ds = off + HDR, de = ds + blen;
    if (de > buf.length) throw new Error("truncated FIASCO block @0x" + addr.toString(16));
    blocks.push({ addr, data: buf.subarray(ds, de) });
    off = de;
  }
  return blocks;
}

// parts: [{ buf, raw, base }]. FIASCO components (MCU/PPM/factory .pmm) splat each
// block at addr-FLASH_BASE; raw region images (virgin EEPROM templates) splat the
// whole buffer at `base`-FLASH_BASE. Matches tools/fls_assemble.py + pmm folding.
function assemble(parts, flashSize) {
  const img = new Uint8Array(flashSize).fill(0xFF);
  let written = 0;
  const put = (data, dst) => {
    if (dst >= 0 && dst + data.length <= flashSize) { img.set(data, dst); written += data.length; }
  };
  for (const p of parts) {
    if (p.raw) {
      put(p.buf, p.base - FLASH_BASE);
    } else {
      const blocks = parseFiasco(p.buf);
      if (!blocks) throw new Error("input is not a FIASCO stream");
      for (const b of blocks) put(b.data, b.addr - FLASH_BASE);
    }
  }
  return { img, written };
}

async function fetchComponent(file) {
  const r = await fetch("fiasco/" + encodeURIComponent(file), { cache: "no-store" });
  if (!r.ok) throw new Error("fetch failed: " + file + " (" + r.status + ")");
  return new Uint8Array(await r.arrayBuffer());
}

// ---- IndexedDB handoff to the emulator -------------------------------------- //
function idb() {
  return new Promise((res, rej) => {
    const r = indexedDB.open(IDB_DB, 1);
    r.onupgradeneeded = () => r.result.createObjectStore(IDB_STORE);
    r.onsuccess = () => res(r.result);
    r.onerror = () => rej(r.error);
  });
}
async function idbPutFw(rec) {
  const db = await idb();
  await new Promise((res, rej) => {
    const t = db.transaction(IDB_STORE, "readwrite");
    t.objectStore(IDB_STORE).put(rec, IDB_KEY);
    t.oncomplete = res; t.onerror = () => rej(t.error);
  });
}

// ---- rendering -------------------------------------------------------------- //
function renderModels() {
  const el = $("models");
  el.innerHTML = "<h2>Model</h2>";
  const models = Object.keys(state.index.models).sort((a, b) => (+a || 1e9) - (+b || 1e9));
  if (!models.length) { el.innerHTML += "<div class='empty'>index is empty</div>"; return; }
  for (const m of models) {
    const mi = state.index.models[m];
    const nver = Object.keys(mi.versions).length;
    const row = document.createElement("div");
    row.className = "row" + (state.model === m ? " sel" : "");
    row.innerHTML = `<span>${m}</span><span class="meta">${mi.typ || ""} · ${nver}v</span>`;
    row.onclick = () => { state.model = m; state.version = null; resetPick(); renderModels(); renderVersions(); renderBuild(); };
    el.appendChild(row);
  }
}

function renderVersions() {
  const el = $("versions");
  el.innerHTML = "<h2>Version</h2>";
  if (!state.model) return;
  const versions = Object.keys(state.index.models[state.model].versions)
    .sort((a, b) => b.localeCompare(a, undefined, { numeric: true }));
  for (const v of versions) {
    const vi = state.index.models[state.model].versions[v];
    const row = document.createElement("div");
    row.className = "row" + (state.version === v ? " sel" : "");
    row.innerHTML = `<span>v${v}</span><span class="meta">${vi.ppm.length} ppm</span>`;
    row.onclick = () => { state.version = v; resetPick(); renderVersions(); renderBuild(); };
    el.appendChild(row);
  }
}

function resetPick() {
  state.mcu = state.ppm = state.eeprom = state.image = state.name = null;
}

const idOf = (c) => c.sha256 || c.file;          // components have sha256; EEPROMs have file

function optButton(comp, kind, labelFn) {
  const b = document.createElement("div");
  const sel = state[kind] && idOf(state[kind]) === idOf(comp);
  b.className = "opt" + (sel ? " sel" : "");
  b.innerHTML = labelFn(comp);
  b.onclick = () => {
    state[kind] = (sel && kind !== "mcu") ? null : comp;   // ppm/eeprom toggle off; mcu always set
    state.image = null;
    renderBuild();
  };
  return b;
}

function renderBuild() {
  const el = $("build");
  if (!state.model || !state.version) {
    el.innerHTML = "<div class='empty'>Pick a model, then a version.</div>"; return;
  }
  const mi = state.index.models[state.model];
  const vi = mi.versions[state.version];
  const sizeMB = mi.flash_size ? (mi.flash_size / 0x100000) + " MB" : "size unknown";
  el.innerHTML = `<h2>${state.model} · v${state.version} · ${sizeMB}</h2>`;

  // MCU (auto-select if exactly one)
  if (!state.mcu && vi.mcu.length === 1) state.mcu = vi.mcu[0];
  const mkPick = (title, kind, list, labelFn, note) => {
    const wrap = document.createElement("div"); wrap.className = "pick";
    wrap.innerHTML = `<label>${title}${note ? ` <span style="text-transform:none">— ${note}</span>` : ""}</label>`;
    const opts = document.createElement("div"); opts.className = "opts";
    if (!list.length) opts.innerHTML = "<span class='meta'>none</span>";
    list.forEach((c) => opts.appendChild(optButton(c, kind, labelFn)));
    wrap.appendChild(opts); return wrap;
  };

  const modTag = (c) => c.modded ? ` <span class="mod">⚙ MOD</span>` : "";
  el.appendChild(mkPick("MCU firmware", "mcu", vi.mcu,
    (c) => `${c.leaf || c.sha256.slice(0, 8)}${c.has_ppm ? " +PPM" : ""}${modTag(c)}`
      + `<small>${(c.bytes / 1024 | 0)} KB${c.has_ppm ? " · MCU+PPM combined" : ""}`
      + `${c.modded ? " · community-modified" : " · factory"}</small>`));

  // Some MCU-update packs ship MCU+PPM as one combined image (has_ppm) — then a
  // separate PPM pick isn't required. Otherwise PPM is required.
  const mcuHasPpm = !!(state.mcu && state.mcu.has_ppm);
  if (vi.ppm.length) {
    el.appendChild(mkPick("PPM (language pack)", "ppm",
      vi.ppm.slice().sort((a, b) => String(a.variant).localeCompare(String(b.variant))),
      (c) => `PPM ${c.variant || "?"}${modTag(c)}<small>${(c.bytes / 1024 | 0)} KB`
        + `${c.modded ? " · community-modified" : ""}</small>`,
      mcuHasPpm ? "optional — MCU image already includes a PPM; pick to override"
                : "Nokia language group; pick one"));
  } else {
    const w = document.createElement("div"); w.className = "pick";
    w.innerHTML = `<label>PPM (language pack)</label><div class="${mcuHasPpm ? 'prov' : 'out warn'}">`
      + (mcuHasPpm
          ? `✓ bundled in the MCU image (MCU+PPM combined dump for v${state.version}).`
          : `No separate PPM for v${state.version} in the library, and the MCU image isn't a `
            + `combined dump — build would have no language pack.`)
      + `</div>`;
    el.appendChild(w);
  }

  // EEPROM — make-or-break for in-flash models; ranked by per-version boot verdict.
  // A verdict is keyed by (eeprom, MCU version): the SAME EEPROM can be Menu-clean
  // on one fw version and security-code/Contact-Service on another.
  const vkey = `${state.model}_${state.version}`;
  const VRANK = { menu: 0, "": 1, "security-code": 2, "contact-service": 3, "no-boot": 4 };
  const verdictOf = (s) => (s.verdicts && s.verdicts[vkey]) || "";
  const BADGE = { menu: "✓ Menu", "security-code": "🔑 Sec-code",
                  "contact-service": "✗ Contact Svc", "no-boot": "✗ No boot", "": "? untested" };
  let eeMissing = false;
  if (mi.eeprom_in_flash) {
    const foldable = (mi.eeprom_sources || []).filter((s) => !s.external).slice()
      .sort((a, b) => VRANK[verdictOf(a)] - VRANK[verdictOf(b)]
                   || String(a.source).localeCompare(String(b.source)));
    if (foldable.length) {
      // default to the best verdict for THIS version; re-pick when version changes
      if (!state.eeprom || !foldable.some((s) => idOf(s) === idOf(state.eeprom)))
        state.eeprom = foldable[0];
      el.appendChild(mkPick("EEPROM / PMM", "eeprom", foldable,
        (c) => {
          const tag = c.source === "known-good" ? `known-good v${c.origin_version}` : c.source;
          return `${BADGE[verdictOf(c)]} · ${tag}`
            + `<small>${c.name.replace(/\.(pmm|fls)$/i, "")}</small>`;
        },
        `in-flash — folded in; verdict shown for v${state.version}`));
    } else {
      eeMissing = true;
      const w = document.createElement("div"); w.className = "pick";
      w.innerHTML = `<label>EEPROM / PMM</label><div class="out warn">No EEPROM source `
        + `for ${state.model} in ref/, firmware/eeprom/, or the known-good catalog. NVRAM `
        + `stays blank (0xFF) → expect CONTACT SERVICE.</div>`;
      el.appendChild(w);
    }
  } else {
    const w = document.createElement("div"); w.className = "pick";
    const ext = (mi.eeprom_sources || []).find((s) => s.external);
    w.innerHTML = `<label>EEPROM</label><div class="prov">External 24Cxx chip — not part `
      + `of the .fls. The emulator supplies a default for boot-testing.`
      + (ext ? ` <a href="fiasco/${encodeURIComponent(ext.file)}" download>download chip image</a>` : "")
      + `</div>`;
    el.appendChild(w);
  }

  // actions
  const act = document.createElement("div"); act.className = "actions";
  const canBuild = !!(state.mcu && (state.ppm || mcuHasPpm) && mi.flash_size);
  act.innerHTML =
    `<button class="primary" id="asm" ${canBuild ? "" : "disabled"}>Assemble .fls</button>` +
    `<button id="dlbtn" disabled>Download</button>` +
    `<button id="bootbtn" disabled>Boot in emulator ↗</button>`;
  el.appendChild(act);

  const out = document.createElement("div"); out.className = "out"; out.id = "out";
  out.textContent = canBuild ? "Ready to assemble."
    : (state.mcu ? "Select a PPM variant (or pick a combined MCU+PPM image)."
                 : "Select an MCU.");
  el.appendChild(out);

  // provenance
  const prov = document.createElement("div"); prov.className = "prov";
  const lines = [];
  if (state.mcu) lines.push(`MCU  <code>${state.mcu.sha256.slice(0, 12)}</code> ${state.mcu.addr_lo}–${state.mcu.addr_hi} · ${(state.mcu.sources || []).join(", ")}`);
  if (state.ppm) lines.push(`PPM  <code>${state.ppm.sha256.slice(0, 12)}</code> ${state.ppm.addr_lo}–${state.ppm.addr_hi} · ${(state.ppm.sources || []).join(", ")}`);
  if (state.eeprom) {
    const ee = state.eeprom, v = (ee.verdicts && ee.verdicts[`${state.model}_${state.version}`]) || "untested";
    lines.push(`EE   <code>${ee.source}${ee.origin_version ? " v" + ee.origin_version : ""}</code> `
      + `${ee.addr_lo}–${ee.addr_hi} · v${state.version} verdict: ${v}`);
  }
  prov.innerHTML = lines.join("<br>");
  el.appendChild(prov);

  $("asm").onclick = doAssemble;
}

function fmtName() {
  const typ = state.index.models[state.model].typ || "";
  const ppm = state.ppm ? (state.ppm.variant || "") : (state.mcu.has_ppm ? "PPMbundled" : "");
  const ee = state.eeprom ? " (EEPROM)" : "";
  const mod = (state.mcu.modded || (state.ppm && state.ppm.modded)) ? " [MOD]" : "";
  return `Nokia ${state.model} ${typ} v${state.version} ${ppm}${ee}${mod} [built].fls`
    .replace(/\s+/g, " ").trim();
}

async function doAssemble() {
  const out = $("out"); const mi = state.index.models[state.model];
  out.textContent = "Fetching components…";
  try {
    const parts = [{ buf: await fetchComponent(state.mcu.file), raw: false }];
    if (state.ppm)   // combined MCU+PPM images carry the PPM already; a separate pick overrides
      parts.push({ buf: await fetchComponent(state.ppm.file), raw: false });
    if (state.eeprom) {
      const eb = await fetchComponent(state.eeprom.file);
      parts.push(state.eeprom.raw
        ? { buf: eb, raw: true, base: parseInt(state.eeprom.base, 16) }
        : { buf: eb, raw: false });
    }
    const { img, written } = assemble(parts, mi.flash_size);
    state.image = img;
    state.name = fmtName();
    const fill = (100 * written / mi.flash_size).toFixed(1);
    const eeNote = state.eeprom
      ? `\nEEPROM: ${state.eeprom.source} (${state.eeprom.name})`
      : (mi.eeprom_in_flash ? `\n⚠ no EEPROM folded — NVRAM blank, likely CONTACT SERVICE` : "");
    out.innerHTML = `<span class="ok">✓ assembled ${(img.length / 0x100000)} MB`
      + ` (${fill}% written, rest 0xFF)</span>\n${state.name}`
      + `<span class="${state.eeprom || !mi.eeprom_in_flash ? "" : "warn"}">${eeNote}</span>`;
    $("dlbtn").disabled = false; $("bootbtn").disabled = false;
    $("dlbtn").onclick = doDownload;
    $("bootbtn").onclick = doBoot;
  } catch (e) {
    out.innerHTML = `<span class="warn">✗ ${e.message}</span>`;
  }
}

function doDownload() {
  const blob = new Blob([state.image], { type: "application/octet-stream" });
  const a = $("dl"); a.href = URL.createObjectURL(blob); a.download = state.name; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 4000);
}

async function doBoot() {
  const out = $("out");
  try {
    await idbPutFw({ name: state.name, bytes: state.image });
    out.innerHTML += `\n<span class="ok">→ handed to emulator, opening…</span>`;
    window.open("index.html", "_blank");
  } catch (e) {
    out.innerHTML += `\n<span class="warn">✗ boot handoff failed: ${e.message}</span>`;
  }
}

// ---- boot ------------------------------------------------------------------- //
(async function init() {
  try {
    const r = await fetch("fiasco/index.json", { cache: "no-store" });
    if (!r.ok) throw new Error("no fiasco/index.json (" + r.status + ") — run tools/fwlib_index.py and stage web/fiasco/");
    state.index = await r.json();
  } catch (e) {
    $("models").innerHTML = "<h2>Model</h2><div class='empty'>" + e.message + "</div>";
    return;
  }
  renderModels();
})();
