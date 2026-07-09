#!/usr/bin/env node
// gen_fw_manifest.mjs — build the web firmware-inventory manifest.
//
// Scans the local firmware/ + flash/ dumps (copyrighted, NEVER committed),
// parses each filename into model / version / variant / state tags, symlinks
// every image under web/fw/ so the static http.server can serve it, and writes
// web/firmware-manifest.json for the left-hand inventory column in the web UI.
//
// Both the manifest and web/fw/ are gitignored — this is a LOCAL-DEV convenience.
// The committed public demo ships no manifest, and the UI degrades to a friendly
// "run `make fw-manifest`" message when the fetch 404s.
//
//   node tools/gen_fw_manifest.mjs            # regenerate (run by `make fw-manifest`)
//
import { promises as fs } from "node:fs";
import { existsSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const SRC_DIRS = ["firmware", "firmware/custom", "flash"];  // scanned for *.fls (top level + custom-FW dir)
const WEB = path.join(ROOT, "web");
const FW_LINK_DIR = path.join(WEB, "fw");         // web/fw/<name>.fls → ../../<src>
const MANIFEST = path.join(WEB, "firmware-manifest.json");

// ── filename → structured fields ───────────────────────────────────────────
// Examples handled:
//   "Nokia 5110 NSE-1 v5.30 A (FAID off, cksum fixed).fls"
//   "Nokia 8210 NSM-3 v5.31 A (EEPROM).fls"
//   "Factory Reset 3310 NR1 v5.79.fls"   "FuBu3310 v6.39 (PPM B).fls"
//   "My 3310 NR1 v5.79.fls"   "7110 v5.01 Converted MCU+PPM D.fls"
function parseName(stem) {
  // Parenthetical state/variant tags, e.g. (EEPROM), (FAID off, cksum fixed).
  const tags = [];
  let s = stem.replace(/\(([^)]*)\)/g, (_, t) => {
    t.split(",").forEach((x) => { const v = x.trim(); if (v) tags.push(v); });
    return "";
  });
  const version = (s.match(/v\s?(\d+\.\d+)/i) || [])[1] || "";
  const family = (s.match(/\b(N[A-Z]{2}-\d+[A-Z]?|NR\d|NME-?\d)\b/) || [])[1] || "";
  // Model number: first 3–4 digit run (optionally trailing 'i'), excluding the
  // version digits (those follow a 'v'). Strip the version token first.
  const noVer = s.replace(/v\s?\d+\.\d+/i, " ");
  const model = (noVer.match(/\b(\d{3,4}i?)\b/) || [])[1] || "";
  // Variant letter: a lone capital after the version (… v5.30 A …).
  const variant = (s.match(/v\s?\d+\.\d+\s+([A-Z])\b/i) || [])[1] || "";
  return { model, family, version, variant, tags };
}

// ── curated boot-state seeds (web HLE core) ─────────────────────────────────
// Honest, minimal seeds from project memory so the column isn't all-grey on
// first view. Live in-browser runs OVERRIDE these and persist per-image. Keys
// are substrings matched against the file stem; first match wins.
//   boots   — known to reach standby in the web (HLE) build
//   no-boot — known to wedge (CONTACT SERVICE / reset) without provisioning
//   cosim   — needs the native DSP co-sim (5110/MAD2); web HLE differs
const SEED = [
  ["3310 NR1 v5.79", "boots"],
  ["FuBu3310 v6.39", "boots"],
  ["3310 NHM-5 v6.39", "boots"],
  ["3410 NHM-2 v5.46 A (FAID off, cksum fixed)", "boots"],
  ["3410", "no-boot"],                 // plain/assembled 3410 → CONTACT SERVICE
  ["8210 NSM-3 v5.31 A (EEPROM)", "boots"],
  ["8250 NSM-3D v6.02", "boots"],
  ["8850 NSM-2 v5.31", "boots"],
  ["5210 NSM-5 v5.40 A (EEPROM)", "boots"],
  ["5110", "cosim"],                   // MAD2 DSP co-sim is native-only
];
function seedState(stem) {
  for (const [needle, state] of SEED) if (stem.includes(needle)) return state;
  return null;
}

function isEeprom(stem, tags) {
  return /EEPROM/i.test(stem) || tags.some((t) => /EEPROM/i.test(t));
}

const MODEL_SORT = (m) => parseInt(m, 10) || 9999;

async function main() {
  // Fresh symlink dir each run (stale links removed if a dump is deleted).
  await fs.rm(FW_LINK_DIR, { recursive: true, force: true });
  await fs.mkdir(FW_LINK_DIR, { recursive: true });

  const items = [];
  const usedLinks = new Map();
  for (const dir of SRC_DIRS) {
    const abs = path.join(ROOT, dir);
    if (!existsSync(abs)) continue;
    let entries;
    try { entries = await fs.readdir(abs); } catch { continue; }
    for (const fname of entries.sort()) {
      if (!fname.toLowerCase().endsWith(".fls")) continue;
      if (fname.toLowerCase().endsWith(".patched.fls")) continue;   // diagnostic, transient
      const rel = path.join(dir, fname);
      let size;
      try { size = (await fs.stat(path.join(abs, fname))).size; } catch { continue; }
      const stem = fname.replace(/\.fls$/i, "");
      const f = parseName(stem);

      // Symlink name: basename, de-duplicated across the two source dirs.
      let linkName = fname;
      if (usedLinks.has(linkName)) linkName = `${dir}__${fname}`;
      usedLinks.set(linkName, true);
      const linkPath = path.join(FW_LINK_DIR, linkName);
      const target = path.relative(FW_LINK_DIR, path.join(abs, fname));   // ../../firmware/X.fls
      await fs.symlink(target, linkPath);

      items.push({
        id: rel.replace(/[^A-Za-z0-9]+/g, "-").toLowerCase(),
        name: stem,
        model: f.model || "?",
        modelSort: MODEL_SORT(f.model),
        family: f.family,
        version: f.version,
        variant: f.variant,
        tags: f.tags,
        sizeMB: +(size / 1048576).toFixed(1),
        eeprom: isEeprom(stem, f.tags),
        seedState: seedState(stem),
        url: "fw/" + encodeURIComponent(linkName),
        src: rel,
      });
    }
  }

  // Group/sort: by model number, then version, then name.
  items.sort((a, b) =>
    a.modelSort - b.modelSort ||
    a.version.localeCompare(b.version, undefined, { numeric: true }) ||
    a.name.localeCompare(b.name));

  const manifest = {
    generatedAt: new Date().toISOString(),
    count: items.length,
    note: "Local-dev firmware inventory; not committed (firmware is copyrighted).",
    items,
  };
  await fs.writeFile(MANIFEST, JSON.stringify(manifest, null, 2));
  console.log(`[fw-manifest] ${items.length} images → ${path.relative(ROOT, MANIFEST)} ` +
              `(+ ${items.length} symlinks in ${path.relative(ROOT, FW_LINK_DIR)}/)`);
}

main().catch((e) => { console.error("[fw-manifest] failed:", e); process.exit(1); });
