#!/usr/bin/env node
// web_fingerprint.mjs — content-address the immutable web assets so the browser
// can NEVER serve a stale core. For each canonical build output we compute a
// SHA-256 over its bytes, copy it to `<base>.<hash12>.<ext>`, and record the
// mapping in web/asset-manifest.json. The loader (index.html) fetches the
// manifest with `cache:"no-store"` and loads only the hashed (immutable) URLs,
// so a given URL's bytes never change → cache can't go stale.
//
// The canonical files (dct3.js / dct3.wasm / dct3.data / main.js) are LEFT in
// place untouched — native harnesses (tools/nav.mjs, tools/web_boot_test.mjs)
// load them directly from disk and are unaffected.
//
// Run by the Makefile after `emcc -o web/dct3.js`. Idempotent; prunes stale
// hashed copies from previous builds first.

import { readFileSync, writeFileSync, copyFileSync, readdirSync, unlinkSync, existsSync } from "node:fs";
import { createHash } from "node:crypto";
import { join } from "node:path";

const WEB = "web";
// Canonical assets to fingerprint. dct3.data is only present when the wasm was
// linked with --preload-file; treat it as optional.
const ASSETS = ["dct3.js", "dct3.wasm", "dct3.data", "main.js"];
const HASHED_RE = /^(?:dct3|main)\.[0-9a-f]{12}\.(?:js|wasm|data)$/;

function hash12(path) {
  return createHash("sha256").update(readFileSync(path)).digest("hex").slice(0, 12);
}

// 1. Prune stale hashed copies from prior builds.
for (const name of readdirSync(WEB)) {
  if (HASHED_RE.test(name)) unlinkSync(join(WEB, name));
}

// 2. Hash + copy each present asset to an immutable filename.
const files = {};
let wasmHash = null;
for (const name of ASSETS) {
  const src = join(WEB, name);
  if (!existsSync(src)) continue;
  const h = hash12(src);
  const dot = name.lastIndexOf(".");
  const hashed = `${name.slice(0, dot)}.${h}${name.slice(dot)}`;
  copyFileSync(src, join(WEB, hashed));
  files[name] = hashed;
  if (name === "dct3.wasm") wasmHash = h;
}

// 3. Write the manifest (the ONLY mutable URL the browser fetches; no-store).
const manifest = {
  files,
  wasmHash,
  builtAt: new Date().toISOString(),
};
writeFileSync(join(WEB, "asset-manifest.json"), JSON.stringify(manifest, null, 2) + "\n");

console.log(`[fingerprint] wasm=${wasmHash}  ${Object.values(files).join("  ")}`);
