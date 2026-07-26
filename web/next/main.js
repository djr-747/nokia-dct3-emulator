/* =================================================================
   web/next/main.js — minimal, defensive controller for the new UI.

   Design rules:
     1. The wasm emulator is shared with /web/ (same dct3.js/.wasm/.data).
        This file owns only UI + run-loop.
     2. Every entry point that could throw is wrapped. Errors surface in
        the error banner — never a silent failure, never a blank page.
     3. The frame loop has a kill-switch: repeated throws stop the loop
        and put the page into a clearly-broken-but-still-rendered state
        instead of spamming the banner 60 times per second.
     4. Optional emulator exports degrade gracefully — if dct3.js doesn't
        export a symbol, the UI feature it backs is disabled, but the
        rest of the app keeps working.
   ================================================================= */

(function () {
  "use strict";

  // ---------------------------------------------------------------
  // Error reporting. Wire BEFORE we touch anything async.
  // ---------------------------------------------------------------
  var errBanner = document.getElementById("err-banner");
  var errText   = document.getElementById("err-text");
  var errClose  = document.getElementById("err-close");
  var statusEl  = document.getElementById("status");

  // Errors stack into the same banner; we keep the most recent N so a
  // burst of failures doesn't grow the DOM unboundedly.
  var ERR_KEEP = 5;
  var errLog = [];

  function showError(label, err) {
    try {
      var msg = (err && err.stack) ? err.stack : (err && err.message) ? err.message : String(err);
      var entry = "[" + label + "] " + msg;
      errLog.push(entry);
      if (errLog.length > ERR_KEEP) errLog = errLog.slice(-ERR_KEEP);
      if (errText)   errText.textContent = errLog.join("\n\n");
      if (errBanner) errBanner.hidden = false;
      // Mirror to the console for devtools / save-page sessions.
      // eslint-disable-next-line no-console
      if (typeof console !== "undefined" && console.error) console.error(label, err);
    } catch (e) {
      // If the error handler itself fails, last-ditch alert so the
      // failure isn't completely silent. Should never happen.
      try { console.error("error-handler-failed", e, "while-handling", err); } catch (_) {}
    }
  }
  if (errClose) errClose.addEventListener("click", function () { errBanner.hidden = true; });

  window.addEventListener("error", function (e) {
    showError("window.error", e.error || e.message || e);
  });
  window.addEventListener("unhandledrejection", function (e) {
    showError("unhandledrejection", e.reason || e);
  });

  function setStatus(s) { if (statusEl) statusEl.textContent = s; }

  // Tiny wrapper: run fn, log any throw under `label`, return undefined
  // on failure. Use for one-shot init steps where we want the rest of
  // the page to keep loading.
  function tryDo(label, fn) {
    try { return fn(); }
    catch (e) { showError(label, e); return undefined; }
  }

  // ---------------------------------------------------------------
  // Welcome modal. First-load only; suppressed by localStorage flag.
  //
  // The modal is a real PRE-BOOT WALL: start() waits on welcomeReady
  // before calling C.boot(), so the phone doesn't begin executing
  // firmware until the user dismisses (or has dismissed previously).
  // This keeps "Press the green key to boot →" honest, and avoids the
  // confusing state of an already-running phone behind the overlay.
  // ---------------------------------------------------------------
  var WELCOME_KEY = "dct3.next.welcomed";
  var welcomeEl = document.getElementById("welcome");

  // Promise the boot path awaits. Resolved when the user dismisses,
  // or immediately if they've dismissed in a prior session.
  //
  // Wired three ways for resilience (any one works in isolation):
  //   1. Inline onclick="__dct3DismissWelcome()" on the button
  //      (the HTML — always-on, no binding-timing risk).
  //   2. Inline onclick on the overlay calls __dct3OverlayClick()
  //      so a click outside the card dismisses too.
  //   3. Keyboard fallback (Enter / Space / Escape).
  var welcomeReady = new Promise(function (resolve) {
    function dismissWelcome() {
      // eslint-disable-next-line no-console
      if (typeof console !== "undefined") console.log("[dct3] welcome dismissed");
      if (welcomeEl) welcomeEl.hidden = true;
      try { localStorage.setItem(WELCOME_KEY, "1"); } catch (_) {}
      resolve();
    }

    // Expose globals BEFORE any seen-check so the inline onclick=…
    // attributes on the welcome card can always find them. Calling
    // dismiss on an already-dismissed welcome is a no-op (resolve()
    // is idempotent in Promise semantics).
    window.__dct3DismissWelcome = dismissWelcome;
    window.__dct3OverlayClick = function (e) {
      // Only the overlay backdrop dismisses; the .welcome-card has
      // event.stopPropagation() so clicks inside it don't reach here.
      if (e && e.target && e.target.id === "welcome") dismissWelcome();
    };

    var seen = false;
    try { seen = localStorage.getItem(WELCOME_KEY) === "1"; }
    catch (_) { /* storage blocked — treat as unseen, never throw */ }

    if (!welcomeEl || seen) {
      if (welcomeEl) welcomeEl.hidden = true;
      resolve();
      return;
    }

    welcomeEl.hidden = false;

    // Keyboard fallback (Enter / Space / Escape) — keeps a keyboard
    // user from needing to aim a cursor at the button.
    document.addEventListener("keydown", function (e) {
      if (welcomeEl.hidden) return;
      if (e.key === "Enter" || e.key === " " || e.key === "Escape") {
        e.preventDefault();
        dismissWelcome();
      }
    });
  });

  // ---------------------------------------------------------------
  // Dev tools toggle.
  // ---------------------------------------------------------------
  var devBtn   = document.getElementById("btn-devtools");
  var devPanel = document.getElementById("devtools");
  function setDevOpen(open) {
    if (!devPanel) return;
    devPanel.hidden = !open;
    if (devBtn) devBtn.textContent = open ? "Hide developer tools" : "Show developer tools";
  }
  if (devBtn) devBtn.addEventListener("click", function () {
    setDevOpen(devPanel && devPanel.hidden);
  });

  // ---------------------------------------------------------------
  // Boot the emulator. dct3.js (Emscripten) defines a Module factory
  // on window; we call it with our locate-file override.
  // ---------------------------------------------------------------
  // dct3.js (Emscripten output) exposes its module factory under
  // `DCT3Module`. We also accept `window.Module` / `window.dct3` as
  // fallbacks in case the build switches generator settings.
  var ModuleFactory =
    window.DCT3Module || window.Module || window.dct3 || null;
  if (typeof ModuleFactory !== "function") {
    setStatus("Emulator script missing.");
    showError("init", new Error("dct3.js did not expose a Module factory (looked for DCT3Module / Module / dct3 on window). Reload may help."));
    return;
  }

  var modulePromise = ModuleFactory(
    window.__DCT3_LOCATE ? { locateFile: window.__DCT3_LOCATE } : {}
  );

  // Prefetch the default firmware BEHIND the welcome wall. The welcome gates
  // BOOTING, not downloading — on slow links the multi-MB image streams while
  // the user reads the card instead of starting only when they dismiss it.
  // Failures resolve to null (never an unhandled rejection); the boot path
  // then falls back to a fresh fetch with real error surfacing.
  var fwPrefetch = (typeof window.DCT3_DEFAULT_FW === "string" && window.fetch)
    ? fetch(window.DCT3_DEFAULT_FW)
        .then(function (r) { return r.ok ? r.arrayBuffer() : null; })
        .catch(function () { return null; })
    : Promise.resolve(null);

  modulePromise.then(start).catch(function (e) {
    setStatus("Failed to boot.");
    showError("module-load", e);
  });

  // ---------------------------------------------------------------
  // Once Emscripten is ready: wrap exports, wait for the welcome to
  // be dismissed (or no-op if already-dismissed), then boot.
  // ---------------------------------------------------------------
  function start(mod) {
    if (welcomeEl && !welcomeEl.hidden) {
      setStatus("Ready — press Got it to start.");
    } else {
      setStatus("Booting…");
    }

    // cwrap helpers. Each is wrapped in optWrap so a missing export
    // doesn't crash init — it just disables the matching UI control.
    function optWrap(name, ret, args) {
      try {
        if (typeof mod.cwrap !== "function") return null;
        return mod.cwrap(name, ret, args || []);
      } catch (e) {
        showError("cwrap " + name, e);
        return null;
      }
    }
    function req(name, ret, args) {
      var fn = optWrap(name, ret, args);
      if (!fn) throw new Error("Required emulator export missing: " + name);
      return fn;
    }

    var C;
    try {
      C = {
        boot:        req("dct3_web_boot", "number"),
        runCyc:      req("dct3_web_run_cycles", null, ["number"]),
        fb:          req("dct3_web_fb", "number"),
        lcdMode:     req("dct3_web_lcd_mode", "number"),
        leds:        req("dct3_web_leds", "number"),
        key:         req("dct3_web_key", null, ["number", "number", "number"]),
        // Raw matrix set/clear (no auto-release). We use this for the
        // tap-vs-hold model: sets the bit on mousedown, clears it on
        // mouseup with a minimum-hold guarantee. The sequenced `key`
        // path is kept as a fallback if raw isn't exported.
        keyRaw:      optWrap("dct3_web_key_raw", null, ["number", "number", "number"]),
        power:       req("dct3_web_power", null, ["number"]),

        // Optional — UI features that depend on these self-disable.
        setKeyHold:     optWrap("dct3_web_set_key_hold", null, ["number"]),
        setRecover:     optWrap("dct3_web_set_recover", null, ["number"]),
        setCharger:     optWrap("dct3_web_set_charger", null, ["number"]),
        setSim:         optWrap("dct3_web_set_sim", null, ["number"]),
        setBattery:     optWrap("dct3_web_set_battery", null, ["number"]),
        getBattery:     optWrap("dct3_web_get_battery", "number"),
        // SIM PIN gate: the legacy UI's chk-pin defaults to ON. With
        // PIN OFF the firmware skips the lock screen, attempts network
        // registration on an incomplete GSM model, and falls into the
        // "no coverage" message (visible only when this gate is open).
        setSimPinEnabled: optWrap("dct3_web_set_sim_pin_enabled", null, ["number"]),
        setSimPin:        optWrap("dct3_web_set_sim_pin", null, ["number"]),
        // Audio paths. Two distinct sources on a 3310:
        //   - "Buzzer" = the piezo: square-wave, freq = 13 MHz / buzzerDiv.
        //     buzzerOn = sustained; buzzerChirp = sub-frame edges + the
        //     divider value at the edge (so very brief blips between
        //     polls are still captured).
        //   - "DSP tone" = the earpiece tone generator: sine, toneHz +
        //     optional toneHz2 for DTMF (column + row). Drives UI beeps.
        buzzerOn:       optWrap("dct3_web_buzzer_on", "number"),
        buzzerDiv:      optWrap("dct3_web_buzzer_div", "number"),
        buzzerChirp:    optWrap("dct3_web_buzzer_chirp", "number"),
        toneHz:         optWrap("dct3_web_tone_hz", "number"),
        toneHz2:        optWrap("dct3_web_tone_hz2", "number"),
        // Unified PCM earpiece stream (buzzer + DSP HLE tone/DTMF from emu_audio.c —
        // the same stream the native SDL GUI plays).
        pcmRead:        optWrap("dct3_web_pcm_read", "number", ["number"]),
        pcmPtr:         optWrap("dct3_web_pcm_ptr", "number"),
        pcmRate:        optWrap("dct3_web_pcm_rate", "number"),
        toneAmp:        optWrap("dct3_web_tone_amp", "number"),
        poweredOff:     optWrap("dct3_web_powered_off", "number"),
        // For the boot fast-forward branch — legacy busy-loops dct3_web_run
        // (instruction count) until C.step() crosses ~46M, then switches
        // to real-time cycle pacing. Without this, the boot animation
        // plays at real time; 46M also covers the organic SIM init
        // (disp49 reaches its standby/insert-SIM state ~24-32M steps).
        run:    optWrap("dct3_web_run", null, ["number"]),
        step:   optWrap("dct3_web_step", "number"),

        // Model + LCD geometry (valid after boot) — drives shell selection and the
        // canvas size for non-3310 images loaded via the firmware picker.
        model:    optWrap("dct3_web_model", "string"),
        lcdW:     optWrap("dct3_web_lcd_w", "number"),
        lcdH:     optWrap("dct3_web_lcd_h", "number"),
        lcdBanks: optWrap("dct3_web_lcd_banks", "number"),
        // Keypad VISUAL family (0=3310/B, 1=NSM Family-A, 2=Family-C, 3=7110/Navi-roller)
        // — drives the model-aware grid layout when no photo shell is registered.
        kpFamily: optWrap("dct3_web_kp_family", "number"),
        // Model-aware logical key (KK_*): the shell sends soft1/up/1/… and the active
        // profile maps it to the matrix — so the page never hardcodes a per-model grid.
        keyLogical: optWrap("dct3_web_key_logical", null, ["number", "number"]),
        // Faithful variant: drives a REAL matrix edge (set on down / clear on up), no
        // auto-release — the firmware decides tap vs hold, so press-and-hold works
        // (= native GUI). Interactive input routes here; keyLogical is the fallback.
        keyLogicalRaw: optWrap("dct3_web_key_logical_raw", null, ["number", "number"]),
        // Vibra motor state (CCONT PUP bit4, latched by mad2) — mirrored as a CSS
        // shake and, where the host device has a motor, real haptics.
        vibraOn: optWrap("dct3_web_vibra_on", "number"),
        // Network-side events (ROM-6 engine models): queue an incoming call /
        // SMS that the emulated network pages to the camped phone. Sender is
        // digits-only; SMS text ≤120 chars. Host pages drive these (e.g. a
        // first-visit welcome SMS) via dct3.api.incomingSms(from, text).
        incomingCall: optWrap("dct3_web_incoming_call", "number", ["string"]),
        incomingSms:  optWrap("dct3_web_incoming_sms", "number", ["string", "string"]),
        // NVRAM persistence surface. The flash NVRAM partition lives inside the
        // flat memory at ramPtr()+eepromOff(); early-MAD2 serial-bus models
        // (5110/…) keep theirs in a separate external I2C 24Cxx buffer instead.
        ramPtr:          optWrap("dct3_web_ram_ptr", "number"),
        eepromOff:       optWrap("dct3_web_eeprom_off", "number"),
        eepromSize:      optWrap("dct3_web_eeprom_size", "number"),
        eepromWrites:    optWrap("dct3_web_eeprom_writes", "number"),
        i2cEepromPtr:    optWrap("dct3_web_i2c_eeprom_ptr", "number"),
        i2cEepromSize:   optWrap("dct3_web_i2c_eeprom_size", "number"),
        i2cEepromWrites: optWrap("dct3_web_i2c_eeprom_writes", "number"),
        // swSIM card persistence. Not a memory window like the two above — the
        // software SIM's filesystem lives inside swICC, so it is serialised on
        // demand into swICC's own FS format and mounted back the same way.
        simWrites:       optWrap("dct3_web_sim_writes", "number"),
        simSnapshot:     optWrap("dct3_web_sim_snapshot", "number"),
        simSnapshotLen:  optWrap("dct3_web_sim_snapshot_len", "number"),
        simRestore:      optWrap("dct3_web_sim_restore", "number", ["number", "number"]),
        simWriteEf:      optWrap("dct3_web_sim_write_ef", "number", ["number", "number", "number", "number"]),
      };
    } catch (e) {
      setStatus("Emulator API mismatch.");
      showError("api-bind", e);
      return;
    }

    // -------------------------------------------------------------
    // Apply dev-toggle defaults to the emulator. Defaults match what
    // the legacy UI ships with so the boot path is the same.
    // -------------------------------------------------------------
    function applyToggle(setter, value) {
      if (typeof setter !== "function") return;
      try { setter(value ? 1 : 0); }
      catch (e) { showError("toggle", e); }
    }
    var chkRecover    = document.getElementById("chk-recover");
    var chkCharger    = document.getElementById("chk-charger");
    var chkSim        = document.getElementById("chk-sim");
    if (chkRecover) { applyToggle(C.setRecover, chkRecover.checked);
                      chkRecover.addEventListener("change", function () { applyToggle(C.setRecover, chkRecover.checked); }); }
    if (chkCharger) { applyToggle(C.setCharger, chkCharger.checked);
                      chkCharger.addEventListener("change", function () { applyToggle(C.setCharger, chkCharger.checked); }); }
    if (chkSim)     { applyToggle(C.setSim, chkSim.checked);
                      chkSim.addEventListener("change", function () { applyToggle(C.setSim, chkSim.checked); }); }

    var slBattery  = document.getElementById("sl-battery");
    var outBattery = document.getElementById("out-battery");
    function syncBattery() {
      if (!slBattery) return;
      var v = parseInt(slBattery.value, 10) | 0;
      if (outBattery) outBattery.textContent = v.toString();
      if (C.setBattery) tryDo("setBattery", function () { C.setBattery(v); });
    }
    if (slBattery) { slBattery.addEventListener("input", syncBattery); syncBattery(); }

    // -------------------------------------------------------------
    // Boot the firmware — but only after the welcome modal is gone.
    // Awaiting welcomeReady gates the firmware execution so the
    // visible state matches the modal copy ("Press the green key to
    // boot →") and the user doesn't see a phone already running.
    //
    // Some emulator state is RESET by C.boot() (mad2_init zeroes
    // g_mad2 — SIM defaults to PRESENT, recover defaults to ON).
    // We must re-apply UI toggles AFTER boot so the visible checkbox
    // state actually matches what the model sees. Without this the
    // phone boots SIM-present even when the box is unchecked, sending
    // it down the SIM-rejected timeout path that diverges from /web/.
    // -------------------------------------------------------------
    function reapplyPostBoot() {
      flushPCM();                     // clear stale audio from the previous model/boot
      if (chkSim && C.setSim)         applyToggle(C.setSim, chkSim.checked);
      if (chkCharger && C.setCharger) applyToggle(C.setCharger, chkCharger.checked);
      if (chkRecover && C.setRecover) applyToggle(C.setRecover, chkRecover.checked);
      if (slBattery && C.setBattery) {
        var v = parseInt(slBattery.value, 10) | 0;
        tryDo("setBattery-postboot", function () { C.setBattery(v); });
      }
      // SIM PIN OFF by default — visitor doesn't have to type a PIN to use
      // the phone. (Legacy's chk-pin defaults ON; we diverge intentionally.)
      // The stored PIN is still set to 1234 so if a curious user enables PIN
      // later via dev tools, the firmware knows what to expect.
      if (C.setSimPinEnabled) tryDo("setSimPinEnabled", function () { C.setSimPinEnabled(0); });
      if (C.setSimPin)        tryDo("setSimPin",        function () { C.setSimPin(1234); });
    }

    // -------------------------------------------------------------
    // NVRAM (EEPROM) persistence. The firmware's settings, clock, contacts and
    // security config live in the flash NVRAM partition (ramPtr()+eepromOff(),
    // eepromSize() bytes); early-MAD2 models add an external I2C 24Cxx chip
    // (i2cEeprom* trio, size 0 when the model has none). Both regions are
    // snapshotted to localStorage when the firmware programs them and overlaid
    // after every boot, so settings survive reloads and model switches.
    //
    // The key is namespaced per firmware: model name + FNV-1a over the first
    // 256 KB of the image (boot/code — stable across a J2ME game injection,
    // which rewrites the PMM store much higher in flash). Default ON here
    // (visitor expectation: the phone remembers); the devtools checkbox opts
    // out for reproducible boots, and "Wipe saved settings" recovers from a
    // bad persisted state. All storage access is guarded — a blocked
    // localStorage never breaks boot.
    // -------------------------------------------------------------
    var PERSIST_KEY = "dct3.next.persist";
    var nvramPersist = true;
    try { nvramPersist = localStorage.getItem(PERSIST_KEY) !== "0"; } catch (_) {}
    var curFwId = "default";
    var eeLastWrites = -1, i2cLastWrites = -1;
    var nvramSuppress = false;      // set during a wipe so unload-save can't undo it

    // Backing store is IndexedDB, NOT localStorage: the 3410's partition alone is
    // 576 KB (base64 ≈ 1.5 MB of UTF-16), so a couple of models blow through the
    // ~5 MB localStorage origin quota (seen in the wild as QuotaExceededError).
    // IDB stores the bytes natively and has no such practical cap. Reads must be
    // synchronous at boot-overlay time, so everything lives in an in-memory cache
    // (nvramCache) that is loaded once before the first boot (nvramReady) and
    // write-through persisted to IDB in the background.
    var NVDB = "dct3-nvram", NVSTORE = "nvram";
    var nvramCache = {};            // eeKey/i2cKey -> Uint8Array
    function nvdb() {
      return new Promise(function (res, rej) {
        var r = indexedDB.open(NVDB, 1);
        r.onupgradeneeded = function () { r.result.createObjectStore(NVSTORE); };
        r.onsuccess = function () { res(r.result); };
        r.onerror = function () { rej(r.error); };
      });
    }
    function nvIdbPut(key, u8) {
      return nvdb().then(function (db) { return new Promise(function (res, rej) {
        var t = db.transaction(NVSTORE, "readwrite");
        t.objectStore(NVSTORE).put(u8, key);
        t.oncomplete = res; t.onerror = function () { rej(t.error); };
      }); }).catch(function (e) {
        if (typeof console !== "undefined") console.warn("[dct3] NVRAM persist failed:", e);
      });
    }
    function nvIdbDel(key) {
      return nvdb().then(function (db) { return new Promise(function (res) {
        var t = db.transaction(NVSTORE, "readwrite");
        t.objectStore(NVSTORE).delete(key);
        t.oncomplete = res; t.onerror = res;
      }); }).catch(function () {});
    }
    // Load every saved region into the cache; the first boot awaits this.
    var nvramReady = nvdb().then(function (db) {
      return new Promise(function (res) {
        var st = db.transaction(NVSTORE, "readonly").objectStore(NVSTORE);
        var kq = st.getAllKeys(), vq = st.getAll(), done = 0;
        function fin() {
          if (++done < 2) return;
          for (var i = 0; i < kq.result.length; i++)
            nvramCache[kq.result[i]] = new Uint8Array(vq.result[i]);
          res();
        }
        kq.onsuccess = vq.onsuccess = fin;
        kq.onerror = vq.onerror = function () { res(); };
      });
    }).catch(function () {}).then(function () {
      // One-time migration from the earlier localStorage scheme — adopts the
      // save AND frees the origin's quota. Only OUR key shape is touched
      // (model-<8hex>); the legacy /web UI's plain-hash keys are left alone.
      try {
        var mine = [];
        for (var i = 0; i < localStorage.length; i++) {
          var k = localStorage.key(i);
          if (/^dct3_(i2c)?eeprom_.+-[0-9a-f]{8}$/.test(k)) mine.push(k);
        }
        mine.forEach(function (k) {
          if (!nvramCache[k]) {
            try { nvramCache[k] = b64dec(localStorage.getItem(k)); nvIdbPut(k, nvramCache[k]); }
            catch (_) {}
          }
          try { localStorage.removeItem(k); } catch (_) {}
        });
      } catch (_) {}
    });

    function fwIdentity() {
      try {
        var fw = mod.FS.readFile("/fw.fls");
        var h = 0x811c9dc5 >>> 0, i;
        var n = Math.min(fw.length, 0x40000);
        for (i = 0; i < n; i++) { h ^= fw[i]; h = Math.imul(h, 0x01000193) >>> 0; }
        // Fold in the image's LAST 64 KB too: two same-code images differing
        // only in their shipped EEPROM (e.g. a bundled-image FAID fix) must NOT
        // share a key, or the old image's saved NVRAM would overlay — and
        // effectively resurrect — the state the new image was shipped to fix.
        // The tail sits outside the 3410's PMM game-store blocks, so J2ME
        // injection still keeps its key.
        for (i = Math.max(0, fw.length - 0x10000); i < fw.length; i++) { h ^= fw[i]; h = Math.imul(h, 0x01000193) >>> 0; }
        h = (h ^ fw.length) >>> 0;
        return ((C.model && C.model()) || "fw") + "-" + ("0000000" + h.toString(16)).slice(-8);
      } catch (e) { return "default"; }
    }
    function eeKey()  { return "dct3_eeprom_" + curFwId; }
    function i2cKey() { return "dct3_i2ceeprom_" + curFwId; }

    function b64dec(b64) {          // migration-only (old localStorage saves)
      var s = atob(b64), u8 = new Uint8Array(s.length);
      for (var i = 0; i < s.length; i++) u8[i] = s.charCodeAt(i);
      return u8;
    }
    function eepromSlice() {
      if (!C.ramPtr || !C.eepromOff || !C.eepromSize) return null;
      var base = C.ramPtr() + C.eepromOff(), size = C.eepromSize();
      return size ? mod.HEAPU8.subarray(base, base + size) : null;
    }
    function i2cSlice() {
      if (!C.i2cEepromPtr || !C.i2cEepromSize) return null;
      var sz = C.i2cEepromSize(); if (!sz) return null;
      var base = C.i2cEepromPtr();
      return mod.HEAPU8.subarray(base, base + sz);
    }
    // "Blank" = every byte erased (0xFF) or zero. Not real NVRAM — never persist
    // or overlay it (injecting a blank partition breaks firmware consistency checks).
    function eepromBlank(u8) {
      for (var i = 0; i < u8.length; i++) { var x = u8[i]; if (x !== 0 && x !== 0xff) return false; }
      return true;
    }
    // Auto-saves only fire when the firmware actually PROGRAMMED the region this
    // run (writes counter moved), so a broken boot never overwrites a good save.
    // The cache write is synchronous truth; the IDB put trails in the background
    // (failures console.warn — never the error banner, and never a retry storm).
    function saveEeprom(force) {
      if (nvramSuppress) return "off";
      if (!force && !nvramPersist) return "off";
      if (!C.eepromWrites || (!force && C.eepromWrites() === 0)) return "nochange";
      var slice = eepromSlice(); if (!slice) return "off";
      if (eepromBlank(slice)) return "blank";
      var copy = new Uint8Array(slice);       // detach from the wasm heap before async persist
      nvramCache[eeKey()] = copy;
      nvIdbPut(eeKey(), copy);
      eeLastWrites = C.eepromWrites();
      return "saved";
    }
    function loadEeprom() {
      if (!nvramPersist) return false;
      if (!C.ramPtr || !C.eepromOff || !C.eepromSize) return false;
      var u8 = nvramCache[eeKey()];
      if (!u8 || u8.length !== C.eepromSize()) return false;
      if (eepromBlank(u8)) { delete nvramCache[eeKey()]; nvIdbDel(eeKey()); return false; }
      mod.HEAPU8.set(u8, C.ramPtr() + C.eepromOff());
      if (typeof console !== "undefined") console.log("[dct3] restored", u8.length, "B EEPROM (" + eeKey() + ")");
      return true;
    }
    function saveI2cEeprom(force) {
      if (nvramSuppress) return "off";
      if (!force && !nvramPersist) return "off";
      var slice = i2cSlice(); if (!slice) return "off";
      if (!C.i2cEepromWrites || (!force && C.i2cEepromWrites() === 0)) return "nochange";
      if (eepromBlank(slice)) return "blank";
      var copy = new Uint8Array(slice);
      nvramCache[i2cKey()] = copy;
      nvIdbPut(i2cKey(), copy);
      i2cLastWrites = C.i2cEepromWrites();
      return "saved";
    }
    function loadI2cEeprom() {
      if (!nvramPersist) return false;
      var sz = (C.i2cEepromSize && C.i2cEepromSize()) || 0; if (!sz) return false;
      var u8 = nvramCache[i2cKey()];
      if (!u8 || u8.length !== sz) return false;
      if (eepromBlank(u8)) { delete nvramCache[i2cKey()]; nvIdbDel(i2cKey()); return false; }
      mod.HEAPU8.set(u8, C.i2cEepromPtr());
      if (typeof console !== "undefined") console.log("[dct3] restored", u8.length, "B I2C EEPROM (" + i2cKey() + ")");
      return true;
    }
    // ---- swSIM card ------------------------------------------------------
    // ONE key for the whole site, deliberately NOT namespaced per firmware like
    // the two EEPROM regions above: a SIM is a card you carry between handsets,
    // so the contacts and messages saved on it follow the visitor when they
    // switch models. (The phone's own memory stays per-firmware, as it should.)
    // The blob is swICC's own FS format — magic-tagged, so a stale or foreign
    // snapshot is rejected by the loader rather than silently mounted, and a
    // rejection costs the saved card but never the boot.
    var SIM_KEY = "dct3_swsim_v1";
    var simLastWrites = -1;
    function saveSimCard(force) {
      if (nvramSuppress) return "off";
      if (!force && !nvramPersist) return "off";
      if (!C.simSnapshot || !C.simWrites) return "off";
      if (!force && C.simWrites() === 0) return "nochange";
      var ptr = C.simSnapshot(), len = C.simSnapshotLen ? C.simSnapshotLen() : 0;
      if (!ptr || len <= 0) return "off";                  // card never came up
      var copy = new Uint8Array(mod.HEAPU8.subarray(ptr, ptr + len));
      nvramCache[SIM_KEY] = copy;
      nvIdbPut(SIM_KEY, copy);
      simLastWrites = C.simWrites();
      return "saved";
    }
    function loadSimCard() {
      if (!nvramPersist || !C.simRestore) return false;
      var u8 = nvramCache[SIM_KEY];
      if (!u8 || u8.length < 16) return false;
      var p = mod._malloc(u8.length), ok = 0;
      try { mod.HEAPU8.set(u8, p); ok = C.simRestore(p, u8.length); }
      finally { mod._free(p); }
      if (!ok) {                                           // unmountable -> drop it
        delete nvramCache[SIM_KEY]; nvIdbDel(SIM_KEY);
        if (typeof console !== "undefined") console.warn("[dct3] saved SIM rejected — factory card");
        return false;
      }
      // The operator name is a page SETTING (patched into gsm.json before boot),
      // not card data — so it has to override the SPN carried by the snapshot,
      // or dct3SetOperatorName() would silently do nothing for a returning visitor.
      applySpnToCard();
      if (typeof console !== "undefined") console.log("[dct3] restored", u8.length, "B SIM card");
      return true;
    }
    // Write EF_SPN (7F20/6F46) straight onto the live card from the current knob.
    function applySpnToCard() {
      // opName is a `var` declared further down; if a boot ever runs before that
      // line executes, leave the restored SPN alone rather than blanking it.
      if (!C.simWriteEf || typeof spnHex !== "function" || typeof opName !== "string") return;
      var hex = spnHex(opName), n = hex.length / 2;
      var p = mod._malloc(n);
      try {
        for (var i = 0; i < n; i++) mod.HEAPU8[p + i] = parseInt(hex.substr(i * 2, 2), 16);
        C.simWriteEf(0x7F20, 0x6F46, p, n);
      } finally { mod._free(p); }
    }

    // Persist ALL THREE regions; each self-gates on presence + write activity.
    function saveNvram(force) {
      var r = saveEeprom(force); saveI2cEeprom(force); saveSimCard(force); return r;
    }

    setInterval(function () {
      if (C.eepromWrites && C.eepromWrites() !== eeLastWrites) saveEeprom();
      if (C.i2cEepromWrites && C.i2cEepromWrites() !== i2cLastWrites) saveI2cEeprom();
      if (C.simWrites && C.simWrites() !== simLastWrites) saveSimCard();
    }, 3000);
    document.addEventListener("visibilitychange", function () { if (document.hidden) saveNvram(); });
    window.addEventListener("beforeunload", function () { saveNvram(); });

    window.dct3SaveEeprom  = function () { return saveNvram(true); };
    window.dct3ResetEeprom = function () {
      nvramSuppress = true;         // the reload's unload-save must not re-persist the wipe
      delete nvramCache[eeKey()]; delete nvramCache[i2cKey()]; delete nvramCache[SIM_KEY];
      var reload = function () { location.reload(); };
      Promise.all([nvIdbDel(eeKey()), nvIdbDel(i2cKey()), nvIdbDel(SIM_KEY)]).then(reload, reload);
    };

    // Devtools: persistence opt-out + wipe.
    var chkPersist = document.getElementById("chk-persist");
    if (chkPersist) {
      chkPersist.checked = nvramPersist;
      chkPersist.addEventListener("change", function () {
        nvramPersist = chkPersist.checked;
        try { localStorage.setItem(PERSIST_KEY, nvramPersist ? "1" : "0"); } catch (_) {}
        if (nvramPersist) saveNvram(true);      // snapshot right away on opt-in
      });
    }
    var btnEeReset = document.getElementById("btn-ee-reset");
    if (btnEeReset) btnEeReset.addEventListener("click", function () { window.dct3ResetEeprom(); });

    // -------------------------------------------------------------
    // Central (re)boot path. Every boot — first boot, reboot, firmware swap,
    // game injection, force reset — funnels through here so the sequencing is
    // identical everywhere: persist the outgoing image's NVRAM, boot, re-key,
    // overlay the new image's saved NVRAM, then re-apply the UI toggles that
    // mad2_init clobbers and re-mount the shell for the detected model.
    // -------------------------------------------------------------
    // `afterSave` (optional) runs between the outgoing save and the boot+overlay —
    // the game injector uses it to patch the saved snapshot (see dct3InjectGame).
    function doBoot(afterSave) {
      saveNvram();                  // keep the outgoing session's settings
      if (typeof afterSave === "function") afterSave();
      C.boot();
      curFwId = fwIdentity();
      loadEeprom();
      loadI2cEeprom();
      // The card is mounted AFTER boot but before the run loop hands the firmware
      // its first APDU — boot() itself never talks to the SIM (swSIM is built
      // lazily), so this is the one safe window to swap the filesystem underneath.
      loadSimCard();
      eeLastWrites  = (C.eepromWrites    && C.eepromWrites())    || 0;
      i2cLastWrites = (C.i2cEepromWrites && C.i2cEepromWrites()) || 0;
      simLastWrites = (C.simWrites       && C.simWrites())       || 0;
      reapplyPostBoot();
      applyModel();
      halted = false;
      lastT = null;
    }

    // -------------------------------------------------------------
    // Operator name (SPN). The emulated network is always PLMN 208-01, which
    // the firmware's built-in table names "Orange F". The SIM's Service
    // Provider Name (EF_SPN, 6F46) takes precedence on the home network, and
    // swSIM's filesystem is a plain JSON file in MEMFS — so we patch it there
    // before boot (swSIM re-reads the file on every dct3_web_boot). This is a
    // CAPABILITY, not branding: the host page sets the default via
    // window.DCT3_OPERATOR_NAME (like DCT3_DEFAULT_FW), and anyone can play
    // with it live via dct3SetOperatorName("Name") (≤16 GSM-alphabet chars;
    // "" reverts to the firmware's own name) — also in devtools. A user's
    // choice persists per browser and wins over the host default. No-op on
    // cores without swSIM.
    // -------------------------------------------------------------
    var OPNAME_KEY = "dct3.next.opname";
    var SWSIM_FS = "/third_party/swsim/gsm.json";
    var opName = String(window.DCT3_OPERATOR_NAME || "");
    try { var _on = localStorage.getItem(OPNAME_KEY); if (_on !== null) opName = _on; } catch (_) {}
    function spnHex(name) {
      if (!name) return "FF".repeat(17);            // virgin EF → firmware default name
      // Display-condition 00: "display of registered PLMN NOT required" — the
      // phone keeps showing the SIM's name after it registers. (01 makes it
      // swap to the firmware's own PLMN-table name at registration.)
      var h = "00";
      for (var i = 0; i < 16; i++) {
        var c = i < name.length ? name.charCodeAt(i) & 0x7f : 0xff;
        h += ("0" + c.toString(16)).slice(-2);
      }
      return h.toUpperCase();
    }
    function patchSpn(name) {
      try {
        if (!mod.FS || !mod.FS.readFile) return false;
        var fs = JSON.parse(new TextDecoder().decode(mod.FS.readFile(SWSIM_FS)));
        var hits = 0;
        (function walk(node) {
          if (Array.isArray(node)) { node.forEach(walk); return; }
          if (!node || typeof node !== "object") return;
          if (node.id === "6F46" && node.contents)
            { node.contents = { type: "hex", contents: spnHex(name) }; hits++; }
          for (var k in node) walk(node[k]);
        })(fs);
        if (!hits) return false;
        mod.FS.writeFile(SWSIM_FS, new TextEncoder().encode(JSON.stringify(fs)));
        return true;
      } catch (e) { return false; }                 // no swSIM fs on this core → no-op
    }
    patchSpn(opName);                               // ahead of the first boot
    // swSIM parses its filesystem ONCE per process (g_ready latch), so a
    // mid-session change can only land via a full page reload — the pre-boot
    // patchSpn above then applies the new name.
    window.dct3SetOperatorName = function (name) {
      name = String(name == null ? "" : name).slice(0, 16);
      try { localStorage.setItem(OPNAME_KEY, name); } catch (_) {}
      location.reload();
    };
    var inOpname = document.getElementById("in-opname");
    var btnOpname = document.getElementById("btn-opname");
    if (inOpname) inOpname.value = opName;
    if (btnOpname) btnOpname.addEventListener("click", function () {
      window.dct3SetOperatorName(inOpname ? inOpname.value : opName);
    });

    // Fetch a raw .fls into MEMFS at /fw.fls. Used for firmware-free hosting
    // (no fw baked into dct3.data) and by the model switcher below.
    function loadFwToMemfs(url) {
      return fetch(url).then(function (r) {
        if (!r.ok) throw new Error("fetch " + url + " -> " + r.status);
        return r.arrayBuffer();
      }).then(function (ab) { mod.FS.writeFile("/fw.fls", new Uint8Array(ab)); });
    }

    // Public switch API for a model menu (e.g. retro-phone): swap the .fls,
    // reboot, and re-mount the shell for the newly-detected model.
    window.dct3SwapFirmware = function (url) {
      return loadFwToMemfs(url).then(function () {
        tryDo("fw-swap", doBoot);
      }, function (e) { showError("fw-swap", e); });
    };

    welcomeReady.then(function () {
      // Firmware-free hosting: land the configured default image in MEMFS
      // before the first boot. Normally the prefetch (started at script load,
      // behind the welcome wall) already has the bytes; fall back to fetching
      // now if it failed. No-op when a fw was preloaded via dct3.data.
      if (!(window.DCT3_DEFAULT_FW && mod.FS && mod.FS.writeFile)) return null;
      return fwPrefetch.then(function (ab) {
        if (ab) { mod.FS.writeFile("/fw.fls", new Uint8Array(ab)); return null; }
        return loadFwToMemfs(window.DCT3_DEFAULT_FW);
      });
    }).then(function () {
      return nvramReady;            // saved-NVRAM cache must be loaded before boot 1
    }).then(function () {
      setStatus("Booting…");
      try { doBoot(); }                  // boot + NVRAM overlay + shell mount
      catch (e) {
        setStatus("Firmware boot failed.");
        showError("boot", e);
        return;
      }
      setStatus("Running.");
      requestAnimationFrame(frame);
    });

    // -------------------------------------------------------------
    // LCD rendering. PCD8544 framebuffer is 6 banks × 84 columns,
    // 8 vertical pixels per bank → unpacked to a 84×48 ImageData.
    // -------------------------------------------------------------
    var canvas = document.getElementById("lcd");
    var ctx;
    var img;
    var LCDW = 84, LCDH = 48, LCDBANKS = 6;     // model-aware; re-synced on each boot
    try {
      ctx = canvas.getContext("2d");
      img = ctx.createImageData(LCDW, LCDH);
    } catch (e) {
      showError("canvas-init", e);
      return;
    }

    // Default DCT3 LCD palette (yellow-green). A mounted shell may override it
    // (shellBg/shellBgLit/shellPixel) so the canvas blends into the photo glass.
    var ON      = [0x2b, 0x39, 0x1a, 0xff];  // dark olive-green pixel
    var OFF     = [0xae, 0xc4, 0x8e, 0xff];  // background, backlight OFF
    var OFF_LIT = [0xcf, 0xe9, 0x84, 0xff];  // background, backlight ON
    var shellBg = null, shellBgLit = null, shellPixel = null;   // per-shell palette (null ⇒ default)
    var shellRoot = null, shellLcdEl = null, shellLitState = -1; // active shell DOM + last panel bg
    var shellDef = null;                                        // active shell unit (screenshot geometry)
    var rgbStr = function (c) { return "rgb(" + c[0] + "," + c[1] + "," + c[2] + ")"; };

    // Resize the canvas backing store + ImageData to the active model's geometry.
    function syncLcdGeometry() {
      LCDW     = (C.lcdW     && C.lcdW())     || 84;
      LCDH     = (C.lcdH     && C.lcdH())     || 48;
      LCDBANKS = (C.lcdBanks && C.lcdBanks()) || Math.ceil(LCDH / 8);
      if (canvas.width !== LCDW || canvas.height !== LCDH) {
        canvas.width = LCDW; canvas.height = LCDH;
        try { img = ctx.createImageData(LCDW, LCDH); }
        catch (e) { showError("imgdata", e); }
      }
    }

    function render() {
      var ptr = C.fb();
      var fb = mod.HEAPU8.subarray(ptr, ptr + LCDBANKS * LCDW);
      var mode = C.lcdMode();
      var leds = C.leds();
      var backlight = !!(leds & 1);     // bit0 = LCD backlight
      var frontLit  = leds & 2;         // bit1 = keypad/front lamp (glows shell glyphs)
      document.body.classList.toggle("lcd-off", !backlight);
      document.body.classList.toggle("kbd-lit", !!frontLit);
      // Amber-palette shells follow the front lamp; the default green LCD follows the
      // LCD-backlight bit. (Matches /web/.)
      var base = (shellRoot && shellBg) ? (frontLit && shellBgLit ? shellBgLit : shellBg)
                                        : (backlight ? OFF_LIT : OFF);
      var onc  = (shellRoot && shellPixel) ? shellPixel : ON;
      // The glass panel brightens with the canvas so the whole screen changes
      // uniformly (no lit rectangle floating in a dim panel).
      if (shellLcdEl) {
        var bg = rgbStr(base);
        if (shellLitState !== bg) { shellLcdEl.style.background = bg; shellLitState = bg; }
      }
      if (shellRoot) shellRoot.style.setProperty("--light-on", frontLit ? "1" : "0");
      var d = img.data;
      var o = 0;
      for (var y = 0; y < LCDH; y++) {
        var bank = (y >> 3) * LCDW;
        var bit  = y & 7;
        for (var x = 0; x < LCDW; x++) {
          var on = (fb[bank + x] >> bit) & 1;
          if      (mode === 0) on = 0;
          else if (mode === 1) on = 1;
          else if (mode === 3) on ^= 1;
          var c = on ? onc : base;
          d[o++] = c[0]; d[o++] = c[1]; d[o++] = c[2]; d[o++] = c[3];
        }
      }
      ctx.putImageData(img, 0, 0);
    }

    // -------------------------------------------------------------
    // Run loop. Cycle-paced to real time: we advance (wall-dt × 13 MHz
    // × speed) cycles per frame so the emulated clock tracks reality
    // regardless of monitor refresh rate. Matches the legacy UI; see
    // its main.js for the long comment about why instruction-pacing
    // was wrong.
    // -------------------------------------------------------------
    var TARGET_HZ    = 13e6;
    var MAX_FRAME_DT = 0.05;     // s
    var speedMul     = 1.0;
    var lastT        = null;
    // Boot fast-forward: instruction-paced busy loop until the firmware has
    // executed ~46M instructions (= "OS-ready" threshold per legacy). Lets
    // the boot sequence reach the live OS in ~1-2 s of wall clock instead of
    // ~3.5 s at real-time pacing. Matches legacy /web/main.js exactly.
    var BOOT_CHUNK     = 250000;
    var BOOT_BUDGET_MS = 12;
    var BOOT_INSN_TGT  = 46e6;

    // Frame-loop kill-switch. If `render` or `runCyc` throws this many
    // times in a row, we stop pumping frames so the banner isn't
    // flooded and the page stays responsive.
    var consecutiveFails = 0;
    var FAIL_LIMIT = 5;
    var loopStopped = false;

    // Power-off latch. The mad2 power_off bit fires when the firmware
    // writes WDT=0 to CCONT — its own clean-shutdown signal. We don't
    // halt on wall-clock anymore; the firmware drives the power state
    // machine fully, so single tap = Profiles menu, second tap =
    // advance, long-press from idle = firmware-driven shutdown.
    var halted = false;

    // -------------------------------------------------------------
    // Vibra. mad2 latches the CCONT vibra enable; we mirror it two ways:
    //   1. a CSS shake on the phone body (works everywhere), and
    //   2. the device's real vibration motor via navigator.vibrate — Android
    //      browsers have it; iOS Safari has no vibration API, so it silently
    //      degrades to the visual shake there.
    // While the motor is on we re-issue a pulse before the previous one runs
    // out (each vibrate() call replaces the pattern); on release vibrate(0)
    // cancels immediately so the haptics track the firmware's edge.
    // -------------------------------------------------------------
    var vibStyle = document.createElement("style");
    vibStyle.textContent =
      "@keyframes dct3shake{0%,100%{transform:translateX(0)}25%{transform:translateX(-2px)}75%{transform:translateX(2px)}}" +
      // shake the grid chassis (.phone) AND the device shell (.shell-host wrapper —
      // NOT .shell-root, whose transform:scale would be clobbered by the animation)
      ".vibrating .phone,.vibrating .shell-host{animation:dct3shake .08s linear infinite}";
    document.head.appendChild(vibStyle);
    var vibLast = false, vibPulseAt = 0;
    function syncVibra() {
      if (!C.vibraOn) return;
      var on = false;
      try { on = !!C.vibraOn(); } catch (_) {}
      if (on !== vibLast) document.body.classList.toggle("vibrating", on);
      try {
        if (navigator.vibrate) {
          var now = performance.now();
          if (on && now - vibPulseAt > 400) { navigator.vibrate(600); vibPulseAt = now; }
          else if (!on && vibLast) { navigator.vibrate(0); vibPulseAt = 0; }
        }
      } catch (_) {}
      vibLast = on;
    }

    var slSpeed  = document.getElementById("sl-speed");
    var outSpeed = document.getElementById("out-speed");
    function syncSpeed() {
      if (!slSpeed) return;
      var pct = parseInt(slSpeed.value, 10) | 0;
      speedMul = pct / 100;
      if (outSpeed) outSpeed.textContent = (speedMul.toFixed(2)) + "×";
    }
    if (slSpeed) { slSpeed.addEventListener("input", syncSpeed); syncSpeed(); }

    function frame(now) {
      if (loopStopped) return;
      try {
        pumpPCM();                          // drain the unified PCM ring (no-op if audio off)
        // CCONT clean-shutdown detection. The firmware ran its full
        // power-off sequence (animation, NVRAM flush, WDT=0 to CCONT)
        // and mad2 latched power_off. Park the CPU; tap power to reboot.
        if (!halted && C.poweredOff && C.poweredOff()) {
          halted = true;
          setStatus("Powered off — tap ⏻ Power to switch on.");
        }
        if (halted) {
          // Stop pumping CPU. Keep rendering so the cleared LCD stays
          // visible; the firmware already blanked the framebuffer
          // during the shutdown animation. syncVibra still runs so a
          // shake/haptic active at shutdown is released, not stuck.
          render();
          syncVibra();
          requestAnimationFrame(frame);
          return;
        }

        // Boot phase: busy-loop instruction-count chunks for up to 12 ms wall
        // time per frame, until the firmware crosses the OS-ready threshold.
        // Real-time pacing takes over after that. This matches /web/ exactly
        // and lets the organic early boot (DSP self-test verdict, SIM init)
        // complete before any host-time-dependent UI work.
        var booting = C.step && C.step() < BOOT_INSN_TGT;
        if (booting && C.run) {
          var t0 = performance.now();
          do { C.run(BOOT_CHUNK); } while (performance.now() - t0 < BOOT_BUDGET_MS);
          lastT = now;                     // resync pacing for the transition frame
        } else {
          if (lastT === null) { lastT = now; }
          var dt = (now - lastT) / 1000;
          lastT = now;
          if (dt > MAX_FRAME_DT) dt = MAX_FRAME_DT;
          if (dt > 0) {
            var cycles = Math.round(TARGET_HZ * speedMul * dt);
            if (cycles > 0) C.runCyc(cycles);
          }
        }
        render();
        syncVibra();
        consecutiveFails = 0;
      } catch (e) {
        consecutiveFails++;
        showError("frame", e);
        if (consecutiveFails >= FAIL_LIMIT) {
          loopStopped = true;
          setStatus("Run loop halted after repeated errors — reload to retry.");
          return;
        }
      }
      requestAnimationFrame(frame);
    }

    // (The frame loop is started by the welcomeReady handler above,
    // so it doesn't render an empty LCD before the user dismisses.)

    // -------------------------------------------------------------
    // Keypad: on-screen grid (model-aware, built below) + physical keyboard.
    // Every input path routes LOGICAL key labels through the active model's own
    // keymap via pressLogical()/keyLogicalRaw — the page hardcodes no per-model
    // matrix. The firmware's own scan/repeat/long-press timing decides tap vs hold.
    // -------------------------------------------------------------
    var MIN_HOLD_INSNS = 200000;                   // ≈15 ms @13 MHz — one keypad-scan window
    // Auto-release window for the queued-tap fallback (old wasm without key_logical_raw).
    if (C.setKeyHold) tryDo("setKeyHold", function () { C.setKeyHold(MIN_HOLD_INSNS); });

    // -------------------------------------------------------------
    // Model-aware grid keypad (used when the active model has no photo shell).
    // Mirrors the legacy /web builder + the native GUI's family_layout: one
    // 3-column grid per VISUAL family, every button a LOGICAL label routed through
    // the model's own keymap (pressLogical). Family 3 (7110) swaps up/down for the
    // Navi roller (roll up / press / roll down), also driven by the mouse wheel.
    var padHost = document.querySelector(".pad");
    // ''=empty spacer cell. B(3310): Menu/Names + up/down. A/C: soft + send/end.
    var PAD = {
      0: [ ["soft1", "up", "soft2"], ["", "down", ""],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
      1: [ ["soft1", "up", "soft2"], ["send", "down", "end"],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
      2: [ ["soft1", "up", "soft2"], ["send", "down", "end"],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
      3: [ ["soft1", "wheelup", "soft2"], ["send", "wheelpress", "end"],
           ["", "wheeldown", ""],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
    };
    var GLYPH = { up:"▲", down:"▼", soft1:"Menu", soft2:"Names",
                  send:"", end:"", wheelup:"▲", wheelpress:"●", wheeldown:"▼",
                  volup:"V+", voldown:"V−" };
    var KEYCLASS = { soft1:"softkey", soft2:"softkey", send:"send", end:"end",
                     up:"nav", down:"nav", wheelup:"nav", wheelpress:"nav",
                     wheeldown:"nav", volup:"vol", voldown:"vol" };
    function glyphFor(family, label) {
      if (family === 0) { if (label === "soft1") return "Menu"; if (label === "soft2") return "C"; }
      if (label in GLYPH) return GLYPH[label];
      return label;
    }
    function makeKey(family, label) {
      var b = document.createElement("button");
      b.className = "key" + (KEYCLASS[label] ? " " + KEYCLASS[label] : "");
      b.dataset.key = label;
      b.setAttribute("aria-label", label);
      if (label === "end") {
        var s = document.createElement("span"); s.className = "hangup"; s.textContent = "📞";
        b.appendChild(s);
      } else if (label === "send") {
        b.textContent = "📞";
      } else {
        b.textContent = glyphFor(family, label);
      }
      bindZone(b, label);              // pointer down/up → pressLogical(label)
      return b;
    }
    var currentFamily = 0;
    function buildKeypad(family) {
      if (!padHost) return;
      family = (family | 0);
      currentFamily = family;
      padHost.innerHTML = "";
      (PAD[family] || PAD[0]).forEach(function (rowarr) {
        var r = document.createElement("div"); r.className = "padrow";
        rowarr.forEach(function (label) {
          var cell = document.createElement("div"); cell.className = "padcell";
          if (label) cell.appendChild(makeKey(family, label));
          r.appendChild(cell);
        });
        padHost.appendChild(r);
      });
    }
    // Navi Roller via the mouse wheel — only the 7110 (family 3) has one. Wheel up =
    // roll up, down = roll down; one detent per notch with a short cooldown so a fast
    // scroll doesn't flood the encoder.
    var rollAt = 0;
    window.addEventListener("wheel", function (e) {
      if (currentFamily !== 3) return;
      var t = e.target;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA")) return;
      e.preventDefault();
      var now = performance.now();
      if (now - rollAt < 30) return;
      rollAt = now;
      pressLogical(e.deltaY < 0 ? "wheelup" : "wheeldown", true);
      pressLogical(e.deltaY < 0 ? "wheelup" : "wheeldown", false);
    }, { passive: false });

    // -------------------------------------------------------------
    // Device shell. Mounts a shared photo body (shells/<model>/) with the live
    // #lcd canvas seated on its glass and transparent hit-zones over each key,
    // routed through the model-aware logical-key API (so no per-model matrix is
    // hardcoded). Falls back to the .chassis grid when no shell is registered.
    // -------------------------------------------------------------
    var KK = { "0":1,"1":2,"2":3,"3":4,"4":5,"5":6,"6":7,"7":8,"8":9,"9":10,
               "*":11,"#":12, up:13, down:14, soft1:15, soft2:16, send:17, end:18,
               volup:19, voldown:20, pwr:21,
               // 7110 Navi Roller — the profile maps these to the optical-encoder
               // rotate/press, NOT to matrix up/down (see keylines_7110).
               wheelup:22, wheeldown:23, wheelpress:24 };
    var SHELLS = window.DCT3_SHELLS || {};
    var ASSET = function (p) { return (window.DCT3_SHELL_PREFIX || "") + p; };
    var shellHost = document.getElementById("shell");
    var lcdHome   = canvas.parentElement;     // .chassis — the canvas's fallback home
    var SHELL_MAX_W = 300;

    function hexRgba(h) {
      h = h.replace("#", ""); if (h.length === 3) h = h.split("").map(function (c) { return c + c; }).join("");
      var n = parseInt(h, 16); return [(n >> 16) & 255, (n >> 8) & 255, n & 255, 255];
    }
    function pickShell(model) {
      if (!model || !shellHost) return null;
      if (SHELLS[model]) return SHELLS[model];
      var alias = (window.DCT3_SHELL_ALIASES || {})[model];   // shared body (3330→3310)
      if (alias && SHELLS[alias]) return SHELLS[alias];
      for (var k in SHELLS) if (model.indexOf(k) >= 0) return SHELLS[k];
      return null;
    }
    // Route a shell zone / keyboard label to the firmware. pwr drives the real
    // power path; everything else is a model-aware logical keypress. Faithful edge:
    // set the matrix bit on DOWN, clear on UP — the firmware's own scan/repeat/
    // long-press timing decides tap vs hold (= native GUI), so press-and-hold works.
    // A small min-hold floor lets a very fast click still span a keypad scan; the
    // deferred release is skipped if the key was pressed again meanwhile.
    var KEY_MIN_HOLD_MS = 20;            // ~> one firmware keypad-scan period (~17 ms @13 MHz)
    var keyHeldAt = {};                  // label -> down timestamp (ms); absent = released
    function pressLogical(label, down) {
      if (label === "pwr") {
        // Off → on: the run loop is parked while halted, so toggling the power
        // line alone does nothing. Cold-reboot instead — same path the panel
        // ⏻ button takes on mousedown when halted.
        if (down && halted) {
          tryDo("shell-power-on", function () {
            doBoot();
            setStatus("Booting…");
          });
          return;
        }
        tryDo("shell-power", function () { C.power(down ? 1 : 0); });
        return;
      }
      var id = KK[label];
      if (id == null) return;
      var raw = C.keyLogicalRaw;
      if (!raw) {                         // old wasm: queued-tap fallback
        if (C.keyLogical) tryDo("shell-key", function () { C.keyLogical(id, down ? 1 : 0); });
        return;
      }
      if (down) {
        if (keyHeldAt[label] != null) return;          // idempotent (autorepeat / re-entry)
        keyHeldAt[label] = performance.now();
        tryDo("shell-key-down", function () { raw(id, 1); });
      } else {
        var t0 = keyHeldAt[label];
        if (t0 == null) return;
        delete keyHeldAt[label];
        var elapsed = performance.now() - t0;
        var release = function () { if (keyHeldAt[label] == null) tryDo("shell-key-up", function () { raw(id, 0); }); };
        if (elapsed >= KEY_MIN_HOLD_MS) release();
        else setTimeout(release, Math.ceil(KEY_MIN_HOLD_MS - elapsed));
      }
    }
    function bindZone(b, label) {
      var z = false;
      function down(e) { e.preventDefault(); if (z) return; z = true; b.classList.add("active"); pressLogical(label, true); }
      function up(e)   { if (e) e.preventDefault(); if (!z) return; z = false; b.classList.remove("active"); pressLogical(label, false); }
      b.addEventListener("pointerdown", down);
      b.addEventListener("pointerup", up);
      b.addEventListener("pointercancel", up);
      b.addEventListener("pointerleave", up);
    }
    // Size the grid-mode #lcd to the ACTIVE model's real aspect ratio (the CSS
    // default only fits the 84×48 3310; a 96×65 7110 would otherwise stretch). Fixed
    // display width, height derived from LCDH/LCDW so pixels stay square.
    var GRID_LCD_W = 240;
    function sizeGridLcd() {
      if (document.body.classList.contains("shell-mode")) return;
      var w = LCDW || 84, h = LCDH || 48;
      canvas.style.width  = GRID_LCD_W + "px";
      canvas.style.height = Math.round(GRID_LCD_W * h / w) + "px";
    }
    function teardownShell() {
      document.body.classList.remove("shell-mode");
      shellBg = shellBgLit = shellPixel = null; shellLcdEl = null; shellLitState = -1;
      // Reseat the canvas at its grid home — ABOVE the keypad (before .pad), not
      // appended to the end (which put the screen below the keys). Then size it.
      if (lcdHome && canvas.parentElement !== lcdHome) {
        if (padHost && padHost.parentElement === lcdHome) lcdHome.insertBefore(canvas, padHost);
        else lcdHome.appendChild(canvas);
      }
      sizeGridLcd();
      if (shellHost) { shellHost.innerHTML = ""; shellHost.style.width = ""; shellHost.style.height = ""; }
      shellRoot = null;
      shellDef = null;
    }
    function buildShell(def) {
      shellHost.innerHTML = "";                   // detaches #lcd if it lived here — re-added below
      // Fit the phone to the viewport in BOTH axes so tall candybars (3310/3410)
      // never overflow. On narrow (mobile) screens use most of the width and leave
      // headroom for page chrome / a menu bar; on desktop cap the width.
      var vw = window.innerWidth || 400, vh = window.innerHeight || 800;
      var mobile = vw < 640;
      var maxW = mobile ? vw * 0.96 : SHELL_MAX_W;
      var maxH = vh * (mobile ? 0.70 : 0.92);
      var s = Math.min(1, maxW / def.w, maxH / def.h);
      var root = document.createElement("div");
      root.className = "shell-root";
      root.style.width = def.w + "px"; root.style.height = def.h + "px";
      root.style.transform = "scale(" + s + ")";
      shellHost.style.width = Math.round(def.w * s) + "px";
      shellHost.style.height = Math.round(def.h * s) + "px";
      root.style.setProperty("--shell-light", def.light || "#ffc24d");
      shellBg    = def.lcdColor   ? hexRgba(def.lcdColor)   : null;
      shellPixel = def.pixelColor ? hexRgba(def.pixelColor) : null;
      shellBgLit = def.lcdColorLit ? hexRgba(def.lcdColorLit)
                 : (shellBg ? shellBg.map(function (v, i) { return i < 3 ? Math.round(v + (255 - v) * 0.4) : v; }) : null);
      shellLitState = -1;

      var pimg = document.createElement("img");
      pimg.className = "shell-photo"; pimg.src = ASSET(def.img); pimg.draggable = false; pimg.alt = "";
      root.appendChild(pimg);

      if (def.mask) {                             // backlight glow through the glyph mask
        ["bloom", "sharp"].forEach(function (cls) {
          var dd = document.createElement("div");
          dd.className = "shell-light " + cls;
          dd.style.webkitMaskImage = "url(" + ASSET(def.mask) + ")";
          dd.style.maskImage = "url(" + ASSET(def.mask) + ")";
          root.appendChild(dd);
        });
      }

      var lcd = document.createElement("div");
      lcd.className = "shell-lcd";
      var L = def.lcd;
      lcd.style.left = L.left + "px"; lcd.style.top = L.top + "px";
      lcd.style.width = L.w + "px";   lcd.style.height = L.h + "px";
      if (L.radius != null) lcd.style.borderRadius = L.radius + "px";
      root.appendChild(lcd);
      shellLcdEl = lcd;
      if (L.canvasW) canvas.style.width = L.canvasW + "px";
      if (L.canvasH) canvas.style.height = L.canvasH + "px";
      lcd.appendChild(canvas);                    // reseat the shared #lcd canvas

      (def.zones || []).forEach(function (zz) {
        var b = document.createElement("button");
        b.className = "shell-zone" + (zz.key === "pwr" ? " pwr" : "");
        if (zz.path && zz.path.length) {
          // Vector hit-zone (angled/perspective photos, e.g. 3310/3410): each node is
          // [x, y, inX, inY, outX, outY]; use the anchor points. Position a button at the
          // node bounding box and clip its hit-area to the polygon through the anchors.
          var xs = zz.path.map(function (n) { return n[0]; });
          var ys = zz.path.map(function (n) { return n[1]; });
          var minX = Math.min.apply(null, xs), minY = Math.min.apply(null, ys);
          var maxX = Math.max.apply(null, xs), maxY = Math.max.apply(null, ys);
          b.style.left = minX + "px"; b.style.top = minY + "px";
          b.style.width = (maxX - minX) + "px"; b.style.height = (maxY - minY) + "px";
          var poly = zz.path.map(function (n) {
            return (n[0] - minX) + "px " + (n[1] - minY) + "px";
          }).join(",");
          b.style.webkitClipPath = "polygon(" + poly + ")";
          b.style.clipPath = "polygon(" + poly + ")";
        } else {
          b.style.left = zz.left + "px"; b.style.top = zz.top + "px";
          b.style.width = zz.w + "px"; b.style.height = zz.h + "px";
          b.style.borderRadius = zz.r || "50%";
        }
        b.setAttribute("aria-label", zz.key);
        bindZone(b, zz.key);
        root.appendChild(b);
      });

      shellHost.appendChild(root);
      shellRoot = root;
      shellDef = def;                             // geometry source for dct3Screenshot()
      document.body.classList.add("shell-mode");
    }
    // Pick shell vs grid for the active model + size the canvas. Call after each boot
    // (the firmware — hence the model — may have changed via the picker).
    function applyModel() {
      syncLcdGeometry();
      var model = (C.model && C.model()) || "3310";
      // Keep the page heading honest: the static HTML says 3310, but the booted
      // image decides the model (retro-phone opens on a 3410 and can switch at
      // runtime). Host pages (model menus) listen for dct3modelchange to sync
      // their own chrome.
      var h1 = document.querySelector(".topbar h1");
      if (h1) {
        h1.textContent = "Nokia " + model + " ";
        var sm = document.createElement("small");
        sm.textContent = "in your browser";
        h1.appendChild(sm);
      }
      try {
        window.dispatchEvent(new CustomEvent("dct3modelchange", { detail: { model: String(model) } }));
      } catch (_) {}
      // The game loader is 3410-only (only the 3410 has a PMM J2ME game store) — hide
      // it for every other model, and re-show it if the user switches back.
      var gl = document.getElementById("game-loader");
      if (gl) {
        gl.style.display = (String(model).indexOf("3410") >= 0) ? "" : "none";
        if (String(model).indexOf("3410") < 0) { gl.classList.remove("ok"); var gn = document.getElementById("game-name"); if (gn) gn.textContent = ""; }
      }
      var def = tryDo("pickShell", function () { return pickShell(model); });
      try {
        if (def) { buildShell(def); }
        else {
          // No photo shell → the model-aware grid. Build the keypad for this model's
          // VISUAL family (7110 = Navi roller) and reseat/size the screen above it.
          var fam = tryDo("kpFamily", function () { return (C.kpFamily && C.kpFamily()) | 0; }) || 0;
          buildKeypad(fam);
          teardownShell();
        }
      }
      catch (e) { showError("buildShell", e); teardownShell(); }
    }

    // -------------------------------------------------------------
    // Screenshot. Composites what the visitor is looking at into a PNG,
    // entirely in-page: the shell photo (already decoded in the DOM, so no
    // refetch and no canvas taint — it is same-origin either way), the lit
    // glass panel, and the live LCD canvas at its true pixel grid. Shell-less
    // models fall back to the bare screen, upscaled.
    //
    // Deliberately local-only: no external library, no network, no upload —
    // the image is handed straight to the browser's download/share sheet and
    // never leaves the device. Host pages get the API surface
    // (window.dct3Screenshot) plus window.DCT3_SHOT_PREFIX for the filename.
    // -------------------------------------------------------------
    function shotRect(c, x, y, w, h, r) {
      if (!r) { c.rect(x, y, w, h); return; }
      if (c.roundRect) { c.roundRect(x, y, w, h, r); return; }
      r = Math.min(r, w / 2, h / 2);              // pre-roundRect browsers
      c.moveTo(x + r, y);
      c.arcTo(x + w, y, x + w, y + h, r);
      c.arcTo(x + w, y + h, x, y + h, r);
      c.arcTo(x, y + h, x, y, r);
      c.arcTo(x, y, x + w, y, r);
      c.closePath();
    }
    // Draw the LCD canvas into dst at the given box, centred and unsmoothed so
    // the 84×48 framebuffer stays a crisp pixel grid instead of a blur.
    function shotDrawLcd(c, x, y, w, h) {
      c.imageSmoothingEnabled = false;
      c.msImageSmoothingEnabled = false;
      c.drawImage(canvas, x, y, w, h);
      c.imageSmoothingEnabled = true;
    }
    function shotCompose() {
      var out = document.createElement("canvas");
      var c, photo = shellRoot && shellRoot.querySelector("img.shell-photo");
      // --- Shell mode: photo + glass + screen, in the shell's own coordinates
      // (the photos are authored at exactly def.w × def.h, so this is native
      // resolution — the on-screen transform:scale is a viewport fit only).
      if (shellDef && photo && photo.complete && photo.naturalWidth) {
        var L = shellDef.lcd;
        out.width = shellDef.w; out.height = shellDef.h;
        c = out.getContext("2d");
        c.drawImage(photo, 0, 0, shellDef.w, shellDef.h);
        c.save();
        c.beginPath();
        shotRect(c, L.left, L.top, L.w, L.h, L.radius || 0);
        c.clip();
        // The panel background IS the backlight state (render() keeps it in
        // sync); reuse it so a screenshot of a dark phone looks dark.
        c.fillStyle = (shellLcdEl && shellLcdEl.style.background) || rgbStr(OFF);
        c.fillRect(L.left, L.top, L.w, L.h);
        var cw = L.canvasW || L.w, ch = L.canvasH || L.h;
        shotDrawLcd(c, L.left + (L.w - cw) / 2, L.top + (L.h - ch) / 2, cw, ch);
        c.restore();
        return out;
      }
      // --- Grid mode (no photo shell): the screen alone, upscaled to something
      // worth sharing, on a border of the current LCD background.
      var scale = Math.max(2, Math.round(600 / (canvas.width || 84)));
      var pad = 12 * (scale / 4 < 1 ? 1 : Math.round(scale / 4));
      out.width = canvas.width * scale + pad * 2;
      out.height = canvas.height * scale + pad * 2;
      c = out.getContext("2d");
      c.fillStyle = (shellRoot && shellLcdEl && shellLcdEl.style.background) || rgbStr(OFF);
      c.fillRect(0, 0, out.width, out.height);
      shotDrawLcd(c, pad, pad, canvas.width * scale, canvas.height * scale);
      return out;
    }
    function shotName() {
      var prefix = String(window.DCT3_SHOT_PREFIX || "nokia").replace(/[^\w.-]+/g, "-");
      var model = String((C.model && C.model()) || "dct3").replace(/[^\w.-]+/g, "-");
      var d = new Date(), p = function (n) { return (n < 10 ? "0" : "") + n; };
      return prefix + "-" + model + "-" + d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()) +
             "-" + p(d.getHours()) + p(d.getMinutes()) + p(d.getSeconds()) + ".png";
    }
    // Public API: resolves to the PNG Blob (host pages can do their own thing
    // with it). Rendering is synchronous off the live canvas, so the shot is
    // always the frame the visitor just saw.
    window.dct3Screenshot = function () {
      return new Promise(function (res, rej) {
        var out;
        try { out = shotCompose(); } catch (e) { rej(e); return; }
        if (!out.toBlob) { rej(new Error("canvas.toBlob unsupported")); return; }
        out.toBlob(function (b) { b ? res(b) : rej(new Error("encode failed")); }, "image/png");
      });
    };
    // Save it. Touch devices get the share sheet when the browser offers one
    // (on iOS that is the only route to the camera roll); everything else gets
    // a plain download. Both are user-gesture driven and purely local.
    function shotSave(blob) {
      var name = shotName();
      var coarse = (navigator.maxTouchPoints || 0) > 0 ||
                   (window.matchMedia && window.matchMedia("(pointer: coarse)").matches);
      if (coarse && window.File && navigator.share && navigator.canShare) {
        try {
          var file = new File([blob], name, { type: "image/png" });
          if (navigator.canShare({ files: [file] })) {
            return navigator.share({ files: [file] }).catch(function () {});
          }
        } catch (_) {}
      }
      var url = URL.createObjectURL(blob);
      var a = document.createElement("a");
      a.href = url; a.download = name; a.rel = "noopener";
      document.body.appendChild(a); a.click(); a.remove();
      setTimeout(function () { URL.revokeObjectURL(url); }, 30000);
      return Promise.resolve();
    }
    var btnShot = document.getElementById("btn-shot");
    if (btnShot) {
      var shotLabel = btnShot.textContent;
      var shotTimer = 0;
      btnShot.addEventListener("click", function () {
        function flash(msg) {
          btnShot.textContent = msg;
          if (shotTimer) clearTimeout(shotTimer);
          shotTimer = setTimeout(function () { btnShot.textContent = shotLabel; shotTimer = 0; }, 2200);
        }
        window.dct3Screenshot()
          .then(function (blob) { return shotSave(blob).then(function () { flash("✔ Saved"); }); })
          .catch(function (e) { showError("screenshot", e); flash("Couldn't save"); });
      });
    }

    // Physical keyboard → logical key label, routed through the model-aware
    // pressLogical() (defined with the shell code below). Non-flipped: keyboard "1"
    // drives the phone's 1 key; Enter = Menu/soft1, Backspace = Clear/soft2.
    var KMAP = {
      "1":"1","2":"2","3":"3","4":"4","5":"5","6":"6","7":"7","8":"8","9":"9",
      "0":"0","*":"*","#":"#",
      "ArrowUp":"up","ArrowDown":"down","Enter":"soft1","Backspace":"soft2",
      "ArrowLeft":"send","ArrowRight":"end"
    };
    // Keypad flip: a PC numpad has 7-8-9 on top; a phone has 1-2-3 on top, so games
    // that use the number grid for direction feel upside-down. When flipped, the top
    // and bottom digit rows swap (1↔7, 2↔8, 3↔9) so the numpad's layout matches the
    // phone's. Toggled by #btn-kbflip; persisted. Affects physical keys only — the
    // on-screen keypad still shows the true phone layout.
    var FLIP = { "1":"7","2":"8","3":"9","7":"1","8":"2","9":"3" };
    var kbFlip = false;
    try { kbFlip = localStorage.getItem("dct3.kbflip") === "1"; } catch (_) {}
    function mapKey(k) { return (kbFlip && FLIP[k]) ? FLIP[k] : k; }
    var held = {};
    var releaseTimer = {};
    var AUTOREPEAT_MS = 90;
    window.addEventListener("keydown", function (e) {
      // Don't intercept while focus is in a text/number input (dev panel).
      var t = e.target;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA")) return;
      var label = KMAP[mapKey(e.key)];
      if (!label) return;
      e.preventDefault();
      if (releaseTimer[e.key]) { clearTimeout(releaseTimer[e.key]); releaseTimer[e.key] = 0; }
      if (e.repeat || held[e.key]) return;
      held[e.key] = label;                 // remember the label pressed so a mid-hold flip still releases it
      pressLogical(label, true);
    });
    window.addEventListener("keyup", function (e) {
      var t = e.target;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA")) return;
      if (!held[e.key] && !KMAP[mapKey(e.key)]) return;
      e.preventDefault();
      var label = held[e.key] || KMAP[mapKey(e.key)];
      releaseTimer[e.key] = setTimeout(function () {
        releaseTimer[e.key] = 0;
        held[e.key] = false;
        pressLogical(label, false);
      }, AUTOREPEAT_MS);
    });

    // Keypad-flip toggle button.
    var kbFlipBtn = document.getElementById("btn-kbflip");
    function syncKbFlip() {
      if (!kbFlipBtn) return;
      kbFlipBtn.classList.toggle("on", kbFlip);
      kbFlipBtn.setAttribute("aria-pressed", kbFlip ? "true" : "false");
      kbFlipBtn.textContent = kbFlip ? "⇳ Numpad top row: 1-2-3" : "⇳ Numpad top row: 7-8-9";
    }
    if (kbFlipBtn) {
      kbFlipBtn.addEventListener("click", function () {
        kbFlip = !kbFlip;
        try { localStorage.setItem("dct3.kbflip", kbFlip ? "1" : "0"); } catch (_) {}
        syncKbFlip();
      });
      syncKbFlip();
    }

    // -------------------------------------------------------------
    // Power button. Tap = momentary press; 3 s hold = power off.
    // (The wasm side actually drives the timing; we just pass key
    // events. The 3 s rule comes from the firmware.)
    // -------------------------------------------------------------
    // Power button. The firmware owns the normal tap-vs-hold semantics:
    //   - 1 tap from idle → Profiles menu opens
    //   - 2nd tap → advances the Profiles menu
    //   - long-press while in menu → selects the highlighted profile
    //   - long-press from idle (~3 s) → firmware runs shutdown,
    //     eventually writes WDT=0 to CCONT → mad2.power_off latches
    //     → we halt
    // JS just passes press/release for those; no wall-clock timers gate
    // the firmware-driven flow.
    //
    // Hardware-likeness: on a real device the CCONT chip itself force-
    // resets the SoC after a sustained power-button hold (the user-of-
    // last-resort that recovers a frozen phone). We don't model that
    // at the CCONT level yet, so we substitute a 15-second wall-clock
    // hold here. If/when CCONT modelling lands, this JS timer should
    // retire and the reset should come from mad2.
    var btnPower = document.getElementById("btn-power");
    var POWER_FORCE_RESET_MS = 15000;
    var pwrForceTimer = 0;

    function forceReset() {
      // eslint-disable-next-line no-console
      if (typeof console !== "undefined") console.log("[dct3] 15 s power-hold → force reset (CCONT-faithful)");
      tryDo("power-force-reset", function () {
        doBoot();
        setStatus("Force reset (15 s hold) — booting…");
      });
    }
    function cancelForceReset() {
      if (pwrForceTimer) { clearTimeout(pwrForceTimer); pwrForceTimer = 0; }
    }

    if (btnPower) {
      btnPower.addEventListener("mousedown", function () {
        // Arm the force-reset timer on every press. Whether the phone
        // is on, off, in the menu, or crashed, holding 15 s reboots.
        cancelForceReset();
        pwrForceTimer = setTimeout(forceReset, POWER_FORCE_RESET_MS);

        if (halted) {
          // Off → on. Cold reboot for a fresh power-up state.
          tryDo("power-on-reboot", function () {
            doBoot();
            setStatus("Booting…");
          });
          return;
        }
        tryDo("power-down", function () { C.power(1); });
      });
      btnPower.addEventListener("mouseup", function () {
        cancelForceReset();
        if (halted) return;                   // power-on was handled on mousedown
        tryDo("power-up", function () { C.power(0); });
      });
      btnPower.addEventListener("mouseleave", function () {
        cancelForceReset();
        if (halted) return;
        tryDo("power-leave", function () { C.power(0); });
      });
    }

    var btnReboot = document.getElementById("btn-reboot");
    if (btnReboot) btnReboot.addEventListener("click", function () {
      tryDo("reboot", doBoot);
    });

    // -------------------------------------------------------------
    // Firmware upload. Write a raw .fls into MEMFS and reboot into it.
    // dct3_web_boot() re-reads /fw.fls on every call, so write + boot swaps
    // the image; applyModel() then re-mounts the matching shell + resizes the
    // canvas (the model can change). Same MEMFS path the legacy UI uses.
    // -------------------------------------------------------------
    var fwFile = document.getElementById("fw-file");
    var fwName = document.getElementById("fw-name");
    if (fwFile) fwFile.addEventListener("change", function (e) {
      var f = e.target.files && e.target.files[0];
      if (!f) return;
      f.arrayBuffer().then(function (ab) {
        var buf = new Uint8Array(ab);
        if (buf.length < 0x100000 || buf.length > 0x400000) {   // expect a ~1–4 MB raw dump
          if (fwName) fwName.textContent = "Unexpected size " + (buf.length / 1048576).toFixed(2) + " MB (want a raw .fls)";
          return;
        }
        if (!mod.FS || !mod.FS.writeFile) { showError("fw-write", new Error("MEMFS not available")); return; }
        try { mod.FS.writeFile("/fw.fls", buf); }
        catch (err) { showError("fw-write", err); if (fwName) fwName.textContent = "Load failed — see console"; return; }
        tryDo("fw-boot", doBoot);       // re-detect model → swap shell + canvas geometry
        var model = (C.model && C.model()) || "?";
        if (fwName) fwName.textContent = f.name + " · " + (buf.length / 1048576).toFixed(1) + " MB · " + model;
        setStatus("Running " + model + ".");
        // eslint-disable-next-line no-console
        if (typeof console !== "undefined") console.log("[dct3] firmware override:", f.name, buf.length, "bytes → model", model);
      }).catch(function (err) {
        showError("fw-read", err); if (fwName) fwName.textContent = "Read failed";
      });
    });

    // --- Game injection (3410 J2ME): REPLACE the built-in MIDlet slot with a new game.
    // A game = id 0x90 JAR (games block) + id 0x90 JAD (apps block) + id 0x91 registry
    // nodes in the PMMCAT store. We swap the new JAR/JAD payloads into the built-in
    // entries, rename the registry nodes, and re-serialize each 64 KB block
    // (core = f4 90|idx|55 ff|kind=sum16(payload)|val=len|offset=next-entry-pos). Replacing
    // reuses the factory game's ~41 KB slot + working menu registry, so real games fit and
    // appear where the built-in one did. Tree edges are by id-index (position-independent),
    // so re-serialize is safe.
    var PMM_MAGIC = [0x50, 0x4d, 0x4d, 0x43, 0x41, 0x54], PMM_BLOCK = 0x10000, PMM_CAT = 0x20;
    function pmmIsPK(p) { return p[0] === 0x50 && p[1] === 0x4b && p[2] === 0x03 && p[3] === 0x04; }
    function pmmSum16(b, n) { var s = 0; for (var i = 0; i < n; i++) s = (s + b[i]) & 0xffff; return s; }
    function pmmFindCats(img) {
      var out = [];
      for (var i = 0; i + 6 <= img.length; i++) {
        var hit = true;
        for (var j = 0; j < 6; j++) if (img[i + j] !== PMM_MAGIC[j]) { hit = false; break; }
        if (!hit) continue;
        var blk = i - 6;
        if (blk < 0 || img[blk] !== 0xf0 || img[blk + 1] !== 0xf0) continue;
        var p = blk + PMM_CAT;
        if (img[p] === 0xf4 && img[p + 4] === 0x55 && img[p + 5] === 0xff) out.push(blk);
      }
      return out;
    }
    function pmmNonFF(img, blk) {
      var e = Math.min(blk + PMM_BLOCK, img.length);
      while (e > blk && img[e - 1] === 0xff) e--;
      return e;
    }
    function pmmParse(img, blk) {
      var end = pmmNonFF(img, blk), starts = [], p;
      for (p = blk + PMM_CAT; p + 6 <= end; p++)
        if (img[p] === 0xf4 && img[p + 4] === 0x55 && img[p + 5] === 0xff) starts.push(p);
      var ents = [];
      for (var k = 0; k < starts.length; k++) {
        var s = starts[k], nx = (k + 1 < starts.length) ? starts[k + 1] : end;
        ents.push({ id: img[s + 1], idx: (img[s + 2] << 8) | img[s + 3],
          val: (img[s + 8] << 8) | img[s + 9], sOrig: s - blk, gap: nx - s,
          payload: img.slice(s + 12, nx) });
      }
      return ents;
    }
    function pmmSerialize(img, blk, ents) {
      var buf = new Uint8Array(PMM_BLOCK); buf.fill(0xff);
      for (var i = 0; i < PMM_CAT; i++) buf[i] = img[blk + i];
      var pos = PMM_CAT;
      for (var e2 = 0; e2 < ents.length; e2++) {
        var e = ents[e2], body;
        if (e.rebuild) {
          var val = e.payload.length, kind = pmmSum16(e.payload, val);
          body = new Uint8Array(12 + val);
          body[0] = 0xf4; body[1] = e.id; body[2] = (e.idx >> 8) & 0xff; body[3] = e.idx & 0xff;
          body[4] = 0x55; body[5] = 0xff;
          body[6] = (kind >> 8) & 0xff; body[7] = kind & 0xff;
          body[8] = (val >> 8) & 0xff; body[9] = val & 0xff;
          body.set(e.payload, 12);
        } else {
          body = img.slice(blk + e.sOrig, blk + e.sOrig + e.gap);
        }
        if (pos + body.length > PMM_BLOCK)
          throw new Error("game too big for the game slot by " + (pos + body.length - PMM_BLOCK) + " B — pick a smaller MIDlet");
        buf.set(body, pos);
        var off = pos + body.length;
        buf[pos + 10] = (off >> 8) & 0xff; buf[pos + 11] = off & 0xff;
        pos += body.length;
      }
      img.set(buf, blk);
    }
    function pmmNodeName(p) {
      var nc = (p[0] << 8) | p[1], n = "";
      for (var q = 0; q < nc; q++) if (p[2 + q * 2] === 0) n += String.fromCharCode(p[3 + q * 2]);
      return n;
    }
    function pmmRenameNode(p, name) {
      var nc = (p[0] << 8) | p[1], meta = p.slice(2 + nc * 2);
      var nb = new Uint8Array(2 + name.length * 2 + meta.length);
      nb[0] = (name.length >> 8) & 0xff; nb[1] = name.length & 0xff;
      for (var q = 0; q < name.length; q++) { nb[2 + q * 2] = 0; nb[3 + q * 2] = name.charCodeAt(q) & 0xff; }
      nb.set(meta, 2 + name.length * 2);
      return nb;
    }
    function pmmInjectGame(fw, jar, jad, name) {
      var img = new Uint8Array(fw), cats = pmmFindCats(img);
      if (!cats.length) throw new Error("no PMMCAT store — game injection is 3410-only");
      var games = -1, apps = -1, b, e;
      for (var ci = 0; ci < cats.length; ci++) {
        b = cats[ci]; e = pmmParse(img, b);
        var hasJar = e.some(function (x) { return x.id === 0x90 && pmmIsPK(x.payload); });
        var hasPlain = e.some(function (x) { return x.id === 0x90 && !pmmIsPK(x.payload); });
        if (games < 0 && hasJar) games = b;
        else if (apps < 0 && hasPlain) apps = b;
      }
      if (games < 0) throw new Error("no games block (id 0x90 JAR) — is this a 3410?");
      if (apps < 0) for (var cj = 0; cj < cats.length; cj++) if (cats[cj] !== games) { apps = cats[cj]; break; }
      var oldBase = null;
      var blocks = [games, apps];
      for (var bi = 0; bi < blocks.length; bi++) {
        var el = pmmParse(img, blocks[bi]);
        for (var xi = 0; xi < el.length; xi++) if (el[xi].id === 0x91) {
          var n = pmmNodeName(el[xi].payload);
          if (/\.jar$/i.test(n)) oldBase = n.replace(/\.jar$/i, "");
        }
      }
      function renameTrio(list) {
        for (var i = 0; i < list.length; i++) if (list[i].id === 0x91) {
          var nm = pmmNodeName(list[i].payload);
          if (oldBase && (nm === oldBase || nm === oldBase + ".jar" || nm === oldBase + ".jad")) {
            var suf = nm === oldBase ? "" : nm.slice(oldBase.length);
            list[i].payload = pmmRenameNode(list[i].payload, name + suf); list[i].rebuild = true;
          }
        }
      }
      var gList = pmmParse(img, games);
      for (var gi = 0; gi < gList.length; gi++) if (gList[gi].id === 0x90 && pmmIsPK(gList[gi].payload)) { gList[gi].payload = jar; gList[gi].rebuild = true; }
      renameTrio(gList); pmmSerialize(img, games, gList);
      var aList = pmmParse(img, apps);
      for (var ai = 0; ai < aList.length; ai++) if (aList[ai].id === 0x90 && !pmmIsPK(aList[ai].payload) && aList[ai].val > 20) { aList[ai].payload = jad; aList[ai].rebuild = true; }
      renameTrio(aList); pmmSerialize(img, apps, aList);
      return img;
    }

    // Public API + devtools control: inject a MIDlet into the live /fw.fls and reboot.
    window.dct3InjectGame = function (jar, jad, name) {
      var fw = mod.FS.readFile("/fw.fls");
      var modified = pmmInjectGame(fw, jar, jad, name);   // throws on bad image / too big
      mod.FS.writeFile("/fw.fls", modified);
      tryDo("game-inject", function () {
        doBoot(function () {
          // The 3410's PMM game store lives INSIDE the persisted NVRAM region
          // (games/apps catalog blocks at the top of flash). A saved snapshot
          // would overlay the OLD store over the fresh injection on this boot —
          // so apply the SAME swap to the snapshot (the PMM blocks are
          // self-contained, so the injector works on the snapshot buffer
          // directly). Runs after doBoot's save (which captured the latest
          // settings) and before the overlay. If the snapshot can't take the
          // game, drop it: the injected flash must win over stale settings.
          // Side effect, by design: once re-saved, the injected game survives
          // page reloads via the NVRAM snapshot.
          var snap = nvramCache[eeKey()];
          if (!snap) return;
          try {
            var patched = pmmInjectGame(snap, jar, jad, name);
            nvramCache[eeKey()] = patched;
            nvIdbPut(eeKey(), patched);
          } catch (e) {
            if (typeof console !== "undefined") console.warn("[dct3] saved NVRAM can't take the game (" + e.message + ") — dropping the save so the injection wins");
            delete nvramCache[eeKey()];
            nvIdbDel(eeKey());
          }
        });
      });
    };
    // --- .jad generation --------------------------------------------------------
    // iOS's file picker refuses .jad files, so the descriptor is OPTIONAL: when the
    // user supplies only the .jar we synthesize the .jad from the JAR's own
    // META-INF/MANIFEST.MF (which carries MIDlet-Name/-Version/-Vendor/-1 etc.),
    // adding the two attributes only the descriptor has: MIDlet-Jar-URL + -Size.
    // Tiny ZIP reader: end-of-central-directory → central header → local data.
    function zipU16(b, o) { return b[o] | (b[o + 1] << 8); }
    function zipU32(b, o) { return (b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0; }
    function zipFindEntry(u8, want) {
      var lo = Math.max(0, u8.length - 0x10000 - 22), e = -1;   // EOCD ≤ 64 KB comment
      for (var i = u8.length - 22; i >= lo; i--)
        if (u8[i] === 0x50 && u8[i + 1] === 0x4b && u8[i + 2] === 0x05 && u8[i + 3] === 0x06) { e = i; break; }
      if (e < 0) return null;
      var count = zipU16(u8, e + 10), p = zipU32(u8, e + 16);
      want = want.toLowerCase();
      for (var k = 0; k < count && p + 46 <= u8.length; k++) {
        if (zipU32(u8, p) !== 0x02014b50) break;
        var nlen = zipU16(u8, p + 28), xlen = zipU16(u8, p + 30), clen = zipU16(u8, p + 32);
        var name = "";
        for (var q = 0; q < nlen; q++) name += String.fromCharCode(u8[p + 46 + q]);
        if (name.toLowerCase() === want) {
          var lh = zipU32(u8, p + 42);
          if (zipU32(u8, lh) !== 0x04034b50) return null;
          var dOff = lh + 30 + zipU16(u8, lh + 26) + zipU16(u8, lh + 28);
          return { method: zipU16(u8, p + 10), data: u8.subarray(dOff, dOff + zipU32(u8, p + 20)) };
        }
        p += 46 + nlen + xlen + clen;
      }
      return null;
    }
    function zipReadText(u8, name) {          // -> Promise<string|null>
      var ent = zipFindEntry(u8, name);
      if (!ent) return Promise.resolve(null);
      if (ent.method === 0) return Promise.resolve(new TextDecoder().decode(ent.data));
      if (ent.method !== 8 || typeof DecompressionStream === "undefined") return Promise.resolve(null);
      try {
        var ds = new DecompressionStream("deflate-raw");
        return new Response(new Blob([ent.data]).stream().pipeThrough(ds)).text()
          .catch(function () { return null; });
      } catch (_) { return Promise.resolve(null); }
    }
    function makeJad(jar, jarName) {          // -> Promise<Uint8Array>
      var base = jarName.replace(/\.[^.]+$/, "");
      return zipReadText(jar, "META-INF/MANIFEST.MF").then(function (mf) {
        var attrs = {}, order = [];
        if (mf) {
          // Unfold manifest continuation lines (a leading space/tab continues the
          // previous attribute), then keep everything except jar-packaging noise.
          mf.replace(/\r\n?/g, "\n").replace(/\n[ \t]/g, "").split("\n").forEach(function (line) {
            var c = line.indexOf(":");
            if (c <= 0) return;
            var k = line.slice(0, c).trim(), v = line.slice(c + 1).trim();
            if (!k || /^(Manifest-Version|Created-By|Ant-Version)$/i.test(k)) return;
            if (!(k in attrs)) order.push(k);
            attrs[k] = v;
          });
        }
        function ensure(k, v) { if (!attrs[k]) { attrs[k] = v; if (order.indexOf(k) < 0) order.push(k); } }
        ensure("MIDlet-Name", base);
        ensure("MIDlet-Version", "1.0");
        ensure("MIDlet-Vendor", "unknown");
        ensure("MicroEdition-Profile", "MIDP-1.0");
        ensure("MicroEdition-Configuration", "CLDC-1.0");
        if (order.indexOf("MIDlet-Jar-URL") < 0) order.push("MIDlet-Jar-URL");
        attrs["MIDlet-Jar-URL"] = base + ".jar";
        if (order.indexOf("MIDlet-Jar-Size") < 0) order.push("MIDlet-Jar-Size");
        attrs["MIDlet-Jar-Size"] = String(jar.length);
        if (!attrs["MIDlet-1"] && typeof console !== "undefined")
          console.warn("[dct3] JAR manifest has no MIDlet-1 — generated .jad may be incomplete");
        var text = order.map(function (k) { return k + ": " + attrs[k]; }).join("\r\n") + "\r\n";
        return new TextEncoder().encode(text);
      });
    }

    // --- Uploaded-game library (IndexedDB) --------------------------------------
    // Every uploaded game is stashed in IndexedDB ({name, jar, jad ArrayBuffers}),
    // listed under the loader, and re-injectable with one click on a later visit —
    // no re-upload. All storage failures degrade to the old one-shot upload flow.
    var GDB_NAME = "dct3-games", GDB_STORE = "games";
    function gdb() {
      return new Promise(function (res, rej) {
        var r = indexedDB.open(GDB_NAME, 1);
        r.onupgradeneeded = function () { r.result.createObjectStore(GDB_STORE, { keyPath: "name" }); };
        r.onsuccess = function () { res(r.result); };
        r.onerror = function () { rej(r.error); };
      });
    }
    function gamesPut(rec) {
      return gdb().then(function (db) { return new Promise(function (res, rej) {
        var t = db.transaction(GDB_STORE, "readwrite");
        t.objectStore(GDB_STORE).put(rec);
        t.oncomplete = res; t.onerror = function () { rej(t.error); };
      }); });
    }
    function gamesAll() {
      return gdb().then(function (db) { return new Promise(function (res, rej) {
        var g = db.transaction(GDB_STORE, "readonly").objectStore(GDB_STORE).getAll();
        g.onsuccess = function () { res(g.result || []); };
        g.onerror = function () { rej(g.error); };
      }); }).catch(function () { return []; });
    }
    function gamesDelete(name) {
      return gdb().then(function (db) { return new Promise(function (res) {
        var t = db.transaction(GDB_STORE, "readwrite");
        t.objectStore(GDB_STORE).delete(name);
        t.oncomplete = res; t.onerror = res;
      }); }).catch(function () {});
    }

    var gameFile = document.getElementById("game-file");
    var gameName = document.getElementById("game-name");
    var gameLoader = document.getElementById("game-loader");
    var gameListEl = document.getElementById("game-list");
    function gameSay(m) { if (gameName) gameName.textContent = m; }

    // Inject a game record into the phone. If another model is active and the host
    // page published a model→firmware map (window.DCT3_MODEL_FW, e.g. retro-phone's
    // menu), auto-switch to the 3410 first; otherwise ask the user to switch.
    function injectGameRec(rec) {
      if (gameLoader) gameLoader.classList.remove("ok");
      function go() {
        try { window.dct3InjectGame(new Uint8Array(rec.jar), new Uint8Array(rec.jad), rec.name); }
        catch (err) { gameSay("Couldn't load that game: " + err.message); return; }
        gameSay("✔ " + rec.name + " loaded (" + (rec.jar.byteLength / 1024).toFixed(1) + " KB) — open Games ▸ More games");
        if (gameLoader) gameLoader.classList.add("ok");
        setStatus("Running " + ((C.model && C.model()) || "3410") + " with " + rec.name + ".");
      }
      var curModel = (C.model && C.model()) || "";
      if (curModel.indexOf("3410") >= 0) { go(); return; }
      var fwMap = window.DCT3_MODEL_FW || {};
      if (fwMap["3410"] && typeof window.dct3SwapFirmware === "function") {
        gameSay("Switching to the Nokia 3410…");
        window.dct3SwapFirmware(fwMap["3410"]).then(go);
      } else {
        gameSay("Games load on the Nokia 3410 — switch to it in the model menu first, then try again.");
      }
    }
    function renderGameList() {
      if (!gameListEl) return;
      gamesAll().then(function (list) {
        gameListEl.innerHTML = "";
        gameListEl.hidden = !list.length;
        if (!list.length) return;
        var h = document.createElement("p");
        h.className = "game-list-title";
        h.textContent = "Your games";
        gameListEl.appendChild(h);
        list.sort(function (a, b) { return (b.added || 0) - (a.added || 0); });
        list.forEach(function (rec) {
          var row = document.createElement("div"); row.className = "game-row";
          var play = document.createElement("button");
          play.type = "button"; play.className = "game-play";
          play.textContent = "▶ " + rec.name + " · " + Math.max(1, Math.round(rec.jar.byteLength / 1024)) + " KB";
          play.addEventListener("click", function () { injectGameRec(rec); });
          var del = document.createElement("button");
          del.type = "button"; del.className = "game-del";
          del.textContent = "✕"; del.title = "Remove " + rec.name + " from your games";
          del.addEventListener("click", function () { gamesDelete(rec.name).then(renderGameList); });
          row.appendChild(play); row.appendChild(del);
          gameListEl.appendChild(row);
        });
      });
    }
    renderGameList();

    if (gameFile) gameFile.addEventListener("change", function (e) {
      var files = e.target.files ? Array.prototype.slice.call(e.target.files) : [];
      var jarF = null, jadF = null;
      for (var i = 0; i < files.length; i++) {
        if (/\.jar$/i.test(files[i].name)) jarF = files[i];
        else if (/\.jad$/i.test(files[i].name)) jadF = files[i];
      }
      if (!jarF) { gameSay("Select the game's .jar file (the .jad is optional)."); return; }
      jarF.arrayBuffer().then(function (jarBuf) {
        var jar = new Uint8Array(jarBuf);
        if (jar[0] !== 0x50 || jar[1] !== 0x4b || jar[2] !== 0x03 || jar[3] !== 0x04) {
          gameSay("That .jar isn't a ZIP/JAR (no PK magic).");
          return;
        }
        var jadP = jadF
          ? jadF.arrayBuffer().then(function (b) { return new Uint8Array(b); })
          : makeJad(jar, jarF.name);
        return jadP.then(function (jad) {
          var base = jarF.name.replace(/\.[^.]+$/, "");
          var rec = { name: base, jar: jarBuf, jad: jad.buffer.slice(jad.byteOffset, jad.byteOffset + jad.byteLength), added: Date.now() };
          // Persist first so the game is in the library even if injection fails
          // (e.g. too big for the slot) or another model is active.
          gamesPut(rec).then(renderGameList, function (err) {
            if (typeof console !== "undefined") console.warn("[dct3] game persist failed:", err);
          });
          injectGameRec(rec);
        });
      }).catch(function (err) { showError("game-read", err); gameSay("Read failed"); });
      e.target.value = "";              // allow re-picking the same file later
    });

    // -------------------------------------------------------------
    // Audio: piezo buzzer (square) + DSP tone (sine, optional DTMF).
    // Disabled until the user ticks Sound — browsers require a user
    // gesture before an AudioContext is allowed to produce output, and
    // a fresh context may also start in `suspended` state, so we call
    // resume() on every enable.
    //
    // Buzzer frequency: 13 MHz core clock / buzzerDiv. The divider is
    // 16-bit; an ultrasonic value gets octave-folded to audible range
    // so short ringtone chirps still ring (matches legacy /web/).
    //
    // chirp = (rising-edges in low byte) << 0 | (div-at-edge << 8) —
    // captures sub-frame buzzer pulses our 16 ms poll would miss.
    // -------------------------------------------------------------
    var chkAudio = document.getElementById("chk-audio");
    var audioCtx = null, audioOn = false, pcmNode = null;
    // PCM ring pulled from the wasm, resampled by the node to the AudioContext rate.
    var PCMQ = 1 << 15, PCMQ_MASK = PCMQ - 1, PCM_PRIME = 2048;
    var pcmq = new Float32Array(PCMQ);
    var pcmqHead = 0, pcmqTail = 0, pcmFrac = 0, pcmPrimed = false, pcmLast = 0;
    var pcmRate = 18642;                     // producer Hz (refreshed from C.pcmRate)

    // Drain the wasm PCM ring into the JS queue — called every frame. The producer
    // streams constantly (silence included), so while audio is off we still drain and
    // discard, otherwise enabling later would replay ~1 s of stale ring.
    function pumpPCM() {
      if (!C.pcmRead) return;
      if (!audioOn || !audioCtx) {
        try { while (C.pcmRead(2048) === 2048) { /* discard */ } } catch (_) {}
        return;
      }
      var r = C.pcmRate ? (C.pcmRate() | 0) : 0; if (r > 0) pcmRate = r;
      var base = (C.pcmPtr() | 0) >> 1;       // int16 index into HEAP16
      for (;;) {
        var n = C.pcmRead(2048);
        if (n <= 0) break;
        var view = mod.HEAP16.subarray(base, base + n);
        for (var i = 0; i < n; i++) {
          if (((pcmqHead + 1) & PCMQ_MASK) === (pcmqTail & PCMQ_MASK)) break;   // full → drop
          pcmq[pcmqHead & PCMQ_MASK] = view[i] / 32768;
          pcmqHead++;
        }
        if (n < 2048) break;
      }
      // Cap latency: the emulator produces PCM in per-frame bursts and can run ahead,
      // so the queue can grow and delay tones (button → long-buffered beep). Keep only
      // a small cushion; drop the oldest samples so audio tracks the action.
      var LAT_CAP = PCM_PRIME * 2;              // ~85 ms @ 48 kHz
      if (pcmqHead - pcmqTail > LAT_CAP) pcmqTail = pcmqHead - LAT_CAP;
    }

    // Discard queued audio + drain the wasm ring — called after every (re)boot and
    // model swap so a beep/tone from the previous session can't bleed into the new one.
    function flushPCM() {
      pcmqHead = pcmqTail = 0; pcmFrac = 0; pcmPrimed = false; pcmLast = 0;
      try { if (C.pcmRead) while (C.pcmRead(2048) === 2048) { /* drain */ } } catch (_) {}
    }

    function audioStart() {
      var Ctor = window.AudioContext || window.webkitAudioContext;
      if (!Ctor) {
        showError("audio", new Error("WebAudio not available in this browser."));
        if (chkAudio) chkAudio.checked = false;
        return;
      }
      audioOn = true;
      if (audioCtx) { try { if (audioCtx.resume) audioCtx.resume(); } catch (_) {} return; }
      try {
        audioCtx = new Ctor();
        pcmRate = (C.pcmRate && C.pcmRate()) || 18642;
        pcmqHead = pcmqTail = 0; pcmFrac = 0; pcmPrimed = false; pcmLast = 0;
        // One node plays the unified earpiece PCM (buzzer + DSP tone/codec), resampled.
        pcmNode = audioCtx.createScriptProcessor(1024, 0, 1);
        pcmNode.onaudioprocess = function (e) {
          var out = e.outputBuffer.getChannelData(0);
          var ratio = pcmRate / audioCtx.sampleRate;   // input samples per output sample
          // Hold silent until a cushion is buffered so per-frame burst production can't
          // starve the node into clicks — a realtime source just needs a little latency.
          if (!pcmPrimed) {
            if (((pcmqHead - pcmqTail) & PCMQ_MASK) >= PCM_PRIME) pcmPrimed = true;
            else { out.fill(0); return; }
          }
          for (var i = 0; i < out.length; i++) {
            if (((pcmqHead - pcmqTail) & PCMQ_MASK) < 2) {          // underrun → fade + re-prime
              pcmLast *= 0.985; out[i] = pcmLast;
              if (Math.abs(pcmLast) < 1e-4) { pcmLast = 0; pcmPrimed = false; }
              continue;
            }
            var a = pcmq[pcmqTail & PCMQ_MASK], b = pcmq[(pcmqTail + 1) & PCMQ_MASK];
            out[i] = pcmLast = a + (b - a) * pcmFrac;
            pcmFrac += ratio;
            while (pcmFrac >= 1 && ((pcmqHead - pcmqTail) & PCMQ_MASK) >= 2) { pcmFrac -= 1; pcmqTail++; }
          }
        };
        pcmNode.connect(audioCtx.destination);
      } catch (e) {
        showError("audio-start", e);
        if (chkAudio) chkAudio.checked = false;
        audioOn = false;
      }
    }
    function audioStop() {
      audioOn = false;
      pcmqHead = pcmqTail = 0; pcmPrimed = false; pcmLast = 0;   // flush; re-enable starts clean
      try { if (audioCtx && audioCtx.suspend) audioCtx.suspend(); } catch (_) {}
    }
    if (chkAudio) chkAudio.addEventListener("change", function () {
      if (chkAudio.checked) audioStart(); else audioStop();
    });

    // Re-fit the phone shell on viewport changes (resize, orientation) — debounced.
    var _refitT = 0;
    window.addEventListener("resize", function () {
      clearTimeout(_refitT);
      _refitT = setTimeout(function () { if (shellRoot) applyModel(); }, 150);
    });

    // -------------------------------------------------------------
    // Console helpers for power users / debugging.
    // -------------------------------------------------------------
    window.dct3 = window.dct3 || {};
    window.dct3.api = C;
    window.dct3.mod = mod;
    window.dct3.showDev = function () { setDevOpen(true); };

  } // start()
})();
