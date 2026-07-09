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
        setBypass:      optWrap("dct3_web_set_bypass", null, ["number"]),
        setSkipSeclock: optWrap("dct3_web_set_skip_seclock", null, ["number"]),
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
        // Boot spike: explicit set to 0 (force OFF) rather than leaving
        // at -1 (auto). The 3310 profile's pin_verdict_default is also 0,
        // so the effective state should be identical, but setting it
        // explicitly removes any ambiguity and makes getSpike() report
        // the value we actually want.
        setSpike:         optWrap("dct3_web_set_spike", null, ["number"]),
        getSpike:         optWrap("dct3_web_get_spike", "number"),
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
        // plays at real time and any firmware path that depends on early
        // ordering may diverge — including the SIM-gate bypass arming,
        // which only activates after the DSP responder writes the 0xC8
        // verdict (src/web/main.c:579-581).
        run:    optWrap("dct3_web_run", null, ["number"]),
        step:   optWrap("dct3_web_step", "number"),

        // Model + LCD geometry (valid after boot) — drives shell selection and the
        // canvas size for non-3310 images loaded via the firmware picker.
        model:    optWrap("dct3_web_model", "string"),
        lcdW:     optWrap("dct3_web_lcd_w", "number"),
        lcdH:     optWrap("dct3_web_lcd_h", "number"),
        lcdBanks: optWrap("dct3_web_lcd_banks", "number"),
        // Model-aware logical key (KK_*): the shell sends soft1/up/1/… and the active
        // profile maps it to the matrix — so the page never hardcodes a per-model grid.
        keyLogical: optWrap("dct3_web_key_logical", null, ["number", "number"]),
        // Faithful variant: drives a REAL matrix edge (set on down / clear on up), no
        // auto-release — the firmware decides tap vs hold, so press-and-hold works
        // (= native GUI). Interactive input routes here; keyLogical is the fallback.
        keyLogicalRaw: optWrap("dct3_web_key_logical_raw", null, ["number", "number"]),
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
    var chkBypass     = document.getElementById("chk-bypass");
    var chkSkip       = document.getElementById("chk-skip-seclock");
    var chkRecover    = document.getElementById("chk-recover");
    var chkCharger    = document.getElementById("chk-charger");
    var chkSim        = document.getElementById("chk-sim");
    if (chkBypass)  { applyToggle(C.setBypass, chkBypass.checked);
                      chkBypass.addEventListener("change", function () { applyToggle(C.setBypass, chkBypass.checked); }); }
    if (chkSkip)    { applyToggle(C.setSkipSeclock, chkSkip.checked);
                      chkSkip.addEventListener("change", function () { applyToggle(C.setSkipSeclock, chkSkip.checked); }); }
    if (chkRecover) { applyToggle(C.setRecover, chkRecover.checked);
                      chkRecover.addEventListener("change", function () { applyToggle(C.setRecover, chkRecover.checked); }); }
    if (chkCharger) { applyToggle(C.setCharger, chkCharger.checked);
                      chkCharger.addEventListener("change", function () { applyToggle(C.setCharger, chkCharger.checked); }); }
    if (chkSim)     { applyToggle(C.setSim, chkSim.checked);
                      chkSim.addEventListener("change", function () { applyToggle(C.setSim, chkSim.checked); }); }

    // Force boot spike OFF before C.boot() runs (not -1/auto). g_spike_force
    // is a static C global that survives the boot reset, so this sticks.
    if (C.setSpike) tryDo("setSpike-init", function () { C.setSpike(0); });

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
    // (g_bypass_sim and g_skip_seclock are static and survive boot,
    //  so their UI toggles don't need re-application.)
    // -------------------------------------------------------------
    function reapplyPostBoot() {
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
        tryDo("fw-swap", function () { C.boot(); reapplyPostBoot(); applyModel(); });
      }, function (e) { showError("fw-swap", e); });
    };

    welcomeReady.then(function () {
      // Firmware-free hosting: fetch the configured default image into MEMFS
      // before the first boot. No-op when a fw was preloaded via dct3.data.
      return (window.DCT3_DEFAULT_FW && mod.FS && mod.FS.writeFile)
        ? loadFwToMemfs(window.DCT3_DEFAULT_FW) : null;
    }).then(function () {
      setStatus("Booting…");
      try { C.boot(); }
      catch (e) {
        setStatus("Firmware boot failed.");
        showError("boot", e);
        return;
      }
      reapplyPostBoot();
      applyModel();                      // mount the device shell for the booted model
      // Verify spike is actually 0 post-boot. resolve_spike() re-applies
      // pin_verdict_default and then the override, so this is the true
      // effective state.
      if (C.getSpike) {
        try {
          var sp = C.getSpike() | 0;
          // eslint-disable-next-line no-console
          if (typeof console !== "undefined") console.log("[dct3] boot spike effective =", sp, sp ? "(ON — would pin verdict to 0xC8)" : "(OFF — organic verdict)");
        } catch (_) {}
      }
      setStatus("Running.");
      lastT = null;                      // reset frame-loop pacing
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
          // during the shutdown animation.
          render();
          requestAnimationFrame(frame);
          return;
        }

        // Boot phase: busy-loop instruction-count chunks for up to 12 ms wall
        // time per frame, until the firmware crosses the OS-ready threshold.
        // Real-time pacing takes over after that. This matches /web/ exactly
        // and ensures the early-boot ordering (DSP verdict → spike arm → SIM
        // gate bypass) completes before any host-time-dependent UI work.
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
    // Keypad: on-screen buttons + physical keyboard.
    // Per the firmware's keymap table (flash 0x32E718), each key is
    // identified by a (row, col) matrix pair. data-row/data-col on
    // each button is the contract.
    // -------------------------------------------------------------
    // Tap-vs-hold model:
    //   - mousedown sets the matrix bit via dct3_web_key_raw.
    //   - mouseup clears the bit, BUT guarantees a minimum 200K-insn
    //     hold (≈15 ms wall at 1× pacing). Very-fast clicks still
    //     register, the firmware's ~222K-insn scan period never misses.
    //   - A real long-press keeps the bit set until physical release,
    //     so the firmware's own long-press handlers (e.g. long-Menu
    //     → voice dialling) trigger correctly.
    // Fallback: if dct3_web_key_raw isn't exported, fall back to the
    // sequenced dct3_web_key (which ignores physical release).
    var MIN_HOLD_INSNS = 200000;
    var MIN_HOLD_MS    = MIN_HOLD_INSNS / 13000;   // 13 MHz core ≈ 15 ms
    var pressed = new Map();                       // (row<<8)|col → state

    function press(row, col, down) {
      var code = ((row | 0) << 8) | (col | 0);
      if (down) {
        if (pressed.has(code)) return;             // already down — idempotent
        if (C.keyRaw) {
          try { C.keyRaw(row | 0, col | 0, 1); }
          catch (e) { showError("keyRaw down", e); return; }
        } else {
          try { C.key(row | 0, col | 0, 1); }     // sequenced fallback
          catch (e) { showError("key down", e); return; }
        }
        pressed.set(code, { downAt: performance.now() });
      } else {
        var info = pressed.get(code);
        if (!info) return;
        pressed.delete(code);
        if (!C.keyRaw) return;                     // fallback owns its up-edge
        var elapsed = performance.now() - info.downAt;
        if (elapsed >= MIN_HOLD_MS) {
          try { C.keyRaw(row | 0, col | 0, 0); }
          catch (e) { showError("keyRaw up", e); }
        } else {
          // Defer the up-edge so the firmware sees at least the scan/
          // deliver window. Without this an extra-fast click could miss
          // the ~222K-insn keypad scan and the press would never land.
          setTimeout(function () {
            try { C.keyRaw(row | 0, col | 0, 0); }
            catch (e) { showError("keyRaw up-defer", e); }
          }, Math.ceil(MIN_HOLD_MS - elapsed));
        }
      }
    }

    // Still set the sequenced auto-release window for the C.key fallback
    // path (in case an older wasm build doesn't export key_raw).
    if (C.setKeyHold) tryDo("setKeyHold", function () { C.setKeyHold(MIN_HOLD_INSNS); });

    var buttons = document.querySelectorAll(".key");
    buttons.forEach(function (b) {
      var row = parseInt(b.getAttribute("data-row"), 10);
      var col = parseInt(b.getAttribute("data-col"), 10);
      if (!isFinite(row) || !isFinite(col)) return;
      function down(e) { e.preventDefault(); b.classList.add("down"); press(row, col, true); }
      function up(e)   { e.preventDefault(); b.classList.remove("down"); press(row, col, false); }
      b.addEventListener("mousedown", down);
      b.addEventListener("mouseup", up);
      b.addEventListener("mouseleave", function () { b.classList.remove("down"); });
      b.addEventListener("touchstart", down, { passive: false });
      b.addEventListener("touchend", up, { passive: false });
    });

    // -------------------------------------------------------------
    // Device shell. Mounts a shared photo body (shells/<model>/) with the live
    // #lcd canvas seated on its glass and transparent hit-zones over each key,
    // routed through the model-aware logical-key API (so no per-model matrix is
    // hardcoded). Falls back to the .chassis grid when no shell is registered.
    // -------------------------------------------------------------
    var KK = { "0":1,"1":2,"2":3,"3":4,"4":5,"5":6,"6":7,"7":8,"8":9,"9":10,
               "*":11,"#":12, up:13, down:14, soft1:15, soft2:16, send:17, end:18,
               volup:19, voldown:20, pwr:21 };
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
            C.boot();
            reapplyPostBoot();
            applyModel();
            halted = false;
            lastT = null;
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
    function teardownShell() {
      if (!shellRoot) return;
      document.body.classList.remove("shell-mode");
      shellBg = shellBgLit = shellPixel = null; shellLcdEl = null; shellLitState = -1;
      canvas.style.width = ""; canvas.style.height = "";
      if (lcdHome) lcdHome.appendChild(canvas);   // canvas → chassis fallback home
      shellHost.innerHTML = ""; shellHost.style.width = ""; shellHost.style.height = "";
      shellRoot = null;
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
      document.body.classList.add("shell-mode");
    }
    // Pick shell vs grid for the active model + size the canvas. Call after each boot
    // (the firmware — hence the model — may have changed via the picker).
    function applyModel() {
      syncLcdGeometry();
      var def = tryDo("pickShell", function () { return pickShell((C.model && C.model()) || "3310"); });
      try { if (def) buildShell(def); else teardownShell(); }
      catch (e) { showError("buildShell", e); teardownShell(); }
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
    var held = {};
    var releaseTimer = {};
    var AUTOREPEAT_MS = 90;
    window.addEventListener("keydown", function (e) {
      // Don't intercept while focus is in a text/number input (dev panel).
      var t = e.target;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA")) return;
      var label = KMAP[e.key];
      if (!label) return;
      e.preventDefault();
      if (releaseTimer[e.key]) { clearTimeout(releaseTimer[e.key]); releaseTimer[e.key] = 0; }
      if (e.repeat || held[e.key]) return;
      held[e.key] = true;
      pressLogical(label, true);
    });
    window.addEventListener("keyup", function (e) {
      var t = e.target;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA")) return;
      var label = KMAP[e.key];
      if (!label) return;
      e.preventDefault();
      releaseTimer[e.key] = setTimeout(function () {
        releaseTimer[e.key] = 0;
        held[e.key] = false;
        pressLogical(label, false);
      }, AUTOREPEAT_MS);
    });

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
        C.boot();
        reapplyPostBoot();
        applyModel();
        halted = false;
        lastT = null;
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
            C.boot();
            reapplyPostBoot();
            applyModel();
            halted = false;
            lastT = null;
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
      tryDo("reboot", function () {
        // Cheapest reliable reboot: re-call dct3_web_boot. Resets the
        // emulator to a fresh power-on state with the same firmware,
        // then re-applies the UI toggles that mad2_init clobbers.
        C.boot();
        reapplyPostBoot();
        applyModel();
        lastT = null;                  // re-pace the frame loop
      });
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
        tryDo("fw-boot", function () {
          C.boot();
          reapplyPostBoot();
          applyModel();                 // re-detect model → swap shell + canvas geometry
          halted = false;
          lastT = null;
        });
        var model = (C.model && C.model()) || "?";
        if (fwName) fwName.textContent = f.name + " · " + (buf.length / 1048576).toFixed(1) + " MB · " + model;
        setStatus("Running " + model + ".");
        // eslint-disable-next-line no-console
        if (typeof console !== "undefined") console.log("[dct3] firmware override:", f.name, buf.length, "bytes → model", model);
      }).catch(function (err) {
        showError("fw-read", err); if (fwName) fwName.textContent = "Read failed";
      });
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
