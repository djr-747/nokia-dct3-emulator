// DCT3 web shell — Phase 2: boot the firmware in-browser, render the PCD8544
// LCD to a canvas, and feed the keypad matrix from buttons / the keyboard.
(function () {
  "use strict";

  const status = document.getElementById("status");
  const canvas = document.getElementById("lcd");
  const ctx2d = canvas.getContext("2d");
  // LCD geometry — defaults to the 3310 PCD8544 (84x48, 6 banks); re-read from the
  // wasm per detected model in applyModel() (3410 = 96x65/9 banks, etc.).
  let LCDW = 84, LCDH = 48, LCDBANKS = 6;
  let img = ctx2d.createImageData(LCDW, LCDH);

  // Logical keys (KeyId enum in src/models/model.h). The UI sends LOGICAL keys; the
  // active profile resolves each to that model's (row,col) matrix line in the wasm
  // (dct3_web_key_logical) — so the page never hardcodes a per-model matrix.
  const KK = { none:0, '0':1,'1':2,'2':3,'3':4,'4':5,'5':6,'6':7,'7':8,'8':9,'9':10,
    '*':11, '#':12, up:13, down:14, soft1:15, soft2:16, send:17, end:18,
    volup:19, voldown:20, pwr:21,
    wheelup:22, wheeldown:23, wheelpress:24 };   // 7110 Navi Roller (no model maps these yet)
  // Canonical replay/keymap labels → KeyId. 'menu'/'c' are the 3310's soft-key labels,
  // kept for replay-bundle compatibility (they map to soft1/soft2).
  const LABEL_KK = Object.assign({}, KK, { menu: KK.soft1, c: KK.soft2, names: KK.soft2 });

  // Nokia LCD palette: dark pixels on a yellow-green backlight. OFF_LIT is the
  // brighter background when the LCD backlight LEDs are on (McuGenIO 0x20 bit3).
  const ON = [0x2b, 0x39, 0x1a, 0xff];
  const OFF = [0xae, 0xc4, 0x8e, 0xff];
  const OFF_LIT_DEFAULT = [0xcf, 0xe9, 0x84, 0xff];   // classic yellow-green backlight glow
  let OFF_LIT = OFF_LIT_DEFAULT.slice();    // replaced by the profile LED colour when set
  // Keypad backlight: green-tinted button background when keypad LEDs are on
  // (CTRL-I/O-3 0x33 bit1) — toggled via the `kbd-lit` class on <body>.
  const KBD_LIT_DEFAULT = ".kbd-lit .key{background:#cfe986;border-color:#9fbf66}";
  const ledStyle = document.createElement("style");
  ledStyle.textContent = KBD_LIT_DEFAULT;
  document.head.appendChild(ledStyle);

  // Period-correct backlight colour from the model profile (dct3_web_led_rgb,
  // 0xRRGGBB; 0 = keep the classic yellow-green). Called after boot/applyModel
  // with the wasm-export table (C is scoped to the module-load callback).
  function applyLedColor(C) {
    if (!C.ledRgb) return;                       // older wasm without the export
    const c = C.ledRgb(0);
    // Reset to the classic glow when the new model reports no LED colour — otherwise
    // switching from a coloured model (5210 amber, 8250 blue) to a plain one (3310)
    // would leave the old backlight background in place.
    OFF_LIT = c ? [(c >> 16) & 255, (c >> 8) & 255, c & 255, 0xff] : OFF_LIT_DEFAULT.slice();
    const k = C.ledRgb(1) || c;
    if (k) {
      const hex = (v) => "#" + v.toString(16).padStart(6, "0");
      const dim = ((((k >> 16) & 255) * 3 >> 2) << 16) | ((((k >> 8) & 255) * 3 >> 2) << 8) | (((k & 255) * 3 >> 2));
      ledStyle.textContent = ".kbd-lit .key{background:" + hex(k) + ";border-color:" + hex(dim) + "}";
    } else {
      ledStyle.textContent = KBD_LIT_DEFAULT;    // likewise reset the keypad backlight tint
    }
  }

  // Version the wasm/data fetches too (set by the cache-busting loader in
  // index.html) so a refresh never serves a stale core.
  const moduleArgs = window.__DCT3_LOCATE ? { locateFile: window.__DCT3_LOCATE } : {};
  DCT3Module(moduleArgs).then(async (mod) => {
    const _a = window.__DCT3_ASSETS || {};
    console.log("[DCT3] main.js loaded — wasm=" + (_a.wasmHash || "?") + " builtAt=" + (_a.builtAt || "?") + " core=" + mod.cwrap("dct3_version", "string", [])());
    const C = {
      version: mod.cwrap("dct3_version", "string", []),
      boot: mod.cwrap("dct3_web_boot", "number", []),
      run: mod.cwrap("dct3_web_run", null, ["number"]),
      runCyc: mod.cwrap("dct3_web_run_cycles", null, ["number"]),
      cycles: mod.cwrap("dct3_web_cycles", "number", []),
      fb: mod.cwrap("dct3_web_fb", "number", []),
      step: mod.cwrap("dct3_web_step", "number", []),
      lcdWrites: mod.cwrap("dct3_web_lcd_writes", "number", []),
      lcdMode: mod.cwrap("dct3_web_lcd_mode", "number", []),
      leds: mod.cwrap("dct3_web_leds", "number", []),
      ledRgb: mod._dct3_web_led_rgb ? mod.cwrap("dct3_web_led_rgb", "number", ["number"]) : null,
      buzzerOn: mod.cwrap("dct3_web_buzzer_on", "number", []),
      buzzerDiv: mod.cwrap("dct3_web_buzzer_div", "number", []),
      buzzerVol: mod.cwrap("dct3_web_buzzer_vol", "number", []),
      buzzerChirp: mod.cwrap("dct3_web_buzzer_chirp", "number", []),
      pcmRead: mod.cwrap("dct3_web_pcm_read", "number", ["number"]),
      pcmPtr: mod.cwrap("dct3_web_pcm_ptr", "number", []),
      pcmRate: mod.cwrap("dct3_web_pcm_rate", "number", []),
      toneHz: mod.cwrap("dct3_web_tone_hz", "number", []),
      toneHz2: mod.cwrap("dct3_web_tone_hz2", "number", []),
      toneAmp: mod.cwrap("dct3_web_tone_amp", "number", []),
      vibraOn: mod.cwrap("dct3_web_vibra_on", "number", []),
      setBattery: mod.cwrap("dct3_web_set_battery", null, ["number"]),
      getBattery: mod.cwrap("dct3_web_get_battery", "number", []),
      setCharger: mod.cwrap("dct3_web_set_charger", null, ["number"]),
      getCharger: mod.cwrap("dct3_web_get_charger", "number", []),
      setSim: mod.cwrap("dct3_web_set_sim", null, ["number"]),
      getSim: mod.cwrap("dct3_web_get_sim", "number", []),
      simApdus: mod.cwrap("dct3_web_sim_apdus", "number", []),
      setSimPinEnabled: mod.cwrap("dct3_web_set_sim_pin_enabled", null, ["number"]),
      setSimPin: mod.cwrap("dct3_web_set_sim_pin", null, ["number"]),
      setSimPuk: mod.cwrap("dct3_web_set_sim_puk", null, ["number"]),
      simPinState: mod.cwrap("dct3_web_sim_pin_state", "number", []),
      ccontMask: mod.cwrap("dct3_web_ccont_mask", "number", []),
      ccontLines: mod.cwrap("dct3_web_ccont_lines", "number", []),
      rtcNow: mod.cwrap("dct3_web_rtc_now", "number", []),
      rtcMinEdges: mod.cwrap("dct3_web_rtc_min_edges", "number", []),
      rtcWrites: mod.cwrap("dct3_web_rtc_writes", "number", []),
      rtcRaw: mod.cwrap("dct3_web_rtc_raw", "number", []),
      rtcWrPc: mod.cwrap("dct3_web_rtc_wr_pc", "number", []),
      rtcWrLr: mod.cwrap("dct3_web_rtc_wr_lr", "number", []),
      getstrOn: mod.cwrap("dct3_web_getstr_on", null, ["number"]),
      getstrDump: mod.cwrap("dct3_web_getstr_dump", "string", []),
      key: mod.cwrap("dct3_web_key", null, ["number", "number", "number"]),
      keyLogical: mod.cwrap("dct3_web_key_logical", "number", ["number", "number"]),
      // Faithful interactive key: real matrix edge (set on down / clear on up), no
      // auto-release — the firmware's own timing decides tap vs hold (= native GUI).
      keyLogicalRaw: (() => { try { return mod.cwrap("dct3_web_key_logical_raw", "number", ["number", "number"]); } catch (e) { return null; } })(),
      model: mod.cwrap("dct3_web_model", "string", []),
      flashHi: mod.cwrap("dct3_web_flash_hi", "number", []),
      lcdW: mod.cwrap("dct3_web_lcd_w", "number", []),
      lcdH: mod.cwrap("dct3_web_lcd_h", "number", []),
      lcdBanks: mod.cwrap("dct3_web_lcd_banks", "number", []),
      kpFamily: mod.cwrap("dct3_web_kp_family", "number", []),
      power: mod.cwrap("dct3_web_power", null, ["number"]),
      setSkipSeclock: mod.cwrap("dct3_web_set_skip_seclock", null, ["number"]),
      seccodeReset: mod.cwrap("dct3_web_seccode_reset", "number", []),
      t0ticks: mod.cwrap("dct3_web_t0_ticks", "number", []),
      t1edges: mod.cwrap("dct3_web_t1_edges", "number", []),
      t0ctr: mod.cwrap("dct3_web_t0_counter", "number", []),
      t1ctr: mod.cwrap("dct3_web_t1_counter", "number", []),
      fiqs: mod.cwrap("dct3_web_fiqs", "number", []),
      irqs: mod.cwrap("dct3_web_irqs", "number", []),
      dbgWatch: mod.cwrap("dct3_web_dbg_watch", null, ["number", "number"]),
      dbgCount: mod.cwrap("dct3_web_dbg_count", "number", ["number"]),
      setRecover: mod.cwrap("dct3_web_set_recover", null,     ["number"]),
      getRecover: mod.cwrap("dct3_web_get_recover", "number", []),
      setRebootEarly: mod.cwrap("dct3_web_set_reboot_early", null,     ["number"]),
      getRebootEarly: mod.cwrap("dct3_web_get_reboot_early", "number", []),
      // Post-mortem text + assertion-ring counter. mad2_render_postmortem() populates
      // postmortem[0..len) at every reset catch — read as a UTF-8 byte view from HEAPU8.
      postmortemBuf: mod.cwrap("dct3_web_postmortem_buf", "number", []),
      postmortemLen: mod.cwrap("dct3_web_postmortem_len", "number", []),
      assertCount:   mod.cwrap("dct3_web_assert_count",   "number", []),
      heapFailCount: mod.cwrap("dct3_web_heap_fail_count", "number", []),
      heapFailLr:    mod.cwrap("dct3_web_heap_fail_lr",    "number", []),
      resetReq:        mod.cwrap("dct3_web_reset_request",     "number", []),
      resetLastReason: mod.cwrap("dct3_web_reset_last_reason", "number", []),
      traceOn:    mod.cwrap("dct3_web_trace_on",    null,     ["number"]),
      traceCount: mod.cwrap("dct3_web_trace_count", "number", []),
      traceCyc:   mod.cwrap("dct3_web_trace_cyc",   "number", ["number"]),
      traceStep:  mod.cwrap("dct3_web_trace_step",  "number", ["number"]),
      tracePc:    mod.cwrap("dct3_web_trace_pc",    "number", ["number"]),
      traceCpsr:  mod.cwrap("dct3_web_trace_cpsr",  "number", ["number"]),
      traceFiq:   mod.cwrap("dct3_web_trace_fiq",   "number", ["number"]),
      traceIrq:   mod.cwrap("dct3_web_trace_irq",   "number", ["number"]),
      ram: mod.cwrap("dct3_web_ram", "number", ["number"]),
      kbd: mod.cwrap("dct3_web_kbd", "number", ["number"]),
      setKeyHold: mod.cwrap("dct3_web_set_key_hold", null, ["number"]),
      getKeyHold: mod.cwrap("dct3_web_get_key_hold", "number", []),
      setOneshot: mod.cwrap("dct3_web_set_oneshot", null, ["number"]),
      msglogPc: mod.cwrap("dct3_web_msglog_pc", null, ["number"]),
      msglogBuf: mod.cwrap("dct3_web_msglog_buf", "number", []),
      msglogLrBuf: mod.cwrap("dct3_web_msglog_lrbuf", "number", []),
      msglogSize: mod.cwrap("dct3_web_msglog_size", "number", []),
      msglogW: mod.cwrap("dct3_web_msglog_w", "number", []),
      call: mod.cwrap("dct3_web_call", null, ["number", "number", "number"]),
      callCount: mod.cwrap("dct3_web_call_count", "number", []),
      regspike: mod.cwrap("dct3_web_regspike", null, ["number", "number", "number"]),
      regspikeCount: mod.cwrap("dct3_web_regspike_count", "number", []),
      ramPtr: mod.cwrap("dct3_web_ram_ptr", "number", []),
      eepromOff: mod.cwrap("dct3_web_eeprom_off", "number", []),
      eepromSize: mod.cwrap("dct3_web_eeprom_size", "number", []),
      eepromWrites: mod.cwrap("dct3_web_eeprom_writes", "number", []),
      i2cEepromPtr: mod.cwrap("dct3_web_i2c_eeprom_ptr", "number", []),     // external 24Cxx buffer (5110/…)
      i2cEepromSize: mod.cwrap("dct3_web_i2c_eeprom_size", "number", []),   // 0 = model has no external EEPROM
      i2cEepromWrites: mod.cwrap("dct3_web_i2c_eeprom_writes", "number", []),
      poweredOff: mod.cwrap("dct3_web_powered_off", "number", []),
      reg: mod.cwrap("dct3_web_reg", "number", ["number"]),
      pc: mod.cwrap("dct3_web_pc", "number", []),
    };

    const diagAddrs = document.getElementById("diag-addrs");
    const diagOut = document.getElementById("diag-out");
    const hex2 = (v) => v.toString(16).toUpperCase().padStart(2, "0");

    // --- UI message logger ---------------------------------------------------
    // Known message ids (low 14 bits; top 2 bits = arg count). From the NokiX SDK
    // / boot-trace notes — extend as needed.
    const MSG_NAMES = {
      // Keypress events (NokiX ui_messages.h). NOTE: KEYPRESS/KEYRELEASE (0xc8/0xc9)
      // are the per-key down/up events; KEY_DOWN/KEY_UP (0x366/0x367) are a separate
      // higher-level pair. A normal tap emits 0xc8 then 0xc9 (and 0xca in numeric mode).
      0x0c8: "KEYPRESS", 0x0c9: "KEYRELEASE", 0x0ca: "KEYPRESS_CHAR",
      0x0cb: "KEYPRESS_NEW_CHAR", 0x0cc: "KEYPRESS_MODIFY_CHAR",
      0x366: "KEY_DOWN", 0x367: "KEY_UP", 0x370: "KEY_PICKUP", 0x371: "KEY_HANGUP",
      0x37a: "KEY_ACTION1", 0x37b: "KEY_ACTION2",
      // Tones / lights / timers (often follow a keypress).
      0x190: "PLAY_SYSTEM_TONE", 0x1b59: "PLAY_KEYPAD_TONE",
      0x1b5a: "START_TIMER", 0x1b5b: "ABORT_TIMER", 0x1f5: "TIMED_LIGHTS_ON",
      // UI / dispatcher / window.
      0x226: "DISPLAY_MESSAGE", 0x320: "POWER_UP", 0x32d: "NUMERIC_MODE",
      0x35c: "CLEAR", 0x517: "UPPER_SET", 0x57c: "DIALOG_DRAW", 0x5a7: "ANIMATION_POINT",
      0x5dc: "D_INIT", 0x5de: "D_QUIT", 0x5e0: "START_HANDLER", 0x5e2: "D_PHONE_STARTUP",
      0xbcc: "WIN_CREATE", 0xc3a: "WIN_DESC",
    };
    const msgName = (id) => MSG_NAMES[id & 0x3fff] || "";
    const msglogChk = document.getElementById("chk-msglog");
    const msglogOut = document.getElementById("msglog-out");
    let msglogLastW = 0;
    let msgPerFrame = 0;                  // most recent non-zero per-frame count
    const msgTotals = new Map();          // cumulative counts (persist across frames)
    const msgCaller = new Map();          // id -> Map(callerLR -> count)
    function msglogReset() { msgTotals.clear(); msgCaller.clear(); msglogLastW = C.msglogW(); msgPerFrame = 0; }
    function renderMsglog() {
      if (!msglogChk || !msglogChk.checked) { if (msglogOut) msglogOut.textContent = ""; return; }
      const w = C.msglogW(), N = C.msglogSize();
      const base = C.msglogBuf() >> 2;                 // uint32 index into HEAP
      const lbase = C.msglogLrBuf() >> 2;
      const ring = mod.HEAPU32.subarray(base, base + N);
      const lring = mod.HEAPU32.subarray(lbase, lbase + N);
      let from = msglogLastW, n = w - from;
      if (n > N) { from = w - N; n = N; }              // capped by ring size
      for (let i = from; i < w; i++) {                 // accumulate into the totals
        const id = ring[i % N] >>> 0, lr = lring[i % N] >>> 0;
        msgTotals.set(id, (msgTotals.get(id) || 0) + 1);
        let cm = msgCaller.get(id); if (!cm) { cm = new Map(); msgCaller.set(id, cm); }
        cm.set(lr, (cm.get(lr) || 0) + 1);
      }
      msglogLastW = w;
      if (n > 0) msgPerFrame = n;
      const argc = (id) => (id >>> 14) & 3;
      const topLr = (id) => { const cm = msgCaller.get(id); if (!cm) return 0; return [...cm.entries()].sort((a,b)=>b[1]-a[1])[0][0]; };
      const rows = [...msgTotals.entries()].sort((a, b) => b[1] - a[1]).slice(0, 14);
      const text = `this frame: ${n}   last active: ${msgPerFrame}/frame   total: ${w}\n` +
        rows.map(([id, c]) => {
          const nm = msgName(id);
          return `  ${c.toString().padStart(5)}×  0x${id.toString(16).padStart(4, "0")}` +
                 (argc(id) ? ` [${argc(id)}p]` : "") + (nm ? `  ${nm}` : "").padEnd(nm ? 0 : 0) +
                 `   <-${"0x" + (topLr(id) >>> 0).toString(16)}`;
        }).join("\n");
      // Only touch the DOM when the text changes, so a selection isn't wiped
      // every frame (lets you select/copy once activity goes idle).
      if (text !== msglogOut.textContent) msglogOut.textContent = text;
    }

    function renderDiag() {
      let lines = [];
      const addrs = diagAddrs.value.split(",").map((s) => s.trim()).filter(Boolean);
      for (const a of addrs) {
        const addr = parseInt(a, 16) >>> 0;
        if (!isNaN(addr)) lines.push("[" + addr.toString(16).toUpperCase().padStart(6, "0") + "] = " + hex2(C.ram(addr) & 0xff));
      }
      let kb = "kbd cols rows0-7:";
      for (let r = 0; r < 8; r++) kb += " " + hex2(C.kbd(r));
      lines.push(kb + "  special:" + hex2(C.kbd(8)));
      diagOut.textContent = lines.join("\n");
      renderMsglog();
    }

    // --- Timer ticks/edges + handler-reset trace -----------------------------
    // Timer0/FIQ4 = the scheduler heartbeat; Timer1/FIQ5 overflow edges drive the
    // soft-timer walk (0x298F68) that fires the inactivity/menu-close/LED timers
    // (the "handler reset" that returns to home). Plus up to 6 PC hit-counters so
    // you can watch e.g. the walk + the return-to-home dispatch fire in real time.
    const timersOut = document.getElementById("timers-out");
    const tracePcsIn = document.getElementById("trace-pcs");
    let tracePcs = [];
    function applyTracePcs() {
      tracePcs = (tracePcsIn ? tracePcsIn.value : "").split(",").map((s) => s.trim())
        .filter(Boolean).map((s) => parseInt(s, 16) >>> 0).filter((n) => !isNaN(n)).slice(0, 6);
      for (let i = 0; i < 6; i++) C.dbgWatch(i, tracePcs[i] || 0);   // 0 clears the slot + resets its count
    }
    if (tracePcsIn) { tracePcsIn.addEventListener("change", applyTracePcs); applyTracePcs(); }
    let tPrevT0 = 0, tPrevT1 = 0, tPrevMs = 0;
    function renderTimers() {
      if (!timersOut) return;
      const now = performance.now(), t0 = C.t0ticks(), t1 = C.t1edges();
      let rate = "";
      if (tPrevMs) { const dt = (now - tPrevMs) / 1000;
        if (dt > 0) rate = `  (T0 ${Math.round((t0 - tPrevT0) / dt)}/s, T1 ${((t1 - tPrevT1) / dt).toFixed(1)}/s)`; }
      tPrevT0 = t0; tPrevT1 = t1; tPrevMs = now;
      let s = `T0 ticks ${t0 | 0}   T1 edges/FIQ5 ${t1 | 0}${rate}\n`
            + `T0ctr ${(C.t0ctr() & 0xffff).toString(16)}  T1ctr ${(C.t1ctr() & 0xffff).toString(16)}`
            + `  FIQ ${C.fiqs() | 0}  IRQ ${C.irqs() | 0}  disp 0x${(C.ram(0x11FD08) & 0xff).toString(16)}`;
      if (tracePcs.length) s += "\n" + tracePcs.map((p, i) => `@${p.toString(16)}×${C.dbgCount(i) | 0}`).join("   ");
      if (s !== timersOut.textContent) timersOut.textContent = s;
    }

    console.log("[DCT3] core", C.version());

    // --- EEPROM persistence ---------------------------------------------------
    // The firmware's NVRAM (settings, clock, contacts, security config) lives in
    // the EEPROM region (device 0x3D0000-0x400000) of the flat memory. We snapshot
    // it to localStorage when it changes and overlay it on the next boot, so changes
    // survive reloads. Code/PPM (below the profile's eeprom_base) is never persisted (load fresh).
    // EEPROM/NVRAM is persisted PER FIRMWARE IMAGE: the localStorage key is namespaced
    // by a hash of the firmware's CODE region (everything below the NVRAM partition),
    // so a picked FuBu image and the built-in image each keep their own saved NVRAM and
    // never clobber each other. (Old single-key "dct3_v639_eeprom_v1" is migrated below.)
    let curFwId = "default";
    function fwIdentity() {
      try {
        const fw = mod.FS.readFile("/fw.fls");          // Uint8Array of the live image
        const codeLen = Math.max(0, fw.length - C.eepromSize());   // exclude the NVRAM partition
        let h = 0x811c9dc5 >>> 0;                        // FNV-1a over the code region
        for (let i = 0; i < codeLen; i++) { h ^= fw[i]; h = Math.imul(h, 0x01000193) >>> 0; }
        h = (h ^ codeLen) >>> 0;
        return ("0000000" + h.toString(16)).slice(-8);
      } catch (e) { return "default"; }
    }
    function eeKey() { return "dct3_eeprom_" + curFwId; }
    const EE_KEY_LEGACY = "dct3_v639_eeprom_v1";
    let lastFwBase = "";              // base name of the active firmware (for merged-.fls download names)

    // --- Picked-firmware persistence across page reloads (IndexedDB) ----------------
    // The firmware picker writes /fw.fls into MEMFS, which is rebuilt from the baked-in
    // image on every page load. To make "load a FuBu .fls, change settings, reload" keep
    // BOTH the image and its (per-image) NVRAM, we stash the last picked image in
    // IndexedDB and restore it before the first boot. dct3ClearFirmware() reverts to the
    // built-in image. All guarded so a storage failure never blocks boot.
    const IDB_DB = "dct3", IDB_STORE = "fw", IDB_KEY = "lastfw";
    function idb() {
      return new Promise((res, rej) => {
        const r = indexedDB.open(IDB_DB, 1);
        r.onupgradeneeded = () => r.result.createObjectStore(IDB_STORE);
        r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error);
      });
    }
    async function idbPutFw(rec) { try { const db = await idb(); await new Promise((res, rej) => {
      const t = db.transaction(IDB_STORE, "readwrite"); t.objectStore(IDB_STORE).put(rec, IDB_KEY);
      t.oncomplete = res; t.onerror = () => rej(t.error); }); } catch (e) { console.warn("[DCT3] firmware persist failed:", e); } }
    async function idbGetFw() { try { const db = await idb(); return await new Promise((res, rej) => {
      const t = db.transaction(IDB_STORE, "readonly"); const g = t.objectStore(IDB_STORE).get(IDB_KEY);
      g.onsuccess = () => res(g.result || null); g.onerror = () => rej(g.error); }); } catch (e) { return null; } }
    async function idbClearFw() { try { const db = await idb(); await new Promise((res) => {
      const t = db.transaction(IDB_STORE, "readwrite"); t.objectStore(IDB_STORE).delete(IDB_KEY); t.oncomplete = res; }); } catch (e) {} }
    window.dct3ClearFirmware = function () { idbClearFw().then(() => location.reload()); };

    function b64enc(u8) {
      let s = "", CH = 0x8000;
      for (let i = 0; i < u8.length; i += CH) s += String.fromCharCode.apply(null, u8.subarray(i, i + CH));
      return btoa(s);
    }
    function b64dec(b64) {
      const s = atob(b64), u8 = new Uint8Array(s.length);
      for (let i = 0; i < s.length; i++) u8[i] = s.charCodeAt(i);
      return u8;
    }
    let eeLastWrites = -1;
    // Set during an explicit reset so the reload's beforeunload/visibility handlers
    // can't immediately re-persist the NVRAM we just wiped (the reset-undo bug).
    let eepromSuppressSave = false;
    // User-facing persistence toggle (defaults OFF so boots are repeatable from the
    // image's own EEPROM; mirrored to the chk-persist-eeprom checkbox below). When
    // OFF: skip the auto-load at boot AND skip auto-save (interval / visibility /
    // unload). The manual Save button still works — explicit intent.
    // Why default off: saved Profile/clock/ringtone settings silently change boot
    // behaviour, so two pasters of the same Replay JSON could diverge on each other's
    // NVRAM. Off = reproducible. Tick to re-enable persistence for a normal session.
    let eepromPersist = false;
    // True once a user-supplied firmware is loaded via the picker. Persistence still
    // works (keyed per-image via eeKey()); this flag only drives the "restore the
    // picked image across reloads" path (IndexedDB) and the UI label.
    let customFw = false;
    // A user-loaded EEPROM .bin (via "Load EEPROM"). When set, boot() overlays it into
    // the partition AFTER the C boot reloads flash, taking precedence over the
    // localStorage NVRAM. Persists across reboots within the session (Reset EEPROM
    // clears it). No blank-guard: loading a virgin/blank dump on purpose is allowed.
    let pendingEeprom = null;
    // Power-off state. `poweringOff` = wall-clock ms when a long power-hold started the
    // shutdown (0 = not shutting down); `halted` = the CPU is parked (phone is off).
    let poweringOff = 0;
    let halted = false;
    let crashed = false;            // CPU halted on a detected crash/spin (until reboot)
    // Self-heal for a corrupt persisted NVRAM. A localStorage overlay can carry a partition
    // state that crashes the next cold boot (see eeprom-corruption notes). If a boot-time
    // spin is detected while such an overlay was applied, we clear it once and reboot from
    // the image's own (clean) EEPROM. eeOverlaidThisBoot is true ONLY for the localStorage
    // overlay (never a user-loaded EEPROM file). eeSkipOverlay forces the recovery boot to
    // ignore localStorage; eeRecovered makes it one-shot so we never reboot-loop.
    let eeOverlaidThisBoot = false, eeSkipOverlay = false, eeRecovered = false;
    function eepromSlice() {
      const base = C.ramPtr() + C.eepromOff(), size = C.eepromSize();
      return mod.HEAPU8.subarray(base, base + size);
    }
    // A region is "blank" if every byte is erased (0xFF) or zero. That isn't real
    // NVRAM — never persist or overlay it: injecting a blank EEPROM breaks v6.39's
    // first-boot consistency checks (see memory firmware-inventory).
    function eepromBlank(u8) {
      for (let i = 0; i < u8.length; i++) { const x = u8[i]; if (x !== 0 && x !== 0xff) return false; }
      return true;
    }
    // Returns "saved" | "nochange" | "blank" | "off" | "error". eepromWrites() now
    // counts programs that landed in the 0x3D0000 NVRAM partition specifically, so
    // the auto-save fires exactly when the firmware commits a setting to flash (e.g.
    // a language change writes its record there — verified). `force` bypasses that
    // gate for a manual Save (snapshots whatever is in the region right now); DCT3
    // also DEFERS some commits, so force is the fallback when a change is still only
    // in the working-RAM shadow and hasn't been flushed to the partition yet.
    function saveEeprom(force) {
      if (eepromSuppressSave) return "off";
      if (!force && !eepromPersist) return "off";   // auto-saves disabled by user
      if (!force && C.eepromWrites() === 0) return "nochange";  // auto-save: real program only
      try {
        const slice = eepromSlice();
        if (eepromBlank(slice)) return "blank";  // never persist a blank/erased region
        localStorage.setItem(eeKey(), b64enc(slice));            // per-image key
        eeLastWrites = C.eepromWrites();
        return "saved";
      } catch (e) { console.warn("[DCT3] EEPROM save failed:", e); return "error"; }
    }
    function loadEeprom() {
      if (!eepromPersist) return false;   // user opted out — boot the image's own NVRAM
      try {
        let b64 = localStorage.getItem(eeKey());
        // One-time migration: an older build stored the default image's NVRAM under a
        // single fixed key. Adopt it for whatever image is active on first run.
        if (!b64 && curFwId === "default") b64 = localStorage.getItem(EE_KEY_LEGACY);
        if (!b64) return false;
        const u8 = b64dec(b64);
        if (u8.length !== C.eepromSize()) return false;
        if (eepromBlank(u8)) {                   // never inject a blank EEPROM; self-heal
          console.warn("[DCT3] stored EEPROM is blank — ignoring it, using the embedded image");
          try { localStorage.removeItem(eeKey()); } catch (e) {}
          return false;
        }
        mod.HEAPU8.set(u8, C.ramPtr() + C.eepromOff());
        console.log("[DCT3] restored", u8.length, "B EEPROM from localStorage");
        return true;
      } catch (e) { console.warn("[DCT3] EEPROM load failed:", e); return false; }
    }

    // --- External I2C-EEPROM (24Cxx) persistence -------------------------------
    // The early-MAD2 serial-bus models (5110/6110/…) keep their NVRAM in a separate 24Cxx chip
    // (a distinct buffer, NOT the flat-RAM partition), so it gets its OWN localStorage key +
    // overlay — mirroring the flash path above. The C side eager-loads the baked NokiX default
    // into the buffer during boot(), so the overlay below lands on a populated image. Every
    // function is a no-op when the active model has no external EEPROM (i2cEepromSize() == 0),
    // so the in-flash-EEPROM models (3310 etc.) are unaffected.
    function i2cKey() { return "dct3_i2ceeprom_" + curFwId; }
    let i2cLastWrites = -1;
    function i2cSlice() {
      const sz = C.i2cEepromSize(); if (!sz) return null;
      const base = C.i2cEepromPtr();
      return mod.HEAPU8.subarray(base, base + sz);
    }
    function saveI2cEeprom(force) {
      if (eepromSuppressSave) return "off";
      if (!force && !eepromPersist) return "off";
      const slice = i2cSlice(); if (!slice) return "off";        // model has no external EEPROM
      if (!force && C.i2cEepromWrites() === 0) return "nochange"; // auto-save: real write only
      try {
        if (eepromBlank(slice)) return "blank";
        localStorage.setItem(i2cKey(), b64enc(slice));
        i2cLastWrites = C.i2cEepromWrites();
        return "saved";
      } catch (e) { console.warn("[DCT3] I2C EEPROM save failed:", e); return "error"; }
    }
    function loadI2cEeprom() {
      if (!eepromPersist) return false;
      const sz = C.i2cEepromSize(); if (!sz) return false;
      try {
        const b64 = localStorage.getItem(i2cKey());
        if (!b64) return false;
        const u8 = b64dec(b64);
        if (u8.length !== sz) return false;
        if (eepromBlank(u8)) { try { localStorage.removeItem(i2cKey()); } catch (e) {} return false; }
        mod.HEAPU8.set(u8, C.i2cEepromPtr());
        console.log("[DCT3] restored", u8.length, "B external I2C EEPROM from localStorage");
        return true;
      } catch (e) { console.warn("[DCT3] I2C EEPROM load failed:", e); return false; }
    }
    // Persist BOTH NVRAM regions (flash partition + external I2C); each self-gates on presence.
    function saveNvram(force) { const r = saveEeprom(force); saveI2cEeprom(force); return r; }

    function boot() {
      const rc = C.boot();
      if (rc !== 0) { status.textContent = "Boot failed (rc=" + rc + ")"; throw new Error("boot " + rc); }
      curFwId = fwIdentity();       // identify the active image so NVRAM is keyed to it
      applyModel();                 // detected model → LCD size + keypad family (auto screen/keymat)
      applyLedColor(C);             // period-correct backlight glow (5210 orange, 8250 blue, ...)
      halted = false; poweringOff = 0; crashed = false;   // a (re)boot powers the phone back on
      { const p = document.getElementById("crash-panel"); if (p) p.style.display = "none"; }
      resetReplayLog();             // step counter restarts at 0 — start a fresh replay log
      if (C.traceOn) C.traceOn(1);  // arm divergence-finder so any halt has a comparable trace
      eeOverlaidThisBoot = false;
      const skipOverlay = eeSkipOverlay;   // capture before reset (also gates the I2C overlay below)
      if (eeSkipOverlay) {          // recovery boot: ignore localStorage, use the image's own EEPROM
        eeSkipOverlay = false;
        console.log("[DCT3] recovery boot — using the image's embedded EEPROM (localStorage overlay skipped)");
      } else if (pendingEeprom) {   // a user-loaded EEPROM file wins over localStorage/embedded
        mod.HEAPU8.set(pendingEeprom, C.ramPtr() + C.eepromOff());
        console.log("[DCT3] overlaid loaded EEPROM file (" + pendingEeprom.length + " B)");
      } else {
        eeOverlaidThisBoot = loadEeprom();   // overlay this image's persisted NVRAM before run
      }
      // External I2C EEPROM always overlays from localStorage (no user-file path), on top of the
      // baked default the C side eager-loaded — except on a recovery boot. No-op without one.
      if (!skipOverlay) loadI2cEeprom();
      eeLastWrites = C.eepromWrites();
      i2cLastWrites = C.i2cEepromWrites();
      // mad2_init resets the SIM to its defaults (present, PIN 1234) on every boot, so
      // re-apply whatever the SIM panel shows — otherwise the "SIM inserted" checkbox
      // (and PIN settings) silently disagree with the model after a (re)boot.
      if (chkSim && C.setSim) C.setSim(chkSim.checked ? 1 : 0);
      if (chkPin && C.setSimPinEnabled) C.setSimPinEnabled(chkPin.checked ? 1 : 0);
      if (inPin && C.setSimPin) { const pv = parseInt(inPin.value, 10); if (!isNaN(pv)) C.setSimPin(pv); }
      // Auto-recover gate: mad2_init defaults to ON; re-apply the checkbox state so a
      // user toggle persists across reboots (boot zeroes g_mad2 → recover_enabled = 1).
      const chkRecover = document.getElementById("chk-recover");
      if (chkRecover && C.setRecover) C.setRecover(chkRecover.checked ? 1 : 0);
      // Eager panic-chain intercept (mad2_init defaults to OFF); re-apply so a user
      // toggle persists across reboots.
      const chkRebootEarly = document.getElementById("chk-reboot-early");
      if (chkRebootEarly && C.setRebootEarly) C.setRebootEarly(chkRebootEarly.checked ? 1 : 0);
    }

    // Persist only when the firmware actually PROGRAMMED NVRAM this run (eepromWrites
    // > 0); a no-change or broken boot then never overwrites a known-good saved state.
    // Flush on tab-hide / unload too. saveEeprom() self-gates on the suppress flag,
    // the write count, and the blank check.
    setInterval(() => {
      if (C.eepromWrites() !== eeLastWrites) saveEeprom();
      if (C.i2cEepromWrites() !== i2cLastWrites) saveI2cEeprom();
    }, 3000);
    document.addEventListener("visibilitychange", () => { if (document.hidden) saveNvram(); });
    window.addEventListener("beforeunload", () => saveNvram());   // no event-as-force
    window.dct3ResetEeprom = function () {
      eepromSuppressSave = true;    // stop the reload's unload-save from clobbering the wipe
      try { localStorage.removeItem(eeKey()); localStorage.removeItem(EE_KEY_LEGACY);
            localStorage.removeItem(i2cKey()); } catch (e) {}
      location.reload();
    };
    window.dct3SaveEeprom = function () { return saveNvram(true); };   // console helper

    function render() {
      const ptr = C.fb();
      const fb = mod.HEAPU8.subarray(ptr, ptr + LCDBANKS * LCDW);  // flat, stride = width
      const mode = C.lcdMode();   // PCD8544 display-control: 0 blank,1 all-on,2 normal,3 inverse
      const leds = C.leds();      // bit0 = LCD backlight, bit1 = keypad backlight
      // LCD palette: a mounted shell may override it (e.g. the 5210's amber screen)
      // so the canvas blends into the photo glass; otherwise the default green DCT3 LCD.
      // Shell screen brightness follows the FRONT illumination lamp (bit1 — the same
      // signal that glows the keypad glyphs): on a candybar the display + keypad share
      // one backlight that lights on activity and idles off. (The separate LCD-backlight
      // bit0 stays on after boot, so it can't drive a visible dim/bright change.) The
      // default green LCD keeps its classic bit0 OFF/OFF_LIT behaviour.
      const frontLit = leds & 2;
      const base = (shellRoot && shellBg)    ? (frontLit && shellBgLit ? shellBgLit : shellBg)
                                             : ((leds & 1) ? OFF_LIT : OFF);
      const onc  = (shellRoot && shellPixel) ? shellPixel : ON;
      // The div "holding" the screen brightens with the canvas so the WHOLE glass
      // changes uniformly (no lit rectangle floating in a dim panel). This runs for
      // EVERY shell — palette shells (5210) and the default-green LCD (3310) alike —
      // so the panel's letterbox border never lags the canvas at a different lit
      // level (the old "square in a square"). `base` already encodes the right
      // on/off colour for whichever path is active.
      if (shellLcdEl) {
        const bg = rgbStr(base);
        if (shellLitState !== bg) { shellLcdEl.style.background = bg; shellLitState = bg; }
      }
      document.body.classList.toggle("kbd-lit", !!(leds & 2));
      // shell mode: glow the keypad backlight (light through the glyph mask) when
      // the keypad LED is on — same signal that lights the grid keys.
      if (shellRoot) shellRoot.style.setProperty("--light-on", (lightPreview || (leds & 2)) ? "1" : "0");
      const d = img.data;
      let o = 0;
      for (let y = 0; y < LCDH; y++) {
        const bank = (y >> 3) * LCDW;
        const bit = y & 7;
        for (let x = 0; x < LCDW; x++) {
          let on = (fb[bank + x] >> bit) & 1;
          if (mode === 0) on = 0; else if (mode === 1) on = 1; else if (mode === 3) on ^= 1;
          const c = on ? onc : base;
          d[o++] = c[0]; d[o++] = c[1]; d[o++] = c[2]; d[o++] = c[3];
        }
      }
      ctx2d.putImageData(img, 0, 0);
    }

    // Boot fast (~45M instructions): run as many chunks as fit a 12 ms wall
    // budget. Once at the OS, PACE BY CYCLES to real-time: advance a fixed number
    // of CPU cycles per animation frame so the emulated clock = 13 MHz regardless
    // of the workload's cycles-per-instruction. Pacing by INSTRUCTION count (the
    // old 150k/frame) assumed ~1.5 cpi, but the real cpi is ~2.7, so the phone ran
    // ~1.8x too fast — which made v6.39's tick-based timers (Snake's move cadence)
    // run away and crash. Cycle pacing fixes every cycle-derived timer at once.
    // We advance (measured wall-seconds since the last frame × 13 MHz × speed) CPU
    // cycles, NOT a fixed amount per frame — so the emulated clock tracks real time
    // on ANY display refresh rate (60/120/144 Hz) and self-corrects after a dropped
    // or throttled frame, instead of assuming exactly 60 fps. dt is clamped so a
    // tab-switch/stall doesn't trigger a huge catch-up sprint (the clock just pauses
    // for that gap). 13 MHz = the DCT3 MAD2 ARM core clock.
    const BOOT_CHUNK = 250000, BOOT_BUDGET_MS = 12;
    const TARGET_HZ = 13e6;            // real DCT3 ARM clock (1x)
    const MAX_FRAME_DT = 0.05;         // s — cap catch-up after a stall
    const FAST_BUDGET_MS = 12;         // per-frame compute cap when fast-forwarding (>1x)
    let speedMul = 1.0;                // from the Speed slider (1.0 = 13 MHz)
    let lastFrameT = null;             // perf timestamp of the previous frame
    // Effective-clock + fps readout, averaged over a ~400 ms window.
    let measT0 = 0, measC0 = 0, measFrames = 0, effMHz = 0, effFps = 0;
    // --- Replay log + spin/crash detection ----------------------------------
    // Every keypress is logged with the INSTRUCTION COUNTER (step) at press time, not
    // wall clock: the emulator is deterministic given the same inputs at the same
    // emulated time, so a step-keyed log replays EXACTLY in tools/nav.mjs (--replay)
    // regardless of pacing — the way to reproduce a spin from a real session. The log
    // resets each boot (step restarts at 0). A spin/crash is flagged when the PC stays
    // pinned to one address while the LCD is frozen (covers both a tight spin loop and a
    // crash that wedges the PC) — surfaced in the console + status line with the replay.
    let keyLog = [];
    function logKey(kind, label) {
      keyLog.push({ t: Math.round(performance.now()), step: Math.round(C.step()),
                    cyc: Math.round(C.cycles()), kind, key: label });
    }
    const spinWin = [];                 // recent PC samples (one per frame)
    let spinFired = false;              // reported the current spin already
    let lastLcdSeen = -1, lastLcdChangeStep = 0;
    let outFrames = 0;                  // consecutive frames where PC is outside legit ranges
    const heapFailWin = [];             // recent heap_fail_count samples (one per frame)
    let heapWarned = false;             // surfaced the heap-exhaustion warning already
    function resetReplayLog() { keyLog = []; spinWin.length = 0; spinFired = false; lastLcdSeen = -1; lastLcdChangeStep = 0; outFrames = 0; heapFailWin.length = 0; heapWarned = false; cancelReplay(); }
    window.dct3KeyLog = () => keyLog.slice();
    window.dct3SpinInfo = null;
    // Post-mortem reader. mad2 populates a UTF-8 buffer at every reset catch with the
    // firmware's own narrative (reason, caller-site label, exception type for reason 5,
    // assertion-ring tail). Returns "" if no catch yet this session. See
    //.
    function readPostmortem() {
      try {
        const len = C.postmortemLen ? (C.postmortemLen() >>> 0) : 0;
        if (!len) return "";
        const ptr = C.postmortemBuf();
        const view = mod.HEAPU8.subarray(ptr, ptr + len);
        return new TextDecoder("utf-8").decode(view);
      } catch (e) { return ""; }
    }
    window.dct3DumpPostmortem = readPostmortem;
    // Deep crash/spin diagnostics: full registers + a backtrace. Memory is BIG-ENDIAN
    // (DCT3 ARM). We read PC/LR, then walk the stack (from SP) for plausible Thumb return
    // addresses (in flash, LSB set), and for each address scan backward for the function
    // PROLOGUE (`push {..,lr}` = Thumb 0xB5xx) to mark each frame's function start.
    // FLASH_HI = top of the model's flash (model-aware: 4 MB images run code up to 0x600000;
    // a fixed 0x400000 would drop every upper-bank frame). Falls back to 0x400000 pre-boot.
    const FLASH_LO = 0x200040, FLASH_HI = (C.flashHi && C.flashHi() >>> 0) || 0x400000;
    const rd16be = (a) => ((C.ram(a) << 8) | C.ram(a + 1)) >>> 0;
    const rd32be = (a) => ((C.ram(a) << 24) | (C.ram(a + 1) << 16) | (C.ram(a + 2) << 8) | C.ram(a + 3)) >>> 0;
    function fnStart(addr) {            // nearest Thumb `push {..,lr}` at/below addr (function prologue)
      addr &= ~1;
      for (let a = addr; a >= FLASH_LO && a > addr - 0x1000; a -= 2)
        if ((rd16be(a) & 0xFF00) === 0xB500) return a;
      return 0;
    }
    const hx = (v) => "0x" + (v >>> 0).toString(16);
    function deepDiag() {
      const r = []; for (let i = 0; i < 16; i++) r.push(C.reg(i) >>> 0);
      const cpsr = C.reg(16) >>> 0, thumb = !!(cpsr & 0x20);
      const pc = r[15], lr = r[14], sp = r[13];
      const frames = [];
      const add = (what, addr) => { const f = fnStart(addr & ~1); frames.push(`${what.padEnd(9)} ${hx(addr)}${f ? `  in fn ${hx(f)}` : ''}`); };
      add("PC", pc); add("LR", lr);
      let n = 0;
      for (let a = sp; a < sp + 0x300 && n < 14; a += 4) {       // stack walk for return addresses
        const w = rd32be(a);
        if ((w & 1) && w >= FLASH_LO && w < FLASH_HI) { add("stk@" + hx(a).slice(2), w); n++; }
      }
      const regstr = r.map((v, i) => `r${i}=${hx(v)}`).join(" ");
      return { pc: hx(pc), sp: hx(sp), lr: hx(lr), cpsr: hx(cpsr), mode: thumb ? "Thumb" : "ARM",
               regs: r.map(v => hx(v)), text: `regs: ${regstr}\ncpsr=${hx(cpsr)} (${thumb ? "Thumb" : "ARM"})\nbacktrace:\n  ` + frames.join("\n  ") };
    }
    // Repro config snapshot — everything needed to reproduce a halt/crash deterministically
    // (no entropy): firmware image + identity, SIM/FAID flags, EEPROM overlay
    // source. Goes at the top of every crash report + the downloaded .txt.
    function cfgSummary() {
      const ck = (id) => { const e = document.getElementById(id); return e ? (e.checked ? "on" : "off") : "?"; };
      const fwEl = document.getElementById("fw-name");
      const fwn = (fwEl && fwEl.textContent && fwEl.textContent.trim()) || lastFwBase ||
                  "(default baked image: My 3310 NR1 v5.79.fls)";
      const ee = pendingEeprom ? "loaded EEPROM file overlay"
               : eeOverlaidThisBoot ? ("localStorage NVRAM overlay (key " + eeKey() + ")")
               : "image-embedded (no overlay)";
      return "config (repro — match these exactly):\n" +
        "  firmware:   " + fwn + "\n" +
        "  fw-id:      " + curFwId + (customFw ? "  [user-loaded]" : "  [baked]") + "\n" +
        "  SIM:        " + ((C.getSim && C.getSim()) ? "inserted" : "absent") + "   PIN: " + ck("chk-pin") + "\n" +
        "  FAID Pass:  " + ck("chk-skip-seclock") + "\n" +
        "  EEPROM:     " + ee + "\n";
    }
    window.dct3Config = cfgSummary;
    window.dct3Diag = function () { const d = deepDiag(); console.log("[DCT3] CPU diag @ " + d.pc + " (" + d.mode + ")\n" + d.text); return d; };

    window.dct3DumpReplay = function () {
      const replay = keyLog.filter(e => e.kind === 'key' || e.kind === 'power').map(e => ({ key: e.key, step: e.step }));
      const json = JSON.stringify(replay);
      console.log(`[DCT3] replay log (${keyLog.length} events, build=${(window.__DCT3_ASSETS||{}).wasmHash||'?'}). Reproduce in the harness:\n` +
                  `  node tools/nav.mjs "<fw.fls>" --replay '${json}'`);
      try { console.table(keyLog); } catch (e) {}
      return json;
    };

    // --- Structured Replay Bundle: in-browser deterministic repro ------------
    // The flat dct3DumpReplay above is compatible with nav.mjs --replay; this
    // additionally bundles the boot config (SIM/PIN/bypass/FAID/spike) + fw-id
    // so Paste&Replay restores the EXACT pre-crash state before running the
    // step-keyed timeline. Goal: paste a crash report in another tab and either
    // confirm the bug is deterministic or watch it not reproduce (= fluky).
    let replayQueue = null, replayIdx = 0, replayStartedAt = 0;
    let replayTargetStep = 0;   // bundle.context.step (legacy, for status display only)
    let replayTargetCyc  = 0;   // bundle.context.cyc — the DETERMINISTIC stop target
    let replayTargetPc   = '';  // bundle.context.pc — for the comparison report
    let replayHalted = false;   // one-shot guard: halt the CPU once we've reached the target
    const replayStatusEl = document.getElementById('replay-status');
    function setReplayStatus(html) {
      if (!replayStatusEl) return;
      if (html === null) { replayStatusEl.style.display = 'none'; replayStatusEl.textContent = ''; return; }
      replayStatusEl.style.display = 'block';
      replayStatusEl.innerHTML = html;
      // Show the crash panel as the host for the status, even if no crash fired,
      // so paste-replay outside a crash still surfaces progress.
      const panel = document.getElementById('crash-panel');
      if (panel && panel.style.display === 'none') panel.style.display = 'block';
    }
    // Cleared on every (re)boot via resetReplayLog (see the call site below).
    function cancelReplay() { replayQueue = null; replayIdx = 0; replayTargetStep = 0; replayTargetCyc = 0; replayTargetPc = ''; replayHalted = false; }
    function replayActive() { return replayQueue !== null; }

    window.dct3CopyReplay = async function () {
      // Cycle-keyed events (cyc) are deterministic across replays; step is kept for
      // backward compat with nav.mjs --replay and human-readable diffs.
      const events = keyLog.filter(e => e.kind === 'key' || e.kind === 'power')
                           .map(e => ({ key: e.key, step: e.step, cyc: e.cyc }));
      const ck = (id) => { const el = document.getElementById(id); return el ? !!el.checked : null; };
      const bundle = {
        v: 1, kind: 'dct3-replay',
        fwId: curFwId, fwName: lastFwBase || '',
        config: {
          sim:    (C.getSim ? !!C.getSim() : ck('chk-sim')),
          pin:    ck('chk-pin'),
          faid:   ck('chk-skip-seclock'),
          // bypass/spike: REMOVED 2026-07-15 (organic boot needs neither; old
          // bundles carrying them are accepted and the keys ignored).
          // Auto-recover is intentionally NOT bundled: it's a viewer-side policy choice
          // (how the recipient wants to handle crashes), not a firmware-state property.
          // Two recipients of the same bundle may pick different recover settings.
        },
        events,
        context: window.dct3SpinInfo
          ? { pc: window.dct3SpinInfo.pc, step: window.dct3SpinInfo.step, cyc: window.dct3SpinInfo.cyc }
          : null,
      };
      const json = JSON.stringify(bundle);
      try {
        await navigator.clipboard.writeText(json);
        console.log(`[replay] copied bundle (${events.length} events, fwId=${bundle.fwId}) to clipboard`);
        setReplayStatus(`Copied <b>${events.length}</b> events + config to clipboard (fwId ${bundle.fwId}).`);
        setTimeout(() => { if (!replayQueue) setReplayStatus(null); }, 4000);
      } catch (e) {
        // clipboard write may be denied (insecure context, no user gesture) — fall back
        window.prompt('Copy this Replay JSON:', json);
      }
      return bundle;
    };

    window.dct3PasteReplay = async function (input) {
      let raw = input;
      if (!raw) {
        try { raw = await navigator.clipboard.readText(); }
        catch (e) { raw = window.prompt('Paste Replay JSON:'); }
      }
      if (!raw) return;
      let bundle;
      try { bundle = JSON.parse(raw); }
      catch (e) {
        // Tolerate JSON embedded in a larger text block (e.g. inside a crash report)
        const m = raw.match(/\{[\s\S]*\}/);
        try { bundle = m ? JSON.parse(m[0]) : null; } catch (_) { bundle = null; }
      }
      if (!bundle || !Array.isArray(bundle.events)) {
        console.error('[replay] invalid Replay JSON (missing events array)');
        setReplayStatus('<b style="color:#c00">Invalid Replay JSON.</b>');
        return;
      }
      // fw-id mismatch is a soft warning — sometimes useful for cross-build repros
      let warn = '';
      if (bundle.fwId && curFwId && bundle.fwId !== curFwId) {
        warn = `<b style="color:#c80">⚠ fw-id mismatch: bundle=${bundle.fwId}, current=${curFwId} — timing may diverge.</b><br>`;
        console.warn(`[replay] fw-id mismatch: bundle=${bundle.fwId} current=${curFwId}`);
      }
      // Apply config (set the visible checkboxes + push to the core where the
      // checkbox handlers do it). FAID takes effect at next boot; SIM/PIN are
      // re-applied by boot() itself from the checkboxes. (bundle.bypass/.spike
      // from pre-2026-07-15 bundles are ignored — the boot is organic now.)
      const cfg = bundle.config || {};
      const setChk = (id, v) => { const el = document.getElementById(id); if (el && typeof v === 'boolean') el.checked = v; };
      setChk('chk-sim', cfg.sim);
      setChk('chk-pin', cfg.pin);
      setChk('chk-skip-seclock', cfg.faid);
      if (typeof cfg.faid    === 'boolean' && C.setSkipSeclock) C.setSkipSeclock(cfg.faid ? 1 : 0);
      // Auto-recover (cfg.recover) intentionally NOT applied — viewer's checkbox wins.
      // Cold reboot for a clean step=0 start. boot() re-applies SIM/PIN from the
      // checkboxes we just updated and calls resetReplayLog (which cancels any
      // in-progress replay), so the queue is set AFTER boot() returns.
      boot();
      // Auto-arm the divergence-finder trace ring — two paste-replays of the same bundle
      // will now produce dumpable traces that dct3DiffTraces() can compare instruction-by-
      // instruction. Cheap (one cmp+branch per step + occasional writes).
      if (C.traceOn) C.traceOn(1);
      // Clear the stale crash report text so the panel just shows the live replay status.
      const ta = document.getElementById('crash-text');
      if (ta) ta.value = '';
      replayQueue = bundle.events.slice();
      replayIdx = 0;
      replayStartedAt = performance.now();
      // Even with 0 events, the bundle carries a target (halt/crash point) we advance to.
      // The DETERMINISTIC target is cyc (cycle count); step is kept for the status display
      // and as a fallback when an older bundle didn't record cyc.
      replayTargetCyc  = (bundle.context && Number.isFinite(bundle.context.cyc))  ? bundle.context.cyc  : 0;
      replayTargetStep = (bundle.context && Number.isFinite(bundle.context.step)) ? bundle.context.step : 0;
      replayTargetPc   = bundle.context && bundle.context.pc ? String(bundle.context.pc) : '';
      replayHalted = false;
      const ctx = replayTargetCyc
        ? ` · advancing to cyc ${(replayTargetCyc/1e6).toFixed(1)}M (original PC ${replayTargetPc || '?'})`
        : (replayTargetStep ? ` · advancing to step ${(replayTargetStep/1e6).toFixed(1)}M (legacy bundle)` : '');
      console.log(`[replay] running ${replayQueue.length} events (fwId=${bundle.fwId})${ctx}`);
      setReplayStatus(warn + `Replaying <b>${replayQueue.length}</b> events… (fwId ${bundle.fwId})${ctx}`);
    };

    // Frame-loop dispatcher. Called from frame() right after C.runCyc so the
    // current step reflects all cycles executed this frame. Multiple events can
    // fire in one frame if they were recorded close together.
    function pumpReplayQueue() {
      if (!replayQueue) return;
      // No separate reset-detection here — checkSpin now treats firmware self-reset
      // as its first trigger, opens the crash panel, sets crashed=true, and the frame
      // loop pauses (which cancels replay via the boot/halt path naturally). Unified.
      // Cycle-keyed dispatch is deterministic; fall back to step for older bundles
      // that didn't record cyc on events.
      const cyc = C.cycles(), step = C.step();
      const evKey = (ev) => Number.isFinite(ev.cyc) ? cyc >= ev.cyc : step >= ev.step;
      while (replayIdx < replayQueue.length && evKey(replayQueue[replayIdx])) {
        const ev = replayQueue[replayIdx++];
        if (ev.key === 'pwr' || ev.key === 'power') { C.power(1); C.power(0); }
        else if (ev.key === 'off')                  { C.power(1); }
        else {
          const id = LABEL_KK[ev.key];
          if (id !== undefined) C.keyLogical(id, 1);   // model-aware; firmware auto-releases via KEY_HOLD_INSNS
          else console.warn(`[replay] unknown key '${ev.key}' — skipped`);
        }
      }
      const elapsed = ((performance.now() - replayStartedAt) / 1000).toFixed(1);
      const eventsDone = replayIdx >= replayQueue.length;
      // Prefer cycle-keyed target for determinism; fall back to step for legacy bundles.
      const useCyc = replayTargetCyc > 0;
      const targetDue = replayTargetCyc > 0 || replayTargetStep > 0;
      const reachedTarget = useCyc ? cyc >= replayTargetCyc
                                   : (replayTargetStep > 0 && step >= replayTargetStep);
      if (!eventsDone) {
        const next = replayQueue[replayIdx];
        const nm = Number.isFinite(next.cyc) ? next.cyc / 1e6 : next.step / 1e6;
        const cm = useCyc ? cyc / 1e6 : step / 1e6;
        setReplayStatus(`Replaying <b>${replayIdx}/${replayQueue.length}</b> · next: <b>${next.key}</b> @ ${nm.toFixed(1)}M · cur ${cm.toFixed(1)}M · ${elapsed}s`);
        return;
      }
      if (targetDue && !reachedTarget) {
        const tgt = useCyc ? replayTargetCyc : replayTargetStep;
        const cur = useCyc ? cyc : step;
        const togo = (tgt - cur) / 1e6;
        setReplayStatus(`Events done (${replayQueue.length}); advancing to ${useCyc ? 'cyc' : 'step'} ${(tgt/1e6).toFixed(1)}M · cur ${(cur/1e6).toFixed(1)}M · ${togo.toFixed(1)}M to go · ${elapsed}s`);
        return;
      }
      if (reachedTarget && !replayHalted) {
        replayHalted = true;
        const haltBtn = document.getElementById('btn-halt-cpu');
        if (haltBtn) haltBtn.click();
        const curPc = (C.pc() >>> 0).toString(16);
        const matched = replayTargetPc && curPc === replayTargetPc.replace(/^0x/, '').toLowerCase();
        const verdict = matched ? '<b style="color:#080">PC MATCH ✓ (deterministic)</b>'
                                : `<b style="color:#c80">PC differs: replay 0x${curPc} vs original ${replayTargetPc}</b>`;
        // Step + cyc diff diagnostics — if step matches but PC differs, the issue is
        // pipeline/PC computation; if step differs, there's real instruction-stream
        // divergence between the original run and this replay.
        const dStep = replayTargetStep ? (step - replayTargetStep) : 0;
        const dCyc  = useCyc ? (cyc - replayTargetCyc) : 0;
        const diff = `<br><small>cyc Δ=${dCyc>=0?'+':''}${dCyc} · step ${step}${dStep ? ` (Δ=${dStep>=0?'+':''}${dStep})` : ''}</small>`;
        const where = useCyc ? `cyc ${(cyc/1e6).toFixed(2)}M (target ${(replayTargetCyc/1e6).toFixed(2)}M)`
                             : `step ${(step/1e6).toFixed(2)}M`;
        setReplayStatus(`Reached ${where} after ${elapsed}s. ${verdict}${diff}`);
        return;
      }
      const ctxMsg = (window.dct3SpinInfo && replayQueue.length > 0 && window.dct3SpinInfo.reason !== 'manual-halt')
        ? ` · CRASH FIRED @ ${window.dct3SpinInfo.pc} step ${(window.dct3SpinInfo.step/1e6).toFixed(1)}M (deterministic ✓)`
        : ' · watching for crash…';
      setReplayStatus(`Replay finished: <b>${replayQueue.length}</b> events in ${elapsed}s.${ctxMsg}`);
    }
    window.dct3CancelReplay = cancelReplay;

    // --- Divergence-finder: per-cycle trace ring + diff ----------------------
    // dct3DumpTrace() returns the captured trace as an array; dct3DiffTraces(a, b)
    // finds the FIRST sample where two traces disagree. Use to pinpoint where two
    // replays of the same bundle take different code paths.
    //
    // Workflow:
    //   1. Paste & Replay (which auto-arms tracing) → run completes / halts.
    //   2. T1 = dct3DumpTrace();  (copy via JSON.stringify if needed)
    //   3. Reload + Paste & Replay again → T2 = dct3DumpTrace().
    //   4. dct3DiffTraces(T1, T2) — prints + returns the first divergence point.
    window.dct3DumpTrace = function () {
      const n = (C.traceCount ? C.traceCount() : 0) | 0;
      const out = new Array(n);
      for (let i = 0; i < n; i++) {
        out[i] = {
          cyc:  C.traceCyc(i),
          step: C.traceStep(i),
          pc:   C.tracePc(i) >>> 0,
          cpsr: C.traceCpsr(i) >>> 0,
          fiq:  C.traceFiq(i) | 0,
          irq:  C.traceIrq(i) | 0,
        };
      }
      console.log(`[trace] dumped ${n} samples (every 100k cycles)`);
      return out;
    };
    window.dct3DiffTraces = function (a, b) {
      if (!Array.isArray(a) || !Array.isArray(b)) { console.error('[trace] need two arrays'); return null; }
      const n = Math.min(a.length, b.length);
      for (let i = 0; i < n; i++) {
        const x = a[i], y = b[i];
        if (x.cyc !== y.cyc || x.pc !== y.pc || x.step !== y.step ||
            x.cpsr !== y.cpsr || x.fiq !== y.fiq || x.irq !== y.irq) {
          console.log(`[trace] FIRST DIVERGENCE at sample #${i}:`);
          console.log(`  A: cyc=${x.cyc}  step=${x.step}  pc=0x${x.pc.toString(16)}  cpsr=0x${x.cpsr.toString(16)}  fiq=${x.fiq}  irq=${x.irq}`);
          console.log(`  B: cyc=${y.cyc}  step=${y.step}  pc=0x${y.pc.toString(16)}  cpsr=0x${y.cpsr.toString(16)}  fiq=${y.fiq}  irq=${y.irq}`);
          if (i > 0) {
            const p = a[i-1];
            console.log(`  last match (#${i-1}): cyc=${p.cyc}  step=${p.step}  pc=0x${p.pc.toString(16)}`);
          }
          return { firstDiffIndex: i, sampleA: x, sampleB: y, lastMatch: i > 0 ? a[i-1] : null };
        }
      }
      console.log(`[trace] traces match across ${n} samples (A=${a.length}, B=${b.length})`);
      return null;
    };

    // Sample the PC once per frame; flag a spin/crash if it pins to one address while the
    // LCD has been frozen for a while (idle screens keep redrawing, so they don't trip it).
    function checkSpin() {
      const pc = C.pc() >>> 0, lcd = C.lcdWrites(), step = C.step();
      if (lcd !== lastLcdSeen) { lastLcdSeen = lcd; lastLcdChangeStep = step; spinFired = false; }  // progress → re-arm
      // Heap-exhaustion warning (non-halting). A sustained stream of allocation FAILURES (the
      // allocator returned NULL and parked on the memory semaphore) means the heap is starved
      // — usually the precursor to a heap-smash wild PC, which the triggers below will catch
      // and halt on. Normal operation has ZERO alloc failures, so any sustained burst is
      // abnormal. We surface it loudly ONCE but keep running so the eventual crash is captured.
      if (C.heapFailCount && !heapWarned) {
        const hf = C.heapFailCount();
        heapFailWin.push(hf); if (heapFailWin.length > 240) heapFailWin.shift();
        if (heapFailWin.length >= 240 && hf - heapFailWin[0] >= 24) {
          heapWarned = true;
          const hlr = (C.heapFailLr ? C.heapFailLr() : 0) >>> 0;
          console.warn(`[DCT3] ⚠ HEAP EXHAUSTION — ${hf} alloc failures (${hf - heapFailWin[0]} in the last ~240 frames), last caller LR=0x${hlr.toString(16)}. The heap is starved; a wild-PC crash often follows.`);
          if (status && !spinFired) status.textContent = `⚠ HEAP EXHAUSTION (${hf} alloc fails) — see console`;
        }
      }
      // Legit PC ranges: ARM vectors [0,0x20), then RAM/code-copy + flash [0x100000,flashHi).
      // flashHi is MODEL-AWARE (4 MB images legitimately execute up to 0x600000) — a fixed
      // 0x400000 ceiling false-trips every call into the upper bank as a wild-PC. Mirrors the
      // C detector (fault.c / telemetry.c). Falls back to 0x400000 pre-boot.
      const flashHi = (C.flashHi && C.flashHi() >>> 0) || 0x400000;
      const pcInLegit = pc < 0x20 || (pc >= 0x100000 && pc < flashHi);
      // Severity split. A HIGH wild PC (≥ flashHi) is NEVER a legit transient: no code is
      // mapped there and exceptions only ever vector LOW (0x00-0x20), so the firmware loaded
      // a corrupt value into PC (heap-smash / bad function pointer). The core masks the fetch
      // to 16 MiB so the CPU limps through zero-fill, but it is FATAL — on real HW the
      // watchdog reboots and, the root cause persisting, it re-crashes. So fire on sight.
      // A LOW out-of-bounds PC (0x20..0xFFFFF) CAN be a one-instruction exception-entry
      // artifact (FIQ vector+4 = 0x20), so it must persist a little before we trust it.
      const pcWildHigh = pc >= flashHi;
      outFrames = pcInLegit ? 0 : outFrames + 1;
      spinWin.push(pc); if (spinWin.length > 150) spinWin.shift();
      if (spinFired) return;
      // Triggers in priority order — all flow through the same crash-panel path. Each one
      // headlines its OWN fault PC: a wild-PC crash must NOT be reported as the most-common
      // windowed PC (that mislabels a 9%-of-window housekeeping loop as "the spin"). A frozen
      // LCD is never a trigger by itself — idle/standby screens legitimately quiet the LCD for
      // hundreds of millions of instructions, so only a pinned PC counts as a real spin.
      let trigger = null, kind = null, faultPc = pc;
      if (C.resetReq && C.resetReq()) {
        kind = 'RESET'; faultPc = pc;
        trigger = `firmware self-reset (reason ${C.resetLastReason ? C.resetLastReason() : '?'})`;
      } else if (pcWildHigh && outFrames >= 2) {
        kind = 'WILD-PC'; faultPc = pc;        // out of bounds "big time" — fire on sight
        trigger = `wild PC 0x${pc.toString(16)} — branched outside all mapped memory`;
      } else if (!pcInLegit && outFrames >= 12) {
        kind = 'WILD-PC'; faultPc = pc;        // sustained low/MMIO-region PC — march-to-null
        trigger = `PC out of bounds (low) — sustained ${outFrames} frames at 0x${pc.toString(16)}`;
      } else if (spinWin.length >= 150) {
        const counts = {}; let topPc = 0, topN = 0;
        for (const p of spinWin) { const n = (counts[p] = (counts[p] || 0) + 1); if (n > topN) { topN = n; topPc = p; } }
        const pinnedShare = topN / spinWin.length, lcdQuiet = step - lastLcdChangeStep;
        if (pinnedShare > 0.9 && lcdQuiet > 8e6) {   // genuinely stuck on one PC, not idle
          kind = 'SPIN'; faultPc = topPc;
          trigger = `tight-loop spin — PC pinned ${Math.round(pinnedShare*100)}% at 0x${topPc.toString(16)}, LCD frozen ${(lcdQuiet/1e6).toFixed(1)}M insns`;
        }
      }
      if (!trigger) return;
      spinFired = true;
      console.error(`[DCT3] ⚠ ${kind} trigger: ${trigger}`);
      // Self-heal: a boot-time fault (the LCD froze early, before the ~46M interactive
      // hand-off) while this image's localStorage NVRAM was overlaid means that saved NVRAM
      // is corrupt. Clear it once and reboot from the image's own clean EEPROM, rather than
      // wedging on the crash panel. Post-interaction crashes freeze late, so they keep the
      // normal crash panel.
      if (eeOverlaidThisBoot && !eeRecovered && lastLcdChangeStep < 40e6) {
        eeRecovered = true;
        eepromSuppressSave = true;                       // don't let an unload-save re-clobber
        try { localStorage.removeItem(eeKey()); localStorage.removeItem(EE_KEY_LEGACY); } catch (e) {}
        console.warn(`[DCT3] persisted NVRAM caused a boot fault at 0x${faultPc.toString(16)} — ` +
                     `cleared it and rebooting from the image's own EEPROM (the saved settings ` +
                     `for this image were corrupt and have been reset).`);
        eeSkipOverlay = true;
        try { boot(); } catch (e) {}                     // boot() resets crashed/spin state
        eepromSuppressSave = false;
        if (status) status.textContent = "Recovered: cleared corrupt saved settings, rebooting…";
        return;
      }
      const diag = deepDiag();
      // mad2's post-mortem block (firmware's own narrative — assertion ring, caller label,
      // exception mode). Populated by mad2_render_postmortem() at the catch site for resets;
      // empty for a pure wild-PC/spin — that's fine, the backtrace carries the call chain.
      const postmortem = readPostmortem();
      const label = kind === 'SPIN' ? 'SPIN' : kind === 'RESET' ? 'RESET' : 'CRASH';
      const info = { pc: '0x' + faultPc.toString(16), kind,
                     step: Math.round(step), cyc: Math.round(C.cycles()), keyLog: keyLog.slice(), diag,
                     postmortem };
      window.dct3SpinInfo = info;
      const pmTail = postmortem ? `\n${postmortem}` : "";
      console.error(`[DCT3] ⚠ ${label} at ${info.pc} — ${trigger}. step=${info.step} build=${(window.__DCT3_ASSETS||{}).wasmHash||'?'}\n${diag.text}${pmTail}`);
      const replayJson = window.dct3DumpReplay();
      // HALT the CPU and surface everything in the UI (copyable) + offer a memory dump.
      crashed = true;
      const panel = document.getElementById("crash-panel"), ta = document.getElementById("crash-text");
      if (ta) ta.value =
        `${label} at ${info.pc}  — ${trigger}  step=${info.step}\n\n` +
        `${cfgSummary()}\n${diag.text}\n${pmTail}\nreplay (nav.mjs --replay):\n${replayJson}\n`;
      if (panel) panel.style.display = "block";
      if (status) status.textContent = `⚠ ${label} at ${info.pc} (step ${(info.step / 1e6).toFixed(1)}M) — dct3DumpReplay() in console`;
    }

    function frame(now) {
      // Power-off: park the CPU once the phone has switched off — either the firmware
      // signalled the regulator cut (C.poweredOff(), e.g. a menu "Switch off") or a
      // deliberate power-button hold has had time to run the shutdown animation + flush
      // NVRAM. Snapshot the flushed NVRAM (per-image) so it persists. A power tap reboots.
      if (!halted && (C.poweredOff() || (poweringOff && performance.now() - poweringOff > 2200))) {
        saveNvram(true);
        C.power(0);
        halted = true; poweringOff = 0;
        console.log("[DCT3] powered off — CPU parked, NVRAM flushed + saved");
      }
      if (halted) {
        render();                          // show the (cleared) off screen
        status.textContent = "Powered off — tap ⏻ Power to switch on";
        requestAnimationFrame(frame);
        return;                            // CPU parked: do not run
      }
      if (crashed) {                       // crash/spin halt — freeze, keep the crash panel
        render();
        requestAnimationFrame(frame);
        return;
      }
      const booting = C.step() < 46e6;
      // Replay mode: identical fixed-cycle batches per frame, no host-time pacing and
      // no boot fast-forward differentiation. Two replays of the same bundle now run
      // exactly the same cycle sequence → same instruction stream → same PC at halt.
      // Trades realtime feel for determinism (the whole point of replay).
      const REPLAY_CYC_PER_FRAME = 10_000_000;   // ~13× realtime, fast enough to verify
      if (replayActive()) {
        const rem = replayTargetCyc > 0 ? Math.max(0, replayTargetCyc - C.cycles()) : 0;
        const batch = (replayTargetCyc > 0 && rem < REPLAY_CYC_PER_FRAME)
          ? rem                              // last mile: aim exactly at target cycle
          : REPLAY_CYC_PER_FRAME;            // free running: fixed batch every frame
        if (batch > 0) C.runCyc(batch);
        lastFrameT = now; measT0 = now; measC0 = C.cycles(); measFrames = 0;
      } else if (booting) {
        const t0 = performance.now();
        do { C.run(BOOT_CHUNK); } while (performance.now() - t0 < BOOT_BUDGET_MS);
        lastFrameT = now; measT0 = now; measC0 = C.cycles(); measFrames = 0;
      } else {
        if (lastFrameT == null) { lastFrameT = now; measT0 = now; measC0 = C.cycles(); }
        let dt = (now - lastFrameT) / 1000;
        lastFrameT = now;
        if (dt > MAX_FRAME_DT) dt = MAX_FRAME_DT;
        if (dt > 0) {
          const want = Math.round(TARGET_HZ * speedMul * dt);
          if (speedMul <= 1.0) {
            C.runCyc(want);            // real-time: run exactly the wall-clock amount (clock-accurate)
          } else {
            // Fast-forward (>1x): run toward `want`, but stop after FAST_BUDGET_MS of
            // compute so the frame still presents on time. The phone runs as fast as the
            // CPU allows and the frame rate stays smooth instead of collapsing; any deficit
            // is NOT carried over (next frame recomputes from real elapsed time → no spiral).
            const t0 = performance.now();
            const CHUNK = 200000;
            let done = 0;
            do {
              C.runCyc(Math.min(CHUNK, want - done));
              done += CHUNK;
            } while (done < want && performance.now() - t0 < FAST_BUDGET_MS);
          }
        }
        measFrames++;
        const win = (now - measT0) / 1000;
        if (win >= 0.4) {              // refresh the readout ~2.5x/s
          effMHz = (C.cycles() - measC0) / win / 1e6;
          effFps = measFrames / win;
          measT0 = now; measC0 = C.cycles(); measFrames = 0;
        }
      }
      // Always pump the replay (also during a "booting" frame — replay's own boot is
      // fast-forwarded above, and an early event could fire before step crosses 46M).
      if (replayActive() || !booting) pumpReplayQueue();
      render();
      renderDiag();
      renderTimers();
      updatePeripherals();
      if (!booting) checkSpin();          // flag a stuck PC / frozen LCD (spin or crash)
      if (!booting) pumpType();           // paced character injection (Type text box)
      const step = C.step();
      const h2 = (v) => v.toString(16).toUpperCase().padStart(2, "0");
      // CCONT shadow RTC region (0x111A58..0x111A5F) — the firmware's RAM copy of the
      // RTC regs; the displayed clock + "set time" should land here. Shows whether a
      // set commits a non-zero value, separate from our model's RTC.
      let shadow = "";
      for (let i = 0x111a58; i < 0x111a60; i++) shadow += h2(C.ram(i) & 0xff);
      if (spinFired) { requestAnimationFrame(frame); return; }   // keep the SPIN/CRASH status visible
      status.textContent = booting
        ? "Booting… " + (step / 1e6).toFixed(1) + "M instructions"
        : effMHz.toFixed(1) + " MHz · " + effFps.toFixed(0) + " fps · " +
          (step / 1e6).toFixed(0) + "M insns · CCmask " + h2(C.ccontMask()) +
          " min#" + C.rtcMinEdges() + " · RTCwr " + C.rtcWrites() +
          " raw " + (C.rtcRaw() >>> 0).toString(16).padStart(8, "0") +
          " · shadow@A58 " + shadow;
      requestAnimationFrame(frame);
    }

    // --- input -------------------------------------------------------------
    // Set `DCT3_KEYLOG = 1` in the console to log every key edge sent to the
    // core — useful to see if the OS is spamming presses (autorepeat) per hold.
    let lastKeyRC = { row: 1, col: 1 };   // remember last key (for the KEYRELEASE test button)
    // Send a LOGICAL key (label). The wasm resolves it to THIS model's (row,col) via
    // the profile matrix, so the page is model-agnostic. Replay logs the label, which
    // replays correctly per-model (the bundle carries the fw-id → model).
    // Faithful edge: set the matrix bit on DOWN, clear on UP — the firmware's own
    // scan period / repeat grace / long-press thresholds decide tap-vs-hold, so
    // press-and-HOLD works as on hardware (menu auto-repeat, hold-1 voicemail,
    // long-Menu voice dial, …), exactly like the native GUI's key_apply. A small
    // min-hold floor guarantees a very fast click still spans a keypad scan. The
    // up-edge is re-checked so a re-press before the deferred release isn't cleared.
    // (Old wasm without the raw export falls back to the queued-tap path.)
    const KEY_MIN_HOLD_MS = 20;          // ~> one firmware keypad-scan period (~17 ms @13 MHz)
    const keyHeldAt = new Map();         // label -> down timestamp (ms); absent = released
    function pressKey(label, down) {
      const id = LABEL_KK[label];
      if (id === undefined) return;
      if (window.DCT3_KEYLOG) console.log(`[key] ${down ? "DOWN" : "up  "} ${label} @${C.step().toFixed(0)}`);
      const raw = C.keyLogicalRaw;
      if (!raw) { if (down) logKey('key', label); C.keyLogical(id, down ? 1 : 0); return; }
      if (down) {
        if (keyHeldAt.has(label)) return;             // idempotent (autorepeat / re-entry)
        logKey('key', label);                         // replay log (model-agnostic label)
        keyHeldAt.set(label, performance.now());
        raw(id, 1);
      } else {
        const t0 = keyHeldAt.get(label);
        if (t0 === undefined) return;
        keyHeldAt.delete(label);
        const elapsed = performance.now() - t0;
        const release = () => { if (!keyHeldAt.has(label)) raw(id, 0); };  // skip if re-pressed
        if (elapsed >= KEY_MIN_HOLD_MS) release();
        else setTimeout(release, Math.ceil(KEY_MIN_HOLD_MS - elapsed));
      }
    }

    // On-screen keypad layout — mirrors the native GUI (tools/gui_sdl.c family_layout):
    // a unified 3-column grid below the LCD ('' = empty cell), plus (Family A only) the
    // volume keys stacked to the LEFT of the LCD. Matrix lines come from the profile.
    //   B(3310): [MENU UP ·] [C DN ·] + digits
    //   A/C:     [SL UP SR] [SND DN END] + digits   (A also: V+/V- on the side)
    const PAD = {
      0: [ ["soft1", "up", ""], ["soft2", "down", ""],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
      1: [ ["soft1", "up", "soft2"], ["send", "down", "end"],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
      2: [ ["soft1", "up", "soft2"], ["send", "down", "end"],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
      // 7110/NSE-5: Navi Roller replaces up/down — roll up/select/down in the nav column;
      // send/end flank it; no volume keys (the 7110 has none).
      3: [ ["soft1", "wheelup", "soft2"], ["send", "wheelpress", "end"],
           ["", "wheeldown", ""],
           ["1", "2", "3"], ["4", "5", "6"], ["7", "8", "9"], ["*", "0", "#"] ],
    };
    const SIDE_VOL = { 1: ["volup", "voldown"] };   // Family A only (left of the LCD)
    // Terse glyphs matching the GUI's KEYMETA; Family B relabels the soft keys (glyph_for).
    const GLYPH = { "0":"0","1":"1","2":"2","3":"3","4":"4","5":"5","6":"6","7":"7","8":"8","9":"9",
      "*":"*", "#":"#", up:"UP", down:"DN", soft1:"SL", soft2:"SR",
      send:"SND", end:"END", volup:"V+", voldown:"V−",
      wheelup:"▲", wheelpress:"●", wheeldown:"▼" };   // Navi Roller
    const KEYCLASS = { send:"send", end:"end", volup:"vol", voldown:"vol", up:"nav", down:"nav",
      wheelup:"nav", wheeldown:"nav", wheelpress:"nav" };
    function glyphFor(family, label) {
      if (family === 0) { if (label === "soft1") return "MENU"; if (label === "soft2") return "C"; }
      return GLYPH[label] || label;
    }

    function bindKey(b, label) {
      let pressed = false;
      const down = (e) => { e.preventDefault(); if (pressed) return; pressed = true; b.classList.add("down"); pressKey(label, true); };
      const up   = (e) => { if (e) e.preventDefault(); if (!pressed) return; pressed = false; b.classList.remove("down"); pressKey(label, false); };
      b.addEventListener("mousedown", down); b.addEventListener("mouseup", up); b.addEventListener("mouseleave", up);
      b.addEventListener("touchstart", down, { passive: false }); b.addEventListener("touchend", up, { passive: false });
    }
    function makeKey(family, label, extraClass) {
      const b = document.createElement("button");
      b.className = "key" + (KEYCLASS[label] ? " " + KEYCLASS[label] : "") +
                    ((label === "soft1" || label === "soft2") ? " softkey" : "") +
                    (extraClass ? " " + extraClass : "");
      b.dataset.key = label;
      if (label === "soft1" || label === "soft2") {
        // soft keys render as a plain line (---) via CSS — no text
      } else if (label === "send") {
        b.textContent = "📞";                       // call (green)
      } else if (label === "end") {
        const s = document.createElement("span");   // hang-up = receiver rotated down (red)
        s.className = "hangup"; s.textContent = "📞";
        b.appendChild(s);
      } else {
        b.textContent = glyphFor(family, label);
      }
      b.setAttribute("aria-label", label);
      bindKey(b, label);
      return b;
    }

    // (Re)build the keypad DOM for a visual family. Called by applyModel() at boot and
    // when the manual Keymap override changes.
    function buildKeypad(family) {
      const pad = document.getElementById("pad");
      const side = document.getElementById("side-vol");
      if (!pad) return;
      pad.innerHTML = ""; if (side) side.innerHTML = "";
      (PAD[family] || PAD[0]).forEach((rowarr) => {
        rowarr.forEach((label) => {
          const cell = document.createElement("div");
          cell.className = "padcell" + (label ? "" : " empty");
          if (label) cell.appendChild(makeKey(family, label));
          pad.appendChild(cell);
        });
      });
      const vol = SIDE_VOL[family];
      if (side && vol) vol.forEach((label) => side.appendChild(makeKey(family, label)));
    }

    // Manual keymap override: "auto" follows the detected model's family; otherwise a
    // forced KeypadFamily id (for testing a mismatched layout). Set once below.
    let keymapOverride = "auto";
    function currentFamily() {
      return keymapOverride === "auto" ? (C.kpFamily() | 0) : (parseInt(keymapOverride, 10) | 0);
    }
    // Re-read the detected model's LCD geometry + keypad family and reconfigure the
    // page: resize the canvas + framebuffer, relabel the device, rebuild the keypad.
    // Called after every (re)boot and whenever the firmware changes.
    function applyModel() {
      LCDW = C.lcdW() || 84;
      LCDH = C.lcdH() || 48;
      LCDBANKS = C.lcdBanks() || Math.ceil(LCDH / 8);
      if (canvas.width !== LCDW || canvas.height !== LCDH) {
        canvas.width = LCDW; canvas.height = LCDH;
        img = ctx2d.createImageData(LCDW, LCDH);   // resize the backing pixel buffer
      }
      const lbl = document.getElementById("detected-model");
      if (lbl) lbl.textContent = (C.model() || "?") + " · " + LCDW + "×" + LCDH;
      if (window.__typeSpikePc) window.__typeSpikePc(C.model() || "");   // char-post PC per model
      layoutForModel();             // device shell (auto by model) or classic grid
    }
    {
      const sel = document.getElementById("keymap-override");
      if (sel) sel.addEventListener("change", () => { keymapOverride = sel.value; buildKeypad(currentFamily()); });
    }

    // ===== device shells (realistic photo body, auto-selected by model) =======
    // A model with an entry in shells/shells.js renders as a real cut-out photo of
    // the phone: the live #lcd canvas is seated on its screen and transparent
    // hit-zones over each key drive the SAME logical-key pipeline as the grid /
    // keyboard. Models without a shell fall back to the classic grid keypad. Shell
    // selection is automatic (by C.model()), exactly like the keypad family + LCD
    // geometry; the View override forces the grid for layout testing.
    const SHELLS = window.DCT3_SHELLS || {};
    const LCD_OFF_HEX = "#aec48e";          // unlit LCD bg (matches OFF) for panel letterbox
    let viewOverride = "auto";              // "auto" = shell when available, else grid
    let showHitAreas = false;
    let shellRoot = null;                   // mounted .shell-root (null ⇒ grid mode)
    let shellLcdEl = null;                   // the .shell-lcd panel (the div "holding" the screen)
    // Per-shell LCD palette (null ⇒ default green DCT3 LCD). bg = backlight-off screen
    // colour, bgLit = backlight-on (brighter) colour, pixel = lit-pixel colour. The
    // panel AND the canvas use the SAME colour so the whole glass brightens uniformly.
    let shellBg = null, shellBgLit = null, shellPixel = null;
    let shellLitState = -1;                  // last applied panel bg colour string (-1 = none yet)
    let shellDef = null;                     // active shell def (export keeps img/mask/palette)
    let shellScale = 1;                      // shell-root CSS scale (screen px ⇄ design px)
    let editTarget = "off";                  // which layer is editable: off | zones | screen | mask
    let editMode = false;                    // derived: editTarget !== "off"
    let holdLight = false;                   // persistent "hold keypad backlight on" toggle
    let selectedEl = null;                   // currently selected editable (a .shell-zone, .shell-glow, or the LCD)
    // Live backlight ("bglight") overlay settings — reset from the def in buildShell,
    // tuned live by the backlight editor, read back by exportShell. lightPreview forces
    // the glow ON (--light-on=1) while editing so it's visible without the firmware
    // driving the keypad LEDs. Defaults match the hardcoded values in style.css.
    const LIGHT_BLUR_DEFAULT = 2.4, LIGHT_BLOOM_DEFAULT = 0.85, LIGHT_COLOR_DEFAULT = "#ffc24d";
    let lightColor = LIGHT_COLOR_DEFAULT, lightBlur = LIGHT_BLUR_DEFAULT, lightBloom = LIGHT_BLOOM_DEFAULT;
    let lightPreview = false;
    let glowSeq = 0;                         // monotonic id for added glow regions
    // Glyph-mask registration nudge: design-px offset (maskX/Y) + independent horizontal
    // and vertical stretch (maskSX/SY) — drives mask-position/mask-size of the .shell-light
    // glow so the lit glyphs line up with the photo. Reset from the def in buildShell.
    let maskX = 0, maskY = 0, maskSX = 1, maskSY = 1;
    let maskFrameEl = null;                  // editor handle frame for the glyph mask (edit mode only)
    // Vector (bézier) key shapes. Each zone may carry a closed cubic path: an array of
    // nodes {x,y, ix,iy, ox,oy} = anchor + in/out control points (design px). When present
    // the zone's hit-area is a clip-path:path(); the SVG overlay edits the nodes/handles.
    let vecPaths = {};                       // key -> node array (live, edited in place)
    let zoneEls = {};                        // key -> .shell-zone button
    let zoneRect = {};                       // key -> {left,top,w,h} (rect fallback for seeding)
    let vectorSvg = null;                    // SVG handle overlay (one per shell)
    let vecZoneKey = null;                   // zone currently shape-edited
    let vecSelNode = null;                   // selected node index (for Delete)
    const hexRgba = (h) => {                  // "#f8e0a0" → [248,224,160,255]
      h = h.replace("#", ""); if (h.length === 3) h = h.split("").map((c) => c + c).join("");
      const n = parseInt(h, 16); return [(n >> 16) & 255, (n >> 8) & 255, n & 255, 255];
    };
    const rgbStr = (c) => "rgb(" + c[0] + "," + c[1] + "," + c[2] + ")";
    const lcdHome = canvas.parentElement;   // .lcd-row — the canvas's grid-mode home
    const shellHost = document.getElementById("shell");
    const SHELL_MAX_W = 320;                // fit the body box into the phone column

    function pickShell(model) {
      if (!model || !shellHost) return null;
      if (SHELLS[model]) return SHELLS[model];                              // exact
      const alias = (window.DCT3_SHELL_ALIASES || {})[model];               // shared body (3330→3310)
      if (alias && SHELLS[alias]) return SHELLS[alias];
      for (const k in SHELLS) if (model.indexOf(k) >= 0) return SHELLS[k];  // substring
      return null;
    }

    function buildShell(def) {
      shellHost.innerHTML = "";                       // clean rebuild on model switch
      const s = Math.min(1, SHELL_MAX_W / def.w);
      shellDef = def; shellScale = s; selectedEl = null; maskFrameEl = null;
      const root = document.createElement("div");
      root.className = "shell-root";
      root.style.width = def.w + "px"; root.style.height = def.h + "px";
      root.style.transform = "scale(" + s + ")";
      shellHost.style.width = Math.round(def.w * s) + "px";   // reserve scaled footprint
      shellHost.style.height = Math.round(def.h * s) + "px";
      // Backlight overlay: colour + blur + intensity drive both the masked-glyph glow
      // and any glow regions; reset the live editor state from the def on every build.
      lightColor = def.light || LIGHT_COLOR_DEFAULT;
      lightBlur  = def.lightBlur  != null ? def.lightBlur  : LIGHT_BLUR_DEFAULT;
      lightBloom = def.lightBloom != null ? def.lightBloom : LIGHT_BLOOM_DEFAULT;
      maskX = def.maskX || 0; maskY = def.maskY || 0;
      maskSX = def.maskScaleX != null ? def.maskScaleX : 1;
      maskSY = def.maskScaleY != null ? def.maskScaleY : 1;
      glowSeq = 0;
      applyLightVars(root);
      shellBg    = def.lcdColor   ? hexRgba(def.lcdColor)   : null;  // per-model LCD palette
      shellPixel = def.pixelColor ? hexRgba(def.pixelColor) : null;
      // lit screen colour: explicit lcdColorLit, else brighten lcdColor ~40% toward white
      shellBgLit = def.lcdColorLit ? hexRgba(def.lcdColorLit)
                 : (shellBg ? shellBg.map((v, i) => i < 3 ? Math.round(v + (255 - v) * 0.4) : v) : null);
      shellLitState = -1;

      const ASSET = (p) => (window.DCT3_SHELL_PREFIX || "") + p;  // "" at web root; "../" when nested
      const img = document.createElement("img");
      img.className = "shell-photo"; img.src = ASSET(def.img); img.draggable = false; img.alt = "";
      root.appendChild(img);

      if (def.mask) {                                 // amber backlight through the glyphs
        ["bloom", "sharp"].forEach((cls) => {
          const d = document.createElement("div");
          d.className = "shell-light " + cls;
          d.style.webkitMaskImage = "url(" + ASSET(def.mask) + ")";
          d.style.maskImage = "url(" + ASSET(def.mask) + ")";
          root.appendChild(d);
        });
      }

      // Glow regions: free-form soft pools of the backlight colour, independent of the
      // glyph mask (a model with no mask, or extra-lit areas the mask doesn't cover).
      (def.glow || []).forEach((g) => buildGlow(root, g));

      const lcd = document.createElement("div");      // green plate + live canvas
      lcd.className = "shell-lcd";
      const L = def.lcd;
      lcd.style.left = L.left + "px"; lcd.style.top = L.top + "px";
      lcd.style.width = L.w + "px";   lcd.style.height = L.h + "px";
      lcd.style.background = L.bg || def.lcdColor || LCD_OFF_HEX;  // initial; render() drives it live
      if (L.radius != null) lcd.style.borderRadius = L.radius + "px";
      lcd.dataset.key = "lcd";                        // editor: the LCD rect is editable too
      root.appendChild(lcd);
      shellLcdEl = lcd;
      if (L.canvasW) canvas.style.width = L.canvasW + "px";
      if (L.canvasH) canvas.style.height = L.canvasH + "px";
      lcd.appendChild(canvas);                        // reparent the shared #lcd canvas

      vecPaths = {}; zoneEls = {}; zoneRect = {}; vecZoneKey = null; vecSelNode = null;
      (def.zones || []).forEach((z) => {              // clickable hit-zones
        const b = document.createElement("button");
        b.className = "shell-zone" + (z.key === "pwr" ? " pwr" : "");
        b.dataset.key = z.key;
        b.style.left = z.left + "px"; b.style.top = z.top + "px";
        b.style.width = z.w + "px";   b.style.height = z.h + "px";
        b.style.borderRadius = z.r || "50%";
        b.setAttribute("aria-label", z.key);
        bindZone(b, z.key);
        root.appendChild(b);
        zoneEls[z.key] = b;
        zoneRect[z.key] = { left: z.left, top: z.top, w: z.w, h: z.h };
        if (z.path) {                                 // already a bézier shape → clip to it
          vecPaths[z.key] = z.path.map((p) => ({ x: p[0], y: p[1], ix: p[2], iy: p[3], ox: p[4], oy: p[5] }));
          applyZoneClip(z.key);
        }
      });
      vectorSvg = document.createElementNS(SVGNS, "svg");
      vectorSvg.setAttribute("class", "shell-vector");
      vectorSvg.setAttribute("viewBox", "0 0 " + def.w + " " + def.h);
      vectorSvg.style.width = def.w + "px"; vectorSvg.style.height = def.h + "px";
      vectorSvg.addEventListener("pointerdown", vecPointerDown);
      vectorSvg.addEventListener("dblclick", vecDblClick);
      root.appendChild(vectorSvg);

      if (def.mask) buildMaskFrame(root);              // editor handle for the glyph mask
      root.addEventListener("pointerdown", editorDown, true);  // capture: pre-empt key handlers in edit mode
      shellHost.appendChild(root);
      shellRoot = root;
      document.body.classList.add("shell-mode");
      applyHitAreas();
      applyEditMode();                                // re-apply editor affordances on model switch
    }

    function teardownShell() {
      if (!shellRoot) return;
      document.body.classList.remove("shell-mode");
      shellBg = null; shellBgLit = null; shellPixel = null; shellLcdEl = null; shellLitState = -1; maskFrameEl = null;
      vectorSvg = null; vecZoneKey = null; vecSelNode = null; vecPaths = {}; zoneEls = {}; zoneRect = {};
      canvas.style.width = ""; canvas.style.height = "";   // drop shell-only sizing
      if (lcdHome) lcdHome.appendChild(canvas);            // canvas → grid home
      shellHost.innerHTML = ""; shellHost.style.width = ""; shellHost.style.height = "";
      shellRoot = null;
    }

    function bindZone(b, label) {
      // pwr drives the real power path (tap/hold ⇒ on / off); everything else is a
      // normal logical keypress. pointerleave releases on drag-off (like the grid).
      const isPwr = label === "pwr";
      let pressed = false;
      const down = (e) => {
        e.preventDefault(); if (pressed) return; pressed = true; b.classList.add("active");
        if (isPwr) powerDown(e); else pressKey(label, true);
      };
      const up = (e) => {
        if (e) e.preventDefault(); if (!pressed) return; pressed = false; b.classList.remove("active");
        if (isPwr) powerUp(); else pressKey(label, false);
      };
      b.addEventListener("pointerdown", down);
      b.addEventListener("pointerup", up);
      b.addEventListener("pointercancel", up);
      b.addEventListener("pointerleave", up);
    }

    function applyHitAreas() {
      if (!shellRoot) return;
      shellRoot.style.setProperty("--hz-border", showHitAreas ? "rgba(255,40,90,.9)"  : "transparent");
      shellRoot.style.setProperty("--hz-bg",     showHitAreas ? "rgba(255,40,90,.16)" : "transparent");
    }

    // ---- Backlight ("bglight") overlay editor -------------------------------
    // Push the live colour/blur/intensity onto the shell root as CSS vars — the
    // masked-glyph glow and every glow region read them. Called on build and on every
    // editor tweak.
    function applyLightVars(root) {
      root = root || shellRoot; if (!root) return;
      root.style.setProperty("--shell-light", lightColor);
      root.style.setProperty("--light-blur", lightBlur + "px");
      root.style.setProperty("--light-bloom", lightBloom);
      root.style.setProperty("--mask-x", maskX + "px");
      root.style.setProperty("--mask-y", maskY + "px");
      root.style.setProperty("--mask-sx", maskSX);
      root.style.setProperty("--mask-sy", maskSY);
      positionMaskFrame();
    }

    // ---- Glyph-mask handle frame --------------------------------------------
    // A dashed border around the mask's projected box with a move grip (top-left) and a
    // stretch grip (bottom-right), so the mask is directly draggable in edit mode — the
    // overlay itself can't be selected (pointer-events:none), so it needs explicit grips.
    function buildMaskFrame(root) {
      const f = document.createElement("div");
      f.className = "shell-mask-frame"; f.dataset.key = "mask";
      const mv = document.createElement("div"); mv.className = "mask-grip mask-move"; mv.title = "drag: move glyph mask";
      const mz = document.createElement("div"); mz.className = "mask-grip mask-size"; mz.title = "drag: stretch glyph mask";
      f.appendChild(mv); f.appendChild(mz);
      root.appendChild(f);
      maskFrameEl = f;
      bindMaskGrip(mv, "move"); bindMaskGrip(mz, "size");
      positionMaskFrame();
    }

    function positionMaskFrame() {
      if (!maskFrameEl || !shellDef) return;
      maskFrameEl.style.left = maskX + "px"; maskFrameEl.style.top = maskY + "px";
      maskFrameEl.style.width = (shellDef.w * maskSX) + "px";
      maskFrameEl.style.height = (shellDef.h * maskSY) + "px";
    }

    function bindMaskGrip(el, mode) {
      el.addEventListener("pointerdown", (e) => {
        if (!editMode) return;
        e.preventDefault(); e.stopPropagation();
        const sx = e.clientX, sy = e.clientY, bx = maskX, by = maskY, bsx = maskSX, bsy = maskSY;
        const move = (ev) => {
          const dx = (ev.clientX - sx) / shellScale, dy = (ev.clientY - sy) / shellScale;
          if (mode === "move") { maskX = Math.round(bx + dx); maskY = Math.round(by + dy); }
          else { maskSX = Math.max(0.5, +(bsx + dx / shellDef.w).toFixed(3));
                 maskSY = Math.max(0.5, +(bsy + dy / shellDef.h).toFixed(3)); }
          applyLightVars(); syncLightEditor();
        };
        const up = () => { window.removeEventListener("pointermove", move); window.removeEventListener("pointerup", up); };
        window.addEventListener("pointermove", move); window.addEventListener("pointerup", up);
      });
    }

    // ---- Vector (bézier) key-shape editor -----------------------------------
    // A zone's hit-area can be a closed cubic bézier (clip-path:path) instead of a rect.
    // The SVG overlay shows draggable anchors + control handles per node; drag an anchor to
    // move the node (handles ride along), a control dot to bend the curve. Double-click the
    // outline to insert a node, Delete to remove the selected one. Exports as `path`.
    const SVGNS = "http://www.w3.org/2000/svg";
    const lerp = (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
    const rr = (v) => Math.round(v * 10) / 10;

    // 4-node ellipse fitted to a zone's rect (the smooth starting shape on convert).
    function seedBezier(rect) {
      const k = 0.5523, cx = rect.left + rect.w / 2, cy = rect.top + rect.h / 2, rx = rect.w / 2, ry = rect.h / 2;
      return [
        { x: cx,      y: cy - ry, ix: cx - k * rx, iy: cy - ry,      ox: cx + k * rx, oy: cy - ry },
        { x: cx + rx, y: cy,      ix: cx + rx,      iy: cy - k * ry,  ox: cx + rx,      oy: cy + k * ry },
        { x: cx,      y: cy + ry, ix: cx + k * rx, iy: cy + ry,      ox: cx - k * rx, oy: cy + ry },
        { x: cx - rx, y: cy,      ix: cx - rx,      iy: cy + k * ry,  ox: cx - rx,      oy: cy - k * ry },
      ];
    }

    function pathD(nodes) {
      let d = "M " + rr(nodes[0].x) + " " + rr(nodes[0].y);
      for (let i = 0; i < nodes.length; i++) {
        const a = nodes[i], b = nodes[(i + 1) % nodes.length];
        d += " C " + rr(a.ox) + " " + rr(a.oy) + " " + rr(b.ix) + " " + rr(b.iy) + " " + rr(b.x) + " " + rr(b.y);
      }
      return d + " Z";
    }

    // Apply / refresh a zone's clip-path from its node list (turns the button full-frame).
    function applyZoneClip(key) {
      const b = zoneEls[key], nodes = vecPaths[key];
      if (!b || !nodes) return;
      const css = 'path("' + pathD(nodes) + '")';
      b.classList.add("vector");
      b.style.left = "0px"; b.style.top = "0px"; b.style.width = "100%"; b.style.height = "100%";
      b.style.borderRadius = "0";
      b.style.clipPath = css; b.style.webkitClipPath = css;
    }

    // Select a zone for shape editing (seeding a bézier from its rect on first touch).
    function selectVectorZone(key) {
      vecSelNode = null;
      vecZoneKey = key || null;
      if (!vecZoneKey) { vectorSvg.classList.remove("active"); vectorSvg.replaceChildren(); updateReadout(); return; }
      if (!vecPaths[vecZoneKey]) { vecPaths[vecZoneKey] = seedBezier(zoneRect[vecZoneKey]); applyZoneClip(vecZoneKey); }
      vectorSvg.classList.add("active");
      renderVec();
      updateReadout();
    }

    function svgEl(tag, attrs) {
      const e = document.createElementNS(SVGNS, tag);
      for (const k in attrs) e.setAttribute(k, attrs[k]);
      return e;
    }

    function renderVec() {
      vectorSvg.replaceChildren();
      const nodes = vecPaths[vecZoneKey]; if (!nodes) return;
      const d = pathD(nodes);
      vectorSvg.appendChild(svgEl("path", { class: "vec-outline", d }));
      vectorSvg.appendChild(svgEl("path", { class: "vec-hit", d }));
      nodes.forEach((n, i) => {
        vectorSvg.appendChild(svgEl("line", { class: "vec-hline", x1: n.x, y1: n.y, x2: n.ix, y2: n.iy }));
        vectorSvg.appendChild(svgEl("line", { class: "vec-hline", x1: n.x, y1: n.y, x2: n.ox, y2: n.oy }));
        vectorSvg.appendChild(svgEl("circle", { class: "vec-ctrl", "data-node": i, "data-kind": "i", cx: n.ix, cy: n.iy, r: 7 }));
        vectorSvg.appendChild(svgEl("circle", { class: "vec-ctrl", "data-node": i, "data-kind": "o", cx: n.ox, cy: n.oy, r: 7 }));
        vectorSvg.appendChild(svgEl("rect", { class: "vec-anchor" + (i === vecSelNode ? " sel" : ""), "data-node": i, "data-kind": "a",
          x: n.x - 8, y: n.y - 8, width: 16, height: 16 }));
      });
    }

    // client → SVG user (design px) coordinates.
    function svgPoint(e) {
      const m = vectorSvg.getScreenCTM(); if (!m) return { x: 0, y: 0 };
      const p = vectorSvg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY;
      const u = p.matrixTransform(m.inverse());
      return { x: u.x, y: u.y };
    }

    function vecPointerDown(e) {
      const t = e.target;
      if (!t.classList || !(t.classList.contains("vec-anchor") || t.classList.contains("vec-ctrl"))) return;
      e.preventDefault(); e.stopPropagation();
      const i = +t.dataset.node, kind = t.dataset.kind, n = vecPaths[vecZoneKey][i];
      vecSelNode = i;
      const s = svgPoint(e), base = { x: n.x, y: n.y, ix: n.ix, iy: n.iy, ox: n.ox, oy: n.oy };
      const move = (ev) => {
        const p = svgPoint(ev), dx = p.x - s.x, dy = p.y - s.y;
        if (kind === "a") { n.x = base.x + dx; n.y = base.y + dy; n.ix = base.ix + dx; n.iy = base.iy + dy; n.ox = base.ox + dx; n.oy = base.oy + dy; }
        else if (kind === "i") { n.ix = base.ix + dx; n.iy = base.iy + dy; }
        else { n.ox = base.ox + dx; n.oy = base.oy + dy; }
        applyZoneClip(vecZoneKey); renderVec();
      };
      const up = () => { window.removeEventListener("pointermove", move); window.removeEventListener("pointerup", up); };
      window.addEventListener("pointermove", move); window.addEventListener("pointerup", up);
      renderVec();
    }

    // de Casteljau midpoint split → insert a node on the nearest segment.
    function vecDblClick(e) {
      if (!e.target.classList || !e.target.classList.contains("vec-hit")) return;
      e.preventDefault();
      const nodes = vecPaths[vecZoneKey]; if (!nodes) return;
      const p = svgPoint(e);
      let best = 0, bd = Infinity;
      for (let i = 0; i < nodes.length; i++) {
        const a = nodes[i], b = nodes[(i + 1) % nodes.length];
        const P0 = { x: a.x, y: a.y }, P1 = { x: a.ox, y: a.oy }, P2 = { x: b.ix, y: b.iy }, P3 = { x: b.x, y: b.y };
        const m = lerp(lerp(lerp(P0, P1, .5), lerp(P1, P2, .5), .5), lerp(lerp(P1, P2, .5), lerp(P2, P3, .5), .5), .5);
        const dd = (m.x - p.x) ** 2 + (m.y - p.y) ** 2;
        if (dd < bd) { bd = dd; best = i; }
      }
      const a = nodes[best], b = nodes[(best + 1) % nodes.length];
      const P0 = { x: a.x, y: a.y }, P1 = { x: a.ox, y: a.oy }, P2 = { x: b.ix, y: b.iy }, P3 = { x: b.x, y: b.y };
      const A = lerp(P0, P1, .5), B = lerp(P1, P2, .5), C = lerp(P2, P3, .5), D = lerp(A, B, .5), E = lerp(B, C, .5), F = lerp(D, E, .5);
      a.ox = A.x; a.oy = A.y; b.ix = C.x; b.iy = C.y;
      nodes.splice(best + 1, 0, { x: F.x, y: F.y, ix: D.x, iy: D.y, ox: E.x, oy: E.y });
      applyZoneClip(vecZoneKey); renderVec();
    }

    // Build one glow-region div from a {left,top,w,h,radius} def (design px).
    function buildGlow(root, g) {
      const d = document.createElement("div");
      d.className = "shell-glow";
      d.dataset.key = "glow" + (glowSeq++);
      d.dataset.radius = g.radius != null ? g.radius : 24;
      d.style.left = (g.left || 0) + "px"; d.style.top = (g.top || 0) + "px";
      d.style.width = (g.w || 80) + "px";  d.style.height = (g.h || 80) + "px";
      d.style.borderRadius = d.dataset.radius + "px";
      root.appendChild(d);
      return d;
    }

    // Add a fresh glow region at the shell centre and select it for dragging.
    function addGlow() {
      if (!shellRoot || !shellDef) return;
      const w = 120, h = 80;
      const d = buildGlow(shellRoot, { left: Math.round(shellDef.w / 2 - w / 2),
                                       top: Math.round(shellDef.h / 2 - h / 2), w, h, radius: 24 });
      selectEl(d);
    }

    // Mirror the live backlight state into the editor controls (called when the panel
    // is shown or the shell is rebuilt).
    function syncLightEditor() {
      const c = document.getElementById("sle-color"); if (c) c.value = lightColor;
      const i = document.getElementById("sle-intensity"); if (i) i.value = lightBloom;
      const b = document.getElementById("sle-blur"); if (b) b.value = lightBlur;
      const mx = document.getElementById("sle-maskx"); if (mx) mx.value = maskX;
      const my = document.getElementById("sle-masky"); if (my) my.value = maskY;
      const msx = document.getElementById("sle-masksx"); if (msx) msx.value = maskSX;
      const msy = document.getElementById("sle-masksy"); if (msy) msy.value = maskSY;
    }

    // ---- Shell zone editor --------------------------------------------------
    // Drag a zone to move it, grab its bottom-right corner to resize; click to
    // select then arrow-nudge (Shift = resize, Alt = ×10 step). The LCD rect is
    // editable too. Export emits a self-contained shell.js (geometry from the live
    // DOM, everything else preserved from the active def) to save over
    // web/shells/<model>/shell.js. All geometry is DESIGN px = screen px / scale.
    let drag = null;

    const liveGeo = (el) => ({
      left: Math.round(parseFloat(el.style.left)   || 0),
      top:  Math.round(parseFloat(el.style.top)    || 0),
      w:    Math.round(parseFloat(el.style.width)  || 0),
      h:    Math.round(parseFloat(el.style.height) || 0),
    });
    const setGeo = (el, g) => {
      el.style.left = g.left + "px"; el.style.top = g.top + "px";
      el.style.width = g.w + "px";   el.style.height = g.h + "px";
    };

    function selectEl(el) {
      if (selectedEl && selectedEl !== el) selectedEl.classList.remove("selected");
      selectedEl = el || null;
      if (selectedEl) selectedEl.classList.add("selected");
      updateReadout();
    }

    function editorDown(e) {
      if (!editMode) return;                          // normal mode: let the keypress handlers run
      if (editTarget === "shape") {                   // bézier: select a key, overlay edits its nodes
        if (e.target.closest(".shell-vector")) return;       // overlay handles its own pointers
        const z = e.target.closest(".shell-zone");
        e.preventDefault(); e.stopPropagation();
        selectVectorZone(z ? z.dataset.key : null);
        return;
      }
      // only the active layer's elements are selectable (Edit dropdown): keeps it manageable
      const sel = editTarget === "zones" ? ".shell-zone"
                : editTarget === "screen" ? ".shell-lcd"
                : editTarget === "mask" ? ".shell-glow" : null;
      if (!sel) return;
      const el = e.target.closest(sel);
      if (!el) return;
      e.preventDefault(); e.stopPropagation();        // capture-phase: block the key bound to this zone
      selectEl(el);
      const r = el.getBoundingClientRect();
      // bottom-right corner = resize; shrink the grab zone on small elements so they stay movable
      const cw = Math.min(18, (r.right - r.left) * 0.4), ch = Math.min(18, (r.bottom - r.top) * 0.4);
      const corner = e.clientX > r.right - cw && e.clientY > r.bottom - ch;
      const g = liveGeo(el);
      drag = { el, mode: corner ? "resize" : "move", px: e.clientX, py: e.clientY, l: g.left, t: g.top, w: g.w, h: g.h };
      window.addEventListener("pointermove", editorMove);
      window.addEventListener("pointerup", editorUp);
    }
    function editorMove(e) {
      if (!drag) return;
      const dx = (e.clientX - drag.px) / shellScale, dy = (e.clientY - drag.py) / shellScale;
      if (drag.mode === "move")
        setGeo(drag.el, { left: Math.round(drag.l + dx), top: Math.round(drag.t + dy), w: drag.w, h: drag.h });
      else
        setGeo(drag.el, { left: drag.l, top: drag.t,
                          w: Math.max(8, Math.round(drag.w + dx)), h: Math.max(8, Math.round(drag.h + dy)) });
      updateReadout();
    }
    function editorUp() {
      drag = null;
      window.removeEventListener("pointermove", editorMove);
      window.removeEventListener("pointerup", editorUp);
    }

    // Returns true if the key was consumed by the editor (so it isn't sent as a keypress).
    function editorKey(e) {
      if (editTarget === "shape") {                   // bézier node ops
        if (e.key === "Escape") { selectVectorZone(null); e.preventDefault(); return true; }
        if ((e.key === "Delete" || e.key === "Backspace") && vecZoneKey && vecSelNode != null) {
          const nodes = vecPaths[vecZoneKey];
          if (nodes && nodes.length > 3) { nodes.splice(vecSelNode, 1); vecSelNode = null; applyZoneClip(vecZoneKey); renderVec(); }
          e.preventDefault(); return true;
        }
        return false;
      }
      if (e.key === "Escape") { selectEl(null); e.preventDefault(); return true; }
      if (!selectedEl) return false;
      // Delete/Backspace removes a selected glow region (zones + LCD are fixed).
      if ((e.key === "Delete" || e.key === "Backspace") && selectedEl.classList.contains("shell-glow")) {
        const g = selectedEl; selectEl(null); g.remove(); e.preventDefault(); return true;
      }
      const dirs = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] };
      const d = dirs[e.key];
      if (!d) return false;
      const step = e.altKey ? 10 : 1, g = liveGeo(selectedEl);
      if (e.shiftKey) { g.w = Math.max(8, g.w + d[0] * step); g.h = Math.max(8, g.h + d[1] * step); }
      else            { g.left += d[0] * step; g.top += d[1] * step; }
      setGeo(selectedEl, g); updateReadout();
      e.preventDefault(); return true;
    }

    function applyEditMode() {
      editMode = editTarget !== "off";
      const on = editMode && !!shellRoot;
      document.body.classList.toggle("shell-editing", on);
      // scope affordances to the active layer so only its handles/borders show
      document.body.classList.toggle("edit-zones",  on && editTarget === "zones");
      document.body.classList.toggle("edit-screen", on && editTarget === "screen");
      document.body.classList.toggle("edit-mask",   on && editTarget === "mask");
      document.body.classList.toggle("edit-shape",  on && editTarget === "shape");
      if (selectedEl) { selectedEl.classList.remove("selected"); selectedEl = null; }
      if (!(on && editTarget === "shape") && vectorSvg) selectVectorZone(null);   // drop the bézier overlay
      const exp = document.getElementById("btn-export-shell");
      if (exp) exp.style.display = editMode ? "" : "none";
      const led = document.getElementById("shell-light-editor");
      if (led) led.style.display = (on && editTarget === "mask") ? "" : "none";
      // Key-mask editing forces the glow lit (so edits are visible); otherwise the persistent
      // "💡 Lights" toggle holds it on. Without either it follows the firmware (leds & 2).
      recomputeLight();
      if (on && editTarget === "mask") syncLightEditor();
      updateReadout();
    }

    // Effective keypad-glow hold: on while key-mask editing, or while the Lights toggle is set.
    function recomputeLight() { lightPreview = holdLight || (editMode && editTarget === "mask"); }

    function updateReadout() {
      const ro = document.getElementById("shell-edit-readout");
      if (!ro) return;
      if (!editMode) { ro.textContent = ""; return; }
      if (editTarget === "shape") {
        ro.textContent = vecZoneKey
          ? "[" + vecZoneKey + "] shape — drag anchors/handles · double-click outline to add a point · Delete removes · Esc deselects"
          : "shapes: click a key to edit its outline (it becomes an editable bézier)";
        return;
      }
      if (!selectedEl) { ro.textContent = "edit: click a zone — drag to move · drag corner to resize · arrows nudge (Shift = resize · Alt = ×10)"; return; }
      const g = liveGeo(selectedEl);
      ro.textContent = "[" + selectedEl.dataset.key + "]  left " + g.left + "  top " + g.top + "  w " + g.w + "  h " + g.h;
    }

    const shellName = () => {
      const S = window.DCT3_SHELLS || {};
      return Object.keys(S).find((k) => S[k] === shellDef) || (C.model() || "model");
    };

    function exportShell() {
      if (!shellDef || !shellRoot) return;
      const name = shellName(), L = shellDef.lcd || {};
      const lg = shellLcdEl ? liveGeo(shellLcdEl) : { left: L.left, top: L.top, w: L.w, h: L.h };
      const out = [];
      out.push("// Nokia " + name + " shell unit — exported from the in-page zone editor.");
      out.push('registerShell("' + name + '", {');
      out.push("  img:  " + JSON.stringify(shellDef.img) + ",");
      if (shellDef.mask) out.push("  mask: " + JSON.stringify(shellDef.mask) + ",");
      out.push("  w: " + shellDef.w + ", h: " + shellDef.h + ",");
      let lcd = "  lcd:  { left: " + lg.left + ", top: " + lg.top + ", w: " + lg.w + ", h: " + lg.h;
      if (L.radius  != null) lcd += ", radius: " + L.radius;
      if (L.canvasW != null) lcd += ", canvasW: " + L.canvasW;
      if (L.canvasH != null) lcd += ", canvasH: " + L.canvasH;
      out.push(lcd + " },");
      if (shellDef.lcdColor)    out.push("  lcdColor: " + JSON.stringify(shellDef.lcdColor) + ",");
      if (shellDef.lcdColorLit) out.push("  lcdColorLit: " + JSON.stringify(shellDef.lcdColorLit) + ",");
      if (shellDef.pixelColor)  out.push("  pixelColor: " + JSON.stringify(shellDef.pixelColor) + ",");
      // Backlight overlay: live colour + tuned blur/intensity (only emit non-defaults).
      out.push("  light: " + JSON.stringify(lightColor) + ",");
      if (Math.abs(lightBlur  - LIGHT_BLUR_DEFAULT)  > 1e-3) out.push("  lightBlur: " + lightBlur + ",");
      if (Math.abs(lightBloom - LIGHT_BLOOM_DEFAULT) > 1e-3) out.push("  lightBloom: " + lightBloom + ",");
      if (maskX) out.push("  maskX: " + maskX + ",");
      if (maskY) out.push("  maskY: " + maskY + ",");
      if (Math.abs(maskSX - 1) > 1e-4) out.push("  maskScaleX: " + maskSX + ",");
      if (Math.abs(maskSY - 1) > 1e-4) out.push("  maskScaleY: " + maskSY + ",");
      const glows = shellRoot.querySelectorAll(".shell-glow");
      if (glows.length) {
        out.push("  glow: [");
        Array.prototype.forEach.call(glows, (el) => {
          const g = liveGeo(el), r = el.dataset.radius;
          let s = "    { left: " + g.left + ", top: " + g.top + ", w: " + g.w + ", h: " + g.h;
          if (r != null) s += ", radius: " + r;
          out.push(s + " },");
        });
        out.push("  ],");
      }
      out.push("  zones: [");
      const rByKey = {};
      (shellDef.zones || []).forEach((z) => { rByKey[z.key] = z.r; });
      Array.prototype.forEach.call(shellRoot.querySelectorAll(".shell-zone"), (el) => {
        const key = el.dataset.key;
        if (vecPaths[key]) {                          // bézier shape → emit the node path
          const pts = vecPaths[key].map((n) => "[" + [n.x, n.y, n.ix, n.iy, n.ox, n.oy].map(Math.round).join(",") + "]").join(", ");
          out.push("    { key: " + JSON.stringify(key) + ", path: [" + pts + "] },");
          return;
        }
        const g = liveGeo(el);
        let s = "    { key: " + JSON.stringify(key) + ", left: " + g.left + ", top: " + g.top + ", w: " + g.w + ", h: " + g.h;
        if (rByKey[key] != null) s += ", r: " + JSON.stringify(rByKey[key]);
        out.push(s + " },");
      });
      out.push("  ]");
      out.push("});");
      showExport(out.join("\n"));
    }

    function showExport(text) {
      let modal = document.getElementById("shell-export-modal");
      if (!modal) {
        modal = document.createElement("div");
        modal.id = "shell-export-modal"; modal.className = "shell-export-modal";
        modal.innerHTML =
          '<div class="sx-box">' +
          '<div class="sx-head">Shell config — save over <code>web/shells/&lt;model&gt;/shell.js</code></div>' +
          '<textarea id="sx-text" spellcheck="false"></textarea>' +
          '<div class="sx-actions"><button id="sx-copy">Copy</button><button id="sx-close">Close</button></div>' +
          "</div>";
        document.body.appendChild(modal);
        modal.querySelector("#sx-close").addEventListener("click", () => { modal.style.display = "none"; });
        modal.addEventListener("click", (e) => { if (e.target === modal) modal.style.display = "none"; });
        modal.querySelector("#sx-copy").addEventListener("click", () => {
          const ta = modal.querySelector("#sx-text"), btn = modal.querySelector("#sx-copy");
          const done = (ok) => { btn.textContent = ok ? "Copied ✓" : "Copy failed"; setTimeout(() => (btn.textContent = "Copy"), 1200); };
          ta.select();
          if (navigator.clipboard) navigator.clipboard.writeText(ta.value).then(() => done(true), () => done(false));
          else { try { done(document.execCommand("copy")); } catch (_) { done(false); } }
        });
      }
      modal.querySelector("#sx-text").value = text;
      modal.style.display = "flex";
      console.log("[DCT3] shell config:\n" + text);
    }

    // Pick shell vs grid for the active model. Called by applyModel() each boot and
    // when the View override flips.
    function layoutForModel() {
      const def = viewOverride === "grid" ? null : pickShell(C.model() || "");
      if (def) buildShell(def);
      else { teardownShell(); buildKeypad(currentFamily()); }
    }
    {
      const vsel = document.getElementById("view-override");
      if (vsel) vsel.addEventListener("change", () => { viewOverride = vsel.value; layoutForModel(); });
      const hchk = document.getElementById("chk-hitareas");
      if (hchk) hchk.addEventListener("change", () => { showHitAreas = hchk.checked; applyHitAreas(); });
      const etgt = document.getElementById("edit-target");
      if (etgt) etgt.addEventListener("change", () => { editTarget = etgt.value; applyEditMode(); });
      const hold = document.getElementById("chk-holdlight");
      if (hold) hold.addEventListener("change", () => { holdLight = hold.checked; recomputeLight(); });
      const ebtn = document.getElementById("btn-export-shell");
      if (ebtn) ebtn.addEventListener("click", exportShell);
      // Backlight overlay editor controls (live CSS-var updates on the shell root).
      const sc = document.getElementById("sle-color");
      if (sc) sc.addEventListener("input", () => { lightColor = sc.value; applyLightVars(); });
      const si = document.getElementById("sle-intensity");
      if (si) si.addEventListener("input", () => { lightBloom = parseFloat(si.value); applyLightVars(); });
      const sb = document.getElementById("sle-blur");
      if (sb) sb.addEventListener("input", () => { lightBlur = parseFloat(sb.value); applyLightVars(); });
      const sg = document.getElementById("sle-add-glow");
      if (sg) sg.addEventListener("click", addGlow);
      // Glyph-mask registration nudge.
      const smx = document.getElementById("sle-maskx");
      if (smx) smx.addEventListener("input", () => { maskX = parseFloat(smx.value) || 0; applyLightVars(); });
      const smy = document.getElementById("sle-masky");
      if (smy) smy.addEventListener("input", () => { maskY = parseFloat(smy.value) || 0; applyLightVars(); });
      const smsx = document.getElementById("sle-masksx");
      if (smsx) smsx.addEventListener("input", () => { maskSX = parseFloat(smsx.value) || 1; applyLightVars(); });
      const smsy = document.getElementById("sle-masksy");
      if (smsy) smsy.addEventListener("input", () => { maskSY = parseFloat(smsy.value) || 1; applyLightVars(); });
    }

    // Physical keyboard -> logical key label. Digit rows are FLIPPED to match the phone's
    // spatial layout: a keyboard numpad/number-row has 7-8-9 on top and 1-2-3 on the
    // bottom, but the phone is the reverse — so the host top row (7 8 9) drives the phone's
    // top row (1 2 3) and the host bottom row (1 2 3) drives the phone's bottom row (7 8 9).
    // Middle row (4 5 6) and 0/*/# are unchanged.
    const KMAP = {
      "7": "1", "8": "2", "9": "3",
      "4": "4", "5": "5", "6": "6",
      "1": "7", "2": "8", "3": "9",
      "0": "0", "*": "*", "#": "#",
      // WASD → the phone's directional digits (2=up, 4=left, 6=right, 8=down — the
      // standard Nokia keypad directions used by Snake et al.).
      "w": "2", "W": "2", "a": "4", "A": "4",
      "s": "8", "S": "8", "d": "6", "D": "6",
      "ArrowUp": "up", "ArrowDown": "down",
      "Enter": "soft1", "Backspace": "soft2",
      "ArrowLeft": "send", "ArrowRight": "end",
    };
    // Don't steal keys while the user is typing in a config/diagnostics field.
    const inField = (e) => {
      const t = e.target, n = t && t.tagName;
      return n === "INPUT" || n === "TEXTAREA" || n === "SELECT" || (t && t.isContentEditable);
    };
    // Collapse OS key-autorepeat into a single logical press, so HOLDING a nav
    // key doesn't roll the menu. Two repeat shapes to defeat:
    //   - modern browsers: repeated keydown events (e.repeat === true)
    //   - X11/Linux: phantom keyup+keydown PAIRS for every repeat tick
    // Fix: ignore e.repeat keydowns, and DEFER the keyup release briefly — if a
    // keydown for the same key arrives within the window it was an autorepeat
    // (cancel the release); only a real release (no keydown follows) lets go.
    // (The firmware-side hold is still capped in the wasm, see KEY_HOLD_INSNS.)
    const held = {};
    const releaseTimer = {};
    const AUTOREPEAT_MS = 90;            // > the OS repeat interval (~25-40ms)
    window.addEventListener("keydown", (e) => {
      if (inField(e)) return;                // let config/diagnostics fields receive typing
      if (editMode && editorKey(e)) return;  // edit mode: arrows nudge the selected zone
      const m = KMAP[e.key];
      if (!m) return;
      e.preventDefault();
      if (releaseTimer[e.key]) { clearTimeout(releaseTimer[e.key]); releaseTimer[e.key] = 0; }
      if (e.repeat || held[e.key]) return;   // autorepeat -> already pressed
      held[e.key] = true;
      pressKey(m, true);
    });
    window.addEventListener("keyup", (e) => {
      const m = KMAP[e.key];
      if (!m) return;
      if (inField(e)) return;
      e.preventDefault();
      if (releaseTimer[e.key]) clearTimeout(releaseTimer[e.key]);
      releaseTimer[e.key] = setTimeout(() => {
        releaseTimer[e.key] = 0; held[e.key] = false;
        pressKey(m, false);
      }, AUTOREPEAT_MS);
    });

    // Navi Roller via the mouse wheel: on the 7110 (family 3) scrolling the wheel over
    // the phone rotates the roller — wheel up = scroll up, wheel down = scroll down. One
    // detent per wheel notch (deltaY sign); a small cooldown avoids flooding the encoder.
    let rollAt = 0;
    window.addEventListener("wheel", (e) => {
      if (currentFamily() !== 3) return;   // only the 7110 has a roller
      if (inField(e)) return;
      e.preventDefault();
      const now = performance.now();
      if (now - rollAt < 30) return;       // one step per ~30ms burst
      rollAt = now;
      pressKey(e.deltaY < 0 ? "wheelup" : "wheeldown", true);
    }, { passive: false });

    document.getElementById("btn-reboot").addEventListener("click", boot);
    const saveBtn = document.getElementById("btn-save-eeprom");
    if (saveBtn) saveBtn.addEventListener("click", () => {
      const r = saveNvram(true);
      const msg = { saved: "Saved ✓", nochange: "Saved ✓", blank: "Nothing to save (blank)",
                    error: "Save failed", off: "(reset pending)", custom: "Custom FW — not saved" }[r] || r;
      if (r === "saved") console.log("[DCT3] manual EEPROM save: " + (C.eepromSize() / 1024).toFixed(0) + " KB -> localStorage");
      saveBtn.textContent = msg;
      setTimeout(() => (saveBtn.textContent = "Save EEPROM"), 1200);
    });
    document.getElementById("btn-reset-eeprom").addEventListener("click", () => {
      if (confirm("Wipe saved EEPROM/NVRAM (settings, clock, contacts) and reload the original image?"))
        window.dct3ResetEeprom();
    });
    // Security-code unlock: at a "Security code" screen, type any 5 digits, click this,
    // then press OK. It rewrites the entry buffer to the real code 12345 (working around
    // the keypad first-keypress char-null bug) so the firmware's own verify+accept path
    // runs. Re-click before each gate (boot lock + Restore-Factory-Settings re-prompt).
    const seccodeBtn = document.getElementById("btn-seccode-unlock");
    if (seccodeBtn) seccodeBtn.addEventListener("click", () => {
      const ok = C.seccodeReset && C.seccodeReset() === 1;
      seccodeBtn.textContent = ok ? "Entry set → 12345 ✓" : "Not supported";
      setTimeout(() => (seccodeBtn.textContent = "Unlock security code"), 1400);
    });

    // --- Firmware picker: load a user-supplied raw .fls into MEMFS and reboot into it.
    // Web boot reads /fw.fls raw (no FIASCO unwrap), so this expects a raw ~2 MB DCT3
    // dump. The image is stashed in IndexedDB so it (and its per-image NVRAM) survive a
    // page reload; dct3ClearFirmware() reverts to the built-in image.
    const fwFile = document.getElementById("fw-file");
    const fwName = document.getElementById("fw-name");
    if (fwFile) fwFile.addEventListener("change", async (e) => {
      const f = e.target.files && e.target.files[0];
      if (!f) return;
      let buf;
      try { buf = new Uint8Array(await f.arrayBuffer()); }
      catch (err) { console.error("[DCT3] read failed:", err); if (fwName) fwName.textContent = "read failed"; return; }
      if (buf.length < 0x100000 || buf.length > 0x400000) {   // sanity: expect a ~2 MB raw dump
        if (fwName) fwName.textContent = "unexpected size " + (buf.length / 1024 / 1024).toFixed(2) + " MB";
        return;
      }
      try { mod.FS.writeFile("/fw.fls", buf); }
      catch (err) { console.error("[DCT3] FS write failed:", err); if (fwName) fwName.textContent = "load failed"; return; }
      customFw = true;
      lastFwBase = f.name.replace(/\.[^.]+$/, "");
      idbPutFw({ name: f.name, bytes: buf });               // persist across reloads
      if (fwName) fwName.textContent = f.name + " · " + (buf.length / 1024 / 1024).toFixed(1) + " MB";
      console.log("[DCT3] firmware override:", f.name, buf.length, "bytes — rebooting");
      try { boot(); } catch (err) { if (fwName) fwName.textContent = "boot failed (rc) — see console"; }
    });

    // --- Game injection (3410 J2ME): REPLACE the factory MIDlet slot with a new game.
    // JS port of the offline PMMCAT re-serializer (cf. tools/nvram/nvram_pmm.c). A game =
    // id 0x90 JAR (games block) + id 0x90 JAD (apps block) + id 0x91 registry nodes. We
    // swap the new JAR/JAD payloads into the existing entries and rename the nodes, then
    // re-serialize each 64 KB block (entry core = f4 90|idx|55 ff|kind=sum16(payload)|
    // val=len|offset=next-entry-pos). Replacing (not appending) reuses the factory game's
    // ~41 KB slot + its working menu registry, so real games fit and appear where the
    // built-in one did. Tree edges are by id-index (position-independent), so re-serialize
    // is safe.
    const PMM_MAGIC = [0x50, 0x4d, 0x4d, 0x43, 0x41, 0x54];   // "PMMCAT"
    const PMM_BLOCK = 0x10000, PMM_CAT = 0x20;
    const isPK = (p) => p[0] === 0x50 && p[1] === 0x4b && p[2] === 0x03 && p[3] === 0x04;
    const sum16 = (b, n) => { let s = 0; for (let i = 0; i < n; i++) s = (s + b[i]) & 0xffff; return s; };

    function pmmFindRealCatalogs(img) {
      const out = [];
      for (let i = 0; i + 6 <= img.length; i++) {
        let hit = true;
        for (let j = 0; j < 6; j++) if (img[i + j] !== PMM_MAGIC[j]) { hit = false; break; }
        if (!hit) continue;
        const blk = i - 6;
        if (blk < 0 || img[blk] !== 0xf0 || img[blk + 1] !== 0xf0) continue;
        const p = blk + PMM_CAT;
        if (img[p] === 0xf4 && img[p + 4] === 0x55 && img[p + 5] === 0xff) out.push(blk);
      }
      return out;
    }
    function pmmNonFF(img, blk) {
      let e = Math.min(blk + PMM_BLOCK, img.length);
      while (e > blk && img[e - 1] === 0xff) e--;
      return e;
    }
    // Parse one block into entries {id, idx, val, payload (bytes to next signature), sOrig, gap}.
    function pmmParse(img, blk) {
      const end = pmmNonFF(img, blk), starts = [];
      for (let p = blk + PMM_CAT; p + 6 <= end; p++)
        if (img[p] === 0xf4 && img[p + 4] === 0x55 && img[p + 5] === 0xff) starts.push(p);
      const ents = [];
      for (let k = 0; k < starts.length; k++) {
        const s = starts[k], nx = (k + 1 < starts.length) ? starts[k + 1] : end;
        ents.push({ id: img[s + 1], idx: (img[s + 2] << 8) | img[s + 3],
          val: (img[s + 8] << 8) | img[s + 9], sOrig: s - blk, gap: nx - s,
          payload: img.slice(s + 12, nx) });
      }
      return ents;
    }
    // Re-lay a block from an entry list, patching each offset field to the next entry pos.
    // Entries flagged {rebuild:true} get a fresh core (kind=sum16, val=len); others keep bytes.
    function pmmSerialize(img, blk, ents) {
      const buf = new Uint8Array(PMM_BLOCK); buf.fill(0xff);
      for (let i = 0; i < PMM_CAT; i++) buf[i] = img[blk + i];   // header
      let pos = PMM_CAT;
      for (const e of ents) {
        let body;
        if (e.rebuild) {
          const val = e.payload.length, kind = sum16(e.payload, val);
          body = new Uint8Array(12 + val);
          body[0] = 0xf4; body[1] = e.id; body[2] = (e.idx >> 8) & 0xff; body[3] = e.idx & 0xff;
          body[4] = 0x55; body[5] = 0xff;
          body[6] = (kind >> 8) & 0xff; body[7] = kind & 0xff;
          body[8] = (val >> 8) & 0xff; body[9] = val & 0xff;
          body.set(e.payload, 12);
        } else {
          body = img.slice(blk + e.sOrig, blk + e.sOrig + e.gap);   // keep verbatim (incl trailer)
        }
        if (pos + body.length > PMM_BLOCK)
          throw new Error("game too big for the games block by " + (pos + body.length - PMM_BLOCK) + " B — pick a smaller MIDlet");
        buf.set(body, pos);
        const off = pos + body.length;                    // offset field = next entry position
        buf[pos + 10] = (off >> 8) & 0xff; buf[pos + 11] = off & 0xff;
        pos += body.length;
      }
      img.set(buf, blk);
    }
    const nodeName = (p) => { const nc = (p[0] << 8) | p[1]; let n = ""; for (let q = 0; q < nc; q++) if (p[2 + q * 2] === 0) n += String.fromCharCode(p[3 + q * 2]); return n; };
    function renameNode(p, name) {
      const nc = (p[0] << 8) | p[1], meta = p.slice(2 + nc * 2);
      const nb = new Uint8Array(2 + name.length * 2 + meta.length);
      nb[0] = (name.length >> 8) & 0xff; nb[1] = name.length & 0xff;
      for (let q = 0; q < name.length; q++) { nb[2 + q * 2] = 0; nb[3 + q * 2] = name.charCodeAt(q) & 0xff; }
      nb.set(meta, 2 + name.length * 2);
      return nb;
    }
    // Replace the factory MIDlet with (jar, jad); rename its nodes to `name`. Returns new image.
    function injectGame(fw, jar, jad, name) {
      const img = new Uint8Array(fw);
      const cats = pmmFindRealCatalogs(img);
      if (!cats.length) throw new Error("no PMMCAT store — game injection is 3410-only");
      let games = -1, apps = -1;
      for (const b of cats) {
        const e = pmmParse(img, b);
        if (games < 0 && e.some((x) => x.id === 0x90 && isPK(x.payload))) games = b;
        else if (apps < 0 && e.some((x) => x.id === 0x90 && !isPK(x.payload))) apps = b;
      }
      if (games < 0) throw new Error("no games block (id 0x90 JAR) — is this a 3410?");
      if (apps < 0) apps = cats.find((b) => b !== games);
      // Old base name = the ".jar" registry node's stem, so we rename the whole node trio.
      let oldBase = null;
      for (const b of [games, apps]) for (const e of pmmParse(img, b))
        if (e.id === 0x91) { const n = nodeName(e.payload); if (/\.jar$/i.test(n)) oldBase = n.replace(/\.jar$/i, ""); }
      const renameTrio = (list) => { for (const e of list) if (e.id === 0x91) {
        const n = nodeName(e.payload);
        if (oldBase && (n === oldBase || n === oldBase + ".jar" || n === oldBase + ".jad")) {
          const suf = n === oldBase ? "" : n.slice(oldBase.length);
          e.payload = renameNode(e.payload, name + suf); e.rebuild = true;
        }
      } };
      const gList = pmmParse(img, games);
      for (const e of gList) if (e.id === 0x90 && isPK(e.payload)) { e.payload = jar; e.rebuild = true; }
      renameTrio(gList); pmmSerialize(img, games, gList);
      const aList = pmmParse(img, apps);
      for (const e of aList) if (e.id === 0x90 && !isPK(e.payload) && e.val > 20) { e.payload = jad; e.rebuild = true; }
      renameTrio(aList); pmmSerialize(img, apps, aList);
      return img;
    }

    const gameFile = document.getElementById("game-file");
    const gameName = document.getElementById("game-name");
    if (gameFile) gameFile.addEventListener("change", async (e) => {
      const files = Array.from(e.target.files || []);
      const say = (m) => { if (gameName) gameName.textContent = m; };
      let jarF = files.find((f) => /\.jar$/i.test(f.name));
      let jadF = files.find((f) => /\.jad$/i.test(f.name));
      if (!jarF || !jadF) { say("select BOTH the .jar and its .jad"); return; }
      let jar, jad;
      try {
        jar = new Uint8Array(await jarF.arrayBuffer());
        jad = new Uint8Array(await jadF.arrayBuffer());
      } catch (err) { console.error("[DCT3] game read failed:", err); say("read failed"); return; }
      if (jar[0] !== 0x50 || jar[1] !== 0x4b || jar[2] !== 0x03 || jar[3] !== 0x04) { say("not a JAR (no PK magic)"); return; }
      const base = jarF.name.replace(/\.[^.]+$/, "");
      let modified;
      try {
        const fw = mod.FS.readFile("/fw.fls");     // current live image
        modified = injectGame(fw, jar, jad, base); // replaces the built-in MIDlet slot
      } catch (err) { console.error("[DCT3] inject failed:", err); say("inject failed: " + err.message); return; }
      try { mod.FS.writeFile("/fw.fls", modified); } catch (err) { say("write failed"); return; }
      customFw = true;
      lastFwBase = (lastFwBase || "3410") + "+" + base;
      idbPutFw({ name: lastFwBase + ".fls", bytes: modified });   // persist across the reload
      say(base + " injected — rebooting…");
      console.log("[DCT3] game injected:", base, "jar", jar.length, "jad", jad.length, "B — reloading");
      // The wasm boots firmware once per process (2nd boot wedges), so reload for a clean boot.
      setTimeout(() => location.reload(), 400);
    });

    // --- Firmware inventory (far-left column) -----------------------------------
    // Render the local firmware library from web/firmware-manifest.json (built by
    // `make fw-manifest`), grouped by model with state badges. Clicking a row
    // fetches that .fls over HTTP (served via web/fw/ symlinks), writes it to
    // /fw.fls, persists it for reload, and reboots into it — same path as the file
    // picker above. After the boot settles we observe the LIVE result (boots /
    // no-boot) and persist it per-image so the badge self-corrects over the seed.
    const fwListEl = document.getElementById("fw-list");
    const fwSearchEl = document.getElementById("fw-search");
    const FW_STATE_KEY = "dct3_fw_state";   // { [id]: "boots"|"no-boot", ... } live results
    let fwManifest = null, fwActiveId = null, fwStateEvalTimer = 0;

    function fwLoadStates() {
      try { return JSON.parse(localStorage.getItem(FW_STATE_KEY) || "{}") || {}; }
      catch { return {}; }
    }
    function fwSaveState(id, state) {
      const m = fwLoadStates(); m[id] = state;
      try { localStorage.setItem(FW_STATE_KEY, JSON.stringify(m)); } catch {}
    }
    // Live state (localStorage) wins over the manifest seed; eeprom is independent.
    function fwEffectiveState(item) {
      return fwLoadStates()[item.id] || item.seedState || "untested";
    }

    function fwBadges(item) {
      const out = [];
      const st = fwEffectiveState(item);
      const LABELS = { boots: "boots", "no-boot": "no boot", cosim: "cosim", untested: "untested" };
      out.push(`<span class="fw-badge ${st}">${LABELS[st] || st}</span>`);
      if (item.eeprom) out.push('<span class="fw-badge ee">EEPROM</span>');
      return out.join("");
    }

    function fwRender(filter) {
      if (!fwListEl) return;
      if (!fwManifest) return;
      const items = (fwManifest.items || []).filter((it) => {
        if (!filter) return true;
        const hay = (it.name + " " + it.model + " " + it.version + " " + (it.tags || []).join(" ")).toLowerCase();
        return hay.includes(filter.toLowerCase());
      });
      if (!items.length) { fwListEl.innerHTML = '<div class="fw-hint">No firmware matches.</div>'; return; }
      let html = "", lastModel = null;
      for (const it of items) {
        if (it.model !== lastModel) { html += `<div class="fw-group">${it.model}</div>`; lastModel = it.model; }
        const meta = [it.family, it.version && "v" + it.version, it.variant, it.sizeMB + " MB"]
          .filter(Boolean).join(" · ");
        const active = it.id === fwActiveId ? " active" : "";
        html += `<div class="fw-item${active}" data-id="${it.id}" data-url="${it.url}" title="${it.src}">` +
                  `<div class="fw-title">${it.name}</div>` +
                  `<div class="fw-meta">${meta}</div>` +
                  `<div class="fw-badges">${fwBadges(it)}</div>` +
                `</div>`;
      }
      fwListEl.innerHTML = html;
    }

    async function fwSelect(item, rowEl) {
      if (rowEl) rowEl.classList.add("loading");
      if (status) status.textContent = "Loading " + item.name + "…";
      let buf;
      try {
        const r = await fetch(item.url, { cache: "no-store" });
        if (!r.ok) throw new Error("HTTP " + r.status);
        buf = new Uint8Array(await r.arrayBuffer());
      } catch (e) {
        console.error("[DCT3] firmware fetch failed:", item.url, e);
        if (status) status.textContent = "Load failed: " + item.name + " (" + e.message + ")";
        if (rowEl) rowEl.classList.remove("loading");
        return;
      }
      try { mod.FS.writeFile("/fw.fls", buf); }
      catch (e) { console.error("[DCT3] FS write failed:", e); if (rowEl) rowEl.classList.remove("loading"); return; }
      customFw = true;
      lastFwBase = item.name;
      fwActiveId = item.id;
      idbPutFw({ name: item.name + ".fls", bytes: buf });     // survive a page reload
      if (fwName) fwName.textContent = item.name + " · " + (buf.length / 1048576).toFixed(1) + " MB";
      console.log("[DCT3] inventory load:", item.name, buf.length, "bytes — rebooting");
      try { boot(); } catch (e) { if (status) status.textContent = "boot failed (rc) — see console"; }
      if (rowEl) rowEl.classList.remove("loading");
      fwRender(fwSearchEl ? fwSearchEl.value.trim() : "");    // refresh active highlight

      // Observe the live boot result and persist it so the badge self-corrects.
      // ~8 s of real time = plenty for the frame loop to reach standby or trip the
      // crash/spin detector. Don't downgrade a prior "boots" to "untested" on a
      // slow machine — only record a definite verdict.
      clearTimeout(fwStateEvalTimer);
      const evalId = item.id;
      fwStateEvalTimer = setTimeout(() => {
        if (fwActiveId !== evalId) return;                    // user moved on
        let verdict = null;
        if (crashed || spinFired) verdict = "no-boot";
        else if (C.lcdWrites() > 0 && C.step() > 30e6) verdict = "boots";  // ran + drew
        if (verdict) {
          fwSaveState(evalId, verdict);
          console.log("[DCT3] live state for", item.name, "→", verdict);
          fwRender(fwSearchEl ? fwSearchEl.value.trim() : "");
        }
      }, 8000);
    }

    if (fwListEl) {
      fwListEl.addEventListener("click", (e) => {
        const row = e.target.closest && e.target.closest(".fw-item");
        if (!row || !fwManifest) return;
        const item = (fwManifest.items || []).find((it) => it.id === row.dataset.id);
        if (item) fwSelect(item, row);
      });
      if (fwSearchEl) fwSearchEl.addEventListener("input", () => fwRender(fwSearchEl.value.trim()));
      fetch("firmware-manifest.json", { cache: "no-store" })
        .then((r) => { if (!r.ok) throw new Error("HTTP " + r.status); return r.json(); })
        .then((m) => {
          fwManifest = m;
          console.log("[DCT3] firmware inventory:", m.count, "images");
          fwRender("");
        })
        .catch((e) => {
          console.warn("[DCT3] no firmware manifest:", e.message);
          fwListEl.innerHTML = '<div class="fw-hint">No local firmware inventory. ' +
            'Run <code>make fw-manifest</code> (or <code>make serve</code>) to index ' +
            'your <code>firmware/</code> + <code>flash/</code> dumps.</div>';
        });
    }

    // --- EEPROM → file: download the current 192 KB NVRAM partition as a .bin.
    const eeFileBtn = document.getElementById("btn-eeprom-file");
    if (eeFileBtn) eeFileBtn.addEventListener("click", () => {
      const copy = new Uint8Array(eepromSlice());   // detach-safe copy of the live partition
      const blob = new Blob([copy], { type: "application/octet-stream" });
      const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = "dct3_eeprom_" + ts + ".bin";
      document.body.appendChild(a); a.click(); a.remove();
      setTimeout(() => URL.revokeObjectURL(a.href), 1000);
      console.log("[DCT3] EEPROM downloaded:", copy.length, "B");
      eeFileBtn.textContent = "Saved ✓";
      setTimeout(() => (eeFileBtn.textContent = "EEPROM → file"), 1200);
    });

    // --- Merge EEPROM → firmware .fls: bake the live NVRAM partition back into the
    // loaded firmware image and download it as a new .fls. Workflow: factory-reset (or
    // change the language) in the phone UI, optionally hold Power to flush, then click
    // this to write the clean EEPROM into the flash. The NVRAM partition sits at the TOP
    // of the image (file offset = length − partition size) in the 2 MB DCT3 layout.
    const mergeBtn = document.getElementById("btn-merge-fls");
    if (mergeBtn) mergeBtn.addEventListener("click", () => {
      try {
        const fw = new Uint8Array(mod.FS.readFile("/fw.fls"));  // current image (code + old EEPROM)
        const part = new Uint8Array(eepromSlice());             // live committed NVRAM
        const off = fw.length - part.length;                    // partition at top of image
        if (off < 0) { mergeBtn.textContent = "size mismatch"; return; }
        fw.set(part, off);                                      // splice the live EEPROM in
        const blob = new Blob([fw], { type: "application/octet-stream" });
        const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
        const a = document.createElement("a");
        a.href = URL.createObjectURL(blob);
        a.download = (lastFwBase || "dct3") + "_eeprom-merged_" + ts + ".fls";
        document.body.appendChild(a); a.click(); a.remove();
        setTimeout(() => URL.revokeObjectURL(a.href), 1000);
        console.log("[DCT3] merged EEPROM into firmware (" + (fw.length / 1024 / 1024).toFixed(2) +
                    " MB), partition spliced at 0x" + off.toString(16) + " — downloaded .fls");
        mergeBtn.textContent = "Merged ✓";
        setTimeout(() => (mergeBtn.textContent = "Merge → .fls"), 1400);
      } catch (e) { console.error("[DCT3] merge failed:", e); mergeBtn.textContent = "merge failed";
        setTimeout(() => (mergeBtn.textContent = "Merge → .fls"), 1400); }
    });

    // --- Crash dump: on a halt, download the FULL flat memory (RAM + flash + NVRAM) for
    // offline analysis / harness replay, plus a .txt of the diag + replay. Reboot clears it.
    const MEM_SPAN = 0x400000;                 // full DCT3 flat address space (0..4 MB)
    const dlBlob = (blob, name) => { const a = document.createElement("a"); a.href = URL.createObjectURL(blob);
      a.download = name; document.body.appendChild(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(a.href), 1000); };
    const crashDumpBtn = document.getElementById("btn-crash-dump");
    if (crashDumpBtn) crashDumpBtn.addEventListener("click", () => {
      const base = C.ramPtr();
      const mem = new Uint8Array(mod.HEAPU8.subarray(base, base + MEM_SPAN));   // detach-safe copy
      const ts = new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
      const stem = (lastFwBase || "dct3") + "_crashdump_" + ts;
      dlBlob(new Blob([mem], { type: "application/octet-stream" }), stem + ".bin");
      const txt = document.getElementById("crash-text");
      dlBlob(new Blob([txt ? txt.value : ""], { type: "text/plain" }), stem + ".txt");
      console.log("[DCT3] crash dump: " + (MEM_SPAN / 1048576) + " MB memory + diag .txt downloaded");
    });
    const crashResumeBtn = document.getElementById("btn-crash-resume");
    if (crashResumeBtn) crashResumeBtn.addEventListener("click", () => { try { boot(); } catch (e) {} });
    ["btn-copy-replay", "btn-copy-replay-2"].forEach((id) => {
      const b = document.getElementById(id);
      if (b) b.addEventListener("click", () => { window.dct3CopyReplay(); });
    });
    ["btn-paste-replay", "btn-paste-replay-2"].forEach((id) => {
      const b = document.getElementById(id);
      if (b) b.addEventListener("click", () => { window.dct3PasteReplay(); });
    });
    // Persist NVRAM checkbox — sync to the runtime flag. Sync once at startup so
    // the (default unchecked) HTML state wins over the let-init even if a future
    // edit flips one without the other.
    const chkPersist = document.getElementById("chk-persist-eeprom");
    if (chkPersist) {
      eepromPersist = chkPersist.checked;
      chkPersist.addEventListener("change", (e) => {
        eepromPersist = e.target.checked;
        console.log("[DCT3] NVRAM persistence " + (eepromPersist ? "ON" : "OFF (manual Save still works)"));
      });
    }

    // --- Halt CPU: freeze the run loop on demand and emit the SAME report the spin
    // detector drops (PC/registers + stack backtrace + keypress transcript). For crashes
    // the auto spin-detector never catches (a wrong-but-not-pinned PC, a runaway timer, a
    // reset-spin), this captures where the CPU is + how you got there, so it can be pasted
    // back / replayed via `node tools/nav.mjs <fw> --replay '<json>'`. Reboot clears it.
    const haltBtn = document.getElementById("btn-halt-cpu");
    if (haltBtn) haltBtn.addEventListener("click", () => {
      if (halted) { if (status) status.textContent = "CPU is already powered off"; return; }
      const diag = deepDiag();
      const step = C.step(), cyc = C.cycles();
      const replayJson = window.dct3DumpReplay();
      window.dct3SpinInfo = { pc: diag.pc, step: Math.round(step), cyc: Math.round(cyc),
                             keyLog: keyLog.slice(), diag, reason: "manual-halt" };
      crashed = true;                       // park the frame loop (honoured in frame())
      const panel = document.getElementById("crash-panel"), ta = document.getElementById("crash-text");
      if (ta) ta.value =
        `MANUAL HALT at ${diag.pc} (${diag.mode})  step=${Math.round(step)}  (${(step / 1e6).toFixed(1)}M insns, ${(cyc / 1e6).toFixed(1)}M cyc)\n\n` +
        `${cfgSummary()}\n${diag.text}\n\nreplay (nav.mjs --replay):\n${replayJson}\n`;
      if (panel) panel.style.display = "block";
      if (status) status.textContent = `⏸ CPU halted at ${diag.pc} (step ${(step / 1e6).toFixed(1)}M) — crash report below`;
      console.log(`[DCT3] manual halt @ ${diag.pc} (${diag.mode}) step=${Math.round(step)}\n${diag.text}`);
    });

    // FIASCO/.pmm unwrap: a virgin EEPROM dump (e.g. "Nokia 3310 virgin EEPROM.pmm")
    // is a chain of blocks — [0x0b, addr:3 BE, flag:1, len:2 LE, pad:1, cksum:1] then
    // <len> data bytes — covering the NVRAM partition. Reassemble each block at
    // (addr - PART_BASE) into a 0xFF-filled partition image. The partition window comes
    // from the active profile (2 MB images: 0x3D0000/192 KB; 4 MB images push it late and
    // larger). Returns null if it isn't a FIASCO stream. (Same wrapping the native
    // boot_trace handles for .390.)
    function unwrapPmm(buf) {
      const PART_BASE = C.eepromOff(), PART_SIZE = C.eepromSize();
      const out = new Uint8Array(PART_SIZE).fill(0xff);
      let off = 0, n = 0;
      while (off + 9 <= buf.length && buf[off] === 0x0b) {
        const addr = (buf[off + 1] << 16) | (buf[off + 2] << 8) | buf[off + 3];
        const len = buf[off + 5] | (buf[off + 6] << 8);
        const dst = addr - PART_BASE;
        if (dst >= 0 && dst < PART_SIZE) {
          const span = Math.min(len, PART_SIZE - dst);
          out.set(buf.subarray(off + 9, off + 9 + span), dst);
        }
        off += 9 + len; n++;
      }
      return n > 0 ? out : null;
    }

    // --- Load EEPROM: read a 192 KB raw .bin OR a FIASCO-wrapped .pmm (virgin/factory
    // dump) and reboot into it (overlay wins over saved NVRAM). No blank-guard: loading
    // a virgin image on purpose is allowed (use Save EEPROM to persist it).
    const eeFile = document.getElementById("eeprom-file");
    const eeName = document.getElementById("eeprom-file-name");
    if (eeFile) eeFile.addEventListener("change", async (e) => {
      const f = e.target.files && e.target.files[0];
      if (!f) return;
      let buf;
      try { buf = new Uint8Array(await f.arrayBuffer()); }
      catch (err) { console.error("[DCT3] EEPROM read failed:", err); if (eeName) eeName.textContent = "read failed"; return; }
      const want = C.eepromSize(), partBase = C.eepromOff();
      let part = null, kind = "";
      if (buf.length === want) { part = buf; kind = "raw"; }            // raw full-partition .bin
      else if (buf[0] === 0x0b) { part = unwrapPmm(buf); kind = "pmm"; } // FIASCO .pmm/.390
      else if (buf[0] === 0xf0 && buf[1] === 0xf0 && buf.length <= want) {
        // Virgin partition image in the native f0f0 block framing (e.g. "5210 virgin eeprom
        // 00580000.fls" — only the small active EEPROM block, not the full window). Splat it
        // into a want-sized 0xFF buffer at its NAMED base (the 8-hex token in the filename
        // that lands inside [partBase, partBase+want)); fall back to offset 0.
        let fileBase = partBase;
        for (const t of (f.name.match(/[0-9a-fA-F]{8}/g) || [])) {
          const v = parseInt(t, 16) >>> 0;
          if (v >= partBase && v < partBase + want) { fileBase = v; break; }
        }
        const dst = fileBase - partBase;
        if (dst >= 0 && dst + buf.length <= want) {
          const o = new Uint8Array(want).fill(0xff);
          o.set(buf, dst);
          part = o; kind = "virgin@0x" + fileBase.toString(16);
        }
      }
      if (!part) {
        if (eeName) eeName.textContent = "unrecognized (" + buf.length + " B; need " + (want / 1024).toFixed(0) + " KB raw, a .pmm, or an f0f0 virgin partition)";
        eeFile.value = "";
        return;
      }
      pendingEeprom = part;
      if (eeName) eeName.textContent = f.name + " · " + kind + " " + (part.length / 1024).toFixed(0) + " KB — rebooting";
      console.log("[DCT3] EEPROM file loaded:", f.name, "(" + kind + ", " + buf.length + " B src) — rebooting into it");
      try { boot(); } catch (err) { if (eeName) eeName.textContent = "boot failed — see console"; }
    });

    // --- KEYRELEASE test: invoke the firmware key-up handler (0x2EBD38) on demand.
    // The handler takes a keycode in r0; we use the field value, or derive the last
    // key's keycode from the keymap table (flash 0x32E718, row*5+col). Lets you hold
    // a key (menu rolls) then click to see if the release stops the scroll.
    const KEY_UP_HANDLER = 0x2ebd38;
    const krBtn = document.getElementById("btn-keyrelease");
    const krCode = document.getElementById("keyrelease-code");
    const krOut = document.getElementById("keyrelease-out");
    if (krBtn) krBtn.addEventListener("click", () => {
      let kc = parseInt(krCode.value, 16);
      if (isNaN(kc)) kc = C.ram(0x32e718 + lastKeyRC.row * 5 + lastKeyRC.col) & 0xff; // from keymap
      const before = C.callCount();
      C.call(KEY_UP_HANDLER, kc, 0);
      if (krOut) krOut.textContent = `up-handler(0x${kc.toString(16)}) queued`;
      // confirm it ran on the next frame
      setTimeout(() => { if (krOut) krOut.textContent = (C.callCount() > before ? "fired " : "pending ") + `up-handler(0x${kc.toString(16)})`; }, 60);
    });

    // --- Generic firmware call (debug): Call fn(r0, r1).
    const callBtn = document.getElementById("btn-call");
    if (callBtn) callBtn.addEventListener("click", () => {
      const fn = parseInt(document.getElementById("call-fn").value, 16);
      const r0 = parseInt(document.getElementById("call-r0").value, 16) || 0;
      const r1 = parseInt(document.getElementById("call-r1").value, 16) || 0;
      const out = document.getElementById("call-out");
      if (isNaN(fn)) { if (out) out.textContent = "bad fn"; return; }
      const before = C.callCount();
      C.call(fn, r0, r1);
      if (out) out.textContent = `0x${fn.toString(16)}(0x${r0.toString(16)},0x${r1.toString(16)}) queued`;
      setTimeout(() => { if (out) out.textContent = (C.callCount() > before ? "fired " : "pending ") + `0x${fn.toString(16)}`; }, 60);
    });

    // --- Type-string injector (MATRIX path — faithful + crash-safe). Each char is typed
    // as REAL keypresses through the keypad (the existing auto down/up queue), so the
    // firmware's editor runs its own multi-tap + commit exactly like pressing keys — no
    // editor desync, no heap corruption. (send_message(0xCA) char injection was abandoned:
    // KEY_DOWN/UP never pass through send_message, so a bare char has no down/up to balance
    // and corrupts the editor.) Digits / * / # are one key each (perfect in a number/PIN
    // field); letters are multi-tap on the ITU key (2=abc … 9=wxyz, 0=space) for text fields.
    // --- Type-string injector (REG-SPIKE path). Per char: latch a one-shot reg spike at
    // the firmware's char-post call site (overwrites r1 = the posted char), then send ONE
    // real keypress. The firmware computes its own char, we replace it in-register, and it
    // goes out through the firmware's genuine balanced down/up/commit flow — table-agnostic,
    // any letter in one press, no multi-tap, no message injection, no editor desync.
    // Char-post PC is per-build (captured: 3410 char post @0x39F9F2, LR 0x39F9F7).
    const SPIKE_PC_BY_MODEL = { "3410": 0x39F9F2 };
    let spikePc = 0x39F9F2;
    window.__typeSpikePc = (model) => { if (SPIKE_PC_BY_MODEL[model]) spikePc = SPIKE_PC_BY_MODEL[model]; };
    let typeQ = [], typeBusy = false, typeSpikeMark = 0, typeWait = 0, typeKeyAlt = 0;
    function pumpType() {           // called once per steady-state frame
      if (typeBusy) {
        if (C.regspikeCount() > typeSpikeMark) typeBusy = false;   // spike fired → char posted, next
        else if (++typeWait > 120) typeBusy = false;               // watchdog (not on a text field)
        return;
      }
      if (!typeQ.length) return;
      const ch = typeQ.shift();
      typeSpikeMark = C.regspikeCount();
      C.regspike(spikePc, 1, ch & 0xff);                           // latch: char-post r1 → ch
      // ALTERNATE between two number keys so the editor sees a "different key" each char and
      // auto-commits the previous one (advancing the cursor) without waiting for the multi-tap
      // timeout — fast typing. The key is just a commit trigger; the spike sets the actual char.
      C.keyLogical(LABEL_KK[(typeKeyAlt ^= 1) ? "2" : "3"], 1);
      typeBusy = true; typeWait = 0;
      const out = document.getElementById("type-out");
      if (out) out.textContent = typeQ.length ? `typing… ${typeQ.length} left` : "done";
    }
    const typeBtn = document.getElementById("btn-type");
    if (typeBtn) typeBtn.addEventListener("click", () => {
      const el = document.getElementById("type-text");
      typeQ = Array.from((el && el.value) || "").map((c) => c.charCodeAt(0) & 0xff);
      const out = document.getElementById("type-out");
      if (out) out.textContent = `queued ${typeQ.length}`;
    });
    // "Skip security code": neutralise the FuBu v6.39 disp77 FAID lock (checksum
    // completeness —). Takes effect on next reboot.
    const chkSeclock = document.getElementById("chk-skip-seclock");
    if (chkSeclock) chkSeclock.addEventListener("change", (e) => C.setSkipSeclock(e.target.checked ? 1 : 0));
    // Auto-recover crash master gate (mad2 reset-reason recovery; see mad2.c case 0x01).
    // Takes effect immediately — no reboot needed. The next firmware self-reset will
    // either recover-in-place (on) or warm-reboot (off).
    const chkRecover = document.getElementById("chk-recover");
    if (chkRecover && C.setRecover) chkRecover.addEventListener("change", (e) => {
      C.setRecover(e.target.checked ? 1 : 0);
      console.log("[DCT3] auto-recover " + (e.target.checked ? "ON" : "OFF (resets will warm-reboot)"));
    });
    // Eager panic-chain intercept (RESET_EARLY equivalent). Takes effect immediately; the
    // next firmware self-reset is caught at the reboot-fn / fatal-handler ENTRY rather than
    // at the trailing [0x20001]|=4 write. Off = byte-identical to the existing late catch.
    const chkRebootEarly = document.getElementById("chk-reboot-early");
    if (chkRebootEarly && C.setRebootEarly) chkRebootEarly.addEventListener("change", (e) => {
      C.setRebootEarly(e.target.checked ? 1 : 0);
      console.log("[DCT3] reboot early-intercept " + (e.target.checked ? "ON (catch at panic entry)" : "OFF (late catch)"));
    });
    const chkOneshot = document.getElementById("chk-oneshot");
    if (chkOneshot) chkOneshot.addEventListener("change", (e) => C.setOneshot(e.target.checked ? 1 : 0));

    // Live timing sliders. Key hold = how long (in emulated instructions) a tap
    // is held down before auto-release; lower = fewer repeats on animated menus.
    // Pace = instructions run per animation frame (overall emulated speed).
    function wireSlider(id, outId, onChange, fmt) {
      const el = document.getElementById(id), out = document.getElementById(outId);
      if (!el) return;
      const apply = () => { const v = +el.value; out.textContent = fmt ? fmt(v) : v; onChange(v); };
      el.addEventListener("input", apply);
      apply();
    }
    const kInsns = (v) => v >= 1e6 ? (v / 1e6).toFixed(2) + "M insns" : (v / 1000).toFixed(0) + "k insns";
    wireSlider("sl-keyhold", "out-keyhold", (v) => C.setKeyHold(v), kInsns);
    // Speed = real-time multiplier. 100 (%) == 13 MHz (1x); scales cycles/frame.
    wireSlider("sl-speed", "out-speed",
      (v) => { speedMul = v / 100; },
      (v) => (v / 100).toFixed(2) + "× (~" + (TARGET_HZ * v / 100 / 1e6).toFixed(1) + " MHz)");

    // UI-message logger: capture r0 at the send/route core (default) or any PC.
    const msglogAddr = document.getElementById("msglog-addr");
    function applyMsglog() {
      const on = msglogChk && msglogChk.checked;
      const pc = msglogAddr ? (parseInt(msglogAddr.value, 16) >>> 0) : 0x2e84b6;
      C.msglogPc(on ? (pc || 0x2e84b6) : 0);     // re-arm resets the wasm counter
      msglogReset();
    }
    if (msglogChk) msglogChk.addEventListener("change", applyMsglog);
    if (msglogAddr) msglogAddr.addEventListener("change", applyMsglog);
    document.querySelectorAll(".msglog-presets button").forEach((b) => {
      b.addEventListener("click", () => {
        msglogAddr.value = b.dataset.pc;
        if (msglogChk) msglogChk.checked = true;
        applyMsglog();
      });
    });
    const msglogClear = document.getElementById("btn-msglog-clear");
    if (msglogClear) msglogClear.addEventListener("click", msglogReset);
    const msglogCopy = document.getElementById("btn-msglog-copy");
    if (msglogCopy) msglogCopy.addEventListener("click", () => {
      const t = msglogOut.textContent || "";
      const done = () => { msglogCopy.textContent = "Copied"; setTimeout(() => (msglogCopy.textContent = "Copy"), 900); };
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(t).then(done, () => fallbackCopy(t, done));
      } else { fallbackCopy(t, done); }
    });
    function fallbackCopy(t, done) {
      const ta = document.createElement("textarea");
      ta.value = t; ta.style.position = "fixed"; ta.style.opacity = "0";
      document.body.appendChild(ta); ta.select();
      try { document.execCommand("copy"); done(); } catch (e) { /* ignore */ }
      document.body.removeChild(ta);
    }

    // --- Unified PCM audio -> Web Audio, vibra, battery/charger, power button ----
    // Browsers block audio until a user gesture, so the AudioContext is created only
    // when the Sound box is ticked. NEITHER voice is a Web Audio oscillator any more:
    // the buzzer AND the DSP tone (keypad beep / DTMF) both arrive as one PCM stream
    // (emu_audio.c, on the SAME channel as the DSP codec), drained here through one
    // ScriptProcessor node — the identical unified PCM path the SDL GUI plays.
    let audioCtx = null, audioOn = false;
    let pcmNode = null;                         // ScriptProcessor playing the unified PCM stream
    let lastTone = -1;                          // level-change detect for the [tone] log
    let lastBuzz = -1;                          // level-change detect for the [buzz] log
    const BUZZ_CLOCK = 13e6;                    // buzzer freq = 13 MHz / divider (for the log only)
    const buzzOut = document.getElementById("buzz-out");
    function buzzHz(div) {                       // log-only pitch estimate (audio itself is PCM now)
      if (!div) return 0;
      let f = BUZZ_CLOCK / div;
      while (f > 20000) f /= 2;
      while (f > 0 && f < 20) f *= 2;
      return f;
    }

    // PCM queue at the producer (codec) rate; the node resamples it to audioCtx.sampleRate.
    const PCMQ = 1 << 15, PCMQ_MASK = PCMQ - 1;
    const PCM_PRIME = 2048;                      // ring cushion (~43 ms @48k) before playback starts, so the
    const pcmq = new Float32Array(PCMQ);        //   per-frame burst production can't underrun into clicks
    let pcmqHead = 0, pcmqTail = 0, pcmFrac = 0; // ring indices + fractional resample position
    let pcmPrimed = false, pcmLast = 0;          // priming gate + last sample (click-free underrun fade)
    let pcmRate = 18642;                         // producer Hz (refreshed from C.pcmRate on enable)

    // Pull all pending PCM from the wasm ring into the JS queue (called each frame). The
    // producer streams CONSTANTLY (silence included), so when audio is off we still drain
    // the wasm ring — discarding — or enabling sound later would play ~1 s of stale ring.
    function pumpPCM() {
      if (!audioOn || !audioCtx) {
        while (C.pcmRead(2048) === 2048) { /* discard */ }
        return;
      }
      const r = C.pcmRate();                    // track the producer rate (HLE 48k; codec takeover changes it)
      if (r > 0) pcmRate = r;
      const base = C.pcmPtr() >> 1;             // int16 index
      for (;;) {
        const n = C.pcmRead(2048);
        if (n <= 0) break;
        const view = mod.HEAP16.subarray(base, base + n);
        for (let i = 0; i < n; i++) {
          if (((pcmqHead + 1) & PCMQ_MASK) === (pcmqTail & PCMQ_MASK)) break; // queue full -> drop
          pcmq[pcmqHead & PCMQ_MASK] = view[i] / 32768;
          pcmqHead++;
        }
        if (n < 2048) break;
      }
    }
    function enableAudio() {
      audioOn = true;
      if (audioCtx) { if (audioCtx.resume) audioCtx.resume(); return; }
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) { console.warn("[DCT3] no Web Audio in this browser"); return; }
      audioCtx = new AC();
      pcmRate = C.pcmRate() || 18642;
      pcmqHead = pcmqTail = 0; pcmFrac = 0; pcmPrimed = false; pcmLast = 0;
      // One node plays the unified earpiece PCM (buzzer + DSP tone/codec), resampled on the fly.
      pcmNode = audioCtx.createScriptProcessor(1024, 0, 1);
      pcmNode.onaudioprocess = (e) => {
        const out = e.outputBuffer.getChannelData(0);
        const ratio = pcmRate / audioCtx.sampleRate;   // input samples per output sample (~0.388)
        // Hold silent until a cushion is buffered, so the per-frame burst production can't
        // starve the node into clicks; a realtime source needs no time-stretch, just latency.
        if (!pcmPrimed) {
          if (((pcmqHead - pcmqTail) & PCMQ_MASK) >= PCM_PRIME) pcmPrimed = true;
          else { out.fill(0); return; }
        }
        for (let i = 0; i < out.length; i++) {
          if (((pcmqHead - pcmqTail) & PCMQ_MASK) < 2) {          // underrun -> fade to silence + re-prime
            pcmLast *= 0.985; out[i] = pcmLast;
            if (Math.abs(pcmLast) < 1e-4) { pcmLast = 0; pcmPrimed = false; }
            continue;
          }
          const a = pcmq[pcmqTail & PCMQ_MASK], b = pcmq[(pcmqTail + 1) & PCMQ_MASK];
          out[i] = pcmLast = a + (b - a) * pcmFrac;
          pcmFrac += ratio;
          while (pcmFrac >= 1 && ((pcmqHead - pcmqTail) & PCMQ_MASK) >= 2) { pcmFrac -= 1; pcmqTail++; }
        }
      };
      pcmNode.connect(audioCtx.destination);
      // The DSP tone (keypad beep / DTMF) is no longer a Web Audio oscillator — it is
      // synthesized into the same unified PCM stream by emu_audio.c and played by pcmNode.
    }
    function disableAudio() {
      audioOn = false;
      pcmqHead = pcmqTail = 0; pcmPrimed = false; pcmLast = 0;   // flush queued PCM; re-enable starts clean
    }

    function updatePeripherals() {
      const on = C.buzzerOn(), div = C.buzzerDiv(), vol = C.buzzerVol();
      const chirp = C.buzzerChirp();                    // rising edges (lo byte) + div-at-edge; clears
      const edges = chirp & 0xff, chirpDiv = (chirp >>> 8) & 0xffff;
      // Log a sustained level change OR a sub-frame chirp the level poll wouldn't see.
      if (on !== lastBuzz || (!on && edges > 0)) {
        if (on || edges > 0) {
          const d = on ? div : chirpDiv;
          console.log(`[buzz] ${on ? "ON" : "chirp"} div=${d} (0x${d.toString(16)}) vol=${vol} -> ${buzzHz(d).toFixed(0)} Hz` + (edges > 1 ? ` x${edges}` : ""));
        } else if (lastBuzz === 1) console.log("[buzz] off");
        lastBuzz = on;
      }
      // The buzzer voice is now PCM (emu_audio.c, onset stamped at the register write):
      // just drain the wasm ring into the play queue. No per-frame oscillator scheduling,
      // no chirp-blip special case — a sub-frame chirp is already in the sample stream.
      pumpPCM();
      // DSP tone (keypad beep / DTMF) is ALSO in the unified PCM stream now (emu_audio.c
      // HLE-synthesizes the COBBA tone registers) — drained by the same pumpPCM() above.
      // These reads are just for the [tone] log + the on-screen readout, not audio.
      const toneHz = C.toneHz ? C.toneHz() : 0;
      const toneHz2 = C.toneHz2 ? C.toneHz2() : 0;
      if (toneHz !== lastTone) {
        if (toneHz > 0) console.log(`[tone] ${toneHz} Hz${toneHz2 ? ` + ${toneHz2} Hz (DTMF)` : ""} (amp 0x${C.toneAmp().toString(16)})`);
        lastTone = toneHz;
      }
      if (buzzOut) buzzOut.textContent =
        toneHz > 0 ? `♪ ${toneHz}${toneHz2 ? "+" + toneHz2 : ""}Hz`
        : on ? `♪ div=${div} ~${buzzHz(div).toFixed(0)}Hz` : (edges > 0 ? `♪ chirp ~${buzzHz(chirpDiv).toFixed(0)}Hz` : "");
      document.body.classList.toggle("vibrating", !!C.vibraOn());
    }

    // vibra shake (injected style, like the backlight style)
    const vibStyle = document.createElement("style");
    vibStyle.textContent =
      "@keyframes dct3shake{0%,100%{transform:translateX(0)}25%{transform:translateX(-2px)}75%{transform:translateX(2px)}}" +
      // shake the classic grid (.phone) AND the device shell (.shell-host wrapper — NOT
      // .shell-root, whose transform:scale would be clobbered by the animation's transform)
      ".vibrating .phone,.vibrating .shell-host{animation:dct3shake .08s linear infinite}";
    document.head.appendChild(vibStyle);

    const chkAudio = document.getElementById("chk-audio");
    if (chkAudio) chkAudio.addEventListener("change", (e) => e.target.checked ? enableAudio() : disableAudio());

    // Power button: tap = momentary press, hold ≈ power off. The firmware decides
    // tap (profiles) vs hold (power off) by duration; a ~150 ms min-hold guarantees a
    // quick click is still caught by the keypad scan. A hold past POWEROFF_HOLD_MS is
    // treated as a deliberate power-off: we let the firmware run its shutdown (animation
    // + NVRAM flush), then the frame loop parks the CPU (see "power-off" there). When the
    // phone is OFF (halted), a tap powers it back on (reboot).
    const POWEROFF_HOLD_MS = 2500;
    // Power tap/hold handlers — shared by the config ⏻ button and the shell's Power
    // hit-zone. Tap = momentary power key; hold ≈ 2.5s = power off; tap while off = boot.
    let pwrDownAt = 0, pwrRelTimer = 0;
    function powerDown(e) {
      if (e) e.preventDefault();
      if (halted) { try { boot(); } catch (err) {} return; }   // off → power on
      if (pwrRelTimer) { clearTimeout(pwrRelTimer); pwrRelTimer = 0; }
      pwrDownAt = performance.now(); C.power(1);
    }
    function powerUp() {
      if (!pwrDownAt) return;
      const held = performance.now() - pwrDownAt; pwrDownAt = 0;
      const rel = () => { pwrRelTimer = 0; C.power(0); };
      logKey('power', held >= POWEROFF_HOLD_MS ? 'off' : 'pwr');   // replay log
      if (held >= POWEROFF_HOLD_MS) {        // deliberate power-off hold
        if (!poweringOff) poweringOff = performance.now();
        rel();                               // release the key; firmware finishes shutdown
      } else if (held < 150) {
        pwrRelTimer = setTimeout(rel, 150 - held);
      } else rel();
    }
    const pwrBtn = document.getElementById("btn-power");
    if (pwrBtn) {
      pwrBtn.addEventListener("mousedown", powerDown);
      pwrBtn.addEventListener("touchstart", powerDown, { passive: false });
      pwrBtn.addEventListener("mouseup", powerUp);
      pwrBtn.addEventListener("mouseleave", powerUp);
      pwrBtn.addEventListener("touchend", powerUp);
    }

    // Battery slider + charger toggle (CCONT A/D ch2 / ch5).
    const CHARGER_PRESENT = 0x2c0;       // a "charger connected" Vchar reading (calibratable)
    wireSlider("sl-battery", "out-battery",
      (v) => C.setBattery(v),
      (v) => "0x" + v.toString(16).toUpperCase());
    const chkCharger = document.getElementById("chk-charger");
    const outCharger = document.getElementById("out-charger");
    if (chkCharger) chkCharger.addEventListener("change", (e) => {
      C.setCharger(e.target.checked ? CHARGER_PRESENT : 0);
      if (outCharger) outCharger.textContent = e.target.checked ? "(0x" + CHARGER_PRESENT.toString(16) + ")" : "";
    });

    // SIM card present + CHV1 (PIN) controls. The model defaults to a present SIM with
    // CHV1 enabled (PIN 1234). out-sim shows the live APDU count; out-pin shows the
    // CHV1 runtime state (enabled / verified / tries left), polled each second.
    const chkSim = document.getElementById("chk-sim");
    const outSim = document.getElementById("out-sim");
    const chkPin = document.getElementById("chk-pin");
    const inPin  = document.getElementById("in-pin");
    const outPin = document.getElementById("out-pin");
    if (chkSim) chkSim.addEventListener("change", (e) => C.setSim(e.target.checked ? 1 : 0));
    if (chkPin) chkPin.addEventListener("change", (e) => C.setSimPinEnabled(e.target.checked ? 1 : 0));
    if (inPin)  inPin.addEventListener("change", (e) => {
      const v = parseInt(e.target.value, 10); if (!isNaN(v)) C.setSimPin(v);
    });
    setInterval(() => {
      if (outSim) outSim.textContent = "(" + C.simApdus() + " APDUs)";
      if (outPin) {
        const s = C.simPinState();
        outPin.textContent = "[" + ((s & 1) ? "on" : "off") + ((s & 2) ? ", verified" : "")
          + ", " + ((s >> 8) & 0xF) + " tries]";
      }
    }, 1000);

    // PPM string tracer console helpers: dct3Trace() to start, reproduce a screen, then
    // dct3Dump() to print every on-screen string with its PPM ID and the firmware caller.
    window.dct3Trace = (on = 1) => { C.getstrOn(on ? 1 : 0);
      return on ? "tracing get_string — reproduce the screen, then dct3Dump()" : "stopped"; };
    window.dct3Dump = () => { const s = C.getstrDump(); console.log(s || "(no strings captured)");
      return (s ? s.split("\n").length - 1 : 0) + " strings"; };

    // Restore a previously picked firmware (survives page reload) before the first boot,
    // so its image + per-image NVRAM come back. Falls back to the built-in image on any
    // failure. Use dct3ClearFirmware() to revert to built-in.
    try {
      const rec = await idbGetFw();
      if (rec && rec.bytes) {
        mod.FS.writeFile("/fw.fls", rec.bytes);
        customFw = true; lastFwBase = (rec.name || "fw").replace(/\.[^.]+$/, "");
        if (fwName) fwName.textContent = rec.name + " · restored";
        console.log("[DCT3] restored picked firmware from IndexedDB:", rec.name);
      }
    } catch (e) { console.warn("[DCT3] firmware restore failed:", e); }

    boot();
    requestAnimationFrame(frame);
  }).catch((err) => {
    console.error("[DCT3] load failed:", err);
    status.textContent = "WASM failed to load (see console)";
  });
})();
