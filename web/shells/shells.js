// Per-model phone "shells" for the web emulator — SELF-CONTAINED UNITS.
//
// Each shell is a folder under  web/shells/<model>/  holding everything it needs:
//   nokia-<model>.png      body cut-out photo (transparent surround)
//   key-glyphs-mask.png    keypad-backlight glyph mask (optional)
//   shell.js               registers the config via registerShell("<model>", {...})
//
// This file is just the LOADER + manifest. It defines the registry + helper, then
// pulls each unit in (parser-blocking, so window.DCT3_SHELLS is fully populated
// before main.js runs). main.js picks one by matching the folder name against the
// detected model (exact, then substring) in pickShell().
//
// === Adding / tuning a model ===
//   1. Drop a folder under web/shells/<model>/ with its photo (+ optional mask).
//   2. Add a shell.js there that calls registerShell("<model>", { ... }).
//   3. Add "<model>" to DCT3_SHELL_MANIFEST below.
//   4. Calibrate in-page: View = device shell, then "⚙ Edit zones" — drag/resize the
//      hit-zones (and the LCD rect) onto the real buttons, "Export shell.js", and save
//      the result over web/shells/<model>/shell.js. No re-measuring by hand.
//
// Coordinate space: the body box (the photo scaled to `w`×`h` design px). Every
// lcd/zone rect is in that space; the shell layer scales the whole box to fit the
// column. Keys use the emulator's LOGICAL labels (KK in main.js): up, down, soft1,
// soft2, send, end, 0-9, *, #, pwr, volup, voldown. `pwr` is routed through the real
// power path (tap = momentary, hold ≈ power off); every other key through the normal
// keypad matrix — so shell input is identical to the grid keypad/keyboard.
window.DCT3_SHELLS = window.DCT3_SHELLS || {};
window.registerShell = function (name, def) { window.DCT3_SHELLS[name] = def; };

// Folder names, matched against the detected model. Order = substring-match priority.
window.DCT3_SHELL_MANIFEST = ["3310", "3410", "5210"];
// Path prefix to the web root. "" when loaded from /web/; pages nested deeper (e.g.
// /web/next/) set window.DCT3_SHELL_PREFIX = "../" before this script so the unit +
// asset URLs resolve. Consumers (main.js) prefix def.img/def.mask with the same value.
(function (v, p) {
  window.DCT3_SHELL_MANIFEST.forEach(function (m) {
    document.write('<scr' + 'ipt src="' + p + 'shells/' + m + '/shell.js?v=' + v + '"></scr' + 'ipt>');
  });
})(Date.now(), window.DCT3_SHELL_PREFIX || "");
