// Web boot regression test — asserts the 3310 reaches a LIVE boot (not a blank/stuck
// screen). Runs the actual web build (web/dct3.js + .wasm + .data) headlessly under
// node, CYCLE-PACED exactly like the browser (dct3_web_run_cycles), and checks the
// firmware actually drove the LCD and reached the display dispatcher.
//
// Why this exists: a stale/incremental wasm (e.g. a header change that didn't trigger
// a .o rebuild → struct-layout mismatch across translation units) boots to a blank
// screen even though the source is fine. The native make-test is step-paced and can't
// see this; this test is the cycle-paced oracle. Run: `make webtest` (or
// `node tools/web_boot_test.mjs [webdir]`).
//
// Pass criterion (a good live boot, from the known-good 629d5d9 baseline):
//   lcd_writes large (firmware is driving the PCD8544) AND the display dispatcher ran.
// Blank/stuck regression looks like lcd_writes≈2, disp==0.

import { createRequire } from 'module';
const require = createRequire(import.meta.url);
const webDir = process.argv[2] || new URL('../web', import.meta.url).pathname;
process.chdir(webDir);

const M = await (require(webDir + '/dct3.js'))();
const boot   = M.cwrap('dct3_web_boot', 'number', []);
const runCyc = M.cwrap('dct3_web_run_cycles', null, ['number']);
const ram    = M.cwrap('dct3_web_ram', 'number', ['number']);
const lcdw   = M.cwrap('dct3_web_lcd_writes', 'number', []);
const pcf    = M.cwrap('dct3_web_pc', 'number', []);

const rc = boot();
if (rc !== 0) { console.error(`FAIL: dct3_web_boot returned ${rc}`); process.exit(1); }

// ~260M cycles ≈ 20 s of phone time at 13 MHz, in browser-sized frames (216667 cyc).
for (let i = 0; i < 1200; i++) runCyc(216667);

const lcd  = lcdw();
const disp = ram(0x11FD08);     // 3310 active display dispatcher id (0 = none ran)
const pc   = pcf();
const LCD_MIN = 1000;           // good boot draws thousands of LCD bytes; blank ≈ 2

console.log(`web boot: lcd_writes=${lcd} disp=${disp} PC=0x${pc.toString(16)}`);
if (lcd < LCD_MIN || disp === 0) {
    console.error(`FAIL: web 3310 did not reach a live boot ` +
                  `(lcd_writes=${lcd} < ${LCD_MIN} or disp=${disp}==0). ` +
                  `Likely a stale/incremental wasm — run \`make clean && make\`.`);
    process.exit(1);
}
console.log('PASS: web 3310 reached a live boot.');
