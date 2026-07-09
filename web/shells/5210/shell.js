// Nokia 5210 (NSM-5 / MAD2, 84×48, family-A keypad) — self-contained shell unit.
// Rugged candybar: white glyphs on dark keys (the mask lights them warm-white).
// Family-A nav cluster maps cleanly — LEFT soft (Menu/Select) = soft1, RIGHT soft
// (Names) = soft2, plus up/down, green Send / red End, side volume keys, and a
// dedicated top Power button. Photo + mask are 380×806 (1:1 design space). The
// 5210's screen glass is AMBER, so the LCD renders in an amber palette
// (lcdColor/pixelColor) — the panel + canvas then blend seamlessly into the photo
// screen, exactly like the design (vs the default greenish DCT3 LCD).
registerShell("5210", {
  img:  "shells/5210/nokia-5210.png",
  mask: "shells/5210/key-glyphs-mask.png",
  w: 380, h: 806,
  lcd:  { left: 91, top: 196, w: 197, h: 151, radius: 5, canvasW: 196, canvasH: 112 },
  lcdColor: "#c9ba82",      // dim/muted amber — LCD backlight OFF (unlit STN)
  lcdColorLit: "#f8e0a0",   // bright amber — backlight ON (the design/photo screen colour)
  pixelColor: "#2a2410",    // dark-amber lit pixels
  light: "#ffe9b0",         // warm-white glow through the white glyphs
  zones: [
    { key: "pwr",     left: 245, top: 8,   w: 54, h: 29, r: "8px 4px 4px 8px" },  // top power button
    { key: "volup",   left: 13,  top: 85,  w: 25, h: 74, r: "8px 4px 4px 8px" },  // left side
    { key: "voldown", left: 10,  top: 165, w: 24, h: 87, r: "8px 4px 4px 8px" },
    { key: "soft1",   left: 78,  top: 426, w: 71, h: 61, r: "50%" },              // left soft = Menu/Select
    { key: "soft2",   left: 231, top: 426, w: 71, h: 61, r: "50%" },              // right soft = Names
    { key: "up",      left: 162, top: 453, w: 56, h: 39, r: "12px 12px 4px 4px" },
    { key: "down",    left: 156, top: 502, w: 66, h: 39, r: "4px 4px 12px 12px" },
    { key: "send",    left: 80,  top: 497, w: 73, h: 33, r: "34px 14px 20px 22px" }, // green call
    { key: "end",     left: 224, top: 497, w: 81, h: 38, r: "14px 34px 22px 20px" }, // red end
    { key: "1", left: 73,  top: 532, w: 75, h: 57, r: "16px" },
    { key: "2", left: 151, top: 544, w: 83, h: 54, r: "16px" },
    { key: "3", left: 236, top: 537, w: 76, h: 52, r: "16px" },
    { key: "4", left: 74,  top: 592, w: 75, h: 38, r: "16px" },
    { key: "5", left: 157, top: 602, w: 67, h: 37, r: "16px" },
    { key: "6", left: 236, top: 592, w: 73, h: 40, r: "16px" },
    { key: "7", left: 91,  top: 636, w: 68, h: 45, r: "16px" },
    { key: "8", left: 159, top: 646, w: 66, h: 40, r: "16px" },
    { key: "9", left: 230, top: 640, w: 59, h: 41, r: "16px" },
    { key: "*", left: 95,  top: 690, w: 59, h: 34, r: "16px" },
    { key: "0", left: 161, top: 692, w: 61, h: 38, r: "16px" },
    { key: "#", left: 226, top: 690, w: 52, h: 34, r: "16px" }
  ]
});
