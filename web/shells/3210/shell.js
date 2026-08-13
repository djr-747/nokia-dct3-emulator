// Nokia 3210 (NSE-8 / MAD2, 84×48) — self-contained shell unit.
// Bar: green glyphs on white keys.
// Photo + mask are 450x1050 (1:1 design space).
registerShell("3210", {
  img:  "shells/3210/nokia-3210.png",
  w: 450, h: 1050,
  lcd:  { left: 80, top: 258, w: 292, h: 208, radius: 10, canvasW: 252, canvasH: 172.8},
  zones: [
    { key: "pwr",     left: 150, top: 5, w: 150, h: 30, r: "0px 0px 10px 10px" },  // top power button
    { key: "soft1",   left: 160,  top: 520, w: 128, h: 55, r: "50%" },              // left soft = Menu/Select
    { key: "soft2",   left: 62, top: 522, w: 78, h: 66, r: "50%" },              // right soft = Names
    { key: "up",      left: 331, top: 524, w: 56, h: 57, r: "12px 12px 4px 4px" },
    { key: "down",    left: 275, top: 581, w: 56, h: 57, r: "4px 4px 12px 12px" },
    { key: "1", left: 70,  top: 660, w: 86, h: 54, r: "16px" },
    { key: "2", left: 176, top: 670, w: 94, h: 50, r: "16px" },
    { key: "3", left: 288, top: 660, w: 86, h: 54, r: "16px" },
    { key: "4", left: 70,  top: 736, w: 86, h: 54, r: "16px" },
    { key: "5", left: 176, top: 746, w: 94, h: 50, r: "16px" },
    { key: "6", left: 288, top: 736, w: 86, h: 54, r: "16px" },
    { key: "7", left: 70,  top: 810, w: 86, h: 54, r: "16px" },
    { key: "8", left: 176, top: 820, w: 94, h: 50, r: "16px" },
    { key: "9", left: 288, top: 810, w: 86, h: 54, r: "16px" },
    { key: "*", left: 70,  top: 886, w: 86, h: 54, r: "16px" },
    { key: "0", left: 176, top: 896, w: 94, h: 50, r: "16px" },
    { key: "#", left: 288, top: 886, w: 86, h: 54, r: "16px" }
  ]
});
