// Nokia 5110 (NSE-1 / MAD2, 84×48) — self-contained shell unit.
// Bar: white glyphs on gray keys.
// Photo + mask are 350x1100 (1:1 design space).
registerShell("5110", {
  img:  "shells/5110/nokia-5110.png",
  w: 350, h: 1100,
  lcd:  { left: 57, top: 410, w: 236, h: 162, radius: 10, canvasW: 210, canvasH: 144},
  zones: [
    { key: "pwr",     left: 254, top: 212, w: 40, h: 40, r: "10px 10px 10px 10px" },  // top power button
    { key: "soft1",   left: 128,  top: 614, w: 94, h: 64, r: "50%" },              // left soft = Menu/Select
    { key: "soft2",   left: 54, top: 612, w: 62, h: 50, r: "50%" },              // right soft = Names
    { key: "up",      left: 248, top: 614, w: 56, h: 44, r: "12px 12px 4px 4px" },
    { key: "down",    left: 238, top: 678, w: 50, h: 50, r: "4px 4px 12px 12px" },
    { key: "1", left: 40,  top: 746, w: 74, h: 46, r: "16px" },
    { key: "2", left: 138, top: 746, w: 74, h: 46, r: "16px" },
    { key: "3", left: 234, top: 746, w: 74, h: 46, r: "16px" },
    { key: "4", left: 40,  top: 810, w: 74, h: 46, r: "16px" },
    { key: "5", left: 138, top: 810, w: 74, h: 46, r: "16px" },
    { key: "6", left: 234, top: 810, w: 74, h: 46, r: "16px" },
    { key: "7", left: 40,  top: 874, w: 74, h: 46, r: "16px" },
    { key: "8", left: 138, top: 874, w: 74, h: 46, r: "16px" },
    { key: "9", left: 234, top: 874, w: 74, h: 46, r: "16px" },
    { key: "*", left: 48,  top: 946, w: 58, h: 46, r: "16px" },
    { key: "0", left: 138, top: 938, w: 74, h: 50, r: "16px" },
    { key: "#", left: 244, top: 946, w: 58, h: 46, r: "16px" }
  ]
});
