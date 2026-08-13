// Nokia 8210 (NSM-3 / MAD2, 84×48) — self-contained shell unit.
// Bar: green glyphs on gray keys.
// Photo + mask are 500×1100 (1:1 design space).
registerShell("8210", {
  img:  "shells/8210/nokia-8210.png",
  w: 500, h: 1100,
  lcd:  { left: 97, top: 292, w: 306, h: 208, radius: 10, canvasW: 252, canvasH: 172.8 },
  zones: [
    { key: "pwr",     left: 150, top: 28,   w: 200, h: 30, r: "8px 4px 4px 8px" },  // top power button
    { key: "volup",   left: 19,  top: 142,  w: 14, h: 82, r: "8px 4px 4px 8px" },  // left side
    { key: "voldown", left: 13,  top: 242, w: 10, h: 78, r: "8px 4px 4px 8px" },
    { key: "soft1",   left: 108,  top: 532, w: 92, h: 68, r: "50%" },              // left soft = Menu/Select
    { key: "soft2",   left: 302, top: 532, w: 92, h: 68, r: "50%" },              // right soft = Names
    { key: "up",      left: 162, top: 554, w: 178, h: 54, r: "12px 12px 4px 4px" },
    { key: "down",    left: 188, top: 608, w: 126, h: 68, r: "4px 4px 12px 12px" },
    { key: "send",    left: 106,  top: 604, w: 94, h: 68, r: "34px 14px 20px 22px" }, // green call
    { key: "end",     left: 302, top: 604, w: 94, h: 68, r: "14px 34px 22px 20px" }, // red end
    { key: "1", left: 60,  top: 700, w: 106, h: 60, r: "16px" },
    { key: "2", left: 198, top: 716, w: 106, h: 48, r: "16px" },
    { key: "3", left: 334, top: 700, w: 106, h: 60, r: "16px" },
    { key: "4", left: 64,  top: 764, w: 106, h: 60, r: "16px" },
    { key: "5", left: 198, top: 780, w: 106, h: 48, r: "16px" },
    { key: "6", left: 330, top: 764, w: 106, h: 60, r: "16px" },
    { key: "7", left: 66,  top: 826, w: 106, h: 60, r: "16px" },
    { key: "8", left: 198, top: 842, w: 106, h: 48, r: "16px" },
    { key: "9", left: 326, top: 826, w: 106, h: 60, r: "16px" },
    { key: "*", left: 70,  top: 888, w: 108, h: 60, r: "16px" },
    { key: "0", left: 198, top: 904, w: 106, h: 48, r: "16px" },
    { key: "#", left: 322, top: 888, w: 108, h: 60, r: "16px" }
  ]
});
