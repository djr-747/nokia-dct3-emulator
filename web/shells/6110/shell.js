// Nokia 6110 (NSE-3 / MAD2, 84×48) — self-contained shell unit.
// Bar: white glyphs on gray keys.
// Photo + mask are 400x1100 (1:1 design space).
registerShell("6110", {
  img:  "shells/6110/nokia-6110.png",
  w: 400, h: 1100,
  lcd:  { left: 88, top: 418, w: 228, h: 158, radius: 10, canvasW: 210, canvasH: 144},
  zones: [
    { key: "pwr",     left: 278, top: 232, w: 32, h: 42, r: "10px 10px 10px 10px" },  // top power button
    { key: "volup",   left: 36,  top: 374,  w: 10, h: 30, r: "8px 4px 4px 8px" },  // left side
    { key: "voldown", left: 36,  top: 436, w: 10, h: 30, r: "8px 4px 4px 8px" },
    { key: "soft1",   left: 98,  top: 606, w: 62, h: 52, r: "50%" },              // left soft = Menu/Select
    { key: "soft2",   left: 246, top: 606, w: 62, h: 52, r: "50%" },              // right soft = Names
    { key: "up",      left: 178, top: 620, w: 48, h: 40, r: "12px 12px 4px 4px" },
    { key: "down",    left: 164, top: 660, w: 74, h: 40, r: "4px 4px 12px 12px" },
    { key: "send",    left: 90,  top: 686, w: 60, h: 40, r: "50%" }, // green call
    { key: "end",     left: 254, top: 686, w: 60, h: 40, r: "50%" }, // red end
    { key: "1", left: 80,  top: 744, w: 58, h: 38, r: "16px" },
    { key: "2", left: 168, top: 744, w: 70, h: 38, r: "16px" },
    { key: "3", left: 268, top: 744, w: 58, h: 38, r: "16px" },
    { key: "4", left: 80,  top: 802, w: 58, h: 38, r: "16px" },
    { key: "5", left: 168, top: 802, w: 70, h: 38, r: "16px" },
    { key: "6", left: 268, top: 802, w: 58, h: 38, r: "16px" },
    { key: "7", left: 80,  top: 860, w: 58, h: 38, r: "16px" },
    { key: "8", left: 168, top: 860, w: 70, h: 38, r: "16px" },
    { key: "9", left: 268, top: 860, w: 58, h: 38, r: "16px" },
    { key: "*", left: 80,  top: 918, w: 58, h: 38, r: "16px" },
    { key: "0", left: 168, top: 918, w: 70, h: 38, r: "16px" },
    { key: "#", left: 268, top: 918, w: 58, h: 38, r: "16px" }
  ]
});
