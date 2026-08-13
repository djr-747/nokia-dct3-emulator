// Nokia 7110 (NSE-5 / MAD2, 96×65, RE'd keypad) — self-contained shell unit.
// Slider: white glyphs on gray keys.
// RE'd nav cluster maps cleanly — LEFT soft (Menu/Select) = soft1, RIGHT soft
// (Names) = soft2, plus wheel, green Send / red End, side volume keys, and a
// dedicated top Power button. Photo + mask are 400x900 (1:1 design space).
registerShell("7110", {
  img:  "shells/7110/nokia-7110.png",
  w: 400, h: 900,
  lcd:  { left: 89, top: 272, w: 216, h: 186, radius: 10, canvasW: 192, canvasH: 156},
  zones: [
    { key: "pwr",     left: 269, top: 160, w: 32, h: 32, r: "10px 10px 10px 10px" },  // top power button
    { key: "soft1",   left: 73,  top: 480, w: 80, h: 40, r: "50%" },              // left soft = Menu/Select
    { key: "soft2",   left: 239, top: 480, w: 80, h: 40, r: "50%" },              // right soft = Names
    { key: "wheelup",      left: 166, top: 506, w: 60, h: 16, r: "12px 12px 4px 4px" },
	{ key: "wheelpress",      left: 166, top: 522, w: 60, h: 16, r: "4px 4px 4px 4px" },
    { key: "wheeldown",    left: 166, top: 538, w: 60, h: 16, r: "4px 4px 12px 12px" },
    { key: "send",    left: 85,  top: 534, w: 68, h: 40, r: "50%" }, // green call
    { key: "end",     left: 239, top: 534, w: 68, h: 40, r: "50%" }, // red end
    { key: "1", left: 93,  top: 588, w: 58, h: 28, r: "16px" },
    { key: "2", left: 167, top: 592, w: 58, h: 28, r: "16px" },
    { key: "3", left: 241, top: 588, w: 58, h: 28, r: "16px" },
    { key: "4", left: 93,  top: 636, w: 58, h: 28, r: "16px" },
    { key: "5", left: 167, top: 640, w: 58, h: 28, r: "16px" },
    { key: "6", left: 241, top: 636, w: 58, h: 28, r: "16px" },
    { key: "7", left: 93,  top: 684, w: 58, h: 28, r: "16px" },
    { key: "8", left: 167, top: 688, w: 58, h: 28, r: "16px" },
    { key: "9", left: 241, top: 684, w: 58, h: 28, r: "16px" },
    { key: "*", left: 93,  top: 732, w: 58, h: 28, r: "16px" },
    { key: "0", left: 167, top: 736, w: 58, h: 28, r: "16px" },
    { key: "#", left: 241, top: 732, w: 58, h: 28, r: "16px" }
  ]
});
