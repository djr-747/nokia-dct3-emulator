# SIM-card bridge — emulator ↔ ESP32 ↔ real GSM SIM

> **Firmware note:** the SIM reader is now one role of the unified `dct3uni` sketch
> (MBus + FBus + SIM) — see ``. `dct3uni` boots into the
> exact framed protocol below, so everything here is unchanged; the standalone
> `flasher/arduino/simbridge` is kept only as the SIM-only reference.

> **Context:** 3310 v5.79 (`firmware/Factory Reset 3310 NR1 v5.79.fls`). mad2 SIM
> model lives in `src/mad2/mad2.c` (SIMI regs `0x20036–0x2003F`, `sim_run_apdu`,
> `sim_process_apdu`, `sim_deliver_atr`). This bridge lets the emulator talk to a
> **real physical SIM** on an ESP32 instead of the synthetic in-memory EF tree —
> so we can (a) capture/validate the real APDU protocol, (b) boot with the card's
> real ICCID/IMSI, and (c) pass `RUN GSM ALGORITHM` (0x88) through to the card's
> real Ki (today stubbed `0x98 0x04`).

## Architecture

Three layers, decided per `AskUserQuestion` 2026-06-03: **APDU-level tap** + **thin
ESP32** + host owns T=0.

```
  ┌────────────────────────┐   APDU      ┌──────────────────┐  bridge   ┌─────────┐  ISO-7816  ┌────────┐
  │ emulator (mad2.c)      │  boundary   │ host T=0 driver  │  frames   │ ESP32   │   T=0      │ real   │
  │ sim_run_apdu()         │────────────▶│ tools/sim_bridge │──USB────▶ │ "thin   │──CLK/IO──▶ │ GSM    │
  │ sim_deliver_atr()      │◀────────────│ .c (TPDU SM)     │◀──serial──│  reader"│◀──RST/VCC─ │ SIM    │
  └────────────────────────┘  data+SW    └──────────────────┘  (115200) └──────────┘            └────────┘
       (Phase C, deferred)        ▲
                                  │
                          tools/simprobe.c  ← bench bring-up, no emulator
```

- **ESP32 = electrical only.** Generates the card clock, drives RST/VCC, shuttles
  bytes on the half-duplex I/O line. **No T=0 logic, no APDU awareness.** Mirrors
  the existing `flasher/` philosophy (smart host, dumb wire) — but it must *own the
  clock*, which a SIM requires.
- **Host (`tools/sim_bridge.c`) = T=0 TPDU state machine.** Procedure bytes, the
  `0x60` NULL, single-byte vs all-remaining ACK, SW1/SW2. POSIX `termios`; built
  only into native targets (never the wasm glob).
- **Tap point = APDU boundary.** mad2 already assembles whole APDUs in `sim_asm`
  (`sim_run_apdu`, `mad2.c:1489`). Phase C reroutes that one dispatch to the bridge
  and pushes the real `data + SW1 SW2` back into the RX FIFO in the *same* framing
  `sim_process_apdu` uses — the firmware can't tell. The emulator's clock is not
  wall-clock-paced, so crossing to the real-time card once per APDU (a few ms,
  bursty at boot) is robust; forwarding raw UART bytes would fight T=0 etu timing.

## Host ↔ ESP32 wire protocol

Binary, framed, runs over the USB-serial link at **115200 8N1** (independent of the
card's own f/372 baud). Little-endian length.

```
request  (host→esp):  0xA5  CMD     LEN_lo LEN_hi  payload[LEN]  CK
response (esp→host):  0x5A  STATUS  LEN_lo LEN_hi  payload[LEN]  CK
CK = XOR of all bytes after the sync byte (CMD/STATUS .. last payload byte)
```

### Commands (request `CMD`)

| CMD  | name        | payload          | response payload            |
|------|-------------|------------------|-----------------------------|
| 0x01 | PING        | —                | ASCII id string + clock kHz |
| 0x02 | ACTIVATE    | —                | ATR bytes (cold reset)      |
| 0x03 | DEACTIVATE  | —                | —                           |
| 0x04 | TRANSFER    | bytes to clock out | bytes read back from card |

`TRANSFER` is the only primitive the host T=0 SM needs:
- payload **non-empty** → clock those bytes out on I/O, discard the half-duplex
  echo, then read the card's reply until the inter-byte gap exceeds WWT.
- payload **empty** (LEN=0) → *read-only continuation*: don't send, just read more
  bytes from the card (used to fetch procedure-byte continuations / SW).

### Response `STATUS`

| STATUS | meaning                              |
|--------|--------------------------------------|
| 0x00   | OK                                   |
| 0x01   | no card present (socket switch open) |
| 0x02   | mute card (no ATR within timeout)    |
| 0x03   | I/O timeout                          |
| 0x04   | malformed request / bad checksum     |

## ISO-7816 wiring (WROOM dev board defaults — adjust in the sketch)

Bring-up uses the same series-resistor trick as the MBus bridge (host already
tolerates self-echo). For a hardened build, use a proper 3 V load switch for VCC
and an open-drain I/O buffer.

```
  ESP32 GPIO25 ── CARD_CLK ─────────────── SIM C3 (CLK)
  ESP32 GPIO26 ── CARD_RST ─────────────── SIM C2 (RST)
  ESP32 GPIO27 ── CARD_VCC_EN ─[switch]──── SIM C1 (VCC, 3.0 V)
  ESP32 GPIO17 ── (Serial1 TX) ─[1k]──┬──── SIM C7 (I/O)
  ESP32 GPIO16 ── (Serial1 RX) ───────┘
                          10k pull-up from I/O ── VCC
  ESP32 GND ───────────────────────────── SIM C5 (GND)
  (SIM C6 VPP unused; optional card-detect switch → GPIO32)
```

- **Card clock** ≈ 3.5712 MHz (LEDC). Card initial etu = f/372 ⇒ **9600 baud**.
- **Convention:** direct only (ATR TS = `0x3B`), 8E2, LSB-first — matches the ESP32
  hardware UART. Inverse-convention cards (TS = `0x3F`) are **not** handled in v1.
- **Voltage:** target 3 V (Class B) SIMs. ESP32's 3.3 V is at the top of 3 V ±10 %;
  prefer a regulated 3.0 V VCC for anything beyond bench bring-up.

## Bring-up sequence (bench, no emulator)

```bash
make simprobe
./build/simprobe /dev/ttyUSB0 atr           # cold reset, dump ATR
./build/simprobe /dev/ttyUSB0 iccid          # SELECT MF / 2FE2, READ BINARY
./build/simprobe /dev/ttyUSB0 imsi           # SELECT 7F20 / 6F07, READ BINARY
./build/simprobe /dev/ttyUSB0 apdu A0A40000023F00   # raw APDU (hex)
```

Validate the captured exchange against the synthetic model: run the emulator with
`SIMLOG=1` and diff the APDU traces.

## Phase C — wiring into mad2 (deferred until hardware is proven)

Gated behind `SIMBRIDGE=/dev/ttyUSB0`; compiled out of the wasm build
(`#ifndef __EMSCRIPTEN__`). Two touch-points only:

1. `sim_deliver_atr()` (`mad2.c:1526`) — when the bridge is open, push the **real**
   ATR instead of the canned `3B 60 00 00`.
2. `sim_run_apdu()` (`mad2.c:1489`) — when a full APDU is assembled, call
   `sim_bridge_apdu()` and push the real `data + SW1 SW2` using the same RX-FIFO
   framing as `sim_process_apdu`. Per-INS fallback to the synthetic model stays
   available (e.g. if the card is absent).

`tools/sim_bridge.c` is added to `TRACE_SRCS`/`GUI_SRCS` explicitly (native lists
are explicit; the wasm build globs `src/` and must not pick up `termios`).
