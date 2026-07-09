/*
 * The minimal SIM-only reference sketch for the emulator's real-SIM bridge.
 *
 * simbridge - ESP32 "thin" ISO-7816 SIM-card reader for the DCT3 emulator's
 * SIM bridge.
 *
 * The host (tools/sim_bridge.c, via tools/simprobe or, later, mad2) owns the
 * whole ISO-7816 *T=0* protocol: procedure bytes, the 0x60 NULL, single-byte vs
 * all-remaining ACK, SW1/SW2. This sketch is "thin": it only does the things a
 * host cannot - generate the card clock, sequence VCC/RST, and clock raw bytes on
 * the half-duplex I/O line. It has NO idea what an APDU is.
 *
 *   host  <--USB framed protocol-->  ESP32  <--ISO-7816 CLK/IO/RST/VCC-->  SIM
 *
 * See docs/sim-bridge-protocol.md for the wire protocol and full wiring.
 * ---------------------------------------------------------------------------
 * WIRING (WROOM dev board defaults; change the GPIO #defines for your board):
 *   GPIO25  CARD_CLK     -> SIM C3 (CLK)
 *   GPIO26  CARD_RST     -> SIM C2 (RST)
 *   GPIO27  CARD_VCC_EN  -> 3.0 V load switch enable -> SIM C1 (VCC)
 *   GPIO17  Serial1 TX --[1k]--+-- SIM C7 (I/O)      (series R like the MBus pump)
 *   GPIO16  Serial1 RX --------+
 *           10k pull-up from I/O to VCC
 *   GND -> SIM C5 (GND)
 *
 * The 1k on TX + shared RX means the ESP32 hears its own transmitted bytes
 * (half-duplex echo); TRANSFER discards exactly the bytes it sent before reading
 * the card's reply. For anything past bench bring-up, replace the 1k hack with a
 * proper open-drain I/O buffer and a regulated 3.0 V VCC switch.
 *
 * CONVENTION: direct only (ATR TS=0x3B), 8E2, LSB-first - the ESP32 hardware UART
 * maps straight onto direct-convention T=0. Inverse-convention cards (TS=0x3F)
 * are NOT supported in v1.
 * ---------------------------------------------------------------------------
 * LEDC API note: supports both arduino-esp32 core 2.x (ledcSetup/ledcAttachPin)
 * and 3.x (ledcAttach), selected at compile time via ESP_ARDUINO_VERSION_MAJOR.
 */

#include <Arduino.h>

// ---- Pin map ---------------------------------------------------------------
#define CARD_CLK     25
#define CARD_RST     26
#define CARD_VCC_EN  27
#define CARD_IO_RX   16   // Serial1 RX  (to SIM I/O)
#define CARD_IO_TX   17   // Serial1 TX  (1k to SIM I/O)
#define CARD_PRES    -1   // optional socket card-detect switch (LOW = inserted); -1 to disable
#define VCC_ACTIVE_HIGH 1 // 1 if CARD_VCC_EN HIGH powers the card

#define HOST_BAUD    115200UL          // USB link to host
#define CARD_CLK_HZ  3571200UL         // card clock; etu = f/372 -> 9600 baud
#define CARD_BAUD    (CARD_CLK_HZ/372) // = 9600
#define LEDC_CH      0
#define CARD         Serial1

// ---- Bridge framing --------------------------------------------------------
#define REQ_SYNC   0xA5
#define RSP_SYNC   0x5A
#define CMD_PING       0x01
#define CMD_ACTIVATE   0x02
#define CMD_DEACTIVATE 0x03
#define CMD_TRANSFER   0x04
#define CMD_SETBAUD    0x05  // payload: 4-byte LE baud — switch the CARD UART rate.
                             // The host decides policy (PPS vs implicit per the ATR's
                             // TA1); activate always resets back to CARD_BAUD (9600).
#define ST_OK       0x00
#define ST_NOCARD   0x01
#define ST_MUTE     0x02
#define ST_TIMEOUT  0x03
#define ST_BADREQ   0x04

// ---- Timeouts (milliseconds) ----------------------------------------------
#define ATR_FIRST_MS   200   // cold reset -> first ATR byte
#define ATR_GAP_MS      20   // inter-byte gap that ends the ATR
#define RX_FIRST_MS    300   // command -> first reply byte (>= card WWT margin)
#define RX_GAP_MS       20   // inter-byte gap that ends a reply burst

static uint8_t txbuf[300];
static uint8_t rxbuf[300];

// ISO-7816 convention. The physical framing (line idles HIGH, start bit = falling
// edge) is the SAME for both conventions, so the hardware UART frames both - do NOT
// hardware-invert (that breaks framing). Only the bit VALUES + ORDER differ:
//   Direct  (TS=0x3B): state H = logic 1, LSB-first  -> byte used as-is.
//   Inverse (TS=0x3F): state H = logic 0, MSB-first  -> byte = ~bitrev(raw).
// conv8() is an involution, so the same transform applies on RX (decode) and TX
// (encode). The host T=0 driver is oblivious to all of this.
static bool g_inverse = false;

static inline uint8_t bitrev8(uint8_t b) {
  b = (uint8_t)((b >> 4) | (b << 4));
  b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
  b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
  return b;
}
static inline uint8_t conv8(uint8_t b) { return g_inverse ? (uint8_t)~bitrev8(b) : b; }

static void cardUartBegin() {
  CARD.end();
  // Parity is over LEVELS at the UART, but the card checks parity over LOGIC bits.
  // Inverse convention inverts every level, which flips even<->odd parity. So a
  // direct card wants 8E2; an inverse card needs the UART set to 8O2 to land on
  // the even-logic parity the card expects (and to read its bytes parity-clean).
  uint32_t cfg = g_inverse ? SERIAL_8O2 : SERIAL_8E2;
  CARD.begin(CARD_BAUD, cfg, CARD_IO_RX, CARD_IO_TX);  // standard framing (no HW invert)
}

// ---- Card power / clock / reset -------------------------------------------
// LEDC: 1-bit resolution -> duty value 1 = 50% (square-wave card clock); 0 = low.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void clockInit() { ledcAttach(CARD_CLK, CARD_CLK_HZ, 1); }
static void clockOn()   { ledcWrite(CARD_CLK, 1); }
static void clockOff()  { ledcWrite(CARD_CLK, 0); }
#else
static void clockInit() { ledcSetup(LEDC_CH, CARD_CLK_HZ, 1); ledcAttachPin(CARD_CLK, LEDC_CH); }
static void clockOn()   { ledcWrite(LEDC_CH, 1); }
static void clockOff()  { ledcWrite(LEDC_CH, 0); }
#endif

static void vccOn()  { digitalWrite(CARD_VCC_EN, VCC_ACTIVE_HIGH ? HIGH : LOW); }
static void vccOff() { digitalWrite(CARD_VCC_EN, VCC_ACTIVE_HIGH ? LOW : HIGH); }

static bool cardPresent() {
#if CARD_PRES >= 0
  return digitalRead(CARD_PRES) == LOW;
#else
  return true;
#endif
}

static void cardDeactivate() {
  digitalWrite(CARD_RST, LOW);
  clockOff();
  vccOff();
  delay(5);
}

// Absolute ceiling on a single read so a continuously-noisy / floating I/O line
// (a marginal connection) can never hang the reader: without this, a line that
// streams bytes with no inter-byte gap loops forever and the sketch wedges (then
// browns out / drops off USB).
#define CARD_READ_MAX_MS 600

// Read raw bytes from the card I/O until the inter-byte gap exceeds gap_ms, or
// first_ms elapses with nothing, or the buffer fills, or CARD_READ_MAX_MS total.
static int cardRead(uint8_t* buf, int cap, uint32_t first_ms, uint32_t gap_ms) {
  int n = 0;
  uint32_t t0 = millis();
  // wait for the first byte
  while (!CARD.available()) {
    if (millis() - t0 > first_ms) return 0;
  }
  uint32_t last = millis();
  for (;;) {
    if (CARD.available()) {
      uint8_t b = conv8((uint8_t)CARD.read());
      if (n < cap) buf[n++] = b;
      // else overflow guard: drain but stop storing
      last = millis();
    } else if (millis() - last > gap_ms) {
      break;
    }
    if (millis() - t0 > CARD_READ_MAX_MS) break;   // hard ceiling — never hang
  }
  return n;
}

// One cold reset per ISO-7816-3 at the current convention: VCC on, clock on, hold
// RST low >=400 clocks, RST high, then read the ATR. Returns ATR length (0=mute).
static int cardResetRead(uint8_t* atr, int cap) {
  cardUartBegin();                   // apply g_inverse to the UART
  digitalWrite(CARD_RST, LOW);
  clockOff();
  vccOff();
  delay(5);
  vccOn();
  delay(2);
  clockOn();
  delayMicroseconds(200);            // >=400 clocks @3.57MHz (~112us)
  while (CARD.available()) CARD.read();   // flush noise
  digitalWrite(CARD_RST, HIGH);      // warm the card -> ATR
  return cardRead(atr, cap, ATR_FIRST_MS, ATR_GAP_MS);
}

// Activate, auto-detecting the convention: try direct; if TS isn't 0x3B, retry in
// inverse mode (g_inverse stays set so all later TRANSFERs use it).
static int cardActivate(uint8_t* atr, int cap) {
  g_inverse = false;
  int n = cardResetRead(atr, cap);
  if (n > 0 && atr[0] == 0x3B) return n;     // direct-convention card
  g_inverse = true;                          // assume inverse, redo cold reset
  n = cardResetRead(atr, cap);
  if (n > 0 && atr[0] == 0x3F) return n;      // inverse-convention card
  g_inverse = false;                          // neither matched; leave direct
  cardUartBegin();
  return n;                                    // return whatever we got (caller checks)
}

// Clock `len` bytes out on I/O, discard the half-duplex echo, then read the reply.
static int cardTransfer(const uint8_t* tx, int len, uint8_t* rx, int cap) {
  while (CARD.available()) CARD.read();   // drop stale bytes
  if (len > 0) {
    if (g_inverse) {                      // encode inverse convention per byte
      for (int i = 0; i < len; ++i) CARD.write(conv8(tx[i]));
    } else {
      CARD.write(tx, len);
    }
    CARD.flush();                         // wait until shifted out
    // discard our own echo (1k-on-TX half-duplex): read back exactly `len` bytes
    int echoed = 0;
    uint32_t t0 = millis();
    while (echoed < len && millis() - t0 < 100) {
      if (CARD.available()) { CARD.read(); echoed++; }
    }
  }
  return cardRead(rx, cap, RX_FIRST_MS, RX_GAP_MS);
}

// ---- Host framing ----------------------------------------------------------
static int readExact(uint8_t* buf, int len, uint32_t to_ms) {
  int n = 0;
  uint32_t t0 = millis();
  while (n < len) {
    if (Serial.available()) { buf[n++] = (uint8_t)Serial.read(); t0 = millis(); }
    else if (millis() - t0 > to_ms) return n;
  }
  return n;
}

static void sendResp(uint8_t status, const uint8_t* payload, int len) {
  uint8_t hdr[4] = { RSP_SYNC, status, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ck = status ^ hdr[2] ^ hdr[3];
  for (int i = 0; i < len; ++i) ck ^= payload[i];
  Serial.write(hdr, 4);
  if (len) Serial.write(payload, len);
  Serial.write(&ck, 1);
}

// Wait for and decode one request frame. Returns cmd (>=0) with payload in
// txbuf/len, or -1 on no/garbled frame.
static int readReq(int* out_len) {
  // sync
  for (;;) {
    if (!Serial.available()) return -1;
    if (Serial.read() == REQ_SYNC) break;
  }
  uint8_t h[3];
  if (readExact(h, 3, 50) != 3) return -1;
  uint8_t cmd = h[0];
  int len = h[1] | (h[2] << 8);
  if (len > (int)sizeof txbuf) return -1;
  if (readExact(txbuf, len, 200) != len) return -1;
  uint8_t ck;
  if (readExact(&ck, 1, 50) != 1) return -1;
  uint8_t calc = cmd ^ h[1] ^ h[2];
  for (int i = 0; i < len; ++i) calc ^= txbuf[i];
  if (calc != ck) { sendResp(ST_BADREQ, NULL, 0); return -1; }
  *out_len = len;
  return cmd;
}

void setup() {
  Serial.begin(HOST_BAUD);
  pinMode(CARD_RST, OUTPUT);    digitalWrite(CARD_RST, LOW);
  pinMode(CARD_VCC_EN, OUTPUT); vccOff();
#if CARD_PRES >= 0
  pinMode(CARD_PRES, INPUT_PULLUP);
#endif
  clockInit();
  clockOff();
  cardUartBegin();
}

void loop() {
  int len = 0;
  int cmd = readReq(&len);
  if (cmd < 0) return;

  switch (cmd) {
    case CMD_PING: {
      char id[40];
      int k = snprintf(id, sizeof id, "SIMBRIDGE v1 clk=%lukHz",
                       (unsigned long)(CARD_CLK_HZ / 1000));
      sendResp(ST_OK, (const uint8_t*)id, k);
      break;
    }
    case CMD_ACTIVATE: {
      if (!cardPresent()) { sendResp(ST_NOCARD, NULL, 0); break; }
      int n = cardActivate(rxbuf, sizeof rxbuf);
      if (n == 0) sendResp(ST_MUTE, NULL, 0);
      else        sendResp(ST_OK, rxbuf, n);
      break;
    }
    case CMD_DEACTIVATE:
      cardDeactivate();
      sendResp(ST_OK, NULL, 0);
      break;
    case CMD_TRANSFER: {
      if (!cardPresent()) { sendResp(ST_NOCARD, NULL, 0); break; }
      int n = cardTransfer(txbuf, len, rxbuf, sizeof rxbuf);
      sendResp(ST_OK, rxbuf, n);   // n may be 0 (caller decides if that's a timeout)
      break;
    }
    case CMD_SETBAUD: {
      // Old implicit-rate GSM SIMs (TA1 set, no TA2, e.g. TA1=0x94 -> F=512/D=8 ->
      // 55800 baud at 3.5712 MHz) go MUTE to T=0 at the default 9600 — they expect
      // the reader to jump to the TA1 rate right after ATR (pre-ISO'97 behavior).
      // updateBaudRate keeps the UART/pins/parity config, changes only the divisor.
      if (len != 4) { sendResp(ST_BADREQ, NULL, 0); break; }
      uint32_t baud = (uint32_t)txbuf[0] | ((uint32_t)txbuf[1] << 8)
                    | ((uint32_t)txbuf[2] << 16) | ((uint32_t)txbuf[3] << 24);
      if (baud < 1200 || baud > 250000) { sendResp(ST_BADREQ, NULL, 0); break; }
      CARD.flush();
      CARD.updateBaudRate(baud);
      sendResp(ST_OK, NULL, 0);
      break;
    }
    default:
      sendResp(ST_BADREQ, NULL, 0);
      break;
  }
}
