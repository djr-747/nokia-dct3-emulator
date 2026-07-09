/*
 * sim_bridge - host-side ISO-7816 T=0 driver for the DCT3 SIM bridge.
 *
 * Talks the framed bridge protocol (docs/sim-bridge-protocol.md) to the ESP32
 * "thin reader" (esp32/simbridge) and runs the whole T=0 TPDU state
 * machine here on the host: procedure bytes, the 0x60 NULL, single-byte vs
 * all-remaining ACK, SW1/SW2.
 *
 * POSIX termios only - this is built into NATIVE targets (tools/simprobe, and
 * later the boot_trace/gui drivers). It must never be picked up by the wasm
 * build (it lives in tools/, outside the src/ glob).
 *
 * GSM 11.11 APDUs are single-data-phase (case 1/2/3); there is no case-4 over the
 * SIM interface (the phone issues a separate GET RESPONSE), so each exchange has
 * at most one data direction.
 */
#ifndef SIM_BRIDGE_H
#define SIM_BRIDGE_H

#include <stdint.h>

#define SIMB_DIR_NONE 0   /* case-1: header only (no data either way)   */
#define SIMB_DIR_IN   1   /* case-3: P3 = Lc, data flows host -> card   */
#define SIMB_DIR_OUT  2   /* case-2: P3 = Le, data flows card -> host   */

/* Open the serial link to the ESP32 (e.g. "/dev/ttyUSB0") at 115200 8N1.
 * Returns 0 on success, <0 on error. */
int sim_bridge_open(const char* dev);
void sim_bridge_close(void);

/* PING the reader; copies its id string into `id` (NUL-terminated). 0 / <0. */
int sim_bridge_ping(char* id, int cap);

/* Cold-reset the card and return the ATR. atr_len in/out (capacity -> length).
 * Returns 0 on OK, -2 no card, -3 mute, <0 other. */
int sim_bridge_activate(uint8_t* atr, int* atr_len);

/* Power the card down. */
int sim_bridge_deactivate(void);

/* Run one T=0 APDU.
 *   hdr[5]      = CLA INS P1 P2 P3
 *   dir         = SIMB_DIR_*
 *   data_in     = outgoing data (DIR_IN), length must equal hdr[4] (Lc)
 *   data_out    = buffer for incoming data (DIR_OUT); *out_len in=cap, out=count
 *   sw1,sw2     = status word (always set on success)
 * Returns 0 on a complete exchange (any SW), <0 on transport/protocol error. */
int sim_bridge_apdu(const uint8_t hdr[5], int dir,
                    const uint8_t* data_in, int data_in_len,
                    uint8_t* data_out, int* out_len,
                    uint8_t* sw1, uint8_t* sw2);

/* Set 1 to trace bridge frames + T=0 steps to stderr (or env SIMBRIDGE_LOG). */
void sim_bridge_set_log(int on);

#endif /* SIM_BRIDGE_H */
