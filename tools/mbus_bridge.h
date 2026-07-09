// mbus_bridge — present the emulated phone's MBUS service bus on a host serial port,
// so a REAL DCT3 service tool (WinTesla / EepromTools / NokiX, e.g. under Wine) can talk
// to the emulated firmware over the genuine FBUS/MBUS service protocol. The tool emits the
// real frames; the emulated firmware runs the real service handlers (read-memory 0xD4,
// EEPROM write 0x64, FAID, …); we just carry the wire. With EE5110SAVE the result is a
// faithfully-provisioned EEPROM — no firmware patch, no HLE, no offline crypto.
//
// Transport modelled = the MBUS UART (0x18/0x19/0x1A); set the tool to MBUS mode. The
// phone↔tool framing is half-duplex single-wire, so inbound bytes are echoed back to the
// tool (the line echo real MBUS tools expect) unless disabled.
//
// This is the ONLY host-serial-touching TU for MBUS; it pokes mad2 only via the
// mbus_rx_push / mbus_tx_out_pop accessors (no I/O-dispatch reimplementation).

#ifndef DCT3_MBUS_BRIDGE_H
#define DCT3_MBUS_BRIDGE_H

struct Mad2;

// Open the bridge. If `port` is a non-empty device path, open it (a real/virtual COM,
// raw 9600 8N1). If NULL/empty, allocate a PTY and print its slave path to stderr (symlink
// it into Wine: `ln -s <slave> ~/.wine/dosdevices/com3`). `echo` != 0 mirrors inbound bytes
// back to the tool (single-wire MBUS line echo). Returns the master fd, or -1 on failure.
int  mbus_bridge_open(const char* port, int echo);

// Per-STEP RX dribble: feed one staged byte to the firmware when its receiver is enabled
// and its RX FIFO is empty (mirrors the MBUSFRAME idiom exactly). Cheap, no syscalls — call
// every step so a byte lands the instant the firmware drains the previous one, including
// across the firmware's brief RX-enable windows that a periodic poll would skip past.
void mbus_bridge_feed(struct Mad2* m);

// Periodic fd I/O (call every N steps): read host->staging (+single-wire echo) and forward
// phone->host TX. Keep this off the hot per-step path so the syscalls stay batched.
void mbus_bridge_poll(struct Mad2* m, int fd);

void mbus_bridge_close(int fd);

#endif // DCT3_MBUS_BRIDGE_H
