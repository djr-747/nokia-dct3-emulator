// MAD2 — MBUS/FBUS receive. Extracted from mad2.c; see mad2_internal.h.
//
// MADos drives MBUS RX over the SAME line as TX (mbus.c): the receive ISR is FIQ2
// (FIQ_MBUSRX); on a received byte the UART sets IO_MBUS_STATUS(0x19) bit5 RXDRDY
// (0x20) and the handler reads IO_MBUS_BYTE(0x1A). We model a deterministic empty RX
// ("no accessory connected") so the firmware's accessory/headset poll sees nothing
// and never hangs. These helpers also let a future model inject accessory bytes (a
// service command, a CarKit handshake) without changing the I/O dispatch: push bytes,
// call mbus_rx_signal, and FIQ2 drives the firmware's MBUS event ISR to read them.

#include "mad2/mad2_internal.h"

int mbus_rx_count(const Mad2* m) {
    return (int)((uint8_t)(m->mbus_rx_tail - m->mbus_rx_head));
}
uint8_t mbus_rx_pop(Mad2* m) {
    if (m->mbus_rx_head == m->mbus_rx_tail) return 0xFF;   // empty: idle line reads 0xFF
    uint8_t b = m->mbus_rx[m->mbus_rx_head++ % sizeof(m->mbus_rx)];
    m->mbus_rx_bytes++;
    if (mbus_rx_count(m) == 0) m->mbus_rxdrdy = 0;         // FIFO drained -> RXDRDY clears
    return b;
}
// Assert RXDRDY + FIQ2 while bytes wait (only meaningful in RECEIVE mode, ctrl bit6).
// Currently unused (no accessory talks); kept for accessory injection.
void mbus_rx_signal(Mad2* m) {
    if (mbus_rx_count(m) > 0 && (m->mbus_ctrl & 0x40)) {   // RX-enable (ctrl bit6) set
        m->mbus_rxdrdy = 1;
        mad2_raise_fiq(m, 2);                       // FIQ2 = MBUS RX
    }
}

// --- Host serial-bridge plumbing (a real service tool over a host COM/PTY) --------------
// Push one received byte into the RX FIFO and pulse FIQ2, exactly as the boot_trace
// MBUSFRAME path does — used by the live bridge to feed bytes a tool sent over the wire.
// Returns 1 if accepted (FIFO had room), 0 if full (caller retries next drain).
int mbus_rx_push(Mad2* m, uint8_t b) {
    if ((uint8_t)(m->mbus_rx_tail - m->mbus_rx_head) >= sizeof(m->mbus_rx)) return 0;  // full
    m->mbus_rx[m->mbus_rx_tail++ % sizeof(m->mbus_rx)] = b;
    mbus_rx_signal(m);
    return 1;
}
// Drain one shifted-out TX byte (the phone's MBUS transmission) for the host to forward.
// Returns the byte (0..255), or -1 when the TX ring is empty.
int mbus_tx_out_pop(Mad2* m) {
    if (m->mbus_tx_out_head == m->mbus_tx_out_tail) return -1;
    return m->mbus_tx_out[m->mbus_tx_out_head++ % sizeof(m->mbus_tx_out)];
}
