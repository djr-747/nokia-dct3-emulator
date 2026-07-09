// MAD2 — keypad matrix scan, keypad IRQ, slide-cover switch. Extracted from
// mad2.c; see mad2_internal.h.

#include "mad2/mad2_internal.h"

// --- Keypad matrix -----------------------------------------------------------
// Columns are active-low: a pressed key pulls its column bit to 0. Idle = 0x1F.
// The scanner first sets row-direction 0xE0 and reads the "special" keys; if it
// sees a key it stops. Otherwise it drives one row low at a time (row signal =
// ~(1<<row)) and reads the per-row columns.
uint8_t kbd_cols(const Mad2* m) {
    KeypadScan scan = m->model ? m->model->keypad.scan : KP_SCAN_PLAIN;
    // 0xE0 = the dedicated power/side special-scan on PLAIN/DIR_AWARE keypads. The SERIAL
    // keypad (5110) instead REUSES DIR_R=0xE0 as the upper-row (5-7) phase of its dir-aware
    // matrix scan, so it must fall through to the scan below — otherwise every phase-1 read
    // returns ~kbd_special_cols (0x1D idle), the fw sees a "special key" and never scans.
    if (scan != KP_SCAN_SERIAL && m->kbd_row_dir == 0xE0) {   // special-key scan (power/side)
        return (uint8_t)(0x1F & ~m->kbd_special_cols);
    }
    // DIR-aware scan (DIR_AWARE + SERIAL) walks rows via DIR_R: exactly ONE row is an OUTPUT
    // at a time, the rest inputs, so a row contributes only when it is both an output (DIR bit
    // set) AND driven low (row-signal bit clear). Without the DIR test mad2 ORs the leftover-low
    // rows from the strobe sequence -> the key is "found" on the wrong strobe -> wrong rawkey.
    // PLAIN (3310) leaves all rows driven, so it uses the row-signal alone.
    int dir_aware = (scan != KP_SCAN_PLAIN);
    uint8_t pressed = 0;
    for (int row = 0; row < 8; ++row) {           // normal scan: row driven low
        int sel = !((m->kbd_row_sig >> row) & 1);
        if (dir_aware) sel = sel && ((m->kbd_row_dir >> row) & 1);   // AND: row is an output
        if (sel) pressed |= m->kbd_norm_cols[row];
    }
    return (uint8_t)(0x1F & ~pressed);
}

// A NORMAL matrix keypad edge (a shell just changed kbd_norm_cols). Raise the keypad
// IRQ0 as every shell already does; additionally, for an 8850-class source-decoding
// dispatcher (keypad.uif_irq), assert the matrix interrupt-source bit (I/O 0x2B bit4)
// so the firmware's IRQ0 dispatcher (0x301714) routes to the keypad handler/scan. The
// firmware acks it by writing IM_C (0x6B), which clears the bit (see mad2_write 0x6B).
// 3310-class models (uif_irq=0) only get the IRQ0 — their IRQ0 goes straight to the ISR.
void mad2_keypad_irq(Mad2* m) {
    // Honor the keypad-column interrupt MASK on the serial-keypad family (reg33_im_c).
    // On these models I/O 0x33 (IM_C) bits0-4 mask the matrix columns: the scan ISR
    // SETs them (|=0x1F, fw 0x290C4C/0x290D2E) while scanning/debouncing and the no-key
    // path CLEARs them (&=0xE0, fw 0x25AF3A) to re-arm. A key edge that arrives while the
    // columns are masked is suppressed by the hardware — it must NOT vector the firmware.
    // Without this gate a key RELEASE (which lands while IM_C is still masked from the
    // press scan) raised a spurious IRQ0; the IRQ handler then decoded the FSM key-state
    // cache from the still-stale rawkey ([0x10B6C8] only refreshes on the next periodic
    // scan after re-arm), latched "key held", and re-armed the auto-repeat — so a single
    // home-screen tap mis-fired the press-and-hold action (speed-dial). The press itself
    // is unaffected: IM_C is armed (cols clear) at the down-edge. See docs/5110 keypad RE.
    if (m->model && m->model->keypad.reg33_im_c && m->mem) {
        uint32_t mask = m->mem_mask ? m->mem_mask : 0x00FFFFFFu;
        uint32_t imc  = (m->model->mem.mmio_base + 0x33u) & mask;
        if (m->mem[imc] & 0x1Fu) return;   // columns masked -> edge suppressed (HW behavior)
    }
    mad2_raise_irq(m, 0);
    if (m->model && m->model->keypad.uif_irq) m->kpd_im_status |= 0x10;  // matrix source bit4
}

// Slide-cover (reed switch) toggle on a slide phone. The cover line is I/O 0x28 bit0
// (read by 0x30175E: bit0==1 -> "open"); a state change raises the reed-switch interrupt
// source (I/O 0x29 bit0) which the 8850 IRQ0 dispatcher (0x301738) routes to the cover
// handler 0x305452 (it acks via IM_R 0x69). Posts the open/closed cover message.
void mad2_slide_set(Mad2* m, int open) {
    if (!m->model || !m->model->keypad.has_slide) return;
    uint8_t v = open ? 1 : 0;
    if (m->slide_open == v) return;            // no edge
    m->slide_open = v;
    m->cover_int_pending = 1;                  // I/O 0x29 bit0 -> dispatcher routes to cover handler
    mad2_raise_irq(m, 0);                    // shared keypad/UIF IRQ0
}
