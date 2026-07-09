// MAD2 — CCONT (power/RTC/ADC controller on the GENSIO bus) + watchdog.
// Extracted from mad2.c; see mad2_internal.h.

#include "mad2/mad2_internal.h"

#include <stdio.h>
#include <stdlib.h>

// --- CCONT (power/RTC controller on the GENSIO bus) --------------------------

// Re-evaluate the CCONT interrupt cascade onto MAD IRQ2 (level-driven). IRQ2 is
// asserted whenever any cascade line (bits 3-7: CHARGER + the RTC ints) is pending
// AND unmasked. The firmware acks by clearing the CCONT source (reg 0x0E write-1)
// and the ASIC latch (I/O 0x09 bit2); we re-run this on both so a still-pending
// source re-asserts, and a fully-serviced one drops the line. (bit0 RTC-battery is
// a status flag, not a cascade source, so it never raises IRQ2.)
#define CC_CASCADE_MASK 0xF8u   // INT3 (charger) + INT4-7 (RTC)
void cc_int_update(Mad2* m) {
    if (m->cc_int_lines & ~m->cc_int_mask & CC_CASCADE_MASK) mad2_raise_irq(m, 2);
    else                                                     mad2_ack_irq(m, 2);
}

// --- Watchdog (CCONT WDT) ----------------------------------------------------
// The firmware kicks the watchdog regularly (ccont_reset_wdt writes a non-zero value
// to BOTH IO_CTSI_WDT(0x20003) and CCONT reg 0x05). A value of 0x00 = power-off, not
// a kick. We model the register state + a kick timestamp; the real time-out reset is
// gated OFF by default (see mad2_init / WDTRESET). `cc` distinguishes the CCONT-reg
// path (reg 0x05) from the CTSI shadow (I/O 0x03) for accurate register readback.
void wdt_kick(Mad2* m, uint8_t value, int cc) {
    if (cc) m->wdt_cc = value; else m->wdt_reg = value;
    // CCONT reg-0x05 / CTSI watchdog value table (MAME-confirmed; RE shows v5.79 writes 0x20
    // then 0x31): 0x20 = arm/enable, 0x31 = load/kick, 0x3F = DISABLE (dog off), 0x00 = power-off.
    // 0x3F is how the firmware legitimately stops the dog (e.g. before a path that can't kick /
    // deep sleep) — it must PAUSE the starvation timer, not read as a feed, or we'd false-trip.
    if (value == 0x3F) {                        // disable: pause the starvation window
        m->wdt_disabled = 1;
    } else if (value != 0x00) {                 // arm (0x20=32s) / kick (0x31=49s) / any other feed
        m->wdt_disabled = 0;
        m->wdt_kicks++;
        m->wdt_last_kick_cyc = m->rtc_mono;     // reset the starvation window
        // Value-driven window: WDReg is denominated in seconds (RE: 0x20=32s arm, 0x31=49s kick),
        // so the timeout is the written byte x the ARM clock — honor whatever value is loaded
        // instead of a hardcoded constant (matches the real reload-from-register behaviour).
        m->wdt_window_cyc = (uint64_t)value * DCT3_ARM_HZ;
    }
    // value == 0x00 falls through: power-off, latched by the power_off path elsewhere.
}

uint8_t ccont_read(Mad2* m) {
    uint8_t reg = m->ccont_addr & 0x0F;
    uint8_t v;
    // CCONT A/D MUX routing: the firmware-selected channel may map to a different
    // m->adc[] quantity index on oddball models (e.g. the 3210 wires VBATT to ch0).
    // adc_route[ch]==0 = identity (standard wiring); see ModelProfile.adc_route.
    uint8_t _ch = m->adc_channel & 7;
    if (m->model->adc_route[_ch]) _ch = m->model->adc_route[_ch] & 7;
    if (reg == 0x02)      v = (uint8_t)(m->adc[_ch] & 0xFF);        // A/D value bits 7:0
    // Reg 3 packs two things: bits[1:0] = A/D value bits 9:8 (the ADC routine at
    // 0x2EF462 masks to these), bits[7:2] = CCONT chip-ID/revision. The boot
    // self-test (test #7, 0x2D2318 via reader 0x2E99A4) reads reg 3 and requires
    // (v & 0xFC) == 0xB0 — else it writes fail code 0xFD to the self-test table and
    // the MMI shows CONTACT SERVICE. Return the ID in the high bits so both readers
    // are satisfied. See docs/boot-trace-3310.md.
    else if (reg == 0x03) v = (uint8_t)(0xB0u | ((m->adc[_ch] >> 8) & 0x03)); // chip-ID | A/D[9:8]
    else if (reg >= 0x07 && reg <= 0x0A) {   // RTC: base time + elapsed seconds (13 MHz clock)
        uint32_t now = m->rtc_base_sec + (uint32_t)((m->rtc_mono - m->rtc_base_cyc) / 13000000u);
        switch (reg) {
            case 0x07: v = (uint8_t)(now % 60);          break;  // second
            case 0x08: v = (uint8_t)((now / 60) % 60);   break;  // minute
            case 0x09: v = (uint8_t)((now / 3600) % 24); break;  // hour
            default:   v = (uint8_t)((now / 86400) % 256); break; // 0x0A day counter
        }
    }
    else if (reg == 0x0E) v = m->cc_int_lines;   // pending interrupt lines (cascade source)
    else if (reg == 0x0F) v = m->cc_int_mask;    // interrupt mask
    else                  v = m->ccont[reg];
    // TEMP(CCLOG): log who reads the CCONT pending-int register (reg 0x0E) — that's
    // the IRQ2 CCONT-cascade ISR sampling which INT fired. Strip before commit.
    {
        static int cclog = -1;
        if (cclog < 0) cclog = getenv("CCLOG") ? 1 : 0;
        if (cclog && reg == 0x0E)
            printf("[cclog] RD  reg0E -> %02X  pc=%06X mono=%llu\n",
                   v, m->cur_io_pc & 0xFFFFFFu, (unsigned long long)m->rtc_mono);
    }
    m->ccont_reads++;
    m->ccont_have_addr = 0;   // read completes the transaction
    if (m->verbose > 0) { m->verbose--; printf("    [ccont] rd  reg %02X -> %02X\n", reg, v); }
    return v;
}

void ccont_byte(Mad2* m, uint8_t b) {
    if (!m->ccont_have_addr) {
        m->ccont_addr = (b >> 3) & 0x0F;  // register# is in the upper bits of the addr byte
        m->ccont_have_addr = 1;
    } else {
        uint8_t reg = m->ccont_addr & 0x0F;
        m->dbg_ccw_count[reg]++; m->dbg_ccw_last[reg] = b;   // TEMP(clock): per-reg write tracker; strip before commit
        if (reg == 0x00 && (b & 0x08)) m->adc_channel = (b >> 4) & 0x07;  // reg0: enable A/D + select channel
        if (reg == 0x0E) { m->cc_int_lines &= (uint8_t)~b; if (b & 0x80) m->rtc_alr_assert_cyc = 0; cc_int_update(m);   // write 1 to release
            // TEMP(CCLOG): log the ISR's INT ack (which line it cleared). Strip before commit.
            static int cclogw = -1;
            if (cclogw < 0) cclogw = getenv("CCLOG") ? 1 : 0;
            if (cclogw) printf("[cclog] ACK reg0E &= ~%02X  pc=%06X mono=%llu\n",
                               b, m->cur_io_pc & 0xFFFFFFu, (unsigned long long)m->rtc_mono);
        }
        else if (reg == 0x0F) { m->cc_int_mask = b; cc_int_update(m); }         // interrupt mask
        else m->ccont[reg] = b;   // second byte = value
        if (reg >= 0x07 && reg <= 0x0A) {    // RTC counter write (sec/min/hr/day)
            // The CCONT RTC is four INDEPENDENT free-running counters (sec 0x07, min 0x08,
            // hr 0x09, day 0x0A) that cascade. Writing ONE counter sets only that counter;
            // the others keep running and the sub-second phase is unbroken. The firmware
            // relies on this: it keeps wall-time in a software base and uses the CCONT RTC as
            // an elapsed counter, periodically resetting just the DAY counter (reg 0x0A=0)
            // after folding whole days into the base — WITHOUT ever writing min/hr. A naive
            // "rebase rtc_base_sec from all four ccont[] shadows + rtc_base_cyc=now" zeroes the
            // never-written min/hr (stale 0 shadows) AND discards the sub-second phase, so a
            // periodic day-reset (~2/s) pins the whole clock at 0 → the on-screen clock freezes.
            // Fix: read-modify-write — capture the current running time, replace only the
            // written counter, preserve the sub-second phase.
            uint64_t el_cyc  = m->rtc_mono - m->rtc_base_cyc;
            uint32_t el_sec  = (uint32_t)(el_cyc / 13000000u);
            uint64_t sub_cyc = el_cyc - (uint64_t)el_sec * 13000000u;   // sub-second remainder
            uint32_t cur = m->rtc_base_sec + el_sec;
            uint32_t s = cur % 60u, mi = (cur / 60u) % 60u, h = (cur / 3600u) % 24u, d = cur / 86400u;
            if      (reg == 0x07) s  = b;
            else if (reg == 0x08) mi = b;
            else if (reg == 0x09) h  = b;
            else /* 0x0A */       d  = b;
            m->rtc_base_sec = d * 86400u + h * 3600u + mi * 60u + s;
            m->rtc_base_cyc = m->rtc_mono - sub_cyc;   // keep the sub-second phase running
            m->rtc_writes++;
            m->rtc_wr_pc = m->cur_io_pc;
            { static int rl = -1; if (rl < 0) rl = getenv("RTCLOG") ? 1 : 0;   // TEMP(RTCLOG); strip before commit
              if (rl) printf("[rtclog] wr reg%02X=%02X pc=%06X -> %02u:%02u:%02u d%u (base=%lus) mono=%llu\n",
                             reg, b, m->cur_io_pc & 0xFFFFFFu, h, mi, s, d,
                             (unsigned long)m->rtc_base_sec, (unsigned long long)m->rtc_mono); }
        }
        // CCONT reg 0x05 = watchdog (CCONT side of ccont_reset_wdt). 0x00 = power-off
        // request (ccont_poweroff), non-zero = a kick. Mirror it through wdt_kick so a
        // CCONT-only kick (without the I/O-0x03 shadow) still feeds the dog.
        if (reg == 0x05) {
            wdt_kick(m, b, /*cc=*/1);
            if (b == 0x00) {
                if (m->wdt_service_mode) {
                    m->wdt_inhibited_count++;    // service mode: ignore the shutdown request
                    if (m->verbose > 0) { m->verbose--; printf("    [wdt] CCONT WDT=0 inhibited by service mode (#%llu)\n", (unsigned long long)m->wdt_inhibited_count); }
                } else {
                    m->power_off = 1;            // ccont_poweroff cleared the WDT -> clean shutdown
                }
            }
        }
        // CCONT reg 0x0B/0x0C = RTC wakeup-alarm minute/hour. Arm the alarm when set.
        if (reg == 0x0B) { m->rtc_alr_min = b & 0x3F; m->rtc_alr_armed = 1; m->rtc_alr_fired = 0; }
        if (reg == 0x0C) { m->rtc_alr_hr  = b & 0x1F; m->rtc_alr_armed = 1; m->rtc_alr_fired = 0; }
        // CCONT reg 0x0D = clock gates (sleep / peripheral-clock control). Shadow only.
        if (reg == 0x0D) m->clk_gates = b;
        m->ccont_have_addr = 0;
        m->ccont_writes++;
        if (m->verbose > 0) { m->verbose--; printf("    [ccont] wr  reg %02X = %02X\n", m->ccont_addr & 0x0F, b); }
    }
}
