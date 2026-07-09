// MAD2 — CTSI interrupt controller, Timer0/Timer1, FIQ8 ct_timer, and the
// per-step tick + IRQ/FIQ poll. Extracted from mad2.c; see mad2_internal.h.
// mad2_timers_tick / mad2_irq_poll / mad2_fiq_poll are public (mad2.h).

#include "mad2/mad2_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Advance Timer0 from the CPU cycle count; latch IRQ4 when it crosses dest.
// Use the real Timer0 frequency relative to the ARM clock so the firmware's
// programmed intervals map to a faithful number of CPU cycles: Timer0 ticks at
// 33055/(div+1) Hz, the MAD ARM at ~13 MHz. This keeps the scheduler tick and
// timed checks consistent and avoids both the ISR-overshoot storm and timeouts.
// (DCT3_ARM_HZ / DCT3_T0_HZ defined near the top with the IO offsets.)
void mad2_timers_tick(Mad2* m, uint32_t cycles) {
    // Monotonic time base for the CCONT RTC. cpu->cycles is periodically rebased down
    // by 0x20000000 (DCT3_EVENT_SLICE) to avoid int32 overflow; undo that here so the
    // accumulator only ever increases (1 RTC second = 13e6 cycles at the ARM clock).
    { int64_t d = (int64_t)cycles - (int64_t)m->rtc_last_cyc;
      if (d < 0) d += 0x20000000;
      m->rtc_mono += (uint64_t)d; m->rtc_last_cyc = cycles; }
    sim_tick(m);   // SIM: deliver pending ATR, re-arm FIQ6 for queued RX, FIQ7 on insert/remove
    // CCONT charger-detect (CContINT3): the CCONT raises INT3 on a charger plug/unplug
    // edge; the firmware's IRQ2 handler then re-reads the charger ADC (ch5) and starts/
    // stops the charging UI. Edge-detect the modelled charger voltage (set by the web UI)
    // and pulse INT3 -> IRQ2. (Charger present ~ adc[5] above a small threshold.)
    { uint8_t present = (m->adc[5] >= 0x100) ? 1u : 0u;
      if (present != m->charger_present) {
          m->charger_present = present;
          m->cc_int_lines |= 0x08;   // INT3 pending
          cc_int_update(m);
      }
      // Charging-current ADC (ch7): a real charger pushes current while charging, and the
      // firmware reads ch7 to decide "charging" vs "not charging" (display_msg DMI_WARNING).
      // Model current flowing while the charger is connected and the battery isn't full;
      // at/near full it tapers to ~0 (which legitimately reads "not charging / full").
      if (!present)                    m->adc[7] = 0x000;          // no charger -> no current
      else if (m->adc[2] < 0x02B0u)    m->adc[7] = 0x0180;         // charging: current flows
      else                             m->adc[7] = 0x0020;         // battery full: taper to trickle

      // Battery-voltage dynamics while charging. The firmware's charge state machine
      // (TASK_17_CHARGE @ 0x0022A578, state 65/0x41) asserts 0x5C0A and bails to
      // state 5 when scaled VBATT >= 0x22C (~3.6 V) AND its phase flag is set —
      // i.e. it concludes "battery is already full, no need to charge". With a
      // statically-modeled adc[2], plugging the charger into a not-quite-full
      // battery shows "Not charging" almost instantly. A real NiMH cell rises
      // slowly toward ~4.2 V while charging. We ramp adc[2] by 1 raw step every
      // VBATT_RISE_PERIOD cycles, capped at VBATT_FULL — fast enough to animate
      // bars in an emulator session (full charge from a low pack ~minutes of
      // emulated time), slow enough that the firmware's charge logic still sees
      // a level it'll act on. Charger unplug freezes the level — discharge isn't
      // modeled (idle wall-clock is short on the emu; no UX value yet).
      #define VBATT_FULL          0x02C0u   // ~3.7 V — firmware's idea of "full"
      #define VBATT_RISE_PERIOD   5000000ull // ~0.4 s emulated per +1 raw step
      if (present && m->adc[2] < VBATT_FULL) {
          if (m->rtc_mono - m->vbatt_rise_last_cyc >= VBATT_RISE_PERIOD) {
              m->adc[2]++;
              m->vbatt_rise_last_cyc = m->rtc_mono;
          }
      } else {
          // Anchor so the first tick after a plug-in doesn't immediately step.
          m->vbatt_rise_last_cyc = m->rtc_mono;
      }
    }
    // ===== PERF: nextEvent body-skip (the hot-path win) =======================================
    // Profiling: the timer-edge body below (RTC-MIN..FIQ4) is ~73% of the 3310's runtime — NOT
    // the arithmetic (removing individual timer divides changed nothing; skipping the whole body
    // was 3.5x) but the cache pressure of touching ~20 scattered Mad2 fields + a per-tick FIQ2
    // ack call EVERY instruction. When every edge source is quiescent the body is a no-op, so
    // bypass it straight to the (free) DSP pump + Timer0. The guard covers EVERY source that can
    // fire or needs per-tick servicing; byte-identical (make guard). CCFORCE5 (a per-tick diag)
    // opts out via `fast`. Timer0 (after the rejoin) still ticks the OS scheduler every step.
    {
        static int fast = -1;
        if (fast < 0) fast = getenv("CCFORCE5") ? 0 : 1;
        if (fast
            && m->rtc_mono < m->rtc_min_next                 // RTC-MIN (INT5) not due
            && m->rtc_mono < m->t1_wrap_next                 // Timer1 16-bit wrap not due
            && !m->rtc_alr_enabled                           // RTC-alarm subsystem not armed (block 1)
            && !(m->cc_int_lines & 0x80)                     // no un-acked RTC-alarm line (block 2)
            && !(m->fiq8_ctrl & 0x01)                        // FIQ8 ct_timer not enabled
            && !m->power_off                                 // not in the power-off latch
            && !m->fiq3_delay && !m->mbus_tx_delay           // no MBUS TxD / TX countdown pending
            && !m->mbus_txe && !m->mbus_rxdrdy               // no MBUS FIQ2 source
            && !(m->fiq_pending & (1u << 2))                 // FIQ2 not pending (still needs deassert)
            && !m->fiq4_period                               // no FIQ4 heartbeat
            && !(m->wdt_reg && m->wdt_cc && !m->wdt_disabled // watchdog won't trip this tick
                 && (m->rtc_mono - m->wdt_last_kick_cyc) > m->wdt_window_cyc))
            goto edge_skip;
    }
    // CCONT RTC-MIN interrupt (CContINT5): fires once per real minute. The 3310 idle
    // clock shows HH:MM, so the firmware drives its per-minute clock tick/redraw from
    // INT5 (it unmasks INT5 in reg 0x0F only once the user sets the time; RTC-SEC/INT4
    // stays masked). The minute boundary is taken from the FREE-RUNNING cycle counter
    // (rtc_mono), NOT the settable base: the firmware periodically re-inits the CCONT
    // and rewrites the RTC, which would otherwise keep resetting the base and starve
    // the edge. 1 minute = 13e6 cycles/s * 60.
    // PERF: cached next-minute TARGET — the per-instruction hot path is a single compare
    // (no divide, no multiply). The boundary is recomputed only AT the edge (a divide once
    // per ~780M cycles). Byte-identical to the old per-tick `min = rtc_mono/PERIOD;
    // if (min != last)`: same crossing cycle, and the edge recompute self-corrects exactly
    // as the divide did (so a reset-to-0 rtc_mono re-syncs identically).
    if (m->rtc_mono >= m->rtc_min_next) {
        m->rtc_last_min = (uint32_t)(m->rtc_mono / (13000000ull * 60ull));
        m->rtc_min_next = ((uint64_t)m->rtc_last_min + 1u) * (13000000ull * 60ull);
        m->cc_int_lines |= 0x20;   // INT5 RTC-MIN pending
        m->rtc_min_edges++;
        cc_int_update(m);
    }
    // TEMP(CCFORCE5): inject INT5 (RTC-MIN) edges early + unmask, to exercise the IRQ2
    // CCONT cascade ISR without waiting a full emulated minute (780M cyc). CCFORCE5_AT =
    // first fire (cycles, default 90M ~ post-idle), CCFORCE5_EVERY = re-fire period.
    // Strip before commit. (Forced unmask is for ISR discovery only, not a fidelity claim.)
    {
        static long at = -2, every = 0; static uint64_t next = 0;
        if (at == -2) {
            at = getenv("CCFORCE5") ? (getenv("CCFORCE5_AT") ? atol(getenv("CCFORCE5_AT")) : 90000000) : -1;
            every = getenv("CCFORCE5_EVERY") ? atol(getenv("CCFORCE5_EVERY")) : 8000000;
            next = (uint64_t)(at < 0 ? 0 : at);
        }
        if (at >= 0 && m->rtc_mono >= next) {
            m->cc_int_mask  &= (uint8_t)~0x20u;   // unmask INT5
            m->cc_int_lines |= 0x20u;             // INT5 pending
            cc_int_update(m);
            next = m->rtc_mono + (uint64_t)(every > 0 ? every : 8000000);
            printf("[ccforce5] inject INT5 @mono=%llu lines=%02X mask=%02X\n",
                   (unsigned long long)m->rtc_mono, m->cc_int_lines, m->cc_int_mask);
        }
    }
    // CCONT RTC-ALARM interrupt (CContINT7 / RTC-ALR, cascade bit7). When an alarm
    // time is armed (regs 0x0B/0x0C written) and the running RTC hour:minute matches
    // the programmed wakeup, the CCONT raises INT7 -> IRQ2 (one-shot per armed window).
    //
    // DELIBERATELY PASSIVE BY DEFAULT (RTCALARM env to arm). Asserting INT7 wedges the
    // CPU in our current OS bring-up: the firmware's IRQ2 cascade handler reads CCONT
    // reg 0x0E, but there is no scheduled RTC-alarm task to ACK bit7 (that task lives
    // behind the manager-scheduling wall, see project-state). cc_int_update is level-
    // driven, so an un-acked INT7 re-asserts IRQ2 every return-from-handler -> infinite
    // IRQ2 ISR (verified: 90k+ IRQs, CPU pinned in IRQ mode at 0x002997B6). This is the
    // same "destructive action gated OFF by default" rule the watchdog/power-off follow:
    // we keep the register state (0x0B/0x0C arm + the wakeup match) faithful, but only
    // pulse INT7 when explicitly enabled. SAFETY: even when enabled, auto-release the
    // line if the firmware hasn't acked it within a bounded window, so it can NEVER
    // infinite-loop the CPU. On real HW (or once the alarm task is scheduled) the
    // firmware acks bit7 and the line drops normally; this only catches the un-serviced
    // case our incomplete OS state produces.
    if (m->rtc_alr_enabled && m->rtc_alr_armed && !m->rtc_alr_fired) {
        uint32_t now = m->rtc_base_sec + (uint32_t)((m->rtc_mono - m->rtc_base_cyc) / 13000000u);
        uint8_t cur_min = (uint8_t)((now / 60) % 60);
        uint8_t cur_hr  = (uint8_t)((now / 3600) % 24);
        if (cur_min == m->rtc_alr_min && cur_hr == m->rtc_alr_hr) {
            m->rtc_alr_fired = 1;        // one-shot until re-armed
            m->cc_int_lines |= 0x80;     // INT7 RTC-ALR pending
            m->rtc_alr_edges++;
            m->rtc_alr_assert_cyc = m->rtc_mono;   // start the un-ack watchdog window
            cc_int_update(m);
        }
    }
    // Auto-release safety: if INT7 stays asserted (firmware never wrote reg 0x0E bit7
    // to clear it) for longer than a short window, force-release it so a never-serviced
    // alarm cannot pin the CPU in the IRQ2 ISR. ~0.25 s at the ARM clock is far longer
    // than any real IRQ2 service path but short enough to break a runaway.
    if ((m->cc_int_lines & 0x80) && m->rtc_alr_assert_cyc &&
        (m->rtc_mono - m->rtc_alr_assert_cyc) > (13000000ull / 4)) {
        m->cc_int_lines &= (uint8_t)~0x80u;   // drop the un-acked INT7 line
        m->rtc_alr_assert_cyc = 0;
        cc_int_update(m);
    }
    // Timer1: a 16-bit free-running time base (IO_CTSI_TMR1 0x20004) and an optional
    // FIQ5 overflow interrupt. Two separate things, gated separately:
    //
    //  - The COUNTER free-runs continuously (33055 Hz CTSI timebase). It is a pure time
    //    base read by timer_get_raw_time and by the boot sleep-finalize gate (0x2E4278
    //    needs [0x20006]-[0x20004] >= 30), so it ALWAYS advances — independent of the
    //    FIQ8 enable. Sampled from CPU cycles so it stays monotonic with the timebase.
    //
    //  - The FIQ5 OVERFLOW INTERRUPT fires on each 16-bit wrap, but ONLY while the
    //    Timer1/FIQ8 overflow interrupt is enabled in IO_PUP_FIQ8 (0x20016 bit0 EN set,
    //    bit2 MSK clear). The 3310 ROM NEVER writes 0x20016 — it relies on the reset
    //    default having it ENABLED (see mad2_init). So fiq8_ctrl powers up ENABLED and
    //    FIQ5 fires on every Timer1 wrap, matching real HW.
    //
    //    Why this matters: the firmware's FIQ5 handler 0x2E4418 -> 0x298F68 walks the
    //    soft-timer list (its time base is the Timer0 counter), ages the timeout timers,
    //    and turns the backlight LEDs off after the inactivity window. NOTHING else drives
    //    that walk (the FIQ4/Timer0 handler 0x2E4426 runs a different worker, 0x298B20),
    //    so FIQ5 is required for the LED timeout. The counter + wrap are derived from the
    //    MONOTONIC rtc_mono accumulator (NOT the raw `cycles` arg — the web caller passes
    //    cpu->cycles, which the core periodically rebases down by 0x20000000; using it
    //    directly made Timer0/Timer1 jump backwards every ~41s, glitching anything that
    //    reads them as an elapsed-time base, e.g. the stopwatch and the LED-timeout walk).
    // PERF: cached 16-bit-wrap TARGET — per-tick is a single compare (no per-instruction
    // mul+div). The Timer1 counter is computed lazily on READ (mad2_t1_value, IO_SLEEP_CTR);
    // the next wrap boundary is recomputed (ceil) only AT the edge. Byte-identical: the wrap
    // fires at the first instruction boundary where floor(rtc_mono*t1_hz/ARM_HZ) crosses the
    // next 65536 multiple — exactly the old `c1 < prev1` crossing. ovf_en gates the FIQ, but
    // the deadline always advances (ovf_en-independent), so toggling it behaves as before.
    if (m->rtc_mono >= m->t1_wrap_next) {
        if (m->t1_ovf_en) { m->t1_overflows++; mad2_raise_fiq(m, 5); }   // FIQ5 (0x2E4418)
        uint64_t counts = (m->rtc_mono * (uint64_t)m->t1_hz) / DCT3_ARM_HZ;
        uint64_t next_boundary = ((counts >> 16) + 1u) << 16;            // next 65536-count wrap
        m->t1_wrap_next = (next_boundary * DCT3_ARM_HZ + m->t1_hz - 1u) / m->t1_hz;  // ceil -> rtc_mono
    }
    // FIQ8 ct_timer: a 100 Hz periodic the firmware enables on demand (0x2E6D8C: 0x20016
    // EN=1, MSK=0) to drive the centisecond soft-timers — the stopwatch up-counter
    // [0x11139C] and countdown [0x111398], serviced by handler 0x2E6DD4. While enabled,
    // set the hardware ACT bit (bit1) on each centisecond boundary so the FIQ dispatch
    // loop 0x2E6FBE runs the handler; the handler acks ACT (write-1-to-clear) and
    // mad2_fiq_poll asserts the FIQ until then. 1 centisecond = 13e6/100 cycles, derived
    // from the MONOTONIC rtc_mono accumulator (not the rebased `cycles`). Disabled at
    // boot (MSK set), so this is dormant until the firmware starts a stopwatch/countdown.
    if ((m->fiq8_ctrl & 0x01) && !(m->fiq8_ctrl & 0x04)) {
        uint64_t cs = m->rtc_mono / (DCT3_ARM_HZ / (uint64_t)m->fiq8_hz);
        if (cs != m->fiq8_last_cs) {
            m->fiq8_last_cs = cs;
            m->fiq8_ctrl |= 0x02;     // ct_timer fired -> ACT pending (FIQ8)
            m->fiq8_ticks++;
        }
    }
    // Watchdog starvation diagnostic + (optional, armed-only) time-out reset. The
    // firmware kicks the dog regularly; if no kick arrives within the time-out window
    // (the real CCONT WDT is 49 s at the RE-confirmed reload value 0x31), record a
    // starvation. We do
    // NOT reset the CPU by default (our pacing can starve it): WDTRESET=1 latches a
    // trip. A power-off (WDT=0) deliberately stops feeding, so don't count that.
    if (m->wdt_reg != 0x00 && m->wdt_cc != 0x00 && !m->power_off && !m->wdt_disabled) {
        // 49 s window on rtc_mono — the RE-confirmed real-HW CCONT WDT timeout
        // (D-09): CCONT_RESET_WDT 0x2EEB94 reloads WDReg with 49 (0x31) every kick,
        // WDReg is denominated in seconds (0x20=32 s default). Was a 6 s placeholder
        // (~8x too short); the firmware kicks ~every 1.49 s so 49 s leaves ~47 s of
        // margin. Measured on rtc_mono (NOT cpu->cycles, which rebases every ~40 s).
        if (m->rtc_mono - m->wdt_last_kick_cyc > m->wdt_window_cyc) {
            m->wdt_starved++;
            m->wdt_last_kick_cyc = m->rtc_mono;   // re-window so we count once per lapse
            if (m->wdt_reset_armed) m->wdt_tripped = 1;  // armed only; no actual halt here
        }
    }
    // Clean power-off (ccont_poweroff wrote WDT/reg05 = 0x00). Surface it as state and
    // treat it as the faithful NVRAM-flush point. We do NOT halt during normal boot:
    // POWEROFF=1 lets a genuine power-off latch the commit (still no CPU stop here, to
    // never risk a boot regression). The web/host can poll power_off to show "off".
    if (m->power_off && !m->power_off_commit) {
        if (m->power_off_armed) m->power_off_commit = 1;  // flush committed (diagnostic)
    }
    if (m->fiq3_delay && --m->fiq3_delay == 0) mad2_raise_fiq(m, 3);  // MBUS TxD-empty kickoff (FIQ3)
    // MBUS transmit engine: when the loaded byte finishes shifting, the TX register
    // goes empty; while the transmitter is enabled this raises FIQ2 (the MBUS event
    // ISR), which loads the next byte. FIQ2 de-asserts when the firmware writes the
    // next byte (clears mbus_txe) or disables TX (clears control bit5), so it fires
    // exactly once per byte and not after the final byte (no storm).
    if (m->mbus_tx_delay && --m->mbus_tx_delay == 0) {
        m->mbus_txe = 1;
        if (m->mbus_tx_pending) {       // the loaded byte finished shifting -> emit to the host ring
            m->mbus_tx_pending = 0;
            if ((uint8_t)(m->mbus_tx_out_tail - m->mbus_tx_out_head) < sizeof(m->mbus_tx_out)) {
                m->mbus_tx_out[m->mbus_tx_out_tail++ % sizeof(m->mbus_tx_out)] = m->mbus_tx_byte;
                m->mbus_tx_bytes++;
            }
        }
    }
    // FIQ2 is the single MBUS event line: it asserts for EITHER TX-empty (while the
    // transmitter is enabled, ctrl bit5) OR RX-data-ready (while the receiver is enabled,
    // ctrl bit6) — the firmware's FIQ2 ISR reads status 0x19 to tell which (bit4 TXE,
    // bit5 RXDRDY). Modelling only the TX half meant an injected RX byte's FIQ2 (set via
    // mbus_rx_signal / the boot_trace MBUSFRAME path) was torn down by this same tick
    // before the per-byte RX ISR could consume it, so multi-byte service frames never
    // assembled. RXDRDY (mbus_rxdrdy) is set only while injected bytes wait and clears on
    // FIFO drain (mbus_rx_pop), so at normal boot the RX term is always 0 → byte-identical.
    if ((m->mbus_txe   && (m->mbus_ctrl & 0x20)) ||
        (m->mbus_rxdrdy && (m->mbus_ctrl & 0x40))) mad2_raise_fiq(m, 2);
    else                                           mad2_ack_fiq(m, 2);
    // Periodic scheduler tick (FIQ4). Edge-detect a tick boundary and latch FIQ4;
    // the handler 0x2E4426 acks it via [0x20008]=0x10. Derive the boundary from the
    // MONOTONIC rtc_mono accumulator, NOT the raw `cycles` arg — the core rebases
    // cycles down by 0x20000000 periodically, which made this heartbeat phase-JUMP
    // at every rebase relative to the soft-timer walk (whose time base is the
    // rtc_mono-derived Timer0). That phase discontinuity let FIQ4 preempt the walk
    // mid-operation and corrupt the heap free-list (the low-battery idle wedge).
    if (m->fiq4_period) {
        uint32_t t = (uint32_t)(m->rtc_mono / m->fiq4_period);
        if (t != m->fiq4_lasttick) { m->fiq4_lasttick = t; mad2_raise_fiq(m, 4); }
    }
    // Body-skip rejoin: a quiescent tick jumps here, bypassing the edge body above.
  edge_skip:
    // DSP pump (mailbox acks, IRQ4 generation, self-test + boot-msg injection) is
    // per-model behaviour (DspOps; see models/model.h, default in src/mad2/dsp_default.c).
    { const DspOps* d = m->dsp_override ? m->dsp_override : m->model->dsp;
      if (d && d->tick) d->tick(m); }
    uint64_t d = (uint64_t)m->t0_div + 1;
    uint16_t c = (uint16_t)((m->rtc_mono * DCT3_T0_HZ) / (DCT3_ARM_HZ * d) + m->t0_offset);   // monotonic base + divider-continuity offset (see Timer1 note)
    uint16_t prev = m->t0_last;
    m->t0_last = c;
    m->t0_counter = c;
    uint16_t span = (uint16_t)(c - prev);
    if (span) {
        uint16_t to_dest = (uint16_t)(m->t0_dest - prev);
        if (to_dest != 0 && to_dest <= span) {
            // Timer0 expiry drives the OS scheduler tick, which is FIQ4 (firmware-verified:
            // FIQ4 handler 0x2E5276 acks the FIQ-active reg I/O 0x08 bit4 and runs the timer
            // queue 0x299838) — NOT IRQ4, which is the DSP mailbox (handler 0x2BCEF0). The old
            // code raised IRQ4 here, which spuriously ran the DSP handler, contaminated the DSP
            // interrupt line, and set the self-test verdict-fail flag [0x11079C] (= why the boot
            // spike was needed). We don't re-raise: the scheduler already runs on the fiq4_period
            // FIQ4 heartbeat. (Blacksphere §8 = FIQ4 Timer0 / IRQ4 DSP; §3's "Timer0->IRQ4" is
            // the wrong line.) Env: T0IRQ4=1 restores the legacy raise; T0FIQ4=1 also pulses FIQ4.
            // FAITHFUL DEFAULT: raise FIQ4 on the real Timer0-destination match (the OS
            // scheduler tick — handler 0x2E5276). T0FIQ4=0 disables it (legacy heartbeat-only
            // A/B); T0IRQ4=1 restores the old buggy IRQ4 raise (contaminated the DSP line).
            if (getenv("T0IRQ4")) mad2_raise_irq(m, 4);
            else { const char* t = getenv("T0FIQ4"); if (!t || strcmp(t, "0")) mad2_raise_fiq(m, 4); }
        }
    }
}

int mad2_irq_poll(const Mad2* m) {
    // Master IRQ disable (0x2000C bit3): the firmware's DISABLE_IRQ (0x2E4116) / heap-free guard
    // (0x299A36) write 0x2000C=0x0A (bit1=FIQ-off, bit3=IRQ-off) — the function is named DISABLE_IRQ,
    // so it blocks IRQ too. Honor it so IRQ4 (DSP mailbox) / IRQ2 (CCONT) cannot preempt a guarded
    // heap operation, same fault class as the FIQ8 stopwatch UAF (faithful default; INTCTRL_GATE=0 reverts).
    if (m->intctrl_gate && (m->int_ctrl & 0x08)) return -1;
    uint8_t eff = (uint8_t)(m->irq_pending & ~m->irq_mask);
    if (!eff) return -1;
    for (int i = 0; i < 8; ++i) if (eff & (1u << i)) return i;
    return -1;
}

// Lowest unmasked pending FIQ (0..7), or -1. FIQ3 (MBUS TxD-ready) is asserted
// edge-style when the firmware unmasks it (see mad2_write IO_FIQ_MASK): we send
// instantly, so TxD-complete fires once; the TxD handler (0x2E35DC) then re-masks
// FIQ3 and finishes the transmit.
int mad2_fiq_poll(const Mad2* m) {
    // Master FIQ disable (0x2000C bit1): the firmware's DISABLE_IRQ / heap-free critical sections
    // write 0x2000C=0x0A to stop the interrupt controller asserting FIQ. Honor it so FIQ8 (and any
    // pending FIQ) cannot preempt a guarded heap operation (faithful default; INTCTRL_GATE=0 reverts).
    if (m->intctrl_gate && (m->int_ctrl & 0x02)) return -1;
    uint8_t eff = (uint8_t)(m->fiq_pending & ~m->fiq_mask);
    for (int i = 0; i < 8; ++i) if (eff & (1u << i)) return i;
    // FIQ8 (ct_timer) is a 9th source NOT in the 0x20008 byte: it is pending when its
    // ctrl reg 0x20016 has ACT (bit1) set and MSK (bit2) clear. The dispatch loop
    // 0x2E6FBE services it (handler 0x2E6DD4) after draining FIQ0-7. Callers only test
    // the result >= 0 (to raise the FIQ), so the sentinel 8 is fine.
    if ((m->fiq8_ctrl & 0x02) && !(m->fiq8_ctrl & 0x04)) return 8;
    return -1;
}

