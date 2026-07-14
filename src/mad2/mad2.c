// MAD2 platform device model — see mad2.h.

#include "mad2/mad2.h"
#include "mad2/mad2_internal.h"
#include "mad2/dsp_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// SIM bridge (real-card shadow comparison). Native-only: the host-side ISO-7816
// T=0 driver uses POSIX serial, so it's compiled out of the wasm build. Activated
// at runtime by the SIMBRIDGE=<serial-dev> env var. See docs/sim-bridge-protocol.md.
#ifndef __EMSCRIPTEN__
#include "sim_bridge.h"
#endif

// The post-mortem caller-LR symbol-table fallback (symbols_3310_v579.h) now
// lives with the attribution code in mad2_diag.c.

// The MMIO window + IO_*/GENSIO register offsets and the DCT3 clock rates now live
// in mad2_internal.h — shared by the bus dispatcher, the timer tick, and the
// peripheral modules. The DSP shared-RAM mailbox addresses are per-model (m->fw).

// EF_PHASE (GSM phase byte) is defined in mad2_sim.c; the SIMPHASE env knob in
// mad2_init mutates EF_PHASE[0] before the SIM is read.
extern uint8_t EF_PHASE[1];

void mad2_init(Mad2* m, const ModelProfile* prof) {
    memset(m, 0, sizeof(*m));
    m->model = prof;
    // Remote-DSP bridge (DSP_BRIDGE env): swap the DSP backend for the transport that
    // talks to a remote DSP (another emulator's c54x co-sim, or real phone proxy FW).
    if (dsp_bridge_enabled()) { m->dsp_override = &mad2_dsp_bridge;
        fprintf(stderr, "[dspb] DSP backend = remote bridge\n"); }
    m->fw    = prof->fw;   // constant fallbacks; the shell overlays signature hits via model_resolve()
    // Healthy power readings so the firmware doesn't abort boot on a flat battery.
    // Sourced from the model profile's CCONT A/D reset defaults (per-model pack).
    // For the 3310: BSI = Battery Size Indicator (a fixed resistor in the pack); 0x26
    // (=38) models a genuine NiMH BMC-3 pack — the firmware classifies BSI<100 as NiMH
    // (cf. MADos apps/charger.c, BMC-3 at 38/62/66) and takes the normal battery-type/
    // charge path. The old 0x100 (=256) fell in the firmware's narrow "service battery"
    // band [218,270] used by the early power-on check (0x2E19A0): a service-band BSI +
    // a cold BTEMP diverted boot to a path that left battery-type=0 and never drew the
    // idle UI, AND the charge state machine refused a room-temperature battery, showing
    // "Not charging" (PPM id 0x0562). A real NiMH BSI satisfies BOTH boot and charge.
    m->adc[2] = prof->battery.vbatt;    // battery voltage: good (~3.7 V)
    m->adc[3] = prof->battery.bsi;      // battery type (BSI): genuine NiMH pack (BMC-3) -> normal path
    m->adc[4] = prof->battery.temp;     // battery temperature: nominal (room temp; in the charge window)
    m->adc[5] = prof->battery.charger;  // charger voltage: none connected
    m->fiq_mask = 0xFF;  // all interrupts masked at reset; firmware enables as needed
    m->irq_mask = 0xFF;
    m->cc_int_mask = 0xFF;  // CCONT cascade powers up all-masked; firmware writes 0xF0 (enables CHARGER/INT3)
    // CCONT power-on cause (reg 0x0E): a cold power-on = the user holding the PWR key, which
    // on models that read the power-on reason from the CCONT (PWRONX) latches a cause bit the
    // startup classifier requires — without it the firmware sees "no valid power-on reason"
    // and powers straight back off. 0 for keypad-driven models. (See ModelProfile.ccont_poweron_int.)
    m->cc_int_lines = prof->ccont_poweron_int;
    { const char* pob = getenv("POBIT"); if (pob && *pob) m->cc_int_lines = (uint8_t)strtoul(pob,0,0); } // A/B power-on-cause override
    m->reset_ctrl = 0x01; // 0x20001 bit0 = powerup (cold power-on reset latch)
    // Reset-reason recovery policy: DEFAULT-ON for every reason byte, with explicit
    // exclusions for ones we know shouldn't silently recover. Most firmware panics are
    // task-driven (GSM/SIM bring-up, broker fault-notifies) — the offending message has
    // been drained from the queue by the time the reset fires, so resuming at the next
    // instruction lets the scheduler keep ticking. New unknown reasons recover
    // automatically rather than killing the session; the [reset] CATCH log makes them
    // visible. Confirmed recoverable in practice: 5, 12, 0x68 (SWDSP=DSP reset), 0x73 (task 2
    // fault-notify). Env knob RESET_NORECOVER=N,M opts out of specific reasons;
    // RESET_RECOVER=N,M flips an excluded one back on.
    //
    // Exclusions:
    //   0    — sentinel / no reason byte (reboot_reason signature missed; RAM = garbage)
    //   4    — factory reset (intentional user action; warm-reboot is correct)
    //   6    — FIQ-canary (usually OUR emu fidelity bug; silently recovering masks it)
    for (int i = 0; i < 256; i++) m->recover_reasons[i] = 1;
    m->recover_reasons[0] = 0;
    m->recover_reasons[4] = 0;
    m->recover_reasons[6] = 0;
    m->recover_enabled = 1;   // master gate on by default; web UI toggle can flip it off
    {
        const char* s = getenv("RESET_NORECOVER");
        if (s) {
            while (*s) {
                while (*s == ',' || *s == ' ') s++;
                if (!*s) break;
                unsigned long r = strtoul(s, (char**)&s, 0);
                if (r < 256) m->recover_reasons[r] = 0;
            }
        }
        s = getenv("RESET_RECOVER");
        if (s) {
            while (*s) {
                while (*s == ',' || *s == ' ') s++;
                if (!*s) break;
                unsigned long r = strtoul(s, (char**)&s, 0);
                if (r < 256) m->recover_reasons[r] = 1;
            }
        }
        // Per-reason extra-pop count (multi-frame stack walk for teardown-fn skipping).
        // RESET_POP=R:N,R:N — for reason R, pop N additional frames past the entry-LR's
        // frame before resuming. Use when the immediate LR-return lands in a die-gracefully
        // fn and we need to skip back further. 0 = plain LR-return (current default).
        // Defaults from in-browser testing:
        //   0x68 (SWDSP=DSP reset): caller is a teardown fn, pop 1 extra frame to skip it.
        m->recover_pop[0x68] = 1;
        s = getenv("RESET_POP");
        if (s) {
            while (*s) {
                while (*s == ',' || *s == ' ') s++;
                if (!*s) break;
                unsigned long r = strtoul(s, (char**)&s, 0);
                if (*s == ':') {
                    s++;
                    unsigned long n = strtoul(s, (char**)&s, 0);
                    if (r < 256 && n <= 255) m->recover_pop[r] = (uint8_t)n;
                }
            }
        }
    }
    m->flash_vpp  = 1;    // flash programming voltage modelled always-on (see 0x33 note)
    // Watchdog: passive by default (model the registers + accept kicks, never reset
    // the CPU on time-out — our emulated timing can starve it). WDTRESET=1 arms a
    // real time-out reset (for RE; risks a boot regression, hence off by default).
    m->wdt_reset_armed = getenv("WDTRESET") ? 1 : 0;
    // Starvation window default = 49 s (the 0x31 kick reload); wdt_kick overrides it from the
    // actual written reload value. Defensive: ensures a sane window even if the gate is ever
    // reached before a kick (the memset-0 would otherwise make a 0-cycle window trip instantly).
    m->wdt_window_cyc = 49ull * DCT3_ARM_HZ;
    // Two DISTINCT timer interrupts, de-conflated (Blacksphere §8: FIQ5 = Timer1/sleep-
    // counter 0x04 overflow; FIQ8 = ct_timer, ctrl reg 0x20016):
    //
    //  - FIQ5 (Timer1 overflow -> handler 0x2E4418 -> 0x298F68): drives the soft-timer
    //    LIST walk that ages the timeout timers and turns the backlight LEDs off after
    //    the inactivity window. Nothing else drives that walk, so FIQ5 must fire. Enable
    //    it here (t1_ovf_en=1); it is still gated by the 0x2000A FIQ mask, which the
    //    firmware controls. (Proven required: WATCH=0x298F68 -> 0 hits with FIQ5 off, 11
    //    with it on.) This was previously (wrongly) tied to the 0x20016 EN bit.
    m->t1_ovf_en = 1;
    //  - FIQ8 (ct_timer, 0x20016 -> handler 0x2E6DD4): a 100 Hz periodic the firmware
    //    enables ON DEMAND (timer_enable 0x2E6D8C: 0x20016 EN=1, MSK=0) to drive the
    //    centisecond soft-timers — the stopwatch up-counter 0x11139C and the countdown
    //    0x111398. The ct_timer powers up DISABLED (MSK set); the firmware enables it
    //    when a soft-timer is armed and disables it (0x20016=4) when the last one stops.
    //    mad2 fires it in mad2_timers_tick while enabled (see the FIQ8 ct_timer block).
    m->fiq8_ctrl = 0x04;  // ct_timer (FIQ8) MASKED/disabled at reset; firmware enables on demand
    // Clean power-off: surface state only by default. POWEROFF=1 lets a genuine
    // WDT=0/reg05=0 power-off latch a stop+NVRAM-flush (off so boot can't self-halt).
    m->power_off_armed = getenv("POWEROFF") ? 1 : 0;
    // Service-mode watchdog inhibit (Nokia diagnostics behaviour). WDTSERVICE=1
    // at boot, OR call dct3_web_set_wdt_service(1) at runtime: WDT=0 writes are
    // logged but no longer latch power_off, so a service engineer can keep the
    // phone running while fault-finding. Off by default so normal boots and
    // user-driven shutdowns behave faithfully.
    m->wdt_service_mode = getenv("WDTSERVICE") ? 1 : 0;
    m->wdt_inhibited_count = 0;
    // Eager panic-chain intercept. Off by default
    // — the existing `[0x20001]|=4` late catch fires as before, byte-identical behaviour.
    // RESET_EARLY=1 arms the entry-time intercept (reboot_fn + fatal_handler hooks).
    m->reboot_early = getenv("RESET_EARLY") ? 1 : 0;
    // RTC alarm INT7: passive by default. Asserting INT7 -> IRQ2 wedges the CPU because
    // no scheduled task acks bit7 in our OS bring-up (level-driven IRQ2 re-fires forever
    // -> infinite ISR spin). RTCALARM=1 arms the assertion (with an un-ack auto-release
    // safety); off by default so a 0x0B/0x0C arm + INT7 unmask can never hang the boot.
    m->rtc_alr_enabled = getenv("RTCALARM") ? 1 : 0;
    // No accessory in the pop-port / headset jack (deterministic "nothing connected").
    m->accessory_present = 0;
    // Power key held: the user powers on by holding PWR. The keypad scanner reads
    // the power key in its "special" pass (row-direction 0xE0) as code 0x80+col;
    // col 1 (=129) is the power key. Holding it makes the firmware derive a valid
    // power-on reason instead of powering back off. (A real session should release
    // it after the boot debounce via keypad input.)
    m->kbd_special_cols = prof->keypad.power_special_cols;
    m->kpd_im_status = 0;   // 8850-class keypad interrupt-source status (I/O 0x2B); see mad2_keypad_irq
    m->kpd_src34 = 0;       // later-serial (8810) keypad matrix interrupt-source (I/O 0x34); see mad2_keypad_irq
    m->slide_open = 0;        // slide phones boot with the cover closed (rest state)
    m->cover_int_pending = 0; // reed-switch interrupt latch (I/O 0x29 bit0)
    // OS scheduler heartbeat (FIQ4). ~5 ms at the ~13 MHz MAD ARM clock; fast
    // enough to age software timers and re-poll the readiness barrier, slow enough
    // not to drown the CPU in ISR overhead. (Tunable; see mad2.h.)
    // FIQ4 scheduler tick: FAITHFUL DEFAULT = the real Timer0-destination match (see the
    // Timer0->FIQ4 block in mad2_timers_tick), NOT a synthetic heartbeat. The old ~65000-cycle
    // heartbeat fired FIQ4 ~27x too often (measured: 33k vs 1.2k FIQ early), phase-jumped on the
    // cycle-rebase, and preempted the heap soft-timer walk -> free-list corruption (the long-run
    // wild-PC residual; clean past 9B once removed). Default 0 = heartbeat OFF. FIQ4HB=<cyc>
    // restores the legacy heartbeat for A/B (old behaviour = FIQ4HB=65000 + T0FIQ4=0).
    m->fiq4_period = 0;
    { const char* hb = getenv("FIQ4HB"); if (hb && *hb) m->fiq4_period = (uint32_t)strtoul(hb, 0, 0); }
    // RTC-MIN (INT5) cached target: first minute boundary (1 min = 13e6 cyc/s * 60). The
    // per-instruction tick compares rtc_mono against this; recomputed at each edge (see mad2_timers).
    m->rtc_min_next = 13000000ull * 60ull;
    // Timer1/CTSI tick rate. Faithful default = DCT3_T0_HZ (33055 Hz), so FIQ5 fires on
    // each 16-bit wrap (~every 2 emu s). T1HZ=<hz> A/B knob: MAME models the CTSI timebase
    // at ~1057 Hz, which stretches the FIQ5-overflow cadence ~31x (~62 emu s/wrap). Under
    // test: is the FIQ5-driven soft-timer-walk cadence (handler 0x2E4418 -> 0x298F68 ages +
    // frees expired timer nodes) the residual free-list-poison driver, the same class of bug
    // as the FIQ4 over-fire we already fixed? If the long-run wild-PC fuse scales with this
    // rate, FIQ5 is confirmed. The walk's AGING time base is Timer0 (unchanged), so slowing
    // Timer1 changes only how OFTEN the walk runs, not the timeout math.
    m->t1_hz = (uint32_t)DCT3_T0_HZ;
    { const char* h = getenv("T1HZ"); if (h && *h) m->t1_hz = (uint32_t)strtoul(h, 0, 0); }
    // Timer1 first 16-bit-wrap target (ceil(65536 counts -> rtc_mono cycles)). The per-
    // instruction tick compares rtc_mono against this; recomputed at each wrap (see mad2_timers).
    m->t1_wrap_next = (65536ull * DCT3_ARM_HZ + m->t1_hz - 1u) / m->t1_hz;
    // FIQ8 ct_timer (centisecond) rate. The stopwatch is a SINGLE centisecond up-counter
    // [0x11139C] incremented +1 per FIQ8 (handler 0x2E6DD4); seconds/minutes/hours are DERIVED
    // in the display formatter 0x2DEFC4 by dividing by 100/6000/360000 — there is NO independent
    // time base. So FIQ8's rate sets BOTH the displayed speed AND how often the centisecond
    // soft-timer handler (per-tick redraw + EEPROM churn) runs. Default 100 Hz. T8HZ=<hz> A/B knob:
    // under test = is FIQ8 firing too fast the driver of the stopwatch-running wild-PC (the same
    // class as the FIQ4 over-fire)? If the crash step scales inversely with this rate, confirmed.
    m->fiq8_hz = 100;
    { const char* h = getenv("T8HZ"); if (h && *h) m->fiq8_hz = (uint32_t)strtoul(h, 0, 0); }
    // FAITHFUL DEFAULT (on): honor the 0x2000C master interrupt-disable in FIQ delivery. The
    // firmware's DISABLE_IRQ (0x2E4116) + inline heap-free guard (0x299A36) write 0x2000C=0x0A
    // (bit1=FIQ-off, bit3=IRQ-off) to enter a critical section — purely via this register, NOT
    // CPSR.F. mad2's delivery historically IGNORED 0x2000C, so FIQ8 (which also bypasses the
    // per-source mask 0x2000A) preempted guarded heap frees -> reentrant free-list corruption ->
    // the long-chased wild-PC UAF (0xAAAAAAA8/0xFFFFFFFE), reproducible fast via the running
    // stopwatch (tools/crash-repro/stopwatch-native.sh). Honoring it defers the FIQ until the
    // critical section ends (faithful: deferred, not dropped). A/B-revert: INTCTRL_GATE=0 restores
    // the old (buggy) ignore-0x2000C behaviour.
    m->intctrl_gate = 1;
    { const char* g = getenv("INTCTRL_GATE"); if (g && *g) m->intctrl_gate = (uint8_t)(atoi(g) != 0); }
    m->lcd_mode = 2;     // PCD8544 powers up configured to normal by the firmware's init
    // SIM card present by default (a modeled GSM test SIM). The firmware's SIM
    // driver detects it via the SIMI UART (ATR on activate) + reads its EFs over
    // T=0, clearing the disp49 SIM gate faithfully. Toggle off to model "no SIM".
    m->sim_present = getenv("SIMABSENT") ? 0 : 1;
    // SIMPHASE env knob: override EF_PHASE byte. 0=Phase 1, 2=Phase 2 (default), 3=Phase 2+.
    { const char* ph = getenv("SIMPHASE"); if (ph && *ph) EF_PHASE[0] = (uint8_t)atoi(ph); }
    // CHV1 (PIN): default = active SIM with CHV1 DISABLED (no PIN prompt) — a convenience
    // for the emulated/fake SIM so boot reaches the live UI without a PIN entry. Knobs:
    // SIMPINON=1 enables CHV1 (PIN "1234", PUK "12345678", 3/10 tries) to exercise the PIN
    // path; SIMPINOFF=1 forces it off; SIMPIN/SIMPUK set the codes.
    {
        const char* pin = getenv("SIMPIN"); if (!pin || !*pin) pin = "1234";
        const char* puk = getenv("SIMPUK"); if (!puk || !*puk) puk = "12345678";
        memset(m->sim_pin, 0xFF, sizeof m->sim_pin);
        memset(m->sim_puk, 0xFF, sizeof m->sim_puk);
        for (int i = 0; i < 8 && pin[i]; ++i) m->sim_pin[i] = (uint8_t)pin[i];
        for (int i = 0; i < 8 && puk[i]; ++i) m->sim_puk[i] = (uint8_t)puk[i];
        m->sim_pin_enabled  = (getenv("SIMPINON") && !getenv("SIMPINOFF")) ? 1 : 0;  // default: no PIN
        m->sim_pin_verified = 0;
        m->sim_pin_tries    = 3;
        m->sim_puk_tries    = 10;
    }
    // SIMWWT: periodic raise of SIMI_UART_INT bit 5 (= ISO-7816 T=0 Work Waiting Time
    // timeout → status 6 to task 19 → retry-counter bumper at 0x29D750). Without this,
    // our instant-responding SIM never makes the firmware's SIM Server see the WWT
    // heartbeat, and the SIM driver's hard 7s timeout reboots the phone at ~110M cyc.
    // SIMWWT=1: default 125 000 cyc (~9.6 ms at 13 MHz = exactly the ISO-7816 WWT @
    // standard rate). SIMWWT=<N>: explicit cycle count. Unset/0: disabled (baseline).
    {
        const char* w = getenv("SIMWWT");
        if (w && *w) {
            uint64_t n = (uint64_t)strtoull(w, NULL, 0);
            m->sim_wwt_threshold_cyc = (n == 1) ? 125000ull : n;
        } else {
            m->sim_wwt_threshold_cyc = 0;   // disabled
        }
        m->sim_wwt_last_active_cyc = 0;
    }
    // (default here so a pre-init read isn't a blank screen; the firmware sets it anyway)
    // DSP runtime boot-indication injector (brute-force; see mad2.h). DSPMSG="t[,l,b..]"
    if (getenv("DSPMSG")) {
        const char* p = getenv("DSPMSG");
        m->dsp_msg_en = 1;
        m->dsp_msg_type = (uint8_t)strtoul(p, (char**)&p, 0);
        if (*p == ',') { p++; m->dsp_msg_len = (uint8_t)strtoul(p, (char**)&p, 0); }
        for (int i = 0; i < m->dsp_msg_len && i < 32 && *p == ','; ++i) {
            p++; m->dsp_msg_body[i] = (uint8_t)strtoul(p, (char**)&p, 0);
        }
        m->dsp_msg_delay = getenv("DSPMSGDELAY") ? (uint32_t)strtoul(getenv("DSPMSGDELAY"), NULL, 0) : 200000;
    }
    // Faithful-path default: the real C54x co-sim (DSP54_COSIM) supplies the DSP behaviour, so the
    // HLE self-test responder must be OFF. Explicit DSPNOSELFTEST=0/1 still wins; else default ON
    // under cosim (mirrors dsp54_faithful() in the c54x glue — kept inline to avoid a core->c54x dep).
    { const char *e = getenv("DSPNOSELFTEST");
      m->dsp_selftest_off = (e && *e) ? (atoi(e) != 0) : (getenv("DSP54_COSIM") ? 1 : 0); }
    // HLE master quiet-gate (DSP54_HLE=1 opts the legacy model back in for A/B): under the co-sim
    // the real DSP supplies EVERY DSP->MCU signal, so the whole HLE tick — block-ack pump + its
    // IRQ4 raise, Cobba command auto-consume, self-test responder, DSPMSG injector, keep-alive
    // FIQ0 stream — must be silent. Only the free-running dsp_steps counter survives. Without
    // this, the pump's IRQ4 and the HLE's m->mem mailbox echoes are FAKED DSP signals racing the
    // real core's organic ones.
    { const char *e = getenv("DSP54_HLE");
      m->dsp_hle_quiet = (e && *e) ? (atoi(e) == 0) : (getenv("DSP54_COSIM") ? 1 : 0); }
    // Eager-load the external I2C EEPROM (baked NokiX blob) for serial-bus models so its
    // calibration is in place before boot AND the web persistence layer can overlay a saved
    // image right after init (a lazy first-access load would race that overlay). No-op for
    // in-flash-EEPROM models (i2c_eeprom_default == NULL), keeping them byte-identical.
    if (prof->i2c_eeprom_default) ext_eeprom_init(m);
}

// --- Root-cause attribution (assertion ring + post-mortem) + eager panic-chain intercept -> mad2_diag.c ---

// --- CCONT (power/RTC/ADC) + Watchdog -> mad2_ccont.c ------------------------

// --- Flash chip (Intel/Sharp CFI command set) -> mad2_flash.c ----------------

// --- MBUS receive -> mad2_mbus.c ---------------------------------------------

// --- Keypad matrix -> mad2_keypad.c ------------------------------------------

// --- PCD8544 LCD -> mad2_lcd.c -----------------------------------------------

// --- SIM card (SIMI UART + ISO-7816 T=0 + GSM 11.11 EF tree + real-card bridge) -> mad2_sim.c ---

// --- I/O hook entry points (MMIO bus dispatcher) -> mad2_bus.c ----------------
// --- CTSI interrupt controller + Timer0/1 + FIQ8 + tick/poll -> mad2_timers.c ---
// --- Framebuffer output -> mad2_lcd.c ----------------------------------------
