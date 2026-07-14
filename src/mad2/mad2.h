// MAD2 platform device model (Phase 2/3).
//
// Implements the peripheral behaviour the firmware needs to get past hardware
// bring-up, behind the dct3_core I/O hooks. Memory is still RAM-backed by the
// core; this layer overrides reads and observes writes for the device registers
// in the primary I/O window (0x20000) plus the GENSIO-attached chips (CCONT
// power controller, PCD8544 LCD) that hang off it.
//
// Registers/protocol are from Project Blacksphere.

#ifndef MAD2_H
#define MAD2_H

#include <stdint.h>

#include "models/model.h"   // ModelProfile, FwAddrs, LCD_MAX_*

typedef struct Mad2 {
    int verbose;                // log decoded CCONT/LCD transactions

    // Active model profile (geometry, ASIC config, memory map, boot knobs) and the
    // firmware addresses resolved against the loaded image (signatures + fallbacks).
    // Set by mad2_init(); the shell fills `fw` via model_resolve() after loading flash.
    const ModelProfile* model;
    // Optional per-instance DSP backend override (e.g. the remote-DSP bridge, selected
    // at runtime via DSP_BRIDGE). NULL = use the model's default DSP ops.
    const struct DspOps* dsp_override;
    FwAddrs  fw;

    // Flash chip (Intel/Sharp command set). The array is the core's RAM backing
    // (code executes from it directly), so the device programs into `mem`.
    uint8_t* mem;               // core RAM backing (set by harness/web shim)
    uint32_t mem_mask;          // address mask for `mem`
    uint8_t  flash_mode;        // 0 = read-array, 1 = read-status, 2 = read-id (CFI/Intel)
    uint8_t  flash_prog;        // 1 = next flash write programs a word
    uint8_t  flash_erase;       // 1 = erase command armed (await 0xD0 confirm)
    uint8_t  flash_sr;          // Intel status register (SR.7 ready, SR.3/4/5 error bits)
    uint8_t  flash_busy;        // remaining read-status polls before SR.7 reasserts (busy->ready edge)
    uint8_t  flash_vpp;         // Vpp programming voltage enabled (IO_UIF_CTRL3 0x33 bit0)
    uint64_t flash_programs;    // count of word programs
    uint64_t flash_erases;      // count of block erases
    uint32_t flash_codewrite;   // DIAG: last program/erase below eeprom_base (non-EEPROM region; legit for a custom FFS in freed MCU/PPM space — informational, not blocked)
    uint64_t flash_cmds;        // DIAG: count of all command writes to the flash region
    uint64_t flash_eeprom_programs; // DIAG: programs landing in the EEPROM partition (>=0x3D0000)
    uint32_t flash_last_cmd_addr;   // DIAG: last flash command/program target address
    uint32_t flash_csr_addr;        // addr of the last command write (status-poll CSR). The
                                    // 3410 flash driver writes its program/erase command to a
                                    // fixed CSR (0x3FFF00) BELOW eeprom_base then polls SR.7
                                    // there — so read-status must be served at this address,
                                    // not only in the EEPROM partition.

    // GENSIO / CCONT transaction state.
    int     ccont_have_addr;    // a register address has been latched
    uint8_t ccont_addr;         // selected CCONT register (0..15)
    uint8_t  ccont[16];         // our CCONT register model
    uint16_t adc[8];            // CCONT A/D channels (2=Vbatt, 3=type, 4=temp, 5=charger)
    int      adc_channel;       // A/D channel selected via CCONT reg00

    // CCONT RTC (regs 0x07 sec / 0x08 min / 0x09 hour / 0x0A day). The firmware reads
    // these to display the clock and to time the backlight-off (~30 s) — a static RTC
    // freezes the clock AND leaves the lights stuck on. Advanced from a monotonic cycle
    // accumulator: now = base (captured when the firmware writes the regs) + elapsed.
    uint64_t rtc_mono;          // monotonic cycle accumulator (undoes the core's rebase)
    uint32_t rtc_last_cyc;      // last raw cpu->cycles seen (for the delta)
    uint64_t rtc_base_cyc;      // rtc_mono snapshot when the clock was last set
    uint32_t rtc_base_sec;      // wall time at that snapshot, as total seconds

    // CCONT interrupt cascade -> MAD IRQ2. CCONT reg 0x0E = pending interrupt lines
    // (bit0 RTC-battery-present status; bit3 CHARGER/INT3; bit4 RTC-SEC/INT4; bit5
    // RTC-MIN; bit6 RTC-DAY; bit7 RTC-ALR), reg 0x0F = mask (0=passed, 1=masked). Any
    // pending+unmasked line asserts IRQ2; the firmware's IRQ2 handler reads 0x0E and
    // writes 1s back to release each source. The 3310 firmware unmasks ONLY CHARGER
    // (it writes mask 0xF0): it software-clocks the RTC rather than taking the per-
    // second INT4, so the clock is NOT driven from here. We pulse INT3 on a charger
    // plug/unplug edge so the firmware re-reads the charger ADC and runs its charge UI.
    uint8_t  cc_int_lines;      // reg 0x0E pending (firmware clears via write-1)
    uint8_t  cc_int_mask;       // reg 0x0F (0=passed, 1=masked); reset = all masked
    uint8_t  charger_present;   // debounced charger presence, for the INT3 edge
    uint64_t vbatt_rise_last_cyc; // rtc_mono of last adc[2] increment while charging (model
                                  // a slowly-rising battery voltage so the firmware exits
                                  // the "battery full" state-machine path on plug-in)
    uint32_t rtc_last_min;      // last RTC minute index seen, for the RTC-MIN (INT5) edge
    uint64_t rtc_min_next;      // cycle-cached next-minute target (per-tick compare; recomputed at edge)
    uint64_t rtc_min_edges;     // count of RTC-MIN (INT5) interrupts raised (diagnostic)
    uint64_t rtc_writes;        // count of firmware writes to RTC regs 0x07-0x0A (diagnostic)
    uint32_t rtc_wr_pc;         // PC of the last RTC-reg write (diagnostic; find the writer)
    uint32_t cur_io_pc;         // PC of the in-progress MMIO write (for attributing ccont writes)

    // --- RTC alarm / wakeup (CCONT regs 0x0B wkup-min / 0x0C wkup-hr) --------
    // The CCONT RTC has wakeup-alarm registers: reg 0x0B = wakeup minute (&0x3F),
    // reg 0x0C = wakeup hour (&0x1F) (MADos ccont_set_wkupmin/hr, ccont.c:344-360).
    // When the running RTC time (hour:min) matches the programmed wakeup, the CCONT
    // raises INT7 (RTC-ALR, cascade bit7) -> IRQ2 like the other CCONT ints. This
    // powers an alarm/wake-at-time. NOTE: the displayed clock does not yet advance
    // (separate manager-scheduling wall) so the match is
    // armed off the FREE-RUNNING CCONT time (same base the RTC-MIN INT5 uses); the
    // alarm path is modelled correctly for when the clock advances. The firmware
    // arms it by setting 0x0B/0x0C and unmasking INT7 (reg 0x0F bit7=0).
    //
    // SAFETY: asserting INT7 -> IRQ2 is DISABLED by default (rtc_alr_enabled, RTCALARM
    // env). With our incomplete OS bring-up there is no scheduled RTC-alarm task to ACK
    // bit7, so a level-driven INT7 re-asserts IRQ2 forever (verified infinite IRQ2 ISR,
    // CPU pinned in IRQ mode). We keep the register state faithful but only PULSE INT7
    // when explicitly enabled, and even then auto-release an un-acked line after a short
    // window (rtc_alr_assert_cyc) so it can never wedge the CPU. Same gated-off rule the
    // watchdog / power-off follow. See the alarm block in mad2_timers_tick.
    uint8_t  rtc_alr_armed;     // an alarm time has been programmed (0x0B/0x0C written)
    uint8_t  rtc_alr_enabled;   // RTCALARM env: actually assert INT7 on match (default 0 = passive)
    uint8_t  rtc_alr_min;       // 0x0B wakeup minute
    uint8_t  rtc_alr_hr;        // 0x0C wakeup hour
    uint8_t  rtc_alr_fired;     // one-shot guard: alarm matched this armed window
    uint64_t rtc_alr_edges;     // count of RTC-ALR (INT7) interrupts raised (diagnostic)
    uint64_t rtc_alr_assert_cyc;// rtc_mono when INT7 was asserted (un-ack auto-release window)

    // --- Sleep / clean power-off (CCONT WDT=0 / clock-gate reg 0x0D) ---------
    // ccont_poweroff (MADos ccont.c:205) writes IO_CTSI_WDT(0x03)=0x00 + CCONT reg
    // 0x05=0x00 to power the phone down cleanly (a long power-key press). CCONT reg
    // 0x0D = clock gates (GENSIO 0x04, PROG 0x08); writing it gates peripheral
    // clocks for sleep. We model the registers + a clean-shutdown latch (a clean
    // power-off is the faithful NVRAM-flush trigger). We do NOT actually halt the
    // CPU during normal boot — power-off is surfaced as state only (POWEROFF env
    // can make a genuine WDT=0 power-off latch the flush + stop, off by default).
    uint8_t  clk_gates;         // CCONT reg 0x0D clock-gate shadow (sleep/clock control)
    uint8_t  power_off;         // clean power-off requested (WDT/reg05 = 0x00 written)
    uint8_t  power_off_commit;  // NVRAM flush committed on the clean power-off (diagnostic)
    uint8_t  power_off_armed;   // POWEROFF env: let a clean WDT=0 actually stop (default 0)

    // DSP stub: boot code-block upload handshake (two ping-pong mailbox slots).
    uint16_t dsp_ack[2];        // pending DSP->MCU ack token for slot 0/1
    uint64_t dsp_acks;          // count
    uint32_t dsp_boot_irq_ctr;  // per-model DSP ops scratch (e.g. 8850 IRQ4 pacing)

    // MCU reset control (0x20001): bit0 powerup, bit1 watchdog, bit2 SW reset.
    // The power-on-reset latch (bit0) must read set at cold boot so the firmware
    // derives a "powerup" power-on reason rather than a watchdog reset.
    uint8_t  reset_ctrl;        // 0x01
    uint8_t  reset_request;     // 0x20001 bit2 written = firmware SW-reset/reboot request
                                // (e.g. the end of a factory reset). The host honours it by
                                // warm-rebooting; mad2 only flags it (never resets the CPU
                                // itself, same rule as power_off).

    // --- Reset reason discrimination ----------
    // The reboot fn 0x2EEBAE (3310 v5.79) is a universal sink: every reset path bl's it
    // with a reason in r0, the fn persists the byte to RAM (m->fw.reboot_reason), then
    // writes [0x20001]|=4 and spins. We catch the spin via the existing reset_request
    // flag and decide what to do based on the reason byte. Reason 5 (watchdog/fatal) also
    // has full CPU state saved by the fatal handler 0x2F0BA4 at m->fw.reboot_save:
    //   [save+0] = interrupted LR (PC to resume at)
    //   [save+4] = SPSR (CPSR to resume with)
    //   [save+8] = handler CPSR (discarded)
    // Reason 5 recovery loads PC + CPSR from there and skips the spin entirely — the
    // task message that caused the panic has already been drained from its mailbox by
    // the time the panic fires, so resuming at the interrupted instruction lets the
    // scheduler keep running without the offending event.
    //
    // Other reasons (4=factory, 6=FIQ-canary, 12=task1, 0x68=SWDSP, etc.) have no
    // RAM-saved state — v1 logs them and falls through to the warm-reboot path so the
    // emulator stays runnable. v2 will add an entry hook at the reboot fn to capture
    // the original caller LR before it's clobbered, enabling LR-return for these too.
    uint8_t  reset_last_reason; // reason byte from the most recent catch (0 = none)
    uint64_t reset_total;       // count of every reset catch (any reason)
    uint64_t reset_counts[16];  // per-reason counters (index = reason & 0x0F for the
                                // common reasons 0..12; high reasons (0x68 = SWDSP/DSP) clamp
                                // to slot 0x0F. reset_last_reason has the unclamped byte.)
    uint32_t reset_last_pc;     // resume PC (reason 5: from *reboot_save; else 0)
    uint32_t reset_last_cpsr;   // resume CPSR (reason 5: from *(reboot_save+4); else 0)
    uint32_t reset_last_lr;     // diagnostic: faulting LR captured at the time of the catch
                                // (for reason 5 = the interrupted PC; same as reset_last_pc)
    uint64_t reset_recovered;   // count of catches that produced a recoverable resume
    uint8_t  recover_enabled;   // master gate: 0 = bypass the whole recovery path (every
                                // reset falls through to warm-reboot regardless of reason).
                                // Default 1. Web UI exposes a checkbox to flip it.
    uint8_t  recover_pending;   // 1 = host should call dct3_core_force_pc_cpsr() then clear
    uint32_t recover_pc;        // target PC for the resume
    uint32_t recover_cpsr;      // target CPSR (encodes ARM/Thumb via bit5 .t)
    uint32_t recover_sp;        // target SP (non-zero = also restore SP via the primitive;
                                // used by multi-frame pop to skip teardown fns — see
                                // recover_pop and the stack-walk in case 0x01)
    uint8_t  recover_pop[256];  // per-reason "extra pop count" — N>0 means walk the stack
                                // for N additional return-addr-looking words past the entry
                                // LR's frame, set recover_pc to the Nth one + recover_sp
                                // past it. Use when the immediate LR-return lands in a
                                // teardown/die-gracefully fn and the caller above it is
                                // also expecting the system to be dead. Env: RESET_POP=R:N,…
    uint8_t  recover_reasons[256]; // 1 = recover that reason byte. Indexed directly by
                                // the reason value (covers 0..255 including SWDSP 0x68).
                                // Defaults: 5 (watchdog/fatal, RAM-saved state), 12 (task1),
                                // 0x68 (SWDSP=DSP reset) — all driven by entry-LR for non-5 reasons.
                                // Env knobs RESET_RECOVER / RESET_NORECOVER patch this at init.
    // Entry hook for the reboot fn: when the host step loop sees PC enter m->fw.reboot_fn,
    // it snapshots {LR, r0, CPSR} here. LR is the caller's return addr (the panic call
    // site), r0 is the reason byte, CPSR is the mode/Thumb state at the call. For reasons
    // without RAM-saved state (12 / 104 / etc.), recovery PC = entry LR and recover_cpsr
    // = entry CPSR — landing us back at the instruction after the bl that triggered the
    // panic. reboot_entry_seen guards against stale data after a warm reboot.
    uint32_t reboot_entry_lr;    // LR at PC==reboot_fn (latest)
    uint32_t reboot_entry_reason;// r0 at the same instant (the reason r0=N convention)
    uint32_t reboot_entry_cpsr;  // CPSR at the same instant (Thumb-bit + mode)
    uint32_t reboot_entry_sp;    // SP at the same instant (caller's frame base — reboot
                                 // fn is noreturn, doesn't push, so this equals the SP
                                 // when the firmware bl'd into it; multi-frame pop walks
                                 // up from here)
    uint8_t  reboot_entry_seen;  // 1 = entry was observed this cycle (cleared on consume)
    // Eager intercept: short-circuit the reboot chain at the EARLIEST safe point — the
    // reboot fn entry (m->fw.reboot_fn) for firmware-internal panics, and the ARM fatal
    // handler entry (m->fw.fatal_handler) for reason-5 CPU exceptions — instead of waiting
    // for the trailing `[0x20001]|=4` write. Skips bl 0x2D724C (the notify -> task 0x11
    // msg 0x44 + log 0x2EDD4A), the CTSI int-control write, and the busy delay; also
    // skips the fatal handler's stmia save. Behaviour with the flag OFF is unchanged
    // (the late catch fires as before).
    // RESET_EARLY=1 / dct3_web_set_reboot_early() flip it.
    uint8_t  reboot_early;       // 1 = trigger recover at reboot_fn / fatal_handler entry

    // --- Root-cause attribution (assertion log + post-mortem) -------------------
    // The firmware's generic assert_log fn (m->fw.assert_log = 0x2EDD4A on v5.79) is the
    // common emit point for ~99 distinct fault sites. The host hooks PC==assert_log and
    // pushes {cycle, code (r0), data (r1), lr (caller)} into this ring on every call,
    // unconditionally — overhead is one PC compare/insn (gated on fw.assert_log != 0).
    // At catch time mad2_render_postmortem() pulls the last N entries to surface "the
    // firmware's own narrative" alongside the reason byte. See
    //.1, §G.6.
    #define MAD2_ASSERT_RING_N 16
    struct {
        uint64_t cyc;            // rtc_mono snapshot at the assert call
        uint32_t lr;             // caller's return addr (the asserting fn's call site)
        uint16_t code;           // r0 — firmware fault code
        uint16_t data;           // r1 — code-specific data half-word
    } assert_ring[MAD2_ASSERT_RING_N];
    uint32_t assert_w;            // ring write index (total count; wraps via & (N-1))

    // Staged-resets ring. Hook on m->fw.reason_setter (the tiny `strb r0,[=reason_byte]`
    // helper, v5.79 = 0x2E0ED8) captures every staged reason BEFORE the firmware tries
    // its graceful shutdown — often 22 seconds before the visible reboot. r0 = the
    // reason being staged, LR = the caller (= the fn that decided "this is fatal").
    // for the SWDSP DSP-reset walk-through.
    #define MAD2_STAGE_RING_N 4
    struct {
        uint64_t cyc;            // rtc_mono when the setter was entered
        uint32_t lr;             // caller's return addr (whichever fn staged the reason)
        uint8_t  reason;         // r0 — the reason being staged (0x68 SWDSP, etc.)
    } stage_ring[MAD2_STAGE_RING_N];
    uint32_t stage_w;             // ring write index (total count; wraps via & (N-1))

    // Heap-exhaustion attribution. Hook on m->fw.malloc_fail (the blocking allocator's
    // fail/block-retry PC, v5.79 = 0x299B26) counts allocation FAILURES — each hit is one
    // alloc that returned NULL and had to park on the memory semaphore. A handful is normal
    // transient pressure; a sustained climb is heap exhaustion (and a frequent precursor to a
    // heap-smash wild-PC). LR = the caller stuck waiting; first/last cyc bound the episode.
    uint64_t heap_fail_count;     // total alloc-fail / block-retry PC hits
    uint32_t heap_fail_lr;        // caller LR at the latest failure
    uint64_t heap_fail_cyc;       // rtc_mono at the latest failure
    uint64_t heap_fail_first_cyc; // rtc_mono at the first failure ever seen

    // Fatal-exception classification (reason 5 only). Captured by the host step loop
    // when PC enters m->fw.fatal_handler — cpu->cpsr.mode at that instant tells the
    // exception type (0x17 ABT = data/prefetch abort, 0x1B UND = undef instr, 0x13 SVC
    // = SWI). 0 = no fatal-handler entry seen this catch.
    uint8_t  fatal_mode;          // ARM mode bits (0x10..0x1F) at fatal_handler entry
    uint32_t fatal_lr;            // banked LR at fatal entry (faulting PC + ARM offset)
    uint32_t fatal_spsr;          // SPSR at fatal entry (interrupted CPSR)

    // Post-mortem text buffer — populated by mad2_render_postmortem() at catch time.
    // A small fixed buffer is enough; the web layer reads it via dct3_web_postmortem_buf.
    char     postmortem[1024];
    uint16_t postmortem_len;
    uint64_t postmortem_at_cyc;   // rtc_mono when the most recent post-mortem was rendered

    // Interrupt controller (CTSI) + timers.
    uint8_t  fiq_pending;       // 0x08  (1=active; write 1 to clear)
    uint8_t  irq_pending;       // 0x09
    uint8_t  fiq_mask;          // 0x0A  (1=masked, 0=passed)
    uint8_t  irq_mask;          // 0x0B
    uint8_t  int_ctrl;          // 0x0C  (global enable)
    uint32_t fiq3_delay;        // steps until MBUS TxD-empty kickoff FIQ3 asserts (one-shot)

    // --- Watchdog (CCONT WDT) ------------------------------------------------
    // The DCT3 watchdog lives in the CCONT power chip. The firmware kicks it on
    // its main loop: ccont_reset_wdt (MADos hw/ccont.c:181) writes BOTH the CTSI
    // shadow IO_CTSI_WDT (I/O 0x20003 = 0x31) and CCONT reg 0x05 = 0x31; a higher
    // value = a longer time-out. ccont_poweroff (ccont.c:205) writes both to 0x00
    // (= immediate power-off). We model the register state + a kick timestamp so
    // the firmware's kick path round-trips faithfully. A REAL time-out reset is
    // gated OFF by default (WDTRESET=1 to arm): our emulated timing can starve the
    // watchdog and a spurious reset would break boot. wdt_starved counts windows
    // where no kick arrived (diagnostic only unless armed).
    uint8_t  wdt_reg;           // I/O 0x20003 IO_CTSI_WDT shadow (last kick value; 0x00 = power-off)
    uint8_t  wdt_cc;            // CCONT reg 0x05 watchdog value (mirror of the CCONT-side kick)
    uint64_t wdt_kicks;         // count of watchdog kicks (firmware fed the dog)
    uint64_t wdt_last_kick_cyc; // rtc_mono snapshot at the last kick (for starvation timing)
    uint64_t wdt_starved;       // diagnostic: windows elapsed with no kick
    uint8_t  wdt_reset_armed;   // WDTRESET env: actually reset/halt on time-out (default 0 = passive)
    uint8_t  wdt_tripped;       // latched: the watchdog time-out fired (clean diagnostic)
    uint8_t  wdt_disabled;      // CCONT WDT disabled via reg-0x05 = 0x3F (MAME value table); starvation timer paused
    uint64_t wdt_window_cyc;    // starvation window in rtc_mono cycles, = (reload value in seconds) x DCT3_ARM_HZ
    // Nokia service-mode watchdog inhibit. Real service tools could disable the CCONT
    // watchdog so fault-finding wasn't interrupted by clean-shutdown writes (WDT=0)
    // or starvation resets. When set: WDT=0 writes are LOGGED via wdt_inhibited_count
    // but the power_off latch is NOT set, so the emulator keeps running and the
    // engineer can inspect the live phone state. Toggle via dct3_web_set_wdt_service.
    uint8_t  wdt_service_mode;
    uint64_t wdt_inhibited_count; // count of WDT=0 writes that were ignored due to service mode

    // --- Timer1 free-running counter + FIQ5 overflow + 0x16 enable -----------
    // MADos Timer1 (hw/timer.c) is a 16-bit free-running time base at IO_CTSI_TMR1
    // (I/O 0x20004); timer_get_raw_time reads the 16-bit value, and on overflow the
    // handler timer_int_handler_time accumulates +0x7FFF on FIQ5 (int_fiq_set_handler
    // 0x05). Enable/disable is in IO_PUP_FIQ8 (I/O 0x20016) bit0 (EN) / bit2 (MSK):
    // timer_disable clears bit0 + sets bit2, timer_enable does the inverse. We derive
    // the 16-bit counter from CPU cycles, raise FIQ5 on each wrap (only while enabled
    // and unmasked per 0x16), so a firmware timer-disable actually quiesces it.
    uint8_t  fiq8_ctrl;         // I/O 0x20016 IO_PUP_FIQ8 (ct_timer): bit0 EN, bit1 ACT, bit2 MSK
    uint8_t  t1_ovf_en;         // FIQ5/Timer1-overflow interrupt enable (decoupled from fiq8_ctrl)
    uint16_t t1_counter;        // 0x20004 Timer1 free-running 16-bit value (derived from cycles) — LEGACY, now lazy via mad2_t1_value
    uint16_t t1_last;           // last sampled value (for wrap detection) — LEGACY (pre cached-target)
    uint64_t t1_wrap_next;      // cycle-cached next 16-bit-wrap target (per-tick compare; recomputed at edge)
    uint64_t t1_overflows;      // count of FIQ5 (Timer1-overflow) interrupts raised
    uint32_t t1_hz;             // Timer1/CTSI tick rate (default DCT3_T0_HZ=33055; T1HZ env A/B knob, MAME=1057)
    uint32_t fiq8_hz;           // FIQ8 ct_timer (centisecond) rate (default 100; T8HZ env A/B knob)
    uint8_t  intctrl_gate;      // honor the 0x2000C master FIQ disable in delivery (faithful default ON; INTCTRL_GATE=0 reverts)
    uint64_t fiq8_last_cs;      // last centisecond index the FIQ8 ct_timer fired at (edge detect)
    uint64_t fiq8_ticks;        // count of FIQ8 ct_timer interrupts raised (centisecond tick)

    // MBUS UART transmit engine (0x18 control / 0x19 status / 0x1A data). The
    // firmware sends a frame byte-by-byte: a one-shot FIQ3 (TxD-empty) kicks off
    // byte 0 (handler 0x2E35DC), then each byte's completion raises FIQ2 (the MBUS
    // event ISR 0x2EFB1C -> 0x2E3686 -> 0x2E36BE) which loads the next byte until
    // the frame length [0x10BE66] hits 0. Status is hardware-driven, not a readback
    // of the firmware's scratch writes.
    uint8_t  mbus_ctrl;         // 0x18 shadow (bit5 = TX enable/start, bit6 = RX enable)
    uint8_t  mbus_txe;          // TX byte shifted out -> status bit4 (TX register empty)
    uint32_t mbus_tx_delay;     // steps until the loaded TX byte finishes shifting
    // TX byte capture (host serial bridge): the 0x1A write latches the byte here; when
    // mbus_tx_delay expires (the byte has "shifted out") it is pushed to mbus_tx_out so a
    // host bridge can forward the phone's MBUS transmissions to a real service tool. Inert
    // unless a bridge drains the ring — at normal boot nothing reads it (byte-identical).
    uint8_t  mbus_tx_byte;      // 0x1A latched byte, shifting out
    uint8_t  mbus_tx_pending;   // a latched byte is mid-shift (emit on completion)
    uint8_t  mbus_tx_out[64];   // shifted-out TX byte ring (host bridge drains)
    uint8_t  mbus_tx_out_head, mbus_tx_out_tail;  // ring indices (count = tail-head)
    uint64_t mbus_tx_bytes;     // diagnostic: bytes the phone shifted out of MBUS TX

    // --- MBUS receive + accessory/pop-port detect ---------------------------
    // MADos drives MBUS RX over the SAME line as TX: the receive ISR is FIQ2
    // (FIQ_MBUSRX, mbus.c:281); the handler reads IO_MBUS_STATUS(0x19) and dispatches
    // on bit5 RXDRDY (0x20) when in RECEIVE mode (mbus_curmode, ctrl bit6), reading
    // the byte from IO_MBUS_BYTE(0x1A). FIQ3 (FIQ_MBUSTIM, int.h:40) is NOT a TX
    // kickoff: it is the 423.1 Hz MBUS bit-timer, masked by default (int_init
    // _io_set(FIQM,0x08)). We model a deterministic "no accessory connected": the RX
    // FIFO is empty (RXDRDY never asserts), so the firmware's accessory/headset poll
    // sees nothing and does not hang. We do NOT invent bus traffic. The RX plumbing
    // (FIFO + RXDRDY status + FIQ2-on-RX) is in place for future accessory injection.
    uint8_t  mbus_rx[16];       // received-byte FIFO (empty = no accessory talking)
    uint8_t  mbus_rx_head, mbus_rx_tail;  // ring indices (count = tail-head)
    uint8_t  mbus_rxdrdy;       // status bit5 (RXDRDY): a received byte is waiting
    uint64_t mbus_rx_bytes;     // diagnostic: bytes the firmware read from MBUS RX
    // Accessory / pop-port (headset) detect: a GenIO line shows whether anything is
    // plugged into the system connector / headset jack. We model "nothing connected"
    // deterministically so a presence poll reads absent (no phantom accessory).
    uint8_t  accessory_present; // 0 = nothing in the pop-port / no headset (deterministic)

    // Periodic system scheduler tick = FIQ4 (FIQ controller bit4). This is the OS
    // heartbeat: its handler 0x2E4426 -> 0x298B20 ages the software-timer queue
    // (subsystem init state machines arm timeouts on it) and, via the FIQ
    // dispatcher clearing [0x11186C], wakes the boot readiness barrier 0x2E0E32 to
    // re-poll. Without it the OS freezes at the barrier. (Hardware source not yet
    // pinned to a specific MAD timer register; modelled as a free-running periodic
    // FIQ off the CPU cycle count.)
    uint32_t fiq4_period;       // cycles per scheduler tick (0 = disabled)
    uint32_t fiq4_lasttick;     // last tick index seen (edge detect)

    // DSP code-block upload handshake (boot). The MCU uploads DSP code blocks from
    // its own list; per block it copies the data into shared RAM, writes the reply
    // word [0x100E4] (=0x0002 "more", =0x0004 "done") and enables the DSP. The real
    // DSP would consume each block, clear [0x100E4], post the next request to
    // [0x100E2] and raise IRQ4. We don't run the DSP, so we model that ack: on a
    // "more" reply, after a short delay, clear [0x100E4], set a [0x100E2] request,
    // and raise IRQ4 so the firmware uploads its next block — until it writes "done".
    uint32_t dsp_cb_delay;      // steps until the modelled DSP requests the next block
    uint64_t dsp_cb_acks;       // count of code-block acks issued
    uint64_t dsp_steps;         // free-running per-step tick counter (dsp_default_tick runs
                                // once per emulated step). Used to time the keep-alive start
                                // off a fixed step budget instead of peeking MCU-private RAM.

    // DSP runtime boot-indication injector (brute-force harness). The real DSP, once
    // its code uploads, posts a startup/"alive" message into the MDIRCV queue (MCU
    // 0x10100, read ptr [0x101CA], write ptr [0x101C8]; word offsets from 0x80) and
    // raises FIQ0 -> handler 0x2BAB82 -> task (4,4) -> drain 0x2BAC72 -> dispatch by
    // (type & 0x1F) -> the DSP subsystem state machine sets its readiness flags. We
    // don't run the DSP, so inject a configurable message once after the upload:
    //   DSPMSG="type[,len,b0,b1,...]"  DSPMSGDELAY=<cycles>  (all hex/dec)
    // Queue word0 = (len<<8)|type; body = len bytes; total 1+(len+1)/2 words.
    uint8_t  dsp_msg_en;        // DSPMSG provided
    uint8_t  dsp_msg_type;      // DSP message type byte (low byte of word0)
    uint8_t  dsp_msg_len;       // body length (high byte of word0)
    uint8_t  dsp_msg_body[32];  // body bytes
    uint32_t dsp_msg_delay;     // cycles after queue-init before injecting (0 = default)
    uint32_t dsp_msg_ctr;       // countdown to inject
    uint8_t  dsp_injected;      // one-shot guard

    // DSP heartbeat test (DSP-watchdog hypothesis verification). Periodic IRQ4/FIQ0
    // pulse from the DSP side; gated by DSPHEARTBEAT_CYC env knob. See
    // src/mad2/dsp_default.c for the resolver. dsp_hb_last is monotonic cycle of the
    // most recent pulse; dsp_hb_pulses is a count for diagnostics.
    uint64_t dsp_hb_last;
    uint64_t dsp_hb_pulses;

    // per-offset access counters for the 256-byte MMIO window. Set
    // env MMIOAUDIT=1 to enable. mad2_mmio_audit_dump() prints the table at end of
    // a run — shows fall-through (unhandled) MMIO addresses touched during boot, so
    // we can identify hardware signals we're not modeling.
    uint32_t mmio_audit_r[0x100];
    uint32_t mmio_audit_w[0x100];
    uint32_t mmio_audit_first_pc[0x100];
    uint32_t mmio_audit_first_pc_w[0x100];
    uint8_t  mmio_audit_first_val_w[0x100];

    // DSP self-test responder (faithful organic boot, no verdict spike). The firmware
    // streams the DSP self-test request (records 19/20/21/22 + IMEI, then a [13,0]
    // "command 13" trigger) over MDISND, marks verdict bit2 ([0x11FF15] |= 0x04 =
    // "DSP self-test pending") and blocks the self-test task waiting for the result.
    // We model the DSP replying: when bit2 is set, post a group-0x74 result [13,0] into
    // MDIRCV + raise FIQ0 -> dispatcher 0x2EE9B6 -> 0x2C2824 -> self-test task command-13
    // (0x2414C2) which clears bit2 (keeping bit6) -> verdict 0xC8. The MCU does the
    // verdict write itself; we only deliver the mailbox reply. Disable with DSPNOSELFTEST.
    uint8_t  dsp_selftest_off;       // env DSPNOSELFTEST: disable the responder (A/B)
    uint8_t  dsp_selftest_replied;   // one-shot guard
    // DSPVIS=1 (A/B experiment): drive every HLE DSP decision from DSP-VISIBLE signals
    // only — no reads of MCU-private RAM ([verdict], [dsp_uploaded]). The self-test
    // reply triggers on the observed MDISND {group 0x70, sub 0x0D} "run self-test"
    // request record (fw.mdisnd_q/mdisnd_tail); the [dsp_uploaded] gates (IRQ4 raise,
    // Cobba auto-consume) use the internal dsp_running latch (set at the first
    // block-ack pump cycle, cleared when the MCU re-parks the boot-status word —
    // i.e. a warm reboot re-arms it). Default OFF: legacy RAM-watch path unchanged.
    uint8_t  dsp_vis;                // env DSPVIS: DSP-visible triggers only
    uint8_t  dsp_st_req;             // DSPVIS: MDISND {0x70,0x0D} request observed
    uint8_t  dsp_running;            // DSPVIS: internal "DSP code running" latch
    uint8_t  dsp_cb_armed_nz;        // DSPVIS: current pump cycle was armed by a REAL
                                     // block delivery (nonzero cb_reply write), not the
                                     // firmware's [cb_reply]=0 clear — only a real block
                                     // latches dsp_running (see dsp_default_tick)
    uint16_t dsp_mdisnd_prev;        // DSPVIS: last observed MDISND tail (word index).
                                     // The write hook sees RAM post-store for plain RAM,
                                     // so the record just enqueued starts HERE, not at
                                     // the (already overwritten) in-RAM tail value.
    uint8_t  dsp_hle_quiet;          // co-sim: silence the whole HLE tick (signals come
                                     // from the real C54x only); DSP54_HLE=1 re-enables
    uint8_t  dsp_no_keepalive;       // ROM-4 (5110/6110): suppress the 3310/ROM-6 perpetual
                                     // MDIRCV keep-alive heartbeat. COSIM-GROUNDED: the real
                                     // ROM-4 DSP is SILENT on MDIRCV at standby (150M cosim:
                                     // last DSP traffic ~55M, only MCU CTSI-WDT kicks after) —
                                     // it has no 0xE4 MDI-activity watchdog to feed. Default 0
                                     // (ROM-6 keep-alive ON, 3310 byte-identical).
    uint16_t t0_div;            // 0x0F  Timer0 divider
    uint16_t t0_dest;          // 0x12  Timer0 destination -> IRQ4
    uint16_t t0_counter;        // 0x10  Timer0 current value (derived from cycles)
    uint16_t t0_last;           // for edge/crossing detection
    uint16_t t0_offset;         // continuity offset: keeps the count fixed across a divider change
    uint64_t irqs_raised;       // delivered count
    uint64_t fiqs_raised;       // delivered FIQ count (MBUS TxD etc.)

    // Keypad matrix (UIF). Rows driven on 0x28 (signal) / 0xA8 (direction),
    // columns read on 0x2A (5 cols). The scanner reads "special" keys (incl. the
    // power key) with row-direction 0xE0, and the normal matrix row-by-row.
    // held_keys: bit set = that matrix position is pressed. Layout TBD by probe.
    uint8_t  kbd_row_sig;       // 0x28 last write (active-low row select)
    uint8_t  kbd_row_dir;       // 0xA8 last write (0xE0 = special-key scan)
    uint8_t  kbd_special_cols;  // columns (bit0..4) pressed in the special scan
    uint8_t  kbd_norm_cols[8];  // per-row columns pressed in the normal scan
    uint32_t kbd_reads;         // count of keypad column-port reads (scan activity)
    uint8_t  kpd_im_status;     // 8850-class UIF keypad interrupt-source status (I/O 0x2B);
                                // bit4 = matrix-key edge pending. Only used when the model's
                                // keypad.uif_irq is set (3310 routes IRQ0 straight to its ISR).
    uint8_t  kpd_src34;         // later-serial keypad matrix interrupt-source (I/O 0x34 bits[4:0]),
                                // set on a matrix-key edge when keypad.irq_src34 is set (8810).
                                // Read-to-clear on the 0x34 read; 0 otherwise.
    uint8_t  slide_open;        // slide-phone cover state (keypad.has_slide): 1 = open (cover
                                // line I/O 0x28 bit0 reads 1), 0 = closed (boot rest state).
    uint8_t  cover_int_pending; // reed-switch interrupt-source latch (I/O 0x29 bit0); set on a
                                // slide toggle, cleared by the IM_R (0x69) ack write.

    // Early-MAD2 serial bus (BusOps mad2_bus_serial): CCONT bit-banged over GENSIO 0x2A/0x2D.
    uint8_t  ext_eeprom_din;   // current data-in byte presented on [0x2D]

    // Early-MAD2 external EEPROM: a 24C16 (2 KB) I2C chip bit-banged on I/O port 0x19
    // (MADos: SDA = bit7, SCL = bit6; write-to-drive, read-to-sample). The 24C16 FSM
    // tracks START/STOP + clocked bits and presents EEPROM data on SDA reads. Backed by
    // the NokiX virgin blob nse-1.bin (loaded once at first use). See ext_eeprom.c.
    uint8_t  i2c_eeprom[32768]; // ext I2C-EEPROM contents (baked NokiX blob / EE5110 file). Sized
                                // to the largest device (24C256 = 32K); the active device size +
                                // word-address width come from ModelProfile.i2c_eeprom_size
                                // (24C16 2K/1-byte addr for the 5110, 24C64 8K/2-byte for the 6110).
    uint8_t  i2c_loaded;        // blob loaded yet?
    uint32_t i2c_eeprom_writes; // count of MCU data-writes to the 24C16 (web auto-save trigger,
                                // mirrors flash_eeprom_programs for the in-flash-EEPROM models)
    uint8_t  i2c_scl, i2c_sda;  // last master-driven SCL / SDA levels
    uint8_t  i2c_active;        // inside a START..STOP transaction
    uint8_t  i2c_phase;         // 0 = device-addr, 1 = word-addr (high for 24C64+), 2 = data,
                                // 3 = word-addr low byte (24C64+ 2-byte addressing)
    uint8_t  i2c_bit;           // bit index 0..8 (8 = ACK clock)
    uint8_t  i2c_shift;         // byte being clocked in/out
    uint8_t  i2c_reading;       // current transfer direction (1 = master reads EEPROM)
    uint8_t  i2c_drive;         // slave is driving SDA this bit (ACK or read-data) -> level in i2c_dout
    uint8_t  i2c_dout;          // SDA level the slave presents (0 = pulled low)
    uint8_t  i2c_rdack;         // current byte was EEPROM-read (1) vs master-written (0) -> who ACKs
    uint8_t  i2c_rbit;          // read-data bit latched at SCL rising (pre-increment, MSB first)
    uint16_t i2c_addr;          // 11-bit EEPROM word address (page<<8 | byte), auto-increment
    uint8_t  i2c_page;          // device-address page bits (A10..A8 of the 24C16)

    // LCD framebuffer (DDRAM). Flat, strided by the ACTIVE width (m->model->lcd.width)
    // so banks pack with no padding: byte index = bank * width + col. Sized to the
    // largest geometry we support (LCD_MAX_*) so a runtime-variable LCD needs no heap.
    // For the 3310 (84x48) this is byte-identical to the old fb[6][84] layout.
    uint8_t fb[LCD_MAX_W * LCD_MAX_BANKS];
    int     lcd_x, lcd_y;       // DDRAM address (col 0..width-1, bank 0..banks-1)
    int     lcd_vaddr;          // V bit: 1 = vertical addressing
    int     lcd_extended;       // H bit: extended instruction set selected
    // PCD8544 display-control mode (set-display command 0x08|D<<2|E): a whole-screen
    // present transform applied at scan time over the persistent DDRAM (m->fb).
    // 0=blank (all OFF/light), 1=all-on (all dark), 2=normal, 3=inverse. Default 2.
    uint8_t lcd_mode;
    // SED1565/SSD-family controller state (LCD_NAVI = 7110). The 7110 LCD is NOT a
    // PCD8544 — it uses the SED1565 command set (page B0-B8, split column 00-0F/10-1F,
    // display AE/AF, all-points A4/A5, normal/reverse A6/A7, ADC A0/A1, 2-byte 0x81/0xAD).
    // These track the SED1565 present-state; lcd_mode is recomputed from them so the
    // shared renderer (mad2_lcd_px) is unchanged. Unused (0) on PCD8544 models.
    uint8_t lcd_arg_pending;    // consume the data byte of a 2-byte command (contrast/indicator)
    uint8_t lcd_disp_on;        // AF=on / AE=off
    uint8_t lcd_allpix;         // A5=all-points-on / A4=normal
    uint8_t lcd_reverse;        // A7=reverse / A6=normal
    uint8_t lcd_seg_remap;      // A1=reverse segment order (mirror X) / A0=normal
    uint64_t lcd_data_writes;
    uint64_t lcd_cmd_writes;
    // LCD write-stream log (diagnostic; armed by a harness, default OFF -> zero effect).
    // Ring of raw port writes as seen by the bus: bits [7:0] = byte, bit 8 = data port
    // (vs command port), bits [10:9] = bus write size (0=byte 1=half 2=word) so a
    // firmware that writes wider than the port models is visible in the stream.
#define LCDLOG_N 65536
    uint8_t  lcdlog_on;
    uint32_t lcdlog_w;          // total writes; ring index = w % LCDLOG_N
    uint16_t lcdlog[LCDLOG_N];

    // Backlight LEDs (GenIO outputs). LCD LEDs = McuGenIO 0x20020 bit3; keypad LEDs
    // = CTRL-I/O-3 0x20033 bit1. Observed (RAM still backs the registers for readback)
    // and surfaced to the web UI as the LCD glow + keypad button backlight.
    uint8_t led_lcd;
    uint8_t led_kbd;

    // Buzzer + vibra (PUP/PWM registers). Observed for the web Audio model; RAM
    // still backs the registers for firmware readback. PUP-control 0x15: bit5 =
    // buzzer enable, bit4 = vibra enable. Buzzer clock divider 0x1C (captured as
    // 16-bit via 0x1C low / 0x1D high — an 8-bit divider would be ultrasonic, so
    // the real width is logged/calibrated from an actual beep). 0x1E = volume.
    // Vibra control 0x1B = freq(bits0-4)+mode(bits5-6).
    uint8_t  buzz_on;          // 0x15 bit5 (current level)
    uint8_t  vibra_on;         // 0x15 bit4
    uint16_t buzz_div;         // 0x1C(/0x1D) tone clock divider; confirmed 16-bit, freq = 13e6/div
    uint8_t  buzz_vol;         // 0x1E
    uint8_t  vibra_ctrl;       // 0x1B
    uint64_t buzz_writes;      // count of divider writes
    // Rising-edge latch so the web Audio model never misses a sub-frame chirp (e.g.
    // Snake's eat-food blip): a beep that enables+disables between two ~16 ms browser
    // frames would be invisible to a level poll. buzz_edges counts off->on transitions
    // since the page last drained it; buzz_div_at_edge snapshots the divider at the edge.
    uint8_t  buzz_edges;
    uint16_t buzz_div_at_edge;

    // --- Host PCM sink (HAL output channel) -----------------
    // When set, every DSP codec DAC sample is delivered here as it is emitted:
    // ch1 = earpiece DAC (COBBA port/DXR 0x21), ch0 = the secondary channel
    // (0x20). Source today = the C54x cosim DXR tap (third_party/c54x
    // pcm_capture, the same stream DSP54_PCMCAP files); an HLE tone synth is a
    // planned second producer. pcm_rate = the producer's sample rate estimate
    // (Hz; cosim default 8000, DSP54_PCMRATE override). Bindings: boot_trace
    // PCMSINK=1 demo counter; web/SDL audio = open. NULL = channel off (the
    // producer-side taps stay zero-cost).
    void   (*pcm_sink)(struct Mad2* m, int ch, int16_t sample);
    double   pcm_rate;
    // Unified audio mixer (emu_audio.c): synthesizes the PWM buzzer into the SAME
    // pcm_sink PCM stream, so every harness plays ONE PCM channel regardless of the
    // source — the cosim DSP codec (DXR tap) and the buzzer both arrive as PCM, and
    // a harness never branches on DSP type. Clocked off rtc_mono at DCT3_CODEC_HZ.
    // No-op when pcm_sink is NULL (headless boots stay byte-identical). Buzzer (MAD2
    // models) and the cosim codec (5110) never overlap in one run.
    uint64_t audio_last_cyc;    // absolute PCM sample index (on the DCT3_CODEC_HZ grid)
                                // rendered so far by emu_audio_render — the mixer cursor
    uint32_t audio_buzz_phase;  // buzzer square-wave phase accumulator (Q16 of a cycle)
    uint32_t audio_tone_ph1;    // DSP HLE tone osc1 (DTMF high / beep) phase accumulator (Q16)
    uint32_t audio_tone_ph2;    // DSP HLE tone osc2 (DTMF low) phase accumulator (Q16)
    uint16_t audio_env;         // voice envelope (Q8, 0..256): short attack/release so a tone
                                // starting/ending mid-phase doesn't hard-step (click)
    uint32_t audio_c_buzz;      // cached voice params for the release ramp — the waveform
    uint32_t audio_c_t1;        //   keeps ringing down at its last pitch after the register
    uint32_t audio_c_t2;        //   gates it off (Q16 phase increments; 0 = voice not active)
    int32_t  audio_buzz_lp1;    // buzzer tone-shaping filter state: two cascaded one-pole
    int32_t  audio_buzz_lp2;    //   low-passes (the piezo disc + case grille roll off the
                                //   square's high harmonics; a raw square is harsher than
                                //   the physical buzzer ever sounded)
    uint8_t  pcm_codec_seen;    // set by the DSP DXR tap on its first ch1 sample: the cosim
                                // codec owns the pcm_sink this run, so the buzzer mixer
                                // stands down (they never overlap — see note above)
    // DCT3 COBBA codec sample rate (Hz) = the rate at which the firmware's tone
    // oscillator assumes the codec runs, so playing one-sample-per-codec-frame at
    // this rate reproduces the commanded pitch. PINNED from the firmware:
    // the 5110 keypad beep's phase increment is a baked 0x0C5C = 3164 over a 16-bit
    // accumulator -> 65536/3164 = 20.713 samples/cycle (confirmed by autocorrelation
    // of the captured beep). Anchored to the established 900 Hz keypad-beep identity,
    // Fc = 900 * 65536 / 3164 = 18642 Hz. (The earlier 18000 was a round approximation
    // of an imprecise "~20 samples/cycle @ 900 Hz" and rendered the beep ~3.4% flat at
    // 869 Hz.) The ABSOLUTE COBBA hardware rate — i.e. whether the keypad beep is truly
    // 900 Hz — is not independently pinned (needs the COBBA/McBSP CLKG-divider config);
    // this value honours the firmware increment + the repo's 900 Hz reference. Producers
    // default pcm_rate to this; harness audio devices open at it. DSP54_PCMRATE overrides.
    #define DCT3_CODEC_HZ 18642

    // HLE mixer (emu_audio.c) output rate. NOT the codec rate: the codec rate only matters
    // for real codec PCM (cosim DXR). The HLE buzzer is a SQUARE wave — synthesized naively
    // at 18.6 kHz its folded harmonics alias into audible grit (Nyquist 9.3 kHz; ringtone
    // dividers can even be ultrasonic). 48 kHz matches what the legacy SDL synth used and
    // what audio devices natively run, so the resample is ~1:1. The mixer publishes this
    // via pcm_rate; the codec DXR tap overrides pcm_rate when a real codec takes the sink.
    #define EMU_AUDIO_HZ 48000

    // DSP COBBA tone-player registers (RAM shadow of the HPI mailbox window). The MCU
    // writes the oscillator frequencies + amplitude here for the DSP to play; when there is
    // no cosim DSP, the HLE DSP backend (dsp_hle_tone in dsp_default.c) reads them and
    // reports the tone for emu_audio to synthesize into the PCM stream.
    // MODEL-INVARIANT: the HPI window base is fixed across DCT3 (`.cobba` = 0x100E0 on
    // every profile), so these offsets are constants, not per-model. Freq regs are in
    // QUARTER-Hz units (freq_Hz = reg >> 2); the tone is silent when amplitude (0x100B6)
    // is 0. osc1 = DTMF column / plain beep, osc2 = DTMF row.
    #define DCT3_TONE_OSC1 0x000100AEu   // reg 0x21 — osc1 freq (1/4 Hz)
    #define DCT3_TONE_OSC2 0x000100B0u   // reg 0x22 — osc2 freq (1/4 Hz)
    #define DCT3_TONE_AMP  0x000100B6u   // reg 0x25 — amplitude gate

    // --- dbgcon: EMULATOR-ONLY developer console -----------
    // A fake MMIO port block for diagnostic patches (NokiX overlays, injected test
    // code): bytes written here render on the HOST console at run time. Lives inside
    // the hooked I/O window [0x10000,0x100000) at an address no DCT3 hardware decodes;
    // on a real phone these writes are inert. Reading DBGCON_CHAR returns 0xDEADBEEF
    // (truncated per access size, BE: byte=0xDE, half=0xDEAD) — the "am I running
    // under the emulator?" probe, so a patch can skip its logging on real hardware.
    //   CHAR : each written byte appends to a line buffer; '\n' (or 127 chars, or a
    //          FLUSH write) prints the line as "[dbgcon] <text>". 16/32-bit writes
    //          append their bytes MSB-first (big-endian store order).
    //   HEX  : any write prints immediately: "[dbgcon] pc=XXXXXX val=... (dec)".
    //   FLUSH: any write flushes a partial CHAR line.
    #define DCT3_DBGCON_CHAR  0x000DEAD0u
    #define DCT3_DBGCON_HEX   0x000DEAD4u
    #define DCT3_DBGCON_FLUSH 0x000DEAD8u
    char     dbgcon_buf[128];   // CHAR line assembly buffer
    uint8_t  dbgcon_len;
    uint32_t dbgcon_writes;     // total dbgcon port writes (test/diag observable)

    // --- SIM card interface (SIMI UART block, I/O 0x36-0x3F) -----------------
    // The DCT3 SIM reader is a dedicated UART (SIMI_*) the firmware drives over
    // I/O 0x20036-0x2003F, with FIQ6 = SIM-UART (RX/TX FIFO events) and FIQ7 =
    // card insert/remove detect. Confirmed against MADos hw/sim.c and the 3310
    // firmware's own SIM driver (init 0x2D49C8, reset 0x2D455E, FIQ6 ISR 0x2D490E,
    // sim_receive 0x2D4A06, sim_transmit 0x2D460E). We model a present card with a
    // canned ISO-7816 ATR + a T=0 byte engine + an in-memory GSM 11.11 EF tree, so
    // the firmware's SIM detect / ATR / EF reads succeed and disp49 (sim_func, the
    // SIM gate) clears faithfully — no anonymous-access poke. NO crypto (RUN GSM
    // ALGORITHM is the separate network-auth wall, out of scope).
    //
    // Register shadows (offsets within the 0x20000 window):
    //   0x36 SIMI_TXD      0x37 SIMI_RXD      0x38 SIMI_UART_INT  0x39 SIMI_CTRL
    //   0x3A SIMI_CLK_CTRL 0x3B SIMI_TXD_LWM  0x3C SIMI_RXD_QUE   0x3D SIMI_RXD_FL
    //   0x3E SIMI_TXD_FL   0x3F SIMI_TXD_QUE
    uint8_t  sim_present;        // 1 = card inserted (controllable via the web UI)
    uint8_t  sim_present_seen;   // last presence the firmware was told (FIQ7 edge)
    uint8_t  sim_uart_int;       // 0x38 pending UART-int lines (bit6=RX-ready, bit5/4=TX, bit1/7=other)
    uint8_t  sim_ctrl;           // 0x39 last write (bit0 warm-reset, bit7 activate/clock)
    uint8_t  sim_clk_ctrl;       // 0x3A (bit0/1 enable; bits2-3 set = ISO inverse convention)
    uint8_t  sim_txd_lwm;        // 0x3B TX low-water-mark (firmware writes 0x60/0x66)
    uint8_t  sim_rxd_fl;         // 0x3D RX FIFO level config
    uint8_t  sim_txd_fl;         // 0x3E TX FIFO flush/level (0x04 = sending, 0x00 = flush)
    // RX FIFO: bytes the card has emitted, waiting for the firmware to read via
    // SIMI_RXD (0x37); SIMI_RXD_QUE (0x3C) reports the count.
    uint8_t  sim_rx[64];
    uint8_t  sim_rx_head, sim_rx_tail;   // ring indices (count = tail-head)
    // TX assembly: bytes the firmware writes to SIMI_TXD (0x36) accumulate here;
    // a flush (SIMI_TXD_FL -> 0) hands the buffer to the T=0 APDU engine.
    uint8_t  sim_tx[280];
    uint16_t sim_tx_len;
    // T=0 protocol engine state. The DCT3 SIM driver transmits a command in chunks
    // (the 5-byte header, then case-3 data bytes one at a time), and after EACH chunk
    // it expects the SIM UART to raise a TX-threshold/empty interrupt (SIMI_UART_INT
    // bit4) which drives the firmware's sim_advance (0x2D46E4) to post SM "status 7"
    // (ACK, send the next bytes). It is NOT a procedure-byte echo: status 7 comes from
    // the TX-empty path, not from any RX byte. We accumulate the chunks into a full
    // APDU and, once the command (+ its case-3 data) is complete, push the response
    // (case-2 data + SW, or SW only for case-3 with the FCP held for GET RESPONSE).
    uint8_t  sim_t0_state;      // reserved T=0 phase flag (0 = idle); reset on card reset
    uint8_t  sim_asm[280];      // running APDU assembly across TX chunks
    uint16_t sim_asm_len;       // bytes assembled so far
    // The currently SELECTed file: GSM is stateful (SELECT then READ BINARY/RECORD
    // /GET RESPONSE operate on the selection). 0 = none/MF.
    uint16_t sim_sel_file;      // last SELECTed EF/DF file id
    uint16_t sim_sel_df;        // current DF (0x7F20 = GSM, 0x3F00 = MF)
    uint8_t  sim_gr_buf[64];    // GET RESPONSE buffer (FCP / SELECT response)
    uint8_t  sim_gr_len;        // bytes pending for a GET RESPONSE (0xC0)
    uint8_t  sim_reset_count;   // diagnostic: number of card resets (ATR deliveries)
    uint8_t  sim_atr_pending;   // ATR queued after activate; deliver on the RX poll
    uint64_t sim_apdus;         // diagnostic: APDUs answered (exposed to the web UI)

    // --- WWT (Work Waiting Time, ISO-7816 T=0) periodic-timeout raise --------
    // The MAD2 SIMI block raises SIMI_UART_INT bit 5 when the configured WWT timer
    // expires between byte transfers (ISO-7816-3 §7.2; the SIM/MCU has up to ~9600
    // etu = ~9.6 ms at the default rate to respond). The firmware's FIQ6 ISR
    // dispatches that to task 19 (the SIM Server, fn 0x29D0F4) with status code 6,
    // whose handler at 0x29D750 is a retry-counter bumper (++; escalate at 4) — i.e.
    // a "SIM unresponsive, retry" event the protocol stack uses to drive its idle
    // poll / inter-APDU heartbeat. We instantaneously answer APDUs, so the firmware
    // never sees this fire organically; without it the SIM driver eventually trips
    // its hard 7s timeout (fn 0x2D724C posts assertion 0x5B06 → fn 0x2EDD4A → soft
    // reboot reason 0x68 ~110M cycles after boot). SIMWWT=N enables the raise: 1 =
    // default ~125k cycles (~9.6 ms at 13 MHz), N>1 = explicit cycle count, unset/0
    // = disabled (faithful to "no WWT modeled", which is the regression-test baseline).
    uint64_t sim_wwt_threshold_cyc;   // cycles between WWT fires (0 = disabled)
    uint64_t sim_wwt_last_active_cyc; // rtc_mono at last RX/TX activity

    // --- CHV1 (PIN) state (GSM 11.11 / 3GPP TS 51.011 §9.2.x) ----------------
    // CHV1 is the SIM card holder verification (the "Enter PIN" code). When enabled
    // and not yet verified, the SIM advertises CHV1-required in the DF FCP and refuses
    // CHV-protected access; the firmware then shows "Enter PIN". VERIFY CHV (INS 0x20,
    // P2=01) compares the supplied PIN (ASCII, 0xFF-padded to 8) against sim_pin; a
    // match marks it verified and clears the gate; a mismatch returns 63 CX (X = tries
    // left) and decrements; at 0 tries CHV1 is blocked (98 40) until UNBLOCK CHV (0x2C)
    // with the PUK resets it. CHANGE/DISABLE/ENABLE CHV are modeled too.
    uint8_t  sim_pin[8];        // stored CHV1, ASCII digits + 0xFF padding (default "1234")
    uint8_t  sim_puk[8];        // stored unblock code (PUK/CHV1-UNBLOCK), default "12345678"
    uint8_t  sim_pin_enabled;   // 1 = CHV1 required (advertised + enforced); 0 = disabled
    uint8_t  sim_pin_verified;  // 1 = CHV1 already verified this session
    uint8_t  sim_pin_tries;     // remaining CHV1 attempts (3 -> blocked at 0)
    uint8_t  sim_puk_tries;     // remaining UNBLOCK attempts (10 -> permanently blocked)

    // Stats.
    uint64_t ccont_reads, ccont_writes;
    uint32_t dbg_ccw_count[16];  // per-CCONT-reg write count
    uint8_t  dbg_ccw_last[16];   // per-CCONT-reg last value written
} Mad2;

// Initialise the device model for a given phone profile (geometry, ASIC config,
// battery/keypad defaults, firmware-address fallbacks). The shell then sets m->mem
// and calls model_resolve() to apply signatures over m->fw.
void mad2_init(Mad2* m, const ModelProfile* prof);

// Matches dct3_io_read_fn / dct3_io_write_fn (ctx = Mad2*).
uint32_t mad2_read(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t ram_value);
void     mad2_write(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t value);

// Shared default DSP behaviour (the faithful 3310/legacy mailbox+FIQ0 model), exposed
// so the per-model C54x co-sim backend (third_party/c54x/mad2_dsp_c54x.c, native-only)
// can forward to it for the pass-through path while the real DSP runs alongside. These
// are the bodies behind `mad2_dsp_default` (see models/model.h DspOps + dsp_default.c).
int  dsp_default_read (Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out);
int  dsp_default_write(Mad2* m, uint32_t addr, int size, uint32_t value);
void dsp_default_tick (Mad2* m);

// --- Interrupt fabric (CTSI) -------------------------------------------------
// Peripherals raise/ack their interrupt LINE through these instead of poking
// fiq_pending/irq_pending directly (: peripherals signal via
// the shared fabric, never each other). `line` is the bit index 0..7; the
// per-line meaning is the CTSI wiring (e.g. FIQ0=DSP/MDIRCV, FIQ2=MBUS-RX,
// FIQ5=Timer1, FIQ6=SIM-UART, FIQ7=SIM-detect; IRQ0=keypad, IRQ2=CCONT cascade,
// IRQ4=Timer0/DSP-mailbox) — see the call sites' comments. (1 = pending; the
// firmware acks by writing 1 to IO_FIQ_ACT/IO_IRQ_ACT, a bulk mask clear that
// stays in mad2_bus.) These are the only sanctioned writers of the two bytes.
static inline void mad2_raise_fiq(Mad2* m, int line) { m->fiq_pending |= (uint8_t)(1u << line); }
static inline void mad2_raise_irq(Mad2* m, int line) { m->irq_pending |= (uint8_t)(1u << line); }
static inline void mad2_ack_fiq  (Mad2* m, int line) { m->fiq_pending &= (uint8_t)~(1u << line); }
static inline void mad2_ack_irq  (Mad2* m, int line) { m->irq_pending &= (uint8_t)~(1u << line); }

// Signal a NORMAL matrix keypad edge (a shell set/cleared kbd_norm_cols): raise the
// keypad IRQ0, and — for 8850-class models whose IRQ0 dispatcher is source-decoding
// (keypad.uif_irq) — assert the matrix interrupt-source bit (I/O 0x2B bit4) so the
// dispatcher routes to the keypad handler. The 3310 (uif_irq=0) is unaffected (it just
// gets the IRQ0 as before). NOT for the power/special-scan edge — that stays plain IRQ0.
void mad2_keypad_irq(Mad2* m);

// Set the slide-cover (reed switch) state on a slide phone (keypad.has_slide): updates
// the cover line (I/O 0x28 bit0) and raises the reed-switch interrupt (I/O 0x29 bit0 +
// IRQ0) so the firmware latches open/closed and posts the cover message. No-op if the
// model has no slide. open != 0 = slide open (keypad exposed); 0 = closed (boot rest).
void mad2_slide_set(Mad2* m, int open);

// Persist the serial-bus external 24C16 EEPROM (5110/6110/3210) to a file — EE5110SAVE bake/persist.
void ext_eeprom_save(Mad2* m, const char* path);

// Advance the timers from the CPU cycle count and latch timer IRQs.
void mad2_timers_tick(Mad2* m, uint32_t cycles);
// Lowest unmasked pending IRQ/FIQ number (0..7), or -1 if none.
int  mad2_irq_poll(const Mad2* m);
int  mad2_fiq_poll(const Mad2* m);

int  mad2_lcd_px(const Mad2* m, int x, int y);    // shared fb unpack: DDRAM bit + display-control transform
void mad2_render_ascii(const Mad2* m);            // dump framebuffer to stdout
int  mad2_save_pgm(const Mad2* m, const char* path); // 84x48 PGM, 0 on success

// Eager-intercept entry point. Call when the host step loop sees PC enter the universal
// reboot fn (m->fw.reboot_fn) or the ARM fatal handler (m->fw.fatal_handler) AND the
// reboot_early knob is set. `source` is a diagnostic tag ("reboot-entry" / "fatal-entry")
// surfaced in the log line. Returns 1 if recover_pending was armed (caller may want to
// advance PC past the panic site or simply let the post-step apply pick it up), 0 if
// the reason isn't on the recover list (the caller should fall through to the normal
// panic chain so the late catch can warm-reboot as before).
int mad2_intercept_panic_early(Mad2* m, uint32_t reason, uint32_t resume_pc,
                               uint32_t resume_cpsr, uint32_t lr_diag,
                               uint32_t resume_sp, const char* source);

// Push an entry into the assertion-log ring. Host step loop calls this when PC hits
// m->fw.assert_log. Cheap: one ring slot write + one increment.
void mad2_record_assertion(Mad2* m, uint16_t code, uint16_t data, uint32_t lr);

// Push an entry into the staged-resets ring. Host step loop calls this when PC hits
// m->fw.reason_setter (the strb-r0-to-reason-byte helper). r0 is the reason being
// staged; LR is the caller's return addr.
void mad2_record_stage(Mad2* m, uint8_t reason, uint32_t lr);

// Capture banked LR/SPSR + current mode at fatal_handler entry, for the post-mortem.
void mad2_record_fatal(Mad2* m, uint8_t cpsr_mode, uint32_t lr_banked, uint32_t spsr);

// Record a heap allocation failure. Host step loop calls this when PC hits
// m->fw.malloc_fail (the blocking allocator's NULL-return / block-retry PC). LR is the
// caller left waiting for memory. Bumps heap_fail_count and stamps lr/cyc.
void mad2_record_heap_fail(Mad2* m, uint32_t lr);

// Render the post-mortem block into m->postmortem (snprintf). Called from the catch
// sites (both the early-intercept helper and the late `[0x20001]|=4` path). `source`
// is a short tag ("reboot-entry", "fatal-entry", "late-catch") that goes into the
// header line. Safe to call multiple times — the buffer is overwritten.
void mad2_render_postmortem(Mad2* m, uint32_t reason, uint32_t resume_pc,
                            uint32_t resume_cpsr, uint32_t lr_diag,
                            const char* source);

// dump per-offset fall-through MMIO access counts.
void mad2_mmio_audit_dump(const Mad2* m);

#endif // MAD2_H
