// MAD2 — MMIO bus dispatcher: the 0x20000 I/O-window read/write router that fans
// out to every peripheral module (CCONT, flash, LCD, keypad, MBUS, SIM, timers/
// interrupt controller). Extracted from mad2.c; see mad2_internal.h.
// mad2_read / mad2_write / mad2_mmio_audit_dump are public (mad2.h).

#include "mad2/mad2_internal.h"
#include "mad2/emu_host.h"     // emu_audio_render — flush the buzzer voice on register writes

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- dbgcon: emulator-only developer console (mad2.h DCT3_DBGCON_*) -----------
// Diagnostic patches write here to print on the host console (native stdout / browser
// devtools). No machine state is touched beyond the console buffer, so enabling a
// patched image cannot perturb an unpatched boot (guard stays byte-identical).
static void dbgcon_flush(Mad2* m) {
    if (!m->dbgcon_len) return;
    printf("[dbgcon] %.*s\n", (int)m->dbgcon_len, m->dbgcon_buf);
    fflush(stdout);
    m->dbgcon_len = 0;
}
static void dbgcon_write(Mad2* m, uint32_t pc, uint32_t addr, int size, uint32_t value) {
    m->dbgcon_writes++;
    switch (addr) {
        case DCT3_DBGCON_CHAR:
            for (int i = size - 1; i >= 0; --i) {          // MSB-first = BE store order
                char c = (char)(value >> (8 * i));
                if (c == '\n') { dbgcon_flush(m); continue; }
                if (c == '\0') continue;
                m->dbgcon_buf[m->dbgcon_len++] = c;
                if (m->dbgcon_len >= sizeof(m->dbgcon_buf) - 1) dbgcon_flush(m);
            }
            break;
        case DCT3_DBGCON_HEX:
            dbgcon_flush(m);                               // keep line/value order coherent
            printf("[dbgcon] pc=%06X val=%0*X (%u)\n", pc & 0xFFFFFFu,
                   size * 2, value & (size >= 4 ? 0xFFFFFFFFu : size >= 2 ? 0xFFFFu : 0xFFu),
                   value & (size >= 4 ? 0xFFFFFFFFu : size >= 2 ? 0xFFFFu : 0xFFu));
            fflush(stdout);
            break;
        case DCT3_DBGCON_FLUSH:
            dbgcon_flush(m);
            break;
        default: break;                                    // reserved slots: swallow
    }
}

// --- I/O hook entry points ---------------------------------------------------

uint32_t mad2_read(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t ram_value) {
    Mad2* m = (Mad2*)ctx;
    m->cur_io_pc = pc;   // TEMP(CCLOG): attribute CCONT reg-0x0E reads to the ISR
    if (addr >= m->model->mem.flash_base &&
        addr < m->model->mem.flash_base + m->model->mem.flash_size)
        return flash_read(m, addr, size, ram_value);
    // dbgcon presence probe (mad2.h): reading DBGCON_CHAR answers "under the emulator?"
    // with 0xDEADBEEF, truncated per access size in BE order (byte=0xDE, half=0xDEAD).
    if (addr == DCT3_DBGCON_CHAR)
        return size >= 4 ? 0xDEADBEEFu : size >= 2 ? 0xDEADu : 0xDEu;
    // DSP mailbox / boot-status reads are per-model behaviour (DspOps; see models/model.h).
    // dsp_override (the remote-DSP bridge) wins over the model default when set.
    { const DspOps* d = m->dsp_override ? m->dsp_override : m->model->dsp;
      if (d && d->read) { uint32_t out; if (d->read(m, addr, size, ram_value, &out)) return out; } }
    // Per-model bus transport (BusOps; see models/model.h): the early-MAD2 serial-
    // attached models own the external-EEPROM I2C lines + the GENSIO status port.
    // Gets first crack like the DSP hook; NULL bus = the memory-mapped MAD2 path.
    if (m->model->bus && m->model->bus->read) {
        uint32_t out;
        if (m->model->bus->read(m, addr, size, ram_value, &out)) return out;
    }
    if (addr >= MMIO_BASE && addr < MMIO_END) {
        // CCONT readback over GENSIO — the port is profile DATA (MAD2 0x6C / serial-bus
        // 0x2D; 0 -> MAD2 default). Matched on READ, so it never collides with the
        // MAD2 START write at 0x2D. (: addressing as data.)
        uint8_t ccont_rp = m->model->gensio.ccont_r ? m->model->gensio.ccont_r : IO_GENSIO_CCONT_R;
        if ((addr - MMIO_BASE) == ccont_rp) return ccont_read(m);
        // Keypad COLUMN read — port is profile DATA (MAD2 0x2A / serial-bus 0x30; 0 -> 0x2A).
        uint8_t kp_col = m->model->keypad.col_port ? m->model->keypad.col_port : 0x2A;
        if ((addr - MMIO_BASE) == kp_col) { m->kbd_reads++; return kbd_cols(m); }
        switch (addr - MMIO_BASE) {
            case 0x00:                                   // MAD2 ASIC version (read-only)
                // Firmware copies this to the DSP shared-mem ASIC-ID slot 0x101E2
                // (0x2BB6DE) + NetMonitor. 3310 v5.79 real-HW value = 0xA1 (was 0x00,
                // which the DSP/NetMon then saw as a bogus ASIC ID). 0 = unset -> legacy
                // RAM fallthrough (unconfirmed models).
                if (m->model->asic.asic_version) return m->model->asic.asic_version;
                break;
            case 0x01: return m->reset_ctrl;             // MCU reset ctrl: bit0 powerup latch
            case 0x02:                                   // DSP reset control
                // Once the firmware releases the DSP (sets bit0, RMW at 0x2BAE22), real HW
                // brings the DSP up and asserts its clock/ready/API status bits — 3310 reads
                // 0x53 after a fresh boot. We only RAM-back bit0, so synthesise the HW status
                // once bit0 is set (ram_value carries the firmware's own bit0 write).
                {
                    uint8_t rel = m->model->asic.dsp_release_mask
                                ? m->model->asic.dsp_release_mask : 0x01;  // 3310 = bit0
                    if (m->model->asic.dsp_reset_running && (ram_value & rel))
                        return m->model->asic.dsp_reset_running;
                }
                break;
            // Keypad COLUMN read (MAD2 0x2A / serial-bus 0x30) is handled above the switch
            // as profile data (keypad.col_port).
            case 0x2B:  // 8850-class keypad interrupt-source status (IM_C): bit4 = matrix-key edge.
                if (m->model && m->model->keypad.uif_irq)
                    return (uint8_t)((ram_value & ~0x10u) | (m->kpd_im_status & 0x10u));
                break;  // 3310-class: RAM-backed (unchanged)
            case 0x28:  // KPD_R; on a slide phone bit0 is the reed-switch cover line (0x30175E)
                if (m->model && m->model->keypad.has_slide)
                    return (uint8_t)((ram_value & ~0x01u) | (m->slide_open & 0x01u));
                break;  // non-slide: RAM-backed (unchanged)
            case 0x29:  // 8850 cover (reed-switch) interrupt source: bit0 = slide-edge pending
                if (m->model && m->model->keypad.has_slide)
                    return (uint8_t)((ram_value & ~0x01u) | (m->cover_int_pending & 0x01u));
                break;
            case IO_SIM_TXD: case IO_SIM_RXD: case IO_SIM_UART_INT: case IO_SIM_CTRL:
            case IO_SIM_CLK_CTRL: case IO_SIM_TXD_LWM: case IO_SIM_RXD_QUE:
            case IO_SIM_RXD_FL: case IO_SIM_TXD_FL: case IO_SIM_TXD_QUE:
                return sim_read(m, (uint8_t)(addr - MMIO_BASE));   // SIM UART block
            case IO_GENSIO_STATUS: return 0x07;          // always ready (wr/txn/rd)
            // CCONT readback (IO_GENSIO_CCONT_R / serial-bus CC_RD) is handled above the
            // switch as profile data (gensio.ccont_r).
            case 0x18: return ram_value & 0x7Fu;          // MBUS control: bit7 (busy) always clear
            // MBUS status, hardware-driven (MADos MBUS_STATUS_*): bit6 SCL (bus idle =
            // ok to TX) always set; bit4 TXDRDY tracks the transmit engine; bit5 RXDRDY
            // (0x20) asserts when a received byte waits (empty by default = no accessory
            // talking). bits[2:0] BITCNT read 0 so the ISR 0x2EFB1C skips the 0x2E3638
            // branch and reaches the per-byte sender 0x2E3686 (firmware's |=7 scratch
            // writes here are ignored).
            case 0x19: return 0x40u | (m->mbus_txe ? 0x10u : 0u) | (m->mbus_rxdrdy ? 0x20u : 0u);
            case 0x1A: return mbus_rx_count(m) > 0 ? mbus_rx_pop(m) : (ram_value & 0xFF);  // MBUS RX byte
            case IO_IRQ_ACT:  return m->irq_pending;
            case IO_FIQ_ACT:  return m->fiq_pending;
            case IO_IRQ_MASK: return m->irq_mask;
            case IO_FIQ_MASK: return m->fiq_mask;
            case IO_INT_CTRL: {
                // Historically we masked bits 1/3 ("FIQ/IRQ active") on read so the
                // dispatch handlers serviced normally. BUT DISABLE_IRQ reads 0x2000C to
                // SNAPSHOT it for ENABLE_IRQ to restore — and if it's read while already
                // disabled (0x0A, e.g. nested in HEAP_FREE's direct write), the mask makes
                // it save 0x00 → restore re-enables interrupts mid-critical-section →
                // FIQ4 races the open heap op → free-list corruption. INTCTRL_RAWREAD=1
                // returns the true latched value (the A/B test for the set-time wild-PC).
                static int raw = -1;
                if (raw < 0) raw = getenv("INTCTRL_RAWREAD") ? 1 : 0;
                return raw ? m->int_ctrl : (m->int_ctrl & ~0x0Au);
            }
            case IO_T0_DIV:   return m->t0_div & 0xFF;
            case IO_T0_VAL:   return size >= 2 ? m->t0_counter : (m->t0_counter & 0xFF);
            case IO_T0_VAL + 1: return (m->t0_counter >> 8) & 0xFF;
            case IO_T0_DEST:  return size >= 2 ? m->t0_dest : (m->t0_dest & 0xFF);
            case IO_T0_DEST + 1: return (m->t0_dest >> 8) & 0xFF;
            // IO_CTSI_TMR1 (0x04/0x05): the 16-bit free-running Timer1 time base.
            // timer_get_raw_time reads it; the timebase derives from CPU cycles and
            // wraps -> FIQ5 (handled in mad2_timers_tick). This is the real Timer1
            // counter (was previously aliased to t0_counter). Top bit kept clear so a
            // sleep-counter consumer that expects a 15-bit value still works.
            case IO_SLEEP_CTR:   return size >= 2 ? (mad2_t1_value(m) & 0x7FFF) : (mad2_t1_value(m) & 0xFF);
            case IO_SLEEP_CTR + 1: return (mad2_t1_value(m) >> 8) & 0x7F;
            // 0x20006: sleep-clock companion read by the boot finalize gate 0x2E4278,
            // which requires ([0x20006] - [0x20004]) >= 30 sleep ticks before posting
            // init-complete (0x2E433A -> logo). The CTSI/sleep clock runs at ~33055 Hz
            // (= DCT3_T0_HZ), so 30 ticks ≈ 0.9 ms of real-HW wait (NOT 30 ms — earlier
            // comment was off by ~30x). We HLE it as Timer1 + 0x40 so the firmware's
            // check passes immediately. Skipping ~1ms is negligible for boot ordering;
            // keeping this shortcut.
            case IO_SLEEP_CTR + 2: {
                uint32_t v = (mad2_t1_value(m) & 0x7FFF) + 0x40;
                return size >= 2 ? (v & 0xFFFF) : (v & 0xFF);
            }
            case IO_SLEEP_CTR + 3: return (((mad2_t1_value(m) & 0x7FFF) + 0x40) >> 8) & 0xFF;
            case IO_CTSI_WDT: return m->wdt_reg;          // watchdog CTSI shadow readback
            case IO_PUP_FIQ8: return m->fiq8_ctrl;        // Timer1/FIQ8 enable readback
            default:
                // TEMP(MMIOAUDIT): log fall-through MMIO reads (no explicit handler).
                // Per-offset counter; dump via mad2_mmio_audit_dump from boot_trace.
                if (getenv("MMIOAUDIT")) {
                    uint8_t o = (uint8_t)(addr - MMIO_BASE);
                    m->mmio_audit_r[o]++;
                    if (m->mmio_audit_r[o] == 1)
                        m->mmio_audit_first_pc[o] = pc & 0xFFFFFFu;
                }
                break;                                    // other regs: RAM-backed
        }
    }
    return ram_value;
}

void mad2_write(void* ctx, uint32_t pc, uint32_t addr, int size, uint32_t value) {
    Mad2* m = (Mad2*)ctx;
    m->cur_io_pc = pc;
    // MDILOG=1 (TEMP diag): trace MCU->DSP traffic (MDISND) — every CPU write into the
    // DSP shared-RAM mailbox/queue window [0x100E0,0x10200) (cobba/cb_req/cb_reply/mbox0/
    // mbox1 + the runtime MDI send/recv queues) and the DSP interrupt signal [0x20002].
    // Lets a run correlate what the MCU sends the DSP with the verdict/MDIRCV state.
    {
        static int mdilog = -1;
        if (mdilog < 0) mdilog = getenv("MDILOG") ? 1 : 0;
        if (mdilog && ((addr >= 0x000100E0u && addr < 0x00010200u) || addr == 0x00020002u))
            printf("[mdisnd] @mono=%llu pc=%06X [%05X]=%0*X\n",
                   (unsigned long long)m->rtc_mono, pc & 0xFFFFFFu, addr,
                   size >= 2 ? 4 : 2, value & (size >= 2 ? 0xFFFFu : 0xFFu));
    }
    // DSPIF_LOG=1 (TEMP measurement): trace the MCU->DSP DSPIF doorbell at 0x30000
    // (16-bit reg, BE: bit1=APIMODE enable DSP mailbox access, bit2=DSPINT interrupt
    // the DSP). Question under test: does DSPINT fire as a bounded boot burst (~95 rings
    // => could clock the DSP->MCU MDIRCV keep-alive to idx 0xDF then go silent like real
    // HW) or sparsely forever (=> NOT the keep-alive clock)? Default OFF, no behaviour change.
    {
        static int dspiflog = -1; static unsigned long long dn = 0, dint = 0;
        if (dspiflog < 0) dspiflog = getenv("DSPIF_LOG") ? 1 : 0;
        if (dspiflog && addr >= 0x00030000u && addr <= 0x00030001u) {
            // byte/halfword writes to the reg both land here; bit2 in the low byte (BE 0x30001).
            uint16_t v16 = (size >= 2) ? (uint16_t)value
                         : (addr == 0x00030001u) ? (uint16_t)(value & 0xFF)
                         : (uint16_t)((value & 0xFF) << 8);
            int apimode = (v16 >> 1) & 1, dspint = (v16 >> 2) & 1;
            if (dspint) dint++;
            uint32_t hp = m->fw.mdircv_head & m->mem_mask, tp = m->fw.mdircv_tail & m->mem_mask;
            unsigned head = m->mem ? ((m->mem[hp] << 8) | m->mem[hp + 1]) : 0;
            unsigned tail = m->mem ? ((m->mem[tp] << 8) | m->mem[tp + 1]) : 0;
            if (dn < 60 || (dn & 0xFF) == 0)
                printf("[dspif] wr#%llu DSPINT-edges=%llu  [%05X]=%04X API=%d INT=%d  mdircv head=0x%02X tail=0x%02X @mono=%llu pc=%06X\n",
                       (unsigned long long)dn, (unsigned long long)dint, addr, v16, apimode, dspint,
                       head & 0xFF, tail & 0xFF, (unsigned long long)m->rtc_mono, pc & 0xFFFFFFu);
            dn++;
        }
    }
    if (addr >= m->model->mem.flash_base &&
        addr < m->model->mem.flash_base + m->model->mem.flash_size) {
        flash_write(m, addr, value, size); return;
    }
    // dbgcon — emulator-only developer console (mad2.h DCT3_DBGCON_*): render writes on
    // the host console. Checked before the DSP/bus hooks so no backend can shadow it.
    if ((addr & ~0xFu) == (DCT3_DBGCON_CHAR & ~0xFu)) {
        dbgcon_write(m, pc, addr, size, value); return;
    }
    // DSP mailbox / code-block writes are per-model behaviour (DspOps; see models/model.h).
    { const DspOps* d = m->dsp_override ? m->dsp_override : m->model->dsp;
      if (d && d->write && d->write(m, addr, size, value)) return; }
    // Per-model bus transport (BusOps): the early-MAD2 serial bus owns the external
    // 24C16 EEPROM I2C lines (0x20/0x24). NULL bus = memory-mapped MAD2 (most models).
    if (m->model->bus && m->model->bus->write && m->model->bus->write(m, addr, size, value)) return;
    if (addr < MMIO_BASE || addr >= MMIO_END) return;
    // LCD + CCONT routing is profile DATA: per-model ports (MAD2 GENSIO LCD 0x2E/0x6E +
    // CCONT 0x2C/START 0x2D; serial-bus LCD 0x2B/0x2C + CC_WR 0x2A). ONE implementation each
    // (: addressing as data). LCD is matched FIRST so serial-bus's LCD-cmd
    // 0x2C wins; the CCONT ports are model-correct (serial-bus ccont_w=0x2A) so no collision.
    {
        uint32_t off = addr - MMIO_BASE;
        if (off == m->model->lcd.io_data) { lcd_data(m, (uint8_t)value); return; }
        if (off == m->model->lcd.io_cmd)  { lcd_command(m, (uint8_t)value); return; }
        uint8_t ccont_wp = m->model->gensio.ccont_w ? m->model->gensio.ccont_w : IO_GENSIO_CCONT_W;
        uint8_t start_p  = m->model->gensio.start   ? m->model->gensio.start   : IO_GENSIO_START;
        if (off == ccont_wp) { ccont_byte(m, (uint8_t)value); return; }   // CCONT reg-select / value
        if (off == start_p)  { m->ccont_have_addr = 0; return; }          // begin transaction
        // Keypad ROW-select + ROW-direction writes — profile data (MAD2 0x28/0xA8,
        // serial-bus 0x31/0x2F; 0 -> MAD2 default). The core still RAM-backs the port (the
        // has_slide read of 0x28 consumes ram_value), so latching here + returning is
        // byte-identical to the old switch cases.
        uint8_t kp_row = m->model->keypad.row_port ? m->model->keypad.row_port : 0x28;
        uint8_t kp_dir = m->model->keypad.dir_port ? m->model->keypad.dir_port : 0xA8;
        if (off == kp_row) { m->kbd_row_sig = (uint8_t)value; return; }   // ROW signal
        if (off == kp_dir) { m->kbd_row_dir = (uint8_t)value; return; }   // ROW direction
    }
    switch (addr - MMIO_BASE) {
        case 0x01:                                              // MCU reset ctrl
            // bit2 = software reset/reboot request: every firmware reset path bl's the
            // universal reboot fn (3310 v5.79 = 0x2EEBAE) with a reason in r0; that fn
            // persists the reason byte to RAM at m->fw.reboot_reason, writes [0x20001]|=4,
            // then spins at `b .`. We discriminate by reading the persisted reason and
            // either recover (reason 5: exception-return from saved fault state) or fall
            // through to warm-reboot via reset_request.
            if (((uint8_t)value & 0x04) && !(m->reset_ctrl & 0x04)) {
                uint32_t mask = m->mem_mask ? m->mem_mask : 0x00FFFFFFu;
                uint8_t reason = 0;
                if (m->fw.reboot_reason && m->mem) {
                    reason = m->mem[m->fw.reboot_reason & mask];
                }
                m->reset_last_reason = reason;
                m->reset_total++;
                m->reset_counts[reason < 16 ? reason : 15]++;
                m->reset_last_pc = 0;
                m->reset_last_cpsr = 0;
                m->reset_last_lr = 0;
                const char* source = NULL;
                // Reason 5 has the fatal handler's 3-word save at m->fw.reboot_save:
                //   [save+0] = interrupted LR (PC to resume at; bit0=Thumb for Thumb code)
                //   [save+4] = SPSR (CPSR to resume with)
                if (reason == 5 && m->fw.reboot_save && m->mem) {
                    uint32_t a = m->fw.reboot_save & mask;
                    const uint8_t* p = m->mem + a;
                    uint32_t saved_lr   = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                                        | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
                    uint32_t saved_spsr = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16)
                                        | ((uint32_t)p[6] <<  8) |  (uint32_t)p[7];
                    m->reset_last_lr   = saved_lr;
                    m->reset_last_pc   = saved_lr;
                    m->reset_last_cpsr = saved_spsr;
                    source = "exc-return";
                }
                // Other reasons: use the entry-LR snapshot captured by the host when the
                // reboot fn (m->fw.reboot_fn) was entered — that's the caller's return
                // address before the noreturn fn's internal bl's destroyed it. The capture
                // must have happened within this reset attempt (reboot_entry_reason matches).
                else if (m->reboot_entry_seen && m->reboot_entry_reason == reason
                         && m->reboot_entry_lr != 0) {
                    m->reset_last_lr   = m->reboot_entry_lr;
                    m->reset_last_pc   = m->reboot_entry_lr;
                    m->reset_last_cpsr = m->reboot_entry_cpsr;
                    source = "lr-return";
                }
                // Multi-frame pop: if recover_pop[reason] > 0 AND we used lr-return (entry
                // SP is known), walk the stack upward looking for plausible saved return
                // addresses, skipping over the requested number of frames. Each match is a
                // halfword-aligned word in the flash range with Thumb-bit set AND the two
                // halfwords immediately before it form a Thumb BL pair (0xF000-0xF7FF
                // prefix + 0xF800-0xFFFF prefix) — same heuristic as the get_string tracer
                // in src/web/main.c. The pop count is per-reason because different panic
                // chains have different teardown depths.
                uint32_t pop_sp = 0;
                if (source && source[0] == 'l' && m->recover_pop[reason] && m->reboot_entry_sp) {
                    uint32_t sp = m->reboot_entry_sp;
                    int need = m->recover_pop[reason];
                    const uint32_t flash_lo = m->model ? m->model->mem.flash_base : 0x00200000u;
                    const uint32_t flash_hi = m->model ? (flash_lo + m->model->mem.flash_size)
                                                       : 0x00400000u;
                    for (int k = 0; k < 256 && need > 0; k++) {
                        uint32_t a = (sp + (uint32_t)k * 4u) & mask;
                        if (!m->mem) break;
                        const uint8_t* p = m->mem + a;
                        uint32_t w = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
                                   | ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
                        uint32_t r = w & ~1u;
                        if (!(w & 1u))               continue;   // need Thumb-bit set
                        if (r < flash_lo || r >= flash_hi) continue;
                        // Halfword immediately before r must be a Thumb BL high half (0xF000..0xF7FF),
                        // and the one at r-2 must be a Thumb BL low half (0xF800..0xFFFF).
                        uint32_t pp1 = (r - 4u) & mask, pp2 = (r - 2u) & mask;
                        uint16_t h1 = ((uint16_t)m->mem[pp1] << 8) | m->mem[pp1 + 1u];
                        uint16_t h2 = ((uint16_t)m->mem[pp2] << 8) | m->mem[pp2 + 1u];
                        if ((h1 & 0xF800u) != 0xF000u || (h2 & 0xF800u) != 0xF800u) continue;
                        // Found one — count it. If this is the Nth, use it as the new resume.
                        if (--need == 0) {
                            m->reset_last_pc = w;
                            pop_sp = (uint32_t)(sp + (uint32_t)k * 4u + 4u);   // past the popped slot
                        }
                    }
                }
                // Policy: recover iff the master gate is on, the reason is on the recover
                // list, AND we found a resume PC (RAM-saved state for reason 5, entry-LR
                // snapshot otherwise). Master gate off = always warm-reboot, no matter what.
                int can_recover = m->recover_enabled && m->recover_reasons[reason] && (m->reset_last_pc != 0);
                (void)source;   // surfaced via the log below
                if (can_recover) {
                    m->recover_pending = 1;
                    m->recover_pc      = m->reset_last_pc;
                    m->recover_cpsr    = m->reset_last_cpsr;
                    m->recover_sp      = pop_sp;   // 0 = keep current SP; non-0 = multi-pop
                    m->reset_recovered++;
                    fprintf(stderr,
                        "[reset] CATCH reason=%u (#%llu) RECOVER (%s%s) pc=0x%08X cpsr=0x%08X\n",
                        (unsigned)reason,
                        (unsigned long long)m->reset_counts[reason < 16 ? reason : 15],
                        source ? source : "?",
                        pop_sp ? "+pop" : "",
                        m->reset_last_pc, m->reset_last_cpsr);
                } else {
                    m->reset_request = 1;
                    const char* why = !m->recover_enabled ? "WARM-REBOOT (auto-recover disabled)"
                                    : (reason == 4) ? "WARM-REBOOT (intentional)"
                                    : (!m->recover_reasons[reason]) ? "WARM-REBOOT (reason not in recover list)"
                                    : "WARM-REBOOT (no resume PC — entry LR not captured?)";
                    fprintf(stderr,
                        "[reset] CATCH reason=%u (#%llu) %s\n",
                        (unsigned)reason,
                        (unsigned long long)m->reset_counts[reason < 16 ? reason : 15],
                        why);
                }
                // Post-mortem render — both branches. Captures the firmware's own narrative
                // (assertion ring + caller-site label + exception type for reason 5) into
                // m->postmortem for the web crash panel and stderr-dump.
                mad2_render_postmortem(m, reason, m->reset_last_pc, m->reset_last_cpsr,
                                        m->reset_last_lr,
                                        can_recover ? "late-catch-recover" : "late-catch-warm");
                fputs(m->postmortem, stderr);
                // Consume the entry-LR snapshot; if another reset fires later it must
                // re-capture (an entry on a stale snapshot would resume in the wrong place).
                m->reboot_entry_seen = 0;
            }
            m->reset_ctrl = (uint8_t)value;
            // Re-arm the bit2 edge detector after a recovery. A successful recovery leaves
            // mad2 state intact (unlike a warm-reboot which zeros it), so reset_ctrl keeps
            // bit2 latched. The next firmware reset attempt rewrites [0x20001]|=4 (a RMW)
            // with the same value → no 0→1 transition → catch goes silent and the CPU
            // spins at the reboot fn's b. forever. Clearing bit2 here makes the next
            // reset write trigger fresh. (Warm-reboot path doesn't need this — the host
            // memset's mad2 anyway.)
            if (m->recover_pending) m->reset_ctrl &= ~0x04u;
            break;
        case IO_CTSI_WDT:   // CTSI watchdog shadow: ccont_reset_wdt writes 0x31 (kick),
            // ccont_poweroff writes 0x00 (clean power-off). Feed the dog / latch power-off.
            // Service-mode: a WDT=0 write is logged but doesn't latch power_off, so the
            // emulator keeps running and the engineer can fault-find a live device.
            wdt_kick(m, (uint8_t)value, /*cc=*/0);
            if ((uint8_t)value == 0x00) {
                if (m->wdt_service_mode) {
                    m->wdt_inhibited_count++;
                    if (m->verbose > 0) { m->verbose--; printf("    [wdt] CTSI WDT=0 inhibited by service mode (#%llu)\n", (unsigned long long)m->wdt_inhibited_count); }
                } else {
                    m->power_off = 1;
                }
            }
            break;
        case IO_PUP_FIQ8:   // ct_timer (FIQ8) ctrl: bit0 EN, bit1 ACT, bit2 MSK. timer_enable
            // (0x2E6D8C) sets EN + clears MSK; timer_disable (0x2E6D74) writes 0x04 (MSK set).
            // bit1 ACT is the HARDWARE interrupt-pending flag, WRITE-1-TO-CLEAR (ack): mad2 sets
            // it each centisecond (mad2_timers_tick) and the FIQ8 handler 0x2E6DD4 acks it by
            // writing ACT=1 ([0x20016]|=2). ACT must NOT latch set on a firmware write, or the
            // FIQ dispatch loop 0x2E6FBE re-reads it as active and re-dispatches forever -> no
            // yield -> reason-5 watchdog reboot (the old Stopwatch-Start crash). So a
            // write clears ACT (the ack); EN/MSK latch. FIQ5
            // (Timer1 overflow) is a SEPARATE interrupt acked via 0x20008 by handler 0x2E4418,
            // so it is not touched here (it used to be, from the old FIQ5/FIQ8 conflation).
            m->fiq8_ctrl = (uint8_t)(value & ~0x02u);            // EN/MSK latch; ACT (bit1) never latches set
            break;
        case 0x18: m->mbus_ctrl = (uint8_t)value & 0x7Fu; break; // MBUS control (TX-en bit5, RX-en bit6)
        case 0x1A:   // MBUS TX data: load byte; loading clears TX-empty, and the byte
            // finishes shifting after mbus_tx_delay -> sets TX-empty -> raises FIQ2.
            m->mbus_txe = 0;
            m->mbus_tx_byte = (uint8_t)value;   // latch for the host bridge (emit on shift-out)
            m->mbus_tx_pending = 1;
            m->mbus_tx_delay = 64;
            break;
        case 0x15: {  // PUP control: bit5 = buzzer enable, bit4 = vibra enable
            emu_audio_render(m);   // flush PCM up to now at the OLD level -> sample-accurate onset/end
            uint8_t newOn = (uint8_t)((value >> 5) & 1);        // (MBUS baud bits stay in RAM)
            if (newOn && !m->buzz_on) {                         // rising edge: latch the chirp so a
                if (m->buzz_edges < 0xFF) m->buzz_edges++;      // sub-frame beep is never missed
                m->buzz_div_at_edge = m->buzz_div;
            }
            m->buzz_on  = newOn;
            m->vibra_on = (uint8_t)((value >> 4) & 1);
            break;
        }
        case 0x1B: m->vibra_ctrl = (uint8_t)value; break;       // vibrator freq/mode
        case 0x1C:   // buzzer divider HIGH byte (IO_PUP_BUZ_FH = bfreq>>8), or full 16-bit if word-written.
            // MADos hw/buzzer.c writes FH(0x1C)=high, FL(0x1D)=low; freq = 13MHz/divider.
            emu_audio_render(m);   // flush at the OLD pitch before the divider changes
            if (size >= 2) m->buzz_div = (uint16_t)value;
            else m->buzz_div = (uint16_t)((m->buzz_div & 0x00FF) | ((uint16_t)((uint8_t)value) << 8));
            m->buzz_writes++;
            break;
        case 0x1D:   // divider LOW byte (IO_PUP_BUZ_FL)
            emu_audio_render(m);
            m->buzz_div = (uint16_t)((m->buzz_div & 0xFF00) | (uint8_t)value); break;
        case 0x1E: m->buzz_vol = (uint8_t)value; break;         // buzzer volume
        // Keypad ROW-select (MAD2 0x28 / serial-bus 0x31) + ROW-direction (MAD2 0xA8 /
        // serial-bus 0x2F) writes are handled above the switch as profile data
        // (keypad.row_port / keypad.dir_port). Confirmed against v5.30 fw keypad
        // init (pc 0x290D7A writes 0x2F=0x1F, 0x290D80 writes 0x31=0x00).
        case 0x6B:  // IM_C ack: the 8850 IRQ0 dispatcher (0x301728) / scan write 0x6B|=0x1F to
                    // service the matrix keypad interrupt -> clear the modeled source bit4.
            if (m->model && m->model->keypad.uif_irq) m->kpd_im_status &= (uint8_t)~0x10u;
            break;  // 3310-class: no-op (it writes 0x6B too but never reads the 0x2B source)
        case 0x69:  // IM_R ack: the cover (reed-switch) handler (0x301748) writes 0x69|=1 to
                    // service the slide interrupt -> clear the modeled cover source bit0.
            if (m->model && m->model->keypad.has_slide) m->cover_int_pending = 0;
            break;
        case IO_SIM_TXD: case IO_SIM_RXD: case IO_SIM_UART_INT: case IO_SIM_CTRL:
        case IO_SIM_CLK_CTRL: case IO_SIM_TXD_LWM: case IO_SIM_RXD_QUE:
        case IO_SIM_RXD_FL: case IO_SIM_TXD_FL: case IO_SIM_TXD_QUE:
            sim_write(m, (uint8_t)(addr - MMIO_BASE), (uint8_t)value); break;   // SIM UART block
        case 0x20: {  // McuGenIO (MAD2) / PUP_GENIO (5110): the LCD-backlight enable bit.
            // MAD2 = bit3 (0x08). The early-serial 5110 multiplexes this port (I2C SDA/SCL
            // on bits 0/2) and lights the backlight on bit5 (0x20) — the MAD2 bit3 is never
            // driven there, so reading it left the 5110 backlight permanently "off". The bit
            // is profile DATA (led.lcd_mask); 0 keeps the MAD2 default 0x08.
            uint8_t lmask = (m->model && m->model->led.lcd_mask) ? m->model->led.lcd_mask : 0x08;
            m->led_lcd = (value & lmask) ? 1 : 0;
            break;
        }
        case 0x33:  // CTRL-I/O-3 bit1 = keypad LEDs on most models. On the serial keypad
                    // (keypad.reg33_im_c) 0x33 is instead IM_C, the keypad interrupt-MASK
                    // register — the scan ISR masks columns (|=0x1F) while scanning (fw
                    // 0x290C4C/0x290D2E) and the no-key path unmasks (&=0xE0) to re-arm (fw
                    // 0x25AF3A). It is RAM-backed, so the read returns the last write and the
                    // RMW works; don't hijack it as led_kbd there (every scan would flip the
                    // modeled LED state).
            if (!(m->model && m->model->keypad.reg33_im_c)) m->led_kbd = (value >> 1) & 1;
            break;
            // NOTE: register 0x33 bit0 is the flash Vpp line per MADos flash_vpp
            // (flashvpp==0 path), but it is SHARED with the keypad-LED register and
            // the 3310 ties Vpp permanently enabled. Tracking bit0 here would let a
            // routine LED-off write spuriously disable Vpp and FAIL a real NVRAM
            // commit, so we keep Vpp modelled as always-on (flash_vpp defaults 1 at
            // init) and rely on the busy-edge + error/clear-status modelling, which
            // is what the record-store poll FSM actually depends on.
        // GENSIO CCONT write + START are routed above the switch as profile data
        // (gensio.ccont_w / gensio.start) — see the LCD+CCONT block.
        case IO_IRQ_ACT:  m->irq_pending &= ~(uint8_t)value; cc_int_update(m); break;  // write 1 to clear; CCONT line re-asserts IRQ2 if still pending
        case IO_FIQ_ACT:  m->fiq_pending &= ~(uint8_t)value; break;
        case IO_IRQ_MASK: m->irq_mask = (uint8_t)value; break;
        case IO_FIQ_MASK:
            // Unmasking FIQ3 kicks off a frame in the 3310 firmware: assert it once so
            // the kickoff handler 0x2E35DC sends byte 0. Gate on TX idle (control bit5
            // clear): the per-byte ISR 0x2EFB1C also re-unmasks FIQ3 after every byte,
            // and that re-unmask must NOT re-trigger the kickoff (which would resend
            // byte 0). After byte 0, control bit5 is set, so this is skipped.
            //
            // NOTE on FIQ3 semantics: in MADos FIQ3 (FIQ_MBUSTIM) is the 423.1 Hz MBUS
            // bit-timer, masked by default. The 3310 firmware instead repurposes the
            // FIQ3-unmask edge as a TX kickoff for handler 0x2E35DC (verified against
            // the 3310 ROM, not MADos). We model the 3310 firmware's actual behaviour
            // here (the kickoff), keep the bit-timer free of synthetic traffic, and run
            // MBUS RX over FIQ2 (FIQ_MBUSRX) like MADos — see the 0x19/0x1A read path.
            if ((m->fiq_mask & 0x08) && !((uint8_t)value & 0x08) && !(m->mbus_ctrl & 0x20))
                m->fiq3_delay = 64;
            m->fiq_mask = (uint8_t)value;
            break;
        case IO_INT_CTRL: m->int_ctrl = (uint8_t)value; break;
        case IO_T0_DIV: {
            // The physical Timer0 counter keeps its current value when the prescaler is
            // reprogrammed — only the FUTURE tick rate changes. Our count is an absolute
            // function of (rtc_mono, divider), so a divider change would make it jump
            // discontinuously (the 3310 ROM writes 0x0F=0xFF once at 0x2E4156: a raw 256x
            // drop). Recompute a continuity offset so the visible count is unchanged at the
            // instant of the write; post-change it advances at the new rate from there.
            uint8_t newdiv = (uint8_t)value;
            if (newdiv != (uint8_t)m->t0_div) {
                uint16_t before = m->t0_counter;
                uint64_t nd = (uint64_t)newdiv + 1;
                uint16_t after_raw = (uint16_t)((m->rtc_mono * DCT3_T0_HZ) / (DCT3_ARM_HZ * nd));
                m->t0_offset = (uint16_t)(before - after_raw);
            }
            m->t0_div = newdiv;
            break;
        }
        case IO_T0_DEST:
            if (size >= 2) m->t0_dest = (uint16_t)value;
            else m->t0_dest = (uint16_t)((m->t0_dest & 0xFF00) | (uint8_t)value);
            mad2_ack_irq(m, 4);   // reprogramming Timer0 acks its IRQ4
            break;
        case IO_T0_DEST + 1:
            m->t0_dest = (uint16_t)((m->t0_dest & 0x00FF) | ((uint8_t)value << 8));
            mad2_ack_irq(m, 4);
            break;
        default:
            // TEMP(MMIOAUDIT): log fall-through MMIO writes (no explicit handler).
            if (getenv("MMIOAUDIT")) {
                uint8_t o = (uint8_t)(addr - MMIO_BASE);
                m->mmio_audit_w[o]++;
                if (m->mmio_audit_w[o] == 1) {
                    m->mmio_audit_first_pc_w[o] = pc & 0xFFFFFFu;
                    m->mmio_audit_first_val_w[o] = (uint8_t)value;
                }
            }
            break;
    }
}

// TEMP(MMIOAUDIT): dump per-offset access counts for fall-through MMIO addresses.
// Called by boot_trace at the end of a run when MMIOAUDIT=1.
void mad2_mmio_audit_dump(const Mad2* m) {
    if (!getenv("MMIOAUDIT")) return;
    printf("\n=== MMIO_AUDIT — fall-through (unhandled) MMIO accesses ===\n");
    printf("  offset   reads first_pc   writes first_pc  first_val\n");
    for (int i = 0; i < 0x100; i++) {
        if (m->mmio_audit_r[i] || m->mmio_audit_w[i]) {
            printf("  0x%02X   %6u   0x%06X   %6u   0x%06X   0x%02X\n",
                   i,
                   m->mmio_audit_r[i], m->mmio_audit_first_pc[i],
                   m->mmio_audit_w[i], m->mmio_audit_first_pc_w[i],
                   m->mmio_audit_first_val_w[i]);
        }
    }
    printf("===\n");
}

