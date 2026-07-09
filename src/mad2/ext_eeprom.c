// External 24C16 (2 KB) I2C EEPROM, bit-banged by the EARLY-MAD2 firmware
// (5110/6110/3210/…) on the GENIO ports, NOT the MBUS. (It is all MAD2 — these
// early models just hang an external EEPROM off a serial bus instead of the
// later in-flash EEPROM partition.) RE'd from the 5110 v5.30 reader 0x28C308
// (device addr 0xA0 | page) and its bit primitives 0x28E04C / 0x28E1DC, whose
// port base is r3 = (0x80<<10)+0x20 = 0x20020:
//   port 0x20  bit0 = SDA data,  bit2 = SCL
//   port 0x24  bit0 = SDA direction (1 = MCU drives SDA out, 0 = released/input so the
//                                    EEPROM can drive it for ACK + read data)
// We track START/STOP + clocked bits and present the EEPROM data on SDA when the MCU has
// the line as input, backed by the NokiX virgin blob nse-1.bin. Reached via the model's
// BusOps (mad2_bus_serial, at the bottom of this file). I2CLOG=1 traces transactions.
#include "mad2/mad2_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void i2c_load(Mad2* m) {
    if (m->i2c_loaded) return;
    m->i2c_loaded = 1;
    m->i2c_drive = 1;                    // power-on: MCU drives SDA (idle high)
    memset(m->i2c_eeprom, 0xFF, sizeof m->i2c_eeprom);
    // Source priority: an EE5110= file override (native dev / a baked FAID dump) -> the model's
    // BAKED NokiX virgin blob (ModelProfile.i2c_eeprom_default, src/mad2/ext_eeprom_blobs.h). The
    // baked blob is what makes the WEB build work: the wasm MEMFS has no nse-1.bin to fopen, so a
    // file-only load left the EEPROM virgin 0xFF -> self-test reads garbage calib -> CONTACT SERVICE.
    const char* env = getenv("EE5110");
    FILE* f = env ? fopen(env, "rb") : NULL;
    if (f) {
        size_t n = fread(m->i2c_eeprom, 1, sizeof m->i2c_eeprom, f); fclose(f);
        if (getenv("I2CLOG")) printf("[i2c] loaded %zu bytes from %s\n", n, env);
    } else if (env && getenv("I2CLOG")) {
        printf("[i2c] WARN EE5110 file not found (%s) -> falling back to baked blob\n", env);
    }
    if (!f && m->model && m->model->i2c_eeprom_default) {
        size_t n = m->model->i2c_eeprom_size;
        if (n > sizeof m->i2c_eeprom) n = sizeof m->i2c_eeprom;   // clamp to the 24C16 buffer (2K)
        memcpy(m->i2c_eeprom, m->model->i2c_eeprom_default, n);
        if (getenv("I2CLOG")) printf("[i2c] loaded %zu bytes from baked blob (%s)\n", n, m->model->name);
    } else if (!f && getenv("I2CLOG")) {
        printf("[i2c] WARN no EE5110 file and no baked blob for this model\n");
    }

    // Finalize the "Eeprom Tune Checksum" the boot self-test validates. The NokiX virgin
    // nse-1.bin ships WITHOUT a self-consistent tune checksum (its stored field @0x11E does
    // not equal the sum of its own data), so the firmware's self-test (fn 0x231AF6 ->
    // checksum 0x22EAFC, 5110 v5.30) fails the tune-checksum compare at 0x22EDA8 and clears
    // the MMI-ready verdict bit6 -> false "CONTACT SERVICE". A factory-calibrated phone has a
    // self-consistent external EEPROM; model that by recomputing the field on load. Algorithm:
    //   tune_ck = ( sum(EE[0x40..0x11E)) - EE[0x74] - EE[0x75] ) & 0xFFFF   (stored BE @0x11E)
    // (the EE[0x74..0x76) term is the firmware's 0x27A732 adjustment). Idempotent: a correctly
    // checksummed dump recomputes to the same value. EE5110_RAW=1 skips it (checksum-fault A/B).
    // The tune-checksum offsets + DSP-fault-latch record below are 5110 (24C16) specific. Other
    // serial-EEPROM models (e.g. the 6110's 24C64) have a different EEPROM layout, so applying
    // the 5110 algorithm would CORRUPT them — gate it to the 2K/24C16 device. The 6110's own
    // virgin-provisioning, if needed, is a separate bring-up step. EE5110_RAW also opts out.
    int is_24c16 = !(m->model && m->model->i2c_eeprom_size > 2048);
    if (is_24c16 && !getenv("EE5110_RAW")) {
        unsigned s = 0;
        for (int i = 0x40; i < 0x11E; i++) s += m->i2c_eeprom[i];
        s = (s - m->i2c_eeprom[0x74] - m->i2c_eeprom[0x75]) & 0xFFFF;
        m->i2c_eeprom[0x11E] = (uint8_t)(s >> 8);
        m->i2c_eeprom[0x11F] = (uint8_t)(s & 0xFF);
        if (getenv("I2CLOG")) printf("[i2c] tune checksum finalized @0x11E = 0x%04X\n", s);

        // Provision the DSP-fault latch (record 0x607, 1 byte @EE[0x29E]) on a virgin image.
        // RE'd 2026-06-12 (the legacy reason-0x68 idle reset, 5110 v5.30): the firmware's DSP
        // watchdog (msg-29 handler 0x291C1C -> checker 0x288962, one check per ~94M steps
        // ~= 7.2 s) reads this PERSISTED fault latch from the external 24C16 via the record
        // store (group table flash 0x2A6438; rec 0x607 -> {off 0x29E, len 1} — format proven
        // against the known FAID rec 0x706 -> {0x32C, 8}). bit0 set stages SWDSP reboot reason
        // 0x68, bit1 reason 0x69 (stager 0x2889FC, gated [0x10B541] in {5,6} && [0x10FF94]==0).
        // An ERASED byte (0xFF) reads as "fault latched" -> deterministic 0x68 ~283M after a
        // cold boot (measured; staged @283.5M, fired @296.2M). No code in the MCU image writes
        // rec 0x607 (reader-only) — it is factory/service-provisioned data, so a factory phone
        // ships 0x00 = no fault. Provision ONLY the virgin 0xFF state (a real latched value from
        // a baked dump is preserved). EE5110_RAW=1 skips this too, but is NOT a clean watchdog
        // A/B (it also skips the tune cksum -> CONTACT SERVICE boot, where the msg-29 watchdog
        // never arms; MEASURED: RAW 300M = no trip). The control is this fix absent on a
        // tune-valid image: deterministic 0x68 staged @283.5M (measured pre-fix).
        if (m->i2c_eeprom[0x29E] == 0xFF) {
            m->i2c_eeprom[0x29E] = 0x00;
            if (getenv("I2CLOG")) printf("[i2c] DSP-fault latch (rec 0x607) provisioned @0x29E = 0x00\n");
        }
    }
}

// Eager init entry (called from mad2_init for external-EEPROM models): populate m->i2c_eeprom
// with the baked blob / EE5110 file NOW, so (a) calibration is in place before boot and (b) the
// web persistence layer can overlay a saved image immediately after init — a lazy first-access
// load would race that overlay. Idempotent (i2c_load guards on m->i2c_loaded).
void ext_eeprom_init(Mad2* m) { i2c_load(m); }

// Dump the live 24C16 external-EEPROM buffer to a file. The serial-bus external-EEPROM models
// (5110/6110/3210) have no in-flash EEPROM partition to persist, so a one-time bake (e.g. the
// firmware's SETFAID service writing a valid FAID, or any runtime EEPROM write) is captured by
// saving this buffer and re-loading next boot via EE5110=<file>. Reflects every firmware write
// (the 24C16 FSM writes straight into i2c_eeprom) plus the load-time tune-checksum finalize;
// re-loading is idempotent. Wired to EE5110SAVE=<file> in the native launcher (end of run).
void ext_eeprom_save(Mad2* m, const char* path) {
    if (!path || !*path || !m->i2c_loaded) return;
    FILE* f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "[i2c] SAVE failed to open %s\n", path); return; }
    size_t n = fwrite(m->i2c_eeprom, 1, sizeof m->i2c_eeprom, f);
    fclose(f);
    fprintf(stderr, "[i2c] saved %zu bytes (24C16 ext EEPROM) -> %s\n", n, path);
}

// Write to GENIO port 0x20 (SCL bit2 / SDA-out bit0) or 0x24 (SDA direction bit0).
// Runs the 24C16 FSM on the SCL/SDA edges produced by 0x20 writes.
void ext_eeprom_write(Mad2* m, uint8_t off, uint8_t v) {
    i2c_load(m);
    // Device geometry from the model: 24C16 (2K, 1 word-address byte, 3 page bits in the device
    // address) for the 5110; 24C64 (8K, 2 word-address bytes, device-addr bits = chip-select) for
    // the 6110. addr_mask wraps the auto-increment within the device. Default 24C16 if unset.
    uint32_t dev_size = (m->model && m->model->i2c_eeprom_size) ? m->model->i2c_eeprom_size : 2048u;
    if (dev_size > sizeof m->i2c_eeprom) dev_size = sizeof m->i2c_eeprom;
    uint16_t addr_mask = (uint16_t)(dev_size - 1);
    int two_byte = dev_size > 2048;                      // 24C32/64/128/256: 2-byte word address
    if (off == 0x24) {                                   // SDA direction: bit0 1=drive,0=input
        m->i2c_drive = v & 1;
        return;
    }
    // off == 0x20
    // 5110 EL backlight (PUP_GENIO_LED = bit6/0x40) shares this port with the I2C
    // bit-bang (SDA bit0 / SCL bit2). The bit-bang driver RMWs only bits 0/2 and
    // preserves the upper bits, so bit6 cleanly reflects the led_set state: lit on
    // user activity (keypress), off at idle. Memory-mapped models track led_lcd in
    // mad2_bus case 0x20, but for the serial family that case never runs (this hook
    // claims 0x20 first), so the backlight must be latched here. (MADos led scheme 2:
    // PUP_GENIO bit6 = the EL backlight; bit5/0x20 = the LCD-module DISP line, not it.)
    {
        uint8_t lmask = (m->model && m->model->led.lcd_mask) ? m->model->led.lcd_mask : 0x40;
        m->led_lcd = (v & lmask) ? 1 : 0;
    }
    // I2C SDA/SCL pin positions in I/O 0x20 are per-model (MAD1 default SDA=bit0/SCL=bit2;
    // the 3210 oddball drives SCL on bit3). See ModelProfile.i2c_{sda,scl}_bit.
    uint8_t sda_sh = (m->model) ? m->model->i2c_sda_bit : 0;
    uint8_t scl_sh = (m->model && m->model->i2c_scl_bit) ? m->model->i2c_scl_bit : 2;
    uint8_t scl = (v >> scl_sh) & 1;
    uint8_t sda = (v >> sda_sh) & 1;                     // master-driven SDA level (when driving)
    uint8_t pscl = m->i2c_scl, psda = m->i2c_sda;
    int log = getenv("I2CLOG") ? 1 : 0;

    // START / STOP are only meaningful while the MCU is driving SDA (a falling/rising SDA
    // edge with SCL steady high). When the line is released (input) the EEPROM owns it.
    if (m->i2c_drive && scl && pscl) {                   // SCL steady high: START / STOP
        if (psda && !sda) { m->i2c_active = 1; m->i2c_phase = 0; m->i2c_bit = 0;
                            m->i2c_shift = 0; m->i2c_reading = 0;
                            if (log) printf("[i2c] START\n"); }
        else if (!psda && sda) { if (log) printf("[i2c] STOP\n"); m->i2c_active = 0; }
    }
    // i2c_dout doubles as the ACK-phase flag (1 = we are in the 9th/ACK clock). The MCU sets
    // SCL high THEN separately reads SDA, so the ACK level must persist across the whole 9th
    // clock-high and only resolve on the 9th SCL falling (clearing here would lose the ACK).
    if (m->i2c_active && scl && !pscl) {                 // SCL rising: clock one bit
        if (m->i2c_dout) {
            if (m->i2c_bit == 8) m->i2c_bit = 9;         // 9th (ACK) clock now high
        } else if (m->i2c_bit < 8) {
            if (m->i2c_reading && m->i2c_phase == 2) {
                // EEPROM read: the master raises SCL THEN samples SDA, so present this
                // clock's data bit using the PRE-increment index (MSB first). Latching
                // before the i2c_bit++ below fixes a 1-bit read misalignment that left
                // every sequential read byte shifted left one bit (read = byte<<1 | next>>7),
                // which corrupted the calibration-record checksum -> false CONTACT SERVICE.
                m->i2c_rbit = (uint8_t)((m->i2c_eeprom[m->i2c_addr & addr_mask] >> (7 - m->i2c_bit)) & 1);
            } else {                                     // master-write: sample SDA (MSB first)
                m->i2c_shift = (uint8_t)((m->i2c_shift << 1) | (sda & 1));
            }
            m->i2c_bit++;
            if (m->i2c_bit == 8) {                       // byte complete -> latch, then ACK clock
                // Who ACKs this byte? The EEPROM ACKs every byte the MASTER wrote (device
                // address — read or write — word address, write data). Only the data bytes
                // the EEPROM CLOCKS OUT in a read are master-ACKed. Capture that BEFORE the
                // latch flips i2c_reading/i2c_phase (the read device-addr byte is itself
                // master-written, so it must still get an EEPROM ACK).
                m->i2c_rdack = (m->i2c_reading && m->i2c_phase == 2);
                m->i2c_dout = 1;                         // enter ACK phase (held through 9th high)
                if (!m->i2c_rdack) {
                    if (m->i2c_phase == 0) {             // device addr: 1010 A2 A1 A0 R/W
                        m->i2c_page    = (uint8_t)((m->i2c_shift >> 1) & 0x07);
                        m->i2c_reading = m->i2c_shift & 1;
                        if (m->i2c_reading) {
                            // Read: the address was set by the preceding write-mode addressing.
                            // 24C16 folds the device-addr page bits into the high address bits
                            // (they can change between the addr-set and the read); 24C64+ device
                            // bits are chip-select (no address) so leave the full address intact.
                            if (!two_byte) m->i2c_addr = (uint16_t)((m->i2c_page << 8) | (m->i2c_addr & 0xFF));
                            m->i2c_phase = 2;
                        } else m->i2c_phase = 1;
                        if (log) printf("[i2c] devaddr %02X page=%d %s\n", m->i2c_shift, m->i2c_page,
                                        m->i2c_reading ? "READ" : "WRITE");
                    } else if (m->i2c_phase == 1) {      // word address (24C16: low 8 bits; 24C64+: HIGH byte)
                        if (two_byte) {                  // 24C64+: high byte now, low byte next
                            m->i2c_addr  = (uint16_t)(m->i2c_shift << 8);
                            m->i2c_phase = 3;
                            if (log) printf("[i2c] wordaddr hi -> 0x%02X--\n", m->i2c_shift);
                        } else {                         // 24C16: page<<8 | byte
                            m->i2c_addr  = (uint16_t)((m->i2c_page << 8) | m->i2c_shift);
                            m->i2c_phase = 2;
                            if (log) printf("[i2c] wordaddr -> 0x%03X\n", m->i2c_addr);
                        }
                    } else if (m->i2c_phase == 3) {      // 24C64+ word address LOW byte
                        m->i2c_addr  = (uint16_t)((m->i2c_addr & 0xFF00) | m->i2c_shift);
                        m->i2c_phase = 2;
                        if (log) printf("[i2c] wordaddr -> 0x%04X\n", m->i2c_addr);
                    } else {                             // data write
                        m->i2c_eeprom[m->i2c_addr & addr_mask] = m->i2c_shift;
                        m->i2c_addr = (uint16_t)((m->i2c_addr + 1) & addr_mask);
                        m->i2c_eeprom_writes++;          // NVRAM-changed signal for web auto-save
                    }
                }
            }
        }
    }
    if (m->i2c_active && !scl && pscl && m->i2c_bit == 9) {   // SCL falling AFTER the 9th (ACK) rising
        if (m->i2c_rdack) {                              // master's ACK/NACK of a read-data byte
            if (sda) m->i2c_active = 0;                  // NACK -> end of read
            else     m->i2c_addr = (uint16_t)((m->i2c_addr + 1) & addr_mask);
        }
        m->i2c_dout = 0; m->i2c_bit = 0;                 // leave ACK phase, next byte
    }
    m->i2c_scl = scl; m->i2c_sda = sda;
}

// SDA level the slave presents on a port-0x20 read. Only meaningful when the MCU has
// released the line (i2c_drive == 0); otherwise the bus reads back the master's own level.
// EEPROM pulls SDA low to ACK each master-write byte, and clocks out read data MSB-first.
int ext_eeprom_sda(Mad2* m) {
    // SDA is an open-drain wired-AND: the line is low if EITHER side pulls it low. The MCU
    // pulls low only when it drives a 0; otherwise (driving 1, or released to input) it lets
    // the EEPROM own the line. So the EEPROM's ACK (low after a write byte) wins even while
    // the MCU still has the pin configured as a (high) output.
    int mcu = (m->i2c_drive && !(m->i2c_sda & 1)) ? 0 : 1;
    int eep = 1;
    if (m->i2c_active) {
        if (m->i2c_reading && m->i2c_phase == 2 && m->i2c_bit <= 8)
            eep = m->i2c_rbit;                           // read-data bit (latched at SCL rising)
        else if (m->i2c_dout)                            // 9th (ACK) clock: EEPROM ACKs master
            eep = m->i2c_rdack ? 1 : 0;                  // writes; read-data ACK is master-driven
    }
    return mcu & eep;
}

// --- BusOps: the early-MAD2 serial-attached bus (docs/hal-spec.md) -----------
// On these models CCONT + an external 24C16 EEPROM hang off a bit-banged serial
// bus rather than the memory-mapped GENSIO. This is a model BUS IMPLEMENTATION,
// not a flag the shared dispatcher branches on: mad2_read/mad2_write give it
// first crack at an MMIO access. It owns the GENSIO status port (always-ready)
// and the external-EEPROM I2C lines; CCONT reg-select/readback + the keypad/LCD
// ports are profile DATA (gensio/keypad/lcd specs) handled by the shared path.
static int bus_serial_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    (void)size;
    if (addr < MMIO_BASE || addr >= MMIO_END) return 0;
    uint32_t off = addr - MMIO_BASE;
    if (off == 0x29) {                          // GENSIO STATUS: wr/txn/rd ready
        *out = (ram_value & ~0x07u) | 0x07u;
        return 1;
    }
    if (off == 0x20) {                          // GENIO port 0: external-EEPROM I2C
        // SDA/SCL pin positions are per-model (MAD1 SDA=bit0/SCL=bit2; 3210 SCL=bit3).
        uint8_t sda_sh = m->model ? m->model->i2c_sda_bit : 0;
        uint8_t scl_sh = (m->model && m->model->i2c_scl_bit) ? m->model->i2c_scl_bit : 2;
        uint32_t mask = (1u << sda_sh) | (1u << scl_sh);
        *out = (uint8_t)((ram_value & ~mask)    // present SDA (EEPROM level) + SCL echo at model bits
                         | (ext_eeprom_sda(m) ? (1u << sda_sh) : 0u)
                         | (m->i2c_scl ? (1u << scl_sh) : 0u));
        return 1;
    }
    return 0;
}
static int bus_serial_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    (void)size;
    if (addr < MMIO_BASE || addr >= MMIO_END) return 0;
    uint32_t off = addr - MMIO_BASE;
    if (off == 0x20 || off == 0x24) {           // 24C16 I2C bit-bang (SDA/SCL + direction)
        ext_eeprom_write(m, (uint8_t)off, (uint8_t)value);
        return 1;
    }
    return 0;
}
const BusOps mad2_bus_serial = { "serial-ext-eeprom", bus_serial_read, bus_serial_write };
