// MAD2 — Flash chip (Intel/Sharp CFI command set). Extracted from mad2.c;
// see mad2_internal.h.
//
// The firmware programs the EEPROM partition (settings/NVRAM bookkeeping) using
// the standard Intel CFI protocol exactly like MADos `hw/flash.c`:
//   clear-status 0x50 -> program-setup 0x40 (or erase 0x20 + confirm 0xD0) ->
//   write data -> poll the status register at the flash base until SR.7 (ready).
// Erase/program are bracketed by flash_vpp(1)/flash_vpp(0) (IO_UIF_CTRL3 0x33
// bit0). Our flash is RAM-backed (the EEPROM region is just RAM in the core),
// so we model the device FSM here faithfully:
//   - program ANDs into the array (NOR flash only clears 1->0)
//   - a program/erase needs Vpp enabled, else it FAILS and sets the Vpp-low
//     status bit (SR.3) without modifying the array (matches real HW + MADos)
//   - the status register reports a brief busy->ready edge (SR.7=0 then 1) so a
//     firmware poll loop sees a real transition instead of instant-ready
//   - SR.3/4/5 error bits are 0 on success; clear-status (0x50) clears them
//   - read-id (0x90) returns the array data (the boot ident check is satisfied
//     by the existing image bytes; an explicit CFI ID is not required here)
// Flash region + the MCU+PPM/EEPROM split are per-model (m->model->mem): flash spans
// [flash_base, flash_base+flash_size) (2 MB on 3310/8850, 4 MB on 7110/3330), and the
// EEPROM/NVRAM partition starts at eeprom_base (shifts late on the larger images).

#include "mad2/mad2_internal.h"
#include "core/dct3_flash_id.h"   // shared Intel/Sharp CFI read-ID model (single source)

#define FLASH_BLOCK  0x00010000u     // 64 KB erase block (ASIC-generic)
#define FLASH_SR_READY  0x80u        // SR.7 = write state machine ready
#define FLASH_SR_ERASE  0x20u        // SR.5 = erase error
#define FLASH_SR_PROG   0x10u        // SR.4 = program error
#define FLASH_SR_VPP    0x08u        // SR.3 = Vpp low / programming voltage error
#define FLASH_BUSY_POLLS 2u          // status reads showing SR.7=0 before ready

// Erase-block size for an address. DCT3 MAD2 parts are top-boot Intel flash: the
// top 64 KB of the device is 8x 8 KB PARAMETER blocks (where the FFS/PMM records
// rotate), the rest is 64 KB main blocks. A profile may OVERRIDE the region via
// mem.param_base/param_block_size; otherwise the generic default (top 64 KB / 8 KB)
// applies to any device with a known flash size. Erasing one param block must NOT
// spill into its 8 KB-spaced neighbours — else the FFS's active block gets wiped and
// the next boot's validate fails → CONTACT SERVICE.
static uint32_t flash_erase_blk(const Mad2* m, uint32_t addr) {
    uint32_t pbase = m->model->mem.param_base;
    uint32_t psize = m->model->mem.param_block_size;
    if (!pbase) {                                   // generic top-boot default
        if (!m->model->mem.flash_size) return FLASH_BLOCK;   // unknown geometry: uniform
        pbase = m->model->mem.flash_base + m->model->mem.flash_size - FLASH_BLOCK;
        psize = 0x2000u;                            // 8 KB parameter blocks
    } else if (!psize) {
        psize = 0x2000u;
    }
    return (addr >= pbase) ? psize : FLASH_BLOCK;
}

static void flash_program(Mad2* m, uint32_t addr, uint32_t value, int size) {
    if (addr < m->model->mem.eeprom_base) m->flash_codewrite = addr;   // DIAG only: program below eeprom_base (non-EEPROM region; legit for a custom FFS)
    m->flash_last_cmd_addr = addr;
    // Vpp must be on to program, exactly as MADos brackets flash_write_ram with
    // flash_vpp(1). With Vpp off the write state machine sets SR.3 and the array
    // is unchanged (real-HW behaviour). The firmware enables Vpp around its own
    // record-store writes, so the success path is unaffected.
    if (!m->flash_vpp) { m->flash_sr |= FLASH_SR_VPP | FLASH_SR_PROG; m->flash_busy = 0; return; }
    if (!m->mem) return;
    uint32_t a = addr & m->mem_mask;
    // Flash can only clear bits (1->0); program = old & new, big-endian store.
    if (size >= 2) { m->mem[a] &= (uint8_t)(value >> 8); m->mem[a + 1] &= (uint8_t)value; }
    else           { m->mem[a] &= (uint8_t)value; }
    m->flash_programs++;
    m->flash_busy = FLASH_BUSY_POLLS;        // model the busy->ready edge
    if (addr >= m->model->mem.eeprom_base) m->flash_eeprom_programs++;   // DIAG: EEPROM-partition write
}

static void flash_erase_block(Mad2* m, uint32_t addr) {
    if (addr < m->model->mem.eeprom_base) m->flash_codewrite = addr;   // DIAG only (see flash_program) — recorded, NOT blocked
    m->flash_last_cmd_addr = addr;
    if (!m->flash_vpp) { m->flash_sr |= FLASH_SR_VPP | FLASH_SR_ERASE; m->flash_busy = 0; return; }
    if (m->mem) {
        uint32_t blk  = flash_erase_blk(m, addr);   // boot-block geometry (see helper)
        uint32_t base = (addr & ~(blk - 1)) & m->mem_mask;
        for (uint32_t i = 0; i < blk; ++i) m->mem[base + i] = 0xFF;
    }
    m->flash_erases++;
    m->flash_busy = FLASH_BUSY_POLLS;
}

// Returns the value to deliver for a flash read; ram_value is the array data.
uint32_t flash_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value) {
    // Status is delivered only for reads into the FFS/EEPROM partition — the bank
    // actually being programmed/erased and polled. DCT3-era phones use Intel-style
    // read-while-write ("Wireless") flash: the code partition stays readable and
    // executable while a *different* partition (the FFS/parameter bank) programs.
    // The firmware relies on this — it leaves FIQ ENABLED across an FFS program and
    // its FIQ handlers run from the code partition. One such handler is a heap/canary
    // integrity guard that derefs a baked code-region pointer (0x32E310 -> RAM canary,
    // expect 0xAAAAAAAA). If a code-region read returned the status register during an
    // FFS-bank program it would read garbage, fail the check, and trip a reason-6
    // reboot (the FIQ spin at 0x2EEBEC). Returning array data for code-region reads
    // while the FFS bank is busy models the RWW silicon faithfully and keeps the FFS
    // status poll (which reads the partition it programmed) working.
    if (m->flash_mode == 1 &&
        (addr >= m->model->mem.eeprom_base ||
         (addr & ~7u) == (m->flash_csr_addr & ~7u))) {   // 3410 polls a fixed CSR below eeprom_base
        // Model a brief busy window after a program/erase: SR.7=0 for a couple of
        // polls (with the error bits already valid), then SR.7=1 (ready). The
        // firmware's `and #0x80; beq wait` loop spins until ready, just like HW.
        uint8_t sr = m->flash_sr;
        if (m->flash_busy) { m->flash_busy--; sr &= (uint8_t)~FLASH_SR_READY; }
        else               { sr |= FLASH_SR_READY; }
        return size >= 2 ? (uint32_t)((sr << 8) | sr) : sr;
    }
    // Intel autoselect / read-ID (flash_mode 2). After a 0x90 command the device returns
    // its manufacturer ID at word offset 0, device ID at offset 2, and block-lock/ext at
    // offset 4 (array data elsewhere), until a 0xFF/0xF0 read-array reset. The 3410's
    // flash-chip detector (FLASH_DRIVER_INSTALL 0x3F1304) writes 0x90 to the flash base,
    // REQUIRES the readback to change, then matches mfr/dev against its ID table
    // (0x4C2960, FLASH_ID_LOOKUP 0x3F12B0). Without a real ID the match fails and it
    // installs a NO-OP program/erase driver (stub 0x3F1301 -> [0x12E738]) — so EVERY
    // EEPROM program/erase is silently dropped (the firmware's own writes AND a service
    // reset). We answer with Intel mfr 0x0089 + device 0x8890 = the table's type-1 Intel
    // entry. The 3310's simpler detector skips this check, but real silicon answers the
    // same, so this is faithful for both. Instruction fetches bypass this hook (they use
    // the core's activeRegion pointer), and read-id mode is transient (the probe resets
    // it immediately with 0xFF/0xF0), so bulk array reads are unaffected.
    // read-id (mode 2) -> JEDEC IDs from the shared model; any other mode -> array data.
    return dct3_flash_id_read(m->flash_mode, addr, size, ram_value);
}

// Handles a write to the flash region (command or program data).
void flash_write(Mad2* m, uint32_t addr, uint32_t value, int size) {
    m->flash_cmds++;                      // DIAG: any write to the flash region
    if (m->flash_prog) {                  // previous command was program-setup: this is data
        flash_program(m, addr, value, size);
        m->flash_prog = 0;
        m->flash_mode = 1;                // enter read-status
        return;
    }
    m->flash_last_cmd_addr = addr;
    m->flash_csr_addr = addr;             // status-poll CSR = last command address (the data
                                          // program above early-returns, so this stays at the
                                          // command addr, e.g. the 3410 driver's 0x3FFF00)
    switch (value & 0xFF) {
        case 0xFF: case 0xF0: m->flash_mode = 0; m->flash_erase = 0; break;  // read-array
        case 0x40: case 0x10: m->flash_prog = 1; m->flash_mode = 1; break;   // program setup
        case 0x20:            m->flash_erase = 1; m->flash_mode = 1; break;   // erase setup
        case 0xD0:            if (m->flash_erase) { flash_erase_block(m, addr); m->flash_erase = 0; }
                              m->flash_mode = 1; break;                       // erase confirm / resume
        case 0x70:            m->flash_mode = 1; break;                       // read-status
        case 0x50:            m->flash_sr = 0; m->flash_busy = 0; break;      // clear status register
        case 0x90:            m->flash_mode = 2; break;                       // read-id / autoselect (mfr+device ID)
        default:              break;                                          // ignore unknown
    }
}
