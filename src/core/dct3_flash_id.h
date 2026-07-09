// dct3_flash_id.h — shared DCT3 flash-chip read-ID / autoselect model (Intel/Sharp CFI).
//
// The DCT3 MAD2 flash is a top-boot Intel/Sharp CFI NOR part. After an autoselect
// command (0x90) the device returns its JEDEC identifiers — manufacturer ID at word
// offset 0, device ID at offset 2, block-lock/ext at offset 4 — until a read-array
// reset (0xFF/0xF0). This is real silicon behaviour that BOTH the platform flash model
// (src/mad2/mad2_flash.c) and the bootstrap-capture bridge (tools/lpt_realphone.c) must
// honour, so it lives here as the single source of truth (header-only, no state owned).
//
// Modes match mad2_flash.c's flash_mode: 0 = read-array, 1 = read-status, 2 = read-id.
// We answer Intel mfr 0x0089 + device 0x8890 (the type-1 Intel entry the 3410's flash
// detector matches at FLASH_ID_LOOKUP 0x3F12B0; the 3310 answers the same on real HW).
#ifndef DCT3_FLASH_ID_H
#define DCT3_FLASH_ID_H

#include <stdint.h>

#define DCT3_FLASH_MFR_INTEL  0x0089u   // JEDEC manufacturer: Intel
#define DCT3_FLASH_DEV_TYPE1  0x8890u   // device id (type-1 Intel CFI part)

// Returns the autoselect/read-id word for `addr` when the chip is in read-id mode
// (flash_mode == 2). For any other offset (or non-id reads) the caller's array value
// (`ram_value`) is returned unchanged. `size` is the access width in bytes (>=2 => 16-bit).
static inline uint32_t dct3_flash_id_read(int flash_mode, uint32_t addr, int size, uint32_t ram_value) {
    if (flash_mode != 2) return ram_value;          // read-array / read-status: caller's data
    switch (addr & 0x6u) {
        case 0x0: return size >= 2 ? DCT3_FLASH_MFR_INTEL : ((addr & 1u) ? 0x89u : 0x00u);
        case 0x2: return size >= 2 ? DCT3_FLASH_DEV_TYPE1 : ((addr & 1u) ? 0x90u : 0x88u);
        case 0x4: return size >= 2 ? 0xFFFFu : 0xFFu;       // block-lock / ext
        default:  return ram_value;
    }
}

// Applies a command byte written to the flash region to the read-mode FSM, for the
// command subset that selects what subsequent reads return. Returns 1 if the write was
// a recognised read-mode command (so the caller can skip writing it into the array).
// Program/erase/status commands are left to the caller's fuller FSM (they need Vpp +
// array mutation); this helper only tracks read-array (0xFF/0xF0) vs read-id (0x90).
static inline int dct3_flash_id_cmd(int* flash_mode, uint32_t value) {
    switch (value & 0xFFu) {
        case 0xFFu: case 0xF0u: *flash_mode = 0; return 1;   // read-array reset
        case 0x90u:             *flash_mode = 2; return 1;   // autoselect / read-id
        default:                return 0;                    // not a read-mode command
    }
}

#endif // DCT3_FLASH_ID_H
