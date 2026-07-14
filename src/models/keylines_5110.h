// Shared 5110-family keypad matrix (Family B visual layout on the serial-bus scan HW:
// Menu/Names(C), up/down, digits, * # — no send/end/vol in the matrix).
//
// Calibrated against the live 5110 firmware (rawkey = row*5+col; scan ISR 0x290C7E drives
// DIR_R=1<<row, reads col 0x30; decode store 0x290D0A -> [0x10B6C8]). The matrix is a regular
// grid: keypad columns 1/2/3 sit on scan cols 2/3/4, keypad rows 1..4 on scan rows 1..4 (scan
// row 0 unused). The col-1 nav cluster decodes to rawkeys 6/11/16/21 (rows 1..4) — ALL FOUR
// verified via RAMWATCH=0x10B6C8 on injected presses (each cell -> its row*5+col rawkey).
// (1,1)=Names/C and (4,1)=scroll-up are also confirmed in-GUI; (2,1)=Menu and (3,1)=scroll
// -down follow the same cluster (rawkey 11/16) — full in-menu confirmation is gated behind the
// FAID boot-lock ("Security code") on a virgin EEPROM.
//
// SHARED across the 5110-derived family — 5110 (NSE-1) / 5130 (NSK-1) / 5190 (NSB-1): same
// board family, same serial-bus keypad. Single source of truth; each profile points `.lines`
// here. `static const` so every translation unit that includes it gets its own copy (no
// ODR/link clash) from one definition.
#ifndef DCT3_KEYLINES_5110_H
#define DCT3_KEYLINES_5110_H

#include "models/model.h"

static const KeyLine keylines_5110[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,1,4,0},
    {KK_4,2,2,0}, {KK_5,2,3,0}, {KK_6,2,4,0},
    {KK_7,3,2,0}, {KK_8,3,3,0}, {KK_9,3,4,0},
    {KK_STAR,4,2,0}, {KK_0,4,3,0}, {KK_HASH,4,4,0},
    {KK_SOFT2,1,1,0},   // right softkey "Names" / C (clear) — rawkey 6 -> keycode 0x1A
    {KK_UP,4,1,0},      // scroll up — rawkey 21 -> keycode 0x17, confirmed in-GUI
    {KK_SOFT1,2,1,0},   // left softkey "Menu" — rawkey 11 -> keycode 0x19
    // scroll down: keycode 0x18 lives at rawkey 15 = (row3,col0), NOT (3,1). The keymap
    // table @0x2AB4FC[16] (=(3,1)) is 0x3E (NO KEY) — the earlier (3,1) guess was a dead
    // cell, so the down key did nothing. Verified against the table (idx15=(3,0)=0x18, the
    // 4th nav key alongside up=0x17/Menu=0x19/Names=0x1A); 6110 differs (down IS at (3,1)).
    {KK_DOWN,3,0,0},    // scroll down — rawkey 15 -> keycode 0x18, keymap-confirmed
    {KK_PWR,0,0,0x02},  // special-scan power
};

#endif // DCT3_KEYLINES_5110_H
