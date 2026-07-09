// Shared NSM Family-A keypad matrix (2 soft keys, up/down, send/end, volume, digits).
//
// RE'd from the 8850 firmware keymap table 0x33F504 (normal) / 0x33F520 (special),
// decode fn 0x30540A, index = row*5 + col; keycodes ground-truthed against 3330 source
// (KEY_SEND=0x0E, KEY_END=0x0F, KEY_SOFT_A=0x19, KEY_SOFT_B=0x1A, KEY_UP=0x17,
// KEY_DOWN=0x18). Column 0 (unused on the 3310) carries the Family-A extras.
//
// SHARED across the NSM Family-A phones — 8850 / 8855 / 5210 / 8210 / 8250:
// same MAD2 keymap, so the (row,col) assignments are identical. This is the
// single source of truth; each profile points `.lines` here. `static const` so every
// translation unit that includes it gets its own copy (no ODR/link clash) from one
// definition. Per-model physical differences (slide cover, etc.) stay in the profile.
#ifndef DCT3_KEYLINES_NSM_A_H
#define DCT3_KEYLINES_NSM_A_H

#include "models/model.h"

static const KeyLine keylines_nsm_a[] = {
    {KK_1,1,2,0}, {KK_2,1,3,0}, {KK_3,1,4,0},
    {KK_4,2,2,0}, {KK_5,2,3,0}, {KK_6,2,4,0},
    {KK_7,3,2,0}, {KK_8,3,3,0}, {KK_9,3,4,0},
    {KK_STAR,4,2,0}, {KK_0,4,3,0}, {KK_HASH,4,4,0},
    {KK_SOFT1,1,1,0}, {KK_SOFT2,4,1,0},   // KEY_SOFT_A / KEY_SOFT_B
    {KK_UP,2,1,0}, {KK_DOWN,3,1,0},
    {KK_SEND,2,0,0}, {KK_END,3,0,0},      // green / red
    {KK_VOLUP,4,0,0}, {KK_VOLDOWN,1,0,0},
    {KK_PWR,0,0,0x02},                     // special-scan (boot-hold)
};

#endif // DCT3_KEYLINES_NSM_A_H
