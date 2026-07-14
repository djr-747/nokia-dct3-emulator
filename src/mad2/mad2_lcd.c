// MAD2 — PCD8544 LCD controller + framebuffer output. Extracted from mad2.c;
// see mad2_internal.h.

#include "mad2/mad2_internal.h"

#include <stdio.h>

// --- PCD8544 LCD -------------------------------------------------------------

// --- SED1565 / SSD-family controller (LCD_NAVI = 7110) -----------------------
// The 7110's "Navi" LCD is a SED1565-class page-addressed controller — a DIFFERENT
// command set from the PCD8544. Decoding it as a PCD8544 (the old default) misreads
// every page/column command, so the drawn screen lands in the wrong DDRAM cells and
// the panel renders blank/garbled. Observed 7110 command stream confirms SED1565:
// B0-B8 page, 00-0F/10-1F split column, AF/AE display, A4/A5 all-points, A6/A7
// reverse, A0/A1 ADC, C0/C8 COM, 0x81 contrast (2-byte), 2F/22/33 power, E3 NOP.
static void navi_command(Mad2* m, uint8_t b) {
    if (m->lcd_arg_pending) { m->lcd_arg_pending = 0; return; }   // 2-byte cmd data
    if (b >= 0xB0 && b <= 0xBF) {                  // set page (Y/bank)
        m->lcd_y = b & 0x0F;
        if (m->lcd_y >= m->model->lcd.banks) m->lcd_y = m->model->lcd.banks - 1;
    } else if (b <= 0x0F) {                        // set column address — lower nibble
        m->lcd_x = (m->lcd_x & 0xF0) | (b & 0x0F);
    } else if (b <= 0x1F) {                        // set column address — upper nibble
        m->lcd_x = (m->lcd_x & 0x0F) | ((b & 0x0F) << 4);
    } else if (b >= 0x40 && b <= 0x7F) {           // display start line — scroll, not modelled
        /* no fb effect */
    } else if (b == 0xAE || b == 0xAF) {           // display OFF / ON
        m->lcd_disp_on = b & 1;
    } else if (b == 0xA4 || b == 0xA5) {           // all-points normal / ON
        m->lcd_allpix = b & 1;
    } else if (b == 0xA6 || b == 0xA7) {           // normal / reverse (inverse video)
        m->lcd_reverse = b & 1;
    } else if (b == 0xA0 || b == 0xA1) {           // ADC: segment order normal / reverse
        m->lcd_seg_remap = b & 1;
    } else if (b == 0x81 || b == 0xAD) {           // contrast / static-indicator: next byte is data
        m->lcd_arg_pending = 1;
    }
    // else: power control (28-2F), regulator (20-27), COM dir (C0-C8), reset/NOP — no fb effect.
    // Recompute the shared present transform from the SED1565 state.
    m->lcd_mode = !m->lcd_disp_on ? 0 : (m->lcd_allpix ? 1 : (m->lcd_reverse ? 3 : 2));
}

// The SED1565's DDRAM is 132 columns regardless of the glass width; lcd_x addresses
// DDRAM, and the visible window sits at profile col_offset (7110: cols 18..113).
#define NAVI_DDRAM_W 132

static void navi_data(Mad2* m, uint8_t b) {
    int w = m->model->lcd.width, banks = m->model->lcd.banks;
    // Translate DDRAM column -> visible column: the glass is mounted centered in the
    // 132-col DDRAM, so firmware frame writes land at col_offset..col_offset+w-1
    // (7110 fill/clear streams: always 18..113). Outside the window = no glass.
    int c = m->lcd_x - m->model->lcd.col_offset;
    if (c >= 0 && c < w && m->lcd_y < banks) {
        // Effective X direction = panel segment wiring (profile x_mirror) XOR the
        // controller's runtime ADC select (A0/A1). The 7110 panel is wired reversed,
        // so x_mirror=1 in the profile; the firmware's ADC command toggles from there.
        int mirror = (m->model->lcd.x_mirror ? 1 : 0) ^ (m->lcd_seg_remap ? 1 : 0);
        int col = mirror ? (w - 1 - c) : c;
        m->fb[m->lcd_y * w + col] = b;
    }
    if (m->lcd_x < NAVI_DDRAM_W) m->lcd_x++;        // SED1565: column auto-increments, clamps at DDRAM end
}

void lcd_command(Mad2* m, uint8_t b) {
    m->lcd_cmd_writes++;
    if (m->model->lcd.controller == LCD_NAVI) { navi_command(m, b); return; }
    if ((b & 0xF8) == 0x20) {              // function set
        m->lcd_extended = b & 0x01;        // H
        m->lcd_vaddr    = (b >> 1) & 0x01; // V
    } else if (m->lcd_extended) {
        // temp/bias/Vop — irrelevant to the framebuffer
    } else if ((b & 0xF8) == 0x08) {       // display control: 0 0 0 0 1 D 0 E
        uint8_t d = (b >> 2) & 1, e = b & 1;
        m->lcd_mode = d ? (e ? 3 : 2)      // D=1: E=1 inverse, E=0 normal
                        : (e ? 1 : 0);     // D=0: E=1 all-on, E=0 blank
    } else if ((b & 0xC0) == 0x40) {       // set Y (bank)
        // 4-bit bank field: the 3310's PCD8544 uses 6 banks (0-5), but the 3410's
        // controller is 9 banks (102x72 RAM, rows 0-64). A 3-bit mask (b & 0x07)
        // aliases bank 8 (cmd 0x48) -> bank 0, so bank-8 writes corrupt the TOP of
        // the screen (visible as top-row flicker). Mask 4 bits; clamp guards >banks.
        m->lcd_y = b & 0x0F;
        if (m->lcd_y >= m->model->lcd.banks) m->lcd_y = m->model->lcd.banks - 1;
    } else if (b & 0x80) {                  // set X (col)
        m->lcd_x = b & 0x7F;
        if (m->lcd_x >= m->model->lcd.width) m->lcd_x = m->model->lcd.width - 1;
    }
}

void lcd_data(Mad2* m, uint8_t b) {
    int w = m->model->lcd.width, banks = m->model->lcd.banks;
    m->lcd_data_writes++;
    if (m->model->lcd.controller == LCD_NAVI) { navi_data(m, b); return; }
    if (m->lcd_x < w && m->lcd_y < banks) {
        // x_mirror: this model's panel has reversed segment order — map logical column
        // -> physical (w-1-col) so the fb is stored un-mirrored (5210). Default 0 = no-op.
        int col = m->model->lcd.x_mirror ? (w - 1 - m->lcd_x) : m->lcd_x;
        m->fb[m->lcd_y * w + col] = b;  // flat, stride = width
    }
    if (m->lcd_vaddr) {                      // vertical addressing
        if (++m->lcd_y >= banks) { m->lcd_y = 0; if (++m->lcd_x >= w) m->lcd_x = 0; }
    } else {                                 // horizontal addressing
        if (++m->lcd_x >= w) { m->lcd_x = 0; if (++m->lcd_y >= banks) m->lcd_y = 0; }
    }
}

// --- Framebuffer output ------------------------------------------------------

// The ONE shared framebuffer unpack: raw DDRAM bit + the
// PCD8544 display-control transform (blank/all-on/inverse). Every native
// consumer (ascii/pgm here, gui_sdl) goes through this so blank/inverse
// transitions render faithfully everywhere. (web/main.js mirrors it in JS —
// inherent to the wasm boundary. x_mirror needs nothing here: lcd_data()
// stores the fb un-mirrored at write time.)
int mad2_lcd_px(const Mad2* m, int x, int y) {
    int on = (m->fb[(y >> 3) * m->model->lcd.width + x] >> (y & 7)) & 1;  // raw DDRAM bit (flat)
    switch (m->lcd_mode) {                          // apply the display-control transform
        case 0:  return 0;        // blank: nothing lit
        case 1:  return 1;        // all-on
        case 3:  return on ^ 1;   // inverse
        default: return on;       // normal
    }
}

void mad2_render_ascii(const Mad2* m) {
    int w = m->model->lcd.width, h = m->model->lcd.height;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) putchar(mad2_lcd_px(m, x, y) ? '#' : '.');
        putchar('\n');
    }
}

int mad2_save_pgm(const Mad2* m, const char* path) {
    int w = m->model->lcd.width, h = m->model->lcd.height;
    FILE* f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            fputc(mad2_lcd_px(m, x, y) ? 0 : 255, f);   // pixel on = dark
    fclose(f);
    return 0;
}
