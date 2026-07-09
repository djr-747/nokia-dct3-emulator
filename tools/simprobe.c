/*
 * simprobe - bench bring-up CLI for the DCT3 SIM bridge. Talks to the ESP32 thin
 * reader (flasher/arduino/simbridge) via tools/sim_bridge.c and runs a real SIM,
 * WITHOUT the emulator. Use this to confirm the hardware path (ATR, ICCID, IMSI)
 * before wiring the bridge into mad2.
 *
 *   make simprobe
 *   ./build/simprobe /dev/ttyUSB0 ping
 *   ./build/simprobe /dev/ttyUSB0 atr
 *   ./build/simprobe /dev/ttyUSB0 iccid
 *   ./build/simprobe /dev/ttyUSB0 imsi
 *   ./build/simprobe /dev/ttyUSB0 apdu A0A40000023F00      # raw APDU (hex)
 *
 * Set SIMBRIDGE_LOG=1 to trace every bridge frame + T=0 step.
 */
#include "sim_bridge.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static void hexdump(const char* tag, const uint8_t* b, int n) {
    printf("%s (%d):", tag, n);
    for (int i = 0; i < n; ++i) printf(" %02X", b[i]);
    printf("\n");
}

static int do_atr(void) {
    uint8_t atr[64]; int n = (int)sizeof atr;
    int r = sim_bridge_activate(atr, &n);
    if (r == -2) { fprintf(stderr, "no card present\n"); return 2; }
    if (r == -3) { fprintf(stderr, "mute card (no ATR)\n"); return 3; }
    if (r < 0)   { fprintf(stderr, "activate failed\n"); return 1; }
    hexdump("ATR", atr, n);
    if (n && atr[0] == 0x3B)      printf("convention: direct (TS=3B)\n");
    else if (n && atr[0] == 0x3F) printf("convention: inverse (TS=3F)\n");
    else if (n) fprintf(stderr, "warning: unexpected TS=%02X (not 3B/3F)\n", atr[0]);
    return 0;
}

/* SELECT a file by 2-byte FID (case-3, Lc=2). Returns 0 if SW==90/61/9F. */
static int sel(uint16_t fid) {
    uint8_t hdr[5] = { 0xA0, 0xA4, 0x00, 0x00, 0x02 };
    uint8_t fdat[2] = { (uint8_t)(fid >> 8), (uint8_t)(fid & 0xFF) };
    uint8_t sw1, sw2, out[256]; int olen = (int)sizeof out;
    if (sim_bridge_apdu(hdr, SIMB_DIR_IN, fdat, 2, out, &olen, &sw1, &sw2) < 0) return -1;
    printf("SELECT %04X -> SW %02X%02X\n", fid, sw1, sw2);
    if (sw1 == 0x90 || sw1 == 0x61 || sw1 == 0x9F) return 0;
    return -1;
}

/* READ BINARY (case-2) le bytes from the selected transparent EF. */
static int read_binary(int le, uint8_t* out, int* olen) {
    uint8_t hdr[5] = { 0xA0, 0xB0, 0x00, 0x00, (uint8_t)le };
    uint8_t sw1, sw2;
    if (sim_bridge_apdu(hdr, SIMB_DIR_OUT, NULL, 0, out, olen, &sw1, &sw2) < 0) return -1;
    printf("READ BINARY le=%d -> SW %02X%02X\n", le, sw1, sw2);
    return (sw1 == 0x90) ? 0 : -1;
}

/* GET RESPONSE after a 9FXX select — fetches the FCI (file status: size, type,
 * access conditions, and for MF/DF the CHV1/PIN status byte). */
static int get_response(int le, uint8_t* out, int* olen) {
    uint8_t hdr[5] = { 0xA0, 0xC0, 0x00, 0x00, (uint8_t)le };
    uint8_t sw1, sw2;
    if (sim_bridge_apdu(hdr, SIMB_DIR_OUT, NULL, 0, out, olen, &sw1, &sw2) < 0) return -1;
    return (sw1 == 0x90) ? 0 : -1;
}

/* RUN GSM ALGORITHM (A3/A8) end-to-end shape test against the real card:
 *   ATR -> SELECT MF -> SELECT DF_GSM(7F20) -> A0 88 (RAND[16]) -> 9F0C -> GET RESPONSE 0C
 * -> SRES[4] + Kc[8]. Drives the exact sequence the firmware uses (0x88 is only valid in the
 * GSM context), so it validates the wire SHAPE of our synth's 0x88 path against a real card.
 * randhex may be NULL (defaults to 00 01 .. 0F). */
static int hexparse(const char* s, uint8_t* out, int cap);   /* defined below */
static int do_auth(const char* randhex) {
    uint8_t rand[16];
    if (randhex) {
        if (hexparse(randhex, rand, 16) != 16) { fprintf(stderr, "auth: need 16 RAND bytes\n"); return 64; }
    } else { for (int i = 0; i < 16; ++i) rand[i] = (uint8_t)i; }
    if (do_atr() != 0) return 1;
    if (sel(0x3F00) < 0) { fprintf(stderr, "auth: SELECT MF failed\n"); return 1; }
    if (sel(0x7F20) < 0) { fprintf(stderr, "auth: SELECT DF_GSM failed\n"); return 1; }
    hexdump("RAND", rand, 16);
    uint8_t hdr[5] = { 0xA0, 0x88, 0x00, 0x00, 0x10 };
    uint8_t out[256]; int olen = (int)sizeof out; uint8_t sw1, sw2;
    if (sim_bridge_apdu(hdr, SIMB_DIR_IN, rand, 16, out, &olen, &sw1, &sw2) < 0) {
        fprintf(stderr, "auth: 0x88 exchange failed\n"); return 1; }
    printf("RUN GSM ALGORITHM -> SW %02X%02X\n", sw1, sw2);
    if (sw1 != 0x9F) { printf("(card did not return 9F xx — no SRES/Kc available)\n"); return 1; }
    int le = sw2; uint8_t gr[256]; int gl = (int)sizeof gr;
    uint8_t ghdr[5] = { 0xA0, 0xC0, 0x00, 0x00, (uint8_t)le };
    if (sim_bridge_apdu(ghdr, SIMB_DIR_OUT, NULL, 0, gr, &gl, &sw1, &sw2) < 0) {
        fprintf(stderr, "auth: GET RESPONSE failed\n"); return 1; }
    printf("GET RESPONSE le=%d -> SW %02X%02X\n", le, sw1, sw2);
    if (gl >= 12) { hexdump("SRES", gr, 4); hexdump("Kc", gr + 4, 8); }
    else hexdump("AUTH-RESP", gr, gl);
    return 0;
}

/* SELECT + FCI dump. Returns the FCI length, or <0 on select failure; *fci_len
 * may be 0 if the card gave 90 00 with nothing to fetch. */
static int sel_fci(uint16_t fid, uint8_t* fci, int cap) {
    uint8_t hdr[5] = { 0xA0, 0xA4, 0x00, 0x00, 0x02 };
    uint8_t fdat[2] = { (uint8_t)(fid >> 8), (uint8_t)(fid & 0xFF) };
    uint8_t sw1, sw2, out[256]; int olen = (int)sizeof out;
    if (sim_bridge_apdu(hdr, SIMB_DIR_IN, fdat, 2, out, &olen, &sw1, &sw2) < 0) return -1;
    if (sw1 != 0x90 && sw1 != 0x61 && sw1 != 0x9F) {
        printf("SELECT %04X -> SW %02X%02X (absent/blocked)\n", fid, sw1, sw2);
        return -1;
    }
    int n = 0;
    if (sw1 == 0x9F || sw1 == 0x61) {
        n = sw2 < cap ? sw2 : cap;
        int gl = n;
        if (get_response(n, fci, &gl) == 0) n = gl; else n = 0;
    }
    char tag[24]; snprintf(tag, sizeof tag, "SELECT %04X FCI", fid);
    hexdump(tag, fci, n);
    return n;
}

/* Card profile: ATR + MF/DF FCIs (CHV status) + the boot/acceptance-relevant EFs.
 * EF sizes come from each file's own FCI (bytes 2..3), capped for the dump. */
static int do_profile(void) {
    int rc = do_atr();
    if (rc != 0) return rc;
    uint8_t fci[64];
    static const struct { uint16_t df, ef; const char* name; } EFS[] = {
        { 0x3F00, 0x2FE2, "ICCID"   },
        { 0x7F20, 0x6F07, "IMSI"    },
        { 0x7F20, 0x6FAE, "PHASE"   },
        { 0x7F20, 0x6FAD, "AD"      },
        { 0x7F20, 0x6F38, "SST"     },
        { 0x7F20, 0x6F3E, "GID1"    },
        { 0x7F20, 0x6F3F, "GID2"    },
        { 0x7F20, 0x6F46, "SPN"     },
        { 0x7F20, 0x6F78, "ACC"     },
        { 0x7F20, 0x6F7E, "LOCI"    },
        { 0x7F20, 0x6F20, "KC"      },
        { 0x3F00, 0x2F05, "ELP"     },
        { 0x7F20, 0x6F05, "LP"      },
        { 0x7F20, 0x6F30, "PLMNsel" },
        { 0x7F20, 0x6F31, "HPLMN"   },
        { 0x7F20, 0x6FB7, "ECC"     },
        { 0x7F10, 0x6F3A, "ADN"     },
        { 0x7F10, 0x6F3C, "SMS"     },
        { 0x7F10, 0x6F42, "SMSP"    },
        { 0x7F10, 0x6F43, "SMSS"    },
        { 0x7F10, 0x6F40, "MSISDN"  },
    };
    printf("---- MF / DF_GSM status (FCI byte 13 = file characteristics; "
           "bit8 set = CHV1 DISABLED) ----\n");
    if (sel_fci(0x3F00, fci, sizeof fci) < 0) return 1;
    int have_gsm = sel_fci(0x7F20, fci, sizeof fci) >= 0;
    printf("---- EFs ----\n");
    for (size_t i = 0; i < sizeof EFS / sizeof EFS[0]; ++i) {
        if (EFS[i].df == 0x7F20 && !have_gsm) continue;
        if (sel_fci(0x3F00, fci, sizeof fci) < 0) return 1;     /* re-home to MF */
        if (EFS[i].df != 0x3F00 && sel_fci(EFS[i].df, fci, sizeof fci) < 0) continue;
        int fn = sel_fci(EFS[i].ef, fci, sizeof fci);
        if (fn < 0) continue;
        int size = (fn >= 4) ? ((fci[2] << 8) | fci[3]) : 16;
        if (size > 64) size = 64;
        if (size <= 0) continue;
        uint8_t buf[256]; int n = (int)sizeof buf;
        if (read_binary(size, buf, &n) == 0) hexdump(EFS[i].name, buf, n);
    }
    return 0;
}

static int do_read_ef(uint16_t df, uint16_t ef, int le, const char* label) {
    if (sel(0x3F00) < 0) return 1;            /* MF */
    if (df != 0x3F00 && sel(df) < 0) return 1;
    if (sel(ef) < 0) return 1;
    uint8_t buf[256]; int n = (int)sizeof buf;
    if (read_binary(le, buf, &n) < 0) return 1;
    hexdump(label, buf, n);
    return 0;
}

static int hexparse(const char* s, uint8_t* out, int cap) {
    int n = 0;
    while (*s && n < cap) {
        while (*s && !isxdigit((unsigned char)*s)) s++;
        if (!*s) break;
        char b[3] = { s[0], s[1], 0 };
        if (!isxdigit((unsigned char)b[1])) return -1;
        out[n++] = (uint8_t)strtol(b, NULL, 16);
        s += 2;
    }
    return n;
}

/* Raw APDU: infer direction from INS data-in table (GSM 11.11). */
static int ins_has_indata(uint8_t ins) {
    switch (ins) {
        case 0xA4: case 0x20: case 0x2C: case 0x24: case 0x26: case 0x28:
        case 0xDC: case 0xD6: case 0x32: case 0x88: return 1;  /* case-3 */
        default: return 0;
    }
}

static int do_apdu(const char* hex) {
    uint8_t buf[260];
    int n = hexparse(hex, buf, sizeof buf);
    if (n < 5) { fprintf(stderr, "apdu: need >=5 hex bytes\n"); return 1; }
    uint8_t hdr[5]; memcpy(hdr, buf, 5);
    int dir, dlen = 0; const uint8_t* din = NULL;
    if (ins_has_indata(hdr[1])) {
        dir = SIMB_DIR_IN; din = buf + 5; dlen = n - 5;
        if (dlen != hdr[4]) fprintf(stderr, "warning: Lc=%d but %d data bytes given\n", hdr[4], dlen);
    } else {
        dir = hdr[4] ? SIMB_DIR_OUT : SIMB_DIR_NONE;
    }
    uint8_t sw1, sw2, out[256]; int olen = (int)sizeof out;
    if (sim_bridge_apdu(hdr, dir, din, dlen, out, &olen, &sw1, &sw2) < 0) {
        fprintf(stderr, "apdu exchange failed\n");
        return 1;
    }
    if (dir == SIMB_DIR_OUT) hexdump("DATA", out, olen);
    printf("SW %02X%02X\n", sw1, sw2);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
            "usage: %s <serial-dev> <cmd> [arg]\n"
            "  cmd: ping | atr | iccid | imsi | profile | apdu <hex> | auth [rand-hex]\n", argv[0]);
        return 64;
    }
    const char* dev = argv[1];
    const char* cmd = argv[2];

    if (sim_bridge_open(dev) < 0) return 1;

    int rc = 0;
    if (!strcmp(cmd, "ping")) {
        char id[64];
        if (sim_bridge_ping(id, sizeof id) < 0) { fprintf(stderr, "ping failed\n"); rc = 1; }
        else printf("reader: %s\n", id);
    } else if (!strcmp(cmd, "atr")) {
        rc = do_atr();
    } else if (!strcmp(cmd, "iccid")) {
        if ((rc = do_atr()) == 0) rc = do_read_ef(0x3F00, 0x2FE2, 10, "ICCID");
    } else if (!strcmp(cmd, "imsi")) {
        if ((rc = do_atr()) == 0) rc = do_read_ef(0x7F20, 0x6F07, 9, "IMSI");
    } else if (!strcmp(cmd, "profile")) {
        char id[64];
        if (sim_bridge_ping(id, sizeof id) == 0) printf("reader: %s\n", id);
        rc = do_profile();
    } else if (!strcmp(cmd, "all")) {
        char id[64];
        if (sim_bridge_ping(id, sizeof id) == 0) printf("reader: %s\n", id);
        rc = do_atr();
        if (rc == 0) {
            do_read_ef(0x3F00, 0x2FE2, 10, "ICCID");
            do_read_ef(0x7F20, 0x6F07, 9, "IMSI");
        }
    } else if (!strcmp(cmd, "apdu")) {
        if (argc < 4) { fprintf(stderr, "apdu: missing hex\n"); rc = 64; }
        else { if ((rc = do_atr()) == 0) rc = do_apdu(argv[3]); }
    } else if (!strcmp(cmd, "auth")) {
        rc = do_auth(argc >= 4 ? argv[3] : NULL);
    } else {
        fprintf(stderr, "unknown cmd '%s'\n", cmd);
        rc = 64;
    }

    sim_bridge_deactivate();
    sim_bridge_close();
    return rc;
}
