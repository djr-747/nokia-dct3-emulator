// DCT3 model registry + firmware-address signature resolver. See model.h.

#include "models/model.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Profiles (one translation unit each, under src/models/<model>/).
extern const ModelProfile model_3310;
// DCT3_MODEL_3310_ONLY (release/beta build): compile the registry down to just the
// 3310. This drops the link dependency on every other profile — and, transitively,
// on the C54x co-sim (only the serial-bus profiles 5110/6110/3210 reference it, under
// #ifndef __EMSCRIPTEN__). The 3310 uses the HLE DSP (src/mad2/dsp_default.c,dsp_rom4.c),
// so a 3310-only link needs no c54x at all. See Makefile `gui-release`.
#ifndef DCT3_MODEL_3310_ONLY
extern const ModelProfile model_8850;
extern const ModelProfile model_7110;
extern const ModelProfile model_3410;
extern const ModelProfile model_5210;
extern const ModelProfile model_3330;
extern const ModelProfile model_3350;
extern const ModelProfile model_5510;
extern const ModelProfile model_8250;   // NSM-3D — register BEFORE 8210 (prefix)
extern const ModelProfile model_8210;   // NSM-3
extern const ModelProfile model_8855;   // NSM-4
extern const ModelProfile model_8290;   // NSB-7 (US-band 8210)
extern const ModelProfile model_8890;   // NSB-6 (US-band 8850)
extern const ModelProfile model_5110;   // NSE-1 (serial-bus V1-DCT3 spike — external EEPROM)
extern const ModelProfile model_6110;   // NSE-3 (serial-bus, external 24C64 EEPROM)
extern const ModelProfile model_6210;   // NPE-3 (4 MB, in-flash EEPROM, 96x65)
extern const ModelProfile model_2100;   // NAM-2 (2 MB, 5210 personality + 3410 LCD)
extern const ModelProfile model_3210;   // NSE-8 (2 MB, serial-bus, external 24C128 EEPROM; ROM-4 DSP)
#endif

static const ModelProfile* const REGISTRY[] = {
    &model_3310,   // index 0 = default
#ifndef DCT3_MODEL_3310_ONLY
    &model_8850,
    &model_7110,
    &model_3410,   // 4 MB scaffold — geometry/map only; FwAddrs unresolved (no boot yet)
    &model_5210,   // 4 MB scaffold — NSM-5 v5.40, bring-up in progress
    &model_3330,   // 4 MB scaffold — NHM-6, 3310 HW (84x48, Family B)
    &model_3350,   // 4 MB scaffold — NHM-9, 3410 HW (96x65, Family C)
    &model_5510,   // 4 MB scaffold — NPM-5, 3310-based (LCD/keypad TBD)
    // NSM Family-A scaffolds (shared keypad + BLB-2 battery; DSP scratch unresolved).
    // 8250 (NSM-3D) MUST precede 8210 (NSM-3) — "NSM-3" is a prefix of "NSM-3D", and
    // model_detect is first-match-wins, so the 8250 image must match NSM-3D first.
    &model_8250,   // 2 MB — NSM-3D
    &model_8210,   // 2 MB — NSM-3
    &model_8855,   // 4 MB — NSM-4 (slide)
    &model_8290,   // 2 MB — NSB-7 (US-band 8210)
    &model_8890,   // 2 MB — NSB-6 (US-band 8850, slide)
    &model_5110,   // 1 MB — NSE-1 (serial-bus V1-DCT3 spike; external EEPROM / serial bus)
    &model_6110,   // 1 MB — NSE-3 (serial-bus, external 24C64 EEPROM; ROM-4 HLE DSP)
    &model_6210,   // 4 MB — NPE-3 (in-flash EEPROM, 96x65; 5210/7110-class)
    &model_2100,   // 2 MB — NAM-2 (5210 personality + 3410 LCD; scaffold)
    &model_3210,   // 2 MB — NSE-8 (serial-bus, external 24C128 EEPROM; ROM-4 DSP)
#endif
};
#define N_MODELS ((int)(sizeof(REGISTRY) / sizeof(REGISTRY[0])))

const ModelProfile* model_default(void) { return REGISTRY[0]; }
int                 model_count(void)   { return N_MODELS; }
const ModelProfile* model_at(int i)     { return (i >= 0 && i < N_MODELS) ? REGISTRY[i] : NULL; }

const ModelProfile* model_by_name(const char* name) {
    if (!name) return NULL;
    for (int i = 0; i < N_MODELS; i++)
        if (REGISTRY[i]->name && strcmp(REGISTRY[i]->name, name) == 0) return REGISTRY[i];
    return NULL;
}

// Plain substring search (no memmem dependency).
static int buf_contains(const uint8_t* hay, uint32_t hlen, const char* needle) {
    size_t nlen = strlen(needle);
    if (nlen == 0 || nlen > hlen) return 0;
    for (uint32_t i = 0; i + nlen <= hlen; i++)
        if (memcmp(hay + i, needle, nlen) == 0) return 1;
    return 0;
}

// The product/TYPE code lives in the canonical 32-byte MCU version header at flash
// offset 0x1FC ("V VV.VV\nDD-MM-YY\nTYPE\n", the same window dump_nokix.py hashes for
// the fw-id). Match the ident THERE when the header is present — like nokix/dump_nokix —
// so a stray product code elsewhere in the image can't cause a misdetect. (A 3410 dump
// carries a lone "NHM-5" in its PPM data that a whole-image search wrongly read as a
// 3310.) Images without the canonical header — e.g. a converted MCU+PPM 7110 dump whose
// 0x1FC is not a "V ..." string — fall back to whole-image search (legacy behaviour).
#define VER_HDR_OFF 0x1FCu
#define VER_HDR_LEN 0x20u
static int ident_matches(const uint8_t* flash, uint32_t len, const char* needle) {
    if (len >= VER_HDR_OFF + VER_HDR_LEN && flash[VER_HDR_OFF] == 'V')
        return buf_contains(flash + VER_HDR_OFF, VER_HDR_LEN, needle);
    return buf_contains(flash, len, needle);   // no canonical header → legacy whole-image
}

// The ident's string requirement is satisfied if it asks for no string, or ANY of its
// accepted codes (match / match2) is present in the header window.
static int ident_has_str(const FwIdent* id) {
    return (id->match && *id->match) || (id->match2 && *id->match2);
}
static int ident_str_ok(const FwIdent* id, const uint8_t* flash, uint32_t len) {
    if (!ident_has_str(id)) return 1;                                        // size-only ident
    if (id->match  && *id->match  && ident_matches(flash, len, id->match))  return 1;
    if (id->match2 && *id->match2 && ident_matches(flash, len, id->match2)) return 1;
    return 0;
}

// Autodetect from the raw loaded flash image: first profile whose FwIdent matches
// (size, if specified, AND product/version code in the 0x1FC header, if specified).
// NULL = no match.
const ModelProfile* model_detect(const uint8_t* flash, uint32_t len) {
    if (!flash || !len) return NULL;
    for (int i = 0; i < N_MODELS; i++) {
        const FwIdent* id = &REGISTRY[i]->ident;
        if (id->flash_size && id->flash_size != len) continue;
        if (!ident_str_ok(id, flash, len)) continue;
        if ((id->flash_size && id->flash_size == len) || ident_has_str(id))
            return REGISTRY[i];
    }
    return NULL;
}

// --- Signature resolution -----------------------------------------------------

// Masked byte-signature search over the loaded flash (NokiX smart-match: a function
// body with operands/addresses wildcarded). Halfword-aligned. Returns the device
// address of the first match in [lo,hi), or 0 if none. (Generalized from main.c.)
static uint32_t sig_find(const uint8_t* ram, uint32_t mask, const uint8_t* pat,
                         const uint8_t* msk, int n, uint32_t lo, uint32_t hi) {
    for (uint32_t a = lo & ~1u; a + (uint32_t)n <= hi; a += 2) {
        int ok = 1;
        for (int i = 0; i < n; i++)
            if ((ram[(a + i) & mask] & msk[i]) != (pat[i] & msk[i])) { ok = 0; break; }
        if (ok) return a;
    }
    return 0;
}

// Decode a Thumb LDR-literal (LDR rd,[PC,#imm8*4]) at `at` and return the 32-bit pool
// constant (read big-endian, matching the BE core). Recovers the RAM base address a
// signature-located function operates on. (Generalized from main.c.)
static uint32_t sig_ldr_literal(const uint8_t* ram, uint32_t mask, uint32_t at) {
    uint32_t hw   = ((uint32_t)ram[at & mask] << 8) | ram[(at + 1) & mask];
    uint32_t pool = ((at + 4) & ~3u) + (hw & 0xFF) * 4;   // Align(PC,4) + imm8*4
    return ((uint32_t)ram[pool & mask] << 24) | ((uint32_t)ram[(pool + 1) & mask] << 16)
         | ((uint32_t)ram[(pool + 2) & mask] << 8) | ram[(pool + 3) & mask];
}

void model_resolve(const ModelProfile* prof, const uint8_t* ram, uint32_t ram_mask, FwAddrs* out) {
    if (!prof || !out) return;
    *out = prof->fw;                 // start from the constant fallbacks
    if (!ram) return;
    // First-hit-wins per field: a field may have several signatures (build-prologue
    // variants — e.g. reboot_fn with/without the early `bl notify`). Earlier table
    // entries win, so a later variant only fills a field an earlier sig missed.
    size_t done[64]; int ndone = 0;
    // Walk the shared base table then the optional per-model sigs2, sharing first-hit-wins.
    const SigResolve* tables[2] = { prof->sigs,   prof->sigs2 };
    int               counts[2] = { prof->n_sigs, prof->n_sigs2 };
    for (int t = 0; t < 2; t++) {
      const SigResolve* tbl = tables[t]; int ntbl = counts[t];
      if (!tbl) continue;
      for (int i = 0; i < ntbl; i++) {
        const SigResolve* sr = &tbl[i];
        const Sig* s = &sr->sig;
        if (s->kind == SIG_NONE || !s->pat || !s->mask || !s->len) continue;
        int already = 0;
        for (int k = 0; k < ndone; k++) if (done[k] == sr->field_off) { already = 1; break; }
        if (already) continue;
        uint32_t lo = s->lo ? s->lo : prof->mem.flash_base;
        uint32_t hi = s->hi ? s->hi : (prof->mem.flash_base + prof->mem.flash_size);

        uint32_t val;
        if (s->kind == SIG_CODE) {
            uint32_t m = sig_find(ram, ram_mask, s->pat, s->mask, s->len, lo, hi);
            if (!m) continue;
            val = m;
        } else if (s->kind == SIG_CODE_VLIT) {
            // Ambiguous leaf pattern: accept the match whose loaded literal equals an
            // already-resolved field (verify_off). Loop sig_find past non-verifying hits.
            uint32_t want = *(uint32_t*)((char*)out + s->verify_off);
            uint32_t m = 0, cur = lo;
            while ((m = sig_find(ram, ram_mask, s->pat, s->mask, s->len, cur, hi)) != 0) {
                if (sig_ldr_literal(ram, ram_mask, m + s->lit_off) == want) break;
                cur = m + 2;
            }
            if (!m) continue;
            val = m;
        } else { // SIG_LITERAL
            uint32_t m = sig_find(ram, ram_mask, s->pat, s->mask, s->len, lo, hi);
            if (!m) continue;
            uint32_t lit = sig_ldr_literal(ram, ram_mask, m + s->lit_off);
            // Only accept a literal that lands in the RAM scratch window [io_hi, flash_base).
            if (lit < prof->mem.io_hi || lit >= prof->mem.flash_base) continue;
            val = (uint32_t)((int32_t)lit + s->addend);
        }
        *(uint32_t*)((char*)out + sr->field_off) = val;
        if (ndone < (int)(sizeof(done)/sizeof(done[0]))) done[ndone++] = sr->field_off;
      }
    }
    // SIGDUMP: always print which signatures resolved per build (MISS flags the unresolved)
    // so a new firmware image immediately shows what still needs locating. Both tables.
    fprintf(stderr, "[sigdump] %s:", prof->name ? prof->name : "?");
    for (int t = 0; t < 2; t++) {
      const SigResolve* tbl = tables[t]; int ntbl = counts[t];
      if (!tbl) continue;
      for (int i = 0; i < ntbl; i++) {
        const SigResolve* sr = &tbl[i];
        int dup = 0;   // a field with variant sigs appears once (across both tables)
        for (int tj = 0; tj <= t; tj++) {
            const SigResolve* tbj = tables[tj]; if (!tbj) continue;
            int hi2 = (tj == t) ? i : counts[tj];
            for (int j = 0; j < hi2; j++) if (tbj[j].field_off == sr->field_off) { dup = 1; break; }
            if (dup) break;
        }
        if (dup) continue;
        uint32_t v = *(uint32_t*)((char*)out + sr->field_off);
        if (v) fprintf(stderr, " %s=0x%06X", sr->name, v);
        else   fprintf(stderr, " %s=MISS", sr->name);
      }
    }
    fprintf(stderr, "\n");
}

// The model's matrix line for a logical key, or NULL if absent. The single shared
// KK_*->KeyLine lookup (EmuHost emu_keyline). Harness input TIMING differs, but the
// lookup is one place. No aliasing — a missing key is missing.
const KeyLine* emu_keyline(const ModelProfile* prof, int id) {
    if (!prof || !prof->keypad.lines || id <= KK_NONE || id >= KK_COUNT) return NULL;
    for (int i = 0; i < prof->keypad.n_lines; i++)
        if (prof->keypad.lines[i].id == id) return &prof->keypad.lines[i];
    return NULL;
}

// Capability query: boolean form of emu_keyline (does this model map the key?).
int model_key_present(const ModelProfile* prof, int id) {
    return emu_keyline(prof, id) != NULL;
}
