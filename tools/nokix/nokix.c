/* nokix.c — Regina REXX driver for running NokiX scripts against a .fls.
 *
 * Usage: nokix [-b BASE] [-o OUT.fls] [-v] <firmware.fls> <script.nrx> [args...]
 *
 *   -b BASE   CPU base address of flash (default 0x200000 for DCT3).
 *   -o OUT    If script modifies flash, save here (default <input>.patched.fls).
 *   -v        Verbose: log every host command and function call.
 *
 * DCT3 firmware is big-endian word-invariant: byte at file offset A is the
 * MSB of the 32-bit word at CPU address (base + A). See dct3-big-endian
 * memory.
 *
 * Bridge model:
 *   - ADDRESS NOKIX commands (getbyte / setbyte / find / findbl / ...) flow
 *     through a SUBCOM handler; result lands in REXX special var `rc`.
 *   - Function calls (getenv / setenv / dand / dshr) flow through
 *     RexxRegisterFunctionExe handlers.
 *   - Commands taking binary blobs by reference (e.g. `find ... "patt" "mask"`)
 *     fetch the named variables via RexxVariablePool.
 */

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE   /* for strcasecmp */

#define INCL_RXSUBCOM
#define INCL_RXFUNC
#define INCL_RXSHV
#include <rexxsaa.h>

#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t *g_flash;
static size_t   g_flash_size;
static uint32_t g_flash_base = 0x200000;   /* DCT3 default. */
static int      g_verbose;
static int      g_dirty;                   /* flash modified ⇒ save on exit */
static char    *g_outpath_resolved;        /* destination .fls (resolved early) */

static int save_flash(const char *path);   /* fwd decl for FAIL handler */

/* Free-flash bump allocator for `create`.  Initialised by scanning the loaded
 * flash for the longest run of 0xFF, which is the post-code padding before
 * the FFS partitions on DCT3 firmwares. */
static uint32_t g_free_flash;          /* next free CPU addr (4-byte aligned) */
static uint32_t g_free_flash_end;      /* one past the end of the free region */

/* Free-RAM bump allocator for `reserve_ram`.  RAM lives outside the flash
 * file but is part of the model's address space; we just hand out addresses,
 * never write. */
static uint32_t g_free_ram      = 0x00130000;   /* 3310 default — above .data/.bss */
static uint32_t g_free_ram_end  = 0x00140000;

/* Deferred-execution queue for `runatend`.  Each entry runs a script's named
 * label after the main script returns, before we save the patched flash. */
struct atend { char *script; char *label; char *arg; struct atend *next; };
static struct atend *g_atend_head, *g_atend_tail;

/* LPCS dictionary cache for PPM TEXT subchunk codec.  Populated on first
 * DUMPLANG call that supplies the lpcs variable.  Each entry maps a byte
 * (0x00..0xFF) to a 16-bit Unicode codepoint (BE).  The reverse map lets
 * BUILDLANG re-encode UTF-8 strings back to LPCS bytes. */
static uint8_t  g_lpcs[512];
static int      g_lpcs_loaded;
static uint8_t  g_lpcs_rev[0x10000];   /* codepoint -> byte+1 (0 = none) */

/* ---------- env kv store (binary-safe) ---------- */

struct kv { char *key; char *val; size_t vlen; struct kv *next; };
static struct kv *g_env;

/* NokiX env vars are case-insensitive — `arg func` uppercases lookups while
 * `lvar` from .txt sidecar files stays lowercase, and they must match.
 * We canonicalise to lowercase on every set and lookup. */
static int ieq(const char *a, const char *b) {
    while (*a && *b) {
        int ca = (unsigned char)*a, cb = (unsigned char)*b;
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static void env_set_n(const char *k, const void *v, size_t n)
{
    for (struct kv *e = g_env; e; e = e->next) {
        if (ieq(e->key, k)) {
            free(e->val);
            e->val = malloc(n + 1);
            memcpy(e->val, v, n);
            e->val[n] = 0;
            e->vlen = n;
            return;
        }
    }
    struct kv *e = malloc(sizeof *e);
    e->key = strdup(k);
    e->val = malloc(n + 1);
    memcpy(e->val, v, n);
    e->val[n] = 0;
    e->vlen = n;
    e->next = g_env;
    g_env = e;
}

static void env_set(const char *k, const char *v) { env_set_n(k, v, strlen(v)); }

static const char *env_get(const char *k)
{
    for (struct kv *e = g_env; e; e = e->next)
        if (ieq(e->key, k)) return e->val;
    return NULL;
}

static const char *env_get_n(const char *k, size_t *out_len)
{
    for (struct kv *e = g_env; e; e = e->next)
        if (ieq(e->key, k)) {
            if (out_len) *out_len = e->vlen;
            return e->val;
        }
    if (out_len) *out_len = 0;
    return NULL;
}

/* ---------- flash access ---------- */

static long flash_off(uint32_t cpu_addr)
{
    if (cpu_addr < g_flash_base) return -1;
    long off = (long)(cpu_addr - g_flash_base);
    if ((size_t)off >= g_flash_size) return -1;
    return off;
}

static uint32_t fread_byte(uint32_t a) { long o = flash_off(a); return o < 0 ? 0 : g_flash[o]; }
static uint32_t fread_word(uint32_t a) {
    long o = flash_off(a);
    if (o < 0 || (size_t)o + 1 >= g_flash_size) return 0;
    return (g_flash[o] << 8) | g_flash[o + 1];
}
static uint32_t fread_long(uint32_t a) {
    long o = flash_off(a);
    if (o < 0 || (size_t)o + 3 >= g_flash_size) return 0;
    return ((uint32_t)g_flash[o] << 24) | ((uint32_t)g_flash[o+1] << 16)
         | ((uint32_t)g_flash[o+2] << 8) | g_flash[o+3];
}
static void fwrite_byte(uint32_t a, uint8_t v) {
    long o = flash_off(a); if (o < 0) return; g_flash[o] = v; g_dirty = 1;
}
static void fwrite_word(uint32_t a, uint16_t v) {
    long o = flash_off(a);
    if (o < 0 || (size_t)o + 1 >= g_flash_size) return;
    g_flash[o] = v >> 8; g_flash[o+1] = v & 0xFF; g_dirty = 1;
}
static void fwrite_long(uint32_t a, uint32_t v) {
    long o = flash_off(a);
    if (o < 0 || (size_t)o + 3 >= g_flash_size) return;
    g_flash[o]   = v >> 24; g_flash[o+1] = (v >> 16) & 0xFF;
    g_flash[o+2] = (v >> 8) & 0xFF; g_flash[o+3] = v & 0xFF;
    g_dirty = 1;
}

/* ---------- arg parsing ---------- */

/* REXX numbers come in as ASCII (decimal, or 0x-prefixed hex). */
static int parse_u32(const char *s, size_t n, uint32_t *out)
{
    char buf[32];
    if (n == 0 || n >= sizeof buf) return -1;
    memcpy(buf, s, n); buf[n] = 0;
    errno = 0;
    char *end;
    int base = (n > 1 && buf[0] == '0' && (buf[1] == 'x' || buf[1] == 'X')) ? 16 : 10;
    unsigned long v = strtoul(buf, &end, base);
    if (errno || end == buf) return -1;
    *out = (uint32_t)v;
    return 0;
}

/* ---------- RXSTRING helpers ---------- */

static void ret_long(PRXSTRING ret, long v)
{
    char buf[32];
    int n = snprintf(buf, sizeof buf, "%ld", v);
    if (ret->strptr && (size_t)n < 256) {
        memcpy(ret->strptr, buf, n); ret->strlength = n;
    } else {
        ret->strptr = malloc(n + 1);
        memcpy(ret->strptr, buf, n); ret->strptr[n] = 0; ret->strlength = n;
    }
}

static void ret_bytes(PRXSTRING ret, const void *data, size_t n)
{
    if (ret->strptr && n < 256) {
        memcpy(ret->strptr, data, n); ret->strlength = n;
    } else {
        ret->strptr = malloc(n ? n : 1);
        if (n) memcpy(ret->strptr, data, n);
        ret->strlength = n;
    }
}

/* Fetch a REXX variable's value as raw bytes.  Caller must free *out_val. */
static int rexx_var_fetch(const char *name, char **out_val, size_t *out_len)
{
    SHVBLOCK shv = {0};
    shv.shvcode = RXSHV_FETCH;
    MAKERXSTRING(shv.shvname, (char *)name, strlen(name));
    shv.shvnamelen = strlen(name);
    shv.shvvalue.strptr = NULL;
    shv.shvvalue.strlength = 0;
    APIRET rc = RexxVariablePool(&shv);
    if (rc != 0 || !shv.shvvalue.strptr) {
        *out_val = NULL; *out_len = 0;
        return -1;
    }
    *out_val = malloc(shv.shvvalue.strlength);
    memcpy(*out_val, shv.shvvalue.strptr, shv.shvvalue.strlength);
    *out_len = shv.shvvalue.strlength;
    RexxFreeMemory(shv.shvvalue.strptr);
    return 0;
}

/* ---------- ARM Thumb instruction codecs ---------- */

/* Decode a 32-bit Thumb BL/BLX pair at addr.  Returns target on success, 0 on
 * failure (and *ok cleared).  addr is the address of the first halfword. */
static uint32_t thumb_bl_decode(uint32_t addr, int *ok)
{
    *ok = 0;
    uint32_t hw1 = fread_word(addr);
    uint32_t hw2 = fread_word(addr + 2);
    if ((hw1 & 0xF800) != 0xF000) return 0;            /* not BL hi-half */
    if ((hw2 & 0xF800) != 0xF800 &&                    /* not BL lo-half */
        (hw2 & 0xF800) != 0xE800) return 0;            /* nor BLX */
    int32_t imm22 = ((hw1 & 0x7FF) << 11) | (hw2 & 0x7FF);
    if (imm22 & (1 << 21)) imm22 |= ~((1 << 22) - 1); /* sign-extend */
    uint32_t target = (addr + 4) + (uint32_t)(imm22 << 1);
    if ((hw2 & 0xF800) == 0xE800) target &= ~3u;       /* BLX: align */
    *ok = 1;
    return target;
}

static void thumb_bl_encode(uint32_t addr, uint32_t target)
{
    int32_t off = (int32_t)(target - (addr + 4));
    int32_t imm22 = (off >> 1) & ((1 << 22) - 1);
    uint16_t hw1 = 0xF000 | ((imm22 >> 11) & 0x7FF);
    uint16_t hw2 = 0xF800 | (imm22 & 0x7FF);
    fwrite_word(addr, hw1);
    fwrite_word(addr + 2, hw2);
}

/* Scan halfwords from `from` looking for the next BL.  If want_target != 0,
 * only return matches whose decoded target equals want_target.  Returns
 * address of the BL instruction, or 0 if not found. */
static uint32_t thumb_bl_find(uint32_t from, uint32_t want_target)
{
    /* Halfword-align: BLs are 2-byte aligned. */
    if (from & 1) from++;
    long from_off = flash_off(from);
    if (from_off < 0) return 0;
    for (size_t off = (size_t)from_off; off + 3 < g_flash_size; off += 2) {
        uint16_t hw1 = (g_flash[off] << 8) | g_flash[off + 1];
        if ((hw1 & 0xF800) != 0xF000) continue;
        uint16_t hw2 = (g_flash[off + 2] << 8) | g_flash[off + 3];
        if ((hw2 & 0xF800) != 0xF800 && (hw2 & 0xF800) != 0xE800) continue;
        uint32_t addr = g_flash_base + (uint32_t)off;
        if (want_target == 0) return addr;
        int ok; uint32_t t = thumb_bl_decode(addr, &ok);
        if (ok && t == want_target) return addr;
    }
    return 0;
}

/* Thumb unconditional B (1110 0 imm11) and conditional B<cc> (1101 cond imm8).
 * Decode the branch at addr and return its target.  Sets *ok on success. */
static uint32_t thumb_b_decode(uint32_t addr, int *ok)
{
    *ok = 0;
    uint32_t hw = fread_word(addr);
    int32_t off;
    if ((hw & 0xF800) == 0xE000) {
        int32_t imm11 = hw & 0x7FF;
        if (imm11 & (1 << 10)) imm11 |= ~((1 << 11) - 1);   /* sign-extend */
        off = imm11 << 1;
    } else if ((hw & 0xF000) == 0xD000 && (hw & 0x0F00) != 0x0F00) {
        /* Cond B (0xDFxx is SWI, exclude it). */
        int32_t imm8 = hw & 0xFF;
        if (imm8 & (1 << 7)) imm8 |= ~((1 << 8) - 1);
        off = imm8 << 1;
    } else {
        return 0;
    }
    *ok = 1;
    return (addr + 4) + (uint32_t)off;
}

/* Encode an unconditional Thumb B at addr pointing to target. */
static void thumb_b_encode(uint32_t addr, uint32_t target)
{
    int32_t off = (int32_t)(target - (addr + 4));
    int32_t imm11 = (off >> 1) & 0x7FF;
    uint16_t hw = 0xE000 | (imm11 & 0x7FF);
    fwrite_word(addr, hw);
}

static uint32_t thumb_b_find(uint32_t from, uint32_t want_target)
{
    if (from & 1) from++;
    long from_off = flash_off(from);
    if (from_off < 0) return 0;
    for (size_t off = (size_t)from_off; off + 1 < g_flash_size; off += 2) {
        uint16_t hw = (g_flash[off] << 8) | g_flash[off + 1];
        int is_b = ((hw & 0xF800) == 0xE000) ||
                   (((hw & 0xF000) == 0xD000) && (hw & 0x0F00) != 0x0F00);
        if (!is_b) continue;
        uint32_t addr = g_flash_base + (uint32_t)off;
        if (want_target == 0) return addr;
        int ok; uint32_t t = thumb_b_decode(addr, &ok);
        if (ok && t == want_target) return addr;
    }
    return 0;
}

/* LDR Rd, [PC, #imm8*4] — Thumb encoding 0x48xx..0x4Fxx.  Returns the literal
 * address (i.e. where the loaded value lives), 0 on failure. */
static uint32_t thumb_ldr_pc_decode(uint32_t addr, int *ok)
{
    *ok = 0;
    uint32_t hw = fread_word(addr);
    if ((hw & 0xF800) != 0x4800) return 0;
    uint32_t imm8 = hw & 0xFF;
    uint32_t target = ((addr + 4) & ~3u) + (imm8 << 2);
    *ok = 1;
    return target;
}

static uint32_t thumb_ldr_pc_find(uint32_t from)
{
    if (from & 1) from++;
    long from_off = flash_off(from);
    if (from_off < 0) return 0;
    for (size_t off = (size_t)from_off; off + 1 < g_flash_size; off += 2) {
        uint16_t hw = (g_flash[off] << 8) | g_flash[off + 1];
        if ((hw & 0xF800) == 0x4800)
            return g_flash_base + (uint32_t)off;
    }
    return 0;
}

/* findldr <start> <value> — next LDR-PC at/after start whose loaded 32-bit
 * literal == value, addr in rc.  NokiX kill_faid_check relies on this: each
 * match's literal is then rewritten (setlong), so it no longer matches and the
 * search advances on the next call (the loop terminates naturally). */
static uint32_t thumb_ldr_pc_find_val(uint32_t from, uint32_t want_val)
{
    if (from & 1) from++;
    long from_off = flash_off(from);
    if (from_off < 0) return 0;
    for (size_t off = (size_t)from_off; off + 1 < g_flash_size; off += 2) {
        uint16_t hw = (g_flash[off] << 8) | g_flash[off + 1];
        if ((hw & 0xF800) != 0x4800) continue;
        uint32_t addr = g_flash_base + (uint32_t)off;
        int ok; uint32_t lit = thumb_ldr_pc_decode(addr, &ok);
        if (ok && fread_long(lit) == want_val) return addr;
    }
    return 0;
}

/* ---------- SUBCOM handler ---------- */

/* Split "CMD arg1 arg2 ..." into uppercase verb + arg tokens. */
static int split_cmd(const char *s, size_t len,
                     char *verb, size_t vsz,
                     const char **argv_out, size_t *argl_out, int max)
{
    size_t i = 0;
    while (i < len && isspace((unsigned char)s[i])) i++;
    size_t vstart = i;
    while (i < len && !isspace((unsigned char)s[i])) i++;
    size_t vlen = i - vstart;
    if (vlen == 0 || vlen >= vsz) return -1;
    for (size_t k = 0; k < vlen; k++) verb[k] = toupper((unsigned char)s[vstart + k]);
    verb[vlen] = 0;
    int argc = 0;
    while (i < len && argc < max) {
        while (i < len && isspace((unsigned char)s[i])) i++;
        if (i >= len) break;
        /* Allow quoted args: "name" — strip quotes. */
        if (s[i] == '"') {
            i++;
            size_t astart = i;
            while (i < len && s[i] != '"') i++;
            argv_out[argc] = s + astart;
            argl_out[argc] = i - astart;
            argc++;
            if (i < len) i++; /* skip closing quote */
        } else {
            size_t astart = i;
            while (i < len && !isspace((unsigned char)s[i])) i++;
            argv_out[argc] = s + astart;
            argl_out[argc] = i - astart;
            argc++;
        }
    }
    return argc;
}

static APIRET APIENTRY nokix_subcom(PRXSTRING cmd, PUSHORT flags, PRXSTRING ret)
{
    *flags = RXSUBCOM_OK;
    ret->strlength = 0;

    char verb[32];
    const char *args[8]; size_t arglens[8];
    int argc = split_cmd(cmd->strptr, cmd->strlength, verb, sizeof verb, args, arglens, 8);
    if (argc < 0) {
        if (g_verbose) fprintf(stderr, "[nokix] malformed: %.*s\n",
                               (int)cmd->strlength, cmd->strptr);
        return 0;
    }
    if (g_verbose) fprintf(stderr, "[nokix] %.*s\n",
                           (int)cmd->strlength, cmd->strptr);

    uint32_t a, b;

#define ARG_U32(idx, dst) \
    (argc > (idx) && parse_u32(args[idx], arglens[idx], &(dst)) == 0)
#define ARG_NAME(idx, namebuf) ({ \
        int _ok = 0; \
        if (argc > (idx) && arglens[idx] < sizeof(namebuf)) { \
            memcpy(namebuf, args[idx], arglens[idx]); \
            namebuf[arglens[idx]] = 0; _ok = 1; \
        } _ok; })

    /* --- bounds & reads --- */

    if (!strcmp(verb, "CHECKBOUNDS")) {
        ret_long(ret, ARG_U32(0, a) && flash_off(a) >= 0 ? 1 : 0);
        return 0;
    }
    if (!strcmp(verb, "GETBYTE")) {
        ret_long(ret, ARG_U32(0, a) ? (long)fread_byte(a) : 0);
        return 0;
    }
    if (!strcmp(verb, "GETWORD")) {
        ret_long(ret, ARG_U32(0, a) ? (long)fread_word(a) : 0);
        return 0;
    }
    if (!strcmp(verb, "GETLONG")) {
        ret_long(ret, ARG_U32(0, a) ? (long)fread_long(a) : 0);
        return 0;
    }
    if (!strcmp(verb, "GETDATA")) {
        if (!ARG_U32(0, a)) return 0;
        long off = flash_off(a);
        if (off < 0) return 0;
        size_t n;
        if (argc >= 2) {
            uint32_t want;
            if (parse_u32(args[1], arglens[1], &want) != 0) return 0;
            n = want;
            if ((size_t)off + n > g_flash_size) n = g_flash_size - off;
        } else {
            n = 0;
            while ((size_t)off + n < g_flash_size && g_flash[off + n] != 0) n++;
        }
        ret_bytes(ret, g_flash + off, n);
        return 0;
    }

    /* --- writes --- */

    if (!strcmp(verb, "SETBYTE")) {
        if (ARG_U32(0, a) && ARG_U32(1, b)) fwrite_byte(a, (uint8_t)b);
        return 0;
    }
    if (!strcmp(verb, "SETWORD")) {
        if (ARG_U32(0, a) && ARG_U32(1, b)) fwrite_word(a, (uint16_t)b);
        return 0;
    }
    if (!strcmp(verb, "SETLONG")) {
        if (ARG_U32(0, a) && ARG_U32(1, b)) fwrite_long(a, b);
        return 0;
    }
    if (!strcmp(verb, "SETDATA")) {
        /* setdata <addr> "<varname>"   — value is binary in the named REXX var */
        if (!ARG_U32(0, a) || argc < 2) return 0;
        char namebuf[256];
        if (!ARG_NAME(1, namebuf)) return 0;
        char *val; size_t vlen;
        if (rexx_var_fetch(namebuf, &val, &vlen) < 0) return 0;
        long off = flash_off(a);
        if (off >= 0 && (size_t)off + vlen <= g_flash_size) {
            memcpy(g_flash + off, val, vlen);
            g_dirty = 1;
        }
        free(val);
        return 0;
    }

    /* --- code injection --- */

    /* create "<varname>" — append the binary value of REXX variable <varname>
     * to the free-flash pool, rc = CPU placement address.  This is NokiX's
     * model for injecting Thumb code, .rodata strings, etc. into firmware. */
    if (!strcmp(verb, "CREATE")) {
        if (argc < 1) { ret_long(ret, 0); return 0; }
        char namebuf[256];
        if (!ARG_NAME(0, namebuf)) { ret_long(ret, 0); return 0; }
        char *val = NULL; size_t vlen = 0;
        if (rexx_var_fetch(namebuf, &val, &vlen) < 0 || vlen == 0) {
            free(val); ret_long(ret, 0); return 0;
        }
        /* 4-byte align placement (Thumb literal pools assume word alignment). */
        uint32_t placement = (g_free_flash + 3u) & ~3u;
        if (placement + vlen > g_free_flash_end) {
            fprintf(stderr, "[nokix] create: out of free-flash pool "
                            "(want %zu bytes at 0x%X, pool ends at 0x%X)\n",
                    vlen, placement, g_free_flash_end);
            free(val); ret_long(ret, 0); return 0;
        }
        long off = flash_off(placement);
        if (off < 0) { free(val); ret_long(ret, 0); return 0; }
        memcpy(g_flash + off, val, vlen);
        g_dirty = 1;
        if (g_verbose)
            fprintf(stderr, "[nokix] create %s (%zu bytes) -> 0x%X\n",
                    namebuf, vlen, placement);
        g_free_flash = placement + (uint32_t)vlen;
        free(val);
        ret_long(ret, (long)placement);
        return 0;
    }

    /* --- PPM --- */

    /* release <addr> <len> — write 0xFF over a region (free flash). */
    if (!strcmp(verb, "RELEASE")) {
        if (!ARG_U32(0, a) || !ARG_U32(1, b)) return 0;
        long off = flash_off(a);
        if (off < 0) return 0;
        size_t n = b;
        if ((size_t)off + n > g_flash_size) n = g_flash_size - off;
        memset(g_flash + off, 0xFF, n);
        g_dirty = 1;
        /* If a larger contiguous 0xFF run is now available, retarget the
         * create() pool there.  We re-scan the entire flash. */
        size_t best_s = 0, best_l = 0, i = 0;
        while (i < g_flash_size) {
            if (g_flash[i] != 0xFF) { i++; continue; }
            size_t j = i;
            while (j < g_flash_size && g_flash[j] == 0xFF) j++;
            if (j - i > best_l) { best_s = i; best_l = j - i; }
            i = j;
        }
        if (best_l > (size_t)(g_free_flash_end - g_free_flash)) {
            g_free_flash     = g_flash_base + (uint32_t)best_s;
            g_free_flash_end = g_free_flash + (uint32_t)best_l;
            if (g_verbose)
                fprintf(stderr, "[nokix] release: free pool grew to "
                                "0x%X..0x%X (%zu KB)\n",
                        g_free_flash, g_free_flash_end, best_l / 1024);
        }
        return 0;
    }

    /* calcsum <addr> <len> — sum of 32-bit BE words.  Used for PPM chunk
     * checksums (verified: matches stored value for LPCS chunk). */
    if (!strcmp(verb, "CALCSUM")) {
        if (!ARG_U32(0, a) || !ARG_U32(1, b)) { ret_long(ret, 0); return 0; }
        long off = flash_off(a);
        if (off < 0) { ret_long(ret, 0); return 0; }
        uint32_t sum = 0;
        size_t end = off + b;
        if (end > g_flash_size) end = g_flash_size;
        for (size_t k = off; k + 3 < end; k += 4) {
            sum += ((uint32_t)g_flash[k]     << 24) |
                   ((uint32_t)g_flash[k + 1] << 16) |
                   ((uint32_t)g_flash[k + 2] <<  8) |
                   ((uint32_t)g_flash[k + 3]);
        }
        ret_long(ret, (long)sum);
        return 0;
    }

    /* dumplang <c1> <c2> <flags> "<data_var>" ["<lpcs_var>"]
     * Parse a TEXT subchunk: <length_array> 0xFF <strings>.  Decode each
     * string via the LPCS table (byte -> Unicode codepoint, UTF-8 output) if
     * supplied, otherwise pass-through.  Stash strings as
     * ppm/<c1>/<c2>/<i>/str env vars and the count as ppm/<c1>/<c2>/count. */
    if (!strcmp(verb, "DUMPLANG")) {
        uint32_t c1, c2;
        if (!ARG_U32(0, c1) || !ARG_U32(1, c2)) return 0;
        char dname[64];
        if (argc < 4 || !ARG_NAME(3, dname)) return 0;
        char *data = NULL; size_t dlen = 0;
        if (rexx_var_fetch(dname, &data, &dlen) < 0) return 0;
        /* Cache LPCS for later BUILDLANG use. */
        if (argc >= 5) {
            char lname[64];
            if (ARG_NAME(4, lname)) {
                char *lp = NULL; size_t ll = 0;
                if (rexx_var_fetch(lname, &lp, &ll) == 0 && ll >= 512) {
                    memcpy(g_lpcs, lp, 512);
                    memset(g_lpcs_rev, 0, sizeof g_lpcs_rev);
                    for (int b2 = 0; b2 < 256; b2++) {
                        uint16_t cp = (g_lpcs[b2*2] << 8) | g_lpcs[b2*2 + 1];
                        if (cp && g_lpcs_rev[cp] == 0) g_lpcs_rev[cp] = b2 + 1;
                    }
                    g_lpcs_loaded = 1;
                }
                free(lp);
            }
        }
        /* Locate the 0xFF sentinel.  Naive "first 0xFF" fails when strings
         * have length 255 (their length byte IS 0xFF and lives inside the
         * length array).  The real sentinel is the position N where
         *   sum(data[0..N-1]) + N + 1 == dlen
         * which uniquely identifies it. */
        size_t sent = (size_t)-1;
        size_t running = 0;
        for (size_t k = 0; k < dlen; k++) {
            if ((uint8_t)data[k] == 0xFF && running + k + 1 == dlen) {
                sent = k; break;
            }
            running += (uint8_t)data[k];
        }
        if (sent == (size_t)-1) { free(data); return 0; }
        size_t pos = sent + 1;
        char keybuf[128];
        int idx = 0;
        size_t total_stored = 0;
        /* Store strings as RAW LPCS bytes — the on-flash encoding can include
         * arbitrary control sequences (e.g. UCS-2 escape "\x04\xff..." with
         * embedded NULs).  Decoding to UTF-8 and back is lossy.  BUILDLANG
         * just emits stored bytes verbatim; new strings added via ADDSTRING
         * get UTF-8 → LPCS encoding at insert time. */
        for (size_t k = 0; k < sent; k++) {
            uint8_t L = (uint8_t)data[k];
            if (pos + L > dlen) break;
            snprintf(keybuf, sizeof keybuf, "ppm/%u/%u/%d/str", c1, c2, idx);
            env_set_n(keybuf, data + pos, L);
            total_stored += L;
            pos += L;
            idx++;
        }
        if (g_verbose)
            fprintf(stderr, "[nokix] dumplang c1=%u c2=%u dlen=%zu sent=%zu count=%d stored=%zu bytes\n",
                    c1, c2, dlen, sent, idx, total_stored);
        char cbuf[16];
        snprintf(cbuf, sizeof cbuf, "%d", idx);
        snprintf(keybuf, sizeof keybuf, "ppm/%u/%u/count", c1, c2);
        env_set(keybuf, cbuf);
        free(data);
        return 0;
    }

    /* buildlang <c1> <c2>
     * Inverse of dumplang.  Strings are stored as raw LPCS bytes by both
     * DUMPLANG (verbatim from flash) and ADDSTRING (UTF-8 → LPCS); we just
     * emit them as <lengths> 0xFF <strings>. */
    if (!strcmp(verb, "BUILDLANG")) {
        uint32_t c1, c2;
        if (!ARG_U32(0, c1) || !ARG_U32(1, c2)) return 0;
        char keybuf[128];
        snprintf(keybuf, sizeof keybuf, "ppm/%u/%u/count", c1, c2);
        const char *count_str = env_get(keybuf);
        if (!count_str) { ret_bytes(ret, "", 0); return 0; }
        int count = atoi(count_str);
        size_t *lens = calloc(count > 0 ? count : 1, sizeof *lens);
        const char **vals = calloc(count > 0 ? count : 1, sizeof *vals);
        size_t total_str = 0;
        for (int k = 0; k < count; k++) {
            snprintf(keybuf, sizeof keybuf, "ppm/%u/%u/%d/str", c1, c2, k);
            size_t l = 0;
            vals[k] = env_get_n(keybuf, &l);
            if (l > 255) l = 255;
            lens[k] = l;
            total_str += l;
        }
        size_t total = (size_t)count + 1 + total_str;
        char *buf = malloc(total > 0 ? total : 1);
        size_t p = 0;
        for (int k = 0; k < count; k++) buf[p++] = (char)lens[k];
        buf[p++] = (char)0xFF;
        for (int k = 0; k < count; k++) {
            if (vals[k] && lens[k]) memcpy(buf + p, vals[k], lens[k]);
            p += lens[k];
        }
        ret_bytes(ret, buf, total);
        free(buf);
        free(lens);
        free(vals);
        return 0;
    }

    /* addstring <subchunk_id> "<var>" [<system_id>]
     * Append (or, with system_id, replace) a string in TEXT subchunk
     * <subchunk_id>.  The new string text is in REXX variable <var>.
     * Returns the inserted string's sid in rc. */
    if (!strcmp(verb, "ADDSTRING")) {
        uint32_t sub_id, sys_id = 0;
        int have_sysid = 0;
        if (!ARG_U32(0, sub_id) || argc < 2) { ret_long(ret, 0); return 0; }
        char vname[64];
        if (!ARG_NAME(1, vname)) { ret_long(ret, 0); return 0; }
        if (argc >= 3 && ARG_U32(2, sys_id)) have_sysid = 1;

        /* Fetch the string's bytes from the named REXX var. */
        char *val = NULL; size_t vlen = 0;
        if (rexx_var_fetch(vname, &val, &vlen) < 0) { ret_long(ret, 0); return 0; }

        /* Find the TEXT chunk index.  PPMAN parses all chunks into
         * ppm/<i>, ppm/<i>/count etc. */
        int text_chunk = -1;
        const char *cnt = env_get("ppm/count");
        int top = cnt ? atoi(cnt) : 0;
        for (int i = 0; i < top; i++) {
            char k[32]; snprintf(k, sizeof k, "ppm/%d", i);
            const char *n = env_get(k);
            if (n && !strcmp(n, "TEXT")) { text_chunk = i; break; }
        }
        if (text_chunk < 0) { free(val); ret_long(ret, 0); return 0; }

        char ck[64];
        snprintf(ck, sizeof ck, "ppm/%d/%u/count", text_chunk, sub_id);
        const char *cs = env_get(ck);
        int cur = cs ? atoi(cs) : 0;

        int sid;
        if (have_sysid) {
            sid = (int)sys_id;
        } else {
            sid = cur;
            char nb[16]; snprintf(nb, sizeof nb, "%d", cur + 1);
            env_set(ck, nb);
        }

        /* Convert UTF-8 input to LPCS-encoded bytes for on-flash storage.
         * ASCII chars (< 0x80) are identity under LPCS; higher codepoints
         * are reverse-mapped through g_lpcs_rev. */
        uint8_t lpcs_buf[512];
        size_t lp = 0;
        for (size_t m = 0; m < vlen && lp < sizeof lpcs_buf; ) {
            uint8_t c0 = (uint8_t)val[m];
            uint32_t cp; int nb;
            if (c0 < 0x80)         { cp = c0; nb = 1; }
            else if ((c0 & 0xE0) == 0xC0 && m+1 < vlen) {
                cp = ((c0 & 0x1F) << 6) | (val[m+1] & 0x3F); nb = 2;
            } else if ((c0 & 0xF0) == 0xE0 && m+2 < vlen) {
                cp = ((c0 & 0x0F) << 12) | ((val[m+1] & 0x3F) << 6) | (val[m+2] & 0x3F); nb = 3;
            } else { cp = c0; nb = 1; }
            uint8_t b;
            if (g_lpcs_loaded && cp < 0x10000 && g_lpcs_rev[cp])
                b = (uint8_t)(g_lpcs_rev[cp] - 1);
            else if (cp < 256) b = (uint8_t)cp;
            else b = '?';
            lpcs_buf[lp++] = b;
            m += nb;
        }
        char strkey[96];
        snprintf(strkey, sizeof strkey, "ppm/%d/%u/%d/str", text_chunk, sub_id, sid);
        env_set_n(strkey, lpcs_buf, lp);
        if (g_verbose)
            fprintf(stderr, "[nokix] addstring sub=%u sid=%d count_now=%d "
                            "utf8_bytes=%zu lpcs_bytes=%zu\n",
                    sub_id, sid, cur + 1, vlen, lp);

        free(val);
        ret_long(ret, sid);
        return 0;
    }

    /* dumpfont / buildfont — stub: preserve binary data verbatim in
     * ppm/<c1>/<c2>/data env var. Font format isn't relevant for the
     * patches we support today. */
    if (!strcmp(verb, "DUMPFONT")) {
        uint32_t c1, c2;
        if (!ARG_U32(0, c1) || !ARG_U32(1, c2) || argc < 3) return 0;
        char dname[64];
        if (!ARG_NAME(2, dname)) return 0;
        char *data = NULL; size_t dlen = 0;
        if (rexx_var_fetch(dname, &data, &dlen) < 0) return 0;
        char keybuf[128];
        snprintf(keybuf, sizeof keybuf, "ppm/%u/%u/data", c1, c2);
        env_set_n(keybuf, data, dlen);
        free(data);
        return 0;
    }
    if (!strcmp(verb, "BUILDFONT")) {
        uint32_t c1, c2;
        if (!ARG_U32(0, c1) || !ARG_U32(1, c2)) return 0;
        char keybuf[128];
        snprintf(keybuf, sizeof keybuf, "ppm/%u/%u/data", c1, c2);
        size_t dlen = 0;
        const char *data = env_get_n(keybuf, &dlen);
        if (data) ret_bytes(ret, data, dlen);
        return 0;
    }

    /* --- ENV --- */

    if (!strcmp(verb, "SETENV")) {
        if (argc < 2) return 0;
        char keybuf[256], valbuf[1024];
        if (arglens[0] >= sizeof keybuf) return 0;
        memcpy(keybuf, args[0], arglens[0]); keybuf[arglens[0]] = 0;
        size_t vstart = args[1] - cmd->strptr;
        size_t vlen   = cmd->strlength - vstart;
        if (vlen >= sizeof valbuf) vlen = sizeof valbuf - 1;
        memcpy(valbuf, cmd->strptr + vstart, vlen); valbuf[vlen] = 0;
        env_set(keybuf, valbuf);
        return 0;
    }

    /* --- search --- */

    /* find / findfunc <start> "<patt_var>" "<mask_var>"
     * Looks for first position >= <start> where (flash[i+k] & mask[k]) ==
     * (patt[k] & mask[k]) for all k. rc = CPU addr, or 0 if not found.
     * findfunc additionally constrains matches to halfword-aligned (Thumb
     * function-entry) positions. */
    /* find / findfunc supports both "<start> patt mask" and "<start> patt"
     * (the no-mask form is exact match — equivalent to mask = all 0xFF). */
    if (!strcmp(verb, "FIND") || !strcmp(verb, "FINDFUNC")) {
        size_t step = strcmp(verb, "FINDFUNC") == 0 ? 2 : 1;
        if (!ARG_U32(0, a) || argc < 2) { ret_long(ret, 0); return 0; }
        char pname[256], mname[256];
        if (!ARG_NAME(1, pname)) { ret_long(ret, 0); return 0; }
        int have_mask = (argc >= 3) && ARG_NAME(2, mname);
        char *patt = NULL, *mask = NULL;
        size_t plen = 0, mlen = 0;
        if (rexx_var_fetch(pname, &patt, &plen) < 0 || plen == 0) {
            free(patt); ret_long(ret, 0); return 0;
        }
        if (have_mask) {
            if (rexx_var_fetch(mname, &mask, &mlen) < 0 || mlen != plen) {
                free(patt); free(mask); ret_long(ret, 0); return 0;
            }
        } else {
            /* No mask supplied → exact match (mask = 0xFF * plen). */
            mask = malloc(plen);
            memset(mask, 0xFF, plen);
            mlen = plen;
        }
        long from_off = flash_off(a);
        if (from_off < 0) { free(patt); free(mask); ret_long(ret, 0); return 0; }
        if (step == 2 && (from_off & 1)) from_off++;
        long found = -1;
        for (size_t off = (size_t)from_off; off + plen <= g_flash_size; off += step) {
            int hit = 1;
            for (size_t k = 0; k < plen; k++) {
                if ((g_flash[off + k] & (uint8_t)mask[k])
                    != ((uint8_t)patt[k] & (uint8_t)mask[k])) { hit = 0; break; }
            }
            if (hit) { found = off; break; }
        }
        free(patt); free(mask);
        ret_long(ret, found < 0 ? 0 : (long)(g_flash_base + found));
        return 0;
    }

    /* --- Thumb BL / LDR --- */

    /* findbl <start> [<target>] */
    if (!strcmp(verb, "FINDBL")) {
        if (!ARG_U32(0, a)) { ret_long(ret, 0); return 0; }
        uint32_t want = 0;
        if (argc >= 2) (void)ARG_U32(1, want);
        ret_long(ret, (long)thumb_bl_find(a, want));
        return 0;
    }
    /* getbl <addr> — decoded BL target into rc */
    if (!strcmp(verb, "GETBL")) {
        if (!ARG_U32(0, a)) { ret_long(ret, 0); return 0; }
        int ok; uint32_t t = thumb_bl_decode(a, &ok);
        ret_long(ret, ok ? (long)t : 0);
        return 0;
    }
    /* setbl <addr> <target> */
    if (!strcmp(verb, "SETBL")) {
        if (!ARG_U32(0, a) || !ARG_U32(1, b)) return 0;
        thumb_bl_encode(a, b);
        return 0;
    }
    /* findb <start> [<target>] — next Thumb B (uncond/cond) at/after start */
    if (!strcmp(verb, "FINDB")) {
        if (!ARG_U32(0, a)) { ret_long(ret, 0); return 0; }
        uint32_t want = 0;
        if (argc >= 2) (void)ARG_U32(1, want);
        ret_long(ret, (long)thumb_b_find(a, want));
        return 0;
    }
    /* getb <addr> — decoded B target */
    if (!strcmp(verb, "GETB")) {
        if (!ARG_U32(0, a)) { ret_long(ret, 0); return 0; }
        int ok; uint32_t t = thumb_b_decode(a, &ok);
        ret_long(ret, ok ? (long)t : 0);
        return 0;
    }
    /* setb <addr> <target> — encode unconditional Thumb B */
    if (!strcmp(verb, "SETB")) {
        if (!ARG_U32(0, a) || !ARG_U32(1, b)) return 0;
        thumb_b_encode(a, b);
        return 0;
    }
    /* findldr <start> [value] — next LDR-PC at/after start, addr in rc.
     * With a value arg, only match LDRs whose loaded 32-bit literal == value. */
    if (!strcmp(verb, "FINDLDR")) {
        if (!ARG_U32(0, a)) { ret_long(ret, 0); return 0; }
        if (argc >= 2 && ARG_U32(1, b))
            ret_long(ret, (long)thumb_ldr_pc_find_val(a, b));
        else
            ret_long(ret, (long)thumb_ldr_pc_find(a));
        return 0;
    }
    /* setnop <addr> — overwrite one Thumb halfword with NOP (MOV R8,R8 = 0x46C0,
     * the ARM7TDMI canonical NOP).  Used by kill_faid_check to defuse the
     * checksum-compare instruction after each patched LDR. */
    if (!strcmp(verb, "SETNOP")) {
        if (ARG_U32(0, a)) fwrite_word(a, 0x46C0);
        return 0;
    }
    /* getldr <addr> — literal-pool address that this LDR-PC loads from */
    if (!strcmp(verb, "GETLDR")) {
        if (!ARG_U32(0, a)) { ret_long(ret, 0); return 0; }
        int ok; uint32_t t = thumb_ldr_pc_decode(a, &ok);
        ret_long(ret, ok ? (long)t : 0);
        return 0;
    }

    /* --- control --- */

    /* runatend "<script>" "<label>" <arg>
     * Queue a deferred macro invocation.  After the main script returns we
     * run each queued entry as: RexxStart(script, [label, arg]).  This is
     * NokiX's mechanism for finalisation steps (e.g. ADD_5E0_ID.rx :THE_END
     * which writes the accumulated 5E0 table back to flash). */
    if (!strcmp(verb, "RUNATEND")) {
        if (argc < 2) return 0;
        char scriptbuf[1024], labelbuf[64], argbuf[64];
        if (arglens[0] >= sizeof scriptbuf) return 0;
        memcpy(scriptbuf, args[0], arglens[0]); scriptbuf[arglens[0]] = 0;
        if (arglens[1] >= sizeof labelbuf) return 0;
        memcpy(labelbuf, args[1], arglens[1]); labelbuf[arglens[1]] = 0;
        argbuf[0] = 0;
        if (argc >= 3 && arglens[2] < sizeof argbuf) {
            memcpy(argbuf, args[2], arglens[2]); argbuf[arglens[2]] = 0;
        }
        struct atend *e = malloc(sizeof *e);
        e->script = strdup(scriptbuf);
        e->label  = strdup(labelbuf);
        e->arg    = strdup(argbuf);
        e->next   = NULL;
        if (g_atend_tail) g_atend_tail->next = e; else g_atend_head = e;
        g_atend_tail = e;
        if (g_verbose)
            fprintf(stderr, "[nokix] runatend queued: %s :%s %s\n",
                    scriptbuf, labelbuf, argbuf);
        return 0;
    }

    /* NokiX `fail "msg"` is a hard abort.  Print, save any partial flash
     * modifications (so we can inspect what got written), then exit. */
    if (!strcmp(verb, "FAIL")) {
        fprintf(stderr, "FAIL: %.*s\n",
                (int)(cmd->strlength - 5),
                cmd->strptr + 5);
        if (g_dirty && g_outpath_resolved) {
            if (save_flash(g_outpath_resolved) == 0)
                fprintf(stderr,
                        "[nokix] partial patch saved to %s "
                        "(modifications up to fail point)\n",
                        g_outpath_resolved);
        }
        exit(1);
    }

    /* NokiX idiom: a bare function call as a statement (e.g.
     *   locate("mcu_version_string"); getdata rc
     * ) sends the function's return value through ADDRESS as a "command".
     * The expected behaviour is that `rc` ends up = that value.  We honour
     * this by echoing the verb back as `rc` when the whole command is a
     * single numeric token.  Anything else is logged as UNIMPLEMENTED. */
    if (argc == 0) {
        uint32_t v;
        if (parse_u32(verb, strlen(verb), &v) == 0) {
            ret_long(ret, (long)v);
            return 0;
        }
    }

    if (g_verbose) fprintf(stderr, "[nokix] UNIMPLEMENTED: %s (argc=%d)\n", verb, argc);
    return 0;
}

/* ---------- REXX functions ---------- */

static APIRET APIENTRY rxfn_getenv(PCSZ name, ULONG argc, PRXSTRING argv,
                                   PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    if (argc < 1 || !argv[0].strptr) { ret_bytes(ret, "", 0); return 0; }
    char keybuf[256];
    if (argv[0].strlength >= sizeof keybuf) { ret_bytes(ret, "", 0); return 0; }
    memcpy(keybuf, argv[0].strptr, argv[0].strlength);
    keybuf[argv[0].strlength] = 0;
    const char *v = env_get(keybuf);
    ret_bytes(ret, v ? v : "", v ? strlen(v) : 0);
    return 0;
}

static APIRET APIENTRY rxfn_setenv(PCSZ name, ULONG argc, PRXSTRING argv,
                                   PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    if (argc < 2) { ret_long(ret, -1); return 0; }
    char keybuf[256];
    if (argv[0].strlength >= sizeof keybuf) { ret_long(ret, -1); return 0; }
    memcpy(keybuf, argv[0].strptr, argv[0].strlength); keybuf[argv[0].strlength] = 0;
    /* Binary-safe + size-unlimited: PPM subchunk data blobs can be tens of
     * kilobytes (LDB lang subchunks are ~36 KB). */
    env_set_n(keybuf, argv[1].strptr, argv[1].strlength);
    ret_long(ret, 0);
    return 0;
}

/* Decimal-bitwise REXX builtins used by patch scripts. */
static APIRET APIENTRY rxfn_dand(PCSZ name, ULONG argc, PRXSTRING argv,
                                 PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    uint32_t x = 0, y = 0;
    if (argc >= 1) parse_u32(argv[0].strptr, argv[0].strlength, &x);
    if (argc >= 2) parse_u32(argv[1].strptr, argv[1].strlength, &y);
    ret_long(ret, (long)(x & y));
    return 0;
}
static APIRET APIENTRY rxfn_dor(PCSZ name, ULONG argc, PRXSTRING argv,
                                PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    uint32_t x = 0, y = 0;
    if (argc >= 1) parse_u32(argv[0].strptr, argv[0].strlength, &x);
    if (argc >= 2) parse_u32(argv[1].strptr, argv[1].strlength, &y);
    ret_long(ret, (long)(x | y));
    return 0;
}
static APIRET APIENTRY rxfn_dxor(PCSZ name, ULONG argc, PRXSTRING argv,
                                 PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    uint32_t x = 0, y = 0;
    if (argc >= 1) parse_u32(argv[0].strptr, argv[0].strlength, &x);
    if (argc >= 2) parse_u32(argv[1].strptr, argv[1].strlength, &y);
    ret_long(ret, (long)(x ^ y));
    return 0;
}
static APIRET APIENTRY rxfn_dshr(PCSZ name, ULONG argc, PRXSTRING argv,
                                 PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    uint32_t x = 0, n = 0;
    if (argc >= 1) parse_u32(argv[0].strptr, argv[0].strlength, &x);
    if (argc >= 2) parse_u32(argv[1].strptr, argv[1].strlength, &n);
    ret_long(ret, (long)(x >> n));
    return 0;
}
static APIRET APIENTRY rxfn_dshl(PCSZ name, ULONG argc, PRXSTRING argv,
                                 PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    uint32_t x = 0, n = 0;
    if (argc >= 1) parse_u32(argv[0].strptr, argv[0].strlength, &x);
    if (argc >= 2) parse_u32(argv[1].strptr, argv[1].strlength, &n);
    ret_long(ret, (long)(x << n));
    return 0;
}

/* swapenv(key1, key2) — swap the values of two env keys.  Returns 0. */
static APIRET APIENTRY rxfn_swapenv(PCSZ name, ULONG argc, PRXSTRING argv,
                                    PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    if (argc < 2) { ret_long(ret, -1); return 0; }
    char k1[256], k2[256];
    if (argv[0].strlength >= sizeof k1 || argv[1].strlength >= sizeof k2) {
        ret_long(ret, -1); return 0;
    }
    memcpy(k1, argv[0].strptr, argv[0].strlength); k1[argv[0].strlength] = 0;
    memcpy(k2, argv[1].strptr, argv[1].strlength); k2[argv[1].strlength] = 0;
    const char *v1 = env_get(k1);
    const char *v2 = env_get(k2);
    char *t1 = v1 ? strdup(v1) : strdup("");
    char *t2 = v2 ? strdup(v2) : strdup("");
    env_set(k1, t2);
    env_set(k2, t1);
    free(t1); free(t2);
    ret_long(ret, 0);
    return 0;
}

/* reserve_ram(size) — allocate `size` bytes of RAM, return CPU address. */
static APIRET APIENTRY rxfn_reserve_ram(PCSZ name, ULONG argc, PRXSTRING argv,
                                        PCSZ qname, PRXSTRING ret)
{
    (void)name; (void)qname;
    uint32_t n = 0;
    if (argc >= 1) parse_u32(argv[0].strptr, argv[0].strlength, &n);
    if (n == 0) n = 4;
    uint32_t addr = (g_free_ram + 3u) & ~3u;
    if (addr + n > g_free_ram_end) {
        if (g_verbose) fprintf(stderr, "[nokix] reserve_ram: pool exhausted\n");
        ret_long(ret, 0);
        return 0;
    }
    g_free_ram = addr + n;
    ret_long(ret, (long)addr);
    return 0;
}

/* ---------- C-side PPM writeback ---------- */

/* Encode a TEXT subchunk's strings from env vars into the on-flash format:
 *   <N length bytes> 0xFF <N concatenated strings>
 * UTF-8 codepoints are reverse-mapped through the LPCS dictionary. */
static char *ppm_build_text_sub(int c1, int c2, size_t *out_len)
{
    char ck[64];
    snprintf(ck, sizeof ck, "ppm/%d/%d/count", c1, c2);
    const char *cs = env_get(ck);
    int count = cs ? atoi(cs) : 0;
    /* Strings are stored as raw LPCS bytes (by DUMPLANG verbatim, by
     * ADDSTRING after UTF-8 → LPCS conversion).  Emit length array +
     * sentinel + concatenated strings unchanged. */
    size_t *lens = calloc(count > 0 ? count : 1, sizeof *lens);
    const char **vals = calloc(count > 0 ? count : 1, sizeof *vals);
    size_t total_str = 0;
    for (int k = 0; k < count; k++) {
        snprintf(ck, sizeof ck, "ppm/%d/%d/%d/str", c1, c2, k);
        size_t l = 0;
        vals[k] = env_get_n(ck, &l);
        if (l > 255) l = 255;
        lens[k] = l;
        total_str += l;
    }
    size_t total = (size_t)count + 1 + total_str;
    char *buf = malloc(total + 4);
    size_t p = 0;
    for (int k = 0; k < count; k++) buf[p++] = (char)lens[k];
    buf[p++] = (char)0xFF;
    for (int k = 0; k < count; k++) {
        if (vals[k] && lens[k]) memcpy(buf + p, vals[k], lens[k]);
        p += lens[k];
    }
    free(lens);
    free(vals);
    *out_len = total;
    return buf;
}

/* Append four big-endian bytes. */
static void be32_append(char **buf, size_t *cap, size_t *pos, uint32_t v) {
    if (*pos + 4 > *cap) {
        *cap = (*cap + 4) * 2;
        *buf = realloc(*buf, *cap);
    }
    (*buf)[(*pos)++] = (char)(v >> 24);
    (*buf)[(*pos)++] = (char)((v >> 16) & 0xFF);
    (*buf)[(*pos)++] = (char)((v >> 8) & 0xFF);
    (*buf)[(*pos)++] = (char)(v & 0xFF);
}
static void bytes_append(char **buf, size_t *cap, size_t *pos,
                         const void *src, size_t n) {
    if (*pos + n > *cap) {
        *cap = (*pos + n) * 2;
        *buf = realloc(*buf, *cap);
    }
    memcpy(*buf + *pos, src, n);
    *pos += n;
}
static void byte_append(char **buf, size_t *cap, size_t *pos, uint8_t v) {
    if (*pos + 1 > *cap) {
        *cap = (*cap + 1) * 2;
        *buf = realloc(*buf, *cap);
    }
    (*buf)[(*pos)++] = (char)v;
}

/* Rebuild the entire PPM block from env vars and write it back to flash.
 * Replaces PPMAN.rx :THE_END for the simple "add a TEXT string" case —
 * skips the in-phone language-menu rebuild which requires many model-
 * specific locators we don't have. */
static int ppm_writeback_c(void)
{
    /* Where does PPM go?  Read ppm_struct from flash. */
    const char *struct_str = env_get("locate/ppm_struct");
    if (!struct_str) {
        fprintf(stderr, "[nokix] ppm_writeback: locate/ppm_struct not set\n");
        return -1;
    }
    uint32_t struct_addr = (uint32_t)strtoul(struct_str, NULL, 10);
    uint32_t ppm_dst = fread_long(struct_addr);
    uint32_t ppm_end = fread_long(struct_addr + 4);
    if (g_verbose)
        fprintf(stderr, "[nokix] ppm_writeback: dst=0x%X end=0x%X\n",
                ppm_dst, ppm_end);

    /* Header: "PPM\0" + version(32) + unknown(4) + pack(4) */
    size_t cap = 0x100000, pos = 0;
    char *buf = malloc(cap);
    memcpy(buf, "PPM\0", 4); pos += 4;
    const char *ver = env_get("ppm/version");
    char hdr[36] = {0};
    if (ver) {
        size_t n = strlen(ver);
        memcpy(hdr, ver, n < 32 ? n : 32);
    }
    bytes_append(&buf, &cap, &pos, hdr, 32);
    bytes_append(&buf, &cap, &pos, "\xFF\xFF\xFF\xFF", 4);
    char pack[4] = {0};
    const char *pk = env_get("ppm/pack");
    if (pk) { size_t n = strlen(pk); memcpy(pack, pk, n < 4 ? n : 4); }
    bytes_append(&buf, &cap, &pos, pack, 4);

    /* Walk chunks */
    const char *cs = env_get("ppm/count");
    int c1_count = cs ? atoi(cs) : 0;
    if (g_verbose) fprintf(stderr, "[nokix] ppm_writeback: %d chunks\n", c1_count);
    for (int i1 = 0; i1 < c1_count; i1++) {
        if (g_verbose) {
            char k[32]; snprintf(k, sizeof k, "ppm/%d", i1);
            const char *cn = env_get(k);
            snprintf(k, sizeof k, "ppm/%d/count", i1);
            const char *cnt = env_get(k);
            fprintf(stderr, "[nokix]   chunk[%d] %s (subchunks=%s)\n",
                    i1, cn ? cn : "?", cnt ? cnt : "?");
            int sub_count = cnt ? atoi(cnt) : 0;
            for (int i2 = 0; i2 < sub_count; i2++) {
                snprintf(k, sizeof k, "ppm/%d/%d", i1, i2);
                const char *sn = env_get(k);
                snprintf(k, sizeof k, "ppm/%d/%d/data", i1, i2);
                size_t dl = 0;
                env_get_n(k, &dl);
                fprintf(stderr, "[nokix]     sub[%d] %s data_bytes=%zu\n",
                        i2, sn ? sn : "?", dl);
            }
        }
        size_t chunk_start = pos;
        be32_append(&buf, &cap, &pos, 0);                 /* checksum (filled later) */
        be32_append(&buf, &cap, &pos, 0);                 /* size (filled later) */
        char ck[64];
        snprintf(ck, sizeof ck, "ppm/%d", i1);
        const char *cname = env_get(ck);
        char nm[4] = {0};
        if (cname) { size_t n = strlen(cname); memcpy(nm, cname, n < 4 ? n : 4); }
        bytes_append(&buf, &cap, &pos, nm, 4);            /* name */
        snprintf(ck, sizeof ck, "ppm/%d/version", i1);
        const char *cv = env_get(ck);
        char cvers[8] = {0};
        if (cv) { size_t n = strlen(cv); memcpy(cvers, cv, n < 8 ? n : 8); }
        bytes_append(&buf, &cap, &pos, cvers, 8);         /* version */

        snprintf(ck, sizeof ck, "ppm/%d/count", i1);
        const char *sc = env_get(ck);
        int c2_count = sc ? atoi(sc) : 0;
        for (int i2 = 0; i2 < c2_count; i2++) {
            snprintf(ck, sizeof ck, "ppm/%d/%d/type", i1, i2);
            uint32_t type = strtoul(env_get(ck) ?: "0", NULL, 10);
            snprintf(ck, sizeof ck, "ppm/%d/%d", i1, i2);
            const char *sname = env_get(ck);
            char snm[4] = {0};
            if (sname) { size_t n = strlen(sname); memcpy(snm, sname, n < 4 ? n : 4); }
            snprintf(ck, sizeof ck, "ppm/%d/%d/flags", i1, i2);
            uint32_t flags = strtoul(env_get(ck) ?: "0", NULL, 10);

            /* Build subchunk data. */
            char *data = NULL;
            size_t dlen = 0;
            if (cname && !strcmp(cname, "TEXT") && type != 0) {
                data = ppm_build_text_sub(i1, i2, &dlen);
            } else {
                snprintf(ck, sizeof ck, "ppm/%d/%d/data", i1, i2);
                size_t l = 0;
                const char *d = env_get_n(ck, &l);
                if (d) { data = malloc(l); memcpy(data, d, l); dlen = l; }
            }

            size_t sub_size = 16 + dlen;
            be32_append(&buf, &cap, &pos, type);
            be32_append(&buf, &cap, &pos, (uint32_t)sub_size);
            bytes_append(&buf, &cap, &pos, snm, 4);
            /* flags: PLMN uses 4-byte flags, others 1+3 zeros */
            if (cname && !strcmp(cname, "PLMN")) {
                be32_append(&buf, &cap, &pos, flags);
            } else {
                byte_append(&buf, &cap, &pos, (uint8_t)flags);
                bytes_append(&buf, &cap, &pos, "\0\0\0", 3);
            }
            if (dlen) bytes_append(&buf, &cap, &pos, data, dlen);
            /* Pad to 4 bytes with 0xFF. */
            while ((pos - chunk_start) % 4) byte_append(&buf, &cap, &pos, 0xFF);
            free(data);
        }
        /* Empty terminator subchunk: 16 zero bytes. */
        for (int z = 0; z < 16; z++) byte_append(&buf, &cap, &pos, 0);

        /* Fill chunk size. */
        size_t chunk_len = pos - chunk_start;
        buf[chunk_start + 4] = (char)(chunk_len >> 24);
        buf[chunk_start + 5] = (char)((chunk_len >> 16) & 0xFF);
        buf[chunk_start + 6] = (char)((chunk_len >> 8) & 0xFF);
        buf[chunk_start + 7] = (char)(chunk_len & 0xFF);

        /* Pad chunk to even byte with 0xFF. */
        if (chunk_len % 2) byte_append(&buf, &cap, &pos, 0xFF);
    }
    /* DUMFILE terminator chunk (36 bytes). */
    static const uint8_t dumfile[36] = {
        0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x24,
        'D','U','M','F','I','L','E', 0,
        0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
        0,0,0,0,
    };
    bytes_append(&buf, &cap, &pos, dumfile, 36);

    /* Compute and patch chunk checksums = sum of 32-bit BE words over the
     * chunk content excluding the first 4 bytes. */
    size_t walk = 44;   /* skip PPM header */
    while (walk < pos) {
        if (walk + 8 > pos) break;
        size_t cstart = walk;
        uint32_t clen = ((uint8_t)buf[walk+4] << 24) | ((uint8_t)buf[walk+5] << 16)
                      | ((uint8_t)buf[walk+6] << 8)  | (uint8_t)buf[walk+7];
        if (clen == 0 || clen > pos - walk) break;
        uint32_t sum = 0;
        for (size_t k = cstart + 4; k + 3 < cstart + clen; k += 4) {
            sum += ((uint8_t)buf[k] << 24) | ((uint8_t)buf[k+1] << 16)
                 | ((uint8_t)buf[k+2] << 8) | (uint8_t)buf[k+3];
        }
        buf[cstart]   = (char)(sum >> 24);
        buf[cstart+1] = (char)((sum >> 16) & 0xFF);
        buf[cstart+2] = (char)((sum >> 8) & 0xFF);
        buf[cstart+3] = (char)(sum & 0xFF);
        walk += clen;
        if (clen % 2) walk++;   /* skip the 0xFF pad */
    }

    /* Write to flash at ppm_dst. */
    if (ppm_dst + pos > ppm_end) {
        fprintf(stderr, "[nokix] ppm_writeback: new PPM (%zu bytes) exceeds "
                "ppm_end - ppm_dst (%u bytes)\n", pos, ppm_end - ppm_dst);
        free(buf);
        return -1;
    }
    long off = flash_off(ppm_dst);
    if (off < 0) { free(buf); return -1; }
    memcpy(g_flash + off, buf, pos);
    g_dirty = 1;
    if (g_verbose)
        fprintf(stderr, "[nokix] ppm_writeback: wrote %zu bytes at 0x%X\n",
                pos, ppm_dst);
    free(buf);
    return 0;
}

/* ---------- driver ---------- */

static int load_flash(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    fseek(f, 0, SEEK_END);
    g_flash_size = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    g_flash = malloc(g_flash_size);
    if (!g_flash) { fclose(f); return -1; }
    if (fread(g_flash, 1, g_flash_size, f) != g_flash_size) {
        fclose(f); free(g_flash); g_flash = NULL; return -1;
    }
    fclose(f);
    return 0;
}

static int save_flash(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return -1; }
    int ok = fwrite(g_flash, 1, g_flash_size, f) == g_flash_size;
    fclose(f);
    return ok ? 0 : -1;
}

/* Build "<input>.patched.fls" — replaces final ".fls" with ".patched.fls" or
 * appends if no extension. */
static char *default_outname(const char *in)
{
    const char *dot = strrchr(in, '.');
    size_t blen = dot ? (size_t)(dot - in) : strlen(in);
    char *out = malloc(blen + sizeof ".patched.fls" + 8);
    memcpy(out, in, blen);
    strcpy(out + blen, ".patched.fls");
    return out;
}

int main(int argc, char **argv)
{
    const char *outpath = NULL;
    const char *model = NULL;
    int opt_i = 1;
    while (opt_i < argc && argv[opt_i][0] == '-') {
        if (!strcmp(argv[opt_i], "-v")) { g_verbose = 1; opt_i++; }
        else if (!strcmp(argv[opt_i], "-b") && opt_i + 1 < argc) {
            g_flash_base = (uint32_t)strtoul(argv[opt_i + 1], NULL, 0);
            opt_i += 2;
        } else if (!strcmp(argv[opt_i], "-o") && opt_i + 1 < argc) {
            outpath = argv[opt_i + 1]; opt_i += 2;
        } else if (!strcmp(argv[opt_i], "-m") && opt_i + 1 < argc) {
            model = argv[opt_i + 1]; opt_i += 2;
        } else if (!strcmp(argv[opt_i], "--")) { opt_i++; break; }
        else break;
    }
    if (argc - opt_i < 2) {
        fprintf(stderr,
                "usage: %s [-v] [-b BASE] [-o OUT.fls] <firmware.fls> <script.nrx> [args...]\n",
                argv[0]);
        return 2;
    }
    const char *flash_path  = argv[opt_i];
    const char *script_path = argv[opt_i + 1];

    if (load_flash(flash_path) < 0) return 2;
    if (g_verbose)
        fprintf(stderr, "[nokix] loaded %zu bytes from %s (base 0x%X)\n",
                g_flash_size, flash_path, g_flash_base);

    /* Resolve output path now so the FAIL handler can save partials. */
    g_outpath_resolved = outpath ? strdup(outpath) : default_outname(flash_path);

    /* Auto-detect the create() injection pool: the longest run of 0xFF bytes
     * in the loaded flash.  On DCT3 this is the post-code padding before the
     * FFS partitions (~240 KB on 3310 v5.79). */
    {
        size_t best_start = 0, best_len = 0;
        size_t i = 0;
        while (i < g_flash_size) {
            if (g_flash[i] != 0xFF) { i++; continue; }
            size_t j = i;
            while (j < g_flash_size && g_flash[j] == 0xFF) j++;
            if (j - i > best_len) { best_start = i; best_len = j - i; }
            i = j;
        }
        if (best_len < 0x100) {
            fprintf(stderr, "[nokix] no usable 0xFF pool found "
                            "(longest run = %zu bytes)\n", best_len);
        } else {
            g_free_flash     = g_flash_base + (uint32_t)best_start;
            g_free_flash_end = g_free_flash + (uint32_t)best_len;
            if (g_verbose)
                fprintf(stderr, "[nokix] create-pool: 0x%X..0x%X (%zu KB)\n",
                        g_free_flash, g_free_flash_end, best_len / 1024);
        }
    }

    /* sys/input_flash + sys/project_file + sys/macros_path feed FIND_FILE.rx's
     * search path so scripts can locate their sidecar resources (e.g.
     * anonymous_access_ppm.txt next to the script, locate_ppm_misc.txt in
     * the macros dir). */
    {
        char abs[4096];
        if (realpath(flash_path, abs)) env_set("sys/input_flash", abs);
        if (realpath(script_path, abs)) env_set("sys/project_file", abs);
        const char *macros = getenv("REGINA_MACROS");
        if (macros && realpath(macros, abs)) env_set("sys/macros_path", abs);
    }

    /* Parse the embedded MCU version string at fixed offset 0x1FC (CPU 0x2001FC).
     * Every DCT3 firmware we've seen carries this 32-byte header:
     *   "V VV.VV\n"          version (2 chars, space-padded when version < 10)
     *   "DD-MM-YY\n"         build date
     *   "<TYPE>\n"           e.g. "NHM-5" (3310), "NSM-2" (8850), "NSE-5" (7110)
     *   "(c) NMP."           tail marker
     * We derive sys/firmware_{type,version,date} from this, NOT from -m, so the
     * env vars NokiX's REXX sees match the loaded image. -m still wins on phone
     * model if provided (manual override for test fixtures / unusual builds).
     * Falls back to -m's hardcoded values if parsing fails (defensive). */
    {
        const size_t off = 0x1FC;
        char ftype[16] = "", fvers[8] = "", fdate[16] = "";
        char fmodel_from_type[16] = "";
        int parsed = 0;
        if (g_flash_size >= off + 32) {
            const unsigned char *h = g_flash + off;
            /* Bytes 0..7: "V VV.VV\n" — extract chars 2-6, strip leading space */
            if (h[0] == 'V' && h[1] == ' ' && h[7] == '\n') {
                fvers[0] = (h[2] == ' ') ? '0' : (char)h[2];
                fvers[1] = (char)h[3]; fvers[2] = '.';
                fvers[3] = (char)h[5]; fvers[4] = (char)h[6]; fvers[5] = 0;
                /* Bytes 8..16: "DD-MM-YY\n" — expand to "20YY-MM-DD" or "19YY-MM-DD".
                 * Years 80-99 = 19xx (very early DCT2), 00-79 = 20xx. */
                if (h[10] == '-' && h[13] == '-' && h[16] == '\n') {
                    int yy = (h[14] - '0') * 10 + (h[15] - '0');
                    int century = (yy >= 80) ? 19 : 20;
                    snprintf(fdate, sizeof fdate, "%d%02d-%c%c-%c%c",
                             century, yy, h[11], h[12], h[8], h[9]);
                    /* Bytes 17..: <TYPE>\n */
                    size_t i = 17, j = 0;
                    while (i < 32 && h[i] != '\n' && j < sizeof(ftype) - 1)
                        ftype[j++] = (char)h[i++];
                    ftype[j] = 0;
                    if (j > 0) parsed = 1;
                }
            }
        }
        if (parsed) {
            /* Map firmware_type code -> phone model (subset of NokiX's table in
             * LOCATE.rx:93). Extend as more DCT3 models come online. */
            static const struct { const char *type; const char *model; } map[] = {
                { "NHM-5", "3310" }, { "NHM-6", "3330" }, { "NHM-2", "3410" },
                { "NHM-3", "6210" }, { "NSE-5", "7110" }, { "NSM-2", "8850" },
                { "NSM-3", "8210" }, { "NSE-3", "6110" }, { "NSE-6", "8810" },
                { NULL, NULL }
            };
            for (int i = 0; map[i].type; i++) {
                if (!strcmp(ftype, map[i].type)) {
                    strncpy(fmodel_from_type, map[i].model, sizeof(fmodel_from_type) - 1);
                    break;
                }
            }
            env_set("sys/firmware_type",    ftype);
            env_set("sys/firmware_version", fvers);
            env_set("sys/firmware_date",    fdate);
            if (g_verbose)
                fprintf(stderr,
                        "[nokix] detected from flash@0x%lX: type=%s version=%s date=%s model=%s\n",
                        (unsigned long)(g_flash_base + off), ftype, fvers, fdate,
                        fmodel_from_type[0] ? fmodel_from_type : "(unknown — pass -m to override)");
        }

        /* Phone model: -m wins (explicit override), else use the auto-detected one,
         * else fall through to nothing (LOCATE.rx will fail with a clear error). */
        const char *pm = NULL;
        if (model)                    pm = model;
        else if (fmodel_from_type[0]) pm = fmodel_from_type;
        if (pm) env_set("sys/phone_model", pm);

        /* If we didn't parse the header (rare — non-DCT3 image, or stripped header)
         * AND the user passed -m, use the legacy hardcoded fallback so existing
         * scripts keep working on test fixtures. */
        if (!parsed && model) {
            if (!strcmp(model, "3310")) {
                env_set("sys/firmware_type",    "NHM-5");
                env_set("sys/firmware_version", "05.79");
                env_set("sys/firmware_date",    "2003-09-30");
            } else if (!strcmp(model, "8850")) {
                env_set("sys/firmware_type",    "NSM-2");
                env_set("sys/firmware_version", "05.31");
                env_set("sys/firmware_date",    "2002-06-01");
            }
        }

        /* C-side locators NokiX implements in C rather than via LOCATE_*.rx
         * (e.g. ui_task — no .rx definition anywhere). Per-build addresses; pre-
         * cached here so LOCATE.rx's func/ check short-circuits. v5.79-specific
         * for now; broader builds need their own seeds (or a sig-based locator). */
        if (pm && !strcmp(pm, "3310")
            && env_get("sys/firmware_version") && !strcmp(env_get("sys/firmware_version"), "05.79")) {
            env_set("func/UI_TASK", "2992692");          /* 0x2DA734 */
            env_set("func/TASKS_INIT_TABLE", "3261196"); /* 0x31C30C */
            /* PPM struct (find_ppm's literal pool entry).  Pre-caching this
             * lets LOCATE_DATA.rx skip the findldr → getldr → getlong chain
             * and read ppm/ppm_end directly from flash. */
            env_set("locate/ppm_struct", "3336224");     /* 0x32E820 */
            /* Language-menu rebuild locators inside PPMAN.rx :THE_END.
             * The COMM-subchunk branch constructs an in-phone language
             * switcher menu — irrelevant for anonymous_access.  Pre-cache
             * with addresses that are intentionally outside the flash so
             * the resulting setbyte/setlong/getbyte calls no-op and the
             * script proceeds to the actual PPM rebuild path. */
            env_set("func/NTOA",                "1");
            env_set("func/NOKSTR_INFO",         "1");
            env_set("func/STR_LANGUAGE",        "1");
            env_set("func/STR_AUTOMATIC",       "1");
            env_set("func/ACTION_SELECT_BACK",  "1");
            env_set("func/GFXID_EMPTY",         "1");
        }
    }
    /* Compute a stable firmware ID. Hash the canonical 32-byte MCU version header
     * ("V VV.VV\nDD-MM-YY\n<TYPE>\n(c) NMP."), NOT the whole file — the EEPROM/FFS
     * partition at the tail mutates per session and would give the same code-identical
     * firmware different ids. MAD2 puts the header at 0x1FC; the V1/serial-bus family
     * (5110/6110/8810/…) has none there — its header lives in the PPM/content block at a
     * 64 KB-block boundary +4 (0xN0004), so scan boundaries before the whole-file fallback. */
    {
        uint32_t h = 0x811C9DC5u;  /* FNV-1a 32-bit */
        const size_t HDR_LEN = 32;
        const uint8_t *p = NULL;
        #define NOKIX_IS_HDR(q) ((q)[0]=='V' && (q)[1]==' ' && (q)[7]=='\n' && (q)[10]=='-' && (q)[13]=='-' && (q)[16]=='\n')
        if (g_flash_size >= 0x1FC + HDR_LEN && NOKIX_IS_HDR(g_flash + 0x1FC))
            p = g_flash + 0x1FC;                            /* MAD2 canonical */
        for (size_t off = 0x4; !p && off + HDR_LEN <= g_flash_size; off += 0x10000)
            if (NOKIX_IS_HDR(g_flash + off)) p = g_flash + off;   /* V1/serial-bus block header */
        #undef NOKIX_IS_HDR
        if (p) for (size_t i = 0; i < HDR_LEN; i++)       { h ^= p[i];          h *= 0x01000193u; }
        else   for (size_t i = 0; i < g_flash_size; i++)  { h ^= g_flash[i];    h *= 0x01000193u; }
        if (h == 0) h = 1;        /* never 0, handle() treats 0 as "not found" */
        char buf[16];
        snprintf(buf, sizeof buf, "%u", h);
        env_set("sys/firmware_id", buf);
    }

    APIRET rc;
    rc = RexxRegisterSubcomExe("NOKIX", (RexxSubcomHandler *)nokix_subcom, NULL);
    if (rc) { fprintf(stderr, "RegisterSubcom -> %ld\n", (long)rc); return 1; }

    struct { const char *n; RexxFunctionHandler *h; } fns[] = {
        { "getenv", (RexxFunctionHandler *)rxfn_getenv },
        { "setenv", (RexxFunctionHandler *)rxfn_setenv },
        { "dand",   (RexxFunctionHandler *)rxfn_dand },
        { "dor",    (RexxFunctionHandler *)rxfn_dor },
        { "dxor",   (RexxFunctionHandler *)rxfn_dxor },
        { "dshr",   (RexxFunctionHandler *)rxfn_dshr },
        { "dshl",   (RexxFunctionHandler *)rxfn_dshl },
        { "reserve_ram", (RexxFunctionHandler *)rxfn_reserve_ram },
        { "swapenv",     (RexxFunctionHandler *)rxfn_swapenv },
    };
    for (size_t i = 0; i < sizeof fns / sizeof fns[0]; i++) {
        rc = RexxRegisterFunctionExe((PCSZ)fns[i].n, fns[i].h);
        if (rc && rc != 10)
            fprintf(stderr, "RegisterFunction %s -> %ld\n", fns[i].n, (long)rc);
    }

    /* Join extra args into one REXX argument string. */
    char argbuf[4096] = {0};
    int extra = argc - opt_i - 2;
    for (int i = 0; i < extra; i++) {
        const char *a = argv[opt_i + 2 + i];
        if (i && strlen(argbuf) + 1 < sizeof argbuf) strcat(argbuf, " ");
        if (strlen(argbuf) + strlen(a) < sizeof argbuf) strcat(argbuf, a);
    }
    RXSTRING args[1];
    MAKERXSTRING(args[0], argbuf, strlen(argbuf));

    SHORT rexx_rc = 0;
    RXSTRING result;
    MAKERXSTRING(result, NULL, 0);

    rc = RexxStart(
        argbuf[0] ? 1 : 0, args,
        script_path, NULL,
        "NOKIX",  RXCOMMAND,
        NULL, &rexx_rc, &result);

    if (rc) fprintf(stderr, "RexxStart -> %ld\n", (long)rc);
    if (g_verbose && result.strptr)
        fprintf(stderr, "[nokix] script result: %.*s\n",
                (int)result.strlength, result.strptr);

    /* Drain runatend queue. */
    for (struct atend *e = g_atend_head; e; ) {
        if (g_verbose)
            fprintf(stderr, "[nokix] runatend running: %s :%s %s\n",
                    e->script, e->label, e->arg);
        /* Intercept PPMAN.rx :THE_END — its in-phone-language-menu rebuild
         * path needs a dozen model-specific locators we don't have, but the
         * actual PPM-bytes-back-to-flash work is well-defined.  Do it in C. */
        if (strstr(e->script, "PPMAN.rx") &&
            (!strcasecmp(e->label, "THE_END"))) {
            if (g_verbose) fprintf(stderr, "[nokix] intercepting PPMAN.rx :THE_END (C-side writeback)\n");
            ppm_writeback_c();
            struct atend *next = e->next;
            free(e->script); free(e->label); free(e->arg); free(e);
            e = next;
            continue;
        }
        RXSTRING qargs[2];
        MAKERXSTRING(qargs[0], e->label, strlen(e->label));
        MAKERXSTRING(qargs[1], e->arg,   strlen(e->arg));
        SHORT qrc = 0;
        RXSTRING qresult;
        MAKERXSTRING(qresult, NULL, 0);
        /* Re-register subcom — Regina's per-process state may have been
         * reset when the first RexxStart returned. */
        RexxRegisterSubcomExe("NOKIX", (RexxSubcomHandler *)nokix_subcom, NULL);
        APIRET qstatus = RexxStart(
            e->arg[0] ? 2 : 1, qargs,
            e->script, NULL,
            "NOKIX", RXSUBROUTINE,
            NULL, &qrc, &qresult);
        if (qstatus)
            fprintf(stderr, "[nokix] runatend RexxStart -> %ld qrc=%d result='%.*s'\n",
                    (long)qstatus, (int)qrc,
                    qresult.strptr ? (int)qresult.strlength : 0,
                    qresult.strptr ? qresult.strptr : "");
        struct atend *next = e->next;
        free(e->script); free(e->label); free(e->arg); free(e);
        e = next;
    }
    g_atend_head = g_atend_tail = NULL;

    if (g_dirty) {
        if (save_flash(g_outpath_resolved) == 0)
            fprintf(stderr, "[nokix] wrote patched flash to %s\n",
                    g_outpath_resolved);
        else
            fprintf(stderr, "[nokix] FAILED to write %s\n", g_outpath_resolved);
    } else if (g_verbose) {
        fprintf(stderr, "[nokix] flash unmodified (no save)\n");
    }
    free(g_outpath_resolved);

    free(g_flash);
    return rexx_rc;
}
