// Cross-model security-code reader — firmware-oracle implementation. See seccode.h.
#include "harness/seccode.h"
#include "core/dct3_core.h"
#include "mgba/internal/arm/isa-inlines.h"   // _ARMSetMode / ThumbWritePC (CALL primitive)
#include <string.h>
#include <stdio.h>

uint32_t seccode_call_sync(DCT3Core* c, uint32_t fn,
                           uint32_t r0, uint32_t r1, uint32_t r2, long max_steps) {
    struct ARMCore* cpu = &c->cpu;
    // Save the caller's scratch registers + status. PC is NOT saved: the called function
    // returns to LR naturally (refilling the pipeline), so we just restore r0-r14 + cpsr
    // once it lands back at the sentinel — exactly the web dct3_web_call contract.
    int32_t  save[15];
    for (int k = 0; k < 15; ++k) save[k] = cpu->gprs[k];
    uint32_t save_cpsr = cpu->cpsr.packed;

    uint32_t ret = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);  // return sentinel
    cpu->gprs[0] = r0;
    cpu->gprs[1] = r1;
    cpu->gprs[2] = r2;
    cpu->gprs[ARM_LR] = ret | (cpu->cpsr.t ? 1u : 0u);
    cpu->gprs[ARM_PC] = fn & ~1u;
    _ARMSetMode(cpu, MODE_THUMB);
    cpu->cpsr.t = 1;
    cpu->cycles += ThumbWritePC(cpu);

    uint32_t result = 0;
    for (long s = 0; s < max_steps; ++s) {
        dct3_step(c);
        uint32_t pc = (uint32_t)cpu->gprs[15] - (cpu->cpsr.t ? 4u : 8u);
        if (pc == ret) { result = (uint32_t)cpu->gprs[0]; break; }
    }

    for (int k = 0; k < 15; ++k) cpu->gprs[k] = save[k];
    cpu->cpsr.packed = save_cpsr;
    return result;
}

static uint8_t rd8(DCT3Core* c, uint32_t a) {
    uint32_t w = dct3_read32(c, a & ~3u);
    return (uint8_t)(w >> (24 - 8 * (a & 3)));    // big-endian byte extract
}

// Read an ASCII digit run (stops at the first non-digit: pad nibble, NUL, etc.).
static int read_digits(DCT3Core* c, uint32_t addr, char* out, int max) {
    int n = 0;
    for (int i = 0; i < max - 1; ++i) {
        uint8_t ch = rd8(c, addr + i);
        if (ch < '0' || ch > '9') break;
        out[n++] = (char)ch;
    }
    out[n] = 0;
    return n;
}

int seccode_set(DCT3Core* c, uint32_t encrypt_fn, uint32_t store,
                uint32_t verify_fn, uint32_t scratch, const char* code) {
    if (!encrypt_fn || !store) return 0;
    if (!scratch) scratch = 0x00800000u;
    uint32_t in_s = scratch, out_c = scratch + 0x20;
    dct3_write_bytes(c, in_s, code, (uint32_t)strlen(code) + 1);
    // encrypt(0, code, out_c) -> 4-byte ciphertext under the live key
    seccode_call_sync(c, encrypt_fn, 0, in_s, out_c, 2000000);
    uint8_t ct[4]; for (int i = 0; i < 4; ++i) ct[i] = rd8(c, out_c + i);
    dct3_write_bytes(c, store, ct, 4);              // overwrite the stored code
    if (verify_fn) {                                 // confirm (also resets attempt counter)
        dct3_write_bytes(c, in_s, code, (uint32_t)strlen(code) + 1);
        return seccode_call_sync(c, verify_fn, in_s, 0, 0, 2000000) == 1;
    }
    return 1;
}

void seccode_solve(DCT3Core* c, const SeccodeAddrs* a, uint32_t scratch, SeccodeResult* out) {
    memset(out, 0, sizeof *out);
    if (!scratch) scratch = 0x00800000u;
    const long CAP = 2000000;          // EEPROM/cipher paths can be long; give headroom
    uint32_t out_p = scratch;          // plaintext output buffer
    uint32_t out_a = scratch + 0x20;   // authoritative output buffer
    uint32_t in_s  = scratch + 0x40;   // verify input string

    // --- plaintext: seccode_get(r1 = out buffer) writes the ASCII BCD code -----------
    if (a->get_fn) {
        seccode_call_sync(c, a->get_fn, 0, out_p, 0, CAP);
        char p[12];
        if (read_digits(c, out_p, p, sizeof p) > 0) { out->have_plaintext = 1; strcpy(out->plaintext, p); }
    }

    // --- authoritative: decrypt(r1 = stored block, r2 = out) — read-only, no lockout --
    // The XOR cipher is its own inverse, so decrypt(stored ciphertext) yields the exact
    // code seccode_verify would accept. One call: no wrong-attempt lockout to trip.
    if (a->decrypt_fn && a->block) {
        seccode_call_sync(c, a->decrypt_fn, 0, a->block, out_a, CAP);
        char d[12];
        if (getenv("SECCODE_DEBUG")) {
            printf("[seccode] decrypt out_a raw:");
            for (int i = 0; i < 12; ++i) printf(" %02X", rd8(c, out_a + i));
            printf("  block:");
            for (int i = 0; i < 8; ++i) printf(" %02X", rd8(c, a->block + i));
            printf("\n");
        }
        if (read_digits(c, out_a, d, sizeof d) > 0) { out->have_auth = 1; strcpy(out->authoritative, d); }
    }

    // If we could not decrypt but have a plaintext, fall back to treating it as the code
    // (correct for stock phones; we just can't prove it without the decrypt fn).
    if (!out->have_auth && out->have_plaintext) {
        strcpy(out->authoritative, out->plaintext);
        out->have_auth = 1;
    }

    out->diverged = (out->have_plaintext && out->have_auth &&
                     strcmp(out->plaintext, out->authoritative) != 0);

    // --- confidence: a single verify(authoritative) accepts (and resets the firmware's
    // wrong-attempt counter, so this read leaves no lockout side effect) ---------------
    if (a->verify_fn && getenv("SECCODE_DEBUG")) {   // probe: is the verify/key path live?
        // SECCODE_WIPE=ff|00: overwrite the stored block first, to see what verify does
        // when the security record is blank (regenerate-default vs reject-all).
        const char* wipe = getenv("SECCODE_WIPE");
        if (wipe && a->block) {
            uint8_t v = (uint8_t)strtoul(wipe, 0, 16);
            uint8_t blk[8]; memset(blk, v, sizeof blk);
            dct3_write_bytes(c, a->block, blk, sizeof blk);
            printf("[seccode]   (wiped block 0x%X to 0x%02X)\n", a->block, v);
        }
        // SECCODE_PROVISION=<code> SECCODE_ENCRYPT=<fn>: encrypt <code> under the live
        // key and write it to the block, then verify <code> — proves self-consistent set.
        const char* prov = getenv("SECCODE_PROVISION");
        const char* encs = getenv("SECCODE_ENCRYPT");
        if (prov && encs && a->block) {
            uint32_t enc_fn = (uint32_t)strtoul(encs, 0, 0);
            dct3_write_bytes(c, in_s, prov, strlen(prov) + 1);
            seccode_call_sync(c, enc_fn, 0, in_s, out_a, CAP);  // encrypt(0, code, out_a)
            printf("[seccode]   encrypt(\"%s\") ->", prov);
            for (int i = 0; i < 4; ++i) printf(" %02X", rd8(c, out_a + i));
            printf("\n");
            uint8_t ct[4]; for (int i = 0; i < 4; ++i) ct[i] = rd8(c, out_a + i);
            dct3_write_bytes(c, a->block, ct, 4);              // store ciphertext into block
            printf("[seccode]   (provisioned block with encrypt(\"%s\"))\n", prov);
        }
        static const char* probes[] = {"12345","00000","62628","99999","",};
        for (int i = 0; i < 5; ++i) {
            dct3_write_bytes(c, in_s, probes[i], strlen(probes[i]) + 1);
            printf("[seccode]   probe verify(\"%s\") -> %u\n", probes[i],
                   seccode_call_sync(c, a->verify_fn, in_s, 0, 0, CAP));
        }
    }
    if (a->verify_fn && out->have_auth) {
        dct3_write_bytes(c, in_s, out->authoritative, strlen(out->authoritative) + 1);
        out->verified = (seccode_call_sync(c, a->verify_fn, in_s, 0, 0, CAP) == 1);
    }
}
