#ifndef DCT3_SECCODE_H
#define DCT3_SECCODE_H
// Cross-model security-code reader (shared core; used by the web driver + native tools).
//
// The security/lock code lives in EEPROM two ways: a PLAINTEXT 3-byte BCD record
// (logical 0x110) and an ENCRYPTED block that seccode_verify actually checks. On stock
// phones they agree; on hand-modified images they can diverge (the encrypted one wins).
// Rather than re-implement the per-image cipher, we use the firmware's OWN routines as
// an oracle on the live, booted core:
//   - seccode_get(r1=out_ptr)   writes the ASCII plaintext code to a caller buffer.
//   - seccode_verify(r0=ascii)  returns 1 iff the code is accepted (the ground truth).
// We read the plaintext, ask the firmware to verify it, and (only if it is rejected =
// a modified image) brute-force seccode_verify to recover the code the phone accepts.
#include <stdint.h>
struct DCT3Core;

typedef struct {
    int  have_plaintext;      // plaintext BCD recovered
    char plaintext[12];       // ASCII, e.g. "12345"
    int  have_auth;           // authoritative code recovered (firmware-accepted)
    char authoritative[12];   // ASCII code the firmware actually accepts
    int  verified;            // a verify-fn call accepted the authoritative code
    int  diverged;            // plaintext != authoritative (image was modified)
} SeccodeResult;

// Per-build firmware addresses for the oracle (0 = unavailable; auto-located via sigs).
typedef struct {
    uint32_t get_fn;      // seccode_get(r1=out_ptr) -> writes ASCII plaintext BCD code
    uint32_t decrypt_fn;  // decrypt(r1=block_ptr, r2=out_ptr) -> writes the real ASCII code
    uint32_t block;       // RAM address of the stored encrypted code block (decrypt input)
    uint32_t verify_fn;   // seccode_verify(r0=ascii) -> 1 if accepted (confidence check)
} SeccodeAddrs;

// Synchronously call a Thumb firmware function fn(r0,r1,r2) on a BOOTED, idle core and
// return r0. Saves/restores caller registers (PC returns naturally via LR). Runs at
// most max_steps instructions; returns 0 if it never returns within the budget.
uint32_t seccode_call_sync(struct DCT3Core* c, uint32_t fn,
                           uint32_t r0, uint32_t r1, uint32_t r2, long max_steps);

// Recover the security code via the firmware oracle. scratch is a RAM address the
// firmware never uses (default 0x00800000 — above every model's flash/working set,
// routed to core RAM). Fills *out; leaves have_* = 0 for anything unresolvable.
//   plaintext  <- get_fn          (BCD record; what stock phones show)
//   authoritative <- decrypt_fn(block)   (the code the phone actually accepts)
//   diverged   = plaintext != authoritative (modified image)
//   verified   = a single verify_fn(authoritative) accepted it (confidence)
void seccode_solve(struct DCT3Core* c, const SeccodeAddrs* a,
                   uint32_t scratch, SeccodeResult* out);

// Set the live security code: encrypt(code) under the firmware's own key (encrypt_fn)
// and write the ciphertext to the stored-code RAM address (store) that verify reads.
// Runtime only (must run AFTER the boot setup that derives the stored code, i.e. once
// the phone is at the security-code screen). Returns 1 on success (verify_fn, if given,
// confirms the new code is accepted). Used by the web "reset security code → 12345" hook.
int seccode_set(struct DCT3Core* c, uint32_t encrypt_fn, uint32_t store,
                uint32_t verify_fn, uint32_t scratch, const char* code);

#endif
