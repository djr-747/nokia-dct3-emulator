// Nokia 7110 (NSE-5) ROM-4 DSP HLE responder — DELIBERATELY DISTINCT from the shared
// mad2_dsp_rom4 (5110 / 6110 / 3210).
//
// WHY ITS OWN TU (Dan's directive, 2026-06-18): the 7110 and 5110 are both ROM-4, but their
// DSP self-test CONVERSATIONS are obviously different — different uploaded blocks, different
// cmd-0x70 sub-tests (the 7110 sends sub-0x04/0x06/0x0E via the PORT1 API-command path; the
// 5110 sends sub-0x14/0x16 over the MDISND ring). The 5110's solved self-test lives in the
// C54x co-sim glue (DSP54_SELFTEST_MEAS); copying it here would entangle two unrelated reply
// formats. Keeping the 7110 responder in this file guarantees: (a) the 5110/6110/3210 keep the
// untouched mad2_dsp_rom4 -> `make guard` stays byte-identical by construction; (b) the 7110's
// reply model has its own home to grow without 5110 contamination.
//
// BASE = ROM-4 (same as mad2_dsp_rom4): version 4 (via the profile's dsp_boot_ready, returned
// by dsp_default_read) and NO 3310/ROM-6 perpetual MDIRCV keep-alive (dsp_no_keepalive). The
// boot block-upload / mailbox ack bodies are shared with the family via the dsp_default_*
// helpers (declared in mad2.h) — only the 7110 self-test reply is bespoke.
//
// WHY HLE, NOT CO-SIM (see [[7110-bringup-state]]): the native C54x co-sim runs the 5110's
// resident PROM (re/dsp-5110/raw/dsp_full.bin) while the 7110 uploads its OWN blocks (only 36%
// F073-overlapped), so the 7110's code far-branches into incompatible PROM routines and wedges
// (DSP stuck polling api_ram[0x1200] @PC 0x00ed, IMR=0x0001, never services cmd-0x70). The 3210
// hits the same wall. So the faithful boot-to-standby path is this HLE responder.
//
// ── CONTACT-SERVICE GATE CHAIN (traced 2026-06-17/18; firmware/My 7110 v5.00.fls) ───────────
// Verdict cell [0x17FE15] bit6 = the CONTACT-SERVICE-vs-standby screen gate. Two sub-gates:
//   (1) BUILDER 0x30DExx: leaf 0x49F548 returns [0x167030]; `cmp #1; beq PASS`. [0x167030] is
//       set =1 at 0x432F14 *iff* `ldrh [0x100E4]` (cb_reply) == 0 at the self-test poll. So the
//       builder needs cb_reply == 0 there -> [0x167030]=1 -> PASS (verdict 0xCC, bit6 kept).
//   (2) SEQUENCER 0x310D5A: `ldrb [0x17FE15]; lsr #3; bcs <clear bit6>` — if verdict bit3 (0x08)
//       is still SET it clears bit6 -> CONTACT SERVICE. bit3 = "self-test not complete". It is
//       cleared only when the self-test sequencer reaches its DONE state, which happens when the
//       DSP REPLIES to the cmd-0x70 sub-tests and the MCU processes the results.
//
// ── GROWTH POINT (the actual responder, NOT yet implemented) ─────────────────────────────────
// To reach standby faithfully this module must synthesize the 7110 DSP's cmd-0x70 self-test
// reply so the MCU's own state machine completes (clearing bit3) — NOT poke the verdict. Open RE:
//   * the cmd-0x70 reply CHANNEL+FORMAT the MCU validates (MDIRCV ring record vs PORT2 vs API
//     mailbox clear), per sub-test 0x04/0x06/0x0E;
//   * the reply VALUES that make the 24-item self-test result array (struct @0x17FC60, checked
//     in the builder loop 0x30DF6A over items 0..23) all pass, incl. calib-driven item 18.
// Until then this responder is the ROM-4 base only (7110 still lands on CONTACT SERVICE, but on
// its own clean backend — deterministic, no DSP wedge).

#include "mad2/mad2.h"
#include "models/model.h"
#include <stdlib.h>   // getenv (DSP7110_NOGATEA A/B opt-out; Gate A is default-ON, incl. on web)

// ── 7110 self-test state (this responder is the 7110's sole instance) ────────────────────────
// Gate A — see the GATE-A essay at the call site in dsp_7110_tick. The MCU issues its cmd-0x70
// self-test sub-tests by writing the PORT1 API-command word [0x100A8] with high byte 0x70 (sub
// in the low byte: 0x04/0x06/0x0E). We latch that the DSP has been ASKED to self-test, then —
// once cb_reply [0x100E4] has settled to 0 — raise the DSP's GENINT bit4 ("process stage") IRQ4
// exactly ONCE so the MCU's stage fn 0x432EE0 runs and (with cb_reply==0) sets its own
// [0x167030] builder-PASS flag. We NEVER write [0x167030] / the verdict ourselves.
#define PORT1_CMD 0x000100A8u   // MCU->DSP API command word; high byte 0x70 = self-test cmd

// 7110-only responder state. File-static (mirrors dsp_default.c's tick statics) because the
// Mad2 struct is off-limits to this TU and the 7110 has exactly one DSP instance per process.
static uint8_t st70_seen      = 0;  // a cmd-0x70 self-test command has been issued by the MCU
static uint8_t st70_irq_done  = 0;  // Gate A one-shot: GENINT bit4 IRQ4 already raised

static int dsp_7110_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    // ROM-4 base: faithful version 4 (profile dsp_boot_ready) via the shared boot-status path.
    return dsp_default_read(m, addr, size, ram_value, out);
}

static int dsp_7110_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    // Capture the cmd-0x70 self-test command(s): a PORT1 [0x100A8] write with high byte 0x70.
    // This is the DSP-side trigger for Gate A's one-shot IRQ4 (armed; fired in the tick once
    // cb_reply has cleared). We only OBSERVE here — return 0 so normal RAM-backing proceeds.
    if (addr == PORT1_CMD && ((value >> 8) & 0xFFu) == 0x70u)
        st70_seen = 1;
    // ROM-4 base mailbox/upload write handling.
    return dsp_default_write(m, addr, size, value);
}

static void dsp_7110_tick(Mad2* m) {
    m->dsp_no_keepalive = 1;   // ROM-4: no 3310/ROM-6 perpetual MDIRCV keep-alive
    dsp_default_tick(m);

    if (!m->mem) return;

    // ── GATE A — make the MCU's "process stage" fn 0x432EE0 run with cb_reply==0 ──────────────
    // The self-test result BUILDER (0x30DE9E: `cmp #1; beq PASS`) keeps verdict bit6 iff its
    // leaf 0x49F548 returns [0x167030]==1. [0x167030] is set =1 ONLY at 0x432F14, inside the
    // stage fn 0x432EE0, and ONLY when `ldrh [0x100E4]` (cb_reply) == 0 there. 0x432EE0 is the
    // GENINT bit4 handler (dispatcher 0x4CA4E4 `lsr #5; bcs ->0x432EE0`), reached from the IRQ
    // ISR when GENINT bit4 is pending AND mask [0x2000B] bit4 (0x10) is clear. The mask clears
    // bit4 from step ~2207312 (0x0E) and keeps it clear (0x8E) — so a single IRQ4 in that window
    // runs 0x432EE0. cb_reply settles to 0 by ~step 2207850 (it briefly held 0x0200 during the
    // block-ack), so it is 0 at self-test time.
    //
    // FAITHFUL: a real DSP raises GENINT to the MCU when it has processed a stage; we model that
    // single edge. We do NOT touch [0x167030] / the verdict — the MCU's own 0x432EE0 sets it.
    //
    // ⚠ IRQ4 is SHARED with Timer0/the OS scheduler; a repeated/spurious raise can double-link
    // the task list and wedge (see dsp_default.c). So this is a strict ONE-SHOT, latched on
    // st70_irq_done, gated on: (a) the MCU has actually issued a cmd-0x70 self-test command
    // (st70_seen, captured in the write hook), and (b) cb_reply [0x100E4] == 0 right now.
    //
    // DEFAULT-ON (2026-06-19): Gate A is now driven by the cmd-0x70 trigger UNCONDITIONALLY (no
    // getenv — getenv is NULL in wasm, so a knob-gated Gate A is OFF on the web). With Gate A the
    // verdict ends 0x49 (bit0|bit3|bit6, bit6 SET) instead of the baseline 0x09 (bit6 clear). On
    // both native and web this REMOVES the "CONTACT SERVICE" screen (the bit6=clear text no longer
    // draws — verified: web 00_boot.png goes from ~2.9% dark text to 0% / blank). DSP7110_NOGATEA=1
    // opts out for A/B. NB: bit6-set alone does NOT reach a drawn standby — see GATE B; the MMI
    // parks recomputing the self-test tally checksum (loop 0x47304A over [0x17FC60..]) because
    // bit3 is still set, so the screen is blank, not an idle/clock standby.
    //
    // DEFAULT-OFF (2026-06-19): Gate A is OFF by default (knob DSP7110_GATEA=1 enables it for native
    // A/B). Reason: making it default-on (incl. web) sets bit6 while bit3 stays set — an INCONSISTENT
    // verdict (pass-screen-gate vs not-complete) → the MMI parks recomputing the tally and renders
    // BLANK, which is worse than the honest, stable CONTACT SERVICE screen. Gate A is only useful once
    // GATE B (bit3/bit5) is resolved; until then the clean default = CONTACT SERVICE.
    if (st70_seen && !st70_irq_done && getenv("DSP7110_GATEA")) {   // Gate A OFF by default; DSP7110_GATEA=1 enables (native A/B)
        uint32_t cbr = m->fw.dsp_cb_reply & m->mem_mask;     // [0x100E4], BE halfword
        uint16_t cb_reply = (uint16_t)((m->mem[cbr] << 8) | m->mem[cbr + 1]);
        if (cb_reply == 0) {
            mad2_raise_irq(m, 4);          // DSP GENINT bit4 -> ISR -> 0x4CA4E4 -> 0x432EE0
            st70_irq_done = 1;              // one-shot
        }
    }

    // ── GATE B — verdict bit3 (self-test-incomplete) — A TRUE WALL (verdict-bit5 has no setter) ─
    // bit3 ([0x17FE15] & 0x08) = "self-test not complete"; while set, the sequencer 0x310D5A keeps
    // the phone off standby. bit3 is cleared at EXACTLY ONE site, 0x30E772 (`and #0xF7`), inside
    // fn 0x30E748 (clears iff its arg r0=[msg+9]==0). 0x30E748 is called ONLY from dispatcher arm
    // 0x3109BC, for an INTERNAL task message of type 0x40 / subop 0xA4 (router 0x310E38 type@[msg+3]
    // -> 0x310F2E -> dispatcher 0x3106E4 subop@[msg+8] -> jump-table 0x31083C slot 0x3108F4).
    //
    // The ONLY producer of subop-0xA4 anywhere in the image is 0x30E754 — inside 0x30E748 ITSELF
    // (a self-echo), so no firmware path originates the bit3-clearing message except the self-test
    // STEP-ENGINE completion cascade. That cascade is:
    //   begin-handshake (type-0x40/subop-0x64/[msg+9]==2 @0x310B52: sets bit4 + saves task id to
    //     [0x16BCC8]) -> step engine 0x30F180 (subop-128; descriptor table @0x26F684; issues
    //     cmd-8/cmd-9 pairs via 0x432B00 and demand-drives DSP replies through the GENINT-bit4
    //     measurement handler 0x432EE0 -> [0x167036]/[0x167038], 0x4EC868) -> on completion posts
    //     the subop-0xA4 that 0x30E748 turns into `and #0xF7`.
    //
    // THE WALL: the begin-handshake (0x30E15A `task_send 0x3BB77C`, r1=[0x16BCC8]) is GATED at
    // 0x30E14C on `[0x17FE15] bit5 (0x20)` (`lsr #5; bcc skip`). Verdict bit5 has NO SETTER in this
    // firmware build: it is cleared at init (0x30DE3C `and #0xEF`) and re-cleared in the very arm
    // that tests it (0x30E156 `and #0xEF`); an exhaustive scan of all 9 [0x17FE15] literal-pool
    // sites + every `orr`->verdict-store idiom finds NO `orr #0x20`. So the step engine 0x30F180
    // NEVER runs (0 hits in trace), [0x16BCC8] stays 0, bit4 never sets, and bit3 never clears.
    // This is structural, not a measurement range-check: the gate upstream of the measurement
    // engine (0x432EE0) can never open, so the measurement path is never even entered.
    //
    // No DSP-side signal we can synthesize (MDIRCV group-0x74 record, mailbox write to
    // [0x100E4]/[0x100E2]/[0x100DA], GENINT bit4 / FIQ0 / IRQ4) can set verdict bit5, because the
    // MCU contains no code that ORs 0x20 into [0x17FE15]. The self-test begin-state is set by an
    // EXTERNAL service tool over the FBUS/MBUS local-mode bus (a "begin self-test" command whose
    // receive path sets bit5) — i.e. the 7110's autonomous boot may NOT be designed to run this full
    // self-test to completion; a real 7110 likely reaches standby with this branch left incomplete via
    // a different verdict configuration. (NB: My 7110 v5.00 IS the full provisioned dump WITH an EEPROM
    // partition — calib record verified at flash 0x5FC000, logical EE base 0x5FC026 — so this is NOT a
    // missing-EEPROM problem; the open question is what configures the boot verdict so a factory 7110
    // skips/passes this gate.) FAITHFUL-ONLY rules forbid poking bit3 / bit5 / the verdict directly, so
    // NO ring poke and NO bit5 write here. Resolving standby needs either the verdict-config path RE'd,
    // or the external local-mode begin-self-test bus command modeled. See [[7110-bringup-state]].
}

const DspOps mad2_dsp_7110 = {
    "7110", dsp_7110_read, dsp_7110_write, dsp_7110_tick, dsp_hle_tone,
};
