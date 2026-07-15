// Nokia 7110 (NSE-5) ROM-4 DSP HLE responder — DELIBERATELY DISTINCT from the shared
// mad2_dsp_rom4 (5110 / 6110 / 3210).
//
// WHY ITS OWN TU: the 7110 and 5110 are both ROM-4, but their
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
// WHY HLE, NOT CO-SIM: the native C54x co-sim runs the 5110's
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
static uint8_t st70_sub       = 0;  // low byte of the last cmd-0x70 (sub-test id: 0x04/0x06/0x0E)
static uint8_t st70_irq_done  = 0;  // Gate A one-shot: GENINT bit4 IRQ4 already raised
static uint8_t st70_begin_done = 0; // begin-handshake record {0x64,0x02} injected (one-shot)
static uint8_t st_done        = 0;  // self-test-complete ack {sub 0x0D} posted (one-shot, default path)

// Auto-advance self-test state machine (DSP7110_ST=3, experimental). The DSP orchestrates the
// self-test: it posts each step record {sub 0x80, step N} and the completion record {sub 0xA4, 0}.
// We model that here — kick step 1 once the MCU asks (cmd-0x70 sub-0x0E), then feed the next step
// each time the ring drains (the step engine consumed the prior record), and finally completion.
static uint8_t  st_seq_active = 0;  // sequence running
static uint8_t  st_seq_step   = 0;  // next step index to inject (1..9), then 10 = completion
static uint64_t st_seq_next   = 0;  // pacing: next mono-cycle we may inject at

// Deposit one DSP->MCU MDIRCV record {payload_len, group=0x74, payload...} at the ring tail and
// raise FIQ0 — the same delivery pattern dsp_default.c's boot self-test responder uses (the ring
// pump 0x4E2282 forwards (group & 0xF0)==0x70 records to the self-test manager task via 0x3BBE28).
// Returns 0 if the queue isn't initialised/empty yet (caller retries next tick).
static int dsp_7110_ring_push(Mad2* m, const uint8_t* payload, int len) {
    uint32_t hp = m->fw.mdircv_head & m->mem_mask, tp = m->fw.mdircv_tail & m->mem_mask;
    uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
    uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
    if (head < 0x80 || tail < 0x80 || head != tail) return 0;   // not initialised, or not drained
    // Ring positions are word indices from 0x80; the byte offset into the queue buffer is
    // (pos - 0x80) * 2. Write the record at the CURRENT tail position (the firmware advances both
    // head and tail as it consumes, so they meet at 0x82, 0x84, … — not back at 0x80). Wrap the
    // buffer (100 words) so a long self-test sequence can't run off the end.
    uint16_t words = (uint16_t)(1 + (len + 1) / 2);
    uint16_t pos = tail;
    if ((uint16_t)(pos - 0x80 + words) > 100) pos = 0x80;       // wrap before overrun
    uint32_t q = (m->fw.mdircv_q & m->mem_mask) + (uint32_t)(pos - 0x80) * 2;
    m->mem[q]     = (uint8_t)len;    // word0 high = payload length
    m->mem[q + 1] = 0x74;            // word0 low  = group 0x74 (self-test class)
    for (int i = 0; i < len; ++i) m->mem[q + 2 + i] = payload[i];
    uint16_t nt = (uint16_t)(pos + words);
    m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
    mad2_raise_fiq(m, 0);            // DSP signals MDIRCV
    return 1;
}

static int dsp_7110_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    // ROM-4 base: faithful version 4 (profile dsp_boot_ready) via the shared boot-status path.
    return dsp_default_read(m, addr, size, ram_value, out);
}

static int dsp_7110_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    // Capture the cmd-0x70 self-test command(s): a PORT1 [0x100A8] write with high byte 0x70.
    // This is the DSP-side trigger for Gate A's one-shot IRQ4 (armed; fired in the tick once
    // cb_reply has cleared). We only OBSERVE here — return 0 so normal RAM-backing proceeds.
    if (addr == PORT1_CMD && ((value >> 8) & 0xFFu) == 0x70u) {
        st70_seen = 1;
        st70_sub  = (uint8_t)(value & 0xFFu);   // sub-test id (observed: 0x04, 0x06, 0x0E)
    }
    // ROM-4 base mailbox/upload write handling.
    return dsp_default_write(m, addr, size, value);
}

static void dsp_7110_tick(Mad2* m) {
    m->dsp_no_keepalive = 1;   // ROM-4: no 3310/ROM-6 perpetual MDIRCV keep-alive
    dsp_default_tick(m);

    if (!m->mem) return;

    // ── SELF-TEST COMPLETE ACK — the DSP's cmd-13 reply that clears CS (default-ON) ──────────────
    // CONTACT-SERVICE gate, fully traced 2026-07-13. The MCU self-test result BUILDER (0x30DE18)
    // runs at ~2.75M and, when the DSP-ready flag [0x167030]==1 (kept by dsp_uploaded / the block-ack
    // pump), leaves the verdict [0x17FE15] at 0xCD = bit0|bit2|bit3|bit6|bit7. The self-test task's
    // verdict-finalizer 0x310BFC (run later on internal control code 0xD8) reaches the sequencer
    // 0x310D5A: `ldrb verdict; lsr #3; bcs <clear bit6>`. ⚠ `lsr #3` puts BIT 2 (0x04) — not bit3 —
    // into carry, so the real gate is verdict BIT2: while bit2 is SET the sequencer clears bit6 ->
    // CONTACT SERVICE. bit2 = "self-test result pending / not yet acknowledged".
    //
    // The DSP clears it by REPLYING to the self-test with a group-0x74 sub-13 (0x0D) "complete"
    // record: the ring pump 0x4E2282 forwards it to the self-test task -> DSP-record handler arm
    // 0x30E0EA -> `and #0xFB` clears verdict bit2 (0x30E0FA) and posts the finalize broadcast. This
    // is EXACTLY the cmd-13 self-test-complete reply dsp_default.c synthesizes for the sibling models
    // (3310/2100/3410/…). The shared dsp_default responder misses the 7110 because it only fires while
    // the MDIRCV ring is still at its initial 0x80/0x80 empty state — on the 7110 the ring has already
    // advanced (real DSP boot records) by the time the builder sets bit2, so the 7110 needs its own
    // position-aware post (dsp_7110_ring_push writes at the CURRENT tail).
    //
    // FAITHFUL: a healthy DSP, asked to self-test (MCU cmd-0x70), runs it and reports completion; we
    // model that single reply. We NEVER write the verdict/bit2 ourselves — the MCU's own handler
    // 0x30E0FA clears it. One-shot, gated on: (a) the MCU issued a cmd-0x70 self-test command
    // (st70_seen), and (b) the builder has set bit2 (result pending) with the DSP-ready flag up.
    // Two DSP self-test-complete records, in order (each posted once the ring has drained):
    //   stage 0: {sub 0x0D} -> handler 0x30E0EA `and #0xFB` clears verdict BIT2 (the CS sequencer
    //            0x310D5A gate: lsr#3 -> carry=bit2; bit2 set => clears bit6 => CONTACT SERVICE).
    //   stage 1: {sub 0xA4} -> handler 0x30E748 `and #0xF7` clears verdict BIT3 ("self-test not
    //            complete"); while bit3 is set the MMI parks recomputing the tally checksum
    //            (loop 0x47304A) and renders blank rather than standby.
    // Both are the healthy DSP's "self-test done / all pass" report (cf. dsp_default cmd-13 for the
    // siblings). We never write the verdict — the MCU's own handlers clear bit2/bit3.
    // All three triggers are DSP-visible: st70_seen (PORT1 cmd-0x70), the "result pending" edge
    // is the observed MDISND {0x70,0x0D} run-request (m->dsp_st_req; the 7110 streams the same
    // group-0x70 record sequence as the ROM-6 line, sweep-verified 2026-07-15), and DSP-ready is
    // the internal dsp_running latch (tracking the block-ack pump, = the firmware's own
    // [0x167030]/.dsp_uploaded flag). No MCU-private RAM read.
    if (st70_seen && st_done < 1 && m->dsp_running) {
        const uint8_t bit2_ack[] = { 0x0D, 0x00 };   // clears verdict bit2
        if (m->dsp_st_req && dsp_7110_ring_push(m, bit2_ack, 2)) {
            st_done = 1;
            m->dsp_st_req = 0;                       // request consumed
        }
    }

    // NOTE: the boot-readiness barrier task-0 (0x49F8C0, task table 0x25D3B8) is a
    // low-priority background task that is SUPPOSED to sleep (0x428FE2 etc. never satisfied is normal);
    // poking it "ready" just busy-loops it and starves the MMI — it is NOT the standby-draw gate.
    // The standby-not-drawn cause is in the MMI itself (dispatcher not selecting the idle screen with
    // a healthy verdict)

    // ── RESEARCH KNOBS (all default-OFF; native A/B only; guard byte-identical; web unaffected) ───
    // The self-test conversation was mapped by driving it piecewise with these. Retained as probes.
    //   DSP7110_FORCEUP=1  force the DSP-ready flag [0x167030]=1 (stand in for dsp_uploaded).
    //   DSP7110_GATEA=1    raise one GENINT bit4 so the MCU's stage fn 0x432EE0 sets [0x167030]=1.
    //   DSP7110_ST=1       inject {sub 0x64,2} begin-handshake (sets verdict bit4, saves requester).
    //   DSP7110_ST=2       inject {sub 0x80,1} step-engine kick (subop-0x80 step 1; needs bit2/ready).
    //   DSP7110_ST=3       auto-advance the 9-step engine + {sub 0xA4} completion (clears bit3).
    //   DSP7110_ST=5       inject the {sub 0x0D} complete ack (same as the default responder above).
    // The step engine (0x30F180) issues DSP cmd-8/cmd-9 per step from descriptor table 0x26F684 and
    // reposts the same step until the DSP replies — proven kickable, but NOT needed for standby: the
    // sub-13 bit2-clear ack above is what a factory boot uses (the 9-step run is the service self-test).
    if (getenv("DSP7110_FORCEUP") && m->fw.dsp_uploaded) {
        uint32_t f = m->fw.dsp_uploaded & m->mem_mask;   // [0x167030] via the profile
        if (m->mem[f] == 0) m->mem[f] = 1;
    }
    if (st70_seen && !st70_irq_done && getenv("DSP7110_GATEA")) {
        uint32_t cbr = m->fw.dsp_cb_reply & m->mem_mask;
        uint16_t cb_reply = (uint16_t)((m->mem[cbr] << 8) | m->mem[cbr + 1]);
        if (cb_reply == 0) { mad2_raise_irq(m, 4); st70_irq_done = 1; }
    }
    if (st70_seen && st70_sub == 0x0E && !st70_begin_done && getenv("DSP7110_ST")) {
        uint32_t vd = m->fw.verdict & m->mem_mask;
        int mode = atoi(getenv("DSP7110_ST"));
        const uint8_t begin[] = { 0x64, 0x02 };
        const uint8_t kick[]  = { 0x80, 0x01 };
        const uint8_t done[]  = { 0x0D, 0x00 };
        if (mode == 5) {
            if ((m->mem[vd] & 0x04) && dsp_7110_ring_push(m, done, 2)) st70_begin_done = 1;
        } else if (mode >= 3) {
            if (m->mem[vd] & 0x08) { st_seq_active = 1; st_seq_step = 1; st70_begin_done = 1; }
        } else {
            const uint8_t* p = (mode >= 2) ? kick : begin;
            if ((m->mem[vd] & 0x08) && dsp_7110_ring_push(m, p, 2)) st70_begin_done = 1;
        }
    }
    if (st_seq_active && m->rtc_mono >= st_seq_next) {
        uint8_t rec[2];
        if (st_seq_step <= 9) { rec[0] = 0x80; rec[1] = st_seq_step; }
        else                  { rec[0] = 0xA4; rec[1] = 0x00; }
        if (dsp_7110_ring_push(m, rec, 2)) {
            const char* pe = getenv("DSP7110_STPACE");
            uint64_t pace = pe ? (uint64_t)strtoull(pe, 0, 0) : 500000;
            st_seq_next = m->rtc_mono + pace;
            if (st_seq_step > 9) st_seq_active = 0;
            else st_seq_step++;
        }
    }

}

const DspOps mad2_dsp_7110 = {
    "7110", dsp_7110_read, dsp_7110_write, dsp_7110_tick, dsp_hle_tone,
};

