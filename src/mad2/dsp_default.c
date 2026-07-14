// Default (shared / legacy) DSP behaviour for the MAD2 platform.
//
// This is the DSP boot/runtime handshake the 3310 (and the 7110 stub, and the 8850
// until it gets its own ops) relies on. It was extracted verbatim from mad2.c so the
// 3310 boot stays byte-identical; mad2_read/mad2_write/mad2_tick now dispatch here via
// the per-model `DspOps` vtable (models/model.h). A model with a genuinely different
// DSP boot supplies its own ops and leaves this one untouched.
//
// Addresses are all per-model data in m->fw (FwAddrs): dsp_mbox0/1 (boot-ack slots),
// dsp_cb_req/reply (code-block request/reply halfwords), mdircv_q/head/tail (the
// DSP->MCU message queue), cobba (Cobba command field), dsp_boot_status/ready (the
// boot-status poll slot + reply), verdict + dsp_uploaded (self-test scratch).

#include "mad2/mad2.h"
#include <stdlib.h>   // getenv
#include <stdio.h>    // printf (COBBALOG temp knob)
#include <string.h>   // strcmp (DSPHEARTBEAT_SRC selector)

// Real-HW captured DSP->MCU MDIRCV boot burst (3310 v6.33; NokiX RAM read of the ring,
// frozen at standby;). The ring
// is BIG-ENDIAN (ARM7TDMI BE): MDIRCV_DEQUEUE 0x2BAC72 reads each word and takes
// group = word>>8 = the even/low-address byte. The captured steady stream is KA_REPLAY_N
// records, each TWO ring words: an 0xFF<seq> sequence/sync word (decodes to a group-0xFF
// broker msg) then an 0x03<id> group-0x03 telemetry word. seq is 0x8E for the first 20
// records, then 0x8D (a DSP generation/sequence tick). The DSP wrote this burst during
// boot/network-search (head 0x90->0xDF) then went PERMANENTLY SILENT — the phone never
// resets. DSPKA_REPLAY mode replays this verbatim then falls silent, vs the default
// perpetual single-word group-0x03 keep-alive (the A/B switch).
#define KA_REPLAY_N 39
static const uint8_t ka_replay_seq[KA_REPLAY_N] = {
    0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,   // 20x seq 0x8E
    0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,0x8E,
    0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,   // 19x seq 0x8D
    0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,0x8D,
};
static const uint8_t ka_replay_id[KA_REPLAY_N] = {
    0x74,0x72,0x47,0x5C,0x60,0x5F,0x63,0x6A,0x4F,0x69,   // group-0x03 ids, in captured order
    0x48,0x09,0x61,0x20,0x54,0x36,0x55,0x5B,0x68,0x3A,
    0x3F,0x4C,0x66,0x4D,0x1F,0x53,0x62,0x06,0x28,0x03,
    0x52,0x5D,0x67,0x12,0x37,0x73,0x2B,0x2F,0x1A,
};
// Real-HW early-boot block (ring idx 0x80-0x8F, bytes 0x10100-0x1011F). 16 raw BE words that
// precede the steady stream on silicon (head reached 0xDF = 0x80 + 16 early + 78 steady).
//
// AUTHORITATIVE SOURCE (replaces the prior v6.33 capture which was unfaithful here — the
// DIFFTRACE-PIN found web[0x10104]=0xEDE1 vs real-HW 0xD674): NokiX RAM reads off TWO physical
// 3310 v6.39 units. 0x10100 is the HW-pinned shared MCU<->DSP address space,
// version-invariant; the v6.39 reads are authoritative for v5.79. 7 of 8 dword lines are
// BYTE-IDENTICAL across BOTH units => this is REAL FIXED DETERMINISTIC DSP data (the boot
// self-test signature / constant block), NOT stale/uninit junk and NOT measurement noise.
//
//   addr      unit1        unit2 (where it differs)
//   0x10100:  B4 39 46 A8  (fixed)
//   0x10104:  D6 74 96 90  (fixed)   <- prior code injected EDE1 here; real is D674. THE bug.
//   0x10108:  F7 BB 95 0B  (fixed)
//   0x1010C:  23 99 85 1F  (fixed)
//   0x10110:  8F F5 6C 7A  (fixed)
//   0x10114:  18 B9 02 8A  (fixed)
//   0x10118:  00 60 A2 8B  (fixed)
//   0x1011C:  00 30 03 6A  unit2 = 00 30 00 20  (only differing dword; "00 30" prefix fixed,
//                                                 bytes 3-4 {group,id} vary like steady records)
//
// As 16 BE words (each word = two consecutive bytes, ARM7 big-endian): the dequeue takes
// word>>8 as the high byte. We bake unit1 verbatim. These are NOT {FF,seq,grp,id} records —
// they are the fixed self-test-result payload the firmware reads to validate the DSP and let
// its watchdog cancel stick. DSPKA_REPLAY=2 emits them first so our ring runs 0x80->0xDF
// exactly like silicon. DSPKA_REPLAY=1 = steady 39 records only (skip this block).
static const uint16_t ka_early[16] = {
    0xB439,0x46A8,0xD674,0x9690,0xF7BB,0x950B,0x2399,0x851F,
    0x8FF5,0x6C7A,0x18B9,0x028A,0x0060,0xA28B,0x0030,0x036A,
};

// DSP-region read. Returns 1 (and sets *out) if it owns this address, else 0.
// Non-static: the C54x co-sim backend (mad2_dsp_c54x.c) forwards to this faithful
// legacy mailbox model for the pass-through (non-co-sim) path. See mad2.h.
int dsp_default_read(Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out) {
    (void)size;
    // MAMEDSP=1 (A/B experiment): replace our faithful mailbox/FIQ0 model with MAME's
    // nokia_3310.cpp dsp_ram_r HACK — frozen constants on the four slots it special-cases
    // ("avoid hangs when ARM try to communicate with the DSP"), RAM passthrough otherwise,
    // and NO FIQ0/IRQ4/upload protocol (see the write/tick branches). Lets us see how far
    // OUR firmware boots on MAME's stub. NOT faithful — diagnostic only, default OFF.
    static int mame = -1;
    if (mame < 0) mame = getenv("MAMEDSP") ? 1 : 0;
    if (mame) {
        if (addr >= 0x00010000u && addr <= 0x00010005u) { *out = 0x0001; return 1; }  // ready slots
        if (addr == 0x000100E0u) { *out = 0x0000; return 1; }                          // Cobba
        if (addr == 0x000100FEu) { *out = 0x0001; return 1; }                          // Mbox0
        if (addr == 0x00010100u) { *out = 0x0001; return 1; }                          // Mbox1/MDIRCV
        return 0;                                                                       // else: RAM passthrough
    }
    // DSP boot handshake: a slot reads 0 until the DSP acks (consume-once).
    if (addr == m->fw.dsp_mbox0) { *out = m->dsp_ack[0]; m->dsp_ack[0] = 0; return 1; }
    // dsp_mbox1 (0x10100) doubles as the MDIRCV runtime queue base. The legacy
    // boot-ack ping-pong only applies during the code-block upload, BEFORE the
    // firmware initialises the MDIRCV queue (head ptr mdircv_head set to 0x80 at
    // 0x2BAF74). Once initialised, mbox1 is the real queue word0 — return RAM
    // so injected DSP->MCU messages are read correctly (else word0 reads as 0).
    if (addr == m->fw.dsp_mbox1) {
        uint32_t hp = m->fw.mdircv_head & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        if (head == 0) { *out = m->dsp_ack[1]; m->dsp_ack[1] = 0; return 1; }
        *out = ram_value; return 1;                  // MDIRCV queue initialised: real word
    }
    // DSP boot-status word: MCU parks 0xFFFF and waits for the DSP to report ready.
    if (addr == 0x00010002u && (ram_value & 0xFFFF) == 0xFFFF) { *out = 0; return 1; }
    // Per-profile DSP boot-status slot: the firmware posts a command to the HPI
    // mailbox, pulses the DSP, parks this slot at 0xFFFF and polls it for the DSP's
    // ready/version reply. Returning dsp_boot_ready (5/6 on the 8850) lets the boot
    // proceed to the DSP code-block upload (handshaked via dsp_mbox0/1 above).
    if (m->fw.dsp_boot_status && addr == m->fw.dsp_boot_status &&
        (ram_value & 0xFFFF) == 0xFFFF) { *out = m->fw.dsp_boot_ready; return 1; }
    // Second boot-status word (5110/serial-bus): the loader requires [0x10004]==[0x10006] before it
    // accepts the DSP version reply (0x271AF4). The DSP writes the same version to both; model
    // it by returning dsp_boot_ready for the second word too, so the cross-check passes and the
    // firmware builds a valid branch table + sets dsp_uploaded itself. See docs §7e/§8e.
    if (m->fw.dsp_boot_status2 && addr == m->fw.dsp_boot_status2 &&
        (ram_value & 0xFFFF) == 0xFFFF) { *out = m->fw.dsp_boot_ready; return 1; }
    return 0;
}

// DSP-region write. Returns 1 if handled (stop further mad2_write processing), else 0.
int dsp_default_write(Mad2* m, uint32_t addr, int size, uint32_t value) {
    (void)size;
    // DSPVIS: observe (never intercept — the register has real MMIO handling) the MCU
    // asserting the DSP control register [0x20002] back to its boot value 0x01 — the
    // DSP reset/hold state the boot sequence starts from (profile .dsp_reset_running:
    // read-back 0x53 = released/running). A DSP put back into reset loses its run
    // state, so this is the faithful warm-reboot re-arm for the dsp_running latch.
    // (A read-level re-arm on the parked boot-status word is WRONG: the parked 0xFFFF
    // lingers in RAM — on the 3310 inside the MDISND ring itself — so any later stray
    // read/record would disarm the latch mid-run and stall the Cobba auto-consume.)
    if (m->dsp_vis && addr == 0x00020002u && (value & 0xFFu) == 0x01u)
        m->dsp_running = 0;
    // MAMEDSP=1: MAME's dsp_ram_w just COMBINE_DATA's (plain store), so let the core
    // RAM-back every DSP-region write — no upload-ack protocol.
    { static int mame = -1; if (mame < 0) mame = getenv("MAMEDSP") ? 1 : 0;
      if (mame) return 0; }
    // DSP boot handshake: writing one slot makes the DSP signal the paired slot
    // ready (echo the token so the MCU's "!= 0" wait passes).
    if (addr == m->fw.dsp_mbox0) { m->dsp_ack[1] = (uint16_t)value ? (uint16_t)value : 1; m->dsp_acks++; return 1; }
    if (addr == m->fw.dsp_mbox1) {                    // see read side: only the legacy
        uint32_t hp = m->fw.mdircv_head & m->mem_mask;// boot-ack phase (queue head==0);
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        if (head == 0) { m->dsp_ack[0] = (uint16_t)value ? (uint16_t)value : 1; m->dsp_acks++; return 1; }
        return 0;                                     // queue initialised: let the core's RAM-back stand
    }
    // DSPVIS: observe (never intercept) the MCU bumping the MDISND write pointer — the
    // moment a new MCU->DSP request record becomes visible to the DSP. The record was
    // already written at the OLD tail (BE word0 = {len<<8 | group}, payload follows).
    // The self-test protocol streams the calibration blocks (group 0x70 subs 0x13..0x16),
    // marks its MCU-private "pending" bit, then enqueues {0x70, 0x0D} = "run self-test,
    // report result" — THAT record is the faithful trigger for the group-0x74 {0x0D,0}
    // completion reply (it arrives after the MCU is parked waiting, so no race).
    if (m->dsp_vis && m->fw.mdisnd_tail && m->mem &&
        addr == m->fw.mdisnd_tail && size >= 2) {
        uint16_t prev = m->dsp_mdisnd_prev;                   // record just enqueued starts here
        uint32_t ring = m->fw.mdisnd_tail - m->fw.mdisnd_q;   // ring byte size (3310 = 0xA4)
        uint32_t off0 = (uint32_t)prev * 2;                   // word0 never wraps (enqueue
        uint32_t offs = off0 + 2;                             // wraps only PAST each store)
        if (offs >= ring) offs -= ring;                       // payload may wrap to base
        if (off0 < ring && (value & 0xFFFFu) != prev &&       // a real enqueue, not a re-park
            m->mem[(m->fw.mdisnd_q + off0 + 1) & m->mem_mask] == 0x70 &&
            m->mem[(m->fw.mdisnd_q + offs)     & m->mem_mask] == 0x0D) m->dsp_st_req = 1;
        if (getenv("DSPVIS_LOG"))
            printf("[dspvis] MDISND tail %02X->%02X rec={%02X %02X %02X} st_req=%d @mono=%llu\n",
                   prev, (unsigned)(value & 0xFFFF),
                   m->mem[(m->fw.mdisnd_q + off0) & m->mem_mask],
                   m->mem[(m->fw.mdisnd_q + off0 + 1) & m->mem_mask],
                   m->mem[(m->fw.mdisnd_q + offs) & m->mem_mask],
                   m->dsp_st_req, (unsigned long long)m->rtc_mono);
        m->dsp_mdisnd_prev = (uint16_t)(value & 0xFFFFu);
        return 0;                                 // observe only: the write still lands
    }
    // DSP code-block reply (dsp_cb_reply): firmware acked an upload. 0x0002 = more to
    // go (model the DSP requesting the next block via IRQ4), 0x0004 = upload complete.
    if (addr == m->fw.dsp_cb_reply) {
        if ((value & 0xFFFFu) != 0x0004u) m->dsp_cb_delay = 256;  // pump the next request
        else                              m->dsp_cb_delay = 0;    // boot upload complete
        m->dsp_cb_armed_nz = (value & 0xFFFFu) != 0;  // DSPVIS: real block vs [reply]=0 clear
        return 1;
    }
    return 0;
}

// Per-step DSP pump: mailbox acks, IRQ4 generation, self-test + boot-msg injection.
void dsp_default_tick(Mad2* m) {
    // MAMEDSP=1: MAME instantiates no DSP core and raises no DSP interrupt — no upload
    // pump, no FIQ0/IRQ4, no self-test responder, no keep-alive. Pump nothing.
    { static int mame = -1; if (mame < 0) mame = getenv("MAMEDSP") ? 1 : 0;
      if (mame) return; }
    m->dsp_steps++;   // free-running step counter (this tick runs once per emulated step)
    // Co-sim HLE quiet-gate (set in mad2.c under DSP54_COSIM; DSP54_HLE=1 re-enables): the real
    // C54x supplies every DSP->MCU signal, so the ENTIRE modelled tick below — block-ack pump +
    // IRQ4 raise, Cobba auto-consume, self-test responder, DSPMSG injector, keep-alive FIQ0
    // stream — is a fake-signal source and must not run. Only the step counter above survives.
    if (m->dsp_hle_quiet) return;
    // DSP code-block ack: emulate the DSP consuming an uploaded block and asking for
    // the next. The firmware walks its OWN block list and POLLS the reply word to pace
    // the upload, so we clear the reply (DSP consumed it) and post a "next" request prod.
    //
    // IRQ4 = the DSP->MCU "block processed" signal. The firmware's DSP message pump
    // (0x2BB430) is dispatched from the Timer/IRQ handler (0x2E4F38 -> 0x2E6ED2) when
    // IRQ4 (bit4 of [0x20009]) is pending; the pump sets [0x11038C] (DSP-uploaded). The
    // boot subsystem-starter (0x2E0B26) checks [0x11038C] at ~step 2145168 and bails
    // (resuming only tasks 7,1,2) if it is still 0 -> the app/MMI layer never launches.
    // Without IRQ4 the pump only runs on a much later trigger (~2155796), losing the
    // race by ~10.6k steps. So raise IRQ4 here to fix the boot ordering FAITHFULLY (the
    // real DSP raises it per block) -- but ONLY while [0x11038C]==0: one IRQ4 after the
    // first block ack runs the pump and sets the flag in time. We must stop after that
    // because IRQ4 is shared with Timer0/the OS scheduler and our interrupt model
    // double-links the task list if a spurious IRQ4 races the FIQ4 worker (0x298B20)
    // once the app tasks are live (boot wedges in FIQ mode at 0x2EEBEA). DSPIRQ=1 forces
    // the raise on every ack to reproduce that corruption for RE.
    if (m->dsp_cb_delay && --m->dsp_cb_delay == 0 && m->mem) {
        uint32_t reply = m->fw.dsp_cb_reply & m->mem_mask;
        uint32_t req   = m->fw.dsp_cb_req   & m->mem_mask;
        m->mem[reply] = 0x00; m->mem[reply + 1] = 0x00;   // DSP cleared reply: ready for more
        m->mem[req]   = 0x00; m->mem[req + 1]   = 0x01;   // next code-block request (BE halfword)
        // DSPVIS gate: a real DSP cannot read [dsp_uploaded]; it knows only its OWN
        // state ("have I acked a real block yet"). The internal dsp_running latch is
        // set only on a pump cycle armed by a REAL block delivery (nonzero cb_reply
        // write; dsp_cb_armed_nz) — the firmware also clears [cb_reply]=0 during DSP
        // init, and latching on THAT expiry suppresses the real block-ack IRQ4, so the
        // firmware pump never uploads block 2 (self-test then fails C8->88; measured).
        // Raise pattern reproduced exactly from DSP-visible events: clear-armed expiry
        // raises (legacy: flag still 0), first real-block expiry raises + latches,
        // later expiries silent (legacy: [dsp_uploaded] set by the pump ~100 steps
        // after the real-block raise).
        { int first = m->dsp_vis ? !m->dsp_running
                                 : !m->mem[m->fw.dsp_uploaded & m->mem_mask];
          if ((first || getenv("DSPIRQ")) && !getenv("DSPNOIRQ4"))
              mad2_raise_irq(m, 4); // DSP "block done" IRQ4 -> pump -> [0x11038C], pre-Phase-2 only
        }
        if (m->dsp_cb_armed_nz) m->dsp_running = 1;  // DSPVIS: real block acked = DSP code live
        m->dsp_cb_acks++;
    }
    // DSP Cobba command field [0x100E0]: the MCU writes a command halfword, signals the
    // DSP (DSPINT), and polls [0x100E0]==0 to know the DSP has consumed it. In Phase 2 the
    // idle/audio path (DSP command sender 0x2BB0B4) checks [0x100E0]==0 at entry (0x2BB0CE)
    // and bails (return 0) while it is non-zero. We don't run the DSP, so the first command
    // would stick at [0x100E0]!=0 forever and every later command would bail — the boot
    // wedges in the Phase-2 DSP poll (idle loop 0x2ADF84) and never clears the segment test.
    // Model the DSP as consuming the command (clear it after it is written). GATE this on
    // the DSP code upload being complete ([0x11038C]!=0, set by the DSP message pump): the
    // Cobba/audio command path is only exercised once the DSP is running. Clearing [0x100E0]
    // during EARLY boot (before the upload) perturbs the boot path and stops the MMI message
    // router (0x2E84B6) from running later — so only consume Cobba commands once the DSP is up.
    // (Faithful to "DSP acks the command once it's running"; responses still come via MDI/queue.)
    // DSPVIS: gate on the internal dsp_running latch instead of the MCU-private flag —
    // same edge (first block-ack pump cycle; the firmware's own [dsp_uploaded] write
    // follows ~100 steps later, and the first Cobba command only ~27k steps after that).
    if (m->mem && (m->dsp_vis ? m->dsp_running
                              : m->mem[m->fw.dsp_uploaded & m->mem_mask])) {
        uint32_t cobba = m->fw.cobba & m->mem_mask;
        if (m->mem[cobba] || m->mem[cobba + 1]) {
            // COBBALOG=1: log each MCU->DSP->COBBA audio command value + mono cycle, to
            // correlate audio commands (e.g. keypad tones) with their trigger. TEMP knob.
            if (getenv("COBBALOG"))
                printf("[cobba] cmd=%02X%02X  @mono=%llu\n",
                       m->mem[cobba], m->mem[cobba + 1], (unsigned long long)m->rtc_mono);
            m->mem[cobba] = 0; m->mem[cobba + 1] = 0;
        }
    }
    // DSP self-test responder: faithful organic verdict pass (no spike). See mad2.h.
    // The firmware streams the self-test request to the DSP then marks verdict bit2
    // ([0x11FF15] |= 0x04, "DSP self-test pending") and blocks the self-test task on the
    // result. Model the DSP replying: post a group-0x74 result [13,0] into the (empty)
    // MDIRCV queue + raise FIQ0 -> dispatcher 0x2EE9B6 -> 0x2C2824 (forwards group+payload
    // to the self-test task) -> command-13 0x2414C2 clears bit2 (keeps bit6) -> 0xC8.
    // The MCU does the verdict write itself; we only deliver the mailbox reply. One-shot.
    // (Skipped when a manual DSPMSG injection is active, or via DSPNOSELFTEST.)
    if (m->mem && !m->dsp_selftest_off && !m->dsp_selftest_replied && !m->dsp_msg_en) {
        uint32_t verdict = m->fw.verdict & m->mem_mask;
        uint32_t hp = m->fw.mdircv_head & m->mem_mask, tp = m->fw.mdircv_tail & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
        // DSPVIS trigger: the observed MDISND {0x70,0x0D} "run self-test" request (set in
        // dsp_default_write) — the record the MCU enqueues right AFTER parking on verdict
        // bit2, so it strictly follows the legacy trigger with no MCU-private read.
        int pending = m->dsp_vis ? m->dsp_st_req : (m->mem[verdict] & 0x04);
        if (pending && head == 0x80 && tail == 0x80) {  // pending + queue empty
            uint32_t q = m->fw.mdircv_q & m->mem_mask;
            m->mem[q]     = 0x02;   // word0 high = payload len (copy 2 bytes)
            m->mem[q + 1] = 0x74;   // word0 low  = group 0x74 -> 0x2C2824 -> self-test cmd-13
            m->mem[q + 2] = 0x0D;   // payload[0] = sub-command 13
            m->mem[q + 3] = 0x00;   // payload[1] = status 0 (pass; keeps verdict bit6)
            uint16_t words = (uint16_t)(1 + (2 + 1) / 2);          // 2 words
            uint16_t nt = (uint16_t)(0x80 + words);
            m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
            mad2_raise_fiq(m, 0);                                 // FIQ0: DSP signalled MDIRCV
            m->dsp_selftest_replied = 1;
            m->dsp_st_req = 0;                                    // DSPVIS: request consumed
        }
    }
    // DSP runtime boot-indication injector (brute-force; see mad2.h). Once the firmware
    // has set up the (empty) MDIRCV queue (read ptr [0x101CA] == write ptr [0x101C8] ==
    // 0x80), wait DSPMSGDELAY cycles then write one message at MCU 0x10100, bump the
    // write ptr, and raise FIQ0 — exactly once.
    if (m->dsp_msg_en && !m->dsp_injected && m->mem) {
        uint32_t hp = m->fw.mdircv_head & m->mem_mask, tp = m->fw.mdircv_tail & m->mem_mask;
        uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
        uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
        if (head == 0x80 && tail == 0x80) {                 // queue initialised, empty
            if (m->dsp_msg_ctr == 0) m->dsp_msg_ctr = m->dsp_msg_delay;
            else if (--m->dsp_msg_ctr == 0) {
                uint32_t q = m->fw.mdircv_q & m->mem_mask;
                m->mem[q]     = m->dsp_msg_len;             // word0 high byte = body length
                m->mem[q + 1] = m->dsp_msg_type;            // word0 low byte  = DSP msg type
                for (int i = 0; i < m->dsp_msg_len && i < 32; ++i) m->mem[q + 2 + i] = m->dsp_msg_body[i];
                uint16_t words = (uint16_t)(1 + (m->dsp_msg_len + 1) / 2);
                uint16_t nt = (uint16_t)(0x80 + words);      // advance write ptr (not empty)
                m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
                mad2_raise_fiq(m, 0);                      // DSP signals MDIRCV via FIQ0
                m->dsp_injected = 1;
            }
        }
    }

    // DSP STEADY-STATE KEEP-ALIVE (the faithful 0xE4-watchdog cancel; Phase 3, RE-05).
    //
    // After the boot DSP code upload + self-test PASS, the live silicon's DSP keeps
    // producing periodic unsolicited DSP->MCU MDIRCV traffic (the MDI receive stream).
    // The MDI subsystem consumes that traffic and reports task-14 (the readiness state
    // machine) its state-6 completion, so task-14's OWN STATE_FANOUT (0x267734) cancels
    // the 0xE4 (MSG_T4_MDI_FAULT) DSP-liveness watchdog soft-timer it armed once at boot
    // (@step 4.95M).5.
    //
    // HISTORICAL NOTE (pre-ASIC-fix): a silent DSP used to leave that watchdog uncancelled
    // → it expired ~106M / ~296M cyc → 0xE4 → SWDSP_STAGER 0x2D11F4 → reason 0x68. That is
    // NO LONGER TRUE on HEAD — the real-HW ASIC-register reads (0x20000=0xA1 / 0x20002=0x53,
    // commits 7723dd3 + 101458f) now let the firmware see the DSP as released, so silence
    // alone no longer trips 0x68. See the DEFAULT essay below for the measured A/B.
    //
    // The delivery mechanism (used by all modes — replay default + perpetual A/B): emit an
    // MDIRCV entry + raise FIQ0 — exactly the boot self-test responder's delivery pattern
    // (write word0 at the queue tail, advance mdircv_tail, signal FIQ0). IRQ4-FREE (a
    // post-boot IRQ4 double-links the OS task list — boot wedge at 0x2EEBEA), no poke of any
    // firmware-private byte ([0x11FF94]/[0x11FF15]/[0x11FF1F]/timer state), no 0x299E4C, no
    // trampoline: the firmware's own dispatcher routes the message and cancels its watchdog.
    //
    // Gating (never perturb boot): fire ONLY once the DSP code upload completed
    // ([0x11038C] = dsp_uploaded) AND the self-test has replied (dsp_selftest_replied),
    // and skip entirely while a manual DSPMSG injection is active (dsp_msg_en) or the
    // self-test responder is disabled (dsp_selftest_off). Cadence is driven off the
    // monotonic cycle clock (rtc_mono, rebase-safe) at the DSP's steady MDI report rate
    // (~1 s ≈ 5M cyc; clock ≈ 4.93 MHz) — well inside the ~21.5 s / ~101M-step watchdog
    // budget, matching a real steady-state heartbeat, not a bare once-per-window
    // minimum. Only written when the queue has room (head == tail, i.e. empty) so real
    // MDI traffic is never trampled.
    //
    // The completion message: task-14's recv decodes an id 0xFCD..0x100B as
    // state=(id-0xFCD)/9 and indexes the same timer-id table (0x32C3DC) to pick which
    // watchdog to cancel; state-6 col-1 (id 0x1004) is the 0xE4 watchdog. The MDI
    // subsystem posts that to task-14 in response to the DSP's MDI stream. The exact
    // MDIRCV group/payload that drives it was pinned by empirical iteration over the
    // candidate DSP->MCU group bytes (the locked Phase-3 acceptance path): the group
    // kept is the one that makes WATCH=0x267734 fire post-boot AND holds the framebuffer
    // at the DSP-active 14232 (never the DSPNOSELFTEST no-DSP 9072).
    //
    // READ-ONLY cancel-confirmation: task14_state ([0x10AE61]) / task14_status
    // ([0x10B1FC+state]) are sig-resolved FwAddrs fields the emitter may READ to confirm
    // the firmware cancelled its watchdog — never written.
    //
    // Knobs (diagnostics / A-B only):
    //   DSPNOKEEPALIVE=1   disable the keep-alive entirely (silent DSP; clean on HEAD)
    //   DSPKA_GROUP=0xNN   override the MDIRCV group byte (empirical iteration)
    //   DSPKA_CYC=N        override the cadence in cycles (default 5000000 ≈ 1 s)
    //   DSPKA_LOG=1        print each emit (first few + every 64th)
    // START GATE = step budget, NOT MCU RAM. We do not fully model the DSP boot self-test
    // handshake, and its activation flags are unusable as a gate: dsp_selftest_replied only
    // fires in configs that exercise the self-test (never in a plain boot_trace run), and
    // [dsp_uploaded] / task14_state==0xFF are MCU-private bytes a real DSP could not see.
    // The DSP startup timing IS consistent, so we start the steady-state stream on a fixed
    // step budget after boot: once dsp_steps passes DSPKA_START (default 5,000,000 ≈ just
    // after the 0xE4 DSP-liveness watchdog is armed at ~4.95M, the point where cancelling it
    // is meaningful). The DSP-side guards (selftest_off, an active manual DSPMSG injection)
    // still suppress it; the ring init+empty check below still prevents trampling real MDI.
    if (m->mem && !m->dsp_selftest_off && !m->dsp_msg_en && !m->dsp_no_keepalive) {
        static int      ka_init = 0;
        static int      ka_off  = 0;
        static int      ka_log  = 0;
        static uint64_t ka_start = 5000000u;  // DSPKA_START: start step budget (~4.95M watchdog arm)
        static uint32_t ka_cyc  = 25000000u;  // ~5 s @ 4.93 MHz — realistic idle DSP report rate.
                                              // The 0xE4 DSP-liveness watchdog (soft-timer slot 36)
                                              // fires every ~97M steps (~19-21 s) and trips reason-0x68
                                              // unless >=1 non-0xE4 MDI ring entry arrived since the last
                                              // tick (counter [0x111862], BROKER_DECODE depth>0 — a bare
                                              // FIQ0 doorbell with an empty ring does NOT count). ~5 s
                                              // gives ~10 deposits/window: a quiet-idle cadence (vs the old
                                              // chatty ~1 s = ~50/window) with ~10x margin over the >=1
                                              // the firmware needs. DSPKA_CYC overrides for A/B.
        static uint8_t  ka_group = 0x03;      // DSP->MCU MDI group — BIG-ENDIAN high/even byte.
                                              // HW (v6.33 RAM read): real DSP idle stream is group 0x03.
                                              // Old code put 0x04 in the LOW byte => emitted group 0x00
                                              // => broker re-fans to task-2 UI => heap storm ~186M.
        static uint8_t  ka_id    = 0x00;      // entry low/odd byte (id/sub); group alone drives MDI cancel
        static int      ka_replay = 0;        // DEFAULT = perpetual idle heartbeat (faithful; see essay).
                                              // DSPKA_REPLAY: 0=perpetual(default), 1=boot-burst-then-silent
                                              // (A/B; starves the watchdog -> 0x68 @203M), 2=early-block+
                                              // steady (A/B; storms reason-0x6C @824M)
        static int      ka_ridx   = 0;        // replay steady-record cursor (0..KA_REPLAY_N)
        static int      ka_pre    = 0;        // replay early-block word cursor (0..ka_early_n; mode 2)
        static int      ka_early_n = 16;      // DSPKA_EARLY_N: upper bound of early words to emit (bisection)
        static int      ka_early_from = 0;    // DSPKA_EARLY_FROM: lower bound (emit early idx [from, n))
        if (!ka_init) {
            ka_init = 1;
            ka_off = getenv("DSPNOKEEPALIVE") ? 1 : 0;
            ka_log = getenv("DSPKA_LOG") ? 1 : 0;
            // FAITHFUL DEFAULT = perpetual idle heartbeat (ka_replay=0), set above. The DSP
            // deposits a minimal group-0x03 record into the MDIRCV ring + raises FIQ0 on a slow
            // idle cadence (ka_cyc ~5 s), forever. This models the real DSP's genuine periodic MDI
            // activity at standby — NOT a "crude keep-alive". The firmware DEMANDS it (see below).
            //
            //   THE WATCHDOG IS PERPETUAL & COUNTER-DRIVEN — there is NO one-time cancel (RE'd
            //   firmware-wide, 2026-06-05). The 0xE4 DSP-liveness watchdog is soft-timer slot 36
            //   (period 0x9CD), re-armed UNCONDITIONALLY at 3 sites (init 0x232824, stager
            //   0x2D12A4, healthy-tick 0x2EDC5A) with ZERO disarm sites anywhere. Every ~97M-step
            //   (~19-21 s) tick it reads MDI-activity counter [0x111862]: ==0 -> SWDSP_STAGER(1) =
            //   reason 0x68; else re-arm + zero the counter. The counter is bumped (0x2EDB16) ONLY
            //   when a non-0xE4 MDI ring entry is decoded (TASK_4 0x2EDB04). The earlier
            //   "task-14 state-6 cancels the watchdog" model was a MISREAD — STATE_FANOUT touches
            //   no soft-timer. So liveness = >=1 ring deposit per window, forever; a silent DSP
            //   ALWAYS trips 0x68. (The MDIRCV ring routes only to task-4 — it can NEVER reach the
            //   task-14 SWDSP machine, so injecting a handshake msg is architecturally impossible.)
            //
            //   IS IT THE INTERRUPT OR THE MESSAGE? The ring ENTRY, not the bare IRQ. FIQ0 (0x2BAB82) always TASK_SEND_IRQ(4,4), but TASK_4 then calls
            //   BROKER_DECODE 0x2BACF0 which returns 0 on empty queue depth -> TASK_4 0x2EDC46
            //   (cmp r4,#0; beq) loops back WITHOUT bumping. So a bare FIQ0/DSPINT doorbell with an
            //   empty ring does NOT feed the watchdog; a real ring entry must be present. CONTENT
            //   past the group byte is irrelevant — any non-0/non-0xE4 decode counts.
            //
            //   WHY PERPETUAL IS NOW SAFE (the old H5 tradeoff is GONE): perpetual used to drive
            //   the 0x116854 MDI-buffer double-free -> wild-PC @2.46B. That was CURED by honoring
            //   the firmware's 0x2000C master interrupt-disable in FIQ delivery
            // . The ring-wrap guard below (END 0xE4
            //   -> START 0x80) keeps the ring bounded so there is no overrun/leak either.
            //
            //   A/B MEASURED ON HEAD (no-SIM, recovery OFF, faithful baseline, 2026-06-05):
            //     ka_replay=0 (perpetual)        -> CLEAN to 2.6B, 14 watchdog ticks ALL healthy
            //                                       (counter ~51), ZERO 0x68, ZERO wild-PC  ← DEFAULT
            //     ka_replay=1 (burst-then-silent)-> 0x68 @202.8M (39-record burst drains before the
            //                                       2nd ~97M tick -> counter starves). The old
            //                                       "silent is safe / ASIC reads expire-proof the
            //                                       watchdog" claim was REFUTED by direct WDOGLOG.
            //     ka_replay=2 (early+steady)     -> reason-0x6C @824M (early-block broker storm —
            //                                       distinct mechanism; the 16 early words route as
            //                                       standalone MDI records -> idle wedge -> broker
            //                                       supervisor [0x11FF1A] climbs to 15)
            //     DSPNOKEEPALIVE=1 (silent)      -> 0x68 @105.9M (first tick, counter==0)
            //
            // RESIDUAL FAITHFULNESS GAP (deferred): perpetual skips the early block, so our ring
            // reaches ~idx 0xCE, not silicon's 0xDF (cosmetic). Fully faithful = deliver the
            // 16-word early block as a self-test RESULT BLOB (raw data read once for the verdict,
            // NOT 16 routable FIQ0 records — which is what makes REPLAY=2 storm). Unexamined.
            //
            // No firmware-private poke, no forced message, no 0x299E4C, no trampoline, rtc_mono
            // cadence only, no empirical fudge. The DSPKA_* env knobs below stay for A/B RE.
            const char* rp = getenv("DSPKA_REPLAY");
            if (rp && *rp) ka_replay = (int)strtol(rp, 0, 0);
            const char* en = getenv("DSPKA_EARLY_N");          // bisection: upper bound of early words
            if (en && *en) { ka_early_n = (int)strtol(en, 0, 0); if (ka_early_n > 16) ka_early_n = 16; if (ka_early_n < 0) ka_early_n = 0; }
            const char* ef = getenv("DSPKA_EARLY_FROM");        // bisection: lower bound (emit idx [from,n))
            if (ef && *ef) { ka_early_from = (int)strtol(ef, 0, 0); if (ka_early_from < 0) ka_early_from = 0; if (ka_early_from > 16) ka_early_from = 16; }
            ka_pre = ka_early_from;
            const char* c = getenv("DSPKA_CYC");
            if (c && *c) { long v = strtol(c, 0, 0); if (v > 0) ka_cyc = (uint32_t)v; }
            const char* st = getenv("DSPKA_START");   // start step budget (default 5M ≈ watchdog arm)
            if (st && *st) { long v = strtol(st, 0, 0); if (v >= 0) ka_start = (uint64_t)v; }
            const char* g = getenv("DSPKA_GROUP");
            if (g && *g) ka_group = (uint8_t)strtol(g, 0, 0);
            const char* id = getenv("DSPKA_ID");
            if (id && *id) ka_id = (uint8_t)strtol(id, 0, 0);
        }
        // START on the fixed step budget (dsp_steps >= ka_start, ~4.95M = just after the
        // 0xE4 watchdog arm). This replaces the old read of task14_state==0xFF — the
        // readiness state machine's arm point — which never reaches 0xFF in some configs
        // and is MCU-private RAM the DSP could not observe. Emitting earlier would perturb
        // the firmware's own boot use of the ring, so the budget must clear the arm point.
        if (!ka_off && m->dsp_steps >= ka_start
            && (m->dsp_hb_last == 0 || m->rtc_mono - m->dsp_hb_last >= ka_cyc)) {
            uint32_t hp = m->fw.mdircv_head & m->mem_mask;
            uint32_t tp = m->fw.mdircv_tail & m->mem_mask;
            uint16_t head = (uint16_t)((m->mem[hp] << 8) | m->mem[hp + 1]);
            uint16_t tail = (uint16_t)((m->mem[tp] << 8) | m->mem[tp + 1]);
            // Only emit into an empty queue (head == tail) so a real MDI entry is
            // never trampled. The queue base index is 0x80 (word units).
            if (head == tail && head >= 0x80) {
                uint32_t q   = m->fw.mdircv_q & m->mem_mask;
                uint32_t off = (uint32_t)(tail - 0x80) * 2;     // byte offset of word0
                if (ka_replay) {
                    // REPLAY MODE (A/B only): emit one captured 2-word record per cadence tick,
                    // walking the real-HW boot burst, then go SILENT (ka_ridx == KA_REPLAY_N).
                    // KEPT FOR A/B: this STARVES the watchdog (counter hits 0 at the 2nd ~97M
                    // tick -> 0x68 @203M) — proving the watchdog is perpetual/counter-driven, with
                    // no one-time cancel. The default (perpetual, above) is the faithful path.
                    if (ka_replay >= 2 && ka_pre < ka_early_n) {
                        // EARLY-BLOCK PHASE (mode 2): emit the 16 raw boot words first, one word
                        // per tick, so the ring runs 0x80->0xDF exactly like silicon. group = BE
                        // high byte; these are raw self-test-result words, not FF/seq records.
                        uint16_t w = ka_early[ka_pre];
                        m->mem[q + off]     = (uint8_t)(w >> 8);          // word group (BE even byte)
                        m->mem[q + off + 1] = (uint8_t)(w & 0xFF);        // word id/low
                        uint16_t nt = (uint16_t)(tail + 1);              // 1 word
                        m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
                        mad2_raise_fiq(m, 0);                           // FIQ0: DSP signalled MDIRCV
                        m->dsp_hb_last = m->rtc_mono;
                        m->dsp_hb_pulses++;
                        if (ka_log)
                            printf("[dsp-ka] replay early %d/16  %04X (grp=0x%02X id=0x%02X) @mono=%llu  t14=%u\n",
                                   ka_pre + 1, w, w >> 8, w & 0xFF, (unsigned long long)m->rtc_mono,
                                   m->mem[m->fw.task14_state & m->mem_mask]);
                        ka_pre++;
                        if (ka_log && ka_pre == 16)
                            printf("[dsp-ka] replay early block DONE (16 words) -> steady stream @mono=%llu\n",
                                   (unsigned long long)m->rtc_mono);
                    } else if (ka_ridx < KA_REPLAY_N) {
                        // record = word0 0xFF<seq> (group 0xFF sync), word1 0x03<id> (group 0x03).
                        // BIG-ENDIAN: group in the even/low byte (dequeue lsr #8), id in the odd byte.
                        m->mem[q + off]     = 0xFF;                       // word0 group (sync marker)
                        m->mem[q + off + 1] = ka_replay_seq[ka_ridx];     // word0 id  (seq 0x8E/0x8D)
                        m->mem[q + off + 2] = 0x03;                       // word1 group 0x03 (telemetry)
                        m->mem[q + off + 3] = ka_replay_id[ka_ridx];      // word1 id
                        uint16_t nt = (uint16_t)(tail + 2);               // 2 words
                        m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
                        mad2_raise_fiq(m, 0);                           // FIQ0: DSP signalled MDIRCV
                        m->dsp_hb_last = m->rtc_mono;
                        m->dsp_hb_pulses++;
                        if (ka_log)
                            printf("[dsp-ka] replay rec %d/%d  FF%02X 03%02X @mono=%llu  task14_state=%u\n",
                                   ka_ridx + 1, KA_REPLAY_N, ka_replay_seq[ka_ridx], ka_replay_id[ka_ridx],
                                   (unsigned long long)m->rtc_mono,
                                   m->mem[m->fw.task14_state & m->mem_mask]);
                        ka_ridx++;
                        if (ka_log && ka_ridx == KA_REPLAY_N)
                            printf("[dsp-ka] replay BURST COMPLETE (%d records) -> silent @mono=%llu\n",
                                   KA_REPLAY_N, (unsigned long long)m->rtc_mono);
                    }
                    // ka_ridx == KA_REPLAY_N: burst done -> emit nothing further (silent, as silicon).
                } else {
                    // PERPETUAL MODE (default): one group-0x03 word per cadence, forever.
                    // BIG-ENDIAN entry: dequeue (0x2BACA8 lsr r0,r7,#8) takes group = word>>8 =
                    // the EVEN byte; odd byte is the id. Old code wrote the group into the ODD
                    // byte, so it emitted group 0x00 (the UI-storm route), not the intended value.
                    m->mem[q + off]     = ka_group;             // word0 high/even = DSP->MCU group (0x03)
                    m->mem[q + off + 1] = ka_id;                // word0 low/odd  = id/sub
                    uint16_t nt = (uint16_t)(tail + 1);         // 1 word, no payload
                    // Wrap the write ptr at the ring boundary, EXACTLY as the firmware's own
                    // consumer does (MDIRCV_DEQUEUE 0x2BAC72 wraps head END->START via pool
                    // literals base=0x10000, START=0x10100 [idx 0x80], END=0x101C8 [idx 0xE4]).
                    // The MDIRCV ring is a 100-entry circular buffer (idx 0x80..0xE3); the
                    // head/tail ptrs live at 0x101CA/0x101C8, just past END. Real silicon's DSP
                    // goes SILENT at idx 0xDF after the boot burst, so it never wraps — but our
                    // perpetual keep-alive produces forever. WITHOUT this wrap the write ptr
                    // marches past idx 0xDF straight into idx 0xE4 (=0x101C8) and overwrites the
                    // tail pointer itself, corrupting head/tail -> the firmware then floods task-4
                    // with 0x7x msgs -> task-2 mailbox saturates -> the no-SIM idle heap leak
                    // (onset ~185M steps, inversely proportional to keep-alive cadence). Wrapping
                    // it circularly (matching the firmware's END->START) keeps the ring bounded:
                    // no overrun, no leak; the firmware consumes the records identically so the
                    // 0xE4 watchdog cancel (no reason-0x68) is intact.
                    if (nt >= 0xE4) nt = 0x80;                  // ring END (idx 0xE4 / 0x101C8) -> START (idx 0x80 / 0x10100)
                    m->mem[tp] = (uint8_t)(nt >> 8); m->mem[tp + 1] = (uint8_t)nt;
                    mad2_raise_fiq(m, 0);                     // FIQ0: DSP signalled MDIRCV
                    m->dsp_hb_last = m->rtc_mono;
                    m->dsp_hb_pulses++;
                    if (ka_log && (m->dsp_hb_pulses < 8 || (m->dsp_hb_pulses & 0x3F) == 0))
                        printf("[dsp-ka] emit #%llu group=0x%02X @mono=%llu  task14_state=%u\n",
                               (unsigned long long)m->dsp_hb_pulses, ka_group,
                               (unsigned long long)m->rtc_mono,
                               m->mem[m->fw.task14_state & m->mem_mask]);
                }
            }
        }
    }
}

// HLE COBBA tone reader (shared by every HLE DSP backend; see DspOps.hle_tone). The MCU
// programs oscillator frequencies (1/4-Hz units) + amplitude into the HPI mailbox window;
// with no real DSP to play them, we report the active tone so emu_audio synthesizes it into
// the PCM stream. Registers are model-invariant (the HPI base .cobba = 0x100E0 everywhere).
int dsp_hle_tone(Mad2* m, int* f1_hz, int* f2_hz) {
    if (!m->mem) return 0;
    uint32_t amp_a = DCT3_TONE_AMP  & m->mem_mask;
    if (((m->mem[amp_a] << 8) | m->mem[(amp_a + 1) & m->mem_mask]) == 0) return 0;  // amplitude gate
    uint32_t o1 = DCT3_TONE_OSC1 & m->mem_mask, o2 = DCT3_TONE_OSC2 & m->mem_mask;
    int f1 = (int)(((m->mem[o1] << 8) | m->mem[(o1 + 1) & m->mem_mask])) >> 2;       // reg is 1/4 Hz
    if (f1 <= 0) return 0;
    int f2 = (int)(((m->mem[o2] << 8) | m->mem[(o2 + 1) & m->mem_mask])) >> 2;
    *f1_hz = f1;
    *f2_hz = (f2 > 0) ? f2 : 0;
    return 1;
}

const DspOps mad2_dsp_default = {
    "default", dsp_default_read, dsp_default_write, dsp_default_tick, dsp_hle_tone,
};
