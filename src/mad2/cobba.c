// COBBA RX RF front-end sample source — see cobba.h for the architecture note.
//
// Grounding (5110 NSE-1 v5.30 A, DSP vec16 ISR live disasm 0x3238-0x3268): the DSP reads
// 32 words per frame from port 0x27 into a BK=16 circular buffer @0x80; a REAL-coefficient
// FIR (coeffs @0x21A9) runs separately over even words (-> accum A) and odd words
// (-> accum B) — the stream is interleaved I,Q,I,Q..., ONE signed 16-bit component per
// read — then SQDST power accumulation into the 32-bit cell [0xA8], log2 conversion at
// 0x48D0 (exp/norm + mantissa*1541 interpolation), level -> [0xAB], sweep accumulate
// result[ch] += [0xAB]>>2 at 0x9BD, bubble-sort 0x9DD (records = (negated ch, accum)
// pairs at [0xE14+2k]) and a post-transform 0x9EE before the d2m 0x8B post.
//
// Why a constant-envelope tone and not white noise: the per-pass measurement is close to
// a single filtered-sample magnitude, so white noise hands every channel one draw from an
// exponential distribution — measured floor spread was +-6000 accum units, drowning a
// 50 dB programmed gradient. A GMSK carrier is constant-envelope (rotating phase, fixed
// magnitude); |FIR(tone)|^2 is constant -> zero measurement variance, exact monotone
// levels. The phase restarts on every tune so each pass over a channel is identical
// (the same determinism the rom4 HLE applies to its serving-RSSI pattern).
//
// Knobs (native co-sim only; explicit opt-in until promoted):
//   DSP54_RFMODEL=1                enable the synthetic RX source on port 0x27
//   DSP54_RF_CELLS=arfcn:dbm,...   cells on the air (default "1:-60" — the HLE analogue)
//   DSP54_RF_NOISE=<dbm>           floor for every other channel (default -110)
//   DSP54_RF_FS=<dbm>              dBm mapping to full-scale amplitude (calibration)
//   DSP54_RF_AMP=<0..32767>        raw amplitude override, ALL channels (calibration)
//   DSP54_RF_TONE=<0..15>          phase step in 1/16 turns per complex sample
//                                  (default 1; -1 = white-noise source for A/B)
//   DSP54_RFLOG=1                  one line per tuned-channel change + per synth word
//   DSP54_RF_CHTAP=synth           take the channel from the RF SYNTHESIZER programming
//                                  (cobba_rf_synth) instead of the bridge's cell tap
//   DSP54_RF_FCCHSTEP=<0..15>      FCCH tone phase step per complex sample (default 2)
//   DSP54_RF_CHFORCE=<arfcn>       pin the modelled channel (A/B: isolates "is it the tap
//                                  or the burst?" without touching either tap)

#include "mad2/cobba.h"
#include "mad2/gsm_dl.h"
#include <stdio.h>
#include <stdlib.h>

#define RF_MAX_CELLS 32
static struct {
    int      init, on, ncells;
    uint16_t cell_arfcn[RF_MAX_CELLS];
    int      cell_dbm[RF_MAX_CELLS];
    int      noise_dbm, fs_dbm, raw_amp;   // raw_amp -1 = off
    int      tone_step;                    // -1 = noise source
    int      log_on;
    uint32_t lcg;
    uint32_t lastch;
    unsigned phase, parity, nlog;
    // FCCH burst gating (DSP54_RF_FCCH=1): the tone is emitted only during slot 0 of
    // frames whose FN mod 51 carries FCCH ({0,10,20,30,40}); everywhere else the carrier
    // transmits constant-amplitude noise (a BCCH carrier never stops transmitting). The
    // 5110 detector REQUIRES this shape (RE'd spec, handoff): it pairs detection events
    // by spacing {10,11,20,21,...} frames +-9 blocks and ABORTS after 500 above-threshold
    // blocks — a continuous tone degenerates the spacing and trips the abort, which is
    // why the ungated tone always ended in NO_PSW_FOUND.
    //
    // TIMEBASE (DSP54_RF_RPF), pinned off the DSP's own block clock: the hunt ISR
    // (0x32F4) consumes exactly 32 port reads per invocation and the bridge paces it at
    // 32 quarter-symbols, i.e. ONE read per quarter-symbol = 4 reads = 2 complex samples
    // per symbol (2x oversampling). A GSM frame is 1250 symbols = 5000 qs, so a frame is
    // **5000 reads** and the FCCH burst (148 symbols) is 592 reads at frame start; the
    // symbol index inside the burst is pos >> 2. Cross-check from the detector itself:
    // block03 compares the block counter [0x1257] against 1523 and subtracts 1719 from
    // the 32-bit spacing accumulator [0x1262] — 1562 blocks x 32 qs = 49984 qs = 10
    // frames, 1719 x 32 = 11 frames, which only works out with 5000 qs (= 5000 reads)
    // per frame. (999f52b's "20000" inverted the 32-reads-per-32-qs ratio and stretched
    // every model frame to 4 real frames, so the burst spacing the detector pairs on
    // could never land in the +-9-block window.)
    int      fcch_on, xcch_on, xcch_pending, xcch_next;
    int      rpf, fcch_step, fcch_len;
    uint32_t nreads, fn, interval_reads, sync_advance, forced_fcch;
    uint32_t last_sched_pos;
    char     last_burst;
    // Synthesizer tap (cobba_rf_synth): the ARFCN the RF PLL was last programmed to.
    // 0xFFFF = never programmed. `synthtap` (DSP54_RF_CHTAP=synth) makes it authoritative;
    // OFF (default) the bridge's cell tap still supplies the channel — see the header note
    // on why the synth tap cannot be the default YET.
    uint16_t synth_arfcn;
    int      synthtap;
    int      chforce;                      // DSP54_RF_CHFORCE=<arfcn> (A/B override, -1 off)
    unsigned nsynth, nsynthlog;
} g_rf = { .lastch = 0xFFFFFFFFu, .synth_arfcn = 0xFFFFu };

static void rf_lazy(void) {
    if (g_rf.init) return;
    g_rf.init = 1;
    const char *v = getenv("DSP54_RFMODEL");
    g_rf.on = v ? atoi(v) : 0;
    g_rf.noise_dbm = -110; g_rf.fs_dbm = -40; g_rf.raw_amp = -1; g_rf.tone_step = 1;
    g_rf.lcg = 0x1234567u;
    g_rf.log_on = getenv("DSP54_RFLOG") ? 1 : 0;
    if ((v = getenv("DSP54_RF_NOISE")) && *v) g_rf.noise_dbm = atoi(v);
    if ((v = getenv("DSP54_RF_FS"))    && *v) g_rf.fs_dbm    = atoi(v);
    if ((v = getenv("DSP54_RF_AMP"))   && *v) g_rf.raw_amp   = atoi(v);
    if ((v = getenv("DSP54_RF_TONE"))  && *v) g_rf.tone_step = atoi(v);
    g_rf.rpf = 5000;
    g_rf.chforce = -1;
    if ((v = getenv("DSP54_RF_CHFORCE")) && *v) g_rf.chforce = atoi(v);
    if ((v = getenv("DSP54_RF_CHTAP"))   && *v) g_rf.synthtap = (*v == 's');
    if ((v = getenv("DSP54_RF_FCCH"))  && *v) g_rf.fcch_on   = atoi(v);
    if ((v = getenv("DSP54_RF_RPF"))   && *v && atoi(v) > 0) g_rf.rpf = atoi(v);
    if ((v = getenv("DSP54_RF_OFFSET")) && *v) g_rf.nreads = (uint32_t)strtoul(v, 0, 0);
    g_rf.fcch_len = 592;
    if ((v = getenv("DSP54_RF_FCCHLEN")) && *v && atoi(v) > 0) g_rf.fcch_len = atoi(v);
    g_rf.fcch_step = 2;                    // +1/4 turn per symbol (see the FCCH note)
    if ((v = getenv("DSP54_RF_FCCHSTEP")) && *v) g_rf.fcch_step = atoi(v) & 15;
    const char *c = getenv("DSP54_RF_CELLS");
    if (!c || !*c) c = "1:-60";
    while (*c && g_rf.ncells < RF_MAX_CELLS) {
        char *e; long a = strtol(c, &e, 0);
        if (e == c || *e != ':') break;
        long d = strtol(e + 1, &e, 0);
        g_rf.cell_arfcn[g_rf.ncells] = (uint16_t)a;
        g_rf.cell_dbm[g_rf.ncells]   = (int)d;
        g_rf.ncells++;
        if (*e != ',') break;
        c = e + 1;
    }
}

int cobba_rf_enabled(void) { rf_lazy(); return g_rf.on; }
uint32_t cobba_rf_time(void) { rf_lazy(); return g_rf.nreads; }
void cobba_rf_sync_advance(uint32_t quarter_symbols) {
    rf_lazy();
    g_rf.sync_advance = quarter_symbols;
    const char *v = getenv("DSP54_RF_SYNCLEN");
    if (v && *v && atoi(v) > 0) g_rf.fcch_len = atoi(v);
    if (g_rf.log_on)
        fprintf(stderr, "[rf] synchronized air schedule +%u qs at read=%u fcch_len=%d\n",
                quarter_symbols, g_rf.nreads, g_rf.fcch_len);
}
void cobba_rf_sync_fcch(uint32_t quarter_symbols_until_burst) {
    rf_lazy();
    /*
     * The confirmation table starts on the frame-30 FCCH: its following gaps are
     * 10,11,10,10,10,10,11 frames, exactly the sequence block03 schedules.  Put
     * that burst at the firmware's recovered timing reference instead of applying
     * a run-specific absolute displacement to the free-running air clock.
     */
    const uint32_t mf_reads = 51u * (uint32_t)g_rf.rpf;
    const uint32_t frame30 = 30u * (uint32_t)g_rf.rpf;
    uint32_t future = (g_rf.nreads + quarter_symbols_until_burst) % mf_reads;
    uint32_t advance = (frame30 + mf_reads - future) % mf_reads;
    cobba_rf_sync_advance(advance);
}
void cobba_rf_align_sch(void) {
    cobba_rf_align_frame(1u);
}
void cobba_rf_arm_xcch(void) {
    rf_lazy();
    g_rf.xcch_pending = 1;
    g_rf.xcch_next = 0;
}
int cobba_rf_xcch_pending(void) {
    rf_lazy();
    return g_rf.xcch_pending;
}
void cobba_rf_align_next_xcch(void) {
    rf_lazy();
    if (!g_rf.xcch_pending)
        return;
    unsigned frame51 = 2u + (unsigned)g_rf.xcch_next;
    if (frame51 > 5u) {
        g_rf.xcch_pending = 0;
        return;
    }
    /* A control-channel codeword is four independently armed normal-burst
     * windows.  Until CTSI's slot-delta table is represented exactly, bind
     * those observed capture boundaries to the corresponding air frames.
     * The samples remain the real encoded/modulated bursts and the DSP still
     * owns every equalization/FEC/report decision. */
    cobba_rf_align_frame(frame51);
    g_rf.xcch_next++;
    if (g_rf.xcch_next >= 4)
        g_rf.xcch_pending = 0;
}
void cobba_rf_align_frame(unsigned frame51) {
    rf_lazy();
    if (frame51 == 2u && g_rf.xcch_on && !g_rf.xcch_pending)
        return;                 // one alignment, then preserve burst 0/1/2/3 progression
    if (frame51 == 2u) {
        g_rf.xcch_on = 1;
    }
    const uint32_t mf_reads = 51u * (uint32_t)g_rf.rpf;
    int32_t lead = 0;
    const char *e = getenv(frame51 == 1u ? "DSP54_RF_SCHLEAD"
                                         : "DSP54_RF_BCCHLEAD");
    if (e && *e) lead = (int32_t)strtol(e, 0, 0);
    int64_t target = (int64_t)(frame51 % 51u) * (int64_t)g_rf.rpf -
                     (int64_t)lead;
    target %= (int64_t)mf_reads;
    if (target < 0) target += mf_reads;
    const uint32_t sch_start =
        (uint32_t)target;
    uint32_t scheduled = (g_rf.nreads + g_rf.sync_advance) % mf_reads;
    uint32_t delta = (sch_start + mf_reads - scheduled) % mf_reads;
    g_rf.sync_advance = (g_rf.sync_advance + delta) % mf_reads;
    if (g_rf.log_on || getenv("DSP54_FCCHPAIRTRACE"))
        fprintf(stderr,
                "[rf] aligned live RX arm to frame=%u: read=%u oldpos=%u lead=%d delta=%u advance=%u\n",
                frame51, g_rf.nreads, scheduled, lead, delta, g_rf.sync_advance);
}
void cobba_rf_inject_fcch(uint32_t quarter_symbols) {
    rf_lazy();
    g_rf.forced_fcch = quarter_symbols;
    g_rf.phase = 0;
    g_rf.parity = 0;
}
char cobba_rf_last_burst(void) { return g_rf.last_burst; }
uint32_t cobba_rf_last_frame(void) { return g_rf.fn; }
uint32_t cobba_rf_last_pos(void) { return g_rf.last_sched_pos; }

void cobba_rf_advance(uint32_t quarter_symbols) {
    rf_lazy();
    /* The FCCH hunt stimulus is still calibrated against its block-read clock.
     * Reconcile off-air CTSI intervals only after the real post-SCH xCCH command
     * arms normal-burst reception; applying both clocks during hunt double-counts
     * its deliberately sparse reads. */
    if (g_rf.on && g_rf.fcch_on && g_rf.xcch_on) {
        uint32_t old_fn = g_rf.nreads / (uint32_t)g_rf.rpf;
        /*
         * One RF sample read represents one quarter-symbol, but not every enabled
         * receiver mode reads continuously.  In RSSI/search modes the DSP may consume
         * only one short block during a full-frame hardware interval.  Reconcile those
         * reads against elapsed air time at the next frame tick; merely testing the
         * COBBA stream-enable bit made GSM time stall whenever a sparse reader left the
         * stream enabled.
         */
        uint32_t unread = quarter_symbols > g_rf.interval_reads
                        ? quarter_symbols - g_rf.interval_reads : 0u;
        g_rf.nreads += unread;
        g_rf.interval_reads = 0;
        uint32_t new_fn = g_rf.nreads / (uint32_t)g_rf.rpf;
        if (g_rf.log_on) {
            for (uint32_t fn = old_fn + 1u; fn <= new_fn && g_rf.nlog < 4096; fn++) {
                char bt = gsm_ts0_sched(fn % 51u);
                g_rf.nlog++;
                fprintf(stderr, "[rf] frame fn=%u ts0=%c read=%u (rx off)\n",
                        fn, bt ? bt : '-', fn * (uint32_t)g_rf.rpf);
            }
        }
    }
}

// dBm -> peak amplitude, relative to full-scale at g_rf.fs_dbm. Each -6 dB halves the
// amplitude; the 0..5 dB fraction is a Q15 table of 10^(-k/20) (no libm dependency).
static uint16_t rf_amp_for_dbm(int dbm) {
    int att = g_rf.fs_dbm - dbm;                 // dB below full-scale
    if (att <= 0) return 32767;
    static const uint16_t frac[6] = { 32767, 29204, 26028, 23197, 20675, 18426 };
    unsigned oct = (unsigned)att / 6u, rem = (unsigned)att % 6u;
    if (oct >= 15) return 0;
    return (uint16_t)(((uint32_t)frac[rem]) >> oct);
}

// Q15 cos(2*pi*k/16); sin(k) = cos((k-4) & 15).
static const int16_t rf_cos16[16] = {
    32767, 30274, 23170, 12540, 0, -12540, -23170, -30274,
    -32767, -30274, -23170, -12540, 0, 12540, 23170, 30274
};

// === Synthesizer tap ====================================================================
// RE'd from the 5110 mask ROM (fn 0x3FFB RX / 0x4006 TX, live-dump constants):
//
//   A = ARFCN;  A += [0x216B]        ; divider offset (measured 0x13A6 = 5030)
//   if ([0x216A] & 2) call 0x4049    ; packing 1  (16-bit word, tag bits 1:0 = 3)
//   if ([0x216A] & 4) call 0x402E    ; packing 2  (dual-modulus, prescaler 64)
//   if (AR2 == 49) { port(0x31) = AL; port(0x32) = AH; }   else queue the pair
//
// packing 2 (0x402E, the one this build uses — [0x216A] measured 0x0004):
//   Q = N >> 6 ; R = N & 63 ; word = (Q << 7) | R ; word.high |= 0x0030
// packing 1 (0x4049):
//   word = ((N << 3) & 0xFE00) | ((N << 2) & 0xFC) | 3
//
// Sanity check on the offset: ARFCN 1 -> N 5031 at a 200 kHz raster = 1006.2 MHz first
// LO = 935.2 MHz (GSM900 downlink ARFCN 1) + 71.0 MHz IF. That is the 5110's RX IF, so
// the constant really is a synthesizer divider and not a scratch value.
void cobba_rf_synth(uint16_t lo, uint16_t hi, uint16_t cfg, uint16_t off) {
    rf_lazy();
    if (!g_rf.on) return;
    long n = -1;
    if (cfg & 4u) {                          // dual-modulus packing (0x402E)
        if ((hi & 0x0030u) != 0x0030u) return;   // not an N-divider word on this bus
        uint32_t w = ((uint32_t)(hi & 0x000Fu) << 16) | lo;
        n = (long)(((w >> 7) << 6) | (w & 0x3Fu));
    } else if (cfg & 2u) {                   // split-field packing (0x4049)
        if (hi != 0u || (lo & 3u) != 3u) return;
        n = (long)((((long)(lo & 0xFE00u) >> 9) << 6) | ((lo & 0xFCu) >> 2));
    } else return;                           // no packing armed -> not a tune
    long arfcn = n - (long)off;
    g_rf.nsynth++;
    if (arfcn < 0 || arfcn > 1023) {
        // Outside the ARFCN space: a TX-band programming or a non-synth word on the same
        // serial bus. The downlink we model does not move — hold the current tuning, which
        // is what a receiver does when only the transmit side is reprogrammed.
        if (g_rf.log_on && g_rf.nsynthlog < 64) { g_rf.nsynthlog++;
            fprintf(stderr, "[rf] synth %04X:%04X N=%ld -> arfcn %ld (ignored)\n", hi, lo, n, arfcn); }
        return;
    }
    if (g_rf.log_on && g_rf.nsynthlog < 64) { g_rf.nsynthlog++;
        fprintf(stderr, "[rf] synth %04X:%04X N=%ld off=%u -> ARFCN %ld\n", hi, lo, n, off, arfcn); }
    g_rf.synth_arfcn = (uint16_t)arfcn;
}

uint16_t cobba_rf_sample(uint16_t cell_arfcn) {
    rf_lazy();
    uint16_t amp;
    // Channel selection. `DSP54_RF_CHTAP=synth` uses the synthesizer programming (the
    // physically right answer — see cobba_rf_synth). The default still uses the caller's
    // cell tap because the DSP's own tune path is dormant in the co-sim (measured: ONE
    // port-0x31 write in a 300M-step run), so a synth-only model would sit on the noise
    // floor forever and lose the working RSSI report.
    uint16_t arfcn = g_rf.synthtap ? g_rf.synth_arfcn : cell_arfcn;
    if (g_rf.chforce >= 0) arfcn = (uint16_t)g_rf.chforce;
    // Values that are not ARFCNs at all (the cell tap goes stale mid-hunt: measured
    // 30488, 12614, 7950, 2114, ...) mean "no retune" — hold the last valid channel,
    // exactly as the RF module keeps its synthesizer setting until reprogrammed.
    if (arfcn > 1023u) arfcn = (uint16_t)(g_rf.lastch <= 1023u ? g_rf.lastch : 0u);
    int dbm = g_rf.noise_dbm;
    if (g_rf.raw_amp >= 0) amp = (uint16_t)g_rf.raw_amp;
    else {
        for (int i = 0; i < g_rf.ncells; i++)
            if (g_rf.cell_arfcn[i] == arfcn) { dbm = g_rf.cell_dbm[i]; break; }
        amp = rf_amp_for_dbm(dbm);
    }
    if (arfcn != g_rf.lastch) {
        g_rf.lastch = arfcn;
        g_rf.phase = 0; g_rf.parity = 0;
        g_rf.lcg = 0x1234567u ^ (arfcn * 2654435761u);
        if (g_rf.log_on && g_rf.nlog < 4096) { g_rf.nlog++;
            fprintf(stderr, "[rf] tune ch=%u dbm=%d amp=%u\n", arfcn, dbm, amp); }
    }
    int step = g_rf.tone_step;
    if (g_rf.fcch_on) {
        uint32_t sched_read = g_rf.nreads + g_rf.sync_advance;
        uint32_t pos = sched_read % (uint32_t)g_rf.rpf;   // read index within the frame
        g_rf.fn = sched_read / (uint32_t)g_rf.rpf;
        g_rf.nreads++;
        g_rf.interval_reads++;
        char bt = gsm_ts0_sched(g_rf.fn % 51u);
        g_rf.last_burst = bt;
        g_rf.last_sched_pos = pos;
        if (g_rf.log_on && pos == 0 && g_rf.nlog < 4096) {
            g_rf.nlog++;
            fprintf(stderr, "[rf] frame fn=%u ts0=%c read=%u\n",
                    g_rf.fn, bt ? bt : '-', g_rf.nreads - 1u);
        }
        // The FCCH tone rotation is FIXED BY THE DETECTOR, not chosen: the hunt ISR's
        // 8-tap FIR (0x330C-0x3322) accumulates even port words into A and odd into B and
        // emits ONE complex word pair per SYMBOL (32 reads = 16 complex in, 8 complex out
        // — a 2:1 decimation), then the correlator at 0x332B-0x3342 consumes 4 of those
        // complex outputs as
        //     re = I0 + Q1 - I2 - Q3 ,  im = Q0 - I1 - Q2 + I3
        // which is exactly "multiply sample k by (-j)^k and sum" = a coherent derotation
        // of -90 degrees PER SYMBOL. So the tone it integrates to DC is +1/4 turn per
        // symbol (+67.7 kHz, the textbook FCCH). Our `step` is in 1/16 turns per COMPLEX
        // SAMPLE and there are 2 samples per symbol, so step = +2. (The old 14 = -2 gave
        // -1/4 turn per symbol; against a -90 deg/symbol derotator that alternates +-1 and
        // sums to zero — measured: [0x1344] never incremented once in a 300M run.)
        // DSP54_RF_FCCHSTEP=<k> overrides for A/B.
        if (g_rf.forced_fcch) {
            step = g_rf.fcch_step;
            g_rf.forced_fcch--;
        }
        else if (bt == 'F' && pos < (uint32_t)g_rf.fcch_len) step = g_rf.fcch_step;
        else if (bt == 'S' && pos < 592u) {
            // SCH sync burst: MSK phase-path modulation of the real coded SB bits at
            // 2 samples/symbol, in the same rotation sense as the FCCH tone above —
            // d=0 -> +90 deg/symbol (+2 LUT steps per complex sample), d=1 -> -90.
            static uint32_t sb_fn = 0xFFFFFFFFu; static int8_t sb_step[148];
            if (sb_fn != g_rf.fn) { sb_fn = g_rf.fn;
                uint8_t bits[148];
                gsm_sch_burst(g_rf.fn, 5u /*BSIC*/, bits);
                uint8_t state = 0; /* preceding guard/tail bit */
                gsm_gmsk_phase_steps(bits, sb_step, &state);
            }
            unsigned sym = pos >> 2;                       // 4 reads (2 complex) per symbol
            if (sym < 148u) step = (2 * sb_step[sym]) & 15;
            else step = -1;                                // guard tail -> noise below
        }
        else if (g_rf.xcch_on && bt == 'B' && pos < 592u) {
            /*
             * Frames 2..5 are one complete BCCH control block.  Until this branch
             * existed the scheduler advertised `B`, but the sample source fell
             * through to noise for all four bursts: the resident xCCH decoder had
             * literally never been given a BCCH codeword.
             *
             * SI3 is deliberately a fixed, valid 23-byte L2 block for this physical
             * layer test.  Higher layers may replace its identity later; what matters
             * here is that the DSP itself must equalize, deinterleave, convolutionally
             * decode and FIRE-check these 456 transmitted bits before producing MDI
             * 0x80 len=33.
             */
            static const uint8_t si3_l2[23] = {
                0x49, 0x06, 0x1B, 0x00, 0x01, 0x05, 0xF5, 0x10,
                0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x2B, 0x2B, 0x2B, 0x2B
            };
            static int ready;
            static int8_t nb_step[4][148];
            if (!ready) {
                uint8_t coded[4][114];
                gsm_bcch_encode(si3_l2, coded);
                for (unsigned b = 0; b < 4u; b++) {
                    uint8_t bits[148];
                    gsm_normal_burst(coded[b], 5u /* BCC from BSIC 5 */, bits);
                    uint8_t state = 0; /* preceding guard/tail bit */
                    gsm_gmsk_phase_steps(bits, nb_step[b], &state);
                }
                ready = 1;
            }
            unsigned burst = (unsigned)(g_rf.fn % 51u) - 2u;
            unsigned sym = pos >> 2;
            step = (2 * nb_step[burst][sym]) & 15;
        }
        else step = -1;
    }
    if (step < 0) {                     // white-noise source (A/B reference)
        if (g_rf.fcch_on) amp = rf_amp_for_dbm(g_rf.noise_dbm);
        g_rf.lcg = g_rf.lcg * 1103515245u + 12345u;
        int32_t u = (int32_t)(g_rf.lcg >> 16) - 32768;
        return (uint16_t)(int16_t)((u * (int32_t)amp) >> 15);
    }
    // Constant-envelope tone: even read = I = A*cos, odd read = Q = A*sin, then advance.
    int16_t c = g_rf.parity ? rf_cos16[(g_rf.phase - 4u) & 15u] : rf_cos16[g_rf.phase & 15u];
    if (g_rf.parity) g_rf.phase = (g_rf.phase + (unsigned)step) & 15u;
    g_rf.parity ^= 1u;
    return (uint16_t)(int16_t)(((int32_t)c * (int32_t)amp) >> 15);
}
