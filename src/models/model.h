// DCT3 model profile — the per-model facts the platform + shells read instead of
// hardcoding the 3310.
//
// The core (src/core) is model-agnostic; the MAD2 platform (src/mad2) models the
// shared ASIC; this layer carries everything that differs between DCT3 phones
// (3310 / 8850 / 7110 / 3330 / ...): the memory map, the LCD, the keypad, the
// battery ADC defaults, ASIC config, the firmware-build-specific RAM/code
// addresses (located by signature, see below), and the boot/HLE knobs.
//
// See docs/multi-model-architecture.md for the design and rationale.

#ifndef DCT3_MODEL_H
#define DCT3_MODEL_H

#include <stdint.h>
#include <stddef.h>

// LCD framebuffer is a static array sized to the largest geometry we support, so a
// runtime-variable LCD needs no heap. The active region is profile->lcd.{width,banks}.
//   3310 PCD8544 = 84x48 (6 banks); 8850/7110 candybar/Navi = 96x65 (9 banks).
// The array is sized to the largest CONTROLLER RAM, not the visible glass: the 3410's
// controller holds 102x72 internal RAM (96x65 visible), so LCD_MAX_W is 104 (>=102),
// not 96. See LcdSpec.ctrl_width below.
#define LCD_MAX_W      104
#define LCD_MAX_H      72
#define LCD_MAX_BANKS  (LCD_MAX_H / 8)   // 9

// --- Firmware-address signatures (NokiX LOCATE/SMARTFIND style) ---------------
// Firmware-build-dependent addresses move when the image is recompiled, so they are
// located by a masked byte-signature over the loaded flash rather than hardcoded.
// Mirrors main.c's sig_find()/resolve_spike() and NokiX's find/findfunc macros
// (ref/NokiX-scripts/.../macros/). Each address has a constant fallback (today's
// value) used when the signature misses, so day-one behaviour is unchanged.
typedef enum {
    SIG_NONE = 0,   // no signature; use the fallback constant verbatim
    SIG_CODE,       // the match offset IS the wanted code address (a function entry)
    SIG_LITERAL,    // the match locates a fn that loads a RAM base literal; the wanted
                    // RAM address = sig_ldr_literal(match + lit_off) + addend
    SIG_CODE_VLIT,  // like SIG_CODE, but the pattern is ambiguous (matches several leaves)
                    // so accept only the match whose loaded literal (at lit_off) equals an
                    // already-resolved FwAddrs field (verify_off) — e.g. the reason-stamp
                    // leaf that loads reboot_reason. The match offset is the wanted address.
} SigKind;

typedef struct {
    SigKind        kind;
    const uint8_t* pat;       // pattern bytes
    const uint8_t* mask;      // 0xFF = must match, 0x00 = wildcard
    uint8_t        len;       // bytes in pat/mask
    uint16_t       lit_off;   // SIG_LITERAL/SIG_CODE_VLIT: offset (from match) of the ldr-literal
    int32_t        addend;    // SIG_LITERAL: added to the extracted literal
    uint32_t       lo, hi;    // search window (0,0 = profile flash region)
    uint16_t       verify_off; // SIG_CODE_VLIT: offsetof(FwAddrs, field) the loaded literal must equal
} Sig;

// Firmware-build-specific addresses the platform + shells need. All filled by
// model_resolve(): a signature hit overrides the fallback constant in `fw`.
//
// The DSP shared-RAM mailbox block (0x100xx/0x101xx on the 3310) is the MAD2 DSP
// HPI layout + the firmware's standard MDIRCV queue convention — typically constant
// across MAD2 DCT3, but carried here so the platform reads it uniformly. The scratch
// flags (verdict, dsp_uploaded) are genuine per-build RAM variables → signatures.
typedef struct {
    // Boot/self-test scratch (per-build RAM variables).
    uint32_t verdict;        // self-test verdict byte  (3310 v5.79 = 0x11FF15)
    uint32_t sim_gate;       // disp49 SIM-gate bypass byte (3310 = 0x11FD1B)
    uint32_t dsp_uploaded;   // DSP-code-uploaded flag      (3310 = 0x11038C)
    // DSP shared-RAM mailbox / MDIRCV queue (HPI layout).
    uint32_t cobba;          // DSP Cobba command field      (3310 = 0x100E0)
    uint32_t dsp_cb_req;     // code-block request halfword  (3310 = 0x100E2)
    uint32_t dsp_cb_reply;   // code-block reply halfword    (3310 = 0x100E4)
    uint32_t dsp_mbox0;      // boot-ack mailbox slot 0      (3310 = 0x100FE)
    uint32_t dsp_mbox1;      // mailbox slot 1 / MDIRCV base (3310 = 0x10100)
    uint32_t mdircv_q;       // MDIRCV queue word0 base      (3310 = 0x10100)
    uint32_t mdircv_head;    // MDIRCV read pointer          (3310 = 0x101CA)
    uint32_t mdircv_tail;    // MDIRCV write pointer         (3310 = 0x101C8)
    // Shell-side helpers (string tracer + security lock).
    uint32_t get_string;     // get_string()                 (3310 v6.39 = 0x2BBFAC)
    uint32_t w_get_string;   // wide get_string()            (3310 v6.39 = 0x2BBCB8)
    uint32_t faid_cksum;     // stored FAID integrity sum    (FuBu v6.39 = 0x111B8A)
    uint16_t faid_cksum_val; // expected FAID sum            (FuBu v6.39 = 0x03B7)
    // Security-code oracle (firmware's own cipher; used by the web "reset code 12345"
    // hook + seccode reader). encrypt(r0,r1=ascii,r2=out) writes the 4-byte ciphertext;
    // verify(r0=ascii)->1 if accepted; store = RAM addr of the stored code verify reads.
    uint32_t seccode_encrypt; // encrypt fn                   (2100 v5.84 = 0x2FE780)
    uint32_t seccode_verify;  // verify fn                    (2100 v5.84 = 0x2FE992)
    uint32_t seccode_store;   // stored code RAM addr         (2100 v5.84 = 0x10FE78)
    // DSP boot-status mailbox slot: the firmware parks this halfword at 0xFFFF and
    // polls it for the DSP's ready/version reply before uploading the DSP code blocks.
    // mad2 returns dsp_boot_ready while the slot reads 0xFFFF. 0 = unused (model has
    // no such slot, or uses the legacy 0x10002->0 path). (8850 v5.31 = 0x10004, reply 6.)
    uint32_t dsp_boot_status; // DSP boot-status/ready slot   (8850 = 0x10004)
    uint16_t dsp_boot_ready;  // ready/version reply value    (8850 = 6)
    // Second DSP boot-status word the firmware cross-checks against the first (5110/serial-bus
    // loader 0x271AF4 requires [0x10004]==[0x10006] before it builds the DSP branch table
    // and sets the dsp_uploaded flag; without a matching second word the table is malformed
    // and "DSP ROM MISMATCH" persists). mad2 returns dsp_boot_ready for it too. 0 = unused
    // (3310/8850 loaders have no such cross-check). See docs/dsp-5110-mad1.md §7e/§8e.
    uint32_t dsp_boot_status2; // second ready word (5110 = 0x10006); 0 = none
    // Reset/reboot path (mad2 catches [0x20001]|=4 and discriminates by reason; see
    // docs/watchdog-reset-3310.md). All three addresses are sig-located so the
    // reset-recovery framework is firmware-agnostic — a missing signature falls back
    // to a build-specific constant or 0 (mad2 then degrades to warm-reboot).
    // `reboot_fn`     — universal reboot fn entry; LR + r0 captured at this PC give
    //                   the original caller's return addr + reason byte BEFORE the
    //                   noreturn fn's internal bl's clobber them. Used to recover
    //                   reasons 12 / 104 (no RAM-saved state) via LR-return.
    // `reboot_reason` — RAM byte the reboot fn stores the reason at right before it
    //                   raises the CCONT/MCU reset bit.
    // `reboot_save`   — start of the fatal handler's 3-word state save (LR, SPSR,
    //                   CPSR); populated only on reason 5 (watchdog/fatal). Recovery
    //                   loads PC <- *reboot_save, CPSR <- *(reboot_save+4) — lands
    //                   back at the instruction the panic interrupted.
    uint32_t reboot_fn;       // reboot fn entry              (3310 v5.79 = 0x2EEBAE)
    uint32_t reboot_reason;   // reboot reason byte           (3310 v5.79 = 0x11FF94)
    uint32_t reboot_save;     // saved fault state base       (3310 v5.79 = 0x11FF88)
    // ARM fatal-exception handler (undef/swi/pabort/dabort vectors @0x04/0x08/0x0C/0x10
    // all B here). On entry banked LR_<mode> = interrupted PC and SPSR_<mode> =
    // interrupted CPSR — the *earliest* point at which reason 5 ("CPU exception") can be
    // intercepted, *before* the handler stmia's the save block to RAM and bl's the
    // universal reboot fn with r0=5. The reboot_early intercept reads cpu->gprs[ARM_LR]
    // + cpu->spsr.packed here to recover without entering the reboot chain at all.
    // 0 = unknown (no signature hit, fallback constant not set) → eager intercept of
    // reason 5 degrades gracefully to the existing late `[0x20001]|=4` exc-return catch.
    uint32_t fatal_handler;   // ARM fatal-exception entry    (3310 v5.79 = 0x2F0BA4)
    // Generic assertion logger (~99 call sites in v5.79). Signature: `assert_log(code:r0,
    // data:r1)` — dispatches three special codes (0x5C12 / 0x5C0F / 0x5C13) and falls
    // through to a writer. r0 is the firmware's own fault code (one per failure site);
    // r1 is a code-specific data half-word; LR is the asserting call site. The platform
    // hooks PC == assert_log and records {cyc,code,data,lr} into a small ring buffer so
    // every reset catch can dump the firmware's own narrative of what went wrong. See
    // docs/watchdog-deep-re-findings.md §G.1.
    uint32_t assert_log;      // generic assertion logger     (3310 v5.79 = 0x2EDD4A)
    // Reason-byte setter — 3-instruction Thumb fn (ldr r1,=reboot_reason; strb r0,[r1];
    // bx lr). 13 callers in v5.79 — every code path that wants to STAGE a future reset
    // (with a graceful-shutdown sequence in between) writes the reason through this fn,
    // then posts a shutdown request. Hooking it captures the *cause* of a reset, often
    // many seconds before the visible [0x20001]|=4 edge fires. See
    // docs/dsp-reset-chain-3310-v579.md (the 22-second gap between stage and fire) and
    // docs/watchdog-deep-re-findings.md §G.8.
    uint32_t reason_setter;   // staged-reset reason-byte setter (3310 v5.79 = 0x2E0ED8)
    // Heap allocation-failure PC. The blocking allocator (v5.79 = 0x299AF8, ~1564 callers)
    // calls the heap core (0x2E533E); when that returns NULL (no free block) it branches to
    // a fail/block-retry block — it zeroes a flag, parks the task on a memory semaphore, then
    // retries. That branch target is malloc_fail (v5.79 = 0x299B26). Hooking PC==malloc_fail
    // counts allocation FAILURES (not allocations): a few = transient pressure, a sustained
    // climb = heap exhaustion — and a common precursor to a heap-smash wild-PC. At that PC r9
    // = the heap accounting struct and LR = the caller left waiting for memory. 0 = no sig.
    uint32_t malloc_fail;     // alloc-fail / block-retry PC   (3310 v5.79 = 0x299B26)
    // Task-14 (readiness state machine) cancel state — READ-ONLY for the DSP
    // keep-alive emitter (src/mad2/dsp_default.c). The emulated DSP supplies the
    // post-boot MDIRCV traffic the firmware consumes to advance task-14 to its
    // state-6 completion, so task-14's OWN STATE_FANOUT (0x267734) cancels the 0xE4
    // (MSG_T4_MDI_FAULT) DSP-liveness watchdog soft-timer (docs/dsp-reset-chain-3310-
    // v579.md §1.5). The keep-alive READS these to confirm/gate the cancel — it
    // NEVER writes them (the firmware owns every write; the keep-alive only delivers
    // the mailbox message + FIQ0). Both are sig-located off STATE_FANOUT's prologue
    // (ldr r4,=0x10AE58 + 9 = state byte; ldr r6,=0x10B1FC = per-state status base).
    // task14_state  — the task-14 current-state byte ([0x10AE58+9] = 0x10AE61): the
    //                 readiness state machine writes 0..7 then 0xFF (idle). The
    //                 emitter reads it to confirm boot reached state 6 (the 0xE4 arm).
    // task14_status — the per-state status table base ([0x10B1FC + state]): a state's
    //                 byte goes to 2 ("pending") while its subsystem watchdog is armed
    //                 and is advanced on completion. Read-only cancel-confirmation.
    uint32_t task14_state;    // task-14 state byte           (3310 v5.79 = 0x10AE61)
    uint32_t task14_status;   // per-state status table base  (3310 v5.79 = 0x10B1FC)
} FwAddrs;

// Table-driven signature → FwAddrs field. model_resolve() walks the profile's sigs[]
// and writes the resolved value into *(uint32_t*)((char*)fw + field_off).
typedef struct {
    const char* name;        // diagnostic ("sim_gate", "verdict", ...)
    size_t      field_off;   // offsetof(FwAddrs, <field>)
    Sig         sig;
} SigResolve;

// --- Memory map ---------------------------------------------------------------
// flash_base/boot_entry/io_lo/io_hi are constant across MAD2 DCT3; flash_size and
// eeprom_base are what shift (2 MB 3310/8850 vs ~4 MB 7110/3330 → EEPROM lands late).
typedef struct {
    uint32_t flash_base;   // 0x200000
    uint32_t flash_size;   // 0x200000 (2 MB) / 0x400000 (4 MB)
    uint32_t boot_entry;   // 0x200040 (HLE boot-ROM branch target)
    uint32_t io_lo;        // 0x010000 — start of the I/O-hook window (DSP mbox/MMIO)
    uint32_t io_hi;        // 0x100000 — RAM proper starts here (end of I/O window)
    uint32_t mmio_base;    // 0x020000 — MAD2 register window
    uint32_t mmio_size;    // 0x000100
    uint32_t eeprom_base;  // NVRAM/EEPROM partition start (3310 = 0x3D0000)
    uint32_t eeprom_size;  // partition length
    // Device-owned flash range for the core's io_range2 = [flash_base, flash_end).
    // flash_end = flash_base + flash_size (4 MB images extend past 0x400000).
    // Boot-block (top-boot Intel) parameter region: the high-churn FFS/PMM records rotate
    // through SMALL erase blocks at the TOP of the device, while the bulk array uses 64 KB
    // main blocks. This is the SHARED DCT3 MAD2 chip family — the generic default (applied
    // when param_base==0, see mad2_flash.c flash_erase_blk) is the top 64 KB = 8x 8 KB
    // parameter blocks, verified in-image on 3310/3410/8850 (f0f0/"PMM" headers at every
    // 8 KB boundary). Modelling this is REQUIRED for a faithful warm reboot: the firmware's
    // FFS garbage-collector erases ONE param block, and a too-coarse 64 KB erase would wipe
    // the neighbouring param blocks (incl. the active EEPROM block) → next-boot FFS validate
    // fails → CONTACT SERVICE. These two fields are OVERRIDES for a chip that deviates from
    // the generic default (set param_base!=0 to pin a custom region/size).
    uint32_t param_base;        // OVERRIDE: start of the small param region (0 = generic top-64 KB default)
    uint32_t param_block_size;  // OVERRIDE: erase-block size there (0 with a set base = 8 KB)
} MemMap;

// --- LCD ----------------------------------------------------------------------
typedef enum {
    LCD_PCD8544 = 0,    // 3310: 84x48 monochrome, GENSIO data/cmd. 3410 uses the same
                        // PCD8544-compatible controller/command set — only the geometry
                        // differs (102x72 RAM, 96x65 visible; see LcdSpec.ctrl_*).
    LCD_CANDYBAR,       // 8850/8210-class: ~96x65 (controller TBD)
    LCD_NAVI,           // 7110-class: 96x65 Navi (controller TBD)
} LcdController;

typedef struct {
    LcdController controller;
    // VISIBLE geometry — the rendered/cropped output the shells (web/gui/png) display.
    uint8_t  width;     // visible pixels wide  (3310 = 84; 3410 = 96)
    uint8_t  height;    // visible pixels tall  (3310 = 48; 3410 = 65)
    uint8_t  banks;     // ceil(visible height/8) (3310 = 6; 3410 = 9)
    uint8_t  io_data;   // MMIO offset for a data byte (3310 GENSIO = 0x2E)
    uint8_t  io_cmd;    // MMIO offset for a command byte (3310 GENSIO = 0x6E)
    // CONTROLLER-RAM geometry — the LCD chip's internal RAM, which is the framebuffer
    // stride the decode path writes into and the renderer reads with before cropping to
    // width x height. Differs from VISIBLE only when the controller RAM is larger than
    // the glass (3410: 102x72 RAM, 96x65 visible). 0 = "same as the visible field"
    // (3310/8850/7110: controller == visible, so these stay 0 and consumers fall back).
    uint8_t  ctrl_width;   // controller columns (3410 = 102; 0 -> use width)
    uint8_t  ctrl_height;  // controller rows    (3410 = 72;  0 -> use height)
    uint8_t  ctrl_banks;   // ceil(ctrl_height/8)(3410 = 9;   0 -> use banks)
    // Horizontal panel mirror: 1 = the LCD's column/segment order is REVERSED relative
    // to the 3310's PCD8544 (the glass is mounted/wired mirrored, so the firmware writes
    // DDRAM left-to-right but column 0 is physically the RIGHTMOST pixel). The decode
    // path maps logical column c -> physical (width-1-c) at write time so the stored fb
    // is correctly oriented for every consumer (web unpack, native ASCII, PNG). The
    // 5210 (NSM-5) needs this; 3310/3410/8850 etc. = 0 (normal order). (Dan, 2026-06).
    uint8_t  x_mirror;
} LcdSpec;

// --- Keypad -------------------------------------------------------------------
// Per-model keypad: the matrix LINES (which row/col a logical key drives) live in
// the model profile (HW truth, RE'd per build); the VISUAL layout + on-screen glyphs
// + host-keyboard bindings live in the GUI, selected by `family`. The GUI presses a
// key by asserting that model's (row,col) so the firmware's keymap table decodes it.

// Logical keys shared across DCT3 model families. Not every model has every key
// (e.g. only Family A/C have soft keys + send/end; only Family A has volume).
typedef enum {
    KK_NONE = 0,
    KK_0, KK_1, KK_2, KK_3, KK_4, KK_5, KK_6, KK_7, KK_8, KK_9,
    KK_STAR, KK_HASH,
    KK_UP, KK_DOWN,
    KK_SOFT1,    // left soft key  (Family B 3310 = "Menu")
    KK_SOFT2,    // right soft key (Family B 3310 = "Names"/C clear)
    KK_SEND,     // green / call
    KK_END,      // red / hang-up
    KK_VOLUP, KK_VOLDOWN,
    KK_PWR,      // power: special-scan, not the normal matrix
    // Scroll wheel (7110 Navi Roller). A detent = a momentary key tap, so the
    // KeyLine machinery absorbs it; no model maps these yet (7110 wiring = open
    // RE item, docs/hal-spec.md). APPEND-ONLY here: web/main.js hardcodes the
    // KK numbering — never renumber existing ids.
    KK_WHEEL_UP, KK_WHEEL_DOWN, KK_WHEEL_PRESS,
    KK_COUNT
} KeyId;

// One keypad button's HW mapping for a specific model. If `special` != 0 the key is
// driven via m->kbd_special_cols (mask = special) and (row,col) are ignored (PWR);
// otherwise it sets m->kbd_norm_cols[row] bit `col`.
typedef struct {
    uint8_t id;        // KeyId
    uint8_t row, col;  // normal-scan matrix position
    uint8_t special;   // !=0 -> special-scan mask instead of (row,col)
} KeyLine;

// Visual layout family (GUI rendering + which keys exist). Matrix LINES are still
// per-model — two Family-A models can share a layout but differ in (row,col).
typedef enum {
    KP_FAMILY_3310 = 0,  // B: Menu/Names(C), up/down, digits, * #  (no soft/send/end/vol)
    KP_FAMILY_8210 = 1,  // A: 2 soft keys, up/down, send/end, vol+/-, digits, * #
    KP_FAMILY_3410 = 2,  // C: 2 soft keys, up/down, send/end, digits, * #  (no vol)
} KeypadFamily;

// Matrix scan style — honest hardware capability, not a model flag. The scan
// differs in two composable ways (row isolation + whether 0xE0 is a dedicated
// power special-scan); the three real combinations on DCT3 are named here.
typedef enum {
    KP_SCAN_PLAIN = 0,    // 3310-class: 0xE0 = power special-scan; no row isolation
    KP_SCAN_DIR_AWARE,    // 8850-class: 0xE0 = power special-scan; row isolated by DIR
    KP_SCAN_SERIAL,       // early-MAD2 serial keypad (5110): row isolated; 0xE0 is an
                          // upper-row scan PHASE, not a power special-scan
} KeypadScan;

typedef struct {
    uint8_t      power_special_cols;  // m->kbd_special_cols at reset (3310 = 0x02, col 1)
    KeypadFamily family;              // GUI visual-layout selector
    const KeyLine* lines;             // per-model matrix (NULL = legacy/unmapped)
    int            n_lines;
    // --- Behaviour as composable capability DATA (docs/hal-spec.md) -----------
    // These describe what the keypad HARDWARE does — the model owns its behaviour
    // through them; the shared scan/dispatch never branch on model identity.
    KeypadScan     scan;              // matrix scan style (default PLAIN = 3310-class)
    uint8_t        uif_irq;           // 1 = the IRQ0 dispatcher source-decodes a matrix
                                      // interrupt-source register (I/O 0x2B read / 0x6B ack).
                                      // 0 = IRQ0 routes straight to the keypad ISR.
    uint8_t        has_slide;         // 1 = slide phone with a reed-switch cover sensor:
                                      // cover line (I/O 0x28 bit0) + cover interrupt source
                                      // (I/O 0x29 bit0, acked via IM_R 0x69).
    uint8_t        reg33_im_c;        // 1 = I/O 0x33 is the keypad interrupt-MASK register
                                      // (IM_C, RAM-backed); 0 = it is the keypad-LED line.
    long           hold_insns;        // harness key-hold floor (0 -> KEY_HOLD_INSNS default);
                                      // the serial keypad's debounce FSM needs a longer hold.
    // Matrix scan I/O ports — per-model addressing as DATA. col_port is a READ,
    // row/dir are WRITES, so they don't collide with the serial CCONT/GENSIO ports.
    uint8_t        col_port;          // COLUMN read    (3310-class 0x2A / serial 0x30; 0 -> 0x2A)
    uint8_t        row_port;          // ROW-select wr  (3310-class 0x28 / serial 0x31; 0 -> 0x28)
    uint8_t        dir_port;          // ROW-dir wr     (3310-class 0xA8 / serial 0x2F; 0 -> 0xA8)
} KeypadSpec;

// --- Battery / CCONT A/D reset defaults ---------------------------------------
typedef struct {
    uint16_t vbatt;     // adc[2] battery voltage  (3310 good = 0x2C0)
    uint16_t bsi;       // adc[3] battery type/BSI (3310 NiMH BMC-3 = 0x026)
    uint16_t temp;      // adc[4] battery temp     (3310 nominal = 0x200)
    uint16_t charger;   // adc[5] charger voltage  (none = 0x000)
} BatterySpec;

// --- Backlight LED colour (presentation data, period-correct per model) -------
// 0xRRGGBB; 0 = the classic Nokia yellow-green default (harness fallback). The
// on/off STATE stays m->led_lcd / m->led_kbd; this is only what colour the lit
// state renders as (5210 = orange, 8250 = blue, ...). docs/hal-spec.md.
typedef struct {
    uint32_t lcd_rgb;   // LCD backlight glow
    uint32_t kbd_rgb;   // keypad button backlight (0 = follow lcd_rgb)
    // The LCD-backlight ENABLE bit differs by ASIC. MAD2 drives it on McuGenIO I/O 0x20
    // bit3 (0x08). The early-serial 5110 MULTIPLEXES that same I/O 0x20 (PUP_GENIO): bits
    // 0/2 bit-bang the external-EEPROM I2C (SDA/SCL) and bit6 (0x40, MADos PUP_GENIO_LED)
    // is the EL backlight — off at the idle screen, lit on a keypress. (bit5/0x20 =
    // PUP_GENIO_DISP, the LCD-module power line, NOT the backlight.) 0 = the MAD2 default
    // (mask 0x08); set per-model to relocate the bit (5110 = 0x40, latched in ext_eeprom).
    uint8_t  lcd_mask;  // backlight bit mask within I/O 0x20 (0 -> MAD2 default 0x08)
    // Per-model output-device ABSENCE (the serial-keypad 5110 has neither). When set, the
    // GUI hides the corresponding indicator glyph so it can't show a dead/always-off tile.
    uint8_t  no_kbd_led; // 1 = model has no keypad LED  (5110: kpd_led=0 in MADos)
    uint8_t  no_vibra;   // 1 = model has no vibra motor (5110 NSE-1: no internal vibra)
} LedSpec;

// --- GENSIO / serial-bus transport ports (per-model addressing) ---------------
// The CCONT chip is reached over the GENSIO bus, but at DIFFERENT I/O ports on
// MAD2 (memory-mapped GENSIO) vs serial-bus (bit-banged serial). The chip model
// (mad2_ccont.c) is the same silicon; only WHERE it's addressed varies, so the
// ports are profile DATA (docs/hal-spec.md: addressing as data, principle 3).
// 0 = the MAD2 default (IO_GENSIO_* in mad2_internal.h). Collisions across
// models are resolved by ACCESS DIRECTION: e.g. 0x2D is the MAD2 START (write)
// but the serial-bus ccont_r (read) — ccont_r is matched on reads, start on writes.
// (The GENSIO STATUS port stays in the dispatcher: its read VALUE differs per
// model — serial-bus ORs the ready bits into RAM, MAD2 returns a flat 0x07 — so it is
// not pure routing.)
typedef struct {
    uint8_t ccont_w;   // CCONT write (reg-select / value).  MAD2 0x2C / serial-bus 0x2A
    uint8_t ccont_r;   // CCONT readback.                    MAD2 0x6C / serial-bus 0x2D
    uint8_t start;     // begin-transaction (reset addr).    MAD2 0x2D / serial-bus 0 (self-framed)
} GensioSpec;

// --- ASIC variant config (per-model chip values, not behavioural flags) -------
typedef struct {
    uint8_t irq_sources;   // CTSI IRQ width: 3310 = 8; 8850 noted as 16
    uint8_t asic_version;  // MAD2 ASIC version byte read at I/O 0x20000 (offset 0x00).
                           // 3310 v5.79 = 0xA1 (real-HW confirmed). Firmware copies it to
                           // the DSP shared-mem ASIC-ID slot 0x101E2 (0x2BB6DE) + NetMonitor.
                           // 0 = unset -> read falls through to RAM (legacy behaviour).
    uint8_t dsp_reset_running;  // I/O 0x20002 (DSP reset ctrl) read-back once the firmware
                           // has released the DSP (sets dsp_release_mask). Real HW brings the
                           // DSP up and asserts the clock/ready/API status bits -> 3310 reads
                           // 0x53 after a fresh boot. We only RAM-back the release bit, so model
                           // the HW status bits here. 0 = unset -> legacy RAM-back.
    uint8_t dsp_release_mask;   // which [0x20002] bit the firmware sets to release the DSP,
                           // gating the dsp_reset_running status read-back. 3310 releases via
                           // bit0 (0x2BAE22); the 3410 releases via bit2 then polls bit4 for
                           // ready. 0 = default 0x01 (bit0) so existing models are unchanged.
} AsicConfig;

// --- Boot / HLE knobs ---------------------------------------------------------
typedef struct {
    uint8_t skip_seclock_default;  // FAID "pass" default (3310 web = 1)
    uint8_t pin_verdict_default;   // pin self-test verdict at 0xC8 (default 0)
} BootConfig;

// --- Firmware identification (autodetect) -------------------------------------
typedef struct {
    const char* match;     // substring to find in the dump (product code / version)
    const char* match2;    // optional 2nd accepted code (OR'd with match); NULL = unused.
                           // For a profile that serves more than one product code at the
                           // same size — e.g. the 3330 (NHM-6) also covers cooked 4 MB
                           // images that carry the 3310's NHM-5 code.
    uint32_t    flash_size;// expected dump/flash size (0 = don't care)
} FwIdent;

// --- Per-model DSP behaviour --------------------------------------------------
// The DSP boot/runtime handshake (HPI mailbox protocol, code-block upload, IRQ4
// generation, message injection) differs per model, and editing the shared mad2
// code risks the byte-identical 3310 boot. So the DSP behaviour is a per-model
// vtable: mad2_read/mad2_write/mad2_tick dispatch to the profile's `dsp` ops.
// `mad2_dsp_default` carries the current (3310/legacy) behaviour; a model with a
// genuinely different DSP boot supplies its own ops (e.g. src/models/8850/dsp.c).
struct Mad2;   // opaque here; the ops cast it back (defined in src/mad2/mad2.h)
typedef struct DspOps {
    const char* name;
    // DSP-region READ: set *out and return 1 if handled, else 0 (fall through to
    // the normal mad2 read path). Called for every non-flash read.
    int  (*read)(struct Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out);
    // DSP-region WRITE: return 1 if handled (stop further mad2_write processing),
    // else 0. The core already RAM-backs non-MMIO writes, so this is a side-effect hook.
    int  (*write)(struct Mad2* m, uint32_t addr, int size, uint32_t value);
    // Per-step DSP pump: mailbox acks, IRQ4 generation, self-test/boot-msg injection.
    void (*tick)(struct Mad2* m);
    // Optional HLE tone report: for backends that do NOT emit real codec PCM, report the
    // tone the MCU has commanded via the COBBA HPI registers so emu_audio can synthesize it
    // into the shared PCM stream. Return 1 and fill *f1_hz (osc1 / plain beep) + *f2_hz (osc2
    // / DTMF pair, 0 if none) when a tone is active, else 0. NULL for the cosim backend — the
    // real C54x plays the tone itself through the codec (DXR tap), so the mixer must NOT.
    int  (*hle_tone)(struct Mad2* m, int* f1_hz, int* f2_hz);
} DspOps;
// Shared HLE tone reader (defined in dsp_default.c): reads the model-invariant COBBA tone
// registers (DCT3_TONE_*). Every HLE DSP backend points .hle_tone at this; the cosim leaves
// it NULL. Kept in the DSP layer (not the mixer) because tone generation is a DSP function.
int dsp_hle_tone(struct Mad2* m, int* f1_hz, int* f2_hz);
extern const DspOps mad2_dsp_default;   // ROM-6 (3310/33xx/34xx/82xx/8850) — shared legacy behaviour
// HLE DSP responders are organised BY Nokia DSP ROM revision (Dan, 2026-06-16): ROM 4 = 5110/6110
// (src/mad2/dsp_rom4.c, cosim-grounded), ROM 6 = mad2_dsp_default above. Distinct TUs so ROM-4
// fidelity work can't regress the byte-identical 3310 (ROM-6) boot. See
// docs/research/5110-cosim-vs-hle-conversation.md.
extern const DspOps mad2_dsp_rom4;      // ROM-4 (5110/6110/3210) responder — shared
// ROM-4 but a SEPARATE responder for the 7110 (NSE-5): same revision, but its own DSP self-test
// conversation (cmd-0x70 sub-0x04/0x06/0x0E via the PORT1 API path) — kept distinct so the 7110
// reply model never entangles the shared 5110/6110/3210 path. src/models/7110/dsp_7110.c.
extern const DspOps mad2_dsp_7110;      // ROM-4 (7110) responder — 7110-specific self-test
// Real TMS320C54x co-sim backend (third_party/c54x/mad2_dsp_c54x.c). NATIVE-ONLY: the
// 635 KB C54x interpreter is excluded from the wasm build, so the symbol exists only in
// the native link. Reference it under #ifndef __EMSCRIPTEN__ (the wasm build keeps the
// default backend). See docs/dsp-5110-mad1.md §7e + third_party/c54x/.
#ifndef __EMSCRIPTEN__
extern const DspOps mad2_dsp_c54x;
#endif

// --- Per-model bus transport --------------------------------------------------
// It is all MAD2 — but the EARLY MAD2 models (5110/6110/3210/…) hang their CCONT
// and an external 24C16 EEPROM off a BIT-BANGED SERIAL bus instead of the later
// memory-mapped GENSIO + in-flash EEPROM. That is a genuine BEHAVIOUR difference
// (transport), so it is a model implementation, NOT a flag the shared dispatcher
// branches on (docs/hal-spec.md). A model with `bus` set gets first crack at an
// MMIO access: read sets *out + returns 1 when it owns the port; write returns 1
// when handled. `bus == NULL` = the default memory-mapped MAD2 path (most models).
typedef struct BusOps {
    const char* name;
    int (*read)(struct Mad2* m, uint32_t addr, int size, uint32_t ram_value, uint32_t* out);
    int (*write)(struct Mad2* m, uint32_t addr, int size, uint32_t value);
} BusOps;
extern const BusOps mad2_bus_serial;   // early-MAD2 serial-attached EEPROM + bus status

// --- The profile --------------------------------------------------------------
typedef struct ModelProfile {
    const char*    name;          // "3310", "8850", "7110"
    const char*    description;
    MemMap         mem;
    LcdSpec        lcd;
    KeypadSpec     keypad;
    BatterySpec    battery;
    LedSpec        led;           // backlight colours (0 = classic yellow-green)
    GensioSpec     gensio;        // CCONT bus transport ports (0 = MAD2 default)
    AsicConfig     asic;          // per-model ASIC values (version, DSP-reset readback, IRQ width)
    FwAddrs        fw;            // constant fallbacks for every firmware address
    const SigResolve* sigs;       // signature overrides (signature-first)
    int            n_sigs;
    // Optional SECOND sig table, layered on top of `sigs` (model_resolve walks both,
    // sharing first-hit-wins). For per-build addresses whose code SHAPE recurs across the
    // DCT3 family but whose VALUE must NOT be force-resolved on every model (e.g. the
    // 5210's verdict / dsp_uploaded writers — the dsp_uploaded shape also matches 8210/
    // 8850/8855 but at a DIFFERENT, faithfully-hardcoded site there). Scoping these to one
    // model's sigs2 keeps them out of the shared MAD2_SIGS. NULL/0 = unused (every other
    // model). See src/models/mad2_sigs.c MAD2_SIGS_5210.
    const SigResolve* sigs2;
    int            n_sigs2;
    BootConfig     boot;
    FwIdent        ident;
    const DspOps*  dsp;           // per-model DSP behaviour (NULL = no DSP hooks)
    const BusOps*  bus;           // per-model bus transport (NULL = memory-mapped MAD2)
    // External I2C-EEPROM (24Cxx) baked default for early-MAD2 serial-bus models (5110/6110/…):
    // the NokiX virgin blob, auto-selected per model from src/mad2/ext_eeprom_blobs.h. NULL = the
    // model has no external I2C EEPROM (in-flash-EEPROM models, e.g. 3310). ext_eeprom.c loads this
    // (or an EE5110 file override) into m->i2c_eeprom; the web build relies on it (no MEMFS file).
    const uint8_t* i2c_eeprom_default;
    uint32_t       i2c_eeprom_size;
    // DSP HPI bulk code-upload window alias (C54x co-sim only; NATIVE). Most DCT3 map the
    // DSP DARAM upload window at the SAME MCU base as the control/mailbox cells (0x10000),
    // so the firmware writes uploaded DSP code into 0x10000-0x10FFF (DSP word = 0x800 + k).
    // The 3210 (NSE-8) is the oddball: its control/mailbox cells stay at 0x100xx, but its
    // BULK code-upload window is aliased 0x1000 higher — the firmware writes loader1/blocks
    // to 0x11xxx (measured: loader1 lands at MCU 0x11E00, not 0x10E00; same DSP word 0xF00).
    // The cosim's window logic is anchored at 0x10000, so set this to the alias base (0x11000
    // for the 3210) and the backend normalises a [base, base+0x1000) access down to the
    // 0x10000 window before mapping it to the DSP. 0 = no alias (every other model — the
    // upload window already coincides with the control window, so behaviour is unchanged).
    uint32_t       dsp_hpi_alias_base;
    // Early-MAD serial-bus external-EEPROM I2C bit-bang pin positions within I/O 0x20
    // (PUP_GENIO). The MAD1 default (5110/6110) is SDA=bit0, SCL=bit2. The 3210 (oddball)
    // keeps SDA=bit0 but drives SCL on bit3 (RE'd: its bit-bang routine 0x2B0332 sets bit3
    // as the clock; bit2 is held high there, so a fixed bit2-SCL reader sees a constant
    // "clock high" and never advances the I2C FSM → the EEPROM never reads). 0 values =
    // the MAD1 default (sda_bit 0 = bit0; scl_bit 0 = bit2). ext_eeprom.c reads these.
    uint8_t        i2c_sda_bit;   // SDA bit in I/O 0x20 (0 = bit0)
    uint8_t        i2c_scl_bit;   // SCL bit in I/O 0x20 (0 = MAD1 default bit2)
    // CCONT power-on cause (reg 0x0E) latched at cold power-on. The DCT3 power button is
    // wired to the CCONT (PWRONX), not (only) the keypad matrix: at startup the firmware
    // reads CCONT reg 0x0E and, if no power-on-cause bit is set, treats the boot as having
    // no valid power-on reason and powers straight back off. Most models (3310/5110/...)
    // OR the keypad power-key into their reason chain, so they don't need this. The 3210
    // (oddball) reads ONLY CCONT reg 0x0E in its power-on classifier (0x2AF0AE): bit1 =
    // "powered on by PWR key" (→ normal boot). A cold boot = the user holding PWR, so the
    // latch is set at reset. 0 = no CCONT power-on cause (default; keypad-driven models).
    uint8_t        ccont_poweron_int;  // CCONT reg 0x0E power-on-cause bit(s); 0 = none
    // CCONT A/D channel MUX routing. The firmware selects a CCONT A/D channel N (reg-0
    // bits[6:4]) and the model returns m->adc[N]. The MAD2/3310 standard wiring is
    // VBATT=ch2, BSI=ch3, BTEMP=ch4, VCHAR=ch5 — and m->adc[] is indexed by that
    // convention (adc[2]=vbatt, ...). A few oddball models wire the divider chain to
    // DIFFERENT physical channels: the 3210 (NSE-8) routes VBATT to channel 0 (RE'd:
    // its boot battery reader 0x2A84B0 reads ch0 and gates it to the VBATT window
    // [0x2BE..0x314] that brackets the modelled 0x2C0; the per-quantity channel-map
    // table at flash 0x2E2D74 = {00 04 05 06 07 03 02 01} confirms index0->ch0). This
    // is the same oddball-shifted-I/O signature as its DSP HPI alias / I2C SCL bit /
    // CCONT power button. adc_route[N] != 0 remaps a firmware-selected channel N to the
    // m->adc[] index that holds its quantity; 0 = identity (adc[N]). An all-zero array =
    // the standard wiring (every other model). ccont_read consults it. (3210: route ch0
    // -> adc[2] so the VBATT divider on ch0 reads the modelled battery voltage.)
    uint8_t        adc_route[8];   // CCONT A/D channel -> m->adc[] index (0 = identity)
} ModelProfile;

// Registry / selection.
const ModelProfile* model_default(void);                  // 3310
const ModelProfile* model_by_name(const char* name);      // NULL if unknown
const ModelProfile* model_detect(const uint8_t* flash, uint32_t len);  // autodetect, NULL if none
int                 model_count(void);
// Capability query: does this model have logical key `id`? (present in lines[];
// KK_PWR counts via special-scan). Harnesses render/accept only what exists.
int                 model_key_present(const ModelProfile* prof, int id);
const ModelProfile* model_at(int i);

// Resolve firmware addresses against the loaded flash (in the core RAM buffer).
// Copies prof->fw into *out, then applies each signature hit. `ram` is the core RAM
// (flash mapped at prof->mem.flash_base); `ram_mask` is DCT3_RAM_MASK.
void model_resolve(const ModelProfile* prof, const uint8_t* ram, uint32_t ram_mask, FwAddrs* out);

#endif // DCT3_MODEL_H
