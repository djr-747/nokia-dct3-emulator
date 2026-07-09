# DCT3 Emulator — top-level build.
#
#   make             build the WASM module (web/dct3.js + web/dct3.wasm)
#   make test        build & run the native ARM core test suite (gcc)
#   make check-core  verify the vendored mGBA ARM core compiles for wasm32
#   make serve       build, then serve web/ on :8000
#   make clean       remove build artifacts
#
# WASM build needs emscripten (emcc); native test needs a C compiler (cc).

EMCC ?= emcc
CC   ?= cc

CORE_INC := -Ithird_party/mgba-arm/include
APP_INC  := -Isrc $(CORE_INC)
DEFS     := -DENABLE_DEBUGGERS          # compile in the vendored disassembler

# --- Real TMS320C54x DSP co-sim backend (NATIVE-ONLY) ---
# The lifted bbaranoff C54x interpreter + Nokia HPI/port bridge + the mad2 DSP backend.
# Kept under third_party/ (NOT src/) so the wasm APP_SRCS glob never picks it up — the
# 635 KB interpreter would bloat the wasm core. Every NATIVE target that links the model
# profiles (the 5110 profile references mad2_dsp_c54x under #ifndef __EMSCRIPTEN__) must
# also link these objects. See third_party/c54x/.
C54X_DIR  := third_party/c54x
C54X_SRCS := $(C54X_DIR)/calypso_c54x.c $(C54X_DIR)/stubs.c $(C54X_DIR)/dct3_dsp54.c $(C54X_DIR)/mad2_dsp_c54x.c
C54X_INC  := -I$(C54X_DIR) -I$(C54X_DIR)/inc

# --- Sources ---
# Object files are emitted UNDER $(OBJDIR) mirroring the source tree (e.g.
# src/mad2/mad2.c -> build/obj/src/mad2/mad2.o), never next to the .c. Keeps the
# source tree clean and confines all build output to build/ (one `rm -rf build`).
OBJDIR    := build/obj
APP_SRCS  := $(shell find src -name '*.c' 2>/dev/null)
APP_OBJS  := $(patsubst %.c,$(OBJDIR)/%.o,$(APP_SRCS))
APP_HDRS  := $(shell find src -name '*.h' 2>/dev/null)
CORE_SRCS := $(shell find third_party/mgba-arm/src -name '*.c' 2>/dev/null)
CORE_OBJS := $(patsubst %.c,$(OBJDIR)/%.o,$(CORE_SRCS))

# The header-dependency rule below is the FIRST explicit rule in the file, which
# would silently become make's default goal ("make" = build one .o, not the wasm
# the header comment promises). Pin the documented default.
.DEFAULT_GOAL := all

# Header dependency: the per-file %.o rules below don't auto-track .h includes, so a
# struct change in a shared header (mad2.h / model.h) would otherwise leave STALE .o
# objects compiled against the old layout — linking a broken wasm where translation
# units disagree on struct offsets (→ blank boot). Make every app object depend on
# every app header so any header edit forces a rebuild. (Caught by `make webtest`.)
$(APP_OBJS): $(APP_HDRS)

OUT_DIR := web
OUT_JS  := $(OUT_DIR)/dct3.js

CFLAGS      := -std=c99 -O2 -flto -Wall -Wextra $(APP_INC) $(DEFS)
CORE_CFLAGS := -std=c99 -O2 -flto -Wall $(CORE_INC) $(DEFS)   # -O2 (was -Os): faster ARM core

# C functions exposed to JS: boot + run loop + framebuffer + keypad.
EXPORTS := _dct3_version,_dct3_web_boot,_dct3_web_run,_dct3_web_run_cycles,_dct3_web_cycles,_dct3_web_fb,_dct3_web_step,_dct3_web_powered_off,_dct3_web_reset_request,_dct3_web_reset_last_reason,_dct3_web_reset_total,_dct3_web_reset_recovered,_dct3_web_reset_count,_dct3_web_reset_last_pc,_dct3_web_reset_last_cpsr,_dct3_web_warm_reset,_dct3_web_reg,_dct3_web_lcd_writes,_dct3_web_lcd_mode,_dct3_web_leds,_dct3_web_led_rgb,_dct3_web_buzzer_on,_dct3_web_buzzer_div,_dct3_web_buzzer_vol,_dct3_web_buzzer_chirp,_dct3_web_pcm_read,_dct3_web_pcm_ptr,_dct3_web_pcm_rate,_dct3_web_tone_hz,_dct3_web_tone_hz2,_dct3_web_tone_amp,_dct3_web_vibra_on,_dct3_web_set_battery,_dct3_web_get_battery,_dct3_web_set_charger,_dct3_web_get_charger,_dct3_web_set_sim,_dct3_web_get_sim,_dct3_web_sim_apdus,_dct3_web_set_sim_pin_enabled,_dct3_web_set_sim_pin,_dct3_web_set_sim_puk,_dct3_web_sim_pin_state,_dct3_web_ccont_mask,_dct3_web_ccont_lines,_dct3_web_rtc_now,_dct3_web_rtc_min_edges,_dct3_web_rtc_writes,_dct3_web_rtc_raw,_dct3_web_rtc_wr_pc,_dct3_web_rtc_wr_lr,_dct3_web_getstr_on,_dct3_web_getstr_dump,_dct3_web_key,_dct3_web_key_logical,_dct3_web_key_logical_raw,_dct3_web_power,_dct3_web_set_bypass,_dct3_web_set_skip_seclock,_dct3_web_set_spike,_dct3_web_get_spike,_dct3_web_spike_info,_dct3_web_pc,_dct3_web_t0_ticks,_dct3_web_t1_edges,_dct3_web_fiq8_ticks,_dct3_web_t0_counter,_dct3_web_t1_counter,_dct3_web_fiqs,_dct3_web_irqs,_dct3_web_ram,_dct3_web_kbd,_dct3_web_key_raw,_dct3_web_dbg_watch,_dct3_web_dbg_count,_dct3_web_set_key_hold,_dct3_web_get_key_hold,_dct3_web_msglog_pc,_dct3_web_msglog_buf,_dct3_web_msglog_size,_dct3_web_msglog_w,_dct3_web_ram_ptr,_dct3_web_msglog_lrbuf,_dct3_web_set_oneshot,_dct3_web_stub_add,_dct3_web_stub_clear,_dct3_web_stub_hits,_dct3_web_wrwatch_on,_dct3_web_wrwatch_npc,_dct3_web_wrwatch_pc,_dct3_web_wrwatch_net,_dct3_web_wrwatch_cnt,_dct3_web_wrwatch_lr,_dct3_web_wrwatch_szn,_dct3_web_wrwatch_sz,_dct3_web_wrwatch_sza,_dct3_web_wrwatch_szf,_dct3_web_acall_window,_dct3_web_acall_n,_dct3_web_acall_lr,_dct3_web_acall_cnt,_dct3_web_acall_egsz,_dct3_web_call,_dct3_web_call_count,_dct3_web_call_result,_dct3_web_seccode_reset,_dct3_web_regspike,_dct3_web_regspike_count,_dct3_web_cap_set,_dct3_web_cap_val,_dct3_web_cap_hits,_dct3_web_ccw,_dct3_web_difftrace,_dct3_web_sendlog_on,_dct3_web_sendlog_w,_dct3_web_sendlog_at,_dct3_web_sendlog_lr,_dct3_web_eeprom_off,_dct3_web_eeprom_size,_dct3_web_eeprom_writes,_dct3_web_i2c_eeprom_ptr,_dct3_web_i2c_eeprom_size,_dct3_web_i2c_eeprom_writes,_dct3_web_flash_cmds,_dct3_web_flash_programs,_dct3_web_flash_last_addr,_dct3_web_model,_dct3_web_flash_hi,_dct3_web_lcd_w,_dct3_web_lcd_h,_dct3_web_lcd_banks,_dct3_web_kp_family,_dct3_web_ccwr_w,_dct3_web_ccwr_lr,_dct3_web_ccwr_reg,_dct3_web_ccwr_val,_dct3_web_pcring_crashed,_dct3_web_pcring_w,_dct3_web_pcring_n,_dct3_web_pcring_at,_dct3_web_pcring_cpsr,_dct3_web_set_recover,_dct3_web_get_recover,_dct3_web_set_wdt_service,_dct3_web_get_wdt_service,_dct3_web_wdt_inhibited,_dct3_web_set_reboot_early,_dct3_web_get_reboot_early,_dct3_web_postmortem_buf,_dct3_web_postmortem_len,_dct3_web_assert_count,_dct3_web_heap_fail_count,_dct3_web_heap_fail_lr,_dct3_web_leak_on,_dct3_web_heapcurve_on,_dct3_web_heap_used,_dct3_web_leak_count,_dct3_web_leak_dump,_dct3_web_leak_buf,_dct3_web_leak_len,_dct3_web_heapcurve_count,_dct3_web_heapcurve_step,_dct3_web_heapcurve_used,_dct3_web_trace_on,_dct3_web_trace_count,_dct3_web_trace_cyc,_dct3_web_trace_step,_dct3_web_trace_pc,_dct3_web_trace_cpsr,_dct3_web_trace_fiq,_dct3_web_trace_irq,_dct3_web_branch_on,_dct3_web_branch_arm,_dct3_web_branch_count,_dct3_web_branch_pc,_dct3_web_branch_target,_dct3_web_branch_lr,_dct3_web_branch_sp,_dct3_web_branch_step,_dct3_web_branch_type,_dct3_web_branch_depth,_dct3_web_branch_cpsr_mode,_dct3_web_branch_r0,_dct3_web_branch_r1,_dct3_web_branch_r4,_dct3_web_branch_r9,_dct3_web_irq_mask,_dct3_web_fiq_mask,_dct3_web_irq_pending,_dct3_web_fiq_pending

# Flash image preloaded into MEMFS at /fw.fls (fetched at runtime as web/dct3.data).
# Factory-reset v5.79: clean defaults, no carrier customisations, neutral
# starting state for first-time visitors. Override on the command line if
# you want a different baseline: `make web WEB_FW="flash/My 3310 NR1 v5.79.fls"`.
WEB_FW ?= firmware/Factory Reset 3310 NR1 v5.79.fls

LDFLAGS := -O2 -flto --no-entry \
           -sWASM=1 -sMODULARIZE=1 -sEXPORT_NAME=DCT3Module \
           -sALLOW_MEMORY_GROWTH=1 \
           -sEXPORTED_FUNCTIONS=$(EXPORTS) \
           -sEXPORTED_RUNTIME_METHODS=ccall,cwrap,HEAPU8,HEAP16,HEAPU32,FS

.PHONY: all serve clean check-core test webtest trace gui gui-release deb disfw simprobe fw-manifest
MANIFEST := $(OUT_DIR)/asset-manifest.json

all: $(MANIFEST)

# Bring-your-own firmware: preload it into MEMFS at /fw.fls only if present.
# No firmware committed to this repo — without one, the module builds firmware-free
# and the page loads a flash image the user supplies (drop it in as web/dct3.data,
# or via the UI). See README "Firmware — bring your own".
$(OUT_JS): $(APP_OBJS) $(CORE_OBJS)
	@if [ -f "$(WEB_FW)" ]; then \
	  echo "[web] preloading firmware: $(WEB_FW)"; \
	  $(EMCC) $(LDFLAGS) --preload-file "$(WEB_FW)@/fw.fls" $(APP_OBJS) $(CORE_OBJS) -o $(OUT_JS); \
	else \
	  echo "[web] no firmware at '$(WEB_FW)' — building firmware-free module (supply web/dct3.data yourself)"; \
	  $(EMCC) $(LDFLAGS) $(APP_OBJS) $(CORE_OBJS) -o $(OUT_JS); \
	fi

# Content-address the immutable web assets (hashed filenames + no-store manifest)
# so the browser can never serve a stale core. Re-runs when the wasm, the UI
# harness, or the loader change. See tools/web_fingerprint.mjs.
$(MANIFEST): $(OUT_JS) $(OUT_DIR)/main.js $(OUT_DIR)/index.html
	@node tools/web_fingerprint.mjs

$(OBJDIR)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(EMCC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/third_party/mgba-arm/src/%.o: third_party/mgba-arm/src/%.c
	@mkdir -p $(dir $@)
	$(EMCC) $(CORE_CFLAGS) -c $< -o $@

# Standalone compile check for the vendored core (no link).
check-core: $(CORE_OBJS)
	@echo "mGBA ARM core compiles for wasm32 ($(words $(CORE_OBJS)) objects)."

# --- Native test suite: one self-contained gcc build, no shared objects ---
TEST_BIN    := build/dct3_test
TEST_SRCS   := tests/test_core.c src/core/dct3_core.c src/core/dct3_debug_stub.c $(CORE_SRCS)
TEST_CFLAGS := -std=gnu99 -O1 -g -Wall $(APP_INC) -Itools $(DEFS)   # -Itools: mad2.c SIM-bridge include (native only)
# Optimized native build for boot_trace/gui: the co-sim is an interpreter in a
# tight loop (ARM core + 4x C54x per step) — -O1 -g left ~2-3x on the table.
# -O2 keeps -g line info. make test guards correctness; default boot stays
# byte-identical (integer interpreters are opt-level-stable). 2026-06-09.
# -flto: the hot loop is dominated by cheap cross-TU helpers
# (dsp54_hpi_read/peek per cosim tick, the harness hooks) that the per-file build
# could not inline; LTO inlines them and ~doubles the 5110 cosim rate (0.44->0.75x
# realtime), lifting non-cosim models well past realtime. -O3/-march=native were
# tried and REGRESSED (interpreter I-cache pressure) — keep -O2. Integer-stable:
# `make guard` stays byte-identical.
TRACE_CFLAGS := -std=gnu99 -O2 -flto -g -Wall $(APP_INC) -Itools $(DEFS)
TEST_WR_BIN := build/dct3_test_wr
TEST_MAD2_BIN := build/dct3_test_mad2

test: $(TEST_BIN) $(TEST_WR_BIN) $(TEST_MAD2_BIN) $(DBGCON_BIN)
	@./$(TEST_BIN)
	@./$(TEST_WR_BIN)
	@./$(TEST_MAD2_BIN)
	@./$(DBGCON_BIN)

# dbgcon client-lib selftest: the freestanding tools/dbgcon/dbgcon.h driven against the
# REAL emulator port (mad2 device model, no ARM core), proving the emu-vs-real gate.
DBGCON_BIN  := build/dct3_dbgcon_selftest
DBGCON_SRCS := tools/dbgcon/dbgcon_selftest.c tools/dbgcon/dbgcon_stripcheck.c $(shell find src/mad2 src/models -name '*.c' 2>/dev/null) tools/sim_bridge.c $(C54X_SRCS)
$(DBGCON_BIN): $(DBGCON_SRCS) $(APP_HDRS)
	@mkdir -p build
	$(CC) $(TEST_CFLAGS) $(C54X_INC) -Itools $(DBGCON_SRCS) -o $@
dbgcon-selftest: $(DBGCON_BIN)
	@./$(DBGCON_BIN)

# Regression gate for refactor work: three byte-identical canonical boots
# (3310 250M, 3410 250M, 5110 cosim 30M) + the unit suites. ~2-3 min. Pinned
# hashes + recipe in tools/check_guardrails.sh; GATES="3310" to run a subset.
guard: build/dct3_boot_trace $(TEST_BIN) $(TEST_WR_BIN) $(TEST_MAD2_BIN)
	@tools/check_guardrails.sh

# Web boot regression test: build the wasm, then boot it headlessly (cycle-paced, like
# the browser) under node and assert the 3310 reaches a LIVE boot — catches a blank/stuck
# boot from a stale wasm. Needs emcc (build) + node. See tools/web_boot_test.mjs.
webtest: $(OUT_JS)
	@node tools/web_boot_test.mjs

$(TEST_BIN): $(TEST_SRCS)
	@mkdir -p build
	$(CC) $(TEST_CFLAGS) $(TEST_SRCS) -o $@

# Phase 9 Wave-0 instrument-regression checks (WR-01/WR-02/WR-04). The test TU
# #include's src/harness/{heap_shadow,telemetry}.c directly (to read the instrument
# file-statics), so only the test source compiles here — the instruments reference
# Mad2/ARMCore struct fields only (no mad2/core functions to link). TEST_WR_BIN is
# declared up by TEST_BIN so the `test` target can list it as a prerequisite.
TEST_WR_SRCS := tests/test_harness_wr.c
$(TEST_WR_BIN): $(TEST_WR_SRCS) $(HARNESS_SRCS) src/mad2/mad2.h
	@mkdir -p build
	$(CC) $(TEST_CFLAGS) $(TEST_WR_SRCS) -o $@

# MAD2 device-model unit tests. Links the per-subsystem mad2 TUs (the mad2.c split
# made these directly drivable) + the model profiles + the SIM-bridge stub, but NOT
# the mGBA core: the tests poke device registers directly, no firmware execution.
# Self-contained glob (this rule precedes the MAD2_SRCS/MODEL_SRCS definitions, so
# re-glob here rather than reference them).
TEST_MAD2_SRCS := tests/test_mad2.c $(shell find src/mad2 src/models -name '*.c' 2>/dev/null) tools/sim_bridge.c $(C54X_SRCS)
$(TEST_MAD2_BIN): $(TEST_MAD2_SRCS) $(APP_HDRS)
	@mkdir -p build
	$(CC) $(TEST_CFLAGS) $(C54X_INC) $(TEST_MAD2_SRCS) -o $@

# --- Phase 2 boot-trace harness (native) ---
# Override the image with:  make trace FW="flash/Nokia 3310 ....bin"
# Model profiles + registry/resolver (the web build picks these up via the src glob).
MODEL_SRCS := $(shell find src/models -name '*.c' 2>/dev/null)
# Shared harness layer (Phase 8 H0): fault-detect + reset-recovery orchestration,
# consumed by BOTH the native drivers (explicit list below) and the wasm build (the
# APP_SRCS glob at the top auto-picks src/harness — Pitfall 1: native lists are
# EXPLICIT, the wasm build globs; new files must be added to the explicit lists).
HARNESS_SRCS := $(shell find src/harness -name '*.c' 2>/dev/null)
# Device model — globbed so the per-subsystem split (mad2_<sub>.c) is auto-picked
# by the native drivers, exactly like the wasm APP_SRCS glob already does.
MAD2_SRCS := $(shell find src/mad2 -name '*.c' 2>/dev/null)

TRACE_BIN  := build/dct3_boot_trace
TRACE_SRCS := tools/boot_trace.c src/core/dct3_core.c src/core/dct3_debug_stub.c $(MAD2_SRCS) tools/sim_bridge.c tools/mbus_bridge.c $(MODEL_SRCS) $(HARNESS_SRCS) $(CORE_SRCS) $(C54X_SRCS)
FW         ?=

trace: $(TRACE_BIN)
	@./$(TRACE_BIN) $(if $(FW),"$(FW)")

$(TRACE_BIN): $(TRACE_SRCS)
	@mkdir -p build
	$(CC) $(TRACE_CFLAGS) $(C54X_INC) $(TRACE_SRCS) -o $@

# --- Native SDL2 GUI overlay (compile-gated) ------------------------------------
# SAME source set as the headless boot_trace PLUS tools/gui_sdl.c, compiled with
# -DDCT3_SDL so the gui.h overlay is live (the headless target above is built WITHOUT
# -DDCT3_SDL and links no SDL — it must still build on an SDL-less box). All SDL code
# lives in tools/gui_sdl.c; the shared core/mad2/harness/models are reused verbatim.
GUI_BIN     := build/dct3_boot_trace_gui
GUI_SRCS    := $(TRACE_SRCS) tools/gui_sdl.c
SDL_CFLAGS  := $(shell sdl2-config --cflags)
SDL_LIBS    := $(shell sdl2-config --libs)
gui: $(GUI_BIN)
$(GUI_BIN): $(GUI_SRCS)
	@mkdir -p build
	$(CC) $(TRACE_CFLAGS) $(C54X_INC) -DDCT3_SDL $(SDL_CFLAGS) $(GUI_SRCS) $(SDL_LIBS) -o $@

# --- Beta/release GUI: lean 3310-only, no co-sim, debug knobs compiled out --------
# For sharing with a beta tester (+ `make deb`). Differences vs
# the dev `gui` target above:
#   -DDCT3_MODEL_3310_ONLY  registry compiles down to the 3310 (src/models/model.c),
#                           so NO other profile links → NO C54x co-sim linked at all
#                           (only the serial-bus profiles pull it in). 3310 = HLE DSP.
#   -DDCT3_RELEASE          boot_trace.c compiles out its ~120 debug/trace/inject/
#                           bridge env knobs (DBG_ENV macro); keeps the essential boot
#                           + GUI knobs (audio, PWR-hold warm-reset, screenshot, keylog).
# The bridge TUs (sim_bridge/mbus_bridge) stay linked but dormant — their activation
# knobs are gated out, and they carry no c54x dependency. Same shared core/mad2/harness.
GUIREL_BIN   := build/dct3-emu
GUIREL_MODELS := src/models/model.c src/models/3310/profile.c src/models/mad2_sigs.c
GUIREL_SRCS  := tools/boot_trace.c tools/gui_sdl.c src/core/dct3_core.c src/core/dct3_debug_stub.c \
                $(MAD2_SRCS) tools/sim_bridge.c tools/mbus_bridge.c $(GUIREL_MODELS) \
                $(HARNESS_SRCS) $(CORE_SRCS)
# DCT3_ATTRIB → appended to the SDL window title (attribution on the shared build).
# GUIREL_SCRUB → -g0 drops debug info and -ffile-prefix-map rewrites the source paths
# baked into __FILE__ strings (assert/error text), so the shared binary carries no
# /home/<user>/... build path. Override the attribution:  make gui-release DCT3_ATTRIB='...'
DCT3_ATTRIB   ?= By Dan Richardson <djrichardson747@gmail.com>
GUIREL_SCRUB  := -g0 -ffile-prefix-map=$(CURDIR)=dct3-emu -ffile-prefix-map=$(HOME)=~
gui-release: $(GUIREL_BIN)
$(GUIREL_BIN): $(GUIREL_SRCS)
	@mkdir -p build
	$(CC) $(TRACE_CFLAGS) $(GUIREL_SCRUB) $(C54X_INC) -DDCT3_SDL -DDCT3_MODEL_3310_ONLY -DDCT3_RELEASE \
	      '-DDCT3_ATTRIB="$(DCT3_ATTRIB)"' \
	      $(SDL_CFLAGS) $(GUIREL_SRCS) $(SDL_LIBS) -o $@
	@echo "  [gui-release] $(GUIREL_BIN) — 3310-only, no co-sim, knobs stripped, path-scrubbed"

# --- .deb installer for the beta GUI --------------------------------------------
# Assembles a Debian package around the gui-release binary + wrapper + desktop entry
# + README. No firmware is bundled (copyrighted). Uses dpkg-deb --root-owner-group so
# no fakeroot/root is needed. Override version/maintainer:  make deb DEB_VERSION=1.1.0
DEB_VERSION    ?= 1.0.0
DEB_ARCH       ?= $(shell dpkg --print-architecture 2>/dev/null || echo amd64)
DEB_MAINTAINER ?= Dan Richardson <djrichardson747@gmail.com>
DEB_PKG        := build/dct3-emu_$(DEB_VERSION)_$(DEB_ARCH).deb
DEB_ROOT       := build/debpkg
deb: $(GUIREL_BIN)
	@echo "  [deb] staging $(DEB_ROOT)"
	@rm -rf $(DEB_ROOT)
	@install -Dm0755 $(GUIREL_BIN)                 $(DEB_ROOT)/usr/lib/dct3-emu/dct3-emu
	@strip --strip-unneeded $(DEB_ROOT)/usr/lib/dct3-emu/dct3-emu 2>/dev/null || true
	@install -Dm0755 packaging/deb/wrapper.sh       $(DEB_ROOT)/usr/bin/dct3-emu
	@install -Dm0644 packaging/deb/dct3-emu.desktop $(DEB_ROOT)/usr/share/applications/dct3-emu.desktop
	@install -Dm0644 /dev/null                      $(DEB_ROOT)/usr/share/doc/dct3-emu/README.md
	@install -Dm0644 /dev/null                      $(DEB_ROOT)/usr/share/doc/dct3-emu/copyright
	@install -Dm0644 packaging/deb/README.md  $(DEB_ROOT)/usr/share/doc/dct3-emu/README.md
	@install -Dm0644 packaging/deb/copyright  $(DEB_ROOT)/usr/share/doc/dct3-emu/copyright
	@ISIZE=$$(du -k -s $(DEB_ROOT)/usr | cut -f1); \
	  mkdir -p $(DEB_ROOT)/DEBIAN; \
	  sed -e 's/@VERSION@/$(DEB_VERSION)/' -e 's/@ARCH@/$(DEB_ARCH)/' \
	      -e 's|@MAINTAINER@|$(DEB_MAINTAINER)|' -e "s/@ISIZE@/$$ISIZE/" \
	      packaging/deb/control.in > $(DEB_ROOT)/DEBIAN/control
	@dpkg-deb --root-owner-group --build $(DEB_ROOT) $(DEB_PKG)
	@echo "  [deb] built $(DEB_PKG)"
	@dpkg-deb --info $(DEB_PKG) | sed -n '1,4p;/Depends/p;/Description/p'

# --- Standalone firmware disassembler (native): ./build/disfw <flash> <addr> [n] [arm]
DISFW_BIN  := build/disfw
DISFW_SRCS := tools/disfw.c src/core/dct3_core.c src/core/dct3_debug_stub.c $(CORE_SRCS)
disfw: $(DISFW_BIN)
$(DISFW_BIN): $(DISFW_SRCS)
	@mkdir -p build
	$(CC) $(TEST_CFLAGS) $(DISFW_SRCS) -o $@

# --- Real-SIM bridge bring-up CLI (native): ./build/simprobe <dev> <cmd>
# Host-side ISO-7816 T=0 driver over POSIX serial (termios); talks to the ESP32
# thin SIM reader in esp32/simbridge/. Pure serial I/O — does NOT link the
# emulator core or mad2, so it can never affect `make test`. See
# docs/sim-bridge-protocol.md.
SIMPROBE_BIN := build/simprobe
simprobe: $(SIMPROBE_BIN)
$(SIMPROBE_BIN): tools/simprobe.c tools/sim_bridge.c tools/sim_bridge.h
	@mkdir -p build
	$(CC) -std=c99 -O2 -Wall -Wextra tools/simprobe.c tools/sim_bridge.c -o $@

# Firmware inventory for the web UI's left column: scans firmware/ + flash/,
# parses each dump's filename, symlinks them under web/fw/, and writes
# web/firmware-manifest.json. Both outputs are gitignored (copyrighted dumps).
# Regenerated on every `make serve` so newly-dropped firmware shows up.
fw-manifest:
	@node tools/gen_fw_manifest.mjs

serve: all fw-manifest
	@echo "Serving on http://localhost:8000  (Ctrl-C to stop)"
	cd $(OUT_DIR) && python3 -m http.server 8000

clean:
	rm -f $(APP_OBJS) $(CORE_OBJS) $(OUT_JS) $(OUT_DIR)/dct3.wasm $(OUT_DIR)/dct3.wasm.map $(OUT_DIR)/dct3.data
	rm -f $(OUT_DIR)/asset-manifest.json $(OUT_DIR)/dct3.*.wasm $(OUT_DIR)/dct3.*.js $(OUT_DIR)/dct3.*.data $(OUT_DIR)/main.*.js
	rm -f $(OUT_DIR)/firmware-manifest.json
	rm -rf $(OUT_DIR)/fw build
