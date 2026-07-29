# Native build for the DCT3 Android front-end (Nokia 3410 only — see
# src/models/model.c's DCT3_MODEL_3410_ONLY registry).
#
# Links the repo's own C core + MAD2 platform model + shared fault/recovery
# harness + mGBA ARM interpreter, unmodified, against dct3_engine.c (the Android
# driver loop) and dct3_jni.c (the thin JNI wrapper). Only the 3410 profile +
# the shared MAD2_SIGS table are compiled from src/models/ — not the other 25
# phone profiles and not the C54x DSP co-sim (third_party/c54x/), which only the
# serial-bus profiles (5110/6110/3210/7110) need; the 3410 uses the HLE DSP.
cmake_minimum_required(VERSION 3.22)
project(dct3jni C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# android/app/src/main/cpp -> repo root (this module lives inside the main repo,
# not against an external checkout).
get_filename_component(EMU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)

if(NOT EXISTS "${EMU_ROOT}/src/core/dct3_core.c")
    message(FATAL_ERROR "repo root not found at ${EMU_ROOT} — this CMakeLists.txt expects to live at android/app/src/main/cpp/ inside the nokia-dct3-emulator repo")
endif()

file(GLOB MAD2_SRCS     "${EMU_ROOT}/src/mad2/*.c")
file(GLOB HARNESS_SRCS  "${EMU_ROOT}/src/harness/*.c")
file(GLOB MGBA_SRCS     "${EMU_ROOT}/third_party/mgba-arm/src/arm/*.c")

add_library(dct3jni SHARED
    dct3_jni.c
    dct3_engine.c
    "${EMU_ROOT}/src/core/dct3_core.c"
    "${EMU_ROOT}/src/core/dct3_debug_stub.c"
    "${EMU_ROOT}/src/models/model.c"
    "${EMU_ROOT}/src/models/3410/profile.c"
    "${EMU_ROOT}/src/models/mad2_sigs.c"
    "${EMU_ROOT}/tools/sim_bridge.c"
    ${MAD2_SRCS}
    ${HARNESS_SRCS}
    ${MGBA_SRCS}
)

target_include_directories(dct3jni PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${EMU_ROOT}/src"
    "${EMU_ROOT}/third_party/mgba-arm/include"
    "${EMU_ROOT}/tools"
)

# Selects the single-model registry in src/models/model.c (see the edit there) — the
# ONLY nokia-dct3-emulator source change this app required.
target_compile_definitions(dct3jni PRIVATE DCT3_MODEL_3410_ONLY)

# Perf: this interpreter is CPU-bound on a weak armeabi-v7a SoC (QCM2290) — measured
# on-device at only ~34-36% of real-time (13 MHz) with a plain -O2 build. Empirically
# tried, on-device, in this order (nokia-dct3-emulator's Makefile carries -flto for
# its native/x86 build and explicitly found -O3 REGRESSED there from I-cache pressure
# — neither claim held on this ARM target, so don't trust either without remeasuring):
#   -O2 -flto=thin           -> no measurable change (~32-35%, noise)
#   -O3 -flto (full)         -> ~41-44%, consistently — KEPT
# Still well under real-time: this looks like a genuine single-core throughput
# ceiling on this device's cores for a branchy/pointer-chasing decode-dispatch loop
# (CPU governor confirmed already pinned at max frequency — not throttling), not
# something a compiler flag closes the rest of the way.
#
# check_ipo_supported() was tried instead of the raw -flto flags below and rejected:
# its try_compile probe picks up the HOST's ld.gold instead of the NDK's own LLD on
# this toolchain and fails, even though the real target build resolves the correct
# NDK linker fine on its own.
target_compile_options(dct3jni PRIVATE -O3 -flto -Wall)
target_link_options(dct3jni PRIVATE -flto)

target_link_libraries(dct3jni PRIVATE log m)
