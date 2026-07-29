package com.example.dct3nokia

/**
 * Kotlin binding for the native DCT3 engine (app/src/main/cpp/) — a JNI wrapper around
 * the vendored nokia-dct3-emulator core, fixed to the Nokia 3410 model.
 *
 * Every native call is safe to invoke from any thread: the engine takes its own lock
 * internally (see dct3_engine.c), so the render/step loop (a background thread) and
 * key events (the UI thread) never race.
 */
object Dct3Engine {
    init {
        System.loadLibrary("dct3jni")
    }

    /** 0 on success; negative on failure (bad path, empty/short file, alloc failure). */
    fun boot(firmwarePath: String): Int = nativeBoot(firmwarePath)

    /** Advance the emulated clock by ~[cycles] CPU cycles (13 MHz — see [ArmHz]). */
    fun runCycles(cycles: Int) = nativeRunCycles(cycles)

    val lcdWidth: Int get() = nativeLcdWidth()
    val lcdHeight: Int get() = nativeLcdHeight()

    /**
     * Decode the current LCD framebuffer into [out] (row-major, width*height entries,
     * one 0xAARRGGBB int per pixel) — [onArgb] where the firmware lit the pixel,
     * [offArgb] otherwise.
     */
    fun renderPixels(out: IntArray, onArgb: Int, offArgb: Int) =
        nativeRenderPixels(out, onArgb, offArgb)

    /** Drive logical key [id] (see [Key]) up/down as a real matrix edge. Returns true
     *  if the key exists on the 3410 (i.e. the input was actually consumed). */
    fun keyEvent(id: Int, down: Boolean): Boolean = nativeKeyEvent(id, down)

    val isPoweredOff: Boolean get() = nativeIsPoweredOff()
    val isFaulted: Boolean get() = nativeIsFaulted()

    /** Mono 16-bit PCM sample rate [readAudio] fills [out] at (fixed — see EMU_AUDIO_HZ). */
    val audioSampleRate: Int get() = nativeAudioRate()

    /** Drain up to [out].size queued PCM samples (buzzer + keypad-tone audio) into [out].
     *  Returns the count actually written; 0 if none are queued yet. */
    fun readAudio(out: ShortArray): Int = nativeReadAudio(out)

    fun shutdown() = nativeShutdown()

    /** The emulated ARM clock rate (DCT3_ARM_HZ in the C core) — cycles/second. */
    const val ArmHz = 13_000_000

    /**
     * Logical key ids — mirrors the KeyId enum (KK_*) in
     * nokia-dct3-emulator/src/models/model.h, which that file documents as
     * APPEND-ONLY (never renumbered), so these values are a stable contract.
     */
    object Key {
        const val NONE = 0
        const val K0 = 1
        const val K1 = 2
        const val K2 = 3
        const val K3 = 4
        const val K4 = 5
        const val K5 = 6
        const val K6 = 7
        const val K7 = 8
        const val K8 = 9
        const val K9 = 10
        const val STAR = 11
        const val HASH = 12
        const val UP = 13
        const val DOWN = 14
        const val SOFT1 = 15
        const val SOFT2 = 16
        const val SEND = 17
        const val END = 18
        const val VOLUP = 19
        const val VOLDOWN = 20
        const val PWR = 21
    }

    private external fun nativeBoot(path: String): Int
    private external fun nativeRunCycles(cycles: Int)
    private external fun nativeLcdWidth(): Int
    private external fun nativeLcdHeight(): Int
    private external fun nativeRenderPixels(out: IntArray, onArgb: Int, offArgb: Int)
    private external fun nativeKeyEvent(id: Int, down: Boolean): Boolean
    private external fun nativeReadAudio(out: ShortArray): Int
    private external fun nativeAudioRate(): Int
    private external fun nativeIsPoweredOff(): Boolean
    private external fun nativeIsFaulted(): Boolean
    private external fun nativeShutdown()
}
