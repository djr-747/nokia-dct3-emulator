package com.example.dct3nokia

import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioTrack
import android.os.Bundle
import android.util.Log
import android.view.KeyEvent
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import java.io.File
import java.io.FileOutputStream

/**
 * Nokia 3410 emulator front-end. No phone-shell UI is rendered — this Activity owns
 * the whole screen and shows only the emulated LCD (via [EmulatorView]) on a black
 * field, matching real feature-phone-shaped Android hardware (developed and tested
 * against an HMD Terra M / "Cupra").
 *
 * Firmware is bring-your-own (this repo ships no copyrighted Nokia firmware — see the
 * top-level README.md): this Activity looks for a raw NHM-2 (3410) .fls image at
 * [firmwareFile], and falls back to a plain "pick a file" prompt (no permission
 * needed — Storage Access Framework) if it isn't there yet.
 */
class MainActivity : ComponentActivity() {

    private val firmwareFile: File
        get() = File(getExternalFilesDir(null) ?: filesDir, "firmware.fls")

    private lateinit var emulatorView: EmulatorView
    private var emuThread: Thread? = null
    @Volatile private var running = false

    private var audioTrack: AudioTrack? = null
    private val audioBuf = ShortArray(4096)

    private val pickFirmware = registerForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        if (uri == null) return@registerForActivityResult
        runCatching {
            contentResolver.openInputStream(uri)?.use { input ->
                FileOutputStream(firmwareFile).use { output -> input.copyTo(output) }
            }
        }.onFailure { e ->
            Log.e(TAG, "firmware copy failed", e)
        }
        showSetupOrBoot()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        goImmersive()
        showSetupOrBoot()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) goImmersive()
    }

    private fun goImmersive() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        val controller = WindowInsetsControllerCompat(window, window.decorView)
        controller.hide(WindowInsetsCompat.Type.systemBars())
        controller.systemBarsBehavior =
            WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
    }

    // --- Setup screen (only shown before firmware exists / after a boot failure) ---

    private fun showSetupOrBoot() {
        val fw = firmwareFile
        if (fw.isFile && fw.length() > 0) {
            bootAndShowEmulator(fw)
        } else {
            setContentView(buildSetupView(
                "No firmware found.\n\n" +
                    "Push a Nokia 3410 (NHM-2) .fls image to:\n${fw.absolutePath}\n\n" +
                    "then relaunch, or pick a file below."
            ))
        }
    }

    private fun buildSetupView(message: String): View {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(android.graphics.Color.BLACK)
            gravity = android.view.Gravity.CENTER
            setPadding(48, 48, 48, 48)
        }
        root.addView(TextView(this).apply {
            text = message
            setTextColor(android.graphics.Color.WHITE)
            textSize = 14f
        })
        root.addView(Button(this).apply {
            text = "Pick firmware file…"
            setOnClickListener { pickFirmware.launch(arrayOf("*/*")) }
        })
        return root
    }

    private fun bootAndShowEmulator(fw: File) {
        val rc = Dct3Engine.boot(fw.absolutePath)
        if (rc != 0) {
            setContentView(buildSetupView(
                "Boot failed (code $rc) loading:\n${fw.absolutePath}\n\n" +
                    "Make sure it's a raw Nokia 3410 (NHM-2) .fls dump, then pick a different file."
            ))
            return
        }

        emulatorView = EmulatorView(this)
        emulatorView.setLcdSize(Dct3Engine.lcdWidth, Dct3Engine.lcdHeight)
        emulatorView.isFocusable = true
        emulatorView.isFocusableInTouchMode = true
        setContentView(emulatorView, FrameLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT
        ))
        emulatorView.requestFocus()
        setupAudio()
        startEmulationLoop()
    }

    // --- Audio: the buzzer + DSP HLE tone (keypad beeps, ringtones) the shared mad2
    // mixer renders — see Dct3Engine.readAudio / nokia-dct3-emulator/src/mad2/emu_audio.c.

    private fun setupAudio() {
        val rate = Dct3Engine.audioSampleRate
        val minBuf = AudioTrack.getMinBufferSize(
            rate, AudioFormat.CHANNEL_OUT_MONO, AudioFormat.ENCODING_PCM_16BIT
        )
        // A ~200ms cushion against this thread's own 1/30s tick jitter — comfortably
        // more than the ~1/30s of samples produced per tick, well under audible latency.
        val bufBytes = maxOf(minBuf, rate / 5 * 2)
        audioTrack = AudioTrack.Builder()
            .setAudioAttributes(
                AudioAttributes.Builder()
                    .setUsage(AudioAttributes.USAGE_MEDIA)
                    .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
                    .build()
            )
            .setAudioFormat(
                AudioFormat.Builder()
                    .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                    .setSampleRate(rate)
                    .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                    .build()
            )
            .setBufferSizeInBytes(bufBytes)
            .setTransferMode(AudioTrack.MODE_STREAM)
            .build()
    }

    // --- Emulation loop (background thread: advance cycles, push a frame, drain audio) -

    private fun startEmulationLoop() {
        running = true
        audioTrack?.play()
        val cyclesPerTick = Dct3Engine.ArmHz / TICK_HZ
        emuThread = Thread({
            // A CPU-bound background thread otherwise competes for scheduling with the
            // rest of the system at normal priority; this is latency-sensitive real-time
            // work (like audio synthesis), so ask for the same elevated class.
            android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_AUDIO)

            var nextTick = System.nanoTime()
            // Rolling throughput log: how many emulated cycles/sec this device is actually
            // achieving vs. the 13 MHz (Dct3Engine.ArmHz) real-time target — logged once a
            // second so a slow device is visible/measurable, not just "feels slow".
            var perfWindowStart = System.nanoTime()
            var perfCyclesDone = 0L
            var perfTicks = 0

            while (running) {
                val stepStart = System.nanoTime()
                Dct3Engine.runCycles(cyclesPerTick)
                val stepNs = System.nanoTime() - stepStart
                perfCyclesDone += cyclesPerTick
                perfTicks++

                if (Dct3Engine.isFaulted) {
                    Log.e(TAG, "engine halted on an unrecoverable fault — see logcat above")
                } else if (Dct3Engine.isPoweredOff) {
                    Log.i(TAG, "phone powered off")
                }
                emulatorView.updateFrame()

                val n = Dct3Engine.readAudio(audioBuf)
                if (n > 0) audioTrack?.write(audioBuf, 0, n)

                val now = System.nanoTime()
                if (now - perfWindowStart >= 1_000_000_000L) {
                    val achievedHz = perfCyclesDone * 1_000_000_000L / (now - perfWindowStart)
                    val pctOfRealtime = achievedHz * 100 / Dct3Engine.ArmHz
                    Log.d(TAG, "perf: $achievedHz cyc/s ($pctOfRealtime% of realtime), " +
                        "last runCycles() took ${stepNs / 1_000_000}ms for a ${1000 / TICK_HZ}ms tick, " +
                        "$perfTicks ticks/s")
                    perfWindowStart = now
                    perfCyclesDone = 0
                    perfTicks = 0
                }

                nextTick += TICK_NS
                val sleepMs = (nextTick - System.nanoTime()) / 1_000_000
                if (sleepMs > 0) Thread.sleep(sleepMs) else nextTick = System.nanoTime()
            }
        }, "dct3-emu").apply { start() }
    }

    private fun stopEmulationLoop() {
        running = false
        emuThread?.join(500)
        emuThread = null
        audioTrack?.pause()
        audioTrack?.flush()
    }

    override fun onPause() {
        super.onPause()
        stopEmulationLoop()
    }

    override fun onResume() {
        super.onResume()
        if (::emulatorView.isInitialized && !running) startEmulationLoop()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopEmulationLoop()
        audioTrack?.release()
        audioTrack = null
        Dct3Engine.shutdown()
    }

    // --- Physical key input: D-pad, numeric keypad, Menu/Back. No on-screen keypad
    // is drawn; every key here is a real hardware key on the target device. ---------

    override fun onKeyDown(keyCode: Int, event: KeyEvent): Boolean {
        if (!::emulatorView.isInitialized) return super.onKeyDown(keyCode, event)
        val id = mapKey(keyCode)
        if (id != Dct3Engine.Key.NONE && event.repeatCount == 0) {
            if (Dct3Engine.keyEvent(id, true)) return true
        }
        return super.onKeyDown(keyCode, event)
    }

    override fun onKeyUp(keyCode: Int, event: KeyEvent): Boolean {
        if (!::emulatorView.isInitialized) return super.onKeyUp(keyCode, event)
        val id = mapKey(keyCode)
        if (id != Dct3Engine.Key.NONE) {
            if (Dct3Engine.keyEvent(id, false)) return true
        }
        return super.onKeyUp(keyCode, event)
    }

    private fun mapKey(keyCode: Int): Int = when (keyCode) {
        KeyEvent.KEYCODE_0 -> Dct3Engine.Key.K0
        KeyEvent.KEYCODE_1 -> Dct3Engine.Key.K1
        KeyEvent.KEYCODE_2 -> Dct3Engine.Key.K2
        KeyEvent.KEYCODE_3 -> Dct3Engine.Key.K3
        KeyEvent.KEYCODE_4 -> Dct3Engine.Key.K4
        KeyEvent.KEYCODE_5 -> Dct3Engine.Key.K5
        KeyEvent.KEYCODE_6 -> Dct3Engine.Key.K6
        KeyEvent.KEYCODE_7 -> Dct3Engine.Key.K7
        KeyEvent.KEYCODE_8 -> Dct3Engine.Key.K8
        KeyEvent.KEYCODE_9 -> Dct3Engine.Key.K9
        KeyEvent.KEYCODE_STAR -> Dct3Engine.Key.STAR
        KeyEvent.KEYCODE_POUND -> Dct3Engine.Key.HASH
        KeyEvent.KEYCODE_DPAD_UP -> Dct3Engine.Key.UP
        KeyEvent.KEYCODE_DPAD_DOWN -> Dct3Engine.Key.DOWN
        // Spatial soft keys: D-pad left/right sit either side of the physical pad.
        // The right soft key (SOFT2) is also the phone's real "back" action — Nokia's
        // UI relabels it contextually ("Names" at standby, "Exit"/"Back" in a menu) —
        // confirmed on-device, so the dedicated hardware Back button below the display
        // goes here too, NOT to KK_END (the red hang-up key, which jumps straight to
        // standby instead of stepping back one menu level).
        KeyEvent.KEYCODE_DPAD_LEFT -> Dct3Engine.Key.SOFT1
        KeyEvent.KEYCODE_DPAD_RIGHT, 411 -> Dct3Engine.Key.SOFT2
        // The two dedicated hardware buttons just below the display on this device
        // (HMD Terra M) send raw keycodes with no standard KeyEvent.KEYCODE_* constant
        // — confirmed on-device: 414 = Menu, 411 = Back (see above). KEYCODE_MENU is
        // kept too as a harmless fallback.
        414, KeyEvent.KEYCODE_MENU -> Dct3Engine.Key.SOFT1
        KeyEvent.KEYCODE_BACK, KeyEvent.KEYCODE_ENDCALL -> Dct3Engine.Key.END
        KeyEvent.KEYCODE_DPAD_CENTER, KeyEvent.KEYCODE_CALL -> Dct3Engine.Key.SEND
        else -> Dct3Engine.Key.NONE
    }

    companion object {
        private const val TAG = "dct3-mainactivity"
        private const val TICK_HZ = 30
        private const val TICK_NS = 1_000_000_000L / TICK_HZ
    }
}
