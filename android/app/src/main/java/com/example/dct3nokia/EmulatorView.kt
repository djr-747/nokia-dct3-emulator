package com.example.dct3nokia

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Rect
import android.util.AttributeSet
import android.view.View

/**
 * Renders just the Nokia 3410's LCD, scaled up and centered, on a solid black field —
 * no phone-shell chrome. This is meant for real feature-phone-shaped Android handsets
 * (e.g. the HMD Terra M / "Cupra") that already look like an old Nokia; the app only
 * needs to own the screen.
 *
 * Pixels are scaled by an INTEGER factor and drawn with nearest-neighbor sampling so
 * they stay crisp blocks rather than blurring, matching the native SDL front-end's
 * "classic Nokia mono panel" look.
 */
class EmulatorView(context: Context, attrs: AttributeSet? = null) : View(context, attrs) {

    // Classic backlit-LCD palette: dark pixels lit against a pale green-grey glow.
    var onColor: Int = Color.rgb(0x2B, 0x38, 0x24)
    var offColor: Int = Color.rgb(0x9E, 0xBC, 0x8A)

    private var lcdW = 0
    private var lcdH = 0
    private var pixels = IntArray(0)
    private var bitmap: Bitmap? = null
    private val srcRect = Rect()
    private val dstRect = Rect()
    private val paint = Paint().apply { isFilterBitmap = false; isDither = false }

    fun setLcdSize(w: Int, h: Int) {
        if (w <= 0 || h <= 0 || (w == lcdW && h == lcdH)) return
        lcdW = w
        lcdH = h
        pixels = IntArray(w * h)
        bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        srcRect.set(0, 0, w, h)
    }

    /** Pull the current framebuffer from the engine and request a redraw. Call from
     *  the emulation thread; drawing itself still only happens on the UI thread via
     *  the normal invalidate -> onDraw path. */
    fun updateFrame() {
        val bmp = bitmap ?: return
        Dct3Engine.renderPixels(pixels, onColor, offColor)
        bmp.setPixels(pixels, 0, lcdW, 0, 0, lcdW, lcdH)
        postInvalidate()
    }

    override fun onDraw(canvas: Canvas) {
        canvas.drawColor(Color.BLACK)
        val bmp = bitmap ?: return
        if (lcdW <= 0 || lcdH <= 0) return

        val scale = maxOf(1, minOf(width / lcdW, height / lcdH))
        val dw = lcdW * scale
        val dh = lcdH * scale
        val left = (width - dw) / 2
        val top = (height - dh) / 2
        dstRect.set(left, top, left + dw, top + dh)
        canvas.drawBitmap(bmp, srcRect, dstRect, paint)
    }
}
