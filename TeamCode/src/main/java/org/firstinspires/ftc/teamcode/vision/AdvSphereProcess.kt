package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.core.Rect
import kotlin.math.max
import kotlin.math.min

class AdvSphereProcess : VisionProcessor {
    data class PctSquare(val left: Double, val top: Double, val size: Double, val sizeRelativeTo: SizeRelativeTo = SizeRelativeTo.Longer) {
        enum class SizeRelativeTo {
            Width,
            Height,
            Longer,
            Shorter
        }

        fun toRect(target: Mat): Rect {
            val w = target.width()
            val h = target.height()
            val size = when (sizeRelativeTo) {
                SizeRelativeTo.Width -> w * size
                SizeRelativeTo.Height -> h * size
                SizeRelativeTo.Shorter -> min(w, h) * size
                SizeRelativeTo.Longer -> max(w, h) * size
            }.toInt()
            return Rect((w * left).toInt(), (h * top).toInt(), size, size)
        }
    }

    companion object {
        val Position3 = PctSquare(.72, .35, .25, PctSquare.SizeRelativeTo.Width)
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // setup
        TODO("Not yet implemented")
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
        // cv code
        TODO("Not yet implemented")
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        // visualization code
        TODO("Not yet implemented")
    }


}