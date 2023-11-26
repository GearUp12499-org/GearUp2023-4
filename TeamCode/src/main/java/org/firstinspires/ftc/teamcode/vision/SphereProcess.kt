package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

class SphereProcess : VisionProcessor {
    companion object {
        private const val BLUR_RADIUS = 11.0
        private const val HSV_MIN_S = 128.0
        private const val HSV_MIN_V = 32.0
        private val redFrom1 = Scalar(175.0, HSV_MIN_S, HSV_MIN_V)
        private val redTo1 = Scalar(180.0, 255.0, 255.0)
        private val redFrom2 = Scalar(0.0, HSV_MIN_S, HSV_MIN_V)
        private val redTo2 = Scalar(5.0, 255.0, 255.0)

        private val blueFrom = Scalar(95.0, HSV_MIN_S, HSV_MIN_V)
        private val blueTo = Scalar(125.0, 255.0, 255.0)

        private const val circleDP = 1.0
        private const val circleMinDist = 20.0
        private const val circleParam1 = 50.0
        private const val circleParam2 = 40.0
        private const val circleMinRadius = 0
        private const val circleMaxRadius = 0
    }

    private var width: Int = 0
    private var height: Int = 0

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        this.width = width
        this.height = height
        this.scratch = Mat(width, height, CvType.CV_8UC4)
        // We need to do a bitwise OR to look at reddish objects...
        this.red1 = Mat(width, height, CvType.CV_8U)
        this.red2 = Mat(width, height, CvType.CV_8U)
        this.blue = Mat(width, height, CvType.CV_8U)
    }

    private lateinit var scratch: Mat
    private lateinit var red1: Mat
    private lateinit var red2: Mat
    private lateinit var blue: Mat

    private var redCircles: Mat = Mat()
    private var blueCircles: Mat = Mat()

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        frame ?: throw IllegalStateException("passed a null frame to processFrame")
        val frac = 720.0 / frame.width()
        RobotLog.ii("SphereProcess", "Resize")
        Imgproc.resize(frame, scratch, Size(), frac, frac)
        RobotLog.ii("SphereProcess", "Convert")
        Imgproc.cvtColor(scratch, scratch, Imgproc.COLOR_BGR2HSV)
        RobotLog.ii("SphereProcess", "Filter")
        Core.inRange(scratch, redFrom1, redTo1, red1)
        Core.inRange(scratch, redFrom2, redTo2, red2)
        Core.add(red1, red2, red1)
        Core.inRange(scratch, blueFrom, blueTo, blue)

        RobotLog.ii("SphereProcess", "Blur")
        Imgproc.GaussianBlur(red1, red1, Size(BLUR_RADIUS, BLUR_RADIUS), 5.0)
        Imgproc.GaussianBlur(blue, blue, Size(BLUR_RADIUS, BLUR_RADIUS), 5.0)

        RobotLog.ii("SphereProcess", "Circles 1/2")
        Imgproc.HoughCircles(
            red1, redCircles, Imgproc.HOUGH_GRADIENT,
            circleDP,
            circleMinDist, // min. dist. between circles
            circleParam1, // Canny edge detection high threshold
            circleParam2, // minimum number of votes "accumulator threshold"
            // min and max radius (set these values as you desire)
            circleMinRadius, circleMaxRadius,
        )
        RobotLog.ii("SphereProcess", "Circles 2/2")
        Imgproc.HoughCircles(
            blue, blueCircles, Imgproc.HOUGH_GRADIENT,
            circleDP,
            circleMinDist, // min. dist. between circles
            circleParam1, // Canny edge detection high threshold
            circleParam2, // minimum number of votes "accumulator threshold"
            // min and max radius (set these values as you desire)
            circleMinRadius, circleMaxRadius,
        )
        RobotLog.ii("SphereProcess", "Pass Done")
        return null
    }

    private fun draw(canvas: Canvas, scaling: Float, circles: Mat, color: Int) {
        for (x in 0 until circles.cols()) {
            val circle = circles.get(0, x)
            val centerX = circle[0].toFloat() * scaling
            val centerY = circle[1].toFloat() * scaling
            val radius = circle[2].toFloat() * scaling
            canvas.drawPoint(
                centerX, centerY,
                Paint().apply {
                    this.color = color
                    style = Paint.Style.STROKE
                    strokeWidth = 5f
                },
            )
            canvas.drawCircle(
                centerX, centerY, radius,
                Paint().apply {
                    this.color = color
                    style = Paint.Style.STROKE
                    strokeWidth = 5f
                }
            )
        }
    }

    /**
     * Rendering code, called when refreshing the *display*, for visualization only.
     */
    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        canvas ?: return
        RobotLog.ii("SphereProcess", "${redCircles.size()} red, ${blueCircles.size()} blue")
        draw(canvas, scaleBmpPxToCanvasPx, redCircles, Color.RED)
        draw(canvas, scaleBmpPxToCanvasPx, blueCircles, Color.BLUE)
    }

}