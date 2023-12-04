package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.utilities.toInt
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import kotlin.math.max
import kotlin.math.min

class AdvSphereProcess(private val mode: Mode) : VisionProcessor {
    enum class Mode {
        Red,
        Blue
    }

    enum class Result {
        Left,
        Center,
        Right,
        None
    }

    data class PctSquare(
        val left: Double,
        val top: Double,
        val size: Double,
        val sizeRelativeTo: SizeRelativeTo = SizeRelativeTo.Longer
    ) {
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
        private const val HSV_MIN_S = 128.0
        private const val HSV_MIN_V = 32.0
        private val redFrom1 = Scalar(175.0, HSV_MIN_S, HSV_MIN_V)
        private val redTo1 = Scalar(180.0, 255.0, 255.0)
        private val redFrom2 = Scalar(0.0, HSV_MIN_S, HSV_MIN_V)
        private val redTo2 = Scalar(5.0, 255.0, 255.0)

        private val blueFrom = Scalar(95.0, HSV_MIN_S, HSV_MIN_V)
        private val blueTo = Scalar(125.0, 255.0, 255.0)

        val Position1 = PctSquare(.07, .35, .25, PctSquare.SizeRelativeTo.Width)
        val Position2 = PctSquare(.38, .30, .25, PctSquare.SizeRelativeTo.Width)
        val Position3 = PctSquare(.72, .35, .25, PctSquare.SizeRelativeTo.Width)

        // at least X times more than other columns to confidently vote here
        private const val ClearMajorityFactor = 2.0
        private const val BlurSize = 11.0

        private const val circleDP = 1.0
        private const val circleMinDist = 20.0
        private const val circleParam1 = 50.0
        private const val circleParam2 = 40.0
        private const val circleMinRadius = 0
        private const val circleMaxRadius = 0
    }

    private var pos1 = Rect(0, 0, 0, 0)
    private var pos2 = Rect(0, 0, 0, 0)
    private var pos3 = Rect(0, 0, 0, 0)

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
    }

    private var scratch = Mat()
    private var recolor = Mat()
    private var red1 = Mat()
    private var red2 = Mat()
    private var blue = Mat()

    var result: Result = Result.None
        private set

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        frame ?: throw IllegalStateException("passed a null frame to processFrame")
        Imgproc.cvtColor(frame, scratch, Imgproc.COLOR_RGB2BGR)
        val scalingFraction = 720.0 / scratch.width()
        Imgproc.resize(scratch, scratch, Size(), scalingFraction, scalingFraction)
        pos1 = Position1.toRect(scratch)
        pos2 = Position2.toRect(scratch)
        pos3 = Position3.toRect(scratch)

        Imgproc.cvtColor(scratch, recolor, Imgproc.COLOR_BGR2HSV)
        Core.inRange(recolor, redFrom1, redTo1, red1)
        Core.inRange(recolor, redFrom2, redTo2, red2)
        Core.add(red1, red2, red1)
        Core.inRange(recolor, blueFrom, blueTo, blue)

        val redLeft = Mat(red1, pos1)
        val redCenter = Mat(red1, pos2)
        val redRight = Mat(red1, pos3)
        val blueLeft = Mat(blue, pos1)
        val blueCenter = Mat(blue, pos2)
        val blueRight = Mat(blue, pos3)
        try {
            val votesLeft = Core.countNonZero(
                when (mode) {
                    Mode.Red -> redLeft
                    Mode.Blue -> blueLeft
                }
            )
            val votesCenter = Core.countNonZero(
                when (mode) {
                    Mode.Red -> redCenter
                    Mode.Blue -> blueCenter
                }
            )
            val votesRight = Core.countNonZero(
                when (mode) {
                    Mode.Red -> redRight
                    Mode.Blue -> blueRight
                }
            )
            val leftPassed = votesLeft > max(votesCenter, votesRight) * ClearMajorityFactor
            val centerPassed = votesCenter > max(votesLeft, votesRight) * ClearMajorityFactor
            val rightPassed = votesCenter > max(votesLeft, votesCenter) * ClearMajorityFactor
            if (leftPassed.toInt() + centerPassed.toInt() + rightPassed.toInt() == 1) {
                result = when {
                    leftPassed -> Result.Left
                    centerPassed -> Result.Center
                    rightPassed -> Result.Right
                    else -> throw IllegalStateException("What the hell happened here? One is true but none are true for some reason idk")
                }
                return null
            }
        } finally {
            redLeft.release()
            redCenter.release()
            redRight.release()
            blueLeft.release()
            blueCenter.release()
            blueRight.release()
        }

        // We are here if:
        // not confident enough
        // somehow two spots both got the extreme majority

        // Do circles
        Imgproc.cvtColor(scratch, recolor, Imgproc.COLOR_BGR2GRAY)
        Imgproc.blur(recolor, recolor, Size(BlurSize, BlurSize))
        val left = Mat(recolor, pos1)
        val center = Mat(recolor, pos2)
        val right = Mat(recolor, pos3)
        val leftCircles = Mat()
        val centerCircles = Mat()
        val rightCircles = Mat()
        try {
            Imgproc.HoughCircles(
                left, leftCircles, Imgproc.HOUGH_GRADIENT,
                circleDP,
                circleMinDist, // min. dist. between circles
                circleParam1, // Canny edge detection high threshold
                circleParam2, // minimum number of votes "accumulator threshold"
                // min and max radius (set these values as you desire)
                circleMinRadius, circleMaxRadius,
            )
            Imgproc.HoughCircles(
                center, centerCircles, Imgproc.HOUGH_GRADIENT,
                circleDP,
                circleMinDist, // min. dist. between circles
                circleParam1, // Canny edge detection high threshold
                circleParam2, // minimum number of votes "accumulator threshold"
                // min and max radius (set these values as you desire)
                circleMinRadius, circleMaxRadius,
            )
            Imgproc.HoughCircles(
                right, rightCircles, Imgproc.HOUGH_GRADIENT,
                circleDP,
                circleMinDist, // min. dist. between circles
                circleParam1, // Canny edge detection high threshold
                circleParam2, // minimum number of votes "accumulator threshold"
                // min and max radius (set these values as you desire)
                circleMinRadius, circleMaxRadius,
            )
        } finally {
            left.release()
            center.release()
            right.release()
        }
        // I give up.
        result = Result.None
        return null
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        canvas ?: return
        // visualization code
        val name = when (result) {
            Result.None -> "Not sure. :("
            Result.Left -> "Left"
            Result.Center -> "Center"
            Result.Right -> "Right"
        }

        canvas.drawText(name, 10F, 30F, Paint().apply {
            textSize = 28F
        })
    }


}