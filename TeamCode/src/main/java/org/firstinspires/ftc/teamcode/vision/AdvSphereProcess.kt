package org.firstinspires.ftc.teamcode.vision

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.utilities.toInt
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import kotlin.math.max
import kotlin.math.min

class AdvSphereProcess(var mode: Mode, var altBoxes: Boolean) : VisionProcessor {
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

    enum class CircleResult {
        Blue,
        Red,
        None
    }

    data class PctSquare(
        val left: Double,
        val top: Double,
        val width: Double,
        val height: Double,
    ) {
        fun toRect(target: Mat): Rect {
            val w = target.width()
            val h = target.height()
            val sw = (w * width).toInt()
            val sh = (h * height).toInt()
            val top = (h * top).toInt()
            val left = (w * left).toInt()
            return Rect(left, top, min(w - left, sw), min(h - top, sh))
        }
    }

    var votesLeft = 0
        private set
    var votesCenter = 0
        private set
    var votesRight = 0
        private set

    var circlesLeft = 0
        private set
    var circlesCenter = 0
        private set
    var circlesRight = 0
        private set

    companion object {
        private const val HSV_MIN_S = 128.0 // 50%
        private const val HSV_MIN_V_RED = 32.0  // 12.5%
        private val redFrom1 = Scalar(175.0, HSV_MIN_S, HSV_MIN_V_RED)
        private val redTo1 = Scalar(180.0, 255.0, 255.0)
        private val redFrom2 = Scalar(0.0, HSV_MIN_S, HSV_MIN_V_RED)
        private val redTo2 = Scalar(5.0, 255.0, 255.0)

        private const val HSV_MIN_V_BLUE = 70.0 // 27.5%
        private val blueFrom = Scalar(95.0, HSV_MIN_S, HSV_MIN_V_BLUE)
        private val blueTo = Scalar(125.0, 255.0, 255.0)

        val Position1A = PctSquare(.03, .30, .16, .60)
        val Position2A = PctSquare(.36, .30, .16, .60)
        val Position3A = PctSquare(.69, .30, .16, .60)

        val Position1B = PctSquare(.20, .30, .16, .60)
        val Position2B = PctSquare(.51, .30, .16, .60)
        val Position3B = PctSquare(.83, .30, .16, .60)

        // at least X times more than other columns to confidently vote here
        private const val ClearMajorityFactor = 2
        private const val CircleFillVotePct = .5
        private const val BlurSize = 11.0

        private const val circleDP = 1.0
        private const val circleMinDist = 20.0
        private const val circleParam1 = 50.0
        private const val circleParam2 = 40.0
        private const val circleMinRadius = 0
        private const val circleMaxRadius = 0
    }

    private var scalingFraction: Double = 0.5
    private var pos1 = Rect(0, 0, 0, 0)
    private var pos2 = Rect(0, 0, 0, 0)
    private var pos3 = Rect(0, 0, 0, 0)

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
    }

    private var origin = Mat()
    private var scratch = Mat()
    private var maskScratch = Mat()
    private var hsvColor = Mat()
    private var grayColor = Mat()
    private var red1 = Mat()
    private var red2 = Mat()
    private var blue = Mat()
    private var left = Mat()
    private var center = Mat()
    private var right = Mat()

    var result: Result = Result.None
        private set
    var strategy: String = ""
        private set

    private fun processCircle(center: Point, radius: Int): CircleResult {
        val mask = Mat(hsvColor.size(), CvType.CV_8UC1, Scalar(0.0))
        Imgproc.circle(mask, center, radius, Scalar(255.0))
        val total = Core.countNonZero(mask)
        Core.bitwise_and(red1, mask, maskScratch)
        val reds = Core.countNonZero(maskScratch)
        Core.bitwise_and(blue, mask, maskScratch)
        val blues = Core.countNonZero(maskScratch)
        mask.release()
        val isRed = reds / total.toDouble() > CircleFillVotePct
        val isBlue = blues / total.toDouble() > CircleFillVotePct
        return when {
            isRed && isBlue -> CircleResult.None
            isRed -> CircleResult.Red
            isBlue -> CircleResult.Blue
            else -> CircleResult.None
        }
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        frame ?: throw IllegalStateException("passed a null frame to processFrame")
        Imgproc.cvtColor(frame, scratch, Imgproc.COLOR_RGB2BGR)
        Imgproc.cvtColor(frame, origin, Imgproc.COLOR_RGB2BGR)
        scalingFraction = 720.0 / scratch.width()
        Imgproc.resize(scratch, scratch, Size(), scalingFraction, scalingFraction)
        pos1 = (if (altBoxes) Position1B else Position1A).toRect(scratch)
        pos2 = (if (altBoxes) Position2B else Position2A).toRect(scratch)
        pos3 = (if (altBoxes) Position3B else Position3A).toRect(scratch)

        Imgproc.cvtColor(scratch, hsvColor, Imgproc.COLOR_BGR2HSV)
        //Core contains basic image operations like masking
        Core.inRange(hsvColor, redFrom1, redTo1, red1)
        Core.inRange(hsvColor, redFrom2, redTo2, red2)
        Core.add(red1, red2, red1)
        Core.inRange(hsvColor, blueFrom, blueTo, blue)

        left = Mat(if (mode == Mode.Red) red1 else blue, pos1)
        center = Mat(if (mode == Mode.Red) red1 else blue, pos2)
        right = Mat(if (mode == Mode.Red) red1 else blue, pos3)

        val redLeft = Mat(red1, pos1)
        val redCenter = Mat(red1, pos2)
        val redRight = Mat(red1, pos3)
        val blueLeft = Mat(blue, pos1)
        val blueCenter = Mat(blue, pos2)
        val blueRight = Mat(blue, pos3)
        try {
            votesLeft = Core.countNonZero(
                when (mode) {
                    Mode.Red -> redLeft
                    Mode.Blue -> blueLeft
                }
            )
            votesCenter = Core.countNonZero(
                when (mode) {
                    Mode.Red -> redCenter
                    Mode.Blue -> blueCenter
                }
            )
            votesRight = Core.countNonZero(
                when (mode) {
                    Mode.Red -> redRight
                    Mode.Blue -> blueRight
                }
            )
            val leftPassed = votesLeft > max(votesCenter, votesRight) * ClearMajorityFactor
            val centerPassed = votesCenter > max(votesLeft, votesRight) * ClearMajorityFactor
            val rightPassed = votesRight > max(votesLeft, votesCenter) * ClearMajorityFactor
            if (leftPassed.toInt() + centerPassed.toInt() + rightPassed.toInt() == 1) {
                result = when {
                    leftPassed -> Result.Left
                    centerPassed -> Result.Center
                    rightPassed -> Result.Right
                    else -> throw IllegalStateException("What the hell happened here? One is true but none are true for some reason idk")
                }
                strategy = "PixelCount"
                return null
            } else if (leftPassed.toInt() + centerPassed.toInt() + rightPassed.toInt() > 1) {
                result = Result.None
                strategy = "CataFailure!"
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
        Imgproc.cvtColor(scratch, grayColor, Imgproc.COLOR_BGR2GRAY)
        Imgproc.blur(grayColor, grayColor, Size(BlurSize, BlurSize))
        val left = Mat(grayColor, pos1)
        val center = Mat(grayColor, pos2)
        val right = Mat(grayColor, pos3)
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

            var leftRedC = 0
            var leftBlueC = 0
            for (i in 0 until leftCircles.cols()) {
                val circle = leftCircles.get(0, i)
                val centerPoint = Point(circle[0] + pos1.x, circle[1] + pos1.y)
                when (processCircle(centerPoint, circle[2].toInt())) {
                    CircleResult.Red -> leftRedC++
                    CircleResult.Blue -> leftBlueC++
                    else -> {}
                }
            }

            var centerRedC = 0
            var centerBlueC = 0
            for (i in 0 until centerCircles.cols()) {
                val circle = centerCircles.get(0, i)
                val centerPoint = Point(circle[0] + pos2.x, circle[1] + pos2.y)
                when (processCircle(centerPoint, circle[2].toInt())) {
                    CircleResult.Red -> centerRedC++
                    CircleResult.Blue -> centerBlueC++
                    else -> {}
                }
            }

            var rightRedC = 0
            var rightBlueC = 0
            for (i in 0 until rightCircles.cols()) {
                val circle = rightCircles.get(0, i)
                val centerPoint = Point(circle[0] + pos3.x, circle[1] + pos3.y)
                when (processCircle(centerPoint, circle[2].toInt())) {
                    CircleResult.Red -> rightRedC++
                    CircleResult.Blue -> rightBlueC++
                    else -> {}
                }
            }

            val leftC = when (mode) {
                Mode.Red -> leftRedC
                Mode.Blue -> leftBlueC
            }
            circlesLeft = leftC
            val centerC = when (mode) {
                Mode.Red -> centerRedC
                Mode.Blue -> centerBlueC
            }
            circlesCenter = centerC
            val rightC = when (mode) {
                Mode.Red -> rightRedC
                Mode.Blue -> rightBlueC
            }
            circlesRight = rightC
            val maximC = max(max(max(leftC, centerC), rightC), 1)
            val countMatching =
                (if (leftC == maximC) 1 else 0) + (if (centerC == maximC) 1 else 0) + (if (rightC == maximC) 1 else 0)
            strategy = "Circles ($maximC detected)"
            result = when {
                countMatching > 1 -> {
                    strategy = "Circles (too many? $countMatching are $maximC)"
                    result
                } // well that's weird
                countMatching < 1 -> {
                    strategy = "Circles (cached)"
                    result
                } // inconclusive
                leftC == maximC -> Result.Left
                centerC == maximC -> Result.Center
                rightC == maximC -> Result.Right
                else -> result // shouldn't happen but shuts up compiler
            }
        } finally {
            left.release()
            center.release()
            right.release()
        }
        // I give up.
        // actually don't overwrite the result.
//        result = Result.None
        return null
    }

    fun capture() {
        val application: Context = AppUtil.getInstance().application
        application.getExternalFilesDir(null)?.let {
            val path = it.absolutePath
            RobotLog.ii("SphereProcess", "Writing to $path")
            Imgcodecs.imwrite("$path/left.png", left)
            Imgcodecs.imwrite("$path/center.png", center)
            Imgcodecs.imwrite("$path/right.png", right)
            Imgcodecs.imwrite("$path/original_flipped.png", origin)
        }
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        bmp2canv: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        canvas ?: return
        val unscale = (1.0 / scalingFraction) * bmp2canv
        // visualization code
        val name = when (result) {
            Result.None -> "Not sure. :("
            Result.Left -> "Left"
            Result.Center -> "Center"
            Result.Right -> "Right"
        }

        canvas.drawRect(0f, 0f, onscreenWidth * 0.25f, 64f, Paint().apply {
            style = Paint.Style.FILL
            color = 0xc0ffffff.toInt()
        })
        canvas.drawText(name, 10F, 30F, Paint().apply {
            textSize = 28F
        })
        canvas.drawText(strategy, 10F, 60F, Paint().apply {
            textSize = 28F
        })

        val p1r = ARect(
            (pos1.x * unscale).toInt(),
            (pos1.y * unscale).toInt(),
            (pos1.x * unscale + pos1.width * unscale).toInt(),
            (pos1.y * unscale + pos1.height * unscale).toInt()
        )

        val p2r = ARect(
            (pos2.x * unscale).toInt(),
            (pos2.y * unscale).toInt(),
            (pos2.x * unscale + pos2.width * unscale).toInt(),
            (pos2.y * unscale + pos2.height * unscale).toInt()
        )

        val p3r = ARect(
            (pos3.x * unscale).toInt(),
            (pos3.y * unscale).toInt(),
            (pos3.x * unscale + pos3.width * unscale).toInt(),
            (pos3.y * unscale + pos3.height * unscale).toInt()
        )

        val p = Paint().apply {
            style = Paint.Style.FILL
            color = if (altBoxes) 0xc0ffff00.toInt() else 0xc000ffff.toInt()
        }

        canvas.drawRect(p1r, p)
        canvas.drawRect(p2r, p)
        canvas.drawRect(p3r, p)
    }


}