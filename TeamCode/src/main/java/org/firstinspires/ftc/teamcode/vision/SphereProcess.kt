package org.firstinspires.ftc.teamcode.vision

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import androidx.annotation.ColorInt
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgcodecs.Imgcodecs.imwrite
import org.opencv.imgproc.Imgproc


typealias CVRect = org.opencv.core.Rect
typealias ARect = android.graphics.Rect

class SphereProcess : VisionProcessor {
    private var spewControl = 0
    private val maxSpewage = 5

    enum class SectionState {
        None,
        Red,
        Blue,
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

        /**
         * at least this many more pixels of one color in a section,
         * for that section to be considered that color
         */
        private const val MAJORITY = 100
        private const val AreaFillPct = 0.2 // 20%
    }

    private var width: Int = 0
    private var height: Int = 0

    var leftSection: SectionState = SectionState.None
        private set
    var centerSection: SectionState = SectionState.None
        private set
    var rightSection: SectionState = SectionState.None
        private set

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        this.width = width
        this.height = height
        this.scratch = Mat(width, height, CvType.CV_8UC4)
        this.lastKnown = Mat(width, height, CvType.CV_8UC4)
        // We need to do a bitwise OR to look at reddish objects...
        this.red1 = Mat(width, height, CvType.CV_8U)
        this.red2 = Mat(width, height, CvType.CV_8U)
        this.blue = Mat(width, height, CvType.CV_8U)
    }

    private lateinit var lastKnown: Mat
    private lateinit var scratch: Mat
    private lateinit var red1: Mat
    private lateinit var red2: Mat
    private lateinit var blue: Mat

    private fun log(text: String) {
        if (spewControl < maxSpewage) {
            RobotLog.ii("SphereProcess", text)
        }
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        frame ?: throw IllegalStateException("passed a null frame to processFrame")
        Imgproc.cvtColor(frame, lastKnown, Imgproc.COLOR_BGR2RGB)
        val frac = 720.0 / lastKnown.width()
        Imgproc.resize(lastKnown, scratch, Size(), frac, frac)
        lastKnown = scratch.clone()
        Imgproc.cvtColor(scratch, scratch, Imgproc.COLOR_BGR2HSV)
        Core.inRange(scratch, redFrom1, redTo1, red1)
        Core.inRange(scratch, redFrom2, redTo2, red2)
        Core.add(red1, red2, red1)
        Core.inRange(scratch, blueFrom, blueTo, blue)

        // Chop off the top 1/3 and bottom 1/4
        val cropFrom = (scratch.height() / 3.0).toInt()
        val cropTo = (scratch.height() / 4.0).toInt()
        val third = (scratch.width() / 3.0).toInt()

        val firstThirdR =
            Mat(red1, CVRect(0, cropFrom, third, scratch.height() - cropFrom - cropTo))
        val secondThirdR =
            Mat(red1, CVRect(third, cropFrom, third, scratch.height() - cropFrom - cropTo))
        val thirdThirdR =
            Mat(red1, CVRect(third * 2, cropFrom, third, scratch.height() - cropFrom - cropTo))
        val firstThirdB =
            Mat(blue, CVRect(0, cropFrom, third, scratch.height() - cropFrom - cropTo))
        val secondThirdB =
            Mat(blue, CVRect(third, cropFrom, third, scratch.height() - cropFrom - cropTo))
        val thirdThirdB =
            Mat(blue, CVRect(third * 2, cropFrom, third, scratch.height() - cropFrom - cropTo))

        log("red1 @ %x".format(red1.dataAddr()))
        log("blue @ %x".format(blue.dataAddr()))

        // i don't know why this happens but they're switched
        val firstThirdBC = Core.countNonZero(firstThirdR)
        val secondThirdBC = Core.countNonZero(secondThirdR)
        val thirdThirdBC = Core.countNonZero(thirdThirdR)

        val firstThirdRC = Core.countNonZero(firstThirdB)
        val secondThirdRC = Core.countNonZero(secondThirdB)
        val thirdThirdRC = Core.countNonZero(thirdThirdB)

        leftSection = when {
            firstThirdRC - firstThirdBC > MAJORITY -> SectionState.Red
            firstThirdBC - firstThirdRC > MAJORITY -> SectionState.Blue
            else -> SectionState.None
        }
        centerSection = when {
            secondThirdRC - secondThirdBC > MAJORITY -> SectionState.Red
            secondThirdBC - secondThirdRC > MAJORITY -> SectionState.Blue
            else -> SectionState.None
        }
        rightSection = when {
            thirdThirdRC - thirdThirdBC > MAJORITY -> SectionState.Red
            thirdThirdBC - thirdThirdRC > MAJORITY -> SectionState.Blue
            else -> SectionState.None
        }

        log(
            String.format("Red : %5d %5d %5d", firstThirdRC, secondThirdRC, thirdThirdRC)
        )
        log(
            String.format("Blue: %5d %5d %5d", firstThirdBC, secondThirdBC, thirdThirdBC)
        )

        firstThirdR.release()
        secondThirdR.release()
        thirdThirdR.release()
        firstThirdB.release()
        secondThirdB.release()
        thirdThirdB.release()

        log(
            "All done: ${leftSection.name} ${centerSection.name} ${rightSection.name}"
        )
        spewControl++
        return null
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

        val third = (onscreenWidth / 3.0).toInt()
        val stripeSize = ((height / 4.0) * scaleBmpPxToCanvasPx).toInt()
        val stripeTop = onscreenHeight - stripeSize

        @ColorInt val leftCol: Int = when (leftSection) {
            SectionState.Red -> 0x80FF0000.toInt()
            SectionState.Blue -> 0x800080FF.toInt()
            else -> 0x80B0B0B0.toInt()
        }
        val leftColR: ARect = ARect(0, stripeTop, third, onscreenHeight)
        canvas.drawRect(leftColR, Paint().apply {
            color = leftCol
            strokeWidth = 0f
            style = Paint.Style.FILL
        })

        @ColorInt val centerCol: Int = when (centerSection) {
            SectionState.Red -> 0x80FF0000.toInt()
            SectionState.Blue -> 0x800080FF.toInt()
            else -> 0x80B0B0B0.toInt()
        }
        val centerColR: ARect = ARect(third, stripeTop, third * 2, onscreenHeight)
        canvas.drawRect(centerColR, Paint().apply {
            color = centerCol
            strokeWidth = 0f
            style = Paint.Style.FILL
        })

        @ColorInt val rightCol: Int = when (rightSection) {
            SectionState.Red -> 0x80FF0000.toInt()
            SectionState.Blue -> 0x800080FF.toInt()
            else -> 0x80B0B0B0.toInt()
        }
        val rightColR: ARect = ARect(third * 2, stripeTop, third * 3, onscreenHeight)
        canvas.drawRect(rightColR, Paint().apply {
            color = rightCol
            strokeWidth = 0f
            style = Paint.Style.FILL
        })
    }

    fun write() {
        val application: Context = AppUtil.getInstance().application
        application.getExternalFilesDir(null)?.let {
            val path = it.absolutePath
            RobotLog.ii("SphereProcess", "Writing to $path")
            imwrite("$path/base.png", lastKnown)
            imwrite("$path/red1.png", red1)
            imwrite("$path/red2.png", red2)
            imwrite("$path/blue.png", blue)
        }
    }
}