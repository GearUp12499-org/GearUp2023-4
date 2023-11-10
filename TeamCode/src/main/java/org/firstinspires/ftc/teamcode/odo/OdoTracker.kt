package org.firstinspires.ftc.teamcode.odo

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.inches
import java.util.concurrent.TimeUnit

class OdoTracker(val odoPerp: DcMotor, val odoPara1: DcMotor, val odoPara2: DcMotor, val origin: Pose) {
    val r1: LengthUnit = 8.inches
    val d: LengthUnit = 8.inches
    val minTimeBetweenReads = 100.0 // ms

    private var poseBacker: Pose = origin
    @Suppress("MemberVisibilityCanBePrivate")
    var currentPose: Pose
        private set(value) {poseBacker = value}
        get() = poseBacker

    val provisionTask: Task.() -> Unit = {
        // don't contribute to 'finished' checks
        daemon = true
        val timer = ElapsedTime()
        var lastY: Double? = null
        var lastX1: Double? = null
        var lastX2: Double? = null
        onStart { ->
            currentPose = origin
            timer.reset()
        }
        onTick { ->
            val delta = timer.time(TimeUnit.MILLISECONDS)
            if (delta < minTimeBetweenReads) return@onTick
            timer.reset()
            val currentYRaw = odoPerp.currentPosition
            val currentX1Raw = odoPara1.currentPosition
            val currentX2Raw = odoPara2.currentPosition
            if (lastY == null || lastX1 == null || lastX2 == null) {
                lastY = currentYRaw.toDouble()
                lastX1 = currentX1Raw.toDouble()
                lastX2 = currentX2Raw.toDouble()
                return@onTick
            }
            val cY = odoTicksToDistance(currentYRaw - lastY!!)
            val cX1 = odoTicksToDistance(currentX1Raw - lastX1!!)
            val cX2 = odoTicksToDistance(currentX2Raw - lastX2!!)

            val alpha = (cX1 + cX2) / 2.0
            val yd = (cX1 - cX2) / 2.0
            val gamma = yd / d.to.inches.value
            // cY - r1 * Y
            val beta = cY - (r1.to.inches.value * gamma)

            currentPose += Pose(alpha.inches, beta.inches, gamma.radians)
        }
    }

    companion object {
        @JvmStatic
        fun odoTicksToDistance(ticks: Double): Double {
            val tpr = 8192.0 // Ticks Per Revolution
            val radius = 0.69.inches
            val inchPerTick = radius.to.inches / tpr
            // we really don't need the units here
            return (inchPerTick * ticks).value
        }
    }
}