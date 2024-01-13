package org.firstinspires.ftc.teamcode.odo

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.Task
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
        }
    }

    companion object {
        @JvmStatic
        fun odoTicksToDistance(ticks: Int): Double {
            val tpr = 8192.0 // Ticks Per Revolution
            val radius = 0.69.inches
            val inchPerTick = radius.to.inches / tpr
            // we really don't need the units here
            return (inchPerTick * ticks).value
        }
    }
}