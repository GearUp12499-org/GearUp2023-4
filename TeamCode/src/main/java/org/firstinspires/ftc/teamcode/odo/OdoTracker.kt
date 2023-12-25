package org.firstinspires.ftc.teamcode.odo

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.inches
import org.firstinspires.ftc.teamcode.utilities.radians

class OdoTracker(
    robot: RobotConfiguration,
    private val origin: Pose,
) {
    private val odoPerp: DcMotor = robot.odoPerpendicular()
    private val odoPara1: DcMotor = robot.odoParallel1()
    private val odoPara2: DcMotor = robot.odoParallel2()
    private val r1: LengthUnit = (-8).inches // +/- 0.25in
    private val d: LengthUnit = 7.25.inches  // +/- 0.25in
    private val minTimeBetweenReads = 10.0 // ms

    private var poseBacker: Pose = origin

    @Suppress("MemberVisibilityCanBePrivate")
    var currentPose: Pose
        private set(value) {
            poseBacker = value
        }
        get() = poseBacker

    val taskFactory: Task.() -> Unit = {
        // don't contribute to 'finished' checks
        daemon = true
        val timer = ElapsedTime()
        var lastY = odoPerp.currentPosition
        var lastX1 = odoPara1.currentPosition
        var lastX2 = odoPara2.currentPosition
        val dIn = d.to.inches.value
        val rIn = r1.to.inches.value
        onStart { ->
            currentPose = origin
            // In case of executing in .then(...) (why would we do this?) the current values may
            // not match the values at the time the task was created, because the canStart
            // conditions can be modified after the task is created.
            lastY = odoPerp.currentPosition
            lastX1 = odoPara1.currentPosition
            lastX2 = odoPara2.currentPosition
            timer.reset()
        }
        onTick { ->
            val delta = timer.time() * 1000.0
            if (delta < minTimeBetweenReads) return@onTick
            timer.reset()
            val currentYRaw = odoPerp.currentPosition
            val currentX1Raw = odoPara1.currentPosition
            val currentX2Raw = odoPara2.currentPosition
            val cY = odoTicksToDistance(currentYRaw - lastY)
            val cX1 = odoTicksToDistance(currentX1Raw - lastX1)
            val cX2 = odoTicksToDistance(currentX2Raw - lastX2)

            val forward = (cX1 + cX2) / 2.0
            val yd = (cX1 - cX2) / 2.0
            val rotate = yd / dIn
            // cY + r1 * Y
            val right = cY + (rIn * rotate)
            lastY = currentYRaw
            lastX1 = currentX1Raw
            lastX2 = currentX2Raw

            currentPose = currentPose.transform(forward.inches, right.inches, rotate.radians)
        }
    }

    companion object {
        @JvmStatic
        fun odoTicksToDistance(ticks: Number): Double {
            val tpr = 8192.0 // Ticks Per Revolution
            val radius = 0.69.inches
            val inchPerTick = radius/* .to.inches */ / tpr
            // we really don't need the units here
            return (inchPerTick * ticks.toDouble()).value
        }
    }
}