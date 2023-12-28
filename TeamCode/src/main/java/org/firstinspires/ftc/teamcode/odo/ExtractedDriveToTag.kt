package org.firstinspires.ftc.teamcode.odo

import android.util.Log
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.tagPositions
import org.firstinspires.ftc.teamcode.utilities.Move
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.Vector2
import org.firstinspires.ftc.teamcode.utilities.degrees
import org.firstinspires.ftc.teamcode.utilities.inches
import kotlin.math.sqrt

class ExtractedDriveToTag(
    private val robot: RobotConfiguration,
    private val tagTool: AprilTagUpdateTool,
    private val odoTracker: OdoTracker,
    var targetPosition: Int,
    private val telemetry: Telemetry
) {
    companion object {
        private val tagOffset = Vector2(0.0, -20.0)
        val tagTargets = mapOf(
            1 to tagPositions[1]!! + tagOffset,
            2 to tagPositions[2]!! + tagOffset,
            3 to tagPositions[3]!! + tagOffset,
        )
        val xOkWithin = 1.inches
        val yOkPreciseWithin = 1.inches
        const val ROBOT_SIZE = 20.794 // in
        val ramp = ControlRamps(
            0.2,
            0.4,
            0.0,
            24.0
        )
    }

    private val motors = robot.driveMotors()

    val taskFactory: Task.() -> Unit = {
        +robot.driveMotorLock
        val target = Pose(
            tagTargets[targetPosition]!!,
            (-90.0).degrees
        )
        onTick { ->
            if (!tagTool.acquired) {
                telemetry.addLine("no tag acquired yet")
                telemetry.update()
                return@onTick
            }

            val pose = odoTracker.currentPose
            telemetry.addData("current est", pose)
            telemetry.addData("target", target)
            val fromTo = pose.toPose(target)
            telemetry.addData("from/to", fromTo)
            val distance =
                sqrt(
                    fromTo.forward.value * fromTo.forward.value
                            + fromTo.right.value * fromTo.right.value
                )
            val motion =
                fromTo.getSpeeds(ROBOT_SIZE) * ramp.ramp(
                    0.0,
                    distance
                )
            telemetry.addData("speeds", motion)
            val correctedMotion = motion.map(Move::ramp)
            telemetry.addData("powers", correctedMotion)
            correctedMotion.apply(motors)
            telemetry.update()
            Log.i(
                "NewDriveToTag",
                String.format(
                    "Current est: %s\nTarget     : %s\nFrom/To    : %s\nSpeeds    : %s\nPowers    : %s",
                    pose, target, fromTo, motion, correctedMotion
                )
            )
        }
        isCompleted { ->
            (odoTracker.currentPose.x - target.x).abs < xOkWithin && (odoTracker.currentPose.y - target.y).abs < yOkPreciseWithin
        }
        onFinish { ->
            motors.setAll(0.0)
        }
    }
}