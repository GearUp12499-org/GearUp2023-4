package org.firstinspires.ftc.teamcode.odo

import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.minTicks
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.detectSingleToPose
import org.firstinspires.ftc.teamcode.tagPositions
import org.firstinspires.ftc.teamcode.utilities.Move
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.Vector2
import org.firstinspires.ftc.teamcode.utilities.degrees
import org.firstinspires.ftc.teamcode.utilities.inches
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.sqrt

class AprilTagApproach(
    private val source: AprilTagProcessor,
    robot: RobotConfiguration,
    var initialTarget: Int,
    private val telemetry: Telemetry
) {
    private val motors = robot.driveMotors()

    companion object {
        private val tagOffset = Vector2(0.0, -8.0)
        val tagTargets = mapOf(
            1 to tagPositions[1]!! + tagOffset,
            2 to tagPositions[2]!! + tagOffset,
            3 to tagPositions[3]!! + tagOffset,
        )
        val xOkWithin = 1.inches
        val yOkWithin = 6.inches
        val yOkPreciseWithin = 1.inches
        const val ROBOT_SIZE = 20.794 // in
        val ramp = ControlRamps(
            0.2,
            0.4,
            0.0,
            24.0
        )
    }

    val task: Task.() -> Unit = {
        +robot.driveMotorLock
        var lockedTarget = initialTarget
        var targetPosition = Pose(tagTargets[lockedTarget]!!, (-90).degrees)
        var visible = false
        var lastEstimate: Pose? = null
        onStart { ->
            lockedTarget = initialTarget
            targetPosition = Pose(tagTargets[lockedTarget]!!, (-90).degrees)
        }
        onTick { ->
            val detections = source.detections
            var est: Pose? = null
            for (detection in detections) {
                if (detection.id == lockedTarget) {
                    telemetry.addData("Yaw (Z deg)", detection.ftcPose.yaw)
                    est = detectSingleToPose(detection)
                }
            }
            visible = est != null
            if (!visible) {
                telemetry.addLine("what tag?")
                telemetry.update()
                // uh oh
                motors.setAll(0.0)
                return@onTick
            }
            lastEstimate = est

            telemetry.addData("current est", est)
            telemetry.addData("target", targetPosition)

            val poseFromTo = est!!.toPose(targetPosition)
            telemetry.addData("from/to", poseFromTo)
            val distance =
                sqrt(poseFromTo.forward.value * poseFromTo.forward.value + poseFromTo.right.value * poseFromTo.right.value)
            val motion = poseFromTo.getSpeeds(ROBOT_SIZE) * ramp.ramp(0.0, distance)
            val correctedMotion = motion.map(Move::ramp)
            telemetry.addData("motor", correctedMotion)
            correctedMotion.apply(motors)
            telemetry.update()
        }
        isCompleted { ->
            lastEstimate?.let {
                (lastEstimate!!.x - targetPosition.x).abs < xOkWithin && (
                        (!visible
                                && (lastEstimate!!.y - targetPosition.y).abs < yOkWithin) ||
                                (visible && (lastEstimate!!.y - targetPosition.y).abs < yOkPreciseWithin)
                        )
            } ?: false
        }
        onFinish { ->
            motors.setAll(0.0)
        }
        minTicks(1)
    }
}