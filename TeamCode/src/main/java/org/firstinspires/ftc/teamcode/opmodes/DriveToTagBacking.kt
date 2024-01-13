package org.firstinspires.ftc.teamcode.opmodes

import android.util.Size
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.ext.minTicks
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.detectPairToPose
import org.firstinspires.ftc.teamcode.detectSingleToPose
import org.firstinspires.ftc.teamcode.tagPositions
import org.firstinspires.ftc.teamcode.utilities.CollectionUtils.pairs
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.degrees
import org.firstinspires.ftc.teamcode.utilities.inches
import org.firstinspires.ftc.teamcode.utilities.radians
import org.firstinspires.ftc.teamcode.utilities.typedGet
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

class DriveToTagBacking(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry,
    private val gamepad1: Gamepad
) {
    companion object {
        const val TARGET_DISTANCE = 10.5 //in
        const val THRESHOLD_DISTANCE = 0.5 //in
        const val THRESHOLD_ANGLE = 2.5 //degrees
        const val ROBOT_SIZE = 20.794 // in

        const val MIN_SPEED = 0.1

        const val STOPPING_DISTANCE = 8 //in
        const val STOPPING_ANGLE = 22.5
        const val YAW_FACTOR = 0.5

        const val MAX_SPEED = 0.25

        val TargetPosition = Pose(tagPositions[2]!!, 0.radians) + Pose(
            -(STOPPING_DISTANCE).inches,
            0.inches,
            90.degrees
        )

        fun cvtRangeToPower(distance: Double): Double {
            if (distance < 0) return -cvtRangeToPower(-distance)

            if (distance < THRESHOLD_DISTANCE) return 0.0
            if (distance >= STOPPING_DISTANCE) return MAX_SPEED

            val rampDistance = distance - THRESHOLD_DISTANCE
            val gain = (MAX_SPEED - MIN_SPEED) / (STOPPING_DISTANCE - THRESHOLD_DISTANCE)
            return gain * rampDistance + MIN_SPEED
        }

        fun cvtAngleToPower(angle: Double): Double {
            if (angle < 0) return -cvtAngleToPower(-angle)

            if (angle < THRESHOLD_ANGLE) return 0.0
            if (angle >= STOPPING_ANGLE) return MAX_SPEED

            val rampAngle = angle - THRESHOLD_ANGLE
            val gain = (MAX_SPEED - MIN_SPEED) / (STOPPING_ANGLE - THRESHOLD_ANGLE)
            return gain * rampAngle + MIN_SPEED
        }
    }

    private val configuration = RobotConfiguration.currentConfiguration()(hardwareMap)

    private val scheduler = MultitaskScheduler()
    private val driveMotors = configuration.driveMotors()
    private val camera: CameraName = hardwareMap.typedGet("Webcam 1")

    private val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .build()
    private val visionPortal = VisionPortal.Builder()
        .setCamera(camera)
        .addProcessor(aprilTag)
        .setCameraResolution(Size(864, 480))
        .build()

    init {
        telemetry.addData("state", "Setting up")
        telemetry.update()

        var est: Pose? = null
        scheduler.task {
            +configuration.driveMotorLock
            onTick { ->
                val detections = aprilTag.detections
                val positionEstimates = mutableListOf<Pose>()
                for (pair in pairs(detections)) {
                    positionEstimates.add(
                        detectPairToPose(pair.first, pair.second)
                    )
                }
                for (detection in detections) {
                    positionEstimates.add(
                        detectSingleToPose(detection)
                    )
                }

                if (detections.size == 0) {
                    driveMotors.setAll(0.0)
//                    telemetry.addData("Status", "Tags Lost")
//                    telemetry.update()
                    return@onTick
                }

                val average = positionEstimates.reduce { a, b -> a + b } / positionEstimates.size
                telemetry.addData("Status", "OK")
                telemetry.addData("PoseEst", average.toString())
                est = average
                val fromTo = average.toPose(TargetPosition)
                telemetry.addData("Move", fromTo.toString())
                val motion = fromTo.getSpeeds(ROBOT_SIZE)
                telemetry.addData("Power", motion.toString())

//                motion.apply(driveMotors)
                telemetry.update()
            }
            isCompleted { ->
                // (read this as reassigning est to "it" if it isn't nul.)
                est?.let {
                    val k = (it - TargetPosition)
                    (abs(k.x.value) <= THRESHOLD_DISTANCE)
                            && (abs(k.y.value) <= THRESHOLD_DISTANCE)
                            && (abs(k.theta.value) <= THRESHOLD_ANGLE.degrees.to.radians.value)
                } ?: false // or, if it's null, use false instead
            }
            onFinish { ->
                // set all to 0
                driveMotors.setAll(0.0)
            }
            minTicks(1)
        }
        telemetry.addData("state", "Ready")
        telemetry.update()
    }

    fun start(opModeIsActive: () -> Boolean) {
        // Finish tasks OR stop the OpMode OR press B to stop the event loop
        while (scheduler.hasJobs() && opModeIsActive() && !gamepad1.b) {
            scheduler.tick()
        }
        visionPortal.close()
    }
}
