package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.ext.minTicks
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.utility.typedGet
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

class DriveToTagBacking(
    hardwareMap: HardwareMap,
    private val telemetry: Telemetry,
    private val gamepad1: Gamepad
) {
    companion object {
        const val DRIVE_MOTOR_LOCK = "driveMotorLock"
        const val TARGET_DISTANCE = 10.5 //in
        const val THRESHOLD_DISTANCE = 0.5 //in
        const val THRESHOLD_ANGLE = 2.5 //degrees
        const val TARGET_TAG = 2

        const val MIN_SPEED = 0.1

        const val STOPPING_DISTANCE = 8 //in
        const val STOPPING_ANGLE = 22.5
        const val YAW_FACTOR = 0.5

        const val MAX_SPEED = 0.25

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

    private val scheduler = MultitaskScheduler()
    private val frontLeft: DcMotor = hardwareMap.typedGet("frontLeft")
    private val frontRight: DcMotor = hardwareMap.typedGet("frontRight")
    private val backLeft: DcMotor = hardwareMap.typedGet("backLeft")
    private val backRight: DcMotor = hardwareMap.typedGet("backRight")
    private val camera: CameraName = hardwareMap.typedGet("Webcam 1")

    private val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .build()
    private val visionPortal = VisionPortal.Builder()
        .setCamera(camera)
        .addProcessor(aprilTag)
        .build()

    init {
        telemetry.addData("state", "Setting up")
        telemetry.update()
        // Actually calls setDirection(), Kotlin abstracts the getter/setter to a property
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
        var detectionInfo: AprilTagDetection? = null
        // Defaults to 0.0
        var drive: Double
        var turn: Double
        var strafe: Double
        scheduler.task {
            isCompleted { ->
                aprilTag.detections.any {
                    it.id == TARGET_TAG
                }
            }
        }.chain {
            +DRIVE_MOTOR_LOCK
            onTick { ->
                detectionInfo = aprilTag.detections.firstOrNull {
                    it.id == TARGET_TAG
                }
                if (detectionInfo == null) {
                    // Pause motion and hope that it comes back in frame
                    arrayOf(frontLeft, frontRight, backLeft, backRight).forEach { motor ->
                        motor.power = 0.0
                    }
                    return@onTick // label required, returns from the onTick lambda
                }
                // !! is the non-null assertion operator
                val rangeError = detectionInfo!!.ftcPose.range - TARGET_DISTANCE
                // + rangeError => FORWARD
                // - rangeError => BACKWARD
                val headingError = detectionInfo!!.ftcPose.bearing
                // + bearing => strafe LEFT (-)
                // - bearing => strafe RIGHT (+)
                val yawError = detectionInfo!!.ftcPose.yaw
                // + yaw => turn LEFT (anti-clockwise) (-)
                // - yaw => turn RIGHT (clockwise) (+)
                telemetry.addData(
                    "RAW",
                    "Rng %5.2f Bea %5.2f Yaw %5.2f",
                    detectionInfo!!.ftcPose.range,
                    detectionInfo!!.ftcPose.bearing,
                    detectionInfo!!.ftcPose.yaw
                )

                drive = cvtRangeToPower(rangeError)
                strafe = cvtAngleToPower(-headingError)
                // hacky: prevent overturning and pushing the tag out of frame
                turn = cvtAngleToPower(-yawError) * YAW_FACTOR // + turn => cl
                telemetry.addData(
                    "Auto",
                    "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                    drive,
                    strafe,
                    turn
                )
                telemetry.addData(
                    "ERRORS",
                    "Rng %5.2f Bea %5.2f Yaw %5.2f",
                    rangeError,
                    headingError,
                    yawError
                )
                moveRobot(drive, strafe, turn)
                telemetry.update()
            }
            isCompleted { ->
                // (read this as reassigning detectionInfo to "it")
                detectionInfo?.let {
                    abs(it.ftcPose.range - TARGET_DISTANCE) < THRESHOLD_DISTANCE &&
                            abs(it.ftcPose.bearing) < THRESHOLD_ANGLE &&
                            abs(it.ftcPose.yaw) < THRESHOLD_ANGLE
                } ?: false // or, if it's null, use false instead
            }
            onFinish { ->
                // set all to 0
                arrayOf(frontLeft, frontRight, backLeft, backRight).forEach { motor ->
                    motor.power = 0.0
                }
            }
            minTicks(1)
        }
        telemetry.addData("state", "Ready")
        telemetry.update()
    }

    private fun moveRobot(drive: Double, strafe: Double, turn: Double) {
        // see GM0
        // Calculate wheel powers.
        var leftFrontPower = drive + strafe + turn
        var leftBackPower = drive - strafe + turn
        var rightFrontPower = drive - strafe - turn
        var rightBackPower = drive + strafe - turn

        // Normalize wheel speeds.
        val denominator = maxOf(
            abs(leftFrontPower),
            abs(rightFrontPower),
            abs(leftBackPower),
            abs(rightBackPower)
        )
        if (denominator > 1) {
            leftFrontPower /= denominator
            rightFrontPower /= denominator
            leftBackPower /= denominator
            rightBackPower /= denominator
        }
        frontLeft.power = leftFrontPower
        frontRight.power = rightFrontPower
        backLeft.power = leftBackPower
        backRight.power = rightBackPower
        telemetry.addData(
            "Powers",
            "FL %5.2f FR %5.2f BL %5.2f BR %5.2f",
            leftFrontPower,
            rightFrontPower,
            leftBackPower,
            rightBackPower
        )
    }

    fun start(opModeIsActive: () -> Boolean) {
        // Finish tasks OR stop the OpMode OR press B to stop the event loop
        while (scheduler.hasJobs() && opModeIsActive() && !gamepad1.b) {
            scheduler.tick()
        }
        visionPortal.close()
    }
}
