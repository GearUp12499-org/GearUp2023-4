package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range.clip
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
        const val TARGET_DISTANCE = 8 //in
        const val THRESHOLD_DISTANCE = 0.5 //in
        const val THRESHOLD_ANGLE = 5 //degrees
        const val TARGET_TAG = 2

        // steal that power function
        const val SPEED_GAIN = 0.02
        const val TURN_GAIN = 0.01
        const val STRAFE_GAIN = 0.015

        const val MAX_SPEED = 0.5
        const val MAX_TURN = 0.3
        const val MAX_STRAFE = 0.5
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
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE
        var detectionInfo: AprilTagDetection? = null
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
                if (detectionInfo == null) return@onTick
                val rangeError = detectionInfo!!.ftcPose.range - TARGET_DISTANCE
                // + rangeError => FORWARD
                // - rangeError => BACKWARD
                val headingError = detectionInfo!!.ftcPose.bearing
                // + bearing => strafe LEFT
                // - bearing => strafe RIGHT
                val yawError = detectionInfo!!.ftcPose.yaw
                telemetry.addData(
                    "RAW",
                    "Rng %5.2f Bea %5.2f Yaw %5.2f",
                    detectionInfo!!.ftcPose.range,
                    detectionInfo!!.ftcPose.bearing,
                    detectionInfo!!.ftcPose.yaw
                )

                drive = clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED)
                strafe = clip(headingError * STRAFE_GAIN, -MAX_TURN, MAX_TURN)
                turn = clip(-yawError * TURN_GAIN, -MAX_STRAFE, MAX_STRAFE)
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
                detectionInfo?.let {
                    abs(it.ftcPose.range - TARGET_DISTANCE) < THRESHOLD_DISTANCE &&
                            abs(it.ftcPose.bearing) < THRESHOLD_ANGLE &&
                            abs(it.ftcPose.yaw) < THRESHOLD_ANGLE
                } ?: true
            }
            onFinish { ->
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

        // Calculate wheel powers.
        var leftFrontPower = drive - strafe - turn
        var rightFrontPower = drive + strafe + turn
        var leftBackPower = drive + strafe - turn
        var rightBackPower = drive - strafe + turn

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
//        frontLeft.power = leftFrontPower
//        frontRight.power = rightFrontPower
//        backLeft.power = leftBackPower
//        backRight.power = rightBackPower
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
        while (scheduler.hasJobs() && opModeIsActive() && !gamepad1.b) {
            scheduler.tick()
        }
    }
}