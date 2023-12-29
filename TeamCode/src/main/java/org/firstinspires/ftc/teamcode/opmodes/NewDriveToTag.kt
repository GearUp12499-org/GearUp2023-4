package org.firstinspires.ftc.teamcode.opmodes

import android.util.Log
import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Var
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.AprilTagUpdateTool
import org.firstinspires.ftc.teamcode.odo.ControlRamps
import org.firstinspires.ftc.teamcode.odo.OdoTracker
import org.firstinspires.ftc.teamcode.tagPositions
import org.firstinspires.ftc.teamcode.utilities.Move
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.Vector2
import org.firstinspires.ftc.teamcode.utilities.degrees
import org.firstinspires.ftc.teamcode.utilities.inches
import org.firstinspires.ftc.teamcode.utilities.typedGet
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.sqrt

@TeleOp
class NewDriveToTag : LinearOpMode() {
    private var portal: VisionPortal? = null
    private var aprilTag: AprilTagProcessor? = null

    companion object {
        private val tagOffset = Vector2(0.0, -20.0)
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

    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        initVisionPortal(hardwareMap.typedGet("Webcam 1"))
        val tagMotion = AprilTagUpdateTool(aprilTag!!, 2)
        val odo = OdoTracker(robot, Var.AutoPositions.BlueLeft)
        val motors = robot.driveMotors()

        scheduler.task(odo.taskFactory)
        val toolTask = scheduler.task(tagMotion.updateTool(odo))
        scheduler.task {
            +robot.driveMotorLock
            val target = Pose(tagTargets[2]!! + tagOffset, (-90.0).degrees)
            onTick { ->
//                if (!tagMotion.acquired) return@onTick

                val pose = odo.currentPose
                telemetry.addData("current est", pose)
                telemetry.addData("target", target)

                // This turns the From pose and the To pose into a Move.
                // THIS IS THE PROBLEM PART.
                //                vvvvvv
                val fromTo = pose.toPose(target)
                telemetry.addData("from/to", fromTo)

                // Pythagorean theorem.
                val distance =
                    sqrt(
                        fromTo.forward.value * fromTo.forward.value
                                + fromTo.right.value * fromTo.right.value
                    )

                // THIS IS THE PROBLEM PART. (2)
                //                  vvvvvvvvv
                val motion = fromTo.getSpeeds(ROBOT_SIZE) * ramp.ramp(0.0, distance)
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
                (odo.currentPose.x - target.x).abs < xOkWithin && (odo.currentPose.y - target.y).abs < yOkPreciseWithin
            }
            onFinish { ->
                motors.setAll(0.0)
            }
        }

        waitForStart()
        if (!opModeIsActive()) return
        scheduler.runToCompletion(::opModeIsActive)
        while (opModeIsActive()) {
            sleep(20)
        }
    }

    private fun initVisionPortal(webcam: WebcamName) {
        aprilTag = AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build()
        val visionPortalBuilder = VisionPortal.Builder()
        visionPortalBuilder.setCamera(webcam)
        // Medium-sized 16:9ish. See-also builtinwebcamcalibrations.xml
        // Logitech HD Pro Webcam C920
        // use Search Anything in "include non-project" mode to find
        visionPortalBuilder.setCameraResolution(Size(864, 480))
        visionPortalBuilder.addProcessor(aprilTag)
        portal = visionPortalBuilder.build()
    }
}