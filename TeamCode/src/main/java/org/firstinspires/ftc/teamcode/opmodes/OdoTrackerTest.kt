package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.OdoTracker
import org.firstinspires.ftc.teamcode.utilities.Pose

@Disabled
@TeleOp
class OdoTrackerTest : LinearOpMode() {
    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        val tracker = OdoTracker(robot, Pose.zero)
        RobotLog.ii("Hello", "Hello, world!")

        waitForStart()
        if (!opModeIsActive()) return

        scheduler.task(tracker.taskFactory)
        scheduler.task {
            daemon = true
            onTick { ->
                telemetry.addData("X", tracker.currentPose.x)
                telemetry.addData("Y", tracker.currentPose.y)
                telemetry.addData("θ", tracker.currentPose.theta)
                telemetry.update()
            }
        }
        scheduler.task {
            isCompleted { -> !(opModeIsActive() && !gamepad1.a) }
            onFinish { ->
                RobotLog.ii("Results", scheduler.statsheet())
            }
        }
        scheduler.runToCompletion { true }
        while (opModeIsActive()) sleep(20)
    }
}