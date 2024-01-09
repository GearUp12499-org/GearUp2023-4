package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive
import org.firstinspires.ftc.teamcode.odo.TurnPID

@TeleOp
class DriveForward2 : LinearOpMode() {
    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        val drive = KOdometryDrive(scheduler, robot)
        val reference = DriveForwardPID(robot)
        val rotateReference = TurnPID(robot)
        waitForStart()
        rotateReference.TurnRobot(90.0, telemetry)
        sleep(100)

        scheduler.runToCompletion(::opModeIsActive)
        while (opModeIsActive()) {
            sleep(20)
        }
    }
}