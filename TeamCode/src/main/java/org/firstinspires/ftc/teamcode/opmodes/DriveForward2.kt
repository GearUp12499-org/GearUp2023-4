package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive
import org.firstinspires.ftc.teamcode.utilities.feet

@TeleOp
class DriveForward2 : LinearOpMode() {
    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        val drive = KOdometryDrive(scheduler, robot)
        waitForStart()
        drive.strafeLeft(1.feet)
//            .then(drive.driveReverse(6.feet, 5.0))

        scheduler.runToCompletion(::opModeIsActive)
        while (opModeIsActive()) {
            sleep(20)
        }
    }
}