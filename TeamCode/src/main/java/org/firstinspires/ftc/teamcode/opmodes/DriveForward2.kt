package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID
import org.firstinspires.ftc.teamcode.odo.EncoderMath.tick2inch
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive
import org.firstinspires.ftc.teamcode.odo.TurnPID
import org.firstinspires.ftc.teamcode.utilities.feet

@TeleOp
class DriveForward2 : LinearOpMode() {
    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        robot.clearEncoders()
        val drive = KOdometryDrive(scheduler, robot)
        val reference = DriveForwardPID(robot)
        val rotateReference = TurnPID(robot)
        waitForStart()
        drive.strafeLeft(2.feet);
        sleep(100)

        try {
            scheduler.runToCompletion(::opModeIsActive)
            RobotLog.i(scheduler.statsheet())
        } catch (ignore: IndexOutOfBoundsException) {

        }
        while (opModeIsActive()) {
            telemetry.addData("Left     odo", tick2inch(robot.odoParallelLeft().currentPosition))
            telemetry.addData("Right  odo", tick2inch(robot.odoParallelRight().currentPosition))
            telemetry.addData("Strafe odo", tick2inch(robot.odoPerpendicular().currentPosition))
            telemetry.update()
            sleep(20)
        }
    }
}