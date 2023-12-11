package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;

@Autonomous
public class ScoreCenter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        robot.clearEncoders();
        DriveForwardPID driveForward = new DriveForwardPID(robot);
        waitForStart();
        if (!opModeIsActive()) return;
        driveForward.DriveReverse(27.0, telemetry);
        robot.purpleDropper().setPosition(Var.PixelDropper.down);
        sleep(2000);
        robot.purpleDropper().setPosition(Var.PixelDropper.back);

        while (opModeIsActive()) {
            telemetry.addData("TOTAL E", driveForward.sumOfError);
            telemetry.update();
            sleep(50);
        }
    }
}
