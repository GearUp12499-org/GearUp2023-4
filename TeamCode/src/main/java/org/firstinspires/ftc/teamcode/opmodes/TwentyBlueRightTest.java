package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.TurnPID;

@Autonomous
public class TwentyBlueRightTest extends LinearOpMode{
    private DriveForwardPID forwardPID;
    private TurnPID turnPID;
    private RobotConfiguration robot;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        robot.clearEncoders();
        DriveForwardPID drivePID = new DriveForwardPID(robot);
        TurnPID turnPID = new TurnPID(robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (!opModeIsActive()) return;
        /*Left spike marker BlueRightAuto
        /drivePID.DriveReverse(21.0, telemetry);
        turnPID.TurnRobot(45.0, telemetry);
        drivePID.DriveReverse(3.0, telemetry);
        turnPID.TurnRobot(-45.0, telemetry);
        drivePID.strafeLeft(2, telemetry);
        drivePID.DriveReverse(25, telemetry);
        sleep(250);
        turnPID.TurnRobot(95, telemetry);
        drivePID.DriveReverse(80, telemetry);
        drivePID.strafeRight(27, telemetry);*/

        /*Center spike marker BlueRightAuto
        drivePID.DriveReverse(27, telemetry);
        drivePID.strafeLeft(7, telemetry);
        drivePID.DriveReverse(24, telemetry);
        turnPID.TurnRobot(95, telemetry);
        drivePID.DriveReverse(87, telemetry);
        drivePID.strafeRight(22, telemetry);*/

        //Right spike marker BlueRightAuto
        drivePID.DriveReverse(16.0, telemetry);
        turnPID.TurnRobot(-60.0, telemetry);
        turnPID.TurnRobot(60, telemetry);
        drivePID.strafeRight(4.5, telemetry);
        drivePID.DriveReverse(34, telemetry);
        sleep(250);
        turnPID.TurnRobot(90, telemetry);
        drivePID.DriveReverse(78, telemetry);
        drivePID.strafeRight(20.5, telemetry);

        while (opModeIsActive()) {
            telemetry.addData("LEFT  ", drivePID.LeftOdoDist());
            telemetry.addData("RIGHT", drivePID.RightOdoDist());
            telemetry.update();
            sleep(20);
        }
    }
}
