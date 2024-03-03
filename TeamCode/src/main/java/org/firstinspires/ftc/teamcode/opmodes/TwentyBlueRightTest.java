package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive;
import org.firstinspires.ftc.teamcode.odo.SyncFail;
import org.firstinspires.ftc.teamcode.odo.TurnPID;

import dev.aether.collaborative_multitasking.MultitaskScheduler;

@Autonomous
public class TwentyBlueRightTest extends LinearOpMode {
    private DriveForwardPID forwardPID;
    private TurnPID turnPID;
    private RobotConfiguration robot;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        MultitaskScheduler scheduler = new MultitaskScheduler();
        KOdometryDrive odo2 = new KOdometryDrive(scheduler, robot);
        robot.clearEncoders();
        DriveForwardPID drivePID = new DriveForwardPID(robot);
        TurnPID turnPID = new TurnPID(robot);
        SyncFail why = new SyncFail(scheduler, odo2, this::opModeIsActive);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (!opModeIsActive()) return;
        // Ctrl-Slash to comment lines
        /* Left spike marker BlueRightAuto */
        drivePID.DriveReverse(21.0, telemetry);
        turnPID.TurnRobot(45.0);
        drivePID.DriveReverse(3.0, telemetry);

        turnPID.TurnRobot(-45.0);
        drivePID.strafeLeft(2, telemetry);
        drivePID.DriveReverse(25, telemetry);
        sleep(250);
        turnPID.TurnRobot(95);
        drivePID.DriveReverse(80, telemetry);
        drivePID.strafeRight(27, telemetry);

        /* Center spike marker BlueRightAuto */
        drivePID.DriveReverse(27, telemetry);

        drivePID.strafeLeft(7, telemetry);
        drivePID.DriveReverse(24, telemetry);
        turnPID.TurnRobot(95);
        drivePID.DriveReverse(87, telemetry);
        drivePID.strafeRight(22, telemetry);

        //Right spike marker BlueRightAuto
        why.driveReverse(16.0, telemetry);
        turnPID.TurnRobot(-60.0);
        turnPID.TurnRobot(60);
        why.strafeRight(4.5, telemetry);
        why.driveReverse(34, telemetry);
        sleep(250);
        turnPID.TurnRobot(90);
        why.driveReverse(78, telemetry);
        why.strafeRight(20.5, telemetry);

        while (opModeIsActive()) {
            telemetry.addData("LEFT  ", drivePID.LeftOdoDist());
            telemetry.addData("RIGHT", drivePID.RightOdoDist());
            telemetry.update();
            sleep(20);
        }
    }
}
