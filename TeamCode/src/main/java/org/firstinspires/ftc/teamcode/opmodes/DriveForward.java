package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.TurnPID;
import org.firstinspires.ftc.teamcode.utilities.MotorPowers;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

@Autonomous
public class DriveForward extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        MotorSet driveMotors = robot.driveMotors();
        DriveForwardPID pidDrive = new DriveForwardPID(robot);
        TurnPID pidTurn = new TurnPID(robot);
        robot.clearEncoders();
        waitForStart();
        if (!opModeIsActive()) return;

        pidDrive.strafeRight(36, telemetry);
//        pidTurn.TurnRobot(360, telemetry);
//        sleep(1000);
//        pidTurn.TurnRobot(-360, telemetry);

        //MotorPowers test = new MotorPowers(0.44425, -0.44777, -0.42935, 0.42583);
        //test.apply(driveMotors);

        while (opModeIsActive()) {
            telemetry.addData("LEFT  ", pidDrive.LeftOdoDist());
            telemetry.addData("RIGHT", pidDrive.RightOdoDist());
            telemetry.addData("STRAFE DIST: ", pidDrive.StrafeOdoDist());
            telemetry.update();
            sleep(20);
        }
    }
}
