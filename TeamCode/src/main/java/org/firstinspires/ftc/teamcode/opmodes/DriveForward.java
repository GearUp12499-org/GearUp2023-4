package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

@Autonomous
public class DriveForward extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        MotorSet driveMotors = robot.driveMotors();
        DriveForwardPID pidDrive = new DriveForwardPID(robot);

        waitForStart();
        pidDrive.DriveForward(48.0, telemetry);

        while (opModeIsActive()) {
            telemetry.addData("LEFT  ", pidDrive.LeftOdoDist());
            telemetry.addData("RIGHT", pidDrive.RightOdoDist());
            telemetry.update();
            sleep(20);
        }
    }
}