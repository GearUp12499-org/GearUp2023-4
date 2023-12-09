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
        TurnPID pidTurn = new TurnPID(robot);
        robot.clearEncoders();
        waitForStart();
        //pidDrive.DriveForward(-27.0, telemetry);
        //pidTurn.TurnRobot(180);
        while (opModeIsActive()) {
            telemetry.addData("LEFT  ", pidTurn.LeftOdoDist());
            telemetry.addData("RIGHT", pidTurn.RightOdoDist());
            telemetry.update();
            sleep(20);
        }
    }
}
