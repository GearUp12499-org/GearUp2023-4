package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

@TeleOp
public class MotorPowerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);

        waitForStart();
        robot.driveMotors().setAll(-0.175);
        sleep(2000);
        robot.driveMotors().setAll(0.0);
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
