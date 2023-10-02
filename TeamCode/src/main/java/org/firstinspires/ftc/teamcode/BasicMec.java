package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configurations.NeoRobot1;

import java.util.concurrent.TimeUnit;

@TeleOp
public class BasicMec extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeoRobot1 config = new NeoRobot1(hardwareMap);
        waitForStart();
        int targetLeft = 0;
        int targetRight = 0;
        int[] targets = {0, 500, 750, 1000};
        ElapsedTime timer1 = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = timer1.time(TimeUnit.SECONDS);
            timer1.reset();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double fac = gamepad1.left_bumper ? 0.5 : 0.8;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * fac;
            double backLeftPower = (y - x + rx) / denominator * fac;
            double frontRightPower = (y - x - rx) / denominator * fac;
            double backRightPower = (y + x - rx) / denominator * fac;

            config.frontLeft.setPower(frontLeftPower);
            config.backLeft.setPower(backLeftPower);
            config.frontRight.setPower(frontRightPower);
            config.backRight.setPower(backRightPower);

            if (gamepad2.b) targetLeft = targets[0];
            if (gamepad2.dpad_up) targetLeft += 20;
            if (gamepad2.dpad_down) targetLeft -= 20;
            if (gamepad1.dpad_up) {
                targetLeft += 20;
                targetRight = targetLeft;
            }
            if (gamepad1.dpad_down) {
                targetLeft -= 20;
                targetRight = targetLeft;
            }
            if (targetLeft < 0) targetLeft = 0;
            if (targetLeft > 1500) targetLeft = 1500;
            if (targetRight < 0) targetRight = 0;
            if (targetRight > 1500) targetRight = 1500;

            config.slideLeft.setTargetPosition(targetLeft);
            config.slideRight.setTargetPosition(targetRight);

            sleep(5);
        }

        config.frontLeft.setPower(0);
        config.backLeft.setPower(0);
        config.frontRight.setPower(0);
        config.backRight.setPower(0);
    }
}
