package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.configurations.NeoRobot1;

import java.util.concurrent.TimeUnit;

import dev.aether.collaborative_multitasking.MultitaskScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private double max(double a, double... others) {
        if (others.length == 0) return a;
        double[] combined = new double[others.length + 1];
        combined[0] = a;
        Double max = null;
        System.arraycopy(others, 0, combined, 1, others.length);
        for (int i = 0; i < combined.length - 1; i++) {
            double first = max != null ? max : combined[i];
            double second = combined[i + 1];
            max = Math.max(first, second);
        }
        return max;
    }

    @Override
    public void runOpMode() {
        MultitaskScheduler scheduler = new MultitaskScheduler();
        Servo clawGrip = hardwareMap.get(Servo.class, "clawGrip");
        Servo clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        Claw claw = new Claw(scheduler, clawGrip, clawRotate);
        NeoRobot1 config = new NeoRobot1(hardwareMap);
        waitForStart();
        int targetLeft = 0;
        int targetRight = 0;
        int[] targets = {0, 500, 750, 1000};

        double balanceLeft = 1.00;
        double balanceRight = 1.00;
        double balanceFront = 1.00;
        double balanceBack = 1.00;

        double balFL = balanceFront * balanceLeft;
        double balFR = balanceFront * balanceRight;
        double balBL = balanceBack * balanceLeft;
        double balBR = balanceBack * balanceRight;
        double balanceDen = max(Math.abs(balFL), Math.abs(balFR), Math.abs(balBL), Math.abs(balBR), 1);
        balFL /= balanceDen;
        balFR /= balanceDen;
        balBL /= balanceDen;
        balBR /= balanceDen;

        ElapsedTime timer1 = new ElapsedTime();
        while (opModeIsActive()) {
            scheduler.tick();
            double dt = timer1.time(TimeUnit.SECONDS);
            timer1.reset();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double fac = gamepad1.left_bumper ? 1 : 0.75;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * fac;
            double backLeftPower = (y - x + rx) / denominator * fac;
            double frontRightPower = (y - x - rx) / denominator * fac;
            double backRightPower = (y + x - rx) / denominator * fac;

            int SLIDE_LIM = 3000;
            int MOTION_PER_CYCLE = 20;

            config.frontLeft.setPower(frontLeftPower * balFL);
            config.backLeft.setPower(backLeftPower * balBL);
            config.frontRight.setPower(frontRightPower * balFR);
            config.backRight.setPower(backRightPower * balBR);

            if (gamepad2.b) {
                targetLeft = targets[0];
                targetRight = targets[0];
            }
            if (gamepad2.dpad_up) targetLeft += MOTION_PER_CYCLE;
            if (gamepad2.dpad_down) targetLeft -= MOTION_PER_CYCLE;
            if (gamepad1.dpad_up) {
                targetRight += MOTION_PER_CYCLE;
                targetLeft = targetRight;
            }
            if (gamepad1.dpad_down) {
                targetRight -= MOTION_PER_CYCLE;
                targetLeft = targetRight;
            }
            if (targetLeft < 0) targetLeft = 0;
            if (targetLeft > SLIDE_LIM) targetLeft = SLIDE_LIM;
            if (targetRight < 0) targetRight = 0;
            if (targetRight > SLIDE_LIM) targetRight = SLIDE_LIM;

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
