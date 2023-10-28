package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DumperTestBench extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo dumperRotate = hardwareMap.get(Servo.class, "dumperRotate");
        Servo dumperLatch = hardwareMap.get(Servo.class, "dumperLatch");
        ElapsedTime timer = new ElapsedTime();
        final double SPEED_PER_SECOND = 0.25;
        waitForStart();
        double rR = 0.5;
        double rL = 0.5;
        timer.reset();
        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();
            double mag = dt * SPEED_PER_SECOND;

            if (gamepad1.dpad_up) {
                rR += mag;
            }
            if (gamepad1.dpad_down) {
                rR -= mag;
            }
            rR = clamp(rR, 0, 1);

            if (gamepad1.dpad_right) {
                rL += mag;
            }
            if (gamepad1.dpad_left) {
                rL -= mag;
            }
            rL = clamp(rL, 0, 1);

            dumperRotate.setPosition(rR);
            dumperLatch.setPosition(rL);
            telemetry.addData("rotate", rR);
            telemetry.addData("latch", rL);
            telemetry.update();
        }
    }
}
