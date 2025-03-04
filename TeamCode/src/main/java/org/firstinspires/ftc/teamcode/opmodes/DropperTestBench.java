package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DropperTestBench extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo purpleDropper = hardwareMap.get(Servo.class, "purpleDropper");
        ElapsedTime timer = new ElapsedTime();
        final double SPEED_PER_SECOND = 0.25;
        waitForStart();
        double rR = 0.5;
        double rG = 0.5;
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
                rG += mag;
            }
            if (gamepad1.dpad_left) {
                rG -= mag;
            }
            rG = clamp(rG, 0, 1);

            purpleDropper.setPosition(rR);
            telemetry.addData("dropper", rR);
            telemetry.addData("Not Used", rG);
            telemetry.update();
        }
    }
}
