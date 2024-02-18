package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ActuateIntakeDropDown extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo dumperRotate = hardwareMap.get(Servo.class, "dropDown");
        ElapsedTime timer = new ElapsedTime();
        final double SPEED_PER_SECOND = 0.25;
        waitForStart();
        double rR = 0.5;
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

            dumperRotate.setPosition(rR);
            telemetry.addData("rotate", rR);
            telemetry.update();
        }
    }
}
