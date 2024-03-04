package org.firstinspires.ftc.teamcode.opmodes;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

@TeleOp
public class ActuateIntakeDropDown extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        Servo dropDownServo = robot.dropDownServo();
        DcMotor intake = robot.intake();
        CRServo intakeAssist = hardwareMap.get(CRServo.class, "intakeAssist");

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

            double iSpeed = 0.0;
            if (gamepad1.left_bumper) {
                iSpeed = Var.TeleOp.intakePower;
            } else if (gamepad1.right_bumper) {
                iSpeed = -Var.TeleOp.intakePower;
            }
            intake.setPower(iSpeed);
            intakeAssist.setPower(-Math.signum(iSpeed));

            dropDownServo.setPosition(rR);
            telemetry.addData("rotate", rR);
            telemetry.update();
        }
    }
}
