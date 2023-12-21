package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

// Parallel 1       -46289
// Parallel 2       +39138
// Perpendicular    -32947

// Parallel 1       -45182
// Parallel 2       +40068
// Perpendicular    -34579

@TeleOp
public class ReadOdo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        robot.liftLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftLeft().setPower(0);
        robot.liftRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftRight().setPower(0);
        robot.driveMotors().frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveMotors().frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveMotors().backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveMotors().backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Par1", robot.driveMotors().frontLeft.getCurrentPosition());
            telemetry.addData("Par2", robot.driveMotors().backRight.getCurrentPosition());
            telemetry.addData("Perp", intake.getCurrentPosition());
            telemetry.addData("Left lift", robot.liftLeft().getCurrentPosition());
            telemetry.addData("Right lift", robot.liftRight().getCurrentPosition());
            telemetry.update();
            sleep(20);
        }
    }
}
