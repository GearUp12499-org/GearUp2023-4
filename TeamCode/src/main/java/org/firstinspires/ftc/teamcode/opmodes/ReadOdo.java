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
        robot.odoParallelLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoParallelLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoParallelRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoParallelRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoPerpendicular().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoPerpendicular().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Par1", robot.odoParallelLeft().getCurrentPosition());
            telemetry.addData("Par2", robot.odoParallelRight().getCurrentPosition());
            telemetry.addData("Perp", robot.odoPerpendicular().getCurrentPosition());
            telemetry.addData("Left lift", robot.liftLeft().getCurrentPosition());
            telemetry.addData("Right lift", robot.liftRight().getCurrentPosition());
            telemetry.update();
            sleep(20);
        }
    }
}
