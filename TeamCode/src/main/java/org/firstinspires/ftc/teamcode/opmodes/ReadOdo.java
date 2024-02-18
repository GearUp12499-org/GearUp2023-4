package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

@Disabled
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
            telemetry.addData("Left Dist", robot.distanceLeft().getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Dist", robot.distanceRight().getDistance(DistanceUnit.INCH));
            telemetry.addData("Left lift", robot.liftLeft().getCurrentPosition());
            telemetry.addData("Right lift", robot.liftRight().getCurrentPosition());
            telemetry.update();
            sleep(20);
        }
    }
}
