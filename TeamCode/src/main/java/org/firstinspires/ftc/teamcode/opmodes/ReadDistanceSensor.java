package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

@TeleOp
public class ReadDistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        DistanceSensor left = robot.distanceLeft();
        DistanceSensor right = robot.distanceRight();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addLine("press A for updates");
                telemetry.addData("Left", "%.2fin", left.getDistance(DistanceUnit.INCH));
                telemetry.addData("Right", "%.2fin", right.getDistance(DistanceUnit.INCH));
                telemetry.addLine();
                telemetry.addLine();
                robot.tele(telemetry);
                telemetry.update();
            }
            sleep(20);
        }
    }
}
