package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utility.MotorSet;

@Autonomous
public class DirtyStrafeToParkingZone extends LinearOpMode {
    public double OdoToInches (double ticks){
        double ticksPerRotation = 8192;
        double radius_inches = 0.69;
        double num_wheel_rotations = ticks/ticksPerRotation;
        return (num_wheel_rotations * 2 * Math.PI * radius_inches);
    }
    @Override
    public void runOpMode() {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        MotorSet driveMotors = robot.driveMotors();
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        driveMotors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addData("Distance Driven Forward:", OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0));

        telemetry.update();
        double distance = OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0);
        double strafeDistance = OdoToInches(intake.getCurrentPosition());

        while (distance < 30) {
            distance = OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0);
            if (strafeDistance <= 2) {
                strafeDistance = OdoToInches(intake.getCurrentPosition());
                driveMotors.frontLeft.setPower(0.4);
                driveMotors.backLeft.setPower(-0.4);
                driveMotors.frontRight.setPower(-0.4);
                driveMotors.backRight.setPower(0.4);
            } else {
            driveMotors.frontLeft.setPower(0.4);
            driveMotors.backLeft.setPower(0.4);
            driveMotors.frontRight.setPower(0.4);
            driveMotors.backRight.setPower(0.4);
            telemetry.addData("Distance Driven Forward: ", distance);
            telemetry.update();
            }
        }
    }
}
