package org.firstinspires.ftc.teamcode.odo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

public class DriveForwardPIDex {
    public DriveForwardPIDex(){
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
    }
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor.RunMode baseMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    public double ticksToInches(int ticks) {
        double ticksPerRotation = 537.7;
        double radius_Inches = 1.89;
        double num_wheel_rotation = ticks / ticksPerRotation;
        return num_wheel_rotation * 2 * 3.14 * radius_Inches;
        // inches = (ticks/537.7) * 1.89 * 6.28
    }

    public double averageDistanceOfMotors() {
        //inches * 1.89 * 6.28 * 537.7 = ticks
        // double tick;
        // return tick = inch * 1.89 * 6.28 * 537.7;
        return (
                ticksToInches(-frontLeft.getCurrentPosition()
                        + -backLeft.getCurrentPosition()
                        + frontRight.getCurrentPosition()
                        + -backRight.getCurrentPosition())) / 4.0;
    }

    public double readYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) * -1;
    }

    public void driveToX(double distToDriveInches) {
        imu.resetYaw();
        //r equals target value
        double r = 0;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(baseMode);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(baseMode);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(baseMode);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(baseMode);
        //you can use speed to change the speed of the robot
        double speed = 0.2;
        double previousE = 0;
        double sumE = 0;
        // 10 degree error = motor power goes up by .1 because 10 * 0.01
        double kp = 0.01;
        double ki = 0.01;
        double kd = 0;
        while (averageDistanceOfMotors() < distToDriveInches) {
            double y = readYaw();
            double e = r - y;
            sumE += e;
            double deltaE = e - previousE;
            double u = kp * e + ki * sumE + kd * deltaE;

            frontLeft.setPower(speed + u);
            backLeft.setPower(speed + u);
            frontRight.setPower(speed - u);
            backRight.setPower(speed - u);
            telemetry.addData("error", e);
            telemetry.addData("Yaw", readYaw());
            telemetry.update();
            previousE = e;
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    IMU imu;

    public void runOpMode() {
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();


        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(baseMode);
        backLeft.setMode(baseMode);
        frontRight.setMode(baseMode);
        backRight.setMode(baseMode);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //waitForStart();

       // if (isStopRequested()) return;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * 4;
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("frontRight", ticksToInches(frontRight.getCurrentPosition()));
            telemetry.addData("backRight", -ticksToInches(backRight.getCurrentPosition()));
            telemetry.addData("frontLeft", -ticksToInches(frontLeft.getCurrentPosition()));
            telemetry.addData("backLeft", -ticksToInches(backLeft.getCurrentPosition()));
            telemetry.addData("Yaw", readYaw());
            telemetry.update();
            if (gamepad1.a) {
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeft.setMode(baseMode);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(baseMode);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(baseMode);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(baseMode);
            }

    }
}
