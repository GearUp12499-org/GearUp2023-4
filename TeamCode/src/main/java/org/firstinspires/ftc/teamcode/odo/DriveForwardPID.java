package org.firstinspires.ftc.teamcode.odo;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveForwardPID {
    public DriveForwardPID(RobotConfiguration robot) {
        this.robot = robot;
        this.driveMotors = robot.driveMotors();
    }
    public double ticksToInches(int ticks) {
        double ticksPerRotation = 8192.0;
        double radius_Inches = 0.69;
        double num_wheel_rotation = ticks / ticksPerRotation;
        return num_wheel_rotation * 2 * 3.14 * radius_Inches;
        // inches = (ticks/8192.0) * 0.69 * 6.28
    }
    public double RightOdoDist() {
        return ticksToInches(driveMotors.backRight.getCurrentPosition());
    }
    public double LeftOdoDist() {
        return ticksToInches(driveMotors.frontLeft.getCurrentPosition());
    }

    RobotConfiguration robot;
    MotorSet driveMotors;
    DcMotor.RunMode baseMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    public void DriveForward(double target){
        double rdist = RightOdoDist();
        double ldist = LeftOdoDist();
        double speed = 0.3;
        double e = RightOdoDist() - LeftOdoDist();
        double kp = 0.01;
        double u = kp * e;

        while(ldist < target && rdist < target) {
            rdist = RightOdoDist();
            ldist = LeftOdoDist();
            driveMotors.frontLeft.setPower(speed + u);
            driveMotors.backLeft.setPower(speed + u);
            driveMotors.frontRight.setPower(speed - u);
            driveMotors.backRight.setPower(speed - u);
        }
        driveMotors.setAll(0);
    }
}

