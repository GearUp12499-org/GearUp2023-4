package org.firstinspires.ftc.teamcode.odo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

public class TurnPID {
    RobotConfiguration robot;
    MotorSet driveMotors;
    DcMotor.RunMode baseMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    public TurnPID(RobotConfiguration robot) {
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

    public void TurnRobot(double degrees){
        double radius = 7.5;
        //2 * pi * r * degrees/360
        double turnDist = 2 * Math.PI * radius * (degrees/360.0);
        while(Math.abs(LeftOdoDist()) < turnDist && Math.abs(RightOdoDist()) < turnDist){
            driveMotors.backLeft.setPower(-0.6);
            driveMotors.frontLeft.setPower(-0.6);
            driveMotors.frontRight.setPower(0.6);
            driveMotors.backRight.setPower(0.6);
        }
        driveMotors.setAll(0);
    }

}
