package org.firstinspires.ftc.teamcode.odo;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

public class TurnPID {
    RobotConfiguration robot;
    MotorSet<DcMotor> driveMotors;
    DcMotor para1;
    DcMotor para2;
    DcMotor perp;
    double kp = 0.002;
    public static final double RAMPS_DOWN = 18;
    public static final double MAX_SPEED = 0.9;
    public static final double MIN_SPEED_FINAL = 0.37;

    public static double rampDown(double distToTarget) {
        if (distToTarget <= 0) return 0.0;
        if (distToTarget >= RAMPS_DOWN) return MAX_SPEED;
        else return (MAX_SPEED - MIN_SPEED_FINAL) * (distToTarget / RAMPS_DOWN) + MIN_SPEED_FINAL;
    }
    public TurnPID(RobotConfiguration robot) {
        this.robot = robot;
        this.driveMotors = robot.driveMotors();
        this.para1 = robot.odoParallelLeft();
        this.para2 = robot.odoParallelRight();
    }

    public double ticksToInches(int ticks) {
        double ticksPerRotation = 8192.0;
        double radius_Inches = 0.69;
        double num_wheel_rotation = ticks / ticksPerRotation;
        return num_wheel_rotation * 2 * Math.PI * radius_Inches;
        // inches = (ticks/8192.0) * 0.69 * 6.28
    }

    public double RightOdoDist() {
        return ticksToInches(para2.getCurrentPosition());
    }

    public double LeftOdoDist() {
        return ticksToInches(para1.getCurrentPosition());
    }

    public static final double acceptableError = 0.2; //in

    public void TurnRobot(double degrees, Telemetry telemetry) {
        double radius = 7.5;
        double l_base = LeftOdoDist();
        double r_base = RightOdoDist();

        // Every 45 degrees the error increases by about 2.5 degrees --> at power 0.6
        // The error for 45 degrees is 9 degrees
        double errorFix = (Math.abs(degrees) / 25.0) * (2.0) /*+ 9.0*/;
        // If positive degrees, turn left according to the unit circle degrees
        // 2 * pi * r * degrees/360
        double turnDist = 2 * Math.PI * radius * ((Math.abs(degrees) - errorFix) / 360.0);
        if (degrees > 0) {
            while (true) {
                // TURNING COUNTER-CLOCKWISE:
                //  Left Encoder  : NEGATIVE
                //  Right Encoder : POSITIVE
                double l_dist = l_base - LeftOdoDist();
                double r_dist = RightOdoDist() - r_base;
                double average = (l_dist + r_dist) / 2.0;

                if (average > turnDist - acceptableError) break;
                double error = r_dist + l_dist; // could be negative (left wheel has turned more) or positive (right wheel has turned more)

                double correction = kp * error;
                double speed = rampDown(turnDist - average);
                Log.i("TurnPID", String.format("speed: %.2f / turnDist: %.2f average: %.2f / correction: %.4f", speed, turnDist, average, correction));
                driveMotors.backLeft.setPower(-speed - correction);
                driveMotors.frontLeft.setPower(-speed - correction);
                driveMotors.frontRight.setPower(speed + correction);
                driveMotors.backRight.setPower(speed + correction);
                telemetry.addData("Right Odometry: ", RightOdoDist());
                telemetry.addData("Left Odometry: ", LeftOdoDist());
                telemetry.update();
            }
            driveMotors.setAll(0);
        } else {
            while (true) {
                // TURNING CLOCKWISE:
                //  Left Encoder  : POSITIVE
                //  Right Encoder : NEGATIVE
                double l_dist = LeftOdoDist() - l_base;
                double r_dist = r_base - RightOdoDist();
                double average = (l_dist + r_dist) / 2.0;

                if (average > turnDist - acceptableError) break;
                double error = r_dist + l_dist;

                double correction = kp * error;
                double speed = rampDown(turnDist - average);
                Log.i("TurnPID", String.format("speed: %.2f / turnDist: %.2f average: %.2f / correction: %.4f", speed, turnDist, average, correction));

                driveMotors.backLeft.setPower(speed + correction);
                driveMotors.frontLeft.setPower(speed + correction);
                //Turns right, which means negative degrees according to the unit circle
                driveMotors.frontRight.setPower(-speed - correction);
                driveMotors.backRight.setPower(-speed - correction);
                telemetry.addData("Right Odometry: ", RightOdoDist());
                telemetry.addData("Left Odometry: ", LeftOdoDist());
                telemetry.update();
            }
            driveMotors.setAll(0);
        }
    }

}
