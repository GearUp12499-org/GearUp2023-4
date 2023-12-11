package org.firstinspires.ftc.teamcode.odo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

public class DriveForwardPID {
    public static final double MAX_SPEED = 0.4;
    public static final double RAMPS_UP = 6; // in - too slow to go, make it lower
    public static final double RAMPS_DOWN = 24; // in -
    public static final double MIN_SPEED_INITIAL = 0.25;
    public static final double MIN_SPEED_FINAL = 0.15;
    public static final double acceptableError = 0.25; // in

    double rampDown(double distToTarget) {
        if (distToTarget <= 0) return 0.0;
        if (distToTarget >= RAMPS_DOWN) return MAX_SPEED;
        else return (MAX_SPEED - MIN_SPEED_FINAL) * (distToTarget / RAMPS_DOWN) + MIN_SPEED_FINAL;
    }

    double rampUp(double distTravel) {
        if (distTravel <= 0) return MIN_SPEED_INITIAL;
        if (distTravel >= RAMPS_UP) return MAX_SPEED;
        return (MAX_SPEED - MIN_SPEED_INITIAL) * (distTravel / RAMPS_UP) + MIN_SPEED_INITIAL;
    }

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

    public double StrafeOdoDist() {
        return ticksToInches(robot.intake().getCurrentPosition());
    }

    RobotConfiguration robot;
    MotorSet driveMotors;

    public double sumOfError = 0;
    public static final double kp = 0.1;
    public static final double ki = 0.1;

    public void DriveForward(double target, Telemetry telemetry) {
        if (target < 0) {
            DriveReverse(-target, telemetry);
            return;
        }
        double rbase = RightOdoDist();
        double lbase = LeftOdoDist();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_error = 0.0;
        double previousTime = stopwatch.time();

        while (true) {
            double rdist = RightOdoDist() - rbase;
            double ldist = LeftOdoDist() - lbase;
            double avg = (rdist + ldist) / 2.0;

            if (avg > target - acceptableError) break;
            double error = rdist - ldist;

            double currentTime = stopwatch.time();
            double dt = currentTime - previousTime;
            overall_error += error * dt;
            previousTime = currentTime;

            double correction = kp * error + ki * overall_error;
            double speed = Math.min(rampUp(avg), rampDown(target - avg));
            driveMotors.frontLeft.setPower(speed + correction);
            driveMotors.backLeft.setPower(speed + correction);
            driveMotors.frontRight.setPower(speed - correction);
            driveMotors.backRight.setPower(speed - correction);
        }

        driveMotors.setAll(0);
        sumOfError = overall_error;
    }


    public void DriveReverse(double target, Telemetry telemetry) {
        if (target < 0) {
            DriveForward(-target, telemetry);
            return;
        }
        double rbase = RightOdoDist();
        double lbase = LeftOdoDist();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_error = 0.0;

        while (true) {
            double rdist = rbase - RightOdoDist();
            double ldist = lbase - LeftOdoDist();
            double avg = (rdist + ldist) / 2.0;

            if (avg > target - acceptableError) break;

            double error = rdist - ldist;

            double dt = stopwatch.time();
            stopwatch.reset();
            overall_error += error * dt;

            double correction = kp * error + ki * overall_error;
            double speed = -Math.min(rampUp(avg), rampDown(target - avg));
            driveMotors.frontLeft.setPower(speed + correction);
            driveMotors.backLeft.setPower(speed + correction);
            driveMotors.frontRight.setPower(speed - correction);
            driveMotors.backRight.setPower(speed - correction);
        }

        driveMotors.setAll(0);
        sumOfError = overall_error;
    }

    public void strafeRight(double target, Telemetry telemetry) {
        if (target < 0) {
            strafeLeft(-target, telemetry);
            return;
        }
        double s_base = StrafeOdoDist();
        double l_base = LeftOdoDist();
        double r_base = RightOdoDist();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_left = 0.0;
        double overall_right = 0.0;

        while (true) {
            double s_dist = StrafeOdoDist() - s_base;
            double l_err = LeftOdoDist() - l_base;
            double r_err = RightOdoDist() - r_base;

            if (s_dist > target - acceptableError) break;

            double dt = stopwatch.time();
            stopwatch.reset();
            overall_left += l_err * dt;
            overall_right += r_err * dt;

            double left_correct = kp * l_err + ki * overall_left;
            double right_correct = kp * r_err + ki * overall_right;
            double speed = Math.min(rampUp(s_dist), rampDown(target - s_dist));

            driveMotors.frontLeft.setPower(speed + left_correct);
            driveMotors.frontRight.setPower(-speed + right_correct);
            driveMotors.backLeft.setPower(-speed + left_correct);
            driveMotors.backRight.setPower(speed + right_correct);
        }
    }

    public void strafeLeft(double target, Telemetry telemetry) {
        if (target < 0) {
            strafeRight(-target, telemetry);
            return;
        }
        double s_base = StrafeOdoDist();
        double l_base = LeftOdoDist();
        double r_base = RightOdoDist();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_left = 0.0;
        double overall_right = 0.0;

        while (true) {
            double s_dist = s_base - StrafeOdoDist();
            double l_err = l_base - LeftOdoDist();
            double r_err = r_base - RightOdoDist();

            if (s_dist > target - acceptableError) break;

            double dt = stopwatch.time();
            stopwatch.reset();
            overall_left += l_err * dt;
            overall_right += r_err * dt;

            double left_correct = kp * l_err + ki * overall_left;
            double right_correct = kp * r_err + ki * overall_right;
            double speed = Math.min(rampUp(s_dist), rampDown(target - s_dist));

            driveMotors.frontLeft.setPower(-speed + left_correct);
            driveMotors.frontRight.setPower(speed + right_correct);
            driveMotors.backLeft.setPower(speed + left_correct);
            driveMotors.backRight.setPower(-speed + right_correct);
        }
    }
}

