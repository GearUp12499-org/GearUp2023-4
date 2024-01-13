package org.firstinspires.ftc.teamcode.odo;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorPowers;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;
import org.firstinspires.ftc.teamcode.utilities.Move;

public class DriveForwardPID {
    public static final double MAX_SPEED_S = 0.5;
    public static final double acceptableError_S = 1.0; // in
    public static final RampProvider strafeRamp = new QuadraticDownRamps(
            0.10,
            0.05,
            0.5,
            3,
            8
    );


    public static final double MAX_SPEED_D = 0.3; // drive forward/reverse
    public static final double RAMPS_UP_D = 6; // drive forward/reverse
    public static final double RAMPS_DOWN_D = 8; // drive forward/reverse
    public static final double MIN_SPEED_INITIAL_D = 0.05; // drive forward/reverse
    public static final double MIN_SPEED_FINAL_D = 0.05; // drive forward/reverse
    public static final double acceptableError_D = 1.0; // drive forward/reverse
    public static final double kp = 0.8;
    public static final double ki = 0.05;
    private final DcMotor paraRight;
    private final DcMotor paraLeft;
    private final DcMotor perp;
    public double sumOfError = 0;
    RobotConfiguration robot;
    MotorSet<DcMotor> driveMotors;

    public DriveForwardPID(RobotConfiguration robot) {
        this.robot = robot;
        this.driveMotors = robot.driveMotors();
        this.paraRight = robot.odoParallelRight();
        this.paraLeft = robot.odoParallelLeft();
        this.perp = robot.odoPerpendicular();
    }

    public static double rampDownForward(double distToTarget) {
        if (distToTarget <= 0) return 0.0;
        if (distToTarget >= RAMPS_DOWN_D) return MAX_SPEED_D;
        else
            return (MAX_SPEED_D - MIN_SPEED_FINAL_D) * (distToTarget / RAMPS_DOWN_D) + MIN_SPEED_FINAL_D;
    }

    public static double rampUpForward(double distTravel) {
        if (distTravel <= 0) return MIN_SPEED_INITIAL_D;
        if (distTravel >= RAMPS_UP_D) return MAX_SPEED_D;
        return (MAX_SPEED_D - MIN_SPEED_INITIAL_D) * (distTravel / RAMPS_UP_D) + MIN_SPEED_INITIAL_D;
    }

    public double ticksToInches(int ticks) {
        double ticksPerRotation = 8192.0;
        double radius_Inches = 0.69;
        double num_wheel_rotation = ticks / ticksPerRotation;
        return num_wheel_rotation * 2 * 3.14 * radius_Inches;
        // inches = (ticks/8192.0) * 0.69 * 6.28
    }

    public double RightOdoDist() {
        return ticksToInches(paraRight.getCurrentPosition());
    }

    public double LeftOdoDist() {
        return ticksToInches(paraLeft.getCurrentPosition());
    }

    public double StrafeOdoDist() {
        return -ticksToInches(perp.getCurrentPosition());
    }

    public void DriveForward(double target, Telemetry telemetry) {
        DriveForward(target, telemetry, -1.0);
    }

    public void DriveForward(double target, Telemetry telemetry, double timeout) {
        if (target < 0) {
            DriveReverse(-target, telemetry, timeout);
            return;
        }
        double rbase = RightOdoDist();
        double lbase = LeftOdoDist();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_error = 0.0;
        double previousTime = stopwatch.time();

        while (timeout < 0 || timer.time() < timeout) {
            double rdist = RightOdoDist() - rbase;
            double ldist = LeftOdoDist() - lbase;
            double avg = (rdist + ldist) / 2.0;

            if (avg > target - acceptableError_D) break;
            double error = rdist - ldist;

            double currentTime = stopwatch.time();
            double dt = currentTime - previousTime;
            overall_error += error * dt;
            previousTime = currentTime;

            double correction = kp * error + ki * overall_error;
            double speed = Math.min(rampUpForward(avg), rampDownForward(target - avg));

            MotorPowers speeds = new MotorPowers(
                    speed + correction,
                    speed - correction,
                    speed + correction,
                    speed - correction
            );
            speeds = speeds.normalNoStretch(MAX_SPEED_D);
            MotorPowers powers = speeds.map(Move::rampSpeedToPower).normalNoStretch();
            powers.apply(driveMotors);
        }

        driveMotors.setAll(0);
        sumOfError = overall_error;
    }


    public void DriveReverse(double target, Telemetry telemetry) {
        DriveReverse(target, telemetry, -1.0);
    }

    public void DriveReverse(double target, Telemetry telemetry, double timeout) {
        if (target < 0) {
            DriveForward(-target, telemetry);
            return;
        }
        double rbase = RightOdoDist();
        double lbase = LeftOdoDist();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_error = 0.0;

        while (timeout < 0 || timer.time() < timeout) {
            double rdist = rbase - RightOdoDist();
            double ldist = lbase - LeftOdoDist();
            double avg = (rdist + ldist) / 2.0;

            if (avg > target - acceptableError_D) break;

            double error = rdist - ldist;

            double dt = stopwatch.time();
            stopwatch.reset();
            overall_error += error * dt;

            double correction = kp * error + ki * overall_error;
            double speed = -Math.min(rampUpForward(avg), rampDownForward(target - avg));

            MotorPowers speeds = new MotorPowers(
                    speed - correction,
                    speed + correction,
                    speed - correction,
                    speed + correction
            );
            speeds = speeds.normalNoStretch(MAX_SPEED_D);
            MotorPowers powers = speeds.map(Move::rampSpeedToPower).normalNoStretch();
            powers.apply(driveMotors);
        }

        driveMotors.setAll(0);
        sumOfError = overall_error;

    }

    public void strafeRight(double target, Telemetry telemetry) {
        strafeRight(target, telemetry, -1.0);
    }


    public void strafeRight(double target, Telemetry telemetry, double timeout) {
        if (target < 0) {
            strafeLeft(-target, telemetry);
            return;
        }
        double s_base = StrafeOdoDist();
        double l_base = LeftOdoDist();
        double r_base = RightOdoDist();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_left = 0.0;
        double overall_right = 0.0;

        while (timeout < 0 || timer.time() < timeout) {
            double s_dist = StrafeOdoDist() - s_base;
            double l_err = LeftOdoDist() - l_base;
            double r_err = RightOdoDist() - r_base;

            if (s_dist > target - acceptableError_S) break;

            double dt = stopwatch.time();
            stopwatch.reset();
            overall_left += l_err * dt;
            overall_right += r_err * dt;

            double left_correct = kp * l_err + ki * overall_left;
            double right_correct = kp * r_err + ki * overall_right;
            double speed = strafeRamp.ramp(s_dist, target - s_dist);

            MotorPowers speeds = new MotorPowers(
                    speed - left_correct,
                    -speed - right_correct,
                    -speed - left_correct,
                    speed - right_correct
            );
            speeds = speeds.normalNoStretch(MAX_SPEED_S);
            MotorPowers powers = speeds.map(Move::rampSpeedToPower).normalNoStretch();
            powers.apply(driveMotors);

            Log.i("Encoders", String.format("L %+.4f  R %+.4f  P %+.4f", l_err, r_err, s_dist));
        }
        driveMotors.setAll(0);
    }

    public void strafeLeft(double target, Telemetry telemetry) {
        strafeLeft(target, telemetry, -1.0);
    }

    public void strafeLeft(double target, Telemetry telemetry, double timeout) {
        if (target < 0) {
            strafeRight(-target, telemetry);
            return;
        }
        double s_base = StrafeOdoDist();
        double l_base = LeftOdoDist();
        double r_base = RightOdoDist();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stopwatch = new ElapsedTime();
        double overall_left = 0.0;
        double overall_right = 0.0;

        while (timeout < 0 || timer.time() < timeout) {
            double s_dist = s_base - StrafeOdoDist();
            double l_err = LeftOdoDist() - l_base;
            double r_err = RightOdoDist() - r_base;

            if (s_dist > target - acceptableError_S) break;

            double dt = stopwatch.time();
            stopwatch.reset();
            overall_left += l_err * dt;
            overall_right += r_err * dt;

            double left_correct = kp * l_err + ki * overall_left;
            double right_correct = kp * r_err + ki * overall_right;
            double speed = strafeRamp.ramp(s_dist, target - s_dist);

            MotorPowers speeds = new MotorPowers(
                    -speed - left_correct,
                    speed - right_correct,
                    speed - left_correct,
                    -speed - right_correct
            );
            speeds = speeds.normalNoStretch(MAX_SPEED_S);
            MotorPowers powers = speeds.map(Move::rampSpeedToPower).normalNoStretch();
            powers.apply(driveMotors);
        }
        driveMotors.setAll(0);
    }
}

