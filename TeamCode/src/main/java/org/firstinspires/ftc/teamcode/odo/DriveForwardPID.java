package org.firstinspires.ftc.teamcode.odo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

public class DriveForwardPID {
    public static final double MAX_SPEED = 0.4;
    public static final double RAMPS_UP = 6; // in - too slow to go, make it lower
    public static final double RAMPS_DOWN = 18; // in -
    public static final double MIN_SPEED = 0.2;
    public static ElapsedTime loopStopwatch = new ElapsedTime();

    double rampDown(double distToTarget) {
        if (distToTarget <= 0) return 0.0;
        if (distToTarget >= RAMPS_DOWN) return MAX_SPEED;
        else return (MAX_SPEED - MIN_SPEED) * (distToTarget / RAMPS_DOWN) + MIN_SPEED;
    }

    double rampUp(double distTravel) {
        if (distTravel <= 0) return MIN_SPEED;
        if (distTravel >= RAMPS_UP) return MAX_SPEED;
        return (MAX_SPEED - MIN_SPEED) * (distTravel / RAMPS_UP) + MIN_SPEED;
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

    RobotConfiguration robot;
    MotorSet driveMotors;
    DcMotor.RunMode baseMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    public void DriveForward(double target, Telemetry telemetry){
        double base = (RightOdoDist() + LeftOdoDist()) / 2.0;
        double reifiedTarget = target + base;
        double rdist = RightOdoDist();
        double ldist = LeftOdoDist();

        while(ldist < reifiedTarget && rdist < reifiedTarget) {
            rdist = RightOdoDist();
            ldist = LeftOdoDist();
            double e = rdist - ldist;
            double kp = 0.1;
            double u = kp * e;
            double avg = (rdist + ldist) / 2.0;
            double speed = Math.min(rampUp(avg - base), rampDown(reifiedTarget - avg));
            driveMotors.frontLeft.setPower(speed + u);
            driveMotors.backLeft.setPower(speed + u);
            driveMotors.frontRight.setPower(speed - u);
            driveMotors.backRight.setPower(speed - u);
            telemetry.addData("rdist", rdist);
            telemetry.addData("ldist", ldist);
            telemetry.addData("u", u);
            telemetry.addData("target", reifiedTarget);
            telemetry.addData("dt", reifiedTarget - avg);
            telemetry.addData("speeed", speed);
            telemetry.addData("Time inside loop: ", loopStopwatch.time());
            telemetry.update();
        }
        driveMotors.setAll(0);
        //loopStopwatch.reset();

    }
}

