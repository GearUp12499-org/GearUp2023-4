package org.firstinspires.ftc.teamcode.odo;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

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

    public void DriveForward(double target, Telemetry telemetry){
        double reifiedTarget = target + (RightOdoDist() + LeftOdoDist()) / 2.0;
        double rdist = RightOdoDist();
        double ldist = LeftOdoDist();
        double speed = 0.3;

        while(ldist < reifiedTarget && rdist < reifiedTarget) {
            rdist = RightOdoDist();
            ldist = LeftOdoDist();
            double e = rdist - ldist;
            double kp = 0.1;
            double u = kp * e;
            driveMotors.frontLeft.setPower(speed + u);
            driveMotors.backLeft.setPower(speed + u);
            driveMotors.frontRight.setPower(speed - u);
            driveMotors.backRight.setPower(speed - u);
            telemetry.addData("rdist", rdist);
            telemetry.addData("ldist", ldist);
            telemetry.addData("u", u);
            telemetry.addData("target", reifiedTarget);
            telemetry.update();
        }
        driveMotors.setAll(0);
    }
}

