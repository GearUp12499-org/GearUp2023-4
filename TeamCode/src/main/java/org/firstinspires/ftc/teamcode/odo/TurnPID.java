package org.firstinspires.ftc.teamcode.odo;



import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

public class TurnPID {
    RobotConfiguration robot;
    MotorSet driveMotors;
    DcMotor.RunMode baseMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    double motorSign = 1.0;
    public TurnPID(RobotConfiguration robot) {
        this.robot = robot;
        this.driveMotors = robot.driveMotors();
    }
    public double ticksToInches(int ticks) {
        double ticksPerRotation = 8192.0;
        double radius_Inches = 0.69;
        double num_wheel_rotation = ticks / ticksPerRotation;
        return num_wheel_rotation * 2 * Math.PI * radius_Inches;
        // inches = (ticks/8192.0) * 0.69 * 6.28
    }

    public double RightOdoDist() {
        return ticksToInches(driveMotors.backRight.getCurrentPosition());
    }

    public double LeftOdoDist() {
        return ticksToInches(driveMotors.frontLeft.getCurrentPosition());
    }

    public void TurnRobot(double degrees, Telemetry telemetry){
        double radius = 7.5;
        if(degrees > 0){
        //If positive degrees, turn left according to the unit circle degrees
        //2 * pi * r * degrees/360
        double turnDist = 2 * Math.PI * radius * (degrees/360.0);
        //Every 45 degrees the error increases by about 2.5 degrees --> at power 0.6
        //The error for 45 degrees is 9 degrees
        double errorFix = (degrees/45.0)*(2.0) + 9.0;
        //We have to subtract the extra degrees the robot turns from where we want to go
        double actualTurnDist = turnDist - errorFix;

        while(LeftOdoDist() < actualTurnDist && RightOdoDist() < actualTurnDist){
            driveMotors.backLeft.setPower(-0.6);
            driveMotors.frontLeft.setPower(-0.6);
            driveMotors.frontRight.setPower(0.6);
            driveMotors.backRight.setPower(0.6);
            telemetry.addData("Right Odometry: ", RightOdoDist());
            telemetry.addData("Left Odometry: ", LeftOdoDist());
            telemetry.update();
        }
        driveMotors.setAll(0);
        } else {
            double turnDist = 2 * Math.PI * radius * (degrees/360.0);
            double errorFix = (Math.abs(degrees)/45.0)*(2.0) + 9.0;
            double actualTurnDist = turnDist + errorFix;
            while(LeftOdoDist() > actualTurnDist && RightOdoDist() > actualTurnDist){
                driveMotors.backLeft.setPower(0.6);
                driveMotors.frontLeft.setPower(0.6);
                //Turns right, which means negative degrees according to the unit circle
                driveMotors.frontRight.setPower(motorSign * 0.6);
                driveMotors.backRight.setPower(motorSign * 0.6);
                telemetry.addData("Right Odometry: ", RightOdoDist());
                telemetry.addData("Left Odometry: ", LeftOdoDist());
                telemetry.update();
            }
        }
    }

}
