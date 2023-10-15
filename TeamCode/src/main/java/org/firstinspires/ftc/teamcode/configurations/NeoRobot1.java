package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utility.MotorSet;

public class NeoRobot1 {
    public DcMotor frontLeft; // 0
    public DcMotor frontRight; // 1
    public DcMotor backLeft; // 2
    public DcMotor backRight; // 3

    public DcMotor slideLeft; // 0
    public DcMotor slideRight; // 1

    private static void reverse(DcMotor target) {
        target.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private static void brake(DcMotor ...targets) {
        for (DcMotor target : targets ) target.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public NeoRobot1(HardwareMap map) {
        frontLeft = map.get(DcMotor.class, "frontLeft");
        reverse(frontLeft);
        frontRight = map.get(DcMotor.class, "frontRight");
        backLeft = map.get(DcMotor.class, "backLeft");
        reverse(backLeft);
        backRight = map.get(DcMotor.class, "backRight");
        brake(frontLeft, frontRight, backLeft, backRight);

        slideLeft = map.get(DcMotor.class, "slideLeft");
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setTargetPosition(0);
        slideLeft.setPower(1);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight = map.get(DcMotor.class, "slideRight");
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setTargetPosition(0);
        slideRight.setPower(1);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        reverse(slideRight);
    }

    public MotorSet getMotorSet() {
        return new MotorSet(frontLeft, frontRight, backLeft, backRight);
    }
}
