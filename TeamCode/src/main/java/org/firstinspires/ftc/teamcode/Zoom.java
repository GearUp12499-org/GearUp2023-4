package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Zoom extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor daMotor = hardwareMap.get(DcMotor.class, "frontRight");
        daMotor.setPower(0);
        waitForStart();
        daMotor.setPower(1);
        while (opModeIsActive()) {
            sleep(20);
        }
        daMotor.setPower(0);

    }
}
