package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

@TeleOp
public class MoveSlidesToHang extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftRight = hardwareMap.get(DcMotor.class, "slideRight");
        DcMotor liftLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        //This is the ideal position in ticks for the slides --> 1400
        int hangTarget = 0;
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        robot.liftRight().setPower(0.3);
        robot.liftLeft().setPower(0.3);
// Stuff above this runs once and stuff after waitForStart() runs until I press stop
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Slide Motor Ticks: ", hangTarget);
            telemetry.update();
            if(gamepad1.right_trigger > 0.9) {
                hangTarget = 1400;
            } else if (gamepad1.left_trigger > 0.8) {
                hangTarget = 0;
            }
            robot.liftRight().setTargetPosition(hangTarget);
            robot.liftLeft().setTargetPosition(hangTarget);

        }
    }
}
