package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utility.MotorSet;

@Autonomous
public class SimpleAuto extends LinearOpMode {
    public double OdoToInches (double ticks){
        double ticksPerRotation = 8192;
        double radius_inches = 0.69;
        double num_wheel_rotations = ticks/ticksPerRotation;
        return (num_wheel_rotations * 2 * Math.PI * radius_inches);
    }
    @Override
    public void runOpMode() {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        MotorSet driveMotors = robot.driveMotors();
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        MultitaskScheduler scheduler = new MultitaskScheduler();
        Claw claw = new Claw(scheduler, robot.clawGrab(), robot.clawRotate(), robot.getClawLock());
        driveMotors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addData("Distance Driven Forward:", OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0));

        telemetry.update();
        double distance = OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0);
        double strafeDistance = OdoToInches(intake.getCurrentPosition());

        while (distance < 30) {
            distance = OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0);
            if (strafeDistance <= 3) {
                strafeDistance = OdoToInches(intake.getCurrentPosition());
                driveMotors.frontLeft.setPower(0.4);
                driveMotors.backLeft.setPower(-0.4);
                driveMotors.frontRight.setPower(-0.4);
                driveMotors.backRight.setPower(0.4);
            } else {
            driveMotors.frontLeft.setPower(0.4);
            driveMotors.backLeft.setPower(0.4);
            driveMotors.frontRight.setPower(0.4);
            driveMotors.backRight.setPower(0.4);
            // Claw scoring code
                claw.
            telemetry.addData("Distance Driven Forward: ", distance);
            telemetry.update();
            }
        }
    }
}
