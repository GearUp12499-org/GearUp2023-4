package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

import dev.aether.collaborative_multitasking.MultitaskScheduler;

@Autonomous
public class BlueRightFrontstage extends LinearOpMode {
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
        double setPoint = Var.Autonomous.frontStageForwardDistance;
        claw.defaultPos();
        waitForStart();
        claw.rotate.setPosition(Var.Claw.hoverRotate);
        telemetry.addData("Distance Driven Forward:", OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0));

        telemetry.update();
        double distance = OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0);
        while (distance < setPoint) {
            distance = OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0);
            driveMotors.setAll(0.4);
            telemetry.addData("Distance Driven Forward: ", distance);
            telemetry.update();
        }
        // Sets all motors to have 0 power
        driveMotors.setAll(0);
        // Claw scoring codes
        // Uses the Rotate_Hover variable to hover right above ground to drop pixels
        claw.rotate.setPosition(Var.Claw.hoverRotate);
        sleep(1000);
        claw.grip.setPosition(Var.Claw.opened);
        sleep(500);
        claw.rotate.setPosition(Var.Claw.flippedRotate);
        sleep(500);
        // Let's code run until we press the stop button
        while(opModeIsActive()){

        }
    }
}
