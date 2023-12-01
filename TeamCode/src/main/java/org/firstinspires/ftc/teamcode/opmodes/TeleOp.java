package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.abstractions.ApproachObject;
import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.abstractions.Dumper;
import org.firstinspires.ftc.teamcode.configurations.Robot;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

import java.util.concurrent.TimeUnit;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.ext.TimingKt;
import kotlin.Unit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    public static Unit kvoid = Unit.INSTANCE;

    /**
     * Returns the maximum of all the arguments
     *
     * @param a      the first argument
     * @param others the other arguments
     * @return the maximum of all the arguments
     */
    private double max(double a, double... others) {
        if (others.length == 0) return a;
        // this is a really scuffed way to say "[a, others...]"
        // TODO: rewrite this so it's less cursed
        double[] combined = new double[others.length + 1];
        combined[0] = a;
        Double max = null;
        System.arraycopy(others, 0, combined, 1, others.length);

        // compare each element to the next one, and keep the larger one
        for (int i = 0; i < combined.length - 1; i++) {
            double first = max != null ? max : combined[i];
            double second = combined[i + 1];
            max = Math.max(first, second);
        }
        return max;
    }

    public double OdoToInches(double ticks) {
        double ticksPerRotation = 8192;
        double radius_inches = 0.69;
        double num_wheel_rotations = ticks / ticksPerRotation;
        return (num_wheel_rotations * 2 * Math.PI * radius_inches);
    }

    @Override
    public void runOpMode() {

        // used for semi-auto tasks, like the claw
        MultitaskScheduler scheduler = new MultitaskScheduler();
        // get the robot configuration container (see RobotConfiguration.java)
        Robot robot = new Robot(hardwareMap);

        // we don't want to have to call driveMotors() every time because it gets tedious
        MotorSet driveMotors = robot.driveMotors();
        // set up the claw
        Claw claw = new Claw(scheduler, robot.clawGrab(), robot.clawRotate(), robot.getClawLock());
        Dumper dumper = new Dumper(scheduler, robot);
        ApproachObject approachBackdrop = new ApproachObject(scheduler, robot);

        claw.defaultPos();
        dumper.defaultPos();

        //Reset odometry pods
        driveMotors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotors.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        int targetLeft = 0;
        int targetRight = 0;
        int[] targets = {0, 500, 750, 1000};
        int hangTarget = 1400;

        double balFL = Var.TeleOp.balanceFront * Var.TeleOp.balanceLeft;
        double balFR = Var.TeleOp.balanceFront * Var.TeleOp.balanceRight;
        double balBL = Var.TeleOp.balanceBack * Var.TeleOp.balanceLeft;
        double balBR = Var.TeleOp.balanceBack * Var.TeleOp.balanceRight;

        double balanceDen = max(Math.abs(balFL), Math.abs(balFR), Math.abs(balBL), Math.abs(balBR), 1);
        balFL /= balanceDen;
        balFR /= balanceDen;
        balBL /= balanceDen;
        balBR /= balanceDen;

        // Creates Slide motor instance
        DcMotor liftRight = hardwareMap.get(DcMotor.class, "slideRight");
        DcMotor liftLeft = hardwareMap.get(DcMotor.class, "slideLeft");

        // Creates Drone motor instance
        DcMotor drone = hardwareMap.get(DcMotor.class, "drone");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime deltaTimer = new ElapsedTime();
        ElapsedTime frameTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = deltaTimer.time(TimeUnit.MICROSECONDS) / 1_000_000.0;
            scheduler.tick();
            frameTimer.reset();
            deltaTimer.reset();

            // Standard mecanum drive code
            // Compute the necessary powers to apply to the motors
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double fac = gamepad1.left_bumper ? Var.TeleOp.throttle : 1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * fac;
            double backLeftPower = (y - x + rx) / denominator * fac;
            double frontRightPower = (y - x - rx) / denominator * fac;
            double backRightPower = (y + x - rx) / denominator * fac;


            // Apply the powers to the motors
            if (!scheduler.isResourceInUse(robot.getDriveMotorLock())) {
                driveMotors.frontLeft.setPower(frontLeftPower * balFL);
                driveMotors.backLeft.setPower(backLeftPower * balBL);
                driveMotors.frontRight.setPower(frontRightPower * balFR);
                driveMotors.backRight.setPower(backRightPower * balBR);
            }

            // 1350
            int LONG_SLIDE_LIM = 3000;
            int SHORT_SLIDE_LIM = 1500;

            // when B is pressed, reset all the lifts to the first preset
            // none of the other presets are ever used lol
            if (gamepad2.b) {
                targetLeft = targets[0];
                targetRight = targets[0];
                dumper.reset();
            }

            int iLiftSpeed = (int) (Var.TeleOp.liftSpeed * dt);
            // use dpad up and down to move the left lift
            if (gamepad2.dpad_up) targetLeft += iLiftSpeed;
            if (gamepad2.dpad_down) targetLeft -= iLiftSpeed;

            double lsy = -gamepad2.left_stick_y;
            double lsx = gamepad2.left_stick_x;
            if (Math.abs(lsy) > 0.5 || Math.abs(lsx) > 0.5) {
                targetLeft = Var.TeleOp.liftScoringPreset;
            }

            int cLiftSpeed = (int) (Var.TeleOp.climbingLiftSpeed * dt);
            // gamepad 1 dpad up/down is for endgame truss scaling
            // moves the right lift, and synchronizes the left lift with it
            if (gamepad1.dpad_up) {
                targetRight += cLiftSpeed;
                targetLeft = targetRight;
            }
            if (gamepad1.dpad_down) {
                targetRight -= cLiftSpeed;
                targetLeft = targetRight;
            }

            // don't let the lifts go out of bounds
            // (this will cause the motors to break down)
            if (targetLeft < 0) targetLeft = 0;
            if (targetLeft > LONG_SLIDE_LIM) targetLeft = LONG_SLIDE_LIM;
            if (targetRight < 0) targetRight = 0;
            if (targetRight > SHORT_SLIDE_LIM) targetRight = SHORT_SLIDE_LIM;

            if (!scheduler.isResourceInUse(robot.getLiftLock())) {
                robot.liftLeft().setTargetPosition(targetLeft);
                robot.liftRight().setTargetPosition(targetRight);
            }

            if (gamepad2.a) {
                claw.grab();
            }
            if (gamepad2.y) {
                claw.deposit();
            }
            if (gamepad2.x) {
                claw.resetTele();
            }
            if (gamepad2.left_bumper) {
                if (targetLeft >= 250) {
                    if (dumper.getState() == Dumper.State.Dump) dumper.dumpSecond();
                    else dumper.dump();
                }
            }
            if (gamepad2.right_bumper) {
                dumper.reset();
            }

            if (gamepad1.x) {
                // drive close to board
                approachBackdrop.approachNoStack(Var.TeleOp.approachDistance);
            }

            // Makes drone launcher motor go zoom when the right bumper is pressed on game pad 1
            if (gamepad1.right_bumper) {
                drone.setPower(1);
            } else {
                drone.setPower(0);
            }
            // Gets the average ticks of both the slide motors --> Ticks for perfect hang position is 1340 ticks use hangTarget variable
            double slideTicks = (liftRight.getCurrentPosition() + liftLeft.getCurrentPosition()) / 2.0;

            if (gamepad1.y) {
                targetLeft = hangTarget;
                targetRight = hangTarget;
                scheduler.task(c -> {
                    c.require(robot.getLiftLock());
                    c.onStart(() -> {
                        robot.liftLeft().setTargetPosition(hangTarget);
                        robot.liftRight().setTargetPosition(hangTarget);
                        return kvoid;
                    });
                    c.isCompleted(() -> !(robot.liftLeft().isBusy() || robot.liftRight().isBusy()));
                    TimingKt.maxDuration(c, 3000);
                    return kvoid;
                });
            }


            robot.tele(telemetry);

            //Updates the average distance traveled forward: positive is right or forward; negative is backward or left
            telemetry.addData("Distance Driven Forward:", OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition()) / 2.0));
            telemetry.addData("Inches Strafed: ", OdoToInches(intake.getCurrentPosition()));
            telemetry.addData("Slide Motor Ticks: ", slideTicks);
            telemetry.addData("dt", "%.2f ms", dt * 1000);
            telemetry.addData("Left Target", targetLeft);
            telemetry.addData("Right Target", targetRight);
            telemetry.update();
            while (frameTimer.time(TimeUnit.MILLISECONDS) < 5) {
                sleep(1);
            }
        }

        // panic() cleans up 'resources' (Claw, drive motors, etc)
        scheduler.panic();
        // just in case
        driveMotors.setAll(0);
    }
}
