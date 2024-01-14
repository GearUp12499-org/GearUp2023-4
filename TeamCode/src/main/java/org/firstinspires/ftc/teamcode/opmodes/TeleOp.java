package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.abstractions.ApproachObject2;
import org.firstinspires.ftc.teamcode.abstractions.Dumper;
import org.firstinspires.ftc.teamcode.configurations.Robot;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;
import org.firstinspires.ftc.teamcode.utilities.RadianUnit;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.Task;
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
        RobotConfiguration robot = new Robot(hardwareMap);
        DcMotor para1 = robot.odoParallelLeft();
        DcMotor para2 = robot.odoParallelRight();
        DcMotor perp = robot.odoPerpendicular();
        CRServo intakeAssist = hardwareMap.get(CRServo.class, "intakeAssist");
        robot.purpleDropper().setPosition(Var.PixelDropper.back);

        // we don't want to have to call driveMotors() every time because it gets tedious
        MotorSet<DcMotor> driveMotors = robot.driveMotors();

        Servo dumperServo = robot.dumperRotate();
        Servo latch = robot.dumperLatch();
        Dumper dumper = new Dumper(scheduler, robot);
        ApproachObject2 approachBackdrop = new ApproachObject2(scheduler, robot);

        dumperServo.setPosition(Var.Box.idleRotate);
        latch.setPosition(Var.Box.latched);

        //Reset odometry pods
        robot.clearEncoders();

        waitForStart();
        AtomicInteger targetLeft = new AtomicInteger();
        AtomicInteger targetRight = new AtomicInteger();
        int hangTarget = Var.TeleOp.liftHangingPreset;

        double balFL = Var.TeleOp.balanceFront * Var.TeleOp.balanceLeft;
        double balFR = Var.TeleOp.balanceFront * Var.TeleOp.balanceRight;
        double balBL = Var.TeleOp.balanceBack * Var.TeleOp.balanceLeft;
        double balBR = Var.TeleOp.balanceBack * Var.TeleOp.balanceRight;

        double balanceDen = max(Math.abs(balFL), Math.abs(balFR), Math.abs(balBL), Math.abs(balBR), 1);
        balFL /= balanceDen;
        balFR /= balanceDen;
        balBL /= balanceDen;
        balBR /= balanceDen;

        robot.clearEncoders();

        ElapsedTime deltaTimer = new ElapsedTime();
        ElapsedTime frameTimer = new ElapsedTime();
        int bumpCycle = 0;
        boolean lastLeftBumper = false;
        @Nullable Task cancelLift = null;

        while (opModeIsActive()) {
            double dt = deltaTimer.time(TimeUnit.MICROSECONDS) / 1_000_000.0;
            scheduler.tick();
            frameTimer.reset();
            deltaTimer.reset();

            // Standard mecanum drive code
            // Compute the necessary powers to apply to the motors
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double throttleThrow = gamepad1.left_trigger - Var.TeleOp.throttleMinThrow;
            double throttlePct = throttleThrow * (1 / (1 - Var.TeleOp.throttleMinThrow));

            double fac = 1;
            if (gamepad1.left_bumper)
                fac = Var.TeleOp.binThrottle;
            if (throttleThrow >= 0)
                fac = 0.4 - ((0.4 - Var.TeleOp.throttle) * throttlePct);

            if (Math.abs(y) + Math.abs(x) > 0.2) {
                scheduler.filteredStop(task -> task.requirements().contains(robot.getDriveMotorLock()));
            }

            double botHeading = robot.imu().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
            int LONG_SLIDE_LIM = Var.TeleOp.longSlideLimit;
            int SHORT_SLIDE_LIM = Var.TeleOp.shortSlideLimit;

            // when B is pressed, reset all the lifts to the first preset
            // none of the other presets are ever used lol
            if (gamepad2.b) {
                scheduler.filteredStop(task -> task.requirements().contains(robot.getDumperLock()));
                dumperServo.setPosition(Var.Box.idleRotate);
                latch.setPosition(Var.Box.latched);
                if (targetLeft.get() > 1100) {
                    targetLeft.set(0);
                    targetRight.set(0);
                } else {
                    if (cancelLift == null
                            || cancelLift.getState() == Task.State.Finished
                            || cancelLift.getState() == Task.State.Cancelled) {
                        cancelLift = scheduler.task(e -> {
                            TimingKt.maxDuration(e, 300);
                            return kvoid;
                        });
                        cancelLift.then(e -> {
                            // no lock because we don't actually take control of the motors
                            e.onStart(() -> {
                                targetLeft.set(0);
                                targetRight.set(0);
                                return kvoid;
                            });
                            e.isCompleted(() -> true);
                            return kvoid;
                        });
                    }
                }
                bumpCycle = 0;
            }

            int iLiftSpeed = (int) (Var.TeleOp.liftSpeed * dt);
            // use dpad up and down to move the left lift
            if (gamepad2.dpad_up) {
                targetLeft.addAndGet(iLiftSpeed);
                if (cancelLift != null) {
                    cancelLift.requestStop(true);
                }
                cancelLift = null;
            }
            if (gamepad2.dpad_down) {
                targetLeft.addAndGet(-iLiftSpeed);
                if (cancelLift != null) {
                    cancelLift.requestStop(true);
                }
                cancelLift = null;
            }

            if (gamepad2.a) {
                targetLeft.set(Var.TeleOp.liftScoringReadyPreset);
                if (bumpCycle < 1) {
                    dumperServo.setPosition(Var.Box.dumpRotate);
                    bumpCycle = 1;
                }
            }

            int cLiftSpeed = (int) (Var.TeleOp.climbingLiftSpeed * dt);
            // gamepad 1 dpad up/down is for endgame truss scaling
            // moves the right lift, and synchronizes the left lift with it
            if (gamepad1.dpad_up) {
                targetRight.addAndGet(cLiftSpeed);
                targetLeft.set(targetRight.get());
            }
            if (gamepad1.dpad_down) {
                targetRight.addAndGet(-cLiftSpeed);
                targetLeft.set(targetRight.get());
            }

            // don't let the lifts go out of bounds
            // (this will cause the motors to break down)
            if (targetLeft.get() < 0) targetLeft.set(0);
            if (targetLeft.get() > LONG_SLIDE_LIM) targetLeft.set(LONG_SLIDE_LIM);
            if (targetRight.get() < 0) targetRight.set(0);
            if (targetRight.get() > SHORT_SLIDE_LIM) targetRight.set(SHORT_SLIDE_LIM);

            if (!scheduler.isResourceInUse(robot.getLiftLock())) {
                robot.liftLeft().setTargetPosition(targetLeft.get());
                robot.liftRight().setTargetPosition(targetRight.get());
            }
            if (gamepad2.left_bumper && !lastLeftBumper) {
                switch (bumpCycle) {
                    case 0:
                        if (targetLeft.get() >= 250) {
                            dumperServo.setPosition(Var.Box.dumpRotate);
                            bumpCycle++;
                        }
                        break;
                    case 1:
                        latch.setPosition(Var.Box.latch1);
                        bumpCycle++;
                        break;
                    case 2:
                        latch.setPosition(Var.Box.unlatched);
                        bumpCycle++;
                        break;
                    case 3:
                        // nothing else to do (?)
                        break;
                }
            }
            lastLeftBumper = gamepad2.left_bumper;

            if (gamepad2.right_bumper) {
                dumperServo.setPosition(Var.Box.idleRotate);
                latch.setPosition(Var.Box.latched);
                bumpCycle = 0;
            }

            if (gamepad1.x) {
                // drive close to board
                approachBackdrop.approachNoStack(Var.TeleOp.approachDistance);
            }

            // Makes drone launcher motor go zoom when the right bumper is pressed on game pad 1
            if (gamepad1.right_bumper) {
                robot.drone().setPower(1);
            } else {
                robot.drone().setPower(0);
            }
            // Gets the average ticks of both the slide motors --> Ticks for perfect hang position is 1340 ticks use hangTarget variable
            double slideTicks = (robot.liftRight().getCurrentPosition() + robot.liftLeft().getCurrentPosition()) / 2.0;

            if (gamepad1.y) {
                // FIXME this class is not being used elsewhere
                dumper.autoReset();
                targetLeft.set(hangTarget);
                targetRight.set(hangTarget);
                scheduler.task(c -> {
                    c.require(robot.getLiftLock());
                    c.onStart(() -> {
                        robot.liftLeft().setTargetPosition(hangTarget);
                        robot.liftRight().setTargetPosition(hangTarget);
                        return kvoid;
                    });
                    c.isCompleted(() -> !(robot.liftLeft().isBusy() || robot.liftRight().isBusy()));
                    TimingKt.maxDuration(c, Var.TeleOp.cancelApproachDuration);
                    return kvoid;
                });
            }

            if (gamepad1.options) {
                robot.imu().resetYaw();
            }

            if (gamepad2.left_trigger > Var.TeleOp.triggerPct) {
                robot.intake().setPower(Var.TeleOp.intakePower);
                intakeAssist.setPower(1.0);
            } else if (gamepad2.right_trigger > Var.TeleOp.triggerPct) {
                robot.intake().setPower(-Var.TeleOp.intakePower);
                intakeAssist.setPower(-1.0);
            } else {
                robot.intake().setPower(0);
                intakeAssist.setPower(0);
            }


            robot.tele(telemetry);

            //Updates the average distance traveled forward: positive is right or forward; negative is backward or left
            telemetry.addData("Distance Driven Forward:",
                    OdoToInches((para2.getCurrentPosition() + para1.getCurrentPosition()) / 2.0));
            telemetry.addData("Inches Strafed: ", OdoToInches(robot.intake().getCurrentPosition()));
            telemetry.addData("Left Slide Ticks", robot.liftLeft().getCurrentPosition());
            telemetry.addData("Right Slide Ticks", robot.liftRight().getCurrentPosition());
            telemetry.addData("dt", "%.2f ms", dt * 1000);
            telemetry.addData("Heading", new RadianUnit(botHeading).to().getDegrees());
            telemetry.addData("Left Target", targetLeft);
            telemetry.addData("Right Target", targetRight);
            telemetry.update();
            while (frameTimer.time(TimeUnit.MILLISECONDS) < 5) {
                sleep(1);
            }
        }

        // panic() cleans up 'resources' (Claw, drive motors, etc)
        scheduler.panic();
        RobotLog.ii("MultitaskScheduler", scheduler.statsheet());
        // just in case
        driveMotors.setAll(0);
    }
}
