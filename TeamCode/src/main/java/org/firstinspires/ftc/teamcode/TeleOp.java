package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utility.MotorSet;

import java.util.concurrent.TimeUnit;

import dev.aether.collaborative_multitasking.MultitaskScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    /**
     * Returns the maximum of all the arguments
     * @param a the first argument
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

    @Override
    public void runOpMode() {
        // used for semi-auto tasks, like the claw
        MultitaskScheduler scheduler = new MultitaskScheduler();
        // get the robot configuration container (see RobotConfiguration.java)
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        if (robot == null) throw new RuntimeException("Robot configuration not found");

        // we don't want to have to call driveMotors() every time because it gets tedious
        MotorSet driveMotors = robot.driveMotors();
        // set up the claw
        Claw claw = new Claw(scheduler, robot.clawGrab(), robot.clawRotate(), robot.getClawLock());


        waitForStart();
        int targetLeft = 0;
        int targetRight = 0;
        int[] targets = {0, 500, 750, 1000};

        double balanceLeft = 1.00;
        double balanceRight = 1.00;
        double balanceFront = 1.00;
        double balanceBack = 1.00;

        double balFL = balanceFront * balanceLeft;
        double balFR = balanceFront * balanceRight;
        double balBL = balanceBack * balanceLeft;
        double balBR = balanceBack * balanceRight;
        double balanceDen = max(Math.abs(balFL), Math.abs(balFR), Math.abs(balBL), Math.abs(balBR), 1);
        balFL /= balanceDen;
        balFR /= balanceDen;
        balBL /= balanceDen;
        balBR /= balanceDen;

        ElapsedTime timer1 = new ElapsedTime();
        while (opModeIsActive()) {
            scheduler.tick();
            double dt = timer1.time(TimeUnit.SECONDS);
            timer1.reset();

            // Standard mecanum drive code
            // Compute the necessary powers to apply to the motors
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double fac = gamepad1.left_bumper ? 1 : 0.75;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * fac;
            double backLeftPower = (y - x + rx) / denominator * fac;
            double frontRightPower = (y - x - rx) / denominator * fac;
            double backRightPower = (y + x - rx) / denominator * fac;


            // Apply the powers to the motors
            driveMotors.frontLeft.setPower(frontLeftPower * balFL);
            driveMotors.backLeft.setPower(backLeftPower * balBL);
            driveMotors.frontRight.setPower(frontRightPower * balFR);
            driveMotors.backRight.setPower(backRightPower * balBR);

            int SLIDE_LIM = 3000;
            int MOTION_PER_CYCLE = 20;

            // when B is pressed, reset all the lifts to the first preset
            // none of the other presets are ever used lol
            if (gamepad2.b) {
                targetLeft = targets[0];
                targetRight = targets[0];
            }

            // use dpad up and down to move the left lift
            if (gamepad2.dpad_up) targetLeft += MOTION_PER_CYCLE;
            if (gamepad2.dpad_down) targetLeft -= MOTION_PER_CYCLE;

            // gamepad 1 dpad up/down is for endgame truss scaling
            // moves the right lift, and synchronizes the left lift with it
            if (gamepad1.dpad_up) {
                targetRight += MOTION_PER_CYCLE;
                targetLeft = targetRight;
            }
            if (gamepad1.dpad_down) {
                targetRight -= MOTION_PER_CYCLE;
                targetLeft = targetRight;
            }

            // don't let the lifts go out of bounds
            // (this will cause the motors to break down)
            if (targetLeft < 0) targetLeft = 0;
            if (targetLeft > SLIDE_LIM) targetLeft = SLIDE_LIM;
            if (targetRight < 0) targetRight = 0;
            if (targetRight > SLIDE_LIM) targetRight = SLIDE_LIM;

            robot.liftLeft().setTargetPosition(targetLeft);
            robot.liftRight().setTargetPosition(targetRight);

            if (gamepad2.a) {
                claw.grab();
            }
            if (gamepad2.y) {
                claw.deposit();
            }
            if (gamepad2.x) {
                claw.reset();
            }

            // TODO: really should make this based on deltaTime
            // make time between refreshes longer to avoid spamming motors with commands
            sleep(5);
        }

        // panic() cleans up 'resources' (Claw, drive motors, etc)
        scheduler.panic();
        // just in case
        driveMotors.setAll(0);
    }
}
