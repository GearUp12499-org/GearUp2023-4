package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.abstractions.Dumper;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.InchUnit;
import org.firstinspires.ftc.teamcode.utility.MotorSet;

import dev.aether.collaborative_multitasking.MultitaskScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
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
    public double OdoToInches (double ticks){
        double ticksPerRotation = 8192;
        double radius_inches = 0.69;
        double num_wheel_rotations = ticks/ticksPerRotation;
        return (num_wheel_rotations * 2 * Math.PI * radius_inches);
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

        // Creates Slide motor instance
        DcMotor liftRight = hardwareMap.get(DcMotor.class, "slideRight");
        DcMotor liftLeft = hardwareMap.get(DcMotor.class, "slideLeft");

        // Creates Drone motor instance
        DcMotor drone = hardwareMap.get(DcMotor.class, "drone");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime timer1 = new ElapsedTime();
        while (opModeIsActive()) {
            scheduler.tick();
//            double dt = timer1.time(TimeUnit.SECONDS);
            timer1.reset();

            // Standard mecanum drive code
            // Compute the necessary powers to apply to the motors
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double fac = gamepad1.left_bumper ? 0.5 : 1;

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

            int LONG_SLIDE_LIM = 3000;
            int SHORT_SLIDE_LIM = 1500;
            int MOTION_PER_CYCLE = 20;

            // when B is pressed, reset all the lifts to the first preset
            // none of the other presets are ever used lol
            if (gamepad2.b) {
                targetLeft = targets[0];
                targetRight = targets[0];
                dumper.reset();
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
            if (targetLeft > LONG_SLIDE_LIM) targetLeft = LONG_SLIDE_LIM;
            if (targetRight < 0) targetRight = 0;
            if (targetRight > SHORT_SLIDE_LIM) targetRight = SHORT_SLIDE_LIM;

            robot.liftLeft().setTargetPosition(targetLeft);
            robot.liftRight().setTargetPosition(targetRight);

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
                if (dumper.getState() == Dumper.State.Dump) dumper.dumpSecond();
                else dumper.dump();
            }
            if (gamepad2.right_bumper) {
                dumper.reset();
            }

            if (gamepad1.x) {
                // drive close to board
                approachBackdrop.approachNoStack(new InchUnit(3));
            }

            // TODO: really should make this based on deltaTime
            // make time between refreshes longer to avoid spamming motors with commands

            // Makes drone launcher motor go zoom when the right bumper is pressed on game pad 1
            if (gamepad1.right_bumper) {
                drone.setPower(1);
            } else {
                drone.setPower(0);
            }
            // Gets the average ticks of both the slide motors --> Ticks for perfect hang position is 1340 ticks use hangTarget variable
            double slideTicks = (liftRight.getCurrentPosition() + liftLeft.getCurrentPosition())/2.0;

            if(gamepad1.y){
                robot.liftRight().setTargetPosition(hangTarget);
                robot.liftLeft().setTargetPosition(hangTarget);
                robot.liftRight().setPower(0.3);
                robot.liftLeft().setPower(0.3);
                targetLeft = hangTarget;
                targetRight = hangTarget;
                resetRuntime();
                while(robot.liftRight().isBusy() && robot.liftLeft().isBusy() || getRuntime() <= 3){
                    telemetry.addData("Slide Motor Ticks: ", slideTicks);
                }
            }


            robot.tele(telemetry);

            //Updates the average distance traveled forward: positive is right or forward; negative is backward or left
            telemetry.addData("Distance Driven Forward:", OdoToInches((driveMotors.backRight.getCurrentPosition() + driveMotors.frontLeft.getCurrentPosition())/2.0));
            telemetry.addData("Inches Strafed: ", OdoToInches(intake.getCurrentPosition()));
            telemetry.addData("Slide Motor Ticks: ", slideTicks);
            telemetry.update();
        }

        // panic() cleans up 'resources' (Claw, drive motors, etc)
        scheduler.panic();
        // just in case
        driveMotors.setAll(0);
    }
}
