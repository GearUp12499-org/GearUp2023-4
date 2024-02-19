package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utilities.MotorSet;

import java.util.function.Supplier;

class Panic extends RuntimeException {
}

@TeleOp(name = "Hardware Test")
public class HWTest extends LinearOpMode {
    boolean lastA = false;
    boolean lastB = false;

    boolean verbalize = false;

    RobotConfiguration robot;

    /**
     * Ask the user to confirm something with the gamepad.
     *
     * @param message message to display with the prompt
     * @return TRUE if the user pressed A, FALSE if the user pressed B
     */
    boolean confirm(String message) {
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        for (String line : message.split("\n")) {
            telemetry.addLine(line);
        }
        if (verbalize) {
            telemetry.speak(message);
        }
        telemetry.addLine();
        telemetry.addLine("[A]: YES or [B]: NO");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) return true;
            if (gamepad1.b && !lastB) return false;
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
        throw new Panic();
    }

    /**
     * Displays a message and waits for the user to press A.
     *
     * @param message message to display
     */
    void alert(String message) {
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        for (String line : message.split("\n")) {
            telemetry.addLine(line);
        }
        if (verbalize) {
            telemetry.speak(message);
        }
        telemetry.addLine();
        telemetry.addLine("[A]: Confirm");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) return;
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
        throw new Panic();
    }

    void testMotor(String label, DcMotor theMotor) {
        testMotor(label, theMotor, "moving forward");
    }

    void testMotor(String label, DcMotor theMotor, String action) {
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        theMotor.setPower(0.25);
        telemetry.addLine("The " + label + " motor should");
        telemetry.addLine("be " + action + "...");
        telemetry.addLine();
        telemetry.addLine("[A]: OK [B]: STOP");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) {
                theMotor.setPower(0.0);
                return;
            }
            if (gamepad1.b && !lastB) {
                break;
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
        theMotor.setPower(0.0);
        throw new Panic();
    }

    void testMotors() {
        if (!confirm("Test motors?")) return;
        alert("Lift the robot so that the wheels don't\ntouch the ground for the duration of\nthe test.");
        MotorSet<DcMotor> motors = robot.driveMotors();
        testMotor("Front Left", motors.frontLeft);
        testMotor("Front Right", motors.frontRight);
        testMotor("Back Left", motors.backLeft);
        testMotor("Back Right", motors.backRight);
        testMotor("Intake", robot.intake(), "turning reverse (outtaking)");
        alert("Remove the drone from the launcher if it is present...");
        testMotor("Drone", robot.drone(), "spinning");
    }

    void testEncoder(String label, DcMotor encoded) {
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        telemetry.addLine("Getting ready...");
        telemetry.update();
        encoded.setPower(0.0);
        encoded.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoded.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Move the " + label + " encoder...");
        telemetry.addLine();
        telemetry.addLine("[B]: STOP");
        telemetry.update();

        int originalPosition = encoded.getCurrentPosition();
        int thresh = 50;
        while (opModeIsActive()) {
            if (Math.abs(encoded.getCurrentPosition() - originalPosition) > thresh) {
                return;
            }
            if (gamepad1.b && !lastB) {
                break;
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;

        }
        throw new Panic();
    }

    void testButton(String label, Supplier<Boolean> enabled) {
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        telemetry.addLine("Activate the " + label + " sensor...");
        telemetry.addLine();
        telemetry.addLine("[B]: STOP");
        telemetry.update();

        while (opModeIsActive()) {
            if (enabled.get()) {
                break;
            }
            if (gamepad1.b && !lastB) {
                throw new Panic();
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }

        telemetry.addLine("Deactivate the " + label + " sensor...");
        telemetry.addLine();
        telemetry.addLine("[B]: STOP");
        telemetry.update();

        while (opModeIsActive()) {
            if (!enabled.get()) {
                break;
            }
            if (gamepad1.b && !lastB) {
                throw new Panic();
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
    }

    void testEncoders() {
        if (!confirm("Test encoders?")) return;
        testEncoder("Strafe odo", robot.odoPerpendicular());
        testEncoder("Forward odo 1", robot.odoParallelLeft());
        testEncoder("Forward odo 2", robot.odoParallelRight());
        testEncoder("Left lift", robot.liftLeft());
        testEncoder("Right lift", robot.liftRight());
    }

    void testDistance(String label, DistanceSensor distanceSensor) {
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        telemetry.addLine("Activate the " + label + "...");
        telemetry.addLine();
        telemetry.addLine("[B]: STOP");
        telemetry.update();
        double originalPosition = distanceSensor.getDistance(DistanceUnit.MM);
        double thresh = 50;
        while (opModeIsActive()) {
            if (Math.abs(distanceSensor.getDistance(DistanceUnit.MM) - originalPosition) > thresh) {
                return;
            }
            if (gamepad1.b && !lastB) {
                break;
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
        throw new Panic();
    }

    void testSensors() {
        if (!confirm("Test sensors?")) return;
        testDistance("Left", robot.distanceLeft());
        testDistance("Right", robot.distanceRight());
        robot.liftLeft().setPower(0.0);
        robot.liftLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testButton("Lift switch", robot.liftLimitSwitch()::isPressed);
    }

    @Override
    public void runOpMode() {
        robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        waitForStart();
        try {
            verbalize = confirm("Say instructions out loud?");
            testMotors();
            testEncoders();
            testSensors();
        } catch (Panic ignore) {
            while (opModeIsActive()) {
                telemetry.addLine("Stopped.");
                telemetry.update();
                sleep(50);
            }
        }
        while (opModeIsActive()) {
            telemetry.addLine("Done.");
            telemetry.update();
            sleep(50);
        }
    }
}
