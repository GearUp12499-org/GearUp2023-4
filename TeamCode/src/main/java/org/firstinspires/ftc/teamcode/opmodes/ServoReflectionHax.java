package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * We have reached the point in the season that java.lang.reflect is fair game.
 * P.S. Hey, SDK authors - why didn't you expose this method? lul
 */
@TeleOp
public class ServoReflectionHax extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Prepare the reflection hacks
        Method internalSetPwmEnable;
        try {
            internalSetPwmEnable = LynxServoController.class.getDeclaredMethod("internalSetPwmEnable", int.class, boolean.class);
            internalSetPwmEnable.setAccessible(true);
        } catch (NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        Servo target = robot.dumperRotate();
        int port = target.getPortNumber();
        LynxServoController controller = (LynxServoController) target.getController();
        waitForStart();
        target.setPosition(Var.Box.idleRotate);

        boolean lastX = false;
        boolean enabled = true;
        while (opModeIsActive()) {
            if (gamepad1.x && !lastX) {
                enabled = !enabled;
                try {
                    internalSetPwmEnable.invoke(controller, port, enabled);
                } catch (IllegalAccessException | InvocationTargetException e) {
                    RobotLog.ee("OpMode", e, "Who would have thought messing with internals would do this?!?!?\n");
                    throw new RuntimeException(e);
                }
            }
            lastX = gamepad1.x;
            telemetry.addData("Enabled?", enabled ? "Yes!" : "No!");
            telemetry.update();
        }
    }
}
