package org.firstinspires.ftc.teamcode.abstractions;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class ServoPowerManager {
    LynxServoController controller;
    int portNo;
    private static final Method internalSetPwmEnable;
    static {
        try {
            internalSetPwmEnable = LynxServoController.class.getDeclaredMethod("internalSetPwmEnable", int.class, boolean.class);
            internalSetPwmEnable.setAccessible(true);
        } catch (NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }

    public ServoPowerManager(Servo basedOn) {
        controller = (LynxServoController) basedOn.getController();
        portNo = basedOn.getPortNumber();
    }

    public void power(boolean enable) {
        try {
            internalSetPwmEnable.invoke(controller, portNo, enable);
        } catch (InvocationTargetException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    public void powerOn() {
        power(true);
    }

    public void powerOff() {
        power(false);
    }
}
