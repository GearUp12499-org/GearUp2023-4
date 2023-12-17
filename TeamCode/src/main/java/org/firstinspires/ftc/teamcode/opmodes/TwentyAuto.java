package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.TurnPID;
import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

public abstract class TwentyAuto extends LinearOpMode {
    private VisionPortal portal;
    private AdvSphereProcess sphere;
    private DriveForwardPID forwardPID;
    private TurnPID turnPID;
    private RobotConfiguration robot;

    /**
     * Detection mode (Red or Blue.)
     *
     * @return Red or Blue
     */
    protected abstract AdvSphereProcess.Mode modeConf();

    /**
     * Robot starting position
     *
     * @return Left of truss Or Right of truss
     */
    protected abstract StartPosition positionConf();

    private void placePixel() {
        sleep(500);
        robot.purpleDropper().setPosition(Var.PixelDropper.down);
        sleep(1000);
        // interpolate.
        ElapsedTime dt = new ElapsedTime();
        dt.reset();
        while (dt.time() < 1.0) {
            robot.purpleDropper().setPosition(dt.time() * 0.1 + Var.PixelDropper.down);
        }
        robot.purpleDropper().setPosition(Var.PixelDropper.back);
        sleep(500);
    }

    void leftRight() {
        RobotLog.ii("TwentyAuto", "LEFT");
        forwardPID.DriveReverse(21.0, telemetry);
        turnPID.TurnRobot(45.0, telemetry);
        forwardPID.DriveReverse(3.0, telemetry);
        placePixel();
    }

    void centerRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        forwardPID.DriveReverse(27.0, telemetry);
        placePixel();
    }

    void rightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        forwardPID.DriveReverse(16.0, telemetry);
        turnPID.TurnRobot(-60.0, telemetry);
        placePixel();
    }

    void leftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        forwardPID.DriveReverse(17.0, telemetry);
        turnPID.TurnRobot(5.0, telemetry);
        placePixel();
    }

    void centerLeft() {
        RobotLog.ii("TwentyAuto", "CENTER");
        forwardPID.DriveReverse(24.0, telemetry);
        turnPID.TurnRobot(-20.0, telemetry);
        placePixel();
    }

    void rightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        forwardPID.DriveReverse(20.0, telemetry);
        turnPID.TurnRobot(-90.0, telemetry);
        forwardPID.DriveReverse(2.0, telemetry);
        placePixel();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Get robot hardware configs
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        forwardPID = new DriveForwardPID(robot);
        turnPID = new TurnPID(robot);

        // Set up robot hardware
        robot.clearEncoders();
        robot.purpleDropper().setPosition(Var.PixelDropper.up);

        setupVision(hardwareMap.get(CameraName.class, "Webcam 1"));

        // Display current detection results
        while (opModeInInit()) {
            telemetry.addData("Detection", sphere.getResult());
            telemetry.addData("Strategy  ", sphere.getStrategy());
            telemetry.addData("Mode      ", sphere.getMode());
            telemetry.addData("Scoring   ", String.format(
                    Locale.getDefault(),
                    "%d / %d / %d",
                    sphere.getVotesLeft(),
                    sphere.getVotesCenter(),
                    sphere.getVotesRight()
            ));
            telemetry.update();
        }
        waitForStart();
        if (!opModeIsActive()) return;

        // Capture the result and stop the camera to save resources
        AdvSphereProcess.Result result = sphere.getResult();
        portal.close();
        RobotLog.ii("TwentyAuto", "woah it's a " + result);
        telemetry.addData("Action", result);
        telemetry.update();

        Runnable left = positionConf() == StartPosition.LeftOfTruss ? this::leftLeft : this::leftRight;
        Runnable center = positionConf() == StartPosition.LeftOfTruss ? this::centerLeft : this::centerRight;
        Runnable right = positionConf() == StartPosition.LeftOfTruss ? this::rightLeft : this::rightRight;

        switch (result) {
            case Left:
                left.run();
                break;
            case Right:
                right.run();
                break;
            case None:
            case Center:
                center.run();
                break;
        }

        // Stop motors just in case
        robot.driveMotors().setAll(0.0);
        while (opModeIsActive()) {
            sleep(50);
        }
    }

    void setupVision(CameraName camera) {
        sphere = new AdvSphereProcess(modeConf(), positionConf() == StartPosition.LeftOfTruss);
        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(sphere)
                .setCameraResolution(new Size(864, 480))
                .build();
    }

    public enum StartPosition {
        LeftOfTruss,
        RightOfTruss
    }
}
