package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.TurnPID;
import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class TwentyAuto extends LinearOpMode {
    private VisionPortal portal;
    private AdvSphereProcess sphere;
    private DriveForwardPID forwardPID;
    private TurnPID turnPID;
    private RobotConfiguration robot;

    /**
     * Detection mode (Red or Blue.)
     * @return Red or Blue
     */
    protected abstract AdvSphereProcess.Mode modeConf();

    private void placePixel() {
        sleep(500);
        robot.purpleDropper().setPosition(Var.PixelDropper.down);
        sleep(1000);
        robot.purpleDropper().setPosition(Var.PixelDropper.back);
        sleep(500);
    }

    void left() {
        forwardPID.DriveReverse(26.0, telemetry);
        turnPID.TurnRobot(45.0, telemetry);
        forwardPID.DriveReverse(2.0, telemetry);
        placePixel();
    }

    void center() {
        forwardPID.DriveReverse(27.0, telemetry);
        placePixel();
    }

    void right() {
        forwardPID.DriveReverse(27.0, telemetry);
        turnPID.TurnRobot(-80.0, telemetry);
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
            telemetry.update();
        }
        waitForStart();
        if (!opModeIsActive()) return;

        // Capture the result and stop the camera to save resources
        AdvSphereProcess.Result result = sphere.getResult();
        portal.close();

        switch (result) {
            case Left:
                left();
                break;
            case Right:
                right();
                break;
            case None:
            case Center:
                center();
                break;
        }

        // Stop motors just in case
        robot.driveMotors().setAll(0.0);
        while (opModeIsActive()) {
            sleep(50);
        }
    }

    void setupVision(CameraName camera) {
        sphere = new AdvSphereProcess(modeConf());
        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(sphere)
                .setCameraResolution(new Size(864, 480))
                .build();
    }
}
