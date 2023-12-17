package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.TeleOp.kvoid;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.abstractions.Dumper;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.TurnPID;
import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.Task;
import dev.aether.collaborative_multitasking.ext.TimingKt;
import kotlin.Pair;

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

    /**
     * Robot parking options
     *
     * @return Park Left / Right / None
     */
    protected abstract Parking parkingConf();

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

    void unLeftRight() {
        RobotLog.ii("TwentyAuto", "LEFT");
        forwardPID.DriveForward(3.0, telemetry);
        turnPID.TurnRobot(-45.0, telemetry);
        forwardPID.DriveForward(17.0, telemetry);
    }

    void centerRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        forwardPID.DriveReverse(27.0, telemetry);
        placePixel();
    }

    void unCenterRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        forwardPID.DriveForward(23.0, telemetry);
    }

    void rightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        forwardPID.DriveReverse(16.0, telemetry);
        turnPID.TurnRobot(-60.0, telemetry);
        placePixel();
    }

    void unRightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        turnPID.TurnRobot(60.0, telemetry);
        sleep(250);
        forwardPID.DriveForward(12.0, telemetry);
    }

    void leftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        forwardPID.DriveReverse(17.0, telemetry);
        turnPID.TurnRobot(5.0, telemetry);
        placePixel();
    }

    void unLeftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        turnPID.TurnRobot(-5.0, telemetry);
        forwardPID.DriveForward(13.0, telemetry);
    }

    void centerLeft() {
        RobotLog.ii("TwentyAuto", "CENTER");
        forwardPID.DriveReverse(24.0, telemetry);
        turnPID.TurnRobot(-20.0, telemetry);
        placePixel();
    }

    void unCenterLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        turnPID.TurnRobot(20.0, telemetry);
        forwardPID.DriveForward(20.0, telemetry);
    }

    void rightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        forwardPID.DriveReverse(20.0, telemetry);
        turnPID.TurnRobot(-90.0, telemetry);
        forwardPID.DriveReverse(2.0, telemetry);
        placePixel();
    }

    void unRightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        forwardPID.DriveForward(2.0, telemetry);
        turnPID.TurnRobot(90.0, telemetry);
        forwardPID.DriveForward(16.0, telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Get robot hardware configs
        MultitaskScheduler scheduler = new MultitaskScheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        forwardPID = new DriveForwardPID(robot);
        turnPID = new TurnPID(robot);
        Dumper dumper = new Dumper(scheduler, robot);

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
            telemetry.addData("Circles   ", String.format(
                    Locale.getDefault(),
                    "%d / %d / %d",
                    sphere.getCirclesLeft(),
                    sphere.getCirclesCenter(),
                    sphere.getCirclesRight()
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

        boolean posLeft = positionConf() == StartPosition.LeftOfTruss;
        Runnable left = posLeft ? this::leftLeft : this::leftRight;
        Runnable undoLeft = posLeft ? this::unLeftLeft : this::unLeftRight;
        Runnable center = posLeft ? this::centerLeft : this::centerRight;
        Runnable undoCenter = posLeft ? this::unCenterLeft : this::unCenterRight;
        Runnable right = posLeft ? this::rightLeft : this::rightRight;
        Runnable undoRight = posLeft ? this::unRightLeft : this::unRightRight;

        boolean shouldUndo = parkingConf() != Parking.NoParking;

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

        if (shouldUndo) {
            switch (result) {
                case Left:
                    undoLeft.run();
                    break;
                case Right:
                    undoRight.run();
                    break;
                case None:
                case Center:
                    undoCenter.run();
                    break;
            }
            double modifier = parkingConf() == Parking.MoveLeft ? 1 : -1;
            turnPID.TurnRobot(modifier * 100.0, telemetry);
            forwardPID.DriveReverse(33.0, telemetry, 5.0);

//            robot.liftLeft().setTargetPosition(1000);

            // encoder ticks
            int target = 1000;
            int acceptError = 50;
            /*
             * 1. go to the target position within the acceptError
             * 2. dump the box
             * 3. wait 500ms
             * 4. reset the box
             * 5. go to 0 within acceptError
             *
             * run to completion.
             */
            Task t1 = scheduler.task((x) -> {
                x.onStart(() -> {
                    robot.liftLeft().setTargetPosition(target);
                    return kvoid;
                });
                x.isCompleted(() -> Math.abs(robot.liftLeft().getCurrentPosition() - target) < acceptError);
                return kvoid;
            });
            Pair<Task, Task> beforeAfter = dumper.autoDumpSecond();
            t1.then(beforeAfter.getFirst());
            beforeAfter.getSecond()
                    .then((x) -> {
                        TimingKt.maxDuration(x, 500);
                        return kvoid;
                    })
                    .then(dumper.autoReset())
                    .then((x) -> {
                        x.onStart(() -> {
                            robot.liftLeft().setTargetPosition(0);
                            return kvoid;
                        });
                        x.isCompleted(() -> Math.abs(robot.liftLeft().getCurrentPosition()) < acceptError);
                        return kvoid;
                    });

            scheduler.runToCompletion(this::opModeIsActive);
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

    public enum Parking {
        MoveLeft,
        MoveRight,
        NoParking
    }
}
