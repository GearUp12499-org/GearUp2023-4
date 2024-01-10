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
import org.firstinspires.ftc.teamcode.abstractions.ApproachObject2;
import org.firstinspires.ftc.teamcode.abstractions.Dumper;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.AprilTagUpdateTool;
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID;
import org.firstinspires.ftc.teamcode.odo.ExtractedDriveToTag;
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive;
import org.firstinspires.ftc.teamcode.odo.OdoTracker;
import org.firstinspires.ftc.teamcode.odo.SyncFail;
import org.firstinspires.ftc.teamcode.odo.TurnPID;
import org.firstinspires.ftc.teamcode.utilities.InchUnit;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.Task;
import dev.aether.collaborative_multitasking.ext.TimingKt;
import kotlin.Pair;

public abstract class TwentyAuto extends LinearOpMode {
    private VisionPortal portal;
    private AdvSphereProcess sphere;
    private AprilTagProcessor aprilTag;
    private DriveForwardPID forwardPID;
    private TurnPID turnPID;
    private SyncFail why;
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
        why.DriveReverse(21.0, telemetry);
        turnPID.TurnRobot(45.0, telemetry);
        why.DriveReverse(3.0, telemetry);
        placePixel();
    }

    void unLeftRight() {
        RobotLog.ii("TwentyAuto", "LEFT");
        why.DriveForward(3.0, telemetry);
        turnPID.TurnRobot(-45.0, telemetry);
        why.DriveForward(17.0, telemetry);
    }

    void centerRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        why.DriveReverse(24.0, telemetry);
        placePixel();
    }

    void unCenterRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        why.DriveForward(21.0, telemetry);
    }

    void rightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        why.DriveReverse(16.0, telemetry);
        turnPID.TurnRobot(-60.0, telemetry);
        placePixel();
    }

    void unRightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        turnPID.TurnRobot(60.0, telemetry);
        sleep(250);
        why.DriveForward(12.0, telemetry);
    }

    void leftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        why.DriveReverse(17.0, telemetry);
        turnPID.TurnRobot(5.0, telemetry);
        placePixel();
    }

    void unLeftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        turnPID.TurnRobot(-5.0, telemetry);
        why.DriveForward(13.0, telemetry);
    }

    void centerLeft() {
        RobotLog.ii("TwentyAuto", "CENTER");
        why.DriveReverse(24.0, telemetry);
        turnPID.TurnRobot(-20.0, telemetry);
        placePixel();
    }

    void unCenterLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        turnPID.TurnRobot(20.0, telemetry);
        why.DriveForward(20.0, telemetry);
    }

    void rightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        why.DriveReverse(20.0, telemetry);
        turnPID.TurnRobot(-90.0, telemetry);
        why.DriveReverse(2.0, telemetry);
        placePixel();
    }

    void unRightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        why.DriveForward(2.0, telemetry);
        turnPID.TurnRobot(90.0, telemetry);
        why.DriveForward(16.0, telemetry);
    }

    private void scoreYellowPixel(MultitaskScheduler scheduler, RobotConfiguration robot, ApproachObject2 xButton, KOdometryDrive drive) {
        scheduler.task(e -> {
                    e.onStart(() -> {
                        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
                        robot.dumperLatch().setPosition(Var.Box.latched);
                        robot.dumperRotate().setPosition(Var.Box.dumpRotate);
                        return kvoid;
                    });
                    e.isCompleted(() -> robot.liftLeft().getCurrentPosition() >= Var.AutoPositions.LiftScoring - 20);
                    TimingKt.minDuration(e, 100);
                    return kvoid;
                })
                .then(xButton.approach(new InchUnit(4.0)))
                .then(e -> {
                    TimingKt.delay(e, 200);
                    return kvoid;
                })
                .then(e -> {
                    e.onStart(() -> {
                        robot.dumperLatch().setPosition(Var.Box.unlatched);
                        return kvoid;
                    });
                    TimingKt.maxDuration(e, 1000);
                    return kvoid;
                })
                .then(drive.driveForward(new InchUnit(2.0)));
        scheduler.runToCompletion(this::opModeIsActive);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Get robot hardware configs
        MultitaskScheduler scheduler = new MultitaskScheduler();
        setupVision(hardwareMap.get(CameraName.class, "Webcam 1"));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        forwardPID = new DriveForwardPID(robot);
        turnPID = new TurnPID(robot);
        KOdometryDrive newOdo = new KOdometryDrive(scheduler, robot);
        why = new SyncFail(scheduler, newOdo);
        OdoTracker tracker = new OdoTracker(robot, Pose.zero);
        AprilTagUpdateTool trackerTagUpdate = new AprilTagUpdateTool(aprilTag, 2);
        ExtractedDriveToTag driveToTag = new ExtractedDriveToTag(
                robot, trackerTagUpdate, tracker, 2, telemetry
        );
        ApproachObject2 theXButton = new ApproachObject2(scheduler, robot);
        Dumper dumper = new Dumper(scheduler, robot);

        // Set up robot hardware
        robot.clearEncoders();
        robot.purpleDropper().setPosition(Var.PixelDropper.up);


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

        // Capture the result
        AdvSphereProcess.Result result = sphere.getResult();
        RobotLog.ii("TwentyAuto", "woah it's a " + result);
        telemetry.addData("Action", result);
        telemetry.update();

//        scoreYellowPixel(scheduler, robot, theXButton, newOdo);
//        while (opModeIsActive()) {
//            sleep(100);
//        }
//        if (true) return;

        boolean posLeft = positionConf() == StartPosition.LeftOfTruss;
        Runnable left = posLeft ? this::leftLeft : this::leftRight;
        Runnable undoLeft = posLeft ? this::unLeftLeft : this::unLeftRight;
        Runnable center = posLeft ? this::centerLeft : this::centerRight;
        Runnable undoCenter = posLeft ? this::unCenterLeft : this::unCenterRight;
        Runnable right = posLeft ? this::rightLeft : this::rightRight;
        Runnable undoRight = posLeft ? this::unRightLeft : this::unRightRight;

        boolean shouldUndo = parkingConf() != Parking.NoParking;

        int targetTag = 0;
        switch (result) {
            case Left:
                targetTag = 1;
                left.run();
                break;
            case Right:
                targetTag = 3;
                right.run();
                break;
            case None:
            case Center:
                targetTag = 2;
                center.run();
                break;
        }
        trackerTagUpdate.setInitialTarget(targetTag);
        driveToTag.setTargetPosition(targetTag);

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
            // PUT IF STATEMENT TO BRANCH HERE

            if (parkingConf() == Parking.OtherParking)
                throw new IllegalStateException("Don't know how to handle OtherParking here");
            double modifier = parkingConf() == Parking.MoveLeft ? 1 : -1;
            turnPID.TurnRobot(modifier * 90.0, telemetry);
            sleep(100);
            if (modeConf() == AdvSphereProcess.Mode.Red) {
                why.DriveReverse(32.0);
                Task t1 = scheduler.task((x) -> {
                    x.onStart(() -> {
                        robot.liftLeft().setTargetPosition(1000);
                        return kvoid;
                    });
                    x.isCompleted(() -> Math.abs(robot.liftLeft().getCurrentPosition() - 1000) < 50);
                    return kvoid;
                });
                Pair<Task, Task> beforeAfter = dumper.autoDumpSecond();
                t1
                        .then(beforeAfter)
                        .then((x) -> {
                            TimingKt.maxDuration(x, 500);
                            return kvoid;
                        })
                        .then(dumper.autoReset())
                        .then(newOdo.driveForward(new InchUnit(2)))
                        .then((x) -> {
                            x.onStart(() -> {
                                robot.liftLeft().setTargetPosition(0);
                                return kvoid;
                            });
                            x.isCompleted(() -> Math.abs(robot.liftLeft().getCurrentPosition()) < 50);
                            return kvoid;
                        });
                scheduler.runToCompletion(this::opModeIsActive);
                return;
            }
            why.DriveReverse(19.0, telemetry, 5.0);
            // Left: 9.5", Center: 15", Right: 21.5"
            Task navigateInFrontOfTag;

            scheduler.task(tracker.getTaskFactory());
            scheduler.task(trackerTagUpdate.updateTool(tracker));


            switch (result) {
                case Left:
                    navigateInFrontOfTag = newOdo.strafeLeft(
                            new InchUnit(11.0).plus(Var.AutoPositions.RobotWidth.div(2))
                    );
                    break;
                case None:
                case Center:
                    navigateInFrontOfTag = newOdo.strafeLeft(
                            new InchUnit(16.5).plus(Var.AutoPositions.RobotWidth.div(2))
                    );
                    break;
                case Right:
                    navigateInFrontOfTag = newOdo.strafeLeft(
                            new InchUnit(23.0).plus(Var.AutoPositions.RobotWidth.div(2))
                    );
                    break;
                default:
                    throw new IllegalStateException();
            }
            navigateInFrontOfTag
                    .then(e -> {
                        TimingKt.maxDuration(e, 200);
                        return kvoid;
                    })
                    .then(driveToTag.getTaskFactory())
                    .then(theXButton.approach(new InchUnit(3.5)));

            scheduler.runToCompletion(this::opModeIsActive);
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
                    .then(newOdo.driveForward(new InchUnit(2)))
                    .then((x) -> {
                        x.onStart(() -> {
                            robot.liftLeft().setTargetPosition(0);
                            return kvoid;
                        });
                        x.isCompleted(() -> Math.abs(robot.liftLeft().getCurrentPosition()) < acceptError);
                        return kvoid;
                    })
                    // TODO: make this number depend on detection result
                    .then(newOdo.strafeRight(new InchUnit(24)));
            // TODO: Potentially back up to be sure

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
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(sphere)
                .addProcessor(aprilTag)
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
        NoParking,
        OtherParking
    }
}
