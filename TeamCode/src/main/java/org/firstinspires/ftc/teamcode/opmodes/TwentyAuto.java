package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.abstractions.AprilTagToPoseKt.detectSingleToPose;
import static org.firstinspires.ftc.teamcode.opmodes.TeleOp.kvoid;

import android.util.Log;
import android.util.Size;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Var;
import org.firstinspires.ftc.teamcode.abstractions.ApproachObject2;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive;
import org.firstinspires.ftc.teamcode.odo.SyncFail;
import org.firstinspires.ftc.teamcode.odo.TurnPID;
import org.firstinspires.ftc.teamcode.utilities.DegreeUnit;
import org.firstinspires.ftc.teamcode.utilities.FootUnit;
import org.firstinspires.ftc.teamcode.utilities.InchUnit;
import org.firstinspires.ftc.teamcode.utilities.LengthUnit;
import org.firstinspires.ftc.teamcode.utilities.Move;
import org.firstinspires.ftc.teamcode.utilities.Pose;
import org.firstinspires.ftc.teamcode.utilities.RadianUnit;
import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.function.BiConsumer;

import dev.aether.collaborative_multitasking.MultitaskScheduler;
import dev.aether.collaborative_multitasking.ext.TimingKt;

public abstract class TwentyAuto extends LinearOpMode {
    private VisionPortal portal;
    private AdvSphereProcess sphere;
    private AprilTagProcessor aprilTag;
    private TurnPID turnPID;
    private SyncFail why;
    private RobotConfiguration robot;
    private ApproachObject2 theXButton;
    private KOdometryDrive newOdo;

    private static final double RedArrivalTime = .75;

    private double clearDuration = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    public static final Move cameraAdjustment = new Move(
            new InchUnit(6.75),
            new InchUnit(0),
            new RadianUnit(0)
    );

    private static final HashMap<Integer, Double> tagXPositions = new HashMap<>();

    static {
        tagXPositions.put(1, 29.5);
        tagXPositions.put(2, 35.5);
        tagXPositions.put(3, 41.5);

        tagXPositions.put(4, 101.5);
        tagXPositions.put(5, 107.5);
        tagXPositions.put(6, 113.5);
    }

    /**
     * Detection mode (Red or Blue.)
     *
     * @return Red or Blue
     */
    protected abstract AdvSphereProcess.Mode modeConf();

    protected abstract AllianceColor allianceColor();

    protected abstract FieldGlobalPosition globalPosition();

    private ElapsedTime timer2 = new ElapsedTime();
    private MultitaskScheduler scheduler = null;


    private boolean panic_button() {
        if (timer2.time() >= 28.5) {
            scheduler.filteredStop(e -> true);
            scheduler.panic();
            robot.dumperRotate().setPosition(Var.Box.idleRotate);
            robot.liftLeft().setTargetPosition(0);
            while (opModeIsActive()) {
                sleep(10);
            }
            throw new Panic();
        }
        return opModeIsActive();
    }

    /**
     * Robot starting position
     *
     * @return Left of truss Or Right of truss
     */
    protected StartPosition positionConf() {
        if (allianceColor() == AllianceColor.Red) {
            if (globalPosition() == FieldGlobalPosition.Backstage)
                return StartPosition.RightOfTruss;
            else
                return StartPosition.LeftOfTruss;
        } else {
            if (globalPosition() == FieldGlobalPosition.Backstage) {
                return StartPosition.LeftOfTruss;
            } else {
                return StartPosition.RightOfTruss;
            }
        }
    }

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
        while (dt.time() < 0.5) {
            robot.purpleDropper().setPosition(dt.time() * 0.2 + Var.PixelDropper.down);
        }
        robot.purpleDropper().setPosition(Var.PixelDropper.back);
        sleep(500);
    }

    void leftRight() {
        RobotLog.ii("TwentyAuto", "LEFT");
        why.DriveReverse(21.0, telemetry);
        turnPID.TurnRobot(45.0);
        why.DriveReverse(2.0, telemetry);
        placePixel();
    }

    void unLeftRight() {
        RobotLog.ii("TwentyAuto", "LEFT");
        why.DriveForward(2.0, telemetry);
        turnPID.TurnRobot(-45.0);
        why.DriveForward(17.0, telemetry);
    }

    void centerRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        why.DriveReverse(25.0, telemetry);
        placePixel();
    }

    void unCenterRight() {
        RobotLog.ii("TwentyAuto", "CENTER");
        why.DriveForward(21.0, telemetry);
    }

    void rightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        why.DriveReverse(14.0, telemetry);
        turnPID.TurnRobot(-60.0);
        placePixel();
    }

    void unRightRight() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        turnPID.TurnRobot(60.0);
        sleep(250);
        why.DriveForward(10.0, telemetry);
    }

    void leftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        why.DriveReverse(17.0, telemetry);
        turnPID.TurnRobot(5.0);
        placePixel();
    }

    void unLeftLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        turnPID.TurnRobot(-5.0);
        why.DriveForward(1.0, telemetry);
    }

    void centerLeft() {
        RobotLog.ii("TwentyAuto", "CENTER");
        why.DriveReverse(22.0, telemetry);
        turnPID.TurnRobot(-20.0);
        placePixel();
    }

    void unCenterLeft() {
        RobotLog.ii("TwentyAuto", "LEFT");
        turnPID.TurnRobot(20.0);
        why.DriveForward(6.0, telemetry);
    }

    void rightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        why.DriveReverse(20.0, telemetry);
        turnPID.TurnRobot(-90.0);
        why.DriveReverse(4.0, telemetry);
        placePixel();
    }

    void unRightLeft() {
        RobotLog.ii("TwentyAuto", "RIGHT");
        why.DriveForward(4.0, telemetry);
        turnPID.TurnRobot(90.0);
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
                    TimingKt.maxDuration(e, 3000);
                    return kvoid;
                })
                .then(xButton.approach(new InchUnit(3.0)))
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
                .then(drive.driveForward(new InchUnit(5.0)));
        scheduler.runToCompletion(this::panic_button);
    }

    /**
     * ideally wait 100ms before calling.
     *
     * @param aprilTag april tag vision source.
     * @param targetID target tag id.
     */
    private void alignTag(AprilTagProcessor aprilTag, int targetID) {
        Log.i("20auto", "Aligning with tag " + targetID);
        // Rotate right by theta + 90deg
        List<AprilTagDetection> detections = aprilTag.getDetections();
        @Nullable AprilTagDetection primary = null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetID) primary = detection;
        }
        if (primary == null) {
            if (detections.size() > 0)
                primary = detections.get(0);
            else {
                // uhhhh
//                throw new IllegalStateException("what the hell am i supposed to do here");
                Log.w("TwentyAuto", "OH SHIT WE CAN'T SEE THE TAGS");
                return; // whelp
            }
        }

        Pose posed = detectSingleToPose(primary).transform(cameraAdjustment);
        Log.i("20auto", "    align: pose estimate: " + posed);

        double turnDegs = posed.getTheta()
                .plus(new DegreeUnit(90))
                .to().getDegrees()
                .getValue();

        turnPID.TurnRobot(-turnDegs);
        sleep(100);
        // haha unboxing
        Double t = tagXPositions.get(targetID);
        if (t == null)
            throw new IllegalArgumentException("Not sure what " + targetID + "'s tag position is");
        double xPosition = t;

        why.StrafeLeft(xPosition - posed.getX().getValue());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            run();
        } catch (Panic e) {
            Log.e("20auto", "panic failed");
        }
    }

    public void run() {
        // Get robot hardware configs
        scheduler = new MultitaskScheduler();
        setupVision(hardwareMap.get(CameraName.class, "Webcam 1"));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        turnPID = new TurnPID(robot);
        newOdo = new KOdometryDrive(scheduler, robot);
        why = new SyncFail(scheduler, newOdo, this::panic_button);
        theXButton = new ApproachObject2(scheduler, robot, 18.0);

        // Set up robot hardware
        robot.clearEncoders();
        robot.purpleDropper().setPosition(Var.PixelDropper.up);
        robot.dropDownServo().setPosition(Var.DropDownServo.up);

        double buttonRepeatDelay = 0.4;
        double buttonRepeatRate = 0.2;
        double maxAwait = 22.0;

        ElapsedTime dUpRepeat = new ElapsedTime();
        double upNextInterval = buttonRepeatRate;
        boolean lastDUp = false;
        ElapsedTime dDownRepeat = new ElapsedTime();
        double downNextInterval = buttonRepeatRate;
        boolean lastDDown = false;

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
            telemetry.addLine();
            telemetry.addLine("Configuration (GP1)");
            telemetry.addData("TimeToPartnerCleared", clearDuration);
            telemetry.addLine("dpad +/- 1s");
            telemetry.update();
            if (gamepad1.dpad_up && !lastDUp) {
                clearDuration += 1.0;
                upNextInterval = 0.0;
                dUpRepeat.reset();
            }
            if (gamepad1.dpad_up) {
                double timer = dUpRepeat.time() - buttonRepeatDelay;
                if (timer >= 0 && timer >= upNextInterval) {
                    upNextInterval += buttonRepeatRate;
                    clearDuration += 1.0;
                }
            }
            if (gamepad1.dpad_down && !lastDDown) {
                clearDuration -= 1.0;
                downNextInterval = 0.0;
                dDownRepeat.reset();
            }
            if (gamepad1.dpad_down) {
                double timer = dDownRepeat.time() - buttonRepeatDelay;
                if (timer >= 0 && timer >= downNextInterval) {
                    downNextInterval += buttonRepeatRate;
                    clearDuration -= 1.0;
                }
            }
            if (clearDuration < 0) clearDuration = 0;
            if (clearDuration > maxAwait) clearDuration = maxAwait;

            lastDUp = gamepad1.dpad_up;
            lastDDown = gamepad1.dpad_down;
        }
        waitForStart();
        if (!opModeIsActive()) return;

        if (allianceColor() == AllianceColor.Red) {
            clearDuration -= RedArrivalTime;
        } else {
            clearDuration -= RedArrivalTime; // TODO
        }
        timer.reset();
        timer2.reset();

        // Capture the result
        AdvSphereProcess.Result result = sphere.getResult();
        RobotLog.ii("TwentyAuto", "woah it's a " + result);
        telemetry.addData("Action", result);
        telemetry.update();

        boolean posLeft = positionConf() == StartPosition.LeftOfTruss;

        int targetTag = 0;

        Runnable left = posLeft ? this::leftLeft : this::leftRight;
        Runnable center = posLeft ? this::centerLeft : this::centerRight;
        Runnable right = posLeft ? this::rightLeft : this::rightRight;

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

        if (allianceColor() == AllianceColor.Red) targetTag += 3;

        if (globalPosition() == FieldGlobalPosition.Backstage) {
            // Return to common position. Not amazing efficiency but it's what we had.
            backstage(result, posLeft, targetTag);
        } else { // Frontstage scripts
            frontstage(result, targetTag);
        }

        // Stop motors just in case
        robot.driveMotors().setAll(0.0);
        while (opModeIsActive()) {
            sleep(50);
        }
    }

    private void frontstage(AdvSphereProcess.Result result, int targetTag) {
        Consumer<Double> rightOnBlue = allianceColor() == AllianceColor.Blue ? why::StrafeRight : why::StrafeLeft;
        navFrontstage(result, rightOnBlue);

        sleep(200);
        alignTag(aprilTag, targetTag);
        sleep(200);
        alignTag(aprilTag, targetTag);
        scoreYellowPixel(scheduler, robot, theXButton, newOdo);

        // back off More
        newOdo.driveForward(new InchUnit(2), 2.0) // this would be a REALLY BAD place to get stuck
                .then(e -> {
                    e.onStart(() -> {
                        robot.dumperRotate().setPosition(Var.Box.idleRotate);
                        return kvoid;
                    });
                    TimingKt.maxDuration(e, 300);
                    return kvoid;
                }).then(e -> {
                    e.require(robot.getLiftLock());
                    e.onStart(() -> {
                        robot.liftLeft().setTargetPosition(0);
                        robot.liftRight().setTargetPosition(0);
                        return kvoid;
                    });
                    e.isCompleted(() ->
                            !(robot.liftLeft().isBusy() || robot.liftRight().isBusy()));
                    TimingKt.maxDuration(e, 3000);
                    return kvoid;
                });

        scheduler.runToCompletion(this::panic_button);
    }

    private void backstage(AdvSphereProcess.Result result, boolean posLeft, int targetTag) {
        Runnable undoLeft = posLeft ? this::unLeftLeft : this::unLeftRight;
        Runnable undoCenter = posLeft ? this::unCenterLeft : this::unCenterRight;
        Runnable undoRight = posLeft ? this::unRightLeft : this::unRightRight;

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

        Consumer<LengthUnit> awayFromWall =
                posLeft ? newOdo::strafeLeft : newOdo::strafeRight;
        BiConsumer<LengthUnit, Double> toWallWithTimeout =
                posLeft ? newOdo::strafeRight : newOdo::strafeLeft;

        navBackstage(scheduler, result, awayFromWall);

        sleep(200);

        alignTag(aprilTag, targetTag);
        sleep(200);
        alignTag(aprilTag, targetTag);
        scoreYellowPixel(scheduler, robot, theXButton, newOdo);

        //noinspection DataFlowIssue i am going to shove you into a box :))))
        LengthUnit strafeMotion = new InchUnit(tagXPositions.getOrDefault(targetTag, 35.5));
        if (allianceColor() == AllianceColor.Red)
            strafeMotion = new FootUnit(12).minus(strafeMotion);
        strafeMotion = strafeMotion
                .minus(Var.AutoPositions.RobotWidth.div(2))
                .minus(new InchUnit(2));

        toWallWithTimeout.accept(strafeMotion, 3.0);
        scheduler.runToCompletion(this::panic_button);

        /*
         * 1. go to the target position within the acceptError
         * 2. dump the box
         * 3. wait 500ms
         * 4. reset the box
         * 5. go to 0 within acceptError
         *
         * run to completion.
         */
        robot.dumperRotate().setPosition(Var.Box.idleRotate);
        newOdo.driveReverse(new InchUnit(6.0), 2.0);
        scheduler.task(e -> {
            TimingKt.maxDuration(e, 300);
            return kvoid;
        }).then(e -> {
            e.require(robot.getLiftLock());
            e.onStart(() -> {
                robot.liftLeft().setTargetPosition(0);
                robot.liftRight().setTargetPosition(0);
                return kvoid;
            });
            e.isCompleted(() ->
                    !(robot.liftLeft().isBusy() || robot.liftRight().isBusy()));
            TimingKt.maxDuration(e, 3000);
            return kvoid;
        });

        scheduler.runToCompletion(this::panic_button);
    }

    private void navBackstage(
            MultitaskScheduler scheduler,
            AdvSphereProcess.Result cvResult,
            Consumer<LengthUnit> doAwayFromWall
    ) {
        double modifier = parkingConf() == Parking.MoveLeft ? 1 : -1;
        turnPID.TurnRobot(modifier * 90.0);
        sleep(100);
        why.DriveReverse(19.0, telemetry, 5.0);
        // Left: 9.5", Center: 15", Right: 21.5"

        int nearMedFar;
        switch (cvResult) {
            case Left:
                nearMedFar = 0;
                break;
            case None:
            case Center:
            default:
                nearMedFar = 1;
                break;
            case Right:
                nearMedFar = 2;
                break;
        }

        if (allianceColor() == AllianceColor.Red) nearMedFar = 2 - nearMedFar;
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);

//        switch (nearMedFar) {
//            case 0:
//                doAwayFromWall.accept(
//                        new InchUnit(-1).plus(Var.AutoPositions.RobotWidth.div(2))
//                );
//                break;
//            case 1:
//            default:
//                doAwayFromWall.accept(
//                        new InchUnit(4.5).plus(Var.AutoPositions.RobotWidth.div(2))
//                );
//                break;
//            case 2:
//                doAwayFromWall.accept(
//                        new InchUnit(11.0).plus(Var.AutoPositions.RobotWidth.div(2))
//                );
//                break;
//        }
        scheduler.runToCompletion(this::panic_button);
    }

    public static final double[] frontstageTravelDistances = {18.5, 20.0, 27.0};

    private void frontstageLeft() {
        turnPID.TurnRobot(-45.0);
        why.StrafeLeft(2);
        why.DriveReverse(27);
        sleep(250);
        turnPID.TurnRobot(95);
        why.DriveReverse(48);
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
        while (timer.time() < clearDuration) {
            telemetry.addLine(String.format(
                    Locale.getDefault(),
                    "Waiting to continue %.2fs / %.2fs",
                    timer.time(),
                    clearDuration
            ));
            telemetry.update();
            sleep(50);
        }
        why.DriveReverse(32);
    }

    private void frontstageLeftB() {
        turnPID.TurnRobot(-5.0);
        why.StrafeLeft(4);
        why.DriveReverse(32);
        sleep(250);
        turnPID.TurnRobot(-95);
        why.DriveReverse(48);
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
        while (timer.time() < clearDuration) {
            telemetry.addLine(String.format(
                    Locale.getDefault(),
                    "Waiting to continue %.2fs / %.2fs",
                    timer.time(),
                    clearDuration
            ));
            telemetry.update();
            sleep(50);
        }
        why.DriveReverse(30);
    }

    private void frontstageCenter() {
        why.StrafeLeft(7);
        why.DriveReverse(24);
        turnPID.TurnRobot(95);
        why.DriveReverse(48);
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
        while (timer.time() < clearDuration) {
            telemetry.addLine(String.format(
                    Locale.getDefault(),
                    "Waiting to continue %.2fs / %.2fs",
                    timer.time(),
                    clearDuration
            ));
            telemetry.update();
            sleep(50);
        }
        why.DriveReverse(39);
    }

    private void frontstageCenterB() {
        turnPID.TurnRobot(20);
        why.StrafeRight(18);
        why.DriveReverse(24);
        turnPID.TurnRobot(-90);
        why.DriveReverse(72);
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
        while (timer.time() < clearDuration) {
            telemetry.addLine(String.format(
                    Locale.getDefault(),
                    "Waiting to continue %.2fs / %.2fs",
                    timer.time(),
                    clearDuration
            ));
            telemetry.update();
            sleep(50);
        }
        why.DriveReverse(22);
    }

    private void frontstageRight() {
        turnPID.TurnRobot(60);
        why.StrafeRight(4.5);
        why.DriveReverse(34);
        sleep(250);
        turnPID.TurnRobot(90);
        why.DriveReverse(48);
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
        while (timer.time() < clearDuration) {
            telemetry.addLine(String.format(
                    Locale.getDefault(),
                    "Waiting to continue %.2fs / %.2fs",
                    timer.time(),
                    clearDuration
            ));
            telemetry.update();
            sleep(50);
        }
        why.DriveReverse(30);
    }

    private void frontstageRightB() {
        why.DriveForward(2.0);
        turnPID.TurnRobot(90);
        why.DriveReverse(28);
        sleep(250);
        turnPID.TurnRobot(-90);
        why.DriveReverse(48);
        robot.liftLeft().setTargetPosition(Var.AutoPositions.LiftScoring);
        while (timer.time() < clearDuration) {
            telemetry.addLine(String.format(
                    Locale.getDefault(),
                    "Waiting to continue %.2fs / %.2fs",
                    timer.time(),
                    clearDuration
            ));
            telemetry.update();
            sleep(50);
        }
        why.DriveReverse(30);
    }

    private void navFrontstage(
            AdvSphereProcess.Result cvResult,
            Consumer<Double> rightOnBlue
    ) {
        boolean main = allianceColor() == AllianceColor.Blue;

        int travelDistanceIdx;
        switch (cvResult) {
            case Left:
                travelDistanceIdx = 2;
                if (main)
                    frontstageLeft();
                else
                    frontstageLeftB();
                break;
            case Center:
            case None:
            default:
                travelDistanceIdx = 1;
                if (main)
                    frontstageCenter();
                else
                    frontstageCenterB();
                break;
            case Right:
                travelDistanceIdx = 0;
                if (main)
                    frontstageRight();
                else
                    frontstageRightB();
                break;
        }

        if (allianceColor() == AllianceColor.Red) {
            travelDistanceIdx = 2 - travelDistanceIdx;
        }

        double distance = frontstageTravelDistances[travelDistanceIdx];
        rightOnBlue.accept(distance);
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
        NoParking
    }

    public enum AllianceColor {
        Red,
        Blue
    }

    public enum FieldGlobalPosition {
        Frontstage,
        Backstage
    }
}
