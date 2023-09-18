package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.Line;
import org.firstinspires.ftc.teamcode.utility.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class ExampleMultistageAuto extends LinearOpMode {
    public static final Map<Integer, Vector2> TAG_POSITION = new HashMap<>();
    // Change this if you switch the angle mode on the AprilTags.
    private static final double APRIL_TAG_ANGLE_CONVERSION_FACTOR = Math.PI / 180.0;

    static {
        final double DISTANCE_FROM_WALL = 10.75/* in */;
        TAG_POSITION.put(1, new Vector2(29.5, DISTANCE_FROM_WALL));
        TAG_POSITION.put(2, new Vector2(35.5, DISTANCE_FROM_WALL));
        TAG_POSITION.put(3, new Vector2(41.5, DISTANCE_FROM_WALL));
    }

    /**
     * Stop trying to adjust when this close in inches
     */
    public static final double DISTANCE_THRESHOLD = 0.5;
    private WebcamName webcam;
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    private FtcDashboard dashboardInstance;

    private double aprilTagAngleToRadians(double providedAngle) {
        return providedAngle * APRIL_TAG_ANGLE_CONVERSION_FACTOR;
    }

    @Override
    public void runOpMode() {
        try {
            dashboardInstance = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboardInstance.getTelemetry());
            webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            initVisionPortal();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    telemetry.update();
                    sleep(20);
                }
            }
        } finally {
            if (portal != null) portal.close();
        }
    }

    private Vector2 getCameraFieldLocalPosition(AprilTagDetection detection) {
        double distance = detection.ftcPose.range;
        double thetaTotal = aprilTagAngleToRadians(detection.ftcPose.bearing);
        double thetaFromStraightOn = aprilTagAngleToRadians(detection.ftcPose.yaw);
        // sin thetaTotal = localX / distance
        // distance sin thetaLocal = localX
        double localX = distance * Math.sin(thetaTotal);
        double localY = distance * Math.cos(thetaTotal);
        Vector2 localTargetPosition = new Vector2(localX, localY);
        Line cameraLocalY = Line.pointPoint(new Vector2(0, 0), new Vector2(0, 1));
        Line cameraLocalX = Line.pointPoint(new Vector2(0, 0), new Vector2(1, 0));
    }

    private void initVisionPortal() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(webcam);
        visionPortalBuilder.addProcessor(aprilTag);
        portal = visionPortalBuilder.build();
    }
}
