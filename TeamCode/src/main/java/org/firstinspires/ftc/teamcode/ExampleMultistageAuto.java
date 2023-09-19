package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utility.CollectionUtils.pairs;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utility.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp()
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

    private double tagAngleToRads(double providedAngle) {
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
                    List<Pair<AprilTagDetection, AprilTagDetection>> pairs = pairs(aprilTag.getDetections());
                    for (Pair<AprilTagDetection, AprilTagDetection> pair : pairs) {
                        telemetry.addData("with", pair.first.id + " and " + pair.second.id);
                        Vector2 solution = detectionPairToCameraGlobalPos(pair.first, pair.second);
                        telemetry.addData("i'm at", solution);
                        telemetry.addLine();
                    }
                    telemetry.update();
                    sleep(1000);
                }
            }
        } finally {
            if (portal != null) portal.close();
        }
    }

    private Vector2 detectionToCameraGlobalPos2(AprilTagDetection detection) {
        double K = detection.ftcPose.range;
        double phi = -tagAngleToRads(detection.ftcPose.yaw);
        telemetry.addData("K (range)", "%.3f", K);
        telemetry.addData("Yaw (radians)", "%.3f", phi);
        // the yaw happens to be the angle at the camera
        // such that the two legs of a right triangle w/ phi are aligned with the tag

        double global_X = K * Math.sin(phi); // would be Y rel. to camera
        double global_Y = K * Math.cos(phi); // would be X rel. to camera
        telemetry.addData("Tag-relative", new Vector2(global_X, global_Y));
        Vector2 F_tag = TAG_POSITION.get(detection.id);
        if (F_tag == null) {
            throw new IllegalArgumentException("i don't know where tag#" + detection.id + " is");
        }
        // offset the relative position with the known pos of the tag
        return F_tag.add(new Vector2(global_X, global_Y));
    }

    private Vector2 detectionPairToCameraGlobalPos(AprilTagDetection one, AprilTagDetection two) {
        double Ka = one.ftcPose.range;
        double Kb = two.ftcPose.range;
        telemetry.addData("Ka (range)", Ka);
        telemetry.addData("Kb (range)", Kb);
        Vector2 onePosition = TAG_POSITION.get(one.id);
        Vector2 twoPosition = TAG_POSITION.get(two.id);
        if (onePosition == null) throw new IllegalArgumentException("i don't know where tag#" + one.id + " is");
        if (twoPosition == null) throw new IllegalArgumentException("i don't know where tag#" + two.id + " is");

        // let D = half the distance between the two tags...
        double d2 = Math.abs(onePosition.x - twoPosition.x);
        double d = d2 / 2.0;

        telemetry.addData("2*d", d2);
        telemetry.addData("d", d);

        // derived solution to X with pythagorean theorem
        double x = (Math.pow(Ka, 2) - Math.pow(Kb, 2)) / (4 * d);
        // Ka^2 - (x + d)^2 = Y^2
        // solve for Y by plugging back into Pythagorean Theorem
        double y1 = Math.sqrt(Math.pow(Ka, 2) - Math.pow(x + d, 2));
        // Other solution (same answer, different variables)
        double y4 = Math.sqrt(Math.pow(Kb, 2) - Math.pow(x - d, 2));
        telemetry.addData("y1", y1);
        telemetry.addData("y4", y4);
        // add to the midpoint between the two tags (tag 1 + D or tag 2 - D)
        return new Vector2(x, y1).add(onePosition.add(twoPosition).scale(0.5));
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
