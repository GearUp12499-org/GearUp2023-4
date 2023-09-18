package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

public class ExampleMultistageAuto extends LinearOpMode {
    @SafeVarargs
    private static <T> Set<T> setOf(T... items) {
        Set<T> theSet = new HashSet<>();
        Collections.addAll(theSet, items);
        return theSet;
    }

    enum Target {
        LEFT(setOf("BlueAllianceLeft", "RedAllianceLeft")),
        CENTER(setOf("BlueAllianceCenter", "RedAllianceCenter")),
        RIGHT(setOf("BlueAllianceRight", "RedAllianceRight"));
        public final Set<String> tags;

        Target(Set<String> tags) {
            this.tags = tags;
        }
    }

    private Target theTarget;
    private WebcamName webcam;
    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    private FtcDashboard dashboardInstance;


    @Override
    public void runOpMode() {
        try {
            theTarget = Target.CENTER;

            dashboardInstance = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboardInstance.getTelemetry());
            webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            initVisionPortal();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    AprilTagDetection tag = getMatchingTag();
                    telemetry.update();
                    sleep(20);
                }
            }
        } finally {
            if (portal != null) portal.close();
        }
    }

    private AprilTagDetection getMatchingTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        Optional<AprilTagDetection> relevantTag = detections.stream().filter(x -> theTarget.tags.contains(x.metadata.name)).findFirst();
        return relevantTag.orElse(null);
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
