package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class TakePictures extends LinearOpMode {
    WebcamName webcam;
    VisionPortal portal;
    SphereProcess spheres;


    @Override
    public void runOpMode() throws InterruptedException {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        initVisionPortal();
        waitForStart();
        boolean lastABtn = false;
        while (opModeIsActive()) {
            telemetry.addLine("Press A to take pictures");
            telemetry.update();
            if (gamepad1.a && !lastABtn) {
                spheres.write();
            }
            lastABtn = gamepad1.a;
        }
    }

    private void initVisionPortal() {
        spheres = new SphereProcess();

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(webcam);
        visionPortalBuilder.setCameraResolution(new Size(864, 480));
        visionPortalBuilder.addProcessor(spheres);
        portal = visionPortalBuilder.build();
    }
}
