package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class TakePictures extends LinearOpMode {
    WebcamName webcam;
    VisionPortal portal;
    AdvSphereProcess spheres;

    AdvSphereProcess.Mode mode = AdvSphereProcess.Mode.Red;
    boolean altBoxes = false;


    @Override
    public void runOpMode() throws InterruptedException {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        initVisionPortal();
        waitForStart();
        boolean lastABtn = false;
        boolean lastXBtn = false;
        boolean lastYBtn = false;
        while (opModeIsActive()) {
            telemetry.addLine("Press A to capture");
            telemetry.addData("Mode [X]", mode);
            telemetry.addData("Alternative Boxes? [Y]", altBoxes);
            telemetry.update();
            if (gamepad1.a && !lastABtn) {
                spheres.capture();
            }
            if (gamepad1.x && !lastXBtn) {
                mode = mode == AdvSphereProcess.Mode.Red ?
                        AdvSphereProcess.Mode.Blue :
                        AdvSphereProcess.Mode.Red;
                spheres.setMode(mode);
            }
            if (gamepad1.y && !lastYBtn) {
                altBoxes = !altBoxes;
                spheres.setAltBoxes(altBoxes);
            }
            lastABtn = gamepad1.a;
            lastXBtn = gamepad1.x;
            lastYBtn = gamepad1.y;
        }
    }

    private void initVisionPortal() {
        spheres = new AdvSphereProcess(mode, altBoxes);

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(webcam);
        visionPortalBuilder.setCameraResolution(new Size(864, 480));
        visionPortalBuilder.addProcessor(spheres);
        portal = visionPortalBuilder.build();
    }
}
