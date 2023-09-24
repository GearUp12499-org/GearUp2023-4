package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveToTagAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveToTagBacking backingScript = new DriveToTagBacking(hardwareMap, telemetry);
        waitForStart();
        telemetry.addLine("press X to start");
        telemetry.addLine("");
        telemetry.update();
        backingScript.start(this::opModeIsActive);
        telemetry.addLine("Completed");
        telemetry.update();
        while (opModeIsActive()) {
            sleep(20);
        }
    }
}
