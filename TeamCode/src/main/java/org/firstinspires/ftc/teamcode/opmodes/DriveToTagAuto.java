package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveToTagAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // weirdo delegation thing. we're rewriting this anyway
        DriveToTagBacking backingScript = new DriveToTagBacking(hardwareMap, telemetry, gamepad1);
        waitForStart();
        telemetry.addLine("press X to start");
        telemetry.addLine("");
        telemetry.update();
        while (!gamepad1.x && opModeIsActive()) {
            sleep(20);
        }
        backingScript.start(this::opModeIsActive);
        while (opModeIsActive()) {
            sleep(20);
        }
    }
}
