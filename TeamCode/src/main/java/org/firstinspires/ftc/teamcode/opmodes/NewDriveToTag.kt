package org.firstinspires.ftc.teamcode.opmodes

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.AprilTagApproach
import org.firstinspires.ftc.teamcode.utilities.typedGet
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@TeleOp
class NewDriveToTag : LinearOpMode() {
    private var portal: VisionPortal? = null
    private var aprilTag: AprilTagProcessor? = null

    override fun runOpMode() {
        val scheduler = MultitaskScheduler()
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        initVisionPortal(hardwareMap.typedGet("Webcam 1"))
        val tagMotion = AprilTagApproach(aprilTag!!, robot, 2, telemetry)

        scheduler.task(tagMotion.task)

        waitForStart()
        if (!opModeIsActive()) return
        scheduler.runToCompletion(::opModeIsActive)
        while (opModeIsActive()) {
            sleep(20)
        }
    }

    private fun initVisionPortal(webcam: WebcamName) {
        aprilTag = AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build()
        val visionPortalBuilder = VisionPortal.Builder()
        visionPortalBuilder.setCamera(webcam)
        // Medium-sized 16:9ish. See-also builtinwebcamcalibrations.xml
        // Logitech HD Pro Webcam C920
        // use Search Anything in "include non-project" mode to find
        visionPortalBuilder.setCameraResolution(Size(864, 480))
        visionPortalBuilder.addProcessor(aprilTag)
        portal = visionPortalBuilder.build()
    }
}