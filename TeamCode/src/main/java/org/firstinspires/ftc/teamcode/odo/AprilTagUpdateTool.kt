package org.firstinspires.ftc.teamcode.odo

import android.util.Log
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.teamcode.detectSingleToPose
import org.firstinspires.ftc.teamcode.utilities.Move
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.inches
import org.firstinspires.ftc.teamcode.utilities.radians
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

// Robot is 41cm wide = 16.142 in
class AprilTagUpdateTool(
    private val source: AprilTagProcessor,
    var initialTarget: Int,
) {
    companion object {
        val CameraOffset = Move(6.75.inches, 0.inches, 0.radians)
    }
    var acquired: Boolean = false
        private set
    private var lastReadData: AprilTagPoseFtc? = null

    fun updateTool(aux: OdoTracker): Task.() -> Unit {
        return {
            daemon = true
            Log.i("AprilTagUpdateTool", "using target $initialTarget")
            var lockedTarget = initialTarget
            var visible: Boolean
            onStart { ->
                lockedTarget = initialTarget
                acquired = false
            }
            onTick { ->
                val detections = source.detections
                var est: Pose? = null
                var theDetection: AprilTagDetection? = null
                for (detection in detections) {
                    if (detection.id == lockedTarget) {
                        est = detectSingleToPose(detection).transform(CameraOffset)
                        theDetection = detection
                    }
                }
                visible = est != null && theDetection != null
                if (!visible) return@onTick
                if (!(theDetection!!.ftcPose eq lastReadData)) {
                    acquired = true
                    lastReadData = theDetection.ftcPose
                    aux.update(est!!)
                }
            }
        }
    }

    private infix fun AprilTagPoseFtc.eq(other: AprilTagPoseFtc?): Boolean {
        if (other == null) return false
        return this.x == other.x
                && this.y == other.y
                && this.z == other.z
                && this.yaw == other.yaw
                && this.pitch == other.pitch
                && this.roll == other.roll
                && this.range == other.range
                && this.bearing == other.bearing
                && this.elevation == other.elevation
    }

    fun unacq() {
        acquired = false
        lastReadData = null
    }
}