package org.firstinspires.ftc.teamcode.odo

import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.utilities.inches

class SyncFail(val scheduler: MultitaskScheduler, val kOdometryDrive: KOdometryDrive) {
    fun DriveForward(distance: Double, vararg whoTFCares: Any?) {
        kOdometryDrive.driveForward(distance.inches)
        scheduler.runToCompletion {true}
    }
    fun DriveReverse(distance: Double, vararg whoTFCares: Any?) {
        kOdometryDrive.driveReverse(distance.inches)
        scheduler.runToCompletion {true}
    }
}