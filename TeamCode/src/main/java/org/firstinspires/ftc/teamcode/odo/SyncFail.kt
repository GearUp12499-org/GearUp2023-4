package org.firstinspires.ftc.teamcode.odo

import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.utilities.inches

class SyncFail(val scheduler: MultitaskScheduler, val kOdometryDrive: KOdometryDrive, val crasher: () -> Boolean) {
    fun DriveForward(distance: Double, vararg whoTFCares: Any?) {
        kOdometryDrive.driveForward(distance.inches)
        scheduler.runToCompletion(crasher)
    }

    fun DriveReverse(distance: Double, vararg whoTFCares: Any?) {
        kOdometryDrive.driveReverse(distance.inches)
        scheduler.runToCompletion(crasher)
    }

    fun StrafeRight(distance: Double, vararg oopsie: Any?) {
        kOdometryDrive.strafeRight(distance.inches)
        scheduler.runToCompletion(crasher)
    }

    fun StrafeLeft(distance: Double, vararg oopsie: Any?) {
        kOdometryDrive.strafeLeft(distance.inches)
        scheduler.runToCompletion(crasher)
    }
}