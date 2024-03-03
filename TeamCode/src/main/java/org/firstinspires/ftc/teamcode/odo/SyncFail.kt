package org.firstinspires.ftc.teamcode.odo

import dev.aether.collaborative_multitasking.MultitaskScheduler
import org.firstinspires.ftc.teamcode.utilities.inches

class SyncFail(val scheduler: MultitaskScheduler, val kOdometryDrive: KOdometryDrive, val crasher: () -> Boolean) {
    fun driveForward(distance: Double, vararg whoTFCares: Any?) {
        kOdometryDrive.driveForward(distance.inches)
        scheduler.runToCompletion(crasher)
    }
    fun driveForwardWithCollision(distance: Double) {
        kOdometryDrive.driveForward(distance.inches, true)
        scheduler.runToCompletion(crasher)
    }

    fun driveReverse(distance: Double, vararg whoTFCares: Any?) {
        kOdometryDrive.driveReverse(distance.inches)
        scheduler.runToCompletion(crasher)
    }
    fun driveReverseWithCollision(distance: Double) {
        kOdometryDrive.driveReverse(distance.inches, true)
        scheduler.runToCompletion(crasher)
    }

    fun strafeRight(distance: Double, vararg oopsie: Any?) {
        kOdometryDrive.strafeRight(distance.inches)
        scheduler.runToCompletion(crasher)
    }

    fun strafeLeft(distance: Double, vararg oopsie: Any?) {
        kOdometryDrive.strafeLeft(distance.inches)
        scheduler.runToCompletion(crasher)
    }
}