package org.firstinspires.ftc.tests

import org.firstinspires.ftc.teamcode.odo.DriveForwardPID
import org.firstinspires.ftc.teamcode.odo.KOdometryDrive
import org.firstinspires.ftc.teamcode.odo.TurnPID
import kotlin.math.min
import kotlin.test.Test
import kotlin.test.fail

class TestDriveImplParity {
    companion object {
        private val targets = listOf(0.0, 1.0, 10.0, 100.0)
        private val positions = listOf(0.0, 0.5, 1.0, 2.0, 5.0, 9.0, 10.0, 11.0, 20.0, 50.0, 99.0, 100.0, 101.0, 125.0)
    }

    @Test
    fun `test drive forward ramps, kod ramps`() {
        var run = 0
        val throws = mutableListOf<String>()
        for (target in targets) {
            for (position in positions) {
                val kod = KOdometryDrive.ForwardingCurve.ramp(position, target - position)
                val java = min(
                    DriveForwardPID.rampUpForward(position),
                    DriveForwardPID.rampDownForward(target - position)
                )
                if (!decimalEquals(kod, java)) {
                    throws.add("parity error: (kt) $kod != (java) $java. test: target $target position $position")
                }
                run++
            }
        }
        println("(Forward ramps: ${run - throws.size} permutations passed, ${throws.size} failed)")
        for (throwable in throws) {
            System.err.println(throwable)
        }
        if (throws.size > 0) {
            fail("${throws.size} parity errors found")
        }
    }

    @Test
    fun `test strafe ramps, kod strafe ramps`() {
        var run = 0
        val throws = mutableListOf<String>()
        for (target in targets) {
            for (position in positions) {
                val kod = KOdometryDrive.StrafingCurve.ramp(position, target - position)
                val java = min(
                    DriveForwardPID.rampUp(position),
                    DriveForwardPID.rampDown(target - position)
                )
                if (!decimalEquals(kod, java)) {
                    throws.add("parity error: (kt) $kod != (java) $java. test: target $target position $position")
                }
                run++
            }
        }
        println("(Strafe ramps: ${run - throws.size} permutations passed, ${throws.size} failed)")
        for (throwable in throws) {
            System.err.println(throwable)
        }
        if (throws.size > 0) {
            fail("${throws.size} parity errors found")
        }
    }

    @Test
    fun `test turning ramps, kod turning ramps`() {
        var run = 0
        val throws = mutableListOf<String>()
        for (target in targets) {
            for (position in positions) {
                val kod = KOdometryDrive.TurningCurve.ramp(position, target - position)
                val java = TurnPID.rampDown(target - position)
                if (!decimalEquals(kod, java)) {
                    throws.add("parity error: (kt) $kod != (java) $java. test: target $target position $position")
                }
                run++
            }
        }
        println("(Turning ramps: ${run - throws.size} permutations passed, ${throws.size} failed)")
        for (throwable in throws) {
            System.err.println(throwable)
        }
        if (throws.size > 0) {
            fail("${throws.size} parity errors found")
        }
    }
}