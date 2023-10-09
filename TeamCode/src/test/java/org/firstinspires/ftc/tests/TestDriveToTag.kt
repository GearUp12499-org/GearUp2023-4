package org.firstinspires.ftc.tests

import org.firstinspires.ftc.teamcode.DriveToTagBacking
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.MAX_SPEED
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.MIN_SPEED
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.STOPPING_ANGLE
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.THRESHOLD_ANGLE
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.THRESHOLD_DISTANCE
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.cvtAngleToPower
import org.firstinspires.ftc.teamcode.DriveToTagBacking.Companion.cvtRangeToPower
import kotlin.math.abs
import kotlin.test.Test

class TestDriveToTag {
    @Test
    fun `test power function at zero distance`() {
        assert(cvtRangeToPower(0.0) near 0.0)
    }

    @Test
    fun `test power function at threshold distance`() {
        val actual = cvtRangeToPower(THRESHOLD_DISTANCE)
        val expected = MIN_SPEED
        assert(
            actual near expected
        ) { "expected $expected, got $actual" }
    }

    @Test
    fun `test negative threshold distance`() {
        val actual = cvtRangeToPower(-THRESHOLD_DISTANCE)
        val expected = -MIN_SPEED
        assert(
            actual near expected
        ) { "expected $expected, got $actual" }
    }

    @Test
    fun `test power function at extreme distance`() {
        assert(
            cvtRangeToPower(DriveToTagBacking.STOPPING_DISTANCE + 1.0)
                    near MAX_SPEED
        )
    }

    @Test
    fun `test power function at negative extreme distance`() {
        assert(
            cvtRangeToPower(-DriveToTagBacking.STOPPING_DISTANCE - 1.0)
                    near -MAX_SPEED
        )
    }

    @Test
    fun `test angle function at zero`() {
        assert(
            cvtAngleToPower(0.0) near 0.0
        )
    }

    @Test
    fun `test angle function at threshold angle`() {
        assert(
            cvtAngleToPower(THRESHOLD_ANGLE) near MIN_SPEED
        )
    }

    @Test
    fun `test angle function at negative threshold angle`() {
        assert(
            cvtAngleToPower(-THRESHOLD_ANGLE) near -MIN_SPEED
        )
    }

    @Test
    fun `test angle function at extreme angle`() {
        assert(
            cvtAngleToPower(STOPPING_ANGLE + 1.0) near MAX_SPEED
        )
    }

    @Test
    fun `test angle function at negative extreme angle`() {
        assert(
            cvtAngleToPower(-STOPPING_ANGLE - 1.0) near -MAX_SPEED
        )
    }
}

private const val EPSILON = 0.0001
private infix fun Double.near(other: Double): Boolean {
    return abs(this - other) < EPSILON
}
