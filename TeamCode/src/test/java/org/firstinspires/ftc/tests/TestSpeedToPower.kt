package org.firstinspires.ftc.tests

import org.firstinspires.ftc.teamcode.utilities.Move
import kotlin.math.abs
import kotlin.test.Test

class TestSpeedToPower {
    @Test
    fun `test zero`() {
        assert(Move.rampSpeedToPower(0.0) near 0.0) {
            "${Move.rampSpeedToPower(0.0)} not = 0.0"
        }
    }

    @Test
    fun `test full`() {
        assert(Move.rampSpeedToPower(1.0) near 1.0) {
            "${Move.rampSpeedToPower(1.0)} not = 1.0"
        }
    }

    @Test
    fun `test small`() {
        assert(Move.rampSpeedToPower(.1) > .2) {
            "${Move.rampSpeedToPower(.1)} not greater than .2"
        }
    }
    @Test
    fun `test neg zero`() {
        assert(Move.rampSpeedToPower(-0.0) near -0.0) {
            "${Move.rampSpeedToPower(-0.0)} not = -0.0"
        }
    }

    @Test
    fun `test neg full`() {
        assert(Move.rampSpeedToPower(-1.0) near -1.0) {
            "${Move.rampSpeedToPower(-1.0)} not = -1.0"
        }
    }

    @Test
    fun `test neg small`() {
        assert(Move.rampSpeedToPower(-.1) < -.2) {
            "${Move.rampSpeedToPower(-.1)} not = -.2"
        }
    }

    @Test
    fun `test positive spread`() {
        for (i in 1 until 100) {
            assert(Move.rampSpeedToPower(i / 100.0) in (0.2).rangeTo(1.0)) {
                "${Move.rampSpeedToPower(i / 100.0)} (for k=${i/100.0}) is too small"
            }
        }
    }

    @Test
    fun `test negative spread`() {
        for (i in 1 until 100) {
            assert(Move.rampSpeedToPower(i / -100.0) in (-1.0).rangeTo(-.2)) {
                "${Move.rampSpeedToPower(i / -100.0)} (for k=${i / -100.0}) is too small"
            }
        }
    }
}

private const val EPSILON = 0.0001
private infix fun Double.near(other: Double): Boolean {
    return abs(this - other) < EPSILON
}