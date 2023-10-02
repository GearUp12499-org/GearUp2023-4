package org.firstinspires.ftc.tests

import org.firstinspires.ftc.teamcode.odo.MeasureUnit
import org.firstinspires.ftc.teamcode.odo.Pose
import org.firstinspires.ftc.teamcode.odo.degrees
import org.firstinspires.ftc.teamcode.odo.feet
import org.firstinspires.ftc.teamcode.odo.inches
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.fail

class TestPose {
    private fun nearEqual(a: Double, b: Double, epsilon: Double = 0.001) {
        if (abs(a - b) > epsilon) {
            fail("expected $a, got $b")
        }
    }
    private fun nearEqual(a: MeasureUnit<*>, b: MeasureUnit<*>, epsilon: Double = 0.001) {
        if (abs((a - b).value) > epsilon) {
            fail("expected $a, got $b (${a.convertOther(b)})")
        }
    }

    @Test
    fun `move forward origin`() {
        val start = Pose.zero
        val finish = start.forward(10.inches)
        nearEqual(10.inches, finish.x)
        nearEqual(0.inches, finish.y)
        nearEqual(0.degrees, finish.theta)
    }

    @Test
    fun `turn move +y`() {
        val start = Pose.zero
        val finish = start.turnCounterClockwise(90.degrees).forward(1.feet)
        nearEqual(0.inches, finish.x)
        nearEqual(1.feet, finish.y)
        nearEqual(90.degrees, finish.theta)
    }

    @Test
    fun `turn and move`() {
        val start = Pose.zero
        val finish = start.transform(1.feet, 0.inches, 90.degrees)
        nearEqual((sqrt(2.0)/2).feet, finish.x)
        nearEqual(finish.x, finish.y)
    }
}