@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.odo

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class DeltaPose(forward: LengthUnit, right: LengthUnit, turn: RotationUnit) {
    val forward = forward.to.inches
    val right = right.to.inches
    val turn = turn.to.radians


    override fun toString(): String = "DeltaPose[forward=$forward right=$right turn=$turn]"
}

class Pose(x: LengthUnit, y: LengthUnit, theta: RotationUnit) {
    val x = x.to.inches
    val y = y.to.inches
    val theta = theta.to.radians

    fun forward(distance: LengthUnit): Pose {
        return Pose(
            (x.value + distance.to.inches.value * cos(theta.value)).inches,
            (y.value + distance.to.inches.value * sin(theta.value)).inches,
            theta
        )
    }

    fun right(distance: LengthUnit): Pose {
        return Pose(
            (x.value + distance.to.inches.value * sin(theta.value)).inches,
            (y.value - distance.to.inches.value * cos(theta.value)).inches,
            theta
        )
    }

    fun turnCounterClockwise(angle: RotationUnit): Pose {
        return Pose(x, y, (theta.value + angle.to.radians.value).radians)
    }

    fun turnClockwise(angle: RotationUnit) = turnCounterClockwise(-angle)

    fun transform(forward: LengthUnit, right: LengthUnit, ccw: RotationUnit): Pose =
        // TODO: do some integration stuff - ask bill
        turnCounterClockwise(ccw / 2.0).forward(forward).right(right)
            .turnCounterClockwise(ccw / 2.0)

    fun to(target: Pose): DeltaPose {
        // TODO: these equations don't match the Python. why
        val theta = theta.value
        val alpha =
            (target.x * cos(theta)) + (target.y * sin(theta)) - (x * cos(theta)) - (y * sin(theta))
        val beta =
            (target.x * sin(theta)) - (target.y * cos(theta)) - (x * sin(theta)) + (y * cos(theta))
        return DeltaPose(
            alpha,
            beta,
            // rem: remainder - keeps the sign of the dividend (first value)
            // same as (%) operator but more explicit about behavior
            ((target.theta.value - this.theta.value).rem(2.0 * PI)).radians
        )
    }

    override fun equals(other: Any?): Boolean = when (other) {
        (other == null) -> false
        is Pose ->
            if (abs(x.value - other.x.value) > EPSILON_DISTANCE) false
            else if (abs(y.value - other.y.value) > EPSILON_DISTANCE) false
            else abs(theta.value - other.theta.value) <= EPSILON_ANGLE
        else -> false
    }

    fun copy() = Pose(x, y, theta)

    override fun toString(): String = "Pose[x=$x y=$y r=${theta.to.degrees}]"
    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + theta.hashCode()
        return result
    }

    companion object {
        val zero = Pose(0.inches, 0.inches, 0.radians)
        const val EPSILON_DISTANCE = 0.1 // in
        const val EPSILON_ANGLE = 0.035 // rad, ~2deg

        private val ROBOT_SIZE_X = 17.inches
        private val ROBOT_SIZE_Y = 17.inches
        private val HALF_ROBOT_X = ROBOT_SIZE_X / 2.0
        private val HALF_ROBOT_Y = ROBOT_SIZE_Y / 2.0
        private val FIELD_SIZE = 12.feet
        private val TILE = 2.feet

        // Robot is centered in the tile and flush against the wall.
        val startBlueDropSide = Pose(HALF_ROBOT_X, TILE * 3.5, 0.radians)
        val startBlueLoadSide = Pose(HALF_ROBOT_X, TILE * 1.5, 0.radians)

        val startRedDropSide = Pose(FIELD_SIZE.to.inches - HALF_ROBOT_X, TILE * 3.5, 180.degrees)
        val startRedLoadSide = Pose(FIELD_SIZE.to.inches - HALF_ROBOT_X, TILE * 1.5, 180.degrees)
    }
}
