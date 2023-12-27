@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.utilities

import org.firstinspires.ftc.teamcode.Var
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

const val TAU = 2 * PI

class Move(forward: LengthUnit, right: LengthUnit, turn: RotationUnit) {
    val forward = forward.to.inches
    val right = right.to.inches
    val turn = turn.to.radians

    companion object {
        private const val RAMP_ALPHA = 0.25
        private const val RAMP_EPSILON = 0.01
        fun ramp(speed: Double): Double {
            val sign = speed.sign
            return when {
                abs(speed) < RAMP_EPSILON -> 0.0
                else -> RAMP_ALPHA + (1 - RAMP_ALPHA) * abs(speed)
            } * sign
        }
    }

    fun getSpeeds(robotSize: Double): MotorPowers {
        val rotationInches = robotSize * turn.value * Var.Pose.turnBias
        val forward = forward.value * sqrt(2.0) * Var.Pose.straightBias
        val right = right.value * sqrt(2.0) * Var.Pose.strafeBias

        val proportions = MotorPowers(
            frontLeft = forward + right - rotationInches,
            frontRight = forward - right + rotationInches,
            backLeft = forward - right - rotationInches,
            backRight = forward + right + rotationInches
        )
        return proportions.normalize()
    }

    override fun toString(): String =
        "DeltaPose[forward=$forward right=$right turn=${turn.to.degrees}]"
}

class Pose(x: LengthUnit, y: LengthUnit, theta: RotationUnit) {
    constructor(x: Vector2, y: RotationUnit) : this(x.x.inches, x.y.inches, y)

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
        turnCounterClockwise(ccw / 2.0).forward(forward).right(right)
            .turnCounterClockwise(ccw / 2.0)

    fun transform(move: Move): Pose = transform(move.forward, move.right, move.turn)

    fun toPose(target: Pose): Move {
        // TODO: these equations don't match the Python. why
        val theta = theta.value
        val dx = target.x - x
        val dy = target.y - y
        val alpha =
            (dx * cos(theta)) + (dy * sin(theta))
        val beta =
            (dx * sin(theta)) - (dy * cos(theta))
        return Move(
            alpha,
            beta,
            ((target.theta - this.theta).norm()).to.radians
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

    operator fun plus(other: Pose) =
        Pose(this.x + other.x, this.y + other.y, this.theta + other.theta)
    operator fun plus(other: Move) = transform(other)

    operator fun div(other: Int) = Pose(this.x / other, this.y / other, this.theta / other)
    operator fun div(other: Double) = Pose(this.x / other, this.y / other, this.theta / other)
    operator fun minus(other: Pose) =
        Pose(this.x - other.x, this.y - other.y, this.theta - other.theta)

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
