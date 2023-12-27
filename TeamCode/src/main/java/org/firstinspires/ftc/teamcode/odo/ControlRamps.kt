package org.firstinspires.ftc.teamcode.odo

import kotlin.math.min

/**
 * Control ramps. To disable a ramp, set its size to 0.0 (or negative.)
 */
class ControlRamps(
    private val startMinimum: Double,
    private val finalMinimum: Double,
    private val maximum: Double,
    private val upRampSize: Double,
    private val downRampSize: Double,
) {
    constructor(minimum: Double, maximum: Double, upRampSize: Double, downRampSize: Double) : this(
        minimum,
        minimum,
        maximum,
        upRampSize,
        downRampSize
    )

    private fun rampDown(distToEnd: Double): Double {
        return when {
            distToEnd <= 0 -> 0.0
            distToEnd >= downRampSize -> maximum
            else ->
                (maximum - finalMinimum) * (distToEnd / downRampSize) + finalMinimum
        }
    }

    private fun rampUp(distFromStart: Double): Double {
        // WARNING: ORDER MATTERS! (especially if passing a static 0.0 to a size-0 ramp.)
        // The first matching condition will be returned!
        return when {
            distFromStart >= upRampSize -> maximum
            distFromStart <= 0 -> startMinimum
            else ->
                (maximum - startMinimum) * (distFromStart / upRampSize) + startMinimum
        }
    }

    fun ramp(distFromStart: Double, distToEnd: Double) = min(rampUp(distFromStart), rampDown(distToEnd))
}