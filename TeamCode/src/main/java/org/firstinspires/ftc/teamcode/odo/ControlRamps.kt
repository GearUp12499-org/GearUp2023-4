package org.firstinspires.ftc.teamcode.odo

import kotlin.math.min
import kotlin.math.pow

interface RampProvider {
    fun ramp(distFromStart: Double, distToEnd: Double): Double
}

interface LeftRampProvider {
    fun rampUp(distFromStart: Double): Double
}

interface RightRampProvider {
    fun rampDown(distToEnd: Double): Double
}

class LinearUpRamp(
    private val startMinimum: Double,
    private val maximum: Double,
    private val upRampSize: Double
) : LeftRampProvider {
    override fun rampUp(distFromStart: Double): Double {
        // WARNING: ORDER MATTERS! (especially if passing a static 0.0 to a size-0 ramp.)
        // The first matching condition will be returned!
        return when {
            distFromStart >= upRampSize -> maximum
            distFromStart <= 0 -> startMinimum
            else ->
                (maximum - startMinimum) * (distFromStart / upRampSize) + startMinimum
        }
    }
}

class LinearDownRamp(
    private val finalMinimum: Double,
    private val maximum: Double,
    private val downRampSize: Double
) : RightRampProvider {
    override fun rampDown(distToEnd: Double): Double {
        return when {
            distToEnd <= 0 -> 0.0
            distToEnd >= downRampSize -> maximum
            else ->
                (maximum - finalMinimum) * (distToEnd / downRampSize) + finalMinimum
        }
    }
}

class QuadraticDownRamp(
    private val finalMinimum: Double,
    private val maximum: Double,
    private val downRampSize: Double
) : RightRampProvider {
    private val c = (maximum - finalMinimum) / downRampSize.pow(2)
    override fun rampDown(distToEnd: Double): Double {
        return when {
            distToEnd <= 0 -> 0.0
            distToEnd >= downRampSize -> maximum
            else ->
                c * distToEnd.pow(2) + finalMinimum
        }
    }

}


/**
 * Control ramps. To disable a ramp, set its size to 0.0 (or negative.)
 */
abstract class AbstractControlRamps(
    startMinimum: Double,
    finalMinimum: Double,
    maximum: Double,
    upRampSize: Double,
    downRampSize: Double,
) : RampProvider {

    protected abstract val left: LeftRampProvider
    protected abstract val right: RightRampProvider

    override fun ramp(distFromStart: Double, distToEnd: Double) =
        min(left.rampUp(distFromStart), right.rampDown(distToEnd))
}

class ControlRamps(
    startMinimum: Double,
    finalMinimum: Double,
    maximum: Double,
    upRampSize: Double,
    downRampSize: Double
) : AbstractControlRamps(startMinimum, finalMinimum, maximum, upRampSize, downRampSize) {
    constructor(minimum: Double, maximum: Double, upRampSize: Double, downRampSize: Double) : this(
        minimum,
        minimum,
        maximum,
        upRampSize,
        downRampSize
    )

    override val left by lazy { LinearUpRamp(startMinimum, maximum, upRampSize) }
    override val right by lazy { LinearDownRamp(finalMinimum, maximum, downRampSize) }
}

class QuadraticDownRamps(
    startMinimum: Double,
    finalMinimum: Double,
    maximum: Double,
    upRampSize: Double,
    downRampSize: Double
) : AbstractControlRamps(startMinimum, finalMinimum, maximum, upRampSize, downRampSize) {
    constructor(minimum: Double, maximum: Double, upRampSize: Double, downRampSize: Double) : this(
        minimum,
        minimum,
        maximum,
        upRampSize,
        downRampSize
    )

    override val left by lazy { LinearUpRamp(startMinimum, maximum, upRampSize) }
    override val right by lazy { QuadraticDownRamp(finalMinimum, maximum, downRampSize) }
}