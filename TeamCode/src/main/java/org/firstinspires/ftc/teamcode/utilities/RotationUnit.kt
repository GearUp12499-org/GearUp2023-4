package org.firstinspires.ftc.teamcode.utilities

import kotlin.math.PI

abstract class RotationUnit(value: Double) :
    MeasureUnit<RotationUnit>(value) {
    protected abstract val rules: RotationConversionRules
    override val to: RotationConverter get() = RotationConverter(rules, value)

    /**
     * Normalize angle to range (-pi, pi] (rads) or equivalent units
     */
    abstract fun norm(): RotationUnit
}

data class RotationConversionRules(
    val toDegrees: Double,
    val toRadians: Double,
)

class RotationConverter(private val rules: RotationConversionRules, private val value: Double) :
    Converter<RotationUnit>() {
    val degrees: DegreeUnit get() = DegreeUnit(rules.toDegrees * value)
    val radians: RadianUnit get() = RadianUnit(rules.toRadians * value)
}

class DegreeUnit(value: Double) : RotationUnit(value) {
    override val label = "deg"
    override fun newInstance(n: Double) = DegreeUnit(n)

    override val rules = RotationConversionRules(
        1.0,
        PI / 180.0
    )

    override fun convertOther(other: MeasureUnit<*>): DegreeUnit {
        return when (other) {
            is RotationUnit -> other.to.degrees
            else -> throw IllegalArgumentException("Cannot convert $other to $this")
        }
    }

    override fun norm(): DegreeUnit {
        // always positive (mod)
        val a = value.mod(360.0)
        return when {
            a > 180.0 -> DegreeUnit(a - 360.0)
            else -> DegreeUnit(a)
        }
    }
}

val Double.degrees: DegreeUnit get() = DegreeUnit(this)
val Int.degrees: DegreeUnit get() = DegreeUnit(this.toDouble())

class RadianUnit(value: Double) : RotationUnit(value) {
    companion object {
        const val TAU = 2.0 * PI
    }

    override val label = "rad"
    override fun newInstance(n: Double) = RadianUnit(n)

    override val rules = RotationConversionRules(
        180.0 / PI,
        1.0
    )

    override fun convertOther(other: MeasureUnit<*>): RadianUnit {
        return when (other) {
            is RotationUnit -> other.to.radians
            else -> throw IllegalArgumentException("Cannot convert $other to $this")
        }
    }

    override fun norm(): RadianUnit {
        // always positive (mod)
        val a = value.mod(TAU)
        return when {
            a > PI -> RadianUnit(a - TAU)
            else -> RadianUnit(a)
        }
    }
}

val Double.radians: RadianUnit get() = RadianUnit(this)
val Int.radians: RadianUnit get() = RadianUnit(this.toDouble())
