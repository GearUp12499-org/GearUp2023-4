package org.firstinspires.ftc.teamcode.odo

import kotlin.math.PI

abstract class RotationUnit(value: Double) :
    MeasureUnit<RotationUnit>(value) {
    protected abstract val rules: RotationConversionRules
    override val to: RotationConverter get() = RotationConverter(rules, value)
}

data class RotationConversionRules(
    val toDegrees: Double,
    val toRadians: Double
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
}

val Double.degrees: DegreeUnit get() = DegreeUnit(this)
val Int.degrees: DegreeUnit get() = DegreeUnit(this.toDouble())

class RadianUnit(value: Double) : RotationUnit(value) {
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
}

val Double.radians: RadianUnit get() = RadianUnit(this)
val Int.radians: RadianUnit get() = RadianUnit(this.toDouble())
