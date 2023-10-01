package org.firstinspires.ftc.teamcode.odo

import kotlin.math.PI

abstract class RotationUnit(value: Double) :
    MeasureUnit<RotationConversionRules, RotationConverter, RotationUnit>(value) {
    override val to: RotationConverter get() = RotationConverter(rules, value)
    override fun convertOther(other: RotationUnit): RotationUnit {

    }
}

data class RotationConversionRules(
    val toDegrees: Double,
    val toRadians: Double
)

class RotationConverter(private val rules: RotationConversionRules, private val value: Double) {
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
}

val Double.radians: RadianUnit get() = RadianUnit(this)
val Int.radians: RadianUnit get() = RadianUnit(this.toDouble())
