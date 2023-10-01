package org.firstinspires.ftc.teamcode.odo

abstract class LengthUnit(value: Double) :
    MeasureUnit<LengthConversionRules, LengthConverter, LengthUnit>(value) {
    override val to: LengthConverter get() = LengthConverter(rules, value)
}

data class LengthConversionRules(
    val toInches: Double,
    val toFeet: Double
)

class LengthConverter(private val rules: LengthConversionRules, private val value: Double) {
    val inches: InchUnit get() = InchUnit(rules.toInches * value)
    val feet: FootUnit get() = FootUnit(rules.toFeet * value)
}

class InchUnit(value: Double) : LengthUnit(value) {
    override val label = "in"
    override fun newInstance(n: Double) = InchUnit(n)

    override val rules = LengthConversionRules(
        1.0,
        1.0 / 12.0
    )
}

val Double.inches: InchUnit get() = InchUnit(this)
val Int.inches: InchUnit get() = InchUnit(this.toDouble())

class FootUnit(value: Double) : LengthUnit(value) {
    override val label = "ft"
    override fun newInstance(n: Double) = FootUnit(n)

    override val rules = LengthConversionRules(
        12.0,
        1.0
    )
}

val Double.feet: FootUnit get() = FootUnit(this)
val Int.feet: FootUnit get() = FootUnit(this.toDouble())
