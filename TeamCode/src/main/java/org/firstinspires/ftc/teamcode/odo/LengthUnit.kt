package org.firstinspires.ftc.teamcode.odo

abstract class LengthUnit(value: Double) :
    MeasureUnit<LengthUnit>(value) {
    protected abstract val rules: LengthConversionRules
    override val to: LengthConverter get() = LengthConverter(rules, value)
}

data class LengthConversionRules(
    val toInches: Double,
    val toFeet: Double
)

class LengthConverter(private val rules: LengthConversionRules, private val value: Double) :
    Converter<LengthUnit>() {
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

    override fun convertOther(other: MeasureUnit<*>): LengthUnit {
        return when (other) {
            is LengthUnit -> other.to.inches
            else -> throw IllegalArgumentException("Cannot convert $other to $this")
        }
    }
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

    override fun convertOther(other: MeasureUnit<*>): LengthUnit {
        return when (other) {
            is LengthUnit -> other.to.feet
            else -> throw IllegalArgumentException("Cannot convert $other to $this")
        }
    }
}

val Double.feet: FootUnit get() = FootUnit(this)
val Int.feet: FootUnit get() = FootUnit(this.toDouble())
