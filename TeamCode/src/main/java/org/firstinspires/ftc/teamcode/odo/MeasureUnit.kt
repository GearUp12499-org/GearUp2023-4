package org.firstinspires.ftc.teamcode.odo

abstract class MeasureUnit<ConvertRules, Converter, DimType>(val value: Double) {
    operator fun div(other: Double) = newInstance(this.value / other)
    operator fun div(other: MeasureUnit<ConvertRules, Converter, DimType>): DimType {
        if (other::class == this::class) return newInstance(this.value / other.value)
        throw IllegalArgumentException("Cannot divide $this by $other - convert first")
    }

    operator fun times(other: Double) = newInstance(this.value * other)
    operator fun times(other: MeasureUnit<ConvertRules, Converter, DimType>): DimType {
        if (other::class == this::class) return newInstance(this.value * other.value)
        throw IllegalArgumentException("Cannot multiply $this by $other - convert first")
    }

    abstract val label: String
    protected abstract fun newInstance(n: Double): DimType

    protected abstract fun convertOther(other: MeasureUnit<ConvertRules, Converter, DimType>): MeasureUnit<ConvertRules, Converter, DimType>

    operator fun plus(other: MeasureUnit<ConvertRules, Converter, DimType>): DimType {
        if (other::class == this::class) return newInstance(this.value + other.value)
        throw IllegalArgumentException("Cannot add $other to $this - convert first")
    }

    operator fun minus(other: MeasureUnit<ConvertRules, Converter, DimType>): DimType {
        if (other::class == this::class) return newInstance(this.value - other.value)
        throw IllegalArgumentException("Cannot subtract $other from $this - convert first")
    }

    operator fun unaryMinus(): DimType {
        return newInstance(-this.value)
    }


    protected abstract val rules: ConvertRules
    abstract val to: Converter
}