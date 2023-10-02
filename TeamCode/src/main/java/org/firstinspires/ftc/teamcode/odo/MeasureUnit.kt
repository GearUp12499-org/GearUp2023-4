package org.firstinspires.ftc.teamcode.odo

abstract class Converter<out R>

abstract class MeasureUnit<This: MeasureUnit<This>>(val value: Double) {
    operator fun div(other: Double) = newInstance(this.value / other)
    operator fun div(other: MeasureUnit<*>): This {
        return newInstance(convertOther(other).value)
    }

    operator fun times(other: Double) = newInstance(this.value * other)
    operator fun times(other: MeasureUnit<*>): This {
        if (other::class == this::class) return newInstance(this.value * other.value)
        throw IllegalArgumentException("Cannot multiply $this by $other - convert first")
    }

    abstract val label: String
    protected abstract fun newInstance(n: Double): This

    operator fun plus(other: MeasureUnit<*>): This {
        if (other::class == this::class) return newInstance(this.value + other.value)
        throw IllegalArgumentException("Cannot add $other to $this - convert first")
    }
    abstract fun convertOther(other: MeasureUnit<*>): This

    operator fun minus(other: MeasureUnit<*>): This {
        if (other::class == this::class) return newInstance(this.value - other.value)
        throw IllegalArgumentException("Cannot subtract $other from $this - convert first")
    }

    operator fun unaryMinus(): This {
        return newInstance(-this.value)
    }

    abstract val to: Converter<MeasureUnit<This>>
}