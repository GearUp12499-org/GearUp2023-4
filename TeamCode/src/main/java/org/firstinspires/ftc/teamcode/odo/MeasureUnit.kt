package org.firstinspires.ftc.teamcode.odo

abstract class Converter<out R>

abstract class MeasureUnit<This: MeasureUnit<This>>(val value: Double) {
    operator fun div(other: Double) = newInstance(this.value / other)
    operator fun div(other: Int) = div(other.toDouble())
    operator fun div(other: MeasureUnit<*>): Double {
        return this.value / convertOther(other).value
    }

    operator fun times(other: Double) = newInstance(this.value * other)
    operator fun times(other: MeasureUnit<*>): This {
        return newInstance(this.value * convertOther(other).value)
    }

    abstract val label: String
    override fun toString(): String = String.format("[%.4f %s]", value, label)

    protected abstract fun newInstance(n: Double): This

    operator fun plus(other: MeasureUnit<*>): This {
        return newInstance(this.value + convertOther(other).value)
    }
    abstract fun convertOther(other: MeasureUnit<*>): This

    operator fun minus(other: MeasureUnit<*>): This {
        return newInstance(this.value - convertOther(other).value)
    }

    operator fun unaryMinus(): This {
        return newInstance(-this.value)
    }

    operator fun times(ticks: Int) = times(ticks.toDouble())

    operator fun compareTo(other: MeasureUnit<*>): Int =
        this.value.compareTo(convertOther(other).value)

    abstract val to: Converter<MeasureUnit<This>>
}