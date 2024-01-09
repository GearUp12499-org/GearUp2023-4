package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.abs

data class MotorPowers(
    @JvmField val frontLeft: Double,
    @JvmField val frontRight: Double,
    @JvmField val backLeft: Double,
    @JvmField val backRight: Double
) {
    fun apply(
        frontLeftMotor: DcMotor,
        frontRightMotor: DcMotor,
        backLeftMotor: DcMotor,
        backRightMotor: DcMotor
    ) {
        frontLeftMotor.power = frontLeft
        frontRightMotor.power = frontRight
        backLeftMotor.power = backLeft
        backRightMotor.power = backRight
    }

    fun apply(motors: MotorSet<DcMotor>) {
        apply(
            frontLeftMotor = motors.frontLeft,
            frontRightMotor = motors.frontRight,
            backLeftMotor = motors.backLeft,
            backRightMotor = motors.backRight
        )
    }

    /**
     * Normalize powers. Warning: may PULL SMALL NUMBERS UP to 1.
     */
    @JvmOverloads
    fun normalize(factor: Double = 1.0): MotorPowers {
        val den = maxOf(
            abs(frontLeft),
            abs(frontRight),
            abs(backLeft),
            abs(backRight)
        )
        return (this / den) * factor
    }

    /**
     * Normalize powers if they are too large.
     */
    @JvmOverloads
    fun normalNoStretch(max: Double = 1.0): MotorPowers {
        val den = maxOf(
            abs(frontLeft),
            abs(frontRight),
            abs(backLeft),
            abs(backRight),
            max
        )
        return (this / den) * max
    }

    operator fun div(den: Double): MotorPowers {
        return MotorPowers(
            frontLeft / den,
            frontRight / den,
            backLeft / den,
            backRight / den
        )
    }

    operator fun times(factor: Double): MotorPowers {
        return MotorPowers(
            frontLeft * factor,
            frontRight * factor,
            backLeft * factor,
            backRight * factor
        )
    }

    fun map(funct: (Double) -> Double): MotorPowers {
        return MotorPowers(
            funct(frontLeft),
            funct(frontRight),
            funct(backLeft),
            funct(backRight)
        )
    }
}

data class MotorSet<out T: DcMotor>(
    @JvmField val frontLeft: T,
    @JvmField val frontRight: T,
    @JvmField val backLeft: T,
    @JvmField val backRight: T
) {
    fun setAll(p: Double) = MotorPowers(p, p, p, p).apply(this)
    fun each(function: T.() -> Unit) {
        frontLeft.function()
        frontRight.function()
        backLeft.function()
        backRight.function()
    }
}