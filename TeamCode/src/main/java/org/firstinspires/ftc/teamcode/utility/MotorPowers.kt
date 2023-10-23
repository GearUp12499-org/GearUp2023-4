package org.firstinspires.ftc.teamcode.utility

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

    fun apply(motors: MotorSet) {
        apply(
            frontLeftMotor = motors.frontLeft,
            frontRightMotor = motors.frontRight,
            backLeftMotor = motors.backLeft,
            backRightMotor = motors.backRight
        )
    }

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
}

data class MotorSet(
    @JvmField val frontLeft: DcMotor,
    @JvmField val frontRight: DcMotor,
    @JvmField val backLeft: DcMotor,
    @JvmField val backRight: DcMotor
) {
    fun setAll(p: Double) = MotorPowers(p, p, p, p).apply(this)
}