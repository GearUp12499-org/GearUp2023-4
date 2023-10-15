package org.firstinspires.ftc.teamcode.utility

import com.qualcomm.robotcore.hardware.DcMotor

data class MotorPowers(
    val frontLeft: Double,
    val frontRight: Double,
    val backLeft: Double,
    val backRight: Double
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
}

data class MotorSet(
    val frontLeft: DcMotor,
    val frontRight: DcMotor,
    val backLeft: DcMotor,
    val backRight: DcMotor
)