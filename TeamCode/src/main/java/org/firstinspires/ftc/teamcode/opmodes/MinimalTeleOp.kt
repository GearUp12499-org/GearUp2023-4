package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import kotlin.math.abs
import kotlin.math.max

@TeleOp
class MinimalTeleOp : LinearOpMode() {
    override fun runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        val robot = RobotConfiguration.currentConfiguration()(hardwareMap)
        
        waitForStart()
        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble() // Remember, this is reversed!
            val x = gamepad1.left_stick_x * 1.1 // Counteract imperfect strafing
            val rx = gamepad1.right_stick_x.toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            val denominator = max(abs(y) + abs(x) + abs(rx), 1.0) * 1
            val frontLeftPower: Double = (y + x + rx) / denominator
            val backLeftPower: Double = (y - x + rx) / denominator
            val frontRightPower: Double = (y - x - rx) / denominator
            val backRightPower: Double = (y + x - rx) / denominator

            robot.driveMotors().frontLeft.power = frontLeftPower
            robot.driveMotors().backLeft.power = backLeftPower
            robot.driveMotors().frontRight.power = frontRightPower
            robot.driveMotors().backRight.power = backRightPower
        }
    }
}