package org.firstinspires.ftc.tests

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.tests.debugging.logStack
import org.firstinspires.ftc.tests.mock.FakeServo
import org.firstinspires.ftc.tests.mock.MockRobot
import org.firstinspires.ftc.tests.mock.simulate
import kotlin.test.Test

class TestVirtualRobot {
    @Test
    fun `test Robot driving`() {
        val robot = MockRobot()
        robot.clearEncoders()
        robot.driveMotors().setAll(1.0)
        robot.advance(1.0)
        robot.driveMotors().setAll(0.0)
    }

    @Test
    fun `test Robot servo simulation`() {
        val robot = MockRobot()
        val servo = robot.purpleDropper
        var passed = false
        servo.teleport(1.0)
        servo.reachedTarget += {
            MinimalLogger.i("TestVirtualRobot", "Caller test")
            MinimalLogger.i("TestVirtualRobot", logStack())
            passed = true
        }

        servo.position = 0.0

        robot.simulate(10.0) {
            !passed
        }
        assert(passed) { "servo did not reach target position, got to ${servo.actualPos}" }
    }

    @Test
    fun `test Servo mocking`() {
        val servo = FakeServo("test")
        servo.position = 0.0
        servo.teleport(1.0)
        assert(servo.position == 1.0) { "servo did not teleport to 1.0" }
        servo.position = 0.0
        assert(servo.position == 0.0) { "servo did not reset to 0.0" }
        servo.direction = Servo.Direction.REVERSE
        servo.position = 0.0
        assert(servo.position == 0.0) { "servo interface doesn't handle reversing (API)" }
        servo.teleport(0.0)
        assert(servo.actualPos == 1.0) { "servo interface doesn't handle reversing (hardware)" }
        servo.position = 1.0
        assert(servo.position == 1.0) { "servo interface doesn't handle reversing (API)" }
        servo.teleport(1.0)
        assert(servo.actualPos == 0.0) { "servo interface doesn't handle reversing (hardware)" }
        servo.direction = Servo.Direction.FORWARD
    }
}