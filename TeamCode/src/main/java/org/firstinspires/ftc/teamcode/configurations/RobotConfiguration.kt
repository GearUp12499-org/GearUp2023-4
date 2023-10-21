package org.firstinspires.ftc.teamcode.configurations

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.SharedResource
import org.firstinspires.ftc.teamcode.utility.MotorSet

abstract class RobotConfiguration {
    companion object {
        @JvmStatic
        fun currentConfiguration(): (HardwareMap) -> RobotConfiguration = ::NeoRobot1
    }

    protected abstract val driveMotors: MotorSet?
    fun driveMotors(): MotorSet = driveMotors ?: throw NullPointerException("Robot configuration has no drive motors but they were requested")

    abstract val driveMotorLock: SharedResource

    protected abstract val clawGrab: Servo?
    fun clawGrab(): Servo = clawGrab ?: throw NullPointerException("Robot configuration has no claw grab servo but it was requested")
    protected abstract val clawRotate: Servo?
    fun clawRotate(): Servo = clawRotate ?: throw NullPointerException("Robot configuration has no claw rotate servo but it was requested")
    abstract val clawLock: SharedResource

    protected abstract val liftLeft: DcMotor?
    fun liftLeft(): DcMotor = liftLeft ?: throw NullPointerException("Robot configuration has no left lift motor but it was requested")
    protected abstract val liftRight: DcMotor?
    fun liftRight(): DcMotor = liftRight ?: throw NullPointerException("Robot configuration has no right lift motor but it was requested")
}