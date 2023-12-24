package org.firstinspires.ftc.teamcode.configurations

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.SharedResource
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utilities.MotorSet

abstract class RobotConfiguration {
    companion object {
        @JvmStatic
        fun currentConfiguration(): (HardwareMap) -> RobotConfiguration = ::Robot
    }

    protected abstract val driveMotors: MotorSet?
    fun driveMotors(): MotorSet = driveMotors
        ?: throw NullPointerException("Robot configuration has no drive motors but they were requested")

    abstract val driveMotorLock: SharedResource

    abstract val purpleDropper: Servo?
    fun purpleDropper(): Servo = purpleDropper
        ?: throw NullPointerException("Robot configuration has no purple dropper but it was requested")

    protected abstract val liftLeft: DcMotor?
    fun liftLeft(): DcMotor = liftLeft
        ?: throw NullPointerException("Robot configuration has no left lift motor but it was requested")

    protected abstract val liftRight: DcMotor?
    fun liftRight(): DcMotor = liftRight
        ?: throw NullPointerException("Robot configuration has no right lift motor but it was requested")

    abstract val liftLock: SharedResource

    protected abstract val dumperRotate: Servo?
    fun dumperRotate(): Servo = dumperRotate
        ?: throw NullPointerException("Robot configuration has no dumper rotate servo but it was requested")

    protected abstract val dumperLatch: Servo?
    fun dumperLatch(): Servo = dumperLatch
        ?: throw NullPointerException("Robot configuration has no dumper lock servo but it was requested")

    abstract val dumperLock: SharedResource

    protected abstract val drone: DcMotor?
    fun drone(): DcMotor = drone
        ?: throw NullPointerException("Robot configuration has no drone motor but it was requested")

    abstract val droneLock: SharedResource

    protected abstract val intake: DcMotor?
    fun intake(): DcMotor =
        intake ?: throw NullPointerException("Robot configuration has no intake motor.")


    protected abstract val distanceLeft: DistanceSensor?
    fun distanceLeft(): DistanceSensor = distanceLeft
        ?: throw NullPointerException("Robot configuration has no left distance sensor but it was requested")

    protected abstract val distanceRight: DistanceSensor?
    fun distanceRight(): DistanceSensor = distanceRight
        ?: throw NullPointerException("Robot configuration has no right distance sensor but it was requested")

    protected abstract val imu: IMU?
    fun imu(): IMU = imu
        ?: throw NullPointerException("Robot configuration has no IMU (for some reason) but it was requested")

    protected abstract val odoPerpendicular: DcMotor?
    fun odoPerpendicular(): DcMotor = odoPerpendicular
        ?: throw NullPointerException("Robot configuration has no Perpendicular Odometry Wheel.")


    protected abstract val odoParallel1: DcMotor?
    fun odoParallel1(): DcMotor = odoParallel1
        ?: throw NullPointerException("Robot configuration has no Perpendicular Odometry Wheel.")

    protected abstract val odoParallel2: DcMotor?
    fun odoParallel2(): DcMotor = odoParallel2
        ?: throw NullPointerException("Robot configuration has no Perpendicular Odometry Wheel.")

    private fun deviceStatus(name: String, device: Any?): String = "$name: " + when (device) {
        (device == null) -> "DISCONNECTED"
        is DcMotor -> "${device.mode} P${device.power}"
        is Servo -> "-> ${device.position}"
        is DistanceSensor -> "${device.getDistance(DistanceUnit.INCH)}in"
        else -> "${device!!::class.simpleName} READY"
    }

    fun tele(t: Telemetry) {
        for (field in mapOf(
            "driveMotors" to driveMotors,
            "liftLeft" to liftLeft,
            "liftRight" to liftRight,
            "dumperRotate" to dumperRotate,
            "dumperLatch" to dumperLatch,
            "drone" to drone,
            "distanceLeft" to distanceLeft,
            "distanceRight" to distanceRight
        ).entries
        ) {
            t.addLine(deviceStatus(field.key, field.value))
        }
    }

    abstract fun clearEncoders()
}