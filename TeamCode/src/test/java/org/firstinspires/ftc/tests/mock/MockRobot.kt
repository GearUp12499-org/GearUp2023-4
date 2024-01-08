package org.firstinspires.ftc.tests.mock

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.IMU
import dev.aether.collaborative_multitasking.SharedResource
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.utilities.MotorSet
import org.firstinspires.ftc.tests.MinimalLogger

class MockRobot : RobotConfiguration(), Simulated {
    private val frontLeft = FakeDcMotor("frontLeft")
    private val frontRight = FakeDcMotor("frontRight")
    private val backLeft = FakeDcMotor("backLeft")
    private val backRight = FakeDcMotor("backRight")

    override val driveMotors: MotorSet<FakeDcMotor>
            by lazy { MotorSet(frontLeft, frontRight, backLeft, backRight) }
    override val driveMotorLock: SharedResource
        get() = SharedResource("driveMotorLock") { driveMotors.setAll(0.0) }
    override val purpleDropper: FakeServo
            by lazy { FakeServo("purpleDropper") }
    override val liftLeft: FakeDcMotor
            by lazy { FakeDcMotor("liftLeft") }
    override val liftRight: FakeDcMotor
            by lazy { FakeDcMotor("liftRight") }
    override val liftLock: SharedResource
        get() = SharedResource("liftLock") {
            liftLeft.power = 0.0
            liftRight.power = 0.0
        }
    override val dumperRotate: FakeServo by lazy {
        FakeServo(
            "dumperRotate",
            rules = FakeServo.SimulationPresets.NoLoadGoBildaTorque
        )
    }
    override val dumperLatch: FakeServo by lazy {
        FakeServo(
            "dumperLatch",
            rules = FakeServo.SimulationPresets.HS448HB
        )
    }
    override val dumperLock: SharedResource
        get() = SharedResource("dumperLock")
    override val drone: FakeDcMotor
            by lazy { FakeDcMotor("drone") }
    override val droneLock: SharedResource
        get() = SharedResource("droneLock") {
            drone.power = 0.0
        }
    override val intake: FakeDcMotor
            by lazy { FakeDcMotor("intake") }
    override val distanceLeft: DistanceSensor?
        get() = MinimalLogger.w("MockRobot", "Distance sensors not implemented") { null }
    override val distanceRight: DistanceSensor?
        get() = MinimalLogger.w("MockRobot", "Distance sensors not implemented") { null }
    override val imu: IMU?
        get() = MinimalLogger.w("MockRobot", "IMU not implemented") { null }
    override val odoPerpendicular: FakeDcMotor
        get() = intake
    override val odoParallelLeft: FakeDcMotor
        get() = frontLeft
    override val odoParallelRight: FakeDcMotor
        get() = frontRight

    override fun clearEncoders() {
        clearEncoder(
            liftLeft,
            liftRight,
            odoParallelLeft,
            odoParallelRight,
            odoPerpendicular
        )
    }

    init {
        setReverse(frontLeft)
        setReverse(backLeft)
        enableBrakes(frontLeft, frontRight, backLeft, backRight)

        liftLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftLeft.targetPosition = 0
        liftLeft.power = 1.0
        liftLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftRight.targetPosition = 0
        liftRight.power = 1.0
        liftRight.mode = DcMotor.RunMode.RUN_TO_POSITION
        setReverse(liftRight)
    }

    private val updateOrder: List<Simulated> = listOf(
        driveMotors.frontLeft,
        driveMotors.frontRight,
        driveMotors.backLeft,
        driveMotors.backRight,
        dumperRotate,
        dumperLatch,
        purpleDropper
    )

    override fun advance(seconds: Double) {
        updateOrder.forEach { it.advance(seconds) }
    }

    companion object {
        private fun setReverse(target: DcMotor) {
            target.direction = DcMotorSimple.Direction.REVERSE
        }

        private fun enableBrakes(vararg targets: DcMotor?) {
            for (target in targets) {
                target ?: continue
                target.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }
        }

        private fun clearEncoder(vararg targets: DcMotor?) {
            for (target in targets) {
                target ?: continue
                val lastMode = target.mode
                target.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                target.mode = lastMode
            }
        }
    }
}