package org.firstinspires.ftc.tests.mock

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.tests.MinimalLogger
import org.firstinspires.ftc.tests.debugging.logStack
import kotlin.math.abs

class FakeServo(
    private val name: String,
    private val port: Int = 0,
    startingPosition: Double = 0.0,
    private val rules: SimulatorRules = SimulationPresets.NoLoadGoBildaSpeed,
) : Servo, Simulated {
    data class SimulatorRules(val speedPerSecond: Double)

    object SimulationPresets {
        val Stuck = SimulatorRules(speedPerSecond = 0.0)
        val NoLoadGoBildaTorque = SimulatorRules(speedPerSecond = 0.8000)
        val NoLoadGoBildaSpeed = SimulatorRules(speedPerSecond = 1.8182)
        val NoLoadGoBildaSuperSpeed = SimulatorRules(speedPerSecond = 3.6364)
        val HS448HB = SimulatorRules(speedPerSecond = 1.4354)
    }

    var actualPos: Double = startingPosition
        private set(newValue) {
            MinimalLogger.v("FakeServo", logStack())
            field = newValue
        }
    val reachedTarget: NotifyList<FakeServo> = mutableListOf()
    fun teleport(newPosition: Double) {
        position = newPosition
        actualPos = iPosition
        reachedTarget.forEach { it(this) }
    }

    fun rawTeleport(newPosition: Double) {
        iPosition = newPosition
        actualPos = iPosition
        reachedTarget.forEach { it(this) }
    }

    /**
     * Assumption: Servo motion is linear.
     */
    override fun advance(seconds: Double) {
        val distanceToTravel = abs(iPosition - actualPos)
        if (distanceToTravel == 0.0) return
        if (seconds * rules.speedPerSecond > distanceToTravel) {
            actualPos = iPosition
            MinimalLogger.i("FakeServo", "made it!")
            reachedTarget.forEach { it(this) }
            return
        }
        val travel = (iPosition - actualPos) * (rules.speedPerSecond * seconds)
        actualPos += travel
        MinimalLogger.v("FakeServo", "sim: $travel, now $actualPos")
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Other

    override fun getDeviceName(): String = name

    override fun getConnectionInfo(): String = "(virtual)"

    override fun getVersion(): Int = 1

    override fun resetDeviceConfigurationForOpMode() {
        TODO("Not yet implemented")
    }

    override fun close() {
        MinimalLogger.nyi("FakeServo", "close")
    }

    override fun getController(): ServoController? {
        MinimalLogger.nyi("FakeServo", "getController")
        return null
    }

    override fun getPortNumber(): Int = port

    private var iDirection: Servo.Direction = Servo.Direction.FORWARD
        set(value) {
            MinimalLogger.d("FakeServo", "direction set to $value on $name")
            field = value
        }

    override fun setDirection(direction: Servo.Direction?) {
        iDirection = direction ?: Servo.Direction.FORWARD
    }

    override fun getDirection(): Servo.Direction = iDirection

    private var iPosition: Double = 0.0
        set(value) {
            MinimalLogger.d("FakeServo", "position set to $value on $name")
            field = value
        }
    private var limitPositionMin: Double = 0.0
    private var limitPositionMax: Double = 1.0

    override fun setPosition(position: Double) {
        var clipped = Range.clip(position, 0.0, 1.0)
        if (direction == Servo.Direction.REVERSE) {
            clipped = reverse(clipped)
        }
        val scaled = Range.scale(clipped, 0.0, 1.0, limitPositionMin, limitPositionMax)
        iPosition = scaled
    }

    override fun getPosition(): Double {
        var position = iPosition
        if (direction == Servo.Direction.REVERSE) {
            position = reverse(position)
        }
        return Range.scale(position, limitPositionMin, limitPositionMax, 0.0, 1.0)
    }

    override fun scaleRange(min: Double, max: Double) {
        val clippedMin = Range.clip(min, 0.0, 1.0)
        val clippedMax = Range.clip(max, 0.0, 1.0)

        if (clippedMin >= clippedMax) {
            throw IllegalArgumentException("min must be less than max")
        }

        limitPositionMin = clippedMin
        limitPositionMax = clippedMax
    }

    private fun reverse(position: Double) = 1.0 - position
}