package org.firstinspires.ftc.tests.mock

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.tests.MinimalLogger

class FakeDcMotor(private val name: String, private val port: Int = 0) : DcMotorEx, Simulated {
    data class SimulatorRules(
        /**
         * Minimum speed to start moving, when the motor is not already moving.
         * This is to simulate the motor's inertia.
         */
        val startingMinPower: Double = 0.0,
        /**
         * If the power goes below this value while running, the motor will stop.
         */
        val stallPower: Double = 0.0,
        /**
         * Rate, in revolution/second, at which the motor travels at max speed.
         * This naively assumes that the motor is under the same load at all speeds.
         * Use the conditions that the motor is under "most of the time" during the simulation.
         */
        val maxRate: Double = 0.0,
    )

    object SimulationPresets {

    }

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Other

    override fun getDeviceName(): String = name

    override fun getConnectionInfo(): String = "(virtual)"

    override fun getVersion(): Int = 1

    override fun resetDeviceConfigurationForOpMode() {
        direction = DcMotorSimple.Direction.FORWARD
    }

    override fun close() {
        MinimalLogger.nyi("FakeDcMotor", "close")
    }

    private var iDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
        set(value) {
            MinimalLogger.d("FakeDcMotor", "direction set to $value on $name")
            field = value
        }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        iDirection = direction ?: DcMotorSimple.Direction.FORWARD
    }

    override fun getDirection(): DcMotorSimple.Direction = iDirection

    private var iPower: Double = 0.0
        set(value) {
            MinimalLogger.d("FakeDcMotor", "power set to $value on $name")
            field = value
        }

    override fun setPower(power: Double) {
        iPower = power
    }

    override fun getPower(): Double = iPower

    override fun getMotorType(): MotorConfigurationType? = null

    override fun setMotorType(motorType: MotorConfigurationType?) {
        MinimalLogger.nyi("FakeDcMotor", "setMotorType")
    }

    override fun getController(): DcMotorController? {
        MinimalLogger.nyi("FakeDcMotor", "getController")
        return null
    }

    override fun getPortNumber(): Int = port

    private var iZPB: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN
        set(value) {
            MinimalLogger.d("FakeDcMotor", "zeroPowerBehavior set to $value on $name")
            field = value
        }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        if (zeroPowerBehavior == null) {
            MinimalLogger.nyie("FakeDcMotor", "setZeroPowerBehavior", "zeroPowerBehavior is null")
            throw IllegalStateException()
        }
        if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.UNKNOWN) {
            throw IllegalStateException("DcMotor zeroPowerBehaviour cannot be set to UNKNOWN.")
        }
        iZPB = zeroPowerBehavior
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior = iZPB

    @Deprecated(
        "See DcMotor interface docs for more information",
        ReplaceWith("setZeroPowerBehaviour(DcMotor.ZeroPowerBehaviour.FLOAT)")
    )
    override fun setPowerFloat() {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }

    override fun getPowerFloat(): Boolean = zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT

    private var iTargetPosition: Int = 0
        set(value) {
            MinimalLogger.d("FakeDcMotor", "targetPosition set to $value on $name")
            field = value
        }

    override fun setTargetPosition(position: Int) {
        val modifier = if (direction == DcMotorSimple.Direction.REVERSE) -1 else 1
        iTargetPosition = position * modifier
    }

    override fun getTargetPosition(): Int {
        val modifier = if (direction == DcMotorSimple.Direction.REVERSE) -1 else 1
        return iTargetPosition * modifier
    }

    override fun isBusy(): Boolean {
        TODO("Simulated components not yet implemented")
    }

    override fun getCurrentPosition(): Int {
        TODO("Virtual encoders not yet implemented")
    }

    private var iRunMode: RunMode = RunMode.RUN_WITHOUT_ENCODER
        set(value) {
            MinimalLogger.d("FakeDcMotor", "runMode set to $value on $name")
            field = value
        }

    override fun setMode(mode: RunMode?) {
        if (mode == null) {
            throw IllegalStateException("setting runMode to null would throw NPE in Java")
        }
        @Suppress("Deprecation")
        val modeMigrated = when (mode) {
            RunMode.RUN_WITHOUT_ENCODERS -> MinimalLogger.w(
                "FakeDcMotor",
                "[deprecated] RUN_WITHOUT_ENCODERS is deprecated, use RUN_WITHOUT_ENCODER instead"
            ) { RunMode.RUN_WITHOUT_ENCODER }

            RunMode.RUN_USING_ENCODERS -> MinimalLogger.w(
                "FakeDcMotor",
                "[deprecated] RUN_USING_ENCODERS is deprecated, use RUN_USING_ENCODER instead"
            ) { RunMode.RUN_USING_ENCODER }

            RunMode.RESET_ENCODERS -> MinimalLogger.w(
                "FakeDcMotor",
                "[deprecated] RESET_ENCODERS is deprecated, use STOP_AND_RESET_ENCODER instead"
            ) { RunMode.STOP_AND_RESET_ENCODER }

            else -> mode
        }
        iRunMode = modeMigrated
    }

    override fun getMode(): RunMode = iRunMode

    override fun setMotorEnable() {
        TODO("Energizing/De-energizing motors not yet implemented")
    }

    override fun setMotorDisable() {
        TODO("Energizing/De-energizing motors not yet implemented")
    }

    override fun isMotorEnabled(): Boolean {
        MinimalLogger.nyi("FakeDcMotor", "isMotorEnabled")
        return true
    }

    private var iVelocity: Double = 0.0
        set(value) {
            MinimalLogger.d("FakeDcMotor", "velocity set to $value on $name")
            field = value
        }

    override fun setVelocity(angularRate: Double) {
        mode = when (mode) {
            RunMode.RUN_USING_ENCODER -> RunMode.RUN_WITHOUT_ENCODER
            RunMode.RUN_TO_POSITION -> RunMode.RUN_TO_POSITION
            else -> MinimalLogger.w(
                "FakeDcMotor",
                "setVelocity called in $mode, which is not a velocity-controlled mode. Switching to RUN_WITHOUT_ENCODER, which may cause unexpected behavior."
            ) { RunMode.RUN_WITHOUT_ENCODER }
        }
        MinimalLogger.nyi("FakeDcMotor", "setVelocity")
        iVelocity = angularRate
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit?) {
        // WIP
        velocity = angularRate
    }

    override fun getVelocity(): Double {
        MinimalLogger.nyi("FakeDcMotor", "getVelocity(<no args>)")
        return iVelocity
    }

    override fun getVelocity(unit: AngleUnit?): Double {
        MinimalLogger.nyi("FakeDcMotor", "getVelocity(AngleUnit)")
        return iVelocity
    }

    @Deprecated(
        "Use implementation with PIDF coefficients instead",
        ReplaceWith("setVelocityPIDFCoefficients(mode, PIDFCoefficients(pidCoefficients))")
    )
    override fun setPIDCoefficients(mode: RunMode?, pidCoefficients: PIDCoefficients?) {
        setPIDFCoefficients(mode, PIDFCoefficients(pidCoefficients))
    }

    override fun setPIDFCoefficients(mode: RunMode?, pidfCoefficients: PIDFCoefficients?) {
        TODO("Not yet implemented")
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        TODO("Not yet implemented")
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        TODO("Not yet implemented")
    }

    override fun getPIDCoefficients(mode: RunMode?): PIDCoefficients {
        TODO("Not yet implemented")
    }

    override fun getPIDFCoefficients(mode: RunMode?): PIDFCoefficients {
        TODO("Not yet implemented")
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        TODO("Not yet implemented")
    }

    override fun getTargetPositionTolerance(): Int {
        TODO("Not yet implemented")
    }

    override fun getCurrent(unit: CurrentUnit?): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAlert(unit: CurrentUnit?): Double {
        TODO("Not yet implemented")
    }

    override fun setCurrentAlert(current: Double, unit: CurrentUnit?) {
        TODO("Not yet implemented")
    }

    override fun isOverCurrent(): Boolean {
        TODO("Not yet implemented")
    }

    override fun advance(seconds: Double) {

    }
}