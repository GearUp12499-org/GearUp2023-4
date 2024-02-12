package org.firstinspires.ftc.teamcode.odo

import android.util.Log
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.EncoderMath.tick2inch
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.MotorPowers
import org.firstinspires.ftc.teamcode.utilities.Move
import org.firstinspires.ftc.teamcode.utilities.RotationUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * do not use (yet!!)
 */
class KOdometryDrive(
    private val scheduler: MultitaskScheduler,
    robot: RobotConfiguration,
) {
    companion object {
        const val ACCEPTABLE_ERROR_STRAFE = .5
        const val ACCEPTABLE_ERROR_FWDBCK = 1.0
//        const val ACCEPTABLE_ERROR_TURN = 0.2
        const val kpFwd = 0.2
        const val kpStr = 0.6
        const val kpRot = 0.002
        const val kiFwd = 1.0 // FIXME
        const val kiStr = 0.05

        const val ForwardMaxSpeed = 0.3
        val ForwardingCurve = QuadraticDownRamps(
            0.05,
            0.05,
            ForwardMaxSpeed,
            6.0,
            12.0,
        )

        const val StrafingMaxSpeed = 0.5

        // previous version of this curve @ 9405a9d1f89cb164c81c14ca659724933698ae92
        val StrafingCurve = QuadraticDownRamps(
            .1,
            .1,
            StrafingMaxSpeed,
            .5, // in SECONDS, not inches
            8.0,
        )

        const val TurningMaxSpeed = 0.9
        val TurningCurve = ControlRamps(
            0.37,
            TurningMaxSpeed,
            0.0,
            18.0
        )
    }

    private val driveMotors = robot.driveMotors()
    private val rightOdo = robot.odoParallelRight()
    private val leftOdo = robot.odoParallelLeft()
    private val strafeOdo = robot.odoPerpendicular()
    private val dmLock = robot.driveMotorLock

    // Which of these are reversed!?!?
    private fun distanceLeft() = tick2inch(leftOdo.currentPosition)
    private fun distanceRight() = tick2inch(rightOdo.currentPosition)
    private fun distanceStrafe() = -tick2inch(strafeOdo.currentPosition)

    @JvmOverloads
    fun driveForward(target: LengthUnit, timeout: Double = 4.0): Task {
        val distInch = abs(target.to.inches.value)
        val switcher = target.value.sign

        return scheduler.task {
            +dmLock
            var lBase = 0.0
            var rBase = 0.0
            val timeoutT = ElapsedTime()
            val deltaT = ElapsedTime()
            var previousTime = deltaT.time()
            var sumError = 0.0

            var complete = false
            var lastPower: MotorPowers? = null

            onStart { ->
                lBase = distanceLeft()
                rBase = distanceRight()
                timeoutT.reset()
                sumError = 0.0
                previousTime = deltaT.time()
            }
            onTick { ->
                val lDist = (distanceLeft() - lBase) * switcher
                val rDist = (distanceRight() - rBase) * switcher
                val average = (lDist + rDist) / 2.0

                if (average > distInch - ACCEPTABLE_ERROR_FWDBCK) {
                    Log.i(
                        "KOD",
                        String.format(
                            "L/R %+.4f %+.4f AVG %+.4f LastSpd %s",
                            lDist,
                            rDist,
                            average,
                            lastPower ?: "<none>"
                        )
                    )
                    complete = true
                    return@onTick
                }

                val pError = rDist - lDist
                val currentTime = deltaT.time()
                val dt = currentTime - previousTime
                previousTime = currentTime
                sumError += pError * dt

                val correction = (kpFwd * pError + kiFwd * sumError) * switcher
                // FIXME timeoutT.time()
                val speed = ForwardingCurve.ramp(average, distInch - average) * switcher

                var speeds = MotorPowers(
                    frontLeft = speed + correction,
                    backLeft = speed + correction,
                    frontRight = speed - correction,
                    backRight = speed - correction
                )
                speeds = speeds.normalNoStretch(ForwardMaxSpeed)
                // normalization on the next line only to keep consistency with rotation
                val powers = speeds.map(Move::rampSpeedToPower).normalNoStretch()
                lastPower = powers
                powers.apply(driveMotors)

                /* Log.i(
                    "KOD",
                    "SumOfError ${
                        String.format(
                            "%+.8f",
                            sumError
                        )
                    } in*sec ki=$kiFwd, Error ${String.format("%+.8f", pError)} kp=$kpFwd"
                ) */
            }
            isCompleted { -> complete || (timeout > 0 && timeoutT.time() >= timeout) }
            onFinish { ->
                driveMotors.setAll(0.0)
            }
        }
    }

    @JvmOverloads
    fun driveReverse(target: LengthUnit, timeout: Double = -1.0): Task {
        return driveForward(-target, timeout)
    }

    @JvmOverloads
    fun strafeRight(targetAnyUnit: LengthUnit, timeout: Double = 3.0): Task {
        val target = abs(targetAnyUnit.to.inches.value)
        Log.i(
            "KOD",
            String.format("strafe ${if (targetAnyUnit.value < 0) "left" else "right"} $target inches")
        )
        val switcher = targetAnyUnit.value.sign

        return scheduler.task {
            +dmLock
            var sBase = 0.0
            var lBase = 0.0
            var rBase = 0.0

            val timeoutT = ElapsedTime()
            val deltaT = ElapsedTime()

            var sumErrorLeft = 0.0
            var sumErrorRight = 0.0

            var completed = false

            onStart { ->
                sBase = distanceStrafe()
                lBase = distanceLeft()
                rBase = distanceRight()
                timeoutT.reset()
                deltaT.reset()
            }
            onTick { ->
                val sDist = (distanceStrafe() - sBase) * switcher
                val lErr = distanceLeft() - lBase
                val rErr = distanceRight() - rBase

                if (sDist > target - ACCEPTABLE_ERROR_STRAFE) {
                    completed = true
                    return@onTick
                }

                val dt = deltaT.time()
                deltaT.reset()

                sumErrorLeft += lErr * dt
                sumErrorRight += rErr * dt

                val lCorrect = kpStr * lErr + kiStr * sumErrorLeft
                val rCorrect = kpStr * rErr + kiStr * sumErrorRight
                val speed = StrafingCurve.ramp(timeoutT.time(), target - sDist) * switcher
//                Log.i("Encoders", String.format("L %+.4f  R %+.4f  P %+.4f", lErr, rErr, sDist))
                if (abs(lCorrect) > 1 || abs(rCorrect) > 1) {
                    // uh oh!!!
                    Log.e(
                        "KOdometryDrive", String.format(
                            "PANIC: base: %+.4f lC: %+.4f rC: %+.4f = " +
                                    "FL %+.4f / FR %+.4f / BL %+.4f / BR %+.4f",
                            speed,
                            lCorrect,
                            rCorrect,
                            speed + lCorrect,
                            -speed + rCorrect,
                            -speed + lCorrect,
                            speed + rCorrect
                        )
                    )
                }
                var speeds = MotorPowers(
                    frontLeft = speed - lCorrect,
                    frontRight = -speed - rCorrect,
                    backLeft = -speed - lCorrect,
                    backRight = speed - rCorrect
                )
                speeds = speeds.normalNoStretch(StrafingMaxSpeed)
                // normalization on the next line only to keep consistency with rotation
                val powers = speeds.map(Move::rampSpeedToPower).normalNoStretch()
                powers.apply(driveMotors)
            }
            isCompleted { -> completed || (timeout > 0 && timeoutT.time() >= timeout) }
            onFinish { ->
                driveMotors.setAll(0.0)
            }
        }
    }

    @JvmOverloads
    fun strafeLeft(target: LengthUnit, timeout: Double = -1.0) = strafeRight(-target, timeout)

    // !!! UNTESTED DO NOT USE AAAAAAAAAAAAAAAAAA
    /**
     * Turn the robot counterclockwise (left) by the given angle.
     * If the angle is negative, the robot will turn clockwise (right).
     * Timeout currently does Absolutely Nothing.
     */
    @JvmOverloads
    fun turnCounterClockwise(angle: RotationUnit, timeout: Double = -1.0): Task {
        val robotRadius = 7.5
        val target = abs(angle.to.degrees.value)
        val switcher = angle.value.sign

        // Every 45 degrees the error increases by about 2.5 degrees --> at power 0.6
        // The error for 45 degrees is 9 degrees
        val errorAdjustment = (abs(target) / 25.0) * 2.0 /* + 9.0 */
        val turnDist = 2 * PI * robotRadius * ((abs(target) - errorAdjustment) / 360.0)
        return scheduler.task {
            +dmLock
            var lBase = 0.0
            var rBase = 0.0
            var complete = false

            onStart { ->
                lBase = distanceLeft()
                rBase = distanceRight()
            }
            onTick { ->
                val lDist = (lBase - distanceLeft()) * switcher
                val rDist = (distanceRight() - rBase) * switcher
                val average = (lDist + rDist) / 2.0

                if (average > turnDist - ACCEPTABLE_ERROR_FWDBCK) {
                    complete = true
                    return@onTick
                }
                val error = rDist + lDist // why is this not wrong?!?
                Log.i(
                    "KOD",
                    String.format(
                        "Turn: L %+.4f  R %+.4f  Avg %+.4f  errorP %+.4f",
                        lDist,
                        rDist,
                        average,
                        error
                    )
                )

                val correction = kpRot * error * switcher
                // Parity: there is no up-ramp here. see ForwardingCurve definition
                val speed = ForwardingCurve.ramp(0.0, turnDist - average) * switcher
                val speeds = MotorPowers(
                    backLeft = -speed - correction,
                    frontLeft = -speed - correction,
                    frontRight = speed + correction,
                    backRight = speed + correction
                )
                // oh well, compat
//                speeds = speeds.normalNoStretch(TurningMaxSpeed)
//                 because TurningMaxSpeed is .9, we might need to normalize here
//                val powers = speeds.map(Move::rampSpeedToPower).normalNoStretch()
                speeds.apply(driveMotors)
            }
            isCompleted { -> complete }
            onFinish { ->
                driveMotors.setAll(0.0)
            }
        }
    }

    @JvmOverloads
    fun turnClockwise(angle: RotationUnit, timeout: Double = -1.0) =
        turnCounterClockwise(-angle, timeout)
}