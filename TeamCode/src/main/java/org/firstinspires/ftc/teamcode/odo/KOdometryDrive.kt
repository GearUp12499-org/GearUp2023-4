package org.firstinspires.ftc.teamcode.odo

import android.util.Log
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.EncoderMath.tick2inch
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.MotorPowers
import kotlin.math.abs
import kotlin.math.sign

class KOdometryDrive(
    private val scheduler: MultitaskScheduler,
    robot: RobotConfiguration
) {
    companion object {
        const val ACCEPTABLE_ERROR_STRAFE = .5
        const val ACCEPTABLE_ERROR_FWDBCK = 1.0
        const val kpFwd = 0.2
        const val kpStr = 0.2
        const val kiFwd = 1.0 // FIXME
        const val kiStr = 0.05
        val ForwardingCurve = ControlRamps(
            0.25,
            0.15,
            0.3,
            6.0,
            8.0,
        )
        // previous version of this curve @ 9405a9d1f89cb164c81c14ca659724933698ae92
        val StrafingCurve = ControlRamps(
            .3,
            .25,
            .5,
            3.0,
            6.0,
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
    fun driveForward(target: LengthUnit, timeout: Double = -1.0): Task {
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
                    complete = true
                    return@onTick
                }

                val pError = rDist - lDist
                val currentTime = deltaT.time()
                val dt = currentTime - previousTime
                previousTime = currentTime
                sumError += pError * dt

                val correction = (kpFwd * pError + kiFwd * sumError) * switcher
                val speed = ForwardingCurve.ramp(average, distInch - average) * switcher
                driveMotors.frontLeft.power = speed + correction
                driveMotors.backLeft.power = speed + correction
                driveMotors.frontRight.power = speed - correction
                driveMotors.backRight.power = speed - correction
                Log.i("KOD", "SumOfError ${String.format("%+.8f", sumError)} in*sec ki=$kiFwd, Error ${String.format("%+.8f", pError)} kp=$kpFwd")
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
    fun strafeRight(target: LengthUnit, timeout: Double = -1.0): Task {
        val distInch = abs(target.to.inches.value)
        Log.i(
            "KOD",
            String.format("strafe ${if (target.value < 0) "left" else "right"} $distInch inches")
        )
        val switcher = target.value.sign

        return scheduler.task {
            +dmLock
            var sBase = 0.0
            var lBase = 0.0
            var rBase = 0.0

            val timeoutT = ElapsedTime()
            val deltaT = ElapsedTime()
            var previousTime = deltaT.time()

            var sumErrorLeft = 0.0
            var sumErrorRight = 0.0

            var completed = false

            onStart { ->
                sBase = distanceStrafe()
                lBase = distanceLeft()
                rBase = distanceRight()
                timeoutT.reset()
                previousTime = deltaT.time()
            }
            onTick { ->
                val sDist = (sBase - distanceStrafe()) * switcher
                val lErr = distanceLeft() - lBase
                val rErr = distanceRight() - rBase

                if (sDist > distInch - ACCEPTABLE_ERROR_STRAFE) {
                    completed = true
                    return@onTick
                }

                val currentTime = deltaT.time()
                val dt = currentTime - previousTime
                previousTime = currentTime

                sumErrorLeft += lErr * dt
                sumErrorRight += rErr * dt

                val lCorrect = kpStr * lErr + kiStr * sumErrorLeft
                val rCorrect = kpStr * rErr + kiStr * sumErrorRight
                val speed = StrafingCurve.ramp(sDist, distInch - sDist) * switcher

                if (abs(lCorrect) > .5 || abs(rCorrect) > .5) {
                    // we're screwed
                    driveMotors.setAll(0.0)

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
                } else { // Divide (speed + correct) by max abs power
                    val powers = MotorPowers(
                        frontLeft = speed - lCorrect,
                        frontRight = -speed - rCorrect,
                        backLeft = -speed - lCorrect,
                        backRight = speed - rCorrect
                    )
                    powers.normalize().apply(driveMotors)
                }
            }
            isCompleted { -> completed || (timeout > 0 && timeoutT.time() >= timeout) }
            onFinish { ->
                driveMotors.setAll(0.0)
            }
        }
    }

    @JvmOverloads
    fun strafeLeft(target: LengthUnit, timeout: Double = -1.0) = strafeRight(-target, timeout)
}