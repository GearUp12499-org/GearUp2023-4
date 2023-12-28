package org.firstinspires.ftc.teamcode.odo

import android.util.Log
import com.qualcomm.robotcore.util.ElapsedTime
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID.rampDown
import org.firstinspires.ftc.teamcode.odo.DriveForwardPID.rampUp
import org.firstinspires.ftc.teamcode.odo.EncoderMath.tick2inch
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

class KOdometryDrive(
    private val scheduler: MultitaskScheduler,
    robot: RobotConfiguration
) {
    companion object {
        const val ACCEPTABLE_ERROR = .5
        const val kpFwd = 0.2
        const val kiFwd = 0.05
        val StrafingCurve = ControlRamps(
            .3,
            .25,
            .4,
            6.0,
            24.0,
        )
    }

    private val driveMotors = robot.driveMotors()
    private val rightOdo = driveMotors.backRight
    private val leftOdo = driveMotors.frontLeft
    private val strafeOdo = robot.intake()
    private val dmLock = robot.driveMotorLock

    private fun distanceLeft() = tick2inch(leftOdo.currentPosition)
    private fun distanceRight() = tick2inch(rightOdo.currentPosition)
    private fun distanceStrafe() = tick2inch(strafeOdo.currentPosition)

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

                if (average > distInch - ACCEPTABLE_ERROR) {
                    complete = true
                    return@onTick
                }

                val pError = rDist - lDist
                val currentTime = deltaT.time()
                val dt = currentTime - previousTime
                previousTime = currentTime
                sumError += pError * dt

                val correction = (kpFwd * pError + kiFwd * sumError) * switcher
                val speed = min(rampUp(average), rampDown(distInch - average)) * switcher
                driveMotors.frontLeft.power = speed + correction
                driveMotors.backLeft.power = speed + correction
                driveMotors.frontRight.power = speed - correction
                driveMotors.backRight.power = speed - correction
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
                // FIXME: is `* switcher` correct here?
                val lErr = (distanceLeft() - lBase) * switcher
                val rErr = (distanceRight() - rBase) * switcher

                if (sDist > distInch - ACCEPTABLE_ERROR) {
                    completed = true
                    return@onTick
                }

                val currentTime = deltaT.time()
                val dt = currentTime - previousTime
                previousTime = currentTime

                sumErrorLeft += lErr * dt
                sumErrorRight += rErr * dt

                val lCorrect = kpFwd * lErr + kiFwd * sumErrorLeft
                val rCorrect = kpFwd * rErr + kiFwd * sumErrorRight
                val speed = StrafingCurve.ramp(sDist, distInch - sDist) * switcher

                // FIXME: are these signs correct?
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
                } else {
                    driveMotors.frontLeft.power = speed + lCorrect
                    driveMotors.backLeft.power = -speed + lCorrect
                    driveMotors.frontRight.power = -speed + rCorrect
                    driveMotors.backRight.power = speed + rCorrect
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