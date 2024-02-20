package org.firstinspires.ftc.teamcode.abstractions

import android.util.Log
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.MotorPowers
import org.firstinspires.ftc.teamcode.utilities.Move
import org.firstinspires.ftc.teamcode.utilities.inches
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import org.firstinspires.ftc.teamcode.Var.ApproachObject as AOV

class ApproachObject2 @JvmOverloads constructor(
    private val scheduler: MultitaskScheduler,
    robot: RobotConfiguration,
    val SensorOverflow: Double = 100.0
) {
    companion object {
        const val kP = 0.1
        const val ACCEPTABLE_ERROR = 0.25
    }

    private val motors = robot.driveMotors()
    private val distanceLeft = robot.distanceLeft()
    private val distanceRight = robot.distanceRight()
    private val dmLock = robot.driveMotorLock
    private fun rampDown(distToTarget: LengthUnit): Double {
        return when {
            distToTarget <= 0.inches -> 0.0
            distToTarget >= (AOV.stoppingDistance) -> AOV.maxSpeed
            else -> {
                (AOV.maxSpeed - AOV.minSpeed) * (distToTarget / AOV.stoppingDistance) + AOV.minSpeed
            }
        }
    }

    fun approach(targetDistance: LengthUnit): Task {
        val di = targetDistance.to.inches.value
        return scheduler.task {
            +dmLock
            var completed = false
            var backupValue = 50.0

            val allowCorrections = false
            onStart { ->
                // don't stop immediately lmao
                backupValue = 50.0
            }
            onTick { ->
                var left = distanceLeft.getDistance(DistanceUnit.INCH)
                var right = distanceRight.getDistance(DistanceUnit.INCH)

                if (left < SensorOverflow) backupValue = left
                if (right < SensorOverflow) backupValue = right
                if (left > SensorOverflow && right > SensorOverflow) {
                    left = backupValue
                    right = backupValue
                } else if (left > SensorOverflow) {
                    left = right
                } else if (right > SensorOverflow) {
                    right = left
                }

                val distTravel = min(left, right)

                if (distTravel < di + ACCEPTABLE_ERROR) {
                    completed = true
                    return@onTick
                }

                val err = right - left

//                val correction = (kP * virtErr)
                val correction = 0.0
                val speed = -rampDown((distTravel - di).inches)
                val speeds = MotorPowers(
                    speed + correction,
                    speed - correction,
                    speed + correction,
                    speed - correction
                )
                val powers = speeds.map(Move::rampSpeedToPower)
                powers.apply(motors)
                if (allowCorrections)
                    Log.d(
                        "ApproachObject2",
                        String.format(
                            "LR\t%.3f\t%.3f\t Err \t%.3f\tTrav\t%.3f\tSLeft\t%+.3f\tSRight\t%+.3f\tPLeft\t%+.3f\tPRight\t%+.3f",
                            left,
                            right,
                            err,
                            distTravel,
                            speeds.frontLeft,
                            speeds.frontRight,
                            powers.frontLeft,
                            powers.frontRight
                        )
                    )
                else
                    Log.d(
                        "ApproachObject2",
                        String.format(
                            "LR\t%.3f\t%.3f\t(Err)\t%+.3f\tTrav\t%.3f\t(correction off)\tSAll\t%+.3f\tPAll\t%+.3f",
                            left,
                            right,
                            err,
                            distTravel,
                            speeds.frontLeft,
                            powers.frontLeft
                        )
                    )
            }
            isCompleted { -> completed }
            maxDuration(3000)
            onFinish { ->
                motors.setAll(0.0)
            }
        }
    }

    fun approachNoStack(distance: LengthUnit): Task? {
        if (scheduler.isResourceInUse(dmLock)) return null
        return approach(distance)
    }
}