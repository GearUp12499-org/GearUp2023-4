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

                val avg = (left + right) / 2.0

                if (avg < di + ACCEPTABLE_ERROR) {
                    completed = true
                    return@onTick
                }

                val err = right - left
                val correction = (kP * err)
                val speed = -rampDown((avg - di).inches)
                var speeds = MotorPowers(
                    speed + correction,
                    speed - correction,
                    speed + correction,
                    speed - correction
                )
                speeds = speeds.map(Move::rampSpeedToPower)
                speeds.apply(motors)
                Log.d(
                    "ApproachObject2",
                    String.format(
                        "LR %.3f %.3f Avg %.3f Left %+.3f Right %+.3f",
                        left,
                        right,
                        avg,
                        speeds.frontLeft,
                        speeds.frontRight
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