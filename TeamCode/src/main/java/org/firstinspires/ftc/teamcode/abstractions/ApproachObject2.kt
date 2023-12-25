package org.firstinspires.ftc.teamcode.abstractions

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.inches
import org.firstinspires.ftc.teamcode.Var.ApproachObject as AOV

class ApproachObject2(private val scheduler: MultitaskScheduler, robot: RobotConfiguration) {
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
            onTick { ->
                val left = distanceLeft.getDistance(DistanceUnit.INCH)
                val right = distanceRight.getDistance(DistanceUnit.INCH)
                val avg = (left + right) / 2.0

                if (avg < di + ACCEPTABLE_ERROR) {
                    completed = true
                    return@onTick
                }

                val err = right - left
                val correction = (kP * err)
                val speed = -rampDown((avg - di).inches)
                motors.frontLeft.power = speed + correction
                motors.backLeft.power = speed + correction
                motors.frontRight.power = speed - correction
                motors.backRight.power = speed - correction
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