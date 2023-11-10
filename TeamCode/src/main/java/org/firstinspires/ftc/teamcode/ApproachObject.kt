package org.firstinspires.ftc.teamcode

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.LengthUnit
import org.firstinspires.ftc.teamcode.odo.inches

class ApproachObject(
    private val scheduler: MultitaskScheduler?,
    robot: RobotConfiguration
) {
    companion object {
        const val maximumSpeed = 0.3
        const val minimumSpeed = 0.1
        val stoppingDistance = 6.inches
    }

    private val motors = robot.driveMotors()
    private val distanceLeft = robot.distanceLeft()
    private val distanceRight = robot.distanceRight()
    private val dmLock = robot.driveMotorLock

    private fun powerFunction(target: LengthUnit, distance: LengthUnit): Double {
        return when {
            distance <= target -> 0.0
            distance >= (target + stoppingDistance) -> maximumSpeed

            else ->
                (maximumSpeed - minimumSpeed) * ((distance - target) / stoppingDistance) + minimumSpeed
        }
    }

    fun approach(distance: LengthUnit): Task {
        scheduler ?: throw AssertionError("Need scheduler to make task")
        val inches = distance.to.inches.value
        return scheduler.task {
            +dmLock
            onStart { ->
                motors.setAll(maximumSpeed)
            }
            onTick { ->
                val errorLeft = distanceLeft.getDistance(DistanceUnit.INCH) - inches
                val errorRight = distanceRight.getDistance(DistanceUnit.INCH) - inches
                println("going Left " + powerFunction(distance, errorLeft.inches) + " ...Right " + powerFunction(distance, errorRight.inches))
                motors.frontRight.power = -powerFunction(distance, errorRight.inches)
                motors.frontLeft.power = -powerFunction(distance, errorLeft.inches)
                motors.backLeft.power = -powerFunction(distance, errorLeft.inches)
                motors.backRight.power = -powerFunction(distance, errorRight.inches)
            }
            isCompleted { ->
                val left = distanceLeft.getDistance(DistanceUnit.INCH)
                val right = distanceRight.getDistance(DistanceUnit.INCH)
                return@isCompleted (
                        (left.inches <= distance)
                                && (right.inches <= distance))
            }
            maxDuration(1000)
            onFinish { ->
                motors.setAll(0.0)
            }
        }
    }

    fun approachNoStack(distance: LengthUnit): Task? {
        scheduler ?: throw AssertionError("Need scheduler to make task")
        if (scheduler.isResourceInUse(dmLock)) return null
        return approach(distance)
    }
}