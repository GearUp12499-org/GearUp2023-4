package org.firstinspires.ftc.teamcode.abstractions

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Var
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.utilities.LengthUnit
import org.firstinspires.ftc.teamcode.utilities.inches

class ApproachObject(
    private val scheduler: MultitaskScheduler?,
    robot: RobotConfiguration
) {

    private val motors = robot.driveMotors()
    private val distanceLeft = robot.distanceLeft()
    private val distanceRight = robot.distanceRight()
    private val dmLock = robot.driveMotorLock

    private fun powerFunction(target: LengthUnit, distance: LengthUnit): Double {
        return when {
            distance <= target -> 0.0
            distance >= (target + Var.ApproachObject.stoppingDistance) -> Var.ApproachObject.maxSpeed

            else -> {
                (
                        (Var.ApproachObject.maxSpeed - Var.ApproachObject.minSpeed)
                                * ((distance - target) / Var.ApproachObject.stoppingDistance)
                                + Var.ApproachObject.minSpeed
                        )
            }
        }
    }

    fun approach(distance: LengthUnit): Task {
        scheduler ?: throw AssertionError("Need scheduler to make task")
        return scheduler.task {
            +dmLock
            onStart { ->
                motors.setAll(Var.ApproachObject.maxSpeed)
            }
            onTick { ->
                val left = distanceLeft.getDistance(DistanceUnit.INCH)
                val right = distanceRight.getDistance(DistanceUnit.INCH)
                println(
                    "going Left ${left.inches} : ${
                        powerFunction(
                            distance,
                            left.inches
                        )
                    } ...Right ${right.inches} : ${powerFunction(distance, right.inches)}"
                )
                motors.frontRight.power = -powerFunction(distance, right.inches)
                motors.frontLeft.power = -powerFunction(distance, left.inches)
                motors.backLeft.power = -powerFunction(distance, left.inches)
                motors.backRight.power = -powerFunction(distance, right.inches)
            }
            isCompleted { ->
                val left = distanceLeft.getDistance(DistanceUnit.INCH).inches
                val right = distanceRight.getDistance(DistanceUnit.INCH).inches
                return@isCompleted (
                        ((left <= distance) && (right <= distance))
                                || left > Var.ApproachObject.panicDistance
                                || right > Var.ApproachObject.panicDistance
                        )
            }
            maxDuration(3000)
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