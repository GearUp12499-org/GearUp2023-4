package org.firstinspires.ftc.teamcode

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.teamcode.abstractions.Claw
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration

class GenericDRAutoB(
    private val robotConfig: RobotConfiguration,
    private val claw: Claw,
    private val scheduler: MultitaskScheduler
) {
    // TODO: INCOMPLETE
    private val driveMotors = robotConfig.driveMotors()

    companion object {
        const val DriveForwardSpeed = 0.25
        const val DriveForwardTime = 1000 // ms

        const val TurnSpeed = 0.25
        const val TurnDuration = 750 // ms
    }

    fun pos1(): Task {
        return scheduler.task {
            +robotConfig.driveMotorLock
            onStart { ->
                driveMotors.setAll(DriveForwardSpeed)
            }
            maxDuration(DriveForwardTime)
            onFinish { ->
                driveMotors.setAll(0.0)
            }
        }.then(claw.release())
    }

    fun turn(fac: Double): Task {
        return scheduler.task {
            +robotConfig.driveMotorLock
        }
    }
}