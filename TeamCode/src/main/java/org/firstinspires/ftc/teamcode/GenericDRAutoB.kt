package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.IMU
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.abstractions.Claw
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration
import org.firstinspires.ftc.teamcode.odo.RotationUnit
import org.firstinspires.ftc.teamcode.odo.degrees
import org.firstinspires.ftc.teamcode.utility.MotorPowers

/**
 * @param imu IMU in degrees, reset to 0
 */
class GenericDRAutoB(
    private val robotConfig: RobotConfiguration,
    private val scheduler: MultitaskScheduler,
    private val claw: Claw,
    private val imu: IMU,
) {
    // TODO: INCOMPLETE
    private val driveMotors = robotConfig.driveMotors()

    companion object {
        const val DriveForwardSpeed = 0.25
        const val DriveForwardTime = 1000 // ms

        const val TurnSpeed = 0.25
        const val TurnDuration = 750 // ms
    }

    private fun getHeading(): RotationUnit {
        return imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES).degrees
    }

    private fun doA180(): Task {
        return scheduler.task {
            // Placeholder values...
            var startHeading: RotationUnit = 0.degrees
            var targetHeading: RotationUnit = 180.degrees
            var needFlip = false
            +robotConfig.driveMotorLock
            onStart { ->
                startHeading = getHeading()
                targetHeading = (startHeading + targetHeading)
                // FIXME: if this ends up near 180 we can totally pass by it
                needFlip = targetHeading > 180.degrees
            }
        }
    }

    private fun forwardPart(): Task {
        return scheduler.task {
            +robotConfig.driveMotorLock
            onStart { ->
                driveMotors.setAll(DriveForwardSpeed)
            }
            maxDuration(DriveForwardTime)
            onFinish { ->
                driveMotors.setAll(0.0)
            }
        }
    }

    fun left(): Task = forwardPart().then(turn(TurnSpeed)).then(claw.release())
    fun center(): Task = forwardPart().then(claw.release())
    fun right(): Task = forwardPart().then(turn(-TurnSpeed)).then(claw.release())

    private fun turn(fac: Double, duration: Int = TurnDuration): Task {
        return scheduler.task {
            +robotConfig.driveMotorLock
            onStart { ->
                MotorPowers(
                    1 * fac,
                    -1 * fac,
                    1 * fac,
                    -1 * fac
                ).apply(driveMotors)
            }
            maxDuration(duration)
        }
    }
}