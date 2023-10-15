package org.firstinspires.ftc.teamcode.abstractions

import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.ext.minDuration
import org.firstinspires.ftc.teamcode.configurations.RobotLocks

class Claw(
    private val scheduler: MultitaskScheduler,
    private val grip: Servo,
    private val rotate: Servo
) {
    companion object {
        // massive TODO get actual numbers lmao
        const val ROTATE_HOVER = 0.1
        const val ROTATE_CLOSING = 0.0
        const val ROTATE_FLIP = 0.5
        const val GRIP_OPEN = 0.5
        const val GRIP_CLOSED = 0.5
        const val CloseRotateTime = 100 // ms
        const val GripTime = 100 // ms
        const val FlipTime = 1000 // ms
    }

    private enum class State {
        Idle,
        Closing,
        Closed,
        Flipping
    }

    private var clawState: State = State.Idle

    fun restore() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return // don't try to do two things at once
        scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = State.Flipping
                grip.position = GRIP_OPEN
                rotate.position = ROTATE_HOVER
            }
        }
    }

    fun attempt() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return // don't try to do two things at once
        scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = State.Closing
                rotate.position = ROTATE_CLOSING
            }
            minDuration(CloseRotateTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                grip.position = GRIP_CLOSED
            }
            minDuration(GripTime)
            onFinish { ->
                clawState = State.Closed
            }
        }
    }

    fun commit() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return // don't try to do two things at once
        scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = State.Flipping
                rotate.position = ROTATE_FLIP
            }
            minDuration(FlipTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                grip.position = GRIP_OPEN
            }
            minDuration(GripTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                rotate.position = ROTATE_HOVER
            }
            minDuration(FlipTime)
            onFinish { ->
                clawState = State.Idle
            }
        }
    }
}