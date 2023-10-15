package org.firstinspires.ftc.teamcode.abstractions

import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.teamcode.configurations.RobotLocks

class Claw(
    private val scheduler: MultitaskScheduler,
    private val grip: Servo,
    private val rotate: Servo
) {
    companion object {
        // massive TODO get actual numbers lmao
        const val ROTATE_HOVER = 0.83
        const val ROTATE_CLOSING = 0.868
        const val ROTATE_FLIP = 0.255
        const val ROTATE_STOW = 0.5425
        const val GRIP_OPEN = 0.57
        const val GRIP_CLOSED = 0.825
        const val CloseRotateTime = 100 // ms
        const val GripTime = 100 // ms
        const val FlipTime = 1000 // ms
    }

    private enum class State {
        Ready,
        Closing,
        Closed,
        Flipping,
        Stowed
    }

    private var clawState: State = State.Ready

    fun restore() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return // don't try to do two things at once
        scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = State.Flipping
                grip.position = GRIP_OPEN
                rotate.position = ROTATE_HOVER
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = State.Ready
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
            maxDuration(CloseRotateTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                grip.position = GRIP_CLOSED
            }
            maxDuration(GripTime)
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
            maxDuration(FlipTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                grip.position = GRIP_OPEN
            }
            maxDuration(GripTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                rotate.position = ROTATE_STOW
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = State.Stowed
            }
        }
    }
}