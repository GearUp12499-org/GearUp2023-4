@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.abstractions

import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.teamcode.configurations.RobotLocks

class Claw(
    private val scheduler: MultitaskScheduler,
    private val grip: Servo,
    private val rotate: Servo
) {
    companion object {
        const val ROTATE_HOVER = 0.83
        const val ROTATE_CLOSING = 0.868
        const val ROTATE_FLIP = 0.255
        const val ROTATE_STOW = 0.5425
        const val GRIP_OPEN = 0.57
        const val GRIP_CLOSED = 0.825
        // TODO trim timings
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

    private enum class StateV2 {
        Hover,
        Stowed,
        Active
    }

    private var clawState: StateV2 = StateV2.Hover

    fun grab() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return
        // (dep: hover) open -> grab -> hover
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val main = scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = StateV2.Active
                grip.position = GRIP_OPEN
                rotate.position = ROTATE_CLOSING
            }
            maxDuration(FlipTime)
        }.then {
            +RobotLocks.claw
            onStart { ->
                grip.position = GRIP_CLOSED
            }
            maxDuration(GripTime)
        }.then(gotoHover())
        base?.then(main)
    }

    fun deposit() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return
        // (dep: hover) -> deposit -> stowed
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val main = scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = StateV2.Active
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
                clawState = StateV2.Stowed
            }
        }
        base?.then(main)
    }

    fun reset() {
        if (scheduler.isResourceInUse(RobotLocks.claw)) return
        // (hover and open)
        scheduler.task {
            +RobotLocks.claw
            onStart { ->
                clawState = StateV2.Active
                grip.position = GRIP_OPEN
                rotate.position = ROTATE_HOVER
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = StateV2.Hover
            }
        }
    }

    private fun gotoHover(): Task {
        return scheduler.task {
            +RobotLocks.claw
            onStart { ->
                rotate.position = ROTATE_HOVER
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = StateV2.Hover
            }
        }
    }
}