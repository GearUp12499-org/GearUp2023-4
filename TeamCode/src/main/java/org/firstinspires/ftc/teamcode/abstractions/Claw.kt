@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.abstractions

import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.teamcode.ClawVar

class Claw(
    private val scheduler: MultitaskScheduler,
    @JvmField val grip: Servo,
    @JvmField val rotate: Servo,
    private val lock: SharedResource,
) {
    companion object {
        // Try to avoid changing these here. Instead, change them in Variables.kt.
        const val ROTATE_HOVER = ClawVar.HoverRotation
        const val ROTATE_CLOSING = ClawVar.ClosingRotation
        const val ROTATE_FLIP = ClawVar.FlippedRotation
        const val ROTATE_STOW = ClawVar.StowedRotation
        const val GRIP_OPEN = ClawVar.ClawOpened
        const val GRIP_CLOSED = ClawVar.ClawClosed

        // TODO trim timings
        const val CloseRotateTime = 100 // ms
        const val GripTime = 450 // ms
        const val FlipTime = 500 // ms
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
        if (scheduler.isResourceInUse(lock)) return
        // (dep: hover) open -> grab -> hover
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val main = scheduler.task {
            +lock
            onStart { ->
                clawState = StateV2.Active
                grip.position = GRIP_OPEN
                rotate.position = ROTATE_CLOSING
            }
            maxDuration(FlipTime)
        }.then {
            +lock
            onStart { ->
                grip.position = GRIP_CLOSED
            }
            maxDuration(GripTime)
        }.then(gotoHover())
        base?.then(main)
    }

    fun deposit() {
        if (scheduler.isResourceInUse(lock)) return
        // (dep: hover) -> deposit -> stowed
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val main = scheduler.task {
            +lock
            onStart { ->
                clawState = StateV2.Active
                rotate.position = ROTATE_FLIP
            }
            maxDuration(FlipTime)
        }.then {
            +lock
            onStart { ->
                grip.position = GRIP_OPEN
            }
            maxDuration(GripTime)
        }.then {
            +lock
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
        if (scheduler.isResourceInUse(lock)) return
        // (hover and open)
        scheduler.task {
            +lock
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
            +lock
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