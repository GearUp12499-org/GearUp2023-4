@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.abstractions

import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.teamcode.Var

/**
 * This class represents the robot's claw. For manual control of the claw's positions,
 * get a reference to the Servo from your RobotConfiguration.
 * @see org.firstinspires.ftc.teamcode.configurations.RobotConfiguration.clawGrab
 * @see org.firstinspires.ftc.teamcode.configurations.RobotConfiguration.clawRotate
 *
 * @param scheduler Scheduler used to manage the claw's tasks. If null, then methods relying on
 *                  the scheduler will throw an AssertionError.
 * @param grip Servo used to control the claw's grip.
 * @param rotate Servo used to control the claw's rotation.
 * @param lock SharedResource used to prevent the claw's tasks from running at the same time as another.
 */
class Claw(
    private val scheduler: MultitaskScheduler?,
    @JvmField val grip: Servo,
    @JvmField val rotate: Servo,
    private val lock: SharedResource,
) {
    constructor(grip: Servo, rotate: Servo, lock: SharedResource) : this(null, grip, rotate, lock)

    companion object {
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


    /**
     * Schedule a task to grab an item with the claw.
     * Action is automatically cancelled if the claw is already in use.
     */
    fun grab() {
        scheduler
            ?: throw AssertionError("The Claw instance needs to be defined with a scheduler to use this method")
        if (scheduler.isResourceInUse(lock)) return
        // (dep: hover) open -> grab -> hover
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val main = scheduler.task {
            +lock
            onStart { ->
                clawState = StateV2.Active
                grip.position = Var.CLAW_OPENED
                rotate.position = Var.CLAW_CLOSING_ROTATION
            }
            maxDuration(FlipTime)
        }.then {
            +lock
            onStart { ->
                grip.position = Var.CLAW_CLOSED
            }
            maxDuration(GripTime)
        }.then(gotoHover())
        base?.then(main)
    }

    /**
     * Schedule a task to drop the item that is currently in the claw into the box.
     * Action is automatically cancelled if the claw is already in use.
     */
    fun deposit() {
        scheduler
            ?: throw AssertionError("The Claw instance needs to be defined with a scheduler to use this method")
        if (scheduler.isResourceInUse(lock)) return
        // (dep: hover) -> deposit -> stowed
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val main = scheduler.task {
            +lock
            onStart { ->
                clawState = StateV2.Active
                rotate.position = Var.CLAW_FLIPPED_ROTATION
            }
            maxDuration(FlipTime)
        }.then {
            +lock
            onStart { ->
                grip.position = Var.CLAW_OPENED
            }
            maxDuration(GripTime)
        }.then {
            +lock
            onStart { ->
                rotate.position = Var.CLAW_STOWED_ROTATION
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = StateV2.Stowed
            }
        }
        base?.then(main)
    }

    /**
     * Schedule a task to move the claw to the hovering position.
     * Action is automatically cancelled if the claw is already in use.
     */
    fun resetTele() {
        scheduler
            ?: throw AssertionError("The Claw instance needs to be defined with a scheduler to use this method")
        if (scheduler.isResourceInUse(lock)) return
        // (hover and open)
        reset()
    }

    /**
     * Schedule a task to move the claw to the hovering position.
     */
    fun reset(): Task {
        scheduler
            ?: throw AssertionError("The Claw instance needs to be defined with a scheduler to use this method")
        return scheduler.task {
            +lock
            onStart { ->
                clawState = StateV2.Active
                grip.position = Var.CLAW_OPENED
                rotate.position = Var.CLAW_HOVER_ROTATION
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = StateV2.Hover
            }
        }
    }

    private fun gotoHover(): Task {
        scheduler
            ?: throw AssertionError("The Claw instance needs to be defined with a scheduler to use this method")
        return scheduler.task {
            +lock
            onStart { ->
                rotate.position = Var.CLAW_HOVER_ROTATION
            }
            maxDuration(FlipTime)
            onFinish { ->
                clawState = StateV2.Hover
            }
        }
    }

    /**
     * WIP; schedules a task to open the claw.
     */
    fun release(): Task {
        scheduler
            ?: throw AssertionError("The Claw instance needs to be defined with a scheduler to use this method")
        var base: Task? = null
        if (clawState != StateV2.Hover) base = gotoHover()
        val ext = scheduler.task {
            +lock
            onStart { ->
                grip.position = Var.CLAW_OPENED
            }
            maxDuration(GripTime)
        }
        base?.then(ext)
        return base ?: ext
    }
}