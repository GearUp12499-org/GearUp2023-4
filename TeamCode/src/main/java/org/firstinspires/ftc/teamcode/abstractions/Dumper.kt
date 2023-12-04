package org.firstinspires.ftc.teamcode.abstractions

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
import org.firstinspires.ftc.teamcode.Var
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration

class Dumper(
    private val scheduler: MultitaskScheduler,
    config: RobotConfiguration
) {
    @JvmField
    val rotate = config.dumperRotate()

    @JvmField
    val latch = config.dumperLatch()
    private val lock = config.dumperLock

    companion object {
        // TODO trim timings
        const val RotateTime = 1000 // ms
        const val LatchTime = 200 // ms
    }

    enum class State {
        Idle,
        Dump,
        Dump2
    }

    private var dumperState: State = State.Idle

    fun dump() {
        if (scheduler.isResourceInUse(lock)) return
        // (dep: idle) dump
        scheduler.task {
            +lock
            onStart { ->
                dumperState = State.Dump
                rotate.position = Var.Box.dumpRotate
                latch.position = Var.Box.latched
            }
            maxDuration(RotateTime)
        }
    }

    fun dumpSecond() {
        if (scheduler.isResourceInUse(lock)) return
        var base: Task? = null
        if (dumperState == State.Idle) base = scheduler.task {
            +lock
            onStart { ->
                dumperState = State.Dump
                rotate.position = Var.Box.dumpRotate
                latch.position = Var.Box.latched
            }
            maxDuration(RotateTime)
        }
        // (dep: idle) dump
        val add = scheduler.task {
            +lock
            onStart { ->
                dumperState = State.Dump2
                latch.position = Var.Box.unlatched
            }
            maxDuration(LatchTime)
        }
        base?.then(add)
    }

    fun reset() {
        if (scheduler.isResourceInUse(lock)) return
        scheduler.task {
            +lock
            onStart { ->
                dumperState = State.Idle
                rotate.position = Var.Box.idleRotate
                latch.position = Var.Box.latched
            }
            maxDuration(RotateTime)
        }
    }


    val state: State get() = dumperState

    fun defaultPos() {
        rotate.position = Var.Box.idleRotate
        latch.position = Var.Box.latched
    }
}
