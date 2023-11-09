package org.firstinspires.ftc.teamcode.abstractions

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.maxDuration
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
        const val ROTATE_IDLE = 0.531
        const val ROTATE_DUMP = 0.316
        const val UNLATCHED = 0.4
        const val LATCHED = 0.78

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
                rotate.position = ROTATE_DUMP
                latch.position = LATCHED
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
                rotate.position = ROTATE_DUMP
                latch.position = LATCHED
            }
            maxDuration(RotateTime)
        }
        // (dep: idle) dump
        val add = scheduler.task {
            +lock
            onStart { ->
                dumperState = State.Dump2
                latch.position = UNLATCHED
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
                rotate.position = ROTATE_IDLE
                latch.position = LATCHED
            }
            maxDuration(RotateTime)
        }
    }


    val state: State get() = dumperState

}