package dev.aether.collaborative_multitasking

import kotlin.math.ceil
import kotlin.math.max

class MultitaskScheduler : Scheduler() {
    private val locks: MutableMap<String, Int?> = mutableMapOf()
    private val tasks: MutableMap<Int, Task> = mutableMapOf()
    private val lockIdName: MutableMap<String, SharedResource> = mutableMapOf()

    companion object {
        const val evict = true
        const val rollingAverageSize = 10000

        private fun <T : Comparable<T>, N : List<T>> percentile(k: N, l: Double): T {
            val index = ceil(l * k.size).toInt()
            return k[max(0, index - 1)]
        }
    }

    val tickTimes: MutableList<Double> = mutableListOf()

    override var nextId: Int = 0
        private set
    private var tickCount = 0

    private fun selectState(state: Task.State): List<Task> {
        return tasks.values.filter { it.state == state }
    }

    private fun allFreed(requirements: Set<SharedResource>): Boolean {
        return requirements.all { locks[it.id] == null }
    }

    private fun tickMarkStartable() {
        selectState(Task.State.NotStarted)
            .filter {
                it.invokeCanStart()
            }
            .forEach {
                if (allFreed(it.requirements())) {
                    it.setState(Task.State.Starting)
                    // acquire locks
                    for (lock in it.requirements()) {
                        println("task ${it.myId} acquired $lock")
                        locks[lock.id] = it.myId
                        lockIdName[lock.id] = lock
                        println("locks: $locks")
                    }
                }
            }
    }

    private fun tickStartMarked() {
        selectState(Task.State.Starting)
            .forEach {
                try {
                    it.invokeOnStart()
                    if (it.invokeIsCompleted()) {
                        it.setState(Task.State.Finishing)
                    } else {
                        it.setState(Task.State.Ticking)
                    }
                } catch (e: Exception) {
                    System.err.println(
                        String.format(
                            "Error while marking %s to start:",
                            it.toString()
                        )
                    )
                    e.printStackTrace()
                }
            }
    }

    private fun tickTick() {
        selectState(Task.State.Ticking)
            .forEach {
                try {
                    it.invokeOnTick()
                    if (it.invokeIsCompleted()) it.setState(Task.State.Finishing)
                } catch (e: Exception) {
                    System.err.println(String.format("Error while ticking %s:", it.toString()))
                    e.printStackTrace()
                }
            }
    }

    private fun tickFinish() {
        val candidates = selectState(Task.State.Finishing)
        candidates.forEach(::release)
    }

    private fun release(task: Task) {
        if (task.state == Task.State.NotStarted) {
            task.setState(Task.State.Finished)
            return
        }
        try {
            task.invokeOnFinish()
        } catch (e: Exception) {
            System.err.println(
                String.format(
                    "Error while processing %s finish handler:",
                    task.toString()
                )
            )
            e.printStackTrace()
        }
        task.setState(Task.State.Finished)
        for (lock in task.requirements()) {
            if (locks[lock.id] != task.myId) {
                throw IllegalStateException("task ${task.myId} (which just finished) does not own lock $lock that it is supposed to own")
            }
            locks[lock.id] = null
            println("task ${task.myId} released $lock")
        }
    }

    override fun tick() {
        val start = System.nanoTime()
        tickMarkStartable()
        tickStartMarked()
        tickTick()
        tickFinish()
        tickCount++
        val durat = (System.nanoTime() - start) / 1000000.0
        tickTimes.add(durat)
        if (durat > 1000) {
            System.err.println(String.format("Warning: tick %d took %.2f ms", tickCount - 1, durat))
        }
        if (evict && tickTimes.size > rollingAverageSize) tickTimes.removeAt(0)
    }

    fun statsheet(): String {
        val s = tickTimes.sorted()
        return String.format(
            "%d samples:\n" +
                    "  [Min  ][1%%   ][5%%   ][32%%  ][50%%  ][68%%  ][95%%  ][99%%  ][Max  ]\n" +
                    "  %7.1f%7.1f%7.1f%7.1f%7.1f%7.1f%7.1f%7.1f%7.1f",
            s.size,
            percentile(s, 0.0),
            percentile(s, 0.01),
            percentile(s, 0.05),
            percentile(s, 0.32),
            percentile(s, 0.5),
            percentile(s, 0.68),
            percentile(s, 0.95),
            percentile(s, 0.99),
            percentile(s, 1.0)
        )
    }

    override fun getTicks(): Int {
        return tickCount
    }

    override fun task(configure: Task.() -> Unit): Task {
        val task = Task(this)
        task.configure()
        task.register()
        return task
    }

    override fun register(task: Task): Int {
        val id = nextId++
        tasks[id] = task
        for (lock in task.requirements()) {
            if (!locks.containsKey(lock.id)) {
                locks[lock.id] = null
            }
        }
        return id
    }

    override fun isResourceInUse(resource: SharedResource): Boolean {
        return locks[resource.id] != null
    }

    override fun panic() {
        for (task in tasks.values) {
            if (task.state == Task.State.Finished || task.state == Task.State.NotStarted) continue
            task.invokeOnFinish()
            task.setState(Task.State.Finished)
        }

        for (lock in lockIdName.values) {
            lock.panic()
        }
    }

    fun hasJobs(): Boolean {
        return tasks.values.any { it.state != Task.State.Finished && !it.daemon }
    }

    fun runToCompletion(ok: () -> Boolean) {
        while (hasJobs() && ok()) {
            tick()
        }
    }

    /**
     * Stops any tasks matching the predicate that are not already finished or haven't started yet.
     * Resources owned by matching tasks are guaranteed to be released after this call.
     */
    fun filteredStop(predicate: (Task) -> Boolean) {
        tasks.values
            .filter { it.state != Task.State.Finished && it.state != Task.State.NotStarted }
            .filter(predicate)
            .forEach(::release)
    }
}