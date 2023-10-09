package dev.aether.collaborative_multitasking

class MultitaskScheduler {
    private val locks: MutableMap<String, Int?> = mutableMapOf()
    private val tasks: MutableMap<Int, Task> = mutableMapOf()
    private var nextId = 0
    private var tickCount = 0

    private fun selectState(state: Task.State): List<Task> {
        return tasks.values.filter { it.state == state }
    }

    private fun allFreed(requirements: Set<String>): Boolean {
        return requirements.all { locks[it] == null }
    }

    private fun tickMarkStartable() {
        selectState(Task.State.NotStarted)
            .filter {
                it.invokeCanStart() && allFreed(it.requirements())
            }
            .forEach {
                it.setState(Task.State.Starting)
                // acquire locks
                for (lock in it.requirements()) {
                    println("task ${it.myId} acquired $lock")
                    locks[lock] = it.myId
                }
            }
    }

    private fun tickStartMarked() {
        selectState(Task.State.Starting)
            .forEach {
                it.invokeOnStart()
                if (it.invokeIsCompleted()) {
                    it.setState(Task.State.Finishing)
                } else {
                    it.setState(Task.State.Ticking)
                }
            }
    }

    private fun tickTick() {
        selectState(Task.State.Ticking)
            .forEach {
                it.invokeOnTick()
                if (it.invokeIsCompleted()) it.setState(Task.State.Finishing)
            }
    }

    private fun tickFinish() {
        val candidates = selectState(Task.State.Finishing)
        candidates.forEach {
            it.invokeOnFinish()
            it.setState(Task.State.Finished)
            // release locks
            for (lock in it.requirements()) {
                if (locks[lock] != it.myId) {
                    throw IllegalStateException("task ${it.myId} (which just finished) does not own lock $lock that it is supposed to own")
                }
                locks[lock] = null
                println("task ${it.myId} released $lock")
            }
        }
        candidates.forEach {
            it.invokeThen()  // after finish, do the next task (maybe)
        }
    }

    fun tick() {
        tickMarkStartable()
        tickStartMarked()
        tickTick()
        tickFinish()
        tickCount++
    }

    fun getTicks(): Int {
        return tickCount
    }

    fun task(configure: Task.() -> Unit): Task {
        val task = Task(this)
        task.configure()
        task.register()
        return task
    }
    internal fun register(task: Task): Int {
        val id = nextId++
        tasks[id] = task
        for (lock in task.requirements()) {
            if (!locks.containsKey(lock)) {
                locks[lock] = null
            }
        }
        return id
    }

    fun hasJobs(): Boolean {
        return tasks.values.any { it.state != Task.State.Finished }
    }

    fun runToCompletion() {
        while (hasJobs()) {
            tick()
        }
    }
}