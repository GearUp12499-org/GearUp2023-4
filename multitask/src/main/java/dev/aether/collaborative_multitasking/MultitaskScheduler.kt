package dev.aether.collaborative_multitasking

class MultitaskScheduler : Scheduler() {
    private val locks: MutableMap<Loq, Int?> = mutableMapOf()
    private val tasks: MutableMap<Int, Task> = mutableMapOf()
    override var nextId: Int = 0
        private set
    private var tickCount = 0

    private fun selectState(state: Task.State): List<Task> {
        return tasks.values.filter { it.state == state }
    }

    private fun allFreed(requirements: Set<Loq>): Boolean {
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
    }

    override fun tick() {
        tickMarkStartable()
        tickStartMarked()
        tickTick()
        tickFinish()
        tickCount++
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
            if (!locks.containsKey(lock)) {
                locks[lock] = null
            }
        }
        return id
    }

    override fun isResourceInUse(resource: Loq): Boolean {
        return locks[resource] != null
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