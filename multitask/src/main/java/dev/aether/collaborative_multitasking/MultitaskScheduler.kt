package dev.aether.collaborative_multitasking

class MultitaskScheduler {
    fun task(configure: Task.() -> Unit): Task {
        val task = Task(this)
        task.configure()
        return task
    }
}