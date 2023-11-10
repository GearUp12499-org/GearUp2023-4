@file:Suppress("unused")

package dev.aether.collaborative_multitasking

/*
@startuml
hide empty description
state Task {
    state "Waiting" as NotStarted
    state "Starting" as Starting {
        state "Event" as StartEvent : call start()
        state "isCompleted()" as StartIsCompleted : check for completion
        StartEvent -> StartIsCompleted
    }
    state "Ticking" as Ticking {
        state "Event" as TickEvent : call tick()
        state "isCompleted()" as TickIsCompleted : check for completion
        TickEvent -> TickIsCompleted
    }
    state "Finishing" as Finishing {
        state "Event" as FinishEvent : call finish()
        state "then(...)" as FinishAndThen : queue next tasks
        FinishEvent -> FinishAndThen
    }
    state "Finished" as Finished
    NotStarted <- [*]
    NotStarted --> Starting : canStart and\n scheduler choice
    Starting -> Ticking : not completed
    Starting --> Finishing : completed
    Ticking -> Ticking : scheduler tick
    Ticking --> Finishing : completed
    Finishing --> Finished
    Finished -> [*]
}
@enduml
 */

typealias TaskQuery2<T> = (task: Task, scheduler: Scheduler) -> T
typealias TaskQuery1<T> = (task: Task) -> T
typealias Producer<T> = () -> T
typealias JTaskQuery2<T> = java.util.function.BiFunction<Task, Scheduler, T>
typealias JTaskQuery1<T> = java.util.function.Function<Task, T>
typealias JProducer<T> = java.util.function.Supplier<T>
typealias TaskAction2 = TaskQuery2<Unit>
typealias TaskAction1 = TaskQuery1<Unit>
typealias Runnable = () -> Unit
typealias JTaskAction2 = java.util.function.BiConsumer<Task, Scheduler>
typealias JTaskAction1 = java.util.function.Consumer<Task>
typealias JRunnable = java.lang.Runnable

class Task constructor(
    internal var scheduler: Scheduler,
) {
    enum class State(val order: Int) {
        NotStarted(0),
        Starting(1),
        Ticking(2),
        Finishing(3),
        Finished(4),
    }

    var state: State = State.NotStarted
        private set

    var startedAt: Int? = null
        private set

    fun setState(newState: State) {
        println("task ${myId}: transition: ${state.name} -> ${newState.name}")
        if (state.order > newState.order) {
            throw IllegalStateException("cannot move from ${state.name} to ${newState.name}")
        }
        if (state == newState) return
        when (newState) {
            State.Starting -> startedAt = scheduler.getTicks()
            State.Finishing -> println("task ${myId}: finishing at ${scheduler.getTicks()} (run for ${scheduler.getTicks() - (startedAt ?: 0)} ticks)")
            else -> {}
        }
        state = newState
    }

    private var requirements: MutableSet<SharedResource> = mutableSetOf()

    internal var canStart: TaskQuery2<Boolean> = { _: Task, _: Scheduler -> true }
    internal var onStart: TaskAction2 = { _: Task, _: Scheduler -> }
    private var onTick: TaskAction2 = { _: Task, _: Scheduler -> }
    internal var isCompleted: TaskQuery2<Boolean> = { _: Task, _: Scheduler -> false }
    private var onFinish: TaskAction2 = { _: Task, _: Scheduler -> }
    private var then: TaskAction2 = { _: Task, _: Scheduler -> }

    var daemon = false
    var myId: Int? = null
        private set

    fun canStart(block: TaskQuery2<Boolean>) {
        canStart = block
    }

    fun canStart(block: TaskQuery1<Boolean>) {
        canStart = { that: Task, _: Scheduler -> block(that) }
    }

    fun canStart(block: Producer<Boolean>) {
        canStart = { _: Task, _: Scheduler -> block() }
    }

    fun canStart(block: JTaskQuery2<Boolean>) {
        canStart = { that: Task, scheduler2: Scheduler -> block.apply(that, scheduler2) }
    }

    fun canStart(block: JTaskQuery1<Boolean>) {
        canStart = { that: Task, _: Scheduler -> block.apply(that) }
    }

    fun canStart(block: JProducer<Boolean>) {
        canStart = { _: Task, _: Scheduler -> block.get() }
    }

    fun invokeCanStart(): Boolean {
        return canStart(this, scheduler)
    }

    fun onStart(block: TaskAction2) {
        onStart = block
    }

    fun onStart(block: TaskAction1) {
        onStart = { that: Task, _: Scheduler -> block(that) }
    }

    fun onStart(block: Runnable) {
        onStart = { _: Task, _: Scheduler -> block() }
    }

    fun onStart(block: JTaskAction2) {
        onStart = { that: Task, scheduler2: Scheduler -> block.accept(that, scheduler2) }
    }

    fun onStart(block: JTaskAction1) {
        onStart = { that: Task, _: Scheduler -> block.accept(that) }
    }

    fun onStart(block: JRunnable) {
        onStart = { _: Task, _: Scheduler -> block.run() }
    }

    fun invokeOnStart() {
        onStart(this, scheduler)
    }

    fun onTick(block: TaskAction2) {
        onTick = block
    }

    fun onTick(block: TaskAction1) {
        onTick = { that: Task, _: Scheduler -> block(that) }
    }

    fun onTick(block: Runnable) {
        onTick = { _: Task, _: Scheduler -> block() }
    }

    fun onTick(block: JTaskAction2) {
        onTick = { that: Task, scheduler2: Scheduler -> block.accept(that, scheduler2) }
    }

    fun onTick(block: JTaskAction1) {
        onTick = { that: Task, _: Scheduler -> block.accept(that) }
    }

    fun onTick(block: JRunnable) {
        onTick = { _: Task, _: Scheduler -> block.run() }
    }

    fun invokeOnTick() {
        onTick(this, scheduler)
    }

    fun isCompleted(block: TaskQuery2<Boolean>) {
        isCompleted = block
    }

    fun isCompleted(block: TaskQuery1<Boolean>) {
        isCompleted = { that: Task, _: Scheduler -> block(that) }
    }

    fun isCompleted(block: Producer<Boolean>) {
        isCompleted = { _: Task, _: Scheduler -> block() }
    }

    fun isCompleted(block: JTaskQuery2<Boolean>) {
        isCompleted = { that: Task, scheduler2: Scheduler -> block.apply(that, scheduler2) }
    }

    fun isCompleted(block: JTaskQuery1<Boolean>) {
        isCompleted = { that: Task, _: Scheduler -> block.apply(that) }
    }

    fun isCompleted(block: JProducer<Boolean>) {
        isCompleted = { _: Task, _: Scheduler -> block.get() }
    }

    fun invokeIsCompleted(): Boolean {
        return isCompleted(this, scheduler)
    }

    fun onFinish(block: TaskAction2) {
        onFinish = block
    }

    fun onFinish(block: TaskAction1) {
        onFinish = { that: Task, _: Scheduler -> block(that) }
    }

    fun onFinish(block: Runnable) {
        onFinish = { _: Task, _: Scheduler -> block() }
    }

    fun onFinish(block: JTaskAction2) {
        onFinish = { that: Task, scheduler2: Scheduler -> block.accept(that, scheduler2) }
    }

    fun onFinish(block: JTaskAction1) {
        onFinish = { that: Task, _: Scheduler -> block.accept(that) }
    }

    fun onFinish(block: JRunnable) {
        onFinish = { _: Task, _: Scheduler -> block.run() }
    }

    fun invokeOnFinish() {
        onFinish(this, scheduler)
    }

    operator fun SharedResource.unaryPlus() {
        require(this)
    }

    private fun unrequire(lockName: SharedResource) {
        requirements.remove(lockName)
    }

    fun require(lockName: SharedResource) {
        requirements.add(lockName)
    }

    fun requirements(): Set<SharedResource> {
        return requirements.toSet()
    }

    internal fun register() {
        myId = scheduler.register(this)
    }

    fun then(configure: Task.() -> Unit): Task {
        val task = Task(scheduler)
        task.configure()
        then(task)
        task.register() // ready to go
        return task
    }

    fun then(task: Task): Task {
        val capturedCanStart = task.canStart
        task.canStart = { that: Task, scheduler2: Scheduler ->
            capturedCanStart(that, scheduler2) && this.state == State.Finished
        }
        return task
    }

    fun apply(configure: Task.() -> Unit) {
        this.configure()
    }
}
