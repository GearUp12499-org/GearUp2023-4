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

typealias TaskQuery2<T> = (task: Task, scheduler: MultitaskScheduler) -> T
typealias TaskQuery1<T> = (task: Task) -> T
typealias Producer<T> = () -> T
typealias TaskAction2 = TaskQuery2<Unit>
typealias TaskAction1 = TaskQuery1<Unit>
typealias Runnable = () -> Unit

class Task constructor(
    private val scheduler: MultitaskScheduler,
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
            State.Finishing -> println("task ${myId}: finishing at ${scheduler.getTicks()} (run for ${scheduler.getTicks() - (startedAt?:0)} ticks)")
            else -> {}
        }
        state = newState
    }

    private var requirements: MutableSet<String> = mutableSetOf()

    private var canStart: TaskQuery2<Boolean> = { _: Task, _: MultitaskScheduler -> true }
    internal var onStart: TaskAction2 = { _: Task, _: MultitaskScheduler -> }
    private var onTick: TaskAction2 = { _: Task, _: MultitaskScheduler -> }
    internal var isCompleted: TaskQuery2<Boolean> = { _: Task, _: MultitaskScheduler -> false }
    private var onFinish: TaskAction2 = { _: Task, _: MultitaskScheduler -> }
    private var then: TaskAction2 = { _: Task, _: MultitaskScheduler -> }

    var myId: Int? = null
        private set

    fun canStart(block: TaskQuery2<Boolean>) {
        canStart = block
    }
    fun canStart(block: TaskQuery1<Boolean>) {
        canStart = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun canStart(block: Producer<Boolean>) {
        canStart = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeCanStart(): Boolean {
        return canStart(this, scheduler)
    }

    fun onStart(block: TaskAction2) {
        onStart = block
    }
    fun onStart(block: TaskAction1) {
        onStart = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun onStart(block: Runnable) {
        onStart = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeOnStart() {
        onStart(this, scheduler)
    }

    fun onTick(block: TaskAction2) {
        onTick = block
    }
    fun onTick(block: TaskAction1) {
        onTick = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun onTick(block: Runnable) {
        onTick = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeOnTick() {
        onTick(this, scheduler)
    }

    fun isCompleted(block: TaskQuery2<Boolean>) {
        isCompleted = block
    }
    fun isCompleted(block: TaskQuery1<Boolean>) {
        isCompleted = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun isCompleted(block: Producer<Boolean>) {
        isCompleted = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeIsCompleted(): Boolean {
        return isCompleted(this, scheduler)
    }

    fun onFinish(block: TaskAction2) {
        onFinish = block
    }
    fun onFinish(block: TaskAction1) {
        onFinish = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun onFinish(block: Runnable) {
        onFinish = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeOnFinish() {
        onFinish(this, scheduler)
    }

    fun then(block: TaskAction2) {
        then = block
    }
    fun then(block: TaskAction1) {
        then = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun then(block: Runnable) {
        then = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeThen() {
        then(this, scheduler)
    }

    operator fun String.unaryPlus() {
        require(this)
    }

    @Suppress("MemberVisibilityCanBePrivate")
    fun require(lockName: String) {
        requirements.add(lockName)
    }

    fun requirements(): Set<String> {
        return requirements.toSet()
    }

    internal fun register() {
        myId = scheduler.register(this)
    }

    fun chain(configure: Task.() -> Unit): Task {
        val task = Task(scheduler)
        task.configure()
        val capturedCanStart = task.canStart
        task.canStart = { that: Task, scheduler2: MultitaskScheduler ->
            capturedCanStart(that, scheduler2) && this.state == State.Finished
        }
        task.register() // ready to go
        return task
    }

    fun apply(configure: Task.() -> Unit) {
        this.configure()
    }
}