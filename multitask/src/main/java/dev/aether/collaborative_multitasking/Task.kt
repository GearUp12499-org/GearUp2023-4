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
class Task constructor(
    private val scheduler: MultitaskScheduler,
) {
    enum class State {
        NotStarted,
        Starting,
        Ticking,
        Finishing,
        Finished,
    }

    var state: State = State.NotStarted
        private set

    private var canStart: ((task: Task, scheduler: MultitaskScheduler) -> Boolean) = { _: Task, _: MultitaskScheduler -> true }
    private var onStart: ((task: Task, scheduler: MultitaskScheduler) -> Unit) = { _: Task, _: MultitaskScheduler -> }
    private var onTick: ((task: Task, scheduler: MultitaskScheduler) -> Unit) = { _: Task, _: MultitaskScheduler -> }
    private var isCompleted: ((task: Task, scheduler: MultitaskScheduler) -> Boolean) = { _: Task, _: MultitaskScheduler -> true }
    private var onFinish: ((task: Task, scheduler: MultitaskScheduler) -> Unit) = { _: Task, _: MultitaskScheduler -> }
    private var then: ((task: Task, scheduler: MultitaskScheduler) -> Unit) = { _: Task, _: MultitaskScheduler -> }

    fun canStart(block: (task: Task, scheduler: MultitaskScheduler) -> Boolean) {
        canStart = block
    }
    fun canStart(block: (task: Task) -> Boolean) {
        canStart = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun canStart(block: () -> Boolean) {
        canStart = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeCanStart(): Boolean {
        return canStart(this, scheduler)
    }

    fun onStart(block: (task: Task, scheduler: MultitaskScheduler) -> Unit) {
        onStart = block
    }
    fun onStart(block: (task: Task) -> Unit) {
        onStart = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun onStart(block: () -> Unit) {
        onStart = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeOnStart() {
        onStart(this, scheduler)
    }

    fun onTick(block: (task: Task, scheduler: MultitaskScheduler) -> Unit) {
        onTick = block
    }
    fun onTick(block: (task: Task) -> Unit) {
        onTick = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun onTick(block: () -> Unit) {
        onTick = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeOnTick() {
        onTick(this, scheduler)
    }

    fun isCompleted(block: (task: Task, scheduler: MultitaskScheduler) -> Boolean) {
        isCompleted = block
    }
    fun isCompleted(block: (task: Task) -> Boolean) {
        isCompleted = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun isCompleted(block: () -> Boolean) {
        isCompleted = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeIsCompleted(): Boolean {
        return isCompleted(this, scheduler)
    }

    fun onFinish(block: (task: Task, scheduler: MultitaskScheduler) -> Unit) {
        onFinish = block
    }
    fun onFinish(block: (task: Task) -> Unit) {
        onFinish = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun onFinish(block: () -> Unit) {
        onFinish = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeOnFinish() {
        onFinish(this, scheduler)
    }

    fun then(block: (task: Task, scheduler: MultitaskScheduler) -> Unit) {
        then = block
    }
    fun then(block: (task: Task) -> Unit) {
        then = { that: Task, _: MultitaskScheduler -> block(that) }
    }
    fun then(block: () -> Unit) {
        then = { _: Task, _: MultitaskScheduler -> block() }
    }
    fun invokeThen() {
        then(this, scheduler)
    }

    fun runTaskAfter(configure: Task.() -> Unit): Task {
        val task = Task(scheduler)
        task.configure()
        val capturedCanStart = task.canStart
        task.canStart = { that: Task, scheduler2: MultitaskScheduler ->
            capturedCanStart(that, scheduler2) && this.state == State.Finished
        }
        return task
    }
}