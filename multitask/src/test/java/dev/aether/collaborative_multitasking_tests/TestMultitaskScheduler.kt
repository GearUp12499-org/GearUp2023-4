package dev.aether.collaborative_multitasking_tests

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import dev.aether.collaborative_multitasking.ext.delay
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals

internal class TestMultitaskScheduler {
    @Test
    fun `Create a scheduler and task`() {
        val scheduler = MultitaskScheduler()
        val task = scheduler.task {}
        assertNotEquals(null, task.myId, "task not registered")
    }

    @Test
    fun `Create tests in a chain`() {
        val scheduler = MultitaskScheduler()
        val task1 = scheduler.task {}
        val task2 = task1.chain {}
        val task3 = task2.chain {}
        assertNotEquals(null, task1.myId, "task1 not registered")
        assertNotEquals(null, task2.myId, "task2 not registered")
        assertNotEquals(null, task3.myId, "task3 not registered")
    }

    @Test
    fun `Start a task`() {
        val scheduler = MultitaskScheduler()
        var onStartCalled = false
        var onTickCalled = false
        val task = scheduler.task {
            onStart { ->
                onStartCalled = true
            }
            onTick { ->
                onTickCalled = true
            }
            isCompleted { -> false }
        }
        assertEquals(Task.State.NotStarted, task.state, "task not in NotStarted state")
        scheduler.tick()
        assert(onStartCalled) { "onStart not called" }
        assert(onTickCalled) { "onTick not called" }
    }

    @Test
    fun `Run a task to completion`() {
        val scheduler = MultitaskScheduler()
        val tickCount = 256
        val task = scheduler.task {
            isCompleted { _, scheduler -> scheduler.getTicks() >= tickCount }
        }
        scheduler.runToCompletion() // blocking
        assertEquals(Task.State.Finished, task.state, "task not in Finished state")
        assert(scheduler.getTicks() >= tickCount) { "scheduler stopped early" }
    }

    @Test
    fun `Run tasks sharing a lock`() {
        val scheduler = MultitaskScheduler()
        val driveMotorLock = "drive_motor"
        val task1life = 512
        var task1start: Int? = null
        val task2life = 128
        var task2start: Int? = null
        scheduler.task {
            +driveMotorLock
            onStart { ->
                task1start = scheduler.getTicks()
            }
            isCompleted { that, scheduler ->
                if (that.startedAt != null) {
                    // if we get screwed by a multithreaded access then we have bigger problems
                    scheduler.getTicks() - that.startedAt!! >= task1life
                } else {
                    false
                }
            }
        }
        scheduler.tick() // ensure the first task has started and owns the motor lock...
        assertNotEquals(null, task1start, "task1 not started")
        scheduler.task {
            +driveMotorLock
            onStart { ->
                task2start = scheduler.getTicks()
            }
            isCompleted { that, scheduler ->
                if (that.startedAt != null) {
                    scheduler.getTicks() - that.startedAt!! >= task2life
                } else {
                    false
                }
            }
        }
        scheduler.runToCompletion()
        assertNotEquals(null, task2start, "task2 not started")
        val task1finish = task1start!! + task1life
        assert(task2start!! >= task1finish) { "task2 started before task1 finished $task1finish $task2start" }
    }

    @Test
    fun `Test delays`() {
        val scheduler = MultitaskScheduler()
        val duration = 100
        val passAfter = duration - 5
        scheduler.task {
            delay(duration)
        }
        val start = System.currentTimeMillis()
        scheduler.runToCompletion()
        val end = System.currentTimeMillis()
        assert(end - start >= passAfter) { "task completed too early" }
    }
}
