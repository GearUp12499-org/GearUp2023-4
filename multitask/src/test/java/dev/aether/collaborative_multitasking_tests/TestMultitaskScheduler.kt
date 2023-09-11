package dev.aether.collaborative_multitasking_tests

import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.Task
import kotlin.test.Test
import kotlin.test.assertEquals

internal class TestMultitaskScheduler {
    @Test
    fun `test create empty task`() {
        val scheduler = MultitaskScheduler()
        val task = scheduler.task {}
        assertEquals(Task.State.NotStarted, task.state)
    }

    @Test
    fun `test extend task`() {
        val scheduler = MultitaskScheduler()
        val task = scheduler.task {}
            .runTaskAfter {
                canStart { -> true }
            }
        assertEquals(task.invokeCanStart(), false, "Task should not be able to start because previous task is not finished")
    }
}