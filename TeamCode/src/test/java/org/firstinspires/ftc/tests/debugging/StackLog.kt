package org.firstinspires.ftc.tests.debugging

private val interestingFrames = listOf(
    "org.firstinspires.ftc",
    "dev.aether"
)

fun logStack(): String {
    try { throw Exception() }
    catch (e: Exception) {
        val frames = e.stackTrace
        val lines = StringBuilder()
        lines.append("\nStack trace snapshot (most recent call first):\n")
        var ignoreCounter = 1
        val stackEntryLength = frames.size.toString().length
        var lastUsedStackFrame = 0
        for ((stackEntry, frame) in frames.withIndex()) {
            if (interestingFrames.none { frame.className.contains(it) }) {
                continue
            }
            if (frame.methodName.equals("logStack")) ignoreCounter = 2
            if (ignoreCounter > 0) {
                ignoreCounter -= 1
                continue
            }
            if (lastUsedStackFrame != stackEntry - 1) {
                lines.append("... ${stackEntry - lastUsedStackFrame} internal frames ...\n")
            }
            lines.append("${stackEntry.toString().padEnd(stackEntryLength)} ${frame.className}#${frame.methodName} (line ${frame.lineNumber})\n")
            lastUsedStackFrame = stackEntry
        }
        return lines.toString()
    }
}