@file:Suppress("unused")

package org.firstinspires.ftc.tests

import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

@Suppress("MemberVisibilityCanBePrivate")
object MinimalLogger {
    private val severity: Map<String, Int> = mapOf(
        "verbose" to 0,
        "debug" to 1,
        "info" to 2,
        "warning" to 3,
        "warn" to 3,
        "error" to 4,
        "err" to 4,
        "fatal" to 5,
        "assert" to 6
    )

    const val currentLevel = "debug"
    private val currentLevelN = severity[currentLevel]!!

    private fun log(level: String, tag: String, message: String) {
        val levelN = severity[level.lowercase()] ?: 3 // default: warning
        if (currentLevelN > levelN) return
        val now = LocalDateTime.now()
        val formatter = DateTimeFormatter.ofPattern("HH:mm:ss.SSS")
        val timestamp = now.format(formatter)
        println("$timestamp ${level.padEnd(7)} [${tag.padEnd(18)}] $message")
    }

    private const val DefaultTag = "<out>"

    fun v(message: String) = log("VERBOSE", DefaultTag, message)
    fun v(tag: String, message: String) = log("VERBOSE", tag, message)
    fun <T> v(tag: String, message: String, provider: () -> T): T {
        v(tag, message)
        return provider()
    }

    fun d(message: String) = log("DEBUG", DefaultTag, message)
    fun d(tag: String, message: String) = log("DEBUG", tag, message)
    fun <T> d(tag: String, message: String, provider: () -> T): T {
        d(tag, message)
        return provider()
    }

    fun i(message: String) = log("INFO", DefaultTag, message)
    fun i(tag: String, message: String) = log("INFO", tag, message)
    fun <T> i(tag: String, message: String, provider: () -> T): T {
        i(tag, message)
        return provider()
    }

    fun w(message: String) = log("WARN", DefaultTag, message)
    fun w(tag: String, message: String) = log("WARN", tag, message)
    fun <T> w(tag: String, message: String, provider: () -> T): T {
        w(tag, message)
        return provider()
    }

    fun e(message: String) = log("ERROR", DefaultTag, message)
    fun e(tag: String, message: String) = log("ERROR", tag, message)
    fun <T> e(tag: String, message: String, provider: () -> T): T {
        e(tag, message)
        return provider()
    }

    fun f(message: String) = log("FATAL", DefaultTag, message)
    fun f(tag: String, message: String) = log("FATAL", tag, message)
    fun <T> f(tag: String, message: String, provider: () -> T): T {
        f(tag, message)
        return provider()
    }

    fun a(message: String) = log("ASSERT", DefaultTag, message)
    fun a(tag: String, message: String) = log("ASSERT", tag, message)
    fun <T> a(tag: String, message: String, provider: () -> T): T {
        a(tag, message)
        return provider()
    }

    // Extensions for Mocking Libraries
    @JvmOverloads
    fun nyi(tag: String, method: String, ctx: String? = null) {
        var message = "Method $method is not implemented"
        if (ctx != null) {
            message += " when $ctx"
        }
        message += "."
        w(tag, message)
    }

    @JvmOverloads
    fun nyie(tag: String, method: String, ctx: String? = null) {
        var message = "Method $method is not implemented"
        if (ctx != null) {
            message += " when $ctx"
        }
        message += "."
        e(tag, message)
    }
}