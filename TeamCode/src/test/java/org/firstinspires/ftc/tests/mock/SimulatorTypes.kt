package org.firstinspires.ftc.tests.mock

import org.firstinspires.ftc.tests.MinimalLogger

typealias ProvideSubscription<Src, Data> = (src: Src, data: Data) -> Unit
typealias ProvideList<Src, Data> = MutableList<ProvideSubscription<Src, Data>>
typealias NotifySubscription<Src> = (src: Src) -> Unit
typealias NotifyList<Src> = MutableList<NotifySubscription<Src>>

fun Simulated.simulate(
    totalDuration: Double,
    step: Double = 0.05,
    continueCondition: () -> Boolean = { true },
) {
    var timeAdvanced = 0.0
    val realStart = System.currentTimeMillis()
    while (timeAdvanced < totalDuration && continueCondition()) {
        advance(step)
        timeAdvanced += step
    }
    val realEnd = System.currentTimeMillis()
    val seconds = (realEnd - realStart) / 1000.0
    MinimalLogger.i(
        "Simulator",
        String.format(
            "simulated %.2f seconds in %.2f seconds (%.2f vsec/sec)",
            timeAdvanced,
            seconds,
            timeAdvanced / seconds
        )
    )
}
