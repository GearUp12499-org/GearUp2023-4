package org.firstinspires.ftc.tests

import kotlin.math.abs

fun decimalEquals(a: Float, b: Float, epsilon: Float = 0.0001f): Boolean {
    return abs(a - b) < epsilon
}
fun decimalEquals(a: Double, b: Double, epsilon: Double = 0.0001): Boolean {
    return abs(a - b) < epsilon
}
