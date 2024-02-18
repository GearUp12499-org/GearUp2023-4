package org.firstinspires.ftc.teamcode.odo

import kotlin.math.PI

object EncoderMath {
    private const val ticksPerRotation = 8192.0
    private const val radiusInches = .69

    @JvmStatic
    fun tick2inch(tick: Int): Double {
        val rotations = tick / ticksPerRotation
        return rotations * 2 * PI * radiusInches
    }
}