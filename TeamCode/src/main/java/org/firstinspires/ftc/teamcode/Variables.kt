package org.firstinspires.ftc.teamcode

/*
 * Servo rotations: minimum 0.0, maximum 1.0
 */

/**
 * Claw constants.
 */
object ClawVar {
    // Rotations.

    // 'hover' - just above ground
    const val HoverRotation = 0.669
    // 'closing' - touching ground as much as possible without hindering claw operation
    const val ClosingRotation = 0.720
    // 'flipped' - pixel drop-off location
    const val FlippedRotation = 0.069
    // 'stowed' - straight up or otherwise out of the way
    const val StowedRotation = 0.370

    const val ClawOpened = 0.109
    const val ClawClosed = 0.432
}
