package org.firstinspires.ftc.teamcode
/*
 * Servo rotations: minimum 0.0, maximum 1.0
 */
/**
 * All the things that a driver might need to change, in one place.
 */
object Var {
    // Rotations.

    // 'hover' - just above ground
    const val CLAW_HOVER_ROTATION = 0.669

    // 'closing' - touching ground as much as possible without hindering claw operation
    const val CLAW_CLOSING_ROTATION = 0.720

    // 'flipped' - pixel drop-off location
    const val CLAW_FLIPPED_ROTATION = 0.069

    // 'stowed' - straight up or otherwise out of the way
    const val CLAW_STOWED_ROTATION = 0.370

    const val CLAW_OPENED = 0.109
    const val CLAW_CLOSED = 0.432

    /**
     * The rotation of the box in the idle position.
     * The box is in the idle position when it is not being dumped, and also when the lift
     * is all the way down. Tune (DumperTestBench) with lift at 0 to avoid collisions.
     */
    const val BOX_ROTATE_IDLE = 0.531

    /**
     * The rotation of the box in the dumping position.
     * Tune with the lift raised by hand.
     */
    const val BOX_ROTATE_DUMP = 0.316

    /**
     * Latch position such that it does not block movement of the bottom pixel in the box.
     */
    const val BOX_LATCH_UNLATCHED = 0.4

    /**
     * Latch position such that it blocks movement of the bottom pixel in the box.
     */
    const val BOX_LATCH_LATCHED = 0.78
}
