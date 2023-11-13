package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.Var.ApproachObject
import org.firstinspires.ftc.teamcode.Var.Box
import org.firstinspires.ftc.teamcode.Var.Claw
import org.firstinspires.ftc.teamcode.utilities.inches

/*
 * Servo rotations: minimum 0.0, maximum 1.0
 */
/**
 * All the things that a driver might need to change, in one place.
 * You can Ctrl-Click these:
 * - [Claw] - claw positions (idle, stowed, flipped, closing) & opened, closed
 * - [Box] - box positions (idle, dump) & latched, unlatched
 * - [ApproachObject] - options for the object approach algorithm, aka the 'x' button
 */
object Var {
    object Claw {
        const val hoverRotate = 0.669

        // 'closing' - touching ground as much as possible without hindering claw operation
        const val closingRotate = 0.720

        // 'flipped' - pixel drop-off location
        const val flippedRotate = 0.069

        // 'stowed' - straight up or otherwise out of the way
        const val stowedRotate = 0.370

        const val opened = 0.109
        const val closed = 0.432
    }

    object Box {
        /**
         * The rotation of the box in the idle position.
         * The box is in the idle position when it is not being dumped, and also when the lift
         * is all the way down. Tune (DumperTestBench) with lift at 0 to avoid collisions.
         */
        const val idleRotate = 0.531

        /**
         * The rotation of the box in the dumping position.
         * Tune with the lift raised by hand.
         */
        const val dumpRotate = 0.316

        /**
         * Latch position such that it does not block movement of the bottom pixel in the box.
         */
        const val unlatched = 0.4

        /**
         * Latch position such that it blocks movement of the bottom pixel in the box.
         */
        const val latched = 0.78
    }

    /**
     * Target distance for approaching objects with the 'x' button on Gamepad 1.
     */
    @JvmField
    val TELEOP_APPROACH_DISTANCE = 3.inches

    /**
     * Speed when using the left bumper on Gamepad 1 to decrease the driving speed.
     */
    const val TELEOP_THROTTLE_SPEED = 0.5

    object ApproachObject {
        /**
         * Minimum speed when approaching object, for ramping down when the distance read within
         * [stoppingDistance] of the target distance.
         */
        const val minSpeed = 0.1

        /**
         * Maximum speed when approaching object, for ramping up when the distance read is
         * more than [stoppingDistance] away from the target distance.
         */
        const val maxSpeed = 0.3

        /**
         * Distance at which to start ramping down the speed when approaching an object.
         * @see minSpeed
         * @see maxSpeed
         */
        @JvmField
        val stoppingDistance = 6.inches
    }
}
