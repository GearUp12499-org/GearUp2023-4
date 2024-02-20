package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.Var.ApproachObject
import org.firstinspires.ftc.teamcode.Var.Box
import org.firstinspires.ftc.teamcode.Var.TeleOp
import org.firstinspires.ftc.teamcode.utilities.Pose
import org.firstinspires.ftc.teamcode.utilities.cm
import org.firstinspires.ftc.teamcode.utilities.degrees
import org.firstinspires.ftc.teamcode.utilities.feet
import org.firstinspires.ftc.teamcode.utilities.inches

/*
 * Servo rotations: minimum 0.0, maximum 1.0
 */
/**
 * All the things that a driver might need to change, in one place.
 * You can Ctrl-Click these to jump to them:
 * - [Box] - box positions (idle, dump) & latched, unlatched
 * - [TeleOp] - control configuration: how close to get with 'x', lift speed, etc
 * - [ApproachObject] - options for the object approach algorithm, aka the 'x' button
 */
object Var {
    object Box {
        /**
         * The rotation of the box in the idle position.
         * The box is in the idle position when it is not being dumped, and also when the lift
         * is all the way down. Tune (DumperTestBench) with lift at 0 to avoid collisions.
         */
        const val idleRotate = 0.6069

        /**
         * The rotation of the box in the dumping position.
         * Tune with the lift raised by hand.
         */
        const val dumpRotate = 0.367

        /**
         * Latch position such that it lets go of the top pixel but not the bottom pixel.
         */
        const val latch1 = 0.209

        /**
         * Latch position such that it does not block movement of the bottom pixel in the box.
         */
        const val unlatched = 0.35

        /**
         * Latch position such that it blocks movement of the bottom pixel in the box.
         */
        const val latched = 0.029

        const val RotateTime = 1000 // ms
        const val LatchTime = 200 // ms
    }

    object PixelDropper {
        const val down = 0.177
        const val up = 0.67
        const val back = 1.0
    }

    object DropDownServo {
        @JvmField
        val positions = doubleArrayOf(
            0.60,
            0.4744,
//            0.4538,
//            0.3999,
//            0.3760,
            0.3580
        )
        @JvmField
        val up = positions[0]
    }

    object TeleOp {
        /**
         * Target distance for approaching objects with the 'x' button on Gamepad 1.
         */
        @JvmField
        val approachDistance = 2.5.inches

        /**
         * Cancel approaching object task after this duration (milliseconds)
         */
        const val cancelApproachDuration = 3000

        /**
         * Minimum amount to push the throttle trigger in order to activate the slowing effect
         */
        const val throttleMinThrow = 0.5

        /**
         * Minimum speed when using the left trigger on Gamepad 1 to decrease the driving speed.
         */
        const val throttle = 0.2

        /**
         * Speed when using the left bumper on Gamepad 1 to decrease the driving speed.
         */
        const val binThrottle = 0.5

        /**
         * Scoring (long) lift maximum, encoder counts
         */
        const val longSlideLimit = 3200

        /**
         * Hanging (short) lift maximum, encoder counts
         */
        const val shortSlideLimit = 2100

        /**
         * Lift speed, encoder counts per second
         */
        const val liftSpeed = 2000

        /**
         * Lift speed (driver 1 / hanging lift), encoder counts per second
         */
        const val climbingLiftSpeed = 750

        /**
         * Lift preset height for scoring, encoder ticks
         */
        const val liftScoringReadyPreset = 450

        /**
         * Lift preset height for hanging, encoder ticks
         */
        const val liftHangingPreset = 1910

        /**
         * Left-side motor fudging
         */
        const val balanceLeft = 1.0

        /**
         * Right-side motor fudging
         */
        const val balanceRight = 0.95

        /**
         * Front motor fudging
         */
        const val balanceFront = 1.0

        /**
         * Back motor fudging
         */
        const val balanceBack = 1.0

        /**
         * How much (%) do you have to push the trigger to activate it?
         */
        const val triggerPct = 0.25

        /**
         * How hard we spin the intake?
         */
        const val intakePower = 0.6
    }

    object ApproachObject {
        /**
         * Minimum speed when approaching object, for ramping down when the distance read within
         * [stoppingDistance] of the target distance.
         */
        const val minSpeed = 0.05

        /**
         * Maximum speed when approaching object, for ramping up when the distance read is
         * more than [stoppingDistance] away from the target distance.
         */
        const val maxSpeed = 0.15

        /**
         * Distance at which to start ramping down the speed when approaching an object.
         * @see minSpeed
         * @see maxSpeed
         */
        @JvmField
        val stoppingDistance = 2.feet

        /**
         * If either sensor reads more than this distance, abort!
         */
        @JvmField
        val panicDistance = 24.inches
    }

    object Pose {
        const val straightBias = 1.0
        const val strafeBias = 1.1
        const val turnBias = 1.0
    }

    object AutoPositions {
        @JvmField
        val RobotWidth = 41.cm
        private val RobotBackToOdo = 21.cm

        @JvmField
        val BlueLeft = Pose(RobotBackToOdo, 94.inches - RobotWidth / 2.0, 180.degrees)

        @JvmField
        val BlueLeftRightPixel = Pose(24.5.inches + RobotWidth / 2.0, 83.5.inches, (90).degrees)

        // Lift scoring position: 1280, 1100
        const val LiftScoring = 1280
    }
}
