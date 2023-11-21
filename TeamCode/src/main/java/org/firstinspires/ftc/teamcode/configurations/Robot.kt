package org.firstinspires.ftc.teamcode.configurations

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.aether.collaborative_multitasking.SharedResource
import org.firstinspires.ftc.teamcode.utilities.MotorSet
import org.firstinspires.ftc.teamcode.utilities.typedGet
import org.firstinspires.ftc.teamcode.utilities.typedMaybeGet

class Robot(map: HardwareMap) : RobotConfiguration() {
    @JvmField
    var frontLeft // 0
            : DcMotor = map.typedGet("frontLeft")

    @JvmField
    var frontRight // 1
            : DcMotor = map.typedGet("frontRight")

    @JvmField
    var backLeft // 2
            : DcMotor = map.typedGet("backLeft")

    @JvmField
    var backRight // 3
            : DcMotor = map.typedGet("backRight")

    @JvmField
    var liftLeftB // 0
            : DcMotor = map.typedGet("slideLeft")

    @JvmField
    var liftRightB // 1
            : DcMotor = map.typedGet("slideRight")

    @JvmField
    var clawGrabB // 0
            : Servo? = map.typedMaybeGet("clawGrab")

    @JvmField
    var clawRotateB // 1
            : Servo? = map.typedMaybeGet("clawRotate")

    @JvmField
    var dumperRotateB // 2
            : Servo? = map.typedMaybeGet("dumperRotate")
    @JvmField
    var dumperLatchB // 3
            : Servo? = map.typedMaybeGet("dumperLatch")

    @JvmField
    var droneB // 2
            : DcMotor? = map.typedMaybeGet("drone")


    override val driveMotors: MotorSet = MotorSet(frontLeft, frontRight, backLeft, backRight)
    override val driveMotorLock: SharedResource
        get() = SharedResource("driveMotors") {
            driveMotors.setAll(
                0.0
            )
        }
    override val clawGrab: Servo? get() = clawGrabB
    override val clawRotate: Servo? get() = clawRotateB
    override val clawLock: SharedResource = SharedResource("claw")
    override val liftLeft: DcMotor get() = liftLeftB
    override val liftRight: DcMotor get() = liftRightB
    override val liftLock: SharedResource = SharedResource("lift") {
        liftLeft.power = 0.0
        liftRight.power = 0.0
    }
    override val dumperRotate: Servo? = dumperRotateB
    override val dumperLatch: Servo? = dumperLatchB
    override val dumperLock: SharedResource = SharedResource("dumper")
    override val drone: DcMotor? = droneB
    override val droneLock: SharedResource = SharedResource("drone")

    override val distanceLeft: DistanceSensor? = map.typedMaybeGet("distanceLeft")
    override val distanceRight: DistanceSensor? = map.typedMaybeGet("distanceRight")

    init {
        setReverse(frontLeft)
        setReverse(backLeft)
        enableBrakes(frontLeft, frontRight, backLeft, backRight)

        liftLeftB.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftLeftB.targetPosition = 0
        liftLeftB.power = 1.0
        liftLeftB.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftRightB.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftRightB.targetPosition = 0
        liftRightB.power = 1.0
        liftRightB.mode = DcMotor.RunMode.RUN_TO_POSITION
        setReverse(liftRightB)
    }

    companion object {
        private fun setReverse(target: DcMotor) {
            target.direction = DcMotorSimple.Direction.REVERSE
        }

        private fun enableBrakes(vararg targets: DcMotor) {
            for (target in targets) target.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}