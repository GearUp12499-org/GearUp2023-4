package org.firstinspires.ftc.teamcode.configurations

import dev.aether.collaborative_multitasking.SharedResource

object RobotLocks {
    @Deprecated("Use a RobotConfiguration's locks for panic support.")
    val driveMotors = SharedResource("driveMotors")
}