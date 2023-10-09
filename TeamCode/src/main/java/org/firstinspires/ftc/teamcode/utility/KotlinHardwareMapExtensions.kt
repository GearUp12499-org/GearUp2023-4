package org.firstinspires.ftc.teamcode.utility

import com.qualcomm.robotcore.hardware.HardwareMap

inline fun <reified T> HardwareMap.typedGet(name: String): T {
    return this.get(T::class.java, name)
}