package org.firstinspires.ftc.teamcode.utility

import com.qualcomm.robotcore.hardware.HardwareMap
import java.lang.IllegalArgumentException

inline fun <reified T> HardwareMap.typedGet(name: String): T {
    return this.get(T::class.java, name)
}

inline fun <reified T> HardwareMap.typedMaybeGet(name: String): T? {
    return try {
        this.get(T::class.java, name)
    } catch (e: IllegalArgumentException) {
        null as T
    }
}