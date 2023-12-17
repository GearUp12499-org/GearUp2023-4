/*
 * Copyright (c) 2015 Robert Atkinson, 2023 PenguinEncounter
 *
 *    Ported from the Swerve library by Craig MacFarlane
 *    Based upon contributions and original idea by dmssargent.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Robert Atkinson, Craig MacFarlane nor the names of their contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package dev.aether.ktftc

import android.content.Context
import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ClassUtil
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.internal.opmode.ClassFilter
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndClass
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.lang.reflect.Modifier
import kotlin.reflect.KClass

class KtAnnotatedOpModeFilter : ClassFilter {
    private val context: Context = AppUtil.getDefContext()
    private var registeredOpModes: RegisteredOpModes? = null
    private val defaultOpModeGroupName = OpModeMeta.DefaultGroup;

    class InstanceHolder {
        companion object {
            val theInstance = KtAnnotatedOpModeFilter()
        }
    }

    internal fun opModePrecheck(clazz: KClass<*>?): Boolean {
        if (clazz == null) {
            reportError("Null Kotlin op mode provided.")
            return false
        }
        if (!clazz.su) {
            reportError("'%s' class isn't an OpMode", clazz.simpleName)
            return false
        }
        if (!Modifier.isPublic(clazz.modifiers)) {
            reportError("'%s' class is not declared 'public' and can't be loaded")
        }
    }

    internal fun reportError(format: String, vararg args: Any?) {
        val message = String.format(format, args)
        Log.w(TAG, String.format("Configuration error: %s", message))
        RobotLog.setGlobalErrorMsg(message)
    }

    inner class OpModeManager : AnnotatedOpModeManager {
        override fun register(opModeClass: Class<*>?) {
            TODO("Not yet implemented")
        }

        override fun register(name: String?, opModeClass: Class<out OpMode>?) {
            TODO("Not yet implemented")
        }

        override fun register(name: OpModeMeta?, opModeClass: Class<out OpMode>?) {
            TODO("Not yet implemented")
        }

        override fun register(name: String?, opModeInstance: OpMode?) {
            TODO("Not yet implemented")
        }

        override fun register(name: OpModeMeta?, opModeInstance: OpMode?) {
            TODO("Not yet implemented")
        }
    }

    companion object {
        const val TAG = "KtOpmodeRegistration"
        fun getInstance() = InstanceHolder.theInstance
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------
    protected fun resolveDuplicateName(opModeMetaAndClass: OpModeMetaAndClass): String? {
        return getOpModeName(opModeMetaAndClass) + "-" + opModeMetaAndClass.clazz.simpleName
    }

    private fun getOpModeName(opModeMetaAndClassData: OpModeMetaAndClass): String? {
        return getOpModeName(opModeMetaAndClassData.clazz)
    }

    /** Returns the name we are to use to identify this class  */
    private fun getOpModeName(clazz: Class<*>): String? {
        var name: String
        name =
            if (this.classNameOverrides.containsKey(clazz)) this.classNameOverrides.get(clazz).meta.name
            else if (clazz.isAnnotationPresent(TeleOp::class.java))
                clazz.getAnnotation<TeleOp>(
                    TeleOp::class.java
                )!!.name
            else if (clazz.isAnnotationPresent(Autonomous::class.java))
                clazz.getAnnotation<Autonomous>(
                    Autonomous::class.java
                )!!.name
            else clazz.simpleName
        if (name.trim { it <= ' ' } == "") name = clazz.simpleName
        return name
    }

    internal fun registerAllClasses(registeredOpModes: RegisteredOpModes) {
        this.registeredOpModes = registeredOpModes
        try {

        }
    }

    private fun callOpModeRegistration(predicate: (KClass<*>) -> Boolean) {
    }
}