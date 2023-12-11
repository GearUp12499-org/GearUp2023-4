package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;

@Autonomous
public class TwentyAutoBlue extends TwentyAuto {
    @Override
    protected AdvSphereProcess.Mode modeConf() {
        return AdvSphereProcess.Mode.Blue;
    }
}
