package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;

@Autonomous
public class TwentyAutoBlueLeft extends TwentyAuto {
    @Override
    protected AdvSphereProcess.Mode modeConf() {
        return AdvSphereProcess.Mode.Blue;
    }

    @Override
    protected AllianceColor allianceColor() {
        return AllianceColor.Blue;
    }

    @Override
    protected FieldGlobalPosition globalPosition() {
        return FieldGlobalPosition.Backstage;
    }

    @Override
    protected Parking parkingConf() {
        return Parking.MoveLeft;
    }
}
