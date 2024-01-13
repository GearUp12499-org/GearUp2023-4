package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;

@Autonomous
public class TwentyAutoRedLeft extends TwentyAuto {
    @Override
    protected AdvSphereProcess.Mode modeConf() {
        return AdvSphereProcess.Mode.Red;
    }

    @Override
    protected AllianceColor allianceColor() {
        return AllianceColor.Red;
    }

    @Override
    protected FieldGlobalPosition globalPosition() {
        return FieldGlobalPosition.Frontstage;
    }

    @Override
    protected Parking parkingConf() {
        return Parking.NoParking;
    }
}
