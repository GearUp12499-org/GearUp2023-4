package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.AdvSphereProcess;

@Autonomous
public class TwentyAutoRedRight extends TwentyAuto {
    @Override
    protected AdvSphereProcess.Mode modeConf() {
        return AdvSphereProcess.Mode.Red;
    }

    @Override
    protected StartPosition positionConf() {
        return StartPosition.RightOfTruss;
    }

    @Override
    protected Parking parkingConf() {
        return Parking.NoParking; // TODO: This should be MoveRight, awaiting testing.
    }
}
