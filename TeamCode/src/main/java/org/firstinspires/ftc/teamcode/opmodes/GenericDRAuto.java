package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.abstractions.Claw;
import org.firstinspires.ftc.teamcode.configurations.RobotConfiguration;

import dev.aether.collaborative_multitasking.MultitaskScheduler;

@Autonomous
public class GenericDRAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfiguration robot = RobotConfiguration.currentConfiguration().invoke(hardwareMap);
        MultitaskScheduler scheduler = new MultitaskScheduler();
        Claw claw = new Claw(scheduler, robot.clawGrab(), robot.clawRotate(), robot.getClawLock());

        GenericDRAutoB backing = new GenericDRAutoB(robot, scheduler, claw);

        while (opModeIsActive()) {
            scheduler.tick();
        }
    }
}
