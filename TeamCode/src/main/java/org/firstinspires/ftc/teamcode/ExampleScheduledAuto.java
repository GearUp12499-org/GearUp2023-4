package org.firstinspires.ftc.teamcode;

import static dev.aether.collaborative_multitasking.KotlinHelper.kbu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.MultitaskScheduler;

@Autonomous
public class ExampleScheduledAuto extends LinearOpMode {
    MultitaskScheduler scheduler;
    public static final SharedResource EXAMPLE_LOCK = new SharedResource("example");

    @Override
    public void runOpMode() {
        scheduler = new MultitaskScheduler();
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        scheduler.task(kbu(that -> {
            that.require(EXAMPLE_LOCK);
            that.isCompleted((that2, scheduler) -> {
                if (that2.getStartedAt() != null) {
                    return timer.time(TimeUnit.SECONDS) - that2.getStartedAt() > 3;
                } else {
                    return false;
                }
            });
        }));
        scheduler.task(kbu(that -> {
            that.require(EXAMPLE_LOCK);
            that.isCompleted((that2, scheduler) -> {
                if (that2.getStartedAt() != null) {
                    return timer.time(TimeUnit.SECONDS) - that2.getStartedAt() > 3;
                } else {
                    return false;
                }
            });
        }));
        scheduler.runToCompletion();
        //noinspection StatementWithEmptyBody
        while (!isStopRequested()) {
            // idk
        }
    }
}
