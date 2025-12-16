package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Comparator;

public interface BotTask<OpModeT extends OpMode> extends Comparable<BotTask<OpModeT>> {

    BotTask<OpModeT> waitForStart();

    BotTask<OpModeT>  run();

    BotTask<OpModeT>  init();

    BotTask<OpModeT>  start();

    BotTask<OpModeT>  stop();

    long nextRunMillis();

    BotTask<OpModeT>  nextRunMillis(long value);

    long lastRunDurationMillis();

    BotTask<OpModeT> lastRunDurationMillis(long value);

    BotTask<OpModeT> frequencyMillis(long millis);

    long frequencyMillis();

    BotTask<OpModeT> opMode(OpModeT opMode);
    OpModeT opMode();

    HardwareMap hardwareMap();

    Telemetry telemetry();

    public static class NaturalComparator implements Comparator<BotTask> {
        @Override
        public int compare(BotTask task1, BotTask task2) {
            if (task1.nextRunMillis() < task2.nextRunMillis()) return -1;
            if (task1.nextRunMillis() > task2.nextRunMillis()) return  1;
            return 0;
        }
    }
} // class BotTask<OpModeT>
