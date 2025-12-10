package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class AbstractBotTask<OpModeT extends OpMode> implements BotTask<OpModeT> {
    @Override
    public BotTask<OpModeT>  run() {
        return this;
    }

    @Override
    public BotTask<OpModeT>  init() {
        return this;
    }

    public BotTask<OpModeT> start() {
        return this;
    }

    public BotTask<OpModeT> stop() {
        return this;
    }

    public int compareTo(BotTask<OpModeT> other) {
        return Long.compare(this.nextRunMillis, other.nextRunMillis());
        // TODO: Add task dependencies. Dependent tasks should run after their dependencies
        // when they are scheduled for the same time. For example, if DriveTasks are dependent on GamepadTasks
        // (because the GamepadTask supplies the inputs used by the DriveTask, then driveTask.dependsOn(gamepadTask) should return true.
    }

    @Override
    public BotTask<OpModeT> opMode(OpModeT value) {
        opMode    = value;
        telemetry = opMode.telemetry;

        return this;
    }
    @Override
    public OpModeT opMode() {
        return opMode;
    }

    @Override
    public HardwareMap hardwareMap() {
        return opMode.hardwareMap;
    }

    @Override
    public Telemetry telemetry() {
        return opMode.telemetry;
    }

    @Override
    public BotTask<OpModeT> frequencyMillis(long millis) {
        frequencyMillis = millis;
        return this;
    }

    @Override
    public long frequencyMillis() {
        return frequencyMillis;
    }

    @Override
    public long nextRunMillis() {
        return nextRunMillis;
    }

    @Override
    public BotTask<OpModeT>  nextRunMillis(long value) {
        nextRunMillis = value;
        return this;
    }

    @Override
    public long lastRunDurationMillis() {
        return lastRunDurationMillis;
    }

    @Override
    public BotTask<OpModeT> lastRunDurationMillis(long value) {
        lastRunDurationMillis = value;

        return this;
    }

    protected final ElapsedTime runtime = new ElapsedTime();

    protected OpModeT opMode;

    protected Telemetry telemetry;

    private long frequencyMillis;

    private long nextRunMillis;

    private long lastRunDurationMillis;

} // class AbstractBotTask<OpModeT>
