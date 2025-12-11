package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.Supplier;

public abstract class AbstractOpMode<OpModeT extends OpMode> extends OpMode {

    /** Subclasses should override if they need to add vision processors to their VisionTask. */
    protected AbstractOpMode<OpModeT> addVisionProcessors() {
        return this;
    }

    // TODO: KIll this. It's too complicated. And OpModes need to know their tasks, so tey are already just creating them.
    //  Sure, we could give the tasks names so the OpMode could look them up after they were built ... but why.
    //  KISS, unless it's demonstrably needed. It isn't.
    @SuppressWarnings("UnusedReturnValue")
    public AbstractOpMode<OpModeT> tasks(Object... value)
            throws BotTaskBuilder.Exception, ClassNotFoundException, IllegalAccessException, InstantiationException
    {
        BotTaskBuilder<AbstractOpMode<OpModeT>> builder = (new BotTaskBuilder<AbstractOpMode<OpModeT>>()).opMode(this);
        // TODO: Always include a KeepReactiveTask that does nothing but run every few millis.
        // This will ensure that loop() does not hang for long when it calls sleepUntil, ensuring that the bot quickly exits when told to stop.
        // Otherwise, the RC App WILL notice, and it will forcibly kill the OpMode and the restart the bot.
        // Devastating if that happens at the end of Auto period. Bot will be dead in the water at the start of TeleOp period.
        tasks = new PriorityQueue<>( builder.addTasks(value).tasks() );

        return this;
    }

    public void loop() {
        // TODO: Perhaps use an ElapsedTime instead of tracking start time.
        BotTask<AbstractOpMode<OpModeT>> task = tasks.poll();
        // task will never be null. But, 1) Make static analysis happy and 2) if it is, it should never have been scheduled, so chuck it.
        if (null == task) return;

        long now = System.currentTimeMillis();
        RobotLog.ii(GORILLA_CORE,
                "%s.loop() start: task:%s now:%d nextRun:%d ",
                getClass().getSimpleName(), task.getClass().getSimpleName(),
                now, task.nextRunMillis()
        );
        if (task.nextRunMillis() < now) {
            long nextRun0 = task.nextRunMillis();
            task.nextRunMillis(now);
            RobotLog.ii(GORILLA_CORE,
                    "%s.loop() task %s under run. now:%d task next was %d reset to %d",
                    getClass().getSimpleName(), task.getClass().getSimpleName(),
                    now, nextRun0, task.nextRunMillis()
            );
        }
        RobotLog.ii(GORILLA_CORE, "%s.loop %15s waiting: delta:%d now:%d until:%d",
                getClass().getSimpleName(), task.getClass().getSimpleName(),
                task.nextRunMillis() - now, now, task.nextRunMillis());
        sleepUntil(GORILLA_CORE, "loop waiting for next run", task.nextRunMillis());
        RobotLog.ii(GORILLA_CORE, "%s.loop %15s at %d: running task at %d",
                getClass().getSimpleName(), task.getClass().getSimpleName(), now, task.nextRunMillis());
        task.run();
        task.nextRunMillis(task.nextRunMillis() + task.frequencyMillis());
        tasks.add(task);
        RobotLog.ii(GORILLA_CORE, "%s.loop %15s at %d: done nextRun:%d frequency:%d",
                getClass().getSimpleName(), task.getClass().getSimpleName(),
                now, task.nextRunMillis(), task.frequencyMillis());
    }

    /** Configure telemetry. Configure capacity, order, format, etc. */
    @SuppressWarnings("UnusedReturnValue")
    protected AbstractOpMode<OpModeT> configureTelemetry() {
        return this;
    }

    protected abstract BotTask<OpModeT>[] getTasks();

//    public abstract AbstractOpMode<OpModeT> postOpModeInit();

    @Override
    public void init() {
        telemetry.addData("status", "AbstractOpMode.init(): start");

        for (String name: hardwareMap.getAllNames(HardwareDevice.class)) {
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode.init(): have device %s", name);
        }
        telemetry.update();

        configureTelemetry();
        try {
            tasks((Object[]) getTasks());
        } catch (Exception ex) {
            throw new RuntimeException("AbstractOpMode.init(): Exception while creating tasks", ex);
        }
        telemetry.addData("status", "TeleOpDrive.init(): tasks prepared");
        telemetry.update();

        RobotLog.ii(GORILLA_CORE, "AbstractOpMode.init(): tasks initializing");
        tasks.forEach(task -> {
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode.init(): %s initializing", task.getClass().getSimpleName());
            task.init();
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode.init(): %s initialized", task.getClass().getSimpleName());
        });
        RobotLog.ii(GORILLA_CORE, "AbstractOpMode.init(): tasks initialized");

        telemetry.addData("status", "AbstractOpMode.init(): done");
        telemetry.update();
    }

    @Override
    public void start() {
        tasks.forEach(task -> {
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode start: %s starting", task.getClass().getSimpleName());
            task.start();
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode start: %s started", task.getClass().getSimpleName());
        });
    }

    @Override
    public void stop() {
        tasks.forEach(task -> {
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode stop: %s starting", task.getClass().getSimpleName());
            task.stop();
            RobotLog.ii(GORILLA_CORE, "AbstractOpMode stop: %s started", task.getClass().getSimpleName());
        });
    }

    public static abstract class AbstractOpModeTask<OpModeT extends OpMode> extends AbstractBotTask<AbstractOpMode<OpModeT>> {
    }

    // TODO: Add tasks for IsAuto/isTeleOp, IMU Orientation, Voltage, Motor Bulk Read, BlobFinder
    // IsAuto: use IMU and April tags to figure out if we are Red/Blue alliance, where we started on the field,
    //         movement delay, and any other auto parameters.
    // IMU: ConceptExploringIMUOrientation, SensorIMUOrthogonal, SensorIMUNonOrthogonal
    // Voltage: track bot battery voltage. ConceptTelemetry.
    // Motor Bulk Read: read encoders in bulk "once per cycle", aka at some frequency, ideally just before Drive task.
    //                  should perhaps just be part of the drive task(s).
    // BlobFinder: ConceptVisionColorLocator_Circle - find colored blobs on the field. Great for finding balls and so auto drive tasks can slurp them up.

    // TODO: Add a DriveMecanumTask
    // TODO: Add a DriveAutoToTargetTask. Probably one per type of drive train. Normal wheels, Omni/Holonomic, Mecanum.

    public AbstractOpMode<OpModeT> sleepUntil(
            String tag, String label, long nextRunTime, Supplier<Boolean> canInterrupt
    )
            throws InterruptedException
    {
        long now = System.currentTimeMillis();
//        RobotLog.ii(tag, "sleepUntil start: called by %s to sleep %dms from now %d until %d",
//                label, nextRunTime-now, now, nextRunTime
//        );
        for (long durationUntileNextRun = nextRunTime - now; 0 < durationUntileNextRun; durationUntileNextRun = nextRunTime - now) {
//            RobotLog.ii(tag, "sleepUntil waiting for %s: sleepDuration = %d", label, durationUntileNextRun);

            try {
                Thread.sleep(durationUntileNextRun);
            } catch (InterruptedException ex) {
                if (canInterrupt.get()) {
                    RobotLog.ee(tag, ex, "%s: wait interrupted, interrupt exceptions okay, rethrowing", label);
                    throw ex;
                } else {
                    RobotLog.ee(tag, ex, "%s: wait interrupted, interrupt exceptions deferred, setting interrupted flag", label);
                    Thread.currentThread().interrupt();
                }
            } catch (Throwable ex) {
                RobotLog.ee(tag, ex, "%s: unexpected exception", label);
            }
//            RobotLog.ii(tag, "sleepUntil waited for %s: sleepDuration = %d", label, durationUntileNextRun);
            now = System.currentTimeMillis();
        }
//        RobotLog.ii(
//                tag, "sleepUntil done: called by %s for %d now %d",
//                label, nextRunTime, now);

        return this;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AbstractOpMode<OpModeT> sleepUntil(String tag, String label, long nextRunTime) {
        try {
            return sleepUntil(tag, label, nextRunTime, () -> false);
        } catch (InterruptedException ex) {
            throw new RuntimeException("'Impossible' Interrupt Exception", ex);
        }
    }

    PriorityQueue< BotTask<AbstractOpMode<OpModeT>> > tasks;

    public static final String GORILLA_CORE = "GorillaCore";
} // class AbstractOpMode<OpModeT>
