package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.PriorityQueue;
import java.util.function.Supplier;

public abstract class AbstractOpMode<OpModeT extends OpMode> extends OpMode {

    /** Subclasses should override if they need to add vision processors to their VisionTask. */
    @SuppressWarnings("UnusedReturnValue")
    protected AbstractOpMode<OpModeT> configureAprilTagProcessors() {
        return this;
    }

    /** Subclasses should override if they need to add vision processors to their VisionTask. */
    @SuppressWarnings("UnusedReturnValue")
    protected AbstractOpMode<OpModeT> addVisionProcessors() {
        return this;
    }

    /** Subclasses should set inversionFactor per the field location of bots at start.
     *   1 if red bots start on the red side of the field. Nearer the red wall.
     *  -1 if red bots start on the blue side of the field. Nearer the blue wall.
     */
    protected double inversionFactor = -1;

    /** Which alliance's side of the field is the position on?
     *
     *  THIS WORKS ONLY FOR "SQUARE" FIELDS, where red and blue alliances face each other
     *  on parallel walls of the field.
     *
     *  OpMode must override this if the field is a "diamond" field, where red and blue alliances
     *  are on adjacent walls.
     *
     * @param position
     * @return Alliance for the field position.
     */
    protected Alliance allianceOwningFieldPosition(Position position) {
        double ycm = inversionFactor * CM.fromUnit(position.unit, position.y);
        if ( ycm < -20 ) {
            // Near the red wall, so a blue bot
            return Alliance.Blue;
        } else if ( 20 < ycm ) {
            return Alliance.Red;
        } else {
            return Alliance.Unknown;
        }
    }

    /** Set our alliance based on the first AprilTag detection seen.
     *
     *  It is critical that the robot can see an AprilTag before the start of the match.
     *  Otherwise, it may move to the other side of the field before it sees an AprilTag,
     *  and will therefore set its alliance incorrectly.
     *
     * @param detection
     * @return
     */
    @SuppressWarnings("UnusedReturnValue")
    protected AbstractOpMode<OpModeT> setAllianceFromDetection(AprilTagDetection detection) {
        // We already know our alliance
        if (Alliance.Unknown != alliance) return this;

        // Don't set our alliance if we aren't in Autonomous period. Who knows where we are on the field ...
        if (null == getClass().getAnnotation(Autonomous.class)) return this;

        if (null == detection) throw new NullPointerException("detection is null");

        alliance = allianceOwningFieldPosition(detection.robotPose.getPosition());
        blackboard.put("Alliance", alliance.toString());

        return this;
    }
    @SuppressWarnings("UnusedReturnValue")
    protected AbstractOpMode<OpModeT> onFreshDetections(String label, List<AprilTagDetection> blobs) {
        blobs.forEach(blob -> {
            setAllianceFromDetection(blob);
        });
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

    @Override
    public void init_loop() {
        tasks.forEach(BotTask::waitForStart);
    }

        /** Configure telemetry. Configure capacity, order, format, etc. */
    @SuppressWarnings("UnusedReturnValue")
    protected AbstractOpMode<OpModeT> configureTelemetry() {
        return this;
    }

    protected abstract BotTask<OpModeT>[] getTasks();

    @Override
    public void init() {
        configureTelemetry();

        RobotLog.ii(GORILLA_CORE, "%s.init(): start", getClass().getSimpleName());

        for (String name: hardwareMap.getAllNames(HardwareDevice.class)) {
            RobotLog.ii(GORILLA_CORE, "%s.init(): have device %s", getClass().getSimpleName(), name);
        }

        try {
            tasks((Object[]) getTasks());
        } catch (Exception ex) {
            throw new RuntimeException(String.format("%s.init(): Exception while creating tasks", getClass().getSimpleName()), ex);
        }
        RobotLog.ii(GORILLA_CORE, "%s.init(): tasks prepared", getClass().getSimpleName());

        RobotLog.ii(GORILLA_CORE, "%s.init(): tasks initializing", getClass().getSimpleName());
        tasks.forEach(task -> {
            RobotLog.ii(GORILLA_CORE, "%s.init(): %s initializing", getClass().getSimpleName(), task.getClass().getSimpleName());
            task.init();
            RobotLog.ii(GORILLA_CORE, "%s.init(): %s initialized", getClass().getSimpleName(), task.getClass().getSimpleName());
        });
        RobotLog.ii(GORILLA_CORE, "%s.init(): tasks initialized", getClass().getSimpleName());

        RobotLog.ii(GORILLA_CORE, "%s.init(): done", getClass().getSimpleName());
    }

    @Override
    public void resetRuntime() {
        super.resetRuntime();
        runtime.reset();
    }

    @Override
    public void start() {
        super.start();
        resetRuntime();
        tasks.forEach(task -> {
            RobotLog.ii(GORILLA_CORE, "%s start: %s starting", getClass().getSimpleName(), task.getClass().getSimpleName());
            task.start();
            RobotLog.ii(GORILLA_CORE, "%s start: %s started", getClass().getSimpleName(), task.getClass().getSimpleName());
        });
    }

    @Override
    public void stop() {
        tasks.forEach(task -> {
            RobotLog.ii(GORILLA_CORE, "%s stop: %s starting", getClass().getSimpleName(), task.getClass().getSimpleName());
            task.stop();
            RobotLog.ii(GORILLA_CORE, "%s stop: %s started", getClass().getSimpleName(), task.getClass().getSimpleName());
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

    protected static enum Alliance {
        Unknown,
        Red,
        Blue
    }

    protected PriorityQueue< BotTask<AbstractOpMode<OpModeT>> > tasks;

    protected Alliance alliance = Alliance.Unknown;
    {
        // Get the alliance from the blackboard. If the blackboard does not contain Alliance, then set to Unknown
        // Do not store the alliance value itself! Store the string value and convert when reading it in.
        // Why? Because Alliance values compiled for different OpModes may be different. The code may have been
        // recompiled, different class loaders, who knows. And then the cast fails.
        Object allianceFromBlackboard = blackboard.putIfAbsent("Alliance", Alliance.Unknown.toString());
        if (null != allianceFromBlackboard) {
            try {
                this.alliance = Alliance.valueOf((String)allianceFromBlackboard);
            } catch (IllegalArgumentException ex) {
                // Value does not match an Alliance. Set it to unknown.
                RobotLog.ee(GORILLA_CORE, ex, "'%s' is not an Alliance", allianceFromBlackboard);
                alliance = Alliance.Unknown;
            } catch (Exception ex) {
                // Value does not match an Alliance. Set it to unknown.
                RobotLog.ee(GORILLA_CORE, ex, "'%s' is is a %s and not a java.lang.String", allianceFromBlackboard, allianceFromBlackboard.getClass().getCanonicalName());
                alliance = Alliance.Unknown;
            }
        }
    }

    protected final ElapsedTime runtime = new ElapsedTime();

    public static final String GORILLA_CORE = "GorillaCore";
} // class AbstractOpMode<OpModeT>
