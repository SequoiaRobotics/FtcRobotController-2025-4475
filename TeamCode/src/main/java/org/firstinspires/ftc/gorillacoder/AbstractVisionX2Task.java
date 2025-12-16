package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.vision.VisionPortal.StreamFormat.MJPEG;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;

public abstract class AbstractVisionX2Task<OpModeT extends OpMode> extends AbstractOpMode.AbstractOpModeTask<OpModeT> {

    protected abstract AbstractVisionRunner createVisionRunner();

    /** Initialize:
     *  * How often should the task run? Probably not often, if even ever. The work is done by the runner.
     *  * The VisionRunner.
     */
    @Override
    public abstract AbstractVisionX2Task<OpModeT> init();

    @Override
    public AbstractVisionX2Task<OpModeT> start() {
        return this;
    }

    public AbstractVisionX2Task<OpModeT> stop() {
        visionRunner.stop();

        return this;
    }

    /** VisionTasks with a single portal that need to add VisionProcessors should override.
     *  Note that VisionTasks automatically add an AprilTagProcessor to each portal.
     */
    @SuppressWarnings("UnusedReturnValue")
    public AbstractVisionX2Task<OpModeT> addProcessor(VisionProcessor processor) {
        return this;
    }

    /** VisionTasks with separate right and left portals and processors should override. */
    @SuppressWarnings("UnusedReturnValue")
    public AbstractVisionX2Task<OpModeT> addProcessorLeft(VisionProcessor processor) {
        return this;
    }

    /** VisionTasks with separate right and left portals and processors should override. */
    @SuppressWarnings("UnusedReturnValue")
    public AbstractVisionX2Task<OpModeT> addProcessorRight(VisionProcessor processor) {
        return this;
    }

    //  TODO: Move to AprilTagProcessorBuilder below.
    protected AprilTagProcessor.Builder createAprilTagProcessorBuilder() {
        return new AprilTagProcessor.Builder()
                // .setTagLibrary(tagLibrary)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                // TODO: set where the camera is on the bot
                //    private Position cameraPosition = new Position(DistanceUnit.INCH,
                //            0, 0, 0, 0);
                //    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                //            0, -90, 0, 0);
                // .setCameraPose(cameraPosition, cameraOrientation)
                ;
    }

    //  TODO: Move to AprilTagProcessorBuilder below.
    protected VisionPortal.Builder createVisionPortalBuilder() {
        return new VisionPortal.Builder()
                .setStreamFormat(MJPEG)
                // Each resolution, for each camera model, needs calibration values for good pose estimation.
                // .setCameraResolution(new Size(640, 480))
                .setAutoStartStreamOnBuild(true)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .setShowStatsOverlay(true);
    }

    protected VisionPortal createVisionPortal(CameraName camera, AprilTagProcessor processor) {
        // Consider: adding additional AprilTag library(ies)
        // AprilTagLibrary tagLibrary = ...
        // TODO: Add support for calibration. Needed for non/semi supported cameras like the arducam.
        //       The Logitech C920 is well supported with built in calibrations for:
        //       320x240, 352x288, 432x240, 640x360, 640x480, 800x448, 800x600, 864x480, 960x720,
        //       1024x576, 1280x720, 1600x896, 1920x1080, 2304x1296, 2304x1536
        return createVisionPortalBuilder()
                .setCamera(camera)
                .addProcessors(processor)
                .build();
    }

    @SuppressWarnings("UnusedReturnValue")
    protected AbstractVisionX2Task<OpModeT> waitForPortalState(VisionPortal portal, VisionPortal.CameraState stateToWaitFor, long maxWaitCount) {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionX2Task.waitForPortalState(portal, %d/%s, %d) check if cameras ready: state=%d/%s",
                stateToWaitFor.ordinal(), stateToWaitFor, maxWaitCount, portal.getCameraState().ordinal(), portal.getCameraState()
        );

        // TODO: write waitFor( condition:lambda, checkDelay, limit, tag, label);
        long waitTime0 = System.currentTimeMillis();
        long waitCount = 0;
        for (VisionPortal.CameraState state = portal.getCameraState();
             state.ordinal() < stateToWaitFor.ordinal();
             state = portal.getCameraState()
        ) {
            waitCount++;
            if (maxWaitCount < waitCount) {
                String message = String.format(Locale.US, "AbstractVisionX2Task.waitForPortalState(portal, %d/%s, %d) abort portal is not ready: state=%d/%s",
                        stateToWaitFor.ordinal(), stateToWaitFor, maxWaitCount, state.ordinal(), state
                );
                throw new RuntimeException(message);
            }
            String message = String.format(Locale.US, "AbstractVisionX2Task.waitForPortalState(portal, %d/%s, %d) waiting for portal to be ready, calling sleepUntil until next iteration: waitCount=%d state=%d/%s",
                    stateToWaitFor.ordinal(), stateToWaitFor, maxWaitCount, waitCount, state.ordinal(), state
            );
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, message);
            opMode.sleepUntil(AbstractOpMode.GORILLA_CORE, "AbstractVisionX2Task.waitForPortalState waiting for portal to be ready", System.currentTimeMillis() + 20);
        }
        VisionPortal.CameraState state        = portal.getCameraState();
        long                     waitDuration = System.currentTimeMillis() - waitTime0;
        if (!stateToWaitFor.equals(state)) {
            String message = String.format(Locale.US, "AbstractVisionX2Task.waitForPortalState(%s), abort: portal state is %s after %dms", stateToWaitFor, state, waitDuration);
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, message);
            throw new RuntimeException(message);
        }
        String message = String.format(Locale.US, "AbstractVisionX2Task.waitForPortalState(%s), done: portal state is %s after %dms", stateToWaitFor, state, waitDuration);
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, message);

        return this;
    }

    public ColorBlobLocatorProcessor.Builder createBasicColorBlobLocatorProcessorBuilder(ColorRange color) {
        // See ConceptVisionColorLocator_Circle.
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true);
    }

    public ColorBlobLocatorProcessor.Builder createCircleColorBlobLocatorProcessorBuilder(ColorRange color) {
        return createBasicColorBlobLocatorProcessorBuilder(color)
                // Clear drawing box fit, then set circle color to draw on RC/DS view.
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                ;
    }

    // When oh when will FIRST move from Java 11 to Java 25. Or even 17. *Sigh*
    // TODO: AprilTagDetections should be a record and not a class.
    public static class AprilTagDetections {
        public List<AprilTagDetection> left;
        public List<AprilTagDetection> right;
    } // static class AprilTagDetections

    protected abstract class AbstractVisionRunner implements Runnable {

        @SuppressWarnings("UnusedReturnValue")
        protected abstract AbstractVisionRunner loop();

        protected void updateDetections(
                ConcurrentHashMap<Integer, AprilTagDetection> targets,
                String                                        cameraLabel,
                List<AprilTagDetection>                       detections
        ) {
            Function<AprilTagPoseRaw, String> RawToString = (AprilTagPoseRaw value) ->
                    "x:" + value.x +
                            " y:" + value.y +
                            " z:" + value.z +
                            " R:" + value.R;
            Function<AprilTagPoseFtc, String> FtcToString = (AprilTagPoseFtc value) ->
                    "x:" + value.x +
                            " y:" + value.y +
                            " z:" + value.z +
                            "\n\tbearing:" + value.bearing +
                            " elevation:" + value.elevation +
                            " range:" + value.range +
                            "\n\tpitch:" + value.pitch +
                            " roll:" + value.roll +
                            " yaw:" + value.yaw;
            @SuppressWarnings("unused")
            Function<Pose3D, String> PoseToString = Pose3D::toString;

            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s: %d fresh detections", cameraLabel, detections.size());
            detections.forEach(detection -> {
                RobotLog.ii(
                        AbstractOpMode.GORILLA_CORE, "AbstractVisionX2Task.updateDetections(%s): targets=%s detection=%s",
                        cameraLabel, targets, detection
                );
                targets.put(detection.id, detection);
                if (latestDetectionNanos < detection.frameAcquisitionNanoTime) {
                    latestDetectionNanos = detection.frameAcquisitionNanoTime;
                }
                targetDetectionsCounts.compute(detection.id, (Integer id, Long value) -> null==value?1:value+1);

                String name = null == detection.metadata ? "unknown" : detection.metadata.name;
                RobotLog.ii(
                        AbstractOpMode.GORILLA_CORE,
                        "updateDetections(%s): id:%d name:%s time:%d latest:%d margin:%f hamming:%d\n\t"
                                + "center:%s corners:%d/%s/%s/%s/%s\n\t"
                                + "raw:%s\n\t"
                                + "ftc:%s\n\t"
                                + "bot:%s",
                        cameraLabel,
                        detection.id, name, detection.frameAcquisitionNanoTime, latestDetectionNanos,
                        detection.decisionMargin,
                        detection.hamming,
                        detection.center,
                        detection.corners.length, detection.corners[0], detection.corners[1], detection.corners[2], detection.corners[3],
                        RawToString.apply(detection.rawPose),
                        FtcToString.apply(detection.ftcPose),
                        detection.robotPose
                );
            });

        }

        @Override
        public void run() {
            long time0 = System.currentTimeMillis();
            long nextRunTime = time0;
            int count = 0;
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run start: time0 = %d", time0);
            stopped = false;
            running = true;
            while (running) {
                long now = System.currentTimeMillis();
                if (nextRunTime < now) {
                    nextRunTime = now;
                }
                nextRunTime += RUN_FREQUENCY_MILLIS;
                RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run next run: count=%d now=%d next=%d time0=%d", count, now, nextRunTime, time0);
                loop();
                count++;
                // We can't dawdle after stop. RobotCore will kill us if we don't stop REAL quick.
                try {
                    opMode.sleepUntil(
                            AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run waiting", nextRunTime,
                            () -> !running
                    );
                } catch (InterruptedException ex) {
                    // The only way we can be interrupted is if we are stopping. And we must stop quickly.
                    RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run ending wait due to stop command");
                }
            }
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run resources closing");
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run resources closed");

            stopped = true;
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run done");

        }

        @SuppressWarnings("UnusedReturnValue")
        public AbstractVisionRunner stop() {
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.stop() start");

            running = false;
            visionThread.interrupt();

            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.stop() waiting");
            while (!stopped) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.stop() waited");

            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.stop() done");

            return this;
        }

        public synchronized AprilTagDetections detections() {
            AprilTagDetections result = detections;
            detections = null;
            return result;
        }

        public boolean streaming() {
            return streaming;
        }

        /** Implementations should probably be synchronized. */
        @SuppressWarnings("UnusedReturnValue")
        public abstract AbstractVisionRunner streaming(boolean value);

        @SuppressWarnings("unused")
        public boolean liveView() {
            return liveView;
        }

        /** Implementations should probably be synchronized. */
        @SuppressWarnings("unused")
        public abstract AbstractVisionRunner liveView(boolean value);

        protected AprilTagDetections detections;

        protected volatile boolean streaming = false;

        protected volatile boolean liveView = false;

        // TODO: We really need to have a state variable:
        // created, initializing, initialized, starting, running, stopping, cleaning, stopped.,

        private boolean running = false;

        private boolean stopped = true;

        private final long RUN_FREQUENCY_MILLIS = 100;

    } // class AbstractVisionRunner

    @SuppressWarnings("unused")
    public synchronized AprilTagDetections detections() {
        return visionRunner.detections();
    }

    public AbstractVisionX2Task<OpModeT> cameraLeft(WebcamName value) {
        cameraLeft = value;

        return this;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AbstractVisionX2Task<OpModeT> cameraRight(WebcamName value) {
        cameraRight = value;

        return this;
    }

    @SuppressWarnings({"usedReturnValue", "unused"})
    public boolean streaming() {
        return visionRunner.streaming();
    }

    @SuppressWarnings("unused")
    public synchronized AbstractVisionX2Task<OpModeT> streaming(boolean value) {
        visionRunner.streaming(value);

        return this;
    }

    public ConcurrentHashMap<Integer, AprilTagDetection> targetDetectionsLeft = new ConcurrentHashMap<>();

    public ConcurrentHashMap<Integer, AprilTagDetection> targetDetectionsRight = new ConcurrentHashMap<>();

    // How many times have we seen each target april tag? Possibly helpful to filter out occasional bad tag detections.
    public ConcurrentHashMap<Integer, Long> targetDetectionsCounts = new ConcurrentHashMap<>();

    long latestDetectionNanos;

    protected AbstractVisionRunner visionRunner = createVisionRunner();

    protected Thread visionThread = new Thread(visionRunner);

    WebcamName cameraLeft;

    WebcamName cameraRight;

} // class AbstractVisionX2Task
