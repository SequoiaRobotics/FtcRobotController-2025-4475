package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.vision.VisionPortal.StreamFormat.MJPEG;
import static java.util.concurrent.TimeUnit.SECONDS;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Function;

public abstract class AbstractVisionX2Task<OpModeT extends OpMode> extends AbstractOpMode.AbstractOpModeTask<OpModeT> {

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

            RobotLog.ii(
                    AbstractOpMode.GORILLA_CORE,
                    "updateDetections(%s): id:%d time:%d latest:%d margin:%f hamming:%d\n\t"
                            + "center:%s corners:%d/%s/%s/%s/%s\n\t"
                            + "raw:%s\n\t"
                            + "ftc:%s\n\t"
                            + "bot:%s",
                    cameraLabel,
                    detection.id, detection.frameAcquisitionNanoTime, latestDetectionNanos,
                    detection.decisionMargin,
                    detection.hamming,
                    detection.center,
                    detection.corners.length, detection.corners[0], detection.corners[1], detection.corners[2], detection.corners[3],
                    RawToString.apply(detection.rawPose),
                    FtcToString.apply(detection.ftcPose),
                    detection.robotPose
            );
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        });

    }

    protected abstract class AbstractVisionRunner implements Runnable {

        @SuppressWarnings("UnusedReturnValue")
        protected abstract AbstractVisionRunner loop();

        @SuppressWarnings("UnusedReturnValue")
        protected abstract AbstractVisionRunner openVisionResources();

        @SuppressWarnings("UnusedReturnValue")
        protected abstract AbstractVisionRunner closeVisionResources();

        @Override
        public void run() {
            long time0 = System.currentTimeMillis();
            long nextRunTime = time0;
            int count = 0;
            telemetry.log().add("AbstractVisionRunner.run(): start");
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
            closeVisionResources();
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run resources closed");

            stopped = true;
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.run done");
            telemetry.log().add("AbstractVisionRunner.run(): done");

        }

        protected AprilTagProcessor createAprilTagProcessor() {
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
                    .build();
        }

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
        protected AbstractVisionRunner waitForPortalState(VisionPortal portal, VisionPortal.CameraState stateToWaitFor, long maxWaitCount) {
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
                    String message = String.format("AbstractVisionX2Task.waitForPortalState(portal, %d/%s, %d) abort portal is not ready: state=%d/%s",
                            stateToWaitFor.ordinal(), stateToWaitFor, maxWaitCount, state.ordinal(), state
                    );
                    throw new RuntimeException(message);
                }
                String message = String.format("AbstractVisionX2Task.waitForPortalState(portal, %d/%s, %d) waiting for portal to be ready, calling sleepUntil until next iteration: waitCount=%d state=%d/%s",
                        stateToWaitFor.ordinal(), stateToWaitFor, maxWaitCount, waitCount, state.ordinal(), state
                );
                RobotLog.ii(AbstractOpMode.GORILLA_CORE, message);
                opMode.sleepUntil(AbstractOpMode.GORILLA_CORE, "AbstractVisionX2Task.waitForPortalState waiting for portal to be ready", System.currentTimeMillis() + 20);
            }
            VisionPortal.CameraState state        = portal.getCameraState();
            long                     waitDuration = System.currentTimeMillis() - waitTime0;
            if (!stateToWaitFor.equals(state)) {
                String message = String.format("AbstractVisionX2Task.waitForPortalState(%s), abort: portal state is %s after %dms", stateToWaitFor, state, waitDuration);
                RobotLog.ii(AbstractOpMode.GORILLA_CORE, message);
                telemetry.log().add(message);
                throw new RuntimeException(message);
            }
            String message = String.format("AbstractVisionX2Task.waitForPortalState(%s), done: portal state is %s after %dms", stateToWaitFor, state, waitDuration);
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, message);
            telemetry.log().add(message);

            return this;
        }

        protected AbstractVisionRunner init() {
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.init(), start");

            openVisionResources();

            // TODO: We really need to have a state variable:
            // created, initializing, initialized, starting, running, stopping, cleaning, stopped.,
            stopped = true;
            running = false;

            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionRunner.init(), done");
            return this;
        }

        @SuppressWarnings("UnusedReturnValue")
        public AbstractVisionRunner stop() {
            telemetry.log().add("AbstractVisionRunner.stop(): start");
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
            telemetry.log().add("AbstractVisionRunner.stop(): done");

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

        private boolean running = false;

        private boolean stopped = true;

        private final long RUN_FREQUENCY_MILLIS = 100;

    } // class AbstractVisionRunner

    protected abstract AbstractVisionRunner createVisionRunner();

    public ColorBlobLocatorProcessor.Builder createSimpleColorBlobLocatorProcessorBuilder(ColorRange color) {
        // TODO: Split out only build, from build and add simple config, etc
        // See ConceptVisionColorLocator_Circle.
        ColorBlobLocatorProcessor.Builder result = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                // .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setRoi(ImageRegion.entireFrame())
                // .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                ;
        return result;
        // TODO: To get the current blobs:
        //             List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        //             * The list of Blobs can be filtered to remove unwanted Blobs.
        //             *   Note:  All contours will be still displayed on the Stream Preview, but only those
        //             *          that satisfy the filter conditions will remain in the current list of
        //             *          "blobs".  Multiple filters may be used.
        //             *
        //             * To perform a filter
        //             *   ColorBlobLocatorProcessor.Util.filterByCriteria(criteria, minValue, maxValue, blobs);
        //             *
        //             * The following criteria are currently supported.
        //             *
        //             * ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA
        //             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any
        //             *   that are too big or small. Start with a large range and then refine the range based
        //             *   on the likely size of the desired object in the viewfinder.
        //             *
        //             * ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY
        //             *   A blob's density is an indication of how "full" the contour is.
        //             *   If you put a rubber band around the contour you would get the "Convex Hull" of the
        //             *   contour. The density is the ratio of Contour-area to Convex Hull-area.
        //             *
        //             * ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO
        //             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
        //             *   A perfect Square has an aspect ratio of 1.  All others are > 1
        //             *
        //             * ColorBlobLocatorProcessor.BlobCriteria.BY_ARC_LENGTH
        //             *   A blob's arc length is the perimeter of the blob.
        //             *   This can be used in conjunction with an area filter to detect oddly shaped blobs.
        //             *
        //             * ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY
        //             *   A blob's circularity is how circular it is based on the known area and arc length.
        //             *   A perfect circle has a circularity of 1.  All others are < 1
        //             */
        //            ColorBlobLocatorProcessor.Util.filterByCriteria(
        //                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
        //                    50, 20000, blobs);  // filter out very small blobs.
        //
        //            ColorBlobLocatorProcessor.Util.filterByCriteria(
        //                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
        //                    0.6, 1, blobs);     /* filter out non-circular blobs.
        //                    * NOTE: You may want to adjust the minimum value depending on your use case.
        //                    * Circularity values will be affected by shadows, and will therefore vary based
        //                    * on the location of the camera on your robot and venue lighting. It is strongly
        //                    * encouraged to test your vision on the competition field if your event allows
        //                    * sensor calibration time.
        //                    */
        //
        //            /*
        //             * The list of Blobs can be sorted using the same Blob attributes as listed above.
        //             * No more than one sort call should be made.  Sorting can use ascending or descending order.
        //             * Here is an example.:
        //             *   ColorBlobLocatorProcessor.Util.sortByCriteria(
        //             *      ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
        //             */
        //
        //            telemetry.addLine("Circularity Radius Center");
        //
        //            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        //            for (ColorBlobLocatorProcessor.Blob b : blobs) {
        //
        //                Circle circleFit = b.getCircle();
        //                telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
        //                           b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        //            }
    }

    @Override
    public AbstractVisionX2Task<OpModeT> start() {
        visionThread.start();
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "AbstractVisionX2Task.start(), visionThread started: state=%s", visionThread.getState());

        return this;
    }

    /** Initialize:
     *  * How often should the task run? Probably not often, if even ever. The work is done by the runner.
     *  * The VisionRunner.
     */
    @Override
    public abstract AbstractVisionX2Task<OpModeT> init();

//    protected abstract AbstractVisionX2Task<OpModeT> build();

    public AbstractVisionX2Task<OpModeT> stop() {
        visionRunner.stop();

        return this;
    }

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

    // When oh when will FIRST move from Java 11 to Java 25. Or even 17. *Sigh*
    // TODO: AprilTagDetections should be a record and not a class.
    public static class AprilTagDetections {
        public List<AprilTagDetection> right;
        public List<AprilTagDetection> left;
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
