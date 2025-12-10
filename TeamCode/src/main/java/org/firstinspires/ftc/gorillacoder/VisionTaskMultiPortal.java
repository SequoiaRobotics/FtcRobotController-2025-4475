package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.CAMERA_DEVICE_CLOSED;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.CAMERA_DEVICE_READY;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;
import static java.util.concurrent.TimeUnit.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;

public class VisionTaskMultiPortal<OpModeT extends OpMode> extends AbstractVisionX2Task<OpModeT> {

    protected class VisionMultiRunner extends AbstractVisionRunner {
        public synchronized VisionMultiRunner loop() {
            // Collect the detections that are fresh, update info, switch cameras, done.
            telemetry.addData("left  state", portalLeft.getCameraState());
            telemetry.addData("right state", portalRight.getCameraState());

            if (!STREAMING.equals(portalLeft.getCameraState())) return this;
            if (!STREAMING.equals(portalRight.getCameraState())) return this;

            AprilTagDetections currentDetections = new AprilTagDetections();
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionRunner.run left camera state=%s", portalLeft.getCameraState());
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionRunner.run right camera state=%s", portalLeft.getCameraState());
            currentDetections.left = aprilTagProcessorLeft.getFreshDetections();
            if (null == currentDetections.left) {
                currentDetections.left = new ArrayList<>();
            }
            updateDetections(targetDetectionsLeft, "Left", currentDetections.left);

            currentDetections.right = aprilTagProcessorRight.getFreshDetections();
            if (null == currentDetections.right) {
                currentDetections.right = new ArrayList<>();
            }
            updateDetections(targetDetectionsRight, "Right", currentDetections.right);

            synchronized (this) {
                detections = currentDetections;
            }
            
            return this;
        }

        protected VisionMultiRunner openVisionResources() {

            int[] viewIds   = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
            int viewIdLeft  = viewIds[0];
            int viewIdRight = viewIds[1];

            // TODO: Add processor for artifact (and other object) detections?
            aprilTagProcessorLeft = createAprilTagProcessor();
            portalLeft = createVisionPortalBuilder()
                    .setCamera(cameraLeft)
                    .addProcessors(aprilTagProcessorLeft)
                    .setLiveViewContainerId(viewIdLeft)
                    .build();

            aprilTagProcessorRight = createAprilTagProcessor();
            portalRight = createVisionPortalBuilder()
                    .setCamera(cameraRight)
                    .addProcessors(aprilTagProcessorRight)
                    .setLiveViewContainerId(viewIdRight)
                    .build();

            waitForPortalState(portalLeft,  STREAMING, 200);
            waitForPortalState(portalRight, STREAMING, 20);

            return this;
        }

        @Override
        protected AbstractVisionRunner closeVisionResources() {
//            portalLeft.close();
//            portalRight.close();
//            waitForPortalState(portalLeft,  CAMERA_DEVICE_CLOSED, 1000);
//            waitForPortalState(portalRight, CAMERA_DEVICE_CLOSED, 100);

            return this;
        }

        public synchronized VisionMultiRunner streaming(boolean value) {
            streaming = value;

            waitForPortalState(portalLeft, CAMERA_DEVICE_READY, 20);
            if (value) {
                portalLeft.resumeStreaming();
                portalRight.resumeStreaming();
                // TODO: wait for portals to be streaming
            } else {
                portalLeft.stopStreaming();
                portalRight.stopStreaming();
                // TODO: wait for portals to be not streaming
            }

            return this;
        }

        public synchronized VisionMultiRunner liveView(boolean value) {
            liveView = value;
            if (value) {
                portalLeft.resumeLiveView();
                portalRight.resumeLiveView();
            } else {
                portalLeft.stopLiveView();
                portalRight.stopLiveView();
            }

            return this;
        }

        private AprilTagProcessor aprilTagProcessorLeft;

        AprilTagProcessor aprilTagProcessorRight;

        private VisionPortal portalLeft;

        private VisionPortal portalRight;

    } // class VisionMultiRunner

    protected VisionMultiRunner createVisionRunner() {
        return new VisionMultiRunner();
    }

    @Override
    public VisionTaskMultiPortal<OpModeT> start() {
        visionThread.start();
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskMultiPortal.start(), visionThread started: state=%s", visionThread.getState());

        return this;
    }

    @Override
    public VisionTaskMultiPortal<OpModeT> init() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskMultiPortal.init(), start");

        // Run once per second.
        // This task doesn't really do anything. It's all done by the runner.
        // This task is just a wrapper to hide the runner, and forward calls to it.
        // Heck, we could probably run this once an hour and all would be fine.
        this.frequencyMillis(SECONDS.toMillis(1));

        visionRunner.init();

        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskMultiPortal.init(), done");
        telemetry.log().add("VisionTaskMultiPortal.init(), done");
        telemetry.update();
        return this;
    }

    public VisionTaskMultiPortal<OpModeT> stop() {
        visionRunner.stop();

        return this;
    }

    public synchronized AprilTagDetections detections() {
        return visionRunner.detections();
    }

    public VisionTaskMultiPortal<OpModeT> cameraLeft(WebcamName value) {
        cameraLeft = value;

        return this;
    }

    public VisionTaskMultiPortal<OpModeT> cameraRight(WebcamName value) {
        cameraRight = value;

        return this;
    }

    public boolean streaming() {
        return visionRunner.streaming();
    }

    public synchronized VisionTaskMultiPortal<OpModeT> streaming(boolean value) {
        visionRunner.streaming(value);

        return this;
    }

    public Position botFieldPosition;

//    public Pose

    VisionMultiRunner visionRunner = new VisionMultiRunner();

    Thread visionThread = new Thread(visionRunner);

    WebcamName cameraLeft;

    WebcamName cameraRight;

    VisionPortal.Builder portalLeft = new VisionPortal.Builder();

    VisionPortal.Builder portalRight = new VisionPortal.Builder();

    AprilTagProcessor.Builder atpBuilderLeft = new AprilTagProcessor.Builder();

    AprilTagProcessor.Builder atpBuilderRight = new AprilTagProcessor.Builder();

} // class VisionTaskMultiPortal
