package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.CAMERA_DEVICE_READY;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;
import static java.util.concurrent.TimeUnit.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;

public class VisionTaskMultiPortal<OpModeT extends OpMode> extends AbstractVisionX2Task<OpModeT> {

    protected class VisionMultiRunner extends AbstractVisionRunner {
        public synchronized VisionMultiRunner loop() {
            // Collect the detections that are fresh, update info, switch cameras, done.

            if ( !portalLeft .getCameraState().equals(STREAMING) ) return this;
            if ( !portalRight.getCameraState().equals(STREAMING) ) return this;

            AprilTagDetections currentDetections = new AprilTagDetections();
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionRunner.run  left camera state=%s", portalLeft .getCameraState());
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionRunner.run right camera state=%s", portalRight.getCameraState());

            currentDetections.left  = aprilTagProcessorLeft .getFreshDetections();
            currentDetections.right = aprilTagProcessorRight.getFreshDetections();

            if ( null == currentDetections.left  ) {
                currentDetections.left  = Collections.emptyList();
            }
            if ( null == currentDetections.right ) {
                currentDetections.right = Collections.emptyList();
            }
            opMode.onFreshDetections("Left",  currentDetections.left);
            opMode.onFreshDetections("Right", currentDetections.right);

            updateDetections(targetDetectionsLeft,  "Left",  currentDetections.left);
            updateDetections(targetDetectionsRight, "Right", currentDetections.right);

            synchronized (this) {
                detections = currentDetections;
            }

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

    } // class VisionMultiRunner

    protected VisionMultiRunner createVisionRunner() {
        return new VisionMultiRunner();
    }

    @Override
    public VisionTaskMultiPortal<OpModeT> init() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskMultiPortal.init(), start");

        // Run once per second.
        // This task doesn't really do anything. It's all done by the runner.
        // This task is just a wrapper to hide the runner, and forward calls to it.
        // Heck, we could probably run this once an hour and all would be fine.
        this.frequencyMillis(SECONDS.toMillis(1));

        int viewIds[]   = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        int viewIdLeft  = viewIds[0];
        int viewIdRight = viewIds[1];

        // TODO: Add processor for artifact (and other object) detections?
        portalBuilderLeft = createVisionPortalBuilder()
                .setCamera(cameraLeft)
                .setLiveViewContainerId(viewIdLeft)
                ;
        portalBuilderRight = createVisionPortalBuilder()
                .setCamera(cameraRight)
                .setLiveViewContainerId(viewIdRight)
                ;

        atpBuilderLeft  = createAprilTagProcessorBuilder();
        atpBuilderRight = createAprilTagProcessorBuilder();
        opMode.configureAprilTagProcessors();

        aprilTagProcessorLeft  = atpBuilderLeft.build();
        aprilTagProcessorRight = atpBuilderRight.build();
        portalBuilderLeft.addProcessors(aprilTagProcessorLeft);
        portalBuilderRight.addProcessors(aprilTagProcessorRight);

        opMode.addVisionProcessors();

        portalLeft  = portalBuilderLeft.build();
        portalRight = portalBuilderRight.build();

        // The portals init async and in parallel.
        // If the right finishes first, then there will be no wait.
        // If the left finishes first, give the right just a bit longer to finish.
        waitForPortalState(portalLeft,  STREAMING, 200);
        waitForPortalState(portalRight, STREAMING, 50);

        visionThread.start();
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskMultiPortal.start(), visionThread started: state=%s", visionThread.getState());

        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskMultiPortal.init(), done");
        return this;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AbstractVisionX2Task<OpModeT> addProcessorLeft(VisionProcessor processor) {
        portalBuilderLeft.addProcessor(processor);
        return this;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AbstractVisionX2Task<OpModeT> addProcessorRight(VisionProcessor processor) {
        portalBuilderRight.addProcessor(processor);
        return this;
    }

    public Position botFieldPosition;

//    public Pose

    VisionPortal.Builder portalBuilderLeft = new VisionPortal.Builder();

    VisionPortal.Builder portalBuilderRight = new VisionPortal.Builder();

    public VisionPortal portalLeft;

    public VisionPortal portalRight;

    public AprilTagProcessor.Builder atpBuilderLeft = new AprilTagProcessor.Builder();

    public AprilTagProcessor.Builder atpBuilderRight = new AprilTagProcessor.Builder();

    AprilTagProcessor aprilTagProcessorLeft;

    AprilTagProcessor aprilTagProcessorRight;

} // class VisionTaskMultiPortal
