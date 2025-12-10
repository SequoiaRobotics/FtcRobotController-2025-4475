package org.firstinspires.ftc.gorillacoder;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.CAMERA_DEVICE_CLOSED;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.CAMERA_DEVICE_READY;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;
import static java.util.concurrent.TimeUnit.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class VisionTaskSwitchingCameras<OpModeT extends OpMode> extends AbstractVisionX2Task<OpModeT> {

    protected class VisionSwitchingRunner extends AbstractVisionRunner {
        @SuppressWarnings("ConstantConditions")
        public synchronized VisionSwitchingRunner loop() {
            if (!STREAMING.equals(portal.getCameraState())) {
                RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.run do nothing, not streaming, state=%s camera=%s", portal.getCameraState(), portal.getActiveCamera());

                return this;
            }

            // TODO: Stay more responsive. Each time through loop do just one camera.
            // Collect the detections that are fresh, update info, switch cameras, done.
            telemetry.addData("portal state", portal.getCameraState());
            AprilTagDetections currentDetections = new AprilTagDetections();
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.run state=%s camera=%s", portal.getCameraState(), portal.getActiveCamera());
            currentDetections.left = processor.getFreshDetections();
            if (null == currentDetections.left) {
                currentDetections.left = new ArrayList<>();
            }
            updateDetections(targetDetectionsLeft, "Left", currentDetections.left);

// Remove remaining code. Either RC App can't stop the bot code, or Android can't find the cameras on next OpMode run after running the below.
// Why???? I dunno. It looks essentially the same as the ConceptAprilTagSwitchableCamera code.
if (true) {return this;}
            portal.setActiveCamera(cameraRight);
            opMode.sleepUntil(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.loop waiting after changing to right camera", System.currentTimeMillis() + 20);

            int waitCount = 0;
            long waitTime0 = System.currentTimeMillis();

            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.loop() check if cameras ready after switching: STREAMING=%d state=%d/%s", STREAMING.ordinal(), portal.getCameraState().ordinal(), portal.getCameraState());

            // TODO: write waitFor( condition:lambda, checkDelay, limit, tag, label);
            for (VisionPortal.CameraState state = portal.getCameraState();
                 state.ordinal() < STREAMING.ordinal();
                 state = portal.getCameraState()
            ) {
                waitCount++;
                if (200 < waitCount) {
                    throw new RuntimeException("VisionSwitchingRunner.loop, gave up waiting for portal to be ready after switching, state:" + state);
                }
                RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.loop() waiting for portal to be ready after switching, calling sleepUntil until next iteration: waitCount=%d state=%d/%s", waitCount, state.ordinal(), state);
                opMode.sleepUntil(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.loop waiting for portal to be ready after switching", System.currentTimeMillis() + 20);
            }
            long waitDuration = System.currentTimeMillis() - waitTime0;
            if (!STREAMING.equals(portal.getCameraState())) {
                RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.loop(), abort because portal is not streaming after switching");
                telemetry.log().add("VisionSwitchingRunner.loop(), early exit because portal is not streaming after switching");
                throw new RuntimeException(String.format("VisionSwitchingRunner.loop(), early exit because portal is not streaming after switching: count=%d duration=%d", waitCount, waitDuration));
            }
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionSwitchingRunner.loop() portal ready after switching: wait=%d/%dms state=%s camera=%s", waitCount, waitDuration, portal.getCameraState(), portal.getActiveCamera());
            telemetry.log().add("VisionSwitchingRunner.loop() portal ready after switching: waitCount=%d state=%s camera=%s", waitCount, portal.getCameraState(), portal.getActiveCamera());

            currentDetections.right = processor.getFreshDetections();
            if (null == currentDetections.right) {
                currentDetections.right = new ArrayList<>();
            }
            updateDetections(targetDetectionsRight, "Right", currentDetections.right);

            // While we wait to run again, let the april tag processor process
            portal.setActiveCamera(cameraLeft);
            synchronized (this) {
                detections = currentDetections;
            }
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "Left: %d AprilTags ", currentDetections.left.size());
            RobotLog.ii(AbstractOpMode.GORILLA_CORE, "Right: %d AprilTags ", currentDetections.right.size());

            return this;
        }

        protected VisionSwitchingRunner openVisionResources() {
            ClassFactory classFactory = ClassFactory.getInstance();
            CameraName switchableCamera = classFactory
                    .getCameraManager().nameForSwitchableCamera(cameraLeft, cameraRight);

            // TODO: Add processor for artifact (and other object) detections?
            processor = createAprilTagProcessor();
            portal    = createVisionPortal(switchableCamera, processor);
            waitForPortalState(portal, STREAMING, 200);

            return this;
        }

        @Override
        protected AbstractVisionRunner closeVisionResources() {
            // TODO: Re-enable tis after testing whether commenting it out keeps the bot from "losing" the camera,
            // and not finding it when another OpMode is run.
            // portal.close();
            // waitForPortalState(portal, CAMERA_DEVICE_CLOSED, 1000);

            return this;
        }

        public synchronized VisionSwitchingRunner streaming(boolean value) {
            streaming = value;

            waitForPortalState(portal, CAMERA_DEVICE_READY, 20);
            if (value) {
                portal.resumeStreaming();
            } else {
                portal.stopStreaming();
            }

            return this;
        }

        public synchronized VisionSwitchingRunner liveView(boolean value) {
            liveView = value;
            if (value) {
                portal.resumeLiveView();
            } else {
                portal.stopLiveView();
            }

            return this;
        }

        private AprilTagProcessor processor;

        private VisionPortal portal;
    } // class VisionSwitchingRunner

    protected VisionSwitchingRunner createVisionRunner() {
        return new VisionSwitchingRunner();
    }

    @Override
    public VisionTaskSwitchingCameras<OpModeT> start() {
        visionThread.start();
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskSwitchingCameras.start(), visionThread started: state=%s", visionThread.getState());

        return this;
    }

    @Override
    public VisionTaskSwitchingCameras<OpModeT> init() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskSwitchingCameras.init(), start");

        // Run once per second.
        // This task doesn't really do anything. It's all done by the runner.
        // This task is just a wrapper to hide the runner, and forward calls to it.
        // Heck, we could probably run this once an hour and all would be fine.
        this.frequencyMillis(SECONDS.toMillis(1));

        visionRunner.init();

        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "VisionTaskSwitchingCameras.init(), done");
        telemetry.log().add("VisionTaskSwitchingCameras.init(), done");
        telemetry.update();
        return this;
    }

    public VisionTaskSwitchingCameras<OpModeT> stop() {
        visionRunner.stop();

        return this;
    }

    public synchronized AprilTagDetections detections() {
        return visionRunner.detections();
    }

    public VisionTaskSwitchingCameras<OpModeT> cameraLeft(WebcamName value) {
        cameraLeft = value;

        return this;
    }

    public VisionTaskSwitchingCameras<OpModeT> cameraRight(WebcamName value) {
        cameraRight = value;

        return this;
    }

    public boolean streaming() {
        return visionRunner.streaming();
    }

    public synchronized VisionTaskSwitchingCameras<OpModeT> streaming(boolean value) {
        visionRunner.streaming(value);

        return this;
    }

    VisionSwitchingRunner visionRunner = new VisionSwitchingRunner();

    Thread visionThread = new Thread(visionRunner);

    WebcamName cameraLeft;

    WebcamName cameraRight;
} // class VisionTaskSwitchingCameras
