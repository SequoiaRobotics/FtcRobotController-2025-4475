package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.Blob;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobCriteria;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobFilter;
import static org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobSort;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.gorillacoder.AbstractBotTask;
import org.firstinspires.ftc.gorillacoder.AbstractOpMode;
import org.firstinspires.ftc.gorillacoder.BotTask;
import org.firstinspires.ftc.gorillacoder.DrivePovTask;
import org.firstinspires.ftc.gorillacoder.DriveTankTask;
import org.firstinspires.ftc.gorillacoder.VisionTaskMultiPortal;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.Locale;

@SuppressWarnings("unused")
@TeleOp
public class TeleOpDrive extends AbstractOpMode<TeleOpDrive> {

    public abstract static class TeleOpDriveTask extends AbstractBotTask<TeleOpDrive> {
    }

    protected class GamePadTask extends TeleOpDriveTask {
        @Override
        public GamePadTask run() {
            // POV Drive
            drivePovTask.speed    = gamepad1.right_stick_y;
            drivePovTask.turnRate = gamepad1.right_stick_x;

            // Tank Drive
            driveTankTask.leftPower  = gamepad1.left_stick_y;
            driveTankTask.rightPower = gamepad1.right_stick_y;

            return this;
        }

        @Override
        public GamePadTask init() {
            frequencyMillis(5); // Every 5 ms, 200/second.
            return this;
        }
    } // class GamePadTask

    protected class BallLocationTask extends TeleOpDriveTask {
        @Override
        public BallLocationTask run() {
            String left = "Left camera";
            String purple = "purple";
            blobLocatorPurpleLeft.getBlobs().forEach(blob ->
                    RobotLog.ii( GORILLA_CORE, "%s sees a %s blob radius %f at (%f,%f)/(%f,%f) ",
                            left, purple,
                            blob.getCircle().getRadius(), blob.getCircle().getX(), blob.getCircle().getCenter().x, blob.getCircle().getY(), blob.getCircle().getCenter().y,
                            blob.getArcLength(), blob.getAspectRatio(), blob.getCircularity(), blob.getContourArea(), blob.getDensity()
                    )
            );
            String right = "Right camera";
            String green = "green";
            blobLocatorPurpleLeft.getBlobs().forEach(blob ->
                    RobotLog.ii( GORILLA_CORE, "%s sees a %s blob radius %f at (%f,%f)/(%f,%f) ",
                            right, green,
                            blob.getCircle().getRadius(), blob.getCircle().getX(), blob.getCircle().getCenter().x, blob.getCircle().getY(), blob.getCircle().getCenter().y,
                            blob.getArcLength(), blob.getAspectRatio(), blob.getCircularity(), blob.getContourArea(), blob.getDensity()
                    )
            );

            return this;
        }

        @Override
        public BallLocationTask init() {
            frequencyMillis(100); // Every 5 ms, 200/second.
            return this;
        }
    } // class BallLocationTask

    protected class TelemetryTask extends TeleOpDriveTask {
        @Override
        public TelemetryTask run() {
            // Red Tower seen from left camera
            AprilTagDetection redLeftDetection   = visionTask.targetDetectionsLeft.get(24);
            String            towerRedLeft       = "b:? r:? y:?";
            String            botRedLeft         = "x:? y:? y:?";
            // Blue Tower seen from left camera
            AprilTagDetection blueLeftDetection  = visionTask.targetDetectionsLeft.get(20);
            String            towerBlueLeft      = "b:? r:? y:?";
            String            botBlueLeft        = "x:? y:? y:?";
            // Red Tower seen from right camera
            AprilTagDetection redRightDetection  = visionTask.targetDetectionsRight.get(24);
            String            towerRedRight      = "b:? r:? y:?";
            String            botRedRight        = "x:? y:? y:?";
            // Blue Tower seen from right camera
            AprilTagDetection blueRightDetection = visionTask.targetDetectionsRight.get(20);
            String            towerBlueRight     = "b:? r:? y:?";
            String            botBlueRight       = "x:? y:? y:?";

            if (null != redLeftDetection) {
                towerRedLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                        redLeftDetection.ftcPose.bearing, redLeftDetection.ftcPose.range, redLeftDetection.ftcPose.yaw);
                botRedLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                        redLeftDetection.robotPose.getPosition().x, redLeftDetection.robotPose.getPosition().y, redLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
            }
            if (null != blueLeftDetection) {
                towerBlueLeft = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                        blueLeftDetection.ftcPose.bearing, blueLeftDetection.ftcPose.range, blueLeftDetection.ftcPose.yaw);
                botBlueLeft = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                        blueLeftDetection.robotPose.getPosition().x, blueLeftDetection.robotPose.getPosition().y, blueLeftDetection.robotPose.getOrientation().getYaw(DEGREES));
            }
            if (null != redRightDetection) {
                towerRedRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                        redRightDetection.ftcPose.bearing, redRightDetection.ftcPose.range, redRightDetection.ftcPose.yaw);
                botRedRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                        redRightDetection.robotPose.getPosition().x, redRightDetection.robotPose.getPosition().y, redRightDetection.robotPose.getOrientation().getYaw(DEGREES));
            }
            if (null != blueRightDetection) {
                towerBlueRight = String.format(Locale.US, "b:%.0f r:%.0f y:%.0f",
                        blueRightDetection.ftcPose.bearing, blueRightDetection.ftcPose.range, blueRightDetection.ftcPose.yaw);
                botBlueRight = String.format(Locale.US, "x:%.0f y:%.0f y:%.0f",
                        blueRightDetection.robotPose.getPosition().x, blueRightDetection.robotPose.getPosition().y, blueRightDetection.robotPose.getOrientation().getYaw(DEGREES));
            }

            List<Blob> greenBlobs     = blobLocatorGreenRight.getBlobs();
            List<Blob> purpleBlobs    = blobLocatorPurpleLeft.getBlobs();
            String     greenBlobInfo  = " 0 #1: (  ?,   ?)\na:    d:     r:     l:      c:";
            String     purpleBlobInfo = " 0 #1: (  ?,   ?)\na:    d:     r:     l:      c:";
            if (!greenBlobs.isEmpty()) {
                Blob blob = greenBlobs.get(0);
                greenBlobInfo = String.format(
                        Locale.US, "%2d #1: (%3.0f, %3.0f)\na:%3d d:%.2f r:%.2f l:%6.2f c:%.2f",
                        greenBlobs.size(), blob.getCircle().getX(), blob.getCircle().getY(),
                        blob.getContourArea(), blob.getDensity(), blob.getAspectRatio(), blob.getArcLength(), blob.getCircularity()
                );
            }
            if (!purpleBlobs.isEmpty()) {
                Blob blob = purpleBlobs.get(0);
                purpleBlobInfo = String.format(
                        Locale.US, "%2d #1: (%3.0f, %3.0f)\na:%3d d:%.2f r:%.2f l:%6.2f c:%.2f",
                        purpleBlobs.size(), blob.getCircle().getX(), blob.getCircle().getY(),
                        blob.getContourArea(), blob.getDensity(), blob.getAspectRatio(), blob.getArcLength(), blob.getCircularity()
                );
            }

            telemetry.addData("Status", "running %s", runtime);
            telemetry.addData("Camera", "%s %s", visionTask.portalLeft.getCameraState() , visionTask.portalRight.getCameraState());
            telemetry.addData("Motors", "speed:%.2f  turn:%.2f", drivePovTask.speed, drivePovTask.turnRate);
            telemetry.addData("Motors", " left:%.2f right:%.2f", drivePovTask.leftPower, drivePovTask.rightPower);
            telemetry.addData("Bot RL", "%s", botRedLeft);
            telemetry.addData("    RR", "%s", botRedRight);
            telemetry.addData("    BL", "%s", botBlueLeft);
            telemetry.addData("    BR", "%s", botBlueRight);
            telemetry.addData("Twr RL", "%s", towerRedLeft);
            telemetry.addData("    RR", "%s", towerRedRight);
            telemetry.addData("    BL", "%s", towerBlueLeft);
            telemetry.addData("    BR", "%s", towerBlueRight);
            telemetry.addData("Green ", "%s", greenBlobInfo);
            telemetry.addData("Purple", "%s", purpleBlobInfo);

            telemetry.update();
            return this;
        }

        @Override
        public TelemetryTask init() {
            frequencyMillis(100); // Every 5 ms, 200/second.
            return this;
        }

        // TODO: add this to tasks, call for all tasks in AbstractOpMode.
        public TelemetryTask waitForStart() {
            run();

            return this;
        }
    } // class TelemetryTask

    protected TeleOpDrive configureTelemetry() {
        telemetry.log().setCapacity(100);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        return this;
    }

    @Override
    protected BotTask<TeleOpDrive>[] getTasks() {
        telemetry.addData("status", "TeleOpDrive.createTasks(): tasks created");

        drivePovTask
                .driveLeftFront(hardwareMap.get( DcMotorEx.class, "Drive Front Left"))
                .driveRightFront(hardwareMap.get(DcMotorEx.class, "Drive Front Right"))
                .driveLeftRear(hardwareMap.get(  DcMotorEx.class, "Drive Rear Left"))
                .driveRightRear(hardwareMap.get( DcMotorEx.class, "Drive Rear Right"))
        ;
        visionTask
                .cameraLeft( hardwareMap.get(WebcamName.class, "Webcam Left"))
                .cameraRight(hardwareMap.get(WebcamName.class, "Webcam Right"));
        telemetry.addData("status", "TeleOpDrive.createTasks(): tasks connected to hardware");

        @SuppressWarnings("unchecked")
        BotTask<TeleOpDrive>[] result = new BotTask[] {
                visionTask,
                drivePovTask,
//                driveTankTask,
                gamePadTask,
                new TelemetryTask()
        };
        return result;
    }


    public static final ColorRange ARTIFACT_GREEN = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 16,  50, 118),
            new Scalar(255, 105, 145)
    );

    public static final ColorRange ARTIFACT_PURPLE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 16, 135, 135),
            new Scalar(255, 155, 169)
    );

    protected static final Position cameraPositionLeft  = new Position(INCH, -6.5, 7., 10., 0);
    protected static final Position cameraPositionRight = new Position(INCH,  6.5, 7., 10., 0);
    protected static final YawPitchRollAngles cameraOrientationLeft  = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);
    protected static final YawPitchRollAngles cameraOrientationRight = new YawPitchRollAngles(DEGREES, 0, -90, 0, 0);

    @Override
    protected TeleOpDrive configureAprilTagProcessors() {
        visionTask.atpBuilderLeft. setCameraPose(cameraPositionLeft, cameraOrientationLeft);
        visionTask.atpBuilderRight.setCameraPose(cameraPositionRight, cameraOrientationRight);

        return this;
    }

    @Override
    protected TeleOpDrive addVisionProcessors() {
        // Need to:
        // - Create the right and left portal builders and set default config. VisionTask will do this. Bots can always override.
        // - Create the left and right april tag processor builders and set default config. VisionTask will do this. Bots can always override.
        // - Create and configure other processor builders. For this game, the blob detectors.
        // - Let VisionTask.init() finish up the init, in particular create the processors and add them to the VisionPortals.
        // - Get the map from builder to processor and init our processor member variables.
        ColorBlobLocatorProcessor.Builder builderPurpleLeft = visionTask.createCircleColorBlobLocatorProcessorBuilder(ARTIFACT_PURPLE);
        ColorBlobLocatorProcessor.Builder builderGreenRight = visionTask.createCircleColorBlobLocatorProcessorBuilder(ARTIFACT_GREEN);

        BlobFilter areaFilter     = new BlobFilter(BlobCriteria.BY_CONTOUR_AREA, 20, 1000);
        BlobFilter densityFilter  = new BlobFilter(BlobCriteria.BY_DENSITY,      0.3, 1.0);
        BlobFilter ratioFilter    = new BlobFilter(BlobCriteria.BY_ASPECT_RATIO, 1.0, 2);
        BlobSort   blobSortByArea = new BlobSort(BlobCriteria.BY_CONTOUR_AREA,   SortOrder.ASCENDING);

        blobLocatorPurpleLeft = builderPurpleLeft
                // Smooth the transitions between different colors in image
                .setBlurSize(5)
                // fill in perimeter holes. Dilate to fill in edge divots, then shrink to original size.
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .setDilateSize(15)
                .setErodeSize(15)
                .build();
        blobLocatorGreenRight = builderGreenRight
                // Smooth the transitions between different colors in image
                .setBlurSize(5)
                // fill in perimeter holes. Dilate to fill in edge divots, then shrink to original size.
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .setDilateSize(15)
                .setErodeSize(15)
                .build();

        blobLocatorPurpleLeft.addFilter(areaFilter);
        blobLocatorPurpleLeft.addFilter(densityFilter);
        blobLocatorPurpleLeft.addFilter(ratioFilter);
        blobLocatorPurpleLeft.setSort(blobSortByArea);

        blobLocatorGreenRight.addFilter(areaFilter);
        blobLocatorGreenRight.addFilter(densityFilter);
        blobLocatorGreenRight.addFilter(ratioFilter);
        blobLocatorGreenRight.setSort(blobSortByArea);

        visionTask.addProcessorLeft(blobLocatorPurpleLeft);
        visionTask.addProcessorRight(blobLocatorGreenRight);

        return this;
    }

    protected VisionTaskMultiPortal<TeleOpDrive> visionTask   = new VisionTaskMultiPortal<>();

    protected DrivePovTask<TeleOpDrive>          drivePovTask = new DrivePovTask<>();

    protected DriveTankTask<TeleOpDrive>         driveTankTask = new DriveTankTask<>();

    protected GamePadTask                        gamePadTask  = new GamePadTask();

    ColorBlobLocatorProcessor blobLocatorPurpleLeft;

    ColorBlobLocatorProcessor blobLocatorGreenRight;

} // class TeleOpDrive
