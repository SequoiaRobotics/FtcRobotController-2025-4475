package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.opencv.ColorRange.ARTIFACT_GREEN;
import static org.firstinspires.ftc.vision.opencv.ColorRange.ARTIFACT_PURPLE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.gorillacoder.DrivePovTask;
import org.firstinspires.ftc.gorillacoder.DriveTankTask;
import org.firstinspires.ftc.gorillacoder.VisionTaskMultiPortal;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.gorillacoder.AbstractBotTask;
import org.firstinspires.ftc.gorillacoder.AbstractOpMode;
import org.firstinspires.ftc.gorillacoder.AbstractVisionX2Task;
import org.firstinspires.ftc.gorillacoder.BotTask;
import org.firstinspires.ftc.gorillacoder.VisionTaskSwitchingCameras;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

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

        // Need to:
        // - Create the right and left portal builders and set default config. VisionTask will do this. Bots can always override.
        // - Create the left and right april tag processor builders and set default config. VisionTask will do this. Bots can always override.
        // - Create and configure other processor builders. For this game, the blob detectors.
        // - Let VisionTask.init() finish up the init, in particular create the processors and add them to the VisionPortals.
        // - Get the map from builder to processor and init our processor member variables.
        ColorBlobLocatorProcessor.Builder CBLPBuilderLeft  = visionTask.createSimpleColorBlobLocatorProcessorBuilder(ARTIFACT_PURPLE);
        ColorBlobLocatorProcessor.Builder CBLPBuilderRight = visionTask.createSimpleColorBlobLocatorProcessorBuilder(ARTIFACT_GREEN);

//        visionTask.addProcessorBuilders();

        @SuppressWarnings("unchecked")
        BotTask<TeleOpDrive>[] result = new BotTask[] {
                visionTask,
                drivePovTask,
//                driveTankTask,
                gamePadTask
        };
        return result;
    }

//    protected TeleOpDrive init2() {
//        ColorBlobLocatorProcessor.Builder rightCBLR = visionTask.createSimpleColorBlobLocatorProcessorBuilder(ARTIFACT_GREEN);
//                ;
//        ColorBlobLocatorProcessor.Builder leftCBLR = visionTask.createSimpleColorBlobLocatorProcessorBuilder(ARTIFACT_PURPLE);
//                ;
//        //
//
//        // visionTask.buildPortal();
//
//        return this;
//    }

//    protected AbstractVisionX2Task<TeleOpDrive> visionTask   = new VisionTaskSwitchingCameras<>();
    protected AbstractVisionX2Task<TeleOpDrive> visionTask   = new VisionTaskMultiPortal<>();

    protected DrivePovTask<TeleOpDrive>         drivePovTask = new DrivePovTask<>();

    protected DriveTankTask<TeleOpDrive>        driveTankTask = new DriveTankTask<>();

    protected GamePadTask                       gamePadTask  = new GamePadTask();

} // class TeleOpDrive
