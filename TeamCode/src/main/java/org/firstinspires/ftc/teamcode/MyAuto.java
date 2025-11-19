package org.firstinspires.ftc.robotcontroller.external.samples.studica;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class MyAuto extends OpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    static final double DRIVE_SPEED = 1;
    ElapsedTime elapsedTimer = new ElapsedTime();

    enum DriveState {
        IDLE,
        DRIVING
    }

    DriveState driveState = DriveState.IDLE;
    double driveTimeout;

    private boolean doneDriving = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        elapsedTimer.reset();
    }

    @Override
    public void loop() {
        if (!doneDriving) {
            // Drive forward for 2 seconds (positive powers were going backward before)
            if (driveDistanceByTime(-DRIVE_SPEED, -DRIVE_SPEED, 1)) {
                doneDriving = true;
                telemetry.addData("Status", "Done Driving!");
            }
        }
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public boolean driveDistanceByTime(double leftSpeed, double rightSpeed, double time) {
        switch (driveState) {
            case IDLE:
                leftDrive.setPower(leftSpeed);
                rightDrive.setPower(rightSpeed);
                driveState = DriveState.DRIVING;
                driveTimeout = elapsedTimer.seconds() + time;
                break;
            case DRIVING:
                if (elapsedTimer.seconds() > driveTimeout) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    driveState = DriveState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }
}
