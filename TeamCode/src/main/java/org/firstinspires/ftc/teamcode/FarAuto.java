/*
 * Copyright 2025 FIRST
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
package org.firstinspires.ftc.robotcontroller.external.samples.studica;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class FarAuto extends OpMode {
    static final double FULL_SPEED = 1.0;
    static final double GOAL_SPEED = 0.85;
    static final double STOP_SPEED = 0.0;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor flyWheel = null;
    private CRServo backSpin = null;
    private CRServo indexLeft = null;
    private CRServo indexRight = null;

    private enum FieldSide {
        BLUE,
        RED
    }

    private FieldSide side = FieldSide.BLUE; // Defaults to Blue

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        backSpin = hardwareMap.get(CRServo.class, "backSpin");
        indexLeft = hardwareMap.get(CRServo.class, "leftServo");
        indexRight = hardwareMap.get(CRServo.class, "rightServo");

        // Incase of wiring into the wrong ports these flags can be switched
        backSpin.setDirection(DcMotor.Direction.FORWARD);
        indexLeft.setDirection(DcMotor.Direction.FORWARD);
        indexRight.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (gamepad1.b) {
            side = FieldSide.RED;
        } else if (gamepad1.x) {
            side = FieldSide.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Side", side);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    private enum AutoStep {
        LAUNCH,
        DRIVE_1,
        DRIVE_2,
        TURN_1,
        TURN_2,
        DRIVE_BACKWARDS,
        DONE,
    }

    AutoStep autoStep = AutoStep.DRIVE_1;
    ElapsedTime elapsedTimer = new ElapsedTime();
    static final double LAUNCH_TIME_SECONDS = 13.0;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (autoStep) {
            case DRIVE_1:
                if (driveDistanceByTime(DRIVE_SPEED, DRIVE_SPEED, 6)) {
                    //setLauncher(GOAL_SPEED, FULL_SPEED);
                    elapsedTimer.reset();
                    autoStep = AutoStep.TURN_1;
                }
                break;
            case TURN_1:
                if (side == FieldSide.BLUE) {
                    if (driveDistanceByTime(-TURN_SPEED, TURN_SPEED, 0.5)) {
                        autoStep = AutoStep.DRIVE_2;
                    }
                } else {
                    if (driveDistanceByTime(TURN_SPEED, -TURN_SPEED, 0.5)) {
                        autoStep = AutoStep.DRIVE_2;
                    }
                }
                break;
            case DRIVE_2:
                //spinUpFlywheel();
                if (driveDistanceByTime(DRIVE_SPEED, DRIVE_SPEED, 4.5)) {
                    autoStep = AutoStep.LAUNCH;
                }
                break;
            case LAUNCH:
                setLauncher(GOAL_SPEED, FULL_SPEED);
                if (elapsedTimer.seconds() > LAUNCH_TIME_SECONDS) {
                    setLauncher(STOP_SPEED, STOP_SPEED);
                    autoStep = AutoStep.TURN_2;
                }
                break;
            case TURN_2:
                if (side == FieldSide.BLUE) {
                    if (driveDistanceByTime(TURN_SPEED, -TURN_SPEED, 0.4)) {
                        autoStep = AutoStep.DRIVE_BACKWARDS;
                    }
                } else {
                    if (driveDistanceByTime(-TURN_SPEED, TURN_SPEED, 0.4)) {
                        autoStep = AutoStep.DRIVE_BACKWARDS;
                    }
                }
                break;
            case DRIVE_BACKWARDS:
                if (driveDistanceByTime(-0.67, -0.67, 6.0)) {
                    autoStep = AutoStep.DONE;
                }
                break;
            case DONE:
                break;
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    enum DriveState {
        IDLE,
        DRIVING
    }

    double driveTimeout;
    DriveState driveState = DriveState.IDLE;

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

    /*
     * This sets the 1 flywheel motor, 1 back spin CR servo and the 2 index CR servos to the
     * given power.
     */
    public void setLauncher(double flyPower, double servoPower) {
        flyWheel.setPower(flyPower);
        backSpin.setPower(servoPower);
        indexLeft.setPower(-servoPower);
        indexRight.setPower(-servoPower);
    }

    public void spinUpFlywheel() {
        flyWheel.setPower(FULL_SPEED);
    }
}
