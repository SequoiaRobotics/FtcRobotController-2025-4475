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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp

public class MyTeleOp extends OpMode {
    static final double FULL_SPEED = 1.0;
    static final double GOAL_SPEED = 0.8;
    static final double STOP_SPEED = 0.0;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor flyWheel = null;
    private CRServo backSpin = null;
    private CRServo indexLeft = null;
    private CRServo indexRight = null;

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
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        if (gamepad1.a) {
            setLauncher(GOAL_SPEED, FULL_SPEED);
        } else {
            setLauncher(STOP_SPEED, STOP_SPEED);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * This method does the math and sets the power to motors for
     * an arcade drive.
     */
    public void arcadeDrive(double forward, double rotate) {
        leftDrive.setPower(forward + rotate);
        rightDrive.setPower(forward - rotate);
    }

    /*
     * This sets the 1 flywheel motor, 1 back spin CR servo and the 2 index CR servos to the
     * given power.
     */
    public void setLauncher(double flyPower, double servoPower) {
        flyWheel.setPower(2*flyPower);
        backSpin.setPower(2*servoPower);
        indexLeft.setPower(-servoPower);
        indexRight.setPower(-servoPower);
    }
}
