package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class AbstractDriveTask<OpModeT extends AbstractOpMode<OpModeT>> extends AbstractOpMode.AbstractOpModeTask<OpModeT> {

    public AbstractDriveTask<OpModeT> init() {
        driveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        driveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeftRear.setDirection(DcMotorSimple.Direction.FORWARD);

        return this;
    }

    public AbstractDriveTask<OpModeT> driveLeftRear(DcMotorEx value) {
        driveLeftRear = value;

        return this;
    }

    @SuppressWarnings("UnusedReturnValue")
    public AbstractDriveTask<OpModeT> driveRightRear(DcMotorEx value) {
        driveRightRear = value;

        return this;
    }

    public AbstractDriveTask<OpModeT> driveLeftFront(DcMotorEx value) {
        driveLeftFront = value;

        return this;
    }

    public AbstractDriveTask<OpModeT> driveRightFront(DcMotorEx value) {
        driveRightFront = value;

        return this;
    }

    DcMotorEx driveLeftRear;

    DcMotorEx driveRightRear;

    DcMotorEx driveLeftFront;

    DcMotorEx driveRightFront;

} // abstract class AbstractDriveTask
