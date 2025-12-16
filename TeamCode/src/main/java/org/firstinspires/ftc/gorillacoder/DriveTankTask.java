package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

// Tank Mode uses one stick to control each wheel.
@SuppressWarnings("unused")
public class DriveTankTask<OpModeT extends AbstractOpMode<OpModeT>> extends AbstractDriveTask<OpModeT>  {

    @Override
    public DriveTankTask<OpModeT> run() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() start: left:%f right:%f", getClass().getSimpleName(), leftPower, rightPower);

        // Send calculated power to wheels
        driveLeftRear.setPower(leftPower);
        driveLeftFront.setPower(leftPower);
        driveRightRear.setPower(rightPower);
        driveRightFront.setPower(rightPower);

        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "%s.run() done: left:%f right:%f", getClass().getSimpleName(), leftPower, rightPower);
        return this;
    }

    @Override
    public DriveTankTask<OpModeT> init() {
        super.init();
        this.frequencyMillis(10); // adjust motors every 10ms, 100/second
        return this;
    }

    public double leftPower;

    public double rightPower;

    // DO NOT USE YET. It does not work. Both the PovModeTask and the OpMode are setting DriveTankTask's
    // power levels. If PovModeTask is being used, then nothing besides it should be setting the DriveTankTask's
    // power levels. Need to figure out how to do that cleanly.
    //
    // TODO: Add a notion of dependence, so that DriveTankTask depends upon, and runs immediately after it PovModeTask.
    // Otherwise, DriveTankTask will be slow to respond to controls.
    //
    // The PovModeTask just adjusts the DriveTankTask's power levels.
    // It must run immediately before the DriveTankTask it is for.
    // Otherwise, there will be a delay up to frequency ms until DriveTankTask uses the new power levels.
    public PovModeTask createPovModeTask() {
        return new PovModeTask();
    }

    public class PovModeTask extends AbstractBotTask<OpMode> {
        @Override
        public PovModeTask run() {
            leftPower  = Range.clip(speed - turnRate, -1.0, 1.0);
            rightPower = Range.clip(speed + turnRate, -1.0, 1.0);

            return this;
        }

        public double speed;

        public double turnRate;

    } // class PovModeTask

} // class DriveTankTask
