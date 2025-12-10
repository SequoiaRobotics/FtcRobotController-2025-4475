package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

public class DrivePovTask<OpModeT extends AbstractOpMode<OpModeT>> extends DriveTankTask<OpModeT> {
    @Override
    public DrivePovTask<OpModeT> run() {
        leftPower  = Range.clip(speed - turnRate, -1.0, 1.0);
        rightPower = Range.clip(speed + turnRate, -1.0, 1.0);

        super.run();

        return this;
    }

    @Override
    public DrivePovTask<OpModeT> init() {
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "DrivePovTask init: start");
        super.init();
        this.frequencyMillis(10); // adjust motors every 10ms, 100/second
        RobotLog.ii(AbstractOpMode.GORILLA_CORE, "DrivePovTask init: done");
        return this;
    }

    public double speed;

    public double turnRate;
} // class DrivePovTask
