package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.concurrent.TimeUnit;

public class PowerIntakeTime extends CommandBase {
    private double power;
    private double time;
    private ElapsedTime timer;
    public PowerIntakeTime(double power, double time) {
        this.power = power;
        this.time = time;
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void initialize() {
        timer.reset();
        RobotHardware.getInstance().spintake.intake(-power);
    }

    public boolean isFinished() {
        return timer.time() >= time;
    }
}
