package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class PowerIntakeColor extends CommandBase {

    private double power;
    private int alliance;
    private double time;
    private ElapsedTime timer;
    public PowerIntakeColor(double power, int alliance, double time) {
        this.power = power;
        this.alliance = alliance;
        this.time = time;
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void initialize() {
        RobotHardware.getInstance().spintake.intake(-power);
        timer.reset();
    }

    public boolean isFinished() {
        if (alliance == 0)
            return RobotHardware.getInstance().color.hsvValues[0] > 180 || timer.time() >= time;
        return RobotHardware.getInstance().color.hsvValues[0] > 0 && RobotHardware.getInstance().color.hsvValues[0] < 55 || timer.time() >= time;
    }
}
