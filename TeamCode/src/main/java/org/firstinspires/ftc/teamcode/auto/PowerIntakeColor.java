package org.firstinspires.ftc.teamcode.auto;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class PowerIntakeColor extends CommandBase {

    private double power;
    private int alliance;
    public PowerIntakeColor(double power, int alliance) {
        this.power = power;
        this.alliance = alliance;
    }

    public void initialize() {
        RobotHardware.getInstance().spintake.intake(-power);
    }

    public boolean isFinished() {
        if (alliance == 0)
            return RobotHardware.getInstance().color.hsvValues[0] > 180;
        return RobotHardware.getInstance().color.hsvValues[0] > 0 && RobotHardware.getInstance().color.hsvValues[0] < 55;
    }
}
