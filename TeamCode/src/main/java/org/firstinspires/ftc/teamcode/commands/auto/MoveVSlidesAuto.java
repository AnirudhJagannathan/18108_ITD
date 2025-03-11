package org.firstinspires.ftc.teamcode.commands.auto;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveVSlidesAuto extends CommandBase {
    private final int target;
    private final int TOLERANCE = 50;

    public MoveVSlidesAuto(int target) {
        this.target = target;
        RobotHardware.getInstance().setFinished(false);
    }

    public void initialize() {
        RobotHardware.getInstance().slides.setTargetPosition(target);
    }

    @Override
    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        if (Math.abs(target - instance.slideLeftActuator.getPosition()) < TOLERANCE
                && Math.abs(target - instance.slideRightActuator.getPosition()) < TOLERANCE) {
            RobotHardware.getInstance().setFinished(true);
            return true;
        }
        return false;
    }
}
