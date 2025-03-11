package org.firstinspires.ftc.teamcode.commands.auto;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.MoveHSlides;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveHSlidesAuto extends CommandBase {

    private int target;
    private final int TOLERANCE = 50;

    public MoveHSlidesAuto(int target) {
        this.target = target;
    }

    public void initialize() {
        RobotHardware.getInstance().slides.setTargetPositionH(target);
    }

    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        return Math.abs(target - instance.hSlideActuactor.getPosition()) < TOLERANCE;
    }

}
