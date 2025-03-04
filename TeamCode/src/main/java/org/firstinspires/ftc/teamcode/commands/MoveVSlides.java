package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveVSlides extends SequentialCommandGroup {
    public MoveVSlides(RobotHardware robot, int pos) {
        super(
                new InstantCommand(() -> robot.slideLeftActuator.setTargetPosition(pos)),
                new InstantCommand(() -> robot.slideRightActuator.setTargetPosition(pos))
        );
    }

    @Override
    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        return instance.slideLeftActuator.hasReached() && instance.slideRightActuator.hasReached();
    }
}
