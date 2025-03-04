package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveHSlides extends SequentialCommandGroup {
    public MoveHSlides(RobotHardware robot, double pos) {
        super(
                new InstantCommand(() -> robot.hSlideActuactor.setTargetPosition(pos))
        );
    }

    @Override
    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        return instance.hSlideActuactor.hasReached();
    }
}
