package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Slides;

public class MoveSlides extends SequentialCommandGroup {
    public MoveSlides(RobotHardware robot, int pos) {
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
