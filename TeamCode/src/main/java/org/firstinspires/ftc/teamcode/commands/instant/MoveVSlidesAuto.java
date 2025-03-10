package org.firstinspires.ftc.teamcode.commands.instant;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveVSlidesAuto extends SequentialCommandGroup {

    private final int target;

    public MoveVSlidesAuto(int target) {
        super(
                new InstantCommand(() -> RobotHardware.getInstance().slides.setTargetPosition(target))
        );
        this.target = target;
    }

    public boolean isFinished() {
        int TOLERANCE = 100;
        return Math.abs(RobotHardware.getInstance().slides.getVSlidesPos() - target) < TOLERANCE;
    }
}
