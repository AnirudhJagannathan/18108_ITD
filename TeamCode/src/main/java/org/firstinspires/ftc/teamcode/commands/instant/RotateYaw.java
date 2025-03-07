package org.firstinspires.ftc.teamcode.commands.instant;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;

public class RotateYaw extends InstantCommand {
    public RotateYaw(Claw.YawState state) {
        super(() -> RobotHardware.getInstance().claw.yawUpdateState(state));
    }
}
