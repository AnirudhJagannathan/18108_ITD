package org.firstinspires.ftc.teamcode.commands.instant;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;

public class ToggleClaw extends InstantCommand {
    public ToggleClaw(Claw.ClawState state) {
        super(() -> RobotHardware.getInstance().claw.updateState(state));
    }
}
