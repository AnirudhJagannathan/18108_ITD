package org.firstinspires.ftc.teamcode.commands.instant;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;

public class RotateFEDHES extends InstantCommand {
    public RotateFEDHES(FEDHES.FEDHESState state) {
        super(() -> RobotHardware.getInstance().fedhes.updateState(state)); // TDHES pain remover
    }
}
