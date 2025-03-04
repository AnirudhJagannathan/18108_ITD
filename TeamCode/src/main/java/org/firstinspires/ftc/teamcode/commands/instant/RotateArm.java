package org.firstinspires.ftc.teamcode.commands.instant;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;

public class RotateArm extends InstantCommand {
    public RotateArm(Arm.ArmState state) {
        super(() -> RobotHardware.getInstance().arm.updateState(state));
    }
}
