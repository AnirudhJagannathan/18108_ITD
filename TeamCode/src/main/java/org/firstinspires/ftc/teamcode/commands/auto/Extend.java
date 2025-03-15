package org.firstinspires.ftc.teamcode.commands.auto;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateFEDHES;
import org.firstinspires.ftc.teamcode.commands.instant.RotateYaw;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;

public class Extend extends SequentialCommandGroup {
    public Extend(Claw.YawState yawState, FEDHES.FEDHESState fedhesState) {
        super(
                new RotateYaw(yawState),
                new RotateFEDHES(fedhesState),
                new RotateArm(Arm.ArmState.SPEC)
        );
    }
}
