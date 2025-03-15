package org.firstinspires.ftc.teamcode.commands.auto;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateFEDHES;
import org.firstinspires.ftc.teamcode.commands.instant.RotateYaw;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;

public class Retract extends SequentialCommandGroup {
    public Retract(Claw.YawState yawState, Arm.ArmState armState) {
        super(
                new SequentialCommandGroup(
                        new RotateArm(armState),
                        new RotateFEDHES(FEDHES.FEDHESState.BACK),
                        new RotateYaw(yawState)
                )
        );
    }
}
