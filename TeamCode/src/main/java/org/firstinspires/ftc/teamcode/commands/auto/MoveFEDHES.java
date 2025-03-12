package org.firstinspires.ftc.teamcode.commands.auto;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateFEDHES;
import org.firstinspires.ftc.teamcode.commands.instant.RotateYaw;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;

public class MoveFEDHES extends SequentialCommandGroup {
    public MoveFEDHES(Claw.YawState yawState, FEDHES.FEDHESState fedhesState, Arm.ArmState armState) {
        super(
                new ParallelCommandGroup(
                        new RotateYaw(yawState),
                        new RotateFEDHES(fedhesState),
                        new RotateArm(armState)
                )
        );
    }
}
