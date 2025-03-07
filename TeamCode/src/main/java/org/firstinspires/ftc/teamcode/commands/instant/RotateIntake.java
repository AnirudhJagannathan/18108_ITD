package org.firstinspires.ftc.teamcode.commands.instant;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

public class RotateIntake extends CommandBase {

    private final Intake.IntakeState intakePos;
    public RotateIntake(Intake.IntakeState state) {
        intakePos = state;
    }

    @Override
    public void initialize() {
        RobotHardware.getInstance().spintake.updateState(intakePos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
