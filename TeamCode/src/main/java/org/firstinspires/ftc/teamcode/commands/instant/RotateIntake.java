package org.firstinspires.ftc.teamcode.commands.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

public class RotateIntake extends InstantCommand {
    public RotateIntake(Intake.IntakeState state) {
        super(() -> RobotHardware.getInstance().spintake.updateState(state));
    }
}
