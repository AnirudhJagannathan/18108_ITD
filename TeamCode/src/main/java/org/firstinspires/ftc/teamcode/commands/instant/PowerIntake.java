package org.firstinspires.ftc.teamcode.commands.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class PowerIntake extends InstantCommand {
    public PowerIntake(double power) {
        super(() -> RobotHardware.getInstance().spintake.intake(power));
    }
}
