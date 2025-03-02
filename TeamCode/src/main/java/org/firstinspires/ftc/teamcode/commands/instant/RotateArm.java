package org.firstinspires.ftc.teamcode.commands.instant;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class RotateArm extends InstantCommand {
    public RotateArm(double pos1, double pos2) {
        super(() -> RobotHardware.getInstance().arm.setPosition(pos1, pos2));
    }
}
