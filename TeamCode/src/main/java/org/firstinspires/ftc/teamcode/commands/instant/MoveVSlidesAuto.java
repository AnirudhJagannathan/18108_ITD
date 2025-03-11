package org.firstinspires.ftc.teamcode.commands.instant;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.PIDController;

public class MoveVSlidesAuto extends CommandBase {
    private final int target;

    private final int TOLERANCE = 50;

    public MoveVSlidesAuto(int target) {
        this.target = target;
    }

    public void initialize() {
        RobotHardware.getInstance().slides.setTargetPosition(-600);
    }

    @Override
    public void execute() {
        RobotHardware instance = RobotHardware.getInstance();
        instance.slideLeftActuator.setCurrentPosition(instance.slideLeft.getCurrentPosition());
        instance.slideRightActuator.setCurrentPosition(instance.slideRight.getCurrentPosition());

        // instance.slideLeftActuator
    }

    @Override
    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        return Math.abs(target - instance.slideLeftActuator.getPosition()) < TOLERANCE
                && Math.abs(target - instance.slideRightActuator.getPosition()) < TOLERANCE;
    }
}
