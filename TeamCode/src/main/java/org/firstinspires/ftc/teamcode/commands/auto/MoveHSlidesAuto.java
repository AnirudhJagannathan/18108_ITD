package org.firstinspires.ftc.teamcode.commands.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.MoveHSlides;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveHSlidesAuto extends CommandBase {

    private int target;
    private double time;
    private ElapsedTime timer;
    private final int TOLERANCE = 50;

    public MoveHSlidesAuto(int target, double time) {
        this.target = target;
        this.time = time;
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void initialize() {
        timer.reset();
        RobotHardware.getInstance().slides.setTargetPositionH(target);
    }

    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        if (target < 25 && Math.abs(target - instance.hSlideActuactor.getPosition()) < TOLERANCE || timer.time() >= time)
            instance.hSlides.setPower(0);
        return Math.abs(target - instance.hSlideActuactor.getPosition()) < TOLERANCE || timer.time() >= time;
    }

}
