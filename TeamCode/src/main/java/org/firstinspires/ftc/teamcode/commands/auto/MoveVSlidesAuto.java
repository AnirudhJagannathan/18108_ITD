package org.firstinspires.ftc.teamcode.commands.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class MoveVSlidesAuto extends CommandBase {
    private final int target;
    private final int offset;
    private double time;
    private ElapsedTime timer;

    public MoveVSlidesAuto(int target, int offset, double time) {
        this.target = target;
        this.offset = offset;
        this.time = time;
        RobotHardware.getInstance().setFinished(false);
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void initialize() {
        timer.reset();
        RobotHardware.getInstance().slides.setTargetPosition(target + offset);
    }

    @Override
    public boolean isFinished() {
        RobotHardware instance = RobotHardware.getInstance();
        int TOLERANCE = 75;
        if ((Math.abs(target - instance.slideLeftActuator.getPosition()) < TOLERANCE
                && Math.abs(target - instance.slideRightActuator.getPosition()) < TOLERANCE) || timer.time() >= time) {
            RobotHardware.getInstance().setFinished(true);
            if (target > -25)
                instance.slides.setPower(0);
            return true;
        }
        return false;
    }
}
