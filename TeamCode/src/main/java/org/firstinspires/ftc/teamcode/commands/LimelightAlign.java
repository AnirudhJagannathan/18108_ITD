package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class LimelightAlign extends CommandBase {

    private double tx;
    private final int pipelineNumber; // 0 is yellow, 1 is red, 2 is blue

    public LimelightAlign(int pipelineNumber) {
        this.pipelineNumber = pipelineNumber;
    }

    @Override
    public void initialize() {
        RobotHardware.getInstance().limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        RobotHardware.getInstance().limelight.start(); // This tells Limelight to start looking!

        RobotHardware.getInstance().limelight.pipelineSwitch(pipelineNumber);
    }

    @Override
    public void execute() {
        LLResult result = RobotHardware.getInstance().limelight.getLatestResult();

        double Kp = -0.05f;
        double min_command = 0.03;

        if (result != null && result.isValid()) {
            tx = result.getTx();
        } else {
            tx = 0;
        }

        double steering_adjust = 0.0;
        if (Math.abs(-tx) > 0.5) {
            if (-tx < 0) {
                steering_adjust = Kp * (-tx) + min_command;
            } else {
                steering_adjust = Kp * (-tx) - min_command;
            }
            RobotHardware.getInstance().getMotors()[0].setPower(steering_adjust);
            RobotHardware.getInstance().getMotors()[1].setPower(-steering_adjust);
            RobotHardware.getInstance().getMotors()[2].setPower(-steering_adjust);
            RobotHardware.getInstance().getMotors()[3].setPower(steering_adjust);
        }
        else {
            RobotHardware.getInstance().getMotors()[0].setPower(0);
            RobotHardware.getInstance().getMotors()[1].setPower(0);
            RobotHardware.getInstance().getMotors()[2].setPower(0);
            RobotHardware.getInstance().getMotors()[3].setPower(0);
        }
    }

    public boolean isFinished() {
        return Math.abs(-tx) < 0.5;
    }
}
