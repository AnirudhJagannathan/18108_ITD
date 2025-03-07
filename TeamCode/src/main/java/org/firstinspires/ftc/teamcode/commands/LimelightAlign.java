package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class LimelightAlign extends CommandBase {
    private final int pipelineNumber; // 0 is yellow, 1 is red, 2 is blue
    private final RobotHardware robot;

    public LimelightAlign(RobotHardware robot, int pipelineNumber) {
        this.robot = robot;
        this.pipelineNumber = pipelineNumber;
    }

    @Override
    public void initialize() {
        robot.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        robot.limelight.start(); // This tells Limelight to start looking!
        robot.limelight.pipelineSwitch(pipelineNumber);
    }

    @Override
    public void execute() {
        LLResult result = robot.limelight.getLatestResult();

        double Kp = -0.06f;
        double min_command = 0.03;

        if (result != null && result.isValid()) {
            robot.setTx(result.getTx());
        } else {
            robot.setTx(0);
        }

        double steering_adjust = 0.0;
        if (Math.abs(-robot.getTx()) >= 0.5) {
            if (-robot.getTx() < 0) {
                steering_adjust = Kp * (-robot.getTx()) + min_command;
            } else {
                steering_adjust = Kp * (-robot.getTx()) - min_command;
            }
            robot.getMotors()[0].setPower(steering_adjust);
            robot.getMotors()[1].setPower(-steering_adjust);
            robot.getMotors()[2].setPower(steering_adjust);
            robot.getMotors()[3].setPower(-steering_adjust);
        }
        else {
            robot.getMotors()[0].setPower(0);
            robot.getMotors()[1].setPower(0);
            robot.getMotors()[2].setPower(0);
            robot.getMotors()[3].setPower(0);
        }
    }

    public boolean isFinished() {
        return Math.abs(-robot.getTx()) < 0.5;
    }
}
