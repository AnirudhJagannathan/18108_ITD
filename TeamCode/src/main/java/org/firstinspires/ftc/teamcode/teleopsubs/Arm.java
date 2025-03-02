package org.firstinspires.ftc.teamcode.teleopsubs;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.universal.PIDController;
import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

public class Arm extends WSubsystem {

    private final RobotHardware robot = RobotHardware.getInstance();
    private Servo arm1;
    private Servo arm2;
    private double pos1;
    private double pos2;

    private double init1;
    private double init2;

    private static Arm instance;

    private final double TOLERANCE = 0.005;

    public Arm(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(Servo.class, "arm");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        pos1 = 0.85;
        pos2 = 0.15;

        init1 = 0.85;
        init2 = 0.15;
    }

    public void setPosition(double pos1, double pos2) {
        this.pos1 = pos1;
        this.pos2 = pos2;

        arm1.setPosition(pos1);
        arm2.setPosition(pos2);
    }

    public void deposit() {
        double pos1 = arm1.getPosition();
        double pos2 = arm2.getPosition();

        double delta1 = (0.1 - pos1) / 3;
        double delta2 = (0.9 - pos2) / 3;

        for (int i = 1; i <= 3; i++) {
            arm1.setPosition(pos1 + i * delta1);
            arm2.setPosition(pos2 + i * delta2);
        }

        arm1.setPosition(0.10);
        arm2.setPosition(0.90);
    }

    public void back() {
        double pos1 = arm1.getPosition();
        double pos2 = arm2.getPosition();

        double delta1 = (0.03 - pos1) / 3;
        double delta2 = (0.97 - pos2) / 3;

        for (int i = 1; i <= 3; i++) {
            arm1.setPosition(pos1 + i * delta1);
            arm2.setPosition(pos2 + i * delta2);
        }

        arm1.setPosition(0.03);
        arm2.setPosition(0.97);
    }

    public void front() {
        double pos1 = arm1.getPosition();
        double pos2 = arm2.getPosition();

        double delta1 = (0.85 - pos1) / 3;
        double delta2 = (0.15 - pos2) / 3;

        for (int i = 1; i <= 3; i++) {
            arm1.setPosition(pos1 + i * delta1);
            arm2.setPosition(pos2 + i * delta2);
        }

        arm1.setPosition(0.85);
        arm2.setPosition(0.15);
    }

    public void intermediary() {
        double pos1 = arm1.getPosition();
        double pos2 = arm2.getPosition();

        double delta1 = (0.27 - pos1) / 3;
        double delta2 = (0.1 - pos2) / 3;

        for (int i = 1; i <= 3; i++) {
            arm1.setPosition(pos1 + i * delta1);
            arm2.setPosition(pos2 + i * delta2);
        }

        arm1.setPosition(0.27);
        arm2.setPosition(0.10);
    }

    public double[] getPosition() {
        return new double[]{arm1.getPosition(), arm2.getPosition()};
    }

    public void periodic() {
        double delta1 = (pos1 - init1) / 10;
        double delta2 = (pos2 - init2) / 10;

        if (Math.abs(pos1 - arm1.getPosition()) > TOLERANCE || Math.abs(pos2 - arm2.getPosition()) > TOLERANCE) {
            for (int i = 1; i <= 10; i++) {
                arm1.setPosition(pos1 + i * delta1);
                arm2.setPosition(pos2 + i * delta2);
            }

            arm1.setPosition(pos1);
            arm2.setPosition(pos2);
        }
        else {
            ((PwmControl) arm1).setPwmDisable();
            ((PwmControl) arm2).setPwmDisable();
        }

        CommandScheduler.getInstance().run();
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }
}
