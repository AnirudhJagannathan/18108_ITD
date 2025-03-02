package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmB {

    private Servo arm1;
    private Servo arm2;
    private OpMode opMode;

    public ArmB(HardwareMap hardwareMap, OpMode opMode) {
        arm1 = hardwareMap.get(Servo.class, "arm");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        this.opMode = opMode;
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

        double delta1 = (0 - pos1) / 3;
        double delta2 = (1 - pos2) / 3;

        for (int i = 1; i <= 3; i++) {
            arm1.setPosition(pos1 + i * delta1);
            arm2.setPosition(pos2 + i * delta2);
        }

        arm1.setPosition(0);
        arm2.setPosition(1);
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
}
