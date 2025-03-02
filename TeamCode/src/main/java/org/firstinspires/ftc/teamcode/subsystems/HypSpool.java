package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HypSpool {
    private Servo hypLeft;
    private Servo hypRight;
    private OpMode opMode;

    public HypSpool(HardwareMap hardwareMap, OpMode opMode) {
        hypLeft = hardwareMap.get(Servo.class, "hypLeft");
        hypRight = hardwareMap.get(Servo.class, "hypRight");
        this.opMode = opMode;
    }

    public void rotate(double pos) {
        double pos1 = hypLeft.getPosition();
        double pos2 = hypRight.getPosition();

        double delta1 = (pos - pos1) / 5;
        double delta2 = ((1 - pos) - pos2) / 5;

        for (int i = 1; i <= 5; i++) {
            hypLeft.setPosition(pos1 + i * delta1);
            hypRight.setPosition(pos2 + i * delta2);
        }

        hypLeft.setPosition(pos);
        hypRight.setPosition(1 - pos);
    }
}
