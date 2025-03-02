package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawB {
    private Servo claw;
    private Servo swerve;
    private OpMode opmode;
    private double position;
    private double position_wrist;

    public ClawB(HardwareMap hardwareMap, OpMode opmode){
        claw = hardwareMap.get(Servo.class, "claw");
        swerve = hardwareMap.get(Servo.class, "swerve");
        position = 0;
        position_wrist = 0;
        this.opmode = opmode;
    }

    public void swerve(double pos) {
        swerve.setPosition(pos);
    }

    public void grab() {
        if (opmode.gamepad2.right_trigger > 0)
            position = 0.0;
        else if (opmode.gamepad2.left_trigger > 0)
            position = 0.18;
        claw.setPosition(position);
        opmode.telemetry.addData("Position: ", position);
    }
    public void open(){
        claw.setPosition(0.13);
    }

    public void close() {
        claw.setPosition(0.035);
    }

    public void fullClose() {
        claw.setPosition(0.01);
    }

    public void wideOpen() {
        claw.setPosition(0.18);
    }

    /* public void rotateDown() {
        wrist.setPosition(0.06);
    }
    public void rotateUp() {
        wrist.setPosition(0.49);
    }
    public void rotateBack() {
        wrist.setPosition(0.80);
    }
     */

    public Action closeClaw() {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.close();
                return false;
            }
        };
    }
}