package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    private Servo wrist;
    private Servo swerve;
    private OpMode opmode;
    private double position;
    private double position_wrist;

    public Claw(HardwareMap hardwareMap, OpMode opmode){
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        swerve = hardwareMap.get(Servo.class, "swerve");
        position = 1;
        position_wrist = 0;
        this.opmode = opmode;
    }

    public void grab() {

        if (opmode.gamepad2.right_trigger > 0)
            position = 0.21;
        else if (opmode.gamepad2.left_trigger > 0)
            position = 0.42;
        claw.setPosition(position);
        opmode.telemetry.addData("Position: ", position);
    }
    public void open(){
        claw.setPosition(0.37);
    }

    public void close(){
        claw.setPosition(0.2);
    }

    public void wideOpen() {
        claw.setPosition(0.60);
    }

    public void wrist() {
        if (opmode.gamepad1.dpad_right || opmode.gamepad2.dpad_right)
            wrist.setPosition(1);

        if (opmode.gamepad1.dpad_left || opmode.gamepad2.dpad_left) {
            wrist.setPosition(0.0);
            claw.setPosition(0.6);
        }

        swerve.setPosition(0);

        opmode.telemetry.addData("Wrist", position_wrist);
        opmode.telemetry.update();
    }

    public void manualWrist() {
        if (opmode.gamepad2.start)
            wrist.setPosition(wrist.getPosition() + 0.01);
        if (opmode.gamepad2.back)
            wrist.setPosition(wrist.getPosition() - 0.01);
        opmode.telemetry.addData("Freaky Eagle", wrist.getPosition());
    }

    public void rotateDown() {
        wrist.setPosition(0.06);
    }
    public void rotateUp() {
        wrist.setPosition(0.49);
    }
    public void rotateBack() {
        wrist.setPosition(0.80);
    }

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