package org.firstinspires.ftc.teamcode.teleopsubs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

public class Claw extends WSubsystem {
    private Servo claw;
    private Servo swerve;
    private double position;
    private double position_wrist;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        swerve = hardwareMap.get(Servo.class, "swerve");
        position = 0;
        position_wrist = 0;
    }

    public void swerve(double pos) {
        swerve.setPosition(pos);
    }

    /* public void grab() {
        if (opmode.gamepad2.right_trigger > 0)
            position = 0.0;
        else if (opmode.gamepad2.left_trigger > 0)
            position = 0.18;
        claw.setPosition(position);
        // opmode.telemetry.addData("Position: ", position);
    }
     */

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

    public void periodic() {
        //empty
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }
}
