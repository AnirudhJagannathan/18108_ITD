package org.firstinspires.ftc.teamcode.teleopsubs;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

public class Claw extends WSubsystem {

    private final RobotHardware robot = RobotHardware.getInstance();
    private Servo claw;
    private Servo swerve;
    private double position;
    private double position_wrist;

    public enum ClawState {
        OPEN,
        CLOSED,
        WIDE_OPEN
    }

    public enum YawState {
        LEFT,
        RIGHT,
        CENTER
    }

    public Claw(HardwareMap hardwareMap) {
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

    public void updateState(ClawState state) {
        switch (state) {
            case OPEN:
                robot.clawServo.setPosition(0.25);
                break;
            case CLOSED:
                robot.clawServo.setPosition(0.1);
                break;
            case WIDE_OPEN:
                robot.clawServo.setPosition(0.30);
                break;
        }
    }

    public void yawUpdateState(YawState state) {
        switch (state) {
            case LEFT:
                robot.yawServo.setPosition(0.40);
                break;
            case RIGHT:
                robot.yawServo.setPosition(0.60);
                break;
            case CENTER:
                robot.yawServo.setPosition(0.5);
        }
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
