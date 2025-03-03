package org.firstinspires.ftc.teamcode.teleopsubs;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

public class Intake extends WSubsystem {
    private DcMotorEx intake;

    private NormalizedColorSensor colorSensor;
    private final RobotHardware robot;
    private Servo intake1, intake2;

    private double skibAdjust = 0;

    public enum IntakeState {
        UP,
        DOWN,
        MIDDLE
    }

    public Intake(HardwareMap hardwareMap) {
        /* intake = hardwareMap.get(DcMotorEx.class, "spintake");
        intake1 = hardwareMap.get(Servo.class, "intake1");
        intake2 = hardwareMap.get(Servo.class, "intake2");
        x */

        this.robot = RobotHardware.getInstance();
    }

    public void skibUp() {
        skibAdjust += 0.01;
    }

    public void skibDown() {
        skibAdjust -= 0.01;
    }

    public void intake(double power) {
        robot.intake.setPower(-power);
    }


    public void outtake(double power) {
        robot.intake.setPower(power);
    }

    public void stop() {
        robot.intake.setPower(0);
    }

    public void flipUp() {
        robot.intake1.setPosition(0.54 + skibAdjust);
        robot.intake2.setPosition(0.46 - skibAdjust);
    }

    public void flipDown() {
        robot.intake1.setPosition(0.37 + skibAdjust);
        robot.intake2.setPosition(0.63 - skibAdjust);
    }

    public void flip(boolean up) {
        if (up) {
            robot.intake1.setPosition(0.54 + skibAdjust);
            robot.intake2.setPosition(0.46 - skibAdjust);
        } else {
            robot.intake1.setPosition(0.37 + skibAdjust);
            robot.intake2.setPosition(0.63 - skibAdjust);
        }
    }

    public double[] getPosition() {
        return new double[]{robot.intake1.getPosition(), robot.intake2.getPosition()};
    }

    public void cutPower() {
        ((PwmControl) robot.intake1).setPwmDisable();
        ((PwmControl) robot.intake2).setPwmDisable();
    }

    public void flipMid() {
        robot.intake1.setPosition(0.39);
        robot.intake2.setPosition(0.61);
    }

    public void updateState(IntakeState state) {
        switch (state){
            case UP:
                robot.intake1.setPosition(0.54 + skibAdjust);
                robot.intake2.setPosition(0.46 - skibAdjust);
                break;
            case DOWN:
                robot.intake1.setPosition(0.37 + skibAdjust);
                robot.intake2.setPosition(0.63 - skibAdjust);
                break;
            case MIDDLE:
                robot.intake1.setPosition(0.39);
                robot.intake2.setPosition(0.61);
                break;
        }
    }

    @Override
    public void periodic() {
        // empty
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }
}
