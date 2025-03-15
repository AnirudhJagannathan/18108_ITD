package org.firstinspires.ftc.teamcode.teleopsubs;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

public class FEDHES extends WSubsystem { //You can see the pain in the TDHES

    private final RobotHardware robot = RobotHardware.getInstance();
    private Servo hypLeft;
    private Servo hypRight;

    private FEDHESState state;


    public double bludAdjust = 0;

    public FEDHES(HardwareMap hardwareMap) {
        hypLeft = hardwareMap.get(Servo.class, "hypLeft");
        hypRight = hardwareMap.get(Servo.class, "hypRight");
    }

    public enum FEDHESState {
        BACK,
        DOWN,
        FRONT,
        LESS_FRONT
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

    public void updateState(FEDHESState state) {
        switch (state) {
            case DOWN:
                robot.hypLeft.setPosition(0.32 + bludAdjust);
                robot.hypRight.setPosition(0.68 - bludAdjust);
                this.state = state;
                break;
            case BACK:
                robot.hypLeft.setPosition(0);
                robot.hypRight.setPosition(1);
                this.state = state;
                break;
            case FRONT:
                robot.hypLeft.setPosition(1);
                robot.hypRight.setPosition(0);
                this.state = state;
                break;
            case LESS_FRONT:
                robot.hypLeft.setPosition(0.6);
                robot.hypRight.setPosition(0.4);
        }
    }

    public void cutPower() {
        ((PwmControl) robot.hypLeft.getServo()).setPwmDisable();
        ((PwmControl) robot.hypRight.getServo()).setPwmDisable();
    }

    public FEDHESState getState() {
        return state;
    }

    @Override
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
