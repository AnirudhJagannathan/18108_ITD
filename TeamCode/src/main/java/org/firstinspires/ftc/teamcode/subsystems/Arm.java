package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class Arm {
    private Servo arm;
    private OpMode opmode;

    private double newPos;

    public Arm(HardwareMap hardwareMap, OpMode opmode) {
        arm = hardwareMap.get(Servo.class, "arm");
        this.opmode = opmode;
    }

    public void rotateUp() {
        double pos = arm.getPosition();
        for (double i = pos; i >= 0; i -= 0.01)
            arm.setPosition(i);

        // arm.setPosition(0);
    }

    public void upward() {
        arm.setPosition(0);
    }


    public void rotateUpPID() {
        PIDController control = new PIDController(0.05, 0, 0);
        double increment = 0.005;
        while (arm.getPosition() >= 0) {
            double power = control.update(0, arm.getPosition());
            arm.setPosition(increment * power);
        }
    }

    /*public void rotatetoPointPID(double goal) {
        PIDController control = new PIDController(0.05, 0, 0);
        while (15>=Math.abs(arm.getPosition()-goal)) {
            arm.setPosition()
        }
    }*/

    public void rotateDown() {
        double pos = arm.getPosition();
        for (double i = pos; i <= 0.92; i += 0.01)
            arm.setPosition(i);
    }

    public void downward(){
        arm.setPosition(0.92);
    }

    public void rotateDownPID() {
        PIDController control = new PIDController(0.05, 0, 0);
        double increment = -0.005;
        while (arm.getPosition() <= 1.0) {
            double power = control.update(1.0, arm.getPosition());
            arm.setPosition(increment * power);
        }
    }


    public void rotatePID(double position) {
        PIDController control = new PIDController(0.05, 0, 0);
        if (Math.abs(position - arm.getPosition()) >= 0.05) {
            double power = control.update(position, arm.getPosition() / Math.abs(position - arm.getPosition()));
            arm.setPosition(power);
            opmode.telemetry.addData("power", power);
        }
        else {
            arm.setPosition(position);
        }
        double power = control.update(position, arm.getPosition() / Math.abs(position - arm.getPosition()));
        arm.setPosition(power);
        opmode.telemetry.addData("power", power);
        opmode.telemetry.update();
    }


    public void rotate(double pos) {
        PIDController control = new PIDController(2, 0, 2);
        double increment = 0.005;
        double power = control.update(pos, arm.getPosition());
        if (Math.abs(arm.getPosition() - pos) > 0.01)
            arm.setPosition(arm.getPosition() + power * increment);
    }

    public Action rotateAction(double newPos) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setPosition(newPos);
                return Math.abs(newPos - arm.getPosition()) > 0.01;
            }
        };
    }

    public class RotateBack implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setPosition(0.25);
            return Math.abs(0.25 - arm.getPosition()) > 0.01;
        }
    }

    public Action rotateBack() {
        return new RotateBack();
    }


    public class RotateDeposit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setPosition(0.50);
            return Math.abs(0.50 - arm.getPosition()) > 0.01;
        }
    }

    public Action rotateDeposit() {
        return new RotateDeposit();
    }

    public void deposit() {
        arm.setPosition(0.60);
    }

    public void back() {
        arm.setPosition(1.0);
    }

    public void front() {
        arm.setPosition(0.0);
    }

    public void intermediary() {
        arm.setPosition(0.70);
    }
}