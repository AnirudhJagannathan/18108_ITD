package org.firstinspires.ftc.teamcode.subsystems;

import android.telephony.CellIdentity;
import android.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake {
    // private CRServo intake;
    private DcMotorEx intake;
    private Servo intake1;
    private Servo intake2;
    private OpMode opmode;
    private Limelight3A limelight;

    public double skibAdjust = 0;

    // private final double UP_POSITION = ;


    public Intake(HardwareMap hardwareMap, OpMode opmode) {
        intake = hardwareMap.get(DcMotorEx.class, "spintake");
        intake1 = hardwareMap.get(Servo.class, "intake1");
        intake2 = hardwareMap.get(Servo.class, "intake2");

        this.opmode = opmode;
    }


    public void setSkibAdjust() {
        if (opmode.gamepad2.start) {
            skibAdjust += 0.01;
        }

        if (opmode.gamepad2.back) {
            skibAdjust -= 0.01;
        }
    }

    public void intake(double power) {
        intake.setPower(-power);
    }

    public void outtake(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }

    public void flipUp() {
        intake1.setPosition(0.54 + skibAdjust);
        intake2.setPosition(0.46 - skibAdjust);
    }

    public void flipDown() {
        intake1.setPosition(0.37 + skibAdjust);
        intake2.setPosition(0.63 - skibAdjust);
    }

    public void slideDown() {
        intake1.setPosition(0.26);
        intake2.setPosition(0.64);
    }

    public double[] getPosition() {
        return new double[]{intake1.getPosition(), intake2.getPosition()};
    }

    public void cutPower() {
        ((PwmControl) intake1).setPwmDisable();
        ((PwmControl) intake2).setPwmDisable();
    }

    public void flipMid() {
        intake1.setPosition(0.39);
        intake2.setPosition(0.61);
    }

    public void farUp() {
        intake1.setPosition(0.70);
        intake2.setPosition(0.30);
    }


    public double[] getLow() {
        return new double[]{0.37 + skibAdjust, 0.63 - skibAdjust};
    }
}
