package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Hanging {
    private LinearOpMode opMode;
    // private DcMotor intake;
    // private DcMotor intake2;
    //test
    private CRServo wheel;
    private DcMotorEx tapeMeasure;

    private final int TOLERANCE = 15;

    private final double ratioServoMotor = 7.8;
    private final double inverseRatioServoMotor = 1/ratioServoMotor;

    public Hanging(HardwareMap hardwareMap, LinearOpMode opMode) {

        tapeMeasure = hardwareMap.get(DcMotorEx.class, "tapeMeasure");
        wheel = (hardwareMap.get(CRServo.class,"wheel"));
        tapeMeasure.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.opMode = opMode;

    }

    public void moveHangOut(double power) {
        tapeMeasure.setPower(0.9*power);
        wheel.setPower(inverseRatioServoMotor*power);
    }
    public void moveHangIn(double power) {
        tapeMeasure.setPower(-1*power);
        wheel.setPower(inverseRatioServoMotor*-0.9*power);
    }

}


