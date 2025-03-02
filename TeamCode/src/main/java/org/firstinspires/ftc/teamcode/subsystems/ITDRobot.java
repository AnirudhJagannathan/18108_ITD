package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ITDRobot {
    private OpMode opmode;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    public final double MAX_VOLTAGE = 14.2;

    public ITDRobot(HardwareMap hardwareMap, OpMode opmode) {
        this.opmode = opmode;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void mecanumDriving(double maxSpeed) {
        double drive = opmode.gamepad1.left_stick_y;
        double strafe = opmode.gamepad1.right_stick_x;
        double turn = opmode.gamepad1.left_stick_x;
        double v1, v2, v3, v4;

        if (opmode.gamepad1.right_bumper) {
            v1 = Range.clip(-drive + strafe + turn, -0.25, 0.25);
            v2 = Range.clip(-drive - strafe - turn, -0.25, 0.25);
            v3 = Range.clip(-drive + strafe - turn, -0.25, 0.25);
            v4 = Range.clip(-drive - strafe + turn, -0.25, 0.25);
        }

        else {
            v1 = Range.clip(-drive + strafe + turn, -maxSpeed, maxSpeed);
            v2 = Range.clip(-drive - strafe - turn, -maxSpeed, maxSpeed);
            v3 = Range.clip(-drive + strafe - turn, -maxSpeed, maxSpeed);
            v4 = Range.clip(-drive - strafe + turn, -maxSpeed, maxSpeed);
        }

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
    }

    public DcMotorEx[] getMotors() {
        return new DcMotorEx[] {leftFront, rightFront, leftBack, rightBack};
    }
}