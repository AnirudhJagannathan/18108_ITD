package org.firstinspires.ftc.teamcode.teleopsubs;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.DoubleSupplier;

public class ITDRobot extends SubsystemBase {
    private OpMode opmode;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    private IMU imu;

    public final double MAX_VOLTAGE = 14.2;

    public ITDRobot(HardwareMap hardwareMap, OpMode opmode) {
        this.opmode = opmode;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void mecanumDriving(DoubleSupplier d, DoubleSupplier s, DoubleSupplier t) {
        double drive = d.getAsDouble();
        double strafe = s.getAsDouble();
        double turn = t.getAsDouble();
        double v1, v2, v3, v4;

        if (opmode.gamepad1.right_bumper) {
            v1 = Range.clip(-drive + strafe + turn, -0.25, 0.25);
            v2 = Range.clip(-drive - strafe - turn, -0.25, 0.25);
            v3 = Range.clip(-drive + strafe - turn, -0.25, 0.25);
            v4 = Range.clip(-drive - strafe + turn, -0.25, 0.25);
        }

        else {
            v1 = Range.clip(-drive + strafe + turn, -1, 1);
            v2 = Range.clip(-drive - strafe - turn, -1, 1);
            v3 = Range.clip(-drive + strafe - turn, -1, 1);
            v4 = Range.clip(-drive - strafe + turn, -1, 1);
        }

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);
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

    public void fieldCentric(DoubleSupplier s, DoubleSupplier t, DoubleSupplier d) {
        double drive = -d.getAsDouble();
        double turn = t.getAsDouble();
        double strafe = -s.getAsDouble();

        if (opmode.gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        // rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (-rotY + rotX + turn) / denominator;
        double frontRightPower = (-rotY - rotX - turn) / denominator;
        double backLeftPower = (-rotY - rotX + turn) / denominator;
        double backRightPower = (-rotY + rotX - turn) / denominator;

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }

    public DcMotorEx[] getMotors() {
        return new DcMotorEx[] {leftFront, rightFront, leftBack, rightBack};
    }

    public void periodic() {
        // empty
    }
}
