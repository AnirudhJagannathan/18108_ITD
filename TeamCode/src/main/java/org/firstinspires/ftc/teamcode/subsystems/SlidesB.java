package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SlidesB {
    private DcMotorEx vSlidesA;
    private DcMotorEx vSlidesB;
    // private Servo hSlidesA;
    // private Servo hSlidesB;
    private DcMotorEx hSlides;
    private OpMode opMode;

    private double MIDPOINT = 0.5;
    private double targetPosition = 0;
    private double previousTarget = 0;

    private final int TOLERANCE = 75;



    public SlidesB(HardwareMap hardwareMap, OpMode opMode) {
        /* hSlidesA = hardwareMap.get(Servo.class, "HSlidesA");
        hSlidesB = hardwareMap.get(Servo.class, "HSlidesB");
         */

        vSlidesA = hardwareMap.get(DcMotorEx.class, "VSlidesA");
        vSlidesB = hardwareMap.get(DcMotorEx.class, "VSlidesB");
        hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        vSlidesA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlidesB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vSlidesB.setDirection(DcMotorSimple.Direction.REVERSE);

        hSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        this.opMode = opMode;
    }


    public void moveVSlidesPID(double targetPosition) {
        PIDController control = new PIDController(4,0,0.15);
        double maxError = targetPosition - previousTarget;
        vSlidesA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* if (Math.abs(targetPosition - getVSlidesPos()) > 30) {
            double power = control.update(targetPosition, getVSlidesPos()) / Math.abs(maxError);
            vSlidesB.setPower(power);
        } else {
            vSlidesB.setPower(0)
            previousTarget = targetPosition;
        }
         */

        if (Math.abs(targetPosition - getVSlidesPos()) <= 75 && getVSlidesPos() < 220) {
            vSlidesA.setPower(0);
            vSlidesB.setPower(0);
        } else {
            double power = control.update(targetPosition, getVSlidesPos()) / Math.abs(maxError);
            vSlidesA.setPower(power);
            vSlidesB.setPower(power);
        }

        //opMode.telemetry.addData("slidePos", getVSlidesPos());
        //opMode.telemetry.update();
    }

    public void moveHSlidesPID(double targetPosition) {
        PIDController control = new PIDController(4,0,0.15);
        double maxError = targetPosition - previousTarget;
        double derivative = control.getDerivative();
        hSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* if (Math.abs(targetPosition - getVSlidesPos()) > 30) {
            double power = control.update(targetPosition, getVSlidesPos()) / Math.abs(maxError);
            vSlidesB.setPower(power);
        } else {
            vSlidesB.setPower(0);
            previousTarget = targetPosition;
        }
         */
        double power = control.update(targetPosition, hSlides.getCurrentPosition()) / Math.abs(maxError);
        hSlides.setPower(power);
        if (Math.abs(targetPosition - hSlides.getCurrentPosition()) <= 75 && hSlides.getCurrentPosition() > -220 && derivative < 0) {
            hSlides.setPower(0);
        }
    }


    public void moveVSlidesPIDTeleOp() {
        int minTargetPosition = -3250;  // Lower limit of the vertical slide
        int maxTargetPosition = 0;
        PIDController control = new PIDController(0.02, 0, 0.0);
        double joystickInput = 60 * opMode.gamepad2.right_stick_y;

        vSlidesA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition += joystickInput;
        if (targetPosition > maxTargetPosition) {
            targetPosition = maxTargetPosition;
        } else if (targetPosition < minTargetPosition) {
            targetPosition = minTargetPosition;
        }

        double power = control.update(targetPosition, getVSlidesPos());
        vSlidesA.setPower(power);
        vSlidesB.setPower(power);
    }

    public void moveHSlides() {
        double power = Range.clip(opMode.gamepad2.left_stick_y, -1, 1);
        if (checkLimitsHSlide(power) && (getHSlidesPos() > -200 && power > 0)) {
            hSlides.setPower(0.7 * power);
        }
        else if (checkLimitsHSlide(power)) {
            hSlides.setPower(power);
        }
        else {
            hSlides.setPower(0);
        }
    }

    public Action moveVSlidesHeight(int height) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDController control = new PIDController(8,0,0.15);
                double maxError = height - previousTarget;
                vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (Math.abs(height - getVSlidesPos()) <= 75 && getVSlidesPos() < 220) {
                    vSlidesA.setPower(0);
                    vSlidesB.setPower(0);
                    opMode.telemetry.addData("cond met", true);
                    opMode.telemetry.update();
                } else {
                    double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
                    vSlidesA.setPower(power);
                    vSlidesB.setPower(power);
                }

                double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
                vSlidesA.setPower(power);
                vSlidesB.setPower(power);

                packet.put("slidePos:", getVSlidesPos());
                if (Math.abs(height - getVSlidesPos()) > TOLERANCE)
                    return true;
                else {
                    vSlidesA.setPower(0);
                    vSlidesB.setPower(0);
                    return false;
                }
            }
        };
    }

    public Action moveHSlidesLength(int length) {
        return new Action() {
            boolean initialized = false;
            double initPos = getVSlidesPos();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDController control = new PIDController(8,0,0.15);
                double maxError = length - previousTarget;
                hSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (Math.abs(length - hSlides.getCurrentPosition()) <= 75 && hSlides.getCurrentPosition() > -220) {
                    hSlides.setPower(0);
                    opMode.telemetry.addData("cond met", true);
                    opMode.telemetry.update();
                } else {
                    double power = control.update(length, hSlides.getCurrentPosition()) / Math.abs(length - getHSlidesPos());
                    hSlides.setPower(power);
                }

                double power = control.update(length, hSlides.getCurrentPosition()) / Math.abs(length - getHSlidesPos());
                hSlides.setPower(power);

                packet.put("slidePos:", getHSlidesPos());
                if (Math.abs(length - getHSlidesPos()) > 75)
                    return true;
                else {
                    hSlides.setPower(0);
                    return false;
                }
            } // Eaglederp is not those who know
            // Facts tho- blud

        }; // krishouts...
    }

    public boolean checkLimitsHSlide(double power) {
        return ((hSlides.getCurrentPosition() > -1550 || power > 0) && (hSlides.getCurrentPosition() < 50 || power < 0));
    }

    public int getVSlidesPos() {
        return (int) (vSlidesA.getCurrentPosition() + vSlidesB.getCurrentPosition()) / 2;
    }

    public int getHSlidesPos() {
        return hSlides.getCurrentPosition();
    }

    public void reset() {
        vSlidesA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vSlidesA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
