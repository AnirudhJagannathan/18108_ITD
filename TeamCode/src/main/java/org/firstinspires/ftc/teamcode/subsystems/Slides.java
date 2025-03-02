package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;
import android.text.style.ImageSpan;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {
    private DcMotorEx vSlidesA;
    private DcMotorEx vSlidesB;
    // private Servo hSlidesA;
    // private Servo hSlidesB;
    private DcMotorEx hSlides;
    private OpMode opMode;

    private double MIDPOINT = 0.5;
    private double targetPosition = 0;
    private double previousTarget = 0;

    private final int TOLERANCE = 25;
    private final int HSTOLERANCE = 150;


    public Slides(HardwareMap hardwareMap, OpMode opMode) {
        /* hSlidesA = hardwareMap.get(Servo.class, "HSlidesA");
        hSlidesB = hardwareMap.get(Servo.class, "HSlidesB");
         */

        vSlidesA = hardwareMap.get(DcMotorEx.class, "VSlidesA");
        vSlidesB = hardwareMap.get(DcMotorEx.class, "VSlidesB");
        hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        vSlidesA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlidesB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        this.opMode = opMode;
    }

    public void moveVSlides() {
        double power = opMode.gamepad2.right_stick_y;
        vSlidesA.setPower(power);
        vSlidesB.setPower(-power);
    }
    public void moveVSlidesPID(double targetPosition) {
        PIDController control = new PIDController(8,0,0.15);
        double maxError = targetPosition - previousTarget;
        vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* if (Math.abs(targetPosition - vSlidesB.getCurrentPosition()) > 30) {
            double power = control.update(targetPosition, vSlidesB.getCurrentPosition()) / Math.abs(maxError);
            vSlidesB.setPower(power);
        } else {
            vSlidesB.setPower(0)
            previousTarget = targetPosition;
        }
         */

        if (Math.abs(targetPosition - vSlidesB.getCurrentPosition()) <= 75 && vSlidesB.getCurrentPosition() > -220) {
            vSlidesB.setPower(0);
        } else {
            double power = control.update(targetPosition, vSlidesB.getCurrentPosition()) / Math.abs(maxError);
            vSlidesB.setPower(power);
        }
    }

    public void moveHSlidesPID(double targetPosition) {
        PIDController control = new PIDController(4,0,0.15);
        double maxError = targetPosition - previousTarget;
        double derivative = control.getDerivative();
        hSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* if (Math.abs(targetPosition - vSlidesB.getCurrentPosition()) > 30) {
            double power = control.update(targetPosition, vSlidesB.getCurrentPosition()) / Math.abs(maxError);
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
        int minTargetPosition = -3450;  // Lower limit of the vertical slide
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
        int currentPosition = (vSlidesA.getCurrentPosition() - vSlidesB.getCurrentPosition())/2;

        double power = control.update(targetPosition, currentPosition);
        vSlidesB.setPower(-power);
        vSlidesA.setPower(power);
        opMode.telemetry.addData("Freaky Eagle", power);
        opMode.telemetry.update();

    }

    public void moveHSlides() {
        double power = Range.clip(opMode.gamepad2.left_stick_y, -1, 1);
        if (checkLimitsHSlide(power)) {
            hSlides.setPower(power);
        }
        else {
            hSlides.setPower(0);
        }

        if (opMode.gamepad1.left_bumper) {
            moveVSlidesPID(500);
        }
    }
    public void moveHSlidesSigma() {
        double power = Range.clip(opMode.gamepad2.right_stick_x, -1, 1);
        if (checkLimitsHSlide(power)) {
            hSlides.setPower(power);
        }
        else {
            hSlides.setPower(0);
        }

        if (opMode.gamepad1.left_bumper) {
            moveVSlidesPID(500);
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
                vSlidesA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (Math.abs(height + getVSlidesPos()) <= 25 && getVSlidesPos() < -220) {
                    vSlidesB.setPower(0);
                    vSlidesA.setPower(0);
                } else {
                    double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
                    vSlidesB.setPower(-power);
                    vSlidesA.setPower(power);
                }



                double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
                vSlidesB.setPower(-power);
                vSlidesA.setPower(power);

                opMode.telemetry.addData("power", power);
                opMode.telemetry.update();

                packet.put("slidePos:", getVSlidesPos());
                if (Math.abs(height - getVSlidesPos()) > TOLERANCE)
                    return true;
                else {
                    vSlidesB.setPower(0);
                    vSlidesA.setPower(0);
                    return false;
                }
            }
        };
    }

    public void moveVSlidesHeightTeleOp(int height) {
        PIDController control = new PIDController(8,0,0.15);
        double maxError = height - previousTarget;
        vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(height + getVSlidesPos()) <= 25 && getVSlidesPos() < -220) {
            vSlidesB.setPower(0);
            vSlidesA.setPower(0);
        } else {
            double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
            vSlidesB.setPower(-power);
            vSlidesA.setPower(power);
        }



        double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
        vSlidesB.setPower(-power);
        vSlidesA.setPower(power);

        opMode.telemetry.addData("power", power);
        opMode.telemetry.update();
    }

    public Action moveVSlidesHeightDelay(int height, double delay) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDController control = new PIDController(8,0,0.15);
                double maxError = height - previousTarget;
                vSlidesB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                vSlidesA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (Math.abs(height + getVSlidesPos()) <= 25 && getVSlidesPos() < -220) {
                    vSlidesB.setPower(0);
                    vSlidesA.setPower(0);
                } else {
                    double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
                    vSlidesB.setPower(-power/delay);
                    vSlidesA.setPower(power/delay);
                }


                double power = control.update(height, getVSlidesPos()) / Math.abs(height - getVSlidesPos());
                vSlidesB.setPower(-power/delay);
                vSlidesA.setPower(power/delay);

                opMode.telemetry.addData("power", power);
                opMode.telemetry.update();

                packet.put("slidePos:", getVSlidesPos());
                if (Math.abs(height - getVSlidesPos()) > TOLERANCE)
                    return true;
                else {
                    vSlidesB.setPower(0);
                    vSlidesA.setPower(0);
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

                double power = control.update(length, hSlides.getCurrentPosition()) / Math.abs(length - getHSlidesPos());
                hSlides.setPower(power);

                packet.put("slidePos:", getHSlidesPos());
                if (Math.abs(length - getHSlidesPos()) > HSTOLERANCE)
                    return true;
                else {
                    hSlides.setPower(0);
                    return false;
                }
            }
        };
    }
    public Action moveHSlidesLengthDelay(int length, double delayFactor) {
        return new Action() {
            boolean initialized = false;
            double initPos = getVSlidesPos();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PIDController control = new PIDController(8,0,0.15);
                double maxError = length - previousTarget;
                hSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double power = control.update(length, hSlides.getCurrentPosition()) / Math.abs(length - getHSlidesPos());
                hSlides.setPower(power/delayFactor);

                packet.put("slidePos:", getHSlidesPos());
                if (Math.abs(length - getHSlidesPos()) > HSTOLERANCE)
                    return true;
                else {
                    hSlides.setPower(0);
                    return false;
                }
            }
        };
    }

    public boolean checkLimitsHSlide(double power) {
        return ((hSlides.getCurrentPosition() > -1300 || power > 0) && (hSlides.getCurrentPosition() < 50 || power < 0));
    }

    public int getVSlidesPos() {
        // return (vSlidesA.getCurrentPosition() + vSlidesB.getCurrentPosition()) / 2;
        return ((vSlidesA.getCurrentPosition() - vSlidesB.getCurrentPosition())/2);
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


    /**
     * DELETED METHODS
     */

    /* public void moveHSlidesPIDTeleOp() {
        int minTargetPosition = -1600;  // Lower limit of the vertical slide
        int maxTargetPosition = 0;
        PIDController control = new PIDController(2, 0, 0.1);
        double joystickInput = 50 * opMode.gamepad2.left_stick_y;

        hSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition += joystickInput;
        if (targetPosition > maxTargetPosition) {
            targetPosition = maxTargetPosition;
        } else if (targetPosition < minTargetPosition) {
            targetPosition = minTargetPosition;
        }

        double power = control.update(targetPosition, hSlides.getCurrentPosition());
        hSlides.setPower(power);
    }

    public void depositSpec() {
        vSlidesA.setTargetPositionTolerance(40);
        vSlidesA.setTargetPosition(vSlidesA.getCurrentPosition() - 200);

        vSlidesA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vSlidesA.setPower(-0.8);

        vSlidesA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean checkLimits(double power) {
        double normPower = -power;
        opMode.telemetry.addData("power", normPower);
        opMode.telemetry.update();
        return ((vSlidesA.getCurrentPosition() > -3250 || normPower < 0) && (vSlidesA.getCurrentPosition() < 50 || normPower > 0));
    }
    public Action raiseVSlides(int targetPosition) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    vSlidesB.setPower(-1.0);
                    initialized = true;
                }

                double pos = vSlidesB.getCurrentPosition();

                if (pos > targetPosition)
                    return true;
                else {
                    vSlidesB.setPower(0);
                    return false;
                }
            }
        };
    }

    public Action moveHSlidesHeight(int height, double power) {
        return new Action() {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    if (getHSlidesPos() < height) {
                        hSlides.setPower(power);
                    }
                    else if (getVSlidesPos() > height) {
                        hSlides.setPower(-power);
                    }
                    else {
                        hSlides.setPower(0);
                    }
                    initialized = true;
                }
                telemetryPacket.put("slidePos:", getVSlidesPos());
                return Math.abs(getVSlidesPos() - height) < TOLERANCE;
            }
        };
    }
     */
}