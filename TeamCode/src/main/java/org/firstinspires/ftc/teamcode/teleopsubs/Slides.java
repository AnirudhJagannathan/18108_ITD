package org.firstinspires.ftc.teamcode.teleopsubs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.universal.PIDController;
import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;

public class Slides extends WSubsystem {

    private final RobotHardware robot = RobotHardware.getInstance();
    private DcMotorEx vSlidesA;
    private DcMotorEx vSlidesB;
    // private Servo hSlidesA;
    // private Servo hSlidesB;
    private DcMotorEx hSlides;
    private double MIDPOINT = 0.5;
    private double targetPosition = 0;
    private double previousTarget = 0;

    private final int TOLERANCE = 75;

    private double pos = 0;
    private boolean auto = false;



    public Slides(HardwareMap hardwareMap) {
        /* hSlidesA = hardwareMap.get(Servo.class, "HSlidesA");
        hSlidesB = hardwareMap.get(Servo.class, "HSlidesB");
         */

        vSlidesA = hardwareMap.get(DcMotorEx.class, "VSlidesA");
        vSlidesB = hardwareMap.get(DcMotorEx.class, "VSlidesB");
        hSlides = hardwareMap.get(DcMotorEx.class, "hSlides");
        vSlidesA.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vSlidesB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        vSlidesB.setDirection(DcMotorSimple.Direction.REVERSE);

        hSlides.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void moveVSlidesPID(double targetPosition) {
        PIDController control = new PIDController(4,0,0.15);
        pos = targetPosition;

        double maxError = targetPosition - previousTarget;
        vSlidesA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(targetPosition - getVSlidesPos()) <= 75 && getVSlidesPos() < 220) {
            vSlidesA.setPower(0);
            vSlidesB.setPower(0);
        } else {
            double power = control.update(targetPosition, getVSlidesPos()) / Math.abs(maxError);
            vSlidesA.setPower(power);
            vSlidesB.setPower(power);
        }
    }

    public void moveHSlides(DoubleSupplier joystick) {
        double power = Range.clip(joystick.getAsDouble(), -1, 1);
        if (checkLimitsHSlide(power) && (getHSlidesPos() < 200 && power < 0)) {
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
                vSlidesB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                if (Math.abs(height - getVSlidesPos()) <= 75 && getVSlidesPos() < 220) {
                    vSlidesA.setPower(0);
                    vSlidesB.setPower(0);
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
                hSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                if (Math.abs(length - hSlides.getCurrentPosition()) <= 75 && hSlides.getCurrentPosition() > -220) {
                    hSlides.setPower(0);
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
        return ((hSlides.getCurrentPosition() < 1550 || power < 0) && (hSlides.getCurrentPosition() > 50 || power > 0));
    }

    public int getVSlidesPos() {
        return (int) (robot.slideLeftActuator.getPosition() + robot.slideRightActuator.getPosition()) / 2;
    }

    public double getCurrentPosition() {
        return (double) (Math.abs(vSlidesA.getCurrentPosition()) + Math.abs(vSlidesB.getCurrentPosition())) / 2;
    }

    public int getHSlidesPos() {
        return hSlides.getCurrentPosition();
    }

    public void reset() {
        vSlidesA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vSlidesB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        vSlidesA.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vSlidesB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void periodic() {
        robot.slideLeftActuator.setCurrentPosition(vSlidesA.getCurrentPosition());
        robot.slideRightActuator.setCurrentPosition(vSlidesB.getCurrentPosition());
        if (auto)
            robot.hSlideActuactor.setCurrentPosition(hSlides.getCurrentPosition());

        robot.slideLeftActuator.periodic();
        robot.slideRightActuator.periodic();
        if (auto)
            robot.hSlideActuactor.periodic();
    }

    @Override
    public void read() {
        robot.slideLeftActuator.read();
        robot.slideRightActuator.read();
        if (auto)
            robot.hSlideActuactor.read();
    }

    @Override
    public void write() {
        robot.slideLeftActuator.write();
        robot.slideRightActuator.write();
        if (auto)
            robot.hSlideActuactor.write();
    }

    public boolean hasReachedV() {
        return robot.slideLeftActuator.hasReached() && robot.slideRightActuator.hasReached();
    }

    public boolean hasReachedH() {
        return robot.hSlideActuactor.hasReached();
    }

    public void setAuto(boolean auto) {
        this.auto = auto;
    }

    public void setOffset(double offset) {
        robot.slideLeftActuator.setTargetPositionOffset(offset);
        robot.slideRightActuator.setTargetPositionOffset(offset);
    }

    public void setTargetPosition(int targetPosition) {
        robot.slideLeftActuator.setTargetPosition(targetPosition);
        robot.slideRightActuator.setTargetPosition(targetPosition);
    }

    public void setTargetPositionH(int targetPosition) {
        robot.hSlideActuactor.setTargetPosition(targetPosition);
    }

    public void setVCurrentPosition(double position) {
        robot.slideLeftActuator.setCurrentPosition(position);
        robot.slideRightActuator.setCurrentPosition(position);
    }

    public double[] getOffset() {
        return new double[]{robot.slideLeftActuator.getOffset(), robot.slideRightActuator.getOffset()};
    }
}
