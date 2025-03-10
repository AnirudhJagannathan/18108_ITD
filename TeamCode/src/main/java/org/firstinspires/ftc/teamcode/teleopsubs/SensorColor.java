package org.firstinspires.ftc.teamcode.teleopsubs;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.WSubsystem;

public class SensorColor extends WSubsystem {
    private final RobotHardware robot = RobotHardware.getInstance();
    public float[] hsvValues = new float[3];

    private final int eagleBLUD;

    private NormalizedRGBA colors;

    public SensorColor(int eagleBLUD) {
        this.eagleBLUD = eagleBLUD;
        colors = null;
        if (robot.colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) robot.colorSensor).enableLight(true);
        }
        robot.colorSensor.setGain(2);
    }

    @Override
    public void periodic() {
        if (!robot.outtaking) {
            if (robot.intake1.getPosition() > 0.30 || robot.hSlides.getCurrentPosition() > 250 || robot.intake.getPower() < 0) {
                if (hsvValues[0] > 0 && hsvValues[0] < 55 && eagleBLUD == 1) {
                    robot.intake.setPower(0);
                } else if (hsvValues[0] > 180 && eagleBLUD == 0) {
                    robot.intake.setPower(0);
                } else if (hsvValues[0] > 60 && hsvValues[0] < 120) {
                    robot.intake.setPower(0);
                }
            }
        }
    }

    @Override
    public void read() {
        colors = robot.colorSensor.getNormalizedColors();
    }

    @Override
    public void write() {
        Color.colorToHSV(colors.toColor(), hsvValues);
    }
}