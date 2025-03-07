package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
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
        if (hsvValues[0] > 12 && hsvValues[0] < 35 && eagleBLUD == 1 && robot.intake1.getPosition()>0.30) {
            robot.intake.setPower(0);
        }
        else if (hsvValues[0] > 210 && eagleBLUD == 0 && robot.intake1.getPosition()>0.30) {
            robot.intake.setPower(0);
        }

        else if (hsvValues[0] > 70 && hsvValues[0] < 90 && robot.intake1.getPosition()>0.30) {
            robot.intake.setPower(0);
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