package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {
    private CRServo fourBar;
    private LinearOpMode opMode;
    private double position;

    public FourBar(HardwareMap hardwareMap, LinearOpMode opMode) {
        fourBar = hardwareMap.get(CRServo.class, "fourBar");
        this.opMode = opMode;
    }

    public void rotate() {
        if (opMode.gamepad2.right_stick_y != 0)
            fourBar.setPower(-opMode.gamepad2.right_stick_y);
        else if (opMode.gamepad2.right_bumper)
            fourBar.setPower(-0.1);
        opMode.telemetry.addData("Direction", fourBar.getDirection());
        opMode.telemetry.update();
    }

}