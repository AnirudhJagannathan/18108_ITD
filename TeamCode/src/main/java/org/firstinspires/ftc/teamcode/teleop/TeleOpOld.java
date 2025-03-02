package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;
import org.firstinspires.ftc.teamcode.teleopsubs.ITDRobot;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;
import org.firstinspires.ftc.teamcode.teleopsubs.Slides;

@TeleOp
public class TeleOpOld extends OpMode {
    public enum SlideState {
        START,
        EXTEND,
        LOW,
        SPEC,
        RETRACT
    }

    SlideState slideState = SlideState.START;

    Limelight3A limelight;

    public Arm arm;
    public Claw claw;
    public Slides slides;
    public Intake intake;
    public FEDHES spool;
    public ITDRobot robot;

    public ElapsedTime slideTimer = new ElapsedTime();

    final int SLIDE_BASE = 0;
    final int SLIDE_LOW = -50;

    final int SLIDE_SPEC_START = 1600;
    final int SLIDE_SPEC_END = 900;

    final int SLIDE_NEAR_LOW = 700;

    final int SLIDE_HIGH = 2760;

    double INTAKE_POWER = 0.7;

    int eagleBLUD = 1;

    private NormalizedColorSensor colorSensor;
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern blue;
    private RevBlinkinLedDriver.BlinkinPattern patternOff;
    private RevBlinkinLedDriver.BlinkinPattern yellow;

    private RevBlinkinLedDriver.BlinkinPattern red;

    int slidePos = 0;
    int slidePosH = 1500;

    boolean down = false;

    double tx = 0;

    double maxDrivePower = 1.0;

    double currentVoltage = 0;

    final float[] hsvValues = new float[3];

    private final RobotHardware hardware = RobotHardware.getInstance();

    @Override
    public void init() {
        slideTimer.reset();

        hardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /* limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0);
         */

        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new Slides(hardwareMap);
        intake = new Intake(hardwareMap);
        spool = new FEDHES(hardwareMap);
        robot = new ITDRobot(hardwareMap, this);
        /* blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");


        colorSensor.setGain(2);

        blue = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        patternOff = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        yellow = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        red = RevBlinkinLedDriver.BlinkinPattern.RED;

        blinkinLedDriver.setPattern(patternOff);
        telemetry.addData("LED Info:", blinkinLedDriver.getConnectionInfo());
        telemetry.update();

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
         */
    }

    @Override
    public void loop() {

        if (gamepad1.back)
            eagleBLUD = 1;
        if (gamepad1.start)
            eagleBLUD = 2;

        /* LLResult result = limelight.getLatestResult();

        double Kp = -0.05f;
        double min_command = 0.02;

        if (result != null && result.isValid()) {
            tx = result.getTx();
        }
        else {
            tx = 0;
        }

        if (gamepad1.left_bumper){
            limelight.pipelineSwitch(0);
            double heading_error = -tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0)
            {
                if (heading_error < 0)
                {
                    steering_adjust = Kp*heading_error + min_command;
                }
                else
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
            }
            robot.getMotors()[0].setPower(steering_adjust);
            robot.getMotors()[1].setPower(-steering_adjust);
            robot.getMotors()[2].setPower(steering_adjust);
            robot.getMotors()[3].setPower(-steering_adjust);
        }

        if (gamepad1.left_trigger > 0.1){
            limelight.pipelineSwitch(eagleBLUD);
            double heading_error = -tx;
            double steering_adjust = 0.0;
            if (Math.abs(heading_error) > 1.0)
            {
                if (heading_error < 0)
                {
                    steering_adjust = Kp*heading_error + min_command;
                }
                else
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
            }
            robot.getMotors()[0].setPower(steering_adjust);
            robot.getMotors()[1].setPower(-steering_adjust);
            robot.getMotors()[2].setPower(steering_adjust);
            robot.getMotors()[3].setPower(-steering_adjust);
        }


        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[0] > 12 && hsvValues[0] < 35) {
            blinkinLedDriver.setPattern(red);
        }
        else if (hsvValues[0] > 210){
            blinkinLedDriver.setPattern(blue);
        }

        else if (hsvValues[0] > 70 && hsvValues[0] < 90){
            blinkinLedDriver.setPattern(yellow);
        }


        else {
            blinkinLedDriver.setPattern(patternOff);
        }
         */

        slides.moveVSlidesPID(slidePos);

        switch (slideState) {
            case START:
                if (gamepad1.y || Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                    slidePos = SLIDE_HIGH;
                    slideState = SlideState.EXTEND;
                    down = false;
                } else if (gamepad1.b || gamepad2.right_stick_button) {
                    slidePos = SLIDE_SPEC_START;
                    slideState = SlideState.EXTEND;
                    down = false;
                } else if (gamepad2.left_stick_button) {
                    slidePos = SLIDE_NEAR_LOW;
                    slideState = SlideState.EXTEND;
                } else {
                    slidePos = slides.getVSlidesPos();
                }
                break;
            case EXTEND:
                if (Math.abs(slides.getVSlidesPos() - slidePos) < 75) {
                    slideTimer.reset();
                    if (slidePos == SLIDE_HIGH) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad2.right_stick_button) {
                            slidePos = SLIDE_SPEC_START;
                        }
                        if (gamepad1.left_stick_button) {
                            slidePos = SLIDE_NEAR_LOW;
                        }
                    }
                    else if (slidePos == SLIDE_SPEC_START) {
                        if (gamepad1.x) {
                            slidePos = SLIDE_SPEC_END;
                            slideState = SlideState.SPEC;
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25)
                            slidePos = SLIDE_HIGH;
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad1.left_stick_button)
                            slidePos = SLIDE_NEAR_LOW;
                    }
                    else if (slidePos == SLIDE_NEAR_LOW) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25)
                            slidePos = SLIDE_HIGH;
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad2.right_stick_button)
                            slidePos = SLIDE_SPEC_START;
                    }
                }
                break;
            case SPEC:
                if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                    slidePos = SLIDE_LOW;
                    slideState = SlideState.RETRACT;
                }
                break;
            case RETRACT:
                if (Math.abs(slides.getVSlidesPos() - SLIDE_LOW) < 150) {
                    slideState = SlideState.START;
                }
                break;
            default:
                slideState = SlideState.START;
        }

        /* if (slideState == SlideState.START)
            robot.mecanumDriving(1.0);
        else
            robot.mecanumDriving(0.7);
         */

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            currentVoltage = module.getInputVoltage(VoltageUnit.VOLTS);
        }

        if (currentVoltage < 12)
            robot.mecanumDriving(0.85);
        else if (currentVoltage < 12.5)
            robot.mecanumDriving(0.95);
        else
            robot.mecanumDriving(1.0);


        if (gamepad2.left_bumper)
            intake.intake(INTAKE_POWER);
        if (gamepad2.right_bumper)
            intake.outtake(INTAKE_POWER);
        if (gamepad2.x)
            intake.stop();
        if (gamepad2.dpad_up)
            intake.flipUp();
        if (gamepad2.dpad_down) {
            intake.flipDown();
        }
        /* if (Arrays.equals(intake.getPosition(), intake.getLow()))
            intake.cutPower();
         */


        slides.moveHSlides(() -> gamepad2.left_stick_y);

        /* if (gamepad2.b) {
            arm.back();
        }
        if (gamepad2.y) {
            if (slidePos == SLIDE_HIGH)
                arm.deposit();
            else
                arm.back();
        }
        if (gamepad2.a) {
            arm.front();
        }
        if (gamepad2.left_stick_button) {
            arm.intermediary();
        }
         */

        // intake.setSkibAdjust();

        if (gamepad2.left_trigger > 0.25)
            claw.wideOpen();
        if (gamepad2.right_trigger > 0.25)
            claw.fullClose();

        if (gamepad1.dpad_left) {
            spool.rotate(1);
            claw.swerve(0.3);
            arm.intermediary();
        }
        if (gamepad2.y || gamepad2.b) {
            spool.rotate(0);
            claw.swerve(0.97);
            if (slidePos == SLIDE_HIGH)
                arm.deposit();
            else
                arm.back();
        }
        if (gamepad1.a || gamepad2.a) {
            spool.rotate(0.63);
            claw.swerve(0.97);
            arm.front();
        }

        if (eagleBLUD == 1)
            telemetry.addData("Limelight Sample Color", "Red");
        if (eagleBLUD == 2)
            telemetry.addData("Limelight Sample Color", "Blue");

        telemetry.addData("slidePosV", slides.getVSlidesPos());
        telemetry.addData("slidePosH", slides.getHSlidesPos());
        telemetry.addData("hsvValues[0]", hsvValues[0]);
        telemetry.addData("hsvValues[1]", hsvValues[1]);
        telemetry.addData("hsvValues[2]", hsvValues[2]);
        telemetry.addData("Current Voltage", currentVoltage);
        // telemetry.addData("Upper Bound", Math.floor(currentVoltage) + (currentVoltage % 1) + Math.random() / 15);
        // telemetry.addData("Lower Bound", Math.floor(currentVoltage) - (currentVoltage % 1) - Math.random() / 15);
        telemetry.update();
    }
}
