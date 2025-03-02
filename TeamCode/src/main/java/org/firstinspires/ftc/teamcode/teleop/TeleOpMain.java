package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
import org.firstinspires.ftc.teamcode.commands.MoveSlides;
import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.teleopsubs.ITDRobot;

@TeleOp
public class TeleOpMain extends CommandOpMode {

    private final RobotHardware hardware = RobotHardware.getInstance();

    private GamepadEx gp1;
    private GamepadEx gp2;

    public enum SlideState {
        START,
        EXTEND,
        LOW,
        SPEC,
        RETRACT
    }

    SlideState slideState = SlideState.START;

    Limelight3A limelight;
    // public HypSpool spool;
    public ITDRobot robot;

    public ElapsedTime slideTimer = new ElapsedTime();

    final int SLIDE_BASE = 0;
    final int SLIDE_LOW = 50;

    final int SLIDE_SPEC_START = -930;
    final int SLIDE_SPEC_END = 200;

    final int SLIDE_NEAR_LOW = 100;

    final int SLIDE_HIGH = -1600;

    double INTAKE_POWER = 1.0;

    int slidePos = 0;

    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new ITDRobot(hardwareMap, this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        hardware.setAlliance(0);

        hardware.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        gp2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new RotateArm(0.03, 0.97)));
        gp2.getGamepadButton(GamepadKeys.Button.A)

                .whenPressed(() -> CommandScheduler.getInstance().schedule(new RotateArm(0.85, 0.15)));
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(INTAKE_POWER)));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(-INTAKE_POWER)));
        gp2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(0)));

        hardware.read();

        while (opModeInInit()) {
            if (gamepad1.start) {
                hardware.setColorSensor(new SensorColor(1));
                telemetry.addData("Limelight Sample Color", "Red");
            }
            if (gamepad1.back) {
                hardware.setColorSensor(new SensorColor(0));
                telemetry.addData("Limelight Sample Color", "Blue");
            }
            telemetry.update();
        }
    }

    @Override
    public void run() {
        hardware.periodic();
        hardware.read();
        hardware.write();
        hardware.clearBulkCache();

        /* if (gamepad1.b)
            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, 100));
         */

        /* robot.mecanumDriving(
                () -> gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x
        );
         */

        robot.fieldCentric(
                () -> gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x
        );

        switch (slideState) {
            case START:
                if (gamepad1.y || Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                    slidePos = SLIDE_HIGH;
                    CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else if (gamepad1.b || gamepad2.right_stick_button) {
                    slidePos = SLIDE_SPEC_START;
                    CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else if (gamepad2.left_stick_button) {
                    slidePos = SLIDE_NEAR_LOW;
                    CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else {
                    slidePos = hardware.slides.getVSlidesPos();
                    // CommandScheduler.getInstance().schedule(new MoveSlides(slidePos));
                }
                break;
            case EXTEND:
                if (Math.abs(hardware.slides.getVSlidesPos() - slidePos) < 75) {
                    slideTimer.reset();
                    if (slidePos == SLIDE_HIGH) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad2.right_stick_button) {
                            slidePos = SLIDE_SPEC_START;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                        if (gamepad1.left_stick_button) {
                            slidePos = SLIDE_NEAR_LOW;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                    }
                    else if (slidePos == SLIDE_SPEC_START) {
                        if (gamepad1.x) {
                            slidePos = SLIDE_SPEC_END;
                            slideState = SlideState.SPEC;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                            slidePos = SLIDE_HIGH;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad1.left_stick_button) {
                            slidePos = SLIDE_NEAR_LOW;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                    }
                    else if (slidePos == SLIDE_NEAR_LOW) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                            slidePos = SLIDE_HIGH;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            slideState = SlideState.RETRACT;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
                        if (gamepad2.right_stick_button) {
                            slidePos = SLIDE_SPEC_START;
                            CommandScheduler.getInstance().schedule(new MoveSlides(hardware, slidePos));
                        }
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
                if (Math.abs(hardware.slides.getVSlidesPos() - SLIDE_LOW) < 150) {
                    slideState = SlideState.START;
                }
                break;
            default:
                slideState = SlideState.START;

        }

        telemetry.addData("slidePos", hardware.slides.getVSlidesPos());
        telemetry.addData("hasReached", hardware.slides.hasReached());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
