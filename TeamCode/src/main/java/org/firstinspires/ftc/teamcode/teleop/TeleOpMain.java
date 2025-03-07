package org.firstinspires.ftc.teamcode.teleop;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commands.LimelightAlign;
import org.firstinspires.ftc.teamcode.commands.MoveHSlides;
import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
import org.firstinspires.ftc.teamcode.commands.MoveVSlides;
import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateFEDHES;
import org.firstinspires.ftc.teamcode.commands.instant.RotateIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateYaw;
import org.firstinspires.ftc.teamcode.commands.instant.ToggleClaw;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;
import org.firstinspires.ftc.teamcode.teleopsubs.ITDRobot;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

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
    final int SLIDE_SPEC_END = -700;

    final int SLIDE_NEAR_LOW = -100;

    final int SLIDE_HIGH = -1650;

    double INTAKE_POWER = 1.0;

    final int HSLIDE_INC = 10;

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
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(250).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.UP),
                                                new RotateFEDHES(FEDHES.FEDHESState.BACK)
                                        )
                                )
                        )));
        gp2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(250).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.DOWN),
                                                new RotateFEDHES(FEDHES.FEDHESState.DOWN)
                                        )
                                )
                        )));
        gp2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(250).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.UP),
                                                new RotateFEDHES(FEDHES.FEDHESState.BACK)
                                        )
                                )
                        )));

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateFEDHES(FEDHES.FEDHESState.FRONT),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(600).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.TWIST_SPEC),
                                                new RotateYaw(Claw.YawState.RIGHT)
                                        )
                                )
                        )
                ));


        gp1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(250).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.UP)
                                                // new RotateFEDHES(FEDHES.FEDHESState.BACK)
                                        )
                                )
                        )));
        gp1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(250).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.DOWN)
                                                // new RotateFEDHES(FEDHES.FEDHESState.DOWN)
                                        )
                                )
                        )));
        gp1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(250).andThen(
                                        new ParallelCommandGroup(
                                                new RotateArm(Arm.ArmState.UP)
                                                // new RotateFEDHES(FEDHES.FEDHESState.BACK)
                                        )
                                )
                        )));



        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(INTAKE_POWER)));
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(-INTAKE_POWER)));
        gp2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(0)));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new RotateIntake(Intake.IntakeState.UP)));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new RotateIntake(Intake.IntakeState.DOWN)));

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> CommandScheduler.getInstance().schedule(new LimelightAlign(hardware, hardware.getAlliance() == 0 ? 1 : 2)));

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

        robot.mecanumDriving(
                () -> gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x
        );

        /*robot.fieldCentric(
                () -> gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x
        );*/

        switch (slideState) {
            case START:
                if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                    slidePos = SLIDE_HIGH;
                    CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else if (gamepad2.right_stick_button) {
                    slidePos = SLIDE_SPEC_START;
                    CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else if (gamepad2.left_stick_button) {
                    slidePos = SLIDE_NEAR_LOW;
                    CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
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
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad2.right_stick_button) {
                            slidePos = SLIDE_SPEC_START;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (gamepad1.left_stick_button) {
                            slidePos = SLIDE_NEAR_LOW;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                    }
                    else if (slidePos == SLIDE_SPEC_START) {
                        if (gamepad1.x) {
                            slidePos = SLIDE_SPEC_END;
                            slideState = SlideState.SPEC;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                            slidePos = SLIDE_HIGH;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad1.left_stick_button) {
                            slidePos = SLIDE_NEAR_LOW;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                    }
                    else if (slidePos == SLIDE_NEAR_LOW) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                            slidePos = SLIDE_HIGH;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            slideState = SlideState.RETRACT;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (gamepad2.right_stick_button) {
                            slidePos = SLIDE_SPEC_START;
                            CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                    }
                }
                break;
            case SPEC:
                if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                    slidePos = SLIDE_LOW;
                    CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
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

        hardware.slides.moveHSlides(() -> -gamepad2.left_stick_y);

            //CommandScheduler.getInstance().schedule(new MoveHSlides(hardware, hardware.hSlideActuactor.getTargetPosition() - HSLIDE_INC));



        /* if (gamepad2.dpad_up)
            CommandScheduler.getInstance().schedule(new RotateIntake(Intake.IntakeState.UP));
        if (gamepad2.dpad_down)
            CommandScheduler.getInstance().sched)ule(new RotateIntake(Intake.IntakeState.DOWN));
         */

        /* if (gamepad2.y || gamepad2.b) {
            if (slidePos == SLIDE_HIGH) {
                CommandScheduler.getInstance().schedule(new RotateArm(Arm.ArmState.SAMPLE));
            }
            else {
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new RotateArm(Arm.ArmState.UP),
                        new WaitCommand(400),
                        new RotateYaw(Claw.YawState.LEFT)
                        )
                );
            }
        }
         */

        /*if (gamepad2.a) {
            CommandScheduler.getInstance().schedule(new RotateArm(Arm.ArmState.DOWN));
        }
        if (gamepad1.dpad_left) {
            CommandScheduler.getInstance().schedule(new RotateArm(Arm.ArmState.SPEC));
            CommandScheduler.getInstance().schedule(new RotateFEDHES(FEDHES.FEDHESState.FRONT));
        }*/

        if (gamepad2.right_trigger > 0.25)
            CommandScheduler.getInstance().schedule(new ToggleClaw(Claw.ClawState.CLOSED));
        if (gamepad2.left_trigger > 0.25)
            CommandScheduler.getInstance().schedule(new ToggleClaw(Claw.ClawState.OPEN));

        telemetry.addData("slidePos", hardware.slides.getVSlidesPos());
        telemetry.addData("slidePosH", hardware.slides.getHSlidesPos());
        telemetry.addData("hasReached", hardware.slides.hasReachedV());
        telemetry.addData("arm1", hardware.arm1.getPosition());
        telemetry.addData("arm2", hardware.arm2.getPosition());
        telemetry.addData("tx", hardware.getTx());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
