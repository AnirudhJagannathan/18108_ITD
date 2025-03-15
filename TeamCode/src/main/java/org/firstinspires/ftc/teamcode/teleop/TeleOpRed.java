package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.LimelightAlign;
import org.firstinspires.ftc.teamcode.commands.MoveVSlides;
import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateFEDHES;
import org.firstinspires.ftc.teamcode.commands.instant.RotateIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateYaw;
import org.firstinspires.ftc.teamcode.commands.instant.ToggleClaw;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;
import org.firstinspires.ftc.teamcode.teleopsubs.ITDRobot;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;
import org.firstinspires.ftc.teamcode.teleopsubs.SensorColor;

@TeleOp
public class TeleOpRed extends CommandOpMode {

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

    final int SLIDE_SPEC_START = -620;
    final int SLIDE_SPEC_END = -900;

    final int SLIDE_NEAR_LOW = -100;

    final int SLIDE_HIGH = -1650;

    double INTAKE_POWER = 1.0;

    final int HSLIDE_INC = 10;

    int slidePos = SLIDE_BASE;

    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new ITDRobot(hardwareMap, this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        hardware.setAlliance(1);

        hardware.init(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        gp2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(200).andThen(
                                        new ParallelCommandGroup(
                                                new RotateYaw(Claw.YawState.CENTER),
                                                new RotateArm(Arm.ArmState.SAMPLE),
                                                new RotateFEDHES(FEDHES.FEDHESState.BACK)
                                        )
                                )
                        )));

        gp2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.SPEC)
                                ),
                                new WaitCommand(200).andThen(
                                        new ParallelCommandGroup(
                                                new RotateYaw(Claw.YawState.CENTER),
                                                new RotateArm(Arm.ArmState.DOWN),
                                                new RotateFEDHES(FEDHES.FEDHESState.DOWN)
                                        )
                                ),
                                new WaitCommand(200).andThen(
                                        new ToggleClaw(Claw.ClawState.OPEN)
                                )
                        )));
        gp2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateArm(Arm.ArmState.UP)
                                ),
                                new WaitCommand(200).andThen(
                                        new RotateFEDHES(FEDHES.FEDHESState.BACK)
                                ),
                                new WaitCommand(hardware.fedhes.getState() == FEDHES.FEDHESState.FRONT ? 800 : 250).andThen(
                                        new RotateArm(Arm.ArmState.FINALUP)
                                )
                        )));

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> CommandScheduler.getInstance().schedule(slidePos != SLIDE_LOW ?
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RotateYaw(Claw.YawState.CENTER),
                                        new RotateFEDHES(FEDHES.FEDHESState.FRONT),
                                        new RotateArm(Arm.ArmState.SPEC)
                                )
                        ) : new SequentialCommandGroup()
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
                .whenPressed(() -> CommandScheduler.getInstance().schedule(new PowerIntake(INTAKE_POWER / 2)));
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
                hardware.setColorSensor(new SensorColor(1, true));
                telemetry.addData("Limelight Sample Color", "Red");
            }
            if (gamepad1.back) {
                hardware.setColorSensor(new SensorColor(0, true));
                telemetry.addData("Limelight Sample Color", "Blue");
            }
            telemetry.update();
            hardware.slides.setOffset(-25);
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
                () -> 0.8 * gamepad1.left_stick_y,
                () -> 0.8 * gamepad1.right_stick_x,
                () -> 0.8 * gamepad1.left_stick_x
        );

        /*robot.fieldCentric(
                () -> gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_stick_x
        );
        */

        switch (slideState) {
            case START:
                if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                    hardware.slides.setOffset(-75);
                    slidePos = SLIDE_HIGH;
                    hardware.slides.setTargetPosition(slidePos);
                    // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else if (gamepad2.right_stick_button) {
                    hardware.slides.setOffset(-75);
                    slidePos = SLIDE_SPEC_START;
                    hardware.slides.setTargetPosition(slidePos);
                    // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                } else if (gamepad2.left_stick_button) {
                    slidePos = SLIDE_NEAR_LOW;
                    hardware.slides.setTargetPosition(slidePos);
                    // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                    slideState = SlideState.EXTEND;
                }
                break;
            case EXTEND:
                if (Math.abs(hardware.slides.getVSlidesPos() - slidePos) < 75) {
                    slideTimer.reset();
                    if (slidePos == SLIDE_HIGH) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            hardware.slides.setOffset(-25);
                            hardware.slideLeftActuator.setErrorTolerance(150);
                            hardware.slideRightActuator.setErrorTolerance(150);
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
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
                            /* CommandScheduler.getInstance().schedule(
                                    new MoveVSlides(hardware, slidePos),
                                    new WaitCommand(300).andThen(
                                            new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                    )
                            );
                             */
                            hardware.slides.setTargetPosition(slidePos);
                            CommandScheduler.getInstance().schedule(
                                    new WaitCommand(300).andThen(
                                            new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                    )
                            );
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                            slidePos = SLIDE_HIGH;
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            hardware.slides.setOffset(-25);
                            hardware.slideLeftActuator.setErrorTolerance(150);
                            hardware.slideRightActuator.setErrorTolerance(150);
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad1.left_stick_button) {
                            slidePos = SLIDE_NEAR_LOW;
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                    }
                    else if (slidePos == SLIDE_NEAR_LOW) {
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                            slidePos = SLIDE_HIGH;
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                        if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                            slidePos = SLIDE_LOW;
                            hardware.slides.setOffset(-25);
                            hardware.slideLeftActuator.setErrorTolerance(150);
                            hardware.slideRightActuator.setErrorTolerance(150);
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                            slideState = SlideState.RETRACT;
                        }
                        if (gamepad2.right_stick_button) {
                            slidePos = SLIDE_SPEC_START;
                            hardware.slides.setTargetPosition(slidePos);
                            // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                        }
                    }
                }
                break;
            case SPEC:
                if (Range.clip(gamepad2.right_stick_y, -1, 1) > 0.25) {
                    slidePos = SLIDE_LOW;
                    hardware.slides.setOffset(-25);
                    hardware.slideLeftActuator.setErrorTolerance(150);
                    hardware.slideRightActuator.setErrorTolerance(150);
                    hardware.slides.setTargetPosition(slidePos);
                    // CommandScheduler.getInstance().schedule(new MoveVSlides(hardware, slidePos));
                    slideState = SlideState.RETRACT;
                }
                break;
            case RETRACT:
                if (Math.abs(hardware.slides.getVSlidesPos() - SLIDE_LOW) < 75) {
                    hardware.slideLeftActuator.setErrorTolerance(75);
                    slideState = SlideState.START;
                }
                break;
            default:
                slideState = SlideState.START;

        }


        /* hardware.slides.moveVSlidesPID(slidePos);

        switch (slideState) {
            case START:
                if (gamepad1.y || Range.clip(gamepad2.right_stick_y, -1, 1) < -0.25) {
                    slidePos = SLIDE_HIGH;
                    slideState = SlideState.EXTEND;
                } else if (gamepad1.b || gamepad2.right_stick_button) {
                    slidePos = SLIDE_SPEC_START;
                    slideState = SlideState.EXTEND;
                } else if (gamepad2.left_stick_button) {
                    slidePos = SLIDE_NEAR_LOW;
                    slideState = SlideState.EXTEND;
                } else {
                    slidePos = hardware.slides.getVSlidesPos();
                }
                break;
            case EXTEND:
                if (Math.abs(hardware.slides.getVSlidesPos() - slidePos) < 75) {
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
                if (Math.abs(hardware.slides.getVSlidesPos() - SLIDE_LOW) < 150) {
                    slideState = SlideState.START;
                }
                break;
            default:
                slideState = SlideState.START;
        }
         */

        hardware.slides.moveHSlides(() -> -gamepad2.left_stick_y);

        if (gamepad2.start)
            hardware.fedhes.bludAdjust += 0.01;
        if (gamepad2.back)
            hardware.fedhes.bludAdjust -= 0.01;

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
        if (gamepad2.left_trigger > 0.25) {
            if (hardware.fedhes.getState() == FEDHES.FEDHESState.BACK || hardware.fedhes.getState() == FEDHES.FEDHESState.FRONT) {
                CommandScheduler.getInstance().schedule(new ToggleClaw(Claw.ClawState.WIDE_OPEN));
            }
            else {
                CommandScheduler.getInstance().schedule(new ToggleClaw(Claw.ClawState.OPEN));
            }
        }

        telemetry.addData("slidePos", hardware.slides.getVSlidesPos());
        telemetry.addData("slidePosH", hardware.slides.getHSlidesPos());
        telemetry.addData("hasReached", hardware.slides.hasReachedV());
        telemetry.addData("arm1", hardware.arm1.getPosition());
        telemetry.addData("arm2", hardware.arm2.getPosition());
        telemetry.addData("tx", hardware.getTx());
        telemetry.addData("slideOffset", hardware.slideLeftActuator.getOffset());
        telemetry.update();

        CommandScheduler.getInstance().run();
    }
}
