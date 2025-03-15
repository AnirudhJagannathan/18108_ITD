package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.auto.Extend;
import org.firstinspires.ftc.teamcode.commands.auto.MoveFEDHES;
import org.firstinspires.ftc.teamcode.commands.auto.MoveHSlidesAuto;
import org.firstinspires.ftc.teamcode.commands.auto.MoveVSlidesAuto;
import org.firstinspires.ftc.teamcode.commands.auto.Retract;
import org.firstinspires.ftc.teamcode.commands.auto.Transfer;
import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateFEDHES;
import org.firstinspires.ftc.teamcode.commands.instant.RotateIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateYaw;
import org.firstinspires.ftc.teamcode.commands.instant.ToggleClaw;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.teleopsubs.SensorColor;
import org.firstinspires.ftc.teamcode.teleopsubs.Slides;

import java.util.ArrayList;


@Autonomous
public class BlueRight extends CommandOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Follower follower;

    private Slides slides;

    private SensorColor sensorColor;

    private final RobotHardware hardware = RobotHardware.getInstance();

    GamepadEx gp1;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(6.794, 64.792, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(20, 0, Math.toRadians(0));

    private final Pose testPose = new Pose(5, 0, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(4, -36, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(50, -45, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, -45, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(50, -45, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    private final Pose scoreControl = new Pose(0, -5, Math.toRadians(0));
    private final Pose scoreControl2 = new Pose(100, -26, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain p0,p1,p2,p3, p4, p5, p6, p7, p8, p9, pAlign, p10,p11, p12, p13, p14, pblud, pShift, pShift2, pShift3, pPush;

    private int intakeRegion = -1;

    private double[] intakeRegions = new double[] {75, 73.5, 72, 70.5, 69, 67.5, 66, 64.5, 63, 61.5, 60};

    private boolean startHeld = false;
    private boolean backHeld = false;


    private final ArrayList<PathChain> paths = new ArrayList<>();
    PathChain pathChain = new PathChain();
    PathChain pathChain2 = new PathChain();
    Path path = new Path(
            new BezierLine(new Point(startPose), new Point(scorePose))
    );

    Path path2 = new Path(
            new BezierLine(new Point(scorePose), new Point(testPose))
    );


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        RobotHardware.getInstance().limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        RobotHardware.getInstance().limelight.start(); // This tells Limelight to start looking!

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);

        follower.setStartingPose(startPose);

        follower.setMaxPower(1);



        p0 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.794, 64.792, Point.CARTESIAN),
                                new Point(37.4, intakeRegions[intakeRegion], Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(6)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        p1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(37.4, intakeRegions[intakeRegion], Point.CARTESIAN),
                                new Point(20, 52.638, Point.CARTESIAN),
                                new Point(24, 25.538, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(4)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pblud = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15, 36, Point.CARTESIAN),
                                new Point (7, 36, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(4)
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(30))
                .build();

        p2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(6.794, 64.792, Point.CARTESIAN),
                                new Point(24, 26.238, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .build();

        p3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24, 22.838, Point.CARTESIAN),
                                new Point(24, 14.759, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        p4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24, 14.759, Point.CARTESIAN),
                                new Point(24, 15.759, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-27))
                .build();

        p5 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(63.466, 30.159, Point.CARTESIAN),
                                new Point(20.571, 26.016, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        p6 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(16.571, 26.016, Point.CARTESIAN),
                                new Point(61.478, 29.313, Point.CARTESIAN),
                                new Point(61.809, 16.239, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        p7 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(61.809, 16.239, Point.CARTESIAN),
                                new Point(21.068, 17.068, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        p8 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(17.068, 17.068, Point.CARTESIAN),
                                new Point(61.975, 19.862, Point.CARTESIAN),
                                new Point(62.638, 9.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        p9 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(62.638, 9.8, Point.CARTESIAN),
                                new Point(20.405, 9.8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        p10 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(24, 15.759, Point.CARTESIAN),
                                new Point(11, 40, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(30))
                .build();

        p11 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(11, 40, Point.CARTESIAN),
                                new Point(6.5, 40, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(30))
                .build();

        p12 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(6.5, 40, Point.CARTESIAN),
                                new Point(31, 62, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(5.5)
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(25))
                .build();

        p13 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(31, 58, Point.CARTESIAN),
                                new Point(8.5, 36, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(30))
                .build();

        p14 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(8.5, 36, Point.CARTESIAN),
                                new Point(31.5, 62, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(5.5)
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(25))
                .build();

        pShift = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(6.794, 64.792, Point.CARTESIAN),
                                new Point(24, 26.238, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        pShift2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24, 22.838, Point.CARTESIAN),
                                new Point(24, 14.759, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        pShift3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(24, 14.759, Point.CARTESIAN),
                                new Point(24, 15.759, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-27))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        hardware.setAlliance(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.init(hardwareMap);
        hardware.slides.reset();
        hardware.slides.setOffset(0);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        hardware.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        hardware.limelight.start(); // This tells Limelight to start looking!
        hardware.limelight.pipelineSwitch(2);

        hardware.read();
        hardware.slides.setAuto(true);

        hardware.color.acceptYellow = false;

        schedule(
                new RotateIntake(Intake.IntakeState.UP),
                new ToggleClaw(Claw.ClawState.CLOSED),
                new RotateYaw(Claw.YawState.CENTER)
        );

        while (!isStarted()) {
            hardware.read();
            hardware.write();
            hardware.periodic();
            hardware.clearBulkCache();

            if (gamepad1.start && !startHeld)
                intakeRegion += 1;
            if (gamepad1.back && !backHeld)
                intakeRegion -= 1;

            startHeld = gamepad1.start;
            backHeld = gamepad1.back;

            telemetry.addData("intakeRegion", intakeRegion);
            telemetry.update();

            //telemetry.addData("tx", hardware.limelight.getLatestResult().getTx());
            //telemetry.update();
        }

        buildPaths();

        schedule(
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> follower.setMaxPower(0.8)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p0), // Move to submersible to deposit first spec
                                new MoveVSlidesAuto(-650, -50, 2),
                                new Extend(Claw.YawState.CENTER, FEDHES.FEDHESState.LESS_FRONT),
                                new MoveHSlidesAuto(500, 1) // Reach into sub
                        ),
                        new ParallelCommandGroup(
                                new WaitCommand(50),
                                // new RotateArm(Arm.ArmState.SPEC),
                                new MoveVSlidesAuto(-930, -75, 2) // Hang spec
                        ),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),

                        // Deposit FIRST SAMPLE into obs zone
                        new ParallelCommandGroup(
                                new InstantCommand(() -> hardware.slides.setOffset(0)),
                                new MoveVSlidesAuto(25, 25, 2),
                                new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN),
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                // new InstantCommand(() -> hardware.fedhes.cutPower()),
                                new SequentialCommandGroup(
                                        new ParallelDeadlineGroup(
                                                new PowerIntakeColor(0.8, 0, 1.5),
                                                new RotateIntake(Intake.IntakeState.DOWN),
                                                new MoveHSlidesAuto(900, 1)
                                        )
                                )
                        ), // PMO BLUDMASTER
                        new InstantCommand(() -> follower.setMaxPower(1.0)),
                        new ParallelCommandGroup(
                                new RotateIntake(Intake.IntakeState.UP),
                                new FollowPathCommand(follower, p1),
                                new SequentialCommandGroup(
                                        new MoveHSlidesAuto(-25, 3),
                                        new PowerIntake(-1.0),
                                        new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                )
                        ),
                        new WaitCommand(200),

                        // TRANSFER
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.UP),
                                        new RotateIntake(Intake.IntakeState.DOWN),
                                        new MoveHSlidesAuto(600, 2)
                                ),
                                new WaitCommand(50)
                        ),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new SequentialCommandGroup(
                                        new PowerIntakeColor(0.8, 0, 1.5),
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN)
                                )
                        ),
                        new WaitCommand(250),

                        /* new SequentialCommandGroup(
                                new PowerIntake(0),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(150),
                                new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                                new WaitCommand(150)
                        ),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),


                        // INTAKE
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        // new WaitCommand(1400),
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new MoveHSlidesAuto(600, 2)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(150),
                                        new ParallelCommandGroup(
                                                new MoveHSlidesAuto(200, 1.5),
                                                new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN)
                                        )
                                )
                        ),
                        new WaitCommand(50),
                        new RotateIntake(Intake.IntakeState.DOWN),
                        new PowerIntakeColor(0.8, 0, 1.5),
                        new WaitCommand(250),
                         */

                        // HSLIDES BACK
                        new ParallelCommandGroup(
                                new PowerIntake(0),
                                new FollowPathCommand(follower, p3),
                                new MoveHSlidesAuto(-25, 2),
                                new RotateIntake(Intake.IntakeState.UP),
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                        ),

                        // TRANSFER
                        new SequentialCommandGroup(
                                new PowerIntake(-1.0),
                                new WaitCommand(200),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.UP),
                                        new RotateIntake(Intake.IntakeState.DOWN),
                                        new MoveHSlidesAuto(600, 2)
                                ),
                                new WaitCommand(50)
                        ),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new SequentialCommandGroup(
                                        new PowerIntakeColor(0.8, 0, 1.5),
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN)
                                )
                        ),
                        new WaitCommand(250),

                        // 2ND
                        new ParallelCommandGroup(
                                new PowerIntake(0),
                                new FollowPathCommand(follower, p4),
                                new MoveHSlidesAuto(0, 2),
                                new RotateIntake(Intake.IntakeState.UP),
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                        ),

                        // TRANSFER
                        new SequentialCommandGroup(
                                new PowerIntake(-1.0),
                                new WaitCommand(200),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.UP),
                                        new RotateIntake(Intake.IntakeState.DOWN),
                                        new MoveHSlidesAuto(800, 2)
                                ),
                                new WaitCommand(50)
                        ),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new SequentialCommandGroup(
                                        new PowerIntakeColor(0.8, 0, 1.5),
                                        new ToggleClaw(Claw.ClawState.CLOSED),
                                        new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN)
                                )
                        ),
                        new WaitCommand(250),

                        // 3RD
                        new ParallelCommandGroup(
                                new PowerIntake(0),
                                new FollowPathCommand(follower, p10),
                                new MoveHSlidesAuto(0, 2),
                                new RotateIntake(Intake.IntakeState.UP),
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                        ),

                        // TRANSFER
                        new SequentialCommandGroup(
                                new PowerIntake(-1.0),
                                new WaitCommand(250),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(200),
                                new Retract(Claw.YawState.CENTER, Arm.ArmState.UP),
                                new WaitCommand(450)
                        ),


                        /**
                         * DEPOSIT SPECIMEN
                         */



                        new ParallelCommandGroup(
                                new RotateYaw(Claw.YawState.RIGHT),
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new FollowPathCommand(follower, p11)
                        ),

                        new WaitCommand(350),
                        new ToggleClaw(Claw.ClawState.CLOSED),
                        new WaitCommand(50),


                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                new RotateIntake(Intake.IntakeState.UP),
                                new FollowPathCommand(follower, p12, true),
                                new MoveVSlidesAuto(-650, -75, 2),
                                new SequentialCommandGroup(
                                        // new RotateArm(Arm.ArmState.SAMPLE),
                                        new Extend(Claw.YawState.CENTER, FEDHES.FEDHESState.FRONT),
                                        new WaitCommand(850),
                                        new RotateYaw(Claw.YawState.LEFT)
                                )
                        ),
                        new WaitCommand(100),
                        new MoveVSlidesAuto(-940, -75, 0.5),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new RotateYaw(Claw.YawState.CENTER),
                        new WaitCommand(100),

                        new InstantCommand(() -> follower.setMaxPower(0.95)),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new FollowPathCommand(follower, p13, false),
                                new MoveVSlidesAuto(25, 0, 2),
                                new SequentialCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                                        new WaitCommand(850),
                                        new ParallelCommandGroup(
                                                new RotateYaw(Claw.YawState.RIGHT),
                                                new RotateArm(Arm.ArmState.FINALUP)
                                        ),
                                        new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                )
                        ),
                        new WaitCommand(50),
                        new ToggleClaw(Claw.ClawState.CLOSED),
                        new WaitCommand(50),


                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                new RotateIntake(Intake.IntakeState.UP),
                                new FollowPathCommand(follower, p14, true),
                                new MoveVSlidesAuto(-650, -75, 2),
                                new SequentialCommandGroup(
                                        // new RotateArm(Arm.ArmState.SAMPLE),
                                        new Extend(Claw.YawState.CENTER, FEDHES.FEDHESState.FRONT),
                                        new WaitCommand(850),
                                        new RotateYaw(Claw.YawState.LEFT)
                                )
                        ),
                        new WaitCommand(100),
                        new MoveVSlidesAuto(-940, -75, 2),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new RotateYaw(Claw.YawState.CENTER),
                        new WaitCommand(100),

                        new InstantCommand(() -> follower.setMaxPower(0.95)),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new FollowPathCommand(follower, p13, false),
                                new MoveVSlidesAuto(25, 0, 2),
                                new SequentialCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                                        new WaitCommand(850),
                                        new ParallelCommandGroup(
                                                new RotateYaw(Claw.YawState.RIGHT),
                                                new RotateArm(Arm.ArmState.FINALUP)
                                        ),
                                        new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                )
                        ),
                        new WaitCommand(50),
                        new ToggleClaw(Claw.ClawState.CLOSED),
                        new WaitCommand(50),


                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                new RotateIntake(Intake.IntakeState.UP),
                                new FollowPathCommand(follower, p14, true),
                                new MoveVSlidesAuto(-650, -75, 2),
                                new SequentialCommandGroup(
                                        // new RotateArm(Arm.ArmState.SAMPLE),
                                        new Extend(Claw.YawState.CENTER, FEDHES.FEDHESState.FRONT),
                                        new WaitCommand(850),
                                        new RotateYaw(Claw.YawState.LEFT)
                                )
                        ),
                        new WaitCommand(100),
                        new MoveVSlidesAuto(-940, -75, 0.5),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new RotateYaw(Claw.YawState.CENTER),
                        new WaitCommand(100),


                        new InstantCommand(() -> follower.setMaxPower(0.95)),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new FollowPathCommand(follower, p13, false),
                                new MoveVSlidesAuto(25, 0, 2),
                                new SequentialCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                                        new WaitCommand(850),
                                        new ParallelCommandGroup(
                                                new RotateYaw(Claw.YawState.RIGHT),
                                                new RotateArm(Arm.ArmState.FINALUP)
                                        ),
                                        new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                )
                        ),
                        new WaitCommand(50),
                        new ToggleClaw(Claw.ClawState.CLOSED),
                        new WaitCommand(50),


                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new ParallelCommandGroup(
                                new RotateIntake(Intake.IntakeState.UP),
                                new FollowPathCommand(follower, p14, true),
                                new MoveVSlidesAuto(-650, -75, 2),
                                new SequentialCommandGroup(
                                        // new RotateArm(Arm.ArmState.SAMPLE),
                                        new Extend(Claw.YawState.CENTER, FEDHES.FEDHESState.FRONT),
                                        new WaitCommand(650),
                                        new RotateYaw(Claw.YawState.LEFT)
                                )
                        ),
                        new WaitCommand(100),
                        new MoveVSlidesAuto(-940, -75, 0.5),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new RotateYaw(Claw.YawState.CENTER),
                        new WaitCommand(100),


                        new InstantCommand(() -> follower.setMaxPower(0.95)),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new FollowPathCommand(follower, p13, false),
                                new MoveVSlidesAuto(25, 0, 2),
                                new SequentialCommandGroup(
                                        new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                                        new WaitCommand(650),
                                        new ParallelCommandGroup(
                                                new RotateYaw(Claw.YawState.RIGHT),
                                                new RotateArm(Arm.ArmState.FINALUP)
                                        ),
                                        new ToggleClaw(Claw.ClawState.WIDE_OPEN)
                                )
                        )
                )
        );
    }

    @Override
    public void run() {
        super.run();

        hardware.clearBulkCache();
        hardware.read();
        hardware.periodic();
        hardware.write();

        telemetry.addData("slidePos", hardware.slides.getVSlidesPos());
        telemetry.addData("currentPath", follower.getCurrentPath());
        // telemetry.addData("tx", hardware.limelight.getLatestResult().getTx());
        telemetry.update();
    }
}