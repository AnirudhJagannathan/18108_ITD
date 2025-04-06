package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
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
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.auto.MoveFEDHES;
import org.firstinspires.ftc.teamcode.commands.auto.MoveHSlidesAuto;
import org.firstinspires.ftc.teamcode.commands.auto.MoveVSlidesAuto;
import org.firstinspires.ftc.teamcode.commands.auto.Retract;
import org.firstinspires.ftc.teamcode.commands.auto.Transfer;
import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
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
import org.firstinspires.ftc.teamcode.teleopsubs.SensorColor;
import org.firstinspires.ftc.teamcode.teleopsubs.Slides;

import java.util.ArrayList;


@Autonomous
public class RedLeft extends CommandOpMode {

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
    private final Pose startPose = new Pose(6.794, 112, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(20, 144, Math.toRadians(0));

    private final Pose testPose = new Pose(5, 144, Math.toRadians(0));

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
    private PathChain p0,p1,p2,p3, p4, p5, p6, p7, p8, p9, pAlign, p10,p11, p12, p13, p14, pblud, pShift, pShift2, pShift3, pPush, pskiblud;

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
                                new Point(6.5, 112.5, Point.CARTESIAN),
                                new Point(17.05064516129032, 131.02977667493798, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(2)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-18))
                .build();

        p1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(17.258064516129032, 131.02977667493798, Point.CARTESIAN),
                                new Point(15.4166253101737, 133.5669975186104, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-18), Math.toRadians(-2))
                .build();

        pblud = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15.4166253101737, 133.5669975186104, Point.CARTESIAN),
                                new Point(15.1166253101737, 133.0669975186104, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-2), Math.toRadians(-14))
                .build();

        p2 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(15.1166253101737, 132.5669975186104, Point.CARTESIAN),
                                new Point(16.973, 128.457, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(34))
                .build();

        pskiblud = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(16.973, 128.457, Point.CARTESIAN),
                                new Point(16.1166253101737, 133.0669975186104, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(2)
                .setLinearHeadingInterpolation(Math.toRadians(34), Math.toRadians(-14))
                .build();

        p3 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(16.1166253101737, 133.0669975186104, Point.CARTESIAN),
                                new Point(58.600, 119.881, Point.CARTESIAN),
                                new Point(65.958, 95.653, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
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

        hardware.color.acceptYellow = true;

        schedule(
                new MoveFEDHES(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN, Arm.ArmState.SPEC),
                new RotateIntake(Intake.IntakeState.UP)
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
                        new InstantCommand(() -> follower.setMaxPower(1.0)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p0)
                        ),
                        new ParallelCommandGroup(
                                new MoveHSlidesAuto(700, 2),
                                new MoveVSlidesAuto(-1650, -50, 1.5)
                        ),
                        new SequentialCommandGroup(
                                new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                            new WaitCommand(100),
                            new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                            new WaitCommand(200)
                        ),

                        // INTAKE 1ST SAMPLE
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new MoveVSlidesAuto(-400, -25, 1.5),
                                new MoveHSlidesAuto(920, 2),
                                new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN),
                                new PowerIntakeColor(0.7, 0, 2),
                                new RotateIntake(Intake.IntakeState.DOWN)
                        ),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new MoveVSlidesAuto(25, 25, 2),
                                new RotateIntake(Intake.IntakeState.UP),
                                new MoveHSlidesAuto(-25, 2)
                        ),

                        // TRANSFER
                        new SequentialCommandGroup(
                                new PowerIntake(-1.0),
                                new WaitCommand(300),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(100)
                        ),

                        new PowerIntake(0),

                        new ParallelCommandGroup(
                                new MoveVSlidesAuto(-1650, -75, 2),
                                new FollowPathCommand(follower, p1)
                        ),
                        new SequentialCommandGroup(
                                new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                                new WaitCommand(200),
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new WaitCommand(200)
                        ),


                        // INTAKE 2ND SAMPLE
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new RotateYaw(Claw.YawState.CENTER),
                                new MoveVSlidesAuto(-400, -25, 1.5),
                                new MoveHSlidesAuto(900, 2),
                                new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN),
                                new PowerIntakeColor(0.7, 0, 1.5),
                                new RotateIntake(Intake.IntakeState.DOWN)
                        ),
                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new MoveVSlidesAuto(25, 25, 2),
                                new RotateIntake(Intake.IntakeState.UP),
                                new MoveHSlidesAuto(-25, 2)
                        ),

            // TRANSFER
                        new SequentialCommandGroup(
                            new PowerIntake(-1.0),
                            new WaitCommand(300),
                            new ToggleClaw(Claw.ClawState.CLOSED),
                            new WaitCommand(200)
                        ),

                        new ParallelDeadlineGroup(
                                new MoveVSlidesAuto(-1650, -75, 2),
                                new FollowPathCommand(follower, pblud),
                                new PowerIntake(0)
                        ),

                        new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                        new WaitCommand(300),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new WaitCommand(200),


                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p2),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new RotateYaw(Claw.YawState.CENTER),
                                new MoveVSlidesAuto(-400, -25, 1.5),
                                new MoveHSlidesAuto(520, 1),
                                new Transfer(Claw.YawState.CENTER, FEDHES.FEDHESState.DOWN),
                                new RotateIntake(Intake.IntakeState.DOWN)
                        ),
                        new ParallelCommandGroup(
                                new MoveHSlidesAuto(920, 1),
                                new PowerIntakeColor(0.8, 0, 1.5)
                        ),


                        new ParallelCommandGroup(
                                new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                                new MoveVSlidesAuto(25, 25, 2),
                                new RotateIntake(Intake.IntakeState.UP),
                                new MoveHSlidesAuto(-25, 2)
                        ),

                        new SequentialCommandGroup(
                                new PowerIntake(-1.0),
                                new WaitCommand(300),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new WaitCommand(200)
                        ),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, pskiblud),
                                new PowerIntake(0),
                                new MoveVSlidesAuto(-1650, -75, 2)
                        ),

                        new Retract(Claw.YawState.CENTER, Arm.ArmState.SAMPLE),
                        new WaitCommand(300),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new WaitCommand(200),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p3),
                                new ToggleClaw(Claw.ClawState.CLOSED),
                                new RotateYaw(Claw.YawState.CENTER),
                                new RotateFEDHES(FEDHES.FEDHESState.DOWN),
                                new MoveVSlidesAuto(25, -25, 1.5)
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