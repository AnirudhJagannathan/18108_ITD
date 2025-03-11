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

import org.firstinspires.ftc.teamcode.commands.auto.Extend;
import org.firstinspires.ftc.teamcode.commands.auto.MoveHSlidesAuto;
import org.firstinspires.ftc.teamcode.commands.auto.MoveVSlidesAuto;
import org.firstinspires.ftc.teamcode.commands.auto.Retract;
import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateIntake;
import org.firstinspires.ftc.teamcode.commands.instant.ToggleClaw;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleopsubs.Claw;
import org.firstinspires.ftc.teamcode.teleopsubs.FEDHES;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
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
    private PathChain p0,p1,p2,p3, p4, p5, p6, p7, p8, p9, pAlign;

    private int intakeRegion = -1;

    private double[] intakeRegions = new double[] {75, 72, 69, 66, 63, 60};

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

        follower.setMaxPower(0.8);


        p0 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.794, 64.792, Point.CARTESIAN),
                                new Point(36, intakeRegions[intakeRegion], Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        p1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(33.604, 69.763, Point.CARTESIAN),
                                new Point(20.085, 37.616, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-110))
                .build();

        p2 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(20.085, 37.616, Point.CARTESIAN),
                                new Point(23.530, 32.976, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-110), Math.toRadians(-30))
                .build();

        p3 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(23.530, 32.976, Point.CARTESIAN),
                                new Point(19.554, 29.827, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-120))
                .build();

        p4 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(19.554, 29.827, Point.CARTESIAN),
                                new Point(30.656, 20.382, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-30))
                .build();

        p5 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(30.656, 20.382, Point.CARTESIAN),
                                new Point(23.365, 18.559, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-150))
                .build();

        p6 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(23.365, 18.559, Point.CARTESIAN),
                                new Point(28.833, 14.085, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-40))
                .build();

        p7 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(28.833, 14.085, Point.CARTESIAN),
                                new Point(29.165, 35.793, Point.CARTESIAN),
                                new Point(10.108, 39.438, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(30))
                .build();

        p8 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(10.108, 39.438, Point.CARTESIAN),
                                new Point(33.621, 62.638, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(30))
                .build();

        p9 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(33.621, 62.638, Point.CARTESIAN),
                                new Point(10.108, 39.438, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(30))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();
        hardware.setAlliance(1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.init(hardwareMap);
        hardware.slides.setOffset(-30);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        hardware.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        hardware.limelight.start(); // This tells Limelight to start looking!
        hardware.limelight.pipelineSwitch(2);

        hardware.read();
        hardware.slides.setAuto(true);

        schedule(
                new RotateIntake(Intake.IntakeState.UP),
                new ToggleClaw(Claw.ClawState.CLOSED)
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
                        new ParallelCommandGroup(
                            new FollowPathCommand(follower, p0),
                            new WaitCommand(400),
                            new MoveVSlidesAuto(-650),
                            new Extend(Claw.YawState.CENTER, FEDHES.FEDHESState.LESS_FRONT),
                            new MoveHSlidesAuto(700)
                        ),
                        new WaitCommand(200),
                        new MoveVSlidesAuto(-900),
                        new ToggleClaw(Claw.ClawState.WIDE_OPEN),
                        new WaitCommand(250),
                        new ParallelCommandGroup(
                            new InstantCommand(() -> hardware.slides.setOffset(0)),
                            new MoveVSlidesAuto(25),
                            new Retract(Claw.YawState.CENTER),
                            new ToggleClaw(Claw.ClawState.CLOSED),
                            new RotateIntake(Intake.IntakeState.DOWN)
                        ),
                        new PowerIntakeAuto(0.8, 0)


                        /* new FollowPathCommand(follower, p1),
                        new FollowPathCommand(follower, p2),
                        new FollowPathCommand(follower, p3),
                        new FollowPathCommand(follower, p4),
                        new FollowPathCommand(follower, p5),
                        new FollowPathCommand(follower, p6),
                        new FollowPathCommand(follower, p7),
                        new FollowPathCommand(follower, p8),
                        new FollowPathCommand(follower, p9),
                        new FollowPathCommand(follower, p8),
                        new FollowPathCommand(follower, p9),
                        new FollowPathCommand(follower, p8),
                        new FollowPathCommand(follower, p9),
                        new FollowPathCommand(follower, p8),
                        new FollowPathCommand(follower, p9),
                        new FollowPathCommand(follower, p8),
                        new FollowPathCommand(follower, p9),
                        new FollowPathCommand(follower, p8)
                         */
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
        telemetry.addData("tx", hardware.limelight.getLatestResult().getTx());
        telemetry.update();
    }
}
