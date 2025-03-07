package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.LimelightStrafe;
import org.firstinspires.ftc.teamcode.commands.MoveHSlides;
import org.firstinspires.ftc.teamcode.commands.MoveVSlides;
import org.firstinspires.ftc.teamcode.commands.LimelightAlign;
import org.firstinspires.ftc.teamcode.commands.instant.PowerIntake;
import org.firstinspires.ftc.teamcode.commands.instant.RotateArm;
import org.firstinspires.ftc.teamcode.commands.instant.RotateIntake;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleopsubs.Arm;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;

import java.util.ArrayList;


@Autonomous
public class BlueLeft extends CommandOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Follower follower;

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
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(15, 2, Math.toRadians(0));

    private final Pose testPose = new Pose(15, -5, Math.toRadians(0));

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
    private Path scorePreload, park;


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


        pathChain = follower.pathBuilder().addPath(path)
                        .setConstantHeadingInterpolation(startPose.getHeading())
                        .build();

        pathChain2 = follower.pathBuilder().addPath(path2)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware.init(hardwareMap);

        hardware.setAlliance(1);

        hardware.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        hardware.limelight.start(); // This tells Limelight to start looking!
        hardware.limelight.pipelineSwitch(2);

        gp1 = new GamepadEx(gamepad1);

        while (opModeInInit()) {
            hardware.read();
            hardware.write();
            hardware.periodic();
            hardware.clearBulkCache();

            CommandScheduler.getInstance().schedule(new RotateIntake(Intake.IntakeState.DOWN));

            telemetry.addData("tx", hardware.limelight.getLatestResult().getTx());
            telemetry.update();
        }


        buildPaths();


        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> telemetry.addData("tx", hardware.limelight.getLatestResult().getTx())),
                new RunCommand(() -> telemetry.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, path),
                        new RotateIntake(Intake.IntakeState.DOWN)
                )
        );

        schedule(
                new SequentialCommandGroup(
                    new FollowPathCommand(follower,
                        follower.pathBuilder().addPath(
                            new Path(new BezierLine(
                               new Point(follower.getPose()),
                               new Point(new Pose(follower.getPose().getX(),
                                    follower.getPose().getY() - hardware.limelight.getLatestResult().getTx() / 8,
                                       Math.toRadians(0)
                               ))
                            )))
                            .setConstantHeadingInterpolation(follower.getPose().getHeading())
                            .build()
                    )
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }
}
