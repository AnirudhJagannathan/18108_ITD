package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.MoveVSlides;
import org.firstinspires.ftc.teamcode.commands.LimelightAlign;
import org.firstinspires.ftc.teamcode.commands.instant.RotateIntake;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.teleopsubs.Intake;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import java.util.ArrayList;


@Autonomous
public class BlueLeft extends CommandOpMode {

    private Follower follower;

    private final RobotHardware hardware = RobotHardware.getInstance();
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
    private final Pose scorePose = new Pose(15, 12, Math.toRadians(0));

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
    Path path = new Path(
            new BezierLine(new Point(startPose), new Point(scorePose))
    );

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);

        paths.add(
                // An example path
                follower.pathBuilder()
                        .addPath(
                                path
                        )
                        .setConstantHeadingInterpolation(startPose.getHeading())
                        .build()
        );
    }

    @Override
    public void initialize() {
        super.reset();

        hardware.init(hardwareMap);

        buildPaths();

        schedule(
                // Updates follower to follow path
                new RunCommand(() -> follower.update()),
                new FollowPathCommand(follower, paths.get(0)),
                new RotateIntake(Intake.IntakeState.DOWN),
                new MoveVSlides(hardware, -400),
                new LimelightAlign(2)
                // new PowerIntake(1)

                /*
                 new FollowPathCommand(follower, pathChain),
                new FollowPathCommand(follower, path)
                 */
        );




        while (opModeInInit()) {
            telemetry.addData("paths", paths.size());
            telemetry.update();
        }
    }

    @Override
    public void run() {
        super.run();
    }
}
