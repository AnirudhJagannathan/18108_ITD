package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class LimelightStrafe extends SequentialCommandGroup {

    private final Follower follower;
    public LimelightStrafe(Follower follower, double tx) {
        super(
                new InstantCommand(() -> new FollowPathCommand(follower,
                        follower.pathBuilder().addPath(
                                new Path(new BezierLine(
                                        new Point(follower.getPose()),
                                        new Point(new Pose(follower.getPose().getX(), follower.getPose().getY() + tx))
                                ))
                        ).build()
                ))
        );

        this.follower = follower;
    }

    public boolean isFinished() {
        return !follower.isBusy();
    }
}
