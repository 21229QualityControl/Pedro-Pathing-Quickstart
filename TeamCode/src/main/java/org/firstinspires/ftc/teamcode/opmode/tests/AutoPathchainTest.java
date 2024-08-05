package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.examples.AutoBase;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.util.WaitPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

// Just testing the capabilities of path chains.
@Config
@Autonomous(name = "Pathchain Test",group = "Test")
public final class AutoPathchainTest extends AutoBase {
    public static Point start = new Point(-36, 62, Point.CARTESIAN);
    public static Point intermediate = new Point(-36, 32, Point.CARTESIAN);
    public static Point end = new Point(-36, 50, Point.CARTESIAN);

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(start.getX(), start.getY(), Math.toRadians(-90));
    }

    @Override
    protected void printDescription() { telemetry.addData("Description", "Blue Right Auto 2+7"); }

    @Override
    protected void onRun() {
        firstCycle();
    }

    private void firstCycle() {
        follower.setStartingPose(new Pose2d(start.getX(), start.getY(), Math.toRadians(-90)));

        Path firstPath = new Path(new BezierCurve(
            start,
            intermediate
        ));

        firstPath.setConstantHeadingInterpolation(Math.toRadians(-90));

        Path secondPath = new Path(new BezierCurve(
            intermediate,
            end
        ));

        secondPath.setConstantHeadingInterpolation(Math.toRadians(-90));

        PathChain pathchain = follower.pathBuilder().addPath(firstPath).addPath(secondPath).build();

        sched.addAction(new FollowPathAction(follower, pathchain));

        sched.run();
    }
}
