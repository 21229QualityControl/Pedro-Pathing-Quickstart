package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.examples.AutoBase;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.WaitPositionCommand;

@Config
@Autonomous(name = "Blue Right 2+7", group = "Test")
public class BlueRight2_7 extends AutoBase {
    public static Point start = new Point(-36, 62, Point.CARTESIAN);
    public static Point[] backdrop = {
            new Point(51.5, 28, Point.CARTESIAN),
            new Point(51.5, 34.5, Point.CARTESIAN),
            new Point(51.5,38.0, Point.CARTESIAN)
    };
    public static Point scoring = new Point(48, 40, Point.CARTESIAN);
    public static Point scoringHigh = new Point(47, 40, Point.CARTESIAN);
    public static Point[] spike = {
            new Point(-48, 30, Point.CARTESIAN),
            new Point(28, 22, Point.CARTESIAN), // the last 2 positions aren't accurate
            new Point(32, 35, Point.CARTESIAN)
    };
    public static Point spikeBackedOut = new Point(-48, 50, Point.CARTESIAN);
    public static Point intermediate = new Point(-36, 60, Point.CARTESIAN);
    public static Point pastTruss = new Point(30, 60, Point.CARTESIAN);
    public static Point stack = new Point(-58.5, 37, Point.CARTESIAN);
    public static Point secondStack = new Point(-58.5, 15, Point.CARTESIAN);

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(start.getX(), start.getY(), Math.toRadians(-90));
    }

    @Override
    protected void printDescription() { telemetry.addData("Description", "Blue Right Auto 2+7"); }

    @Override
    protected void onRun() {
        firstCycle();

        intakeStack(false, false, false);
        cycle(false, false);

        intakeStack(false, false, false);
        cycle(false, true);

        intakeStack(false, true, true);
        cycle(true, true);
    }

    private void firstCycle() {
        scorePreload();

        Pose2d currentPose = follower.getPose();
        // Create path to stack
        Path toStack = new Path(new BezierLine(
                new Point(currentPose.position.x, currentPose.position.y, Point.CARTESIAN),
                stack));
        toStack.setZeroPowerAccelerationMultiplier(4);
        toStack.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(205), 0.8);

        sched.addAction(new ParallelAction(
                new FollowPathAction(follower, toStack),
                new SequentialAction(
                        outtake.extendOuttakeBarelyOut(),
                        intake.prepIntakeCount(true, true)
                )
        ));

        sched.addAction(intake.intakeCount(true));
        sched.run();

        currentPose = follower.getPose();
        Path toTruss = new Path(new BezierLine(
                new Point(currentPose.position.x, currentPose.position.y, Point.CARTESIAN),
                intermediate));
        toTruss.setZeroPowerAccelerationMultiplier(4);
        toTruss.setReversed(true);
        toTruss.setConstantHeadingInterpolation(Math.toRadians(205));

        Path toBackstage = new Path(new BezierCurve(
                intermediate,
                pastTruss,
                new Point(40, 40, Point.CARTESIAN),
                scoring
        ));
        toBackstage.setZeroPowerAccelerationMultiplier(4);
        toBackstage.setReversed(true);
        toBackstage.setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(180), 0.05);

        PathChain scoringPath = follower.pathBuilder().addPath(toTruss).addPath(toBackstage).build();

        sched.addAction(new ParallelAction(
                new FollowPathAction(follower, scoringPath),
                new SequentialAction(
                        new WaitPositionCommand(follower, -36, true, true),
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed(),
                        new WaitPositionCommand(follower, 30, true, true),
                        intake.intakeOff(),
                        outtake.extendOuttakePartnerBlocking(),
                        outtake.armScoring(),
                        outtake.wristVerticalFlip(),
                        intake.feedOpen()
                )
        ));
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed());
        sched.run();
    }

    private void scorePreload() {
        follower.setStartingPose(new Pose2d(start.getX(), start.getY(), Math.toRadians(-90)));

        Path purplePath = new Path(
                new BezierLine(start,
                spike[SPIKE]
        ));

        purplePath.setConstantHeadingInterpolation(Math.toRadians(-90));
        purplePath.setZeroPowerAccelerationMultiplier(4);

        sched.addAction(outtake.extendOuttakeBarelyOut());
        sched.addAction(intake.wristPreload());

        sched.addAction(new FollowPathAction(follower, purplePath));
        sched.addAction(intake.wristStored());
        sched.run();

        Path backOut = new Path(
                new BezierLine(spike[SPIKE],
                        spikeBackedOut)
        );

        backOut.setReversed(true);
        backOut.setConstantHeadingInterpolation(Math.toRadians(-90));
        backOut.setZeroPowerAccelerationMultiplier(4);

        sched.addAction(new FollowPathAction(follower, backOut));
        sched.run();
    }

    private void intakeStack(boolean first, boolean nextStack, boolean lastCycle) {
        Pose2d currentPose = follower.getPose();

        Path thruTruss = new Path(new BezierCurve(
                new Point(currentPose.position.x, currentPose.position.y, Point.CARTESIAN),
                pastTruss,
                intermediate
        ));

        thruTruss.setZeroPowerAccelerationMultiplier(4);
        thruTruss.setConstantHeadingInterpolation(Math.toRadians(180));

        Path toStack = new Path(new BezierLine(
                intermediate,
                stack
        ));

        toStack.setZeroPowerAccelerationMultiplier(4);
        toStack.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(205), 0.8);

        PathChain intakePath = follower.pathBuilder().addPath(thruTruss).addPath(toStack).build();

        sched.addAction(new ParallelAction(
                new SequentialAction(
                        new WaitPositionCommand(follower, 48, false, true), // Back out from backdrop
                        new SequentialAction(
                                outtake.wristVertical(),
                                outtake.armStored(),
                                intake.feedClosed(),
                                new SleepAction(0.3),
                                outtake.clawOpen(),
                                outtake.retractOuttakeBlocking()
                        ),
                        new WaitPositionCommand(follower, -36, false, true), // intermediate
                        intake.prepIntakeCount(false, false)
                ),
                new FollowPathAction(follower, intakePath)
        ));
        sched.run();

        sched.addAction(intake.intakeCount(true));
        sched.run();
    }

    private void cycle(boolean second, boolean lastCycle) {
        Pose2d currentPose = follower.getPose();

        Path toTruss = new Path(new BezierLine(
                new Point(currentPose.position.x, currentPose.position.y, Point.CARTESIAN),
                intermediate));
        toTruss.setZeroPowerAccelerationMultiplier(4);
        toTruss.setReversed(true);
        toTruss.setConstantHeadingInterpolation(Math.toRadians(205));

        Path toBackstage = new Path(new BezierCurve(
                intermediate,
                pastTruss,
                new Point(40, 40, Point.CARTESIAN),
                scoring
        ));
        toBackstage.setZeroPowerAccelerationMultiplier(4);
        toBackstage.setReversed(true);
        toBackstage.setLinearHeadingInterpolation(Math.toRadians(205), Math.toRadians(180), 0.05);

        PathChain scoringPath = follower.pathBuilder().addPath(toTruss).addPath(toBackstage).build();

        sched.addAction(new ParallelAction(
                new FollowPathAction(follower, scoringPath),
                new SequentialAction(
                        new WaitPositionCommand(follower, -36, true, true),
                        intake.pixelCount() == 2 ? outtake.clawClosed() : outtake.clawSingleClosed(),
                        new WaitPositionCommand(follower, 30, true, true),
                        intake.intakeOff(),
                        outtake.extendOuttakePartnerBlocking(),
                        outtake.armScoring(),
                        outtake.wristVerticalFlip(),
                        intake.feedOpen()
                )
        ));
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed());
        sched.run();
    }
}
