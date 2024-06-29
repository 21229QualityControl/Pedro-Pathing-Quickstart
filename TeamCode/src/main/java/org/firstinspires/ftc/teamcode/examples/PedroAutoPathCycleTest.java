package org.firstinspires.ftc.teamcode.examples;

import android.util.Log;

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

import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DrivePoseLoggingAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseMessage;
import org.firstinspires.ftc.teamcode.pedroPathing.util.WaitPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;


@Config
@Autonomous(name = "Blue Pedro Cycle Test",group = "Test")
public final class PedroAutoPathCycleTest extends LinearOpMode {
    public static double y_position = 9;
    public static Point[] backdrop = {
            new Point(51.5, 28, Point.CARTESIAN),
            new Point(51.5, 34.5, Point.CARTESIAN),
            new Point(51.5,38.0, Point.CARTESIAN)
    };
    public static Point scoring = new Point(47, 28, Point.CARTESIAN);
    public static Point scoringHigh = new Point(46, 28, Point.CARTESIAN);
    public static Point start = new Point(12, 62, Point.CARTESIAN);
    public static Point[] spike = {
            new Point(9, 35, Point.CARTESIAN),
            new Point(28, 22, Point.CARTESIAN),
            new Point(32, 35, Point.CARTESIAN)
    };
    public static Point stack = new Point(-60, 8, Point.CARTESIAN);
    public static Point secondStack = new Point(-58.5, 15, Point.CARTESIAN);

    protected AutoActionScheduler sched;
    protected Outtake outtake;
    protected Intake intake;
    protected Follower follower;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sched = new AutoActionScheduler(this::update, hardwareMap);

        this.outtake = new Outtake(hardwareMap);
        this.intake = new Intake(hardwareMap);
        outtake.initialize(false);
        intake.initialize();
        outtake.prepInitializeSlides();
        sched.addAction(outtake.clawSingleClosed());
        sched.run();

        while(!isStarted() && !isStopRequested()) {
            outtake.update();
            intake.update();
            telemetry.addLine("Ready to start!! Blue Auto Pedro Pathing Test ...");
            telemetry.update();
        }

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        this.follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(start.getX(), start.getY(), Math.toRadians(-90)));

        // make the scoring spike path
        // TODO: add randomization and vision code to this
        Path purplePath = new Path(
                new BezierCurve(new Point(start.getX(), start.getY(), Point.CARTESIAN),
                        spike[2]));

        purplePath.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180), 0.8);
        purplePath.setZeroPowerAccelerationMultiplier(4);
        // score purple spike
        sched.addAction(outtake.extendOuttakeBarelyOut());
        sched.addAction(intake.wristPreload());
        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "purple_path_begin", true),
                        new FollowPathAction(follower, purplePath, false),
                        new DrivePoseLoggingAction(follower, "purple_path_end")
                ));
        sched.addAction(intake.wristStored());
        sched.run();

        Pose2d currentPose = follower.getPose();
        Log.d("purplePoseX:", Double.toString(currentPose.position.x));
        Log.d("purplePoseY:", Double.toString(currentPose.position.y));
        // create yellow pixel path
        Path yellowPath = new Path(new BezierLine(new Point(currentPose), backdrop[2]));
        yellowPath.setConstantHeadingInterpolation(Math.toRadians(180));
        yellowPath.setZeroPowerAccelerationMultiplier(5);

        follower.update();
        sched.addAction(new ParallelAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "yellow_path_begin"),
                        new FollowPathAction(follower, yellowPath),
                        new DrivePoseLoggingAction(follower, "yellow_path_end"
                )),
                new SequentialAction(
                    outtake.extendOuttakeCloseBlocking(),
                    outtake.armScoring(),
                    outtake.wristVerticalFlip()
                )
        ));
        sched.addAction(outtake.clawOpen());
        sched.addAction(new SleepAction(0.1));
        sched.run();

        intakeStack(true, false, false);
        cycle(false, false);

        intakeStack(false, false, false);
        cycle(false, false);

        intakeStack(false, true, false);
        cycle(false, true);

        intakeStack(false, true, true);
        cycle(true, true);

        boolean firstTime = true;
        Pose2d endPose = follower.getPose();
        while(!isStopRequested() ) {
            if(sched.isEmpty()) {
                if(firstTime) {
                    follower.update();
                    endPose = follower.getPose();
                    firstTime = false;
                    Log.d("Drive_logger", "End drive pose: " + new PoseMessage(endPose));
                    Log.d("Drive_logger","Auto elapsed time (ms): " + String.format("%3.3f",timer.milliseconds()));
                }

                telemetry.addData("Auto elapsed time (ms): ", String.format("%3.3f",timer.milliseconds()));
                telemetry.addData("End Pose: ", new PoseMessage(endPose).toString());
                telemetry.update();
            }
            idle();
        }
    }

    final public void update() {
        intake.update();
        outtake.update();

        telemetry.addData("Time left", 30 - getRuntime());
    }

    private void intakeStack(boolean first, boolean nextStack, boolean lastCycle) {

        Point stagePoint = new Point(follower.getPose().position.x, follower.getPose().position.y, Point.CARTESIAN);
        // create path to go to stack
        Path toStack = null;
        Path toNextStack = null;
        Path pastTruss = null;

        if (nextStack) {
            Log.d("nextStack", "Next Stack Path");

            pastTruss = new Path(new BezierCurve(stagePoint,
                    new Point(45, y_position, Point.CARTESIAN),
                    new Point(-24, y_position, Point.CARTESIAN)
            ));

            toNextStack = new Path(new BezierCurve(
                    new Point(-24, y_position, Point.CARTESIAN),
                    new Point(-50, y_position, Point.CARTESIAN),
                    secondStack
            ));
        }
        else {
            toStack = new Path(new BezierCurve(stagePoint,
                    new Point(45, y_position, Point.CARTESIAN),
                    new Point(24, y_position, Point.CARTESIAN),
                    new Point(-24, y_position, Point.CARTESIAN),
                    stack));
        }

        Log.d("heading:", Double.toString(follower.getPose().heading.toDouble()));
        // drive to stack, retract outtake, start intake
        SequentialAction stackAction;
        if (nextStack) {
            Log.d("num intaked:", Double.toString(intake.numIntaked));
            if (lastCycle) {
                intake.numIntaked = 3;
            }
            pastTruss.setZeroPowerAccelerationMultiplier(4);
            pastTruss.setConstantHeadingInterpolation(Math.PI);
            toNextStack.setZeroPowerAccelerationMultiplier(4);
            toNextStack.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150), 0.8);

            PathChain chain = follower.pathBuilder()
                    .addPath(pastTruss)
                    .addPath(toNextStack).build();

            stackAction = new SequentialAction(
                    new DrivePoseLoggingAction(follower, "stack_path_begin"),
                    new FollowPathAction(follower, chain),
                    new DrivePoseLoggingAction(follower, "stack_path_end")
            );
        }
        else {
            toStack.setZeroPowerAccelerationMultiplier(4);
            toStack.setConstantHeadingInterpolation(Math.PI);
            stackAction = new SequentialAction(
                    new DrivePoseLoggingAction(follower, "stack_path_begin"),
                    new FollowPathAction(follower, toStack),
                    new DrivePoseLoggingAction(follower, "stack_path_end")
            );
        }
        sched.addAction(new ParallelAction(
                stackAction,
                new SequentialAction(
                        new WaitPositionCommand(follower, 48, false, true), // Away from backdrop
                        new SequentialAction(
                                outtake.wristVertical(),
                                outtake.armStored(),
                                outtake.retractOuttakeBlocking()
                        ),
                        new WaitPositionCommand(follower, 24, false, true), // intermediate
                        new SequentialAction(
                                // the intake will only start from the top of that stack during the
                                // very first cycle and the first cycle of the next stack.
                                intake.prepIntakeCount(first || (nextStack && !lastCycle), false),
                                intake.intakeSlow(), // this actually makes the intake go full speed
                                outtake.extendOuttakeBarelyOut()
                        )
                )
        ));
        sched.addAction(new SequentialAction(
                intake.intakeCount(false),
                outtake.retractOuttake()
        ));
        sched.run();
        Log.d("heading:", Double.toString(follower.getPose().heading.toDouble()));
    }

    private void cycle(boolean second, boolean nextStack) {
        Point stackPoint = new Point(follower.getPose().position.x, follower.getPose().position.y, Point.CARTESIAN);
        // create path to get to backdrop
        SequentialAction scoringAction;
        if (nextStack) {
            Path toTruss = new Path(new BezierCurve(stackPoint,
                    new Point(-48, 13, Point.CARTESIAN)
            ));

            toTruss.setZeroPowerAccelerationMultiplier(4);
            toTruss.setReversed(true);
            toTruss.setConstantHeadingInterpolation(Math.toRadians(150));

            Path toBackstage = new Path(new BezierCurve(
                    new Point(-48, y_position, Point.CARTESIAN),
                    new Point(40, y_position, Point.CARTESIAN),
                    new Point(40, 28, Point.CARTESIAN),
                    scoringHigh
            ));

            toBackstage.setZeroPowerAccelerationMultiplier(4);
            toBackstage.setReversed(true);
            toBackstage.setConstantHeadingInterpolation(Math.toRadians(180));

            PathChain scoringPath = follower.pathBuilder().addPath(toTruss).addPath(toBackstage).build();

            scoringAction = new SequentialAction(new FollowPathAction(follower, scoringPath));
        } else {
            Path scoringPath = new Path(new BezierCurve(
                    stackPoint,
                    new Point(-50, 8, Point.CARTESIAN),
                    new Point(24, 9, Point.CARTESIAN),
                    new Point(32, 9, Point.CARTESIAN),
                    scoring
            ));
            scoringPath.setReversed(true);
            scoringPath.setConstantHeadingInterpolation(Math.toRadians(180));
            scoringPath.setZeroPowerAccelerationMultiplier(5);

            scoringAction = new SequentialAction(new FollowPathAction(follower, scoringPath));
        }

        // drive to backdrop, score pixels
        sched.addAction(new ParallelAction(
                scoringAction,
                new SequentialAction(
                        new WaitPositionCommand(follower, -36, true, true), // pastTruss
                        new SequentialAction(
                                intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                                intake.intakeOff()
                        ),
                        new WaitPositionCommand(follower, 8, true, true), // intermediate
                        new SequentialAction(
                                second ? outtake.extendOuttakeCycleHighBlocking() : outtake.extendOuttakeCycleBlocking(),
                                outtake.armScoring(),
                                intake.feedOpen()
                        )
                )
        ));
        sched.addAction(outtake.clawOpen());
        sched.addAction(intake.feedClosed()); // Open & close feed in case outtake doesn't grab
        sched.run();
    }
}
