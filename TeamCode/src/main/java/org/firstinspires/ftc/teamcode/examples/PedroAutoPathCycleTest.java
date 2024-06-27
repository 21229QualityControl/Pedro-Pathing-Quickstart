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
    public static Pose2d starting = new Pose2d(14.5, -62.0, Math.PI/2);
    public static Pose2d backdrop = new Pose2d(48.5, -36.0, Math.PI);
    public static Pose2d spike = new Pose2d(28.5, -24.5, Math.PI);
    public static Pose2d parking = new Pose2d(52.0, -60.0, Math.PI);
    int cycleCount = 0;
    double y_position = 9;

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
        follower.setStartingPose(new Pose2d(12, 62, Math.toRadians(-90)));

        Point backdrop = new Point(51.0,40.0, Point.CARTESIAN);
        Point cycle = new Point(47.0,30.0, Point.CARTESIAN);

        // make the scoring spike path
        // TODO: add randomization and vision code to this
        Path purplePath = new Path(
                new BezierCurve(new Point(12, 62.0, Point.CARTESIAN),
                        new Point(32.0, 28.0, Point.CARTESIAN)));

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
        //follower.setStartingPose(currentPose);
        Log.d("purplePoseX:", Double.toString(currentPose.position.x));
        Log.d("purplePoseY:", Double.toString(currentPose.position.y));
        // create yellow pixel path
        Path yellowPath = new Path(new BezierLine(new Point(currentPose), backdrop));
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
        cycle(true, true);

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
        Pose2d backdrop0 = follower.getPose();

        Point stagePoint = new Point(backdrop0.position.x, backdrop0.position.y, Point.CARTESIAN);

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
                    new Point(-58.5, 15, Point.CARTESIAN)
            ));
        }
        else {
            toStack = new Path(new BezierCurve(stagePoint,
                    new Point(45, y_position, Point.CARTESIAN),
                    new Point(24, y_position, Point.CARTESIAN),
                    new Point(-24, y_position, Point.CARTESIAN),
                    new Point(-59, 8.0, Point.CARTESIAN)));
        }

        Log.d("heading:", Double.toString(follower.getPose().heading.toDouble()));
        // drive to stack, retract outtake, start intake
        SequentialAction stackAction = null;
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
                                new SleepAction(0.25),
                                outtake.armStored(),
                                outtake.retractOuttakeBlocking()
                        ),
                        new WaitPositionCommand(follower, 24, false, true), // intermediate
                        new SequentialAction(
                                // the intake will only start from the top of that stack at the
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
        Pose2d stack0 = follower.getPose();
        Point stackPoint = new Point(stack0.position.x, stack0.position.y, Point.CARTESIAN);
        // create path to get to backdrop
        SequentialAction scoringAction = null;
        if (nextStack) {
            Path toTruss = new Path(new BezierCurve(stackPoint,
                    new Point(-48, 11, Point.CARTESIAN)
            ));

            toTruss.setZeroPowerAccelerationMultiplier(4);
            toTruss.setReversed(true);
            toTruss.setConstantHeadingInterpolation(Math.toRadians(150));

            Path toBackstage = new Path(new BezierCurve(
                    new Point(-48, y_position, Point.CARTESIAN),
                    new Point(45, y_position, Point.CARTESIAN),
                    new Point(46, 28, Point.CARTESIAN)
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
                    new Point(47, 28, Point.CARTESIAN)
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
                        new WaitPositionCommand(follower, 16, true, true), // intermediate
                        new SequentialAction(
//                                second ? outtake.extendOuttakeCycleHighBlocking() : outtake.extendOuttakeCycleBlocking(),
                                outtake.extendOuttakeCycleBlocking(),
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
