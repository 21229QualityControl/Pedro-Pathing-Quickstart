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


@Config
@Autonomous(name = "Blue Pedro Cycle Test",group = "Test")
public final class PedroAutoPathCycleTest extends LinearOpMode {
    public static Pose2d starting = new Pose2d(14.5, -62.0, Math.PI/2);
    public static Pose2d backdrop = new Pose2d(48.5, -36.0, Math.PI);
    public static Pose2d spike = new Pose2d(28.5, -24.5, Math.PI);
    public static Pose2d parking = new Pose2d(52.0, -60.0, Math.PI);

    protected AutoActionScheduler sched;
    protected Outtake outtake;
    protected Intake intake;

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

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose2d(12, 62, Math.toRadians(-90)));

        Point backdrop = new Point(48.0,36.0, Point.CARTESIAN);
        Point cycle = new Point(48.0,30.0, Point.CARTESIAN);
        Point cycle1 = new Point(45.0,30.0, Point.CARTESIAN);

        // make the scoring spike path
        // TODO: add randomization and vision code to this
        Path purplePath = new Path(
                new BezierCurve(new Point(12, 62.0, Point.CARTESIAN),
                        new Point(32.0, 23.0, Point.CARTESIAN)));

        purplePath.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180), 0.8);
        purplePath.setZeroPowerAccelerationMultiplier(4);
        // score purple spike
        sched.addAction(outtake.extendOuttakeBarelyOut());
        sched.addAction(intake.wristPreload());
        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "purple_path_begin", true),
                        new FollowPathAction(follower, purplePath, false),
                        new DrivePoseLoggingAction(follower, "purple_path_end"),
                        new SleepAction(0.5)
                ));
        sched.addAction(intake.wristStored());
        sched.run();

        Pose2d currentPose = follower.getPose();
        //follower.setStartingPose(currentPose);

        // create yellow pixel path
        Path yellowPath = new Path(new BezierLine(new Point(currentPose), backdrop));
//        yellowPath.setLinearHeadingInterpolation(currentPose.heading.toDouble(), Math.toRadians(180));
        yellowPath.setConstantHeadingInterpolation(Math.toRadians(180));
        yellowPath.setZeroPowerAccelerationMultiplier(5);
        //yellowPath.setReversed(true);

        follower.update();
        // score yellow pixel
        sched.addAction(
                new SequentialAction(
                        new DrivePoseLoggingAction(follower, "yellow_path_begin"),
                        new FollowPathAction(follower, yellowPath),
                        new DrivePoseLoggingAction(follower, "yellow_path_end"),
                        new SleepAction(1.25),
                        outtake.extendOuttakeCloseBlocking(),
                        outtake.armScoring(),
                        outtake.wristVerticalFlip()));
        sched.addAction(outtake.clawOpen());
        sched.run();

        int cycleCount = 0;

        double y_position = 9;
        while(cycleCount++ < 3) {

            Pose2d backdrop0 = follower.getPose();

            if(cycleCount == 2) {
//                y_position0 = 10;
//                y_position = 10.5;
                follower.setPose(new Pose2d(backdrop0.position.x, backdrop0.position.y,backdrop0.heading.toDouble()));
            }
            if(cycleCount == 3) {
//                y_position0 = 10.5;
//                y_position = 11.0;
                follower.setPose(new Pose2d(backdrop0.position.x, backdrop0.position.y,backdrop0.heading.toDouble()));
            }

            Point stagePoint = new Point(backdrop0.position.x, backdrop0.position.y, Point.CARTESIAN);
            // create path to go to stack
            Path toStack = new Path(new BezierCurve(stagePoint,
                    new Point(45, y_position, Point.CARTESIAN),
                    new Point(24, y_position, Point.CARTESIAN),
//                    new Point(-0, y_position, Point.CARTESIAN)));
                    new Point(-24, y_position, Point.CARTESIAN),
                    new Point(-54, y_position, Point.CARTESIAN)));

            toStack.setZeroPowerAccelerationMultiplier(5);
            toStack.setConstantHeadingInterpolation(Math.PI);

            Log.d("heading:", Double.toString(follower.getPose().heading.toDouble()));
            // drive to stack, retract outtake, start intake
            sched.addAction(new ParallelAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stack_path_begin"),
                            new FollowPathAction(follower, toStack),
                            new DrivePoseLoggingAction(follower, "stack_path_end"),
                            new SleepAction(1.5)
                    ),
                    new SequentialAction(
                            new WaitPositionCommand(follower, 48, false, true), // Away from backdrop
                            new SequentialAction(
                                    outtake.wristVertical(),
                                    outtake.armStored(),
                                    outtake.retractOuttakeBlocking()
                            ),
                            new WaitPositionCommand(follower, 24, false, true), // intermediate
                            new SequentialAction(
//                                    intake.prepIntakeCount(first || nextStack, false),
                                    intake.prepIntakeCount(true, false),
//                                    nextStack ? intake.intakeOn() : intake.intakeSlow(),
                                    intake.intakeOn(),
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

            Pose2d stack0 = follower.getPose();
            Point stackPoint = new Point(stack0.position.x, stack0.position.y, Point.CARTESIAN);
            // make path to backdrop
            Path toStage = new Path(new BezierCurve(stackPoint,
                    new Point(8, 9, Point.CARTESIAN),
                    new Point(32, 9, Point.CARTESIAN),
//                    cycle1,
                    cycle));

            toStage.setReversed(true);
            toStage.setConstantHeadingInterpolation(Math.toRadians(180));
            toStage.setZeroPowerAccelerationMultiplier(5);

            // drive to backdrop, score pixels
            sched.addAction(new ParallelAction(
                    new SequentialAction(
                            new DrivePoseLoggingAction(follower, "stage_path_begin"),
                            new FollowPathAction(follower, toStage),
                            new DrivePoseLoggingAction(follower, "stage_path_end"),
                            new SleepAction(1.25),
                            new DrivePoseLoggingAction(follower, "cycle_end")
                    ),
                    new SequentialAction(
                            new WaitPositionCommand(follower, -36, true, true), // pastTruss
                            new SequentialAction(
                                    intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                                    intake.intakeOff()
                            ),
                            new WaitPositionCommand(follower, 21, true, true), // intermediate
                            new SequentialAction(
//                                    second ? outtake.extendOuttakeCycleHighBlocking() : outtake.extendOuttakeCycleBlocking(),
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
        telemetry.addData("Time left", 30 - getRuntime());
    }
}
