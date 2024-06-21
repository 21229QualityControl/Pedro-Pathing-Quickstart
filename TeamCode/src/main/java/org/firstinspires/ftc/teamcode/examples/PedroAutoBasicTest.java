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
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


@Config
@Autonomous(name = "Basic Auto Test",group = "Test")
public final class PedroAutoBasicTest extends LinearOpMode {

    protected AutoActionScheduler sched;
    protected Outtake outtake;
    protected Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Memory.LAST_POSE = new Pose2d(0, 0, 0);;
        Memory.RAN_AUTO = true;

        sched = new AutoActionScheduler(this::update, hardwareMap);

        this.outtake = new Outtake(hardwareMap);
        this.intake = new Intake(hardwareMap);
        outtake.initialize(false);
        intake.initialize();
        outtake.prepInitializeSlides();
        sleep(200);

        while (opModeInInit() && outtake.initializeSlides()) {}

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

        // score purple spike
//        sched.addAction(outtake.extendOuttakeCycleBlocking());
        // ??? one line removed ???
        sched.addAction(intake.wristPreload());
        sched.run();

        sched.addAction(outtake.extendOuttakeCloseBlocking());
        sched.run();

//        boolean firstTime = true;
//        Pose2d endPose = follower.getPose();
        while(!isStopRequested() ) {
//            if(sched.isEmpty()) {
//                if(firstTime) {
//                    follower.update();
//                    endPose = follower.getPose();
//                    firstTime = false;
//                    Log.d("Drive_logger", "End drive pose: " + new PoseMessage(endPose));
//                    Log.d("Drive_logger","Auto elapsed time (ms): " + String.format("%3.3f",timer.milliseconds()));
//                }
//
                telemetry.addData("Auto elapsed time (ms): ", String.format("%3.3f",timer.milliseconds()));
//                telemetry.addData("End Pose: ", new PoseMessage(endPose).toString());
                telemetry.update();
//            }
            idle();
        }
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
    }
}
