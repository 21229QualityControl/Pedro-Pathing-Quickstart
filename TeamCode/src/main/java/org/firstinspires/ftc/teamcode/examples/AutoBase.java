package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.AutoActionScheduler;

public abstract class AutoBase extends LinearOpMode {
    protected AutoActionScheduler sched;
    protected ElapsedTime timer;
    protected Follower follower;

    public void runOpMode() throws InterruptedException{
        this.sched = new AutoActionScheduler(this::update, hardwareMap);
        this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.follower = new Follower(hardwareMap);

        onRun();
    }

    final public void update() {
        telemetry.addData("Time left", 30 - getRuntime());
    }

    protected abstract void onRun();
}
