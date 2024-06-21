package org.firstinspires.ftc.teamcode.pedroPathing.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
public class WaitPositionCommand implements Action {
    private final Follower follower;
    private final double position;
    private final boolean x;
    private final boolean lt;

    public WaitPositionCommand(Follower follower, double position, boolean lt, boolean x) {
        this.follower = follower;
        this.position = position;
        // lt stands for "less than"
        this.lt = lt;
        this.x = x;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        double val;
        if (x) {
            val = follower.getPose().position.x;
        } else {
            val = follower.getPose().position.y;
        }
        if (lt) {
            return val < position;
        }
        return val > position;
    }
}