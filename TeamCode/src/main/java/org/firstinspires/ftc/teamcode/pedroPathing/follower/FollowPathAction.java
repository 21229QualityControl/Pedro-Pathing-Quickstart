package org.firstinspires.ftc.teamcode.pedroPathing.follower;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FollowPathAction implements Action {

    private final Follower m_follower;

    private Path m_path;

    private PathChain m_pathChain;
    private boolean holdEnd = false;

    private boolean started = false;

    private Point failsafe = new Point(0, 0, Point.CARTESIAN);

    private Path failsafePath;

    private PathChain failsafePathChain;

    private Path alternatePath;

    private boolean interfered = false;

    private boolean useFailsafe = false;

    private boolean lowVelocity = false;

    private double waitUntil = 1000;

    public FollowPathAction(Follower follower, Path path, boolean holdEnd) {
        this.m_follower = follower;
        this.m_path = path;
        this.holdEnd = holdEnd;
        this.useFailsafe = false;
    }

    public FollowPathAction(Follower follower, Path path) {
        this.m_follower = follower;
        this.m_path = path;
        this.useFailsafe = false;
    }

    public FollowPathAction(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.useFailsafe = false;
    }

    public FollowPathAction(Follower follower, PathChain pathChain) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
        this.useFailsafe = false;
    }

    // A follow path action for scenarios where the robot
    // may get stuck or interfered somewhere (like when it is driving under the truss).
    // TODO: The failsafe path actions don't work properly yet. This will be fixed soon.
    public FollowPathAction(Follower follower, PathChain pathChain, Point failsafe, Path alternatePath) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
        this.failsafe = failsafe;
        this.alternatePath = alternatePath;
        this.useFailsafe = true;
    }


    @Override
    public boolean run(TelemetryPacket packet) {
        Log.d("Robot Velocity", Double.toString(Math.abs(m_follower.getVelocity().getMagnitude())));
        Log.d("Robot Acceleration", Double.toString(m_follower.getAcceleration().getMagnitude()));

        // If the robot was moving at a high velocity and suddenly the velocity lowered,
        // then a timer is set to determine if the robot is actually interfered.
        if (!lowVelocity && (Math.abs(m_follower.getVelocity().getMagnitude()) < 0.2)) {
            this.waitUntil = System.currentTimeMillis() + waitUntil;
            lowVelocity = true;
            Log.d("lowVelocity:","true");
        }

        // If the robot has started following the path and has
        // low velocity for a long time, then it is interfered and needs to follow the failsafe path.
        // This will only happen if useFailsafe is true
        if ((lowVelocity == true) && (System.currentTimeMillis() > waitUntil)
                && (started == true) && useFailsafe == true) {
            interfered = true;
            Log.d("Interfered:","true");
            // Stop following the path, create the failsafe path
            m_follower.breakFollowing();
            failsafePath = new Path(new BezierLine(
                    new Point(m_follower.getPose().position.x, m_follower.getPose().position.y, Point.CARTESIAN),
                    failsafe
            ));
            // Create a new path chain with the failsafe path and the alternate path
            failsafePathChain = m_follower.pathBuilder().addPath(failsafePath).addPath(alternatePath).build();
        }

        if (!interfered) { // If the robot is not interfered, follow the original path(chain).
            Log.d("Interefered:","false");
            if (!started) {
                if (m_path != null) {
                    m_follower.followPath(m_path, holdEnd);
                } else if (m_pathChain != null) {
                    m_follower.followPath(m_pathChain, holdEnd);
                }
                started = true;
            }
        } else {
            // If the robot has been interfered, follow the failsafe pathchain.
            // Then, the robot will stop having low velocity.
           m_follower.followPath(failsafePathChain, holdEnd);
           lowVelocity = false;
        }

        m_follower.update();

        return m_follower.isBusy();
    }
}
