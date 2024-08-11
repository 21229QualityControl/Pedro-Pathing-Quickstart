package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.secondaryHeadingPIDFFeedForward;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.secondaryTranslationalPIDFFeedForward;
import static org.firstinspires.ftc.teamcode.util.control.PIDFControllerKt.EPSILON;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Memory;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.GamePadController;
import org.firstinspires.ftc.teamcode.util.LED;
import org.firstinspires.ftc.teamcode.util.SmartGameTimer;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
//import org.firstinspires.ftc.teamcode.util.control.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Config
@TeleOp(name = "Strafing Enhancements", group = "Drive")
public class TeleOpStrafingEnhancements extends OpMode {
    public static double desiredHeading;

    private GamePadController g1;
    private Follower follower;
    private boolean isStrafeEnhanced = false;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        g1 = new GamePadController(gamepad1);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        g1.update();
        if (g1.startOnce()) { // toggle between strafe enhancing mode and normal driving
            if (isStrafeEnhanced == false) {
                isStrafeEnhanced = true;
                // the desired heading and x position is recorded only once when the start button is pressed
                desiredHeading = follower.getPose().heading.toDouble();
            } else {
                // if strafing enhancement is already on, turn it off. Driving will go back to normal
                isStrafeEnhanced = false;
            }
        }

        if (isStrafeEnhanced == true) {
            strafeEnhancement();
        } else {
            // Normal driving
            follower.setTeleOpMovementVectors(-g1.left_stick_y, -g1.left_stick_x, -gamepad1.right_stick_x, true, false);
            follower.update();
        }
    }

    private void strafeEnhancement() {
        double input_x;
        double input_y;

        input_x = Math.pow(-g1.left_stick_y, 3);
        input_y = Math.pow(-g1.left_stick_x, 3);

        Pose2d currentPose = follower.getPose();
        double currentHeading = currentPose.heading.toDouble();

        double headingError = MathFunctions.getSmallestAngleDifference(desiredHeading, currentHeading)
                * MathFunctions.getTurnDirection(currentHeading, desiredHeading);

        follower.setTeleOpMovementVectors(input_x, input_y, headingError, true, true);
        follower.update();
    }
}
