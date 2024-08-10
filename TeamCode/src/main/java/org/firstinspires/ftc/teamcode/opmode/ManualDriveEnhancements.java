package org.firstinspires.ftc.teamcode.opmode;

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
@TeleOp(group = "Drive")
public class ManualDriveEnhancements extends LinearOpMode {
   public static double TURN_SPEED = 0.75;
   public static double DRIVE_SPEED = 1;
   public static double SLOW_TURN_SPEED = 0.3;
   public static double D2_SLOW_TURN = 0.25;
   public static double SLOW_DRIVE_SPEED = 0.3;
   public static double VISION_RANGE = 20;
   public static double VISION_CLOSE_DIST = 5;
   public static double desiredHeading;
   public static double desiredxPos;

   private SmartGameTimer smartGameTimer;
   private GamePadController g1, g2;
   private Follower follower;
   private ActionScheduler sched;
   private Intake intake;
   private Outtake outtake;
//   private Plane plane;
   private LED led;
   private long lastLoopFinish = 0;
   private Vector driveVector;
   private Vector headingVector;
   private boolean isStrafeEnhanced = false;
   private DcMotorEx leftFront;
   private DcMotorEx leftRear;
   private DcMotorEx rightFront;
   private DcMotorEx rightRear;

   private final PIDFController headingPIDF = new PIDFController(FollowerConstants.teleOpHeadingPIDFCoefficients);
   private final PIDFController teleOpTranslationalPIDF = new PIDFController(FollowerConstants.teleOpTranslationalPIDFCoefficients);

   @Override
   public void runOpMode() throws InterruptedException {
      telemetry.addLine("Initializing...");
      telemetry.update();

      // Init
      g1 = new GamePadController(gamepad1);
      g2 = new GamePadController(gamepad2);
      g1.update();
      g2.update();
      sched = new ActionScheduler();
//      drive = new MecanumDrive(hardwareMap, Memory.LAST_POSE);
      intake = new Intake(hardwareMap);
      outtake = new Outtake(hardwareMap);
//      plane = new Plane(hardwareMap);
      led = new LED(hardwareMap);
      follower = new Follower(hardwareMap);
//      headingPid = new PIDFController(HEADING_PID);
      driveVector = new Vector();
      headingVector = new Vector();

      leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
      leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
      rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
      rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

      leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      follower.startTeleopDrive();

      if (Memory.RAN_AUTO) {
         smartGameTimer = new SmartGameTimer(true);
      } else { // No auto memory, pull in slides
         smartGameTimer = new SmartGameTimer(false);
         // TODO: Un comment out the outtake and make it work
         outtake.prepInitializeSlides();
         telemetry.addLine("Initializing slides...");
         telemetry.update();
         sleep(200);
         while (opModeInInit() && outtake.initializeSlides()) {}
      }
      led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

      // Ready!
      telemetry.addLine("Ready!");
      telemetry.update();
      waitForStart();

      // Opmode start
      if (opModeIsActive()) {
         resetRuntime();
         g1.reset();
         g2.reset();

         // Init opmodes
         outtake.initialize(true);
//         plane.initialize();
         intake.initialize();
         smartGameTimer.resetIfStandard();
      }

      // Main loop
      while (opModeIsActive()) {
         g1.update();
         g2.update();

         move();
         intakeControls();
         outtakeControls();
         ledUpdate();

//         follower.updatePoseEstimate();
         sched.update();
         outtake.update();
         intake.update();

         telemetry.addData("Time left", smartGameTimer.formattedString() + " (" + smartGameTimer.status() + ")");
         telemetry.addData("Pixel Count", intake.pixelCount());

         long finish = System.currentTimeMillis();
         telemetry.addData("Sample Rate (Hz) ",1/((double)(finish - lastLoopFinish)/1000.0));
         telemetry.update();

         lastLoopFinish = finish;
      }

      // On termination
      Memory.LAST_POSE = follower.getPose();
   }

   private double prevInputX = 0;
   private void move() {
      // Main driver controls
      double input_x;
      double input_y;

      // TODO: Because of the new Pedro pathing update, adjust the Manual Drive Enhancements controls accordingly
      input_x = Math.pow(-g1.left_stick_y, 3);
      input_y = Math.pow(-g1.left_stick_x, 3);
      Log.d("input_x", Double.toString(input_x));
      Log.d("input_y", Double.toString(input_y));

      Vector2d input = new Vector2d(input_x, input_y);

      double input_turn = Math.pow(g1.left_trigger - g1.right_trigger, 3) * TURN_SPEED;
      if (g1.leftBumper()) input_turn += SLOW_TURN_SPEED;
      if (g1.rightBumper()) input_turn -= SLOW_TURN_SPEED;

//       Driver 2 slow strafe
      input = input.plus(new Vector2d(g2.left_stick_y * SLOW_DRIVE_SPEED, g2.left_stick_x * SLOW_DRIVE_SPEED));
      input_turn += g2.left_trigger * D2_SLOW_TURN;
      input_turn -= g2.right_trigger * D2_SLOW_TURN;

      if (Math.abs(g2.left_stick_x) != 0 && Math.abs(prevInputX) < EPSILON) {
         headingPIDF.setTargetPosition(follower.getPose().heading.toDouble());
      }
      prevInputX = g2.left_stick_x;
      if (Math.abs(input_turn) > EPSILON) {
         prevInputX = 0;
      }
      if (Math.abs(g1.left_stick_x + g1.left_stick_y + input_turn) < EPSILON && Math.abs(g2.left_stick_x) > 0) { // Do heading lock
//         input_turn = headingPIDF.updateError(follower.getPose().heading.toDouble());
         input_turn = headingPIDF.runPIDF();
         if (g2.left_stick_x > 0) { // Account for heading turn overpowering strafe
            input = input.plus(new Vector2d(0, Math.abs(input_turn)));
         } else {
            input = input.plus(new Vector2d(0, -Math.abs(input_turn)));
         }
      }

      if (g1.startOnce()) { // toggle between strafe enhancing mode and normal driving
         if (isStrafeEnhanced == false) {
            isStrafeEnhanced = true;
            // the desired heading and x position is recorded only once when the start button is pressed
//            desiredHeading = MathFunctions.normalizeAngle(follower.getPose().heading.toDouble());
            desiredHeading = follower.getPose().heading.toDouble();
            desiredxPos = follower.getPose().position.x;
         } else {
            // if strafing enhancement is already on, turn it off. Driving will go back to normal
            isStrafeEnhanced = false;
         }
      }

      if (isStrafeEnhanced == true) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
         strafeEnhancement();
      } else {
         follower.setTeleOpMovementVectors(input_x, input_y, input_turn, true, false);
         follower.update();
      }
   }

   private void strafeEnhancement() {
      telemetry.addLine("Using strafe enhancement");
      double input_x;
      double input_y;

      // TODO: Update all driver controls and make the strafing enhancement accessible by a button
      input_x = Math.pow(-g1.left_stick_y, 3);
      input_y = Math.pow(-g1.left_stick_x, 3);
      Log.d("input_x", Double.toString(input_x));
      Log.d("input_y", Double.toString(input_y));

      // Made the robot strafe in a straight line for now
      Pose2d currentPose = follower.getPose();

      Log.d("desiredHeading:", Double.toString(Math.toDegrees(desiredHeading)));
      double currentHeading = currentPose.heading.toDouble();
      Log.d("desiredHeadingCurrent:", Double.toString(Math.toDegrees(currentHeading)));
      double headingError = MathFunctions.getSmallestAngleDifference(desiredHeading, currentHeading)
              * MathFunctions.getTurnDirection(currentHeading, desiredHeading);
      Log.d("desiredHeadingError:", Double.toString(Math.toDegrees(headingError)));

      // The translational correction can work if the robot is parallel to the x-axis.
      // Otherwise it won't work as well so the translational correction is not used.

      Log.d("desiredxPos", Double.toString(desiredxPos));
      double currentxPos = currentPose.position.x;
      double xPosError = desiredxPos - currentxPos;

      Vector xPosCorrection = new Vector();
      xPosCorrection.setOrthogonalComponents(xPosError, 0);

      Log.d("xPosError", Double.toString(xPosError));

//      // driveVector components are the gamepad x and y values, assuming that they are in the same direction
//      // as the x-axis and y-axis.
//      // set the vector components to correct the robot's x-position.
//      teleOpTranslationalPIDF.updateError(xPosCorrection.getMagnitude());
//      xPosCorrection.setMagnitude(teleOpTranslationalPIDF.runPIDF() + smallTranslationalPIDFFeedForward);
//      driveVector.setOrthogonalComponents((input_x + xPosCorrection.getMagnitude() *
//                      (xPosError < 0 ? -1 : 1)),
//              input_y);
//      Log.d("xPosMagnitude:", Double.toString(xPosCorrection.getMagnitude()));
//      driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
      // driveVector components are the gamepad x and y values, assuming that they are in the same direction
      // as the x-axis and y-axis.
      Log.d("Drive Vector XPos:", Double.toString(driveVector.getXComponent()));
      Log.d("Drive Vector YPos:", Double.toString(driveVector.getYComponent()));

      // If robot heading is not the desired heading, the heading vector will correct it
      headingPIDF.updateError(headingError);
      headingVector.setComponents(MathFunctions.clamp(
              headingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(currentHeading,
                      desiredHeading), -1, 1), currentHeading);
      Log.d("desiredHeading Vector Angle:", Double.toString(headingVector.getTheta()));
      Log.d("desiredHeadingTurnDirection:", Double.toString(MathFunctions.getTurnDirection(currentHeading, desiredHeading)));
      Log.d("Heading Vector XPos:", Double.toString(headingVector.getXComponent()));
      Log.d("Heading Vector YPos:", Double.toString(headingVector.getYComponent()));

      follower.setTeleOpMovementVectors(input_x, input_y, headingError, true, true);
      follower.update();

      Log.d("X Position:", Double.toString(follower.getPose().position.x));
      Log.d("Y Position:", Double.toString(follower.getPose().position.y));
      Log.d("Heading:", Double.toString(follower.getPose().heading.toDouble()));
   }

   private void intakeControls() {
      // Intake controls
      if (intake.isIntakeOverCurrent()) {
         sched.queueActionParallel(intake.wristStored());
         long start = System.currentTimeMillis();
         sched.queueActionParallel(new ActionUtil.RunnableAction(() -> {
            Log.d("CURRENT", intake.isIntakeOverCurrent() ? "true" : "false");
            if (!intake.isIntakeOverCurrent() || intake.pixelCount() == 2) {
               sched.queueAction(intake.wristDown());
               sched.queueAction(intake.intakeOn());
               return false;
            }
            if (System.currentTimeMillis() - start > 1000 && !intake.isReversing()) {
               sched.queueAction(intake.intakeReverse());
            }
            return true;
         }));
      }

      int pixelCount = intake.pixelCount();
      if (intake.isIntaking() && pixelCount == 2) { // Check if already two pixels - Stop intake
         sched.queueAction(intake.intakeReverse());
         sched.queueAction(intake.wristStored());
         sched.queueActionParallel(new SequentialAction(
                 new SleepAction(1),
                 intake.intakeOff(),
                 intake.wristStored()
         ));
      }
      if (g1.aOnce()) {
         sched.cancelParallel();
         if (intake.isIntaking()) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(intake.wristStored());
         } else {
            sched.queueAction(intake.intakeOn());
            sched.queueAction(intake.wristDown());
         }
      }
      if (g1.bOnce()) {
         sched.cancelParallel();
         if (intake.isReversing()) {
            sched.queueAction(intake.intakeOff());
            sched.queueAction(intake.wristStored());
         } else {
            sched.queueAction(intake.intakeReverse());
         }
      }
      if (g1.xOnce()) {
         sched.queueAction(new SequentialAction(
                 intake.feedOpen(),
                 new SleepAction(0.5),
                 intake.feedClosed()
         ));
      }
      if (g1.yOnce()) {
         sched.queueAction(intake.intakeOn());
         sched.queueAction(intake.wristStack(1));
      }

      // Other susbsystems
      if (g1.backOnce()) {
//         sched.queueAction(plane.scorePlane());
      }
      if (g2.startOnce()) {
         if (!outtake.isSlideHanging()) {
            // Handle scoring
            if (outtake.isArmScoring()) {
               sched.queueAction(new SequentialAction(
                       outtake.clawOpen(),
                       outtake.wristVertical(),
                       new SleepAction(0.4),
                       outtake.armStored()
               ));
            }

            // Make mosaic fixer go out to not interfere with hanging
            sched.queueAction(outtake.mosaicFix());
            sched.queueAction(outtake.extendOuttakeHangBlocking());
         } else {
            sched.queueAction(outtake.retractOuttakeHang());
         }
      }
   }

   private void outtakeControls() {
      // Outtake controls
      if (g2.yOnce()) {
         boolean one = false;
         if (!outtake.isArmScoring()) {
            if (intake.pixelCount() == 1) {
               one = true;
               sched.queueAction(new SequentialAction(outtake.extendOuttakeBarelyOut(), new SleepAction(0.3)));
            }
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(
                    intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                    new SleepAction(0.3)
           ));
            sched.queueAction(new ParallelAction(
                    new SequentialAction(new SleepAction(0.4), outtake.armScoring()),
                    outtake.extendOuttakeTeleopBlocking()
            ));
         }
         sched.queueAction(one ? outtake.wristVerticalFlip() : outtake.wristVertical());
      }
      if (g2.xOnce()) {
         if (!outtake.isArmScoring()) {
            if (intake.pixelCount() == 1) {
               sched.queueAction(new SequentialAction(outtake.extendOuttakeBarelyOut(), new SleepAction(0.3)));
            }
            sched.queueAction(intake.intakeOff());
            sched.queueAction(new SequentialAction(
                    intake.pixelCount() == 1 ? outtake.clawSingleClosed() : outtake.clawClosed(),
                    new SleepAction(0.3)
            ));
            sched.queueAction(new ParallelAction(
                 new SequentialAction(
                            new SleepAction(0.2),
                            outtake.armScoring(),
                            new SleepAction(0.2),
                            outtake.wristMosaic(true)
                    ),
                    outtake.extendOuttakeTeleopBlocking()
            ));
         } else {
            sched.queueAction(outtake.wristMosaic(true));
         }
      }

      if (g2.leftBumperOnce()) {
         if (outtake.isWristVertical()) {
            sched.queueAction(outtake.wristSideways(true));
         } else if (outtake.isWristSideways()) {
            sched.queueAction(outtake.wristVerticalFlip());
         }

         if (outtake.isWristMosaic(true)) {
            sched.queueAction(outtake.wristMosaicFlip(true));
         } else if (outtake.isWristMosaic(false)) {
            sched.queueAction(outtake.wristMosaic(true));
         }
      }

      if (g2.leftStickButton()) {
         outtake.resetSlideOffset();
      }

      if (g2.rightBumperOnce()) {
         if (outtake.isWristVertical()) {
            sched.queueAction(outtake.wristSideways(false));
         }
         if (outtake.isWristSideways()) {
            sched.queueAction(outtake.wristVerticalFlip());
         }

         if (outtake.isWristMosaic(false)) {
            sched.queueAction(outtake.wristMosaicFlip(false));
         } else if (outtake.isWristMosaic(true)) {
            sched.queueAction(outtake.wristMosaic(false));
         }
      }

      if (Math.abs(g2.right_stick_y) > 0.01) {
         outtake.slidePIDEnabled = false;
         outtake.setSlidePower(-g2.right_stick_y/1.6);
         Log.d("set slide power", "g2/1.6");
      } else if (!outtake.slidePIDEnabled) {
         outtake.slidePIDEnabled = true;
         sched.queueAction(outtake.lockPosition());
      }
      if (g2.aOnce()) {
         sched.queueAction(new SequentialAction(
                 outtake.mosaicStored(),
                 outtake.clawHalfOpen(),
                 new SleepAction(outtake.isArmScoring() ? 0.5 : 0),
                 outtake.wristVertical(),
                 outtake.armStored(),
                 outtake.clawOpen(),
                 outtake.retractOuttakeBlocking()
         ));
      }
      if (g2.bOnce()) {
         sched.queueAction(new SequentialAction(
                 outtake.wristVertical(),
                 outtake.armStored(),
                 new SleepAction(0.3),
                 outtake.retractOuttakeBlocking(),
                 new SleepAction(0.1),
                 outtake.clawOpen()
         ));
      }
      if (g2.dpadUpOnce()) {
         sched.queueAction(outtake.increaseSlideLayer(1));
      }
      if (g2.dpadDownOnce()) {
         sched.queueAction(outtake.increaseSlideLayer(-1));
      }

      // Press back to toggle between mosaic fix and mosaic stored.
      if (g2.backOnce()) {
         if (outtake.isMosaicFixing()) {
            sched.queueAction(outtake.mosaicStored());
         } else {
            sched.queueAction(outtake.mosaicFix());
            sched.queueAction(outtake.wristVertical());
            sched.queueAction(outtake.armStored());
            sched.queueAction(outtake.clawOpen());
         }
      }
   }

   private double timeLeft() {
      if (!isStarted()) return 120;
      return 120 - smartGameTimer.seconds();
   }
   private boolean isBetween(double number, double min, double max) {
      return min <= number && number < max;
   }
   boolean warning1 = false;
   boolean warning2 = false;
   boolean warning3 = false;
   private void ledUpdate() {
      if (isStrafeEnhanced == true) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
         telemetry.addLine("Strafe Enhancing: True");
      }
      int pixelCount = intake.pixelCount();
      if (pixelCount == 1) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
      } else if (pixelCount == 2) {
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
      } else if (isBetween(timeLeft(), 31, 35)) { // 35-31; prepare for endgame
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
         if (!warning1) {
            g1.rumbleBlips(3);
            warning1 = true;
         }
      } else if (isBetween(timeLeft(), 25, 30)) { // 30-25; endgame starts
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
         if (!warning2) {
            g1.rumbleBlips(3);
            warning2 = true;
         }
      } else if (isBetween(timeLeft(), 0, 8)) { // last 8 sec, go hang
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
         if (!warning3) {
            g1.rumbleBlips(3);
            warning3 = true;
         }
      } else { // default lights, put here for lower priority
         led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
      }
   }
}
