//package org.firstinspires.ftc.teamcode.pedroPathing.localization;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.DualNum;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Rotation2d;
//import com.acmerobotics.roadrunner.Time;
//import com.acmerobotics.roadrunner.Twist2dDual;
//import com.acmerobotics.roadrunner.Vector2dDual;
//import com.acmerobotics.roadrunner.ftc.Encoder;
//import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
//import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
//import com.acmerobotics.roadrunner.ftc.RawEncoder;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;
//
//@Config
//public final class ThreeDeadWheelLocalizerBackup61 implements Localizer {
//    public static class Params {
//        public double par0YTicks = -1600.4302597829123;//-2441.9133801212834;//-2426.512456471011; y position of the first parallel encoder (in tick units)
//        public double par1YTicks = 1589.5159695593209; //2467.6258;//2467.9133801212834;//2530.6258; // y position of the second parallel encoder (in tick units)
//        public double perpXTicks = -427.34187393510723; //-466.222994370873; //-566.222994370873; //-650.620899175634;//-560.3783221003024; // x position of the perpendicular encoder (in tick units)
//    }
//
//    public static Params PARAMS = new Params();
//
//    public final Encoder par0, par1, perp;
//
//    public final double inPerTick;
//
//    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
//
//    private double lastRawHeadingVel=0.0, headingVelOffset=0.0;
//
//    public double imuYawHeading = 0.0;
//
//    public double imuHeadingVelo = 0.0;
//    private ExecutorService imuExecutor = Executors.newSingleThreadExecutor();
//    private Rotation2d lastHeading;
//    private long lastHeadingTime = System.currentTimeMillis();
//
////    private final Object imuLock = new Object();
////    @GuardedBy("imuLock")
//    public IMU imu;
//
//    private boolean initialized;
//
////    private ElapsedTime imuTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
////    public static double IMU_INTERVAL = 100.0;
//
//    public Pose2d pose = new Pose2d(0.0,0.0,0.0);
//
//    public ThreeDeadWheelLocalizerBackup61(HardwareMap hardwareMap, IMU imu, double inPerTick) {
//        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
////        par0.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
//        par1.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
//
//        this.imu = imu;
//
//        lastPar0Pos = par0.getPositionAndVelocity().position;
//        lastPar1Pos = par1.getPositionAndVelocity().position;
//        lastPerpPos = perp.getPositionAndVelocity().position;
//
//        this.inPerTick = inPerTick;
//    }
//
//    public void startIMUThread(LinearOpMode opMode) {
////        imuExecutor.submit(new Runnable() {
////            @Override
////            public void run() {
////                while (!opMode.isStopRequested() && imuTimer.milliseconds() > IMU_INTERVAL) {
////                    synchronized (imuLock) {
////                        imuYawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
////                       // imuHeadingVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
////                    }
////
////                    imuTimer.reset();
////                }
////            }
////        });
//    }
//
//    private int headingCounter = 0;
//
//    public void update() {
//
//        ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
//        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
//        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
//
//        if (!initialized) {
//            initialized = true;
//
//            lastPar0Pos = par0PosVel.position;
//            lastPar1Pos = par1PosVel.position;
//            lastPerpPos = perpPosVel.position;
//
//            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
//            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
//
//            lastHeading = heading;
//
////            return new Twist2dDual<>(
////                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
////                    DualNum.constant(0.0, 2)
////            );
//        }
//
//        int par0PosDelta = par0PosVel.position - lastPar0Pos;
//        int par1PosDelta = par1PosVel.position - lastPar1Pos;
//        int perpPosDelta = perpPosVel.position - lastPerpPos;
//
//        Log.d("Localizer", " p0-delta " + Double.toString(par0PosDelta));
//        Log.d("Localizer", " p1-delta " + Double.toString(par1PosDelta));
//        Log.d("localizer", " perp delta " + Double.toString(perpPosDelta));
//
//        double headingDelta0 = (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
//        double headingDelta = headingDelta0;
//
//        Log.d("localizer", " heading delta " + Double.toString(headingDelta));
//
//        if(headingCounter++ > 5) {
//            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
//            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
////            Log.d("ThreeDeadWheelLocalizer_logger", String.format("Calc:%3.3f, IMU:%3.3f",Math.toDegrees(headingDelta),Math.toDegrees(heading.minus(lastHeading))));
//            headingDelta = heading.minus(lastHeading);
//            lastHeading = heading;
//            Log.d("localizer", " imu heading delta " + Double.toString(headingDelta));
//            headingCounter = 0;
//
//        } else {
//            lastHeading = lastHeading.plus(headingDelta);
//        }
//
////        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
////        double rawHeadingVel = angularVelocity.zRotationRate;
//
//        double calcAngularVelo = (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
//        double rawHeadingVel = calcAngularVelo;
//
//        Log.d("localizer", " heading velocity " + Double.toString(calcAngularVelo));
//
//        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
//        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
//            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
//        }
//        lastRawHeadingVel = rawHeadingVel;
//        double headingVel = headingVelOffset + rawHeadingVel;
//        Log.d("localizer", " heading velocity 2 " + Double.toString(headingVel));
//
//        Twist2dDual<Time> twist = new Twist2dDual<>(
//                new Vector2dDual<>(
//                        new DualNum<Time>(new double[] {
//                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
//                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
//                        }).times(inPerTick),
//                        new DualNum<Time>(new double[] {
//                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
//                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
//                        }).times(inPerTick)
//                ),
//                new DualNum<>(new double[] {
//                        headingDelta,
//                        headingVel,
//                })
//        );
//
//        lastPar0Pos = par0PosVel.position;
//        lastPar1Pos = par1PosVel.position;
//        lastPerpPos = perpPosVel.position;
//
//        pose = pose.plus(twist.value());
//        Log.d("localizer", " twist x " + Double.toString(twist.line.x.value()));
//        Log.d("localizer", " twist y " + Double.toString(twist.line.y.value()));
//        Log.d("localizer", " twist heading " + Double.toString(twist.angle.value()));
///*
//        Log.d("Localizer_logger", String.format("par0_delta: %d | par1_delta: %d | (imu: %3.3f) | velo: %3.3f (imu: %3.3f)" ,
//                par0PosDelta, par1PosDelta, Math.toDegrees(headingDelta), calcAngularVelo, headingVel));
//
//        Log.d("ThreeDeadWheelsOdometry_logger", "Processing time (ms): " + String.format("%3.2f", loopTimer.milliseconds()));
//
//        Log.d("Localizer_logger", "p0: " + lastPar0Pos +
//                " | p1: " + lastPar1Pos + " | perp: " + lastPerpPos  + " | delta_h: " + String.format("%3.3f", headingDelta)
//                + " | (delta_x: " + String.format("%3.3f", twist.value().line.x) + ", delta_y: " + String.format("%3.3f", twist.value().line.y) + ")"
//                + " | (pose_x: " + String.format("%3.3f", pose.position.x) + ", pose_y: " + String.format("%3.3f", pose.position.y) + ")"
//        );
//
//        Log.d("Localizer_logger", " p0-delta " + Double.toString(par0PosDelta) + " p1-delta " + Double.toString(par1PosDelta));
//*/
//        loopTimer.reset();
//
////        return twist;
//    }
//
//    public void setPoseEstimate(Pose2d poseEstimate) {
//        this.pose = poseEstimate;
//    }
//
//    public Pose2d getPoseEstimate() {
//        return this.pose;
//    }
//
//    public void resetHeading(double newHeading) {
//        this.pose = new Pose2d(this.pose.position.x, this.pose.position.y, newHeading);
//    }
//
//}
