package org.firstinspires.ftc.teamcode._2027.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._2027.Config;
import org.firstinspires.ftc.teamcode._2027.util.DriveType;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(1)
            .lateralZeroPowerAcceleration(1)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.0002, 0.006, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0.1, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0003, 0.6, 0.13))
            .centripetalScaling(0.00003).
            mass(13.732);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName (Config.hardware.motor.lfm.id)
            .leftRearMotorName  (Config.hardware.motor.lbm.id)
            .rightFrontMotorName(Config.hardware.motor.rfm.id)
            .rightRearMotorName (Config.hardware.motor.rbm.id)
            .leftFrontMotorDirection (Config.hardware.motor.lfm.direction)
            .leftRearMotorDirection  (Config.hardware.motor.lbm.direction)
            .rightFrontMotorDirection(Config.hardware.motor.rfm.direction)
            .rightRearMotorDirection (Config.hardware.motor.rbm.direction)
            .xVelocity(73.26)   //forward max power
            .yVelocity(52.26)   // <-  max power
            .useBrakeModeInTeleOp(false);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(Config.odometry.forwardOffset / 25.4)
            .strafePodX(Config.odometry.strafeOffset / 25.4) //-150.2
            .distanceUnit(DistanceUnit.INCH).hardwareMapName(Config.odometry.id)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(Config.odometry.forwardDirection)
            .strafeEncoderDirection(Config.odometry.strafeDirection);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .velocity(64.1879440668061)
            .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.X_LOCK)
            .useBrakeModeInTeleOp(true);

    private static double kP = 0.3;
    private static double kD = 0.01;
    private static double kFFront = 0.0130;
    private static double kFBack = 0.0190;

    private static double dtLength = 210 / 25.4 /2;
    private static double dtWidth  = 296 / 25.4 /2;

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.hardware.motor.lfm.id, Config.hardware.servo.lfs.id, Config.hardware.sensor.lfenc.id,
                new PIDFCoefficients(kP, 0, kD, kFFront), Config.hardware.motor.lfm.direction,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(43 % 360), new Pose(dtLength, dtWidth),
                0.004, 3.225, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.hardware.motor.rfm.id, Config.hardware.servo.rfs.id, Config.hardware.sensor.rfenc.id,
                new PIDFCoefficients(kP, 0, kD, kFFront), Config.hardware.motor.rfm.direction,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(0 % 360), new Pose(dtLength, -dtWidth),
                0.001, 3.213, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.hardware.motor.lbm.id, Config.hardware.servo.lbs.id, Config.hardware.sensor.lbenc.id,
                new PIDFCoefficients(kP, 0, kD, kFBack), Config.hardware.motor.lbm.direction,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(32.03 % 360), new Pose(-dtLength, dtWidth),
                0.004, 3.218, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.hardware.motor.rbm.id, Config.hardware.servo.rbs.id, Config.hardware.sensor.rbenc.id,
                new PIDFCoefficients(kP, 0, kD, kFBack), Config.hardware.motor.rbm.direction,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(300 % 360), new Pose(-dtLength, -dtWidth),
                0.002, 3.214, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }
    public static PathConstraints pathConstraints = new PathConstraints(0.9, 2, 2,
            0.1, 30, 1, 10, 1);

    private static DriveType type;
    public static void setDrive(DriveType type){
        Constants.type = type;
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        if(type == null){
            type = DriveType.Mecanum;
        }
        switch (type){
            case Mecanum:
                return new FollowerBuilder(followerConstants, hardwareMap)
                        .mecanumDrivetrain(driveConstants)
                        .pinpointLocalizer(localizerConstants)
                        .pathConstraints(pathConstraints)
                        .build();
            case Swerve:
                return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                        .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap),
                                leftBack(hardwareMap), rightBack(hardwareMap))
                        .pinpointLocalizer(localizerConstants)
                        .build();
        }
        return null;
    }
}