package org.firstinspires.ftc.teamcode._2627.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._2627.config.Config;

@Configurable
public class SwerveConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(1)
            .lateralZeroPowerAcceleration(1)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.0002, 0.006, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0.1, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0003, 0.6, 0.13))
            .centripetalScaling(0.00003).
            mass(13.732);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(Config.Sensor.pinpoint.offsetY / 2.54)
            .strafePodX(Config.Sensor.pinpoint.offsetX / 2.54) //-150.2
            .distanceUnit(DistanceUnit.INCH).hardwareMapName(Config.Sensor.pinpoint.id)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(Config.Sensor.pinpoint.Ydirection)
            .strafeEncoderDirection(Config.Sensor.pinpoint.Xdirection);

    public static com.pedropathing.ftc.drivetrains.SwerveConstants swerveConstants = new com.pedropathing.ftc.drivetrains.SwerveConstants()
            .velocity(64.1879440668061)
            .zeroPowerBehavior(com.pedropathing.ftc.drivetrains.SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES)
            .useBrakeModeInTeleOp(true);

    private static double kP = 0.3;
    private static double kD = 0.0001;
    private static double kFFront = 0.0130;
    private static double kFBack = 0.0190;

    private static double dtLength = 25.0 / 2.54 /2;
    private static double dtWidth  = 24.8 / 2.54 /2;

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.Motor.lfm.id, Config.Servo.lfs.id, Config.Sensor.lfenc.id,
                new PIDFCoefficients(kP, 0, kD, kFFront), DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(305.9010), new Pose(dtLength, dtWidth),
                0.001, 3.275, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.Motor.rfm.id, Config.Servo.rfs.id, Config.Sensor.rfenc.id,
                new PIDFCoefficients(kP, 0, kD, kFFront), DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(313.4558), new Pose(dtLength, -dtWidth),
                0, 3.264, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.Motor.lbm.id, Config.Servo.lbs.id, Config.Sensor.lbenc.id,
                new PIDFCoefficients(kP, 0, kD, kFBack), DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(143.2563), new Pose(-dtLength, dtWidth),
                0.002, 3.294, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(hardwareMap, Config.Motor.rbm.id, Config.Servo.rbs.id, Config.Sensor.rbenc.id,
                new PIDFCoefficients(kP, 0, kD, kFBack), DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD, Math.toRadians(129.4246), new Pose(-dtLength, -dtWidth),
                0, 3.285, false);
        pod.setMotorCachingThreshold(0.05);
        pod.setServoCachingThreshold(0.05);
        return pod;
    }
    public static PathConstraints pathConstraints = new PathConstraints(0.9, 2, 2,
            0.1, 30, 1, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap).pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants, leftFront(hardwareMap), rightFront(hardwareMap),
                        leftBack(hardwareMap), rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants).build();
    }
}