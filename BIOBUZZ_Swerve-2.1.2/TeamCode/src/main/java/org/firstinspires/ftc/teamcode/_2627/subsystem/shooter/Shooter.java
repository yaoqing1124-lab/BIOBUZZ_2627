package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._2627.util.HardwareConfig;
import org.firstinspires.ftc.teamcode._2627.config.MotorConfig;
import org.firstinspires.ftc.teamcode._2627.config.ServoConfig;
import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2627.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2627.util.ShooterConfig;
import org.firstinspires.ftc.teamcode.gc.op.OpBehavior;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

public class Shooter extends OpBehavior {
    private final MotorConfig m_flywheelInfo;
    private final ServoConfig m_yawInfo;
    private DcMotorEx flywheel;
    private Servo yaw1, yaw2;

    private static final ShooterPose s_shooterPose = new ShooterPose();
    private static ShooterPose s_targetPose = new ShooterPose();

    private static boolean s_isEnable = false;
    private static boolean s_isEnablePredict = false;

    private Shooter(MotorConfig motorInfo, ServoConfig servoInfo) {
        m_flywheelInfo = motorInfo;
        m_yawInfo = servoInfo;
    }

    private final static Shooter s_instance;
    static {
        s_instance = new Shooter(HardwareConfig.Motor.flyWheel, HardwareConfig.Servo.yaw1);
        OpObject.RegisterGlobalBehavior(s_instance);
    }

    /** 控制是否進行射擊動作 (推球/供球) */
    public static void setEnable(boolean enable) {
        s_isEnable = enable;
    }

    public static boolean isEnable() {
        return s_isEnable;
    }

    public static void setEnablePredict(boolean enablePredict) {
        s_isEnablePredict = enablePredict;
    }

    public static boolean isEnablePredict() {
        return s_isEnablePredict;
    }

    public static void setTarget(ShooterPose targetPose) {
        if (targetPose != null) {
            s_targetPose = targetPose.copy();
        }
    }

    public static ShooterPose getTarget() {
        return s_targetPose;
    }

    public static ShooterPose getShooterPose() {
        return s_shooterPose;
    }

    @Override
    public void Init() {
        flywheel = m_object.hardwareMap.get(DcMotorEx.class, m_flywheelInfo.id);
        flywheel.setDirection(m_flywheelInfo.direction);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPower(0);

        yaw1 = m_object.hardwareMap.get(Servo.class, m_yawInfo.id);
        if (m_yawInfo.direction != null) {
            yaw1.setDirection(m_yawInfo.direction);
        }
        yaw1.setPosition(0.5);

        try {
            yaw2 = m_object.hardwareMap.get(Servo.class, HardwareConfig.Servo.yaw2.id);
            if (HardwareConfig.Servo.yaw2.direction != null) {
                yaw2.setDirection(HardwareConfig.Servo.yaw2.direction);
            }
            yaw2.setPosition(0.5);
        } catch (Exception ignored) {
            yaw2 = null;
        }

        s_isEnable = false;
    }

    @Override
    public void Start() {
        updateShooterPose();
    }

    @Override
    public void Loop() {
        updateShooterPose();

        // 1. 取得瞄準目標（若開啟預測則使用移動補償目標）
        ShooterPose effectiveTarget = s_targetPose;
        Follower follower = Drivetrain.get_Follower();

        if (s_isEnablePredict && follower != null && follower.getVelocity() != null) {
            double vx = follower.getVelocity().getXComponent();
            double vy = follower.getVelocity().getYComponent();
            effectiveTarget = ShooterCalculator.calculateVirtualTarget(s_shooterPose, s_targetPose, vx, vy);
        }

        // 2. 飛輪保持常轉，隨時維持射擊轉速 (不再因為 !s_isEnable 而停轉)
        updateFlyWheel(effectiveTarget);

        // 3. 雲台隨時鎖定目標角度
        updateYaw(effectiveTarget);

        // 4. 用 isEnable 單純控制「要不要發射球」 (推球進飛輪)
        if (s_isEnable) {
            Intake.setMode(Intake.Mode.shoot);
        } else if (Intake.getMode() == Intake.Mode.shoot) {
            Intake.setMode(Intake.Mode.brake);
        }
    }

    @Override
    public void Stop() {
        // 只有在 OpMode 結束退出時才關閉馬達電源
        if (flywheel != null) {
            flywheel.setPower(0);
            flywheel = null;
        }
        yaw1 = null;
        yaw2 = null;
        s_isEnable = false;
    }

    /**
     * 每幀更新發射器在場地上的實際 Pose
     */
    private void updateShooterPose() {
        Follower follower = Drivetrain.get_Follower();
        if (follower == null || follower.getPose() == null) {
            return;
        }

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getHeading();

        double relX = ShooterConfig.relativeFlyWheel.x;
        double relY = ShooterConfig.relativeFlyWheel.y;

        double worldX = robotX + relX * Math.cos(robotHeading) - relY * Math.sin(robotHeading);
        double worldY = robotY + relX * Math.sin(robotHeading) + relY * Math.cos(robotHeading);

        s_shooterPose.set(worldX, worldY, ShooterConfig.SHOOTER_HEIGHT, robotHeading);
    }

    /**
     * 更新飛輪轉速：有計算出新速度就更新，否則維持現有轉速不做更改，不設為 0
     */
    private void updateFlyWheel(ShooterPose target) {
        if (flywheel == null || target == null) return;

        double requiredVelocity = ShooterCalculator.calculateVelocity(s_shooterPose, target);
        if (requiredVelocity <= 0) {
            // 無法計算（例如超出物理拋物線角度）時：直接返回，維持現有轉速不做更改！
            return;
        }

        // 速度 -> RPM -> ticks/sec
        double targetRPM = ShooterCalculator.Velocity2RPM(requiredVelocity)
                + ShooterConfig.VELOCITY_OFFSET;
        double targetTicksPerSec = ShooterCalculator.RPM2ticksPerSec(targetRPM);

        flywheel.setVelocity(targetTicksPerSec);
    }

    /**
     * 更新雲台舵機位置
     */
    private void updateYaw(ShooterPose target) {
        if (yaw1 == null || target == null) return;

        Follower follower = Drivetrain.get_Follower();
        double heading = (follower != null) ? follower.getHeading() : 0.0;

        double relativeYawDeg = ShooterCalculator.calculateYaw(s_shooterPose, target, heading);
        double servoPos = ShooterCalculator.degree2ServoPosition(
                relativeYawDeg,
                m_yawInfo.offset,
                ShooterConfig.YAW_SERVO_TOTAL_RANGE_DEG
        );

        yaw1.setPosition(servoPos);
        if (yaw2 != null) {
            yaw2.setPosition(servoPos);
        }
    }
}