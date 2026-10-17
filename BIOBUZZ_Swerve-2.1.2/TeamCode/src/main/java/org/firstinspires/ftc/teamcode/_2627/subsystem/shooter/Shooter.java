package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2627.constants.HardwareConfig;
import org.firstinspires.ftc.teamcode._2627.constants.ShooterConfig;
import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2627.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2627.util.MotorConfig;
import org.firstinspires.ftc.teamcode._2627.util.ServoConfig;
import org.firstinspires.ftc.teamcode.gc.op.OpBehavior;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

/**
 * 發射器子系統 (Shooter Subsystem)
 * 參考 2026 發射器子系統架構規格書 (Shooter.md)
 *
 * 系統被劃分為三大層級：
 * 1. 擬合物理層 (Physical Fitting Layer)：雙參數架構 (ENERGY_CONVERSION_RATE, ENERGY_LOSS_RATE)
 * 2. 數學模型層 (Mathematical Model Layer)：拋體運動、虛擬目標轉換、動態移動射擊前饋補償
 * 3. 反饋控制層 (Feedback Control Layer)：飛輪即時閉環轉速維持、雲台 Yaw/Pitch 伺服機追蹤與平滑濾波
 */
public class Shooter extends OpBehavior {

    // ==========================================
    // 硬體物件
    // ==========================================
    private final MotorConfig m_flywheelInfo;
    private final ServoConfig m_yawInfo;
    private DcMotorEx flywheel;
    private Servo yaw1, yaw2;
    private Servo pitch; // 可選仰角舵機 (若硬體無仰角舵機則為 null)

    // ==========================================
    // 全域狀態與旗標
    // ==========================================
    public static boolean isEnable = false;         // 控制是否進行發射 (進球/供球動作)
    public static boolean enableAiming = true;     // 控制是否開啟雲台自動瞄準
    public static boolean enablePredict = true;    // 控制是否開啟底盤動態速度補償

    public static double targetRPM = 0.0;
    public static double actualRPM = 0.0;          // 經滑動平均濾波後的真實轉速

    // 姿態資料結構
    public final static ShooterPose current_position        = new ShooterPose();
    public final static ShooterPose target_position         = new ShooterPose();
    public final static ShooterPose predict_target_position = new ShooterPose();

    // 遙測除錯變數
    private static double s_lastRelativeYawDeg = 0.0;
    private static double s_lastServoPos = 0.5;
    private static double s_lastTargetFieldAngleDeg = 0.0;
    private static double s_lastTargetVelocity = 0.0;

    // 滑動平均轉速濾波緩衝區 (Rolling average buffer)
    private static final int RPM_BUFFER_SIZE = 7;
    private static final double[] m_rpmBuffer = new double[RPM_BUFFER_SIZE];
    private static int m_rpmBufferIdx = 0;
    private static boolean m_rpmBufferFull = false;

    private static Follower m_follower = null;

    // ==========================================
    // 單例註冊
    // ==========================================
    private Shooter(MotorConfig motorInfo, ServoConfig servoInfo) {
        m_flywheelInfo = motorInfo;
        m_yawInfo = servoInfo;
    }

    private final static Shooter s_instance;
    static {
        s_instance = new Shooter(HardwareConfig.Motor.flyWheel, HardwareConfig.Servo.yaw1);
        OpObject.RegisterGlobalBehavior(s_instance);
    }

    public static Shooter getInstance() {
        return s_instance;
    }

    // ==========================================
    // 控制介面 API
    // ==========================================
    public static void setEnable(boolean enable) {
        isEnable = enable;
    }

    public static boolean isEnable() {
        return isEnable;
    }

    public static void setEnableAiming(boolean aiming) {
        enableAiming = aiming;
    }

    public static boolean isEnableAiming() {
        return enableAiming;
    }

    public static void setEnablePredict(boolean predict) {
        enablePredict = predict;
    }

    public static boolean isEnablePredict() {
        return enablePredict;
    }

    public static void setTarget(ShooterPose targetPose) {
        if (targetPose != null) {
            target_position.set(targetPose.x, targetPose.y, targetPose.z, targetPose.yaw);
        }
    }

    public static ShooterPose getTarget() {
        return target_position;
    }

    public static ShooterPose getShooterPose() {
        return current_position;
    }

    public static double getLastRelativeYawDeg() {
        return s_lastRelativeYawDeg;
    }

    public static double getLastServoPos() {
        return s_lastServoPos;
    }

    public static double getLastTargetFieldAngleDeg() {
        return s_lastTargetFieldAngleDeg;
    }

    public static double getTargetRPM() {
        return targetRPM;
    }

    public static double getActualRPM() {
        return actualRPM;
    }

    /**
     * 判定轉速是否穩定達標 (IsStable)
     */
    public static boolean IsStable() {
        if (targetRPM <= 0) return false;
        return Math.abs(targetRPM - actualRPM) < ShooterConfig.RPM_ERROR_THRESHOLD;
    }

    public static boolean isStable() {
        return IsStable();
    }

    public static void resetRpmBuffer() {
        m_rpmBufferIdx = 0;
        m_rpmBufferFull = false;
        java.util.Arrays.fill(m_rpmBuffer, 0.0);
    }

    // ==========================================
    // 生命週期 (Lifecycle)
    // ==========================================
    @Override
    public void Init() {
        // 1. 初始化飛輪馬達
        flywheel = m_object.hardwareMap.get(DcMotorEx.class, m_flywheelInfo.id);
        flywheel.setDirection(m_flywheelInfo.direction);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPower(0);

        // 2. 初始化雲台偏航舵機 (Yaw)
        yaw1 = m_object.hardwareMap.get(Servo.class, m_yawInfo.id);
        if (m_yawInfo.direction != null) {
            yaw1.setDirection(m_yawInfo.direction);
        }
        yaw1.setPosition(0.5);

        // 3. 初始化第二雲台偏航舵機 (若有)
        try {
            yaw2 = m_object.hardwareMap.get(Servo.class, HardwareConfig.Servo.yaw2.id);
            if (HardwareConfig.Servo.yaw2.direction != null) {
                yaw2.setDirection(HardwareConfig.Servo.yaw2.direction);
            }
            yaw2.setPosition(0.5);
        } catch (Exception ignored) {
            yaw2 = null;
        }

        // 4. 初始化仰角舵機 (Pitch，若有)
        try {
            pitch = m_object.hardwareMap.get(Servo.class, HardwareConfig.Servo.pitch.id);
            if (HardwareConfig.Servo.pitch.direction != null) {
                pitch.setDirection(HardwareConfig.Servo.pitch.direction);
            }
        } catch (Exception ignored) {
            pitch = null;
        }

        isEnable = false;
        resetRpmBuffer();
    }

    @Override
    public void Start() {
        m_follower = Drivetrain.get_Follower();
        updateShooterPose();
        resetRpmBuffer();
    }

    @Override
    public void Loop() {
        // --- 階段 1：更新當前發射器場地姿態 (含旋轉中心離心偏置旋轉) ---
        updateShooterPose();

        // --- 階段 2：移動射擊動態預測 (Motion Prediction) ---
        updateMotionPrediction();

        // --- 階段 3：滑動平均讀取飛輪真實轉速 (RPM Buffer Smoothing) ---
        updateActualRPM();

        // --- 階段 4：計算虛擬目標點 (空氣阻力與 Magnus 下墜補償) ---
        ShooterPose virtualTarget = ShooterCalculator.calculateVirtualTarget(
                current_position, predict_target_position);

        // --- 階段 5：計算理想初速與目標 RPM (前饋運算，維持常轉不歸零) ---
        updateVelocityAndTargetRPM(virtualTarget);

        // --- 階段 6：執行飛輪轉速控制 ---
        if (flywheel != null && targetRPM > 0) {
            double targetTicksPerSec = ShooterCalculator.RPMToTicksPerSec(targetRPM);
            flywheel.setVelocity(targetTicksPerSec);
        }

        // --- 階段 7：執行仰角 (Pitch) 反饋調整 (若有仰角舵機) ---
        if (pitch != null) {
            double targetPitchDeg;
            if (IsStable()) {
                targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                        current_position, virtualTarget, ShooterCalculator.RPMToVelocity(actualRPM));
            } else {
                targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                        current_position, virtualTarget, s_lastTargetVelocity);
            }
            update_pitch(targetPitchDeg);
        }

        // --- 階段 8：執行偏航角 (Yaw) 雲台追蹤 ---
        update_yaw();

        // --- 階段 9：供球/射擊動作控制 (isEnable 單純控制供球) ---
        if (isEnable) {
            Intake.setMode(Intake.Mode.shoot);
        } else if (Intake.getMode() == Intake.Mode.shoot) {
            Intake.setMode(Intake.Mode.brake);
        }
    }

    @Override
    public void Stop() {
        if (flywheel != null) {
            flywheel.setPower(0);
            flywheel = null;
        }
        if (yaw1 != null) yaw1.setPosition(0.5);
        if (yaw2 != null) yaw2.setPosition(0.5);
        yaw1 = null;
        yaw2 = null;
        pitch = null;
        m_follower = null;
        isEnable = false;
    }

    // ==========================================
    // 內部運算與硬體控制方法
    // ==========================================

    /**
     * 更新發射器實體在場地上的 Pose (透過底盤 Pose 加上發射器安裝偏移向量的 2D 旋轉變換)
     */
    private void updateShooterPose() {
        Follower follower = (m_follower != null) ? m_follower : Drivetrain.get_Follower();
        if (follower == null || follower.getPose() == null) {
            return;
        }

        Pose robotPose = follower.getPose();
        double heading = robotPose.getHeading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double fwd  = ShooterConfig.relativeFlyWheel.x;
        double left = ShooterConfig.relativeFlyWheel.y;

        double worldX = robotPose.getX() + fwd * cosH - left * sinH;
        double worldY = robotPose.getY() + fwd * sinH + left * cosH;

        current_position.set(worldX, worldY, ShooterConfig.relativeFlyWheel.z, heading);
    }

    /**
     * 移動射擊動態目標補償 (反向平移機器人速度向量 × 飛行時間)
     */
    private void updateMotionPrediction() {
        Follower follower = (m_follower != null) ? m_follower : Drivetrain.get_Follower();
        if (enablePredict && follower != null && follower.getVelocity() != null) {
            Vector rv = follower.getVelocity();
            double ft = ShooterCalculator.calculateFlyTime(
                    current_position, target_position, s_lastTargetVelocity, ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);
            predict_target_position.x = target_position.x - rv.getXComponent() * ft;
            predict_target_position.y = target_position.y - rv.getYComponent() * ft;
            predict_target_position.z = target_position.z;
            predict_target_position.yaw = target_position.yaw;
        } else {
            predict_target_position.set(target_position.x, target_position.y, target_position.z, target_position.yaw);
        }
    }

    /**
     * 讀取並平滑濾波當前真實轉速 (actualRPM)
     */
    private void updateActualRPM() {
        double rawRPM = (flywheel != null)
                ? ShooterCalculator.TicksPerSecToRPM(flywheel.getVelocity()) : 0.0;
        m_rpmBuffer[m_rpmBufferIdx] = rawRPM;
        m_rpmBufferIdx = (m_rpmBufferIdx + 1) % RPM_BUFFER_SIZE;
        if (m_rpmBufferIdx == 0) m_rpmBufferFull = true;

        int count = m_rpmBufferFull ? RPM_BUFFER_SIZE : m_rpmBufferIdx;
        double sum = 0;
        for (int i = 0; i < count; i++) sum += m_rpmBuffer[i];
        actualRPM = (count > 0) ? sum / count : rawRPM;
    }

    /**
     * 計算所需出膛速度與目標 RPM (維持轉速，不隨便歸零)
     */
    private void updateVelocityAndTargetRPM(ShooterPose virtualTarget) {
        double reqVelocity;
        if (pitch != null) {
            reqVelocity = ShooterCalculator.calculateIdealVelocity(
                    current_position, virtualTarget, s_lastTargetVelocity);
        } else {
            reqVelocity = ShooterCalculator.calculateVelocity(
                    current_position, virtualTarget, ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);
        }

        if (reqVelocity > 0) {
            s_lastTargetVelocity = reqVelocity;
            targetRPM = ShooterCalculator.VelocityToRPM(reqVelocity) + ShooterConfig.VELOCITY_OFFSET;
        }
    }

    /**
     * 雲台偏航角控制 (Yaw Auto-Aim)
     */
    private void update_yaw() {
        if (yaw1 == null) return;
        if (!enableAiming) return;

        double target_deg = ShooterCalculator.calculateYaw(current_position, predict_target_position);

        // 套用全局偏移量 (如零位朝向校正)
        target_deg = target_deg + ShooterConfig.YAW_OFFSET_DEG + m_yawInfo.offset;
        while (target_deg >  180) target_deg -= 360;
        while (target_deg < -180) target_deg += 360;

        if (Double.isNaN(target_deg) || Double.isInfinite(target_deg)) return;

        double servoPos = ShooterCalculator.degree2ServoPosition(
                target_deg,
                0,
                ShooterConfig.YAW_SERVO_TOTAL_RANGE_DEG,
                ShooterConfig.YAW_GEAR_RATIO
        );

        yaw1.setPosition(servoPos);
        if (yaw2 != null) {
            yaw2.setPosition(servoPos);
        }

        // 記錄遙測除錯值
        s_lastRelativeYawDeg = target_deg;
        s_lastServoPos = servoPos;
        s_lastTargetFieldAngleDeg = Math.toDegrees(Math.atan2(
                predict_target_position.y - current_position.y,
                predict_target_position.x - current_position.x
        ));
    }

    /**
     * 仰角舵機控制 (Pitch)
     */
    private void update_pitch(double targetPitchDeg) {
        if (pitch == null) return;
        if (Double.isNaN(targetPitchDeg) || Double.isInfinite(targetPitchDeg)) return;

        double range = ShooterConfig.MAX_PITCH_ANGLE_DEG - ShooterConfig.MIN_PITCH_ANGLE_DEG;
        if (range <= 0) range = 35.0;

        double position = (targetPitchDeg - ShooterConfig.MIN_PITCH_ANGLE_DEG) / range;
        pitch.setPosition(Range.clip(position, 0.0, 1.0));
    }
}