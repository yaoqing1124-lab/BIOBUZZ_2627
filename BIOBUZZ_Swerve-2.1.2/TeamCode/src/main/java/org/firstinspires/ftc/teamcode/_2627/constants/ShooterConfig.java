package org.firstinspires.ftc.teamcode._2627.constants;

import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.ShooterPose;

/**
 * 發射器子系統規格設定 (Shooter Subsystem Configuration)
 * 參考 2026 發射器子系統架構規格書 (Shooter.md)
 */
public class ShooterConfig {
    // ==========================================
    // 1. 雙參數架構 - 場地動態擬合常數 (Two-Parameter Architecture)
    // ==========================================
    /** 能量轉換率 (0.5 ~ 0.9，近距離無阻力測試求得，彌補球體擠壓與打滑) */
    public static double ENERGY_CONVERSION_RATE = 0.8;

    /** 能量損失率 (遠距離測試求得，透過虛擬平移目標點補償空氣阻力與 Magnus 效應下墜，例如 0.002) */
    public static double ENERGY_LOSS_RATE = 0.0;

    /** 轉速誤差容忍度 (RPM)，當 |目標RPM - 實際RPM| < 此值時判定為穩定 (IsStable) */
    public static double RPM_ERROR_THRESHOLD = 50.0;

    /** 手動轉速微調偏移量 (RPM) */
    public static double VELOCITY_OFFSET = 0.0;

    /** 進球初速度 (inch/s) */
    public static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 0.0;

    // ==========================================
    // 2. 物理常數 (英制 inch/s^2)
    // ==========================================
    /** 重力加速度 (in/s^2) */
    public final static double G = 386.0885;

    // ==========================================
    // 3. 硬體基本常數
    // ==========================================
    /** 飛輪半徑 (inch) */
    public static double FLYWHEEL_RADIUS_IN = 2.5 / 2.54;

    /** 飛輪能提供的最大線速度極限 (inch/s) */
    public static double MAX_VELOCITY_IN_PER_SEC = 800.0;

    /** 馬達轉子每圈編碼數 (Bare motor = 28.0) */
    public static double PPR = 28.0;

    /** 飛輪馬達傳動比 (馬達轉圈數 / 飛輪轉圈數，例如 1:1 或 0.66) */
    public static double GEAR_RATIO = 0.66;

    // ==========================================
    // 4. 仰角 (Pitch) 控制參數
    // ==========================================
    /** 固定發射器仰角 (度，無仰角伺服馬達時使用) */
    public static double FIXED_LAUNCH_ANGLE_DEG = 65.0;

    /** 理想入射角 (度，期望球體落入籃框時的完美角度，例如 60.0) */
    public static double IDEAL_INCIDENT_ANGLE_DEG = 60.0;

    /** 仰角伺服馬達物理極限最小/最大角度 (度) */
    public static double MIN_PITCH_ANGLE_DEG = 30.0;
    public static double MAX_PITCH_ANGLE_DEG = 65.0;

    /** 仰角手動偏移微調 (度) */
    public static double PITCH_OFFSET_DEG = 0.0;

    // ==========================================
    // 5. 偏航角 (Yaw) 雲台控制參數
    // ==========================================
    /** 舵機總物理行程角度 (度，例如 goBILDA 雙模舵機填 300.0，REV Smart 填 270.0，全行程填 360.0) */
    public static double YAW_SERVO_TOTAL_RANGE_DEG = 360.0;

    /**
     * 雲台舵機傳動比 = (舵機旋轉角度 / 雲台旋轉角度) = (雲台齒數 / 舵機齒數)
     * 例如：舵機 20 齒帶動雲台 60 齒，減速 3 倍，請填 3.0 (雲台轉 1 度，舵機轉 3 度)
     * 若為 1:1 直驅請填 1.0
     * 若旋轉瞄準時方向相反 (車向左轉雲台也向左轉)，請將此值改為負數 (例如 -1.0 或 -3.0)！
     */
    public static double YAW_GEAR_RATIO = 1.0;

    /** 雲台正向角度偏移 (度，預設 0 度為正前；若舵機 0.5 位置朝正後方請填 -180.0) */
    public static double YAW_OFFSET_DEG = 0.0;

    // ==========================================
    // 6. 場地目標與發射器安裝偏移常數 (單位：inch)
    // ==========================================
    /** 籃框目標高度 (inch) */
    public static double BASKET_HEIGHT_IN = (53.5 + 65.6) / 2.0;

    public final static ShooterPose BLUE_TARGET_R = new ShooterPose(-12.9, -19.89, BASKET_HEIGHT_IN);
    public final static ShooterPose BLUE_TARGET_L = new ShooterPose(12.9, 19.89, BASKET_HEIGHT_IN);
    public final static ShooterPose RED_TARGET_R = new ShooterPose(-12.9, 19.89, BASKET_HEIGHT_IN);
    public final static ShooterPose RED_TARGET_L = new ShooterPose(12.9, -19.89, BASKET_HEIGHT_IN);

    /** 發射器旋轉軸心相對於機器人自轉中心偏移 (X: 前+, Y: 左+, Z: 高度) */
    public final static ShooterPose relativeFlyWheel = new ShooterPose(0, 0, 10.43);
}
