package org.firstinspires.ftc.teamcode._2627.util;

import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.ShooterPose;

public class ShooterConfig {
    // ================================
    // 調適參數
    // ================================
    public static double ENERGY_CONVERSION_RATE = 0.80; // 能量轉換率 (0.7 ~ 0.85 視泡棉球擠壓程度)
    public static double VELOCITY_OFFSET = 0;          // 手動轉速偏移 (RPM)
    public static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 0;

    // ================================
    // 物理常數 (英制 inch/s^2)
    // ================================
    public final static double G = 386.0885;

    // ================================
    // 硬體基本常數
    // ================================
    public static double FLYWHEEL_RADIUS_IN = 1.5;     // 飛輪半徑 (inch)
    public static double PPR = 28.0;                   // 馬達轉子每圈編碼數
    public static double GEAR_RATIO = 1.0;             // 傳動比 (例如 1:1 或 0.83)
    public static double FIXED_LAUNCH_ANGLE_DEG = 65.0;// 發射器仰角 (度)
    public static double SHOOTER_HEIGHT = 14.0;        // 發射口距地高度 (inch)
    public static double YAW_SERVO_TOTAL_RANGE_DEG = 180.0; // 舵機總角度範圍 (度)

    // ================================
    // 場地目標位置常數 (單位：inch)
    // ================================
    // 請填入實際場地籃框座標 (X, Y, Z)
    public final static ShooterPose BLUE_TARGET_R = new ShooterPose(72, 144, 42);
    public final static ShooterPose BLUE_TARGET_L = new ShooterPose(48, 144, 42);
    public final static ShooterPose RED_TARGET_R = new ShooterPose(72, 0, 42);
    public final static ShooterPose RED_TARGET_L = new ShooterPose(48, 0, 42);

    // 發射器相對於機器人旋轉中心偏移 (X 前, Y 左, Z 高)
    public final static ShooterPose relativeFlyWheel = new ShooterPose(0, 0, 0);
}
