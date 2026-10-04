package org.firstinspires.ftc.teamcode.Shooter;

public class ShooterConfig {
    // ================================
    // 調適參數
    // ================================
    public final static double ENERGY_CONVERSION_RATE = 0; //能量轉換率
    public final static double ENERGY_LOSS_RATE = 0; //能量損失率
    public final static double RPM_ERROR_THRESHOLD  = 0; //容忍誤差
    public final static double VELOCITY_OFFSET = 0; //手動轉速調整偏移量

    // ================================
    // 物理常數
    // ================================
    public final static double G = 0;

    // ================================
    // 硬體基本常數
    // ================================
    public final static double FLYWHEEL_RADIUS_IN = 0; //飛輪半徑
    public final static double MAX_VELOCITY_IN_PER_SEC = 0; //飛輪最大線速度
    public final static double PPR = 0; //馬達ticks數
    public final static double GEAR_RATIO = 0; //馬達傳動比
    public final static double ELEVATION_ANGLE = 0; //仰角

    // ================================
    //場地常數
    // ================================
    public final static ShooterPose BLUE_TARGET_R = new ShooterPose();
    public final static ShooterPose BLUE_TARGET_L = new ShooterPose();
    public final static ShooterPose RED_TARGET_R = new ShooterPose();
    public final static ShooterPose RED_TARGET_L = new ShooterPose();
}
