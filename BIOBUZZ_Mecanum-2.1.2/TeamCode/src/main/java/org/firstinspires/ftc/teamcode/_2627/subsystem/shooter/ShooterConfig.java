package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

public class ShooterConfig {
    // ================================
    // 調適參數
    // ================================
    public final static double ENERGY_CONVERSION_RATE = 0; //能量轉換率
    public final static double ENERGY_LOSS_RATE = 0; //能量損失率
    public final static double RPM_ERROR_THRESHOLD  = 0; //容忍誤差
    public final static double VELOCITY_OFFSET = 0; //手動轉速調整偏移量
    public final static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 0;

    // ================================
    // 物理常數
    // ================================
    public final static double G = 0;

    // ================================
    // 硬體基本常數
    // ================================
    public final static double FLYWHEEL_RADIUS_IN = 0; //飛輪半徑
    public final static double MAX_VELOCITY_IN_PER_SEC = 0; //飛輪最大線速度
    public final static double PPR = 28; //馬達ticks數
    public final static double GEAR_RATIO = 1; //馬達傳動比
    public final static double ELEVATION_ANGLE = 65; //仰角
    public final static double SHOOTER_HEIGHT = 0; //機器發射器高度

    // ================================
    //場地常數
    // ================================
    public final static ShooterPose BLUE_TARGET_R = new ShooterPose();
    public final static ShooterPose BLUE_TARGET_L = new ShooterPose();
    public final static ShooterPose RED_TARGET_R = new ShooterPose();
    public final static ShooterPose RED_TARGET_L = new ShooterPose();
}
