package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2627.config.Config;

public class ShooterCalculator {
    // ================
    // 物理層轉換
    // ================

    /**
     * encoder ticks 每秒轉換 rpm
     */
    public static double ticksPerSec2RPM(double ticksPerSec){
        return (ticksPerSec / (Config.ShooterConfig.PPR * Config.ShooterConfig.GEAR_RATIO)) * 60.0;
    }
    public static double RPM2ticksPerSec(double rpm){
        return rpm / 60 * (Config.ShooterConfig.PPR * Config.ShooterConfig.GEAR_RATIO);
    }

    /**
     * RPM轉換離開初速度
     */
    public static double RPM2Velocity(double rpm, ShooterPose current, ShooterPose target){
        double rps = rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * Config.ShooterConfig.FLYWHEEL_RADIUS_IN);
        double theoreticalExitVelocity = surfaceVelocity / 2.0;
        return theoreticalExitVelocity * Config.ShooterConfig.ENERGY_CONVERSION_RATE + // 飛輪輸出能量損耗
                Config.ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC +
                calculateDistance(current, target) * (1 - Config.ShooterConfig.ENERGY_LOSS_RATE); //計算遠距離受空氣阻力和馬格努斯效應補償
    }

    /**
     *  初速度轉換馬達RPM
     */
    public static double Velocity2RPM(double velocity) {
        double theoreticalExitVelocity = (velocity - Config.ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC) / Config.ShooterConfig.ENERGY_CONVERSION_RATE;
        double requiredSurfaceVelocity = theoreticalExitVelocity * 2.0;
        double rps = requiredSurfaceVelocity / (2.0 * Math.PI * Config.ShooterConfig.FLYWHEEL_RADIUS_IN);
        return rps * 60.0;
    }



    // ================
    // 運動學
    // ================

    /**
     * 計算飛行時間
     */
    public static double calculateFlyTime(ShooterPose current, ShooterPose target){
        return calculateDistance(current, target) / (calculateVelocity(current, target) * Math.cos(Config.ShooterConfig.ELEVATION_ANGLE));
    }
    /**
     * 計算飛行距離
     */
    public static double calculateDistance(ShooterPose current, ShooterPose target){
        return Math.hypot(current.x - target.x, current.y - current.y);
    }

    /**
     * 計算飛行高度
     * 只能用在判斷當下出去的速度會不會碰到天花板
     */
    public static double calculateMaxH(double velocity){
        return Math.pow(velocity * Math.sin(Config.ShooterConfig.ELEVATION_ANGLE), 2) / (2 * Config.ShooterConfig.G);
    }

    /**
     * 計算理想初速度
     */
    public static double calculateVelocity(ShooterPose current, ShooterPose target){
        double R = calculateDistance(current, target);
        double H = Config.ShooterConfig.H;
        return (R / Math.cos(Math.toRadians(Config.ShooterConfig.ELEVATION_ANGLE))) *
                Math.sqrt(Config.ShooterConfig.G / (2 * R * Math.tan(Math.toRadians(Config.ShooterConfig.ELEVATION_ANGLE) - 2 * H)));
    }

    /**
     * 計算偏航角
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target) {
        double targetYaw_rad = Math.atan2(target.y - current.y, target.x - current.x);
        double heading_rad   = current.yaw;
        double target_deg    = Math.toDegrees(targetYaw_rad - heading_rad);
        while (target_deg >  180) target_deg -= 360;
        while (target_deg < -180) target_deg += 360;
        return Range.clip(target_deg, -177.5, 177.5); //TODO
    }
}
