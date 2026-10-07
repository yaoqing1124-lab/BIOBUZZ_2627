package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.qualcomm.robotcore.util.Range;

public class ShooterCalculator {
    // ================
    // 物理層轉換
    // ================

    /**
     * encoder ticks 每秒轉換 rpm
     */
    public static double ticksPerSec2RPM(double ticksPerSec){
        return (ticksPerSec / (ShooterConfig.PPR * ShooterConfig.GEAR_RATIO)) * 60.0;
    }
    public static double RPM2ticksPerSec(double rpm){
        return rpm / 60 * (ShooterConfig.PPR * ShooterConfig.GEAR_RATIO);
    }

    /**
     * RPM轉換離開初速度
     */
    public static double RPM2Velocity(double rpm){
        double rps = rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * ShooterConfig.FLYWHEEL_RADIUS_IN);
        double theoreticalExitVelocity = surfaceVelocity / 2.0;
        return theoreticalExitVelocity * ShooterConfig.ENERGY_CONVERSION_RATE + ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;
    }

    /**
     *  初速度轉換馬達RPM
     */
    public static double Velocity2RPM(double velocity) {
        double theoreticalExitVelocity = (velocity - ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC) / ShooterConfig.ENERGY_CONVERSION_RATE;
        double requiredSurfaceVelocity = theoreticalExitVelocity * 2.0;
        double rps = requiredSurfaceVelocity / (2.0 * Math.PI * ShooterConfig.FLYWHEEL_RADIUS_IN);
        return rps * 60.0;
    }



    // ================
    // 運動學
    // ================
    /**
     * 計算飛行時間
     */
    public static double calculateFlyTime(ShooterPose current, ShooterPose target){
        //TODO
        return ;
    }

    /**
     * 計算飛行高度
     */
    public static double calculateMaxH(ShooterPose current, ShooterPose target){
        //TODO
        return ;
    }

    /**
     * 計算理想初速度
     */
    public static double calculateVelocity(ShooterPose current, ShooterPose target){
        //TODO
        return ;
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
        return Range.clip(target_deg, -177.5, 177.5);
    }
}
