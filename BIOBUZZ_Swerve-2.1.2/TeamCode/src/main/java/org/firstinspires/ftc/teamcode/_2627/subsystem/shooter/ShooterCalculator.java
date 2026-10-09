package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2627.util.ShooterConfig;

public class ShooterCalculator {

    // ==========================================
    // 1. 物理層：速度與轉速轉換
    // ==========================================

    /**
     * 馬達 ticks/sec 轉換為飛輪 RPM
     */
    public static double ticksPerSec2RPM(double ticksPerSec) {
        double ppr = ShooterConfig.PPR;
        double gearRatio = ShooterConfig.GEAR_RATIO;
        if (ppr <= 0 || gearRatio <= 0) return 0;
        return (ticksPerSec / (ppr * gearRatio)) * 60.0;
    }

    /**
     * 飛輪 RPM 轉換為馬達 ticks/sec
     */
    public static double RPM2ticksPerSec(double rpm) {
        double ppr = ShooterConfig.PPR;
        double gearRatio = ShooterConfig.GEAR_RATIO;
        return (rpm / 60.0) * (ppr * gearRatio);
    }

    /**
     * 發射初速度 (inch/s) 轉換為飛輪所需 RPM
     */
    public static double Velocity2RPM(double exitVelocityInPerSec) {
        if (exitVelocityInPerSec <= 0) return 0;

        double radius = ShooterConfig.FLYWHEEL_RADIUS_IN > 0
                ? ShooterConfig.FLYWHEEL_RADIUS_IN : 1.5;
        double convRate = ShooterConfig.ENERGY_CONVERSION_RATE > 0
                ? ShooterConfig.ENERGY_CONVERSION_RATE : 0.8;
        double intakeV0 = ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;

        // 反推理論飛輪出口速度 (單飛輪靠壁模型，球速理論為表面線速度一半)
        double effectiveV = Math.max(0, exitVelocityInPerSec - intakeV0);
        double theoreticalExitV = effectiveV / convRate;
        double requiredSurfaceV = theoreticalExitV * 2.0;

        // 線速度 (inch/s) -> 轉速 (rev/s) -> RPM
        double rps = requiredSurfaceV / (2.0 * Math.PI * radius);
        return rps * 60.0;
    }

    /**
     * 飛輪 RPM 估算出球初速 (inch/s)
     */
    public static double RPM2Velocity(double rpm) {
        if (rpm <= 0) return 0;

        double radius = ShooterConfig.FLYWHEEL_RADIUS_IN > 0
                ? ShooterConfig.FLYWHEEL_RADIUS_IN : 1.5;
        double convRate = ShooterConfig.ENERGY_CONVERSION_RATE > 0
                ? ShooterConfig.ENERGY_CONVERSION_RATE : 0.8;
        double intakeV0 = ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;

        double rps = rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * radius);
        return (surfaceVelocity / 2.0) * convRate + intakeV0;
    }

    // ==========================================
    // 2. 運動學層：3D 拋體軌跡計算 (全部單位為 inch, 秒, 度)
    // ==========================================

    /**
     * 計算水平投影距離 (inch)
     */
    public static double calculateDistance(ShooterPose current, ShooterPose target) {
        if (current == null || target == null) return 0;
        return Math.hypot(target.x - current.x, target.y - current.y);
    }

    /**
     * 拋體運動：計算命中目標所需的出射初速度 (inch/s)
     */
    public static double calculateVelocity(ShooterPose current, ShooterPose target) {
        if (current == null || target == null) return 0;

        double d = calculateDistance(current, target);
        double deltaH = target.z - current.z;
        double angleRad = Math.toRadians(ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);

        double cosTheta = Math.cos(angleRad);
        double tanTheta = Math.tan(angleRad);

        if (cosTheta <= 0.001) return 0;

        double denominator = 2.0 * (d * tanTheta - deltaH);
        if (denominator <= 0) {
            return 0; // 超出該仰角物理可達射程
        }

        double g = ShooterConfig.G; // 386.0885 in/s^2
        return (d / cosTheta) * Math.sqrt(g / denominator);
    }

    /**
     * 計算飛行時間 (秒)
     */
    public static double calculateFlyTime(ShooterPose current, ShooterPose target) {
        if (current == null || target == null) return 0;

        double d = calculateDistance(current, target);
        double v0 = calculateVelocity(current, target);
        double angleRad = Math.toRadians(ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);
        double cosTheta = Math.cos(angleRad);

        if (v0 <= 0 || cosTheta <= 0.001) return 0;

        return d / (v0 * cosTheta);
    }

    /**
     * 飛行最高點高度 (inch)
     */
    public static double calculateMaxH(double velocity) {
        double angleRad = Math.toRadians(ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);
        double vy = velocity * Math.sin(angleRad);
        return (vy * vy) / (2.0 * ShooterConfig.G);
    }

    // ==========================================
    // 3. 雲台偏航角 (Yaw) 與伺服機映射
    // ==========================================

    /**
     * 計算雲台相對於機器人正前方的瞄準角度 (Degrees)
     * @param robotHeadingRad 機器人當前底盤朝向角 (Radians)
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target, double robotHeadingRad) {
        if (current == null || target == null) return 0;

        double dx = target.x - current.x;
        double dy = target.y - current.y;

        // 場地絕對瞄準角
        double targetFieldAngleRad = Math.atan2(dy, dx);

        // 減去底盤朝向，取得相對車頭的角度差
        double relativeAngleRad = targetFieldAngleRad - robotHeadingRad;

        // 正規化至 [-PI, PI]
        while (relativeAngleRad > Math.PI) relativeAngleRad -= 2.0 * Math.PI;
        while (relativeAngleRad < -Math.PI) relativeAngleRad += 2.0 * Math.PI;

        return Math.toDegrees(relativeAngleRad);
    }

    /**
     * 相對目標角度 (度) 映射為 Servo Position [0.0, 1.0]
     */
    public static double degree2ServoPosition(double relativeDeg, double offsetDeg, double totalRangeDeg) {
        if (totalRangeDeg <= 0) totalRangeDeg = 180.0;
        // 0 度相對角時舵機處於中心位置 (0.5)
        double adjustedDeg = relativeDeg + offsetDeg;
        double position = 0.5 + (adjustedDeg / totalRangeDeg);
        return Range.clip(position, 0.0, 1.0);
    }

    // ==========================================
    // 4. 移動射擊動態目標補償 (Virtual Target Prediction)
    // ==========================================

    /**
     * 計算移動射擊時的虛擬目標點 (不會修改 realTarget 原始物件)
     */
    public static ShooterPose calculateVirtualTarget(ShooterPose current, ShooterPose realTarget,
                                                     double robotVx, double robotVy) {
        if (current == null || realTarget == null) return new ShooterPose();

        double flyTime = calculateFlyTime(current, realTarget);
        double virtX = realTarget.x - robotVx * flyTime;
        double virtY = realTarget.y - robotVy * flyTime;
        double virtZ = realTarget.z;

        return new ShooterPose(virtX, virtY, virtZ);
    }
}