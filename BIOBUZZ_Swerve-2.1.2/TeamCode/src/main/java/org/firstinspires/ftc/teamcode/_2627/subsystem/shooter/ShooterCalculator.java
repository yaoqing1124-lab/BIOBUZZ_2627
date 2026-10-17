package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2627.constants.ShooterConfig;

/**
 * 發射器數學與物理計算核心 (ShooterCalculator)
 * 參考 2026 發射器子系統規格書 (Shooter.md)
 *
 * 系統被劃分為三大層級：
 * 1. 擬合物理層 (Physical Fitting Layer)：雙參數架構 (ENERGY_CONVERSION_RATE, ENERGY_LOSS_RATE)
 * 2. 數學模型層 (Mathematical Model Layer)：拋體運動軌跡、虛擬目標轉換、仰角/偏航角解算
 * 3. 運動學補償 (Motion Prediction Layer)：底盤動態速度補償
 */
public class ShooterCalculator {

    // ==========================================
    // 1. 擬合物理層 (Physical Fitting Layer)
    // ==========================================

    /**
     * 馬達 Encoder Ticks/sec 轉換為飛輪 RPM
     */
    public static double TicksPerSecToRPM(double ticksPerSec) {
        double ppr = ShooterConfig.PPR;
        double gearRatio = ShooterConfig.GEAR_RATIO;
        if (ppr <= 0 || gearRatio <= 0) return 0.0;
        return (ticksPerSec / (ppr * gearRatio)) * 60.0;
    }

    /** 向後相容別名 */
    public static double ticksPerSec2RPM(double ticksPerSec) {
        return TicksPerSecToRPM(ticksPerSec);
    }

    /**
     * 飛輪 RPM 轉換為馬達 Encoder Ticks/sec
     */
    public static double RPMToTicksPerSec(double rpm) {
        double ppr = ShooterConfig.PPR;
        double gearRatio = ShooterConfig.GEAR_RATIO;
        return (rpm / 60.0) * (ppr * gearRatio);
    }

    /** 向後相容別名 */
    public static double RPM2ticksPerSec(double rpm) {
        return RPMToTicksPerSec(rpm);
    }

    /**
     * 階段 I：物理層轉換 (RPM → 出膛初速度 in/s)
     * 單飛輪 + 弧形軌道 (Hooded Shooter)：球體出膛初速理論上為飛輪表面線速度的一半。
     * 最後乘上動態擬合常數 ENERGY_CONVERSION_RATE 彌補打滑與壓縮損耗。
     */
    public static double RPMToVelocity(double rpm) {
        if (rpm <= 0) return 0.0;
        double radius = ShooterConfig.FLYWHEEL_RADIUS_IN > 0 ? ShooterConfig.FLYWHEEL_RADIUS_IN : 1.5;
        double convRate = ShooterConfig.ENERGY_CONVERSION_RATE > 0 ? ShooterConfig.ENERGY_CONVERSION_RATE : 0.8;
        double intakeV0 = ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;

        double rps = rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * radius);
        double theoreticalExitVelocity = surfaceVelocity / 2.0;
        return theoreticalExitVelocity * convRate + intakeV0;
    }

    /** 向後相容別名 */
    public static double RPM2Velocity(double rpm) {
        return RPMToVelocity(rpm);
    }

    /**
     * 階段 I：物理層轉換 (初速度 in/s → 目標 RPM)
     * 將目標初速轉換回飛輪馬達所需的目標 RPM。
     */
    public static double VelocityToRPM(double velocity) {
        if (velocity <= 0) return 0.0;
        double radius = ShooterConfig.FLYWHEEL_RADIUS_IN > 0 ? ShooterConfig.FLYWHEEL_RADIUS_IN : 1.5;
        double convRate = ShooterConfig.ENERGY_CONVERSION_RATE > 0 ? ShooterConfig.ENERGY_CONVERSION_RATE : 0.8;
        double intakeV0 = ShooterConfig.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;

        double effectiveV = Math.max(0, velocity - intakeV0);
        double theoreticalExitVelocity = effectiveV / convRate;
        double requiredSurfaceVelocity = theoreticalExitVelocity * 2.0;
        double rps = requiredSurfaceVelocity / (2.0 * Math.PI * radius);
        return rps * 60.0;
    }

    /** 向後相容別名 */
    public static double Velocity2RPM(double velocity) {
        return VelocityToRPM(velocity);
    }

    /**
     * 現場調適 SOP 步驟 1：近距離測試 (求解 ENERGY_CONVERSION_RATE)
     * 在近距離 (例如 24 吋，忽略空氣阻力) 固定仰角為 45 度發射，
     * 調整轉速至完美命中後，傳入該完美 RPM，此函式回傳純粹硬體傳輸效率。
     *
     * @param distance_in 離目標的水平距離 (inch)
     * @param height_in   與目標的高度差 (inch)
     * @param perfect_rpm 能完美命中的飛輪轉速 (RPM)
     * @return 應寫入 ShooterConfig.ENERGY_CONVERSION_RATE 的數值
     */
    public static double calibrateEnergyConversionRate(double distance_in, double height_in, double perfect_rpm) {
        if (distance_in <= 0 || perfect_rpm <= 0) return 1.0;
        double theta_rad = Math.toRadians(45.0);
        double tan_theta = Math.tan(theta_rad);
        double cos_theta = Math.cos(theta_rad);

        double denominator = 2.0 * Math.pow(cos_theta, 2) * (distance_in * tan_theta - height_in);
        if (denominator <= 0) return 1.0;

        double ideal_velocity = Math.sqrt((ShooterConfig.G * Math.pow(distance_in, 2)) / denominator);

        double rps = perfect_rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * ShooterConfig.FLYWHEEL_RADIUS_IN);
        double theoreticalExitVelocity = surfaceVelocity / 2.0;

        if (theoreticalExitVelocity <= 0) return 1.0;
        return ideal_velocity / theoreticalExitVelocity;
    }

    // ==========================================
    // 2. 數學模型層 (Mathematical Model Layer)
    // ==========================================

    /**
     * 計算水平投影距離 (inch)
     */
    public static double calculateDistance(ShooterPose current, ShooterPose target) {
        if (current == null || target == null) return 0.0;
        return Math.hypot(target.x - current.x, target.y - current.y);
    }

    /**
     * 階段 III：虛擬目標變換 (補償空氣阻力與 Magnus 下墜效應)
     * 透過 ENERGY_LOSS_RATE 產生一個「虛擬目標點 B'」，使得標準拋物線模型自動計算出更大的初速與適當仰角。
     */
    public static ShooterPose calculateVirtualTarget(ShooterPose current, ShooterPose realTarget) {
        if (current == null || realTarget == null) return new ShooterPose();

        double d = Math.hypot(realTarget.x - current.x, realTarget.y - current.y);
        double lossRate = ShooterConfig.ENERGY_LOSS_RATE;

        // 水平距離平移補償空氣阻力，垂直高度平移補償提早下墜
        double virtual_d = d * (1.0 + lossRate * d);
        double virtual_h = (realTarget.z - current.z) + (lossRate * Math.pow(d, 2));

        double yaw_rad = Math.atan2(realTarget.y - current.y, realTarget.x - current.x);
        double virtual_x = current.x + virtual_d * Math.cos(yaw_rad);
        double virtual_y = current.y + virtual_d * Math.sin(yaw_rad);
        double virtual_z = current.z + virtual_h;

        return new ShooterPose(virtual_x, virtual_y, virtual_z, realTarget.yaw);
    }

    /**
     * 固定發射仰角計算：給定發射仰角 θ (度)，計算命中目標所需的出射初速度 (inch/s)
     */
    public static double calculateVelocity(ShooterPose current, ShooterPose target, double launchAngleDeg) {
        if (current == null || target == null) return 0.0;

        double d = calculateDistance(current, target);
        double deltaH = target.z - current.z;
        if (d < 1.0) return 0.0;

        double angleRad = Math.toRadians(launchAngleDeg);
        double cosTheta = Math.cos(angleRad);
        double tanTheta = Math.tan(angleRad);

        if (cosTheta <= 0.001) return 0.0;

        double denominator = 2.0 * Math.pow(cosTheta, 2) * (d * tanTheta - deltaH);
        if (denominator <= 0) return 0.0; // 超出該仰角的物理可達射程

        double g = ShooterConfig.G;
        double v = Math.sqrt((g * Math.pow(d, 2)) / denominator);
        return Math.min(v, ShooterConfig.MAX_VELOCITY_IN_PER_SEC);
    }

    /**
     * 預設多載方法：使用 ShooterConfig.FIXED_LAUNCH_ANGLE_DEG 計算所需初速度
     */
    public static double calculateVelocity(ShooterPose current, ShooterPose target) {
        return calculateVelocity(current, target, ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);
    }

    /**
     * 階段 IV：計算理想初速度 (以理想入射角反求)
     */
    public static double calculateIdealVelocity(ShooterPose current, ShooterPose virtualTarget, double lastVelocity) {
        if (current == null || virtualTarget == null) return lastVelocity;
        double d = Math.hypot(virtualTarget.x - current.x, virtualTarget.y - current.y);
        double h = virtualTarget.z - current.z;
        if (d < 1.0) return lastVelocity;

        double physics_steepest = 90.0 - ShooterConfig.MIN_PITCH_ANGLE_DEG;
        double physics_flattest  = 90.0 - ShooterConfig.MAX_PITCH_ANGLE_DEG;

        double d_cross = 2.0 * h / (Math.tan(Math.toRadians(physics_steepest))
                - Math.tan(Math.toRadians(ShooterConfig.IDEAL_INCIDENT_ANGLE_DEG)));
        double d_min   = h / Math.tan(Math.toRadians(physics_steepest));

        double launchPhysicsDeg;
        if (d >= d_cross && d_cross > d_min) {
            launchPhysicsDeg = incidentToLaunchAngle(ShooterConfig.IDEAL_INCIDENT_ANGLE_DEG, d, h);
        } else if (d <= d_min) {
            launchPhysicsDeg = physics_steepest;
        } else {
            double t = (d - d_min) / Math.max(0.001, d_cross - d_min);
            launchPhysicsDeg = physics_steepest - t * (physics_steepest - physics_flattest);
        }

        launchPhysicsDeg = Range.clip(launchPhysicsDeg, physics_flattest, physics_steepest);

        double theta_rad   = Math.toRadians(launchPhysicsDeg);
        double tan_theta   = Math.tan(theta_rad);
        double cos_theta   = Math.cos(theta_rad);
        double denominator = 2.0 * Math.pow(cos_theta, 2) * (d * tan_theta - h);

        double v_ideal;
        if (denominator > 0) {
            v_ideal = Math.sqrt((ShooterConfig.G * Math.pow(d, 2)) / denominator);
        } else {
            v_ideal = lastVelocity;
        }
        return Math.min(v_ideal, ShooterConfig.MAX_VELOCITY_IN_PER_SEC);
    }

    /**
     * 將理想入射角 (arrival angle) 轉換為發射仰角 (launch angle)
     */
    public static double incidentToLaunchAngle(double incidentAngleDeg, double d, double h) {
        if (d <= 0) return incidentAngleDeg;
        double tan_incident = Math.tan(Math.toRadians(incidentAngleDeg));
        double tan_launch   = tan_incident + (2.0 * h / d);
        return Math.toDegrees(Math.atan(tan_launch));
    }

    /**
     * 階段 V：計算實際仰角 (真實轉速反饋控制)
     * 根據當前真實初速度，反解拋物線二次方程式，求出當前能命中虛擬目標的仰角
     */
    public static double calculateRealPitchAngle(ShooterPose current, ShooterPose virtualTarget, double actual_velocity) {
        if (actual_velocity <= 0 || current == null || virtualTarget == null) {
            return ShooterConfig.MAX_PITCH_ANGLE_DEG;
        }

        double d = Math.hypot(virtualTarget.x - current.x, virtualTarget.y - current.y);
        double h = virtualTarget.z - current.z;
        if (d < 1.0) return ShooterConfig.MAX_PITCH_ANGLE_DEG;

        double A = (ShooterConfig.G * Math.pow(d, 2)) / (2.0 * Math.pow(actual_velocity, 2));
        double B = d;
        double C = h + A;

        double discriminant = B * B - 4.0 * A * C;
        if (discriminant < 0) {
            return ShooterConfig.MAX_PITCH_ANGLE_DEG;
        }

        double tan_theta_low = (B - Math.sqrt(discriminant)) / (2.0 * A);
        double theta_low_physics = Math.toDegrees(Math.atan(tan_theta_low));

        double physics_flattest  = 90.0 - ShooterConfig.MAX_PITCH_ANGLE_DEG;
        double physics_steepest  = 90.0 - ShooterConfig.MIN_PITCH_ANGLE_DEG;

        double selected_physics = Range.clip(theta_low_physics, physics_flattest, physics_steepest);
        double selected_servo   = 90.0 - selected_physics;

        selected_servo += ShooterConfig.PITCH_OFFSET_DEG;
        return Range.clip(selected_servo, ShooterConfig.MIN_PITCH_ANGLE_DEG, ShooterConfig.MAX_PITCH_ANGLE_DEG);
    }

    /**
     * 水平飛行時間 (秒)
     */
    public static double calculateFlyTime(ShooterPose current, ShooterPose target, double velocity, double launchAngleDeg) {
        if (current == null || target == null || velocity <= 0) return 0.0;
        double d = calculateDistance(current, target);
        double cosTheta = Math.cos(Math.toRadians(launchAngleDeg));
        if (cosTheta <= 0.001) return 0.0;
        return d / (velocity * cosTheta);
    }

    /**
     * 垂直拋物線飛行時間 (秒)
     */
    public static double flytimeVertical(double v, double pitchDeg, double h) {
        if (v <= 0) return 0.0;
        double g = ShooterConfig.G;
        double vz = v * Math.sin(Math.toRadians(pitchDeg));
        double disc = vz * vz - 2.0 * g * h;
        if (disc < 0) return 0.0;
        double sqrt = Math.sqrt(disc);
        double tA = (vz + sqrt) / g;
        double tB = (vz - sqrt) / g;
        if (tA > 0 && tB > 0) return Math.max(tA, tB);
        if (tA > 0) return tA;
        if (tB > 0) return tB;
        return 0.0;
    }

    // ==========================================
    // 3. 雲台偏航角 (Yaw) 與伺服機映射
    // ==========================================

    /**
     * 計算雲台相對於當前車頭朝向的角度差 (Degrees)，正值向左 (CCW)，負值向右 (CW)
     * @param current 當前發射器位置 (current.yaw 為底盤朝向角弧度)
     * @param target  目標點位置
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target) {
        if (current == null || target == null) return 0.0;
        return calculateYaw(current, target, current.yaw);
    }

    /**
     * 計算雲台相對於底盤朝向的角度差 (Degrees)
     * @param current 發射器當前位置
     * @param target  目標點位置
     * @param robotHeadingRad 底盤朝向角 (Radians)
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target, double robotHeadingRad) {
        if (current == null || target == null) return 0.0;

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
     *
     * @param relativeDeg 雲台目標相對角度 (度)
     * @param offsetDeg 舵機安裝角度偏移量 (度)
     * @param totalRangeDeg 舵機本身最大物理行程總角度 (度)
     * @param gearRatio 傳動比 (舵機轉動角度 / 雲台轉動角度)
     */
    public static double degree2ServoPosition(double relativeDeg, double offsetDeg, double totalRangeDeg, double gearRatio) {
        if (totalRangeDeg <= 0) totalRangeDeg = 180.0;
        if (gearRatio == 0) gearRatio = 1.0;
        // 1. 雲台所需調整的實體角度
        double adjustedTurretDeg = relativeDeg + offsetDeg;
        // 2. 經傳動比換算為舵機需要轉動的角度
        double servoDeltaDeg = adjustedTurretDeg * gearRatio;
        // 3. 映射至 Servo [0.0, 1.0] 範圍 (0 度相對角時處於中心 0.5)
        double position = 0.5 + (servoDeltaDeg / totalRangeDeg);
        return Range.clip(position, 0.0, 1.0);
    }

    /**
     * 預設多載方法（自動套用 ShooterConfig.YAW_GEAR_RATIO）
     */
    public static double degree2ServoPosition(double relativeDeg, double offsetDeg, double totalRangeDeg) {
        return degree2ServoPosition(relativeDeg, offsetDeg, totalRangeDeg, ShooterConfig.YAW_GEAR_RATIO);
    }

    // ==========================================
    // 4. 動態移動射擊目標補償 (Motion Prediction)
    // ==========================================

    /**
     * 根據底盤速度向量 (vx, vy) 與飛行時間，計算出前饋虛擬目標位置
     */
    public static ShooterPose calculateMotionPredictedTarget(ShooterPose current, ShooterPose realTarget,
                                                             double robotVx, double robotVy, double flyTime) {
        if (current == null || realTarget == null) return new ShooterPose();
        double predX = realTarget.x - robotVx * flyTime;
        double predY = realTarget.y - robotVy * flyTime;
        double predZ = realTarget.z;
        return new ShooterPose(predX, predY, predZ, realTarget.yaw);
    }

    /** 向後相容舊方法 */
    public static ShooterPose calculateVirtualTarget(ShooterPose current, ShooterPose realTarget,
                                                     double robotVx, double robotVy) {
        double v0 = calculateVelocity(current, realTarget);
        double flyTime = calculateFlyTime(current, realTarget, v0, ShooterConfig.FIXED_LAUNCH_ANGLE_DEG);
        return calculateMotionPredictedTarget(current, realTarget, robotVx, robotVy, flyTime);
    }
}