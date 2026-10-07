package org.firstinspires.ftc.teamcode.Shooter;

public class ShooterCalculator {
    // ================
    // 物理層轉換
    // ================

    /**
     * encoder ticks 每秒轉換 rpm
     */
    public static double ticksPerSecToRPM(double ticksPerSecToRPM){
        return ;
    }

    /**
     * rpm 轉換 encoder ticks 每秒
     */
    public static double RPMToTicksPerSec(double rpm){
        return ;
    }

    /**
     * RPM轉換離開初速度
     */
    public static double RPMToVelocity(double rpm){
        return ;
    }

    /**
     *  初速度轉換馬達RPM
     */
    public static double VelocityToRPM(double velocity){
        return ;
    }



    // ================
    // 運動學
    // ================

    /**
     * 計算炮台和目標投影點距離
     */
    public static double distance(ShooterPose pose){
        return ;
    }

    /**
     * 計算飛行時間
     */
    public static double calculateFlyTime(double velocity){
        return ;
    }

    /**
     * 計算飛行高度
     */
    public static double calculateH(double velocity){
        return ;
    }

    /**
     * 計算理想初速度
     */
    public static double calculateVelocity(double distance){
        return ;
    }

    /**
     * 計算偏航角
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target) {
        return ;
    }
}
