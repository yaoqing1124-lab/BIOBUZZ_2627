package org.firstinspires.ftc.teamcode._2627.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.ShooterPose;

public class Config {
    // ================================================
    // DcMotor Config
    // ================================================
    public static class Motor{
        public final static MotorConfig lfm = new MotorConfig("lfm", DcMotor.Direction.FORWARD);
        public final static MotorConfig lbm = new MotorConfig("lbm", DcMotor.Direction.FORWARD);
        public final static MotorConfig rfm = new MotorConfig("rfm", DcMotor.Direction.FORWARD);
        public final static MotorConfig rbm = new MotorConfig("rbm", DcMotor.Direction.FORWARD);
        public final static MotorConfig flyWheel = new MotorConfig("flywheel", DcMotorSimple.Direction.FORWARD);
        public final static MotorConfig intake = new MotorConfig("intake", DcMotorSimple.Direction.FORWARD);
        public final static MotorConfig intakeCtrl = new MotorConfig("intakeCtrl", DcMotorSimple.Direction.FORWARD);
    }
    // ================================================
    // Servo Config
    // ================================================
    public static class Servo{
        private final static double YAW_OFFSET = 0;
        public final static ServoConfig lfs = new ServoConfig("lfs");
        public final static ServoConfig lbs = new ServoConfig("lbs");
        public final static ServoConfig rfs = new ServoConfig("rfs");
        public final static ServoConfig rbs = new ServoConfig("rbs");
        public final static ServoConfig trig = new ServoConfig("trig");
        public final static ServoConfig yaw1 = new ServoConfig("yaw1", YAW_OFFSET);
        public final static ServoConfig yaw2 = new ServoConfig("yaw2", YAW_OFFSET);
        public final static ServoConfig hand = new ServoConfig("hand");

    }
    // ================================================
    // Sensor Config
    // ================================================
    public static class Sensor{
        public final static AnalogConfig lfenc = new AnalogConfig("lfenc");
        public final static AnalogConfig lbenc = new AnalogConfig("lbenc");
        public final static AnalogConfig rfenc = new AnalogConfig("rfenc");
        public final static AnalogConfig rbenc = new AnalogConfig("rbenc");
        public final static PinpointConfig pinpoint = new PinpointConfig("pinpoint", GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD, 0,0);
    }
    public static class Tuning{
        public final static double intakePower = 1;
        public final static double TRIG_OPEN_POSITION = 1;
        public final static double TRIG_CLOSE_POSITION = 0;
    }
    public static class ShooterConfig {
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
        public final static double H = 0;
        public final static ShooterPose BLUE_TARGET_R = new ShooterPose();
        public final static ShooterPose BLUE_TARGET_L = new ShooterPose();
        public final static ShooterPose RED_TARGET_R = new ShooterPose();
        public final static ShooterPose RED_TARGET_L = new ShooterPose();
        public final static ShooterPose relativeFlyWheel = new ShooterPose();
    }

}
