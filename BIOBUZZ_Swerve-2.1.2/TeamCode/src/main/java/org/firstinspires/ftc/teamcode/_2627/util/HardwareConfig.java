package org.firstinspires.ftc.teamcode._2627.util;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode._2627.config.AnalogConfig;
import org.firstinspires.ftc.teamcode._2627.config.MotorConfig;
import org.firstinspires.ftc.teamcode._2627.config.PinpointConfig;
import org.firstinspires.ftc.teamcode._2627.config.ServoConfig;

public class HardwareConfig {
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
//        public final static ServoConfig hand = new ServoConfig("hand");

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
        public final static double TRIG_POSITION = 1;
    }
}
