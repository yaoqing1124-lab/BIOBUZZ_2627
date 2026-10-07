package org.firstinspires.ftc.teamcode._2627.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Config {
    public static class Motor{
        public static MotorConfig lfm = new MotorConfig("lfm", DcMotor.Direction.FORWARD);
        public static MotorConfig lbm = new MotorConfig("lbm", DcMotor.Direction.FORWARD);
        public static MotorConfig rfm = new MotorConfig("rfm", DcMotor.Direction.FORWARD);
        public static MotorConfig rbm = new MotorConfig("rbm", DcMotor.Direction.FORWARD);
        public static MotorConfig flyWheel = new MotorConfig("flywheel", DcMotorSimple.Direction.FORWARD);
        public static MotorConfig intake = new MotorConfig("intake", DcMotorSimple.Direction.FORWARD);
        public static MotorConfig intakeCtrl = new MotorConfig("intakeCtrl", DcMotorSimple.Direction.FORWARD);
    }
    public static class Servo{
        public static ServoConfig lfs = new ServoConfig("lfs");
        public static ServoConfig lbs = new ServoConfig("lbs");
        public static ServoConfig rfs = new ServoConfig("rfs");
        public static ServoConfig rbs = new ServoConfig("rbs");
        public static ServoConfig trig = new ServoConfig("trig");
        public static ServoConfig yaw1 = new ServoConfig("yaw1", 0);
        public static ServoConfig yaw2 = new ServoConfig("yaw2", 0);
    }
    public static class Sensor{
        public static PinpointConfig pinpoint = new PinpointConfig("pinpoint", GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD, 0,0);
    }
    public static class Tuning{
        public final static double intakePower = 1;
        public final static double TRIG_OPEN_POSITION = 1;
        public final static double TRIG_CLOSE_POSITION = 0;
    }
}
