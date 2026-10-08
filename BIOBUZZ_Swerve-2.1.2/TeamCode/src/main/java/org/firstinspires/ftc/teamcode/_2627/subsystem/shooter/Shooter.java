package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._2627.config.Config;
import org.firstinspires.ftc.teamcode._2627.config.MotorConfig;
import org.firstinspires.ftc.teamcode._2627.config.ServoConfig;
import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.gc.op.OpBehavior;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

public class Shooter extends OpBehavior {
    private MotorConfig m_flywheelInfo;
    private ServoConfig m_yawInfo;
    private DcMotorEx flywheel;
    private Servo yaw;
    private Shooter(MotorConfig motorInfo, ServoConfig servoInfo, ShooterPose relativePose) {
        m_flywheelInfo = motorInfo;
        m_yawInfo = servoInfo;
        ShooterPose = relativePose;
    }
    private final static Shooter m_instanceR;
    //    private static final Shooter m_instanceL;
    static {
        m_instanceR = new Shooter(Config.Motor.flyWheel, Config.Servo.yaw1, Config.ShooterConfig.relativeFlyWheel);
//        m_instanceL = new Shooter(motorInfo, servoInfo);
        OpObject.RegisterGlobalBehavior(m_instanceR);
//        OpObject.RegisterGlobalBehavior(m_instanceL);
    }

    private static boolean isEnable = false;
    private static boolean isEnablePredict = false;
    public static void setEnable(boolean isEnable){Shooter.isEnable = isEnable;}
    public static void setEnablePredict(boolean isEnablePredict){Shooter.isEnablePredict = isEnable;}
    @Override
    public void Init() {
        flywheel = m_object.hardwareMap.get(DcMotorEx.class, m_flywheelInfo.id);
        flywheel.setDirection(m_flywheelInfo.direction);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPower(0);

        yaw = m_object.hardwareMap.get(Servo.class, m_yawInfo.id);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        updateFlyWheel();
        updateYaw();
    }

    @Override
    public void Stop() {
        flywheel = null;
        yaw = null;
    }


    /**
     * 更新航向角
     */
    private double degree2position(double degree){
        //TODO
        return 0;
    }
    public void updateYaw(){
        yaw.setPosition(degree2position(ShooterCalculator.calculateYaw(ShooterPose, targetPose)));
    }

    /**
     * 更新轉速
     */
    double RPM;
    public void updateFlyWheel(){
        RPM = ShooterCalculator.RPM2ticksPerSec(ShooterCalculator.calculateVelocity(ShooterPose, targetPose));
        flywheel.setVelocity(RPM);
    }
    private static ShooterPose ShooterPose;
    private static ShooterPose targetPose = new ShooterPose();
    public static void setTarget(ShooterPose targetPose){
        double robotXvelocity = Drivetrain.get_Follower().getVelocity().getXComponent();
        double robotYvelocity = Drivetrain.get_Follower().getVelocity().getYComponent();
        if(isEnablePredict){
            //calculate virtual target
            targetPose.set(targetPose.x - robotXvelocity * ShooterCalculator.calculateFlyTime(ShooterPose, targetPose),
                           targetPose.y - robotYvelocity * ShooterCalculator.calculateFlyTime(ShooterPose, targetPose),
                              targetPose.z);
        }
        Shooter.targetPose = targetPose;
    }

    /**
     * 設定發射器在場地上的位置
     * 機器座標 + 旋轉矩陣(yaw偏移) * 發射器offset
     */
    private void setShooterPose(){
        double robotX = Drivetrain.get_Follower().getPose().getX();
        double robotY = Drivetrain.get_Follower().getPose().getY();
        double robotHeading = Drivetrain.get_Follower().getHeading();
        double relativeX = Config.ShooterConfig.relativeFlyWheel.x;
        double relativeY = Config.ShooterConfig.relativeFlyWheel.y;
        ShooterPose.set(robotX + relativeX * Math.cos(robotHeading) - relativeY * Math.sin(robotHeading),
                        robotY + relativeX * Math.sin(robotHeading) + relativeY * Math.cos(robotHeading),
                           Config.ShooterConfig.SHOOTER_HEIGHT);
    }
}
