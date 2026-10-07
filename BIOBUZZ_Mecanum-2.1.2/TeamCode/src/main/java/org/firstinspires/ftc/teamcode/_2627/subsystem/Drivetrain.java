package org.firstinspires.ftc.teamcode._2627.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.gc.op.OpBehavior;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;
import org.firstinspires.ftc.teamcode._2627.pedroPathing.Constants;

public class Drivetrain extends OpBehavior {

    private Drivetrain(){}
    private final static Drivetrain m_instance;
    static {
        m_instance = new Drivetrain();
        OpObject.RegisterGlobalBehavior(m_instance);
    }

    private Follower m_follower;

    public static Follower get_Follower(){
        return  m_instance.m_follower;
    }

    @Override
    public void Init() {
        m_follower = Constants.createFollower(m_object.hardwareMap);
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        m_follower.update();
        if (isManual) {
            if(isManual && !m_follower.isTeleopDrive()){
                m_follower.startTeleOpDrive();
            }
            m_follower.setTeleOpDrive(m_object.gamepad1.left_stick_x,
                    m_object.gamepad1.left_stick_y,
                    m_object.gamepad1.right_stick_x,
                    false);
        }else{
            if(!m_follower.isBusy()){
                isManual = true;
            }
        }
    }

    @Override
    public void Stop() {
        m_follower = null;
    }

    public static void toPose(Pose pose){
        if(m_instance.m_follower.isTeleopDrive()) {
            m_instance.isManual = false;
            m_instance.m_follower.followPath(m_instance.m_follower.pathBuilder()
                    .addPath(new BezierLine(m_instance.m_follower.getPose(), pose))
                    .setLinearHeadingInterpolation(m_instance.m_follower.getPose().getHeading(), pose.getHeading())
                    .build());
        }
    }

    private double axial = m_object.gamepad1.left_stick_x;
    private double lateral = m_object.gamepad1.left_stick_y;
    private double yaw =  m_object.gamepad1.right_stick_x;
    private boolean isManual = false;
    public void setManualControl(double axial, double lateral, double yaw, boolean isManual){
        this.isManual = isManual;
        this.axial = axial;
        this.lateral = lateral;
        this.yaw = yaw;
    }
    public void setManualControl(boolean isManual){
        this.isManual = isManual;
    }
    public void setManualControl(){
        isManual = true;
    }
}
