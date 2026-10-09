package org.firstinspires.ftc.teamcode._2627.subsystem;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode._2627.util.DriveConstants;
import org.firstinspires.ftc.teamcode.gc.op.OpBehavior;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

public class Drivetrain extends OpBehavior {
    private Drivetrain(){}
    private final static Drivetrain s_instance;
    static {
        s_instance = new Drivetrain();
        OpObject.RegisterGlobalBehavior(s_instance);
    }

    private Follower m_follower;

    public static Follower get_Follower(){
        return  s_instance.m_follower;
    }

    @Override
    public void Init() {
        m_follower = DriveConstants.createFollower(m_object.hardwareMap);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        if (m_follower != null) m_follower.update();
    }

    @Override
    public void Stop() {
        m_follower = null;
    }
}