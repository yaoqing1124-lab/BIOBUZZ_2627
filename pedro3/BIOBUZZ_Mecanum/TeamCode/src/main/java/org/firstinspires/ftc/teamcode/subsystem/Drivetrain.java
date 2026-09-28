package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.z.op.OpBehavior;
import org.firstinspires.ftc.teamcode.z.op.OpObject;

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

    private static boolean isManual = false;
    @Override
    public void Init() {
        m_follower = Constants.create(m_object.hardwareMap);
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        if(isManual){
            m_follower.manual(-m_object.gamepad1.left_stick_x,
                            -m_object.gamepad1.left_stick_y,
                            m_object.gamepad1.right_stick_x);
        }
    }

    @Override
    public void Stop() {
        m_follower = null;
    }
    public static void setManual(){
        isManual = true;
    }
    public boolean toPose(){
        //TODO
        return !m_follower.isBusy();
    }
}
