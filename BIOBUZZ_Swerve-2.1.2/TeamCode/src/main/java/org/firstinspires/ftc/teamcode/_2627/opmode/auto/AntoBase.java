package org.firstinspires.ftc.teamcode._2627.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2627.util.Team;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.StateMachine;

public abstract class AntoBase extends OpObject {
    public static Team team;
    public static Pose savePose;
    protected StateMachine stateMachine = new StateMachine();
    protected Follower follower;
    protected ElapsedTime timer = new ElapsedTime();
    public abstract void setTeam();
    @Override
    public void Init(){
        auto_init();
    }
    @Override
    public void Start(){
        timer.reset();
        follower = Drivetrain.get_Follower();
        AutoConfig.setTeam(team);
        RegisterGlobalBehavior(stateMachine);
        auto_start();
    }
    @Override
    public void Loop(){
        auto_loop();
    }
    @Override
    public void Stop(){
        auto_stop();
        stateMachine = null;
        timer = null;
        savePose = follower.getPose();
        follower.breakFollowing();
        follower = null;
    }
    abstract void auto_init();
    abstract void auto_start();
    abstract void auto_loop();
    abstract void auto_stop();
}
