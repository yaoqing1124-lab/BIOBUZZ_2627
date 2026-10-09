package org.firstinspires.ftc.teamcode._2627.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._2627.util.HardwareConfig;
import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2627.util.PoseConfig;
import org.firstinspires.ftc.teamcode._2627.util.Team;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;
import org.firstinspires.ftc.teamcode.gc.op.behavior.GenericBehavior;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.StateObject;

public abstract class AutoBase extends OpObject {
    public static Team team;
    public static Pose savePose;
    protected Follower follower;
    protected ElapsedTime timer = new ElapsedTime();
    protected StateMachine stateMachine = new StateMachine();
    protected StateObject entryState;

    // ==================
    // life cycle
    // ==================
    @Override
    public void Init() {
        team = setTeam();
        switch (team) {
            case blue:
                PoseConfig.setBlue();
                break;
            case red:
                PoseConfig.setRed();
                break;
        }
    }

    @Override
    public void Start() {
        entryState = setEntryState();
        timer.reset();
        follower = Drivetrain.get_Follower();
        AddBehavior(stateMachine);
//        RegisterGlobalBehavior(stateMachine);
        setStateChain(entryState);
        stateMachine.Run(entryState);
    }

    @Override
    public void Loop() {
    }

    @Override
    public void Stop() {
        stateMachine = null;
        timer = null;
        savePose = follower.getPose();
        follower.breakFollowing();
        follower = null;
    }


    /**
     * 設定alliance, 直接寫return就好 ex:return Team.Blue;
     */
    abstract Team setTeam();

    /**
     * 設定進入狀態 直接寫return就好
     */
    abstract StateObject setEntryState();

    /**
     * 設定狀態鏈 ex:entryState.Then().Then().....;
     */
    abstract void setStateChain(StateObject entryState);
}
