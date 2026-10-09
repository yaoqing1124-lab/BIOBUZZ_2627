package org.firstinspires.ftc.teamcode._2627.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode._2627.util.AutoConfig;
import org.firstinspires.ftc.teamcode._2627.state.FollowPath;
import org.firstinspires.ftc.teamcode._2627.state.Shoot;
import org.firstinspires.ftc.teamcode._2627.util.Team;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.StateObject;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.WaitForMillisecond;
@Disabled
@Autonomous
public class ExampleAuto extends AutoBase {
    @Override //設定聯盟 預設藍方
    Team setTeam() {
        return Team.none;
    }

    @Override //設定進入狀態
    StateObject setEntryState() {
        return new FollowPath(AutoConfig.path.start);
    }

    @Override //設定狀態鏈
    void setStateChain(StateObject entryState) {
        entryState.Then(new WaitForMillisecond(AutoConfig.waitConst.shootTime), new Shoot(true))
                .Then(new Shoot(false));
    }
}
