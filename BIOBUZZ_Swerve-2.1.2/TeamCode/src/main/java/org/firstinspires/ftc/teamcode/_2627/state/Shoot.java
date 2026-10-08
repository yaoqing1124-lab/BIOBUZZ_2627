package org.firstinspires.ftc.teamcode._2627.state;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._2627.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.StateObject;

public class Shoot extends StateObject {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void Init(double timestamp) {
        timer.reset();
        Shooter.setEnable(true);
        if(timer.milliseconds() >= AutoConfig.Tuning.shootTime){
            Shooter.setEnable(false);
        }
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {

    }

    @Override
    public boolean IsDone(double timestamp) {
        return false;
    }
}
