package org.firstinspires.ftc.teamcode._2627.state;

import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.gc.op.behavior.statemachine.StateObject;

public class Shoot extends StateObject {
    private boolean isShoot;
    public Shoot(boolean isShoot){
        this.isShoot = isShoot;
    }
    @Override
    public void Init(double timestamp) {
        Shooter.setEnable(isShoot);
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
    }

    @Override
    public boolean IsDone(double timestamp) {
        return true;
    }
}
