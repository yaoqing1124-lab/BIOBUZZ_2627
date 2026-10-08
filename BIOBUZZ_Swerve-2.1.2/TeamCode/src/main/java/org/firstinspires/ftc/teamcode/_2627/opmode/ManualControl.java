package org.firstinspires.ftc.teamcode._2627.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._2627.config.Config;
import org.firstinspires.ftc.teamcode._2627.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

@TeleOp
public class ManualControl extends OpObject {
    boolean isLeft;

    @Override
    public void Init() {
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        if(gamepad1.circle || gamepad2.circle){
            isLeft = !isLeft;
        }
        switch (team){
            case blue:
                if(isLeft) Shooter.setTarget(Config.ShooterConfig.BLUE_TARGET_L);
                else       Shooter.setTarget(Config.ShooterConfig.BLUE_TARGET_R);
                break;
            case red:
                if(isLeft) Shooter.setTarget(Config.ShooterConfig.RED_TARGET_L);
                else       Shooter.setTarget(Config.ShooterConfig.RED_TARGET_R);
                break;
        }

        if(gamepad1.left_bumper){
            Intake.setMode(Intake.Mode.shoot);
        }else{
            Intake.setMode(Intake.Mode.intake);
        }
    }

    @Override
    public void Stop() {

    }
}
