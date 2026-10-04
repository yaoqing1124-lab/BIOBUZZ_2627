package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterPose;

@TeleOp
public class Test extends OpMode {
    Shooter shooter;
    Intake intake;
    Trigger trigger;

    @Override
    public void init() {
        shooter = new Shooter(new ShooterPose());
        intake = new Intake();
        trigger = new Trigger();
    }

    @Override
    public void loop() {
        shooter.update();
        intake.update();
        trigger.update();
    }
}
