package org.firstinspires.ftc.teamcode._2627.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._2627.config.Config;
import org.firstinspires.ftc.teamcode.gc.op.OpBehavior;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

public class Trigger extends OpBehavior {
    private Servo trig;
    private Trigger(){}
    private final static Trigger m_instance;
    static {
        m_instance = new Trigger();
        OpObject.RegisterGlobalBehavior(m_instance);
    }

    @Override
    public void Init() {
        trig = m_object.hardwareMap.get(Servo.class, Config.Servo.trig.id);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        switch (Intake.getMode()){
            case shoot:
                trig.setPosition(Config.Tuning.TRIG_OPEN_POSITION);
                break;
            case intake:
                trig.setPosition(Config.Tuning.TRIG_CLOSE_POSITION);
                break;
            case brake:
                trig.setPosition(Config.Tuning.TRIG_CLOSE_POSITION);
                break;
        }
    }

    @Override
    public void Stop() {
        trig = null;
    }
}
