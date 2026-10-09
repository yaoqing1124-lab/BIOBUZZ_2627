package org.firstinspires.ftc.teamcode._2627.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._2627.util.HardwareConfig;
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
        trig = m_object.hardwareMap.get(Servo.class, HardwareConfig.Servo.trig.id);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        switch (Intake.getMode()){
            case shoot:
                trig.setPosition(0);
                break;
            case intake:
                trig.setPosition(HardwareConfig.Tuning.TRIG_POSITION);
                break;
            case brake:
                trig.setPosition(HardwareConfig.Tuning.TRIG_POSITION);
                break;
        }
    }

    @Override
    public void Stop() {
        trig = null;
    }
}
