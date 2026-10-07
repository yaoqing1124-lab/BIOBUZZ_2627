package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Trigger {
    Servo servo;
    private boolean isEnable;
    public Trigger(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "trigger");
    }
    public void update(){
        if(isEnable){
            servo.setPosition(isEnable ? Config.hardwareConfig.TRIGGER_OPEN_POSITION : Config.hardwareConfig.TRIGGER_CLOSE_POSITION);
        }
    }
}
