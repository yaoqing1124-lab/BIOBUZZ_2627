package org.firstinspires.ftc.teamcode._2627.config;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoConfig {
    public String id;
    public double offset;
    public ServoConfig(String id){
        this.id = id;
    }
    public ServoConfig(String id, double offset){
        this.id = id;
        this.offset = offset;
    }

}
