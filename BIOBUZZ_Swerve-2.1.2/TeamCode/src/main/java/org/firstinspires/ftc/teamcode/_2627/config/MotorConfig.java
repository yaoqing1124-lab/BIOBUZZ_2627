package org.firstinspires.ftc.teamcode._2627.config;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorConfig {
    public String id;
    public DcMotor.Direction direction;
    public MotorConfig(String id, DcMotor.Direction direction){
        this.id = id;
        this.direction = direction;
    }
}
