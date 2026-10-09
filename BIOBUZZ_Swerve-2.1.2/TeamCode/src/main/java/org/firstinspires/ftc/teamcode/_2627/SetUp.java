package org.firstinspires.ftc.teamcode._2627;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class SetUp extends OpMode {
    Servo trig, yaw1, yaw2;
    @Override
    public void init() {
        trig = hardwareMap.get(Servo.class, "trig");
        yaw1 = hardwareMap.get(Servo.class, "yaw1");
        yaw2 = hardwareMap.get(Servo.class, "yaw2");
    }

    @Override
    public void loop() {
        trig.setPosition(0);
        yaw1.setPosition(0);
        yaw2.setPosition(0);
    }
}
