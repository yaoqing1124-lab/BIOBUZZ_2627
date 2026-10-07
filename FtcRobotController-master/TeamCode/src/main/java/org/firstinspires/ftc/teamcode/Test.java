package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Shooter.ShooterPose;

@TeleOp
public class Test extends OpMode {
    DcMotor up, down;
    @Override
    public void init() {
        up = hardwareMap.get(DcMotorEx.class, "lf");
        down = hardwareMap.get(DcMotorEx.class, "lb");
    }

    @Override
    public void loop() {
        down.setPower(1);
        up.setPower(gamepad1.circle ? 1 : -1);
    }
}
