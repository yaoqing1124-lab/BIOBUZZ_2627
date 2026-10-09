package org.firstinspires.ftc.teamcode._2627.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._2627.opmode.auto.AutoBase;
import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2627.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2627.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode._2627.util.ShooterConfig;
import org.firstinspires.ftc.teamcode._2627.util.Team;
import org.firstinspires.ftc.teamcode.gc.op.OpObject;

@TeleOp
public class ManualControl extends OpObject {
    boolean isLeft; //目標是不是在操作者左邊
    double axial, lateral, yaw;

    @Override
    public void Init() {
        if (AutoBase.team == Team.none || AutoBase.team == null) {
            team = Team.blue;
        }
        if (AutoBase.savePose == null) {
            AutoBase.savePose = new Pose(0, 0, Math.toRadians(0));
        }
    }

    @Override
    public void Start() {
        Drivetrain.get_Follower().setStartingPose(AutoBase.savePose);
        Drivetrain.get_Follower().startTeleOpDrive();
    }

    @Override
    public void Loop() {
        axial = -gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw = -gamepad1.right_stick_x;
        Drivetrain.get_Follower().setTeleOpDrive(axial, lateral, yaw * 0.65, true);
        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            isLeft = !isLeft;
        }
        switch (team) {
            case blue:
                if (isLeft) Shooter.setTarget(ShooterConfig.BLUE_TARGET_L);
                else Shooter.setTarget(ShooterConfig.BLUE_TARGET_R);
                break;
            case red:
                if (isLeft) Shooter.setTarget(ShooterConfig.RED_TARGET_L);
                else Shooter.setTarget(ShooterConfig.RED_TARGET_R);
                break;
        }

        if (gamepad1.left_bumper) {
            Shooter.setEnable(true);
            Intake.setMode(Intake.Mode.shoot);
        } else {
            Shooter.setEnable(false);
            Intake.setMode(Intake.Mode.intake);
        }
    }

    @Override
    public void Stop() {

    }
}
