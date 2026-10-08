package org.firstinspires.ftc.teamcode._2627.opmode.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode._2627.util.Team;

public class AutoConfig {
    public static class Tuning {
        public static double shootTime;
    }

    public static Pose startPose;

    public static void setBlue() {
        startPose = new Pose(0, 0, Math.toRadians(0));
    }

    public static void setRed(){

    }

    public static void setTeam(Team team) {
        switch (team){
            case blue:
                setBlue();
                break;
            case red:
                setRed();
                break;
        }
    }
}
