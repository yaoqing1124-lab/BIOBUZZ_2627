package org.firstinspires.ftc.teamcode._2627.util;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain;

public class AutoConfig {
    public static class waitConst {
        public static double shootTime = 0.1;
        public static double takeFlowerTime = 0.5;
    }
    public static class path{
        public static PathChain start = Drivetrain.get_Follower().pathBuilder().build();
    }

}
