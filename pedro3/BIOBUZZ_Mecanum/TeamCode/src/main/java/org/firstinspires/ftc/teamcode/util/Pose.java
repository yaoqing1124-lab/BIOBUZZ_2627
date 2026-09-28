package org.firstinspires.ftc.teamcode.util;

public class Pose {
    final Pose midPose = new Pose(72, 72, 0);
    double x,y,z;
    public Pose(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public Pose setInvert(){
        double InvertX = 2 * midPose.getX() - x;
        double InvertY = 2 * midPose.getY() - y;
        return new Pose(InvertX, InvertY,z);
    }
    public double getX(){return x;}
    public double getY(){return y;}
    public double getZ(){return z;}
}
