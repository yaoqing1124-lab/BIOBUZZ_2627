package org.firstinspires.ftc.teamcode._2627.subsystem.shooter;

public class ShooterPose {
    public double x;
    public double y;
    public double z;
    public double yaw;

    public ShooterPose() {
        this(0, 0, 0, 0);
    }

    public ShooterPose(double x, double y, double z) {
        this(x, y, z, 0);
    }

    public ShooterPose(double x, double y, double z, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
    }

    // 防禦性複製建構子
    public ShooterPose(ShooterPose other) {
        if (other != null) {
            this.x = other.x;
            this.y = other.y;
            this.z = other.z;
            this.yaw = other.yaw;
        }
    }

    public ShooterPose copy() {
        return new ShooterPose(this);
    }

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(double x, double y, double z, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
    }
}