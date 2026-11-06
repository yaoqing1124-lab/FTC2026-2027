package org.firstinspires.ftc.teamcode._2027.subsystem.shooter;

/**
 * 3D 空間姿態與砲台角度資料結構 (Shooter Position & Orientation)
 */
public class ShooterPose {
    public double x;
    public double y;
    public double z;
    public double turret;
    public double yaw; // 相容別名，與 turret 保持同步

    public ShooterPose() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.turret = 0;
        this.yaw = 0;
    }

    public ShooterPose(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.turret = 0;
        this.yaw = 0;
    }

    public ShooterPose(double x, double y, double z, double turret) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.turret = turret;
        this.yaw = turret;
    }

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(double x, double y, double z, double turret) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.turret = turret;
        this.yaw = turret;
    }
}
