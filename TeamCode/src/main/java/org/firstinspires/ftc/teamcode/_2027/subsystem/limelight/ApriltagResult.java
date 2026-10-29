package org.firstinspires.ftc.teamcode._2027.subsystem.limelight;

import com.pedropathing.geometry.Pose;

public class ApriltagResult {
    public double Tx;
    public double Ty;
    public double Ta;
    public Pose Pose;
    public ApriltagResult(double Tx, double Ty, double Ta, Pose Pose){
        this.Tx   = Tx;
        this.Ty   = Ty;
        this.Ta   = Ta;
        this.Pose = Pose;
    }
}
