package org.firstinspires.ftc.teamcode._2027.subsystem.limelight;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;

public class DetectorResult{
    public ArrayList<Pose> targetPose;
    public ArrayList<String> targetName;
    public DetectorResult(ArrayList<Pose> targetPose, ArrayList<String> targetName){
        this.targetName = targetName;
        this.targetPose = targetPose;
    }
}
