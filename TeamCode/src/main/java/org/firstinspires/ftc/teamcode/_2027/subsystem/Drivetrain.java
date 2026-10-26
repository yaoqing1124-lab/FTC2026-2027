package org.firstinspires.ftc.teamcode._2027.subsystem;

import com.pedropathing.follower.Follower;


import org.firstinspires.ftc.teamcode._2027.Config;
import org.firstinspires.ftc.teamcode._2027.util.DriveType;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode._2027.pedroPathing.Constants;

public class Drivetrain extends OpBehavior {
    private Drivetrain(){}
    private final static Drivetrain s_instance;
    static {
        Constants.setDrive(Config.driveMode);
        s_instance = new Drivetrain();
        OpObject.RegisterGlobalBehavior(s_instance);
    }

    private Follower m_follower;

    public static Follower get_Follower(){
        return  s_instance.m_follower;
    }

    @Override
    public void Init() {
        m_follower = Constants.createFollower(m_object.hardwareMap);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        if (m_follower != null) m_follower.update();
    }

    @Override
    public void Stop() {
        m_follower = null;
    }
}