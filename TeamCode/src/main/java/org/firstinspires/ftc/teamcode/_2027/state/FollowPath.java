package org.firstinspires.ftc.teamcode._2027.state;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;


public class FollowPath extends StateObject {
    private Follower m_follower;
    private final PathChain m_path;
    public FollowPath(PathChain path){
        m_path = path;
    }
    @Override
    public void Init(double timestamp) {
        m_follower = Drivetrain.get_Follower();
        m_follower.followPath(m_path);
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
    }

    @Override
    public boolean IsDone(double timestamp) {
        return !m_follower.isBusy();
    }
}
