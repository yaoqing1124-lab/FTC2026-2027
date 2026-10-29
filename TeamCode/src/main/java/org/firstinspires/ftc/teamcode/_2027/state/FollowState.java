package org.firstinspires.ftc.teamcode._2027.state;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;


public class FollowState extends StateObject {
    private Follower m_follower;
    private PathChain m_path;
    private Pose m_targetPose;
    public FollowState(PathChain path){
        m_path = path;
    }
    public FollowState(Pose targetPose){
        m_targetPose = targetPose;
    }
    public FollowState(){}
    public static Pose currentPose = null;
    @Override
    public void Init(double timestamp) {
        m_follower = Drivetrain.get_Follower();
        currentPose = m_follower.getPose();
        if (m_path == null && m_targetPose != null) {
            m_path = m_follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, m_targetPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), m_targetPose.getHeading())
                    .build();
        }
        if (m_path != null) {
            m_follower.followPath(m_path);
        }
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
        currentPose = m_follower.getPose();
    }

    @Override
    public boolean IsDone(double timestamp) {
        return !m_follower.isBusy();
    }
}
