package org.firstinspires.ftc.teamcode._2027.opmode.auto.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2027.subsystem.limelight.Limelight;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;


public abstract class AutoBase extends OpObject {
    public static Pose savePose;
    protected Follower follower;
    protected ElapsedTime timer = new ElapsedTime();
    protected StateMachine stateMachine = new StateMachine();
    protected StateObject entryState;

    // ==================
    // life cycle
    // ==================
    @Override
    public void Init() {
        OpObject.team = setTeam();
        switch (team) {
            case blue:
                break;
            case red:
                break;
        }
    }

    @Override
    public void Start() {
        entryState = setEntryState();
        timer.reset();
        follower = Drivetrain.get_Follower();
        AddBehavior(stateMachine);
//        RegisterGlobalBehavior(stateMachine);
        setStateChain(entryState);
        stateMachine.Run(entryState);
    }

    @Override
    public void Loop() {
    }

    @Override
    public void Stop() {
        stateMachine = null;
        timer = null;
        savePose = follower.getPose();
        follower.breakFollowing();
        follower = null;
    }


    /**
     * 設定alliance, 直接寫return就好 ex:return Team.Blue;
     */
    public abstract Team setTeam();

    /**
     * 設定進入狀態 直接寫return就好
     */
    public abstract StateObject setEntryState();

    /**
     * 設定狀態鏈 ex:entryState.Then().Then().....;
     */
    public abstract void setStateChain(StateObject entryState);
}
