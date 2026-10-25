package org.firstinspires.ftc.teamcode._2027.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode._2027.opmode.auto.util.AutoBase;
import org.firstinspires.ftc.teamcode._2027.state.FollowPath;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;

//@Disabled
@Autonomous
public class ExampleAuto extends AutoBase {
    @Override //設定聯盟 預設藍方
    public Team setTeam() {
        return Team.none;
    }

    @Override //設定進入狀態
    public StateObject setEntryState() {
        return new FollowPath(null);
    }

    @Override //設定狀態鏈
    public void setStateChain(StateObject entryState) {
//        entryState.Then(new WaitForMillisecond(AutoConfig.waitConst.shootTime), new Shoot(true))
//                .Then(new Shoot(false));
    }
}
