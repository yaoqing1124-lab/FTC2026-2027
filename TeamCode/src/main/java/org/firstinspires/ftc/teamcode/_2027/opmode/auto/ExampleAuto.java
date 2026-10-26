package org.firstinspires.ftc.teamcode._2027.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode._2027.opmode.auto.util.AutoBase;
import org.firstinspires.ftc.teamcode._2027.opmode.auto.util.AutoState;
import org.firstinspires.ftc.teamcode._2027.state.FollowState;
import org.firstinspires.ftc.teamcode._2027.state.Shoot;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.WaitForMillisecond;

@Disabled
@Autonomous
public class ExampleAuto extends AutoBase {
    boolean isPositive = true;
    @Override //設定聯盟 預設藍方
    public Team setTeam() {
        return Team.red;
    }

    @Override //設定進入狀態
    public StateObject setEntryState() {
        return new FollowState(null);
    }

    @Override //設定狀態鏈
    public void setStateChain(StateObject entryState) {
        entryState.Then(new WaitForMillisecond(0), new Shoot(true))  //先執行entryState(FollowPath)然後射球直到wait設定的時間到
                .Then(new Shoot(false)) //關閉shoot
                .Then(new FollowState(null))  //跟隨路徑
                .Then(AutoState.toPipe.UP);
    }
}
