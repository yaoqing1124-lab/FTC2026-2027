package org.firstinspires.ftc.teamcode._2027.state;

import com.pedropathing.follower.Follower;
import com.skeletonarmy.marrow.weaver.HeadingPolicy;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2027.subsystem.limelight.Limelight;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;

import java.util.ArrayList;

public class Marrow extends StateObject {
    private Point[] points;
    private Point start;
    private Point end;
    private ArrayList<Zone> obstacles = new ArrayList<>();
    private Follower follower;
    public Marrow(Point start, Point...points){
        this.start = start;
        this.points = points;
    }
    public Marrow(Point start, Point end, Point...points){
        this.start = start;
        this.points = points;
        this.end = end;
    }
    @Override
    public void Init(double timestamp) {
        follower = Drivetrain.get_Follower();
        obstacles = null; //TODO
        Weaver.setConfig(new PathConfig()
                .intakeWidth(8.0) //進氣寬度。0停止於目標；實際寬度掃過元素
                .robotWidth(16.0) //沿前軸的機器人足跡
                .robotHeight(16.0) //機器人沿著前軸的足跡
                .clearance(4.0) //與障礙物保持額外距離
                .excludeBlockedTargets(true) //落靶被障礙物阻擋而無法觸及
                .smoothing(true) //圓角設計，運動更流暢
                .turnCostWeight(10.0) //在排序目標時，如何才能最大程度地避免轉彎
                .bruteForceOrderLimit(5)); //最大目標數量精確排序
        PathResult result = Weaver.builder()
                .start(start)
                .targets(Limelight.detector.get().targetPose, e -> new Point(e.getX(), e.getY(), e.getHeading()), HeadingPolicy.LOCKED)
                .obstacles(obstacles)
                .maxTargets(4)
                .end(end)
                .build();
        follower.followPath(PedroPathingConverter.toPath(result));
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
        follower.update();
    }

    @Override
    public boolean IsDone(double timestamp) {
        return false;
    }
}
