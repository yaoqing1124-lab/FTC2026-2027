//package org.firstinspires.ftc.teamcode._2027.state;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.skeletonarmy.marrow.weaver.HeadingPolicy;
//import com.skeletonarmy.marrow.weaver.PathConfig;
//import com.skeletonarmy.marrow.weaver.PathResult;
//import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
//import com.skeletonarmy.marrow.weaver.Weaver;
//import com.skeletonarmy.marrow.zones.CircleZone;
//import com.skeletonarmy.marrow.zones.Point;
//import com.skeletonarmy.marrow.zones.PolygonZone;
//
//import org.firstinspires.ftc.teamcode._2027.Config;
//import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
//import org.firstinspires.ftc.teamcode._2027.subsystem.Limelight;
//import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;
//
//public class Marrow extends StateObject {
//    private Follower m_follower;
//    private Pose currentPose;
//    private Double endHeading = null;
//    public Marrow(Pose currentPose, Double endHeading){
//        this.currentPose = currentPose;
//        this.endHeading = endHeading;
//
//    }
//    public Marrow(Pose currentPose){
//        this.currentPose = currentPose;
//
//    }
//    @Override
//    public void Init(double timestamp) {
//        m_follower = Drivetrain.get_Follower();
//
//        Weaver.setConfig(new PathConfig()
//                .intakeWidth(Config.marrow.INTAKE_WIDTH)
//                .robotWidth(Config.marrow.ROBOT_WIDTH)
//                .robotHeight(Config.marrow.ROBOT_HEIGHT));
//        PathResult result = Weaver.builder()
//                .start(new Point(currentPose.getX(), currentPose.getY(), currentPose.getHeading()))
//                .addObstacle(new PolygonZone(new Point( 24, 0), 2, 39))
//                .addObstacle(new PolygonZone(new Point(-24, 0), 2, 39))
//                .targets(Limelight.getGameElements(), e->new Point(e.getX(), e.getY()))
//                .end(new Point(0,0), endHeading == null ? HeadingPolicy.FREE : HeadingPolicy.LOCKED)
//                .maxTargets(4)
//                .build();
//
//
//
//        m_follower.followPath(PedroPathingConverter.toPath(result));
//    }
//
//    @Override
//    public void Loop(double timestamp, double deltaTime) {
//        m_follower.update();
//    }
//
//    @Override
//    public boolean IsDone(double timestamp) {
//        return !m_follower.isBusy();
//    }
//}
