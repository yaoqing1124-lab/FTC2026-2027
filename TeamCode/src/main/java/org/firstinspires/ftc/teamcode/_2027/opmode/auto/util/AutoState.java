package org.firstinspires.ftc.teamcode._2027.opmode.auto.util;

import com.pedropathing.geometry.BezierCurve;

import org.firstinspires.ftc.teamcode._2027.state.FollowState;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.LogicSwitch;
import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.WaitForMillisecond;


//全部參數設定成static並且會傳StateObject
public class AutoState {
    static boolean isPositive = true; //limelight寫一個回傳boolean的function
    // ========================================================================================================================
    // Drive to pipe
    // UP    : 操作者視角較遠的pipe
    // LEFT  : 操作者視角左邊的pipe
    // DOWN  : 操作者視角較近的pipe
    // RIGHT : 操作者視角右邊的pipe
    // ========================================================================================================================
    public static class toPipe {
//        private static Pose start;
//        public toPipe(Pose start){
//            toPipe.start = start;
//        }
        public static StateObject UP    =  new FollowState(Drivetrain.get_Follower().pathBuilder() //如果不能用FollowState.currentPosee改成調用toPipe.start
                .addPath(new BezierCurve(FollowState.currentPose, AutoPose.pipe.UP_PIPE))
                .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.pipe.UP_PIPE.getHeading())
                .build());
        public static StateObject LEFT  =  new FollowState(Drivetrain.get_Follower().pathBuilder()
                .addPath(new BezierCurve(FollowState.currentPose, AutoPose.pipe.LEFT_PIPE))
                .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.pipe.LEFT_PIPE.getHeading())
                .build());
        public static StateObject DOWN  =  new FollowState(Drivetrain.get_Follower().pathBuilder()
                .addPath(new BezierCurve(FollowState.currentPose, AutoPose.pipe.DOWN_PIPE))
                .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.pipe.DOWN_PIPE.getHeading())
                .build());
        public static StateObject RIGHT =  new FollowState(Drivetrain.get_Follower().pathBuilder()
                .addPath(new BezierCurve(FollowState.currentPose, AutoPose.pipe.RIGHT_PIPE))
                .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.pipe.RIGHT_PIPE.getHeading())
                .build());
    }
    // ========================================================================================================================
    // Drive to corner flower
    // CLOSE : 距離操作者較近的角落flower
    // FAR   : 距離操作者較遠的角落flower
    // ========================================================================================================================
    public static class toCorner{
        public static StateObject CLOSE = new FollowState(Drivetrain.get_Follower().pathBuilder()
                .addPath(new BezierCurve(FollowState.currentPose, AutoPose.corner.CLOSE))
                .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.corner.CLOSE.getHeading())
                .build());
        public static StateObject FAR   = new FollowState(Drivetrain.get_Follower().pathBuilder()
                .addPath(new BezierCurve(FollowState.currentPose, AutoPose.corner.FAR))
                .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.corner.FAR.getHeading())
                .build());
    }
    // ========================================================================================================================
    // Auto wait time
    // ex : StateObject.then(new WaitForMillisecond(shoot), new Shoot(true)).then(new Shoot(false));
    // ========================================================================================================================
    public static class wait{
        public static StateObject shoot      = new WaitForMillisecond(300);//(millisecond)
        public static StateObject intake     = new WaitForMillisecond(500);
        public static StateObject takeFlower = new WaitForMillisecond(1000);
    }
    // ========================================================================================================================
    // Drive to shoot position
    // ex : StateObject.then(AutoState.toPipe.UP).then(new WaitForMillisecond(takeFlower)).then(AutoState.toShoot)
    // ========================================================================================================================
    public static StateObject toShoot = new LogicSwitch(() -> isPositive,
            new FollowState(Drivetrain.get_Follower().pathBuilder()         //判斷要去 y > 0 的那邊(true)
                    .addPath(new BezierCurve(FollowState.currentPose, AutoPose.shoot.POSITIVE))
                    .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.shoot.POSITIVE.getHeading())
                    .build()),
            new FollowState(Drivetrain.get_Follower().pathBuilder()         //判斷要去 y < 0 的那邊(false)
                    .addPath(new BezierCurve(FollowState.currentPose, AutoPose.shoot.NEGATIVE))
                    .setLinearHeadingInterpolation(FollowState.currentPose.getHeading(), AutoPose.shoot.NEGATIVE.getHeading())
                    .build()));
}
