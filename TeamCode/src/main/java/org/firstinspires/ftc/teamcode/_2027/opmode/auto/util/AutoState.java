package org.firstinspires.ftc.teamcode._2027.opmode.auto.util;

import com.pedropathing.geometry.BezierCurve;

import org.firstinspires.ftc.teamcode._2027.state.FollowState;
import org.firstinspires.ftc.teamcode._2027.subsystem.limelight.Limelight;
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
        public static StateObject UP()   { return new FollowState(AutoPose.pipe.UP_PIPE);}
        public static StateObject LEFT() { return new FollowState(AutoPose.pipe.LEFT_PIPE);}
        public static StateObject DOWN() { return new FollowState(AutoPose.pipe.DOWN_PIPE);}
        public static StateObject RIGHT(){ return new FollowState(AutoPose.pipe.RIGHT_PIPE);}
    }
    // ========================================================================================================================
    // Drive to corner flower
    // CLOSE : 距離操作者較近的角落flower
    // FAR   : 距離操作者較遠的角落flower
    // ========================================================================================================================
    public static class toCorner{
        public static StateObject CLOSE() {return new FollowState(AutoPose.corner.CLOSE);}
        public static StateObject FAR()   {return new FollowState(AutoPose.corner.FAR);}
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
    public static StateObject toShoot(){ return new LogicSwitch(() -> Limelight.detector.get().targetName.equals("Positive"),
            new FollowState(AutoPose.shoot.POSITIVE),
            new FollowState(AutoPose.shoot.NEGATIVE));
    }
}
