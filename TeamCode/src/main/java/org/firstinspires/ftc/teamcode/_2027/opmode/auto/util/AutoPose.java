package org.firstinspires.ftc.teamcode._2027.opmode.auto.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

public class AutoPose {
    // ========================================================================================================================
    // 全部Pose都是以visualizer俯視圖的位置做命名 URL : https://visualizer.pedropathing.com/
    // ========================================================================================================================
    public static class pipe{
        private final static Pose UP    = new Pose(-24, 72,  Math.toRadians(90));
        private final static Pose LEFT  = new Pose(-72, -24, Math.toRadians(180));
        private final static Pose DOWN  = new Pose(24,  -72, Math.toRadians(270));
        private final static Pose RIGHT = new Pose(72,  24,  Math.toRadians(0));
        public  final static Pose UP_PIPE = (OpObject.team == Team.blue) ? pipe.LEFT : pipe.RIGHT;
        public  final static Pose LEFT_PIPE = (OpObject.team == Team.blue) ? pipe.DOWN : pipe.UP;
        public  final static Pose DOWN_PIPE = (OpObject.team == Team.blue) ? pipe.RIGHT : pipe.LEFT;
        public  final static Pose RIGHT_PIPE = (OpObject.team == Team.blue) ? pipe.UP : pipe.DOWN;
    }
    public static class shoot{
        public  final static Pose POSITIVE = (OpObject.team == Team.blue) ? new Pose(12, 48,  Math.toRadians(270)) : new Pose(-12, 48,  Math.toRadians(270));
        public  final static Pose NEGATIVE = (OpObject.team == Team.blue) ? new Pose(12, -48, Math.toRadians(90))  : new Pose(-12, -48, Math.toRadians(90));
    }
    public static class corner {
        private final static Pose UP = new Pose(72, 72, Math.toRadians(90));
        private final static Pose DOWN = new Pose(-72, -72, Math.toRadians(270));
        public final static Pose CLOSE = (OpObject.team == Team.blue) ? corner.UP : corner.DOWN;
        public final static Pose FAR = (OpObject.team == Team.blue) ? corner.DOWN : corner.UP;
    }
    public static class start{

    }
    public static class end{

    }
}
