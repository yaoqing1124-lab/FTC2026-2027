package org.firstinspires.ftc.teamcode._2027.opmode.auto.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

public class AutoPose {
    // ========================================================================================================================
    // 場地中心原點 (0, 0)，X: [-72, 72], Y: [-72, 72]
    // 操作手站位 (Driver Station):
    //   - Team.red: 站在場地左側 (X = -72, 面向 +X 方向)
    //   - Team.blue: 站在場地右側 (X = +72, 面向 -X 方向)
    // 全部 Pose 命名以 visualizer 俯視圖位置為基準 (URL: https://visualizer.pedropathing.com/)
    // ========================================================================================================================
    public static class pipe{
        private final static Pose UP    = new Pose(-24, 72,  Math.toRadians(90));
        private final static Pose LEFT  = new Pose(-72, -24, Math.toRadians(180));
        private final static Pose DOWN  = new Pose(24,  -72, Math.toRadians(270));
        private final static Pose RIGHT = new Pose(72,  24,  Math.toRadians(0));

        // 操作者視角映射：
        //   - Blue (站右側面向左): 較遠(UP)為 LEFT，較近(DOWN)為 RIGHT，左邊為 DOWN，右邊為 UP
        //   - Red  (站左側面向右): 較遠(UP)為 RIGHT，較近(DOWN)為 LEFT，左邊為 UP，右邊為 DOWN
        public  final static Pose UP_PIPE    = (OpObject.team == Team.blue) ? pipe.LEFT  : pipe.RIGHT;
        public  final static Pose LEFT_PIPE  = (OpObject.team == Team.blue) ? pipe.DOWN  : pipe.UP;
        public  final static Pose DOWN_PIPE  = (OpObject.team == Team.blue) ? pipe.RIGHT : pipe.LEFT;
        public  final static Pose RIGHT_PIPE = (OpObject.team == Team.blue) ? pipe.UP    : pipe.DOWN;
    }

    public static class shoot{
        // 依據聯盟發射位置：Blue 在 X=12，Red 在 X=-12
        public  final static Pose POSITIVE = (OpObject.team == Team.blue) ? new Pose(12, 48,  Math.toRadians(270)) : new Pose(-12, 48,  Math.toRadians(270));
        public  final static Pose NEGATIVE = (OpObject.team == Team.blue) ? new Pose(12, -48, Math.toRadians(90))  : new Pose(-12, -48, Math.toRadians(90));
    }

    public static class corner {
        private final static Pose UP   = new Pose(72, 72, Math.toRadians(90));
        private final static Pose DOWN = new Pose(-72, -72, Math.toRadians(270));

        // 操作者視角距離：
        //   - Blue (站右側 X=72): CLOSE 為右上角 UP (72, 72)，FAR 為左下角 DOWN (-72, -72)
        //   - Red  (站左側 X=-72): CLOSE 為左下角 DOWN (-72, -72)，FAR 為右上角 UP (72, 72)
        public final static Pose CLOSE = (OpObject.team == Team.blue) ? corner.UP   : corner.DOWN;
        public final static Pose FAR   = (OpObject.team == Team.blue) ? corner.DOWN : corner.UP;
    }

    public static class start{
        // 機器開場起始擺放：場地正中央 (0, 0)，朝向上方 (0 度)
        public final static Pose START = new Pose(0, 0, Math.toRadians(0));
    }

    public static class end{

    }
}
