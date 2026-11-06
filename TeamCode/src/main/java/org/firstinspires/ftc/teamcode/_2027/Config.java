package org.firstinspires.ftc.teamcode._2027;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode._2027.util.AnalogConfig;
import org.firstinspires.ftc.teamcode._2027.util.DriveType;
import org.firstinspires.ftc.teamcode._2027.util.LimelightConfig;
import org.firstinspires.ftc.teamcode._2027.util.MotorConfig;
import org.firstinspires.ftc.teamcode._2027.util.ServoConfig;
import org.firstinspires.ftc.teamcode._2027.util.Team;

/**
 * ====================================================================================================
 * 2026-2027 賽季全域機器人硬體與調適參數配置表 (Master Configuration)
 * 支援 FTC Dashboard / Bylazar Fullpanels 即時連線線上調適 (Live Tuning)。
 * ====================================================================================================
 */
@Configurable
public class Config {

    // +==============================================================================================+
    // | 1. 聯盟與底盤驅動模式 (Alliance & Drive Configuration)                                      |
    // +==============================================================================================+
    /** 目前聯盟顏色 (Team.blue 或 Team.red，自走開場會自動覆寫) */
    public static Team team = Team.none;

    /** 底盤驅動模式 (DriveType.Swerve 或 DriveType.Mecanum) */
    public final static DriveType driveMode = DriveType.Swerve;


    // +==============================================================================================+
    // | 2. 硬體連接定義區 (Hardware Devices & Mapping)                                              |
    // +==============================================================================================+
    public static class hardware {

        /** 馬達 (Motors) - 填入 RC 設定的名稱 (ID) 與旋轉方向 */
        public static class motor {
            // 底盤輪馬達
            public final static MotorConfig lfm        = new MotorConfig("lfm",        DcMotor.Direction.FORWARD);
            public final static MotorConfig lbm        = new MotorConfig("lbm",        DcMotor.Direction.FORWARD);
            public final static MotorConfig rfm        = new MotorConfig("rfm",        DcMotor.Direction.REVERSE);
            public final static MotorConfig rbm        = new MotorConfig("rbm",        DcMotor.Direction.REVERSE);

            // 進球輸送機構馬達
            public final static MotorConfig intake     = new MotorConfig("intake",     DcMotorSimple.Direction.FORWARD);
            public final static MotorConfig intakeCtrl = new MotorConfig("intakeCtrl", DcMotorSimple.Direction.REVERSE);

            // 發射器 1 (Small) 飛輪馬達
            public final static MotorConfig flyWheel1    = new MotorConfig("flywheel",   DcMotorSimple.Direction.REVERSE);
            public final static MotorConfig flyWheel     = flyWheel1; // 相容別名
            public final static MotorConfig flyWheelSmall= flyWheel1;

            // 發射器 2 (Big) 飛輪馬達
            public final static MotorConfig flyWheel2    = new MotorConfig("flywheel2",  DcMotorSimple.Direction.REVERSE);
            public final static MotorConfig flyWheelBig  = flyWheel2;
        }

        /** 伺服舵機 (Servos) - 填入 RC 設定的名稱 (ID) */
        public static class servo {
            // 底盤轉向舵機 (Swerve Pods)
            public final static ServoConfig lfs   = new ServoConfig("lfs");
            public final static ServoConfig lbs   = new ServoConfig("lbs");
            public final static ServoConfig rfs   = new ServoConfig("rfs");
            public final static ServoConfig rbs   = new ServoConfig("rbs");

            // 發射器 1 (Small) 伺服
            public final static ServoConfig turret1     = new ServoConfig("turret");
            public final static ServoConfig pitch1      = new ServoConfig("pitch");
            public final static ServoConfig turret      = turret1;   // 相容別名
            public final static ServoConfig pitch       = pitch1;    // 相容別名
            public final static ServoConfig turretSmall = turret1;
            public final static ServoConfig pitchSmall  = pitch1;
            public final static ServoConfig yaw1        = turret1;   // 舊命名相容別名
            public final static ServoConfig yaw         = turret1;
            public final static ServoConfig yawSmall    = turret1;

            // 發射器 2 (Big) 伺服
            public final static ServoConfig turret2     = new ServoConfig("turret2");
            public final static ServoConfig pitch2      = new ServoConfig("pitch2");
            public final static ServoConfig turretBig   = turret2;
            public final static ServoConfig pitchBig    = pitch2;
            public final static ServoConfig yaw2        = turret2;   // 舊命名相容別名
            public final static ServoConfig yawBig      = turret2;
        }

        /** 感測器 (Sensors) - 填入 RC 設定的名稱 (ID) */
        public static class sensor {
            public final static AnalogConfig    lfenc     = new AnalogConfig("lfenc");
            public final static AnalogConfig    lbenc     = new AnalogConfig("lbenc");
            public final static AnalogConfig    rfenc     = new AnalogConfig("rfenc");
            public final static AnalogConfig    rbenc     = new AnalogConfig("rbenc");
            public final static LimelightConfig limelight = new LimelightConfig("limelight");
        }
    }


    // +==============================================================================================+
    // | 3. 定位輪與里程計參數 (Odometry / Pinpoint Configuration)                                    |
    // +==============================================================================================+
    public static class odometry {
        public final static String id            = "pinpoint"; // GoBilda Pinpoint 裝置名稱
        public final static double forwardOffset = 60.6;       // 前後定位輪相對於旋轉中心偏移量 (mm)
        public final static double strafeOffset  = -15.5;      // 橫向定位輪相對於旋轉中心偏移量 (mm)
        public final static GoBildaPinpointDriver.EncoderDirection forwardDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public final static GoBildaPinpointDriver.EncoderDirection strafeDirection  = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }


    // +==============================================================================================+
    // | 4. 機構動力輸出設定 (Mechanism Power)                                                         |
    // +==============================================================================================+
    public static class value {
        /** 進球吸球時的馬達功率 (0.0 ~ 1.0) */
        public static double INTAKE_POWER       = 1.0;

        /** 射擊時進球馬達推送球進入飛輪的功率 (0.0 ~ 1.0) */
        public static double INTAKE_SHOOT_POWER = 1.0;
    }


    // +==============================================================================================+
    // | 5. 場地 Hive 目標座標 (中心為原點 (0, 0)，X, Y 範圍 [-72, 72] 吋)                             |
    // +==============================================================================================+
    public static class field {
        public static class targetPose {
            /** 紅方上方 Hive 目標座標 (X < 0, Y > 0) */
            public static final Pose redPositive  = new Pose(-12.9,  19.89);

            /** 紅方下方 Hive 目標座標 (X < 0, Y < 0) */
            public static final Pose redNegative  = new Pose(-12.9, -19.89);

            /** 藍方上方 Hive 目標座標 (X > 0, Y > 0) */
            public static final Pose bluePositive = new Pose( 12.9,  19.89);

            /** 藍方下方 Hive 目標座標 (X > 0, Y < 0) */
            public static final Pose blueNegative = new Pose( 12.9, -19.89);
        }
    }


    // +==============================================================================================+
    // | 6. 視覺與相機參數 (Vision & LimeLight Pipeline)                                              |
    // +==============================================================================================+
    public static class LLvalue {
        public static class pipeline {
            public static final int apriltag = 0; // AprilTag 辨識管道 ID
            public static final int detector = 1; // 辨識球體管道 ID
            public static final int python   = 2; // 自訂 Python 處理管道 ID
        }
        public static final double DETECTOR_TOLERANCE_MILLISECOND = 100;
    }


    // +==============================================================================================+
    // | 7. 雙發射器調適專區 (Dual Shooter Tuning Parameters)                                         |
    // | 【在此直接填入實測量得的數值即可】                                                            |
    // +==============================================================================================+
    public static class shooter {

        /**
         * ============================================================================
         * 發射器 1 (Small Shooter / Shooter 1) 調適參數
         * ============================================================================
         */
        public static class small {

            // ------------------------------------------------------------------------
            // 7.1.1 雙參數擬合核心 (近距離與遠距離實測求出)
            // ------------------------------------------------------------------------
            /** 能量轉換率：飛輪線速度轉球體出膛初速效率 (通常 0.70 ~ 0.90，近距離實測填入) */
            public static double ENERGY_CONVERSION_RATE = 0.8472;

            /** 能量損失率：補償遠距離空氣阻力與Magnus效應下墜 (通常 0.0001 ~ 0.0004，遠距離實測填入) */
            public static double ENERGY_LOSS_RATE       = 0.0002;

            /** 進球輸送機構推球進發射器的初始線速度 (in/s，通常 0.0 ~ 10.0) */
            public static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 3.0;

            // ------------------------------------------------------------------------
            // 7.1.2 瞄準幾何與反饋控制
            // ------------------------------------------------------------------------
            /** 理想入框入射角度 (度，通常 55.0 ~ 65.0 度) */
            public static double IDEAL_INCIDENT_ANGLE_DEG = 60.0;

            /** 判定轉速穩定閾值 (RPM)：|targetRPM - actualRPM| 小於此值啟用閉迴路仰角微調 */
            public static double RPM_ERROR_THRESHOLD     = 150.0;

            /** 仰角角度手動微調偏移 (度，正負皆可，現場快速修正射角) */
            public static double PITCH_OFFSET_DEG         = 0.0;

            /** 發射器出球口相對於機器底盤旋轉中心的前後偏移量 (in，向前為正) */
            public static double TURRET_FORWARD_IN        = 0.0;

            /** 發射器出球口相對於機器底盤旋轉中心的左右偏移量 (in，向左為正) */
            public static double TURRET_LEFT_IN           = 0.0;

            // ------------------------------------------------------------------------
            // 7.1.3 場地物理常數
            // ------------------------------------------------------------------------
            /** 場地重力加速度 (in/s^2，預設 386.09) */
            public static double G = 386.09;

            /** 出球口與籃框進球點的垂直高度差 (in，籃框高度 53.5 吋減發射器高度) */
            public static double H = 53.5;

            /** 彈道掠過籃框前緣防撞矮牆的額外垂直安全淨空裕度 (in) */
            public static double FRONT_WALL_CLEARANCE_IN = 1.0;

            /** 飛輪機構硬體極限所能輸出的最高安全發射初速限制 (in/s) */
            public static double MAX_VELOCITY_IN_PER_SEC = 512.0;

            // ------------------------------------------------------------------------
            // 7.1.4 舵機硬體零點與線性公式
            // ------------------------------------------------------------------------
            /** 砲台正面向前 (0°) 時的伺服舵機 Position (0.0 ~ 1.0) */
            public static double TURRET_CENTER     = 0.5;

            /** 砲台舵機每單位 Position 變化的旋轉角度 (度/Pos) */
            public static double TURRET_DEG_PER_POS = 0.00395;

            /** 舊命名相容別名 */
            public static double YAW_CENTER = TURRET_CENTER;
            public static double YAW_DEG_PER_POS = TURRET_DEG_PER_POS;

            /** 仰角伺服最小物理限制角度 (度) */
            public static double MIN_PITCH_ANGLE_DEG = 30.0;

            /** 仰角伺服最大物理限制角度 (度) */
            public static double MAX_PITCH_ANGLE_DEG = 54.0;

            /** 仰角轉舵機位置之線性公式斜率 M (pos = M * deg + C) */
            public static double PITCH_SERVO_M = -0.0383333;

            /** 仰角轉舵機位置之線性公式截距 C (pos = M * deg + C) */
            public static double PITCH_SERVO_C = 2.15;

            /** 仰角舵機輸出最小 Position 保護值 (防止卡底盤) */
            public static double PITCH_SERVO_MIN = 0.08;

            /** 仰角舵機輸出最大 Position 保護值 (防止脫軌) */
            public static double PITCH_SERVO_MAX = 1.0;

            // ------------------------------------------------------------------------
            // 7.1.5 馬達物理規格與閉迴路 PIDF
            // ------------------------------------------------------------------------
            /** 馬達單圈編碼器 Ticks 數 (GoBilda 5202 裸馬達 = 28.0) */
            public static double TICKS_PER_REVOLUTION = 28.0;

            /** 馬達軸到飛輪軸的傳動齒輪/皮帶速比 (直驅 = 1.0) */
            public static double GEAR_RATIO = 1.0;

            /** 飛輪外緣射彈接觸面實際測量半徑 (in) */
            public static double FLYWHEEL_RADIUS_IN = 2.0;

            /** 飛輪速度閉迴路控制 PIDF 係數 (P, I, D, F) */
            public static PIDFCoefficients flywheelPID = new PIDFCoefficients(1.0, 0.0, 0.0, 0.0);
        }

        /**
         * ============================================================================
         * 發射器 2 (Big Shooter / Shooter 2) 調適參數
         * ============================================================================
         */
        public static class big {

            // ------------------------------------------------------------------------
            // 7.2.1 雙參數擬合核心 (近距離與遠距離實測求出)
            // ------------------------------------------------------------------------
            /** 能量轉換率：飛輪線速度轉球體出膛初速效率 (通常 0.70 ~ 0.90，近距離實測填入) */
            public static double ENERGY_CONVERSION_RATE = 0.8472;

            /** 能量損失率：補償遠距離空氣阻力與Magnus效應下墜 (通常 0.0001 ~ 0.0004，遠距離實測填入) */
            public static double ENERGY_LOSS_RATE       = 0.0002;

            /** 進球輸送機構推球進發射器的初始線速度 (in/s，通常 0.0 ~ 10.0) */
            public static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 3.0;

            // ------------------------------------------------------------------------
            // 7.2.2 瞄準幾何與反饋控制
            // ------------------------------------------------------------------------
            /** 理想入框入射角度 (度，通常 55.0 ~ 65.0 度) */
            public static double IDEAL_INCIDENT_ANGLE_DEG = 60.0;

            /** 判定轉速穩定閾值 (RPM)：|targetRPM - actualRPM| 小於此值啟用閉迴路仰角微調 */
            public static double RPM_ERROR_THRESHOLD     = 150.0;

            /** 仰角角度手動微調偏移 (度，正負皆可，現場快速修正射角) */
            public static double PITCH_OFFSET_DEG         = 0.0;

            /** 發射器出球口相對於機器底盤旋轉中心的前後偏移量 (in，向前為正) */
            public static double TURRET_FORWARD_IN        = 0.0;

            /** 發射器出球口相對於機器底盤旋轉中心的左右偏移量 (in，向左為正) */
            public static double TURRET_LEFT_IN           = 0.0;

            // ------------------------------------------------------------------------
            // 7.2.3 場地物理常數
            // ------------------------------------------------------------------------
            /** 場地重力加速度 (in/s^2，預設 386.09) */
            public static double G = 386.09;

            /** 出球口與籃框進球點的垂直高度差 (in，籃框高度 53.5 吋減發射器高度) */
            public static double H = 53.5;

            /** 彈道掠過籃框前緣防撞矮牆的額外垂直安全淨空裕度 (in) */
            public static double FRONT_WALL_CLEARANCE_IN = 1.0;

            /** 飛輪機構硬體極限所能輸出的最高安全發射初速限制 (in/s) */
            public static double MAX_VELOCITY_IN_PER_SEC = 512.0;

            // ------------------------------------------------------------------------
            // 7.2.4 舵機硬體零點與線性公式
            // ------------------------------------------------------------------------
            /** 砲台正面向前 (0°) 時的伺服舵機 Position (0.0 ~ 1.0) */
            public static double TURRET_CENTER     = 0.5;

            /** 砲台舵機每單位 Position 變化的旋轉角度 (度/Pos) */
            public static double TURRET_DEG_PER_POS = 0.00395;

            /** 舊命名相容別名 */
            public static double YAW_CENTER = TURRET_CENTER;
            public static double YAW_DEG_PER_POS = TURRET_DEG_PER_POS;

            /** 仰角伺服最小物理限制角度 (度) */
            public static double MIN_PITCH_ANGLE_DEG = 30.0;

            /** 仰角伺服最大物理限制角度 (度) */
            public static double MAX_PITCH_ANGLE_DEG = 54.0;

            /** 仰角轉舵機位置之線性公式斜率 M (pos = M * deg + C) */
            public static double PITCH_SERVO_M = -0.0383333;

            /** 仰角轉舵機位置之線性公式截距 C (pos = M * deg + C) */
            public static double PITCH_SERVO_C = 2.15;

            /** 仰角舵機輸出最小 Position 保護值 (防止卡底盤) */
            public static double PITCH_SERVO_MIN = 0.08;

            /** 仰角舵機輸出最大 Position 保護值 (防止脫軌) */
            public static double PITCH_SERVO_MAX = 1.0;

            // ------------------------------------------------------------------------
            // 7.2.5 馬達物理規格與閉迴路 PIDF
            // ------------------------------------------------------------------------
            /** 馬達單圈編碼器 Ticks 數 (GoBilda 5202 裸馬達 = 28.0) */
            public static double TICKS_PER_REVOLUTION = 28.0;

            /** 馬達軸到飛輪軸的傳動齒輪/皮帶速比 (直驅 = 1.0) */
            public static double GEAR_RATIO = 1.0;

            /** 飛輪外緣射彈接觸面實際測量半徑 (in) */
            public static double FLYWHEEL_RADIUS_IN = 2.0;

            /** 飛輪速度閉迴路控制 PIDF 係數 (P, I, D, F) */
            public static PIDFCoefficients flywheelPID = new PIDFCoefficients(1.0, 0.0, 0.0, 0.0);
        }
    }
}
