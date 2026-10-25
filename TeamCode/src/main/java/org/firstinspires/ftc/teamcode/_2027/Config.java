package org.firstinspires.ftc.teamcode._2027;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode._2027.util.AnalogConfig;
import org.firstinspires.ftc.teamcode._2027.util.LimelightConfig;
import org.firstinspires.ftc.teamcode._2027.util.MotorConfig;
import org.firstinspires.ftc.teamcode._2027.util.ServoConfig;
import org.firstinspires.ftc.teamcode._2027.util.Team;

public class Config {
    Team team = Team.none;
    // +=======================+
    // |硬體設定               |
    // +=======================+
    public static class hardware{
        public static class motor{
            public final static MotorConfig lfm        = new MotorConfig("lfm",        DcMotor.Direction.REVERSE);
            public final static MotorConfig lbm        = new MotorConfig("lbm",        DcMotor.Direction.REVERSE);
            public final static MotorConfig rfm        = new MotorConfig("rfm",        DcMotor.Direction.FORWARD);
            public final static MotorConfig rbm        = new MotorConfig("rbm",        DcMotor.Direction.FORWARD);
            public final static MotorConfig flyWheel   = new MotorConfig("flywheel",   DcMotorSimple.Direction.REVERSE);
            public final static MotorConfig intake     = new MotorConfig("intake",     DcMotorSimple.Direction.REVERSE);
            public final static MotorConfig intakeCtrl = new MotorConfig("intakeCtrl", DcMotorSimple.Direction.FORWARD);
        }
        public static class servo{
            public final static ServoConfig lfs   = new ServoConfig("lfs");
            public final static ServoConfig lbs   = new ServoConfig("lbs");
            public final static ServoConfig rfs   = new ServoConfig("rfs");
            public final static ServoConfig rbs   = new ServoConfig("rbs");
            public final static ServoConfig yaw   = new ServoConfig("yaw");
            public final static ServoConfig pitch = new ServoConfig("pitch");
        }
        public static class sensor{
            public final static AnalogConfig    lfenc        = new AnalogConfig("lfenc");
            public final static AnalogConfig    lbenc        = new AnalogConfig("lbenc");
            public final static AnalogConfig    rfenc        = new AnalogConfig("rfenc");
            public final static AnalogConfig    rbenc        = new AnalogConfig("rbenc");
            public final static LimelightConfig limelight = new LimelightConfig("limelight");
        }
    }

    // +=======================+
    // |定位調適參數            |
    // +=======================+

    public static class odometry {
        public final static String id = "pinpoint"; // odometry configure name
        public final static double forwardOffset = 60.0; //Redo robot -> fix    odometry偏移量(mm)
        public final static double strafeOffset  = 16.6; //Redo robot -> fix    (mm)
        public final static GoBildaPinpointDriver.EncoderDirection forwardDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public final static GoBildaPinpointDriver.EncoderDirection strafeDirection  = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }

    // +=======================+
    // |機器動作參數設定        |
    // +=======================+
    public static class value{
        public final static double INTAKE_POWER = 1;
    }


    // +=======================+
    // |飛輪子系統調適參數      |
    // +=======================+
    public static class shooter{
        // ==========================================
        // 雙參數架構 - 場地動態擬合常數 (Two-Parameter Architecture)
        // 系統只需這兩個參數即可擬合所有的非理想物理因素 (空氣阻力、Magnus 效應、打滑損耗)：
        // 1. ENERGY_CONVERSION_RATE (能量轉換率): 近點無阻力測試求出，代表飛輪轉速轉換為球體初速的效率 (通常 0.5 ~ 0.9)。
        // 2. ENERGY_LOSS_RATE (能量損失率): 遠點測試求出，透過平移虛擬目標點 B' 來彌補空氣阻力造成的動能流失 (通常為微小的正數)。
        // ==========================================
        public static double ENERGY_CONVERSION_RATE = 0.8472;
        public static double ENERGY_LOSS_RATE       = 0.0002;
        public static double INTAKE_INITIAL_VELOCITY_IN_PER_SEC = 3;
        // ==========================================
        // 數學模型與反饋控制參數 (Math & Feedback Control)
        // 1.IDEAL_INCIDENT_ANGLE_DEG: 目標 B 點的理想入射角 (度)，讓球能從上方完美掉入籃筐
        // 2.RPM_ERROR_THRESHOLD: 反饋層的 RPM 誤差容忍度 (RPM)。當真實轉速與目標轉速相差過大時，不進行仰角微調以防伺服抖動
        // 3.PITCH_OFFSET_DEG: 手動仰角微調偏移量 (度)
        // 4.TURRET_FORWARD_IN: 發射器對於機器中心的向前偏移量
        // 5.TURRET_LEFT_IN: 發射器對於機器中心的向左偏移量
        // ==========================================
        public static double IDEAL_INCIDENT_ANGLE_DEG = 60;

        public static double RPM_ERROR_THRESHOLD = 150.0;
        public static double PITCH_OFFSET_DEG = 0.0;
        public static double TURRET_FORWARD_IN = 0.0; //Redo robot -> fix
        public static double TURRET_LEFT_IN    = 0.0; //Redo robot -> fix

        // ==========================================
        // 物理常數
        // 1.G: 重力加速度 (in/s^2)
        // 2.H: 出球點對目標點的高度差 (in) 53.5 - 發射器高度
        // 3.FRONT_WALL_CLEARANCE_IN: 防撞框演算法(控制要射擊再進球點上幾in的位置, 求直徑+安全域值)
        // 4.MAX_VELOCITY_IN_PER_SEC: 飛輪最大線速度 (in/s)，約略值
        // ==========================================
        public static class physical{
            public static final double G = 386.09;
            public static final double H = 53.5 - 0.0; //Redo robot -> fix
            public static double FRONT_WALL_CLEARANCE_IN = 1.0;
            public static final double MAX_VELOCITY_IN_PER_SEC = 512.0;


        }
        // ==========================================
        // 絕對硬體常數
        // 這些數值取決於物理與硬體規格，通常不需要在場地微調：
        // 1. YAW_CENTER: 炮台面向正前方的位置(position)
        // 2. TICKS_PER_REVOLUTION: 直接查閱馬達規格表 (例如 GoBilda 裸馬達 = 28，REV HD Hex = 28)
        // 3. GEAR_RATIO: 馬達軸到飛輪軸的齒輪/皮帶傳動比 (直驅 = 1.0)
        // 4. FLYWHEEL_RADIUS_IN: 使用游標卡尺量測飛輪的實際半徑 (in)
        // 5. MIN_PITCH_ANGLE_DEG: 最小仰角硬體限制 (度)
        // 6. MAX_PITCH_ANGLE_DEG: 最大仰角硬體限制 (度)
        // 7. flywheelController: 飛輪轉速控制
        // ==========================================
        public static class hardware{
            public static final double YAW_CENTER = 0.5; // Reorganization -> change
            public static final double TICKS_PER_REVOLUTION = 28.0; //Redo robot -> fix
            public static final double GEAR_RATIO = 1.0; //Redo robot -> fix
            public static final double FLYWHEEL_RADIUS_IN = 2.0; //Redo robot -> fix
            public static final double MIN_PITCH_ANGLE_DEG = 30.0; //Redo robot -> fix
            public static final double MAX_PITCH_ANGLE_DEG = 54.0; //Redo robot -> fix
            public static final PIDFCoefficients flywheelController = new PIDFCoefficients(1, 0, 0, 0);
        }
    }
    public static class field{
        public static class targetPose{
            public static final Pose bluePositive = new Pose(-12.9, -19.89);
            public static final Pose blueNegative = new Pose( 12.9,  19.89);
            public static final Pose redPositive  = new Pose(-12.9,  19.89);
            public static final Pose redNegative  = new Pose( 12.9, -19.89);
        }
        public static class shootPose{
        }
    }




    public static class marrow{
        public static final double INTAKE_WIDTH = 8.0; //intake寬度。0停止於目標；實際寬度掃過元素
        public static final double ROBOT_WIDTH = 16.0; //沿前軸的機器人足跡
        public static final double ROBOT_HEIGHT = 16.0; //機器人沿著前軸的足跡
    }
}
