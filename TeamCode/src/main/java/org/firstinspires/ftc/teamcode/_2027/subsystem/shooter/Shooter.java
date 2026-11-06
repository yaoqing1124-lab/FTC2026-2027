package org.firstinspires.ftc.teamcode._2027.subsystem.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2027.Config;
import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2027.util.MotorConfig;
import org.firstinspires.ftc.teamcode._2027.util.ServoConfig;
import org.firstinspires.ftc.teamcode._2027.util.ShooterConfig;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

/**
 * 發射器子系統 (Shooter Subsystem)
 * 內部自動維持全時常態瞄準與飛輪前饋轉速就緒，是否發射完全由 Intake 控制。
 * 對外僅提供 isEnablePredict 開關控制以及各項狀態數據之 get 讀取介面。
 */
public class Shooter extends OpBehavior {

    // ==========================================
    // 雙發射器靜態實例 (Dual Subsystem Instances)
    // ==========================================
    public static final Shooter small = new Shooter(ShooterConfig.SMALL);
    public static final Shooter big   = new Shooter(ShooterConfig.BIG);

    // 別名 (Aliases)
    public static final Shooter shooter1 = small;
    public static final Shooter shooter2 = big;

    static {
        // 向 OpObject 註冊全域行為
        OpObject.RegisterGlobalBehavior(small);
        OpObject.RegisterGlobalBehavior(big);
    }

    public static Shooter getSmall() { return small; }
    public static Shooter getBig() { return big; }
    public static Shooter getShooter1() { return shooter1; }
    public static Shooter getShooter2() { return shooter2; }

    // ==========================================
    // 內部成員變數 (封裝保護)
    // ==========================================
    private final ShooterConfig m_config;

    private double targetRPM = 0;
    private double actualRPM = 0;
    private final boolean isEnable = true;      // 始終處於常態就緒狀態
    private final boolean enableAiming = true;  // 始終處於常態瞄準狀態
    private boolean enablePredict = true;       // 對外提供控制

    private final ShooterPose current_position = new ShooterPose();
    private final ShooterPose target_position = new ShooterPose();
    private final ShooterPose predict_target_position = new ShooterPose();

    private DcMotorEx m_flyWheel;
    private Servo m_pitch;
    private Servo m_turret;
    private Follower m_follower;

    private double m_lastTargetVelocity = 0;
    private double m_lastTargetPitch = 40.0;

    // RPM 平滑濾波滑動窗口
    private static final int RPM_BUFFER_SIZE = 7;
    private final double[] m_rpmBuffer = new double[RPM_BUFFER_SIZE];
    private int m_rpmBufferIdx = 0;
    private boolean m_rpmBufferFull = false;

    private Pose m_aimTarget;

    /**
     * 建構子：傳入專屬的 ShooterConfig
     */
    public Shooter(ShooterConfig config) {
        this.m_config = config;
    }

    // ==========================================
    // 對外介面：isEnablePredict 與 get 介面
    // ==========================================

    /**
     * 取得是否啟用移動速度超前預測補償
     */
    public boolean isEnablePredict() {
        return enablePredict;
    }

    /**
     * 設定是否啟用移動速度超前預測補償
     */
    public void setEnablePredict(boolean enablePredict) {
        this.enablePredict = enablePredict;
    }

    /**
     * 靜態聯合設定是否啟用移動預測補償
     */
    public static void setEnablePredictAll(boolean enablePredict) {
        small.setEnablePredict(enablePredict);
        big.setEnablePredict(enablePredict);
    }

    /**
     * 取得當前目標轉速 (RPM)
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * 取得經過平滑濾波後的真實轉速 (RPM)
     */
    public double getActualRPM() {
        return actualRPM;
    }

    /**
     * 取得轉速是否已達標穩定
     */
    public boolean isStable() {
        return Math.abs(targetRPM - actualRPM) < m_config.getRpmErrorThreshold();
    }

    public boolean getIsStable() {
        return isStable();
    }

    /**
     * 取得兩組發射器是否皆已穩定
     */
    public static boolean isAllStable() {
        return small.isStable() && big.isStable();
    }

    /**
     * 取得當前發射器出球口的場地座標
     */
    public ShooterPose getCurrentPosition() {
        return current_position;
    }

    /**
     * 取得當前籃框目標座標
     */
    public ShooterPose getTargetPosition() {
        return target_position;
    }

    /**
     * 取得經移動速度預測補償後的虛擬發射目標座標
     */
    public ShooterPose getPredictTargetPosition() {
        return predict_target_position;
    }

    /**
     * 取得當前計算之目標發射初速 (in/s)
     */
    public double getLastTargetVelocity() {
        return m_lastTargetVelocity;
    }

    /**
     * 取得當前計算之目標仰角角度 (度)
     */
    public double getLastTargetPitch() {
        return m_lastTargetPitch;
    }

    /**
     * 取得該發射器專屬的調適配置物件
     */
    public ShooterConfig getConfig() {
        return m_config;
    }

    // ==========================================
    // 生命週期 (Lifecycle Methods)
    // ==========================================
    @Override
    public void Init() {
        m_flyWheel = init_flyWheel(m_config.motor);
        m_pitch    = init_servo(m_config.pitchServo);
        m_turret   = init_servo(m_config.turretServo);
        if (m_turret != null) {
            m_turret.setPosition(m_config.getTurretCenter());
        }
    }

    @Override
    public void Start() {
        m_follower = Drivetrain.get_Follower();

        // 依據聯盟顏色與座標選擇目標籃框位置
        updateTargetBasket();
        target_position.x = m_aimTarget.getX();
        target_position.y = m_aimTarget.getY();
        target_position.z = m_config.getH();
        resetPitch();
    }

    @Override
    public void Loop() {
        if (m_follower == null) {
            m_follower = Drivetrain.get_Follower();
        }

        // --- 1. 更新目標籃框座標 (防止在 y=0 邊界抖動) ---
        updateTargetBasket();
        target_position.x = m_aimTarget.getX();
        target_position.y = m_aimTarget.getY();
        target_position.z = m_config.getH();

        // --- 2. 機器人中心座標轉換至出球口座標 ---
        Pose current_pose = (m_follower != null) ? m_follower.getPose() : new Pose(0, 0, 0);
        ShooterPose rawPose = new ShooterPose(
                current_pose.getX(),
                current_pose.getY(),
                0,
                current_pose.getHeading()
        );
        ShooterPose turretPose = ShooterCalculator.currentPoseToTurretPose(rawPose, m_config);
        current_position.x      = turretPose.x;
        current_position.y      = turretPose.y;
        current_position.turret = turretPose.turret;

        // --- 3. 機器人移動速度超前補償 (Motion Compensation) ---
        if (enablePredict && m_follower != null) {
            Vector rv = m_follower.getVelocity();
            double ft = ShooterCalculator.flytimeVertical(
                    m_lastTargetVelocity,
                    m_lastTargetPitch,
                    m_config.getH(),
                    m_config.getG()
            );
            predict_target_position.x = target_position.x - rv.getXComponent() * ft;
            predict_target_position.y = target_position.y - rv.getYComponent() * ft;
            predict_target_position.z = target_position.z;
        } else {
            predict_target_position.x = target_position.x;
            predict_target_position.y = target_position.y;
            predict_target_position.z = target_position.z;
        }

        // --- 4. 讀取真實轉速與滑動均值濾波 ---
        double rawRPM = (m_flyWheel != null)
                ? ShooterCalculator.TicksPerSecToRPM(m_flyWheel.getVelocity(), m_config)
                : 0.0;
        m_rpmBuffer[m_rpmBufferIdx] = rawRPM;
        m_rpmBufferIdx = (m_rpmBufferIdx + 1) % RPM_BUFFER_SIZE;
        if (m_rpmBufferIdx == 0) m_rpmBufferFull = true;

        int count = m_rpmBufferFull ? RPM_BUFFER_SIZE : m_rpmBufferIdx;
        double sum = 0;
        for (int i = 0; i < count; i++) sum += m_rpmBuffer[i];
        actualRPM = (count > 0) ? sum / count : rawRPM;

        double actualVelocityInPerSec = ShooterCalculator.RPMToVelocity(actualRPM, m_config);

        // --- 5. 虛擬目標點變換 (空氣阻力與旋轉下墜補償) ---
        ShooterPose virtualTarget = ShooterCalculator.calculateVirtualTarget(
                current_position,
                predict_target_position,
                m_config
        );

        // --- 6. 理想初速與前饋目標 RPM 計算 ---
        double targetVelocityInPerSec = ShooterCalculator.calculateIdealVelocity(
                current_position,
                virtualTarget,
                m_lastTargetVelocity,
                m_config
        );
        targetRPM = ShooterCalculator.VelocityToRPM(targetVelocityInPerSec, m_config);
        m_lastTargetVelocity = targetVelocityInPerSec;

        // --- 7. 仰角閉迴路反饋計算 ---
        double targetPitchDeg;
        if (isStable()) {
            // 已達穩定轉速：以真實轉速反饋計算精準仰角
            targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                    current_position,
                    virtualTarget,
                    actualVelocityInPerSec,
                    m_config
            );
        } else {
            // 轉速提速中：以理想初速保持仰角，防止舵機抖動
            targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                    current_position,
                    virtualTarget,
                    targetVelocityInPerSec,
                    m_config
            );
        }
        m_lastTargetPitch = targetPitchDeg;

        // --- 8. 套用硬體輸出 (全時常態瞄準與飛輪前饋控制) ---
        update_turret();
        update_pitch(targetPitchDeg);

        if (m_flyWheel != null) {
            update_flyWheel(targetRPM);
        }
    }

    @Override
    public void Stop() {
        if (m_flyWheel != null) {
            m_flyWheel.setPower(0);
            m_flyWheel = null;
        }
        if (m_pitch != null) {
            m_pitch.setPosition(m_config.getPitchServoC()); // 歸位
            m_pitch = null;
        }
        if (m_turret != null) {
            m_turret.setPosition(m_config.getTurretCenter());
            m_turret = null;
        }
        m_follower = null;
    }

    // ==========================================
    // 內部私有輔助方法 (Private Helpers)
    // ==========================================
    private void resetPitch() {
        m_rpmBufferIdx = 0;
        m_rpmBufferFull = false;
        java.util.Arrays.fill(m_rpmBuffer, 0.0);
    }

    private DcMotorEx init_flyWheel(MotorConfig config) {
        if (m_object == null || m_object.hardwareMap == null || config == null) return null;
        try {
            DcMotorEx ret = m_object.hardwareMap.get(DcMotorEx.class, config.id);
            ret.setDirection(config.direction);
            ret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ret.setPower(0);
            return ret;
        } catch (Exception e) {
            return null;
        }
    }

    private Servo init_servo(ServoConfig config) {
        if (m_object == null || m_object.hardwareMap == null || config == null) return null;
        try {
            Servo ret = m_object.hardwareMap.get(Servo.class, config.id);
            ret.setDirection(config.direction);
            return ret;
        } catch (Exception e) {
            return null;
        }
    }

    private void update_flyWheel(double targetRpm) {
        if (m_flyWheel == null) return;
        m_flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, m_config.getFlywheelPID());
        double targetTicksPerSec = ShooterCalculator.RPMToTicksPerSec(targetRpm, m_config);
        m_flyWheel.setVelocity(targetTicksPerSec);
    }

    private void update_pitch(double targetPitchDeg) {
        if (m_pitch == null) return;
        if (Double.isNaN(targetPitchDeg) || Double.isInfinite(targetPitchDeg)) return;
        double position = Range.clip(
                m_config.getPitchServoM() * targetPitchDeg + m_config.getPitchServoC(),
                m_config.getPitchServoMin(),
                m_config.getPitchServoMax()
        );
        m_pitch.setPosition(position);
    }

    private void update_turret() {
        if (m_turret == null) return;
        double target_deg = ShooterCalculator.calculateTurret(current_position, predict_target_position);
        // 砲台正向 (0.5) 預設為向後，偏移 180 度轉換為正前方基準
        target_deg = target_deg - 180.0;
        while (target_deg >  180) target_deg -= 360;
        while (target_deg < -180) target_deg += 360;

        final double center = m_config.getTurretCenter();
        final double deg2pos = m_config.getTurretDegPerPos();
        double targetPosition = center;
        if (!Double.isNaN(target_deg) && !Double.isInfinite(target_deg)) {
            targetPosition = Range.clip(center - target_deg * deg2pos / (11/10), 0.0, 1.0);
        }
        m_turret.setPosition(targetPosition);
    }

    private void updateTargetBasket() {
        Team currentTeam = (OpObject.team != null && OpObject.team != Team.none)
                ? OpObject.team
                : Config.team;
        double currentY = (m_follower != null) ? m_follower.getPose().getY() : 0.0;
        double deadband = 2.0; // 邊界磁滯緩衝區 (防止在 Y=0 邊界抖動)

        // 當機器人處於場地上方 (Y > 0) 時自動瞄準上方 Hive，下方 (Y < 0) 時瞄準下方 Hive
        boolean aimUpper;
        if (m_aimTarget == null) {
            aimUpper = (currentY >= 0);
        } else if (currentY > deadband) {
            aimUpper = true;
        } else if (currentY < -deadband) {
            aimUpper = false;
        } else {
            aimUpper = (m_aimTarget.getY() > 0);
        }

        if (currentTeam == Team.blue) {
            m_aimTarget = aimUpper ? Config.field.targetPose.bluePositive : Config.field.targetPose.blueNegative;
        } else {
            m_aimTarget = aimUpper ? Config.field.targetPose.redPositive : Config.field.targetPose.redNegative;
        }
    }
}
