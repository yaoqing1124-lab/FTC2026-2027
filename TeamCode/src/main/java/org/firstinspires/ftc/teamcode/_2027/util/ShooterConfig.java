package org.firstinspires.ftc.teamcode._2027.util;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode._2027.Config;

/**
 * 發射器調適數據與硬體設定包裝類別 (Shooter Configuration & Tuning Data)
 * 封裝單一發射器所需的所有物理、擬合、硬體常數及 PIDF。
 * 統一使用 turret 命名砲台水平轉向伺服與參數。
 * 支援鏈式調用 (Fluent API / Method Chaining)，例如：
 * ShooterConfig.SMALL.setIdealIncidentAngleDeg(58.0).setEnergyLossRate(0.00022);
 * 支援透過 Config.shooter.small / Config.shooter.big 進行 Live Dashboard 即時調適。
 */
public class ShooterConfig {
    // ==========================================
    // 預定義雙發射器實例 (Small & Big / Shooter 1 & 2)
    // 採用鏈式調用 (Method Chaining) 初始化各項參數
    // ==========================================

    /**
     * 發射器 1 (Small Shooter / Shooter 1) 預設配置物件。
     * 於 new 實例處採用鏈式調用初始化所有調適參數，方便使用者直接在此填寫數值；
     * 同步連鎖讀寫 Config.shooter.small 靜態變數。
     */
    public static final ShooterConfig SMALL = new ShooterConfig(
            "small",
            Config.hardware.motor.flyWheelSmall,
            Config.hardware.servo.pitchSmall,
            Config.hardware.servo.turretSmall,
            true
    )
            // ------------------------------------------------------------------------
            // 7.1.1 雙參數擬合核心 (近距離與遠距離實測求出)
            // ------------------------------------------------------------------------
            .setEnergyConversionRate(0.8472)                 // 能量轉換率 (通常 0.70 ~ 0.90，近距離實測填入)
            .setEnergyLossRate(0.0002)                       // 能量損失率 (通常 0.0001 ~ 0.0004，遠距離實測填入)
            .setIntakeInitialVelocityInPerSec(3.0)           // 進球機構初始線速度 (in/s)

            // ------------------------------------------------------------------------
            // 7.1.2 瞄準幾何與反饋控制
            // ------------------------------------------------------------------------
            .setIdealIncidentAngleDeg(60.0)                  // 理想入框入射角度 (度，通常 55.0 ~ 65.0 度)
            .setRpmErrorThreshold(150.0)                     // 判定轉速穩定閾值 (RPM)
            .setPitchOffsetDeg(0.0)                          // 仰角角度手動微調偏移 (度)
            .setTurretForwardIn(0.0)                         // 出球口前後安裝偏移 (in，向前為正)
            .setTurretLeftIn(0.0)                            // 出球口左右安裝偏移 (in，向左為正)

            // ------------------------------------------------------------------------
            // 7.1.3 場地物理常數
            // ------------------------------------------------------------------------
            .setG(386.09)                                    // 場地重力加速度 (in/s^2)
            .setH(53.5)                                      // 出球口與目標籃框垂直高度差 (in)
            .setFrontWallClearanceIn(1.0)                    // 越過防撞矮牆的安全淨空裕度 (in)
            .setMaxVelocityInPerSec(512.0)                   // 飛輪最高安全限制發射初速 (in/s)

            // ------------------------------------------------------------------------
            // 7.1.4 舵機硬體零點與線性公式
            // ------------------------------------------------------------------------
            .setTurretCenter(0.5)                            // 砲台中心舵機位置 (0.0 ~ 1.0)
            .setTurretDegPerPos(0.00395)                     // 砲台舵機每單位位置對應角度 (度/Pos)
            .setMinPitchAngleDeg(30.0)                       // 仰角最小物理限制角度 (度)
            .setMaxPitchAngleDeg(54.0)                       // 仰角最大物理限制角度 (度)
            .setPitchServoM(-0.0383333)                      // 仰角轉舵機線性公式斜率 M (pos = M * deg + C)
            .setPitchServoC(2.15)                            // 仰角轉舵機線性公式截距 C (pos = M * deg + C)
            .setPitchServoMin(0.08)                          // 仰角舵機最小位置保護限制 (0.0 ~ 1.0)
            .setPitchServoMax(1.0)                           // 仰角舵機最大位置保護限制 (0.0 ~ 1.0)

            // ------------------------------------------------------------------------
            // 7.1.5 馬達物理規格與閉迴路 PIDF
            // ------------------------------------------------------------------------
            .setTicksPerRevolution(28.0)                     // 馬達單圈編碼器脈衝數 (Ticks/Rev)
            .setGearRatio(1.0)                               // 傳動齒輪/皮帶速比 (直驅為 1.0)
            .setFlywheelRadiusIn(2.0)                        // 飛輪物理測量半徑 (in)
            .setFlywheelPID(new PIDFCoefficients(1.0, 0.0, 0.0, 0.0)); // 飛輪 PIDF 係數

    /**
     * 發射器 2 (Big Shooter / Shooter 2) 預設配置物件。
     * 於 new 實例處採用鏈式調用初始化所有調適參數，方便使用者直接在此填寫數值；
     * 同步連鎖讀寫 Config.shooter.big 靜態變數。
     */
    public static final ShooterConfig BIG = new ShooterConfig(
            "big",
            Config.hardware.motor.flyWheelBig,
            Config.hardware.servo.pitchBig,
            Config.hardware.servo.turretBig,
            true
    )
            // ------------------------------------------------------------------------
            // 7.2.1 雙參數擬合核心 (近距離與遠距離實測求出)
            // ------------------------------------------------------------------------
            .setEnergyConversionRate(0.8472)                 // 能量轉換率 (通常 0.70 ~ 0.90，近距離實測填入)
            .setEnergyLossRate(0.0002)                       // 能量損失率 (通常 0.0001 ~ 0.0004，遠距離實測填入)
            .setIntakeInitialVelocityInPerSec(3.0)           // 進球機構初始線速度 (in/s)

            // ------------------------------------------------------------------------
            // 7.2.2 瞄準幾何與反饋控制
            // ------------------------------------------------------------------------
            .setIdealIncidentAngleDeg(60.0)                  // 理想入框入射角度 (度，通常 55.0 ~ 65.0 度)
            .setRpmErrorThreshold(150.0)                     // 判定轉速穩定閾值 (RPM)
            .setPitchOffsetDeg(0.0)                          // 仰角角度手動微調偏移 (度)
            .setTurretForwardIn(0.0)                         // 出球口前後安裝偏移 (in，向前為正)
            .setTurretLeftIn(0.0)                            // 出球口左右安裝偏移 (in，向左為正)

            // ------------------------------------------------------------------------
            // 7.2.3 場地物理常數
            // ------------------------------------------------------------------------
            .setG(386.09)                                    // 場地重力加速度 (in/s^2)
            .setH(53.5)                                      // 出球口與目標籃框垂直高度差 (in)
            .setFrontWallClearanceIn(1.0)                    // 越過防撞矮牆的安全淨空裕度 (in)
            .setMaxVelocityInPerSec(512.0)                   // 飛輪最高安全限制發射初速 (in/s)

            // ------------------------------------------------------------------------
            // 7.2.4 舵機硬體零點與線性公式
            // ------------------------------------------------------------------------
            .setTurretCenter(0.5)                            // 砲台中心舵機位置 (0.0 ~ 1.0)
            .setTurretDegPerPos(0.00395)                     // 砲台舵機每單位位置對應角度 (度/Pos)
            .setMinPitchAngleDeg(30.0)                       // 仰角最小物理限制角度 (度)
            .setMaxPitchAngleDeg(54.0)                       // 仰角最大物理限制角度 (度)
            .setPitchServoM(-0.0383333)                      // 仰角轉舵機線性公式斜率 M (pos = M * deg + C)
            .setPitchServoC(2.15)                            // 仰角轉舵機線性公式截距 C (pos = M * deg + C)
            .setPitchServoMin(0.08)                          // 仰角舵機最小位置保護限制 (0.0 ~ 1.0)
            .setPitchServoMax(1.0)                           // 仰角舵機最大位置保護限制 (0.0 ~ 1.0)

            // ------------------------------------------------------------------------
            // 7.2.5 馬達物理規格與閉迴路 PIDF
            // ------------------------------------------------------------------------
            .setTicksPerRevolution(28.0)                     // 馬達單圈編碼器脈衝數 (Ticks/Rev)
            .setGearRatio(1.0)                               // 傳動齒輪/皮帶速比 (直驅為 1.0)
            .setFlywheelRadiusIn(2.0)                        // 飛輪物理測量半徑 (in)
            .setFlywheelPID(new PIDFCoefficients(1.0, 0.0, 0.0, 0.0)); // 飛輪 PIDF 係數

    // ==========================================
    // 硬體元件配置 (Hardware Configurations)
    // ==========================================
    /** 發射器識別名稱 (如 "small" 或 "big") */
    public final String name;

    /** 飛輪驅動馬達硬體配置物件 */
    public final MotorConfig motor;

    /** 俯仰角 (Pitch) 控制伺服舵機配置物件 */
    public final ServoConfig pitchServo;

    /** 砲台水平旋轉 (Turret) 控制伺服舵機配置物件 */
    public final ServoConfig turretServo;

    /** 舊命名相容別名 (指向 turretServo) */
    public final ServoConfig yawServo;

    /** 標註此配置是否動態連結至 Config.java 靜態變數 */
    private final boolean isDynamic;

    // ==========================================
    // 獨立儲存空間 (當作為自訂獨立物件使用時存放之數值)
    // ==========================================

    /**
     * 【能量轉換率 (Energy Conversion Rate)】
     * 飛輪表面線速度轉換為球體出膛初速的真實效率 (無量綱，通常介於 0.6 ~ 0.9)。
     * 於近距離無空氣阻力實測求出。
     */
    public double energyConversionRate;

    /**
     * 【能量損失率 (Energy Loss Rate)】
     * 球體飛行過程中因空氣阻力與旋轉 (Magnus 效應) 造成的動能衰減與提早下墜補償係數 (無量綱，通常為 0.0001 ~ 0.0004)。
     * 於遠距離測試求出。
     */
    public double energyLossRate;

    /**
     * 【進球機構初速 (Intake Initial Velocity)】
     * 當進球機構 (Intake) 將球推送進發射器時，進球輪賦予球體的初始線速度 (in/s)。
     */
    public double intakeInitialVelocityInPerSec;

    /**
     * 【理想入框入射角 (Ideal Incident Angle)】
     * 球體落入目標籃框開口時的期望入射角度 (度，通常設定為 55° ~ 65°)。
     * 角度越大越垂直掉入籃框、容錯率越高，但需要較大的發射動能。
     */
    public double idealIncidentAngleDeg;

    /**
     * 【轉速達標誤差閾值 (RPM Error Threshold)】
     * 真實轉速與目標轉速的允許誤差 (RPM)。
     * 當 |targetRPM - actualRPM| 小於此值時，系統判定 isStable() 為 true 並啟用閉迴路仰角微調；
     * 提速中則維持目標初速之仰角，防止伺服馬達抖動。
     */
    public double rpmErrorThreshold;

    /**
     * 【仰角手動微調偏移 (Pitch Offset)】
     * 仰角伺服角度的手動微調偏移量 (度，可為正負值)，用於現場快速補正整體射角。
     */
    public double pitchOffsetDeg;

    /**
     * 【砲台出球口前後物理偏移 (Turret Forward Offset)】
     * 發射器出球口相對於機器底盤旋轉中心在前後軸向上的物理安裝偏移量 (in，向前為正)。
     */
    public double turretForwardIn;

    /**
     * 【砲台出球口左右物理偏移 (Turret Left Offset)】
     * 發射器出球口相對於機器底盤旋轉中心在左右橫向上的物理安裝偏移量 (in，向左為正)。
     */
    public double turretLeftIn;

    /**
     * 【場地重力加速度 (Gravitational Acceleration)】
     * 拋物線彈道計算所依據的重力加速度 (in/s^2，標準重力加速度約為 386.09 in/s^2)。
     */
    public double g;

    /**
     * 【出球口與目標垂直高度差 (Height Difference)】
     * 發射器出球口高度與目標籃框進球點的垂直高度差 (in，籃框高度約 53.5 吋減去發射器離地高度)。
     */
    public double h;

    /**
     * 【防撞矮牆安全裕度 (Front Wall Clearance)】
     * 拋物線軌跡越過籃框前緣防撞結構時的垂直額外安全淨空高度 (in)。
     */
    public double frontWallClearanceIn;

    /**
     * 【飛輪最大安全初速 (Max Launch Velocity)】
     * 飛輪硬體馬達在物理極限下所能賦予射彈的最大出膛線速度 (in/s)。
     */
    public double maxVelocityInPerSec;

    /**
     * 【砲台中心位置舵機訊號 (Turret Center Position)】
     * 當砲台指向機器人正前方 (0°) 時，所對應的伺服舵機控制位置 (0.0 ~ 1.0，預設通常為 0.5)。
     */
    public double turretCenter;

    /**
     * 【砲台舵機每 Position 對應角度 (Turret Degrees per Position)】
     * 砲台旋轉伺服每 1.0 單位控制位置變化所對應的物理旋轉角度變化量 (度/Pos)。
     */
    public double turretDegPerPos;

    /**
     * 【馬達單圈編碼器脈衝數 (Ticks Per Revolution)】
     * 飛輪驅動馬達每轉動一圈所回傳的編碼器計數值 (Ticks/Rev，例如 GoBilda 5202 裸馬達為 28.0)。
     */
    public double ticksPerRevolution;

    /**
     * 【飛輪機構傳動比 (Gear Ratio)】
     * 馬達輸出軸到飛輪旋轉軸的傳動齒輪/皮帶速比 (直驅為 1.0，升速比例如 1.0/2.0)。
     */
    public double gearRatio;

    /**
     * 【飛輪外圓接觸面實際半徑 (Flywheel Radius)】
     * 飛輪膠輪外緣與射彈接觸點的物理測量半徑 (in)。
     */
    public double flywheelRadiusIn;

    /**
     * 【仰角伺服最小物理限制角度 (Min Pitch Angle)】
     * 仰角調整機構在物理或機械結構上所允許的最小發射角度 (度)。
     */
    public double minPitchAngleDeg;

    /**
     * 【仰角伺服最大物理限制角度 (Max Pitch Angle)】
     * 仰角調整機構在物理或機械結構上所允許的最大發射角度 (度)。
     */
    public double maxPitchAngleDeg;

    /**
     * 【仰角轉伺服位置線性斜率 (Pitch Servo Slope M)】
     * 仰角角度映射為伺服舵機 position 的線性公式斜率 (公式：position = M * deg + C)。
     */
    public double pitchServoM;

    /**
     * 【仰角轉伺服位置線性截距 (Pitch Servo Intercept C)】
     * 仰角角度映射為伺服舵機 position 的線性公式截距 (公式：position = M * deg + C)。
     */
    public double pitchServoC;

    /**
     * 【仰角伺服輸出最小值保護 (Pitch Servo Min Limit)】
     * 防止伺服位置輸出過小導致打到底盤或機構卡死的最小舵機 Position 限制 (0.0 ~ 1.0)。
     */
    public double pitchServoMin;

    /**
     * 【仰角伺服輸出最大值保護 (Pitch Servo Max Limit)】
     * 防止伺服位置輸出過大導致機構脫軌的最大舵機 Position 限制 (0.0 ~ 1.0)。
     */
    public double pitchServoMax;

    /**
     * 【飛輪閉迴路速度控制 PIDF 係數 (Flywheel Velocity PIDF)】
     * RUN_USING_ENCODER 模式下驅動飛輪馬達速度的 PIDF 係數 (P, I, D, F)。
     */
    public PIDFCoefficients flywheelPID;

    /**
     * 自訂獨立配置建構子 (用於非全域獨立實例)
     */
    public ShooterConfig(String name, MotorConfig motor, ServoConfig pitchServo, ServoConfig turretServo) {
        this.name        = name;
        this.motor       = motor;
        this.pitchServo  = pitchServo;
        this.turretServo = turretServo;
        this.yawServo    = turretServo;
        this.isDynamic   = false;
        this.flywheelPID = new PIDFCoefficients(1.0, 0, 0, 0);
    }

    /**
     * 綁定 Config.java 靜態變數的動態配置建構子 (供 small / big 使用)
     */
    private ShooterConfig(String name, MotorConfig motor, ServoConfig pitchServo, ServoConfig turretServo, boolean isDynamic) {
        this.name        = name;
        this.motor       = motor;
        this.pitchServo  = pitchServo;
        this.turretServo = turretServo;
        this.yawServo    = turretServo;
        this.isDynamic   = isDynamic;
    }
    /** 別名：支援大寫、小寫與數字別名調用 */
    public static final ShooterConfig small    = SMALL;
    public static final ShooterConfig big      = BIG;
    public static final ShooterConfig SHOOTER1 = SMALL;
    public static final ShooterConfig SHOOTER2 = BIG;
    public static final ShooterConfig shooter1 = SMALL;
    public static final ShooterConfig shooter2 = BIG;

    // ==========================================
    // Getter & Setter (支援鏈式調用 Method Chaining 與 Config 雙向同步)
    // ==========================================

    /** 取得能量轉換率 (飛輪轉速轉出膛初速效率) */
    public double getEnergyConversionRate() {
        if ("small".equals(name)) return Config.shooter.small.ENERGY_CONVERSION_RATE;
        if ("big".equals(name))   return Config.shooter.big.ENERGY_CONVERSION_RATE;
        return energyConversionRate;
    }
    /** 設定能量轉換率並返回 this 支援鏈式調用 */
    public ShooterConfig setEnergyConversionRate(double val) {
        this.energyConversionRate = val;
        if ("small".equals(name)) Config.shooter.small.ENERGY_CONVERSION_RATE = val;
        if ("big".equals(name))   Config.shooter.big.ENERGY_CONVERSION_RATE = val;
        return this;
    }

    /** 取得能量損失率 (空氣阻力與Magnus效應下墜補償係數) */
    public double getEnergyLossRate() {
        if ("small".equals(name)) return Config.shooter.small.ENERGY_LOSS_RATE;
        if ("big".equals(name))   return Config.shooter.big.ENERGY_LOSS_RATE;
        return energyLossRate;
    }
    /** 設定能量損失率並返回 this 支援鏈式調用 */
    public ShooterConfig setEnergyLossRate(double val) {
        this.energyLossRate = val;
        if ("small".equals(name)) Config.shooter.small.ENERGY_LOSS_RATE = val;
        if ("big".equals(name))   Config.shooter.big.ENERGY_LOSS_RATE = val;
        return this;
    }

    /** 取得進球機構賦予球體的初始推進速度 (in/s) */
    public double getIntakeInitialVelocityInPerSec() {
        if ("small".equals(name)) return Config.shooter.small.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;
        if ("big".equals(name))   return Config.shooter.big.INTAKE_INITIAL_VELOCITY_IN_PER_SEC;
        return intakeInitialVelocityInPerSec;
    }
    /** 設定進球機構推進初速並返回 this 支援鏈式調用 */
    public ShooterConfig setIntakeInitialVelocityInPerSec(double val) {
        this.intakeInitialVelocityInPerSec = val;
        if ("small".equals(name)) Config.shooter.small.INTAKE_INITIAL_VELOCITY_IN_PER_SEC = val;
        if ("big".equals(name))   Config.shooter.big.INTAKE_INITIAL_VELOCITY_IN_PER_SEC = val;
        return this;
    }

    /** 取得落入籃框的期望入射角度 (度) */
    public double getIdealIncidentAngleDeg() {
        if ("small".equals(name)) return Config.shooter.small.IDEAL_INCIDENT_ANGLE_DEG;
        if ("big".equals(name))   return Config.shooter.big.IDEAL_INCIDENT_ANGLE_DEG;
        return idealIncidentAngleDeg;
    }
    /** 設定落入籃框的期望入射角度並返回 this 支援鏈式調用 */
    public ShooterConfig setIdealIncidentAngleDeg(double val) {
        this.idealIncidentAngleDeg = val;
        if ("small".equals(name)) Config.shooter.small.IDEAL_INCIDENT_ANGLE_DEG = val;
        if ("big".equals(name))   Config.shooter.big.IDEAL_INCIDENT_ANGLE_DEG = val;
        return this;
    }

    /** 取得判定飛輪轉速穩定的誤差容許值 (RPM) */
    public double getRpmErrorThreshold() {
        if ("small".equals(name)) return Config.shooter.small.RPM_ERROR_THRESHOLD;
        if ("big".equals(name))   return Config.shooter.big.RPM_ERROR_THRESHOLD;
        return rpmErrorThreshold;
    }
    /** 設定轉速穩定誤差容許值並返回 this 支援鏈式調用 */
    public ShooterConfig setRpmErrorThreshold(double val) {
        this.rpmErrorThreshold = val;
        if ("small".equals(name)) Config.shooter.small.RPM_ERROR_THRESHOLD = val;
        if ("big".equals(name))   Config.shooter.big.RPM_ERROR_THRESHOLD = val;
        return this;
    }

    /** 取得仰角伺服角度手動微調偏移 (度) */
    public double getPitchOffsetDeg() {
        if ("small".equals(name)) return Config.shooter.small.PITCH_OFFSET_DEG;
        if ("big".equals(name))   return Config.shooter.big.PITCH_OFFSET_DEG;
        return pitchOffsetDeg;
    }
    /** 設定仰角伺服角度微調偏移並返回 this 支援鏈式調用 */
    public ShooterConfig setPitchOffsetDeg(double val) {
        this.pitchOffsetDeg = val;
        if ("small".equals(name)) Config.shooter.small.PITCH_OFFSET_DEG = val;
        if ("big".equals(name))   Config.shooter.big.PITCH_OFFSET_DEG = val;
        return this;
    }

    /** 取得出球口相對於機器底盤旋轉中心的前後安裝偏移 (in，向前為正) */
    public double getTurretForwardIn() {
        if ("small".equals(name)) return Config.shooter.small.TURRET_FORWARD_IN;
        if ("big".equals(name))   return Config.shooter.big.TURRET_FORWARD_IN;
        return turretForwardIn;
    }
    /** 設定出球口前後安裝偏移並返回 this 支援鏈式調用 */
    public ShooterConfig setTurretForwardIn(double val) {
        this.turretForwardIn = val;
        if ("small".equals(name)) Config.shooter.small.TURRET_FORWARD_IN = val;
        if ("big".equals(name))   Config.shooter.big.TURRET_FORWARD_IN = val;
        return this;
    }

    /** 取得出球口相對於機器底盤旋轉中心的左右安裝偏移 (in，向左為正) */
    public double getTurretLeftIn() {
        if ("small".equals(name)) return Config.shooter.small.TURRET_LEFT_IN;
        if ("big".equals(name))   return Config.shooter.big.TURRET_LEFT_IN;
        return turretLeftIn;
    }
    /** 設定出球口左右安裝偏移並返回 this 支援鏈式調用 */
    public ShooterConfig setTurretLeftIn(double val) {
        this.turretLeftIn = val;
        if ("small".equals(name)) Config.shooter.small.TURRET_LEFT_IN = val;
        if ("big".equals(name))   Config.shooter.big.TURRET_LEFT_IN = val;
        return this;
    }

    /** 取得場地重力加速度 (in/s^2，預設 386.09) */
    public double getG() {
        if ("small".equals(name)) return Config.shooter.small.G;
        if ("big".equals(name))   return Config.shooter.big.G;
        return g;
    }
    /** 設定場地重力加速度並返回 this 支援鏈式調用 */
    public ShooterConfig setG(double val) {
        this.g = val;
        if ("small".equals(name)) Config.shooter.small.G = val;
        if ("big".equals(name))   Config.shooter.big.G = val;
        return this;
    }

    /** 取得出球口與籃框目標的垂直高度差 (in) */
    public double getH() {
        if ("small".equals(name)) return Config.shooter.small.H;
        if ("big".equals(name))   return Config.shooter.big.H;
        return h;
    }
    /** 設定出球口與目標垂直高度差並返回 this 支援鏈式調用 */
    public ShooterConfig setH(double val) {
        this.h = val;
        if ("small".equals(name)) Config.shooter.small.H = val;
        if ("big".equals(name))   Config.shooter.big.H = val;
        return this;
    }

    /** 取得越過籃框前緣防撞矮牆的安全裕度 (in) */
    public double getFrontWallClearanceIn() {
        if ("small".equals(name)) return Config.shooter.small.FRONT_WALL_CLEARANCE_IN;
        if ("big".equals(name))   return Config.shooter.big.FRONT_WALL_CLEARANCE_IN;
        return frontWallClearanceIn;
    }
    /** 設定越過矮牆安全裕度並返回 this 支援鏈式調用 */
    public ShooterConfig setFrontWallClearanceIn(double val) {
        this.frontWallClearanceIn = val;
        if ("small".equals(name)) Config.shooter.small.FRONT_WALL_CLEARANCE_IN = val;
        if ("big".equals(name))   Config.shooter.big.FRONT_WALL_CLEARANCE_IN = val;
        return this;
    }

    /** 取得飛輪機構最高限制發射初速 (in/s) */
    public double getMaxVelocityInPerSec() {
        if ("small".equals(name)) return Config.shooter.small.MAX_VELOCITY_IN_PER_SEC;
        if ("big".equals(name))   return Config.shooter.big.MAX_VELOCITY_IN_PER_SEC;
        return maxVelocityInPerSec;
    }
    /** 設定最高限制發射初速並返回 this 支援鏈式調用 */
    public ShooterConfig setMaxVelocityInPerSec(double val) {
        this.maxVelocityInPerSec = val;
        if ("small".equals(name)) Config.shooter.small.MAX_VELOCITY_IN_PER_SEC = val;
        if ("big".equals(name))   Config.shooter.big.MAX_VELOCITY_IN_PER_SEC = val;
        return this;
    }

    /** 取得砲台正面向前時對應的舵機位置 (0.0 ~ 1.0) */
    public double getTurretCenter() {
        if ("small".equals(name)) return Config.shooter.small.TURRET_CENTER;
        if ("big".equals(name))   return Config.shooter.big.TURRET_CENTER;
        return turretCenter;
    }
    /** 設定砲台正向中心舵機位置並返回 this 支援鏈式調用 */
    public ShooterConfig setTurretCenter(double val) {
        this.turretCenter = val;
        if ("small".equals(name)) {
            Config.shooter.small.TURRET_CENTER = val;
            Config.shooter.small.YAW_CENTER = val;
        }
        if ("big".equals(name)) {
            Config.shooter.big.TURRET_CENTER = val;
            Config.shooter.big.YAW_CENTER = val;
        }
        return this;
    }

    /** 取得砲台舵機每單位控制位置對應的旋轉角度 (度/Pos) */
    public double getTurretDegPerPos() {
        if ("small".equals(name)) return Config.shooter.small.TURRET_DEG_PER_POS;
        if ("big".equals(name))   return Config.shooter.big.TURRET_DEG_PER_POS;
        return turretDegPerPos;
    }
    /** 設定砲台舵機每單位位置對應旋轉角度並返回 this 支援鏈式調用 */
    public ShooterConfig setTurretDegPerPos(double val) {
        this.turretDegPerPos = val;
        if ("small".equals(name)) {
            Config.shooter.small.TURRET_DEG_PER_POS = val;
            Config.shooter.small.YAW_DEG_PER_POS = val;
        }
        if ("big".equals(name)) {
            Config.shooter.big.TURRET_DEG_PER_POS = val;
            Config.shooter.big.YAW_DEG_PER_POS = val;
        }
        return this;
    }

    // 舊命名相容方法 (Backward Compatibility)
    public double getYawCenter() { return getTurretCenter(); }
    public ShooterConfig setYawCenter(double val) { return setTurretCenter(val); }

    public double getYawDegPerPos() { return getTurretDegPerPos(); }
    public ShooterConfig setYawDegPerPos(double val) { return setTurretDegPerPos(val); }

    /** 取得飛輪馬達單圈編碼器脈衝數 (Ticks/Rev) */
    public double getTicksPerRevolution() {
        if ("small".equals(name)) return Config.shooter.small.TICKS_PER_REVOLUTION;
        if ("big".equals(name))   return Config.shooter.big.TICKS_PER_REVOLUTION;
        return ticksPerRevolution;
    }
    /** 設定馬達單圈編碼器脈衝數並返回 this 支援鏈式調用 */
    public ShooterConfig setTicksPerRevolution(double val) {
        this.ticksPerRevolution = val;
        if ("small".equals(name)) Config.shooter.small.TICKS_PER_REVOLUTION = val;
        if ("big".equals(name))   Config.shooter.big.TICKS_PER_REVOLUTION = val;
        return this;
    }

    /** 取得飛輪傳動齒輪/皮帶速比 (直驅為 1.0) */
    public double getGearRatio() {
        if ("small".equals(name)) return Config.shooter.small.GEAR_RATIO;
        if ("big".equals(name))   return Config.shooter.big.GEAR_RATIO;
        return gearRatio;
    }
    /** 設定傳動速比並返回 this 支援鏈式調用 */
    public ShooterConfig setGearRatio(double val) {
        this.gearRatio = val;
        if ("small".equals(name)) Config.shooter.small.GEAR_RATIO = val;
        if ("big".equals(name))   Config.shooter.big.GEAR_RATIO = val;
        return this;
    }

    /** 取得飛輪實際物理半徑 (in) */
    public double getFlywheelRadiusIn() {
        if ("small".equals(name)) return Config.shooter.small.FLYWHEEL_RADIUS_IN;
        if ("big".equals(name))   return Config.shooter.big.FLYWHEEL_RADIUS_IN;
        return flywheelRadiusIn;
    }
    /** 設定飛輪物理半徑並返回 this 支援鏈式調用 */
    public ShooterConfig setFlywheelRadiusIn(double val) {
        this.flywheelRadiusIn = val;
        if ("small".equals(name)) Config.shooter.small.FLYWHEEL_RADIUS_IN = val;
        if ("big".equals(name))   Config.shooter.big.FLYWHEEL_RADIUS_IN = val;
        return this;
    }

    /** 取得仰角最小允許角度 (度) */
    public double getMinPitchAngleDeg() {
        if ("small".equals(name)) return Config.shooter.small.MIN_PITCH_ANGLE_DEG;
        if ("big".equals(name))   return Config.shooter.big.MIN_PITCH_ANGLE_DEG;
        return minPitchAngleDeg;
    }
    /** 設定仰角最小角度並返回 this 支援鏈式調用 */
    public ShooterConfig setMinPitchAngleDeg(double val) {
        this.minPitchAngleDeg = val;
        if ("small".equals(name)) Config.shooter.small.MIN_PITCH_ANGLE_DEG = val;
        if ("big".equals(name))   Config.shooter.big.MIN_PITCH_ANGLE_DEG = val;
        return this;
    }

    /** 取得仰角最大允許角度 (度) */
    public double getMaxPitchAngleDeg() {
        if ("small".equals(name)) return Config.shooter.small.MAX_PITCH_ANGLE_DEG;
        if ("big".equals(name))   return Config.shooter.big.MAX_PITCH_ANGLE_DEG;
        return maxPitchAngleDeg;
    }
    /** 設定仰角最大角度並返回 this 支援鏈式調用 */
    public ShooterConfig setMaxPitchAngleDeg(double val) {
        this.maxPitchAngleDeg = val;
        if ("small".equals(name)) Config.shooter.small.MAX_PITCH_ANGLE_DEG = val;
        if ("big".equals(name))   Config.shooter.big.MAX_PITCH_ANGLE_DEG = val;
        return this;
    }

    /** 取得仰角轉伺服位置之線性公式斜率 M (pos = M * deg + C) */
    public double getPitchServoM() {
        if ("small".equals(name)) return Config.shooter.small.PITCH_SERVO_M;
        if ("big".equals(name))   return Config.shooter.big.PITCH_SERVO_M;
        return pitchServoM;
    }
    /** 設定仰角轉伺服位置斜率 M 並返回 this 支援鏈式調用 */
    public ShooterConfig setPitchServoM(double val) {
        this.pitchServoM = val;
        if ("small".equals(name)) Config.shooter.small.PITCH_SERVO_M = val;
        if ("big".equals(name))   Config.shooter.big.PITCH_SERVO_M = val;
        return this;
    }

    /** 取得仰角轉伺服位置之線性公式截距 C (pos = M * deg + C) */
    public double getPitchServoC() {
        if ("small".equals(name)) return Config.shooter.small.PITCH_SERVO_C;
        if ("big".equals(name))   return Config.shooter.big.PITCH_SERVO_C;
        return pitchServoC;
    }
    /** 設定仰角轉伺服位置截距 C 並返回 this 支援鏈式調用 */
    public ShooterConfig setPitchServoC(double val) {
        this.pitchServoC = val;
        if ("small".equals(name)) Config.shooter.small.PITCH_SERVO_C = val;
        if ("big".equals(name))   Config.shooter.big.PITCH_SERVO_C = val;
        return this;
    }

    /** 取得仰角伺服最小限制 Position (防止機構卡死) */
    public double getPitchServoMin() {
        if ("small".equals(name)) return Config.shooter.small.PITCH_SERVO_MIN;
        if ("big".equals(name))   return Config.shooter.big.PITCH_SERVO_MIN;
        return pitchServoMin;
    }
    /** 設定仰角伺服最小限制 Position 並返回 this 支援鏈式調用 */
    public ShooterConfig setPitchServoMin(double val) {
        this.pitchServoMin = val;
        if ("small".equals(name)) Config.shooter.small.PITCH_SERVO_MIN = val;
        if ("big".equals(name))   Config.shooter.big.PITCH_SERVO_MIN = val;
        return this;
    }

    /** 取得仰角伺服最大限制 Position (防止機構脫軌) */
    public double getPitchServoMax() {
        if ("small".equals(name)) return Config.shooter.small.PITCH_SERVO_MAX;
        if ("big".equals(name))   return Config.shooter.big.PITCH_SERVO_MAX;
        return pitchServoMax;
    }
    /** 設定仰角伺服最大限制 Position 並返回 this 支援鏈式調用 */
    public ShooterConfig setPitchServoMax(double val) {
        this.pitchServoMax = val;
        if ("small".equals(name)) Config.shooter.small.PITCH_SERVO_MAX = val;
        if ("big".equals(name))   Config.shooter.big.PITCH_SERVO_MAX = val;
        return this;
    }

    /** 取得飛輪閉迴路速度 PIDF 控制係數 */
    public PIDFCoefficients getFlywheelPID() {
        if ("small".equals(name)) return Config.shooter.small.flywheelPID;
        if ("big".equals(name))   return Config.shooter.big.flywheelPID;
        return flywheelPID;
    }
    /** 設定飛輪閉迴路速度 PIDF 係數並返回 this 支援鏈式調用 */
    public ShooterConfig setFlywheelPID(PIDFCoefficients val) {
        this.flywheelPID = val;
        if ("small".equals(name)) Config.shooter.small.flywheelPID = val;
        if ("big".equals(name))   Config.shooter.big.flywheelPID = val;
        return this;
    }
}
