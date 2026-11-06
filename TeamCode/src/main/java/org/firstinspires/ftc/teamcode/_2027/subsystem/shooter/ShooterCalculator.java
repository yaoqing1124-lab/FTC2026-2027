package org.firstinspires.ftc.teamcode._2027.subsystem.shooter;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2027.util.ShooterConfig;

/**
 * 發射器彈道與物理計算器 (Shooter Ballistics & Trajectory Calculator)
 * 負責各式物理層轉換、虛擬座標變換、拋物線逆解、偏航角與調適 SOP。
 * 所有計算皆參數化接受 ShooterConfig，使多個發射器能完全共用計算模型而數據互不干擾。
 */
public class ShooterCalculator {

    /**
     * 從 Encoder Ticks 每秒轉換為 RPM
     */
    public static double TicksPerSecToRPM(double ticksPerSec, ShooterConfig config) {
        return (ticksPerSec / (config.getTicksPerRevolution() * config.getGearRatio())) * 60.0;
    }

    /**
     * 從 RPM 轉換為 Encoder Ticks 每秒
     */
    public static double RPMToTicksPerSec(double rpm, ShooterConfig config) {
        return (rpm / 60.0) * (config.getTicksPerRevolution() * config.getGearRatio());
    }

    /**
     * 階段 I：物理層轉換 (RPM → 初速)
     * 將馬達 RPM 轉換為球體出膛的真實線速度 (in/sec)。
     * 單飛輪 + 弧形軌道 (Hooded Shooter) 特性：球體出膛理論速度為飛輪表面線速度的一半。
     */
    public static double RPMToVelocity(double rpm, ShooterConfig config) {
        double rps = rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * config.getFlywheelRadiusIn());
        double theoreticalExitVelocity = surfaceVelocity / 2.0;
        return theoreticalExitVelocity * config.getEnergyConversionRate() + config.getIntakeInitialVelocityInPerSec();
    }

    /**
     * 階段 I：物理層轉換 (初速 → RPM)
     * 將數學模型算出的目標線速度 (in/sec) 轉換回目標 RPM 供馬達 PIDF 使用。
     */
    public static double VelocityToRPM(double velocity, ShooterConfig config) {
        double theoreticalExitVelocity = (velocity - config.getIntakeInitialVelocityInPerSec()) / config.getEnergyConversionRate();
        double requiredSurfaceVelocity = theoreticalExitVelocity * 2.0;
        double rps = requiredSurfaceVelocity / (2.0 * Math.PI * config.getFlywheelRadiusIn());
        return rps * 60.0;
    }

    /**
     * SOP 步驟 1：近距離測試 (求解 ENERGY_CONVERSION_RATE)
     * 在近距離 (例如 24 吋，忽略空氣阻力) 固定仰角為 45 度發射，
     * 調整至完美命中後，傳入該完美 RPM，此函式會回傳傳輸效率。
     *
     * @param distance_in 離目標的實際水平距離 (in)
     * @param height_in   與目標的垂直高度差 (in)
     * @param perfect_rpm 能在此距離完美命中目標的飛輪轉速
     * @param config      該發射器的設定物件
     * @return 該寫入該發射器 ENERGY_CONVERSION_RATE 的新數值
     */
    public static double calibrateEnergyConversionRate(double distance_in, double height_in, double perfect_rpm, ShooterConfig config) {
        double theta_rad = Math.toRadians(45.0);
        double tan_theta = Math.tan(theta_rad);
        double cos_theta = Math.cos(theta_rad);

        double denominator = 2.0 * Math.pow(cos_theta, 2) * (distance_in * tan_theta - height_in);
        if (denominator <= 0) {
            return 1.0;
        }
        double ideal_velocity = Math.sqrt((config.getG() * Math.pow(distance_in, 2)) / denominator);

        double rps = perfect_rpm / 60.0;
        double surfaceVelocity = rps * (2.0 * Math.PI * config.getFlywheelRadiusIn());
        double theoreticalExitVelocity = surfaceVelocity / 2.0;

        return ideal_velocity / theoreticalExitVelocity;
    }

    /**
     * 階段 III：虛擬座標變換 (數學模型層)
     * 透過 ENERGY_LOSS_RATE 產生一個「虛擬目標點 B'」，補償空氣阻力與 Magnus 效應。
     */
    public static ShooterPose calculateVirtualTarget(ShooterPose current, ShooterPose realTarget, ShooterConfig config) {
        double d = Math.hypot(realTarget.x - current.x, realTarget.y - current.y);

        double virtual_d = d * (1.0 + config.getEnergyLossRate() * d);
        double virtual_h = (realTarget.z - current.z) + (config.getEnergyLossRate() * Math.pow(d, 2));

        double turret_rad = Math.atan2(realTarget.y - current.y, realTarget.x - current.x);
        double virtual_x = current.x + virtual_d * Math.cos(turret_rad);
        double virtual_y = current.y + virtual_d * Math.sin(turret_rad);
        double virtual_z = current.z + virtual_h;

        return new ShooterPose(virtual_x, virtual_y, virtual_z, realTarget.turret);
    }

    /**
     * 階段 IV：計算理想初速 (前饋 Feedforward)
     * 拋射數學於物理坐標系進行 (0°=水平，90°=垂直)
     */
    public static double calculateIdealVelocity(ShooterPose current, ShooterPose virtualTarget, double lastVelocity, ShooterConfig config) {
        double d = Math.hypot(virtualTarget.x - current.x, virtualTarget.y - current.y);
        double h = virtualTarget.z - current.z;
        if (d < 1.0) return lastVelocity;

        // 舵機限制轉換為物理仰角限制 (從水平面起算)：
        // MIN_PITCH_SERVO (30°) → 物理 60° (最陡發射)
        // MAX_PITCH_SERVO (54°) → 物理 36° (最平發射)
        double physics_steepest = 90.0 - config.getMinPitchAngleDeg();
        double physics_flattest  = 90.0 - config.getMaxPitchAngleDeg();

        double d_cross = 2.0 * h / (Math.tan(Math.toRadians(physics_steepest))
                - Math.tan(Math.toRadians(config.getIdealIncidentAngleDeg())));
        double d_min   = h / Math.tan(Math.toRadians(physics_steepest));

        double launchPhysicsDeg;
        if (d >= d_cross) {
            launchPhysicsDeg = incidentToLaunchAngle(config.getIdealIncidentAngleDeg(), d, h);
        } else if (d <= d_min) {
            launchPhysicsDeg = physics_steepest;
        } else {
            double t = (d - d_min) / (d_cross - d_min);
            launchPhysicsDeg = physics_steepest - t * (physics_steepest - physics_flattest);
        }

        launchPhysicsDeg = Range.clip(launchPhysicsDeg, physics_flattest, physics_steepest);

        double theta_rad   = Math.toRadians(launchPhysicsDeg);
        double tan_theta   = Math.tan(theta_rad);
        double cos_theta   = Math.cos(theta_rad);
        double denominator = 2.0 * Math.pow(cos_theta, 2) * (d * tan_theta - h);

        double v_ideal;
        if (denominator > 0) {
            v_ideal = Math.sqrt((config.getG() * Math.pow(d, 2)) / denominator);
        } else {
            v_ideal = lastVelocity;
        }
        v_ideal = Math.min(v_ideal, config.getMaxVelocityInPerSec());

        // 前壁防撞框演算法 (Front wall clearance check)
        double d_fw = frontWallDistance(current, virtualTarget);
        if (d_fw > 0 && d_fw < d) {
            double h_at_fw = ballHeightAt(d_fw, v_ideal, launchPhysicsDeg, config.getG());
            if (h_at_fw < h + config.getFrontWallClearanceIn()) {
                double physicsAngle = launchPhysicsDeg;
                for (int i = 0; i < 20 && physicsAngle < physics_steepest; i++) {
                    physicsAngle = Math.min(physicsAngle + 1.0, physics_steepest);
                    double theta2  = Math.toRadians(physicsAngle);
                    double denom2  = 2.0 * Math.pow(Math.cos(theta2), 2) * (d * Math.tan(theta2) - h);
                    if (denom2 <= 0) continue;
                    double v2 = Math.sqrt(config.getG() * d * d / denom2);
                    if (ballHeightAt(d_fw, v2, physicsAngle, config.getG()) >= h + config.getFrontWallClearanceIn()) {
                        v_ideal = Math.min(v2, config.getMaxVelocityInPerSec());
                        break;
                    }
                }
            }
        }

        return v_ideal;
    }

    /**
     * 將理想入射角 (arrival angle) 轉換為發射仰角 (launch angle)。
     * 兩者均以物理慣例表示 (從水平面量測，0°=平射，90°=垂直)。
     */
    public static double incidentToLaunchAngle(double incidentAngleDeg, double d, double h) {
        if (d <= 0) return incidentAngleDeg;
        double tan_incident = Math.tan(Math.toRadians(incidentAngleDeg));
        double tan_launch   = tan_incident + (2.0 * h / d);
        return Math.toDegrees(Math.atan(tan_launch));
    }

    /**
     * 階段 V：計算實際仰角 (反饋 Feedback)
     * 根據飛輪真實轉速，反解拋物線二次方程式，求出能打中虛擬目標的實際仰角。
     */
    public static double calculateRealPitchAngle(ShooterPose current, ShooterPose virtualTarget, double actual_velocity, ShooterConfig config) {
        if (actual_velocity <= 0) {
            return config.getMaxPitchAngleDeg();
        }

        double d = Math.hypot(virtualTarget.x - current.x, virtualTarget.y - current.y);
        double h = virtualTarget.z - current.z;

        if (d < 1.0) return config.getMaxPitchAngleDeg();

        // 解: A·tan²θ - B·tanθ + C = 0 (θ 為物理慣例)
        double A = (config.getG() * Math.pow(d, 2)) / (2.0 * Math.pow(actual_velocity, 2));
        double B = d;
        double C = h + A;

        double discriminant = B * B - 4.0 * A * C;

        if (discriminant < 0) {
            return config.getMaxPitchAngleDeg();
        }

        double tan_theta_low = (B - Math.sqrt(discriminant)) / (2.0 * A);
        double theta_low_physics = Math.toDegrees(Math.atan(tan_theta_low));

        double physics_flattest  = 90.0 - config.getMaxPitchAngleDeg();
        double physics_steepest  = 90.0 - config.getMinPitchAngleDeg();

        double selected_physics = Range.clip(theta_low_physics, physics_flattest, physics_steepest);
        double selected_servo   = 90.0 - selected_physics;

        selected_servo += config.getPitchOffsetDeg();

        return Range.clip(selected_servo, config.getMinPitchAngleDeg(), config.getMaxPitchAngleDeg());
    }

    /**
     * 砲台方位角 (Turret Angle) 計算
     * 根據當前位置與目標位置，計算砲台所需的相對轉向角度 (°)，正值向右。
     */
    public static double calculateTurret(ShooterPose current, ShooterPose target) {
        double targetTurret_rad = Math.atan2(target.y - current.y, target.x - current.x);
        double heading_rad      = current.turret;
        double target_deg       = Math.toDegrees(targetTurret_rad - heading_rad);
        while (target_deg >  180) target_deg -= 360;
        while (target_deg < -180) target_deg += 360;
        return Range.clip(target_deg, -177.5, 177.5);
    }

    /**
     * 舊命名相容方法 (calculateYaw)
     */
    public static double calculateYaw(ShooterPose current, ShooterPose target) {
        return calculateTurret(current, target);
    }

    /**
     * 將機器人底盤中心 Pose 透過發射器的安裝位移 (TURRET_FORWARD_IN, TURRET_LEFT_IN)
     * 旋轉轉換至發射器出球口的真實場地座標。
     */
    public static ShooterPose currentPoseToTurretPose(ShooterPose currentPose, ShooterConfig config) {
        double heading = currentPose.turret;
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double fwd  = config.getTurretForwardIn();
        double left = config.getTurretLeftIn();

        double turretX = currentPose.x + fwd * cosH - left * sinH;
        double turretY = currentPose.y + fwd * sinH + left * cosH;

        return new ShooterPose(turretX, turretY, currentPose.z, currentPose.turret);
    }

    /**
     * 計算球體垂直飛行時間 (秒)，用於移動補償預測目標點
     */
    public static double flytimeVertical(double v, double pitchDeg, double h, double g) {
        if (v <= 0 || g <= 0) return 0;
        double vz = v * Math.cos(Math.toRadians(pitchDeg));
        double disc = vz * vz - 2.0 * g * h;
        if (disc < 0) return 0;
        double sqrt = Math.sqrt(disc);
        double tA = (vz + sqrt) / g;
        double tB = (vz - sqrt) / g;

        double t = 0;
        if (tA > 0 && tB > 0) t = Math.max(tA, tB);
        else if (tA > 0) t = tA;
        else if (tB > 0) t = tB;

        return t;
    }

    /**
     * 檢查發射軌跡上是否會遭遇目標前方矮牆 (支援場地中心 (0,0) 原點坐標系)
     * 場地邊界為 [-72, 72] in，四個角落分別為 (±72, ±72)。
     * 目標前壁為角落朝向場內 45 度切角，距離角落對角線偏移 FW_OFFSET。
     *
     * @param current 機器人出球口當前座標 (場地中心為 0,0)
     * @param target  發射目標座標 (場地中心為 0,0)
     * @return 若軌跡與前壁相交，回傳機器人到前壁的水平距離 (in)；若無阻擋則回傳 -1
     */
    public static double frontWallDistance(ShooterPose current, ShooterPose target) {
        final double FW_OFFSET = 18.3 / Math.sqrt(2.0); // 約 12.94 in (沿坐標軸單軸位移)
        final double CORNER_BOUND = 72.0;               // 中心原點系下場地邊界半長 (72 in)

        double rx = current.x, ry = current.y;
        double tx = target.x,  ty = target.y;

        // 判定目標所在的象限符號 (+1 或 -1)
        double sx = (tx >= 0) ? 1.0 : -1.0;
        double sy = (ty >= 0) ? 1.0 : -1.0;

        // 在中心原點系下，該角落 45 度切角前壁直線方程式為:
        // sx * x + sy * y = 2.0 * (CORNER_BOUND - FW_OFFSET)
        double fwConst = 2.0 * (CORNER_BOUND - FW_OFFSET);
        double denom   = sx * (tx - rx) + sy * (ty - ry);

        if (Math.abs(denom) < 1e-9) return -1; // 軌跡平行於前壁，無交點

        // 解射線參數 s: P(s) = R + s * (T - R), s 需介於 (0, 1) 表示前壁介於機器人與目標之間
        double s = (fwConst - (sx * rx + sy * ry)) / denom;

        if (s <= 0 || s >= 1) return -1; // 前壁不在機器人與目標的飛行軌跡區間內
        return s * Math.hypot(tx - rx, ty - ry);
    }

    /**
     * 在水平距離 x 處球體的飛行高度 (in)
     */
    public static double ballHeightAt(double x, double v, double physicsAngleDeg, double g) {
        if (v <= 0) return 0;
        double theta = Math.toRadians(physicsAngleDeg);
        return x * Math.tan(theta) - g * x * x / (2.0 * v * v * Math.pow(Math.cos(theta), 2));
    }
}
