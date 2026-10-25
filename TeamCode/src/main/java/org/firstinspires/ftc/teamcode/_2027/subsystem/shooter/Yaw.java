package org.firstinspires.ftc.teamcode._2027.subsystem.shooter;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2027.Config;
import org.firstinspires.ftc.teamcode._2027.util.ServoConfig;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

public class Yaw extends OpBehavior {
    private Yaw(){}
    private final static Yaw m_instance;
    static {
        m_instance = new Yaw();
        OpObject.RegisterGlobalBehavior(m_instance);
    }
    private static Servo m_yaw;

    @Override
    public void Init() {
        m_yaw = init_servo(Config.hardware.servo.yaw);
        m_yaw.setPosition(0.5);
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        update_yaw();
    }

    @Override
    public void Stop() {
        m_yaw.setPosition(Config.shooter.hardware.YAW_CENTER);
        m_yaw = null;
    }
    private Servo init_servo(ServoConfig config) {
        Servo ret = m_object.hardwareMap.get(Servo.class, config.id);
        ret.setDirection(config.direction);
        return ret;
    }
    private static void update_yaw() {
        double target_deg   = ShooterCalculator.calculateYaw(Flywheel.current_position, Flywheel.predict_target_position);
        // Turret neutral (servo 0.5) faces BACKWARD — offset by 180° from robot forward
        target_deg = target_deg - 180.0;
        while (target_deg >  180) target_deg -= 360;
        while (target_deg < -180) target_deg += 360;
        final double center       = Config.shooter.hardware.YAW_CENTER;
        final double deg2position = (1 - 0.5) / 177.5; // 0.00395 per degree
        double targetPosition = center;
        if (Double.isNaN(target_deg) || Double.isInfinite(target_deg)) return;
        targetPosition = Range.clip(center - target_deg * deg2position, 0, 1);
        if (m_yaw != null) {
            m_yaw.setPosition(targetPosition);
        }
    }

}
