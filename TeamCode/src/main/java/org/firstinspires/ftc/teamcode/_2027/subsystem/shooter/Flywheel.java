package org.firstinspires.ftc.teamcode._2027.subsystem.shooter;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode._2027.Config;
import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.util.Mode;
import org.firstinspires.ftc.teamcode._2027.util.MotorConfig;
import org.firstinspires.ftc.teamcode._2027.util.ServoConfig;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

public class Flywheel extends OpBehavior {
    private Flywheel(){}
    private final static Flywheel m_instance;
    static {
        m_instance = new Flywheel();
        OpObject.RegisterGlobalBehavior(m_instance);
    }

    public static Mode mode = Intake.getMode();
    public static void resetPitch(){

    }
    public static double targetRPM = 0;
    public static double actualRPM = 0;
    public static boolean enablePredict = true;

    public final static ShooterPose current_position = new ShooterPose();
    public final static ShooterPose target_position         = new ShooterPose();
    public final static ShooterPose predict_target_position = new ShooterPose();

    private static DcMotorEx m_flyWheel;
    private static Servo m_pitch;

    private final PIDFCoefficients m_flywheelController = Config.shooter.hardware.flywheelController;
    private static Follower m_follower = null;

    private double m_lastTargetVelocity = 0;
    private double m_lastTargetPitch = 40.0;
    private static final int RPM_BUFFER_SIZE = 7;
    private static final double[] m_rpmBuffer = new double[RPM_BUFFER_SIZE];
    private static int m_rpmBufferIdx = 0;
    private static boolean m_rpmBufferFull = false;

    public static boolean IsStable(){
        return Math.abs(targetRPM - actualRPM) < Config.shooter.RPM_ERROR_THRESHOLD;
    }

    @Override
    public void Init() {
        m_flyWheel = init_flyWheel(Config.hardware.motor.flyWheel);
        m_pitch     = init_servo(Config.hardware.servo.pitch);
    }

    @Override
    public void Start() {
        m_follower = Drivetrain.get_Follower();
        aimTarget = (OpObject.team == Team.blue) ?
                (m_follower.getPose().getY() > 0 ? Config.field.targetPose.bluePositive : Config.field.targetPose.blueNegative) :
                (m_follower.getPose().getY() > 0 ? Config.field.targetPose.redPositive  : Config.field.targetPose.redNegative);

        target_position.x = aimTarget.getX();
        target_position.y = aimTarget.getY();
        target_position.z = Config.shooter.physical.H;
        resetPitch();
    }
    Pose aimTarget;
    @Override
    public void Loop() {
        //設定瞄準座標 防止在 y = 0 附近一直擺動
        double thershold = 24.0;
        switch (OpObject.team){
            case blue:
                if(m_follower.getPose().getY() > 0){
                    aimTarget = (Math.abs(m_follower.getPose().getY()) > thershold) ? Config.field.targetPose.bluePositive : aimTarget;
                }else{
                    aimTarget = (Math.abs(m_follower.getPose().getY()) > thershold) ? Config.field.targetPose.blueNegative : aimTarget;
                }
                break;
            case red:
                if(m_follower.getPose().getY() > 0){
                    aimTarget = (Math.abs(m_follower.getPose().getY()) > thershold) ? Config.field.targetPose.redPositive : aimTarget;
                }else{
                    aimTarget = (Math.abs(m_follower.getPose().getY()) > thershold) ? Config.field.targetPose.redNegative : aimTarget;
                }
                break;
        }
        target_position.x = aimTarget.getX();
        target_position.y = aimTarget.getY();
        target_position.z = Config.shooter.physical.H;

        Pose current_pose = m_follower.getPose();
        ShooterPose rawPose = new ShooterPose(
                current_pose.getX(),
                current_pose.getY(),
                0,
                current_pose.getHeading());
        ShooterPose turretPose = currentPoseToTurretPose(rawPose);
        current_position.x   = turretPose.x;
        current_position.y   = turretPose.y;
        current_position.yaw = turretPose.yaw;

        // --- Predict target (motion compensation) ---
        if(enablePredict){
            Vector rv = m_follower.getVelocity();
            double ft = flytimeVertical(m_lastTargetVelocity, m_lastTargetPitch, Config.shooter.physical.H);
            predict_target_position.x = target_position.x - rv.getXComponent() * ft;
            predict_target_position.y = target_position.y - rv.getYComponent() * ft;
            predict_target_position.z = target_position.z;
        } else {
            predict_target_position.x = target_position.x;
            predict_target_position.y = target_position.y;
            predict_target_position.z = target_position.z;
        }

        double rawRPM = (m_flyWheel != null)
                ? ShooterCalculator.TicksPerSecToRPM(m_flyWheel.getVelocity())
                : 0;
        m_rpmBuffer[m_rpmBufferIdx] = rawRPM;
        m_rpmBufferIdx = (m_rpmBufferIdx + 1) % RPM_BUFFER_SIZE;
        if (m_rpmBufferIdx == 0) m_rpmBufferFull = true;
        int count = m_rpmBufferFull ? RPM_BUFFER_SIZE : m_rpmBufferIdx;
        double sum = 0;
        for (int i = 0; i < count; i++) sum += m_rpmBuffer[i];
        actualRPM = (count > 0) ? sum / count : rawRPM;

        double actualVelocityInPerSec = ShooterCalculator.RPMToVelocity(actualRPM);

        ShooterPose virtualTarget = ShooterCalculator.calculateVirtualTarget(
                current_position, predict_target_position);

        double targetVelocityInPerSec = ShooterCalculator.calculateIdealVelocity(
                current_position, virtualTarget, m_lastTargetVelocity);
        targetRPM = ShooterCalculator.VelocityToRPM(targetVelocityInPerSec);


        m_lastTargetVelocity = targetVelocityInPerSec;

        double targetPitchDeg = ShooterCalculator.calculateRealPitchAngle(
                current_position, virtualTarget, targetVelocityInPerSec);




        update_pitch(targetPitchDeg);
        update_flyWheel(m_flyWheel, m_flywheelController, targetRPM);
    }

    @Override
    public void Stop() {
        if (m_flyWheel != null) m_flyWheel.setPower(0);
        m_pitch.setPosition(0.825);
        m_flyWheel = null;
        m_pitch     = null;
        m_follower  = null;
    }
    private DcMotorEx init_flyWheel(MotorConfig config) {
        DcMotorEx ret = m_object.hardwareMap.get(DcMotorEx.class, config.id);
        ret.setDirection(config.direction);
        ret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ret.setPower(0);
        return ret;
    }

    private Servo init_servo(ServoConfig config) {
        Servo ret = m_object.hardwareMap.get(Servo.class, config.id);
        ret.setDirection(config.direction);
        return ret;
    }
    private void update_flyWheel(DcMotorEx motor, PIDFCoefficients controller, double targetRPM) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, controller);
        double targetVelocity = (targetRPM / 60) * Config.shooter.hardware.TICKS_PER_REVOLUTION;
        motor.setVelocity(targetVelocity);
    }

    private static void update_pitch(double targetPitchDeg) {
        if (m_pitch == null) return;
        if (Double.isNaN(targetPitchDeg) || Double.isInfinite(targetPitchDeg)) return;
        double position = Range.clip(-0.0383333 * targetPitchDeg + 2.15, 0.08, 1);
        m_pitch.setPosition(position);
    }



    private ShooterPose currentPoseToTurretPose(ShooterPose currentPose) {
        double heading = currentPose.yaw;
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double fwd  = Config.shooter.TURRET_FORWARD_IN;
        double left = Config.shooter.TURRET_LEFT_IN;

        // Rotate offset from robot frame into field frame
        // field_x += fwd*cos(h) - left*sin(h)
        // field_y += fwd*sin(h) + left*cos(h)
        double turretX = currentPose.x + fwd * cosH - left * sinH;
        double turretY = currentPose.y + fwd * sinH + left * cosH;

        return new ShooterPose(turretX, turretY, currentPose.z, currentPose.yaw);
    }
    private double flytimeVertical(double v, double pitchDeg, double h) {
        // v in in/sec, h in inches, g in in/sec^2
        if (v <= 0) return 0;
        double g = Config.shooter.physical.G;
        double vz = v * Math.cos(Math.toRadians(pitchDeg)); // servo convention: cos not sin
        double disc = vz * vz - 2.0 * g * h;
        if (disc < 0) return 0;
        double sqrt = Math.sqrt(disc);
        double tA = (vz + sqrt) / g;
        double tB = (vz - sqrt) / g;

        // choose the descending (larger) positive root
        double t = 0;
        if (tA > 0 && tB > 0) t = Math.max(tA, tB);
        else if (tA > 0) t = tA;
        else if (tB > 0) t = tB;

        return t;
    }
}