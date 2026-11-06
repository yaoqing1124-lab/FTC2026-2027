package org.firstinspires.ftc.teamcode._2027;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 發射器基準測試與單獨硬體調適 OpMode (Dual Shooter Tuning Bench)
 * 可同時或分別測試 Shooter 1 (Small) 與 Shooter 2 (Big) 的飛輪與伺服舵機。
 * 統一使用 turret 命名砲台水平轉向伺服。
 */
@Configurable
@TeleOp(name = "Shooter Setting Bench", group = "Tuning")
public class Setting extends OpMode {

    // --- Shooter 1 (Small) 調適目標 ---
    public static double targetRPM = 0;
    public static double turretPosition = 0.5;
    public static double yawPosition = turretPosition; // 相容別名
    public static double pitchPosition = 0;
    public static PIDFCoefficients FLYWHEEL_PID = new PIDFCoefficients(1.0, 0, 0, 0);

    // --- Shooter 2 (Big) 調適目標 ---
    public static double targetRPM2 = 0;
    public static double turretPosition2 = 0.5;
    public static double yawPosition2 = turretPosition2; // 相容別名
    public static double pitchPosition2 = 0;
    public static PIDFCoefficients FLYWHEEL2_PID = new PIDFCoefficients(1.0, 0, 0, 0);

    private TelemetryManager telemetryM;
    private Servo turret1, pitch1;
    private DcMotorEx flywheel1;

    private Servo turret2, pitch2;
    private DcMotorEx flywheel2;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // 初始化 Shooter 1 (Small)
        try {
            turret1 = hardwareMap.get(Servo.class, Config.hardware.servo.turret1.id);
        } catch (Exception ignored) {}
        try {
            pitch1 = hardwareMap.get(Servo.class, Config.hardware.servo.pitch1.id);
        } catch (Exception ignored) {}
        try {
            flywheel1 = hardwareMap.get(DcMotorEx.class, Config.hardware.motor.flyWheel1.id);
            flywheel1.setDirection(Config.hardware.motor.flyWheel1.direction);
            flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {}

        // 初始化 Shooter 2 (Big)
        try {
            turret2 = hardwareMap.get(Servo.class, Config.hardware.servo.turret2.id);
        } catch (Exception ignored) {}
        try {
            pitch2 = hardwareMap.get(Servo.class, Config.hardware.servo.pitch2.id);
        } catch (Exception ignored) {}
        try {
            flywheel2 = hardwareMap.get(DcMotorEx.class, Config.hardware.motor.flyWheel2.id);
            flywheel2.setDirection(Config.hardware.motor.flyWheel2.direction);
            flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {}
    }

    @Override
    public void loop() {
        // --- 驅動 Shooter 1 ---
        if (flywheel1 != null) {
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PID);
            flywheel1.setVelocity((targetRPM / 60.0) * Config.shooter.small.TICKS_PER_REVOLUTION);
            double currentRPM1 = (flywheel1.getVelocity() / Config.shooter.small.TICKS_PER_REVOLUTION) * 60.0;
            telemetryM.addData("Shooter1 Target RPM", targetRPM);
            telemetryM.addData("Shooter1 Actual RPM", currentRPM1);
        }
        if (turret1 != null) turret1.setPosition(turretPosition);
        if (pitch1 != null) pitch1.setPosition(pitchPosition);

        // --- 驅動 Shooter 2 ---
        if (flywheel2 != null) {
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL2_PID);
            flywheel2.setVelocity((targetRPM2 / 60.0) * Config.shooter.big.TICKS_PER_REVOLUTION);
            double currentRPM2 = (flywheel2.getVelocity() / Config.shooter.big.TICKS_PER_REVOLUTION) * 60.0;
            telemetryM.addData("Shooter2 Target RPM", targetRPM2);
            telemetryM.addData("Shooter2 Actual RPM", currentRPM2);
        }
        if (turret2 != null) turret2.setPosition(turretPosition2);
        if (pitch2 != null) pitch2.setPosition(pitchPosition2);

        telemetryM.update();
    }

    @Override
    public void stop() {
        if (flywheel1 != null) flywheel1.setPower(0);
        if (flywheel2 != null) flywheel2.setPower(0);
    }
}
