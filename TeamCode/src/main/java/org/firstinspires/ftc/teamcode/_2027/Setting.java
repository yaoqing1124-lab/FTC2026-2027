package org.firstinspires.ftc.teamcode._2027;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class Setting extends OpMode {
    public static class flywheelValue{
        static double targetRPM;
    }
    public static class servoPosition{
        static double yawPosition = 0.5;
        static double pitchPosition = 0;
    }
    TelemetryManager telemetryM;
    PIDFCoefficients FLYWHEEL_PID = new PIDFCoefficients(0,0,0,0);
    Servo yaw, pitch;
    DcMotorEx flywheel;

    @Override
    public void init() {
        yaw   = hardwareMap.get(Servo.class, "yaw");
        pitch = hardwareMap.get(Servo.class, "pitch");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PID);
        flywheel.setVelocity(flywheelValue.targetRPM / 28 * 60);
        yaw.setPosition(servoPosition.yawPosition);
        pitch.setPosition(servoPosition.pitchPosition);
        telemetryM.addData("RPM", flywheel.getVelocity() / 60 * 28);
        telemetryM.update();
    }
}
