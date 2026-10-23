package org.firstinspires.ftc.teamcode._2027.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
public class Intake extends OpBehavior {
    DcMotorEx intake, controlIntake;

    private Intake(){}
    private final static Intake m_instance;
    static {
        m_instance = new Intake();
        OpObject.RegisterGlobalBehavior(m_instance);
    }
    public enum Mode{
        intake, shoot, brake;
    }
    private static Mode m_mode;

    @Override
    public void Init() {
        intake = init(Config.hardware.motor.intake.id, Config.hardware.motor.intake.direction);
        controlIntake = init(Config.hardware.motor.intakeCtrl.id, Config.hardware.motor.intakeCtrl.direction);
        m_mode = Mode.brake;
    }

    @Override
    public void Start() {
    }

    @Override
    public void Loop() {
        switch (m_mode){
            case intake:
                intake.setPower(Config.value.INTAKE_POWER);
                controlIntake.setPower(Config.value.INTAKE_POWER);
                break;
            case shoot:
                intake.setPower(Config.value.INTAKE_POWER);
                controlIntake.setPower(-Config.value.INTAKE_POWER);
                break;
            case brake:
                intake.setPower(0);
                controlIntake.setPower(0);
                break;
        }
    }

    @Override
    public void Stop() {
        intake = null;
        controlIntake = null;
    }

    private DcMotorEx init(String name,  DcMotorSimple.Direction direction){
        DcMotorEx motor;
        motor = m_object.hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
        return motor;
    }
    public static void setMode(Mode mode){
        m_mode = mode;
    }
    public static Mode getMode(){
        return m_mode;
    }
}
