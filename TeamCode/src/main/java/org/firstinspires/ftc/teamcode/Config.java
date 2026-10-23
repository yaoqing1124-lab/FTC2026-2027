package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode._2027.util.AnalogConfig;
import org.firstinspires.ftc.teamcode._2027.util.MotorConfig;
import org.firstinspires.ftc.teamcode._2027.util.ServoConfig;

public class Config {
    // =======================
    // 機器動作參數設定
    // =======================
    public static class value{
        public final static double INTAKE_POWER = 1;
    }
    // =======================
    // 硬體設定
    // =======================
    public static class hardware{
        public static class motor{
            public final static MotorConfig lfm = new MotorConfig("lfm", DcMotor.Direction.FORWARD);
            public final static MotorConfig lbm = new MotorConfig("lbm", DcMotor.Direction.FORWARD);
            public final static MotorConfig rfm = new MotorConfig("rfm", DcMotor.Direction.FORWARD);
            public final static MotorConfig rbm = new MotorConfig("rbm", DcMotor.Direction.FORWARD);
            public final static MotorConfig flyWheel = new MotorConfig("flywheel", DcMotorSimple.Direction.REVERSE);
            public final static MotorConfig intake = new MotorConfig("intake", DcMotorSimple.Direction.REVERSE);
            public final static MotorConfig intakeCtrl = new MotorConfig("intakeCtrl", DcMotorSimple.Direction.FORWARD);
        }
        public static class servo{
            public final static ServoConfig lfs = new ServoConfig("lfs");
            public final static ServoConfig lbs = new ServoConfig("lbs");
            public final static ServoConfig rfs = new ServoConfig("rfs");
            public final static ServoConfig rbs = new ServoConfig("rbs");
        }
        public static class sensor{
            public final static AnalogConfig lfenc = new AnalogConfig("lfenc");
            public final static AnalogConfig lbenc = new AnalogConfig("lbenc");
            public final static AnalogConfig rfenc = new AnalogConfig("rfenc");
            public final static AnalogConfig rbenc = new AnalogConfig("rbenc");
        }
    }
    // =======================
    // 飛輪子系統調適參數
    // =======================
    public static class flywheel{

    }
    // =======================
    // 底盤調適參數
    // =======================
    public static class odometry {
        public final static String id = "pinpoint"; // odometry configure name
        public final static double forwardOffset = 0; // unit:(mm)
        public final static double strafeOffset = 0; // unit:(mm)
        public final static GoBildaPinpointDriver.EncoderDirection forwardDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public final static GoBildaPinpointDriver.EncoderDirection strafeDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
