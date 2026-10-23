package org.firstinspires.ftc.teamcode._2027.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorConfig {
    public String id;
    public DcMotor.Direction direction;
    public MotorConfig(String id, DcMotor.Direction direction){
        this.id = id;
        this.direction = direction;
    }
}
