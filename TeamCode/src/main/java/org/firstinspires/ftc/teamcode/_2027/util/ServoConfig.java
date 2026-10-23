package org.firstinspires.ftc.teamcode._2027.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoConfig {
    public String id;
    public double offset;
    public Servo.Direction direction;
    public ServoConfig(String id){
        this.id = id;
    }
    public ServoConfig(String id, double offset){
        this.id = id;
        this.offset = offset;
    }
    public ServoConfig(String id, Servo.Direction direction, double offset){
        this.id = id;
        this.direction = direction;
        this.offset = offset;
    }

}
