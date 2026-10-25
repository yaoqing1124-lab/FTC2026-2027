package org.firstinspires.ftc.teamcode._2027.state;

import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.util.Mode;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateObject;

public class Shoot extends StateObject {
    private boolean isShoot = false;
    public Shoot(boolean Do){
        isShoot = Do;
    }
    @Override
    public void Init(double timestamp) {

    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
        Intake.setMode(isShoot ? Mode.shoot : Mode.intake);
    }

    @Override
    public boolean IsDone(double timestamp) {
        return true;
    }
}
