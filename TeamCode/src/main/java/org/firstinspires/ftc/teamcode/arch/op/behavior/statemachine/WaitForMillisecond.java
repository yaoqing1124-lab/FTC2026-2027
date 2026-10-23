package org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine;

public class WaitForMillisecond extends StateObject{
    public final double duration;
    public WaitForMillisecond(double duration){
        super();
        loopIntervalMsec = 1;
        this.duration = duration;
    }
    @Override
    public void Init(double timestamp) {

    }

    @Override
    public void Loop(double deltaTime, double timestamp) {

    }

    @Override
    public boolean IsDone(double timestamp) {
        return (timestamp - m_startTime) >= duration;
    }
}
