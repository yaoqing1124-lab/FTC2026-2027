package org.firstinspires.ftc.teamcode.arch.op;

public abstract class OpBehavior {
    protected OpObject m_object;
    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();
}