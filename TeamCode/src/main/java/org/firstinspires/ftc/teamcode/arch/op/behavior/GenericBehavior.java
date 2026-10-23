package org.firstinspires.ftc.teamcode.arch.op.behavior;


import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;

public class GenericBehavior extends OpBehavior {
    public GenericBehavior(Runnable init, Runnable loop){
        m_init = init;
        m_loop = loop;
    }
    private final Runnable m_init;
    private final Runnable m_loop;
    @Override
    public final void Init() {
        m_init.run();
    }

    @Override
    public void Start() {

    }

    @Override
    public final void Loop() {
        m_loop.run();
    }

    @Override
    public void Stop() {

    }
}
