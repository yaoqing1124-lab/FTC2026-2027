package org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine;

import java.util.function.BooleanSupplier;

public class GenericState extends StateObject{
    private final Runnable m_init;
    private final Runnable m_loop;
    private final BooleanSupplier m_isDone;
    public GenericState(Runnable init, Runnable loop, BooleanSupplier isDone){
        m_init   = init;
        m_loop   = loop;
        m_isDone = isDone;
    }

    @Override
    public void Init(double timestamp) {
        m_init.run();
    }

    @Override
    public void Loop(double timestamp, double deltaTime) {
        m_loop.run();
    }

    @Override
    public boolean IsDone(double timestamp) {
        return m_isDone.getAsBoolean();
    }
}
