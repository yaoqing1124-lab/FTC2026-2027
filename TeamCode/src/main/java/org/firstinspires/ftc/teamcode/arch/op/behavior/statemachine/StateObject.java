package org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine;

import java.util.HashSet;
import java.util.List;

public abstract class StateObject {
    protected StateMachine m_machine = null;
    protected StateObject m_prevState = null;
    public final HashSet<StateObject> nextStates = new HashSet<>();
    public double timeoutMsec = -1;
    public double loopIntervalMsec = 20;
    public boolean isNormalTimeout = true;
    protected double m_startTime = -1;
    protected double m_lastLoopTime = -1;

    public abstract void Init(double timestamp);

    /**
     * Called during the state machine's loop if the interval has elapsed.
     * By default, this calls the parameter-less Loop(). Override this to use time information.
     *
     * @param timestamp The current timestamp in milliseconds.
     * @param deltaTime The time in milliseconds since the last loop iteration for this state.
     */
    public abstract void Loop(double timestamp, double deltaTime);

    public abstract boolean IsDone(double timestamp);
    public final void Interrupt(){
        if(m_machine != null)
            m_machine.Interrupt(this);
    }

    /** Add to nextStates and return the first one, allow to chain invoke */
    public final StateObject Then(StateObject... state){
        nextStates.addAll(List.of(state));
        return state[0];
    }
}
