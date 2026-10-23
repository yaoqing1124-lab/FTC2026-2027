package org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;

import java.util.HashSet;

public class StateMachine extends OpBehavior {
    private final ElapsedTime m_timer = new ElapsedTime();
    private final HashSet<StateObject> m_activeStates = new HashSet<>();
    private final HashSet<StateObject> m_statesToAdd = new HashSet<>();
    private final HashSet<StateObject> m_statesToRemove = new HashSet<>();
    private boolean m_isIterating = false;

    @Override
    public void Init() {
        m_timer.reset();
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        double timestamp = m_timer.milliseconds();

        m_isIterating = true;
        // state lifecycle
        for(StateObject state : m_activeStates){
            if(timestamp - state.m_lastLoopTime >= state.loopIntervalMsec){
                double deltaTime = timestamp - state.m_lastLoopTime;
                state.Loop(timestamp, deltaTime);
                state.m_lastLoopTime = timestamp;
            }
            if(state.IsDone(timestamp)){
                Done(state);
                continue;
            }
            if(state.timeoutMsec >= 0 && timestamp - state.m_startTime >= state.timeoutMsec){
                if(state.isNormalTimeout)
                    Done(state);
                else
                    Interrupt(state);
                continue;
            }
        }
        m_isIterating = false;

        // Apply deferred modifications
        m_activeStates.removeAll(m_statesToRemove);
        m_activeStates.addAll(m_statesToAdd);

        m_statesToRemove.clear();
        m_statesToAdd.clear();
    }

    @Override
    public void Stop() {

    }

    public final boolean Run(StateObject state){
        if(state.m_machine != null)
            return false;
        state.m_machine = this;

        if (m_isIterating) {
            m_statesToAdd.add(state);
        } else {
            m_activeStates.add(state);
        }

        double currentTimestamp = m_timer.milliseconds();
        state.Init(currentTimestamp);
        state.m_startTime = currentTimestamp;
        state.m_lastLoopTime = currentTimestamp;
        return true;
    }

    /** Mark a state is done no matter is it actually done */
    public final void Done(StateObject state){
        if(!IsActive(state))
            return;

        if (m_isIterating) {
            m_statesToRemove.add(state);
        } else {
            m_activeStates.remove(state);
        }

        state.m_machine = null;
        for(StateObject nextState : state.nextStates){
            Run(nextState);
            nextState.m_prevState = state;
        }
    }

    /** Interrupt a state, it will not invoke next state */
    public final void Interrupt(StateObject state) {
        if(!IsActive(state))
            return;

        if (m_isIterating) {
            m_statesToRemove.add(state);
        } else {
            m_activeStates.remove(state);
        }

        state.m_machine = null;
    }
    public final boolean IsActive(StateObject state){
        if(state.m_machine != this)
            return false;
        return m_activeStates.contains(state);
    }

}