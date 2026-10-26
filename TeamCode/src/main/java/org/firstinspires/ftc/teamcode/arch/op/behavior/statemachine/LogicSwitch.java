package org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine;

import java.util.function.BooleanSupplier;

public class LogicSwitch extends StateObject {
    private final BooleanSupplier m_condition;
    private final StateObject m_trueBranch;
    private final StateObject m_falseBranch;
    public LogicSwitch(BooleanSupplier condition, StateObject trueBranch, StateObject falseBranch) {
        m_condition = condition;
        m_trueBranch = trueBranch;
        m_falseBranch = falseBranch;
    }
    @Override
    public void Init(double timestamp) {
        nextStates.clear();
        if (m_condition.getAsBoolean()) {
            if (m_trueBranch != null) Then(m_trueBranch);
        } else {
            if (m_falseBranch != null) Then(m_falseBranch);
        }
    }
    @Override
    public void Loop(double timestamp, double deltaTime) {}
    @Override
    public boolean IsDone(double timestamp) {
        return true;
    }
}
