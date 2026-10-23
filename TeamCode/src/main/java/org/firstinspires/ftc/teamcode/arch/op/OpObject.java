package org.firstinspires.ftc.teamcode.arch.op;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.behavior.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.arch.utils.Debug;
import org.jetbrains.annotations.UnknownNullability;

import java.util.HashSet;
public abstract class OpObject extends OpMode {
    public static Team team;
    /*-------------------------------------------------------------
     * Static logic
     * -------------------------------------------------------------*/
    private final static HashSet<OpBehavior> s_globalBehaviors = new HashSet<>();
    public static void RegisterGlobalBehavior(OpBehavior behavior){
        s_globalBehaviors.add(behavior);
    }


    /*-------------------------------------------------------------
    * Instance logic
    * -------------------------------------------------------------*/
    private final HashSet<OpBehavior> m_behaviors = new HashSet<>();

    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();

    @Override
    public final void init() {
        Debug.Log("status : init");
        try {
            Class.forName("org.firstinspires.ftc.teamcode._2627.subsystem.Drivetrain");
            Class.forName("org.firstinspires.ftc.teamcode._2627.subsystem.Intake");
            Class.forName("org.firstinspires.ftc.teamcode._2627.subsystem.Trigger");
            Class.forName("org.firstinspires.ftc.teamcode._2627.subsystem.shooter.Shooter");
            Class.forName("org.firstinspires.ftc.teamcode.gc.utils.Debug");
        } catch (ClassNotFoundException ignored) {}

        Init();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Init();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.m_object = this;
            globalBehavior.Init();
        }
        Debug.Log("wait for start ...");
    }

    @Override
    public void start() {
        Start();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Start();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Start();
        }
    }

    @Override
    public final void loop() {
        Loop();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Loop();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Loop();
        }
    }

    @Override
    public final void stop() {
        Stop();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Stop();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Stop();
            globalBehavior.m_object = null;
        }
    }

    public final void AddBehavior(@UnknownNullability OpBehavior behavior){
        behavior.m_object = this;
        m_behaviors.add(behavior);
    }

    public final void RemoveBehavior(OpBehavior behavior) {
        if(behavior.m_object == this) {
            m_behaviors.remove(behavior);
            behavior.m_object = null;
        }
    }
}
