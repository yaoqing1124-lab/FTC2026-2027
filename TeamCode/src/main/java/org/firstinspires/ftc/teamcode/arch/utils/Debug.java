package org.firstinspires.ftc.teamcode.arch.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

// wrapper for ftc telemetry
public class Debug extends OpBehavior {
    private final static Debug m_instance;
    private static Telemetry m_telemetry;
    public static String separator = " ";
    static {
        m_instance = new Debug();
        OpObject.RegisterGlobalBehavior(m_instance);
    }

    public static void Log(Object... messages){
        if(m_telemetry == null)
            return;

        StringBuilder sb = new StringBuilder();
        for(Object msg : messages){
            sb.append(msg).append(separator);
        }
        m_telemetry.addLine(sb.toString());
    }

    public static void LogIf(boolean isAllow, Object... messages){
        if(isAllow)
            Log(messages);
    }

    @Override
    public void Init() {
        m_telemetry = m_object.telemetry;
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        m_telemetry.update();
    }

    @Override
    public void Stop() {

    }
}
