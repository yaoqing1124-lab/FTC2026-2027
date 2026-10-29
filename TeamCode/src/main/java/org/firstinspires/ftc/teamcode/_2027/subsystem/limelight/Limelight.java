package org.firstinspires.ftc.teamcode._2027.subsystem.limelight;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode._2027.Config;
import org.firstinspires.ftc.teamcode._2027.util.Camera;
import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends OpBehavior {
    private static Limelight3A limelight;
    private static LLResult result;
    private static Camera mode = Camera.none;
    private Limelight(){}
    private final static Limelight m_instance;
    static {
        m_instance = new Limelight();
        OpObject.RegisterGlobalBehavior(m_instance);
    }
    @Override
    public void Init() {
        limelight = m_object.hardwareMap.get(Limelight3A.class, Config.hardware.sensor.limelight.id);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void Start() {

    }

    @Override
    public void Loop() {
        result = limelight.getLatestResult();
        switch (mode){
            case apriltag:
                limelight.pipelineSwitch(Config.LLvalue.pipeline.apriltag); // Switch to pipeline number 0
                break;
            case detector:
                limelight.pipelineSwitch(Config.LLvalue.pipeline.detector);
                break;
            case python:
                limelight.pipelineSwitch(Config.LLvalue.pipeline.python);
                break;
            case none:
                break;
        }
    }

    @Override
    public void Stop() {
        limelight = null;
        result    = null;
    }
    public static class apriltag{
        public static ApriltagResult get(){
            mode = Camera.apriltag;
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    Pose  pedroPose = new Pose(botpose.getPosition().x, botpose.getPosition().y);
                    return new ApriltagResult(result.getTx(), result.getTy(), result.getTa(), pedroPose);
                }
            }
            return null;
        }
    }
    public static class detector{
        public static DetectorResult get(){
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            ArrayList<Pose>   targetPose = new ArrayList<>();
            ArrayList<String> targetName = new ArrayList<>();
            for (LLResultTypes.DetectorResult detection : detections) {
                targetPose.add(new Pose(detection.getTargetXDegrees(), detection.getTargetYDegrees()));
                targetName.add(detection.getClassName());
            }
            return new DetectorResult(targetPose, targetName);
        }
    }
    public static class python{
        private static double[] m_input = new double[8];
        public static void setInput(double[] input){
            mode = Camera.python;
            m_input = input;
        }
        public static double[] getPythonOutput(double[] inputs){
            mode = Camera.python;
            limelight.updatePythonInputs(inputs);
            return result.getPythonOutput();
        }
        public static double[] getPythonOutput(){
            mode = Camera.python;
            limelight.updatePythonInputs(m_input);
            return result.getPythonOutput();
        }
    }
}
