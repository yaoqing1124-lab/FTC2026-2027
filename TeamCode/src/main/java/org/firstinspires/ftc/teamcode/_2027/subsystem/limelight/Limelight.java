//package org.firstinspires.ftc.teamcode._2027.subsystem.limelight;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode._2027.Config;
//import org.firstinspires.ftc.teamcode.arch.op.OpBehavior;
//import org.firstinspires.ftc.teamcode.arch.op.OpObject;
//
//import java.util.ArrayList;
//
//public class Limelight extends OpBehavior {
//    private Limelight(){}
//    private final static Limelight m_instance;
//    static {
//        m_instance = new Limelight();
//        OpObject.RegisterGlobalBehavior(m_instance);
//    }
//    private static Limelight3A m_limelight;
//    private static LLResult result;
//    public enum Mode {
//        python, apriltag, megatag
//    }
//    private static Mode mode;
//    @Override
//    public void Init() {
//        m_limelight = m_object.hardwareMap.get(Limelight3A.class, Config.hardware.sensor.limelight.id);
//        m_limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        m_limelight.start(); // This tells Limelight to start looking!
//    }
//
//    @Override
//    public void Start() {
//        m_limelight.pipelineSwitch(0);
//    }
//
//    @Override
//    public void Loop() {
//        switch(mode){
//            case python:
//                m_limelight.pipelineSwitch(0);
//            case apriltag:
//                m_limelight.pipelineSwitch(1);
//                result = m_limelight.getLatestResult();
//
//        }
//    }
//
//    @Override
//    public void Stop() {
//        m_limelight = null;
//    }
//    public static void setMode(Mode mode){Limelight.mode = mode;}
//    public static double getX(){
//        mode = Mode.apriltag;
//        return result.getTx();
//    }
//    public static double getY(){
//        mode = Mode.apriltag;
//        return result.getTy();
//    }
//
//    /**
//     * 最多8個數字
//     * @param input
//     */
//    public static void setPythonInput(double[] input){
//        mode = Mode.python;
//        m_limelight.updatePythonInputs(input);
//    }
//    public static double[] getPythonOutput(){
//        mode = Mode.python;
//        return result.getPythonOutput();
//    }
//    //應該用不到
//    public static Pose getMegaTagPose(){
//        mode = Mode.megatag;
//        Double x = null, y = null;
//        if (result != null && result.isValid()) {
//            Pose3D botpose = result.getBotpose();
//            if (botpose != null) {
//                x = botpose.getPosition().x;
//                y = botpose.getPosition().y;
//            }
//        }
//        return new Pose(x, y);
//    }
////    public static Pose getMegaTagPose(IMU imu){
////        mode = Mode.apriltag;
////        Double x = null, y = null;
////        double robotYaw = imu.getAngularOrientation().firstAngle;
////        m_limelight.updateRobotOrientation(robotYaw);
////        if (result != null && result.isValid()) {
////            Pose3D botpose_mt2 = result.getBotpose_MT2();
////            if (botpose_mt2 != null) {
////                x = botpose_mt2.getPosition().x;
////                y = botpose_mt2.getPosition().y;
////            }
////        }
////        return new Pose(x, y);
////    }
//    public static void takeSnapShot(){
//        m_limelight.captureSnapshot("1");
//    }
//    public static ArrayList<Pose> getGameElements(){
//        //TODO
//        return new ArrayList<>();
//    }
//}