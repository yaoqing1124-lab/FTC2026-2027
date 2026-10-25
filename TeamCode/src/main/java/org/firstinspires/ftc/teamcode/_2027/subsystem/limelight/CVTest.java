//package org.firstinspires.ftc.teamcode._2027.subsystem.limelight;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode._2027.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
//
//import java.util.ArrayList;
//
//
//@TeleOp
//public class CVTest extends LinearOpMode {
//    private CVPathGen pather;
//    private Follower follower;
//    public static boolean isRed = true;
//    ArrayList<Pose> allBalls = new ArrayList<>();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        follower = Drivetrain.get_Follower();
//        follower.setPose(new Pose(35,8.5,Math.toRadians(90)));
//        follower.update();
//        pather = new CVPathGen(hardwareMap,follower, isRed, telemetry);
//
//        waitForStart();
//        while (opModeIsActive()){
//            follower.update();
//            allBalls = pather.getBalls(false);
//            if(gamepad1.aWasPressed()){
//                Path path = pather.pathGen();
//                if(path != null){
//                    follower.followPath(path);
//                }
//            }
//            if(follower.isBusy() && follower.atParametricEnd()){
//                follower.breakFollowing();
//            }
//            for (Pose ball: allBalls) {
//                telemetry.addLine("(" + ball.x() + ", "+ ball.y() + ")");
//            }
//            telemetry.addData("Robot X", follower.getPose().x());
//            telemetry.addData("Robot Y", follower.getPose().y());
//            telemetry.addData("Remaining", follower.remainingDistance());
//            telemetry.addData("Dist to endpoint", follower.distanceToEndpoint());
//            telemetry.addData("Completion", follower.completion());
//            telemetry.update();
//        }
//    }
//}
