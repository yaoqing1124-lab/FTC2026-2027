package org.firstinspires.ftc.teamcode._2027.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._2027.opmode.auto.util.AutoBase;
import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.subsystem.shooter.Flywheel;
import org.firstinspires.ftc.teamcode._2027.util.Mode;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.arch.utils.Debug;


@TeleOp
public class ManualControl extends OpObject {
    boolean isLeft; //目標是不是在操作者左邊
    double axial, lateral, yaw;

    @Override
    public void Init() {
        if (AutoBase.team == Team.none || AutoBase.team == null) {
            team = Team.blue;
        }
        if (AutoBase.savePose == null) {
            AutoBase.savePose = new Pose(0, 0, Math.toRadians(0));
        }
    }

    @Override
    public void Start() {
        Drivetrain.get_Follower().setStartingPose(AutoBase.savePose);
        Drivetrain.get_Follower().startTeleOpDrive();
        Flywheel.enablePredict = false;
    }

    @Override
    public void Loop() {
        axial   = -gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw     = -gamepad1.right_stick_x;
        Drivetrain.get_Follower().setTeleOpDrive(axial, lateral, yaw * 0.65, false);
        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            isLeft = !isLeft;
        }

        if (gamepad1.left_bumper) {
            Intake.setMode(Mode.shoot);
        } else {
            Intake.setMode(Mode.intake);
        }
        debug();
    }

    @Override
    public void Stop() {

    }
    public void debug(){
        com.pedropathing.follower.Follower follower = Drivetrain.get_Follower();
        double headingDeg = (follower != null) ? Math.toDegrees(follower.getHeading()) : 0.0;

        Debug.Log("team", team);
        Debug.Log("Heading (deg):", String.format("%.1f", headingDeg));
        Debug.Log("intake state", Intake.getMode());
        Debug.Log("pose", follower != null ? follower.getPose() : "null");
        Debug.Log("=========================");
    }
}
