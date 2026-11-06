package org.firstinspires.ftc.teamcode._2027.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode._2027.opmode.auto.util.AutoBase;
import org.firstinspires.ftc.teamcode._2027.opmode.auto.util.AutoPose;
import org.firstinspires.ftc.teamcode._2027.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode._2027.subsystem.Intake;
import org.firstinspires.ftc.teamcode._2027.subsystem.shooter.Shooter;
import org.firstinspires.ftc.teamcode._2027.util.Mode;
import org.firstinspires.ftc.teamcode._2027.util.Team;
import org.firstinspires.ftc.teamcode.arch.op.OpObject;
import org.firstinspires.ftc.teamcode.arch.utils.Debug;

@TeleOp
public class ManualControl extends OpObject {
    boolean isLeft; // 目標是不是在操作者左邊
    double axial, lateral, yaw;

    @Override
    public void Init() {
        if (AutoBase.team == Team.none || AutoBase.team == null) {
            team = Team.blue;
        }
        if (AutoBase.savePose == null) {
            AutoBase.savePose = AutoPose.start.START;
        }
        Shooter.setEnablePredictAll(false);
    }

    @Override
    public void Start() {
        Drivetrain.get_Follower().setStartingPose(AutoBase.savePose);
        Drivetrain.get_Follower().startTeleOpDrive();
    }

    @Override
    public void Loop() {
        // 操作手站位視角 (Driver Station Relative):
        //   - Team.red: 站在場地左邊 (X=-72, 面向 +X 方向)
        //   - Team.blue: 站在場地右邊 (X=+72, 面向 -X 方向)
        // 依據聯盟翻轉操作方向 (與 GC_FTC-main 一致)，確保往前推搖桿皆為遠離操作手向場地內前進
        if (team == Team.blue) {
            axial   = -gamepad1.left_stick_y;
            lateral = -gamepad1.left_stick_x;
        } else {
            axial   = gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
        }
        yaw     = -gamepad1.right_stick_x;
        Drivetrain.get_Follower().setTeleOpDrive(axial, lateral, yaw * 0.65, false);

        // 自動根據目前瞄準的目標與操作手站位判定 isLeft
        boolean aimUpper = (Shooter.small.getTargetPosition() != null && Shooter.small.getTargetPosition().y > 0);
        // Red 站在左邊 (面向右)：上半場 Hive (Y > 0) 在操作手左邊
        // Blue 站在右邊 (面向左)：上半場 Hive (Y > 0) 在操作手右邊，下半場 Hive (Y < 0) 在操作手左邊
        if (team == Team.red) {
            isLeft = aimUpper;
        } else {
            isLeft = !aimUpper;
        }

        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            isLeft = !isLeft;
        }

        // 發射器始終保持常態瞄準，是否射擊完全由 Intake 控制
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
        Debug.Log("Driver Station", (team == Team.red) ? "Left (X=-72, Face +X)" : "Right (X=+72, Face -X)");
        Debug.Log("Target isLeft", isLeft);
        Debug.Log("Heading (deg):", String.format("%.1f", headingDeg));
        Debug.Log("intake state", Intake.getMode());
        Debug.Log("shooter stable", Shooter.small.isStable());
        Debug.Log("shooter RPM", String.format("%.0f / %.0f", Shooter.small.getActualRPM(), Shooter.small.getTargetRPM()));
        Debug.Log("pose", follower != null ? follower.getPose() : "null");
        Debug.Log("=========================");
    }
}
