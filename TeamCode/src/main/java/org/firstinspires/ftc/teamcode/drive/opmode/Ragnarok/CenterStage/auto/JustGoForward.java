package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareCenterStage;

@Autonomous(name = "Just Go Forward", preselectTeleOp = "--MAIN-- TeleOp")
public class JustGoForward extends LinearOpMode {

    public static double BOT_LENGTH = 17.5;
    public static double BOT_WIDTH  = 16.2;
    public static Pose2d START_POSITION = new Pose2d(70/3-BOT_WIDTH/2, -70+BOT_LENGTH/2, -Math.PI/2);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSITION);

        HardwareCenterStage robot = new HardwareCenterStage();
        robot.init(hardwareMap);

        waitForStart();

        // PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        if (isStopRequested()) return;
        sleep(100);


        Trajectory place1 = drive.trajectoryBuilder(START_POSITION)
                .forward(70/3*2-BOT_LENGTH)
                .build();

        drive.followTrajectory(place1);
    }

    static Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }
}
