package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Config
@Autonomous(name="Simple Drop Auto")
public class SimpleDropAuto extends LinearOpMode {

    public static double BOT_LENGTH = 17.5;
    public static double BOT_WIDTH  = 16.2;
    public static Pose2d START_POSITION = new Pose2d(70/3-BOT_WIDTH/2, -70+BOT_LENGTH/2, -Math.PI/2);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-39.875, -62.5, Math.toRadians(-90)));

        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);

        Pose2d spikeDrop2 = new Pose2d(70/6, -70/3-BOT_LENGTH/2, -Math.PI/2);

        waitForStart();

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        if (isStopRequested()) return;
        sleep(100);

        TrajectorySequence place1 = drive.trajectorySequenceBuilder(START_POSITION)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(getVec(spikeDrop2), Math.PI/2)
                .build();


        drive.followTrajectorySequence(place1);
    }

    static Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }
}
