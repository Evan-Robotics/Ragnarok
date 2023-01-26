package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;


@Autonomous(group = "drive")
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(35, -62.5, Math.toRadians(-90)));


        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);


        waitForStart();

        robot.moveClaw(true);
        robot.moveWrist(false);
        robot.moveTwists(false);

        if (isStopRequested()) return;
//
//        Trajectory beginning_to_junct_1 = drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)), true)
//                .back(75.5)
//                .build();
//
//        drive.followTrajectory(beginning_to_junct_1);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(35, 10))
                        .splineTo(new Vector2d(35, 12), Math.toRadians(180))
                        .addDisplacementMarker(() -> {robot.moveTwists(true);})
                        .splineToLinearHeading(new Pose2d(0,12, Math.toRadians(0)), Math.toRadians(0))
                        .build();
        drive.followTrajectory(traj);

//        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
//                .back(43)
//                .build());
//
//        sleep(100);
//
//        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
//                .forward(5)
//                .build());
//
//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
//                .turn(Math.toRadians(45))
//                .build());
//
//        robot.moveTowers(1);
//        robot.moveTwists(true);
//
//        sleep(1000);
//
//        robot.moveTowers(0.3);
//
//        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
//                .back(5)
//                .build());
//
//        sleep(200);
//
//        robot.moveTowers(-0.3);
//
//        sleep(100);
//
//        robot.moveClaw(false);
//
//        robot.moveTowers(0.5);
//
//        sleep(1000);
//
//        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
//                .forward(8)
//                .build());
//
//        robot.moveClaw(false);
//        robot.moveWrist(false);
//        robot.moveTwists(false);
//        sleep(2000);

    }
}
