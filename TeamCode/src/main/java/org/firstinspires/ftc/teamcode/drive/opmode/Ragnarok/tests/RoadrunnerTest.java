package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .back(43)
                .build());

        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .forward(5)
                .build());

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .turn(Math.toRadians(45))
                .build());

        robot.moveTowers(1);
        robot.moveTwists(true);

        sleep(1000);

        robot.moveTowers(0.3);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .back(5)
                .build());

        sleep(200);

        robot.moveTowers(-0.3);

        sleep(100);

        robot.moveClaw(false);

        robot.moveTowers(0.5);

        sleep(1000);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .forward(8)
                .build());

        robot.moveClaw(false);
        robot.moveWrist(false);
        robot.moveTwists(false);
        sleep(2000);

    }
}
