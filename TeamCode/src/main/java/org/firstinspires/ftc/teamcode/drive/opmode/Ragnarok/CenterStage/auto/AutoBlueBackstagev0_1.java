package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareCenterStage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="v0.1 Auto Blue Backstage", preselectTeleOp = "--MAIN-- TeleOp")
public class AutoBlueBackstagev0_1 extends LinearOpMode {

    public static double L = 17.5; // length of bot
    public static double W = 16.2; // width of bot
    public static double R = 70.0; // radius of field
    public static double T = Math.PI * 2; // tau
    public static Pose2d START_POSITION = new Pose2d(R/3 - W/2, R - L/2, -T/4);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSITION);

        HardwareCenterStage robot = new HardwareCenterStage();
        robot.init(hardwareMap);

        Pose2d spikeDrop2 = new Pose2d(R/3, R/3, 0);
        Pose2d boardMid = new Pose2d(R/3*2, R/2, 0);

        waitForStart();

        // PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        if (isStopRequested()) return;
        sleep(100);

        TrajectorySequence place2 = drive.trajectorySequenceBuilder(START_POSITION)
                .setTangent(-T/4)
                .splineToLinearHeading(spikeDrop2, -T/4)
                .addTemporalMarker(()->{
                    robot.moveIntake(-1);
                    robot.setTowerTarget(2350);
                    robot.moveTowers(1);
                    robot.moveArm(true);
                })
                .setTangent(0)
                .splineToConstantHeading(getVec(boardMid), 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(0);
                    robot.moveBucket(-0.5);
                })
                .build();

        drive.followTrajectorySequence(place2);
        sleep(1000);
        robot.moveArm(false);
        robot.setTowerTarget(5);
        sleep(2000);
    }

    static Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }
}
