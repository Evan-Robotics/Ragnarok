package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareCenterStage;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.RedOpenCVMaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="v1.0 Auto Red Backstage", preselectTeleOp = "--MAIN-- TeleOp")
public class AutoRedBackstagev1_0 extends LinearOpMode {

    public static double L = 17.5; // length of bot
    public static double W = 16.2; // width of bot
    public static double R = 70.0; // radius of field
    public static double T = Math.PI * 2; // tau
    public static double SQRT2 = Math.sqrt(2);
    public static double SQRT3 = Math.sqrt(3);
    public static Pose2d START_POSITION = new Pose2d(R/3 - W/2, -R + L/2, T/4); //red

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSITION);

        HardwareCenterStage robot = new HardwareCenterStage();
        robot.init(hardwareMap);

        Pose2d spikeDrop1Node = new Pose2d(R*2/9,-R*7/9, -T/6);
        Pose2d spikeDrop1 = new Pose2d((L+2)/4,-R/3 - 8 - (L+2)/4*SQRT3, -T/6);
        Pose2d spikeDrop2 = new Pose2d(R/3+2, -R/3+1, 0);
        Pose2d spikeDrop3 = new Pose2d((L+2)/4 + R/3, -R/3 - 6 + (L+2)/4*SQRT3, T/6);
        Pose2d board1 = new Pose2d(R/3*2+7, -R/2+4, 0);
        Pose2d board2 = new Pose2d(R/3*2+7, -R/2, 0);
        Pose2d board3 = new Pose2d(R/3*2+7, -R/2-8, 0);
        Pose2d park = new Pose2d(R*2/3, -R*5/6, 0);

        RedOpenCVMaster cv = new RedOpenCVMaster(this);
        cv.observeStick();
        int item = 2;
        int max;

        while (!isStarted() && !isStopRequested()) {
            max = cv.opencv.max;
            if (cv.opencv.avg1 == max) { item = 1; }
            if (cv.opencv.avg2 == max) { item = 2; }
            if (cv.opencv.avg3 == max) { item = 3; }

            telemetry.addData("Detecting", item);
            telemetry.update();
        }

        if (isStopRequested()) return;
        sleep(100);

        TrajectorySequence place1 = drive.trajectorySequenceBuilder(START_POSITION)
                .forward(2)
                .splineToSplineHeading(spikeDrop1Node, T/4)
                .splineToLinearHeading(spikeDrop1, T/3)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    robot.setArmPosition(HardwareCenterStage.ARM_POS_2+0.07);
                })
                .setTangent(0)
                .splineToLinearHeading(board1, 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(0);
                    robot.moveBucket(-0.5);
                })
                .build();

        TrajectorySequence place2 = drive.trajectorySequenceBuilder(START_POSITION)
                .setTangent(T/4)
                .splineToLinearHeading(spikeDrop2, T/4)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    robot.setArmPosition(HardwareCenterStage.ARM_POS_2+0.07);
                })
                .setTangent(0)
                .splineToConstantHeading(getVec(board2), 0)
                .addTemporalMarker(()->{
                     robot.moveIntake(0);
                     robot.moveBucket(-0.5);
                })
                .build();

        TrajectorySequence place3 = drive.trajectorySequenceBuilder(START_POSITION)
                .splineToLinearHeading(spikeDrop3, T/6)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    robot.setArmPosition(HardwareCenterStage.ARM_POS_2+0.07);
                })
                .setTangent(-T/6)
                .splineToLinearHeading(board3, 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(0);
                    robot.moveBucket(-0.5);
                })
                .build();

        switch (item) {
            case 1:
                drive.followTrajectorySequence(place1);
                break;
            case 2:
                drive.followTrajectorySequence(place2);
                break;
            case 3:
                drive.followTrajectorySequence(place3);
        }
        robot.moveIntake(1);
        sleep(2000);
        robot.moveIntake(0);
        robot.moveBucket(0);
        robot.moveArm(false);
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(T*5/8)
                .splineToConstantHeading(getVec(park), -T/4)
                .build());
        sleep(2000);
    }

    static Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }
}
