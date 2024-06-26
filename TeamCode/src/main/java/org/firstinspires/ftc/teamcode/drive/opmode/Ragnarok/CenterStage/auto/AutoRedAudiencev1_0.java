package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareCenterStage;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.RedOpenCVMaster;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="v1.0 Auto Red Audience", preselectTeleOp = "--MAIN-- TeleOp")
public class AutoRedAudiencev1_0 extends LinearOpMode {

    public static double L = 17.5; // length of bot
    public static double W = 16.2; // width of bot
    public static double R = 70.0; // radius of field
    public static double T = Math.PI * 2; // tau
    public static double SQRT2 = Math.sqrt(2);
    public static double SQRT3 = Math.sqrt(3);
    public static Pose2d START_POSITION = new Pose2d(-R*2/3+W/2, -R+L/2, T/4); //red

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSITION);

        HardwareCenterStage robot = new HardwareCenterStage();
        robot.init(hardwareMap);

        Pose2d spikeDrop1 = new Pose2d(-R*2/3 + L/4*SQRT3,-R/3 - 6 + L/4, T/12);
        Pose2d spikeDrop2 = new Pose2d(-R/2 + L/4, -R/3 + L/4*SQRT3, T/6);
        Pose2d spikeDrop3 = new Pose2d(-R/3 - L/2 - 2, -R/3 - 6, T/2 - 1e-6);
        Pose2d pathNode1 = new Pose2d(-R/3, -R/6, 0);
        Pose2d pathNode2 = new Pose2d(R/3, -R/6, 0);
        Pose2d board1 = new Pose2d(R/3*2+9, -R/2+7, 0);
        Pose2d board2 = new Pose2d(R/3*2+9, -R/2, 0);
        Pose2d board3 = new Pose2d(R/3*2+9, -R/2-8, 0);

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
                .splineToLinearHeading(spikeDrop1, T/4)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.7);
                })
                .addSpatialMarker(new Vector2d(R/6, 0), ()->{
                    robot.setArmPosition(HardwareCenterStage.ARM_POS_2+0.02);
                })
                .setTangent(T/4)
                .splineToSplineHeading(pathNode1, 0)
                .strafeTo(getVec(pathNode2))
                .splineToConstantHeading(getVec(board1), 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(0);
                    robot.moveBucket(-0.5);
                })
                .build();

        TrajectorySequence place2 = drive.trajectorySequenceBuilder(START_POSITION)
                .setTangent(T/3)
                .splineToLinearHeading(spikeDrop2, T/6)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.7);
                })
                .addSpatialMarker(new Vector2d(R/6, 0), ()->{
                    robot.setArmPosition(HardwareCenterStage.ARM_POS_2+0.02);
                })
                .setTangent(T/4)
                .splineToSplineHeading(pathNode1, 0)
                .strafeTo(getVec(pathNode2))
                .splineToConstantHeading(getVec(board2), 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(0);
                    robot.moveBucket(-0.5);
                })
                .build();

        TrajectorySequence place3 = drive.trajectorySequenceBuilder(START_POSITION)
                .forward(2)
                .splineToLinearHeading(spikeDrop3, 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.7);
                })
                .addSpatialMarker(new Vector2d(R/6, 0), ()->{
                    robot.setArmPosition(HardwareCenterStage.ARM_POS_2+0.02);
                })
                .setTangent(T/2)
                .splineToSplineHeading(pathNode1, 0)
                .strafeTo(getVec(pathNode2))
                .splineToConstantHeading(getVec(board3), 0)
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
        sleep(1000);
        robot.moveIntake(0);
        robot.moveBucket(0);
        robot.moveArm(false);
        sleep(2000);
    }

    static Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }
}
