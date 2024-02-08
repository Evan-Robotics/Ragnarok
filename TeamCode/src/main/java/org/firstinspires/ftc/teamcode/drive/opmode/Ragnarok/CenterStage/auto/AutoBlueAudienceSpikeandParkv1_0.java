package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.BlueOpenCVMaster;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.CenterStage.HardwareCenterStage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="v1.0 Spike and Park Auto Blue Audience", preselectTeleOp = "--MAIN-- TeleOp")
public class AutoBlueAudienceSpikeandParkv1_0 extends LinearOpMode {

    public static double L = 17.5; // length of bot
    public static double W = 16.2; // width of bot
    public static double R = 70.0; // radius of field
    public static double T = Math.PI * 2; // tau
    public static double SQRT2 = Math.sqrt(2);
    public static double SQRT3 = Math.sqrt(3);
    public static Pose2d START_POSITION = new Pose2d(-R*2/3+W/2, R-L/2, -T/4);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSITION);

        HardwareCenterStage robot = new HardwareCenterStage();
        robot.init(hardwareMap);

        Pose2d spikeDrop3 = new Pose2d(-R*2/3,R/2 + L/2 + 1, T/4);
        Pose2d spikeDrop2 = new Pose2d(-R*2/3-1, R/3, T/2);
        Pose2d spikeDrop1 = new Pose2d(-(L+2)/4 - R/3,R/3 + 6 + (L+2)/4*SQRT3, T/2-T/6);
        Pose2d notPark = new Pose2d(-R/2, R*5/6, T/4);
        Pose2d pathNode = new Pose2d(-R/2+1, R*5/6, T/2);
        Pose2d park = new Pose2d(R*5/6, R*5/6, T/2);

        BlueOpenCVMaster cv = new BlueOpenCVMaster(this);
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

        TrajectorySequence place3 = drive.trajectorySequenceBuilder(START_POSITION)
                .strafeTo(getVec(spikeDrop3))
                .turn(T/2 + 1e-6)
                .turn(-0.02)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.7);
                })
                .strafeTo(getVec(notPark))
                .build();

        TrajectorySequence place2 = drive.trajectorySequenceBuilder(START_POSITION)
                .strafeTo(getVec(spikeDrop2))
                .turn(-T/4-T/60)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.7);
                })
                .setTangent(T/2)
                .splineToConstantHeading(getVec(notPark), T/4)
                .build();

        TrajectorySequence place1 = drive.trajectorySequenceBuilder(START_POSITION)
                .forward(2)
                .splineToLinearHeading(spikeDrop1, 0)
                .addTemporalMarker(()->{
                    robot.moveIntake(-0.7);
                })
                .setTangent(T/2)
                .splineToLinearHeading(notPark, T/4)
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
        sleep(1000);
        robot.moveBucket(0);
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(pathNode, 0)
                .strafeTo(getVec(park))
                .build());
        sleep(2000);
    }

    static Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }
}
