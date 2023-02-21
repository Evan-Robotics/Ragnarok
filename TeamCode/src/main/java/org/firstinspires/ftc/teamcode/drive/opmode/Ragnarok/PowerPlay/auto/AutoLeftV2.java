package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Config
@Autonomous(name="--WIP-- Left")
public class AutoLeftV2 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static Pose2d START_POSITION = new Pose2d(-39.875, -62.5, -Math.PI/2);
    public static int DROP_HEIGHT = 2500;
    public static int FIRST_PICKUP = 150;
    public static int CONE_REDUCE_CONSTANT = 20;
    int highestConePos = FIRST_PICKUP;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSITION);

        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);

        robot.towersPositionMode();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        // GO FROM HERE

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id == 2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        robot.moveClaw(true);
        robot.moveWrist(false);
        robot.moveTwists(false);
        robot.moveGuide(false);

        if (isStopRequested()) return;

        Pose2d prepMovementPose = new Pose2d(-36, -58.25, -Math.PI/2);
        Pose2d overshootPose = new Pose2d(-36, -8, -Math.PI/2);
        Pose2d prepScorePose = new Pose2d(-36, -12, -Math.PI*3/4);
        Pose2d scorePose = new Pose2d(-28, -4, -Math.PI*3/4);
        Pose2d prepLoadPose = new Pose2d(-36, -12, -Math.PI);
        Pose2d loadPose = new Pose2d(-60, -12, -Math.PI);

        TrajectorySequence startToPrepScoreTrajSeq = drive.trajectorySequenceBuilder(START_POSITION)
                .splineToConstantHeading(getVec(prepMovementPose), Math.PI/2)
                .splineToConstantHeading(getVec(overshootPose), Math.PI/2)
                .waitSeconds(0.1)
                .splineToLinearHeading(prepScorePose, -Math.PI/2)
                .build();

        TrajectorySequence scorePreLoadConeTrajSeq = drive.trajectorySequenceBuilder(prepScorePose)
                .strafeTo(getVec(scorePose))
                .addTemporalMarker(()->{
                    robot.setTowerTarget(DROP_HEIGHT);
                    robot.moveTowers(1);
                    robot.moveTwists(true);
                    robot.moveGuide(true);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()->robot.moveClaw(false))
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    robot.moveGuide(false);
                    robot.moveTwists(false);
                    robot.setTowerTarget(5);
                    robot.moveTowers(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.moveClaw(true))
                .strafeTo(getVec(prepScorePose), SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .addTemporalMarker(()->robot.moveTowers(0))
                .build();

        TrajectorySequence scoreConeTrajSeq = drive.trajectorySequenceBuilder(prepScorePose)
                .strafeTo(getVec(scorePose))
                .addTemporalMarker(()->{
                    robot.setTowerTarget(DROP_HEIGHT);
                    robot.moveTowers(1);
                    robot.moveTwists(true);
                    robot.moveWrist(true);
                    robot.moveGuide(true);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()->robot.moveClaw(false))
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    robot.moveGuide(false);
                    robot.moveTwists(false);
                    robot.moveWrist(false);
                    robot.setTowerTarget(5);
                    robot.moveTowers(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->robot.moveClaw(true))
                .strafeTo(getVec(prepScorePose), SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .addTemporalMarker(()->robot.moveTowers(0))
                .build();

        TrajectorySequence grabConeTrajSeq = drive.trajectorySequenceBuilder(prepScorePose)
                .addTemporalMarker(()->{
                    robot.moveClaw(false);
                    robot.setTowerTarget(getConeHeight());
                    robot.moveTowers(0.7);
                })
                .lineToSplineHeading(loadPose)
                .addTemporalMarker(()->{
                    robot.moveClaw(true);
                    robot.setTowerTarget(100);
                    robot.moveTowers(1);
                })
                .waitSeconds(1)
                .addTemporalMarker(()->robot.moveTowers(0))
                .strafeTo(getVec(prepScorePose))
                .build();


        drive.followTrajectorySequence(startToPrepScoreTrajSeq);
        sleep(200);
        drive.followTrajectorySequence(scorePreLoadConeTrajSeq);
        sleep(200);
        drive.followTrajectorySequence(grabConeTrajSeq);
        sleep(200);
        drive.followTrajectorySequence(scoreConeTrajSeq);
        sleep(1000);


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addData("Detected tag ID", detection.id);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    int getConeHeight() {
        highestConePos -= CONE_REDUCE_CONSTANT;
        return highestConePos + CONE_REDUCE_CONSTANT;
    }
}
