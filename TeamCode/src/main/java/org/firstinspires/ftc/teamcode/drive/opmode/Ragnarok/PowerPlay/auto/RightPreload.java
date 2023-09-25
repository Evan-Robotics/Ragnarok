package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous(name="--Consistent-- Preload Right")
@Disabled
public class RightPreload extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static Pose2d START_POSITION = new Pose2d(24+15.5/2, -61.5, -Math.PI/2);
    public static int LOAD_HEIGHT = 1310;

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
        robot.moveWrist(true);

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
        robot.moveWrist(true);
        robot.moveTwists(false);
        robot.moveGuide(false);
        robot.setTowerTarget(5);
        robot.towersPositionMode();

        if (isStopRequested()) return;


        Pose2d strafeNode1 = new Pose2d(32, -60, -Math.PI/2);
        Pose2d strafeNode2 = new Pose2d(34, -52.5, 0);
        Pose2d midPreloadPrepPose = new Pose2d(34,-25, 0);
        Pose2d midPreloadPose = new Pose2d(30, -24, 0);
        Pose2d prepPark = new Pose2d(36, -12, Math.PI/2);
        Pose2d park3Pose = new Pose2d(58, -18, Math.PI/2);
        Pose2d park2Pose = new Pose2d(36, -18, Math.PI/2);
        Pose2d park1Pose = new Pose2d(14, -18, Math.PI/2);

        TrajectorySequence scorePreloadTrajSeq = drive.trajectorySequenceBuilder(START_POSITION)
                .addTemporalMarker(0.5, ()->{
                                robot.setTowerTarget(LOAD_HEIGHT+50);
                                telemetry.addData("tower target", robot.leftTower.getTargetPosition());
                                telemetry.update();
                                robot.moveTowers(1);
                                robot.moveTwists(true);
                })
                .setTangent(Math.PI / 4)
                .splineToConstantHeading(getVec(strafeNode1), Math.PI / 2)
                .splineToSplineHeading(strafeNode2, Math.PI / 2)
                .splineToSplineHeading(midPreloadPrepPose, 2*Math.PI / 3)
                .addTemporalMarker(()->{
                                robot.moveGuide(true);
                })
                .splineToConstantHeading(getVec(midPreloadPose), 0)
                .waitSeconds(0.1)
                .addTemporalMarker(()->{
                                robot.moveClaw(false);
                                sleep(100);
                                robot.moveGuide(false);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                                robot.moveTwists(false);
                                robot.moveWrist(false);
                                robot.setTowerTarget(5);
                })
                .build();


        TrajectorySequence prepParkTrajSeq = drive.trajectorySequenceBuilder(midPreloadPose)
                .setTangent(0)
                .addTemporalMarker(()->{
                    robot.moveClaw(true);
                    robot.setTowerTarget(5);
                    robot.moveGuide(false);
                    robot.moveWrist(false);
                    robot.moveTwists(false);
                })
                .splineToLinearHeading(prepPark, Math.PI /6)
                .build();

        TrajectorySequence park1TrajSeq = drive.trajectorySequenceBuilder(prepPark)
                .setTangent(0)
                .splineToConstantHeading(getVec(park1Pose), Math.PI *3/2)
                .build();

        TrajectorySequence park2TrajSeq = drive.trajectorySequenceBuilder(prepPark)
                .setTangent(Math.PI /2)
                .splineToConstantHeading(getVec(park2Pose), Math.PI * 3/2)
                .build();

        TrajectorySequence park3TrajSeq = drive.trajectorySequenceBuilder(prepPark)
                .setTangent(Math.PI)
                .splineToConstantHeading(getVec(park3Pose), Math.PI * 3/2)
                .build();

        drive.followTrajectorySequence(scorePreloadTrajSeq);
        drive.followTrajectorySequence(prepParkTrajSeq);

        if (tagOfInterest == null || tagOfInterest.id == 2) {
            drive.followTrajectorySequence(park2TrajSeq);
        } else if (tagOfInterest.id == 1) {
            drive.followTrajectorySequence(park1TrajSeq);
        } else if (tagOfInterest.id == 3) {
            drive.followTrajectorySequence(park3TrajSeq);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addData("Detected tag ID", detection.id);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }

    Vector2d getVec(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

}
