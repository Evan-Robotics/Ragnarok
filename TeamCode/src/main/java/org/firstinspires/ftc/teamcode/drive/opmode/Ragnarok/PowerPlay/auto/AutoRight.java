package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="--MAIN-- Right Auto")
public class AutoRight extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);

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

        if (isStopRequested()) return;
//
//        Trajectory beginning_to_junct_1 = drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)), true)
//                .back(75.5)
//                .build();
//
//        drive.followTrajectory(beginning_to_junct_1);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .back(45)
                .build());

        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .forward(7)
                .build());

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .turn(Math.toRadians(45))
                .build());

        robot.moveTowers(1);
        robot.moveTwists(true);

        sleep(700);

        robot.moveTowers(0.3);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .back(10)
                .build());

        sleep(200);

        robot.moveTowers(-0.5);

        sleep(500);

        robot.moveClaw(false);

        robot.moveTowers(0.5);

        sleep(1000);

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .forward(7)
                .build());

        robot.moveClaw(true);
        robot.moveWrist(false);
        robot.moveTwists(false);
        robot.moveTowers(-1);
        sleep(500);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                .turn(Math.toRadians(-50))
                .build());

        switch (tagOfInterest.id) {
            case 1:
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                        .strafeRight(30)
                        .build());
                break;
            case 2:
                break;
            case 3:
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(35, -62.5, Math.toRadians(-90)))
                        .strafeLeft(36)
                        .build());
                break;
        }

        sleep(2000);

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
