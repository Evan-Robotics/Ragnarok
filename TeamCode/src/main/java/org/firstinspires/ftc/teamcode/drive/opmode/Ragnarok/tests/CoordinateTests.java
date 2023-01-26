package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@Config
@TeleOp(name="Visualize Coords", group="drive")
public class CoordinateTests extends LinearOpMode {
    public static double x = -39.875;
    public static double y = -62.5;
    public static double head = -90;

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot   = new HardwareRagnarok();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(x, y, Math.toRadians(head));

        drive.setPoseEstimate(startPose);

        waitForStart();

        while (!isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();


            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            sleep(100);
        }
    }
}
