package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PowerPlay.HardwareRagnarok;

@Config
@Autonomous(name="Visualize Coords", group="drive")
public class CoordinateTests extends LinearOpMode {
    public static double x = 50;
    public static double y = 50;
    public static double head = 0;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot   = new HardwareRagnarok();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(x, y, Math.toRadians(head));

        drive.setPoseEstimate(startPose);

        waitForStart();

        while (!isStopRequested()) {
            sleep(100);
        }
    }
}
